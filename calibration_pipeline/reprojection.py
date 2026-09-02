"""Canonical corner-reprojection backend for calibration and ablations.

This module owns the estimator shared by the paper runner and the opt-in
production STEP-E path.  Methods may differ only in their observation subset
and explicit ``variable_keys`` (freeze mask); residual construction, SE(3)
parameterization, loss, scaling, and termination settings live here.

The canonical real-data loss is a smooth robust corner-reprojection M-estimator:
SciPy TRF with ``soft_l1``, ``f_scale=2 px``, and ``x_scale='jac'``.  These
settings were selected using noise-free synthetic and calibration-train
diagnostics only.  With ``loss='linear'`` the same residual is the usual
pixel-Gaussian maximum-likelihood objective.

Every residual built here is one observation's corner reprojection.  There is
no camera-to-camera residual and no target-pose residual: two fixed cameras
influence each other only by reprojecting onto the same shared target-pose
variable, so ``T_Ci_Cj`` is derived from a converged solution rather than
estimated or constrained.  The objective minimized by this module therefore has
exactly one additive term; the optional second (FK) term lives in
:mod:`calibration_pipeline.fk_factor` and nowhere else.
"""

from __future__ import annotations

import hashlib
import time
from collections import Counter, defaultdict
from dataclasses import dataclass, field
from typing import Dict, List, Mapping, Optional, Sequence, Tuple

import cv2
import numpy as np
from scipy.optimize import least_squares
from scipy.sparse import lil_matrix
from scipy.spatial.transform import Rotation

from calibration_pipeline.apriltag_cube import inv_T


TransformKey = Tuple[str, int]

# Machine-readable statement of what this backend minimizes.  It is serialized
# into every solve's diagnostics so a downstream description cannot silently
# drift from the implementation.
VISUAL_OBJECTIVE_CONTRACT = {
    "n_objective_terms": 1,
    "terms": ("robust_corner_reprojection",),
    "residual": "predicted_corner_uv_minus_measured_corner_uv",
    "residual_domain": "distorted_native_image_pixels",
    "residual_granularity": "one_2D_block_per_detected_corner",
    "robustification": "scipy_global_loss_applied_per_scalar_residual_component",
    "scalar_term_weights_used": False,
    "pose_error_term": False,
    "fk_constraint_term": False,
    "camera_to_camera_residual": False,
    "camera_coupling_mechanism": "shared_target_pose_variables_only",
}

FRAME_PRUNE_REFIT_CONTRACT = {
    "selection_scope": "training_observations_only",
    "frame_unit": "event_id_x_camera_id_all_target_observations",
    "selection_metric": "per_frame_euclidean_corner_reprojection_RMSE_px",
    "threshold": "max(minimum_rmse_px, median + mad_multiplier*1.4826*MAD)",
    "maximum_pruned_fraction": 0.30,
    "coverage_guard": "retain_minimum_observations_for_every_free_transform",
    "refit_initialization": "accepted_first_fit_state_without_perturbation",
    "acceptance_population": "original_unpruned_training_observations",
    "acceptance_metric": "same_full_robust_objective_as_the_first_fit",
    "rollback": "keep_first_fit_unless_successful_refit_strictly_improves_full_objective",
    "heldout_observations_used": False,
    "heldout_observations_pruned": False,
}

# How a gripped-cube observation gets its target pose.  This is the FK axis in
# its sharpest form: the cube is bolted to the gripper, so either FK carries it
# (one constant per grasp) or it does not (six free DoF per event).  A placement
# cannot pose the question this cleanly — its only FK link is a single contact
# measurement taken once per set.
GRIPPED_TARGET_GRASP = "grasp"        # T_base_cube[e] = FK(q_e) @ T_gripper_cube[g]
GRIPPED_TARGET_EVENT = "event_free"   # T_base_cube[e] free, FK unused
GRIPPED_TARGET_MODELS = frozenset({GRIPPED_TARGET_GRASP, GRIPPED_TARGET_EVENT})

RESIDUAL_WEIGHT_PER_CORNER = "per_corner"
RESIDUAL_WEIGHT_EQUAL_OBSERVATION = "equal_observation_total"
RESIDUAL_WEIGHTING_MODES = frozenset({
    RESIDUAL_WEIGHT_PER_CORNER,
    RESIDUAL_WEIGHT_EQUAL_OBSERVATION,
})


def robust_least_squares_cost(
        residuals: np.ndarray, loss: str, scale: float) -> float:
    """Return SciPy's robust least-squares cost for one residual block.

    Keeping this calculation explicit makes the large visual block comparable
    with the much smaller FK block.  The value includes the same ``1/2`` factor
    used by :func:`scipy.optimize.least_squares`.
    """
    values = np.asarray(residuals, dtype=np.float64).reshape(-1)
    scale = float(scale)
    if loss not in {"linear", "huber", "soft_l1"} or scale <= 0.0:
        raise ValueError("invalid robust loss or scale")
    z = np.square(values / scale)
    if loss == "linear":
        rho = z
    elif loss == "soft_l1":
        rho = 2.0 * (np.sqrt(1.0 + z) - 1.0)
    else:
        rho = np.where(z <= 1.0, z, 2.0 * np.sqrt(z) - 1.0)
    return float(0.5 * scale * scale * np.sum(rho))


@dataclass(frozen=True)
class PixelObs:
    marker: str
    cam: int
    event: int
    set_idx: Optional[int]
    object_points: np.ndarray
    image_points: np.ndarray
    # Set only while the robot HOLDS the cube.  Then the target is not a static
    # per-set pose but ``FK(q_event) @ T_gripper_cube[grasp_idx]``: the cube rides
    # the gripper, so one constant per grasp replaces one pose per placement.
    # ``None`` keeps the placement model and the legacy behaviour untouched.
    grasp_idx: Optional[int] = None


def observation_population(
        observations: Sequence[PixelObs], gripper_cam_idx: int) -> dict:
    """Describe the exact residual population before any model is fitted."""
    gripper = int(gripper_cam_idx)

    def empty() -> dict:
        return {"observations": 0, "corners": 0, "residual_components": 0}

    total = empty()
    by_marker: Dict[str, dict] = {}
    by_role = {"eih": empty(), "e2h": empty()}
    by_camera: Dict[str, dict] = {}
    for observation in observations:
        corners = int(len(np.asarray(observation.image_points).reshape(-1, 2)))
        role = "eih" if int(observation.cam) == gripper else "e2h"
        camera = str(int(observation.cam))
        marker = str(observation.marker)
        for bucket in (
                total,
                by_marker.setdefault(marker, empty()),
                by_role[role],
                by_camera.setdefault(camera, empty())):
            bucket["observations"] += 1
            bucket["corners"] += corners
            bucket["residual_components"] += 2 * corners
    return {
        **total,
        "by_marker": dict(sorted(by_marker.items())),
        "by_camera_role": by_role,
        "by_camera": dict(sorted(by_camera.items(), key=lambda item: int(item[0]))),
    }


@dataclass
class PoseState:
    cams: Dict[int, np.ndarray]
    gtc: np.ndarray
    board: Optional[np.ndarray]
    cubes: Dict[int, np.ndarray]
    # T_gripper_cube per grasp.  Empty unless gripped observations are in play.
    grasps: Dict[int, np.ndarray] = field(default_factory=dict)
    # T_base_cube per gripped event, for the arm that refuses to let FK carry the
    # cube.  This is the other half of the FK-on/off contrast: with the grasp
    # model FK fully determines the cube's motion from one constant per grasp;
    # here every gripped event pays its own 6 DoF and FK says nothing.
    event_cubes: Dict[int, np.ndarray] = field(default_factory=dict)

    def clone(self) -> "PoseState":
        return PoseState(
            cams={int(k): np.asarray(v, dtype=np.float64).copy()
                  for k, v in self.cams.items()},
            gtc=np.asarray(self.gtc, dtype=np.float64).copy(),
            board=(None if self.board is None else
                   np.asarray(self.board, dtype=np.float64).copy()),
            cubes={int(k): np.asarray(v, dtype=np.float64).copy()
                   for k, v in self.cubes.items()},
            grasps={int(k): np.asarray(v, dtype=np.float64).copy()
                    for k, v in self.grasps.items()},
            event_cubes={int(k): np.asarray(v, dtype=np.float64).copy()
                         for k, v in self.event_cubes.items()},
        )


@dataclass(frozen=True)
class SE3Scaling:
    """Physical perturbation represented by one optimizer unit.

    Defaults preserve the legacy unit scale.  The convergence study may select
    a different deterministic scale, but every row/entry point must share it.
    """

    rotation_scale_rad: float = 1.0
    translation_scale_m: float = 1.0

    def validate(self) -> None:
        if self.rotation_scale_rad <= 0 or self.translation_scale_m <= 0:
            raise ValueError("SE(3) scales must be positive")


@dataclass(frozen=True)
class SolverOptions:
    method: str = "trf"
    loss: str = "soft_l1"
    f_scale_px: float = 2.0
    max_nfev: int = 300
    xtol: float = 1e-8
    ftol: float = 1e-8
    gtol: float = 1e-8
    scaling: SE3Scaling = SE3Scaling()
    x_scale_mode: str = "jac"
    residual_weighting: str = RESIDUAL_WEIGHT_PER_CORNER

    def validate(self) -> None:
        if self.method != "trf":
            raise ValueError("canonical backend currently requires method='trf'")
        if self.loss not in {"linear", "huber", "soft_l1"}:
            raise ValueError("canonical backend loss must be linear, huber, or soft_l1")
        if self.x_scale_mode not in {"unit", "jac"}:
            raise ValueError("x_scale_mode must be 'unit' or 'jac'")
        if self.residual_weighting not in RESIDUAL_WEIGHTING_MODES:
            raise ValueError(
                "residual_weighting must be one of "
                f"{sorted(RESIDUAL_WEIGHTING_MODES)}")
        if self.f_scale_px <= 0 or self.max_nfev <= 0:
            raise ValueError("invalid solver options")
        self.scaling.validate()

    def to_dict(self) -> dict:
        return {
            "method": self.method,
            "loss": self.loss,
            "f_scale_px": float(self.f_scale_px),
            "max_nfev": int(self.max_nfev),
            "xtol": float(self.xtol),
            "ftol": float(self.ftol),
            "gtol": float(self.gtol),
            "parameterization": "left_local_SE3_perturbation_with_retraction",
            "rotation_scale_rad": float(self.scaling.rotation_scale_rad),
            "translation_scale_m": float(self.scaling.translation_scale_m),
            "scipy_x_scale": ("jac" if self.x_scale_mode == "jac" else 1.0),
            "residual_weighting": self.residual_weighting,
        }


@dataclass(frozen=True)
class FramePruneRefitOptions:
    """Deterministic training-frame prune/refit policy.

    A frame is every target observation stored for one ``(event, camera)``
    image.  Pruning therefore removes board and cube blocks together instead
    of cherry-picking individual corners or one target from the same image.
    """

    enabled: bool = True
    mad_multiplier: float = 3.0
    minimum_rmse_px: float = 4.0
    maximum_fraction: float = 0.30
    minimum_observations_per_variable: int = 2
    minimum_relative_improvement: float = 1e-6

    def validate(self) -> None:
        if not np.isfinite(self.mad_multiplier) or self.mad_multiplier <= 0.0:
            raise ValueError("frame-prune MAD multiplier must be positive")
        if not np.isfinite(self.minimum_rmse_px) or self.minimum_rmse_px < 0.0:
            raise ValueError("frame-prune minimum RMSE must be non-negative")
        if (not np.isfinite(self.maximum_fraction)
                or not 0.0 <= self.maximum_fraction < 1.0):
            raise ValueError("frame-prune maximum fraction must be in [0, 1)")
        if int(self.minimum_observations_per_variable) < 1:
            raise ValueError(
                "frame-prune minimum observations per variable must be >= 1")
        if (not np.isfinite(self.minimum_relative_improvement)
                or self.minimum_relative_improvement < 0.0):
            raise ValueError(
                "frame-prune minimum relative improvement must be non-negative")

    def to_dict(self) -> dict:
        return {
            **FRAME_PRUNE_REFIT_CONTRACT,
            "enabled": bool(self.enabled),
            "mad_multiplier": float(self.mad_multiplier),
            "minimum_rmse_px": float(self.minimum_rmse_px),
            "maximum_fraction": float(self.maximum_fraction),
            "minimum_observations_per_variable": int(
                self.minimum_observations_per_variable),
            "minimum_relative_improvement": float(
                self.minimum_relative_improvement),
        }


def project_points(T_C_O: np.ndarray, object_points: np.ndarray,
                   K: np.ndarray, D: np.ndarray) -> np.ndarray:
    T_C_O = np.asarray(T_C_O, dtype=np.float64)
    rvec = Rotation.from_matrix(T_C_O[:3, :3]).as_rotvec().reshape(3, 1)
    tvec = T_C_O[:3, 3].reshape(3, 1)
    projected, _ = cv2.projectPoints(
        np.asarray(object_points, dtype=np.float64), rvec, tvec,
        np.asarray(K, dtype=np.float64), np.asarray(D, dtype=np.float64),
    )
    return projected.reshape(-1, 2)


def pose_delta(A: np.ndarray, B: np.ndarray) -> Tuple[float, float]:
    error = inv_T(np.asarray(A, dtype=np.float64)) @ np.asarray(B, dtype=np.float64)
    translation_mm = float(np.linalg.norm(error[:3, 3]) * 1000.0)
    rotation_deg = float(np.degrees(np.linalg.norm(
        Rotation.from_matrix(error[:3, :3]).as_rotvec())))
    return translation_mm, rotation_deg


def state_transform(state: PoseState, key: TransformKey) -> np.ndarray:
    kind, idx = key
    if kind == "cam":
        return state.cams[int(idx)]
    if kind == "gtc":
        return state.gtc
    if kind == "board":
        if state.board is None:
            raise KeyError("board pose is absent")
        return state.board
    if kind == "cube":
        return state.cubes[int(idx)]
    if kind == "grasp":
        return state.grasps[int(idx)]
    if kind == "cube_event":
        return state.event_cubes[int(idx)]
    raise KeyError(key)


def set_state_transform(state: PoseState, key: TransformKey, value: np.ndarray) -> None:
    kind, idx = key
    value = np.asarray(value, dtype=np.float64)
    if kind == "cam":
        state.cams[int(idx)] = value
    elif kind == "gtc":
        state.gtc = value
    elif kind == "board":
        state.board = value
    elif kind == "cube":
        state.cubes[int(idx)] = value
    elif kind == "grasp":
        state.grasps[int(idx)] = value
    elif kind == "cube_event":
        state.event_cubes[int(idx)] = value
    else:
        raise KeyError(key)


def retract(reference: np.ndarray, scaled_delta: np.ndarray,
            scaling: SE3Scaling) -> np.ndarray:
    """Apply a left-local tangent perturbation and retract to SE(3)."""
    q = np.asarray(scaled_delta, dtype=np.float64).reshape(6)
    delta = np.eye(4, dtype=np.float64)
    delta[:3, :3] = Rotation.from_rotvec(
        q[:3] * float(scaling.rotation_scale_rad)).as_matrix()
    delta[:3, 3] = q[3:] * float(scaling.translation_scale_m)
    return delta @ np.asarray(reference, dtype=np.float64)


def variable_keys(names: Sequence[str], state: PoseState) -> List[TransformKey]:
    keys: List[TransformKey] = []
    for name in names:
        if name == "T_base_Ci":
            keys.extend(("cam", int(ci)) for ci in sorted(state.cams))
        elif name == "T_gripper_cam":
            keys.append(("gtc", -1))
        elif name == "T_base_board":
            if state.board is None:
                raise RuntimeError("schema requested absent T_base_board")
            keys.append(("board", -1))
        elif name == "T_base_cube_by_set":
            keys.extend(("cube", int(s)) for s in sorted(state.cubes))
        elif name == "T_gripper_cube_by_grasp":
            if not state.grasps:
                raise RuntimeError(
                    "schema requested T_gripper_cube_by_grasp but the state holds "
                    "no grasps — the observations carry no gripped frames")
            keys.extend(("grasp", int(g)) for g in sorted(state.grasps))
        elif name == "T_base_cube_by_event":
            if not state.event_cubes:
                raise RuntimeError(
                    "schema requested T_base_cube_by_event but the state holds no "
                    "per-event cube poses — the observations carry no gripped frames")
            keys.extend(("cube_event", int(e)) for e in sorted(state.event_cubes))
        else:
            raise ValueError(f"unknown variable family {name!r}")
    return keys


def freeze_manifest(state: PoseState, free_keys: Sequence[TransformKey]) -> dict:
    all_keys: List[TransformKey] = (
        [("cam", int(ci)) for ci in sorted(state.cams)]
        + [("gtc", -1)]
        + ([] if state.board is None else [("board", -1)])
        + [("cube", int(s)) for s in sorted(state.cubes)]
        + [("grasp", int(g)) for g in sorted(state.grasps)]
        + [("cube_event", int(e)) for e in sorted(state.event_cubes)]
    )
    free = set(free_keys)
    return {
        "free": [f"{kind}:{idx}" for kind, idx in all_keys if (kind, idx) in free],
        "frozen": [f"{kind}:{idx}" for kind, idx in all_keys if (kind, idx) not in free],
    }


class CornerReprojectionProblem:
    """Per-corner residual with an explicit transform freeze mask."""

    def __init__(self, observations: Sequence[PixelObs],
                 variable_keys_: Sequence[TransformKey], reference_state: PoseState,
                 robot_T: Mapping[int, np.ndarray], K_map: Mapping[int, np.ndarray],
                 D_map: Mapping[int, np.ndarray], gripper_cam_idx: int,
                 scaling: SE3Scaling = SE3Scaling(),
                 gripped_target: str = GRIPPED_TARGET_GRASP,
                 residual_weighting: str = RESIDUAL_WEIGHT_PER_CORNER):
        scaling.validate()
        if gripped_target not in GRIPPED_TARGET_MODELS:
            raise ValueError(
                f"gripped_target must be one of {sorted(GRIPPED_TARGET_MODELS)}")
        self.gripped_target = str(gripped_target)
        if residual_weighting not in RESIDUAL_WEIGHTING_MODES:
            raise ValueError(
                "residual_weighting must be one of "
                f"{sorted(RESIDUAL_WEIGHTING_MODES)}")
        self.residual_weighting = str(residual_weighting)
        self.obs = list(observations)
        self.variable_keys = list(variable_keys_)
        if len(self.variable_keys) != len(set(self.variable_keys)):
            raise ValueError("duplicate optimization variable")
        self.reference_state = reference_state.clone()
        self.robot_T = robot_T
        self.K = K_map
        self.D = D_map
        self.gripper = int(gripper_cam_idx)
        self.scaling = scaling
        self.slices = {key: slice(6 * i, 6 * (i + 1))
                       for i, key in enumerate(self.variable_keys)}
        self.n_params = 6 * len(self.variable_keys)
        self.x0 = np.zeros(self.n_params, dtype=np.float64)
        self.row_offsets: List[Tuple[int, int]] = []
        self.observation_weights: List[float] = []
        row = 0
        for obs in self.obs:
            corner_count = len(np.asarray(obs.image_points).reshape(-1, 2))
            n = 2 * corner_count
            self.row_offsets.append((row, row + n))
            self.observation_weights.append(
                1.0 if self.residual_weighting == RESIDUAL_WEIGHT_PER_CORNER
                else 1.0 / np.sqrt(float(corner_count)))
            row += n
        self.n_residuals = row
        if not self.obs or not self.n_residuals:
            raise RuntimeError("corner-reprojection problem has no observations")
        if not self.n_params:
            raise RuntimeError("corner-reprojection problem has no free variables")
        self.residual_vector(self.x0)

    def unpack(self, x: np.ndarray) -> PoseState:
        state = self.reference_state.clone()
        for key, sl in self.slices.items():
            set_state_transform(
                state, key,
                retract(state_transform(self.reference_state, key), x[sl], self.scaling),
            )
        return state

    def _residual_vector(self, x: np.ndarray, *, apply_weighting: bool) -> np.ndarray:
        state = self.unpack(x)
        chunks = []
        for obs, weight in zip(self.obs, self.observation_weights):
            if obs.marker == "board":
                if state.board is None:
                    raise RuntimeError("board observation with no board pose")
                target = state.board
            elif obs.grasp_idx is not None:
                if self.gripped_target == GRIPPED_TARGET_GRASP:
                    # FK carries the cube; only the constant grasp offset is free.
                    grasp = int(obs.grasp_idx)
                    if grasp not in state.grasps:
                        raise RuntimeError(
                            f"grasp transform unavailable for grasp {grasp}")
                    if int(obs.event) not in self.robot_T:
                        raise RuntimeError(f"robot FK missing for event {obs.event}")
                    target = (np.asarray(self.robot_T[int(obs.event)], dtype=np.float64)
                              @ state.grasps[grasp])
                else:
                    # FK withheld: this event's cube pose is its own free variable.
                    eid = int(obs.event)
                    if eid not in state.event_cubes:
                        raise RuntimeError(f"per-event cube pose unavailable for {eid}")
                    target = state.event_cubes[eid]
            else:
                if obs.set_idx is None or int(obs.set_idx) not in state.cubes:
                    raise RuntimeError(f"cube pose unavailable for set {obs.set_idx}")
                target = state.cubes[int(obs.set_idx)]
            if int(obs.cam) == self.gripper:
                if int(obs.event) not in self.robot_T:
                    raise RuntimeError(f"robot FK missing for event {obs.event}")
                T_base_cam = np.asarray(self.robot_T[int(obs.event)], dtype=np.float64) @ state.gtc
            else:
                if int(obs.cam) not in state.cams:
                    raise RuntimeError(f"fixed camera {obs.cam} is not registered")
                T_base_cam = state.cams[int(obs.cam)]
            prediction = project_points(
                inv_T(T_base_cam) @ target, obs.object_points,
                self.K[int(obs.cam)], self.D[int(obs.cam)],
            )
            residual = (
                prediction - np.asarray(obs.image_points).reshape(-1, 2)
            ).reshape(-1)
            chunks.append(residual * weight if apply_weighting else residual)
        return np.concatenate(chunks).astype(np.float64)

    def raw_residual_vector(self, x: np.ndarray) -> np.ndarray:
        """Unweighted native-pixel residual used for comparable reporting."""
        return self._residual_vector(x, apply_weighting=False)

    def residual_vector(self, x: np.ndarray) -> np.ndarray:
        """Objective residual, optionally normalized per observation block."""
        return self._residual_vector(x, apply_weighting=True)

    # Alias keeps scipy call sites terse and makes the public residual explicit.
    residual = residual_vector

    def jacobian_sparsity(self):
        matrix = lil_matrix((self.n_residuals, self.n_params), dtype=np.int8)
        for obs, (r0, r1) in zip(self.obs, self.row_offsets):
            for key in observation_dependency_keys(
                    obs, self.gripper, self.gripped_target):
                if key in self.slices:
                    matrix[r0:r1, self.slices[key]] = 1
        return matrix.tocsr()


def observation_dependency_keys(
        observation: PixelObs, gripper_cam_idx: int,
        gripped_target: str = GRIPPED_TARGET_GRASP) -> Tuple[TransformKey, ...]:
    """Return the camera and target variables touched by one observation."""
    if gripped_target not in GRIPPED_TARGET_MODELS:
        raise ValueError(
            f"gripped_target must be one of {sorted(GRIPPED_TARGET_MODELS)}")
    camera_key = (
        ("gtc", -1) if int(observation.cam) == int(gripper_cam_idx)
        else ("cam", int(observation.cam)))
    if observation.marker == "board":
        target_key = ("board", -1)
    elif observation.grasp_idx is not None:
        target_key = (
            ("grasp", int(observation.grasp_idx))
            if gripped_target == GRIPPED_TARGET_GRASP
            else ("cube_event", int(observation.event)))
    else:
        if observation.set_idx is None:
            raise ValueError("placed-cube observation has no set index")
        target_key = ("cube", int(observation.set_idx))
    return camera_key, target_key


def visual_objective_cost(
        observations: Sequence[PixelObs],
        variable_keys_: Sequence[TransformKey],
        state: PoseState,
        robot_T: Mapping[int, np.ndarray],
        K_map: Mapping[int, np.ndarray],
        D_map: Mapping[int, np.ndarray],
        gripper_cam_idx: int,
        options: SolverOptions,
        gripped_target: str = GRIPPED_TARGET_GRASP) -> float:
    """Evaluate the canonical robust visual objective without optimizing."""
    options.validate()
    problem = CornerReprojectionProblem(
        observations, variable_keys_, state, robot_T, K_map, D_map,
        gripper_cam_idx, scaling=options.scaling,
        gripped_target=gripped_target,
        residual_weighting=options.residual_weighting,
    )
    return robust_least_squares_cost(
        problem.residual_vector(problem.x0), options.loss, options.f_scale_px)


def _frame_label(frame_key: Tuple[int, int]) -> str:
    return f"E{int(frame_key[0]):06d}:cam{int(frame_key[1])}"


def select_frame_prune_subset(
        observations: Sequence[PixelObs],
        variable_keys_: Sequence[TransformKey],
        fitted_state: PoseState,
        robot_T: Mapping[int, np.ndarray],
        K_map: Mapping[int, np.ndarray],
        D_map: Mapping[int, np.ndarray],
        gripper_cam_idx: int,
        solver_options: SolverOptions,
        prune_options: FramePruneRefitOptions,
        gripped_target: str = GRIPPED_TARGET_GRASP,
        ) -> Tuple[List[PixelObs], dict]:
    """Choose coverage-safe high-residual training frames for one refit."""
    solver_options.validate()
    prune_options.validate()
    population = observation_population(observations, gripper_cam_idx)
    base_report = {
        "contract": prune_options.to_dict(),
        "enabled": bool(prune_options.enabled),
        "selection_population": population,
        "n_input_observations": int(len(observations)),
    }
    if not prune_options.enabled:
        return list(observations), {
            **base_report,
            "attempted": False,
            "reason": "disabled",
            "n_pruned_frames": 0,
            "n_pruned_observations": 0,
        }

    problem = CornerReprojectionProblem(
        observations, variable_keys_, fitted_state, robot_T, K_map, D_map,
        gripper_cam_idx, scaling=solver_options.scaling,
        gripped_target=gripped_target,
        residual_weighting=solver_options.residual_weighting,
    )
    raw = problem.raw_residual_vector(problem.x0)
    frames: Dict[Tuple[int, int], dict] = {}
    for observation, (row0, row1) in zip(problem.obs, problem.row_offsets):
        frame_key = (int(observation.event), int(observation.cam))
        block = np.asarray(raw[row0:row1], dtype=np.float64).reshape(-1, 2)
        frame = frames.setdefault(frame_key, {
            "sum_squared_euclidean_px": 0.0,
            "n_corners": 0,
            "n_observations": 0,
            "markers": set(),
        })
        frame["sum_squared_euclidean_px"] += float(np.sum(np.square(block)))
        frame["n_corners"] += int(len(block))
        frame["n_observations"] += 1
        frame["markers"].add(str(observation.marker))

    for frame in frames.values():
        frame["rmse_px"] = float(np.sqrt(
            frame["sum_squared_euclidean_px"] / max(1, frame["n_corners"])))
    frame_rmses = np.asarray(
        [frame["rmse_px"] for frame in frames.values()], dtype=np.float64)
    median = float(np.median(frame_rmses))
    mad = float(np.median(np.abs(frame_rmses - median)))
    robust_sigma = float(1.4826 * mad)
    threshold = float(max(
        prune_options.minimum_rmse_px,
        median + prune_options.mad_multiplier * robust_sigma,
    ))
    candidates = sorted(
        (key for key, frame in frames.items()
         if float(frame["rmse_px"]) > threshold),
        key=lambda key: (-float(frames[key]["rmse_px"]), key[0], key[1]),
    )
    max_pruned_frames = int(np.floor(
        prune_options.maximum_fraction * len(frames)))

    free_keys = set(variable_keys_)
    dependencies = [
        tuple(key for key in observation_dependency_keys(
            observation, gripper_cam_idx, gripped_target)
              if key in free_keys)
        for observation in observations
    ]
    support_before: Counter = Counter()
    frame_support: Dict[Tuple[int, int], Counter] = defaultdict(Counter)
    for observation, keys in zip(observations, dependencies):
        frame_key = (int(observation.event), int(observation.cam))
        support_before.update(keys)
        frame_support[frame_key].update(keys)
    minimum_support = {
        key: min(int(prune_options.minimum_observations_per_variable),
                 int(support_before[key]))
        for key in free_keys
    }
    support_after = Counter(support_before)
    removed = []
    blocked = []
    for frame_key in candidates:
        if len(removed) >= max_pruned_frames:
            blocked.append((frame_key, "maximum_fraction"))
            continue
        unsafe = [
            key for key, decrement in frame_support[frame_key].items()
            if support_after[key] - decrement < minimum_support[key]
        ]
        if unsafe:
            blocked.append((
                frame_key,
                "coverage_guard:" + ",".join(
                    f"{key[0]}:{key[1]}" for key in sorted(unsafe)),
            ))
            continue
        removed.append(frame_key)
        support_after.subtract(frame_support[frame_key])

    removed_set = set(removed)
    kept = [
        observation for observation in observations
        if (int(observation.event), int(observation.cam)) not in removed_set
    ]
    pruned_observations = len(observations) - len(kept)

    def frame_record(frame_key: Tuple[int, int]) -> dict:
        frame = frames[frame_key]
        return {
            "frame": _frame_label(frame_key),
            "event_id": int(frame_key[0]),
            "camera_id": int(frame_key[1]),
            "rmse_px": float(frame["rmse_px"]),
            "n_corners": int(frame["n_corners"]),
            "n_observations": int(frame["n_observations"]),
            "markers": sorted(frame["markers"]),
        }

    return kept, {
        **base_report,
        "attempted": bool(removed),
        "reason": "candidate_frames_selected" if removed else "no_safe_candidate",
        "n_input_frames": int(len(frames)),
        "n_input_corners": int(population["corners"]),
        "median_frame_rmse_px": median,
        "mad_frame_rmse_px": mad,
        "robust_sigma_px": robust_sigma,
        "threshold_px": threshold,
        "maximum_pruned_frames": max_pruned_frames,
        "candidate_frames": [frame_record(key) for key in candidates],
        "pruned_frames": [frame_record(key) for key in removed],
        "blocked_frames": [
            {**frame_record(key), "reason": reason}
            for key, reason in blocked
        ],
        "n_pruned_frames": int(len(removed)),
        "n_pruned_observations": int(pruned_observations),
        "n_kept_frames": int(len(frames) - len(removed)),
        "n_kept_observations": int(len(kept)),
        "actual_pruned_frame_fraction": float(
            len(removed) / max(1, len(frames))),
        "support_before": {
            f"{key[0]}:{key[1]}": int(support_before[key])
            for key in sorted(free_keys)
        },
        "support_after": {
            f"{key[0]}:{key[1]}": int(support_after[key])
            for key in sorted(free_keys)
        },
    }


def _key_seed(key: TransformKey) -> int:
    raw = f"{key[0]}:{key[1]}".encode("utf-8")
    return int.from_bytes(hashlib.sha256(raw).digest()[:4], "little")


def perturbed_x(problem: CornerReprojectionProblem, seed: int,
                translation_mm: float, rotation_deg: float) -> np.ndarray:
    if int(seed) == 0:
        return problem.x0.copy()
    out = problem.x0.copy()
    for key, sl in problem.slices.items():
        rng = np.random.default_rng(np.random.SeedSequence([int(seed), _key_seed(key)]))
        out[sl.start:sl.start + 3] = (
            rng.normal(0.0, np.deg2rad(rotation_deg), 3)
            / float(problem.scaling.rotation_scale_rad)
        )
        out[sl.start + 3:sl.stop] = (
            rng.normal(0.0, translation_mm / 1000.0, 3)
            / float(problem.scaling.translation_scale_m)
        )
    return out


def jacobian_diagnostics(
    jacobian,
    n_params: int,
    column_factors: Optional[np.ndarray] = None,
    variable_keys_: Optional[Sequence[TransformKey]] = None,
    weak_direction_count: int = 0,
) -> dict:
    matrix = (jacobian.toarray() if hasattr(jacobian, "toarray")
              else np.asarray(jacobian, dtype=np.float64))
    if column_factors is not None:
        factors = np.asarray(column_factors, dtype=np.float64).reshape(-1)
        if matrix.shape[1] != len(factors):
            raise ValueError("Jacobian column-factor length mismatch")
        matrix = matrix * factors.reshape(1, -1)
    if weak_direction_count and variable_keys_ is not None:
        _, singular, right_vectors = np.linalg.svd(
            matrix, full_matrices=False, compute_uv=True)
    else:
        singular = np.linalg.svd(matrix, compute_uv=False)
        right_vectors = None
    largest = float(singular[0]) if len(singular) else 0.0
    tolerance = float(np.finfo(float).eps * max(matrix.shape) * largest)
    rank = int(np.sum(singular > tolerance))
    smallest = float(singular[rank - 1]) if rank else 0.0
    condition = float(largest / smallest) if smallest > 0 else float("inf")
    result = {
        "shape": [int(matrix.shape[0]), int(matrix.shape[1])],
        "rank": rank,
        "n_params": int(n_params),
        "nullity": int(n_params - rank),
        "rank_tolerance": tolerance,
        "largest_singular_value": largest,
        "smallest_identifiable_singular_value": smallest,
        "jacobian_condition_number": condition,
        "gauss_newton_hessian_condition_number": float(condition ** 2),
        "rank_deficient": bool(rank < n_params),
    }
    if right_vectors is not None:
        if 6 * len(variable_keys_) != matrix.shape[1]:
            raise ValueError("variable-key count does not match Jacobian columns")
        directions = []
        count = min(int(weak_direction_count), len(singular))
        for offset in range(1, count + 1):
            vector = np.asarray(right_vectors[-offset], dtype=np.float64)
            components = []
            for index, key in enumerate(variable_keys_):
                block = vector[6 * index:6 * (index + 1)]
                rotation_energy = float(np.sum(np.square(block[:3])))
                translation_energy = float(np.sum(np.square(block[3:])))
                components.append({
                    "variable": f"{key[0]}:{key[1]}",
                    "rotation_energy_fraction": rotation_energy,
                    "translation_energy_fraction": translation_energy,
                    "total_energy_fraction": rotation_energy + translation_energy,
                })
            components.sort(key=lambda item: item["total_energy_fraction"], reverse=True)
            directions.append({
                "rank_from_weakest": offset,
                "singular_value": float(singular[-offset]),
                "dominant_components": components[:min(8, len(components))],
            })
        result["weakest_directions"] = directions
    return result


def coordinate_change_factors(n_transforms: int, source: SE3Scaling,
                              target: SE3Scaling) -> np.ndarray:
    """Return ``dx_source / dx_target`` for the same physical perturbation."""
    source.validate()
    target.validate()
    block = np.array(
        [target.rotation_scale_rad / source.rotation_scale_rad] * 3
        + [target.translation_scale_m / source.translation_scale_m] * 3,
        dtype=np.float64,
    )
    return np.tile(block, int(n_transforms))


def solve_corner_reprojection(
    observations: Sequence[PixelObs],
    variable_keys_: Sequence[TransformKey],
    reference_state: PoseState,
    robot_T: Mapping[int, np.ndarray],
    K_map: Mapping[int, np.ndarray],
    D_map: Mapping[int, np.ndarray],
    gripper_cam_idx: int,
    options: SolverOptions = SolverOptions(),
    seed: int = 0,
    init_translation_mm: float = 0.0,
    init_rotation_deg: float = 0.0,
) -> Tuple[PoseState, dict]:
    """Solve one frozen-mask corner problem and return state + diagnostics."""
    options.validate()
    problem = CornerReprojectionProblem(
        observations, variable_keys_, reference_state, robot_T, K_map, D_map,
        gripper_cam_idx, scaling=options.scaling,
        residual_weighting=options.residual_weighting,
    )
    x0 = perturbed_x(problem, seed, init_translation_mm, init_rotation_deg)
    initial_objective_residual = problem.residual_vector(x0)
    initial_raw_residual = problem.raw_residual_vector(x0)
    started = time.perf_counter()
    solution = least_squares(
        problem.residual_vector,
        x0,
        method=options.method,
        loss=options.loss,
        f_scale=float(options.f_scale_px),
        x_scale=("jac" if options.x_scale_mode == "jac" else 1.0),
        jac_sparsity=problem.jacobian_sparsity(),
        max_nfev=int(options.max_nfev),
        xtol=float(options.xtol),
        ftol=float(options.ftol),
        gtol=float(options.gtol),
    )
    elapsed = float(time.perf_counter() - started)
    final_objective_residual = problem.residual_vector(solution.x)
    final_raw_residual = problem.raw_residual_vector(solution.x)
    state = problem.unpack(solution.x)
    # Optimizer-coordinate gradients/conditions cannot be compared across
    # parameter scalings.  Report an additional fixed physical coordinate:
    # one unit = 1 rad rotation or 0.5 m translation.
    common_diagnostic_scaling = SE3Scaling(
        rotation_scale_rad=1.0, translation_scale_m=0.5)
    common_factors = coordinate_change_factors(
        len(problem.variable_keys), options.scaling, common_diagnostic_scaling)
    optimizer_gradient = np.asarray(solution.grad, dtype=np.float64).reshape(-1)
    common_gradient = optimizer_gradient * common_factors
    diagnostics = {
        "backend": "canonical_corner_reprojection_v1",
        "objective_contract": {
            **VISUAL_OBJECTIVE_CONTRACT,
            "visual_loss": options.loss,
            "visual_f_scale_px": float(options.f_scale_px),
            "residual_weighting": options.residual_weighting,
            "observation_block_scale": (
                "1/sqrt(n_detected_corners)"
                if options.residual_weighting == RESIDUAL_WEIGHT_EQUAL_OBSERVATION
                else "1"),
            "scalar_term_weights_used": bool(
                options.residual_weighting
                == RESIDUAL_WEIGHT_EQUAL_OBSERVATION),
        },
        "solver_options": options.to_dict(),
        "success": bool(solution.success),
        "status": int(solution.status),
        "message": str(solution.message),
        "nfev": int(solution.nfev),
        "optimality": float(solution.optimality),
        "optimizer_coordinate_gradient_inf_norm": float(
            np.linalg.norm(optimizer_gradient, ord=np.inf)),
        "common_scaled_gradient_inf_norm": float(
            np.linalg.norm(common_gradient, ord=np.inf)),
        "active_mask": np.asarray(solution.active_mask).astype(int).tolist(),
        "elapsed_s": elapsed,
        "n_parameters": int(problem.n_params),
        "n_residuals": int(problem.n_residuals),
        "visual_residual_population": observation_population(
            observations, gripper_cam_idx),
        "freeze_manifest": freeze_manifest(reference_state, variable_keys_),
        "variable_keys": [f"{kind}:{idx}" for kind, idx in problem.variable_keys],
        "initial_reprojection_rmse_px": float(np.sqrt(
            np.mean(np.square(initial_raw_residual)))),
        "train_reprojection_rmse_px": float(np.sqrt(
            np.mean(np.square(final_raw_residual)))),
        "initial_objective_residual_rmse": float(np.sqrt(
            np.mean(np.square(initial_objective_residual)))),
        "train_objective_residual_rmse": float(np.sqrt(
            np.mean(np.square(final_objective_residual)))),
        "objective_block_costs": {
            "cost_definition": "0.5 * sum(rho((residual / scale)^2) * scale^2)",
            "visual": {
                "n_residual_components": int(len(final_objective_residual)),
                "loss": str(options.loss),
                "scale_px": float(options.f_scale_px),
                "residual_weighting": options.residual_weighting,
                "initial_raw_l2_cost": float(
                    0.5 * np.sum(np.square(initial_objective_residual))),
                "final_raw_l2_cost": float(
                    0.5 * np.sum(np.square(final_objective_residual))),
                "initial_robust_cost": robust_least_squares_cost(
                    initial_objective_residual, options.loss,
                    options.f_scale_px),
                "final_robust_cost": robust_least_squares_cost(
                    final_objective_residual, options.loss,
                    options.f_scale_px),
                "final_robust_cost_per_component": float(
                    robust_least_squares_cost(
                        final_objective_residual, options.loss,
                        options.f_scale_px)
                    / max(1, len(final_objective_residual))),
            },
            "fk": {
                "active": False,
                "n_factor_blocks": 0,
                "n_residual_components": 0,
                "final_robust_cost": 0.0,
                "fraction_of_total_robust_cost": 0.0,
            },
        },
        "cost": float(solution.cost),
        "jacobian": jacobian_diagnostics(solution.jac, problem.n_params),
        "common_scaled_jacobian": {
            "coordinate_rotation_scale_rad": float(
                common_diagnostic_scaling.rotation_scale_rad),
            "coordinate_translation_scale_m": float(
                common_diagnostic_scaling.translation_scale_m),
            **jacobian_diagnostics(
                solution.jac,
                problem.n_params,
                column_factors=common_factors,
                variable_keys_=problem.variable_keys,
                weak_direction_count=3,
            ),
        },
    }
    return state, diagnostics
