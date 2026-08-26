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
"""

from __future__ import annotations

import hashlib
import time
from dataclasses import dataclass, field
from typing import Dict, List, Mapping, Optional, Sequence, Tuple

import cv2
import numpy as np
from scipy.optimize import least_squares
from scipy.sparse import lil_matrix
from scipy.spatial.transform import Rotation

from calibration_pipeline.apriltag_cube import inv_T


TransformKey = Tuple[str, int]

# How a gripped-cube observation gets its target pose.  This is the FK axis in
# its sharpest form: the cube is bolted to the gripper, so either FK carries it
# (one constant per grasp) or it does not (six free DoF per event).  A placement
# cannot pose the question this cleanly — its only FK link is a single contact
# measurement taken once per set.
GRIPPED_TARGET_GRASP = "grasp"        # T_base_cube[e] = FK(q_e) @ T_gripper_cube[g]
GRIPPED_TARGET_EVENT = "event_free"   # T_base_cube[e] free, FK unused
GRIPPED_TARGET_MODELS = frozenset({GRIPPED_TARGET_GRASP, GRIPPED_TARGET_EVENT})


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

    def validate(self) -> None:
        if self.method != "trf":
            raise ValueError("canonical backend currently requires method='trf'")
        if self.loss not in {"linear", "huber", "soft_l1"}:
            raise ValueError("canonical backend loss must be linear, huber, or soft_l1")
        if self.x_scale_mode not in {"unit", "jac"}:
            raise ValueError("x_scale_mode must be 'unit' or 'jac'")
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
                 gripped_target: str = GRIPPED_TARGET_GRASP):
        scaling.validate()
        if gripped_target not in GRIPPED_TARGET_MODELS:
            raise ValueError(
                f"gripped_target must be one of {sorted(GRIPPED_TARGET_MODELS)}")
        self.gripped_target = str(gripped_target)
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
        row = 0
        for obs in self.obs:
            n = 2 * len(np.asarray(obs.image_points).reshape(-1, 2))
            self.row_offsets.append((row, row + n))
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

    def residual_vector(self, x: np.ndarray) -> np.ndarray:
        state = self.unpack(x)
        chunks = []
        for obs in self.obs:
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
            chunks.append((prediction - np.asarray(obs.image_points).reshape(-1, 2)).reshape(-1))
        return np.concatenate(chunks).astype(np.float64)

    # Alias keeps scipy call sites terse and makes the public residual explicit.
    residual = residual_vector

    def jacobian_sparsity(self):
        matrix = lil_matrix((self.n_residuals, self.n_params), dtype=np.int8)
        for obs, (r0, r1) in zip(self.obs, self.row_offsets):
            camera_key = (("gtc", -1) if int(obs.cam) == self.gripper
                          else ("cam", int(obs.cam)))
            if obs.marker == "board":
                target_key = ("board", -1)
            elif obs.grasp_idx is not None:
                target_key = (("grasp", int(obs.grasp_idx))
                              if self.gripped_target == GRIPPED_TARGET_GRASP
                              else ("cube_event", int(obs.event)))
            else:
                target_key = ("cube", int(obs.set_idx))
            if camera_key in self.slices:
                matrix[r0:r1, self.slices[camera_key]] = 1
            if target_key in self.slices:
                matrix[r0:r1, self.slices[target_key]] = 1
        return matrix.tocsr()


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
    )
    x0 = perturbed_x(problem, seed, init_translation_mm, init_rotation_deg)
    initial_residual = problem.residual_vector(x0)
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
    final_residual = problem.residual_vector(solution.x)
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
        "initial_reprojection_rmse_px": float(np.sqrt(np.mean(np.square(initial_residual)))),
        "train_reprojection_rmse_px": float(np.sqrt(np.mean(np.square(final_residual)))),
        "objective_block_costs": {
            "cost_definition": "0.5 * sum(rho((residual / scale)^2) * scale^2)",
            "visual": {
                "n_residual_components": int(len(final_residual)),
                "loss": str(options.loss),
                "scale_px": float(options.f_scale_px),
                "initial_raw_l2_cost": float(
                    0.5 * np.sum(np.square(initial_residual))),
                "final_raw_l2_cost": float(
                    0.5 * np.sum(np.square(final_residual))),
                "initial_robust_cost": robust_least_squares_cost(
                    initial_residual, options.loss, options.f_scale_px),
                "final_robust_cost": robust_least_squares_cost(
                    final_residual, options.loss, options.f_scale_px),
                "final_robust_cost_per_component": float(
                    robust_least_squares_cost(
                        final_residual, options.loss, options.f_scale_px)
                    / max(1, len(final_residual))),
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
