"""Pixel-level bundle adjustment with an optional, independently robust FK factor.

The visual term is identical whether the FK factor is disabled (A2) or enabled
(A4).  Visual and FK losses are converted to explicit least-squares residuals,
so SciPy's global ``loss`` option cannot accidentally apply one loss to both
terms.  The final A4 factor is

    r = Log(T_fk^-1 T_cube)
    E = rho(r.T Sigma^-1 r)

with twist order ``[rx, ry, rz, tx, ty, tz]`` and SI units (rad, metre).
"""

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Mapping, Optional, Sequence, Tuple

import numpy as np
from scipy.optimize import least_squares
from scipy.sparse import csr_matrix, lil_matrix, vstack as sparse_vstack
from scipy.spatial.transform import Rotation

from apriltag_cube import inv_T
from calibration_reprojection_backend import (
    CornerReprojectionProblem,
    PixelObs,
    PoseState,
    SE3Scaling,
    SolverOptions,
    TransformKey,
    coordinate_change_factors,
    freeze_manifest,
    jacobian_diagnostics,
    perturbed_x,
)


_LOSSES = {"linear", "huber", "soft_l1"}


def _rho(z: np.ndarray, loss: str) -> np.ndarray:
    """SciPy-compatible rho(z), excluding the common 1/2 cost factor."""
    z = np.asarray(z, dtype=np.float64)
    if loss == "linear":
        return z
    if loss == "soft_l1":
        return 2.0 * (np.sqrt(1.0 + z) - 1.0)
    if loss == "huber":
        return np.where(z <= 1.0, z, 2.0 * np.sqrt(z) - 1.0)
    raise ValueError(f"unsupported robust loss {loss!r}")


def robustify_elementwise(values: np.ndarray, loss: str, scale: float) -> np.ndarray:
    """Return residuals whose linear-LS cost equals an elementwise M-estimator."""
    if loss not in _LOSSES or float(scale) <= 0:
        raise ValueError("invalid robust loss or scale")
    values = np.asarray(values, dtype=np.float64)
    if loss == "linear":
        return values.copy()
    z = np.square(values / float(scale))
    ratio = np.ones_like(z)
    active = z > 1e-15
    ratio[active] = np.sqrt(_rho(z[active], loss) / z[active])
    return values * ratio


def robustify_block(values: np.ndarray, loss: str, scale: float) -> np.ndarray:
    """Robustify one vector block as rho(||values||^2 / scale^2)."""
    if loss not in _LOSSES or float(scale) <= 0:
        raise ValueError("invalid robust loss or scale")
    values = np.asarray(values, dtype=np.float64).reshape(-1)
    if loss == "linear":
        return values.copy()
    z = float(np.dot(values, values) / float(scale) ** 2)
    if z <= 1e-15:
        return values.copy()
    return values * float(np.sqrt(_rho(np.asarray([z]), loss)[0] / z))


def se3_log(transform: np.ndarray) -> np.ndarray:
    """SE(3) logarithm in ``[rotation-vector(rad), translation-twist(m)]`` order."""
    transform = np.asarray(transform, dtype=np.float64)
    if transform.shape != (4, 4) or not np.all(np.isfinite(transform)):
        raise ValueError("SE(3) transform must be a finite 4x4 matrix")
    omega = Rotation.from_matrix(transform[:3, :3]).as_rotvec()
    theta = float(np.linalg.norm(omega))
    skew = np.array([
        [0.0, -omega[2], omega[1]],
        [omega[2], 0.0, -omega[0]],
        [-omega[1], omega[0], 0.0],
    ], dtype=np.float64)
    if theta < 1e-7:
        v_inverse = np.eye(3) - 0.5 * skew + (1.0 / 12.0) * (skew @ skew)
    else:
        coefficient = (
            1.0 / theta ** 2
            - (1.0 + np.cos(theta)) / (2.0 * theta * np.sin(theta))
        )
        v_inverse = np.eye(3) - 0.5 * skew + coefficient * (skew @ skew)
    velocity = v_inverse @ transform[:3, 3]
    return np.concatenate([omega, velocity]).astype(np.float64)


def validate_covariance(covariance: np.ndarray) -> np.ndarray:
    covariance = np.asarray(covariance, dtype=np.float64)
    if covariance.shape != (6, 6) or not np.all(np.isfinite(covariance)):
        raise ValueError("FK covariance must be a finite 6x6 matrix")
    if not np.allclose(covariance, covariance.T, atol=1e-12, rtol=1e-9):
        raise ValueError("FK covariance must be symmetric")
    eigenvalues = np.linalg.eigvalsh(covariance)
    if float(np.min(eigenvalues)) <= 0.0:
        raise ValueError("FK covariance must be positive definite")
    return covariance


def diagonal_covariance(translation_std_mm: float, rotation_std_deg: float) -> np.ndarray:
    """Construct a shared diagonal covariance in the module's SI twist units."""
    t = float(translation_std_mm) / 1000.0
    r = np.deg2rad(float(rotation_std_deg))
    if t <= 0.0 or r <= 0.0:
        raise ValueError("FK standard deviations must be positive")
    return np.diag([r * r] * 3 + [t * t] * 3).astype(np.float64)


@dataclass(frozen=True)
class FKFactorSpec:
    """Configuration for A2/A4a/A4b/A4c.

    ``mode='none'`` is A2. ``mode='fixed_probe'`` is the legacy-compatible
    A4a fixed-weight quadratic anchor. ``mode='covariance'`` is A4b/A4c.
    """

    mode: str = "none"
    loss: str = "linear"
    robust_scale: float = 1.0
    fixed_lambda_px_per_mm: float = 0.0
    probe_lever_mm: float = 29.5

    def validate(self) -> None:
        if self.mode not in {"none", "fixed_probe", "covariance"}:
            raise ValueError(f"unknown FK factor mode {self.mode!r}")
        if self.loss not in _LOSSES or float(self.robust_scale) <= 0.0:
            raise ValueError("invalid FK robust loss configuration")
        if self.mode == "fixed_probe":
            if self.loss != "linear":
                raise ValueError("A4a fixed-probe factor must remain quadratic")
            if self.fixed_lambda_px_per_mm <= 0.0 or self.probe_lever_mm <= 0.0:
                raise ValueError("A4a requires positive lambda and probe lever")


def _probe_points(lever_mm: float) -> np.ndarray:
    lever = float(lever_mm) / 1000.0
    return np.asarray([
        [lever, 0.0, 0.0], [-lever, 0.0, 0.0],
        [0.0, lever, 0.0], [0.0, -lever, 0.0],
        [0.0, 0.0, lever], [0.0, 0.0, -lever],
    ], dtype=np.float64)


def _transform_points(transform: np.ndarray, points: np.ndarray) -> np.ndarray:
    transform = np.asarray(transform, dtype=np.float64)
    return points @ transform[:3, :3].T + transform[:3, 3]


class FactorizedFKProblem:
    """Common visual problem plus an optional FK factor on free cube poses."""

    def __init__(
        self,
        observations: Sequence[PixelObs],
        variable_keys_: Sequence[TransformKey],
        reference_state: PoseState,
        robot_T,
        K_map,
        D_map,
        gripper_cam_idx: int,
        visual_loss: str,
        visual_scale_px: float,
        fk_targets: Mapping[int, np.ndarray],
        fk_covariances: Mapping[int, np.ndarray],
        fk_spec: FKFactorSpec,
        scaling: SE3Scaling,
    ):
        if visual_loss not in _LOSSES or float(visual_scale_px) <= 0.0:
            raise ValueError("invalid visual loss configuration")
        fk_spec.validate()
        self.visual = CornerReprojectionProblem(
            observations, variable_keys_, reference_state, robot_T, K_map, D_map,
            gripper_cam_idx, scaling=scaling,
        )
        self.visual_loss = str(visual_loss)
        self.visual_scale_px = float(visual_scale_px)
        self.fk_targets = {
            int(key): np.asarray(value, dtype=np.float64)
            for key, value in fk_targets.items()
        }
        self.fk_spec = fk_spec
        self.factor_keys = [
            key for key in self.visual.variable_keys
            if key[0] == "cube" and int(key[1]) in self.fk_targets
        ] if fk_spec.mode != "none" else []
        self.whiteners = {}
        if fk_spec.mode == "covariance":
            for _, set_index in self.factor_keys:
                if int(set_index) not in fk_covariances:
                    raise ValueError(f"FK covariance missing for set {set_index}")
                covariance = validate_covariance(fk_covariances[int(set_index)])
                self.whiteners[int(set_index)] = np.linalg.inv(np.linalg.cholesky(covariance))
        self.probe = _probe_points(fk_spec.probe_lever_mm)
        self.factor_rows_per_key = 0
        if fk_spec.mode == "fixed_probe":
            self.factor_rows_per_key = int(self.probe.size)
        elif fk_spec.mode == "covariance":
            self.factor_rows_per_key = 6

    @property
    def n_params(self) -> int:
        return self.visual.n_params

    @property
    def variable_keys(self):
        return self.visual.variable_keys

    @property
    def x0(self):
        return self.visual.x0

    def unpack(self, x: np.ndarray) -> PoseState:
        return self.visual.unpack(x)

    def visual_raw(self, x: np.ndarray) -> np.ndarray:
        return self.visual.residual_vector(x)

    def visual_residual(self, x: np.ndarray) -> np.ndarray:
        return robustify_elementwise(
            self.visual_raw(x), self.visual_loss, self.visual_scale_px)

    def factor_raw_blocks(self, x: np.ndarray):
        if not self.factor_keys:
            return []
        state = self.unpack(x)
        blocks = []
        for _, set_index in self.factor_keys:
            estimated = state.cubes[int(set_index)]
            target = self.fk_targets[int(set_index)]
            if self.fk_spec.mode == "fixed_probe":
                delta_mm = (
                    _transform_points(estimated, self.probe)
                    - _transform_points(target, self.probe)
                ) * 1000.0
                block = float(self.fk_spec.fixed_lambda_px_per_mm) * delta_mm.reshape(-1)
            elif self.fk_spec.mode == "covariance":
                tangent = se3_log(inv_T(target) @ estimated)
                block = self.whiteners[int(set_index)] @ tangent
            else:
                raise RuntimeError("factor block requested with mode=none")
            blocks.append((int(set_index), np.asarray(block, dtype=np.float64)))
        return blocks

    def factor_residual(self, x: np.ndarray) -> np.ndarray:
        chunks = [
            robustify_block(block, self.fk_spec.loss, self.fk_spec.robust_scale)
            for _, block in self.factor_raw_blocks(x)
        ]
        return np.concatenate(chunks) if chunks else np.zeros(0, dtype=np.float64)

    def residual(self, x: np.ndarray) -> np.ndarray:
        factor = self.factor_residual(x)
        visual = self.visual_residual(x)
        return visual if not len(factor) else np.concatenate([visual, factor])

    def jacobian_sparsity(self) -> csr_matrix:
        visual = self.visual.jacobian_sparsity()
        if not self.factor_keys:
            return visual
        extra = lil_matrix(
            (len(self.factor_keys) * self.factor_rows_per_key, self.n_params),
            dtype=np.int8,
        )
        for index, key in enumerate(self.factor_keys):
            rows = slice(
                index * self.factor_rows_per_key,
                (index + 1) * self.factor_rows_per_key,
            )
            extra[rows, self.visual.slices[key]] = 1
        return csr_matrix(sparse_vstack([visual, extra.tocsr()]))


def solve_factorized_fk(
    observations: Sequence[PixelObs],
    variable_keys_: Sequence[TransformKey],
    reference_state: PoseState,
    robot_T,
    K_map,
    D_map,
    gripper_cam_idx: int,
    options: SolverOptions,
    fk_targets: Optional[Mapping[int, np.ndarray]] = None,
    fk_covariances: Optional[Mapping[int, np.ndarray]] = None,
    fk_spec: FKFactorSpec = FKFactorSpec(),
    seed: int = 0,
    init_translation_mm: float = 0.0,
    init_rotation_deg: float = 0.0,
) -> Tuple[PoseState, dict]:
    """Solve A2/A4 with explicitly separated visual and FK robust losses."""
    options.validate()
    problem = FactorizedFKProblem(
        observations=observations,
        variable_keys_=variable_keys_,
        reference_state=reference_state,
        robot_T=robot_T,
        K_map=K_map,
        D_map=D_map,
        gripper_cam_idx=gripper_cam_idx,
        visual_loss=options.loss,
        visual_scale_px=options.f_scale_px,
        fk_targets={} if fk_targets is None else fk_targets,
        fk_covariances={} if fk_covariances is None else fk_covariances,
        fk_spec=fk_spec,
        scaling=options.scaling,
    )
    x0 = perturbed_x(problem.visual, seed, init_translation_mm, init_rotation_deg)
    initial_visual = problem.visual_raw(x0)
    started = time.perf_counter()
    solution = least_squares(
        problem.residual,
        x0,
        method=options.method,
        loss="linear",
        f_scale=1.0,
        x_scale=("jac" if options.x_scale_mode == "jac" else 1.0),
        jac_sparsity=problem.jacobian_sparsity(),
        max_nfev=int(options.max_nfev),
        xtol=float(options.xtol),
        ftol=float(options.ftol),
        gtol=float(options.gtol),
    )
    elapsed = float(time.perf_counter() - started)
    state = problem.unpack(solution.x)
    final_visual = problem.visual_raw(solution.x)
    factor_blocks = problem.factor_raw_blocks(solution.x)
    common_scaling = SE3Scaling(rotation_scale_rad=1.0, translation_scale_m=0.5)
    common_factors = coordinate_change_factors(
        len(problem.variable_keys), options.scaling, common_scaling)
    diagnostics = {
        "backend": "factorized_visual_plus_fk_v1",
        "objective_contract": {
            "visual_loss": options.loss,
            "visual_f_scale_px": float(options.f_scale_px),
            "fk_mode": fk_spec.mode,
            "fk_loss": fk_spec.loss,
            "fk_robust_scale": float(fk_spec.robust_scale),
            "scipy_global_loss": "linear",
            "reason": "visual and FK M-estimators are encoded separately",
        },
        "solver_options": options.to_dict(),
        "success": bool(solution.success),
        "status": int(solution.status),
        "message": str(solution.message),
        "nfev": int(solution.nfev),
        "optimality": float(solution.optimality),
        "elapsed_s": elapsed,
        "n_parameters": int(problem.n_params),
        "n_visual_residuals": int(problem.visual.n_residuals),
        "n_fk_residuals": int(sum(len(block) for _, block in factor_blocks)),
        "freeze_manifest": freeze_manifest(reference_state, variable_keys_),
        "variable_keys": [f"{kind}:{idx}" for kind, idx in problem.variable_keys],
        "initial_reprojection_rmse_px": float(np.sqrt(np.mean(np.square(initial_visual)))),
        "train_reprojection_rmse_px": float(np.sqrt(np.mean(np.square(final_visual)))),
        "fk_factor": {
            "active": bool(factor_blocks),
            "sets": [int(set_index) for set_index, _ in factor_blocks],
            "raw_mahalanobis_or_weighted_probe_norm": {
                str(set_index): float(np.linalg.norm(block))
                for set_index, block in factor_blocks
            },
        },
        "cost": float(solution.cost),
        "jacobian": jacobian_diagnostics(solution.jac, problem.n_params),
        "common_scaled_jacobian": jacobian_diagnostics(
            solution.jac,
            problem.n_params,
            column_factors=common_factors,
            variable_keys_=problem.variable_keys,
            weak_direction_count=3,
        ),
    }
    return state, diagnostics
