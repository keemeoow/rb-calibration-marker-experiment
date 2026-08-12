"""Board-free FK-cube alignment from train eye-in-hand cube corners only.

The measurement model is

    T_base_gripper[e] T_gripper_cam T_cam_object[e]
      = T_base_fk_cube_raw[s] T_fk_cube_raw_object.

Only the two constant transforms on the right/left are optimized.  ChArUco
board observations, fixed-camera observations, held-out events, and production
calibration transforms are forbidden inputs.
"""
from __future__ import annotations

import hashlib
import json
from collections import defaultdict
from typing import Dict, Mapping, Optional, Sequence, Tuple

import cv2
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

import CP_common as cp
from apriltag_cube import inv_T
from calibration_reprojection_backend import (
    PixelObs,
    SE3Scaling,
    SolverOptions,
    coordinate_change_factors,
    jacobian_diagnostics,
    pose_delta,
    project_points,
    retract,
)


def _solve_pnp(obs: PixelObs, K_map, D_map) -> Optional[np.ndarray]:
    obj = np.asarray(obs.object_points, dtype=np.float64).reshape(-1, 3)
    pix = np.asarray(obs.image_points, dtype=np.float64).reshape(-1, 2)
    if len(obj) < 4 or len(obj) != len(pix):
        return None
    ok, rvec, tvec = cv2.solvePnP(
        obj, pix, np.asarray(K_map[int(obs.cam)], dtype=np.float64),
        np.asarray(D_map[int(obs.cam)], dtype=np.float64),
        flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok:
        return None
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = cv2.Rodrigues(rvec)[0]
    T[:3, 3] = np.asarray(tvec, dtype=np.float64).reshape(3)
    return T


def _observation_digest(observations: Sequence[PixelObs]) -> str:
    hasher = hashlib.sha256()
    for obs in sorted(observations, key=lambda o: (int(o.event), int(o.cam), int(o.set_idx))):
        header = f"{int(obs.event)}:{int(obs.cam)}:{int(obs.set_idx)}".encode("utf-8")
        hasher.update(header)
        hasher.update(np.asarray(obs.object_points, dtype="<f8").tobytes())
        hasher.update(np.asarray(obs.image_points, dtype="<f8").tobytes())
    return hasher.hexdigest()


def _absolute_pose(vector: np.ndarray) -> np.ndarray:
    """Convert an absolute rotvec/translation parameter to an SE(3) matrix."""
    vector = np.asarray(vector, dtype=np.float64).reshape(6)
    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = Rotation.from_rotvec(vector[:3]).as_matrix()
    transform[:3, 3] = vector[3:]
    return transform


def _global_ax_zb_initial_candidate(pnp_items, raw_by_set, robot_T) -> dict:
    """Board-free low-N initializer for ``A_i X = Z B_i``.

    The legacy initializer needs at least five motions *within one cube set*.
    A global view-budget experiment can instead have one observation in each
    of several sets.  With raw FK cube poses known, every observation still
    constrains the same ``T_gripper_cam`` and FK-to-object delta.  This helper
    minimizes only that cube-PnP pose consistency to obtain an initializer;
    the returned transforms are subsequently refined by the canonical corner
    reprojection objective and are never used as a reported result.
    """
    usable = [
        (int(obs.event), int(obs.set_idx), np.asarray(T_cam_object, float))
        for obs, T_cam_object in pnp_items
        if T_cam_object is not None
        and int(obs.event) in robot_T
        and int(obs.set_idx) in raw_by_set
    ]
    if len(usable) < 3 or len({item[1] for item in usable}) < 3:
        raise RuntimeError(
            "global board-free initializer needs >=3 observations in >=3 sets")

    translation_scale_m = 0.1

    def residual(parameters: np.ndarray) -> np.ndarray:
        gtc = _absolute_pose(parameters[:6])
        delta = _absolute_pose(parameters[6:])
        chunks = []
        for event, set_index, T_cam_object in usable:
            error = (
                inv_T(np.asarray(raw_by_set[set_index], dtype=np.float64))
                @ np.asarray(robot_T[event], dtype=np.float64)
                @ gtc @ T_cam_object @ inv_T(delta))
            chunks.append(Rotation.from_matrix(error[:3, :3]).as_rotvec())
            chunks.append(error[:3, 3] / translation_scale_m)
        return np.concatenate(chunks)

    candidates = []
    # Identity plus deterministic broad rotation starts handle the large,
    # known axis-convention changes without importing any full-data artifact.
    for seed in range(12):
        initial = np.zeros(12, dtype=np.float64)
        if seed:
            rng = np.random.default_rng(
                np.random.SeedSequence([0xA12B, int(seed)]))
            initial[:3] = rng.normal(0.0, 2.0, 3)
            initial[6:9] = rng.normal(0.0, 2.0, 3)
        solution = least_squares(
            residual, initial, method="trf", loss="linear", x_scale="jac",
            max_nfev=500, xtol=1e-10, ftol=1e-10, gtol=1e-10)
        value = residual(solution.x)
        rms = float(np.sqrt(np.mean(np.square(value))))
        if np.isfinite(rms):
            candidates.append((rms, int(seed), solution))
    if not candidates:
        raise RuntimeError("global board-free initializer failed for every start")
    rms, seed, solution = min(candidates, key=lambda item: item[0])
    return {
        "method": "GLOBAL_AX_ZB_NONLINEAR",
        "pose_residual_rms": rms,
        "selected_start_seed": seed,
        "n_starts": len(candidates),
        "used_sets": sorted({item[1] for item in usable}),
        "failed_sets": {},
        "per_set_handeye_count": 0,
        "T_gripper_cam": _absolute_pose(solution.x[:6]),
        "T_fk_cube_center_to_tag_object": _absolute_pose(solution.x[6:]),
    }


def _initial_candidates(observations: Sequence[PixelObs], raw_by_set,
                        robot_T, K_map, D_map) -> Tuple[np.ndarray, np.ndarray, dict]:
    by_set = defaultdict(list)
    pnp_by_event = {}
    for obs in observations:
        T_cam_object = _solve_pnp(obs, K_map, D_map)
        if T_cam_object is None:
            continue
        by_set[int(obs.set_idx)].append((int(obs.event), T_cam_object))
        pnp_by_event[(int(obs.event), int(obs.set_idx))] = T_cam_object
    methods = {
        "TSAI": cv2.CALIB_HAND_EYE_TSAI,
        "PARK": cv2.CALIB_HAND_EYE_PARK,
        "HORAUD": cv2.CALIB_HAND_EYE_HORAUD,
        "DANIILIDIS": cv2.CALIB_HAND_EYE_DANIILIDIS,
    }
    method_results = []
    for method_name, method in methods.items():
        per_set_gtc = []
        used_sets = []
        failures = {}
        for set_index, poses in sorted(by_set.items()):
            poses = [(event, T) for event, T in poses if event in robot_T]
            if len(poses) < 5:
                failures[str(set_index)] = f"support={len(poses)}<5"
                continue
            try:
                R, t = cv2.calibrateHandEye(
                    [np.asarray(robot_T[event])[:3, :3] for event, _ in poses],
                    [np.asarray(robot_T[event])[:3, 3].reshape(3, 1)
                     for event, _ in poses],
                    [T[:3, :3] for _, T in poses],
                    [T[:3, 3].reshape(3, 1) for _, T in poses],
                    method=method,
                )
                R = np.asarray(R, dtype=np.float64).reshape(3, 3)
                determinant = float(np.linalg.det(R))
                orthogonality_error = float(np.linalg.norm(R.T @ R - np.eye(3)))
                if determinant <= 0.0 or orthogonality_error > 1e-5:
                    raise ValueError(
                        "hand-eye returned an invalid rotation "
                        f"(det={determinant:.6g}, ortho={orthogonality_error:.6g})")
                T_gtc = np.eye(4, dtype=np.float64)
                T_gtc[:3, :3] = R
                T_gtc[:3, 3] = np.asarray(t, dtype=np.float64).reshape(3)
                if not np.all(np.isfinite(T_gtc)):
                    raise ValueError("non-finite hand-eye result")
                per_set_gtc.append(T_gtc)
                used_sets.append(int(set_index))
            except Exception as exc:
                failures[str(set_index)] = str(exc)
        if not per_set_gtc:
            continue
        try:
            gtc = cp.robust_se3_average(per_set_gtc, None)[0]
        except (ValueError, np.linalg.LinAlgError):
            # Degenerate low-resolution PnP/hand-eye candidates must not
            # prevent the independent global AX=ZB fallback below.
            continue
        deltas = []
        for (event, set_index), T_cam_object in sorted(pnp_by_event.items()):
            if event not in robot_T or set_index not in raw_by_set:
                continue
            deltas.append(
                inv_T(np.asarray(raw_by_set[set_index], dtype=np.float64))
                @ np.asarray(robot_T[event], dtype=np.float64)
                @ gtc @ T_cam_object)
        if not deltas:
            continue
        fk_delta = cp.robust_se3_average(deltas, None)[0]
        residual_chunks = []
        for obs in observations:
            target = np.asarray(raw_by_set[int(obs.set_idx)]) @ fk_delta
            T_base_cam = np.asarray(robot_T[int(obs.event)]) @ gtc
            prediction = project_points(
                inv_T(T_base_cam) @ target, obs.object_points,
                K_map[int(obs.cam)], D_map[int(obs.cam)])
            residual_chunks.append(
                (prediction - np.asarray(obs.image_points).reshape(-1, 2)).reshape(-1))
        residual = np.concatenate(residual_chunks)
        score = float(np.sqrt(np.mean(np.square(residual))))
        method_results.append({
            "method": method_name,
            "score_reprojection_rmse_px": score,
            "used_sets": used_sets,
            "failed_sets": failures,
            "per_set_handeye_count": len(per_set_gtc),
            "T_gripper_cam": gtc,
            "T_fk_cube_center_to_tag_object": fk_delta,
        })
    if not method_results:
        # Low global view budgets may have <5 poses in every individual set.
        # Use all sets jointly without board or held-out information.
        fallback = _global_ax_zb_initial_candidate(
            [(obs, pnp_by_event.get((int(obs.event), int(obs.set_idx))))
             for obs in observations],
            raw_by_set, robot_T)
        residual_chunks = []
        for obs in observations:
            set_index = int(obs.set_idx)
            event = int(obs.event)
            if set_index not in raw_by_set or event not in robot_T:
                continue
            target = (np.asarray(raw_by_set[set_index], dtype=np.float64)
                      @ fallback["T_fk_cube_center_to_tag_object"])
            T_base_cam = (np.asarray(robot_T[event], dtype=np.float64)
                          @ fallback["T_gripper_cam"])
            prediction = project_points(
                inv_T(T_base_cam) @ target, obs.object_points,
                K_map[int(obs.cam)], D_map[int(obs.cam)])
            residual_chunks.append(
                (prediction - np.asarray(obs.image_points).reshape(-1, 2)).reshape(-1))
        if not residual_chunks:
            raise RuntimeError("global board-free initializer has no corner residuals")
        fallback["score_reprojection_rmse_px"] = float(np.sqrt(np.mean(
            np.square(np.concatenate(residual_chunks)))))
        method_results.append(fallback)
    best = min(method_results, key=lambda item: item["score_reprojection_rmse_px"])
    diagnostics = {
        "selected_method": best["method"],
        "candidates": [{
            key: (value.tolist() if isinstance(value, np.ndarray) else value)
            for key, value in item.items()
            if key not in {"T_gripper_cam", "T_fk_cube_center_to_tag_object"}
        } for item in method_results],
    }
    return best["T_gripper_cam"], best["T_fk_cube_center_to_tag_object"], diagnostics


class BoardFreeFKCubeProblem:
    def __init__(self, observations: Sequence[PixelObs], raw_by_set,
                 robot_T, K_map, D_map, gripper_cam_idx: int,
                 reference_gtc: np.ndarray, reference_delta: np.ndarray,
                 scaling: SE3Scaling):
        self.observations = list(observations)
        self.raw = raw_by_set
        self.robot_T = robot_T
        self.K = K_map
        self.D = D_map
        self.gripper = int(gripper_cam_idx)
        self.reference_gtc = np.asarray(reference_gtc, dtype=np.float64)
        self.reference_delta = np.asarray(reference_delta, dtype=np.float64)
        self.scaling = scaling
        self.x0 = np.zeros(12, dtype=np.float64)
        if not self.observations:
            raise RuntimeError("board-free FK-cube problem has no observations")
        self.residual(self.x0)

    def unpack(self, x: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        x = np.asarray(x, dtype=np.float64).reshape(12)
        return (
            retract(self.reference_gtc, x[:6], self.scaling),
            retract(self.reference_delta, x[6:], self.scaling),
        )

    def residual(self, x: np.ndarray) -> np.ndarray:
        gtc, delta = self.unpack(x)
        chunks = []
        for obs in self.observations:
            if obs.marker != "cube" or int(obs.cam) != self.gripper:
                raise RuntimeError("board-free artifact received a non-eih-cube observation")
            event, set_index = int(obs.event), int(obs.set_idx)
            if event not in self.robot_T or set_index not in self.raw:
                raise RuntimeError("board-free artifact observation lacks raw FK support")
            T_base_cam = np.asarray(self.robot_T[event], dtype=np.float64) @ gtc
            T_base_object = np.asarray(self.raw[set_index], dtype=np.float64) @ delta
            prediction = project_points(
                inv_T(T_base_cam) @ T_base_object,
                obs.object_points, self.K[self.gripper], self.D[self.gripper])
            chunks.append(
                (prediction - np.asarray(obs.image_points).reshape(-1, 2)).reshape(-1))
        return np.concatenate(chunks).astype(np.float64)


def _perturbed_x(seed: int, scaling: SE3Scaling,
                 translation_mm: float, rotation_deg: float) -> np.ndarray:
    if int(seed) == 0:
        return np.zeros(12, dtype=np.float64)
    rng = np.random.default_rng(np.random.SeedSequence([int(seed), 0xB04DF33]))
    out = np.zeros(12, dtype=np.float64)
    for start in (0, 6):
        out[start:start + 3] = (
            rng.normal(0.0, np.deg2rad(rotation_deg), 3)
            / scaling.rotation_scale_rad)
        out[start + 3:start + 6] = (
            rng.normal(0.0, translation_mm / 1000.0, 3)
            / scaling.translation_scale_m)
    return out


def _solve(problem: BoardFreeFKCubeProblem, options: SolverOptions,
           seed: int, init_translation_mm: float,
           init_rotation_deg: float) -> Tuple[np.ndarray, np.ndarray, dict]:
    x0 = _perturbed_x(
        seed, options.scaling, init_translation_mm, init_rotation_deg)
    r0 = problem.residual(x0)
    solution = least_squares(
        problem.residual, x0, method=options.method, loss=options.loss,
        f_scale=options.f_scale_px,
        x_scale=("jac" if options.x_scale_mode == "jac" else 1.0),
        max_nfev=options.max_nfev,
        xtol=options.xtol, ftol=options.ftol, gtol=options.gtol)
    gtc, delta = problem.unpack(solution.x)
    final = problem.residual(solution.x)
    common = SE3Scaling(rotation_scale_rad=1.0, translation_scale_m=0.5)
    factors = coordinate_change_factors(2, options.scaling, common)
    gradient = np.asarray(solution.grad, dtype=np.float64).reshape(-1)
    return gtc, delta, {
        "seed": int(seed),
        "success": bool(solution.success),
        "status": int(solution.status),
        "message": str(solution.message),
        "nfev": int(solution.nfev),
        "optimality": float(solution.optimality),
        "common_scaled_gradient_inf_norm": float(
            np.linalg.norm(gradient * factors, ord=np.inf)),
        "initial_reprojection_rmse_px": float(np.sqrt(np.mean(np.square(r0)))),
        "train_reprojection_rmse_px": float(np.sqrt(np.mean(np.square(final)))),
        "cost": float(solution.cost),
        "jacobian": jacobian_diagnostics(
            solution.jac, 12, column_factors=factors,
            variable_keys_=(('gtc', -1), ('fk_delta', -1)),
            weak_direction_count=3),
        "T_gripper_cam": gtc.tolist(),
        "T_fk_cube_center_to_tag_object": delta.tolist(),
    }


def estimate_board_free_fk_cube_artifact(
    observations: Sequence[PixelObs],
    raw_fk_by_set: Mapping[int, np.ndarray],
    robot_T: Mapping[int, np.ndarray],
    K_map: Mapping[int, np.ndarray],
    D_map: Mapping[int, np.ndarray],
    gripper_cam_idx: int,
    training_set_ids: Sequence[int],
    options: SolverOptions = SolverOptions(),
    num_inits: int = 3,
    init_translation_mm: float = 5.0,
    init_rotation_deg: float = 1.0,
    raw_fk_source_event_by_set: Optional[Mapping[int, int]] = None,
) -> Tuple[Dict[int, np.ndarray], np.ndarray, dict]:
    """Return aligned FK poses, board-free gTc, and hashed provenance artifact."""
    options.validate()
    allowed_sets = {int(s) for s in training_set_ids}
    raw = {int(s): np.asarray(T, dtype=np.float64)
           for s, T in raw_fk_by_set.items()}
    raw_source_events = {
        str(s): (int(raw_fk_source_event_by_set[s])
                 if raw_fk_source_event_by_set is not None
                 and s in raw_fk_source_event_by_set else "external_or_synthetic")
        for s in sorted(raw)
    }
    usable = [obs for obs in observations
              if obs.marker == "cube"
              and int(obs.cam) == int(gripper_cam_idx)
              and obs.set_idx is not None
              and int(obs.set_idx) in allowed_sets
              and int(obs.set_idx) in raw
              and int(obs.event) in robot_T]
    if not usable:
        raise RuntimeError("no train eih cube observations for board-free artifact")
    initial_gtc, initial_delta, init_diag = _initial_candidates(
        usable, raw, robot_T, K_map, D_map)
    problem = BoardFreeFKCubeProblem(
        usable, raw, robot_T, K_map, D_map, gripper_cam_idx,
        initial_gtc, initial_delta, options.scaling)
    runs = []
    for seed in range(int(num_inits)):
        _, _, diag = _solve(
            problem, options, seed, init_translation_mm, init_rotation_deg)
        runs.append(diag)
    if not all(run["success"] for run in runs):
        raise RuntimeError(
            "board-free FK artifact convergence failed: "
            + json.dumps([{k: run[k] for k in ("seed", "status", "message", "nfev")}
                          for run in runs]))
    if any(run["jacobian"]["rank_deficient"] for run in runs):
        raise RuntimeError("board-free FK artifact is rank deficient")
    nominal = runs[0]
    gtc = np.asarray(nominal["T_gripper_cam"], dtype=np.float64)
    delta = np.asarray(
        nominal["T_fk_cube_center_to_tag_object"], dtype=np.float64)
    gtc_dispersion = [pose_delta(gtc, np.asarray(run["T_gripper_cam"])) for run in runs]
    delta_dispersion = [pose_delta(
        delta, np.asarray(run["T_fk_cube_center_to_tag_object"])) for run in runs]
    aligned_all = {int(s): T @ delta for s, T in raw.items()}
    per_set_support = {}
    for set_index in sorted(allowed_sets):
        subset = [obs for obs in usable if int(obs.set_idx) == set_index]
        per_set_support[str(set_index)] = {
            "observations": len(subset),
            "corners": int(sum(len(np.asarray(obs.image_points).reshape(-1, 2))
                               for obs in subset)),
            "events": sorted({int(obs.event) for obs in subset}),
        }
    body = {
        "artifact_schema": "board_free_fk_cube_alignment_v1",
        "canonical_for_rows": ["A3", "B1", "B2"],
        "board_information_used": False,
        "board_observation_count": 0,
        "heldout_information_used": False,
        "estimation_equation": (
            "T_base_gripper[event] @ T_gripper_cam @ T_cam_object[event] "
            "= T_base_fk_cube_raw[set] @ T_fk_cube_center_to_tag_object"),
        "estimation_method": (
            "board_free_cube_pose_initialization_then_joint_eih_cube_corner_reprojection"),
        "raw_fk_pose_source": "meta.set_cube_center_6dof from robot controller tool 4",
        "raw_fk_source_event_by_set": raw_source_events,
        "training_set_ids": sorted(allowed_sets),
        "source_observation_ids": sorted({int(obs.event) for obs in usable}),
        "source_observation_sha256": _observation_digest(usable),
        "source_counts": {
            "observations": len(usable),
            "corners": int(sum(len(np.asarray(obs.image_points).reshape(-1, 2))
                               for obs in usable)),
            "sets": len({int(obs.set_idx) for obs in usable}),
        },
        "per_set_support": per_set_support,
        "solver_options": options.to_dict(),
        "initialization": init_diag,
        "runs": runs,
        "repeatability": {
            "T_gripper_cam_max_translation_mm": float(max(v[0] for v in gtc_dispersion)),
            "T_gripper_cam_max_rotation_deg": float(max(v[1] for v in gtc_dispersion)),
            "T_fk_delta_max_translation_mm": float(max(v[0] for v in delta_dispersion)),
            "T_fk_delta_max_rotation_deg": float(max(v[1] for v in delta_dispersion)),
        },
        "T_gripper_cam": gtc.tolist(),
        "T_fk_cube_center_to_tag_object": delta.tolist(),
        "raw_fk_pose_by_set": {str(s): T.tolist() for s, T in sorted(raw.items())},
        "aligned_fk_pose_by_set": {
            str(s): T.tolist() for s, T in sorted(aligned_all.items())},
    }
    encoded = json.dumps(body, sort_keys=True, separators=(",", ":")).encode("utf-8")
    body["artifact_sha256"] = hashlib.sha256(encoded).hexdigest()
    return aligned_all, gtc, body
