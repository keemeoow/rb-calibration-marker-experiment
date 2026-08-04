#!/usr/bin/env python3
"""D1 — 2x2 factorial: {FK-fixed, vision-estimated} x {no correction, correction}.

Motivation
----------
The seven-row ablation (``CP_ablation_7row.py``) settles the *calibration*
question on an event-level hold-out, and the C1 script settles the *residual
correction* question on a set-level hold-out — but with a different solver, a
different target set, and a different metric.  The composite claim "Ours =
FK-fixed cube pose (A3) + prediction-time residual correction" has therefore
never been measured in one place.  This runner does exactly that and nothing
else.

Design
------
* **Split is set-level (position hold-out), not event-level.**  A residual
  correction is a claim about *spatial generalization*: coefficients learned at
  the training cube positions must transfer to a position the calibration never
  saw.  An event-level hold-out cannot test that, because every position is
  already in the training set.  ``CP_ablation_schema.POSITION_HOLDOUT_ROLE``
  permits this split only under an explicit FK-proxy label, which is what every
  number below carries.
* **Both arms share one backend**: the canonical corner-reprojection problem,
  the canonical solver options, and ``UNIFIED_FREE_VARIABLES`` — A2 and A3 as
  defined in ``CP_ablation_schema``.  The only difference between the arms is
  whether ``T_base_cube_by_set`` is a free variable (A2) or frozen to the
  train-only aligned FK artifact (A3).
* **The correction never touches the calibration.**  It is learned from
  training-set positions after the fit is frozen and applied only to the
  held-out position's predicted centre.  ``bTCi`` and ``gTc`` are unchanged, so
  the pixel-domain control metrics below are identical with and without it —
  that is an invariant this script asserts, not an assumption.

Metrics
-------
primary   ``heldout_position_error_mm``  — FK-proxy.  |p̂(s*) (+corr) − p_FK(s*)|.
control A ``heldout_reprojection_fk_pose_px`` — pixel error at the held-out
          position using the train-only aligned FK pose.  Correction-invariant.
control B ``heldout_e_cross_translation_mm`` — inter-fixed-camera agreement at
          the held-out position.  Uses no FK reference.  Correction-invariant.

Control B exists because the primary metric is defined against FK while the
FK-fixed arm was *fitted* to FK; a metric that never mentions FK is the only
way to see whether an arm is better or merely better-aligned with its own
anchor.  Read the two together or not at all.

Usage
-----
    PYTHONPATH= python CP_D1_fk_correction_2x2.py \
        --root_folder data/session --intrinsics_dir intrinsics \
        --calib_dir data/session/calib_out \
        --out_dir CP_result/D1_fk_correction_2x2
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import os
from collections import defaultdict
from typing import Dict, List, Mapping, Optional, Sequence, Tuple

import numpy as np

import CP_common as cp
import CP_ablation_7row as ab
import Step3_calibration as s3
from CP_ablation_schema import (
    MAIN_ABLATION_CONDITIONS,
    POSITION_HOLDOUT_ROLE,
    UNIFIED_FREE_VARIABLES,
    AblationCondition,
    validate_fk_alignment_artifact,
)
from apriltag_cube import inv_T
from calibration_fk_cube_artifact import estimate_board_free_fk_cube_artifact
from calibration_runtime_utils import (
    get_capture_set_index,
    load_intrinsics_with_depth_scale,
)
from calibration_path_evaluation import observation_id, solve_observed_pose
from calibration_reprojection_backend import (
    CornerReprojectionProblem,
    PixelObs,
    PoseState,
    SE3Scaling,
    SolverOptions,
    coordinate_change_factors,
    freeze_manifest,
    jacobian_diagnostics,
    perturbed_x,
    project_points,
    variable_keys,
    solve_corner_reprojection,
)
from scipy.optimize import least_squares
from scipy.sparse import csr_matrix, lil_matrix, vstack as sparse_vstack
import time
FK_FIXED_ARM = "A3"
CORRECTIONS = ("none", "offset", "se3", "ridge")
CORRECTION_DOF = {"none": 0, "offset": 3, "se3": 6, "ridge": 9}
PREDICTION_MASK_SCHEMA = "set_level_cube_centre_prediction_mask_v1"
PROXY_LABEL = "FK-proxy"


# ── model-independent prediction mask ────────────────────────────────────────
def build_prediction_mask(observations: Sequence[PixelObs], eligible_sets: Sequence[int],
                          gripper: int, K_map, D_map) -> dict:
    """Freeze which cube observations produce each set's centre, before fitting.

    Selection uses PnP validity only.  No fitted transform is consulted, so the
    same population serves both arms and both correction states.
    """
    allowed = {int(s) for s in eligible_sets}
    by_set: Dict[int, List[str]] = defaultdict(list)
    invalid: List[str] = []
    seen: Dict[str, PixelObs] = {}
    for obs in observations:
        if obs.marker != "cube" or obs.set_idx is None:
            continue
        if int(obs.set_idx) not in allowed:
            continue
        key = observation_id(obs)
        if key in seen:
            raise ValueError(f"duplicate cube observation ID: {key}")
        seen[key] = obs
    for key in sorted(seen):
        if solve_observed_pose(seen[key], K_map, D_map) is None:
            invalid.append(key)
        else:
            by_set[int(seen[key].set_idx)].append(key)
    body = {
        "artifact_schema": PREDICTION_MASK_SCHEMA,
        "selection_basis": "measurement_only_PnP_validity_before_model_fit",
        "model_output_used_for_selection": False,
        "gripper_camera_id": int(gripper),
        "set_ids": sorted(allowed),
        "observation_ids_by_set": {str(s): sorted(v) for s, v in sorted(by_set.items())},
        "pnp_invalid_observation_ids": invalid,
        "aggregation": "per_axis_median_of_base_frame_cube_centres",
    }
    body["prediction_mask_sha256"] = hashlib.sha256(
        json.dumps(body, sort_keys=True, separators=(",", ":")).encode("utf-8")
    ).hexdigest()
    return body


def predict_set_centres(mask: Mapping, by_id: Mapping[str, PixelObs],
                        cams: Mapping[int, np.ndarray], gtc: np.ndarray,
                        robot_T: Mapping[int, np.ndarray], gripper: int,
                        K_map, D_map) -> Dict[int, np.ndarray]:
    """Lift every predeclared cube observation into base and take the median.

    Per-axis median matches ``CP_C1.predict_cube_base_pos``: a set carries tens
    of observations and a few PnP flips move a mean by >100 mm.
    """
    out: Dict[int, np.ndarray] = {}
    for set_key, ids in mask["observation_ids_by_set"].items():
        points = []
        for key in ids:
            obs = by_id[key]
            T_cam_target = solve_observed_pose(obs, K_map, D_map)
            if T_cam_target is None:
                raise RuntimeError(f"prevalidated PnP observation became invalid: {key}")
            if int(obs.cam) == int(gripper):
                if int(obs.event) not in robot_T:
                    continue
                T = np.asarray(robot_T[int(obs.event)], float) @ np.asarray(gtc, float) @ T_cam_target
            else:
                if int(obs.cam) not in cams:
                    continue
                T = np.asarray(cams[int(obs.cam)], float) @ T_cam_target
            points.append(T[:3, 3])
        if points:
            out[int(set_key)] = np.median(np.asarray(points, float), axis=0)
    return out


def heldout_cross_translation_mm(mask: Mapping, by_id: Mapping[str, PixelObs],
                                 cams: Mapping[int, np.ndarray], set_index: int,
                                 K_map, D_map) -> Tuple[Optional[float], int]:
    """Inter-fixed-camera agreement at one position.  Never mentions FK.

    RMSE over every same-event fixed-camera pair in the predeclared mask.
    """
    ids = mask["observation_ids_by_set"].get(str(int(set_index)), [])
    gripper = int(mask["gripper_camera_id"])
    by_event: Dict[int, List[np.ndarray]] = defaultdict(list)
    for key in ids:
        obs = by_id[key]
        if int(obs.cam) == gripper or int(obs.cam) not in cams:
            continue
        T_cam_target = solve_observed_pose(obs, K_map, D_map)
        if T_cam_target is None:
            continue
        by_event[int(obs.event)].append(
            (np.asarray(cams[int(obs.cam)], float) @ T_cam_target)[:3, 3])
    values = []
    for _, points in sorted(by_event.items()):
        for i in range(len(points)):
            for j in range(i + 1, len(points)):
                values.append(float(np.linalg.norm(points[i] - points[j])) * 1000.0)
    if not values:
        return None, 0
    return float(np.sqrt(np.mean(np.square(values)))), len(values)


# ── residual-correction models (train positions only) ────────────────────────
def _feature(p: np.ndarray) -> np.ndarray:
    """Linear feature [1, x, y] of the predicted cube centre (metres)."""
    p = np.asarray(p, float).reshape(3)
    return np.array([1.0, p[0], p[1]], dtype=float)


def learn_correction(kind: str, predicted: Mapping[int, np.ndarray],
                     reference: Mapping[int, np.ndarray], train_sets: Sequence[int],
                     ridge_lambda: float):
    pairs = [(np.asarray(predicted[s], float), np.asarray(reference[s], float))
             for s in sorted(train_sets)
             if s in predicted and s in reference]
    if kind == "none":
        return None
    if len(pairs) < 3:
        return None
    src = np.asarray([p for p, _ in pairs], float)
    dst = np.asarray([r for _, r in pairs], float)
    if kind == "offset":
        return np.mean(dst - src, axis=0)
    if kind == "se3":
        return cp.kabsch_rigid(src, dst)
    if kind == "ridge":
        X = np.asarray([_feature(p) for p in src], float)
        Y = dst - src
        reg = float(ridge_lambda) * np.eye(X.shape[1])
        reg[0, 0] = 0.0  # never regularize the intercept
        return np.linalg.solve(X.T @ X + reg, X.T @ Y)
    raise ValueError(f"unknown correction {kind!r}")


def apply_correction(kind: str, param, p: np.ndarray) -> np.ndarray:
    p = np.asarray(p, float).reshape(3)
    if kind == "none" or param is None:
        return p
    if kind == "offset":
        return p + np.asarray(param, float).reshape(3)
    if kind == "se3":
        T = np.asarray(param, float)
        return T[:3, :3] @ p + T[:3, 3]
    if kind == "ridge":
        return p + _feature(p) @ np.asarray(param, float)
    raise ValueError(f"unknown correction {kind!r}")


# ── soft FK anchor on the cube pose ──────────────────────────────────────────
#
# The anchor spans the gap between the two endpoints the 2x2 measured:
#   lambda = 0    -> A2, cube pose entirely free (identical to the canonical solve)
#   lambda -> inf -> A3, cube pose hard-frozen to the FK artifact
#
# It is expressed as the *displacement of the cube's own probe points*, in mm.
# That is the same geometric quantity the reprojection term measures, only in
# millimetres instead of pixels, so ``lambda`` carries the honest unit px/mm:
# lambda=1 means "one millimetre of cube displacement costs as much as one pixel
# of corner reprojection error".  Encoding the anchor as a 6-vector of raw
# translation and rotation instead would make lambda unit-less and
# uninterpretable across rotation/translation.
def anchor_probe_points(lever_mm: float) -> np.ndarray:
    """Six points at +-lever along each cube axis, in metres."""
    lever = float(lever_mm) / 1000.0
    return np.asarray([
        [lever, 0.0, 0.0], [-lever, 0.0, 0.0],
        [0.0, lever, 0.0], [0.0, -lever, 0.0],
        [0.0, 0.0, lever], [0.0, 0.0, -lever],
    ], dtype=np.float64)


def _probe_in_base(T: np.ndarray, probe: np.ndarray) -> np.ndarray:
    T = np.asarray(T, float)
    return probe @ T[:3, :3].T + T[:3, 3]


def solve_anchored_corner_reprojection(
        observations: Sequence[PixelObs], variable_keys_, reference_state: PoseState,
        robot_T, K_map, D_map, gripper_cam_idx: int,
        anchor_targets: Mapping[int, np.ndarray], anchor_lambda: float,
        anchor_lever_mm: float, options: SolverOptions, seed: int,
        init_translation_mm: float, init_rotation_deg: float) -> Tuple[PoseState, dict]:
    """Canonical corner problem plus an optional soft FK anchor on cube poses.

    With ``anchor_lambda == 0`` this is bit-identical to
    ``solve_corner_reprojection``; ``main`` asserts that on the first fold.
    """
    options.validate()
    problem = CornerReprojectionProblem(
        observations, variable_keys_, reference_state, robot_T, K_map, D_map,
        gripper_cam_idx, scaling=options.scaling)
    probe = anchor_probe_points(anchor_lever_mm)
    weight = float(anchor_lambda)
    anchor_keys = [key for key in problem.variable_keys
                   if key[0] == "cube" and int(key[1]) in anchor_targets]
    use_anchor = weight > 0.0 and bool(anchor_keys)

    def residual(x: np.ndarray) -> np.ndarray:
        base = problem.residual_vector(x)
        if not use_anchor:
            return base
        state = problem.unpack(x)
        chunks = [base]
        for _, set_index in anchor_keys:
            estimated = _probe_in_base(state.cubes[int(set_index)], probe)
            target = _probe_in_base(anchor_targets[int(set_index)], probe)
            chunks.append((weight * (estimated - target) * 1000.0).reshape(-1))
        return np.concatenate(chunks)

    sparsity = problem.jacobian_sparsity()
    if use_anchor:
        extra = lil_matrix((len(anchor_keys) * probe.shape[0] * 3, problem.n_params),
                           dtype=np.int8)
        for index, key in enumerate(anchor_keys):
            rows = slice(index * probe.shape[0] * 3, (index + 1) * probe.shape[0] * 3)
            extra[rows, problem.slices[key]] = 1
        sparsity = csr_matrix(sparse_vstack([sparsity, extra.tocsr()]))

    x0 = perturbed_x(problem, seed, init_translation_mm, init_rotation_deg)
    started = time.perf_counter()
    solution = least_squares(
        residual, x0,
        method=options.method, loss=options.loss, f_scale=float(options.f_scale_px),
        x_scale=("jac" if options.x_scale_mode == "jac" else 1.0),
        jac_sparsity=sparsity,
        max_nfev=int(options.max_nfev),
        xtol=float(options.xtol), ftol=float(options.ftol), gtol=float(options.gtol),
    )
    elapsed = float(time.perf_counter() - started)
    state = problem.unpack(solution.x)
    reprojection_only = problem.residual_vector(solution.x)
    common_scaling = SE3Scaling(rotation_scale_rad=1.0, translation_scale_m=0.5)
    common_factors = coordinate_change_factors(
        len(problem.variable_keys), options.scaling, common_scaling)
    anchor_rms_mm = None
    if use_anchor:
        tail = residual(solution.x)[problem.n_residuals:]
        anchor_rms_mm = float(np.sqrt(np.mean(np.square(tail / weight))))
    diagnostics = {
        "backend": "canonical_corner_reprojection_v1_with_optional_fk_anchor",
        "anchor_lambda_px_per_mm": weight,
        "anchor_lever_mm": float(anchor_lever_mm),
        "anchor_active": bool(use_anchor),
        "anchor_n_positions": len(anchor_keys) if use_anchor else 0,
        "anchor_rms_cube_displacement_mm": anchor_rms_mm,
        "solver_options": options.to_dict(),
        "success": bool(solution.success),
        "status": int(solution.status),
        "message": str(solution.message),
        "nfev": int(solution.nfev),
        "optimality": float(solution.optimality),
        "elapsed_s": elapsed,
        "n_parameters": int(problem.n_params),
        "n_residuals": int(problem.n_residuals),
        "freeze_manifest": freeze_manifest(reference_state, variable_keys_),
        "variable_keys": [f"{kind}:{idx}" for kind, idx in problem.variable_keys],
        "train_reprojection_rmse_px": float(np.sqrt(np.mean(np.square(reprojection_only)))),
        "cost": float(solution.cost),
        "jacobian": jacobian_diagnostics(solution.jac, problem.n_params),
        "common_scaled_jacobian": jacobian_diagnostics(
            solution.jac, problem.n_params, column_factors=common_factors),
    }
    return state, diagnostics


def arm_specs(lambdas: Sequence[float]) -> List[dict]:
    """A3 (hard FK fix) plus one vision-estimated arm per anchor weight."""
    specs = [{"key": FK_FIXED_ARM, "row": "A3", "anchor_lambda": None,
              "label": "FK-fixed (hard)"}]
    for value in lambdas:
        key = "A2" if float(value) == 0.0 else f"A2@lam{value:g}"
        specs.append({"key": key, "row": "A2", "anchor_lambda": float(value),
                      "label": ("vision-estimated (no anchor)" if float(value) == 0.0
                                else f"vision-estimated + soft anchor lam={value:g}")})
    return specs


# ── one fold ─────────────────────────────────────────────────────────────────
def reprojection_at_fk_pose(observations: Sequence[PixelObs], set_index: int,
                            cube_pose: np.ndarray, state: PoseState,
                            robot_T, K_map, D_map, gripper: int) -> Tuple[Optional[float], int]:
    """Pixel error at the held-out position using the train-only aligned FK pose.

    Correction-invariant by construction: it depends only on ``state`` and the
    FK pose, neither of which a prediction-time correction touches.
    """
    squared, corners = [], 0
    for obs in observations:
        if obs.marker != "cube" or obs.set_idx is None or int(obs.set_idx) != int(set_index):
            continue
        if int(obs.cam) == int(gripper):
            if int(obs.event) not in robot_T:
                continue
            T_base_cam = np.asarray(robot_T[int(obs.event)], float) @ state.gtc
        else:
            if int(obs.cam) not in state.cams:
                continue
            T_base_cam = state.cams[int(obs.cam)]
        pred = project_points(inv_T(T_base_cam) @ np.asarray(cube_pose, float),
                              obs.object_points, K_map[int(obs.cam)], D_map[int(obs.cam)])
        squared.extend(np.square(pred - np.asarray(obs.image_points).reshape(-1, 2)).reshape(-1).tolist())
        corners += len(pred)
    if not squared:
        return None, 0
    return float(np.sqrt(np.mean(squared))), corners


def run_fold(args, held_out: int, eligible: Sequence[int], observations: Sequence[PixelObs],
             by_id: Mapping[str, PixelObs], prediction_mask: Mapping,
             raw_fk_all: Mapping[int, np.ndarray],
             raw_fk_source_event_by_set: Mapping[int, int],
             robot_T, K_map, D_map, gripper: int,
             verify_lambda_zero: bool = False) -> dict:
    train_sets = [s for s in eligible if int(s) != int(held_out)]
    # Withhold every observation captured at the held-out position, board views
    # included.  Board corners carry no cube-position information, but keeping
    # them would still let the held-out capture events constrain T_base_Ci and
    # T_gripper_cam, and that is an argument this experiment should not have to
    # have.  Both arms lose exactly the same observations.
    heldout_events = {int(obs.event) for obs in observations
                      if obs.marker == "cube" and obs.set_idx is not None
                      and int(obs.set_idx) == int(held_out)}
    train_obs = [obs for obs in observations
                 if int(obs.event) not in heldout_events
                 and (obs.set_idx is None or int(obs.set_idx) in set(train_sets))]

    aligned_fk_all, fixed_gtc_initial, artifact = estimate_board_free_fk_cube_artifact(
        observations=train_obs,
        raw_fk_by_set=raw_fk_all,
        robot_T=robot_T,
        K_map=K_map,
        D_map=D_map,
        gripper_cam_idx=gripper,
        training_set_ids=sorted(train_sets),
        options=SolverOptions(),
        num_inits=int(args.artifact_inits),
        init_translation_mm=float(args.init_translation_mm),
        init_rotation_deg=float(args.init_rotation_deg),
        raw_fk_source_event_by_set=raw_fk_source_event_by_set,
    )
    validate_fk_alignment_artifact(artifact)
    if int(held_out) in set(artifact["training_set_ids"]):
        raise RuntimeError("held-out position leaked into the FK alignment artifact")
    leaked = set(artifact["source_observation_ids"]) & heldout_events
    if leaked:
        raise RuntimeError(f"FK artifact used held-out events {sorted(leaked)}")

    eih_board = [obs for obs in train_obs
                 if obs.marker == "board" and int(obs.cam) == int(gripper)]
    board_gtc, board_initial, handeye_diag = ab.estimate_board_handeye_initial(
        eih_board, robot_T, K_map, D_map, gripper)
    visual_cubes = ab.average_visual_target(
        train_obs, "cube", board_gtc, robot_T, K_map, D_map, gripper)
    missing_visual = sorted(set(train_sets) - set(visual_cubes))
    if missing_visual:
        raise RuntimeError(f"visual cube initialization missing for train sets {missing_visual}")
    fixed_cubes = {int(s): aligned_fk_all[int(s)] for s in train_sets}
    reference = {int(s): np.asarray(aligned_fk_all[int(s)], float)[:3, 3]
                 for s in eligible if int(s) in aligned_fk_all}

    conditions = {c.row: c for c in MAIN_ABLATION_CONDITIONS}
    arms = {}
    for spec in arm_specs(args._lambdas):
        row = spec["row"]
        condition: AblationCondition = conditions[row]
        initial_state, init_diag = ab.make_initial_state(
            condition, train_obs, gripper, robot_T, K_map, D_map,
            board_gtc, board_initial, visual_cubes, fixed_cubes, fixed_gtc_initial)
        fit_obs = ab.filter_observations(
            train_obs, condition, None, gripper, initial_state.cams)
        state, diag = solve_anchored_corner_reprojection(
            observations=fit_obs,
            variable_keys_=variable_keys(UNIFIED_FREE_VARIABLES[row], initial_state),
            reference_state=initial_state,
            robot_T=robot_T,
            K_map=K_map,
            D_map=D_map,
            gripper_cam_idx=gripper,
            anchor_targets=({} if spec["anchor_lambda"] is None else fixed_cubes),
            anchor_lambda=(0.0 if spec["anchor_lambda"] is None
                           else float(spec["anchor_lambda"])),
            anchor_lever_mm=float(args.anchor_lever_mm),
            options=ab.canonical_solver_options(args),
            seed=int(args.seed),
            init_translation_mm=float(args.init_translation_mm),
            init_rotation_deg=float(args.init_rotation_deg),
        )
        if verify_lambda_zero and spec["anchor_lambda"] == 0.0:
            # The anchored solver must reduce exactly to the canonical one at
            # lambda=0, otherwise the whole sweep sits on a different backend
            # than the rest of the paper.
            canonical_state, canonical_diag = solve_corner_reprojection(
                observations=fit_obs,
                variable_keys_=variable_keys(UNIFIED_FREE_VARIABLES[row], initial_state),
                reference_state=initial_state, robot_T=robot_T, K_map=K_map, D_map=D_map,
                gripper_cam_idx=gripper, options=ab.canonical_solver_options(args),
                seed=int(args.seed), init_translation_mm=float(args.init_translation_mm),
                init_rotation_deg=float(args.init_rotation_deg))
            delta = max(
                max(np.abs(state.cams[ci] - canonical_state.cams[ci]).max()
                    for ci in state.cams),
                float(np.abs(state.gtc - canonical_state.gtc).max()))
            if delta > 1e-9:
                raise RuntimeError(
                    f"lambda=0 anchored solve deviates from the canonical solve "
                    f"by {delta:.3e}; the sweep is not on the canonical backend")
            print(f"        [check] lambda=0 == canonical solve "
                  f"(max |dT| {delta:.2e}, nfev {diag['nfev']} vs "
                  f"{canonical_diag['nfev']})", flush=True)

        predicted = predict_set_centres(
            prediction_mask, by_id, state.cams, state.gtc, robot_T, gripper, K_map, D_map)
        if int(held_out) not in predicted:
            raise RuntimeError(f"no held-out prediction for set {held_out}")

        cell = {}
        for kind in CORRECTIONS:
            param = learn_correction(kind, predicted, reference, train_sets,
                                     float(args.ridge_lambda))
            corrected = apply_correction(kind, param, predicted[int(held_out)])
            error_mm = float(np.linalg.norm(corrected - reference[int(held_out)])) * 1000.0
            train_residual = [
                float(np.linalg.norm(
                    apply_correction(kind, param, predicted[s]) - reference[s])) * 1000.0
                for s in train_sets if s in predicted and s in reference]
            cell[kind] = {
                "heldout_position_error_mm": error_mm,
                "correction_dof": CORRECTION_DOF[kind],
                "correction_learned": param is not None or kind == "none",
                "train_position_rmse_mm": (
                    float(np.sqrt(np.mean(np.square(train_residual)))) if train_residual else None),
                "parameters": (None if param is None else np.asarray(param, float).tolist()),
            }

        reproj_px, reproj_corners = reprojection_at_fk_pose(
            observations, held_out, aligned_fk_all[int(held_out)], state,
            robot_T, K_map, D_map, gripper)
        cross_mm, n_pairs = heldout_cross_translation_mm(
            prediction_mask, by_id, state.cams, held_out, K_map, D_map)
        arms[spec["key"]] = {
            "row": row,
            "label": spec["label"],
            "anchor_lambda": spec["anchor_lambda"],
            "converged": bool(diag["success"]),
            "solver": {k: diag[k] for k in
                       ("status", "message", "nfev", "cost",
                        "train_reprojection_rmse_px", "elapsed_s",
                        "anchor_rms_cube_displacement_mm") if k in diag},
            "jacobian": diag.get("jacobian"),
            "initialization": init_diag,
            "n_fit_observations": len(fit_obs),
            "corrections": cell,
            "control_heldout_reprojection_fk_pose_px": reproj_px,
            "control_heldout_reprojection_corners": reproj_corners,
            "control_heldout_e_cross_translation_mm": cross_mm,
            "control_heldout_cross_pairs": n_pairs,
            "raw_predicted_centre_m": predicted[int(held_out)].tolist(),
        }
    return {
        "held_out_set": int(held_out),
        "train_sets": sorted(train_sets),
        "fk_artifact_sha256": artifact["artifact_sha256"],
        "handeye_initialization": handeye_diag,
        "reference_centre_m": reference[int(held_out)].tolist(),
        "arms": arms,
    }


# ── aggregation ──────────────────────────────────────────────────────────────
def _paired_stats(deltas: Sequence[float]) -> dict:
    """mean / std / standard error / t on paired per-fold differences."""
    values = np.asarray(list(deltas), dtype=np.float64)
    if values.size == 0:
        return {"n_folds": 0, "mean_mm": None, "std_mm": None,
                "se_mm": None, "t": None, "folds_negative": 0}
    std = float(values.std(ddof=1)) if values.size > 1 else 0.0
    se = std / np.sqrt(values.size) if values.size > 1 and std > 0 else None
    return {
        "n_folds": int(values.size),
        "mean_mm": float(values.mean()),
        "std_mm": float(values.std()),
        "se_mm": se,
        "t": (float(values.mean() / se) if se else None),
        "folds_negative": int((values < 0).sum()),
    }


def aggregate(folds: Sequence[Mapping], arm_keys: Sequence[str]) -> dict:
    usable = [f for f in folds
              if all(f["arms"][key]["converged"] for key in arm_keys)]
    usable_ids = {int(f["held_out_set"]) for f in usable}

    def errs(arm: str, kind: str) -> List[float]:
        return [f["arms"][arm]["corrections"][kind]["heldout_position_error_mm"]
                for f in usable]

    cells = {}
    for arm in arm_keys:
        for kind in CORRECTIONS:
            values = errs(arm, kind)
            cells[f"{arm}:{kind}"] = {
                "arm": arm, "correction": kind, "n_folds": len(values),
                "rmse_mm": float(np.sqrt(np.mean(np.square(values)))) if values else None,
                "mean_mm": float(np.mean(values)) if values else None,
                "std_mm": float(np.std(values)) if values else None,
                "median_mm": float(np.median(values)) if values else None,
                "max_mm": float(np.max(values)) if values else None,
            }
    controls = {}
    for arm in arm_keys:
        for key, label in (("control_heldout_reprojection_fk_pose_px", "reprojection_fk_pose_px"),
                           ("control_heldout_e_cross_translation_mm", "e_cross_translation_mm")):
            values = [f["arms"][arm][key] for f in usable if f["arms"][arm][key] is not None]
            controls[f"{arm}:{label}"] = {
                "arm": arm, "metric": label, "n_folds": len(values),
                "rmse": float(np.sqrt(np.mean(np.square(values)))) if values else None,
                "mean": float(np.mean(values)) if values else None,
                "std": float(np.std(values)) if values else None,
            }
    conditioning = {}
    for arm in arm_keys:
        cond = [f["arms"][arm]["jacobian"]["jacobian_condition_number"] for f in usable]
        nfev = [f["arms"][arm]["solver"]["nfev"] for f in usable]
        params = [f["arms"][arm]["jacobian"]["n_params"] for f in usable]
        anchor = [f["arms"][arm]["solver"].get("anchor_rms_cube_displacement_mm")
                  for f in usable]
        anchor = [v for v in anchor if v is not None]
        conditioning[arm] = {
            "n_params": int(params[0]) if params else None,
            "jacobian_condition_mean": float(np.mean(cond)) if cond else None,
            "nfev_mean": float(np.mean(nfev)) if nfev else None,
            "anchor_rms_cube_displacement_mm_mean": (
                float(np.mean(anchor)) if anchor else None),
        }

    claims = {}
    # every arm against the FK-fixed endpoint, at every correction state
    for arm in arm_keys:
        if arm == FK_FIXED_ARM:
            continue
        for kind in CORRECTIONS:
            stats = _paired_stats(
                [a - b for a, b in zip(errs(FK_FIXED_ARM, kind), errs(arm, kind))])
            claims[f"{FK_FIXED_ARM}_minus_{arm}_at_{kind}"] = {
                "definition": f"{FK_FIXED_ARM} − {arm}; negative means {FK_FIXED_ARM} better",
                **stats}
    # correction gain within each arm
    for arm in arm_keys:
        for kind in ("offset", "se3", "ridge"):
            stats = _paired_stats(
                [a - b for a, b in zip(errs(arm, kind), errs(arm, "none"))])
            claims[f"{arm}_gain_{kind}"] = {
                "definition": f"{kind} − none within {arm}; negative means the "
                              "correction helped", **stats}
    # best cell by RMSE
    ranked = sorted(
        (c for c in cells.values() if c["rmse_mm"] is not None),
        key=lambda c: c["rmse_mm"])
    claims["best_cells_by_rmse"] = [
        {"arm": c["arm"], "correction": c["correction"], "rmse_mm": c["rmse_mm"]}
        for c in ranked[:5]]
    # is the best anchored arm better than both endpoints?
    if ranked:
        best = ranked[0]
        for endpoint in (FK_FIXED_ARM, "A2"):
            if endpoint in arm_keys and endpoint != best["arm"]:
                claims[f"best_{best['arm']}_{best['correction']}_vs_{endpoint}_same_correction"] = {
                    "definition": f"{best['arm']} − {endpoint} at correction "
                                  f"{best['correction']}; negative means the best cell wins",
                    **_paired_stats([a - b for a, b in zip(
                        errs(best["arm"], best["correction"]),
                        errs(endpoint, best["correction"]))]),
                }
    return {
        "n_folds_total": len(folds),
        "n_folds_usable": len(usable),
        "excluded_folds": [int(f["held_out_set"]) for f in folds
                           if int(f["held_out_set"]) not in usable_ids],
        "arm_keys": list(arm_keys),
        "cells": cells,
        "controls": controls,
        "conditioning": conditioning,
        "claims": claims,
    }


def control_invariance_report(folds: Sequence[Mapping]) -> dict:
    """Assert in the output what the design guarantees: corrections move only
    the FK-referenced position metric, never a pixel or cross-camera metric."""
    return {
        "statement": (
            "The residual correction is applied to the predicted cube centre "
            "after the fit is frozen.  It never modifies T_base_Ci or "
            "T_gripper_cam, so control_heldout_reprojection_fk_pose_px and "
            "control_heldout_e_cross_translation_mm are identical for every "
            "correction state of a given arm."),
        "verified_by_construction": True,
        "consequence": (
            "A correction can never be reported as a reprojection improvement. "
            "Any claim built on this experiment is a claim about held-out "
            "position agreement with the FK proxy only."),
    }


def write_outputs(result: Mapping, out_dir: str) -> None:
    os.makedirs(out_dir, exist_ok=True)
    with open(os.path.join(out_dir, "D1_fk_correction_2x2.json"), "w") as handle:
        json.dump(ab._jsonable(result), handle, indent=2)

    summary = result["summary"]
    cells = summary["cells"]
    arm_keys = summary["arm_keys"]
    labels = {spec["key"]: spec["label"] for spec in result["arm_specs"]}
    with open(os.path.join(out_dir, "D1_fk_correction_2x2.csv"), "w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(["arm", "anchor_lambda_px_per_mm", "correction", "dof",
                         "n_folds", "rmse_mm", "mean_mm", "std_mm", "median_mm", "max_mm"])
        lam = {spec["key"]: spec["anchor_lambda"] for spec in result["arm_specs"]}
        for arm in arm_keys:
            for kind in CORRECTIONS:
                cell = cells[f"{arm}:{kind}"]
                writer.writerow([arm, lam[arm], kind, CORRECTION_DOF[kind],
                                 cell["n_folds"], cell["rmse_mm"], cell["mean_mm"],
                                 cell["std_mm"], cell["median_mm"], cell["max_mm"]])

    def fmt(value, digits=3):
        return "—" if value is None else f"{value:.{digits}f}"

    claims = summary["claims"]
    best = (claims["best_cells_by_rmse"] or [{}])[0]
    lines = [
        "# D1 — FK 고정 × 잔차 보정, soft-anchor λ sweep 포함",
        "",
        f"- split: **position (set-level) hold-out**, {result['split']['strategy']}, "
        f"{summary['n_folds_usable']}/{summary['n_folds_total']} folds usable",
        f"- `RB_ROBOT_POS_SCALE` = **{result['robot_pos_scale']:.4f}** "
        f"({'로봇 원본 값 그대로' if result['robot_pos_scale'] == 1.0 else '병진 보정 적용'}). "
        "다른 값으로 만든 결과와는 FK 기준 자체가 달라 비교할 수 없다.",
        f"- 모든 위치 오차는 `{PROXY_LABEL}`다. 외부 GT가 아니므로 절대 정확도로 읽지 않는다.",
        f"- position hold-out의 역할 규정: `{POSITION_HOLDOUT_ROLE}`",
        "- 모든 arm이 동일 backend·동일 solver 설정·동일 예측 mask를 쓴다. 차이는 "
        "`T_base_cube_by_set`의 처리뿐이다: 하드 고정(A3) / 자유(λ=0) / soft anchor(λ>0).",
        f"- anchor는 큐브 probe 점의 변위(mm)로 표현하고 λ의 단위는 **px/mm**다 "
        f"(lever {result['anchor_lever_mm']:.1f} mm). λ=0은 canonical solve와 동일하며 "
        "실행 시 첫 fold에서 그 동치를 검증한다.",
        "",
        "## 핵심표 — held-out 위치 오차 RMSE (mm, FK-proxy)",
        "",
        "| arm | λ (px/mm) | none | offset(3) | SE(3)(6) | Ridge(9) |",
        "| --- | ---: | ---: | ---: | ---: | ---: |",
    ]
    lam = {spec["key"]: spec["anchor_lambda"] for spec in result["arm_specs"]}
    for arm in arm_keys:
        values = " | ".join(fmt(cells[f"{arm}:{kind}"]["rmse_mm"]) for kind in CORRECTIONS)
        lam_text = "∞ (hard)" if lam[arm] is None else f"{lam[arm]:g}"
        lines.append(f"| {arm} — {labels.get(arm, '')} | {lam_text} | {values} |")
    lines += [
        "",
        f"최저 셀: **{best.get('arm')} @ {best.get('correction')} = "
        f"{fmt(best.get('rmse_mm'))} mm**.",
        "",
        "상위 5개 셀:",
        "",
        "| 순위 | arm | 보정 | RMSE (mm) |",
        "| ---: | --- | --- | ---: |",
    ]
    for rank, entry in enumerate(claims["best_cells_by_rmse"], start=1):
        lines.append(f"| {rank} | {entry['arm']} | {entry['correction']} | "
                     f"{fmt(entry['rmse_mm'])} |")

    lines += [
        "",
        "## paired 판정 — A3(하드 고정) 대비 (음수가 A3 우세)",
        "",
        "| 비교 | mean±std (mm) | SE | t | A3 우세 fold |",
        "| --- | ---: | ---: | ---: | ---: |",
    ]
    for arm in arm_keys:
        if arm == FK_FIXED_ARM:
            continue
        for kind in CORRECTIONS:
            claim = claims[f"{FK_FIXED_ARM}_minus_{arm}_at_{kind}"]
            lines.append(
                f"| A3 − {arm} @ {kind} | {fmt(claim['mean_mm'])}±{fmt(claim['std_mm'])} | "
                f"{fmt(claim['se_mm'])} | {fmt(claim['t'], 2)} | "
                f"{claim['folds_negative']}/{claim['n_folds']} |")

    lines += [
        "",
        "## arm별 보정 이득 (음수가 개선)",
        "",
        "| arm | 보정 | mean±std (mm) | SE | t | 개선 fold |",
        "| --- | --- | ---: | ---: | ---: | ---: |",
    ]
    for arm in arm_keys:
        for kind in ("offset", "se3", "ridge"):
            claim = claims[f"{arm}_gain_{kind}"]
            lines.append(
                f"| {arm} | {kind} − none | {fmt(claim['mean_mm'])}±{fmt(claim['std_mm'])} | "
                f"{fmt(claim['se_mm'])} | {fmt(claim['t'], 2)} | "
                f"{claim['folds_negative']}/{claim['n_folds']} |")

    cond = summary["conditioning"]
    controls = summary["controls"]
    lines += [
        "",
        "## 자유도·조건수와 통제 지표",
        "",
        "보정은 예측 시점에만 적용되고 `T_base_Ci`/`T_gripper_cam`을 바꾸지 않는다. "
        "따라서 오른쪽 두 통제 지표는 보정 유무와 무관하게 동일하며, 어떤 보정 결과도 "
        "재투영 개선으로 보고할 수 없다.",
        "",
        "| arm | n_params | Jacobian cond | nfev | anchor 변위 RMS (mm) | 재투영 (px, FK pose) | e_cross (mm, FK 무관) |",
        "| --- | ---: | ---: | ---: | ---: | ---: | ---: |",
    ]
    for arm in arm_keys:
        c = cond[arm]
        reproj = controls[f"{arm}:reprojection_fk_pose_px"]
        cross = controls[f"{arm}:e_cross_translation_mm"]
        lines.append(
            f"| {arm} | {c['n_params']} | {fmt(c['jacobian_condition_mean'], 1)} | "
            f"{fmt(c['nfev_mean'], 1)} | {fmt(c['anchor_rms_cube_displacement_mm_mean'])} | "
            f"{fmt(reproj['mean'])}±{fmt(reproj['std'])} | "
            f"{fmt(cross['mean'])}±{fmt(cross['std'])} |")

    lines += [
        "",
        "## fold별 원본 (Ridge 보정 적용값, mm)",
        "",
        "| held-out set | " + " | ".join(arm_keys) + " |",
        "| ---: |" + " ---: |" * len(arm_keys),
    ]
    for fold in result["folds"]:
        values = " | ".join(
            fmt(fold["arms"][arm]["corrections"]["ridge"]["heldout_position_error_mm"])
            for arm in arm_keys)
        lines.append(f"| {fold['held_out_set']} | {values} |")

    lines += ["", "## 해석 규칙", "",
              result["control_invariance"]["statement"], "",
              result["control_invariance"]["consequence"], ""]
    with open(os.path.join(out_dir, "D1_fk_correction_2x2.md"), "w") as handle:
        handle.write("\n".join(lines))


def parse_args():
    parser = argparse.ArgumentParser(
        description="2x2 factorial: FK-fixed vs vision-estimated x residual correction")
    parser.add_argument("--root_folder", default="data/session")
    parser.add_argument("--intrinsics_dir", default="intrinsics")
    parser.add_argument("--calib_dir", default="data/session/calib_out")
    parser.add_argument("--out_dir", default="CP_result/D1_fk_correction_2x2")
    parser.add_argument("--min_eih_cube_events", type=int, default=3,
                        help="Sets with fewer train-capable eih cube events are dropped.")
    parser.add_argument("--min_fixed_cube_observations", type=int, default=2,
                        help="Sets with fewer fixed-camera cube observations are dropped.")
    parser.add_argument("--ridge_lambda", type=float, default=1e-3)
    parser.add_argument(
        "--lambdas", default="0,0.1,0.3,1,3,10",
        help=("Soft FK-anchor weights in px/mm for the vision-estimated arm. "
              "0 reproduces the canonical unanchored solve; the hard-fixed A3 arm "
              "is always included as the lambda->inf endpoint."))
    parser.add_argument(
        "--anchor_lever_mm", type=float, default=29.5,
        help=("Half-extent of the cube probe points used to express the anchor as a "
              "displacement in mm; defaults to the cube half-side."))
    parser.add_argument("--seed", type=int, default=0,
                        help="Optimizer initialization seed; 0 uses the unperturbed start.")
    parser.add_argument("--artifact_inits", type=int, default=3)
    parser.add_argument("--init_translation_mm", type=float, default=5.0)
    parser.add_argument("--init_rotation_deg", type=float, default=1.0)
    parser.add_argument("--max_nfev", type=int, default=300)
    parser.add_argument("--tol", type=float, default=1e-8)
    parser.add_argument("--rotation_scale_rad", type=float, default=1.0)
    parser.add_argument("--translation_scale_m", type=float, default=1.0)
    parser.add_argument("--x_scale_mode", choices=["unit", "jac"], default="jac")
    parser.add_argument("--loss", choices=["huber", "soft_l1", "linear"], default="soft_l1")
    parser.add_argument("--f_scale_px", type=float, default=2.0)
    parser.add_argument("--image_scale", type=float, default=1.0)
    parser.add_argument("--folds", default="all",
                        help="'all' for leave-one-position-out, or a comma-separated set list.")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    args._lambdas = [float(x) for x in str(args.lambdas).split(",") if x.strip()]
    if any(value < 0 for value in args._lambdas):
        raise RuntimeError("anchor weights must be non-negative")
    with open(os.path.join(args.root_folder, "meta.json")) as handle:
        meta = json.load(handle)
    all_cam_ids = sorted({int(ci) for cap in meta.get("captures", [])
                          for ci in cap.get("cams", {})})
    gripper = int(meta["gripper_cam_idx"])
    K_map, D_map = {}, {}
    for ci in all_cam_ids:
        K_map[ci], D_map[ci], _ = load_intrinsics_with_depth_scale(args.intrinsics_dir, ci)
    robot_T = s3.load_robot_poses_from_meta(meta)
    observations, cube_cfg_source, cube_reason = ab.detect_observations(
        args, meta, K_map, D_map, all_cam_ids, gripper)

    raw_fk_all = s3.load_nominal_set_cube_transforms(meta)
    raw_fk_source_event_by_set = {}
    for cap in meta.get("captures", []):
        set_index = get_capture_set_index(cap)
        if set_index is not None and int(set_index) not in raw_fk_source_event_by_set:
            raw_fk_source_event_by_set[int(set_index)] = int(cap["event_id"])

    # Eligibility is decided once, before any fit, from measurement counts only.
    eih_events: Dict[int, set] = defaultdict(set)
    fixed_counts: Dict[int, int] = defaultdict(int)
    for obs in observations:
        if obs.marker != "cube" or obs.set_idx is None:
            continue
        if int(obs.cam) == gripper:
            eih_events[int(obs.set_idx)].add(int(obs.event))
        else:
            fixed_counts[int(obs.set_idx)] += 1
    eligible, dropped = [], {}
    for s in sorted(set(eih_events) | set(fixed_counts)):
        if s not in raw_fk_all:
            dropped[str(s)] = "no raw FK cube pose"
            continue
        if len(eih_events.get(s, set())) < int(args.min_eih_cube_events):
            dropped[str(s)] = (f"eih cube events {len(eih_events.get(s, set()))} < "
                               f"{args.min_eih_cube_events}")
            continue
        if fixed_counts.get(s, 0) < int(args.min_fixed_cube_observations):
            dropped[str(s)] = (f"fixed cube observations {fixed_counts.get(s, 0)} < "
                               f"{args.min_fixed_cube_observations}")
            continue
        eligible.append(int(s))
    if len(eligible) < 4:
        raise RuntimeError(f"position hold-out needs >=4 eligible cube positions, got {eligible}")

    prediction_mask = build_prediction_mask(
        observations, eligible, gripper, K_map, D_map)
    by_id = {observation_id(obs): obs for obs in observations
             if obs.marker == "cube" and obs.set_idx is not None
             and int(obs.set_idx) in set(eligible)}

    requested = (eligible if args.folds.strip().lower() == "all"
                 else [int(x) for x in args.folds.split(",") if x.strip()])
    unknown = sorted(set(requested) - set(eligible))
    if unknown:
        raise RuntimeError(f"requested fold positions are not eligible: {unknown}")

    specs = arm_specs(args._lambdas)
    arm_keys = [spec["key"] for spec in specs]
    print(f"[D1] eligible positions: {eligible}")
    print(f"[D1] arms: {arm_keys}")
    if dropped:
        print(f"[D1] dropped positions: {json.dumps(dropped, ensure_ascii=False)}")
    folds = []
    for index, held_out in enumerate(requested):
        print(f"[D1] fold: hold out position {held_out} "
              f"({len(eligible) - 1} train positions)", flush=True)
        fold = run_fold(args, held_out, eligible, observations, by_id, prediction_mask,
                        raw_fk_all, raw_fk_source_event_by_set,
                        robot_T, K_map, D_map, gripper,
                        verify_lambda_zero=(index == 0))
        for key in arm_keys:
            arm = fold["arms"][key]
            print(f"        {key:12s} conv={arm['converged']} "
                  f"none={arm['corrections']['none']['heldout_position_error_mm']:6.2f} "
                  f"ridge={arm['corrections']['ridge']['heldout_position_error_mm']:6.2f} "
                  f"se3={arm['corrections']['se3']['heldout_position_error_mm']:6.2f} "
                  f"(mm)", flush=True)
        folds.append(fold)

    summary = aggregate(folds, arm_keys)
    result = {
        "experiment": "D1_fk_fixed_x_residual_correction_2x2",
        "primary_metric": "heldout_position_error_mm",
        "primary_metric_label": PROXY_LABEL,
        "external_ground_truth_available": False,
        "split": {
            "strategy": "leave_one_cube_position_out",
            "role": POSITION_HOLDOUT_ROLE,
            "eligible_positions": eligible,
            "dropped_positions": dropped,
            "requested_folds": requested,
        },
        "arm_specs": specs,
        "arms": {
            "A3": "cube+board, unified, T_base_cube_by_set frozen to train-only aligned FK",
            "A2": "cube+board, unified, T_base_cube_by_set free (vision-estimated)",
            "A2@lam*": ("same as A2 plus a soft FK anchor on the cube pose; the anchor "
                        "residual is the cube probe-point displacement in mm, so lambda "
                        "has units px/mm"),
        },
        "anchor_lever_mm": float(args.anchor_lever_mm),
        "anchor_lambdas_px_per_mm": args._lambdas,
        "corrections": {kind: CORRECTION_DOF[kind] for kind in CORRECTIONS},
        "correction_protocol": (
            "learned from training positions after the fit is frozen; applied only "
            "to the held-out position's predicted centre"),
        "prediction_mask": prediction_mask,
        "cube_config_source": cube_cfg_source,
        "cube_detection": cube_reason,
        "solver_options": ab.canonical_solver_options(args).to_dict(),
        "ridge_lambda": float(args.ridge_lambda),
        # Record the load-time robot translation scale.  Results produced under
        # different values are not comparable -- the FK cube poses they anchor
        # and supervise against differ by ~12mm -- and nothing else in the
        # artifact reveals which value was used.
        "robot_pos_scale": cp.robot_pos_scale(),
        "control_invariance": control_invariance_report(folds),
        "summary": summary,
        "folds": folds,
    }
    write_outputs(result, args.out_dir)

    cells = summary["cells"]
    print("\n[D1] held-out position error RMSE (mm, FK-proxy)")
    print(f"{'arm':14s} {'none':>9s} {'offset':>9s} {'se3':>9s} {'ridge':>9s}"
          f" {'cond':>8s} {'nfev':>6s}")
    for key in arm_keys:
        values = []
        for kind in CORRECTIONS:
            v = cells[f"{key}:{kind}"]["rmse_mm"]
            values.append("—" if v is None else f"{v:9.3f}")
        c = summary["conditioning"][key]
        print(f"{key:14s} " + " ".join(values)
              + f" {c['jacobian_condition_mean']:8.1f} {c['nfev_mean']:6.1f}")
    print("\n[D1] best cells by RMSE")
    for rank, entry in enumerate(summary["claims"]["best_cells_by_rmse"], start=1):
        print(f"  {rank}. {entry['arm']:14s} @ {entry['correction']:6s} "
              f"{entry['rmse_mm']:.3f} mm")
    print("\n[D1] claims")
    for name, claim in summary["claims"].items():
        if name == "best_cells_by_rmse":
            continue
        print(f"  {name}: {json.dumps(ab._jsonable(claim), ensure_ascii=False)}")
    print(f"\n[D1] wrote {args.out_dir}")


if __name__ == "__main__":
    main()
