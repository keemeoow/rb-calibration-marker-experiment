#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ur3_calibration/leave_one_out_eval.py -- genuine held-out FK-mode comparison
via leave-one-placement-out, upgrading fit_fk_ablation_diagnostic.py's
train-pooled-only (no held-out) no_fk/fixed_fk/factor_fk comparison.

Background (see fit_grasp_offset.py's and fit_fk_ablation_diagnostic.py's
module docstrings for the full story): table1.py's own A2/A3/A4 held-out rows
cannot run on this dataset because session2 photographed each of its 12 floor
placements exactly once, so no set can supply both a train event and a held-
out event under table1.py's event-stratified split
(``build_event_split``/``min_train_eih_cube_events``).

This script builds a DIFFERENT, genuinely held-out split that this dataset
DOES support: leave-one-placement-out. session2's 12 placements each have an
INDEPENDENT (non-vision-derived-per-set) predicted cube pose
``T_base_cube_FK[s]``, reconstructed by ``compute_set_cube_center.py`` from
Pass 1's fitted ``T_gripper_cube`` (session1-only, varying-flange-pose +
fixed-camera cube observations -- never touches any session2 image) combined
with each placement's taught flange pose (from
``session2_pick_and_place.compute_ordered_targets()`` -- derived from the
recorded waypoint, not from that placement's own images either). Already
computed and stored in every session2 capture's
``canonical_set_cube_center_matrix_4x4`` field by ``convert_to_meta.py``; this
script reads it back out via ``get_capture_set_cube_center_transform_raw``
(the same site ``fit_fk_ablation_diagnostic.py`` uses for its ``fixed_fk``
row) and re-applies ``RAW_FK_CUBE_CENTER_TO_OBJECT`` to undo
``compute_set_cube_center.py``'s pre-composition, exactly reproducing
Pass 2's ``T_base_cube_taught[s]``.

For each held-out placement ``h`` in 1..12:
  1. Exclude EVERY observation (board and cube, every camera) captured during
     h's single event (session2 events are 1:1 with set_index, so this is one
     event-id filter) from the fit, for all three FK modes. Fit camera
     extrinsics (``T_base_Ci``, ``T_gripper_cam``, ``T_base_board``) plus each
     mode's cube-pose treatment jointly over session1 (all 15 events, cube
     gripped) + the other 11 session2 placements:
       no_fk     -- T_base_cube_by_set free for the 11 training placements
                    (FK_MODE_NONE), identical in spirit to fit_grasp_offset.py.
       fixed_fk  -- the 11 training placements' cube poses hard-fixed to
                    T_base_cube_FK[s] and removed from the optimizer
                    (FK_MODE_FIXED).
       factor_fk -- T_base_cube_by_set free, plus a soft whitened SE(3) factor
                    (frozen Simulation-compatible covariance, 0.30deg/2.0mm)
                    toward T_fk_raw[s] @ Delta_train, with Delta_train fit
                    from the gripper camera's cube views of ONLY the 11
                    training placements via
                    fk_alignment.estimate_board_free_fk_cube_artifact
                    (FK_MODE_FACTOR).
     T_gripper_cube (the grasp offset) is held FIXED at Pass 1's fitted value
     in every iteration and every mode, rather than refit: it is constrained
     only by session1's gripped-cube observations, which never change across
     leave-one-out iterations (session2 placements carry no grasp_idx
     observations at all), so refitting it 12 times would reproduce
     (near-)the same number 12 times at real compute cost with no informative
     value -- see the module docstring's cross-referenced report section for
     the sanity check that this simplification does not hide leakage (the
     grasp offset never depends on any session2 image, held out or not).
  2. Evaluate ONLY on h: reproject T_base_cube_FK[h] (never touched by the
     fit, for any mode) through each mode's fitted T_base_cam for every real
     AprilTag corner detection actually observed at h (the same corner
     detections loaded by load_cube_board_pixel_observations -- no PnP
     re-solve, no synthetic corners), and record the per-corner Euclidean
     pixel error, split by camera.
  3. Aggregate across all 12 held-out iterations, per FK mode: mean/median/max
     of the per-iteration pooled RMSE, plus a pooled-corner RMSE per camera
     (fixed cams 0/2/3, gripper cam 1) computed by concatenating every held-out
     corner residual for that camera across all 12 iterations.

Usage:
  python ur3_calibration/leave_one_out_eval.py \\
      [--sets 1,2,3,...,12] [--out ur3_calibration/leave_one_out_eval.json]
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from collections import defaultdict
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT))

from calibration_pipeline.apriltag_cube import AprilTagCubeTarget, inv_T  # noqa: E402
from calibration_pipeline.config import get_default_cube_config  # noqa: E402
from calibration_pipeline.fk_alignment import estimate_board_free_fk_cube_artifact  # noqa: E402
from calibration_pipeline.fk_factor import (  # noqa: E402
    FK_MODE_FACTOR, FK_MODE_FIXED, FK_MODE_NONE, FKFactorSpec,
    SIGMA_FK_DEG, SIGMA_FK_MM, diagonal_covariance, solve_factorized_fk,
)
from calibration_pipeline.observations import load_cube_board_pixel_observations  # noqa: E402
from calibration_pipeline.reprojection import (  # noqa: E402
    PoseState, SolverOptions, project_points, variable_keys,
)
from calibration_pipeline.runtime import (  # noqa: E402
    get_capture_set_cube_center_transform_raw, load_intrinsics_with_depth_scale,
)
from calibration_pipeline.schema import RAW_FK_CUBE_CENTER_TO_OBJECT  # noqa: E402

ROOT = str(REPO_ROOT / "data" / "ur3_session12" / "calib_train")
INTRINSICS_DIR = str(REPO_ROOT / "ur3_calibration" / "intrinsics")
PASS1_JSON = REPO_ROOT / "ur3_calibration" / "pass1_grasp_offset.json"
MECHANICAL_MAP = np.asarray(RAW_FK_CUBE_CENTER_TO_OBJECT, dtype=np.float64)
MODES = ("no_fk", "fixed_fk", "factor_fk")


def parse_sets(spec: str, available: list) -> list:
    if not spec:
        return list(available)
    wanted = sorted({int(tok) for tok in spec.split(",") if tok.strip()})
    missing = sorted(set(wanted) - set(available))
    if missing:
        raise ValueError(f"requested sets not available: {missing}")
    return wanted


def evaluate_held_out(state, held_obs, gripper, robot_T, excluded_event,
                       T_base_cube_fk_h, K_map, D_map):
    """Per-corner pixel reprojection error of held-out placement h.

    ``held_obs`` are the ACTUAL AprilTag corner detections recorded for h
    (never used in the fit); ``T_base_cube_fk_h`` is the FK-predicted pose
    (never touched by the fit either, for any mode).
    """
    by_cam = defaultdict(list)
    for obs in held_obs:
        cam = int(obs.cam)
        if cam == gripper:
            T_base_cam = np.asarray(robot_T[excluded_event], dtype=np.float64) @ state.gtc
        else:
            T_base_cam = state.cams[cam]
        predicted = project_points(
            inv_T(T_base_cam) @ T_base_cube_fk_h, obs.object_points,
            K_map[cam], D_map[cam])
        errs = np.linalg.norm(
            predicted - np.asarray(obs.image_points).reshape(-1, 2), axis=1)
        by_cam[cam].extend(errs.tolist())
    all_errs = [e for values in by_cam.values() for e in values]
    return {
        "rmse_px": float(np.sqrt(np.mean(np.square(all_errs)))),
        "n_corners": int(len(all_errs)),
        "by_camera": {
            str(cam): {
                "rmse_px": float(np.sqrt(np.mean(np.square(values)))),
                "n_corners": int(len(values)),
            }
            for cam, values in sorted(by_cam.items())
        },
        "raw_errors_by_camera": {str(cam): values for cam, values in by_cam.items()},
    }


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--sets", default="", help="comma-separated held-out set indices (default: all available)")
    ap.add_argument("--out", default=str(REPO_ROOT / "ur3_calibration" / "leave_one_out_eval.json"))
    args = ap.parse_args()

    meta = json.loads((Path(ROOT) / "meta.json").read_text())
    gripper = int(meta["gripper_cam_idx"])
    all_cam_ids = sorted(meta["cam_indices"])
    K_map, D_map = {}, {}
    for c in all_cam_ids:
        K_map[c], D_map[c], _ = load_intrinsics_with_depth_scale(INTRINSICS_DIR, c)
    cube = AprilTagCubeTarget(get_default_cube_config())
    observations, load_diag = load_cube_board_pixel_observations(
        ROOT, meta, cube, K_map, D_map, all_cam_ids, gripper,
        exclude_gripped_cube=False, fixed_cube_min_corners=8,
        image_scale=1.0, cube_observation_policy="legacy")
    robot_T = {int(c["event_id"]): np.asarray(c["robot_pose_matrix_4x4"]) for c in meta["captures"]}
    print(f"loaded {len(observations)} observations "
          f"({load_diag['n_cube_observations']} cube, {load_diag['n_board_observations']} board)")

    # session2 placements: set_index -> event_id (1:1) and the independent
    # FK-predicted cube pose T_base_cube_FK[s] (see module docstring).
    set_to_event, raw_fk_all = {}, {}
    for c in meta["captures"]:
        s = c.get("set_index")
        if s is None or c.get("cube_gripped"):
            continue
        s = int(s)
        set_to_event[s] = int(c["event_id"])
        T_raw = get_capture_set_cube_center_transform_raw(c)
        if T_raw is not None:
            raw_fk_all[s] = np.asarray(T_raw, dtype=np.float64)
    all_sets = sorted(set_to_event)
    missing_fk = sorted(set(all_sets) - set(raw_fk_all))
    if missing_fk:
        raise RuntimeError(f"sets missing an FK-predicted cube pose: {missing_fk}")
    T_base_cube_fk = {s: raw_fk_all[s] @ MECHANICAL_MAP for s in all_sets}
    print(f"session2 placements available for leave-one-out: {all_sets}")

    held_out_sets = parse_sets(args.sets, all_sets)
    print(f"running {len(held_out_sets)} of {len(all_sets)} leave-one-out iterations: {held_out_sets}")

    pass1 = json.loads(PASS1_JSON.read_text())
    T_gripper_cube_fixed = np.asarray(pass1["T_gripper_cube"], dtype=np.float64)
    vision_cubes_full = {
        int(s): np.asarray(T, dtype=np.float64)
        for s, T in pass1["T_base_cube_by_set_vision_only"].items()
    }
    cam_init_full = {int(c): np.asarray(T, dtype=np.float64) for c, T in pass1["T_base_cam"].items()}
    gtc_init = np.asarray(pass1["T_gripper_cam"], dtype=np.float64)
    board_init = np.asarray(pass1["T_base_board"], dtype=np.float64)

    options = SolverOptions()
    covariance = diagonal_covariance(SIGMA_FK_MM, SIGMA_FK_DEG)

    per_mode_iterations = {mode: [] for mode in MODES}
    per_mode_raw_by_cam = {mode: defaultdict(list) for mode in MODES}

    for h in held_out_sets:
        started = time.perf_counter()
        excluded_event = set_to_event[h]
        train_sets = [s for s in all_sets if s != h]
        train_obs = [o for o in observations if int(o.event) != excluded_event]
        held_obs = [
            o for o in observations
            if int(o.event) == excluded_event and o.marker == "cube"
        ]
        if not held_obs:
            print(f"h={h}: WARNING no cube observations at the held-out event, skipping")
            continue

        base_state = PoseState(
            cams={c: cam_init_full[c].copy() for c in cam_init_full},
            gtc=gtc_init.copy(),
            board=board_init.copy(),
            cubes={s: vision_cubes_full[s].copy() for s in train_sets},
            grasps={0: T_gripper_cube_fixed.copy()},
        )
        free_cube_keys = variable_keys(
            ["T_base_Ci", "T_gripper_cam", "T_base_board", "T_base_cube_by_set"],
            base_state)

        # --- no_fk ---
        state_no_fk, diag_no_fk = solve_factorized_fk(
            observations=train_obs, variable_keys_=free_cube_keys,
            reference_state=base_state, robot_T=robot_T, K_map=K_map, D_map=D_map,
            gripper_cam_idx=gripper, options=options,
            fk_spec=FKFactorSpec(mode=FK_MODE_NONE))

        # --- fixed_fk ---
        state_fixed_init = base_state.clone()
        state_fixed_init.cubes = {s: T_base_cube_fk[s].copy() for s in train_sets}
        fixed_keys = variable_keys(
            ["T_base_Ci", "T_gripper_cam", "T_base_board"], state_fixed_init)
        state_fixed, diag_fixed = solve_factorized_fk(
            observations=train_obs, variable_keys_=fixed_keys,
            reference_state=state_fixed_init, robot_T=robot_T, K_map=K_map, D_map=D_map,
            gripper_cam_idx=gripper, options=options,
            fk_spec=FKFactorSpec(mode=FK_MODE_FIXED))

        # --- factor_fk ---
        eih_cube_obs_train = [
            o for o in train_obs
            if o.marker == "cube" and int(o.cam) == gripper and o.set_idx is not None
            and int(o.set_idx) in train_sets
        ]
        aligned_fk_train, _gtc_unused, artifact = estimate_board_free_fk_cube_artifact(
            observations=eih_cube_obs_train, raw_fk_by_set=raw_fk_all, robot_T=robot_T,
            K_map=K_map, D_map=D_map, gripper_cam_idx=gripper,
            training_set_ids=train_sets, num_inits=3,
            init_translation_mm=5.0, init_rotation_deg=1.0)
        state_factor, diag_factor = solve_factorized_fk(
            observations=train_obs, variable_keys_=free_cube_keys,
            reference_state=base_state, robot_T=robot_T, K_map=K_map, D_map=D_map,
            gripper_cam_idx=gripper, options=options,
            fk_targets={s: aligned_fk_train[s] for s in train_sets},
            fk_covariances={s: covariance for s in train_sets},
            fk_spec=FKFactorSpec(mode=FK_MODE_FACTOR))

        states = {"no_fk": state_no_fk, "fixed_fk": state_fixed, "factor_fk": state_factor}
        diags = {"no_fk": diag_no_fk, "fixed_fk": diag_fixed, "factor_fk": diag_factor}

        line = [f"h={h:2d} (event {excluded_event})"]
        for mode in MODES:
            held = evaluate_held_out(
                states[mode], held_obs, gripper, robot_T, excluded_event,
                T_base_cube_fk[h], K_map, D_map)
            for cam, values in held.pop("raw_errors_by_camera").items():
                per_mode_raw_by_cam[mode][cam].extend(values)
            record = {
                "held_out_set": h,
                "held_out_event": excluded_event,
                "solve_success": bool(diags[mode]["success"]),
                "train_reprojection_rmse_px": diags[mode]["train_reprojection_rmse_px"],
                "held_out_reprojection_rmse_px": held["rmse_px"],
                "held_out_n_corners": held["n_corners"],
                "held_out_rmse_by_camera_px": {
                    cam: rec["rmse_px"] for cam, rec in held["by_camera"].items()},
            }
            if mode == "factor_fk":
                record["fk_delta_translation_mm"] = (
                    np.asarray(artifact["T_fk_cube_center_to_tag_object"])[:3, 3] * 1000).tolist()
            per_mode_iterations[mode].append(record)
            line.append(f"{mode}: train={record['train_reprojection_rmse_px']:.3f}px "
                        f"held_out={record['held_out_reprojection_rmse_px']:.3f}px")
        elapsed = time.perf_counter() - started
        print(" | ".join(line) + f" ({elapsed:.1f}s)")

    def agg(values):
        arr = np.asarray(values, dtype=np.float64)
        return {
            "mean_px": float(np.mean(arr)), "median_px": float(np.median(arr)),
            "max_px": float(np.max(arr)), "min_px": float(np.min(arr)),
            "n_iterations": int(len(arr)),
        }

    summary = {}
    for mode in MODES:
        records = per_mode_iterations[mode]
        pooled_by_cam = {
            cam: {
                "rmse_px": float(np.sqrt(np.mean(np.square(values)))),
                "n_corners": int(len(values)),
            }
            for cam, values in sorted(per_mode_raw_by_cam[mode].items())
        }
        all_pooled = [e for values in per_mode_raw_by_cam[mode].values() for e in values]
        summary[mode] = {
            "held_out_rmse_per_iteration_px_stats": agg(
                [r["held_out_reprojection_rmse_px"] for r in records]),
            "held_out_rmse_pooled_all_corners_px": float(
                np.sqrt(np.mean(np.square(all_pooled)))),
            "held_out_rmse_pooled_by_camera_px": pooled_by_cam,
            "train_reprojection_rmse_px_stats": agg(
                [r["train_reprojection_rmse_px"] for r in records]),
            "n_successful_solves": int(sum(r["solve_success"] for r in records)),
        }

    print()
    print(f"{'mode':>10} {'held_out_mean_px':>18} {'held_out_median_px':>20} "
          f"{'held_out_max_px':>17} {'train_mean_px':>15} {'n_iter':>7}")
    for mode in MODES:
        s = summary[mode]
        print(f"{mode:>10} {s['held_out_rmse_per_iteration_px_stats']['mean_px']:>18.4f} "
              f"{s['held_out_rmse_per_iteration_px_stats']['median_px']:>20.4f} "
              f"{s['held_out_rmse_per_iteration_px_stats']['max_px']:>17.4f} "
              f"{s['train_reprojection_rmse_px_stats']['mean_px']:>15.4f} "
              f"{s['held_out_rmse_per_iteration_px_stats']['n_iterations']:>7d}")

    output = {
        "warning": (
            "Leave-one-placement-out held-out evaluation of the same no_fk/"
            "fixed_fk/factor_fk comparison fit_fk_ablation_diagnostic.py made "
            "train-pooled-only. Camera 1 is the gripper (eye-in-hand); cameras "
            "0/2/3 are the fixed (eye-to-hand) cameras."),
        "gripper_cam_idx": gripper,
        "all_session2_sets": all_sets,
        "held_out_sets_run": held_out_sets,
        "grasp_offset_policy": (
            "T_gripper_cube held fixed at pass1_grasp_offset.json's fitted "
            "value in every iteration/mode (session1-only support, invariant "
            "under leaving out any session2 placement)."),
        "summary": summary,
        "iterations": per_mode_iterations,
    }
    Path(args.out).write_text(json.dumps(output, indent=2))
    print(f"\nwrote {args.out}")


if __name__ == "__main__":
    main()
