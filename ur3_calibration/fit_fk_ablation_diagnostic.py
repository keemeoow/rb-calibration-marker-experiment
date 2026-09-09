#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ur3_calibration/fit_fk_ablation_diagnostic.py -- supplementary no-FK/fixed-FK/
corrected-FK comparison, NOT table1.py's frozen held-out Table 1 metric.

table1.py's own A2/A3/A4 rows cannot run on this dataset (see
fit_grasp_offset.py's module docstring and the final report for the full
root-cause: table1.py's event-stratified split requires several
eye-in-hand-camera cube observations PER SET so it can hold one out within
that same set, but session1 has zero usable eye-in-hand cube views at all
and every session2 placement was photographed exactly once). Confirmed
directly against the real CLI: 05_calibrate.py --rows A2,A3,A4
--include_gripped_cube on this meta.json raises "no cube set supports the
event-stratified split" (default policy) or "cannot infer board square
length without board corners" (min_train_eih_cube_events=0, which admits
sets with zero training events instead).

This script instead reuses the SAME underlying reprojection.py/fk_factor.py
solver table1.py itself calls, pooling ALL observations with no train/held-
out split, to still produce a same-family, honestly-labeled comparison of
the three FK treatments' TRAIN reprojection RMSE:

  no_fk     -- FK_MODE_NONE:  T_base_cube_by_set free, no FK term (A2-like).
               Identical in every respect to fit_grasp_offset.py's own fit,
               whose train_reprojection_rmse_px is simply reported here.
  fixed_fk  -- FK_MODE_FIXED: T_base_cube_by_set REMOVED from the optimizer,
               hard-fixed to set_cube_center_6dof's T_base_cube_taught (A3-
               like).
  factor_fk -- FK_MODE_FACTOR: T_base_cube_by_set stays free, plus a soft
               whitened SE(3) factor pulling it toward T_fk_raw_set @
               Delta_train, with Delta_train fit by
               fk_alignment.estimate_board_free_fk_cube_artifact from the
               gripper camera's own cube views of the 12 placements (A4-
               like) using the frozen Simulation-compatible covariance
               (0.30deg, 2.0mm) from fk_factor.py.

None of these three re-use table1.py's train/held-out split, shared-baseline
freezing, or frame-prune/refit machinery, and none of them is a substitute
for the real, held-out-evaluated A2/A3/A4 -- they only show, on this
project's own solver, what a soft/hard FK constraint does to the pooled
TRAIN residual when the per-set cube pose is otherwise well supported by
session1+session2's actual images. Report train_reprojection_rmse_px for
each; do not present these as Table 1 rows.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT))

from calibration_pipeline.apriltag_cube import AprilTagCubeTarget  # noqa: E402
from calibration_pipeline.config import get_default_cube_config  # noqa: E402
from calibration_pipeline.fk_alignment import estimate_board_free_fk_cube_artifact  # noqa: E402
from calibration_pipeline.fk_factor import (  # noqa: E402
    FK_MODE_FACTOR, FK_MODE_FIXED, FK_MODE_NONE, FKFactorSpec,
    SIGMA_FK_DEG, SIGMA_FK_MM, diagonal_covariance, solve_factorized_fk,
)
from calibration_pipeline.observations import load_cube_board_pixel_observations  # noqa: E402
from calibration_pipeline.reprojection import PoseState, SolverOptions, variable_keys  # noqa: E402
from calibration_pipeline.runtime import (  # noqa: E402
    get_capture_set_cube_center_transform_raw, load_intrinsics_with_depth_scale,
)
from calibration_pipeline.schema import RAW_FK_CUBE_CENTER_TO_OBJECT  # noqa: E402

ROOT = str(REPO_ROOT / "data" / "ur3_session12" / "calib_train")
INTRINSICS_DIR = str(REPO_ROOT / "ur3_calibration" / "intrinsics")
PASS1_JSON = REPO_ROOT / "ur3_calibration" / "pass1_grasp_offset.json"
MECHANICAL_MAP = np.asarray(RAW_FK_CUBE_CENTER_TO_OBJECT, dtype=np.float64)


def main():
    meta = json.loads((Path(ROOT) / "meta.json").read_text())
    gripper = int(meta["gripper_cam_idx"])
    all_cam_ids = sorted(meta["cam_indices"])
    K_map, D_map = {}, {}
    for c in all_cam_ids:
        K_map[c], D_map[c], _ = load_intrinsics_with_depth_scale(INTRINSICS_DIR, c)
    cube = AprilTagCubeTarget(get_default_cube_config())
    observations, _diag = load_cube_board_pixel_observations(
        ROOT, meta, cube, K_map, D_map, all_cam_ids, gripper,
        exclude_gripped_cube=False, fixed_cube_min_corners=8,
        image_scale=1.0, cube_observation_policy="legacy")
    robot_T = {int(c["event_id"]): np.asarray(c["robot_pose_matrix_4x4"]) for c in meta["captures"]}

    raw_fk_all = {}
    for c in meta["captures"]:
        s = c.get("set_index")
        if s is None or int(s) in raw_fk_all or c.get("cube_gripped"):
            continue
        T_raw = get_capture_set_cube_center_transform_raw(c)
        if T_raw is not None:
            raw_fk_all[int(s)] = np.asarray(T_raw, dtype=np.float64)
    print(f"raw_fk_all sets: {sorted(raw_fk_all)}")
    fixed_cubes = {s: T @ MECHANICAL_MAP for s, T in raw_fk_all.items()}

    pass1 = json.loads(PASS1_JSON.read_text())
    T_gripper_cube_init = np.asarray(pass1["T_gripper_cube"], dtype=np.float64)
    vision_cubes = {
        int(s): np.asarray(T, dtype=np.float64)
        for s, T in pass1["T_base_cube_by_set_vision_only"].items()
    }
    cam_init = {int(c): np.asarray(T, dtype=np.float64) for c, T in pass1["T_base_cam"].items()}
    gtc_init = np.asarray(pass1["T_gripper_cam"], dtype=np.float64)
    board_init = np.asarray(pass1["T_base_board"], dtype=np.float64)

    base_state = PoseState(
        cams=dict(cam_init), gtc=gtc_init, board=board_init,
        cubes=dict(vision_cubes), grasps={0: T_gripper_cube_init},
    )
    options = SolverOptions()
    results = {}

    # --- no_fk (A2-like): identical fit to fit_grasp_offset.py ---
    keys_free_cubes = variable_keys(
        ["T_base_Ci", "T_gripper_cam", "T_base_board", "T_base_cube_by_set",
         "T_gripper_cube_by_grasp"], base_state)
    state_no_fk, diag_no_fk = solve_factorized_fk(
        observations=observations, variable_keys_=keys_free_cubes,
        reference_state=base_state, robot_T=robot_T, K_map=K_map, D_map=D_map,
        gripper_cam_idx=gripper, options=options,
        fk_spec=FKFactorSpec(mode=FK_MODE_NONE))
    results["no_fk"] = {
        "train_reprojection_rmse_px": diag_no_fk["train_reprojection_rmse_px"],
        "success": diag_no_fk["success"],
    }

    # --- fixed_fk (A3-like): cube poses removed from the optimizer entirely ---
    state_fixed = base_state.clone()
    state_fixed.cubes = {s: fixed_cubes[s].copy() for s in sorted(fixed_cubes)}
    keys_fixed = variable_keys(
        ["T_base_Ci", "T_gripper_cam", "T_base_board", "T_gripper_cube_by_grasp"],
        state_fixed)
    state_fixed_out, diag_fixed = solve_factorized_fk(
        observations=observations, variable_keys_=keys_fixed,
        reference_state=state_fixed, robot_T=robot_T, K_map=K_map, D_map=D_map,
        gripper_cam_idx=gripper, options=options,
        fk_spec=FKFactorSpec(mode=FK_MODE_FIXED))
    results["fixed_fk"] = {
        "train_reprojection_rmse_px": diag_fixed["train_reprojection_rmse_px"],
        "success": diag_fixed["success"],
    }

    # --- factor_fk (A4-like): free cube poses + soft whitened FK factor ---
    eih_cube_obs = [
        o for o in observations
        if o.marker == "cube" and int(o.cam) == gripper and o.set_idx is not None
        and int(o.set_idx) in fixed_cubes
    ]
    print(f"eih (gripper-cam) cube observations available for Delta_train alignment: "
          f"{len(eih_cube_obs)} across sets "
          f"{sorted({int(o.set_idx) for o in eih_cube_obs})}")
    aligned_fk_all, _fixed_gtc_init, artifact = estimate_board_free_fk_cube_artifact(
        observations=eih_cube_obs, raw_fk_by_set=raw_fk_all, robot_T=robot_T,
        K_map=K_map, D_map=D_map, gripper_cam_idx=gripper,
        training_set_ids=sorted(fixed_cubes), num_inits=3,
        init_translation_mm=5.0, init_rotation_deg=1.0)
    print(f"Delta_train fit: T_fk_cube_center_to_tag_object translation_mm="
          f"{np.round(np.asarray(artifact['T_fk_cube_center_to_tag_object'])[:3, 3] * 1000, 2).tolist()} "
          f"repeatability={artifact['repeatability']}")

    covariance = diagonal_covariance(SIGMA_FK_MM, SIGMA_FK_DEG)
    state_factor, diag_factor = solve_factorized_fk(
        observations=observations, variable_keys_=keys_free_cubes,
        reference_state=base_state, robot_T=robot_T, K_map=K_map, D_map=D_map,
        gripper_cam_idx=gripper, options=options,
        fk_targets={s: aligned_fk_all[s] for s in sorted(fixed_cubes)},
        fk_covariances={s: covariance for s in sorted(fixed_cubes)},
        fk_spec=FKFactorSpec(mode=FK_MODE_FACTOR))
    results["factor_fk"] = {
        "train_reprojection_rmse_px": diag_factor["train_reprojection_rmse_px"],
        "success": diag_factor["success"],
        "fk_factor_whitened_residual_norm": diag_factor["fk_factor"]["raw_whitened_residual_norm"],
    }

    print()
    print(f"{'condition':>10} {'train_reprojection_rmse_px':>28} {'success':>8}")
    for name, r in results.items():
        print(f"{name:>10} {r['train_reprojection_rmse_px']:>28.4f} {str(r['success']):>8}")

    out_path = REPO_ROOT / "ur3_calibration" / "fk_ablation_diagnostic.json"
    out_path.write_text(json.dumps({
        "warning": (
            "NOT table1.py's frozen held-out Table 1 metric -- train-pooled, "
            "no held-out split (blocked; see fit_grasp_offset.py docstring). "
            "Comparable to each other, not to any ABLATION_TEST_table1_methods.json row."),
        "results": results,
        "delta_train_translation_mm": (
            np.asarray(artifact["T_fk_cube_center_to_tag_object"])[:3, 3] * 1000).tolist(),
    }, indent=2))
    print(f"\nwrote {out_path}")


if __name__ == "__main__":
    main()
