#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ur3_calibration/fit_grasp_offset.py -- Pass 1: fit T_gripper_cube directly.

Why this bypasses 05_calibrate.py/table1.py's row machinery (documented in
the final report in detail; summary here):

table1.py's own event-stratified split (``build_event_split``) requires, for
every set_index to be "eligible", at least ``min_train_eih_cube_events + 1``
DISTINCT EVENTS in which the EYE-IN-HAND (gripper) camera sees the cube for
that set -- so it can both train AND hold out an event within the same set.
That assumption matches the Zeus session04 dataset (avg ~6.5 gripper-camera
views per placement) but not this UR3 capture design:

  - session1 (cube gripped, set_index 0): the gripper camera is mounted
    right next to the grasped cube and, in every single one of the 15
    captures, fails to get a usable multi-corner AprilTag detection of it
    (verified directly: 0/15 "core_multiface" AND 0/15 even under the
    permissive "legacy" cube_observation_policy -- it is simply too close/
    obliquely angled). Session1's useful, richly-varying observations of the
    cube are all EYE-TO-HAND (the three fixed cameras watching the gripper
    carry the cube through 15 different robot poses) -- but build_event_split
    only counts eye-in-hand cube events, so set 0 is unconditionally dropped.
  - session2 (cube placed, sets 1..12): each floor placement was photographed
    exactly ONCE (fixed cams + the gripper cam parked at one fixed pose) --
    there is only ever 1 total event per set, so no set can supply both a
    train event and a held-out test event no matter how
    --min_train_eih_cube_events is tuned (build_event_split floors the
    held-out count at ``max(1, ...)``, always consuming the set's only event).

Verified concretely: with --include_gripped_cube and even
--min_train_eih_cube_events 0, table1.py's real A2 run produces zero total
train events and dies in infer_board_square_length_mm ("cannot infer board
square length without board corners") -- confirmed against the actual CLI,
not just this diagnostic script.

This is a capture-density mismatch in table1.py's split algorithm, not a bug
in the converted meta.json and not something to route around by editing
schema.py/table1.py's evaluation methodology. So table1.py's A2/A3/A4 rows
(and their held-out reprojection metric) cannot be produced on this dataset
as captured -- see the final report for the recapture recommendation.

What CAN be done, and what this script does: reprojection.py's own bundle-
adjustment solver (solve_corner_reprojection) does not care about
table1.py's split at all -- it is a generic corner-reprojection least-squares
problem over an explicit variable-key list, and table1.py's own
``variable_keys()`` helper already supports a ``T_gripper_cube_by_grasp`` ->
("grasp", g) family (added for exactly this grip-target model, see
reprojection.py's PoseState.grasps / GRIPPED_TARGET_GRASP). This script calls
that solver directly on ALL of session1+session2's observations pooled
together (no train/held-out split -- this is a one-shot auxiliary
calibration step to recover T_gripper_cube and session2's per-placement cube
poses, not itself a reported ablation row), using cube_observation_policy=
"legacy" (a first-class, documented policy option in observations.py, not a
hack) so single/planar-face cube views are usable too.

Initialization:
  - T_gripper_cam (gripper cam extrinsic) and T_base_board: from
    table1.estimate_board_handeye_initial() on the gripper camera's board
    views (present in both session1 and session2 images).
  - T_gripper_cube and each fixed camera's T_base_Ci: for each fixed camera
    c in {0,2,3}, session1's cube_gripped=True observations captured by c
    give, per event i, T_base_gripper[i] (robot FK) and T_cam_cube[i] (PnP),
    related by the CONSTANT unknowns T_gripper_cube and T_base_C_c via
        T_base_gripper[i] @ T_gripper_cube = T_base_C_c @ T_cam_cube[i]   (*)
    This is an eye-TO-hand configuration (fixed camera, pattern riding the
    gripper) -- the AX=ZB form solved by cv2.calibrateRobotWorldHandEye, NOT
    the AX=XB form solved by cv2.calibrateHandEye/estimate_board_handeye_
    initial (that pair assumes the OPPOSITE physical setup: moving camera,
    fixed pattern -- an earlier version of this script mis-reused it for the
    cube and got 3 wildly inconsistent T_gripper_cube estimates, ~150mm/32deg
    apart, which then made the joint solver diverge; this was caught by the
    per-camera dispersion check below before trusting any fitted number).
    calibrateRobotWorldHandEye solves A[i]@X = Z@B[i]; feeding
    A[i]=T_base_gripper[i], B[i]=T_cam_cube[i] returns X=T_gripper_cube,
    Z=T_base_C_c directly -- exactly (*), with no relabeling trick beyond
    which argument slot each per-frame matrix goes into (see
    estimate_grasp_offset_one_camera() below). Averaging the 3 fixed
    cameras' independent estimates gives both T_gripper_cube's initial guess
    and each camera's initial T_base_Ci.
  - Each session2 set's initial T_base_cube: robust average of
    T_base_C_c(init) @ T_cam_cube[event] over whichever fixed cameras saw
    that placement's single event.

Final fit: solve_corner_reprojection() with variable keys T_base_Ci (0,2,3),
T_gripper_cam, T_base_board, T_base_cube_by_set (sets 1..12),
T_gripper_cube_by_grasp (grasp 0) -- jointly, from every corner observation.

Output: JSON with the fitted T_gripper_cube (matrix + mm/deg), each fitted
session2 T_base_cube_by_set, diagnostics (train reprojection RMSE, per-fixed-
camera-source dispersion), for convert_to_meta.py's --set-cube-center-json
and for the report's sanity checks.
"""

from __future__ import annotations

import argparse
import json
import sys
from collections import defaultdict
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT))

from calibration_pipeline import se3 as cp  # noqa: E402
from calibration_pipeline import table1 as t1  # noqa: E402
from calibration_pipeline.apriltag_cube import AprilTagCubeTarget, inv_T  # noqa: E402
from calibration_pipeline.config import get_default_cube_config  # noqa: E402
from calibration_pipeline.observations import load_cube_board_pixel_observations  # noqa: E402
from calibration_pipeline.path_evaluation import solve_observed_pose  # noqa: E402
from calibration_pipeline.reprojection import (  # noqa: E402
    PoseState, SolverOptions, solve_corner_reprojection, variable_keys,
)
from calibration_pipeline.runtime import load_intrinsics_with_depth_scale  # noqa: E402

# schema.py's frozen mechanical map for A3's raw-FK-fixed row (see
# calibration_pipeline/schema.py::RAW_FK_CUBE_CENTER_TO_OBJECT). It is its
# own inverse (a diag(-1,1,-1) rotation), used only in convert_to_meta.py's
# encoding, not here; kept out of this module deliberately (this module only
# produces T_base_cube, in the SAME frame convention the vision pipeline
# itself uses -- see convert_to_meta.py's docstring for why that needs one
# more step before being stored as set_cube_center_6dof).


def estimate_grasp_offset_one_camera(cube_obs_c, robot_T, K_map, D_map, cam_idx):
    """Eye-to-hand init for one fixed camera c: solve (*) in the module docstring.

    Returns (T_gripper_cube, T_base_C_c, diag) or raises RuntimeError.
    """
    import cv2

    a_R, a_t, b_R, b_t = [], [], [], []
    for obs in cube_obs_c:
        event = int(obs.event)
        if event not in robot_T:
            continue
        T_cam_cube = solve_observed_pose(obs, K_map, D_map)
        if T_cam_cube is None:
            continue
        T_base_gripper = np.asarray(robot_T[event], dtype=np.float64)
        a_R.append(T_base_gripper[:3, :3])
        a_t.append(T_base_gripper[:3, 3].reshape(3, 1))
        b_R.append(T_cam_cube[:3, :3])
        b_t.append(T_cam_cube[:3, 3].reshape(3, 1))
    if len(a_R) < 5:
        raise RuntimeError(f"cam{cam_idx}: only {len(a_R)} usable gripped-cube poses (<5)")
    best = None
    for name, method in (("SHAH", cv2.CALIB_ROBOT_WORLD_HAND_EYE_SHAH),
                         ("LI", cv2.CALIB_ROBOT_WORLD_HAND_EYE_LI)):
        try:
            R_x, t_x, R_z, t_z = cv2.calibrateRobotWorldHandEye(
                a_R, a_t, b_R, b_t, method=method)
        except Exception:
            continue
        T_gripper_cube = np.eye(4)
        T_gripper_cube[:3, :3] = np.asarray(R_x).reshape(3, 3)
        T_gripper_cube[:3, 3] = np.asarray(t_x).reshape(3)
        T_base_cam = np.eye(4)
        T_base_cam[:3, :3] = np.asarray(R_z).reshape(3, 3)
        T_base_cam[:3, 3] = np.asarray(t_z).reshape(3)
        # Residual check: (*) should hold approximately for every frame.
        errs_mm, errs_deg = [], []
        for T_bg_R, T_bg_t, T_cc_R, T_cc_t in zip(a_R, a_t, b_R, b_t):
            T_bg = np.eye(4); T_bg[:3, :3] = T_bg_R; T_bg[:3, 3] = T_bg_t.reshape(3)
            T_cc = np.eye(4); T_cc[:3, :3] = T_cc_R; T_cc[:3, 3] = T_cc_t.reshape(3)
            lhs = T_bg @ T_gripper_cube
            rhs = T_base_cam @ T_cc
            err = inv_T(lhs) @ rhs
            errs_mm.append(float(np.linalg.norm(err[:3, 3]) * 1000.0))
            errs_deg.append(float(np.degrees(np.linalg.norm(
                Rotation.from_matrix(err[:3, :3]).as_rotvec()))))
        score = float(np.median(errs_mm)) + 5.0 * float(np.median(errs_deg))
        if best is None or score < best[0]:
            best = (score, name, T_gripper_cube, T_base_cam,
                    {"n_poses": len(a_R), "method": name,
                     "residual_median_mm": float(np.median(errs_mm)),
                     "residual_median_deg": float(np.median(errs_deg)),
                     "residual_max_mm": float(np.max(errs_mm)),
                     "residual_max_deg": float(np.max(errs_deg))})
    if best is None:
        raise RuntimeError(f"cam{cam_idx}: both robot-world-hand-eye methods failed")
    _, name, T_gripper_cube, T_base_cam, diag = best
    return T_gripper_cube, T_base_cam, diag


def load_robot_T(meta: dict) -> dict:
    robot_T = {}
    for capture in meta["captures"]:
        event = int(capture["event_id"])
        robot_T[event] = np.asarray(capture["robot_pose_matrix_4x4"], dtype=np.float64)
    return robot_T


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--root-folder", default=str(REPO_ROOT / "data" / "ur3_session12" / "calib_train"))
    ap.add_argument("--intrinsics-dir", default=str(REPO_ROOT / "ur3_calibration" / "intrinsics"))
    ap.add_argument("--cube-observation-policy", default="legacy", choices=("legacy", "core_multiface"))
    ap.add_argument("--out", default=str(REPO_ROOT / "ur3_calibration" / "pass1_grasp_offset.json"))
    args = ap.parse_args()

    root = args.root_folder
    meta = json.loads((Path(root) / "meta.json").read_text())
    gripper = int(meta["gripper_cam_idx"])
    fixed_cams = [c for c in meta["cam_indices"] if c != gripper]
    all_cam_ids = sorted(meta["cam_indices"])

    K_map, D_map = {}, {}
    for c in all_cam_ids:
        K_map[c], D_map[c], _ = load_intrinsics_with_depth_scale(args.intrinsics_dir, c)

    cube_cfg = get_default_cube_config()
    cube = AprilTagCubeTarget(cube_cfg)
    observations, diag = load_cube_board_pixel_observations(
        root, meta, cube, K_map, D_map, all_cam_ids, gripper,
        exclude_gripped_cube=False, fixed_cube_min_corners=8,
        image_scale=1.0, cube_observation_policy=args.cube_observation_policy,
    )
    robot_T = load_robot_T(meta)
    print(f"loaded {len(observations)} observations "
          f"({diag['n_cube_observations']} cube, {diag['n_board_observations']} board)")

    gripper_board_obs = [o for o in observations if o.marker == "board" and int(o.cam) == gripper]
    gtc, board_init, gtc_diag = t1.estimate_board_handeye_initial(
        gripper_board_obs, robot_T, K_map, D_map, gripper)
    print(f"initial T_gripper_cam via board handeye: method={gtc_diag['method']} "
          f"score~{gtc_diag['score_mm_equivalent']:.3f} n_poses={gtc_diag['n_poses']}")

    # Per-fixed-camera initial T_gripper_cube + T_base_C_c from session1's
    # gripped-cube observations (see module docstring for the AX=YB algebra).
    grasp_estimates = []
    cam_init = {}
    for c in fixed_cams:
        cube_obs_c = [
            o for o in observations
            if o.marker == "cube" and int(o.cam) == c and o.grasp_idx is not None
        ]
        if len(cube_obs_c) < 5:
            print(f"  cam{c}: only {len(cube_obs_c)} gripped-cube observations, skipping as an initializer")
            continue
        try:
            gripper_cube_c, cam_c_base, diag_c = estimate_grasp_offset_one_camera(
                cube_obs_c, robot_T, K_map, D_map, c)
        except RuntimeError as exc:
            print(f"  cam{c}: hand-eye init failed ({exc}), skipping as an initializer")
            continue
        grasp_estimates.append(gripper_cube_c)
        cam_init[c] = cam_c_base
        t_mm = gripper_cube_c[:3, 3] * 1000.0
        print(f"  cam{c}: initial T_gripper_cube translation_mm={t_mm.round(1).tolist()} "
              f"(n_poses={diag_c['n_poses']}, method={diag_c['method']}, "
              f"residual_median_mm={diag_c['residual_median_mm']:.2f}, "
              f"residual_median_deg={diag_c['residual_median_deg']:.2f})")

    if not grasp_estimates:
        raise RuntimeError(
            "no fixed camera could initialize T_gripper_cube from session1's gripped-cube views")
    grasp_init, grasp_init_diag = cp.robust_se3_average(grasp_estimates, None)
    print(f"initial T_gripper_cube (robust avg of {len(grasp_estimates)} cams): "
          f"translation_mm={ (grasp_init[:3,3]*1000).round(1).tolist() } "
          f"dispersion={grasp_init_diag}")

    # session2 per-set initial T_base_cube: average T_base_C_c(init) @ T_cam_cube
    # over whichever fixed cams (and the gripper cam, via robot_T@gtc) saw it.
    by_set_obs = defaultdict(list)
    for o in observations:
        if o.marker == "cube" and o.set_idx is not None and o.grasp_idx is None:
            by_set_obs[int(o.set_idx)].append(o)
    cube_init = {}
    for s, obs_list in by_set_obs.items():
        values = []
        for o in obs_list:
            T_cam_cube = solve_observed_pose(o, K_map, D_map)
            if T_cam_cube is None:
                continue
            if int(o.cam) == gripper:
                if int(o.event) not in robot_T:
                    continue
                T_base_cam = robot_T[int(o.event)] @ gtc
            elif int(o.cam) in cam_init:
                T_base_cam = cam_init[int(o.cam)]
            else:
                continue
            values.append(T_base_cam @ T_cam_cube)
        if values:
            cube_init[s] = cp.robust_se3_average(values, None)[0]
    print(f"session2 sets with an initial cube pose: {sorted(cube_init)}")
    missing_sets = sorted(set(by_set_obs) - set(cube_init))
    if missing_sets:
        raise RuntimeError(f"could not initialize cube pose for sets {missing_sets}")

    # Any fixed camera we could not initialize from session1 gripped views,
    # initialize from its session2 cube views + the now-known cube_init.
    for c in fixed_cams:
        if c in cam_init:
            continue
        values = []
        for o in observations:
            if o.marker == "cube" and int(o.cam) == c and o.set_idx in cube_init:
                T_cam_cube = solve_observed_pose(o, K_map, D_map)
                if T_cam_cube is not None:
                    values.append(cube_init[int(o.set_idx)] @ inv_T(T_cam_cube))
        if not values:
            raise RuntimeError(f"cam{c} has no usable observations to initialize its pose at all")
        cam_init[c] = cp.robust_se3_average(values, None)[0]
        print(f"  cam{c}: initial T_base_C from session2 cube views (n={len(values)})")

    state = PoseState(
        cams={c: cam_init[c] for c in fixed_cams},
        gtc=gtc,
        board=board_init,
        cubes={s: cube_init[s] for s in sorted(cube_init)},
        grasps={0: grasp_init},
    )
    keys = variable_keys(
        ["T_base_Ci", "T_gripper_cam", "T_base_board", "T_base_cube_by_set",
         "T_gripper_cube_by_grasp"],
        state,
    )
    print(f"solving with {len(keys)} free SE(3) variables over {len(observations)} observations")
    final_state, solve_diag = solve_corner_reprojection(
        observations=observations,
        variable_keys_=keys,
        reference_state=state,
        robot_T=robot_T,
        K_map=K_map,
        D_map=D_map,
        gripper_cam_idx=gripper,
        options=SolverOptions(),
    )
    print(f"solve success={solve_diag['success']} "
          f"train_reprojection_rmse_px={solve_diag['train_reprojection_rmse_px']:.4f} "
          f"(initial was {solve_diag['initial_reprojection_rmse_px']:.4f})")

    T_gripper_cube = final_state.grasps[0]
    t_mm = (T_gripper_cube[:3, 3] * 1000.0).tolist()
    rot_deg = Rotation.from_matrix(T_gripper_cube[:3, :3]).as_rotvec(degrees=True)
    print(f"FITTED T_gripper_cube: translation_mm={np.round(t_mm, 2).tolist()} "
          f"|t|_mm={np.linalg.norm(t_mm):.2f} rotvec_deg={np.round(rot_deg, 2).tolist()} "
          f"|r|_deg={np.linalg.norm(rot_deg):.2f}")

    result = {
        "T_gripper_cube": T_gripper_cube.tolist(),
        "T_gripper_cube_translation_mm": t_mm,
        "T_gripper_cube_rotvec_deg": rot_deg.tolist(),
        "T_gripper_cam": final_state.gtc.tolist(),
        "T_base_board": final_state.board.tolist(),
        "T_base_cam": {str(c): final_state.cams[c].tolist() for c in final_state.cams},
        "T_base_cube_by_set_vision_only": {
            str(s): final_state.cubes[s].tolist() for s in sorted(final_state.cubes)
        },
        "solve_diagnostics": {
            "success": solve_diag["success"],
            "message": solve_diag["message"],
            "nfev": solve_diag["nfev"],
            "train_reprojection_rmse_px": solve_diag["train_reprojection_rmse_px"],
            "initial_reprojection_rmse_px": solve_diag["initial_reprojection_rmse_px"],
            "n_parameters": solve_diag["n_parameters"],
            "n_residuals": solve_diag["n_residuals"],
            "jacobian_rank_deficient": solve_diag["jacobian"]["rank_deficient"],
        },
        "grasp_init_dispersion_from_3_fixed_cams": grasp_init_diag,
        "cube_observation_policy": args.cube_observation_policy,
    }
    Path(args.out).write_text(json.dumps(result, indent=2))
    print(f"wrote {args.out}")


if __name__ == "__main__":
    main()
