#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ur3_calibration/cross_camera_consistency.py -- cross-camera pose-agreement
check for the "fixed FK" calibration, the second evaluation criterion the
user originally asked for (the first, leave-one-placement-out held-out
accuracy, is done -- see leave_one_out_eval.py/leave_one_out_eval.json).

Background: ``calibration_pipeline/path_evaluation.py`` (``evaluate_fixed_to_
fixed_cross_target`` etc., driven via ``tools/evaluate_cross_target.py``) is
the pipeline's built-in tool for this, but it is wired to a frozen evaluation
mask built from a completed ``table1_methods.json`` row -- artifacts this
project cannot produce (blocked by the same event-split issue documented in
``fit_grasp_offset.py`` and ``leave_one_out_eval.py``: session2 photographed
each of its 12 placements exactly once, so ``table1.py``'s A2/A3/A4 rows and
their event-stratified split cannot run on this dataset).

Rather than reinvent the comparison from scratch, this script imports and
reuses ``path_evaluation.py``'s ACTUAL per-observation math directly with
plain ``T_base_cam``/``T_gripper_cam`` matrices instead of the full table1.py
artifact bundle -- confirmed by reading the module that its core functions
take exactly that:
  * ``solve_observed_pose(obs, K_map, D_map)`` -- measurement-only PnP from
    one camera's own detected corners (IPPE for the planar single/two-marker
    case, RANSAC-EPnP+LM otherwise via the SAME ``AprilTagCubeTarget``
    corner-detection machinery already used everywhere else in this chain,
    since ``obs`` here comes from ``load_cube_board_pixel_observations``).
  * ``fixed_camera_cube_pose_in_base(T_base_camera, T_camera_cube_pnp)`` --
    composes a fixed camera's measurement into the base frame
    (``T_base_camera @ T_camera_cube_pnp``); for the gripper camera the
    equivalent chain (see ``evaluate_paths_with_frozen_mask`` and
    ``evaluate_gripper_to_fixed_cross_target`` in the same module) is
    ``T_base_gripper_camera_event @ T_camera_cube_pnp`` with
    ``T_base_gripper_camera_event = robot_T[event] @ T_gripper_cam``.
  * ``fixed_camera_target_pair_disagreement(left, right)`` -- the exact
    translation-mm / rotation-deg pairwise pose disagreement metric
    (``E_CROSS_CONTRACT``'s definition), used unmodified here.
This script only supplies the mask-building/table1.py-specific glue's
functional equivalent (grouping observations by event and enumerating
camera pairs) with plain data, since that glue is what the full artifact
bundle exists to freeze for leakage-safety across many methods -- irrelevant
here, where there is exactly one method (fixed_fk) and no train/held-out
split at all: this is a same-event, same-fit self-consistency diagnostic,
not a held-out generalization test (that is leave_one_out_eval.py's job).

Step 1 -- calibration fit: fit_fk_ablation_diagnostic.py's committed
``fk_ablation_diagnostic.json`` already ran this project's "fixed_fk" fit
pooled over ALL 12 session2 placements + session1, but only saved its
train_reprojection_rmse_px summary, not the fitted T_base_Ci/T_gripper_cam/
T_base_board this script needs. So Step 1 refits it fresh, using the SAME
solver call pattern as ``leave_one_out_eval.py``'s ``fixed_fk`` branch
(T_base_cube_by_set hard-fixed to each placement's independent FK-predicted
pose and removed from the optimizer; T_gripper_cube held fixed at Pass 1's
value, per that script's ``grasp_offset_policy`` -- session1 never changes
across any split, so it is not refit here either), just with every one of
the 12 session2 placements in the training set and none held out. Result
saved to ``full_fixed_fk_fit.json`` (T_base_cam / T_gripper_cam / T_base_board
/ T_gripper_cube_fixed + solver diagnostics) so it can be reused without
refitting.

Step 2 -- cross-camera consistency: for every event (session1's 15
cube-gripped events + session2's 12 placement events) with >=2 cameras
holding a valid cube-corner PnP solve, compute each camera's OWN
independent T_base_cube_via_cam (measurement-only PnP through that event's
detected corners, composed with THIS fit's camera extrinsic -- never the
per-set cube variable the fit itself hard-fixed to FK, which would trivially
agree with itself across cameras and test nothing). For every camera pair
with >=1 simultaneous observation, report pairwise translation(mm)/
rotation(deg) disagreement (mean/median/max), reusing
``fixed_camera_target_pair_disagreement`` unmodified.

Usage:
  python ur3_calibration/cross_camera_consistency.py \\
      [--fit-out ur3_calibration/full_fixed_fk_fit.json] \\
      [--out ur3_calibration/cross_camera_consistency.json] \\
      [--reuse-fit]   # skip Step 1 if --fit-out already exists
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from collections import defaultdict
from itertools import combinations
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT))

from calibration_pipeline.apriltag_cube import AprilTagCubeTarget  # noqa: E402
from calibration_pipeline.config import get_default_cube_config  # noqa: E402
from calibration_pipeline.fk_factor import (  # noqa: E402
    FK_MODE_FIXED, FKFactorSpec, solve_factorized_fk,
)
from calibration_pipeline.observations import load_cube_board_pixel_observations  # noqa: E402
from calibration_pipeline.path_evaluation import (  # noqa: E402
    E_CROSS_CONTRACT, fixed_camera_cube_pose_in_base,
    fixed_camera_target_pair_disagreement, solve_observed_pose,
)
from calibration_pipeline.reprojection import PoseState, SolverOptions, variable_keys  # noqa: E402
from calibration_pipeline.runtime import (  # noqa: E402
    get_capture_set_cube_center_transform_raw, load_intrinsics_with_depth_scale,
)
from calibration_pipeline.schema import RAW_FK_CUBE_CENTER_TO_OBJECT  # noqa: E402

ROOT = str(REPO_ROOT / "data" / "ur3_session12" / "calib_train")
INTRINSICS_DIR = str(REPO_ROOT / "ur3_calibration" / "intrinsics")
PASS1_JSON = REPO_ROOT / "ur3_calibration" / "pass1_grasp_offset.json"
DEVICE_MAP_JSON = REPO_ROOT / "ur3_calibration" / "intrinsics" / "device_map.json"
MECHANICAL_MAP = np.asarray(RAW_FK_CUBE_CENTER_TO_OBJECT, dtype=np.float64)


def load_common_inputs():
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
    return meta, gripper, all_cam_ids, K_map, D_map, cube, observations, load_diag, robot_T


def fit_full_fixed_fk(meta, gripper, K_map, D_map, observations, robot_T, fit_out_path: Path):
    """Fit fixed_fk pooled over ALL 12 session2 placements + session1 (no
    held-out), exactly reproducing leave_one_out_eval.py's fixed_fk branch
    with train_sets = every available set and train_obs = every observation.
    """
    started = time.perf_counter()

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

    pass1 = json.loads(PASS1_JSON.read_text())
    T_gripper_cube_fixed = np.asarray(pass1["T_gripper_cube"], dtype=np.float64)
    vision_cubes_full = {
        int(s): np.asarray(T, dtype=np.float64)
        for s, T in pass1["T_base_cube_by_set_vision_only"].items()
    }
    cam_init_full = {int(c): np.asarray(T, dtype=np.float64) for c, T in pass1["T_base_cam"].items()}
    gtc_init = np.asarray(pass1["T_gripper_cam"], dtype=np.float64)
    board_init = np.asarray(pass1["T_base_board"], dtype=np.float64)

    base_state = PoseState(
        cams={c: cam_init_full[c].copy() for c in cam_init_full},
        gtc=gtc_init.copy(),
        board=board_init.copy(),
        cubes={s: vision_cubes_full[s].copy() for s in all_sets},
        grasps={0: T_gripper_cube_fixed.copy()},
    )

    # No held-out placement: train on every observation, hard-fix every
    # session2 set's cube pose to its FK prediction, and leave
    # T_gripper_cube_by_grasp OUT of the free keys (held fixed at Pass 1's
    # value), exactly matching leave_one_out_eval.py's fixed_fk block and its
    # documented grasp_offset_policy.
    options = SolverOptions()
    state_fixed_init = base_state.clone()
    state_fixed_init.cubes = {s: T_base_cube_fk[s].copy() for s in all_sets}
    fixed_keys = variable_keys(
        ["T_base_Ci", "T_gripper_cam", "T_base_board"], state_fixed_init)
    state_fixed, diag_fixed = solve_factorized_fk(
        observations=observations, variable_keys_=fixed_keys,
        reference_state=state_fixed_init, robot_T=robot_T, K_map=K_map, D_map=D_map,
        gripper_cam_idx=gripper, options=options,
        fk_spec=FKFactorSpec(mode=FK_MODE_FIXED))

    elapsed = time.perf_counter() - started
    print(f"full fixed_fk fit (all {len(all_sets)} session2 placements + "
          f"session1, no held-out): success={diag_fixed['success']} "
          f"train_reprojection_rmse_px={diag_fixed['train_reprojection_rmse_px']:.4f} "
          f"({elapsed:.1f}s)")

    output = {
        "policy": (
            "fixed_fk fit pooled over ALL 12 session2 placements + session1 "
            "(the leave-one-out study's winning FK mode, see "
            "leave_one_out_eval.json), with NO placement held out -- the "
            "production/final calibration this repo recommends. "
            "T_base_cube_by_set is hard-fixed to each placement's "
            "independent FK-predicted pose (removed from the optimizer, "
            "same as leave_one_out_eval.py's fixed_fk branch); "
            "T_gripper_cube is held fixed at pass1_grasp_offset.json's "
            "fitted value (session1-only support, never refit here either)."),
        "gripper_cam_idx": gripper,
        "all_session2_sets": all_sets,
        "n_observations": len(observations),
        "solve_success": bool(diag_fixed["success"]),
        "train_reprojection_rmse_px": diag_fixed["train_reprojection_rmse_px"],
        "elapsed_seconds": elapsed,
        "T_base_cam": {str(c): state_fixed.cams[c].tolist() for c in sorted(state_fixed.cams)},
        "T_gripper_cam": state_fixed.gtc.tolist(),
        "T_base_board": state_fixed.board.tolist(),
        "T_gripper_cube_fixed": T_gripper_cube_fixed.tolist(),
    }
    fit_out_path.write_text(json.dumps(output, indent=2))
    print(f"wrote {fit_out_path}")
    return state_fixed


def load_fit_state(fit_out_path: Path) -> PoseState:
    data = json.loads(fit_out_path.read_text())
    return PoseState(
        cams={int(c): np.asarray(T, dtype=np.float64) for c, T in data["T_base_cam"].items()},
        gtc=np.asarray(data["T_gripper_cam"], dtype=np.float64),
        board=np.asarray(data["T_base_board"], dtype=np.float64),
        cubes={},
        grasps={0: np.asarray(data["T_gripper_cube_fixed"], dtype=np.float64)},
    )


def camera_role_labels():
    device_map = json.loads(DEVICE_MAP_JSON.read_text())
    gripper = int(device_map["gripper_cam_idx"])
    idx_to_serial = {int(v): k for k, v in device_map["serial_to_idx"].items()}
    labels = {}
    for cam, serial in idx_to_serial.items():
        role = "gripper (eye-in-hand)" if cam == gripper else "fixed (eye-to-hand)"
        labels[cam] = f"cam{cam} [{role}, serial {serial}]"
    return labels


def per_camera_cube_pose(obs, state: PoseState, gripper: int, robot_T, K_map, D_map):
    """Independent T_base_cube estimate for one camera's own cube observation.

    Measurement-only PnP (``solve_observed_pose``) through THIS event's
    actually-detected corners, composed with the fitted extrinsic for that
    camera -- never the joint fit's own per-set cube variable (which for
    fixed_fk is hard-fixed to FK and identical across cameras by
    construction, so comparing against it would test nothing about the
    cameras' mutual agreement).
    """
    T_cam_cube = solve_observed_pose(obs, K_map, D_map)
    if T_cam_cube is None:
        return None
    cam = int(obs.cam)
    if cam == gripper:
        event = int(obs.event)
        if event not in robot_T:
            return None
        T_base_cam_event = np.asarray(robot_T[event], dtype=np.float64) @ state.gtc
        return fixed_camera_cube_pose_in_base(T_base_cam_event, T_cam_cube)
    if cam not in state.cams:
        return None
    return fixed_camera_cube_pose_in_base(state.cams[cam], T_cam_cube)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--fit-out", default=str(REPO_ROOT / "ur3_calibration" / "full_fixed_fk_fit.json"))
    ap.add_argument("--out", default=str(REPO_ROOT / "ur3_calibration" / "cross_camera_consistency.json"))
    ap.add_argument("--reuse-fit", action="store_true",
                     help="skip Step 1 (refit) and load an existing --fit-out")
    args = ap.parse_args()

    meta, gripper, all_cam_ids, K_map, D_map, cube, observations, load_diag, robot_T = load_common_inputs()
    print(f"loaded {len(observations)} observations "
          f"({load_diag['cube']['counts']['accepted_observations']} cube accepted)")

    fit_out_path = Path(args.fit_out)
    if args.reuse_fit and fit_out_path.exists():
        print(f"reusing existing fit: {fit_out_path}")
        state = load_fit_state(fit_out_path)
    else:
        state = fit_full_fixed_fk(meta, gripper, K_map, D_map, observations, robot_T, fit_out_path)

    # --- Step 2: cross-camera consistency ---
    # session1 (cube_gripped) events use set_index 0 for all 15 events, which
    # is not a real placement id -- report session1/session2 provenance
    # explicitly per event instead of relying on set_index.
    event_session = {}
    for c in meta["captures"]:
        event_session[int(c["event_id"])] = (
            "session1_gripped" if c.get("cube_gripped") else
            f"session2_set{int(c['set_index'])}")

    cube_obs_by_event = defaultdict(dict)  # event -> {cam: obs}
    for o in observations:
        if o.marker != "cube":
            continue
        cube_obs_by_event[int(o.event)][int(o.cam)] = o

    per_event_poses = {}   # event -> {cam: T_base_cube_via_cam or None}
    pnp_failures = []
    for event, by_cam in sorted(cube_obs_by_event.items()):
        poses = {}
        for cam, obs in sorted(by_cam.items()):
            pose = per_camera_cube_pose(obs, state, gripper, robot_T, K_map, D_map)
            if pose is None:
                pnp_failures.append({"event": event, "cam": cam})
            else:
                poses[cam] = pose
        if len(poses) >= 2:
            per_event_poses[event] = poses

    labels = camera_role_labels()
    pair_records = defaultdict(list)  # (camA,camB) -> list of per-event dt/dr rows
    for event, poses in sorted(per_event_poses.items()):
        cams_present = sorted(poses)
        for a, b in combinations(cams_present, 2):
            dt, dr = fixed_camera_target_pair_disagreement(poses[a], poses[b])
            pair_records[(a, b)].append({
                "event": event, "session": event_session[event],
                "translation_mm": dt, "rotation_deg": dr,
            })

    def agg(values):
        arr = np.asarray(values, dtype=np.float64)
        return {
            "mean": float(np.mean(arr)), "median": float(np.median(arr)),
            "max": float(np.max(arr)), "min": float(np.min(arr)),
            "rmse": float(np.sqrt(np.mean(np.square(arr)))),
        }

    all_pairs = list(combinations(sorted(all_cam_ids), 2))
    pair_summary = {}
    for a, b in all_pairs:
        key = f"{a}-{b}"
        rows = pair_records.get((a, b), [])
        if not rows:
            pair_summary[key] = {
                "camera_a": labels.get(a, str(a)), "camera_b": labels.get(b, str(b)),
                "n_simultaneous_observations": 0,
                "translation_mm": None, "rotation_deg": None,
                "note": "no simultaneous cube observation with a valid PnP solve on both cameras",
            }
            continue
        pair_summary[key] = {
            "camera_a": labels.get(a, str(a)), "camera_b": labels.get(b, str(b)),
            "n_simultaneous_observations": len(rows),
            "n_session1": sum(1 for r in rows if r["session"] == "session1_gripped"),
            "n_session2": sum(1 for r in rows if r["session"] != "session1_gripped"),
            "translation_mm": agg([r["translation_mm"] for r in rows]),
            "rotation_deg": agg([r["rotation_deg"] for r in rows]),
            "per_event": rows,
        }

    print()
    print(f"{'pair':>6} {'n_obs':>6} {'trans_mean_mm':>14} {'trans_median_mm':>16} "
          f"{'trans_max_mm':>13} {'rot_mean_deg':>13} {'rot_median_deg':>15} {'rot_max_deg':>12}")
    for a, b in all_pairs:
        key = f"{a}-{b}"
        rec = pair_summary[key]
        if rec["n_simultaneous_observations"] == 0:
            print(f"{key:>6} {0:>6} {'--':>14} {'--':>16} {'--':>13} {'--':>13} {'--':>15} {'--':>12}")
            continue
        t, r = rec["translation_mm"], rec["rotation_deg"]
        print(f"{key:>6} {rec['n_simultaneous_observations']:>6} "
              f"{t['mean']:>14.3f} {t['median']:>16.3f} {t['max']:>13.3f} "
              f"{r['mean']:>13.3f} {r['median']:>15.3f} {r['max']:>12.3f}")

    output = {
        "warning": (
            "Cross-camera cube-pose consistency for the fixed_fk calibration "
            "fit ONCE over ALL 12 session2 placements + session1 (no "
            "held-out placement) -- see full_fixed_fk_fit.json. This is a "
            "same-event, same-fit self-consistency diagnostic (path_"
            "evaluation.py's E_CROSS_CONTRACT methodology, reused directly "
            "via solve_observed_pose/fixed_camera_cube_pose_in_base/"
            "fixed_camera_target_pair_disagreement), NOT a held-out "
            "generalization test -- that is leave_one_out_eval.py's job. "
            "Every per-camera pose here is an independent measurement-only "
            "PnP solve through that camera's own detected corners composed "
            "with its fitted extrinsic; it never reads the joint fit's "
            "per-set cube variable (hard-fixed to FK by construction, so "
            "identical across cameras and uninformative for this check)."),
        "e_cross_contract_reused": E_CROSS_CONTRACT,
        "fit_source": str(fit_out_path),
        "gripper_cam_idx": gripper,
        "camera_labels": labels,
        "per_camera_pose_chain": {
            "fixed_camera": "T_base_cube_via_cam = T_base_camera[fitted] @ T_camera_cube_pnp",
            "gripper_camera": (
                "T_base_cube_via_cam = robot_T[event] @ T_gripper_cam[fitted] "
                "@ T_camera_cube_pnp"),
        },
        "n_events_with_cube_observations": len(cube_obs_by_event),
        "n_events_with_2plus_camera_solves": len(per_event_poses),
        "n_pnp_failures": len(pnp_failures),
        "pnp_failures": pnp_failures,
        "pairwise": pair_summary,
    }
    Path(args.out).write_text(json.dumps(output, indent=2))
    print(f"\nwrote {args.out}")


if __name__ == "__main__":
    main()
