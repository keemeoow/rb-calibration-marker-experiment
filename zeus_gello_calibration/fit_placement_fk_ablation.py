#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""zeus_gello_calibration/fit_placement_fk_ablation.py -- no-FK vs fixed-FK 비교.

session2_pick_and_place.py가 남긴 "큐브를 바닥에 놓고 찍은" 사진들
(session2_floor_board_dual_cam/capture_placed/<idx:03d>/)을 이용해서, 각
placement의 큐브 pose를 두 가지 방식으로 구해 카메라 extrinsics(T_base_Ci)를
같이 최적화하고, train reprojection RMSE를 비교한다:

  no_fk    -- 이미지(PnP)만으로 각 placement의 큐브 pose를 자유 변수로 최적화
              (T_base_cube_by_set). FK/로봇 좌표는 전혀 안 씀.
  fixed_fk -- session2_pick_and_place.py가 그 placement에서 실제로 명령했던
              place 목표 pose(build_plan의 dest_pose, x/y/rz는 원본 session2
              캡처에서, z/ry/rx는 GRASP_REF_POSE 고정값)에 session1 데이터로
              구한 T_gripper_cube(flange-to-cube, pass1_grasp_offset_replayed.json)
              를 곱해서 만든 FK 기반 값으로 큐브 pose를 고정(옵티마이저에서 제외).
              "그 순간 그리퍼가 정확히 그 자리에 있었고 큐브를 놓았으니, 큐브는
              정확히 FK(그 pose) @ T_gripper_cube에 있다"는 가정.

두 조건 모두 카메라 extrinsics는 자유 변수로 같이 풀고(session1 fit을
초기값으로), placement별 큐브 pose 취급 방식만 다르다. train reprojection
RMSE 차이가 곧 "FK 보정이 실제 도움이 되는지"에 대한 지표다 -- ur3_calibration/
fit_fk_ablation_diagnostic.py와 같은 취지의 비교지만, UR3 쪽은 board-jig
기반 mechanical FK를 쓰고 이건 T_gripper_cube 기반이라 방식이 다르다(사용자
지시대로).

주의: 이건 table1.py의 정식 held-out Table 1 지표가 아니다 (train-pooled,
train/held-out 분리 없음) -- 두 조건끼리 비교하는 용도.

사용법:
  python fit_placement_fk_ablation.py
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT))

from calibration_pipeline import se3 as cp  # noqa: E402
from calibration_pipeline.apriltag_cube import AprilTagCubeTarget, inv_T  # noqa: E402
from calibration_pipeline.config import get_default_cube_config  # noqa: E402
from calibration_pipeline.observations import load_cube_pixel_observations  # noqa: E402
from calibration_pipeline.path_evaluation import solve_observed_pose  # noqa: E402
from calibration_pipeline.reprojection import (  # noqa: E402
    PoseState, SolverOptions, solve_corner_reprojection, variable_keys,
)
from robot.backends.zeus_client import pose6_to_T  # noqa: E402

from fit_grasp_offset import (  # noqa: E402
    LOCAL_CAM_IDS, NOT_GRIPPER_SENTINEL, load_intrinsics_by_label, load_robot_T,
)
from session2_pick_and_place import (  # noqa: E402
    SESSION2_DIR_DEFAULT, compute_ordered_targets,
)

FIT_JSON_DEFAULT = REPO_ROOT / "zeus_gello_calibration" / "pass1_grasp_offset_replayed.json"


def build_synthetic_meta_placed(capture_root: Path, capture_indices) -> dict:
    """load_cube_pixel_observations 스키마용 in-memory meta -- cube_gripped=False,
    각 캡처는 자기 자신의 set_index를 가짐 (session1의 build_synthetic_meta와
    거의 같지만 gripped 대신 set 기반)."""
    label_by_id = {v: k for k, v in LOCAL_CAM_IDS.items()}
    captures = []
    for idx in capture_indices:
        folder = f"{idx:03d}"
        cams = {}
        for local_id, label in label_by_id.items():
            rel = f"capture_placed/{folder}/cam_{label}.png"
            if (capture_root.parent / rel).is_file():
                cams[str(local_id)] = {"saved": True, "rgb_path": rel}
        captures.append({
            "event_id": int(idx),
            "cube_gripped": False,
            "set_index": int(idx),
            "cams": cams,
        })
    return {"captures": captures}


def init_cube_poses_from_images(observations, K_map, D_map, cam_init, set_ids):
    """각 placement(set)마다, 그 set을 본 카메라들의 PnP + 초기 카메라
    extrinsics로 T_base_cube 후보를 만들어 robust 평균 (no_fk 초기값)."""
    by_set = {s: [] for s in set_ids}
    for obs in observations:
        s = obs.set_idx
        if s is None or int(s) not in by_set:
            continue
        cam_id = int(obs.cam)
        if cam_id not in cam_init:
            continue
        T_cam_cube = solve_observed_pose(obs, K_map, D_map)
        if T_cam_cube is None:
            continue
        by_set[int(s)].append(cam_init[cam_id] @ T_cam_cube)
    init = {}
    diag = {}
    for s, cands in by_set.items():
        if not cands:
            continue
        if len(cands) == 1:
            init[s] = cands[0]
            diag[s] = {"n_cams": 1}
        else:
            T_avg, d = cp.robust_se3_average(cands, None)
            init[s] = T_avg
            diag[s] = {"n_cams": len(cands), **d}
    return init, diag


def build_fixed_fk_cube_poses(items_by_index, T_gripper_cube):
    """set_index -> T_base_cube = pose6_to_T(그 placement의 실제 place 목표 pose) @ T_gripper_cube."""
    return {
        idx: pose6_to_T(item["target"]) @ T_gripper_cube
        for idx, item in items_by_index.items()
    }


def summarize(name, state, diag, cam_init, ref_cubes):
    t_mm = {}
    for c, T in state.cams.items():
        t_mm[c] = np.round(T[:3, 3] * 1000, 2).tolist()
    cam_shift_mm = {
        c: float(np.linalg.norm(state.cams[c][:3, 3] - cam_init[c][:3, 3]) * 1000)
        for c in state.cams
    }
    return {
        "success": diag["success"],
        "train_reprojection_rmse_px": diag["train_reprojection_rmse_px"],
        "initial_reprojection_rmse_px": diag["initial_reprojection_rmse_px"],
        "n_parameters": diag["n_parameters"],
        "n_residuals": diag["n_residuals"],
        "T_base_cam_translation_mm": t_mm,
        "camera_shift_from_session1_init_mm": cam_shift_mm,
    }


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--session2-dir", default=str(SESSION2_DIR_DEFAULT))
    ap.add_argument("--capture-subdir", default="capture_placed")
    ap.add_argument("--zeus-intrinsics-dir", default=str(REPO_ROOT / "intrinsics"))
    ap.add_argument("--ur3-intrinsics-dir", default=str(REPO_ROOT / "ur3_calibration" / "intrinsics"))
    ap.add_argument("--device-map", default=str(REPO_ROOT / "intrinsics" / "device_map.json"))
    ap.add_argument("--fit-json", default=str(FIT_JSON_DEFAULT),
                    help="T_gripper_cube(session1)/카메라 초기 extrinsics 출처")
    ap.add_argument("--fixed-min-corners", type=int, default=8)
    ap.add_argument("--cube-observation-policy", default="legacy", choices=("legacy", "core_multiface"))
    ap.add_argument("--out", default=str(REPO_ROOT / "zeus_gello_calibration" / "placement_fk_ablation.json"))
    args = ap.parse_args()

    session2_dir = Path(args.session2_dir)
    capture_root = session2_dir / args.capture_subdir
    capture_dirs = sorted(p for p in capture_root.iterdir() if p.is_dir())
    capture_indices = [int(p.name) for p in capture_dirs]
    print(f"placement captures: {capture_root}  ({len(capture_indices)}개: {capture_indices})")

    items = compute_ordered_targets(session2_dir)
    if len(items) != len(capture_indices):
        print(f"[WARN] compute_ordered_targets()가 {len(items)}개, 캡처 폴더가 "
              f"{len(capture_indices)}개 -- session2_pick_and_place.py 실행 순서와 안 맞을 수 있습니다.")
    items_by_index = {idx: items[idx] for idx in capture_indices if idx < len(items)}

    K_map, D_map = load_intrinsics_by_label(
        Path(args.zeus_intrinsics_dir), Path(args.ur3_intrinsics_dir), Path(args.device_map))

    fit = json.loads(Path(args.fit_json).read_text())
    T_gripper_cube = np.asarray(fit["T_gripper_cube"], dtype=np.float64)
    cam_init = {int(k.split("_", 1)[0]): np.asarray(v, dtype=np.float64)
                for k, v in fit["T_base_cam"].items()}
    print(f"session1 fit에서 로드: T_gripper_cube, 카메라 초기 extrinsics {sorted(cam_init)} (from {args.fit_json})")

    meta = build_synthetic_meta_placed(capture_root, capture_indices)
    robot_T = load_robot_T(session2_dir, capture_indices, args.capture_subdir)

    cube_cfg = get_default_cube_config()
    cube = AprilTagCubeTarget(cube_cfg)
    all_cam_ids = sorted(LOCAL_CAM_IDS.values())
    observations, diag = load_cube_pixel_observations(
        str(session2_dir), meta, cube, K_map, D_map, all_cam_ids,
        gripper_cam_idx=NOT_GRIPPER_SENTINEL, exclude_gripped=False,
        fixed_min_corners=args.fixed_min_corners, image_scale=1.0,
        observation_policy=args.cube_observation_policy,
    )
    used_observations = [o for o in observations if int(o.cam) in cam_init]
    print(f"loaded {len(observations)} cube observations, {len(used_observations)} usable "
          f"(camera has session1 init) (policy={args.cube_observation_policy})")

    set_ids = sorted(items_by_index)
    cube_init_no_fk, cube_init_diag = init_cube_poses_from_images(
        used_observations, K_map, D_map, cam_init, set_ids)
    missing_no_fk = sorted(set(set_ids) - set(cube_init_no_fk))
    if missing_no_fk:
        print(f"[WARN] 이미지만으로 초기 큐브 pose를 못 만든 set: {missing_no_fk} (no_fk에서 제외)")

    cube_fixed_fk = build_fixed_fk_cube_poses(items_by_index, T_gripper_cube)

    options = SolverOptions()
    results = {}

    # --- no_fk: 카메라 extrinsics + 큐브 pose(set별) 전부 자유 변수 ---
    no_fk_set_ids = sorted(cube_init_no_fk)
    state_no_fk = PoseState(
        cams=dict(cam_init), gtc=np.eye(4), board=None,
        cubes={s: cube_init_no_fk[s] for s in no_fk_set_ids}, grasps={},
    )
    obs_no_fk = [o for o in used_observations if o.set_idx is not None and int(o.set_idx) in cube_init_no_fk]
    keys_no_fk = variable_keys(["T_base_Ci", "T_base_cube_by_set"], state_no_fk)
    final_no_fk, diag_no_fk = solve_corner_reprojection(
        observations=obs_no_fk, variable_keys_=keys_no_fk, reference_state=state_no_fk,
        robot_T=robot_T, K_map=K_map, D_map=D_map,
        gripper_cam_idx=NOT_GRIPPER_SENTINEL, options=options,
    )
    results["no_fk"] = summarize("no_fk", final_no_fk, diag_no_fk, cam_init, cube_init_no_fk)
    results["no_fk"]["n_sets_used"] = len(no_fk_set_ids)

    # --- fixed_fk: 카메라 extrinsics만 자유, 큐브 pose는 FK(place 목표)@T_gripper_cube로 고정 ---
    state_fixed = PoseState(
        cams=dict(cam_init), gtc=np.eye(4), board=None,
        cubes=dict(cube_fixed_fk), grasps={},
    )
    obs_fixed = [o for o in used_observations if o.set_idx is not None and int(o.set_idx) in cube_fixed_fk]
    keys_fixed = variable_keys(["T_base_Ci"], state_fixed)
    final_fixed, diag_fixed = solve_corner_reprojection(
        observations=obs_fixed, variable_keys_=keys_fixed, reference_state=state_fixed,
        robot_T=robot_T, K_map=K_map, D_map=D_map,
        gripper_cam_idx=NOT_GRIPPER_SENTINEL, options=options,
    )
    results["fixed_fk"] = summarize("fixed_fk", final_fixed, diag_fixed, cam_init, cube_fixed_fk)
    results["fixed_fk"]["n_sets_used"] = len(cube_fixed_fk)

    print()
    print(f"{'condition':>10} {'n_sets':>8} {'train_rmse_px':>15} {'initial_rmse_px':>17} {'success':>8}")
    for name, r in results.items():
        print(f"{name:>10} {r['n_sets_used']:>8d} {r['train_reprojection_rmse_px']:>15.4f} "
              f"{r['initial_reprojection_rmse_px']:>17.4f} {str(r['success']):>8}")

    out = {
        "warning": ("train-pooled, no held-out split -- table1.py의 정식 지표가 아님. "
                    "no_fk/fixed_fk 두 조건끼리 비교하는 용도."),
        "session2_dir": str(session2_dir),
        "capture_subdir": args.capture_subdir,
        "fit_json_source": args.fit_json,
        "results": results,
        "cube_init_diag_no_fk": cube_init_diag,
    }
    Path(args.out).write_text(json.dumps(out, indent=2))
    print(f"\nwrote {args.out}")


if __name__ == "__main__":
    main()
