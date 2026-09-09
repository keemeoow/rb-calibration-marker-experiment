#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""zeus_gello_calibration/eval_heldout_and_consistency.py -- 통합/독립 x
raw-fk/no-fk 4가지 방식에 대해 (1) session2 세트 leave-one-out held-out
reprojection RMSE, (2) 카메라 간 큐브 pose 일치도(cross-camera consistency,
mm/deg)를 계산한다. late_table1(CP_result/session04)의 "Heldout Cube RMSE"/
"Cross-view Cube px"/"Cam-common Cube mm/deg" 지표를 Zeus 데이터로 재현한 것.

Held-out: session2의 15개 세트를 하나씩 빼고(session1+session3는 항상 포함)
나머지로 다시 fit한 뒤, 뺐던 세트의 이미지들로만(그 세트를 본 카메라들의 PnP를
frozen 카메라 extrinsics로 base 좌표계에 옮겨 평균) 큐브 pose를 추정하고,
그 pose로 그 세트의 코너들을 재투영해서 오차를 잰다 -- 학습에 전혀 안 쓰인
세트에 대한 일반화 성능.

Cross-camera consistency: (held-out 아닌) 전체 데이터로 한 fit에서, 같은
세트를 본 카메라들이 각자 독립적으로 계산한 큐브 pose끼리 얼마나 다른지
(평행이동 mm, 회전 deg) -- 카메라들끼리 서로 동의하는 정도.

사용법:
  python eval_heldout_and_consistency.py
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT))

from calibration_pipeline import se3 as cp  # noqa: E402
from calibration_pipeline.apriltag_cube import inv_T  # noqa: E402
from calibration_pipeline.path_evaluation import solve_observed_pose  # noqa: E402
from calibration_pipeline.reprojection import pose_delta, project_points  # noqa: E402
from calibration_pipeline.table1 import estimate_board_handeye_initial  # noqa: E402

from fit_calibration_methods import (  # noqa: E402
    GRIPPER_LOCAL_ID, SESSION1_DIR_DEFAULT, SESSION3_DIR_DEFAULT, load_all_data,
    rmse_px, solve_sequential, solve_unified,
)
from session2_pick_and_place import SESSION2_DIR_DEFAULT  # noqa: E402


def camera_cube_estimate(obs, cams, gtc, robot_T, K_map, D_map, gripper_id):
    T_cam_cube = solve_observed_pose(obs, K_map, D_map)
    if T_cam_cube is None:
        return None
    c = int(obs.cam)
    if c == gripper_id:
        if gtc is None or int(obs.event) not in robot_T:
            return None
        return robot_T[int(obs.event)] @ gtc @ T_cam_cube
    if cams is None or c not in cams:
        return None
    return cams[c] @ T_cam_cube


def fit_frozen(method, data_fold, fk_mode, gtc_init, board_init):
    """(cams, gtc) 프리즈된 값 반환 -- 통합/sequential 공통 인터페이스.
    sequential(table1.py A1 방식)은 no_fk(estimated)에서만 존재한다."""
    if method == "통합":
        state, _, _ = solve_unified(data_fold, fk_mode, gtc_init, board_init)
        return state.cams, state.gtc
    final1, _, final2, _, _, _ = solve_sequential(data_fold, gtc_init, board_init)
    return final2.cams, final1.gtc


def evaluate_heldout(method, data, fk_mode, gtc_init, board_init, robot_T_all, K_map, D_map, set_ids):
    all_errs = []
    per_set = {}
    obs_all_s2 = data["obs_s2_fixed"] + data["obs_s2_gripper"]
    for s in set_ids:
        data_fold = dict(data)
        data_fold["obs_s2_fixed"] = [o for o in data["obs_s2_fixed"] if o.set_idx is None or int(o.set_idx) != s]
        data_fold["obs_s2_gripper"] = [o for o in data["obs_s2_gripper"] if o.set_idx is None or int(o.set_idx) != s]
        data_fold["items_by_index"] = {k: v for k, v in data["items_by_index"].items() if k != s}
        try:
            cams, gtc = fit_frozen(method, data_fold, fk_mode, gtc_init, board_init)
        except Exception as exc:
            print(f"    [WARN] {method} fk={fk_mode} set={s}: fold fit 실패 ({exc}), 건너뜀")
            continue

        heldout_obs = [o for o in obs_all_s2 if o.set_idx is not None and int(o.set_idx) == s]
        cands = [camera_cube_estimate(o, cams, gtc, robot_T_all, K_map, D_map, GRIPPER_LOCAL_ID) for o in heldout_obs]
        cands = [c for c in cands if c is not None]
        if not cands:
            continue
        T_pose = cands[0] if len(cands) == 1 else cp.robust_se3_average(cands, None)[0]

        set_errs = []
        for o in heldout_obs:
            c = int(o.cam)
            if c == GRIPPER_LOCAL_ID:
                if int(o.event) not in robot_T_all:
                    continue
                T_base_cam = robot_T_all[int(o.event)] @ gtc
            else:
                if c not in cams:
                    continue
                T_base_cam = cams[c]
            pred = project_points(inv_T(T_base_cam) @ T_pose, o.object_points, K_map[c], D_map[c])
            e = np.linalg.norm(pred - o.image_points, axis=1)
            set_errs.extend(e.tolist())
        all_errs.extend(set_errs)
        per_set[s] = rmse_px(set_errs)
    return rmse_px(all_errs), per_set


def cross_camera_consistency(obs_all_s2, cams, gtc, robot_T_all, K_map, D_map, gripper_id):
    by_set = {}
    for o in obs_all_s2:
        if o.set_idx is None:
            continue
        by_set.setdefault(int(o.set_idx), []).append(o)
    trans_mm, rot_deg, n_pairs = [], [], 0
    for s, obs_list in by_set.items():
        cam_pose = {}
        for o in obs_list:
            est = camera_cube_estimate(o, cams, gtc, robot_T_all, K_map, D_map, gripper_id)
            if est is not None:
                cam_pose[int(o.cam)] = est  # 세트당 카메라 1관측 가정 (session2 구조상 맞음)
        cam_ids = sorted(cam_pose)
        for i in range(len(cam_ids)):
            for j in range(i + 1, len(cam_ids)):
                d_mm, d_deg = pose_delta(cam_pose[cam_ids[i]], cam_pose[cam_ids[j]])
                trans_mm.append(d_mm)
                rot_deg.append(d_deg)
                n_pairs += 1
    return (float(np.mean(trans_mm)) if trans_mm else float("nan"),
            float(np.mean(rot_deg)) if rot_deg else float("nan"), n_pairs)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--session1-dir", default=str(SESSION1_DIR_DEFAULT))
    ap.add_argument("--session1-capture-subdir", default="capture_replayed")
    ap.add_argument("--session2-dir", default=str(SESSION2_DIR_DEFAULT))
    ap.add_argument("--session2-capture-subdir", default="capture_placed")
    ap.add_argument("--session3-dir", default=str(SESSION3_DIR_DEFAULT))
    ap.add_argument("--session3-capture-subdir", default="capture_replayed")
    ap.add_argument("--zeus-intrinsics-dir", default=str(REPO_ROOT / "intrinsics"))
    ap.add_argument("--ur3-intrinsics-dir", default=str(REPO_ROOT / "ur3_calibration" / "intrinsics"))
    ap.add_argument("--device-map", default=str(REPO_ROOT / "intrinsics" / "device_map.json"))
    ap.add_argument("--fit-json", default=str(REPO_ROOT / "zeus_gello_calibration" / "pass1_grasp_offset_replayed.json"))
    ap.add_argument("--fixed-min-corners", type=int, default=8)
    ap.add_argument("--cube-observation-policy", default="legacy", choices=("legacy", "core_multiface"))
    ap.add_argument("--out", default=str(REPO_ROOT / "zeus_gello_calibration" / "heldout_and_consistency.json"))
    args = ap.parse_args()

    data = load_all_data(args)
    K_map, D_map = data["K_map"], data["D_map"]
    set_ids = sorted(data["items_by_index"])
    robot_T_all = {**data["robot_T_s1"], **data["robot_T_s2_gripper"], **data["robot_T_s3"]}
    obs_all_s2 = data["obs_s2_fixed"] + data["obs_s2_gripper"]

    gtc_init, board_init, _eih_diag = estimate_board_handeye_initial(
        data["obs_s3"], data["robot_T_s3"], K_map, D_map, GRIPPER_LOCAL_ID)

    results = {}
    # sequential(table1.py A1 방식)은 no_fk에서만 존재 -- raw-fk+sequential
    # 조합은 table1.py에 없어서 안 만듦 (fit_calibration_methods.py 참고).
    for method, fk_mode, label in (
        ("통합", "no_fk", "통합_no-fk"), ("통합", "fixed_fk", "통합_raw-fk"),
        ("sequential", "no_fk", "sequential_no-fk"),
    ):
        print(f"[{label}] leave-one-out held-out 계산 중 ({len(set_ids)}개 세트)...")
        heldout_rmse, per_set = evaluate_heldout(
            method, data, fk_mode, gtc_init, board_init, robot_T_all, K_map, D_map, set_ids)

        cams_full, gtc_full = fit_frozen(method, data, fk_mode, gtc_init, board_init)
        trans_mm, rot_deg, n_pairs = cross_camera_consistency(
            obs_all_s2, cams_full, gtc_full, robot_T_all, K_map, D_map, GRIPPER_LOCAL_ID)

        results[label] = {
            "heldout_cube_rmse_px": heldout_rmse,
            "n_heldout_sets": len(per_set),
            "cross_camera_translation_mm": trans_mm,
            "cross_camera_rotation_deg": rot_deg,
            "n_camera_pairs": n_pairs,
            "per_set_heldout_rmse_px": per_set,
        }

    print(f"\n{'condition':>16} {'heldout_rmse_px':>16} {'cross_cam_mm':>13} {'cross_cam_deg':>14} {'n_pairs':>8}")
    for name in ("통합_raw-fk", "통합_no-fk", "sequential_no-fk"):
        r = results[name]
        print(f"{name:>16} {r['heldout_cube_rmse_px']:>16.4f} {r['cross_camera_translation_mm']:>13.4f} "
              f"{r['cross_camera_rotation_deg']:>14.4f} {r['n_camera_pairs']:>8d}")

    Path(args.out).write_text(json.dumps({"results": results}, indent=2))
    print(f"\nwrote {args.out}")


if __name__ == "__main__":
    main()
