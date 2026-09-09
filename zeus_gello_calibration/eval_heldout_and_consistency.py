#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""zeus_gello_calibration/eval_heldout_and_consistency.py -- 통합/독립 x
raw-fk/no-fk 3가지 방식(통합_no-fk, 통합_raw-fk, 독립_no-fk)에 대해
(1) session2 세트 leave-one-out held-out
reprojection RMSE, (2) 카메라 간 큐브 pose 일치도(cross-camera consistency,
mm/deg)를 계산한다. late_table1(CP_result/session04)의 "Heldout Cube RMSE"/
"Cross-view Cube px"/"Cam-common Cube mm/deg" 지표를 Zeus 데이터로 재현한 것.

Held-out: session2의 15개 세트를 하나씩 빼고(session1+session3는 항상 포함)
나머지로 다시 fit한 뒤, 뺐던 세트의 코너들을 재투영해서 오차를 잰다 -- 학습에
전혀 안 쓰인 세트에 대한 일반화 성능.

*** 정답(ground truth) 값 ***: 예전 버전은 "빠진 세트를 그 세트 자신의
사진들로 삼각측량"해서 정답을 만들었는데, 이러면 정답을 만드는 재료와 검증할
때 쓰는 재료가 똑같은 카메라의 같은 사진이라 카메라들의 공통 편향을 못 잡는다
(4명한테 물어보고 평균 내서 정답 삼은 뒤 다시 그 4명한테 맞는지 물어보는 것과
같음). 지금은 카메라를 전혀 안 쓰고, **그 세트에서 로봇이 실제로 명령받아
이동한 FK 위치 @ session1에서 구한 T_gripper_cube**로 정답을 만든다 --
완전히 비전과 무관한 값이라 카메라들의 공통 편향까지 잡아낼 수 있다.

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
    GRIPPER_LOCAL_ID, SESSION1_DIR_DEFAULT, SESSION3_DIR_DEFAULT, fk_anchor_cubes, load_all_data,
    rmse_px, solve_parallel_fixed, solve_parallel_gripper, solve_unified,
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
    """(cams, gtc) 프리즈된 값 반환 -- 통합/진짜독립 공통 인터페이스.
    진짜독립은 no_fk(estimated)에서만 존재한다."""
    if method == "통합":
        state, _, _ = solve_unified(data_fold, fk_mode, gtc_init, board_init)
        return state.cams, state.gtc
    # method == "독립_true": 고정캠/그리퍼 그룹을 완전히 따로 (핸드오프 없음)
    state_fixed, _, _ = solve_parallel_fixed(data_fold)
    state_gripper, _, _ = solve_parallel_gripper(data_fold, gtc_init, board_init)
    return state_fixed.cams, state_gripper.gtc


def evaluate_heldout(method, data, fk_mode, gtc_init, board_init, robot_T_all, K_map, D_map, set_ids):
    all_errs = []
    per_set = {}
    obs_all_s2 = data["obs_s2_fixed"] + data["obs_s2_gripper"]
    grasp_init = data["grasp_init"]
    for s in set_ids:
        if s not in data["items_by_index"]:
            continue
        data_fold = dict(data)
        data_fold["obs_s2_fixed"] = [o for o in data["obs_s2_fixed"] if o.set_idx is None or int(o.set_idx) != s]
        data_fold["obs_s2_gripper"] = [o for o in data["obs_s2_gripper"] if o.set_idx is None or int(o.set_idx) != s]
        data_fold["items_by_index"] = {k: v for k, v in data["items_by_index"].items() if k != s}
        try:
            cams, gtc = fit_frozen(method, data_fold, fk_mode, gtc_init, board_init)
        except Exception as exc:
            print(f"    [WARN] {method} fk={fk_mode} set={s}: fold fit 실패 ({exc}), 건너뜀")
            continue

        # 정답 = 카메라 전혀 안 쓰고, 그 세트에서 로봇이 실제로 명령받아 간
        # FK 위치 @ session1의 T_gripper_cube로 계산 (비전 무관, 완전 독립).
        T_gt = fk_anchor_cubes({s: data["items_by_index"][s]}, grasp_init)[s]

        heldout_obs = [o for o in obs_all_s2 if o.set_idx is not None and int(o.set_idx) == s]
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
            pred = project_points(inv_T(T_base_cam) @ T_gt, o.object_points, K_map[c], D_map[c])
            e = np.linalg.norm(pred - o.image_points, axis=1)
            set_errs.extend(e.tolist())
        if not set_errs:
            continue
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
    for method, fk_mode, label in (
        ("통합", "no_fk", "통합_no-fk"), ("통합", "fixed_fk", "통합_raw-fk"),
        ("독립_true", "no_fk", "독립_no-fk"),
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
    for name in ("통합_raw-fk", "통합_no-fk", "독립_no-fk"):
        r = results[name]
        print(f"{name:>16} {r['heldout_cube_rmse_px']:>16.4f} {r['cross_camera_translation_mm']:>13.4f} "
              f"{r['cross_camera_rotation_deg']:>14.4f} {r['n_camera_pairs']:>8d}")

    Path(args.out).write_text(json.dumps({"results": results}, indent=2))
    print(f"\nwrote {args.out}")


if __name__ == "__main__":
    main()
