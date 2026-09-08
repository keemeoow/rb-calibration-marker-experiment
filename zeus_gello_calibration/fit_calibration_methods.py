#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""zeus_gello_calibration/fit_calibration_methods.py -- 통합/독립 x raw-fk/no-fk
4가지 방식을 전부 돌려서 비교한다 (Simulation/core/methods.py의 solve_unified/
solve_independent 개념을 실측 Zeus 데이터에 이식).

  통합(unified)      -- session1(그리퍼로 쥔 큐브, 고정캠3)+session2(바닥 큐브,
                        고정캠3)+session3(바닥 보드, 그리퍼캠1)을 전부 하나의
                        pixel reprojection 최소제곱에 넣어 T_base_Ci(3) +
                        T_gripper_cam + T_base_board + T_gripper_cube_by_grasp
                        (+session2 조건별)를 동시에 푼다. (fit_full_calibration.py
                        그대로 재사용)
  독립(independent)  -- 고정캠 서브시스템과 그리퍼캠(eye-in-hand) 서브시스템을
                        *따로* 푼다(서로의 잔차에 관여하지 않음). 원본
                        Simulation 코드의 solve_independent와 같은 아이디어지만,
                        원본은 그리퍼가 큐브도 볼 수 있어 그걸로 base-frame
                        gauge를 잡는 반면, 우리 실측 그리퍼캠은 큐브를 전혀
                        못 본다(0/16, 0/15 검출) -- 그래서:
                          raw-fk : 고정캠 각각을 session1에서 구한 T_gripper_cube
                                   로 만든 FK 앵커(그 placement에서 실제 명령한
                                   place pose @ T_gripper_cube)에 개별 역산해서
                                   고정. 세션1 자체는 (원본 코드와 동일하게)
                                   독립 방식에선 쓰지 않는다.
                          no-fk  : FK 앵커 없이, 고정캠 3대끼리의 교차 검증만으로
                                   (카메라 0을 내부 게이지 원점으로 삼아) 상대
                                   기하를 복원 -- 로봇 base 절대좌표가 아니라
                                   "카메라 0 기준 상대 gauge"에서의 self-
                                   consistency다(실측 데이터에 그리퍼-큐브
                                   앵커가 없어서 절대 gauge를 없이 만들 방법이
                                   없다 -- 이 caveat을 결과에 명시한다).
                        그리퍼캠(T_gripper_cam/T_base_board)은 raw-fk/no-fk
                        조건과 무관하게 항상 session3만으로 독립적으로 구한다
                        (estimate_board_handeye_initial, 큐브 FK 개념이
                        아예 안 들어가는 문제라 두 조건에서 동일).

*** 통합은 session1+2+3을 전부 pool하고, 독립은 session2+3만 쓴다(session1은
원본 방법론 자체가 grasp-FK 앵커 없이는 다룰 수단이 없음) -- 그래서 RMSE 숫자를
그대로 1:1 비교하면 안 되고, "각 방법론이 자기 방식대로 최선을 다했을 때 남는
잔차가 어느 정도인가"로 읽어야 한다. table1.py 정식 held-out 지표 아님.

사용법:
  python fit_calibration_methods.py
"""

from __future__ import annotations

import argparse
import dataclasses
import json
import sys
from pathlib import Path

import cv2
import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT))

from calibration_pipeline import se3 as cp  # noqa: E402
from calibration_pipeline.apriltag_cube import AprilTagCubeTarget, inv_T  # noqa: E402
from calibration_pipeline.board_config import charuco_config_from_dict  # noqa: E402
from calibration_pipeline.charuco import CharucoTarget  # noqa: E402
from calibration_pipeline.config import get_default_cube_config  # noqa: E402
from calibration_pipeline.observations import (  # noqa: E402
    load_board_pixel_observations, load_cube_pixel_observations,
)
from calibration_pipeline.path_evaluation import solve_observed_pose  # noqa: E402
from calibration_pipeline.reprojection import (  # noqa: E402
    PoseState, SolverOptions, project_points, solve_corner_reprojection, variable_keys,
)
from calibration_pipeline.table1 import estimate_board_handeye_initial  # noqa: E402
from robot.backends.zeus_client import pose6_to_T  # noqa: E402

from fit_grasp_offset import LOCAL_CAM_IDS, build_synthetic_meta, load_intrinsics_by_label, load_robot_T  # noqa: E402
from fit_placement_fk_ablation import (  # noqa: E402
    SESSION2_EVENT_OFFSET, build_synthetic_meta_placed, init_cube_poses_from_images,
)
from fit_full_calibration import (  # noqa: E402
    CHARUCO_BOARD_CONFIG, GRIPPER_LOCAL_ID, SESSION1_DIR_DEFAULT, SESSION3_DIR_DEFAULT,
    SESSION3_EVENT_OFFSET, build_synthetic_meta_board,
)
from session2_pick_and_place import SESSION2_DIR_DEFAULT, compute_ordered_targets  # noqa: E402

FIT_JSON_DEFAULT = REPO_ROOT / "zeus_gello_calibration" / "pass1_grasp_offset_replayed.json"


def rmse_px(errs):
    errs = np.asarray(errs, dtype=np.float64)
    return float(np.sqrt(np.mean(errs ** 2))) if errs.size else float("nan")


# ------------------------------------------------------------- 독립(independent)
def solve_independent_fixed_raw_fk(obs_s2_raw, K_map, D_map, cam_ids, items_by_index, T_gripper_cube):
    """카메라별로 FK 앵커(그 placement에서 실제 명령한 pose @ T_gripper_cube)에
    개별 역산 -- session1은 안 쓴다(원본 독립 방식과 동일)."""
    fk_anchor = {idx: pose6_to_T(item["target"]) @ T_gripper_cube for idx, item in items_by_index.items()}
    cams, errs_by_cam = {}, {}
    all_errs = []
    for c in cam_ids:
        cands = []
        for o in obs_s2_raw:
            if int(o.cam) != c or o.set_idx is None or int(o.set_idx) not in fk_anchor:
                continue
            T_cam_cube = solve_observed_pose(o, K_map, D_map)
            if T_cam_cube is None:
                continue
            cands.append(fk_anchor[int(o.set_idx)] @ inv_T(T_cam_cube))
        if not cands:
            continue
        cams[c] = cands[0] if len(cands) == 1 else cp.robust_se3_average(cands, None)[0]
    errs = []
    for o in obs_s2_raw:
        c = int(o.cam)
        if c not in cams or o.set_idx is None or int(o.set_idx) not in fk_anchor:
            continue
        pred = project_points(inv_T(cams[c]) @ fk_anchor[int(o.set_idx)], o.object_points, K_map[c], D_map[c])
        e = np.linalg.norm(pred - o.image_points, axis=1)
        errs.extend(e.tolist())
    return cams, fk_anchor, rmse_px(errs), len(errs) // 2


def solve_independent_fixed_no_fk(obs_s2_raw, K_map, D_map, cam_ids, n_iters=3):
    """FK 앵커 없이 고정캠 3대끼리의 교차검증만으로 상대 기하 복원.
    카메라 min(cam_ids)를 내부 게이지 원점(항등)으로 고정 -- 로봇 base 절대좌표가
    아니라 이 카메라 기준 상대 gauge다 (그리퍼가 큐브를 못 봐서 절대 gauge를
    잡을 FK-프리 수단이 실측 데이터엔 없음)."""
    ref_cam = min(cam_ids)
    cam_cube = {}
    for o in obs_s2_raw:
        if o.set_idx is None:
            continue
        T = solve_observed_pose(o, K_map, D_map)
        if T is not None:
            cam_cube[(int(o.cam), int(o.set_idx))] = T
    cams = {ref_cam: np.eye(4)}
    for _ in range(n_iters):
        target = {}
        all_sets = {s for (_, s) in cam_cube}
        for s in all_sets:
            Ts = [cams[c] @ cam_cube[(c, s)] for c in cams if (c, s) in cam_cube]
            if Ts:
                target[s] = Ts[0] if len(Ts) == 1 else cp.robust_se3_average(Ts, None)[0]
        new_cams = {ref_cam: np.eye(4)}
        for c in cam_ids:
            if c == ref_cam:
                continue
            Ts = [target[s] @ inv_T(cam_cube[(c, s)]) for s in target if (c, s) in cam_cube]
            if Ts:
                new_cams[c] = Ts[0] if len(Ts) == 1 else cp.robust_se3_average(Ts, None)[0]
        cams = new_cams
    # final target consensus + RMSE
    target = {}
    for s in {s for (_, s) in cam_cube}:
        Ts = [cams[c] @ cam_cube[(c, s)] for c in cams if (c, s) in cam_cube]
        if Ts:
            target[s] = Ts[0] if len(Ts) == 1 else cp.robust_se3_average(Ts, None)[0]
    errs = []
    for o in obs_s2_raw:
        c, s = int(o.cam), o.set_idx
        if c not in cams or s is None or int(s) not in target:
            continue
        pred = project_points(inv_T(cams[c]) @ target[int(s)], o.object_points, K_map[c], D_map[c])
        e = np.linalg.norm(pred - o.image_points, axis=1)
        errs.extend(e.tolist())
    return cams, target, rmse_px(errs), len(errs) // 2, ref_cam


def solve_gripper_eih(obs_s3, robot_T_s3, K_map, D_map):
    gtc, board, diag = estimate_board_handeye_initial(obs_s3, robot_T_s3, K_map, D_map, GRIPPER_LOCAL_ID)
    errs = []
    for o in obs_s3:
        T_base_cam = robot_T_s3[int(o.event)] @ gtc
        pred = project_points(inv_T(T_base_cam) @ board, o.object_points, K_map[GRIPPER_LOCAL_ID], D_map[GRIPPER_LOCAL_ID])
        e = np.linalg.norm(pred - o.image_points, axis=1)
        errs.extend(e.tolist())
    return gtc, board, rmse_px(errs), len(errs) // 2, diag


# ------------------------------------------------------------- 통합(unified) 재사용
def solve_unified(obs_s1, obs_s2_raw, obs_s3, robot_T_combined, cam_init, grasp_init,
                  gtc_init, board_init, K_map, D_map, items_by_index, fk_mode):
    set_ids = sorted(items_by_index)
    options = SolverOptions()
    if fk_mode == "no_fk":
        cube_init, _ = init_cube_poses_from_images(obs_s2_raw, K_map, D_map, cam_init, set_ids)
        obs = obs_s1 + [o for o in obs_s2_raw if o.set_idx is not None and int(o.set_idx) in cube_init] + obs_s3
        state = PoseState(cams=dict(cam_init), gtc=gtc_init.copy(), board=board_init.copy(),
                          cubes=dict(cube_init), grasps={0: grasp_init.copy()})
        keys = variable_keys(
            ["T_base_Ci", "T_gripper_cam", "T_base_board", "T_base_cube_by_set", "T_gripper_cube_by_grasp"], state)
    else:
        obs_s2_grasp = [dataclasses.replace(o, set_idx=None, grasp_idx=0)
                        for o in obs_s2_raw if int(o.event) - SESSION2_EVENT_OFFSET in items_by_index]
        obs = obs_s1 + obs_s2_grasp + obs_s3
        state = PoseState(cams=dict(cam_init), gtc=gtc_init.copy(), board=board_init.copy(),
                          cubes={}, grasps={0: grasp_init.copy()})
        keys = variable_keys(["T_base_Ci", "T_gripper_cam", "T_base_board", "T_gripper_cube_by_grasp"], state)
    final_state, diag = solve_corner_reprojection(
        observations=obs, variable_keys_=keys, reference_state=state,
        robot_T=robot_T_combined, K_map=K_map, D_map=D_map,
        gripper_cam_idx=GRIPPER_LOCAL_ID, options=options,
    )
    return final_state, diag


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
    ap.add_argument("--fit-json", default=str(FIT_JSON_DEFAULT))
    ap.add_argument("--fixed-min-corners", type=int, default=8)
    ap.add_argument("--cube-observation-policy", default="legacy", choices=("legacy", "core_multiface"))
    ap.add_argument("--out", default=str(REPO_ROOT / "zeus_gello_calibration" / "calibration_methods_comparison.json"))
    args = ap.parse_args()

    session1_dir, session2_dir, session3_dir = Path(args.session1_dir), Path(args.session2_dir), Path(args.session3_dir)
    s1_root = session1_dir / args.session1_capture_subdir
    s2_root = session2_dir / args.session2_capture_subdir
    s3_root = session3_dir / args.session3_capture_subdir
    s1_idx = sorted(int(p.name) for p in s1_root.iterdir() if p.is_dir())
    s2_idx = sorted(int(p.name) for p in s2_root.iterdir() if p.is_dir())
    s3_idx = sorted(int(p.name) for p in s3_root.iterdir() if p.is_dir())
    items = compute_ordered_targets(session2_dir)
    items_by_index = {idx: items[idx] for idx in s2_idx if idx < len(items)}

    K_map, D_map = load_intrinsics_by_label(
        Path(args.zeus_intrinsics_dir), Path(args.ur3_intrinsics_dir), Path(args.device_map))
    fit = json.loads(Path(args.fit_json).read_text())
    grasp_init = np.asarray(fit["T_gripper_cube"], dtype=np.float64)
    cam_init = {int(k.split("_", 1)[0]): np.asarray(v, dtype=np.float64) for k, v in fit["T_base_cam"].items()}
    fixed_ids = sorted(cam_init)

    cube_cfg = get_default_cube_config()
    cube = AprilTagCubeTarget(cube_cfg)

    meta_s1 = build_synthetic_meta(session1_dir, s1_idx, args.session1_capture_subdir)
    robot_T_s1 = load_robot_T(session1_dir, s1_idx, args.session1_capture_subdir)
    obs_s1, _ = load_cube_pixel_observations(
        str(session1_dir), meta_s1, cube, K_map, D_map, fixed_ids, gripper_cam_idx=-999,
        exclude_gripped=False, fixed_min_corners=args.fixed_min_corners, image_scale=1.0,
        observation_policy=args.cube_observation_policy)
    obs_s1 = [o for o in obs_s1 if int(o.cam) in cam_init]

    meta_s2 = build_synthetic_meta_placed(s2_root, s2_idx)
    obs_s2_raw, _ = load_cube_pixel_observations(
        str(session2_dir), meta_s2, cube, K_map, D_map, fixed_ids, gripper_cam_idx=-999,
        exclude_gripped=False, fixed_min_corners=args.fixed_min_corners, image_scale=1.0,
        observation_policy=args.cube_observation_policy)
    obs_s2_raw = [o for o in obs_s2_raw if int(o.cam) in cam_init]

    charuco_target = CharucoTarget(charuco_config_from_dict(CHARUCO_BOARD_CONFIG))
    meta_s3 = build_synthetic_meta_board(session3_dir, args.session3_capture_subdir, s3_idx, charuco_target)
    robot_T_s3 = {SESSION3_EVENT_OFFSET + k: v
                  for k, v in load_robot_T(session3_dir, s3_idx, args.session3_capture_subdir).items()}
    obs_s3 = load_board_pixel_observations(
        str(session3_dir), meta_s3, [GRIPPER_LOCAL_ID], gripper_cam_idx=GRIPPER_LOCAL_ID, image_scale=1.0)

    robot_T_s2_commanded = {SESSION2_EVENT_OFFSET + idx: pose6_to_T(item["target"])
                            for idx, item in items_by_index.items()}
    robot_T_combined = {**robot_T_s1, **robot_T_s2_commanded, **robot_T_s3}

    print(f"session1 {len(obs_s1)}개 / session2 {len(obs_s2_raw)}개 / session3 {len(obs_s3)}개 관측치\n")

    results = {}

    # 그리퍼캠 eye-in-hand: raw-fk/no-fk 무관, 통합/독립 공통 초기값으로 재사용
    gtc_init, board_init, gtc_rmse, gtc_n, eih_diag = solve_gripper_eih(obs_s3, robot_T_s3, K_map, D_map)
    print(f"[그리퍼캠 eye-in-hand] rmse_px={gtc_rmse:.4f} n_corners={gtc_n} (raw-fk/no-fk 공통, {eih_diag})\n")

    # --- 통합 raw-fk / no-fk ---
    for fk_mode in ("fixed_fk", "no_fk"):
        state, diag = solve_unified(obs_s1, obs_s2_raw, obs_s3, robot_T_combined, cam_init, grasp_init,
                                    gtc_init, board_init, K_map, D_map, items_by_index, fk_mode)
        label = "통합_raw-fk" if fk_mode == "fixed_fk" else "통합_no-fk"
        results[label] = {
            "success": diag["success"],
            "rmse_px": diag["train_reprojection_rmse_px"],
            "n_corners": diag["n_residuals"] // 2,
            "note": "session1+session2+session3 전체 pool",
        }

    # --- 독립 raw-fk ---
    cams_rawfk, fk_anchor, rmse_rawfk, n_rawfk = solve_independent_fixed_raw_fk(
        obs_s2_raw, K_map, D_map, fixed_ids, items_by_index, grasp_init)
    results["독립_raw-fk"] = {
        "success": len(cams_rawfk) == len(fixed_ids),
        "rmse_px_fixed_cams": rmse_rawfk, "n_corners_fixed_cams": n_rawfk,
        "rmse_px_gripper": gtc_rmse, "n_corners_gripper": gtc_n,
        "note": "고정캠: session2만, FK 앵커(session1 T_gripper_cube) 사용, session1 자체는 미사용. "
                "그리퍼캠: 위와 공통.",
    }

    # --- 독립 no-fk ---
    cams_nofk, target_nofk, rmse_nofk, n_nofk, ref_cam = solve_independent_fixed_no_fk(
        obs_s2_raw, K_map, D_map, fixed_ids)
    results["독립_no-fk"] = {
        "success": len(cams_nofk) == len(fixed_ids),
        "rmse_px_fixed_cams": rmse_nofk, "n_corners_fixed_cams": n_nofk,
        "rmse_px_gripper": gtc_rmse, "n_corners_gripper": gtc_n,
        "note": f"고정캠: session2만, FK 미사용, 카메라{ref_cam}을 내부 게이지 원점으로 삼은 "
                "상대 gauge에서의 self-consistency (로봇 base 절대좌표 아님). 그리퍼캠: 위와 공통.",
    }

    print(f"{'condition':>16} {'rmse_px':>12} {'n_corners':>10}   note")
    for name in ("통합_raw-fk", "통합_no-fk", "독립_raw-fk", "독립_no-fk"):
        r = results[name]
        if "rmse_px" in r:
            print(f"{name:>16} {r['rmse_px']:>12.4f} {r['n_corners']:>10d}   {r['note']}")
        else:
            print(f"{name:>16} {'fixed:'+format(r['rmse_px_fixed_cams'],'.4f'):>12} "
                  f"{r['n_corners_fixed_cams']:>10d}   {r['note']}")
            print(f"{'':>16} {'grip:'+format(r['rmse_px_gripper'],'.4f'):>12} {r['n_corners_gripper']:>10d}")

    out = {
        "warning": ("통합은 session1+2+3 pool, 독립은 session2(+session3 그리퍼캠 별도)만 사용 -- "
                    "직접 1:1 비교가 아니라 각 방법론이 자기 방식으로 최선을 다한 잔차 비교. "
                    "독립_no-fk의 고정캠 RMSE는 로봇 base 절대좌표가 아닌 카메라 내부 상대 gauge 기준."),
        "results": results,
    }
    Path(args.out).write_text(json.dumps(out, indent=2))
    print(f"\nwrote {args.out}")


if __name__ == "__main__":
    main()
