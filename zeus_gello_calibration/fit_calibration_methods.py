#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""zeus_gello_calibration/fit_calibration_methods.py -- 통합/독립 x raw-fk/no-fk
4가지 방식을 "같은 데이터, 다른 solve 전략"으로 비교한다.

*** 핵심 원칙: 통합이든 독립이든 캘리브레이션에 쓰는 데이터의 총량은 완전히
같아야 한다. 차이는 그 데이터를 하나로 묶어서 푸느냐(통합) vs 두 그룹으로
나눠서 따로 푸느냐(독립)뿐이다. ***

데이터 풀 (통합/독립 공통, 총 4가지 소스):
  session1        -- 고정캠 3대가 그리퍼로 쥔 큐브를 봄 (grasp+FK 모델)
  session2-고정캠  -- 고정캠 3대가 바닥에 놓인 큐브를 봄 (세트별)
  session2-그리퍼캠 -- ★그리퍼캠도 바닥에 놓인 큐브를 본다★ (capture_placed의
                      새 파킹 위치에서 실측 확인: PnP 15/15 성공, err<1px).
                      이전 버전은 이걸 빠뜨렸다.
  session3-그리퍼캠 -- 그리퍼캠이 바닥 마커보드를 봄 (eye-in-hand)

  no_fk    : session2(고정캠+그리퍼캠 둘 다)의 큐브 pose를 세트별 자유 변수로.
  raw-fk   : session2의 큐브 pose를 "그 placement에서 실제 명령한 place pose
             @ T_gripper_cube" FK값으로 고정(정적 상수, 최적화 중 안 바뀜).

  통합(unified)     -- 위 4개 소스를 전부 하나의 최소제곱에 넣어 한 번에 푼다.
  독립(independent) -- 두 그룹으로 쪼개서 각자 최소제곱을 돌리고 서로의
                       residual에 관여하지 않는다:
                         그룹A(고정캠): session1 + session2-고정캠
                         그룹B(그리퍼캠): session2-그리퍼캠 + session3-그리퍼캠
                       raw-fk에서 그룹B가 필요한 T_gripper_cube는 그룹A를 먼저
                       풀어서 나온 값을 (다시 최적화하지 않고) 상수로 넘겨받는다
                       -- 이건 "독립적으로 각자 풀고, 이미 풀린 값을 순차적으로
                       재사용"이라 여전히 독립이다(residual을 동시에 최적화하지
                       않으므로).

table1.py의 정식 held-out 지표 아님 (train-pooled). no_fk/raw-fk x 통합/독립
네 조건끼리 비교하는 용도.

사용법:
  python fit_calibration_methods.py
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import cv2
import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT))

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

from calibration_pipeline import se3 as cp  # noqa: E402
from fit_grasp_offset import LOCAL_CAM_IDS, build_synthetic_meta, load_intrinsics_by_label, load_robot_T  # noqa: E402
from fit_placement_fk_ablation import SESSION2_EVENT_OFFSET, build_synthetic_meta_placed  # noqa: E402
from fit_full_calibration import CHARUCO_BOARD_CONFIG, GRIPPER_LOCAL_ID, build_synthetic_meta_board  # noqa: E402
from session2_pick_and_place import SESSION2_DIR_DEFAULT, compute_ordered_targets  # noqa: E402

SESSION1_DIR_DEFAULT = REPO_ROOT / "zeus_gello_calibration" / "data" / "session1_handheld_fixed_cam"
SESSION3_DIR_DEFAULT = REPO_ROOT / "zeus_gello_calibration" / "data" / "session3_wrist_motion_gripper_cam"
FIT_JSON_DEFAULT = REPO_ROOT / "zeus_gello_calibration" / "pass1_grasp_offset_replayed.json"

# 이벤트 id 네임스페이스 충돌 방지 (robot_T 딕셔너리 키). session2는
# build_synthetic_meta_placed가 이미 event_id에 SESSION2_EVENT_OFFSET(1000)을
# 박아서 만들기 때문에(고정캠/그리퍼캠 공용 meta), 그리퍼캠 쪽도 그대로
# 재사용한다 -- 별도 offset을 또 더하면 이중 offset 버그가 난다.
SESSION3_EVENT_OFFSET = 2000


def rmse_px(errs):
    errs = np.asarray(errs, dtype=np.float64)
    return float(np.sqrt(np.mean(errs ** 2))) if errs.size else float("nan")


def fk_anchor_cubes(items_by_index, T_gripper_cube):
    return {idx: pose6_to_T(item["target"]) @ T_gripper_cube for idx, item in items_by_index.items()}


def init_cube_poses(obs_list, K_map, D_map, cam_init, gtc_init, robot_T, gripper_id, set_ids):
    """세트별 T_base_cube 초기값 -- 고정캠은 cam_init[c]@T_cam_cube, 그리퍼캠은
    robot_T[event]@gtc_init@T_cam_cube (그 사진 찍은 순간의 실제 로봇 pose 사용)."""
    by_set = {s: [] for s in set_ids}
    for o in obs_list:
        if o.set_idx is None or int(o.set_idx) not in by_set:
            continue
        T_cam_cube = solve_observed_pose(o, K_map, D_map)
        if T_cam_cube is None:
            continue
        c = int(o.cam)
        if c == gripper_id:
            if int(o.event) not in robot_T:
                continue
            cand = robot_T[int(o.event)] @ gtc_init @ T_cam_cube
        else:
            if c not in cam_init:
                continue
            cand = cam_init[c] @ T_cam_cube
        by_set[int(o.set_idx)].append(cand)
    init = {}
    for s, cands in by_set.items():
        if not cands:
            continue
        init[s] = cands[0] if len(cands) == 1 else cp.robust_se3_average(cands, None)[0]
    return init


def load_all_data(args):
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
    all_cam_ids = sorted(set(fixed_ids) | {GRIPPER_LOCAL_ID})

    cube = AprilTagCubeTarget(get_default_cube_config())

    # session1 (고정캠, grasp+FK) -- 그리퍼캠은 여기선 항상 0검출이라 굳이 안 실음
    meta_s1 = build_synthetic_meta(session1_dir, s1_idx, args.session1_capture_subdir)
    robot_T_s1 = load_robot_T(session1_dir, s1_idx, args.session1_capture_subdir)
    obs_s1, _ = load_cube_pixel_observations(
        str(session1_dir), meta_s1, cube, K_map, D_map, fixed_ids, gripper_cam_idx=-999,
        exclude_gripped=False, fixed_min_corners=args.fixed_min_corners, image_scale=1.0,
        observation_policy=args.cube_observation_policy)
    obs_s1 = [o for o in obs_s1 if int(o.cam) in cam_init]

    # session2 (고정캠 3대 + 그리퍼캠 1대, 전부 큐브-세트) -- 하나의 meta로 같이 로드
    meta_s2 = build_synthetic_meta_placed(s2_root, s2_idx)
    obs_s2_all, _ = load_cube_pixel_observations(
        str(session2_dir), meta_s2, cube, K_map, D_map, all_cam_ids, gripper_cam_idx=-999,
        exclude_gripped=False, fixed_min_corners=args.fixed_min_corners, image_scale=1.0,
        observation_policy=args.cube_observation_policy)
    obs_s2_fixed = [o for o in obs_s2_all if int(o.cam) in cam_init]
    obs_s2_gripper_raw = [o for o in obs_s2_all if int(o.cam) == GRIPPER_LOCAL_ID]
    # 그리퍼캠 관측치의 event id를 그 사진을 실제로 찍은 순간(파킹 pose)의 로봇
    # pose와 연결 -- 큐브를 놓은 순간이 아니라 촬영한 순간의 FK가 카메라 pose다.
    robot_T_s2_photo = load_robot_T(session2_dir, s2_idx, args.session2_capture_subdir)
    obs_s2_gripper = obs_s2_gripper_raw  # event id는 이미 build_synthetic_meta_placed가 offset해서 나옴
    robot_T_s2_gripper = {SESSION2_EVENT_OFFSET + k: v for k, v in robot_T_s2_photo.items()}
    print(f"session2 그리퍼캠 큐브 관측치: {len(obs_s2_gripper)}개 (신규 -- 예전엔 빠뜨렸음)")

    # session3 (그리퍼캠, 보드)
    charuco_target = CharucoTarget(charuco_config_from_dict(CHARUCO_BOARD_CONFIG))
    meta_s3 = build_synthetic_meta_board(session3_dir, args.session3_capture_subdir, s3_idx, charuco_target)
    robot_T_s3 = {SESSION3_EVENT_OFFSET + k: v
                  for k, v in load_robot_T(session3_dir, s3_idx, args.session3_capture_subdir).items()}
    obs_s3 = load_board_pixel_observations(
        str(session3_dir), meta_s3, [GRIPPER_LOCAL_ID], gripper_cam_idx=GRIPPER_LOCAL_ID, image_scale=1.0)

    print(f"session1 {len(obs_s1)}개 / session2-고정캠 {len(obs_s2_fixed)}개 / "
          f"session2-그리퍼캠 {len(obs_s2_gripper)}개 / session3-그리퍼캠 {len(obs_s3)}개")
    print(f"총 관측치: {len(obs_s1)+len(obs_s2_fixed)+len(obs_s2_gripper)+len(obs_s3)}개 "
          f"(통합/독립 공통, 같은 양)\n")

    return dict(
        cam_init=cam_init, grasp_init=grasp_init, K_map=K_map, D_map=D_map,
        obs_s1=obs_s1, robot_T_s1=robot_T_s1,
        obs_s2_fixed=obs_s2_fixed, obs_s2_gripper=obs_s2_gripper, robot_T_s2_gripper=robot_T_s2_gripper,
        obs_s3=obs_s3, robot_T_s3=robot_T_s3,
        items_by_index=items_by_index,
    )


# ------------------------------------------------------------------ 통합(unified)
def solve_unified(data, fk_mode, gtc_init, board_init):
    cam_init, grasp_init = data["cam_init"], data["grasp_init"]
    K_map, D_map = data["K_map"], data["D_map"]
    obs_s2 = data["obs_s2_fixed"] + data["obs_s2_gripper"]
    set_ids = sorted(data["items_by_index"])
    robot_T = {**data["robot_T_s1"], **data["robot_T_s2_gripper"], **data["robot_T_s3"]}
    observations = data["obs_s1"] + obs_s2 + data["obs_s3"]

    if fk_mode == "no_fk":
        cubes = init_cube_poses(obs_s2, K_map, D_map, cam_init, gtc_init, robot_T, GRIPPER_LOCAL_ID, set_ids)
        cube_key = ["T_base_cube_by_set"]
    else:
        cubes = fk_anchor_cubes(data["items_by_index"], grasp_init)
        cube_key = []
    observations = [o for o in observations
                    if o.set_idx is None or int(o.set_idx) in cubes or o.grasp_idx is not None]

    state = PoseState(cams=dict(cam_init), gtc=gtc_init.copy(), board=board_init.copy(),
                      cubes=dict(cubes), grasps={0: grasp_init.copy()})
    keys = variable_keys(["T_base_Ci", "T_gripper_cam", "T_base_board"] + cube_key + ["T_gripper_cube_by_grasp"], state)
    final_state, diag = solve_corner_reprojection(
        observations=observations, variable_keys_=keys, reference_state=state,
        robot_T=robot_T, K_map=K_map, D_map=D_map,
        gripper_cam_idx=GRIPPER_LOCAL_ID, options=SolverOptions(),
    )
    return final_state, diag, len(observations)


# --------------------------------------------------------------- 독립(independent)
def solve_independent_group_a(data, fk_mode):
    """그룹A: 고정캠 3대, session1(grasp+FK) + session2-고정캠만."""
    cam_init, grasp_init = data["cam_init"], data["grasp_init"]
    K_map, D_map = data["K_map"], data["D_map"]
    obs_s2 = data["obs_s2_fixed"]
    set_ids = sorted(data["items_by_index"])

    if fk_mode == "no_fk":
        cubes = init_cube_poses(obs_s2, K_map, D_map, cam_init, np.eye(4), {}, -999, set_ids)
        cube_key = ["T_base_cube_by_set"]
        observations = data["obs_s1"] + [o for o in obs_s2 if o.set_idx is not None and int(o.set_idx) in cubes]
    else:
        cubes = fk_anchor_cubes(data["items_by_index"], grasp_init)
        cube_key = []
        observations = data["obs_s1"] + obs_s2

    state = PoseState(cams=dict(cam_init), gtc=np.eye(4), board=None,
                      cubes=dict(cubes), grasps={0: grasp_init.copy()})
    keys = variable_keys(["T_base_Ci"] + cube_key + ["T_gripper_cube_by_grasp"], state)
    final_state, diag = solve_corner_reprojection(
        observations=observations, variable_keys_=keys, reference_state=state,
        robot_T=data["robot_T_s1"], K_map=K_map, D_map=D_map,
        gripper_cam_idx=-999, options=SolverOptions(),
    )
    return final_state, diag, len(observations)


def solve_independent_group_b(data, fk_mode, gtc_init, board_init, T_gripper_cube_from_a):
    """그룹B: 그리퍼캠 1대, session2-그리퍼캠 + session3만. raw-fk는 그룹A에서
    이미 풀린 T_gripper_cube를 상수로 재사용(동시 최적화 아님 -- 여전히 독립)."""
    K_map, D_map = data["K_map"], data["D_map"]
    obs_s2g = data["obs_s2_gripper"]
    set_ids = sorted(data["items_by_index"])
    robot_T = {**data["robot_T_s2_gripper"], **data["robot_T_s3"]}
    observations = obs_s2g + data["obs_s3"]

    if fk_mode == "no_fk":
        cubes = init_cube_poses(obs_s2g, K_map, D_map, {}, gtc_init, robot_T, GRIPPER_LOCAL_ID, set_ids)
        cube_key = ["T_base_cube_by_set"]
        observations = [o for o in observations
                        if o.set_idx is None or int(o.set_idx) in cubes]
    else:
        cubes = fk_anchor_cubes(data["items_by_index"], T_gripper_cube_from_a)
        cube_key = []

    state = PoseState(cams={}, gtc=gtc_init.copy(), board=board_init.copy(), cubes=dict(cubes), grasps={})
    keys = variable_keys(["T_gripper_cam", "T_base_board"] + cube_key, state)
    final_state, diag = solve_corner_reprojection(
        observations=observations, variable_keys_=keys, reference_state=state,
        robot_T=robot_T, K_map=K_map, D_map=D_map,
        gripper_cam_idx=GRIPPER_LOCAL_ID, options=SolverOptions(),
    )
    return final_state, diag, len(observations)


def per_corner_errors(state, observations, robot_T, K_map, D_map, gripper_id):
    errs = []
    for o in observations:
        if o.marker == "board":
            target = state.board
        elif o.grasp_idx is not None:
            target = robot_T[int(o.event)] @ state.grasps[int(o.grasp_idx)]
        else:
            target = state.cubes[int(o.set_idx)]
        if int(o.cam) == gripper_id:
            T_base_cam = robot_T[int(o.event)] @ state.gtc
        else:
            T_base_cam = state.cams[int(o.cam)]
        pred = project_points(inv_T(T_base_cam) @ target, o.object_points, K_map[int(o.cam)], D_map[int(o.cam)])
        errs.extend(np.linalg.norm(pred - o.image_points, axis=1).tolist())
    return errs


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

    data = load_all_data(args)
    K_map, D_map = data["K_map"], data["D_map"]

    gtc_init, board_init, eih_diag = estimate_board_handeye_initial(
        data["obs_s3"], data["robot_T_s3"], K_map, D_map, GRIPPER_LOCAL_ID)
    print(f"T_gripper_cam 초기값 (session3만): t_mm={np.round(gtc_init[:3,3]*1000,2).tolist()} ({eih_diag})\n")

    results = {}
    for fk_mode, label in (("no_fk", "통합_no-fk"), ("fixed_fk", "통합_raw-fk")):
        state, diag, n_obs = solve_unified(data, fk_mode, gtc_init, board_init)
        results[label] = {"success": diag["success"], "rmse_px": diag["train_reprojection_rmse_px"],
                          "n_corners": diag["n_residuals"] // 2, "n_observations": n_obs}

    for fk_mode, label in (("no_fk", "독립_no-fk"), ("fixed_fk", "독립_raw-fk")):
        state_a, diag_a, n_a = solve_independent_group_a(data, fk_mode)
        T_gripper_cube_a = state_a.grasps[0]
        state_b, diag_b, n_b = solve_independent_group_b(data, fk_mode, gtc_init, board_init, T_gripper_cube_a)
        errs_a = per_corner_errors(state_a, (data["obs_s1"] +
                                             [o for o in data["obs_s2_fixed"]
                                              if o.set_idx is None or int(o.set_idx) in state_a.cubes]),
                                   data["robot_T_s1"], K_map, D_map, -999)
        robot_T_b = {**data["robot_T_s2_gripper"], **data["robot_T_s3"]}
        errs_b = per_corner_errors(state_b, ([o for o in data["obs_s2_gripper"]
                                              if o.set_idx is None or int(o.set_idx) in state_b.cubes]
                                             + data["obs_s3"]),
                                   robot_T_b, K_map, D_map, GRIPPER_LOCAL_ID)
        combined_rmse = rmse_px(errs_a + errs_b)
        results[label] = {
            "success": bool(diag_a["success"] and diag_b["success"]),
            "rmse_px": combined_rmse,
            "n_corners": (len(errs_a) + len(errs_b)) // 2,
            "n_observations": n_a + n_b,
            "group_a_fixed_cams_rmse_px": rmse_px(errs_a),
            "group_b_gripper_rmse_px": rmse_px(errs_b),
        }

    total_n = len(data["obs_s1"]) + len(data["obs_s2_fixed"]) + len(data["obs_s2_gripper"]) + len(data["obs_s3"])
    print(f"{'condition':>14} {'rmse_px':>10} {'n_corners':>10} {'n_obs':>7}   detail")
    for name in ("통합_raw-fk", "통합_no-fk", "독립_raw-fk", "독립_no-fk"):
        r = results[name]
        detail = ""
        if "group_a_fixed_cams_rmse_px" in r:
            detail = f"고정캠 {r['group_a_fixed_cams_rmse_px']:.4f} / 그리퍼 {r['group_b_gripper_rmse_px']:.4f}"
        print(f"{name:>14} {r['rmse_px']:>10.4f} {r['n_corners']:>10d} {r['n_observations']:>7d}   {detail}")
    print(f"\n(참고: 데이터 풀 총 observation 수 = {total_n}, 통합/독립 공통)")

    out = {
        "warning": "train-pooled, held-out 분리 없음 -- table1.py 정식 지표 아님. "
                   "통합/독립 모두 session1+session2(고정+그리퍼)+session3 동일 데이터 사용.",
        "total_observations": total_n,
        "results": results,
    }
    Path(args.out).write_text(json.dumps(out, indent=2, default=lambda o: str(o)))
    print(f"\nwrote {args.out}")


if __name__ == "__main__":
    main()
