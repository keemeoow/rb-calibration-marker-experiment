#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""zeus_gello_calibration/table1_zeus.py -- Zeus 데이터로 Table 1을 계산한다.

핵심 차이: FK를 offset으로 보정하지 않는다
--------------------------------------------
CP_result/session04(late_table1)의 A4/A5는 큐브 자세를 이렇게 만든다.

    A4/A5:  T^B_cube(s) = T^B_fk_raw(s) · Delta      (Delta = 영상으로 적합한 offset)

Zeus 데이터에서는 이 경로를 쓰지 않는다. 큐브가 그리퍼에 물려 있으므로 FK는
**flange-to-cube 그 자체**이고, 보정할 offset이 없다.

    이 파일:  T^B_cube(s) = T^B_flange(place_s) · T^flange_cube

여기서 ``T^flange_cube``는 pass1_grasp_offset_replayed.json이 session1에서 구한
상수이며 (translation ≈ (0.15, 0.74, 161.79) mm), 모든 row가 **같은 값**을 쓴다.
어떤 row도 이 값을 event/placement별로 재보정하지 않는다.  이 아티팩트는
session*/capture_replayed 촬영본과 짝이다 -- capture 원본에는
pass1_grasp_offset.json 을 써야 하며 둘을 섞으면 안 된다.

수식이 달라지는 지점
--------------------
1. 큐브 자세의 출처가 "raw FK pose + 기계 좌표변환 + 적합된 Delta"에서
   "flange 자세 · 상수 grasp 변환" 하나로 줄었다. 추정할 자유도가 없다.
2. 그래서 late_table1의 A5(vision-aligned FK hard fixed)는 **정의 자체가
   성립하지 않아 제외**한다. A5는 Delta를 영상으로 맞추는 row이기 때문이다.
3. A3는 late_table1에서 "raw FK + 사전등록 기계 좌표변환"이었지만 여기서는
   "flange-to-cube 하드 고정"이 된다. 이름은 같고 내용은 다르다.
4. A4는 살아남는다. offset을 적합하는 게 아니라, 자유로운 큐브 자세를 위
   FK 예측 쪽으로 공분산 가중해 끌어당기는 soft factor이기 때문이다.

목적함수는 late_table1과 동일하다. 잔차는 corner 재투영 하나뿐이다.

    r_k = π( K_c, D_c, (T^B_Cc(e))⁻¹ · T^B_O , X_k ) − x_k          [px]
    고정캠 i : T^B_Cc = T^B_Ci
    그리퍼캠 g: T^B_Cc(e) = T^B_G(e) · T^G_Cg
    session1(큐브를 쥔 상태): T^B_cube(e) = T^B_G(e) · T^gripper_cube  (grasp 모델)

held-out 기준을 두 개로 나눈 이유
---------------------------------
ABLATION_RESULTS.md가 이미 지적한 순환이 있다. held-out 정답을 FK로 만들면
FK로 학습한 row가 이기는 게 당연하다. 그래서 두 기준을 **따로** 보고한다.

  FK 기준     : T^B_cube(h) = FK(h) · T^flange_cube 로 예측하고 재투영.
                FK 고정/soft factor row에 구조적으로 유리하다.
  VISION 기준 : held-out placement 자신의 관측만으로, 그 row의 외부 파라미터를
                동결한 채 큐브 자세를 다중카메라 번들로 추정한 뒤 재투영.
                카메라들끼리의 일치도이지 절대 정확도가 아니다.

두 기준에서 방향이 일치하는 대비만 결론으로 쓸 수 있다.

사용법:
  python zeus_gello_calibration/table1_zeus.py
  python zeus_gello_calibration/table1_zeus.py --rows A2,A3 --folds 5
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from calibration_pipeline.fk_factor import (  # noqa: E402
    FKFactorSpec, FK_MODE_FACTOR, FK_MODE_NONE, diagonal_covariance,
    solve_factorized_fk,
)
from calibration_pipeline.path_evaluation import solve_observed_pose  # noqa: E402
from calibration_pipeline.reprojection import (  # noqa: E402
    PoseState, SolverOptions, pose_delta, project_points,
    solve_corner_reprojection, variable_keys,
)
from calibration_pipeline.se3 import robust_se3_average  # noqa: E402
from calibration_pipeline.apriltag_cube import inv_T  # noqa: E402
from calibration_pipeline.table1 import estimate_board_handeye_initial  # noqa: E402

# fit_calibration_methods 는 import 사슬로 capture_session -> pyrealsense2 를
# 끌고 온다.  이 파일은 저장된 영상만 읽어 적합하므로 카메라 SDK 가 필요 없다.
# 하드웨어 없는 환경에서 돌리기 위해 없을 때만 빈 모듈로 대체한다.
try:  # pragma: no cover - 환경에 따라 갈림
    import pyrealsense2  # noqa: F401
except ModuleNotFoundError:
    import types
    sys.modules["pyrealsense2"] = types.ModuleType("pyrealsense2")

import fit_calibration_methods as fcm  # noqa: E402

GRIPPER = fcm.GRIPPER_LOCAL_ID

# late_table1의 A4가 쓰는 preflight 사전값과 같은 크기.  측정된 로봇 공분산이
# 아니므로 A4는 확정 근거가 아니라 민감도 점검으로만 읽는다.
SIGMA_FK_MM, SIGMA_FK_DEG = 2.0, 0.30


# ---------------------------------------------------------------- row 정의
# targets : calibration에 넣는 표적.  cube를 빼면 그 row는 큐브를 평가에만 쓴다.
# opt     : "uni" = 하나의 최소제곱, "seq" = eye-in-hand 먼저 풀고 동결 후 고정캠.
# fk      : "none"   큐브 자세 자유 (VISION)
#           "fixed"  T^B_cube = FK · T^flange_cube 하드 고정 (자유도 0)
#           "factor" 큐브 자세 자유 + 같은 FK 예측으로 끌어당기는 soft factor
ROWS = {
    "A0": dict(label="baseline (board-on-gripper only)", targets=("board",), opt="seq", fk="none"),
    "A1": dict(label="+cube, sequential", targets=("board", "cube"), opt="seq", fk="none"),
    "A2": dict(label="+unified, VISION", targets=("board", "cube"), opt="uni", fk="none"),
    "A3": dict(label="flange-to-cube hard fixed", targets=("board", "cube"), opt="uni", fk="fixed"),
    "A4": dict(label="flange-to-cube soft factor", targets=("board", "cube"), opt="uni", fk="factor"),
    "B1": dict(label="-Unified (soft factor, sequential)", targets=("board", "cube"), opt="seq", fk="factor"),
    "B2": dict(label="-board (cube only)", targets=("cube",), opt="uni", fk="none"),
    "B3": dict(label="-cube (board only, unified)", targets=("board",), opt="uni", fk="none"),
}
ROW_ORDER = ("A0", "A1", "A2", "A3", "A4", "A5", "B1", "B2", "B3")


def fk_cube_poses(items_by_index, T_flange_cube):
    """T^B_cube(s) = T^B_flange(place_s) · T^flange_cube.  offset 보정 없음."""
    return {int(s): fcm.pose6_to_T(item["target"]) @ T_flange_cube
            for s, item in items_by_index.items()}


def split_observations(data, targets, drop_set=None):
    """row가 쓰는 관측을 고른다.  drop_set은 held-out placement를 뺀다."""
    cube_obs, board_obs = [], []
    if "cube" in targets:
        cube_obs = list(data["obs_s1"])  # 쥔 큐브 (grasp 모델, placement 아님)
        for o in data["obs_s2_fixed"] + data["obs_s2_gripper"]:
            if drop_set is not None and o.set_idx is not None and int(o.set_idx) == int(drop_set):
                continue
            cube_obs.append(o)
    if "board" in targets:
        board_obs = list(data["obs_s3"])
    return cube_obs, board_obs


def held_out_observations(data, set_index):
    return [o for o in data["obs_s2_fixed"] + data["obs_s2_gripper"]
            if o.set_idx is not None and int(o.set_idx) == int(set_index)]


def make_state(data, cubes, grasp_init, board_init, gtc_init):
    return PoseState(cams=dict(data["cam_init"]), gtc=gtc_init.copy(),
                     board=board_init.copy(), cubes=dict(cubes),
                     grasps={0: grasp_init.copy()})


def fit_row(row, data, drop_set, robot_T, board_init, gtc_init):
    """한 row를 적합한다.  반환: (final_state, train_rmse_px, n_obs, ok)."""
    spec = ROWS[row]
    grasp_init = data["grasp_init"]
    cube_obs, board_obs = split_observations(data, spec["targets"], drop_set)
    K_map, D_map = data["K_map"], data["D_map"]
    options = SolverOptions()

    # 큐브 자세의 출처
    if "cube" not in spec["targets"]:
        cubes, cube_key = {}, []
    elif spec["fk"] == "fixed":
        cubes = fk_cube_poses(data["items_by_index"], grasp_init)
        cube_key = []                      # 자유도 없음 -- 하드 고정
    else:
        set_ids = sorted(data["items_by_index"])
        cubes = fcm.init_cube_poses(cube_obs, K_map, D_map, data["cam_init"],
                                    gtc_init, robot_T, GRIPPER, set_ids)
        cube_key = ["T_base_cube_by_set"]

    observations = cube_obs + board_obs
    if "cube" in spec["targets"]:
        observations = [o for o in observations
                        if o.set_idx is None or int(o.set_idx) in cubes or o.grasp_idx is not None]
    if not observations:
        return None, float("nan"), 0, False

    board_families = ["T_base_board"] if "board" in spec["targets"] else []
    grasp_families = ["T_gripper_cube_by_grasp"] if "cube" in spec["targets"] else []
    state = make_state(data, cubes, grasp_init, board_init, gtc_init)

    if spec["opt"] == "uni":
        families = ["T_base_Ci", "T_gripper_cam"] + board_families + cube_key + grasp_families
        keys = variable_keys(families, state)
        if spec["fk"] == "factor":
            cov = diagonal_covariance(SIGMA_FK_MM, SIGMA_FK_DEG)
            targets_fk = fk_cube_poses(data["items_by_index"], grasp_init)
            final, diag = solve_factorized_fk(
                observations=observations, variable_keys_=keys, reference_state=state,
                robot_T=robot_T, K_map=K_map, D_map=D_map, gripper_cam_idx=GRIPPER,
                options=options,
                fk_targets={s: t for s, t in targets_fk.items() if s in cubes},
                fk_covariances={s: cov for s in cubes},
                fk_spec=FKFactorSpec(mode=FK_MODE_FACTOR, loss="huber", robust_scale=3.0))
        else:
            final, diag = solve_corner_reprojection(
                observations=observations, variable_keys_=keys, reference_state=state,
                robot_T=robot_T, K_map=K_map, D_map=D_map, gripper_cam_idx=GRIPPER,
                options=options)
        ok = bool(diag.get("success", False))
    else:
        # sequential: eye-in-hand 먼저 (그리퍼캠 관측), 그 결과를 동결하고 고정캠.
        eih = [o for o in observations if int(o.cam) == GRIPPER]
        e2h = [o for o in observations if int(o.cam) != GRIPPER]
        if not eih or not e2h:
            return None, float("nan"), 0, False
        keys1 = variable_keys(["T_gripper_cam"] + board_families + cube_key, state)
        if spec["fk"] == "factor":
            # B1: soft FK factor 는 큐브 자세가 자유변수인 stage1 에 건다.
            # stage2 는 큐브를 동결하므로 FK 항이 걸릴 자유도가 없다.
            cov = diagonal_covariance(SIGMA_FK_MM, SIGMA_FK_DEG)
            targets_fk = fk_cube_poses(data["items_by_index"], grasp_init)
            stage1, d1 = solve_factorized_fk(
                observations=eih, variable_keys_=keys1, reference_state=state,
                robot_T=robot_T, K_map=K_map, D_map=D_map, gripper_cam_idx=GRIPPER,
                options=options,
                fk_targets={s: t for s, t in targets_fk.items() if s in cubes},
                fk_covariances={s: cov for s in cubes},
                fk_spec=FKFactorSpec(mode=FK_MODE_FACTOR, loss="huber", robust_scale=3.0))
        else:
            stage1, d1 = solve_corner_reprojection(
                observations=eih, variable_keys_=keys1, reference_state=state,
                robot_T=robot_T, K_map=K_map, D_map=D_map, gripper_cam_idx=GRIPPER, options=options)
        keys2 = variable_keys(["T_base_Ci"] + grasp_families, stage1)
        final, d2 = solve_corner_reprojection(
            observations=e2h, variable_keys_=keys2, reference_state=stage1,
            robot_T=robot_T, K_map=K_map, D_map=D_map, gripper_cam_idx=GRIPPER, options=options)
        ok = bool(d1.get("success", False) and d2.get("success", False))

    errs = fcm.per_corner_errors(final, observations, robot_T, K_map, D_map, GRIPPER)
    return final, fcm.rmse_px(errs), len(observations), ok


# ------------------------------------------------------------- held-out 평가
def camera_pose_estimates(obs_list, state, robot_T, K_map, D_map):
    """각 관측이 독립적으로 본 T^B_cube 후보."""
    out = []
    for o in obs_list:
        T_cam_obj = solve_observed_pose(o, K_map, D_map)
        if T_cam_obj is None:
            continue
        c = int(o.cam)
        if c == GRIPPER:
            if int(o.event) not in robot_T:
                continue
            out.append(robot_T[int(o.event)] @ state.gtc @ T_cam_obj)
        elif c in state.cams:
            out.append(state.cams[c] @ T_cam_obj)
    return out


def reproject_rmse(obs_list, T_base_cube, state, robot_T, K_map, D_map):
    errs = []
    for o in obs_list:
        c = int(o.cam)
        if c == GRIPPER:
            if int(o.event) not in robot_T:
                continue
            T_base_cam = robot_T[int(o.event)] @ state.gtc
        elif c in state.cams:
            T_base_cam = state.cams[c]
        else:
            continue
        uv = project_points(inv_T(T_base_cam) @ T_base_cube, o.object_points,
                            K_map[c], D_map[c])
        errs.extend(np.linalg.norm(uv - np.asarray(o.image_points).reshape(-1, 2), axis=1))
    return fcm.rmse_px(errs), len(errs)


def evaluate_fold(row, data, held_set, robot_T, board_init, gtc_init):
    final, train_px, n_obs, ok = fit_row(row, data, held_set, robot_T, board_init, gtc_init)
    if final is None:
        return None
    K_map, D_map = data["K_map"], data["D_map"]
    obs_h = held_out_observations(data, held_set)
    if not obs_h:
        return None

    # FK 기준: 모든 row가 같은 상수 T^flange_cube 를 쓴다 (offset 재보정 없음).
    T_fk = fk_cube_poses(data["items_by_index"], data["grasp_init"])[int(held_set)]
    px_fk, n_corners = reproject_rmse(obs_h, T_fk, final, robot_T, K_map, D_map)

    # VISION 기준: held-out placement 자신의 관측만으로 자세를 추정.
    cands = camera_pose_estimates(obs_h, final, robot_T, K_map, D_map)
    px_vis = float("nan"); mm = deg = float("nan"); spread_mm = float("nan")
    if cands:
        T_vis = cands[0] if len(cands) == 1 else robust_se3_average(cands, None)[0]
        px_vis, _ = reproject_rmse(obs_h, T_vis, final, robot_T, K_map, D_map)
        mm, deg = pose_delta(T_vis, T_fk)
        if len(cands) > 1:
            spread_mm = float(np.mean([pose_delta(T_vis, c)[0] for c in cands]))
    return dict(set=int(held_set), converged=ok, n_train_obs=n_obs,
                train_px=train_px, heldout_px_fk=px_fk, heldout_px_vision=px_vis,
                vision_vs_fk_mm=mm, vision_vs_fk_deg=deg,
                camera_spread_mm=spread_mm, n_heldout_corners=n_corners,
                n_camera_estimates=len(cands))


def aggregate(folds):
    ok = [f for f in folds if f is not None]
    if not ok:
        return {}
    def m(key):
        v = [f[key] for f in ok if f[key] == f[key]]
        return float(np.mean(v)) if v else float("nan")
    return dict(
        n_folds=len(ok), n_converged=sum(1 for f in ok if f["converged"]),
        train_px=m("train_px"), heldout_px_fk=m("heldout_px_fk"),
        heldout_px_vision=m("heldout_px_vision"),
        vision_vs_fk_mm=m("vision_vs_fk_mm"), vision_vs_fk_deg=m("vision_vs_fk_deg"),
        camera_spread_mm=m("camera_spread_mm"),
        n_heldout_corners=int(sum(f["n_heldout_corners"] for f in ok)),
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    fcm_parser_defaults = dict(
        session1_dir=str(fcm.SESSION1_DIR_DEFAULT),
        session1_capture_subdir="capture_replayed",
        session2_dir=str(fcm.SESSION2_DIR_DEFAULT),
        session2_capture_subdir="capture_placed",
        session3_dir=str(fcm.SESSION3_DIR_DEFAULT),
        session3_capture_subdir="capture_replayed",
        zeus_intrinsics_dir=str(REPO_ROOT / "intrinsics"),
        ur3_intrinsics_dir=str(REPO_ROOT / "ur3_calibration" / "intrinsics"),
        device_map=str(REPO_ROOT / "intrinsics" / "device_map.json"),
        fit_json=str(fcm.FIT_JSON_DEFAULT),   # capture_replayed 와 짝인 replayed 아티팩트
        fixed_min_corners=8,
        cube_observation_policy="legacy",
    )
    for key, value in fcm_parser_defaults.items():
        parser.add_argument(f"--{key.replace('_', '-')}", default=value,
                            type=type(value) if not isinstance(value, bool) else str)
    parser.add_argument("--rows", default=",".join(r for r in ROW_ORDER if r in ROWS))
    parser.add_argument("--folds", type=int, default=0, help="0 = 모든 placement")
    parser.add_argument("--out", default=str(Path(__file__).resolve().parent / "table1_zeus.json"))
    args = parser.parse_args()

    data = fcm.load_all_data(args)
    robot_T = {**data["robot_T_s1"], **data["robot_T_s2_gripper"], **data["robot_T_s3"]}
    # fit_calibration_methods.main() 과 동일한 초기화: session3 보드만으로 hand-eye 초기값
    gtc_init, board_init, eih_diag = estimate_board_handeye_initial(
        data["obs_s3"], data["robot_T_s3"], data["K_map"], data["D_map"], GRIPPER)
    print(f"T_gripper_cam 초기값 t_mm={np.round(gtc_init[:3,3]*1000,2).tolist()} ({eih_diag})")

    set_ids = sorted(int(s) for s in data["items_by_index"])
    if args.folds:
        set_ids = set_ids[:args.folds]
    rows = [r.strip() for r in args.rows.split(",") if r.strip() in ROWS]

    print(f"\nplacement {len(set_ids)}개 leave-one-out x row {len(rows)}개\n")
    result = {
        "schema": "table1_zeus_flange_to_cube_v1",
        "fk_definition": "T_base_cube[s] = T_base_flange(place_s) @ T_flange_cube  (offset 보정 없음)",
        "T_flange_cube_source": args.fit_json,
        "T_flange_cube_translation_mm": (np.asarray(data["grasp_init"])[:3, 3] * 1000.0).tolist(),
        "excluded_rows": {"A5": "raw FK @ Delta_train 이라 offset 방식 -- 이 데이터에서는 정의 불가"},
        "heldout_criteria": {
            "fk": "T_base_cube[h] = FK(h) @ T_flange_cube 로 예측 후 재투영 (FK row에 구조적으로 유리)",
            "vision": "held-out placement 자신의 관측으로 자세 추정 후 재투영 (카메라 일치도)",
        },
        "fk_factor_prior": {"translation_std_mm": SIGMA_FK_MM, "rotation_std_deg": SIGMA_FK_DEG,
                            "measured": False},
        "rows": {},
    }
    for row in rows:
        t0 = time.time()
        folds = [evaluate_fold(row, data, s, robot_T, board_init, gtc_init) for s in set_ids]
        agg = aggregate(folds)
        result["rows"][row] = {"condition": ROWS[row], "folds": [f for f in folds if f],
                              "summary": agg}
        print(f"  {row:<3} {ROWS[row]['label']:<36} "
              f"train {agg.get('train_px', float('nan')):.3f}px  "
              f"heldout(FK) {agg.get('heldout_px_fk', float('nan')):.3f}px  "
              f"heldout(VISION) {agg.get('heldout_px_vision', float('nan')):.3f}px  "
              f"[{time.time()-t0:.0f}s]")

    Path(args.out).write_text(json.dumps(result, indent=2, ensure_ascii=False), encoding="utf-8")
    print(f"\n[SAVE] {args.out}")


if __name__ == "__main__":
    main()
