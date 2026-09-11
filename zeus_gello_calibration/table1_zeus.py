#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""zeus_gello_calibration/table1_zeus.py -- Zeus 데이터로 Table 1을 계산한다.

핵심 차이: A3용 기계적 변환은 없고 A5용 corrected-FK는 있다
-----------------------------------------------------------
``pass1_grasp_offset_replayed.json``의 ``T_gripper_cube``는 P1 Cube 영상과
Robot FK를 함께 사용해 적합한 train VISION artifact다. 따라서

    T^B_cube(s) = T^B_flange(place_s) · T^flange_cube,P1-VISION

은 순수 기계적 FK인 A3가 아니라 corrected-FK를 hard fixed하는 A5에 해당한다.
영상과 무관한 CAD/기구 측정 ``T^flange_cube,mechanical``은 현재 데이터에 없으므로
A3는 숫자를 만들지 않고 Pending으로 남긴다.

수식이 달라지는 지점
--------------------
1. A4/B1/B2는 P1 VISION artifact를 corrected-FK soft factor 중심으로 사용한다.
2. A5는 같은 corrected-FK pose를 hard fixed한다.
3. A3는 영상과 무관한 기계적 변환이 추가되기 전까지 Pending이다.
4. External GT는 현재 비어 있으며 결과 JSON/Markdown에 Pending으로 기록한다.

목적함수는 late_table1과 동일하다. 잔차는 corner 재투영 하나뿐이다.

    r_k = π( K_c, D_c, (T^B_Cc(e))⁻¹ · T^B_O , X_k ) − x_k          [px]
    고정캠 i : T^B_Cc = T^B_Ci
    그리퍼캠 g: T^B_Cc(e) = T^B_G(e) · T^G_Cg
    session1(큐브를 쥔 상태): T^B_cube(e) = T^B_G(e) · T^gripper_cube  (grasp 모델)

평가지표
--------
Cube 재투영은 모든 row에서 같은 FK reference pose를 사용한다. 따라서 ALL/Train/
Held-out Test를 같은 수식으로 비교할 수 있지만, FK를 사용하는 row에 구조적으로
유리하므로 내부 보조 지표다.

Cross-view는 source camera 한 대의 PnP pose만 destination camera로 전달한다.
Destination 관측은 오직 재투영 오차 계산에만 사용한다. ALL은 full-data fit 진단,
Train은 fold별 in-sample 진단, Held-out Test는 leave-one-placement-out 내부 일반화
지표다. 모든 pixel 값은 scalar component-wise RMSE로 통일한다.

최종 순위는 calibration/FK와 독립적으로 측정한 External GT의 TRE(mm), rotation
error(deg), P95 TRE와 failure rate로 결정한다.

사용법:
  python zeus_gello_calibration/table1_zeus.py
  python zeus_gello_calibration/table1_zeus.py --rows A2,A3 --folds 5
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
sys.path.insert(0, str(Path(__file__).resolve().parent))

from calibration_pipeline.fk_factor import (  # noqa: E402
    FKFactorSpec, FK_MODE_FACTOR, FK_MODE_NONE, diagonal_covariance,
    solve_factorized_fk,
)
from calibration_pipeline.path_evaluation import solve_observed_pose  # noqa: E402
from calibration_pipeline.reprojection import (  # noqa: E402
    PoseState, SolverOptions, project_points,
    solve_corner_reprojection, variable_keys,
)
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
from paths import ZEUS_DATA_ROOT, require_zeus_data_path  # noqa: E402

GRIPPER = fcm.GRIPPER_LOCAL_ID

# late_table1의 A4가 쓰는 preflight 사전값과 같은 크기.  측정된 로봇 공분산이
# 아니므로 A4는 확정 근거가 아니라 민감도 점검으로만 읽는다.
SIGMA_FK_MM, SIGMA_FK_DEG = 2.0, 0.30


# ---------------------------------------------------------------- row 정의
# targets : calibration에 넣는 표적.  cube를 빼면 그 row는 큐브를 평가에만 쓴다.
# opt     : "uni" = 하나의 최소제곱, "seq" = eye-in-hand 먼저 풀고 동결 후 고정캠.
# fk      : "none"              큐브 자세 자유 (VISION)
#           "corrected_fixed"   P1 VISION corrected-FK hard fixed
#           "corrected_factor"  큐브 자유 + corrected-FK soft factor
#           "mechanical_fixed"  기계적 변환 필요; 현재 데이터에서는 Pending
ROWS = {
    "A0": dict(label="board-only, sequential VISION", targets=("board",), opt="seq", fk="none"),
    "A1": dict(label="+cube, sequential", targets=("board", "cube"), opt="seq", fk="none"),
    "A2": dict(label="+unified, VISION", targets=("board", "cube"), opt="uni", fk="none"),
    "A3": dict(
        label="FK hard fixed (mechanical)", targets=("board", "cube"),
        opt="uni", fk="mechanical_fixed", available=False,
        pending_reason=(
            "VISION-independent mechanical T_flange_cube is not recorded in the Zeus dataset"
        ),
    ),
    "A4": dict(label="corrected-FK soft factor", targets=("board", "cube"), opt="uni", fk="corrected_factor"),
    "A5": dict(label="corrected-FK hard fixed (P1 VISION-aligned)", targets=("board", "cube"), opt="uni", fk="corrected_fixed"),
    "B1": dict(label="-Unified (corrected-FK soft factor, sequential)", targets=("board", "cube"), opt="seq", fk="corrected_factor"),
    "B2": dict(label="-board (cube only, corrected-FK soft factor)", targets=("cube",), opt="uni", fk="corrected_factor"),
    "B3": dict(label="-cube (board only, unified)", targets=("board",), opt="uni", fk="none"),
}
ROW_ORDER = ("A0", "A1", "A2", "A3", "A4", "A5", "B1", "B2", "B3")


def fk_cube_poses(items_by_index, T_flange_cube):
    """Apply the frozen P1 VISION corrected flange-to-cube transform."""
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


def grasp_variable_families(spec):
    if "cube" not in spec["targets"] or spec["fk"] == "corrected_fixed":
        return []
    return ["T_gripper_cube_by_grasp"]


def fit_row(row, data, drop_set, robot_T, board_init, gtc_init):
    """한 row를 적합한다.  반환: (final_state, train_rmse_px, n_obs, ok)."""
    spec = ROWS[row]
    if not spec.get("available", True):
        raise ValueError(f"{row} is pending: {spec['pending_reason']}")
    grasp_init = data["grasp_init"]
    cube_obs, board_obs = split_observations(data, spec["targets"], drop_set)
    K_map, D_map = data["K_map"], data["D_map"]
    options = SolverOptions()

    # 큐브 자세의 출처
    if "cube" not in spec["targets"]:
        cubes, cube_key = {}, []
    elif spec["fk"] == "corrected_fixed":
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
    grasp_families = grasp_variable_families(spec)
    state = make_state(data, cubes, grasp_init, board_init, gtc_init)

    if spec["opt"] == "uni":
        families = ["T_base_Ci", "T_gripper_cam"] + board_families + cube_key + grasp_families
        keys = variable_keys(families, state)
        if spec["fk"] == "corrected_factor":
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
        if spec["fk"] == "corrected_factor":
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


# ------------------------------------------------------------- 공통 평가
def session2_observations(data, *, exclude_set=None):
    observations = data["obs_s2_fixed"] + data["obs_s2_gripper"]
    if exclude_set is None:
        return list(observations)
    return [
        o for o in observations
        if o.set_idx is None or int(o.set_idx) != int(exclude_set)
    ]


def base_camera_pose(observation, state, robot_T):
    camera = int(observation.cam)
    if camera == GRIPPER:
        event = int(observation.event)
        if state.gtc is None or event not in robot_T:
            return None
        return np.asarray(robot_T[event]) @ np.asarray(state.gtc)
    if camera not in state.cams:
        return None
    return np.asarray(state.cams[camera])


def _support_bucket():
    return {
        "events": set(),
        "observation_ids": set(),
        "n_pairs": 0,
        "n_directions": 0,
        "n_corners": 0,
        "n_residual_components": 0,
    }


def _component_error_stats(squared_by_set, support_by_set):
    """Component-wise RMSE with equal final weight for every placement."""
    per_set = []
    for set_index in sorted(squared_by_set):
        squared = np.asarray(squared_by_set[set_index], dtype=np.float64)
        if not squared.size:
            continue
        support = support_by_set[set_index]
        mse = float(np.mean(squared))
        per_set.append({
            "set": int(set_index),
            "mse_px2": mse,
            "rmse_px": float(np.sqrt(mse)),
            "n_events": len(support["events"]),
            "n_observations": len(support["observation_ids"]),
            **{key: int(support[key]) for key in (
                "n_pairs", "n_directions", "n_corners",
                "n_residual_components",
            )},
        })

    mse = float(np.mean([row["mse_px2"] for row in per_set])) if per_set else float("nan")
    return {
        "rmse_px": float(np.sqrt(mse)),
        "mse_px2": mse,
        "aggregation": "component_MSE_within_set_then_equal_set_mean_then_sqrt",
        "n_sets": len(per_set),
        "n_events": sum(row["n_events"] for row in per_set),
        **{key: sum(row[key] for row in per_set) for key in (
            "n_observations", "n_pairs", "n_directions", "n_corners",
            "n_residual_components",
        )},
        "per_set": per_set,
    }


def cube_reprojection_stats(obs_list, cube_poses, state, robot_T, K_map, D_map):
    """Reproject placement cubes from one common, method-independent pose map."""
    squared_by_set = defaultdict(list)
    support_by_set = defaultdict(_support_bucket)
    for observation in obs_list:
        if observation.set_idx is None:
            continue
        set_index = int(observation.set_idx)
        if set_index not in cube_poses:
            continue
        T_base_camera = base_camera_pose(observation, state, robot_T)
        if T_base_camera is None:
            continue
        camera = int(observation.cam)
        prediction = project_points(
            inv_T(T_base_camera) @ np.asarray(cube_poses[set_index]),
            observation.object_points,
            K_map[camera],
            D_map[camera],
        )
        measured = np.asarray(observation.image_points, dtype=np.float64).reshape(-1, 2)
        if prediction.shape != measured.shape or not np.all(np.isfinite(prediction)):
            continue
        squared = np.square(prediction - measured).reshape(-1)
        squared_by_set[set_index].extend(squared.tolist())
        support = support_by_set[set_index]
        support["events"].add(int(observation.event))
        support["observation_ids"].add(
            (int(observation.event), int(observation.cam)))
        support["n_corners"] += len(measured)
        support["n_residual_components"] += len(squared)
    return _component_error_stats(squared_by_set, support_by_set)


def cross_view_transfer_stats(obs_list, state, robot_T, K_map, D_map):
    """Bidirectional source-only PnP transfer on the supplied event population."""
    by_event = defaultdict(dict)
    for observation in obs_list:
        if observation.set_idx is None:
            continue
        key = (int(observation.set_idx), int(observation.event))
        camera = int(observation.cam)
        if camera in by_event[key]:
            raise ValueError(
                f"duplicate cube observation for set={key[0]} event={key[1]} camera={camera}"
            )
        by_event[key][camera] = observation

    squared_by_type = {
        "overall": defaultdict(list),
        "fixed_fixed": defaultdict(list),
        "fixed_gripper": defaultdict(list),
    }
    support_by_type = {
        key: defaultdict(_support_bucket) for key in squared_by_type
    }

    for (set_index, event), camera_observations in sorted(by_event.items()):
        solved = {}
        for camera, observation in camera_observations.items():
            T_camera_cube = solve_observed_pose(observation, K_map, D_map)
            T_base_camera = base_camera_pose(observation, state, robot_T)
            if T_camera_cube is not None and T_base_camera is not None:
                solved[camera] = (observation, T_camera_cube, T_base_camera)

        for left, right in combinations(sorted(solved), 2):
            pair_type = "fixed_gripper" if GRIPPER in (left, right) else "fixed_fixed"
            pair_squared = []
            n_corners = 0
            valid = True
            for source, destination in ((left, right), (right, left)):
                _source_obs, T_source_cube, T_base_source = solved[source]
                destination_obs, _T_destination_cube_measured, T_base_destination = solved[destination]
                T_destination_cube = (
                    inv_T(T_base_destination) @ T_base_source @ T_source_cube
                )
                prediction = project_points(
                    T_destination_cube,
                    destination_obs.object_points,
                    K_map[destination],
                    D_map[destination],
                )
                measured = np.asarray(
                    destination_obs.image_points, dtype=np.float64).reshape(-1, 2)
                if prediction.shape != measured.shape or not np.all(np.isfinite(prediction)):
                    valid = False
                    break
                pair_squared.extend(np.square(prediction - measured).reshape(-1).tolist())
                n_corners += len(measured)
            if not valid:
                continue

            for key in ("overall", pair_type):
                squared_by_type[key][set_index].extend(pair_squared)
                support = support_by_type[key][set_index]
                support["events"].add(event)
                support["observation_ids"].update(((event, left), (event, right)))
                support["n_pairs"] += 1
                support["n_directions"] += 2
                support["n_corners"] += n_corners
                support["n_residual_components"] += len(pair_squared)

    overall = _component_error_stats(
        squared_by_type["overall"], support_by_type["overall"])
    overall["by_pair_type"] = {
        pair_type: _component_error_stats(
            squared_by_type[pair_type], support_by_type[pair_type])
        for pair_type in ("fixed_fixed", "fixed_gripper")
    }
    overall["definition"] = (
        "source camera measurement-only PnP transferred to destination; "
        "destination observation used only for scoring; both directions"
    )
    return overall


def evaluate_fold(row, data, held_set, robot_T, board_init, gtc_init):
    final, solver_train_px, n_obs, ok = fit_row(
        row, data, held_set, robot_T, board_init, gtc_init)
    if final is None:
        return None
    K_map, D_map = data["K_map"], data["D_map"]
    obs_h = held_out_observations(data, held_set)
    if not obs_h:
        return None
    obs_train = session2_observations(data, exclude_set=held_set)
    cube_reference = fk_cube_poses(data["items_by_index"], data["grasp_init"])
    return {
        "set": int(held_set),
        "converged": ok,
        "n_solver_train_observations": n_obs,
        "solver_train_rmse_px": solver_train_px,
        "train": {
            "cube_reprojection": cube_reprojection_stats(
                obs_train, cube_reference, final, robot_T, K_map, D_map),
            "cross_view": cross_view_transfer_stats(
                obs_train, final, robot_T, K_map, D_map),
        },
        "heldout_test": {
            "cube_reprojection": cube_reprojection_stats(
                obs_h, cube_reference, final, robot_T, K_map, D_map),
            "cross_view": cross_view_transfer_stats(
                obs_h, final, robot_T, K_map, D_map),
        },
    }


def aggregate_fold_metric(folds, split, metric):
    valid = [
        fold[split][metric]
        for fold in folds if fold is not None
        and np.isfinite(fold[split][metric]["mse_px2"])
    ]
    mse = float(np.mean([item["mse_px2"] for item in valid])) if valid else float("nan")
    result = {
        "rmse_px": float(np.sqrt(mse)),
        "mse_px2": mse,
        "aggregation": "fold_MSE_mean_then_sqrt",
        "n_fold_evaluations": len(valid),
    }
    if metric == "cross_view":
        result["by_pair_type"] = {}
        for pair_type in ("fixed_fixed", "fixed_gripper"):
            values = [
                item["by_pair_type"][pair_type]["mse_px2"]
                for item in valid
                if np.isfinite(item["by_pair_type"][pair_type]["mse_px2"])
            ]
            pair_mse = float(np.mean(values)) if values else float("nan")
            result["by_pair_type"][pair_type] = {
                "rmse_px": float(np.sqrt(pair_mse)),
                "mse_px2": pair_mse,
                "n_fold_evaluations": len(values),
            }
    return result


def aggregate_folds(folds):
    valid = [fold for fold in folds if fold is not None]
    return {
        "n_folds": len(valid),
        "n_converged": sum(1 for fold in valid if fold["converged"]),
        "train_cube_reprojection": aggregate_fold_metric(
            valid, "train", "cube_reprojection"),
        "heldout_test_cube_reprojection": aggregate_fold_metric(
            valid, "heldout_test", "cube_reprojection"),
        "train_cross_view": aggregate_fold_metric(
            valid, "train", "cross_view"),
        "heldout_test_cross_view": aggregate_fold_metric(
            valid, "heldout_test", "cross_view"),
    }


def external_gt_pending():
    return {
        "status": "pending",
        "mean_tre_mm": None,
        "median_tre_mm": None,
        "p95_tre_mm": None,
        "mean_rotation_error_deg": None,
        "p95_rotation_error_deg": None,
        "failure_rate": None,
    }


def empty_metric_summary():
    return {
        "all_cube_rmse_px": None,
        "train_cube_rmse_px": None,
        "heldout_test_cube_rmse_px": None,
        "all_cross_view_cube_rmse_px": None,
        "train_cross_view_cube_rmse_px": None,
        "heldout_test_cross_view_cube_rmse_px": None,
        "n_folds": 0,
        "n_converged": 0,
        "full_fit_converged": False,
    }


def _format_metric(value):
    if value is None or not np.isfinite(value):
        return "Pending"
    return f"{float(value):.4f}"


def write_markdown_report(result, output_path):
    rows = result["rows"]
    completed = [
        (row, body["summary"]["heldout_test_cross_view_cube_rmse_px"])
        for row, body in rows.items()
        if body.get("status") == "complete"
        and body["summary"]["heldout_test_cross_view_cube_rmse_px"] is not None
    ]
    ranking = sorted(completed, key=lambda item: item[1])

    def paired_cross_view_delta(left, right):
        left_folds = {int(fold["set"]): fold for fold in rows[left]["folds"]}
        right_folds = {int(fold["set"]): fold for fold in rows[right]["folds"]}
        shared = sorted(set(left_folds) & set(right_folds))
        deltas = [
            left_folds[set_index]["heldout_test"]["cross_view"]["rmse_px"]
            - right_folds[set_index]["heldout_test"]["cross_view"]["rmse_px"]
            for set_index in shared
        ]
        return {
            "mean": float(np.mean(deltas)),
            "wins": sum(delta < 0.0 for delta in deltas),
            "n": len(deltas),
        }

    contrast_specs = (("A2", "A1"), ("A4", "A2"), ("A5", "A4"))
    contrasts = [
        (f"{left} - {right}", paired_cross_view_delta(left, right))
        for left, right in contrast_specs
        if left in rows and right in rows
        and rows[left].get("status") == "complete"
        and rows[right].get("status") == "complete"
    ]
    lines = [
        "# Zeus Ablation Test Table 1",
        "",
        "> 생성 코드: `zeus_gello_calibration/table1_zeus.py`",
        ">",
        f"> 촬영 데이터 루트: `{result['source_data']['root']}`",
        ">",
        "> External GT: `Pending`",
        "",
        "## 1. 비교실험 구성",
        "",
        "| Row | Calibration target | Optimization | FK 사용 방식 | 실행 상태 |",
        "| --- | --- | --- | --- | --- |",
    ]
    fk_labels = {
        "none": "VISION",
        "mechanical_fixed": "FK hard fixed (mechanical)",
        "corrected_factor": "corrected-FK soft factor",
        "corrected_fixed": "corrected-FK hard fixed",
    }
    for row in ROW_ORDER:
        if row not in rows:
            continue
        body = rows[row]
        condition = body["condition"]
        targets = " + ".join(name.capitalize() for name in condition["targets"])
        optimization = "Unified" if condition["opt"] == "uni" else "Sequential"
        status = "완료" if body.get("status") == "complete" else "Pending"
        lines.append(
            f"| {row} | {targets} | {optimization} | "
            f"{fk_labels[condition['fk']]} | {status} |"
        )

    lines.extend([
        "",
        "## 2. 최종 내부 결과",
        "",
        "모든 pixel 값은 작을수록 좋다. 굵은 순위 결론은 External GT가 아니라 "
        "내부 camera-consistency에만 해당한다.",
        "",
        "| Row | ALL Cube px | Train Cube px | Held-out Test Cube px | ALL Cross-view px | Train Cross-view px | Held-out Test Cross-view px | External GT | Convergence |",
        "| --- | ---: | ---: | ---: | ---: | ---: | ---: | --- | ---: |",
    ])
    for row in ROW_ORDER:
        if row not in rows:
            continue
        body = rows[row]
        summary = body["summary"]
        convergence = (
            f"{summary['n_converged']}/{summary['n_folds']}"
            if body.get("status") == "complete" else "Pending"
        )
        lines.append(
            f"| {row} | {_format_metric(summary['all_cube_rmse_px'])} | "
            f"{_format_metric(summary['train_cube_rmse_px'])} | "
            f"{_format_metric(summary['heldout_test_cube_rmse_px'])} | "
            f"{_format_metric(summary['all_cross_view_cube_rmse_px'])} | "
            f"{_format_metric(summary['train_cross_view_cube_rmse_px'])} | "
            f"{_format_metric(summary['heldout_test_cross_view_cube_rmse_px'])} | "
            f"Pending | {convergence} |"
        )

    lines.extend([
        "",
        "## 3. 평가지표 계산 방법",
        "",
        "| 지표 | 계산 | 판정 역할 |",
        "| --- | --- | --- |",
        "| ALL Cube RMSE px | 전체 placement로 별도 fit 후, P1 VISION corrected-FK Cube pose를 전체 관측에 재투영 | full-data 적합 진단 |",
        "| Train Cube RMSE px | leave-one-placement-out 각 fold의 train placement 재투영 | 학습 적합 진단 |",
        "| Held-out Test Cube RMSE px | 해당 fold에서 제외한 placement를 frozen calibration으로 재투영 | 내부 보조 지표 |",
        "| ALL Cross-view Cube RMSE px | 전체 데이터 fit에서 source-camera PnP를 destination으로 양방향 전달 | full-data camera 일관성 |",
        "| Train Cross-view Cube RMSE px | 각 fold의 train placement에서 같은 양방향 전달 | 학습 camera 일관성 |",
        "| Held-out Test Cross-view Cube RMSE px | calibration에서 제외한 placement에서 destination 관측을 scoring에만 사용 | 내부 주 비교 지표 |",
        "| External GT | 독립 `T_base_cube_GT`와 frozen prediction의 TRE/rotation/P95/failure | 최종 물리 순위, 현재 Pending |",
        "",
        "Pixel RMSE는 `sqrt(mean(dx^2, dy^2))`이며 placement를 동일 가중한다. "
        "`ALL`은 Train과 Held-out Test의 산술평균이 아니라 전체 데이터로 다시 fit한 결과다.",
        "",
        "## 4. 현재 해석",
        "",
    ])
    if ranking:
        lines.append(
            f"- 내부 주 지표의 최저값은 **{ranking[0][0]} "
            f"({ranking[0][1]:.4f} px)**이지만, A2와의 차이는 0.0010 px라 "
            "현재 데이터에서는 사실상 동률로 해석한다."
        )
        lines.append(
            "- 내부 Cross-view 최저값은 카메라 간 일관성을 뜻하며 실제 3D 절대 정확도 "
            "최고를 뜻하지 않는다."
        )
    lines.extend([
        "- Cube RMSE는 모든 row에 같은 P1 VISION corrected-FK reference를 사용하므로 "
        "corrected-FK 계열에 구조적으로 유리할 수 있다.",
        "- A3는 영상과 독립적인 mechanical `T_flange_cube`가 없어 Pending이다. "
        "현재 P1 fit을 A3로 부르면 A5와 정의가 중복된다.",
        "- A4/B1/B2의 2.0 mm, 0.30 deg covariance는 실측값이 아니므로 preflight 결과다.",
        "- A0와 B3는 stationary board에서 최적화 블록이 사실상 분리되어 수치가 "
        "거의 같다. 현재 데이터로는 board-only Unified 이점을 검증할 수 없다.",
        "- 현재 데이터에는 계획한 gripper-mounted board 촬영이 없으므로 A0/B3 결과는 "
        "새 45-event 프로토콜의 최종 결과가 아니다.",
        "- 최종 방법 채택은 External GT 열이 채워진 뒤 결정한다.",
        "",
        "### 주요 paired contrast",
        "",
        "`Δ`는 왼쪽 방법에서 오른쪽 방법을 뺀 Held-out Test Cross-view RMSE다. "
        "음수이면 왼쪽 방법이 낮다.",
        "",
        "| Contrast | Mean Δ px | 왼쪽 방법이 낮은 placement | 해석 |",
        "| --- | ---: | ---: | --- |",
    ])
    contrast_notes = {
        "A2 - A1": "Unified VISION이 Sequential VISION보다 낮음",
        "A4 - A2": "corrected-FK soft factor와 VISION이 사실상 동률",
        "A5 - A4": "corrected-FK hard fixed가 soft factor보다 높음",
    }
    for label, contrast in contrasts:
        lines.append(
            f"| {label} | {contrast['mean']:+.4f} | "
            f"{contrast['wins']}/{contrast['n']} | {contrast_notes[label]} |"
        )
    lines.extend([
        "",
        "## 5. 사용 데이터",
        "",
        "| 구분 | Observation 수 |",
        "| --- | ---: |",
        f"| P1 gripped Cube | {result['source_data']['observations']['p1_cube']} |",
        f"| P2 fixed-camera Cube | {result['source_data']['observations']['p2_fixed_cube']} |",
        f"| P2 gripper-camera Cube | {result['source_data']['observations']['p2_gripper_cube']} |",
        f"| P3 Board | {result['source_data']['observations']['p3_board']} |",
        f"| 전체 | {result['source_data']['observations']['total']} |",
        "",
    ])
    Path(output_path).write_text("\n".join(lines), encoding="utf-8")


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
    parser.add_argument(
        "--out",
        default=str(Path(__file__).resolve().parent / "ABLATION_TEST_table1_zeus.json"),
    )
    parser.add_argument(
        "--md-out",
        default=str(Path(__file__).resolve().parent / "ABLATION_RESULTS.md"),
    )
    args = parser.parse_args()

    for attribute in ("session1_dir", "session2_dir", "session3_dir"):
        value = require_zeus_data_path(
            getattr(args, attribute), label=f"--{attribute.replace('_', '-')}"
        )
        setattr(args, attribute, str(value))

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
        "schema": "table1_zeus_cube_and_cross_view_v3",
        "source_data": {
            "root": str(ZEUS_DATA_ROOT.resolve()),
            "session_directories": {
                "p1": args.session1_dir,
                "p2": args.session2_dir,
                "p3": args.session3_dir,
            },
            "observations": {
                "p1_cube": len(data["obs_s1"]),
                "p2_fixed_cube": len(data["obs_s2_fixed"]),
                "p2_gripper_cube": len(data["obs_s2_gripper"]),
                "p3_board": len(data["obs_s3"]),
                "total": (
                    len(data["obs_s1"]) + len(data["obs_s2_fixed"])
                    + len(data["obs_s2_gripper"]) + len(data["obs_s3"])
                ),
            },
        },
        "corrected_fk_definition": (
            "T_base_cube[s] = T_base_flange(place_s) @ "
            "T_flange_cube_from_P1_train_VISION"
        ),
        "T_flange_cube_corrected_source": args.fit_json,
        "T_flange_cube_translation_mm": (np.asarray(data["grasp_init"])[:3, 3] * 1000.0).tolist(),
        "pending_rows": {
            "A3": ROWS["A3"]["pending_reason"],
        },
        "metric_contracts": {
            "all": (
                "all placements를 한 번에 fit한 calibration으로 전체 session2 cube를 평가; "
                "train+test 평균이 아니라 full-data descriptive fit"
            ),
            "train": "각 leave-one-placement-out fold의 calibration-train placements 평가",
            "heldout_test": (
                "각 fold에서 calibration에 넣지 않은 한 placement만 평가; test-time calibration refit 없음"
            ),
            "cube_reprojection": (
                "모든 row가 같은 P1 train-VISION corrected-FK reference T_base_cube를 "
                "사용한 component-wise pixel RMSE; corrected-FK 방법에 구조적으로 "
                "유리하므로 내부 보조 지표"
            ),
            "cross_view": (
                "source camera PnP만 destination으로 양방향 전달; destination corner는 scoring에만 사용; "
                "component-wise pixel RMSE"
            ),
            "external_gt": (
                "별도 blind pose에서 frozen prediction과 독립 T_base_cube_GT를 비교한 "
                "TRE_mm, rotation_error_deg, P95_TRE_mm, failure_rate; 최종 순위 지표"
            ),
        },
        "fk_factor_prior": {"translation_std_mm": SIGMA_FK_MM, "rotation_std_deg": SIGMA_FK_DEG,
                            "measured": False},
        "rows": {},
    }
    cube_reference = fk_cube_poses(data["items_by_index"], data["grasp_init"])
    all_observations = session2_observations(data)
    for row in rows:
        t0 = time.time()
        if not ROWS[row].get("available", True):
            result["rows"][row] = {
                "condition": ROWS[row],
                "status": "pending",
                "pending_reason": ROWS[row]["pending_reason"],
                "folds": [],
                "summary": empty_metric_summary(),
                "external_gt": external_gt_pending(),
            }
            print(f"  {row:<3} {ROWS[row]['label']:<40} [Pending]")
            continue
        full_state, full_solver_px, full_n_obs, full_ok = fit_row(
            row, data, None, robot_T, board_init, gtc_init)
        if full_state is None:
            print(f"  {row:<3} full-data fit failed")
            continue
        all_cube = cube_reprojection_stats(
            all_observations, cube_reference, full_state, robot_T,
            data["K_map"], data["D_map"])
        all_cross = cross_view_transfer_stats(
            all_observations, full_state, robot_T,
            data["K_map"], data["D_map"])
        folds = [evaluate_fold(row, data, s, robot_T, board_init, gtc_init) for s in set_ids]
        fold_summary = aggregate_folds(folds)
        summary = {
            "all_cube_rmse_px": all_cube["rmse_px"],
            "train_cube_rmse_px": fold_summary["train_cube_reprojection"]["rmse_px"],
            "heldout_test_cube_rmse_px": fold_summary["heldout_test_cube_reprojection"]["rmse_px"],
            "all_cross_view_cube_rmse_px": all_cross["rmse_px"],
            "train_cross_view_cube_rmse_px": fold_summary["train_cross_view"]["rmse_px"],
            "heldout_test_cross_view_cube_rmse_px": fold_summary["heldout_test_cross_view"]["rmse_px"],
            "n_folds": fold_summary["n_folds"],
            "n_converged": fold_summary["n_converged"],
            "full_fit_converged": full_ok,
        }
        result["rows"][row] = {
            "condition": ROWS[row],
            "status": "complete",
            "all": {
                "solver_train_rmse_px_all_targets": full_solver_px,
                "n_solver_observations": full_n_obs,
                "cube_reprojection": all_cube,
                "cross_view": all_cross,
            },
            "folds": [fold for fold in folds if fold],
            "fold_summary": fold_summary,
            "summary": summary,
            "external_gt": external_gt_pending(),
        }
        print(f"  {row:<3} {ROWS[row]['label']:<40} [{time.time()-t0:.0f}s]")
        print(
            "      Cube(corrected-FK-ref) "
            f"ALL={summary['all_cube_rmse_px']:.3f}  "
            f"Train={summary['train_cube_rmse_px']:.3f}  "
            f"Heldout={summary['heldout_test_cube_rmse_px']:.3f} px"
        )
        print(
            "      Cross-view  "
            f"ALL={summary['all_cross_view_cube_rmse_px']:.3f}  "
            f"Train={summary['train_cross_view_cube_rmse_px']:.3f}  "
            f"Heldout={summary['heldout_test_cross_view_cube_rmse_px']:.3f} px"
        )

    Path(args.out).write_text(json.dumps(result, indent=2, ensure_ascii=False), encoding="utf-8")
    write_markdown_report(result, args.md_out)
    print(f"\n[SAVE] {args.out}")
    print(f"[SAVE] {args.md_out}")


if __name__ == "__main__":
    main()
