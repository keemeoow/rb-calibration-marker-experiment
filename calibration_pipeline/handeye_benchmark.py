#!/usr/bin/env python3
"""Compare one session with classical OpenCV hand-eye methods.

All executable baselines use the official nominal metric configuration, exact
train/held-out split, train-only Board PnP, and the frozen set-anchor evaluation
mask.  No fitted scale, held-out refit, or output-dependent rejection is used.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import os
from collections import defaultdict
from typing import Mapping, Sequence

import cv2
import numpy as np
from scipy.spatial.transform import Rotation

from calibration_pipeline import se3 as cp
from calibration_pipeline import table1
from calibration_pipeline.apriltag_cube import inv_T
from calibration_pipeline.evaluation import jsonable, pixel_reprojection_metrics, serialize_state
from calibration_pipeline.path_evaluation import (
    build_fixed_to_fixed_cross_target_mask,
    build_gripper_to_fixed_cross_target_mask,
    evaluate_fixed_to_fixed_cross_target,
    evaluate_gripper_to_fixed_cross_target,
    solve_observed_pose,
)
from calibration_pipeline.reprojection import PixelObs, PoseState, pose_delta
from calibration_pipeline.schema import DEFAULT_SPLIT_SEED


ARTIFACT_SCHEMA = "session_handeye_method_comparison_v1"
CURRENT_METHODS = ("A0", "A2", "A3")
CLASSICAL_METHODS = {
    "opencv_tsai_lenz": ("Tsai–Lenz", cv2.CALIB_HAND_EYE_TSAI),
    "opencv_park_martin": ("Park–Martin", cv2.CALIB_HAND_EYE_PARK),
    "opencv_horaud": ("Horaud", cv2.CALIB_HAND_EYE_HORAUD),
    "opencv_andreff": ("Andreff", cv2.CALIB_HAND_EYE_ANDREFF),
    "opencv_daniilidis": ("Daniilidis", cv2.CALIB_HAND_EYE_DANIILIDIS),
}
ROBOT_WORLD_METHODS = {
    "opencv_robot_world_shah": (
        "Shah robot-world/hand-eye", cv2.CALIB_ROBOT_WORLD_HAND_EYE_SHAH),
    "opencv_robot_world_li": (
        "Li robot-world/hand-eye", cv2.CALIB_ROBOT_WORLD_HAND_EYE_LI),
}


def _file_sha256(path: str) -> str:
    digest = hashlib.sha256()
    with open(path, "rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _transform(rotation, translation) -> np.ndarray:
    value = np.eye(4, dtype=np.float64)
    value[:3, :3] = np.asarray(rotation, dtype=np.float64).reshape(3, 3)
    value[:3, 3] = np.asarray(translation, dtype=np.float64).reshape(3)
    return value


def _validate_transform(name: str, transform: np.ndarray) -> None:
    value = np.asarray(transform, dtype=np.float64)
    if value.shape != (4, 4) or not np.all(np.isfinite(value)):
        raise ValueError(f"{name} is not a finite 4x4 transform")
    if not np.allclose(value[3], [0.0, 0.0, 0.0, 1.0], atol=1e-9):
        raise ValueError(f"{name} has an invalid homogeneous row")
    rotation = value[:3, :3]
    if (not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-5)
            or not np.isclose(np.linalg.det(rotation), 1.0, atol=1e-5)):
        raise ValueError(f"{name} rotation is not in SO(3)")


def _random_transform(rng: np.random.Generator, translation_m: float) -> np.ndarray:
    axis = rng.normal(size=3)
    axis /= np.linalg.norm(axis)
    value = np.eye(4, dtype=np.float64)
    value[:3, :3] = Rotation.from_rotvec(
        axis * rng.uniform(-2.5, 2.5)).as_matrix()
    value[:3, 3] = rng.uniform(-translation_m, translation_m, size=3)
    return value


def run_synthetic_direction_contract(seed: int = 17) -> dict:
    """Prove the OpenCV input/output directions on a noise-free scene."""
    rng = np.random.default_rng(int(seed))
    truth_gtc = _random_transform(rng, 0.15)
    truth_base_board = _random_transform(rng, 0.4)
    robots, camera_boards = [], []
    for _ in range(30):
        T_base_gripper = _random_transform(rng, 0.6)
        robots.append(T_base_gripper)
        camera_boards.append(
            inv_T(T_base_gripper @ truth_gtc) @ truth_base_board)

    results = {}
    for key, (label, method) in CLASSICAL_METHODS.items():
        rotation, translation = cv2.calibrateHandEye(
            [pose[:3, :3] for pose in robots],
            [pose[:3, 3].reshape(3, 1) for pose in robots],
            [pose[:3, :3] for pose in camera_boards],
            [pose[:3, 3].reshape(3, 1) for pose in camera_boards],
            method=method)
        dt, dr = pose_delta(truth_gtc, _transform(rotation, translation))
        results[key] = {
            "label": label, "handeye_translation_error_mm": dt,
            "handeye_rotation_error_deg": dr}

    for key, (label, method) in ROBOT_WORLD_METHODS.items():
        output = cv2.calibrateRobotWorldHandEye(
            [pose[:3, :3] for pose in camera_boards],
            [pose[:3, 3].reshape(3, 1) for pose in camera_boards],
            [inv_T(pose)[:3, :3] for pose in robots],
            [inv_T(pose)[:3, 3].reshape(3, 1) for pose in robots],
            method=method)
        estimate_board = inv_T(_transform(output[0], output[1]))
        estimate_gtc = inv_T(_transform(output[2], output[3]))
        dt, dr = pose_delta(truth_gtc, estimate_gtc)
        bt, br = pose_delta(truth_base_board, estimate_board)
        results[key] = {
            "label": label, "handeye_translation_error_mm": dt,
            "handeye_rotation_error_deg": dr,
            "base_board_translation_error_mm": bt,
            "base_board_rotation_error_deg": br}

    tolerances = {"translation_mm": 1e-6, "rotation_deg": 1e-6}
    passed = all(
        result["handeye_translation_error_mm"] <= tolerances["translation_mm"]
        and result["handeye_rotation_error_deg"] <= tolerances["rotation_deg"]
        and result.get("base_board_translation_error_mm", 0.0)
        <= tolerances["translation_mm"]
        and result.get("base_board_rotation_error_deg", 0.0)
        <= tolerances["rotation_deg"]
        for result in results.values())
    return {
        "passed": bool(passed), "seed": int(seed), "n_poses": len(robots),
        "tolerances": tolerances, "methods": results}


def _train_board_pairs(
        observations: Sequence[PixelObs], robot_T: Mapping[int, np.ndarray],
        K_map, D_map, gripper: int) -> list[tuple[int, np.ndarray]]:
    by_event: dict[int, list[np.ndarray]] = defaultdict(list)
    for observation in observations:
        event = int(observation.event)
        if (observation.marker != "board" or int(observation.cam) != int(gripper)
                or event not in robot_T):
            continue
        pose = solve_observed_pose(observation, K_map, D_map)
        if pose is not None:
            by_event[event].append(pose)
    pairs = []
    for event, poses in sorted(by_event.items()):
        pose = poses[0] if len(poses) == 1 else cp.robust_se3_average(poses)[0]
        pairs.append((event, pose))
    if len(pairs) < 5:
        raise RuntimeError(f"at least five train Board poses are required, got {len(pairs)}")
    return pairs


def _board_spread(board: np.ndarray, candidates: Sequence[np.ndarray]) -> dict:
    deltas = [pose_delta(board, candidate) for candidate in candidates]
    return {
        "n_poses": len(deltas),
        "translation_rmse_mm": float(np.sqrt(np.mean([x[0] ** 2 for x in deltas]))),
        "rotation_rmse_deg": float(np.sqrt(np.mean([x[1] ** 2 for x in deltas]))),
    }


def _estimate_method(
        method_key: str, pairs: Sequence[tuple[int, np.ndarray]], robot_T,
        train_observations: Sequence[PixelObs], K_map, D_map,
        gripper: int) -> tuple[PoseState, dict]:
    if method_key in CLASSICAL_METHODS:
        label, method = CLASSICAL_METHODS[method_key]
        rotation, translation = cv2.calibrateHandEye(
            [robot_T[event][:3, :3] for event, _ in pairs],
            [robot_T[event][:3, 3].reshape(3, 1) for event, _ in pairs],
            [pose[:3, :3] for _, pose in pairs],
            [pose[:3, 3].reshape(3, 1) for _, pose in pairs], method=method)
        gtc = _transform(rotation, translation)
        candidates = [robot_T[event] @ gtc @ pose for event, pose in pairs]
        board, average_diagnostics = cp.robust_se3_average(candidates)
        family = "OpenCV calibrateHandEye"
        convention = "T_gripper_camera returned directly"
    elif method_key in ROBOT_WORLD_METHODS:
        label, method = ROBOT_WORLD_METHODS[method_key]
        output = cv2.calibrateRobotWorldHandEye(
            [pose[:3, :3] for _, pose in pairs],
            [pose[:3, 3].reshape(3, 1) for _, pose in pairs],
            [inv_T(robot_T[event])[:3, :3] for event, _ in pairs],
            [inv_T(robot_T[event])[:3, 3].reshape(3, 1) for event, _ in pairs],
            method=method)
        board = inv_T(_transform(output[0], output[1]))
        gtc = inv_T(_transform(output[2], output[3]))
        candidates = [robot_T[event] @ gtc @ pose for event, pose in pairs]
        average_diagnostics = None
        family = "OpenCV calibrateRobotWorldHandEye"
        convention = "returned T_world_base and T_camera_gripper are inverted"
    else:
        raise KeyError(f"unknown method {method_key}")

    _validate_transform(f"{method_key}.T_gripper_camera", gtc)
    _validate_transform(f"{method_key}.T_base_board", board)
    fixed_board = [
        observation for observation in train_observations
        if observation.marker == "board" and int(observation.cam) != int(gripper)]
    cams, sources = table1.estimate_fixed_camera_initials(
        fixed_board, board, {}, K_map, D_map, gripper)
    if not cams:
        raise RuntimeError(f"{method_key}: no fixed camera was registered")
    for camera, value in cams.items():
        _validate_transform(f"{method_key}.T_base_camera[{camera}]", value)
    return PoseState(cams=cams, gtc=gtc, board=board, cubes={}), {
        "label": label, "family": family, "fit_target": "train-only Board",
        "n_train_eye_in_hand_board_poses": len(pairs),
        "fixed_camera_registration": "train-only Board robust SE3 average",
        "fixed_camera_source_counts": sources,
        "board_pose_spread": _board_spread(board, candidates),
        "board_average_diagnostics": average_diagnostics,
        "opencv_output_convention": convention,
    }


def _metric_fields(prefix: str, result: Mapping, target: str) -> dict:
    values = result["by_target"][target]
    return {
        f"{prefix}_{target}_pixel_rmse_px": values["cross_view_pixel_transfer_rmse_px"],
        f"{prefix}_{target}_translation_rmse_mm": values["pose_consistency_translation_rmse_mm"],
        f"{prefix}_{target}_rotation_rmse_deg": values["pose_consistency_rotation_rmse_deg"],
    }


def _evaluate_method(state: PoseState, prepared, masks: Mapping) -> tuple[dict, dict]:
    train_board = [x for x in prepared.train_obs if x.marker == "board"]
    heldout_board = [x for x in prepared.test_obs if x.marker == "board"]
    train = pixel_reprojection_metrics(
        train_board, state, prepared.robot_T, prepared.K_map, prepared.D_map,
        prepared.gripper)
    heldout = pixel_reprojection_metrics(
        heldout_board, state, prepared.robot_T, prepared.K_map, prepared.D_map,
        prepared.gripper)
    all_observations = list(prepared.train_obs) + list(prepared.test_obs)
    fixed = evaluate_fixed_to_fixed_cross_target(
        prepared.test_obs, state.cams, prepared.K_map, prepared.D_map,
        masks["fixed_to_fixed"])
    gripper = evaluate_gripper_to_fixed_cross_target(
        all_observations, state.cams, state.gtc, prepared.robot_T,
        prepared.K_map, prepared.D_map, masks["gripper_to_fixed"])
    detail = {
        "transforms": serialize_state(state), "train_board_reprojection": train,
        "heldout_board_reprojection": heldout, "fixed_to_fixed": fixed,
        "gripper_to_fixed": gripper}
    summary = {
        "train_board_reprojection_rmse_px": train["overall"]["rmse_px"],
        "heldout_board_reprojection_rmse_px": heldout["overall"]["rmse_px"],
        **_metric_fields("fixed_to_fixed", fixed, "board"),
        **_metric_fields("fixed_to_fixed", fixed, "cube"),
        **_metric_fields("gripper_to_fixed", gripper, "board"),
        **_metric_fields("gripper_to_fixed", gripper, "cube"),
    }
    return detail, summary


def _mean(values) -> float:
    return float(np.mean([float(value) for value in values]))


def _load_current_rows(table_path: str, cross_path: str, split: Mapping) -> list[dict]:
    with open(table_path) as handle:
        table_result = json.load(handle)
    with open(cross_path) as handle:
        cross_result = json.load(handle)
    scale = table_result.get("protocol", {}).get("board_metric_scale", {})
    if scale.get("enabled") is not False or float(scale.get("scale", -1.0)) != 1.0:
        raise RuntimeError("official comparison input is not nominal-scale")
    if (table_result.get("protocol", {}).get("split") != split
            or cross_result.get("protocol", {}).get("split") != split):
        raise RuntimeError("stored result split does not match the benchmark split")
    cross_by_method = {row["method"]: row for row in cross_result["summary"]}
    labels = {
        "A0": ("A0 current board-only", "Board; sequential nonlinear reprojection"),
        "A2": ("A2 current Board+Cube", "Board+Cube; unified nonlinear reprojection"),
        "A3": ("A3 current full", "Board+Cube; unified reprojection; FK-fixed Cube poses"),
    }
    output = []
    for method in CURRENT_METHODS:
        runs, cross = table_result["rows"][method]["runs"], cross_by_method[method]
        label, fit = labels[method]
        row = {
            "method": method, "label": label,
            "family": "current calibration pipeline", "fit": fit,
            "status": "complete", "n_runs": len(runs),
            "train_board_reprojection_rmse_px": _mean(
                run["train_reprojection"]["board"]["rmse_px"] for run in runs),
            "heldout_board_reprojection_rmse_px": _mean(
                run["heldout_reprojection"]["board"]["rmse_px"] for run in runs),
        }
        for scope in ("fixed_to_fixed", "gripper_to_fixed"):
            for target in ("board", "cube"):
                source = f"{scope}_{target}"
                row[f"{source}_pixel_rmse_px"] = cross[
                    f"{source}_cross_view_pixel_transfer_rmse_px_mean"]
                row[f"{source}_translation_rmse_mm"] = cross[
                    f"{source}_pose_consistency_translation_rmse_mm_mean"]
                row[f"{source}_rotation_rmse_deg"] = cross[
                    f"{source}_pose_consistency_rotation_rmse_deg_mean"]
        output.append(row)
    return output


def _fmt(value) -> str:
    return "—" if value is None else f"{float(value):.3f}"


def _write_markdown(result: Mapping, path: str) -> None:
    complete = [row for row in result["summary"] if row["status"] == "complete"]
    closed = [row for row in complete if row["method"].startswith("opencv_")]
    best_board = min(closed, key=lambda row: row["gripper_to_fixed_board_pixel_rmse_px"])
    best_cube = min(closed, key=lambda row: row["gripper_to_fixed_cube_pixel_rmse_px"])
    a3 = next(row for row in complete if row["method"] == "A3")
    protocol = result["protocol"]
    lines = [
        "# Session04 캘리브레이션 방법 비교", "",
        "> 상태: 외부 GT 전 내부 비교. 아래 수치는 절대 위치 정확도나 논문 간 SOTA 순위가 아니라, 동일한 Session04 관측에서 측정한 재투영 및 경로 일관성이다.", "",
        "## 핵심 결과", "",
        f"- OpenCV 계열 중 Gripper-to-Fixed Board pixel 최저는 **{best_board['label']} ({_fmt(best_board['gripper_to_fixed_board_pixel_rmse_px'])} px)** 이다.",
        f"- OpenCV 계열 중 Gripper-to-Fixed Cube pixel 최저는 **{best_cube['label']} ({_fmt(best_cube['gripper_to_fixed_cube_pixel_rmse_px'])} px)** 이다.",
        f"- 현재 A3는 Board/Cube 각각 **{_fmt(a3['gripper_to_fixed_board_pixel_rmse_px'])} / {_fmt(a3['gripper_to_fixed_cube_pixel_rmse_px'])} px** 이다. 두 표적을 임의의 단일 점수로 합치지 않았다.",
        f"- Shah 대비 A3는 Cube pixel/translation이 각각 **{(1.0 - a3['gripper_to_fixed_cube_pixel_rmse_px'] / best_cube['gripper_to_fixed_cube_pixel_rmse_px']) * 100.0:.1f}% / {(1.0 - a3['gripper_to_fixed_cube_translation_rmse_mm'] / best_cube['gripper_to_fixed_cube_translation_rmse_mm']) * 100.0:.1f}% 낮지만**, Board pixel은 **{(a3['gripper_to_fixed_board_pixel_rmse_px'] / best_board['gripper_to_fixed_board_pixel_rmse_px'] - 1.0) * 100.0:.1f}% 높다**. A3가 전체적으로 우월한 것이 아니라 Board–Cube 절충점이 다르다.",
        "- 이 비교로 hand-eye와 Robot FK 오차를 분리할 수 없으므로 mm/deg도 두 경로의 일관성으로 해석한다.", "",
        "## 동일 조건", "",
        f"- 데이터: `{protocol['dataset_root']}`",
        f"- 학습 Event: {protocol['split']['train_events']}",
        f"- Held-out Event: {protocol['split']['test_events']}",
        f"- 평가 set: {protocol['split']['eligible_sets']}",
        "- 물리 scale: nominal `1.0`; 데이터 추정 scale 미사용",
        "- OpenCV 7종: train-only Board PnP와 Robot FK로 fit; train-only Board로 고정카메라 등록",
        "- 평가: 같은 set의 첫 fixed anchor × 해당 set의 모든 held-out gripper Event; Event→set→set 동일가중",
        "- held-out refit 및 모델 출력 기반 관측 제거 없음", "",
        "## 전체 경로 비교", "",
        "| 방법 | Fit 정보 | Held-out Board reproj (px) | G→F Board (px / mm / deg) | G→F Cube (px / mm / deg) |",
        "| --- | --- | ---: | ---: | ---: |",
    ]
    for row in complete:
        lines.append(
            f"| {row['label']} | {row['fit']} | {_fmt(row['heldout_board_reprojection_rmse_px'])} | "
            f"{_fmt(row['gripper_to_fixed_board_pixel_rmse_px'])} / {_fmt(row['gripper_to_fixed_board_translation_rmse_mm'])} / {_fmt(row['gripper_to_fixed_board_rotation_rmse_deg'])} | "
            f"{_fmt(row['gripper_to_fixed_cube_pixel_rmse_px'])} / {_fmt(row['gripper_to_fixed_cube_translation_rmse_mm'])} / {_fmt(row['gripper_to_fixed_cube_rotation_rmse_deg'])} |")
    lines.extend([
        "", "`G→F`는 Gripper-to-Fixed이다. px/mm/deg는 합산하지 않는다. A0와 OpenCV 7종은 Board-only 정보 예산이 같고, A2/A3에는 Cube 정보의 효과가 포함된다.", "",
        "## 고정카메라 부분 비교", "",
        "| 방법 | Fixed Board (px / mm / deg) | Fixed Cube (px / mm / deg) |",
        "| --- | ---: | ---: |",
    ])
    for row in complete:
        lines.append(
            f"| {row['label']} | {_fmt(row['fixed_to_fixed_board_pixel_rmse_px'])} / {_fmt(row['fixed_to_fixed_board_translation_rmse_mm'])} / {_fmt(row['fixed_to_fixed_board_rotation_rmse_deg'])} | "
            f"{_fmt(row['fixed_to_fixed_cube_pixel_rmse_px'])} / {_fmt(row['fixed_to_fixed_cube_translation_rmse_mm'])} / {_fmt(row['fixed_to_fixed_cube_rotation_rmse_deg'])} |")
    lines.extend([
        "", "OpenCV 7종의 Fixed-to-Fixed 값이 같은 것은 오류가 아니다. 이 기준선들은 동일한 train-only Board 관측으로 고정카메라를 등록하고 hand-eye 해법만 바꾸므로, 공통 Board 좌표계의 강체변환은 고정카메라 상대 pose에서 상쇄된다.",
        "", "## 결과가 말해 주는 현재 문제", "",
        "- Board-only 계열은 Board에서 좋지만 Cube로 일반화할 때 오차가 커진다.",
        "- A2/A3는 Cube를 공동최적화하면서 Cube 오차를 줄이는 대신 Board 쪽 오차가 증가한다.",
        "- 따라서 현재 잔차는 단순히 고전 hand-eye 해법이 약해서 생긴 것으로 보기 어렵다. 한 외부파라미터가 Board와 Cube를 동시에 만족하지 못하는 **표적 간 모델/측정 불일치**가 남아 있다는 증거다.",
        "- 가능한 원인은 Cube 3D geometry·corner ordering·실측 치수, Board 치수, intrinsic/distortion, 또는 표적별 검출 systematic bias다. 이 표만으로 원인 하나를 확정하지는 않는다.",
        "- 이 불일치를 해결하기 전에는 더 복잡한 SOTA 최적화가 한 표적의 오차를 다른 표적으로 이동시키는 결과가 될 수 있다.",
        "", "## 최근 방법과의 적용성 비교", "",
        "| 방법 | 핵심 | Session04 수치 | 현재 판단 |", "| --- | --- | --- | --- |",
        "| OpenCV hand-eye 5종 | AX=XB closed-form/separable/simultaneous 해법 | 실행 완료 | Board-only 고전 기준선 |",
        "| OpenCV Shah/Li | robot-world와 hand-eye 동시 추정 | 실행 완료 | Board-only 동시추정 기준선 |",
        "| 현재 A3 | Board+Cube raw-corner 공동최적화 + FK-fixed Cube pose | 실행 완료 | 현재 데이터 구조에 직접 맞음 |",
        "| Allegro et al. Multi-Camera Hand-Eye (RA-L 2024, ICRA 2025) | camera-base와 camera-camera 상대 pose 공동최적화 | 미실행 | 공개 C++ 구현의 Session04 adapter와 동일 mask 연결 필요 |",
        "| Tabb & Yousef iterative robot-world/hand-eye (2019) | 재투영오차 직접 최소화; multiple-eye 확장 | 미실행 | 동일 관측 모델 이식 필요 |", "",
        "Allegro 공개 구현은 카메라별 동일 프레임 수를 요구한다. Session04는 fixed camera를 set당 한 번만 저장하므로 fixed 영상을 복제하지 않고 누락 프레임으로 표현하는 adapter가 필요하다.", "",
        "## 검증 및 출처", "",
        f"- 합성 좌표계 계약: **{'PASS' if result['synthetic_direction_contract']['passed'] else 'FAIL'}** ({result['synthetic_direction_contract']['n_poses']} poses, 7 methods)",
        f"- OpenCV 버전: `{protocol['opencv_version']}`",
        f"- fixed-to-fixed mask SHA-256: `{protocol['evaluation_masks']['fixed_to_fixed']}`",
        f"- gripper-to-fixed mask SHA-256: `{protocol['evaluation_masks']['gripper_to_fixed']}`",
        "- [OpenCV 공식 문서](https://docs.opencv.org/doc/doxygen/html/d4/d93/group__calib.html)",
        "- [Allegro et al. 논문](https://arxiv.org/abs/2406.11392), [공개 구현](https://github.com/davidea97/Multi-Camera-Hand-Eye-Calibration)",
        "- [Tabb & Yousef](https://arxiv.org/abs/1907.12425)", "",
        "## 해석 한계", "",
        "외부 tracker/정밀 치구/독립 3D GT가 없어 절대 정확도는 평가하지 않았다. Fixed-to-Fixed는 공통 계통오차를 놓칠 수 있고, Gripper-to-Fixed는 hand-eye와 FK 오차를 함께 포함한다. 따라서 논문 원문 수치와의 직접 순위표가 아니다.", "",
    ])
    failed = [row for row in result["summary"] if row["status"] != "complete"]
    if failed:
        lines.extend(["## 실패한 실행", ""])
        lines.extend(f"- {row['label']}: `{row.get('error', 'unknown')}`" for row in failed)
        lines.append("")
    with open(path, "w") as handle:
        handle.write("\n".join(lines))


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description="Session hand-eye method comparison")
    parser.add_argument("--root_folder", default="data/session04/calib_train")
    parser.add_argument("--intrinsics_dir", default="intrinsics")
    parser.add_argument("--include_sets", default="0-12")
    parser.add_argument("--test_fraction", type=float, default=0.2)
    parser.add_argument("--split_seed", type=int, default=DEFAULT_SPLIT_SEED)
    parser.add_argument("--min_train_eih_cube_events", type=int, default=3)
    parser.add_argument("--observation_manifest", default="data/session04/calib_out/capture_filter/Step2b_observation_manifest.json")
    parser.add_argument("--observation_filter_policy", choices=("standard", "strict"), default="standard")
    parser.add_argument("--table1_result", default="CP_result/session04/late_table1/table1_methods.json")
    parser.add_argument("--cross_target_result", default="CP_result/session04/cross_target_evaluation/cross_target_evaluation.json")
    parser.add_argument("--out_dir", default="CP_result/session04/handeye_method_comparison")
    return parser.parse_args(argv)


def main(argv=None) -> None:
    args = parse_args(argv)
    synthetic = run_synthetic_direction_contract()
    if not synthetic["passed"]:
        raise RuntimeError("OpenCV transform-direction synthetic contract failed")
    prepare_argv = [
        "--root_folder", args.root_folder, "--intrinsics_dir", args.intrinsics_dir,
        "--include_sets", args.include_sets, "--test_fraction", str(args.test_fraction),
        "--split_seed", str(args.split_seed), "--min_train_eih_cube_events",
        str(args.min_train_eih_cube_events), "--observation-manifest",
        args.observation_manifest, "--observation-filter-policy",
        args.observation_filter_policy, "--rows", "A0"]
    prepared = table1.prepare_ablation_data(table1.parse_args(prepare_argv))
    if prepared.board_metric_scale.get("enabled") is not False:
        raise RuntimeError("benchmark must use nominal physical scale")

    current_rows = _load_current_rows(
        args.table1_result, args.cross_target_result, prepared.split)
    with open(args.cross_target_result) as handle:
        stored_cross = json.load(handle)
    fixed_cameras = list(stored_cross["protocol"]["evaluation_fixed_camera_intersection"])
    event_roles = {
        **{int(event): "train" for event in prepared.split["train_events"]},
        **{int(event): "heldout" for event in prepared.split["test_events"]}}
    path_observations = list(prepared.train_obs) + list(prepared.test_obs)
    fixed_mask = build_fixed_to_fixed_cross_target_mask(
        prepared.test_obs, fixed_cameras, prepared.K_map, prepared.D_map,
        set_filter=prepared.split["eligible_sets"])
    gripper_mask = build_gripper_to_fixed_cross_target_mask(
        prepared.test_obs, fixed_cameras, prepared.gripper, prepared.K_map,
        prepared.D_map, set_filter=prepared.split["eligible_sets"],
        fixed_anchor_observations=path_observations, event_roles=event_roles)
    if fixed_mask["evaluation_mask_sha256"] != stored_cross["protocol"]["fixed_to_fixed_evaluation"]["evaluation_mask_sha256"]:
        raise RuntimeError("fixed-to-fixed evaluation mask drift")
    if gripper_mask["evaluation_mask_sha256"] != stored_cross["protocol"]["gripper_to_fixed_evaluation"]["evaluation_mask_sha256"]:
        raise RuntimeError("gripper-to-fixed evaluation mask drift")

    pairs = _train_board_pairs(
        prepared.train_obs, prepared.robot_T, prepared.K_map,
        prepared.D_map, prepared.gripper)
    details, baseline_rows = {}, []
    methods = CLASSICAL_METHODS | ROBOT_WORLD_METHODS
    for method_key, (label, _) in methods.items():
        print(f"[HAND-EYE] {label}")
        try:
            state, diagnostics = _estimate_method(
                method_key, pairs, prepared.robot_T, prepared.train_obs,
                prepared.K_map, prepared.D_map, prepared.gripper)
            missing = sorted(set(fixed_cameras) - set(state.cams))
            if missing:
                raise RuntimeError(f"missing fixed cameras {missing}")
            detail, metrics = _evaluate_method(
                state, prepared, {"fixed_to_fixed": fixed_mask,
                                  "gripper_to_fixed": gripper_mask})
            details[method_key] = {
                "status": "complete", "diagnostics": diagnostics,
                "evaluation": detail}
            baseline_rows.append({
                "method": method_key, "label": label,
                "family": diagnostics["family"],
                "fit": "Board-only closed-form + robust fixed-camera registration",
                "status": "complete", "n_runs": 1, **metrics})
        except Exception as error:
            details[method_key] = {"status": "failed", "error": repr(error)}
            baseline_rows.append({
                "method": method_key, "label": label, "family": "OpenCV",
                "fit": "Board-only", "status": "failed", "n_runs": 1,
                "error": repr(error)})

    result = {
        "artifact_schema": ARTIFACT_SCHEMA,
        "protocol": {
            "dataset_root": os.path.abspath(args.root_folder),
            "intrinsics_dir": os.path.abspath(args.intrinsics_dir),
            "observation_manifest": {
                "path": os.path.abspath(args.observation_manifest),
                "sha256": _file_sha256(args.observation_manifest),
                "policy": args.observation_filter_policy},
            "table1_result": {"path": os.path.abspath(args.table1_result),
                              "sha256": _file_sha256(args.table1_result)},
            "cross_target_result": {
                "path": os.path.abspath(args.cross_target_result),
                "sha256": _file_sha256(args.cross_target_result)},
            "opencv_version": cv2.__version__, "split": prepared.split,
            "board_metric_scale": prepared.board_metric_scale,
            "fit_population": "train-only Board; one PnP pose per gripper event",
            "n_train_eye_in_hand_board_poses": len(pairs),
            "fixed_camera_registration": "train-only Board robust SE3 average",
            "heldout_refit": False, "model_dependent_observation_rejection": False,
            "external_ground_truth_used": False,
            "evaluation_masks": {
                "fixed_to_fixed": fixed_mask["evaluation_mask_sha256"],
                "gripper_to_fixed": gripper_mask["evaluation_mask_sha256"]},
            "fixed_camera_ids": fixed_cameras,
            "gripper_camera_id": prepared.gripper},
        "synthetic_direction_contract": synthetic,
        "summary": current_rows + baseline_rows,
        "opencv_method_details": details}
    os.makedirs(args.out_dir, exist_ok=True)
    json_path = os.path.join(args.out_dir, "handeye_method_comparison.json")
    csv_path = os.path.join(args.out_dir, "handeye_method_comparison.csv")
    md_path = os.path.join(args.out_dir, "README.md")
    with open(json_path, "w") as handle:
        json.dump(jsonable(result), handle, indent=2, allow_nan=False)
    fieldnames = []
    for row in result["summary"]:
        fieldnames.extend(field for field in row if field not in fieldnames)
    with open(csv_path, "w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames, lineterminator="\n")
        writer.writeheader()
        writer.writerows(result["summary"])
    _write_markdown(result, md_path)
    print(f"[DONE] {md_path}")


if __name__ == "__main__":
    main()
