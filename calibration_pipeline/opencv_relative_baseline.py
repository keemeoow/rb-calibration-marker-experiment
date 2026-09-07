#!/usr/bin/env python3
"""Independent OpenCV PnP fixed-camera relative-pose reference baseline.

This diagnostic deliberately avoids the repository's joint optimizer, robot
FK, hand-eye transform, and shared base-frame target pose.  Each train target
seen by an anchor camera and another fixed camera yields one direct relative
transform.  OpenCV PnP plus a preregistered robust SE(3) average produces the
calibration, which is then frozen and evaluated on the same held-out
fixed-to-fixed board/cube mask used by the main methods.

This is policy B: an independent FK-free reference baseline.  It is a
transparent public-library reference, not external GT or a SOTA claim.
"""

from __future__ import annotations

import argparse
import csv
import json
import os
from collections import defaultdict
from types import SimpleNamespace
from typing import Mapping, Sequence

import numpy as np

from calibration_pipeline.apriltag_cube import inv_T
from calibration_pipeline.evaluation import jsonable
from calibration_pipeline.path_evaluation import (
    CROSS_TARGETS,
    build_fixed_to_fixed_cross_target_mask,
    evaluate_fixed_to_fixed_cross_target,
    solve_observed_pose,
)
from calibration_pipeline.reprojection import PixelObs, pose_delta
from calibration_pipeline.schema import (
    DEFAULT_SPLIT_SEED,
    RELATIVE_POSE_REPORTING_CONTRACT,
)
from calibration_pipeline.se3 import robust_se3_average
from calibration_pipeline.runtime import (
    filter_meta_by_set_indices,
    load_intrinsics_with_depth_scale,
    DEFAULT_SESSION_ROOT,
    apply_session_defaults,
)
from calibration_pipeline import table1


BASELINES: Mapping[str, tuple[str, ...]] = {
    "opencv_board": ("board",),
    "opencv_cube": ("cube",),
    "opencv_board_cube": ("board", "cube"),
}


def direct_relative_candidates(
        observations: Sequence[PixelObs], fixed_cameras: Sequence[int],
        anchor: int, targets: Sequence[str], K_map, D_map) -> dict[int, list]:
    allowed_cameras = set(map(int, fixed_cameras))
    allowed_targets = set(map(str, targets))
    grouped = defaultdict(dict)
    for observation in observations:
        camera = int(observation.cam)
        marker = str(observation.marker)
        if camera not in allowed_cameras or marker not in allowed_targets:
            continue
        grouped[(marker, int(observation.event))][camera] = observation

    candidates = {camera: [] for camera in fixed_cameras if camera != anchor}
    for _, by_camera in sorted(grouped.items()):
        if anchor not in by_camera:
            continue
        T_anchor_target = solve_observed_pose(
            by_camera[anchor], K_map, D_map)
        if T_anchor_target is None:
            continue
        for camera in candidates:
            if camera not in by_camera:
                continue
            T_camera_target = solve_observed_pose(
                by_camera[camera], K_map, D_map)
            if T_camera_target is None:
                continue
            # Gauge: B is the anchor-camera frame.  T^B_Ci is all the evaluator
            # needs; no robot-base pose or target pose enters this relation.
            candidates[camera].append(
                T_anchor_target @ inv_T(T_camera_target))
    return candidates


def fit_baseline(candidates: Mapping[int, list], anchor: int) -> tuple[dict, dict]:
    cameras = {int(anchor): np.eye(4, dtype=np.float64)}
    diagnostics = {str(anchor): {
        "role": "gauge_anchor", "num_total": 0, "num_inliers": 0}}
    for camera, transforms in sorted(candidates.items()):
        if not transforms:
            raise RuntimeError(
                f"anchor camera {anchor} has no direct train overlap with camera {camera}")
        average, detail = robust_se3_average(list(transforms))
        cameras[int(camera)] = average
        diagnostics[str(camera)] = detail
    return cameras, diagnostics


def summarize(name: str, result: dict, diagnostics: dict) -> dict:
    row = {"baseline": name}
    for target in CROSS_TARGETS:
        metrics = result["by_target"][target]
        for field in (
                "cross_view_pixel_transfer_rmse_px",
                "pose_consistency_translation_rmse_mm",
                "pose_consistency_rotation_rmse_deg"):
            row[f"{target}_{field}"] = metrics[field]
        row[f"n_{target}_pairs"] = metrics["n_pairs"]
    non_anchor = [value for value in diagnostics.values()
                  if value.get("role") != "gauge_anchor"]
    row["n_train_relative_candidates"] = sum(
        int(value["num_total"]) for value in non_anchor)
    row["n_train_relative_inliers"] = sum(
        int(value["num_inliers"]) for value in non_anchor)
    return row


def markdown_report(summary: Sequence[Mapping], conflict: Mapping) -> str:
    score_fields = (
        "board_cross_view_pixel_transfer_rmse_px",
        "board_pose_consistency_translation_rmse_mm",
        "cube_cross_view_pixel_transfer_rmse_px",
        "cube_pose_consistency_translation_rmse_mm",
    )
    best = {
        field: min(float(row[field]) for row in summary)
        for field in score_fields
    }

    def fmt_score(row: Mapping, field: str) -> str:
        formatted = f"{float(row[field]):.4f}"
        best_formatted = f"{best[field]:.4f}"
        return f"**{formatted}**" if formatted == best_formatted else formatted

    lines = [
        "# B — Independent OpenCV Relative-pose Reference Baseline",
        "",
        "이 기준선은 OpenCV PnP로 학습 영상의 고정카메라 상대 자세를 직접 "
        "계산하는 독립 FK-free 기준선이다. Main-method transform, Joint "
        "optimizer, Robot FK, Hand–Eye, "
        "shared target pose를 사용하지 않는다. SOTA 비교나 절대 정확도 "
        "주장이 아니다.",
        "",
        "A의 방법별 cross-view pixel transfer/e_cross는 held-out 자기 일관성을 보는 "
        "보조 지표이고, 이 B가 그 값과 독립적으로 계산되는 relative-pose "
        "기준선이다.",
        "",
        "> **굵은 값**은 Board 또는 Cube 평가 열의 최솟값이다. 외부 GT "
        "정확도 순위를 뜻하지 않는다.",
        "",
        "| Train target | Board transfer px | Board translation mm | "
        "Cube transfer px | Cube translation mm | Train candidates/inliers |",
        "| --- | ---: | ---: | ---: | ---: | ---: |",
    ]
    labels = {
        "opencv_board": "Board only",
        "opencv_cube": "Cube only",
        "opencv_board_cube": "Board + Cube naive average",
    }
    for row in summary:
        lines.append(
            f"| {labels.get(row['baseline'], row['baseline'])} | "
            f"{fmt_score(row, 'board_cross_view_pixel_transfer_rmse_px')} | "
            f"{fmt_score(row, 'board_pose_consistency_translation_rmse_mm')} | "
            f"{fmt_score(row, 'cube_cross_view_pixel_transfer_rmse_px')} | "
            f"{fmt_score(row, 'cube_pose_consistency_translation_rmse_mm')} | "
            f"{row['n_train_relative_candidates']} / "
            f"{row['n_train_relative_inliers']} |")
    lines.extend([
        "",
        "## Board-vs-Cube Relative-transform Conflict",
        "",
        "같은 물리적 카메라 관계를 board train 관측과 cube train 관측에서 "
        "각각 계산한 뒤 비교한 값이다. FK는 들어가지 않는다.",
        "",
        "| Camera | Translation disagreement mm | Rotation disagreement deg |",
        "| --- | ---: | ---: |",
    ])
    for camera, values in conflict["per_camera"].items():
        lines.append(
            f"| {camera} | {values['translation_mm']:.4f} | "
            f"{values['rotation_deg']:.4f} |")
    lines.extend([
        "",
        "해석: Board-only는 held-out board에서 좋지만 cube에서는 나쁘고, "
        "Cube-only는 그 반대다. 즉 현재 큰 수치는 custom optimizer 하나만의 "
        "문제라기보다 target geometry/detection/pose convention 사이의 "
        "불일치 가능성을 함께 보여준다. Board와 cube 후보를 단순 평균한 "
        "결과도 양쪽 모두를 해결하지 못했다.",
        "",
        "다음 진단: 동일 event의 board-PnP 상대 자세와 cube-PnP 상대 자세의 "
        "차이를 camera pair별로 분해하고, cube 3D geometry·corner ordering·"
        "intrinsic/distortion을 우선 점검한다.",
    ])
    return "\n".join(lines) + "\n"


def validate_payload(payload: Mapping) -> None:
    """Keep baseline B independent from every fitted main-method transform."""
    protocol = payload.get("protocol", {})
    if protocol.get("relative_pose_reporting") != \
            RELATIVE_POSE_REPORTING_CONTRACT:
        raise ValueError("relative-pose reporting policy drift")
    expected = RELATIVE_POSE_REPORTING_CONTRACT[
        "independent_reference_baseline"]
    if protocol.get("independent_reference_contract") != expected:
        raise ValueError("independent relative-pose baseline contract is missing")
    for key in (
            "uses_fitted_main_method_camera_poses", "uses_joint_optimizer",
            "uses_robot_fk", "uses_handeye", "uses_shared_target_pose"):
        if protocol.get(key) is not False:
            raise ValueError(f"independent baseline dependency violation: {key}")


def parse_args(argv=None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="OpenCV PnP FK-free fixed-camera reference baseline")
    parser.add_argument("--root_folder", default=DEFAULT_SESSION_ROOT)
    parser.add_argument("--intrinsics_dir", default="intrinsics")
    parser.add_argument("--calib_dir", default="data/session/calib_out")
    parser.add_argument("--include_sets", default="5-12")
    parser.add_argument("--test_fraction", type=float, default=0.2)
    parser.add_argument("--split_seed", type=int, default=DEFAULT_SPLIT_SEED)
    parser.add_argument("--min_train_eih_cube_events", type=int, default=3)
    parser.add_argument("--image_scale", type=float, default=1.0)
    parser.add_argument(
        "--observation-manifest", "--observation_manifest",
        dest="observation_manifest")
    parser.add_argument(
        "--observation-filter-policy", "--observation_filter_policy",
        dest="observation_filter_policy", choices=("standard", "strict"),
        default="standard")
    parser.add_argument(
        "--align-board-metric-scale", "--align_board_metric_scale",
        dest="align_board_metric_scale", action="store_true")
    parser.add_argument(
        "--out_dir", default=None,
        help="Default: CP_result/<session>/opencv_relative_baseline.")
    args = parser.parse_args(argv)
    return apply_session_defaults(args, {'out_dir': 'opencv_relative_dir',
         'observation_manifest': 'observation_manifest'})


def prepare_visual_only_data(args) -> SimpleNamespace:
    """Load and split pixels without executing any FK/hand-eye initializer."""
    with open(os.path.join(args.root_folder, "meta.json")) as handle:
        meta = json.load(handle)
    meta, _ = filter_meta_by_set_indices(meta, args.include_sets)
    camera_ids = sorted({
        int(camera) for capture in meta.get("captures", [])
        for camera in capture.get("cams", {})})
    gripper = int(meta["gripper_cam_idx"])
    K_map, D_map = {}, {}
    for camera in camera_ids:
        K_map[camera], D_map[camera], _ = load_intrinsics_with_depth_scale(
            args.intrinsics_dir, camera)
    observations, _, _ = table1.detect_observations(
        args, meta, K_map, D_map, camera_ids, gripper)
    split = table1.build_event_split(
        observations, gripper, args.test_fraction, args.split_seed,
        args.min_train_eih_cube_events)
    eligible = set(split["eligible_sets"])
    pool = [observation for observation in observations
            if observation.set_idx in eligible]
    train_events = set(split["train_events"])
    test_events = set(split["test_events"])
    train_obs = [observation for observation in pool
                 if int(observation.event) in train_events]
    test_obs = [observation for observation in pool
                if int(observation.event) in test_events]
    if bool(getattr(args, "align_board_metric_scale", False)):
        board_metric_scale = table1.estimate_train_board_metric_scale(
            train_obs, gripper, K_map, D_map)
        observations = table1.apply_board_metric_scale(
            observations, board_metric_scale["scale"])
        pool = [observation for observation in observations
                if observation.set_idx in eligible]
        train_obs = [observation for observation in pool
                     if int(observation.event) in train_events]
        test_obs = [observation for observation in pool
                    if int(observation.event) in test_events]
    else:
        board_metric_scale = {
            "mode": "nominal_config", "enabled": False, "scale": 1.0,
            "heldout_observations_used": False,
        }
    provenance = table1._source_data_provenance(  # noqa: SLF001
        args, camera_ids, pool, train_obs, test_obs)
    return SimpleNamespace(
        gripper=gripper, K_map=K_map, D_map=D_map, split=split,
        train_obs=train_obs, test_obs=test_obs,
        source_data_provenance=provenance,
        board_metric_scale=board_metric_scale)


def main(argv=None) -> None:
    args = parse_args(argv)
    prepared = prepare_visual_only_data(args)
    train_cameras = {
        int(obs.cam) for obs in prepared.train_obs
        if int(obs.cam) != prepared.gripper}
    test_cameras = {
        int(obs.cam) for obs in prepared.test_obs
        if int(obs.cam) != prepared.gripper}
    fixed_cameras = sorted(train_cameras & test_cameras)
    if len(fixed_cameras) < 2:
        raise RuntimeError("reference baseline needs two fixed cameras")
    anchor = fixed_cameras[0]
    evaluation_mask = build_fixed_to_fixed_cross_target_mask(
        prepared.test_obs, fixed_cameras, prepared.K_map, prepared.D_map,
        set_filter=prepared.split["eligible_sets"])

    results, summary, fitted_cameras = {}, [], {}
    for name, targets in BASELINES.items():
        candidates = direct_relative_candidates(
            prepared.train_obs, fixed_cameras, anchor, targets,
            prepared.K_map, prepared.D_map)
        cameras, diagnostics = fit_baseline(candidates, anchor)
        fitted_cameras[name] = cameras
        evaluation = evaluate_fixed_to_fixed_cross_target(
            prepared.test_obs, cameras, prepared.K_map, prepared.D_map,
            evaluation_mask)
        results[name] = {
            "train_targets": list(targets),
            "T_anchor_camera": {
                str(camera): transform.tolist()
                for camera, transform in sorted(cameras.items())},
            "fit_diagnostics": diagnostics,
            "heldout_fixed_to_fixed": evaluation,
        }
        summary.append(summarize(name, evaluation, diagnostics))

    per_camera_conflict = {}
    for camera in fixed_cameras:
        if camera == anchor:
            continue
        translation_mm, rotation_deg = pose_delta(
            fitted_cameras["opencv_board"][camera],
            fitted_cameras["opencv_cube"][camera])
        per_camera_conflict[str(camera)] = {
            "translation_mm": float(translation_mm),
            "rotation_deg": float(rotation_deg),
        }
    conflict = {
        "definition": (
            "pose_delta(T_anchor_camera_from_board_train, "
            "T_anchor_camera_from_cube_train)"),
        "uses_robot_fk": False,
        "per_camera": per_camera_conflict,
        "translation_rmse_mm": float(np.sqrt(np.mean([
            value["translation_mm"] ** 2
            for value in per_camera_conflict.values()]))),
        "rotation_rmse_deg": float(np.sqrt(np.mean([
            value["rotation_deg"] ** 2
            for value in per_camera_conflict.values()]))),
    }

    payload = {
        "artifact_schema": "opencv_relative_reference_baseline_v1",
        "protocol": {
            "role": "independent_FK_free_relative_pose_reference_baseline",
            "reporting_tier": "independent_reference",
            "relative_pose_reporting": RELATIVE_POSE_REPORTING_CONTRACT,
            "independent_reference_contract": RELATIVE_POSE_REPORTING_CONTRACT[
                "independent_reference_baseline"],
            "not_a_sota_claim": True,
            "calibration": (
                "OpenCV measurement-only PnP direct anchor-camera relative "
                "transforms plus preregistered MAD-trimmed SE3 average"),
            "uses_joint_optimizer": False,
            "uses_fitted_main_method_camera_poses": False,
            "uses_robot_fk": False,
            "uses_handeye": False,
            "uses_shared_target_pose": False,
            "test_time_refit": False,
            "fixed_camera_ids": fixed_cameras,
            "gauge_anchor_camera": anchor,
            "split": prepared.split,
            "source_data_provenance": prepared.source_data_provenance,
            "board_metric_scale": prepared.board_metric_scale,
            "evaluation_mask_sha256": evaluation_mask[
                "evaluation_mask_sha256"],
        },
        "results": results,
        "board_vs_cube_relative_transform_conflict": conflict,
        "summary": summary,
    }
    validate_payload(payload)
    os.makedirs(args.out_dir, exist_ok=True)
    json_path = os.path.join(args.out_dir, "opencv_relative_baseline.json")
    csv_path = os.path.join(args.out_dir, "opencv_relative_baseline.csv")
    with open(json_path, "w") as handle:
        json.dump(jsonable(payload), handle, indent=2)
    with open(csv_path, "w", newline="") as handle:
        writer = csv.DictWriter(
            handle, fieldnames=list(summary[0]), lineterminator="\n")
        writer.writeheader()
        writer.writerows(summary)
    with open(os.path.join(args.out_dir, "OPENCV_RELATIVE_BASELINE.md"), "w") as handle:
        handle.write(markdown_report(summary, conflict))
    print(f"[DONE] {json_path}")


if __name__ == "__main__":
    main()
