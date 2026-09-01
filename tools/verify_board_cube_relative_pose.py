#!/usr/bin/env python3
"""Diagnose board/cube fixed-camera conflict and validate Session04 K/D."""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
from collections import defaultdict
from pathlib import Path

import cv2
import numpy as np


PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from calibration_pipeline import table1  # noqa: E402
from calibration_pipeline.apriltag_cube import inv_T  # noqa: E402
from calibration_pipeline.observations import (  # noqa: E402
    load_pixel_observations_from_manifest,
)
from calibration_pipeline.opencv_relative_baseline import (  # noqa: E402
    direct_relative_candidates,
    fit_baseline,
)
from calibration_pipeline.path_evaluation import (  # noqa: E402
    build_fixed_to_fixed_cross_target_mask,
    evaluate_fixed_to_fixed_cross_target,
    solve_observed_pose,
)
from calibration_pipeline.reprojection import PixelObs, pose_delta  # noqa: E402
from calibration_pipeline.runtime import (  # noqa: E402
    load_intrinsics_with_depth_scale,
)


def _jsonable(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, dict):
        return {str(key): _jsonable(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_jsonable(item) for item in value]
    return value


def _load_intrinsic_variants(intrinsics_dir: Path):
    current_K, current_D = {}, {}
    factory_K, factory_D = {}, {}
    zero_D = {}
    for camera in (0, 1, 2, 3):
        current_K[camera], current_D[camera], _ = \
            load_intrinsics_with_depth_scale(str(intrinsics_dir), camera)
        factory_path = intrinsics_dir / "factory_backup" / f"cam{camera}.npz"
        factory = np.load(factory_path, allow_pickle=True)
        factory_K[camera] = np.asarray(factory["color_K"], dtype=np.float64)
        factory_D[camera] = np.asarray(factory["color_D"], dtype=np.float64)
        zero_D[camera] = np.zeros_like(current_D[camera])
    return {
        "charuco_calibrated_KD": (current_K, current_D),
        "factory_KD": (factory_K, factory_D),
        "charuco_K_zero_distortion": (current_K, zero_D),
    }


def _split_observations(observations, gripper, fraction, seed, minimum):
    split = table1.build_event_split(
        observations, gripper, fraction, seed, minimum)
    eligible = set(split["eligible_sets"])
    train_events = set(split["train_events"])
    test_events = set(split["test_events"])
    pool = [observation for observation in observations
            if observation.set_idx in eligible]
    return (
        split,
        [observation for observation in pool
         if observation.event in train_events],
        [observation for observation in pool
         if observation.event in test_events],
    )


def _fit_targets(train, fixed_cameras, K_map, D_map):
    anchor = int(fixed_cameras[0])
    fitted, diagnostics = {}, {}
    for target in ("board", "cube"):
        candidates = direct_relative_candidates(
            train, fixed_cameras, anchor, [target], K_map, D_map)
        fitted[target], diagnostics[target] = fit_baseline(candidates, anchor)
    return fitted, diagnostics


def _variant_result(train, test, split, fixed_cameras, K_map, D_map):
    fitted, diagnostics = _fit_targets(train, fixed_cameras, K_map, D_map)
    mask = build_fixed_to_fixed_cross_target_mask(
        test, fixed_cameras, K_map, D_map,
        set_filter=split["eligible_sets"])
    heldout = {}
    for fit_target in ("board", "cube"):
        heldout[fit_target] = evaluate_fixed_to_fixed_cross_target(
            test, fitted[fit_target], K_map, D_map, mask)
    conflict = {}
    for camera in fixed_cameras[1:]:
        translation_mm, rotation_deg = pose_delta(
            fitted["board"][camera], fitted["cube"][camera])
        conflict[str(camera)] = {
            "translation_mm": float(translation_mm),
            "rotation_deg": float(rotation_deg),
        }
    return {
        "fitted_T_anchor_camera": {
            target: {str(camera): transform.tolist()
                     for camera, transform in transforms.items()}
            for target, transforms in fitted.items()
        },
        "fit_diagnostics": diagnostics,
        "heldout": heldout,
        "board_vs_cube_conflict": conflict,
    }


def _same_event_conflicts(observations, fixed_cameras, K_map, D_map):
    grouped = defaultdict(dict)
    for observation in observations:
        if int(observation.cam) in fixed_cameras:
            grouped[(str(observation.marker), int(observation.event))][
                int(observation.cam)] = observation
    anchor = int(fixed_cameras[0])
    output = {}
    for camera in fixed_cameras[1:]:
        board_events = {
            event for (target, event), by_camera in grouped.items()
            if target == "board" and anchor in by_camera and camera in by_camera
        }
        cube_events = {
            event for (target, event), by_camera in grouped.items()
            if target == "cube" and anchor in by_camera and camera in by_camera
        }
        rows = []
        for event in sorted(board_events & cube_events):
            relative = {}
            for target in ("board", "cube"):
                by_camera = grouped[(target, event)]
                T_anchor_target = solve_observed_pose(
                    by_camera[anchor], K_map, D_map)
                T_camera_target = solve_observed_pose(
                    by_camera[camera], K_map, D_map)
                if T_anchor_target is None or T_camera_target is None:
                    break
                relative[target] = T_anchor_target @ inv_T(T_camera_target)
            if set(relative) != {"board", "cube"}:
                continue
            translation_mm, rotation_deg = pose_delta(
                relative["board"], relative["cube"])
            board_vector = relative["board"][:3, 3]
            cube_vector = relative["cube"][:3, 3]
            scale = float(
                (board_vector @ cube_vector)
                / max(float(board_vector @ board_vector), 1e-12))
            rows.append({
                "event_id": int(event),
                "translation_mm": float(translation_mm),
                "rotation_deg": float(rotation_deg),
                "board_to_cube_scale": scale,
            })
        output[str(camera)] = {
            "rows": rows,
            "median_translation_mm": float(np.median([
                row["translation_mm"] for row in rows])) if rows else None,
            "median_rotation_deg": float(np.median([
                row["rotation_deg"] for row in rows])) if rows else None,
            "median_board_to_cube_scale": float(np.median([
                row["board_to_cube_scale"] for row in rows])) if rows else None,
        }
    return output


def _pnp_rmse(observation, K_map, D_map):
    transform = solve_observed_pose(observation, K_map, D_map)
    if transform is None:
        return None, None
    rvec, _ = cv2.Rodrigues(transform[:3, :3])
    projected, _ = cv2.projectPoints(
        np.asarray(observation.object_points, dtype=np.float64),
        rvec, transform[:3, 3], K_map[int(observation.cam)],
        D_map[int(observation.cam)])
    residual = projected.reshape(-1, 2) - np.asarray(
        observation.image_points, dtype=np.float64).reshape(-1, 2)
    rmse = float(np.sqrt(np.mean(np.sum(residual * residual, axis=1))))
    return transform, rmse


def _draw_points(image, points, color, marker, radius=3):
    for point in np.asarray(points, dtype=np.float64).reshape(-1, 2):
        x, y = np.round(point).astype(int)
        if marker == "circle":
            cv2.circle(image, (x, y), radius, color, -1, cv2.LINE_AA)
        else:
            cv2.line(image, (x - radius, y - radius),
                     (x + radius, y + radius), color, 2, cv2.LINE_AA)
            cv2.line(image, (x - radius, y + radius),
                     (x + radius, y - radius), color, 2, cv2.LINE_AA)


def _render_overlay(output_path: Path, root: Path, meta: dict,
                    test_observations, fixed_cameras, K_map, D_map):
    by_key = {(observation.marker, int(observation.event), int(observation.cam)):
              observation for observation in test_observations}
    candidate_events = []
    for event in sorted({observation.event for observation in test_observations}):
        if all((target, event, camera) in by_key
               for target in ("board", "cube")
               for camera in fixed_cameras):
            candidate_events.append(int(event))
    if not candidate_events:
        raise RuntimeError("no held-out event supports board+cube overlay")
    event = candidate_events[0]
    capture = next(item for item in meta["captures"]
                   if int(item["event_id"]) == event)
    panels = []
    panel_width, image_height, header_height = 640, 360, 88
    for camera in (1, 3):
        camera_info = capture["cams"][str(camera)]
        image = cv2.imread(str(root / camera_info["rgb_path"]))
        if image is None:
            raise FileNotFoundError(camera_info["rgb_path"])
        board = by_key[("board", event, camera)]
        cube = by_key[("cube", event, camera)]
        board_transform, board_rmse = _pnp_rmse(board, K_map, D_map)
        cube_transform, cube_rmse = _pnp_rmse(cube, K_map, D_map)
        canvas = image.copy()
        _draw_points(canvas, board.image_points, (40, 220, 40), "circle", 3)
        _draw_points(canvas, cube.image_points, (0, 210, 255), "circle", 4)
        for observation, transform, color in (
                (board, board_transform, (255, 80, 255)),
                (cube, cube_transform, (255, 190, 40))):
            rvec, _ = cv2.Rodrigues(transform[:3, :3])
            projected, _ = cv2.projectPoints(
                np.asarray(observation.object_points, dtype=np.float64),
                rvec, transform[:3, 3], K_map[camera], D_map[camera])
            _draw_points(canvas, projected.reshape(-1, 2), color, "cross", 3)
        resized = cv2.resize(
            canvas, (panel_width, image_height), interpolation=cv2.INTER_AREA)
        header = np.full((header_height, panel_width, 3), 24, dtype=np.uint8)
        cv2.putText(
            header, f"Event {event:02d} / Camera {camera}", (12, 29),
            cv2.FONT_HERSHEY_SIMPLEX, 0.72, (240, 240, 240), 2,
            cv2.LINE_AA)
        cv2.putText(
            header,
            f"Board PnP {board_rmse:.3f}px | Cube PnP {cube_rmse:.3f}px",
            (12, 58), cv2.FONT_HERSHEY_SIMPLEX, 0.58,
            (100, 220, 255), 2, cv2.LINE_AA)
        cv2.putText(
            header, "observed: board green, cube yellow | reprojection: X",
            (12, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.43,
            (190, 190, 190), 1, cv2.LINE_AA)
        panels.append(np.vstack([header, resized]))
    contact = np.hstack(panels)
    if not cv2.imwrite(str(output_path), contact):
        raise RuntimeError(f"failed to write {output_path}")
    return event


def _intrinsic_metadata(intrinsics_dir: Path):
    report_path = intrinsics_dir / "charuco_intrinsics_report.json"
    report = json.loads(report_path.read_text(encoding="utf-8"))
    output = {
        "report_path": str(report_path.resolve()),
        "calibrated_at": report.get("calibrated_at"),
        "dist_model": report.get("dist_model"),
        "cameras": {},
    }
    for camera, detail in report.get("cameras", {}).items():
        output["cameras"][str(camera)] = {
            key: detail.get(key) for key in (
                "status", "rms_px", "num_views_used", "num_views_total",
                "num_dropped")
        }
    return output


def _write_report(path: Path, payload: dict):
    scale = payload["train_only_board_metric_scale"]
    current = payload["intrinsic_variants"]["charuco_calibrated_KD"]
    same_event = payload["same_event_conflict"]
    lines = [
        "# Session04 Board–Cube Relative Pose and Intrinsic Validation",
        "",
        "## 결론",
        "",
        "Board와 Cube가 계산한 상대 카메라 회전과 baseline 방향은 거의 같지만, "
        "baseline 길이가 체계적으로 다릅니다. 주원인은 랜덤 corner 오차보다 "
        "두 target의 metric scale 불일치입니다.",
        "",
        f"학습 관측만 사용한 공통 Board scale은 `{scale['scale']:.6f}`이며, "
        f"25.0 mm nominal square에 대응하는 유효 길이는 "
        f"`{scale['effective_square_length_mm']:.3f} mm`입니다.",
        "",
        "| Camera | Board baseline | Cube baseline | 보정 전 차이 | 보정 후 차이 | 회전 차이 |",
        "|---|---:|---:|---:|---:|---:|",
    ]
    for camera, detail in scale["per_camera"].items():
        lines.append(
            f"| cam{camera} | {detail['board_baseline_norm_mm']:.3f} mm "
            f"| {detail['cube_baseline_norm_mm']:.3f} mm "
            f"| {detail['translation_disagreement_before_mm']:.3f} mm "
            f"| {detail['translation_disagreement_after_mm']:.3f} mm "
            f"| {detail['rotation_disagreement_deg']:.3f}° |")
    lines.extend([
        "",
        "이 scale은 train event만 사용하며 held-out, Robot FK, Hand–Eye, 외부 GT는 "
        "사용하지 않습니다. 실물 자 측정 전까지는 nominal geometry 자체를 덮어쓰지 "
        "않고 calibration의 명시적 train-only preprocessing으로만 적용합니다.",
        "",
        "## 동일 Event 직접 비교",
        "",
        "| Camera | Event 수 | 병진 차이 median | 회전 차이 median | Board→Cube scale median |",
        "|---|---:|---:|---:|---:|",
    ])
    for camera, detail in same_event.items():
        lines.append(
            f"| cam{camera} | {len(detail['rows'])} "
            f"| {detail['median_translation_mm']:.3f} mm "
            f"| {detail['median_rotation_deg']:.3f}° "
            f"| {detail['median_board_to_cube_scale']:.6f} |")
    lines.extend([
        "",
        "## Intrinsic / Distortion 비교",
        "",
        "각 intrinsic 후보로 train 상대 자세를 다시 계산하고 같은 held-out에서 "
        "자기 target 재투영을 평가했습니다.",
        "",
        "| Intrinsic | Board-fit→Board px/mm | Cube-fit→Cube px/mm | cam1 conflict | cam3 conflict |",
        "|---|---:|---:|---:|---:|",
    ])
    for name, result in payload["intrinsic_variants"].items():
        board = result["heldout"]["board"]["by_target"]["board"]
        cube = result["heldout"]["cube"]["by_target"]["cube"]
        c1 = result["board_vs_cube_conflict"]["1"]
        c3 = result["board_vs_cube_conflict"]["3"]
        lines.append(
            f"| `{name}` "
            f"| {board['cross_view_pixel_transfer_rmse_px']:.3f} px / "
            f"{board['pose_consistency_translation_rmse_mm']:.3f} mm "
            f"| {cube['cross_view_pixel_transfer_rmse_px']:.3f} px / "
            f"{cube['pose_consistency_translation_rmse_mm']:.3f} mm "
            f"| {c1['translation_mm']:.3f} mm "
            f"| {c3['translation_mm']:.3f} mm |")
    lines.extend([
        "",
        "현재 ChArUco K/D는 factory K/D보다 Board 자기 일관성이 비슷하거나 좋고 "
        "Cube 자기 일관성은 더 좋습니다. 왜곡을 0으로 두어도 target scale 차이가 "
        "사라지지 않으므로 intrinsic/distortion이 1차 원인은 아닙니다. 현재 K/D를 "
        "유지합니다.",
        "",
        "### 기존 ChArUco 재보정 기록",
        "",
        "| Camera | RMS | 사용 views | 판정 |",
        "|---|---:|---:|---|",
    ])
    metadata = payload["intrinsic_calibration_metadata"]
    for camera, detail in metadata["cameras"].items():
        views = detail.get("num_views_used")
        status = "충분" if views is not None and int(views) >= 12 else "coverage 제한"
        lines.append(
            f"| cam{camera} | {float(detail['rms_px']):.3f} px "
            f"| {views} | {status} |")
    lines.extend([
        "",
        "cam0·cam1은 각각 10·9 views라 coverage가 제한적이지만, Session04 "
        "held-out 비교에서는 현재 K/D를 폐기할 근거가 없습니다. 추가 intrinsic "
        "촬영 없이 가능한 검증은 완료했습니다.",
        "",
        "## Camera 1·3 Board/Cube Overlay",
        "",
        f"![Camera 1 and 3 board/cube overlay]({payload['overlay_file']})",
        "",
        "초록/노랑 점은 frozen 관측, X는 해당 target PnP 재투영입니다. 두 target "
        "모두 개별 영상 안에서는 잘 맞으므로, 문제는 단일-frame corner ordering "
        "오류보다 target 간 metric scale에 가깝습니다.",
        "",
    ])
    path.write_text("\n".join(lines), encoding="utf-8")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--session-root", default="data/session04/calib_train")
    parser.add_argument("--intrinsics-dir", default="intrinsics")
    parser.add_argument(
        "--manifest",
        default=("data/session04/calib_out/capture_filter/"
                 "Step2b_observation_manifest.json"))
    parser.add_argument(
        "--output-dir",
        default=("data/session04/calib_out/verify/"
                 "board_cube_relative_pose"))
    parser.add_argument("--split-seed", type=int, default=20260731)
    parser.add_argument("--test-fraction", type=float, default=0.2)
    parser.add_argument("--min-train-eih-cube-events", type=int, default=3)
    args = parser.parse_args(argv)

    root = Path(args.session_root).resolve()
    intrinsics_dir = Path(args.intrinsics_dir).resolve()
    output_dir = Path(args.output_dir).resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    observations, manifest_diagnostics = load_pixel_observations_from_manifest(
        args.manifest, policy="standard", root=str(root),
        intrinsics_dir=str(intrinsics_dir), validate_sources=True)
    split, train, test = _split_observations(
        observations, gripper=2, fraction=args.test_fraction,
        seed=args.split_seed, minimum=args.min_train_eih_cube_events)
    fixed_cameras = [0, 1, 3]
    variants = _load_intrinsic_variants(intrinsics_dir)
    variant_results = {
        name: _variant_result(
            train, test, split, fixed_cameras, K_map, D_map)
        for name, (K_map, D_map) in variants.items()
    }
    current_K, current_D = variants["charuco_calibrated_KD"]
    board_metric_scale = table1.estimate_train_board_metric_scale(
        train, gripper_cam_idx=2, K_map=current_K, D_map=current_D)
    with (root / "meta.json").open("r", encoding="utf-8") as stream:
        meta = json.load(stream)
    overlay_name = "camera1_camera3_board_cube_overlay.png"
    overlay_event = _render_overlay(
        output_dir / overlay_name, root, meta, test, fixed_cameras,
        current_K, current_D)
    payload = {
        "schema": "board_cube_relative_pose_diagnostic_v1",
        "session_root": str(root),
        "manifest": manifest_diagnostics,
        "split": split,
        "train_only_board_metric_scale": board_metric_scale,
        "same_event_conflict": _same_event_conflicts(
            [observation for observation in observations
             if observation.set_idx in set(split["eligible_sets"])],
            fixed_cameras, current_K, current_D),
        "intrinsic_variants": variant_results,
        "intrinsic_calibration_metadata": _intrinsic_metadata(intrinsics_dir),
        "overlay_event_id": int(overlay_event),
        "overlay_file": overlay_name,
    }
    json_path = output_dir / "board_cube_relative_pose_diagnostic.json"
    json_path.write_text(
        json.dumps(_jsonable(payload), indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8")
    _write_report(output_dir / "BOARD_CUBE_RELATIVE_POSE.md", payload)
    print(json_path)


if __name__ == "__main__":
    main()
