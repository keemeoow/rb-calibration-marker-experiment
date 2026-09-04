#!/usr/bin/env python3
"""Render robot-base point-cloud diagnostics from aligned Session04 depth."""

from __future__ import annotations

import argparse
import csv
import json
import sys
from collections import defaultdict
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

import cv2
import numpy as np
from PIL import Image, ImageDraw

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from calibration_pipeline.observations import (  # noqa: E402
    load_pixel_observations_from_manifest,
)
from calibration_pipeline.runtime import load_intrinsics_with_depth_scale  # noqa: E402
from tools.make_calibration_result_presentation import (  # noqa: E402
    BLUE,
    GREEN,
    INK,
    LINE,
    MUTED,
    NAVY,
    ORANGE,
    PANEL,
    PAPER,
    PURPLE,
    RED,
    TEAL,
    draw_wrapped,
    font,
    rounded,
)


DEFAULT_OUT_DIR = ROOT / "CP_result/session04/robot_base_pointcloud"
DEFAULT_MATRIX_JSON = ROOT / "CP_result/session04/late_table1/calibration_matrices.json"
DEFAULT_MANIFEST = (
    ROOT / "data/session04/calib_out/capture_filter/Step2b_observation_manifest.json"
)
DEFAULT_SESSION_ROOT = ROOT / "data/session04/calib_train"
DEFAULT_INTRINSICS = ROOT / "intrinsics"

CAMERA_COLORS = {
    0: "#2563EB",
    1: "#F97316",
    2: "#7C3AED",
    3: "#059669",
}
TARGET_COLORS = {"board": "#0F766E", "cube": "#D97706"}
METHOD_ORDER = ("A0", "A1", "A2", "A3", "A4", "A5", "B1", "B2", "B3")
METHOD_TARGETS = {
    "A0": ("board",),
    "A1": ("board", "cube"),
    "A2": ("board", "cube"),
    "A3": ("board", "cube"),
    "A4": ("board", "cube"),
    "A5": ("board", "cube"),
    "B1": ("board", "cube"),
    "B2": ("cube",),
    "B3": ("board",),
}
METHOD_ROLES = {
    "A0": "board-only sequential baseline",
    "A1": "board+cube sequential",
    "A2": "board+cube unified internal main",
    "A3": "raw-FK hard fixed diagnostic",
    "A4": "soft-FK preflight",
    "A5": "vision-aligned FK hard post-hoc",
    "B1": "-Unified soft-FK baseline",
    "B2": "-board cube-only soft-FK",
    "B3": "-cube board-only unified",
}


@dataclass(frozen=True)
class CloudRecord:
    method: str
    event_id: int
    set_idx: int | None
    target: str
    camera_id: int
    points_base_m: np.ndarray
    residual_mm: np.ndarray
    accepted_mask: np.ndarray
    model_polygons_base_m: list[np.ndarray]


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


def _load_method_transforms(matrix_json: Path, method: str, seed: int) -> dict:
    payload = json.loads(matrix_json.read_text(encoding="utf-8"))
    row = payload["rows"][method]
    runs = row["runs"]
    run = next((item for item in runs if int(item.get("seed", -1)) == seed), runs[0])
    transforms = run["transforms"]
    return {
        "T_base_Ci": {
            int(camera): np.asarray(matrix, dtype=np.float64)
            for camera, matrix in transforms["T_base_Ci"].items()
        },
        "T_gripper_cam": np.asarray(transforms["T_gripper_cam"], dtype=np.float64),
        "T_base_board": np.asarray(transforms["T_base_board"], dtype=np.float64),
        "T_base_cube_by_set": {
            int(set_idx): np.asarray(matrix, dtype=np.float64)
            for set_idx, matrix in transforms["T_base_cube_by_set"].items()
        },
    }


def _parse_methods(value: str, matrix_json: Path) -> list[str]:
    payload = json.loads(matrix_json.read_text(encoding="utf-8"))
    available = set(payload["rows"])
    requested = [item.strip() for item in value.replace(",", " ").split()
                 if item.strip()]
    if not requested or requested == ["all"]:
        requested = [method for method in METHOD_ORDER if method in available]
    if any(item.lower() == "all" for item in requested):
        requested = [method for method in METHOD_ORDER if method in available]
    unknown = [method for method in requested if method not in available]
    if unknown:
        raise ValueError(f"unknown methods in {matrix_json}: {unknown}")
    return requested


def _parse_targets(value: str) -> set[str]:
    targets = {item.strip() for item in value.replace(",", " ").split()
               if item.strip()}
    if not targets or "all" in targets:
        return {"board", "cube"}
    unknown = targets - {"board", "cube"}
    if unknown:
        raise ValueError(f"unknown targets: {sorted(unknown)}")
    return targets


def _method_sort_key(method: str) -> tuple[int, str]:
    try:
        return METHOD_ORDER.index(method), method
    except ValueError:
        return len(METHOD_ORDER), method


def _transform_points(T: np.ndarray, points: np.ndarray) -> np.ndarray:
    points = np.asarray(points, dtype=np.float64).reshape(-1, 3)
    return (T[:3, :3] @ points.T).T + T[:3, 3]


def _plane_from_points(points: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    points = np.asarray(points, dtype=np.float64).reshape(-1, 3)
    center = np.mean(points, axis=0)
    _, _, vh = np.linalg.svd(points - center, full_matrices=False)
    normal = vh[-1]
    norm = float(np.linalg.norm(normal))
    if norm <= 1e-12:
        raise ValueError("degenerate model plane")
    normal = normal / norm
    return center, normal


def _camera_transform(method_transforms: dict, capture: dict, camera_id: int) -> np.ndarray:
    if camera_id in method_transforms["T_base_Ci"]:
        return method_transforms["T_base_Ci"][camera_id]
    if "robot_pose_matrix_4x4" not in capture:
        raise KeyError(f"event {capture.get('event_id')} has no robot pose")
    return (
        np.asarray(capture["robot_pose_matrix_4x4"], dtype=np.float64)
        @ method_transforms["T_gripper_cam"]
    )


def _target_transform(method_transforms: dict, target: str, set_idx: int | None) -> np.ndarray:
    if target == "board":
        return method_transforms["T_base_board"]
    if set_idx is None or int(set_idx) not in method_transforms["T_base_cube_by_set"]:
        raise KeyError(f"no cube pose for set {set_idx}")
    return method_transforms["T_base_cube_by_set"][int(set_idx)]


def _polygon_mask(
    image_shape: tuple[int, int],
    polygon: np.ndarray,
    erode_px: int,
) -> np.ndarray:
    height, width = image_shape
    mask = np.zeros((height, width), dtype=np.uint8)
    pts = np.round(np.asarray(polygon, dtype=np.float64)).astype(np.int32)
    if len(pts) < 3:
        return mask
    cv2.fillConvexPoly(mask, pts.reshape(-1, 2), 1)
    if erode_px > 0:
        kernel = np.ones((erode_px, erode_px), dtype=np.uint8)
        mask = cv2.erode(mask, kernel, iterations=1)
    return mask


def _backproject_depth(
    depth: np.ndarray,
    depth_scale: float,
    K: np.ndarray,
    D: np.ndarray,
    pixels_uv: np.ndarray,
) -> np.ndarray:
    pixels_uv = np.asarray(pixels_uv, dtype=np.float64).reshape(-1, 2)
    u = np.clip(np.round(pixels_uv[:, 0]).astype(int), 0, depth.shape[1] - 1)
    v = np.clip(np.round(pixels_uv[:, 1]).astype(int), 0, depth.shape[0] - 1)
    z = depth[v, u].astype(np.float64) * float(depth_scale)
    valid = np.isfinite(z) & (z > 0.05) & (z < 3.0)
    pixels_uv = pixels_uv[valid]
    z = z[valid]
    if len(z) == 0:
        return np.empty((0, 3), dtype=np.float64)
    normalized = cv2.undistortPoints(
        pixels_uv.reshape(-1, 1, 2),
        np.asarray(K, dtype=np.float64),
        np.asarray(D, dtype=np.float64),
    ).reshape(-1, 2)
    return np.column_stack([normalized[:, 0] * z, normalized[:, 1] * z, z])


def _sample_mask(mask: np.ndarray, stride: int, max_points: int) -> np.ndarray:
    ys, xs = np.where(mask > 0)
    if len(xs) == 0:
        return np.empty((0, 2), dtype=np.float64)
    keep = ((xs % stride) == 0) & ((ys % stride) == 0)
    xs, ys = xs[keep], ys[keep]
    if len(xs) > max_points:
        index = np.linspace(0, len(xs) - 1, max_points).astype(int)
        xs, ys = xs[index], ys[index]
    return np.column_stack([xs, ys]).astype(np.float64)


def _iter_surface_blocks(target: str, image_points: np.ndarray, object_points: np.ndarray):
    image_points = np.asarray(image_points, dtype=np.float64).reshape(-1, 2)
    object_points = np.asarray(object_points, dtype=np.float64).reshape(-1, 3)
    if target == "board":
        hull = cv2.convexHull(image_points.astype(np.float32)).reshape(-1, 2)
        yield hull, object_points
        return
    for start in range(0, len(image_points), 4):
        img_block = image_points[start:start + 4]
        obj_block = object_points[start:start + 4]
        if len(img_block) == 4 and len(obj_block) == 4:
            yield img_block, obj_block


def _record_cloud(
    *,
    method: str,
    event_id: int,
    observation,
    capture: dict,
    method_transforms: dict,
    K_map: dict[int, np.ndarray],
    D_map: dict[int, np.ndarray],
    depth_scale_map: dict[int, float],
    session_root: Path,
    max_plane_abs_mm: float,
    stride: int,
    max_points_per_surface: int,
    erode_px: int,
) -> CloudRecord | None:
    camera_id = int(observation.cam)
    camera_info = capture.get("cams", {}).get(str(camera_id), {})
    depth_path = camera_info.get("depth_path")
    if not depth_path:
        return None
    depth = cv2.imread(str(session_root / depth_path), cv2.IMREAD_UNCHANGED)
    if depth is None:
        return None
    T_base_camera = _camera_transform(method_transforms, capture, camera_id)
    T_base_target = _target_transform(
        method_transforms, str(observation.marker), observation.set_idx)

    all_points_base = []
    all_residuals = []
    all_accepted = []
    model_polygons = []
    for polygon, object_block in _iter_surface_blocks(
            str(observation.marker), observation.image_points,
            observation.object_points):
        mask = _polygon_mask(depth.shape, polygon, erode_px)
        pixels = _sample_mask(mask, stride, max_points_per_surface)
        if len(pixels) == 0:
            continue
        points_camera = _backproject_depth(
            depth, depth_scale_map[camera_id], K_map[camera_id],
            D_map[camera_id], pixels)
        if len(points_camera) == 0:
            continue
        points_base = _transform_points(T_base_camera, points_camera)
        model_points = _transform_points(T_base_target, object_block)
        plane_point, plane_normal = _plane_from_points(model_points)
        residual_mm = (points_base - plane_point) @ plane_normal * 1000.0
        accepted = np.abs(residual_mm) <= float(max_plane_abs_mm)
        all_points_base.append(points_base)
        all_residuals.append(residual_mm)
        all_accepted.append(accepted)
        model_polygons.append(model_points)

    if not all_points_base:
        return None
    return CloudRecord(
        method=method,
        event_id=int(event_id),
        set_idx=None if observation.set_idx is None else int(observation.set_idx),
        target=str(observation.marker),
        camera_id=camera_id,
        points_base_m=np.concatenate(all_points_base, axis=0),
        residual_mm=np.concatenate(all_residuals, axis=0),
        accepted_mask=np.concatenate(all_accepted, axis=0),
        model_polygons_base_m=model_polygons,
    )


def _metric_row(record: CloudRecord) -> dict:
    accepted = record.accepted_mask
    residual = record.residual_mm[accepted]
    points = record.points_base_m[accepted]
    total = int(len(record.residual_mm))
    if len(residual) == 0:
        return {
            "method": record.method,
            "event_id": record.event_id,
            "set_idx": record.set_idx,
            "target": record.target,
            "camera_id": record.camera_id,
            "raw_samples": total,
            "accepted_samples": 0,
            "accepted_ratio": 0.0,
        }
    return {
        "method": record.method,
        "event_id": record.event_id,
        "set_idx": record.set_idx,
        "target": record.target,
        "camera_id": record.camera_id,
        "raw_samples": total,
        "accepted_samples": int(len(residual)),
        "accepted_ratio": float(len(residual) / max(total, 1)),
        "signed_median_mm": float(np.median(residual)),
        "abs_median_mm": float(np.median(np.abs(residual))),
        "signed_rmse_mm": float(np.sqrt(np.mean(residual ** 2))),
        "abs_p95_mm": float(np.percentile(np.abs(residual), 95)),
        "centroid_base_m": np.mean(points, axis=0).tolist(),
    }


def _summary_rows(rows: list[dict]) -> list[dict]:
    grouped = defaultdict(list)
    for row in rows:
        if int(row.get("accepted_samples", 0)) > 0:
            grouped[(row["method"], row["target"])].append(row)
    output = []
    for (method, target), items in sorted(
            grouped.items(), key=lambda item: (_method_sort_key(item[0][0]), item[0][1])):
        weights = np.asarray([row["accepted_samples"] for row in items], dtype=np.float64)
        signed_rmse = np.asarray([row["signed_rmse_mm"] for row in items], dtype=np.float64)
        abs_median = np.asarray([row["abs_median_mm"] for row in items], dtype=np.float64)
        p95 = np.asarray([row["abs_p95_mm"] for row in items], dtype=np.float64)
        output.append({
            "method": method,
            "target": target,
            "observations": int(len(items)),
            "accepted_samples": int(np.sum(weights)),
            "weighted_signed_rmse_mm": float(
                np.sqrt(np.sum(weights * signed_rmse ** 2) / max(np.sum(weights), 1.0))),
            "weighted_abs_median_mm": float(
                np.sum(weights * abs_median) / max(np.sum(weights), 1.0)),
            "weighted_abs_p95_mm": float(
                np.sum(weights * p95) / max(np.sum(weights), 1.0)),
        })
    return output


def _method_summary_rows(target_rows: list[dict]) -> list[dict]:
    by_method = defaultdict(dict)
    for row in target_rows:
        by_method[row["method"]][row["target"]] = row
    output = []
    for method in sorted(by_method, key=_method_sort_key):
        target_map = by_method[method]
        items = list(target_map.values())
        weights = np.asarray([row["accepted_samples"] for row in items], dtype=np.float64)
        rmses = np.asarray([row["weighted_signed_rmse_mm"] for row in items], dtype=np.float64)
        output.append({
            "method": method,
            "role": METHOD_ROLES.get(method, ""),
            "targets": "+".join(METHOD_TARGETS.get(method, tuple(sorted(target_map)))),
            "observations": int(sum(row["observations"] for row in items)),
            "accepted_samples": int(np.sum(weights)),
            "board_rmse_mm": (
                target_map["board"]["weighted_signed_rmse_mm"]
                if "board" in target_map else None),
            "cube_rmse_mm": (
                target_map["cube"]["weighted_signed_rmse_mm"]
                if "cube" in target_map else None),
            "combined_rmse_mm": float(
                np.sqrt(np.sum(weights * rmses ** 2) / max(np.sum(weights), 1.0))),
        })
    return output


def _project_points(
    points_m: np.ndarray,
    dims: tuple[int, int],
) -> np.ndarray:
    points_mm = np.asarray(points_m, dtype=np.float64).reshape(-1, 3) * 1000.0
    if dims == (0, 1):
        return points_mm[:, [0, 1]]
    if dims == (0, 2):
        return points_mm[:, [0, 2]]
    return points_mm[:, [1, 2]]


def _draw_projection_panel(
    draw: ImageDraw.ImageDraw,
    box: tuple[int, int, int, int],
    title: str,
    dims: tuple[int, int],
    records: list[CloudRecord],
    bounds: tuple[np.ndarray, np.ndarray],
) -> None:
    x0, y0, x1, y1 = box
    rounded(draw, box, fill=PAPER, outline=LINE, radius=12)
    draw.text((x0 + 18, y0 + 14), title, font=font(19, "bold"), fill=INK)
    plot = (x0 + 42, y0 + 54, x1 - 22, y1 - 30)
    draw.rectangle(plot, outline="#CBD5E1", width=1)
    lo, hi = bounds
    lo2 = lo[list(dims)]
    hi2 = hi[list(dims)]
    span = np.maximum(hi2 - lo2, 1.0)

    def to_px(points_m: np.ndarray) -> np.ndarray:
        pts = _project_points(points_m, dims)
        px = plot[0] + (pts[:, 0] - lo2[0]) / span[0] * (plot[2] - plot[0])
        py = plot[3] - (pts[:, 1] - lo2[1]) / span[1] * (plot[3] - plot[1])
        return np.column_stack([px, py])

    for record in records:
        for poly in record.model_polygons_base_m:
            pts = to_px(poly)
            xy = [tuple(map(float, point)) for point in pts]
            if len(xy) >= 2:
                draw.line(xy + [xy[0]], fill="#111827", width=2)

    for record in records:
        accepted_points = record.points_base_m[record.accepted_mask]
        if len(accepted_points) == 0:
            continue
        if len(accepted_points) > 1200:
            idx = np.linspace(0, len(accepted_points) - 1, 1200).astype(int)
            accepted_points = accepted_points[idx]
        pts = to_px(accepted_points)
        color = CAMERA_COLORS.get(record.camera_id, "#64748B")
        for px, py in pts:
            draw.ellipse((px - 1.4, py - 1.4, px + 1.4, py + 1.4), fill=color)

    origin = to_px(np.zeros((1, 3), dtype=np.float64))[0]
    if plot[0] <= origin[0] <= plot[2] and plot[1] <= origin[1] <= plot[3]:
        draw.ellipse((origin[0] - 5, origin[1] - 5, origin[0] + 5, origin[1] + 5),
                     fill=RED)
        draw.text((origin[0] + 8, origin[1] - 12), "base", font=font(13), fill=RED)
    draw.text((plot[0], plot[3] + 7), "mm in robot base frame", font=font(12), fill=MUTED)


def _render_projection_image(
    path: Path,
    *,
    method: str,
    event_id: int,
    records: list[CloudRecord],
) -> None:
    img = Image.new("RGB", (1600, 900), PANEL)
    draw = ImageDraw.Draw(img)
    draw.text((64, 44), f"Robot-base point cloud | {method} | Event {event_id:04d}",
              font=font(42, "black"), fill=INK)
    draw_wrapped(
        draw,
        (66, 102),
        "Aligned depth samples inside detected board/cube polygons are transformed into robot-base coordinates. Black outlines are fitted visual target planes.",
        font(22), MUTED, 1380, gap=8, max_lines=2,
    )
    legend_x = 64
    for camera_id in sorted({record.camera_id for record in records}):
        color = CAMERA_COLORS.get(camera_id, "#64748B")
        draw.ellipse((legend_x, 154, legend_x + 18, 172), fill=color)
        draw.text((legend_x + 26, 149), f"cam{camera_id}", font=font(18, "bold"), fill=INK)
        legend_x += 120
    draw.rectangle((legend_x, 158, legend_x + 28, 162), fill="#111827")
    draw.text((legend_x + 38, 149), "visual model plane", font=font(18, "bold"), fill=INK)

    points = [record.points_base_m[record.accepted_mask] for record in records
              if np.any(record.accepted_mask)]
    model = [poly for record in records for poly in record.model_polygons_base_m]
    if not points:
        draw.text((64, 250), "No accepted depth points", font=font(28, "bold"), fill=RED)
        img.save(path)
        return
    all_points = np.concatenate(points + model, axis=0) * 1000.0
    lo = np.percentile(all_points, 2, axis=0) - 35.0
    hi = np.percentile(all_points, 98, axis=0) + 35.0
    bounds = (lo, hi)
    _draw_projection_panel(draw, (64, 205, 1540, 435), "XY top view", (0, 1), records, bounds)
    _draw_projection_panel(draw, (64, 460, 788, 820), "XZ side view", (0, 2), records, bounds)
    _draw_projection_panel(draw, (816, 460, 1540, 820), "YZ side view", (1, 2), records, bounds)
    img.save(path)


def _write_csv(path: Path, rows: list[dict]) -> None:
    fields = [
        "method", "event_id", "set_idx", "target", "camera_id",
        "raw_samples", "accepted_samples", "accepted_ratio",
        "signed_median_mm", "abs_median_mm", "signed_rmse_mm",
        "abs_p95_mm", "centroid_base_m",
    ]
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)


def _write_report(path: Path, payload: dict) -> None:
    method_rows = payload["method_summary"]
    target_rows = payload["summary"]
    event_images = payload["event_images"]
    lines = [
        "# Robot-base Point-cloud Diagnostic",
        "",
        "> **역할:** 8/3 피드백 #17에 대한 현재 데이터 기반 산출물이다. "
        "aligned depth를 각 Table 1 비교실험 row의 calibration transform으로 "
        "robot-base frame에 올려 정합을 시각화한다. 이 depth는 캘리브레이션 "
        "목적함수나 external GT가 아니므로 absolute robot-task accuracy로 "
        "해석하지 않는다.",
        "",
        "## Comparison-row Summary",
        "",
        "| Method | Experiment role | Targets | Obs | Depth samples | Board RMSE | Cube RMSE | Combined RMSE |",
        "| --- | --- | --- | ---: | ---: | ---: | ---: | ---: |",
    ]
    for row in method_rows:
        board = "N/A" if row["board_rmse_mm"] is None else f"{row['board_rmse_mm']:.3f} mm"
        cube = "N/A" if row["cube_rmse_mm"] is None else f"{row['cube_rmse_mm']:.3f} mm"
        lines.append(
            f"| {row['method']} | {row['role']} | {row['targets']} "
            f"| {row['observations']} | {row['accepted_samples']} "
            f"| {board} | {cube} | {row['combined_rmse_mm']:.3f} mm |")
    lines.extend([
        "",
        "## Target-level Detail",
        "",
        "| Method | Target | Obs | Depth samples | Median | RMSE | P95 |",
        "| --- | --- | ---: | ---: | ---: | ---: | ---: |",
    ])
    for row in target_rows:
        lines.append(
            f"| {row['method']} | {row['target']} | {row['observations']} "
            f"| {row['accepted_samples']} "
            f"| {row['weighted_abs_median_mm']:.3f} mm "
            f"| {row['weighted_signed_rmse_mm']:.3f} mm "
            f"| {row['weighted_abs_p95_mm']:.3f} mm |")
    lines.extend([
        "",
        "## Visual Evidence",
        "",
    ])
    for method in sorted(event_images, key=_method_sort_key):
        lines.extend([
            f"### {method} — {METHOD_ROLES.get(method, '')}",
            "",
        ])
        for event_id, image_name in sorted(
                event_images[method].items(), key=lambda item: int(item[0])):
            lines.extend([
                f"#### Event {int(event_id):04d}",
                "",
                f"![Robot-base point cloud {method} event {int(event_id):04d}]({image_name})",
                "",
            ])
    lines.extend([
        "## Reading Rules",
        "",
        "- 각 row는 Table 1의 marker 구성만 평가한다. A0/B3는 board-only, B2는 cube-only다.",
        "- 이 표의 RMSE는 selected target polygon 내부 aligned depth point와 해당 row가 예측한 target plane 사이의 robot-base frame distance다.",
        "- `Combined RMSE`는 board/cube sample 수로 가중한 row 내부 진단값이며, external GT 순위가 아니다.",
        "",
    ])
    lines.extend([
        "## Interpretation",
        "",
        "- 현재 산출물은 point cloud를 **카메라 좌표가 아니라 robot-base 좌표계**에서 표현한다.",
        "- 검은 outline은 각 row의 visual calibration이 예측한 board/cube plane이고, 색 점은 실제 aligned depth point다.",
        "- 큰 깊이/plane 차이는 external GT 오차가 아니라 depth registration, target localization, intrinsic coverage, target surface sampling이 섞인 진단 신호다.",
        "- 따라서 #17은 `비교실험 구성별 diagnostic 구현 완료 / physical GT 아님`으로 상태를 둔다.",
        "",
    ])
    path.write_text("\n".join(lines), encoding="utf-8")


def run(args: argparse.Namespace) -> dict:
    matrix_json = Path(args.matrix_json).resolve()
    methods_arg = args.methods if args.methods else args.method
    methods = _parse_methods(methods_arg, matrix_json)
    session_root = Path(args.session_root).resolve()
    intrinsics_dir = Path(args.intrinsics_dir).resolve()
    manifest = Path(args.manifest).resolve()
    observations, manifest_diag = load_pixel_observations_from_manifest(
        str(manifest),
        policy=args.observation_filter_policy,
        root=str(session_root),
        intrinsics_dir=str(intrinsics_dir),
        validate_sources=True,
        allow_relocated_root=True,
    )
    meta = json.loads((session_root / "meta.json").read_text(encoding="utf-8"))
    capture_by_event = {
        int(capture["event_id"]): capture for capture in meta["captures"]
    }
    camera_ids = sorted({int(camera) for capture in meta["captures"]
                         for camera in capture.get("cams", {})})
    K_map, D_map, depth_scale_map = {}, {}, {}
    for camera_id in camera_ids:
        K_map[camera_id], D_map[camera_id], depth_scale_map[camera_id] = (
            load_intrinsics_with_depth_scale(str(intrinsics_dir), camera_id))

    wanted_events = {
        int(value) for value in str(args.events).replace(",", " ").split()
        if value.strip()
    }
    wanted_targets = _parse_targets(args.targets)
    records = []
    method_targets = {}
    for method in methods:
        method_transforms = _load_method_transforms(matrix_json, method, args.seed)
        active_targets = tuple(
            target for target in METHOD_TARGETS.get(method, ("board", "cube"))
            if target in wanted_targets
        )
        method_targets[method] = active_targets
        for observation in observations:
            event_id = int(observation.event)
            if wanted_events and event_id not in wanted_events:
                continue
            if str(observation.marker) not in active_targets:
                continue
            capture = capture_by_event.get(event_id)
            if capture is None:
                continue
            try:
                record = _record_cloud(
                    method=method,
                    event_id=event_id,
                    observation=observation,
                    capture=capture,
                    method_transforms=method_transforms,
                    K_map=K_map,
                    D_map=D_map,
                    depth_scale_map=depth_scale_map,
                    session_root=session_root,
                    max_plane_abs_mm=args.max_plane_abs_mm,
                    stride=args.stride,
                    max_points_per_surface=args.max_points_per_surface,
                    erode_px=args.erode_px,
                )
            except KeyError:
                continue
            if record is not None:
                records.append(record)

    if not records:
        raise RuntimeError("no point-cloud records could be generated")
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    metric_rows = [_metric_row(record) for record in records]
    summary = _summary_rows(metric_rows)
    method_summary = _method_summary_rows(summary)
    _write_csv(output_dir / "robot_base_pointcloud_metrics.csv", metric_rows)

    by_event = defaultdict(list)
    by_method_event = defaultdict(lambda: defaultdict(list))
    for record in records:
        by_event[int(record.event_id)].append(record)
        by_method_event[record.method][int(record.event_id)].append(record)
    event_images = {}
    for method in sorted(by_method_event, key=_method_sort_key):
        event_images[method] = {}
        for event_id, event_records in sorted(by_method_event[method].items()):
            image_name = f"robot_base_pointcloud_{method}_event{event_id:04d}.png"
            _render_projection_image(
                output_dir / image_name,
                method=method,
                event_id=event_id,
                records=event_records,
            )
            if method == "A2":
                _render_projection_image(
                    output_dir / f"robot_base_pointcloud_event{event_id:04d}.png",
                    method=method,
                    event_id=event_id,
                    records=event_records,
                )
            event_images[method][str(event_id)] = image_name

    payload = {
        "schema": "robot_base_pointcloud_diagnostic_v1",
        "methods": methods,
        "method_targets": {
            method: list(targets) for method, targets in method_targets.items()
        },
        "representative_seed": int(args.seed),
        "role": "diagnostic_only_not_external_gt",
        "depth_role": (
            "aligned depth is transformed to robot-base frame for visual "
            "diagnostics; it is not used as calibration objective or physical GT"),
        "source_manifest": str(manifest),
        "manifest": manifest_diag,
        "events": sorted(by_event),
        "max_plane_abs_mm": float(args.max_plane_abs_mm),
        "records": metric_rows,
        "summary": summary,
        "method_summary": method_summary,
        "event_images": event_images,
    }
    (output_dir / "robot_base_pointcloud_diagnostic.json").write_text(
        json.dumps(_jsonable(payload), indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8")
    _write_report(output_dir / "ROBOT_BASE_POINTCLOUD_DIAGNOSTIC.md", payload)
    return payload


def main(argv: Iterable[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--method", default="A2",
                        help="Single Table 1 row to render when --methods is not set.")
    parser.add_argument("--methods", default=None,
                        help="Comma-separated rows, or 'all' for A0-A5/B1-B3.")
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--events", default="24,54,72")
    parser.add_argument("--targets", default="board,cube")
    parser.add_argument("--matrix-json", default=str(DEFAULT_MATRIX_JSON))
    parser.add_argument("--manifest", default=str(DEFAULT_MANIFEST))
    parser.add_argument("--session-root", default=str(DEFAULT_SESSION_ROOT))
    parser.add_argument("--intrinsics-dir", default=str(DEFAULT_INTRINSICS))
    parser.add_argument("--observation-filter-policy", default="standard")
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUT_DIR)
    parser.add_argument("--stride", type=int, default=5)
    parser.add_argument("--erode-px", type=int, default=5)
    parser.add_argument("--max-points-per-surface", type=int, default=2500)
    parser.add_argument("--max-plane-abs-mm", type=float, default=50.0)
    args = parser.parse_args(list(argv) if argv is not None else None)
    payload = run(args)
    print(args.output_dir / "ROBOT_BASE_POINTCLOUD_DIAGNOSTIC.md")
    n_images = sum(len(images) for images in payload["event_images"].values())
    print(
        f"[DONE] {len(payload['records'])} records, "
        f"{n_images} method/event images, methods={','.join(payload['methods'])}")


if __name__ == "__main__":
    main()
