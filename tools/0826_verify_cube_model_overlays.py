#!/usr/bin/env python3
"""Render session capture overlays that audit the AprilTag cube model."""

from __future__ import annotations

import argparse
import json
import os
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Tuple

import cv2
import matplotlib
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from mpl_toolkits.mplot3d.art3d import Poly3DCollection  # noqa: E402

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from calibration_pipeline.apriltag_cube import AprilTagCubeTarget
from calibration_pipeline.config import (
    CUBE_BODY_BOTTOM_Z_M,
    CUBE_BODY_HEIGHT_M,
    CUBE_BODY_TOP_Z_M,
)
from calibration_pipeline.cube_config import load_cube_config_from_meta
from calibration_pipeline.runtime import load_intrinsics_with_depth_scale


DETECTED = (40, 240, 70)
PROJECTED = (255, 80, 255)
NORMAL = (0, 170, 255)
WHITE = (245, 245, 245)
BLACK = (15, 15, 15)
CORNER_COLORS = ((0, 215, 255), (255, 170, 30), (90, 255, 90), (80, 80, 255))
ID_COLORS_RGB = {
    0: (0.95, 0.64, 0.10),
    1: (0.94, 0.32, 0.22),
    2: (0.20, 0.66, 0.94),
    3: (0.18, 0.76, 0.48),
    4: (0.58, 0.42, 0.94),
    5: (0.95, 0.35, 0.70),
}


@dataclass
class CameraPose:
    rvec: np.ndarray
    tvec: np.ndarray
    rmse_px: float
    used_ids: List[int]


def _put_label(image, text, origin, scale=0.55, color=WHITE, background=BLACK):
    x, y = int(origin[0]), int(origin[1])
    font = cv2.FONT_HERSHEY_SIMPLEX
    thickness = max(1, int(round(scale * 2.0)))
    (width, height), baseline = cv2.getTextSize(text, font, scale, thickness)
    cv2.rectangle(
        image,
        (x - 4, y - height - 5),
        (x + width + 4, y + baseline + 4),
        background,
        -1,
        cv2.LINE_AA,
    )
    cv2.putText(image, text, (x, y), font, scale, color, thickness, cv2.LINE_AA)


def _project(points_3d, pose: CameraPose, K, D):
    points, _ = cv2.projectPoints(
        np.asarray(points_3d, dtype=np.float64).reshape(-1, 3),
        pose.rvec,
        pose.tvec,
        K,
        D,
    )
    return points.reshape(-1, 2)


def _solve_camera_pose(target, camera_info, K, D):
    corners, ids = [], []
    for marker in camera_info.get("markers") or []:
        corners.append(np.asarray(marker["corners_2d"], dtype=np.float64).reshape(1, 4, 2))
        ids.append(int(marker["marker_id"]))
    object_points, image_points, used = target.build_correspondences(
        corners, ids, min_markers=1, min_aspect=0.0
    )
    if object_points is None:
        raise RuntimeError("capture contains no configured cube markers")
    ok, rvec, tvec = cv2.solvePnP(
        object_points,
        image_points,
        K,
        D,
        flags=cv2.SOLVEPNP_ITERATIVE,
    )
    if not ok:
        raise RuntimeError("multi-marker solvePnP failed")
    projected, _ = cv2.projectPoints(object_points, rvec, tvec, K, D)
    residual = projected.reshape(-1, 2) - image_points.reshape(-1, 2)
    rmse = float(np.sqrt(np.mean(np.sum(residual * residual, axis=1))))
    return CameraPose(rvec=rvec, tvec=tvec, rmse_px=rmse, used_ids=used)


def _cube_box_points(target):
    """Envelope box of the drawn cube: 59mm square footprint, asymmetric z.

    The z extent is read from the target's own marker planes so a session that
    was captured with an earlier cube CAD is drawn with the box it actually had:
    the top of the +Z protrusion is the top-marker plane, and the body bottom is
    half the (revision-invariant) 57mm body below the side-marker plane.
    """
    cfg = target.cfg
    d = float(cfg.cube_side_m) / 2.0
    top_z = [target.model.marker_pose_in_rig(mid)[2, 3]
             for mid in cfg.marker_ids if cfg.id_to_face.get(int(mid)) == "+Z"]
    side_z = [target.model.marker_pose_in_rig(mid)[2, 3]
              for mid in cfg.marker_ids
              if cfg.id_to_face.get(int(mid)) in ("+X", "-X", "+Y", "-Y")]
    z_top = max(top_z) if top_z else CUBE_BODY_TOP_Z_M
    z_bottom = (min(side_z) - CUBE_BODY_HEIGHT_M / 2.0) if side_z else CUBE_BODY_BOTTOM_Z_M
    return np.array(
        [[x, y, z] for x in (-d, d) for y in (-d, d) for z in (z_bottom, z_top)],
        dtype=np.float64,
    )


BOX_EDGES = (
    (0, 1), (0, 2), (0, 4), (1, 3), (1, 5), (2, 3),
    (2, 6), (3, 7), (4, 5), (4, 6), (5, 7), (6, 7),
)


def _draw_model_overlay(image, camera_idx, event_id, camera_info, target, pose, K, D):
    canvas = image.copy()

    box_2d = _project(_cube_box_points(target), pose, K, D)
    for a, b in BOX_EDGES:
        cv2.line(
            canvas,
            tuple(np.round(box_2d[a]).astype(int)),
            tuple(np.round(box_2d[b]).astype(int)),
            (200, 200, 200),
            1,
            cv2.LINE_AA,
        )

    origin_axes = np.array(
        [[0.0, 0.0, 0.0], [0.035, 0.0, 0.0], [0.0, 0.035, 0.0], [0.0, 0.0, 0.035]],
        dtype=np.float64,
    )
    axes_2d = _project(origin_axes, pose, K, D)
    axis_colors = ((0, 0, 255), (0, 220, 0), (255, 80, 20))
    for index, (label, color) in enumerate(zip(("X", "Y", "Z"), axis_colors), start=1):
        cv2.arrowedLine(
            canvas,
            tuple(np.round(axes_2d[0]).astype(int)),
            tuple(np.round(axes_2d[index]).astype(int)),
            color,
            3,
            cv2.LINE_AA,
            tipLength=0.18,
        )
        cv2.putText(
            canvas,
            label,
            tuple(np.round(axes_2d[index] + [4, -4]).astype(int)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            color,
            2,
            cv2.LINE_AA,
        )

    for marker in camera_info.get("markers") or []:
        marker_id = int(marker["marker_id"])
        detected = np.asarray(marker["corners_2d"], dtype=np.float64).reshape(4, 2)
        model_corners = target.model.marker_corners_in_rig(marker_id)
        projected = _project(model_corners, pose, K, D)
        cv2.polylines(canvas, [np.round(detected).astype(np.int32)], True, DETECTED, 4, cv2.LINE_AA)
        cv2.polylines(canvas, [np.round(projected).astype(np.int32)], True, PROJECTED, 2, cv2.LINE_AA)

        for corner_index, point in enumerate(detected):
            point_i = tuple(np.round(point).astype(int))
            color = CORNER_COLORS[corner_index]
            cv2.circle(canvas, point_i, 6, BLACK, -1, cv2.LINE_AA)
            cv2.circle(canvas, point_i, 4, color, -1, cv2.LINE_AA)
            cv2.putText(
                canvas,
                f"p{corner_index}",
                (point_i[0] + 7, point_i[1] - 6),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.48,
                color,
                2,
                cv2.LINE_AA,
            )

        marker_pose = target.model.marker_pose_in_rig(marker_id)
        center = marker_pose[:3, 3]
        normal_end = center + 0.018 * marker_pose[:3, 2]
        normal_2d = _project(np.vstack([center, normal_end]), pose, K, D)
        cv2.arrowedLine(
            canvas,
            tuple(np.round(normal_2d[0]).astype(int)),
            tuple(np.round(normal_2d[1]).astype(int)),
            NORMAL,
            3,
            cv2.LINE_AA,
            tipLength=0.25,
        )
        label_point = np.mean(detected, axis=0) + np.array([10.0, -12.0])
        _put_label(
            canvas,
            f"ID {marker_id}  {target.cfg.id_to_face[marker_id]}  "
            f"{target.model.marker_size(marker_id) * 1000:.0f}mm",
            label_point,
            scale=0.50,
        )

    _put_label(
        canvas,
        f"cam{camera_idx} | event {event_id:05d} | IDs {','.join(map(str, pose.used_ids))} "
        f"| PnP RMSE {pose.rmse_px:.2f}px",
        (12, 28),
        scale=0.62,
    )
    legend_y = canvas.shape[0] - 18
    _put_label(
        canvas,
        "green: detected | magenta: model reprojection | orange: outward normal | X/Y/Z: cube axes",
        (12, legend_y),
        scale=0.45,
    )
    return canvas


def _fit_panel(image, size):
    width, height = size
    scale = min(width / image.shape[1], height / image.shape[0])
    resized = cv2.resize(
        image,
        (int(round(image.shape[1] * scale)), int(round(image.shape[0] * scale))),
        interpolation=cv2.INTER_AREA,
    )
    panel = np.full((height, width, 3), 24, dtype=np.uint8)
    x = (width - resized.shape[1]) // 2
    y = (height - resized.shape[0]) // 2
    panel[y:y + resized.shape[0], x:x + resized.shape[1]] = resized
    return panel


def _render_camera_grid(overlays: Dict[int, np.ndarray], destination):
    cameras = sorted(overlays)
    height = max(image.shape[0] for image in overlays.values())
    width = max(image.shape[1] for image in overlays.values())
    panels = [_fit_panel(overlays[camera], (width, height)) for camera in cameras]
    while len(panels) < 4:
        panels.append(np.full((height, width, 3), 24, dtype=np.uint8))
    grid = np.vstack([np.hstack(panels[:2]), np.hstack(panels[2:4])])
    cv2.imwrite(destination, grid)


def _render_marker_gallery(cap, raw_images, camera_models, target, destination):
    selections = {}
    for camera_text, camera_info in cap.get("cams", {}).items():
        camera = int(camera_text)
        for marker in camera_info.get("markers") or []:
            marker_id = int(marker["marker_id"])
            corners = np.asarray(marker["corners_2d"], dtype=np.float64).reshape(4, 2)
            area = abs(float(cv2.contourArea(corners.astype(np.float32))))
            if marker_id not in selections or area > selections[marker_id][0]:
                selections[marker_id] = (area, camera, corners)

    panel_width, panel_height, header = 640, 440, 62
    panels = []
    for marker_id in target.cfg.marker_ids:
        _, camera, corners = selections[int(marker_id)]
        pose, K, D = camera_models[camera]
        image = raw_images[camera].copy()
        projected = _project(target.model.marker_corners_in_rig(int(marker_id)), pose, K, D)
        cv2.polylines(image, [np.round(corners).astype(np.int32)], True, DETECTED, 5, cv2.LINE_AA)
        cv2.polylines(image, [np.round(projected).astype(np.int32)], True, PROJECTED, 3, cv2.LINE_AA)
        for corner_index, point in enumerate(corners):
            point_i = tuple(np.round(point).astype(int))
            color = CORNER_COLORS[corner_index]
            cv2.circle(image, point_i, 8, BLACK, -1, cv2.LINE_AA)
            cv2.circle(image, point_i, 5, color, -1, cv2.LINE_AA)
            cv2.putText(
                image,
                f"p{corner_index}",
                (point_i[0] + 8, point_i[1] - 8),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.58,
                color,
                2,
                cv2.LINE_AA,
            )
        marker_pose = target.model.marker_pose_in_rig(int(marker_id))
        center_3d = marker_pose[:3, 3]
        normal_3d = marker_pose[:3, 2]
        normal_2d = _project(
            np.vstack([center_3d, center_3d + 0.018 * normal_3d]), pose, K, D
        )
        cv2.arrowedLine(
            image,
            tuple(np.round(normal_2d[0]).astype(int)),
            tuple(np.round(normal_2d[1]).astype(int)),
            NORMAL,
            4,
            cv2.LINE_AA,
            tipLength=0.24,
        )
        x0, y0 = np.min(corners, axis=0)
        x1, y1 = np.max(corners, axis=0)
        pad = max(x1 - x0, y1 - y0) * 0.85
        xa = max(0, int(np.floor(x0 - pad)))
        ya = max(0, int(np.floor(y0 - pad)))
        xb = min(image.shape[1], int(np.ceil(x1 + pad)))
        yb = min(image.shape[0], int(np.ceil(y1 + pad)))
        crop = image[ya:yb, xa:xb]
        fitted = _fit_panel(crop, (panel_width, panel_height - header))
        panel = np.full((panel_height, panel_width, 3), 24, dtype=np.uint8)
        panel[header:, :] = fitted
        center = np.asarray(target.cfg.marker_center_m[int(marker_id)]) * 1000.0
        normal = np.rint(normal_3d).astype(int)
        title = (
            f"ID {marker_id} | face {target.cfg.id_to_face[int(marker_id)]} | "
            f"size {target.model.marker_size(int(marker_id)) * 1000:.0f}mm | cam{camera}"
        )
        detail = (
            f"center ({center[0]:+.1f},{center[1]:+.1f},{center[2]:+.1f})mm | "
            f"normal ({normal[0]:+d},{normal[1]:+d},{normal[2]:+d})"
        )
        cv2.putText(panel, title, (12, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.60, WHITE, 2, cv2.LINE_AA)
        cv2.putText(panel, detail, (12, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.50, (205, 205, 205), 1, cv2.LINE_AA)
        panels.append(panel)

    gallery = np.vstack([np.hstack(panels[:3]), np.hstack(panels[3:6])])
    cv2.imwrite(destination, gallery)


def _set_axes_equal(ax, half_extent=0.048):
    ax.set_xlim(-half_extent, half_extent)
    ax.set_ylim(-half_extent, half_extent)
    ax.set_zlim(-half_extent, half_extent)
    ax.set_box_aspect((1, 1, 1))


def _render_geometry_reference(target, destination):
    fig = plt.figure(figsize=(18, 9), dpi=140, facecolor="#11151b")
    views = ((24, -54, "+X / +Y / +Z view"), (24, 126, "-X / -Y / +Z view"))
    box = _cube_box_points(target) * 1000.0

    for subplot, (elevation, azimuth, title) in enumerate(views, start=1):
        ax = fig.add_subplot(1, 2, subplot, projection="3d", facecolor="#11151b")
        for a, b in BOX_EDGES:
            ax.plot(*zip(box[a], box[b]), color="#8b949e", linewidth=1.1, alpha=0.7)

        for marker_id in target.cfg.marker_ids:
            corners = target.model.marker_corners_in_rig(marker_id) * 1000.0
            color = ID_COLORS_RGB[int(marker_id)]
            polygon = Poly3DCollection([corners], alpha=0.70, facecolor=color, edgecolor=color, linewidth=2.2)
            ax.add_collection3d(polygon)
            pose = target.model.marker_pose_in_rig(marker_id)
            center = pose[:3, 3] * 1000.0
            normal = pose[:3, 2]
            ax.quiver(
                center[0], center[1], center[2],
                normal[0], normal[1], normal[2],
                length=14.0, normalize=True, color="#ffb347", linewidth=2.0,
            )
            ax.text(
                center[0], center[1], center[2],
                f" ID {marker_id}\n {target.cfg.id_to_face[int(marker_id)]}",
                color="white", fontsize=9, fontweight="bold",
            )
            for corner_index, corner in enumerate(corners):
                ax.scatter(*corner, color=[color], s=16, depthshade=False)
                ax.text(*corner, f"p{corner_index}", color="#d7dde5", fontsize=7)

        ax.quiver(0, 0, 0, 35, 0, 0, color="#ff4d4d", arrow_length_ratio=0.10, linewidth=2)
        ax.quiver(0, 0, 0, 0, 35, 0, color="#55dd77", arrow_length_ratio=0.10, linewidth=2)
        ax.quiver(0, 0, 0, 0, 0, 35, color="#4da6ff", arrow_length_ratio=0.10, linewidth=2)
        ax.text(37, 0, 0, "+X", color="#ff7777")
        ax.text(0, 37, 0, "+Y", color="#77ee99")
        ax.text(0, 0, 37, "+Z", color="#77baff")
        _set_axes_equal(ax, 48)
        ax.view_init(elev=elevation, azim=azimuth)
        ax.set_xlabel("X [mm]", color="white")
        ax.set_ylabel("Y [mm]", color="white")
        ax.set_zlabel("Z [mm]", color="white")
        ax.tick_params(colors="#aeb7c2", labelsize=8)
        ax.set_title(title, color="white", fontsize=14, pad=14)
        for axis in (ax.xaxis, ax.yaxis, ax.zaxis):
            axis.pane.fill = False
            axis.pane.set_edgecolor("#3a424d")
            axis._axinfo["grid"]["color"] = (0.25, 0.29, 0.34, 0.45)

    fig.suptitle(
        "59 mm AprilTag cube model | p0 -> p3 clockwise from outside | orange arrows = outward normals",
        color="white",
        fontsize=16,
        y=0.97,
    )
    fig.tight_layout(rect=(0, 0, 1, 0.94))
    fig.savefig(destination, facecolor=fig.get_facecolor(), bbox_inches="tight")
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--session-root", default="data/session04/calib_train")
    parser.add_argument("--intrinsics-dir", default="intrinsics")
    parser.add_argument(
        "--output-dir",
        default="data/session04/calib_out/verify/cube_model_validation",
    )
    parser.add_argument("--event", type=int, default=0)
    args = parser.parse_args()

    with open(os.path.join(args.session_root, "meta.json"), "r", encoding="utf-8") as stream:
        meta = json.load(stream)
    cfg, _ = load_cube_config_from_meta(args.session_root)
    target = AprilTagCubeTarget(cfg)
    capture = next(
        capture for capture in meta["captures"] if int(capture["event_id"]) == int(args.event)
    )
    os.makedirs(args.output_dir, exist_ok=True)

    raw_images, overlays, camera_models = {}, {}, {}
    for camera_text, camera_info in capture.get("cams", {}).items():
        camera = int(camera_text)
        image = cv2.imread(os.path.join(args.session_root, camera_info["rgb_path"]))
        if image is None:
            raise FileNotFoundError(camera_info["rgb_path"])
        K, D, _ = load_intrinsics_with_depth_scale(args.intrinsics_dir, camera)
        pose = _solve_camera_pose(target, camera_info, K, D)
        raw_images[camera] = image
        camera_models[camera] = (pose, K, D)
        overlays[camera] = _draw_model_overlay(
            image, camera, args.event, camera_info, target, pose, K, D
        )

    event_name = f"event{args.event:05d}"
    grid_path = os.path.join(args.output_dir, f"cube_model_overlay_{event_name}.png")
    gallery_path = os.path.join(args.output_dir, f"cube_marker_gallery_{event_name}.png")
    geometry_path = os.path.join(args.output_dir, "cube_geometry_reference.png")
    _render_camera_grid(overlays, grid_path)
    _render_marker_gallery(capture, raw_images, camera_models, target, gallery_path)
    _render_geometry_reference(target, geometry_path)

    print(grid_path)
    print(gallery_path)
    print(geometry_path)


if __name__ == "__main__":
    main()
