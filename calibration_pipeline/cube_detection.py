"""AprilTag-cube raw-corner observations used by the reprojection solver."""

from __future__ import annotations

import os
from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Tuple

import cv2
import numpy as np

from calibration_pipeline.apriltag_cube import AprilTagCubeTarget
from calibration_pipeline.runtime import get_capture_set_index


@dataclass
class CornerObservation:
    cam: int
    event: int
    set_idx: Optional[int]
    object_points: np.ndarray
    image_points: np.ndarray
    err_hint_px: float
    grasp_idx: Optional[int] = None


def _marker_aspect_ratio(image_points: np.ndarray) -> float:
    points = np.asarray(image_points, dtype=np.float64).reshape(4, 2)
    lengths = [
        np.linalg.norm(points[(index + 1) % 4] - points[index])
        for index in range(4)
    ]
    return float(min(lengths) / max(max(lengths), 1e-12))


def _object_corners(cube: AprilTagCubeTarget,
                    marker_id: int) -> Optional[np.ndarray]:
    """Adapt supported cube-model APIs to one ordered 4x3 corner array."""
    model = cube.model
    marker_id = int(marker_id)
    for name in (
        "marker_corners_in_rig", "get_marker_object_corners",
        "marker_object_corners", "get_marker_corners_3d",
        "marker_corners_3d", "object_corners", "corners_3d",
    ):
        method = getattr(model, name, None)
        if not callable(method):
            continue
        try:
            points = np.asarray(method(marker_id), dtype=np.float64)
        except Exception:
            continue
        if points.shape == (4, 3):
            return points

    for name in (
        "marker_corners_obj", "marker_corners_3d", "object_points_by_id",
        "corners_by_marker", "markers",
    ):
        values = getattr(model, name, None)
        if not isinstance(values, dict) or marker_id not in values:
            continue
        value = values[marker_id]
        if isinstance(value, dict):
            for key in ("corners_3d", "object_points", "obj_pts", "points"):
                if key in value:
                    points = np.asarray(value[key], dtype=np.float64)
                    if points.shape == (4, 3):
                        return points
        else:
            points = np.asarray(value, dtype=np.float64)
            if points.shape == (4, 3):
                return points
    return None


def detect_corner_observations(
    root: str,
    meta: Dict[str, Any],
    cube: AprilTagCubeTarget,
    K_map: Dict[int, np.ndarray],
    D_map: Dict[int, np.ndarray],
    all_cam_ids: List[int],
    gripper_cam_idx: int,
    max_err_fixed: float,
    max_err_gripper: float,
    min_aspect_fixed: float,
    min_aspect_gripper: float,
    exclude_gripped: bool = False,
    image_scale: float = 1.0,
) -> Tuple[List[CornerObservation], str]:
    """Detect native-pixel cube corners and explain an empty result."""
    del K_map, D_map  # Detection is geometry-only; projection uses these later.
    image_scale = float(image_scale)
    if not np.isfinite(image_scale) or image_scale <= 0.0:
        raise ValueError("image_scale must be finite and positive")
    observations: List[CornerObservation] = []
    images_read = images_missing = detections = 0
    object_corner_failures = aspect_rejections = 0

    for capture in meta.get("captures", []):
        event = int(capture.get("event_id", -1))
        if event < 0 or (exclude_gripped and capture.get("cube_gripped")):
            continue
        set_index = get_capture_set_index(capture)
        gripped = bool(capture.get("cube_gripped"))
        grasp = capture.get("grasp_id")
        if gripped and grasp is None:
            raise ValueError(
                f"capture event {event} is cube_gripped but has no grasp_id")
        grasp = int(grasp) if gripped else None

        for camera_text, camera_info in capture.get("cams", {}).items():
            camera = int(camera_text)
            if camera not in all_cam_ids or not camera_info.get("saved"):
                continue
            relative_path = camera_info.get("rgb_path", "")
            if not relative_path:
                images_missing += 1
                continue
            image = cv2.imread(os.path.join(root, relative_path))
            if image is None:
                images_missing += 1
                continue
            if image_scale != 1.0:
                interpolation = (
                    cv2.INTER_AREA if image_scale < 1.0 else cv2.INTER_CUBIC)
                image = cv2.resize(
                    image, None, fx=image_scale, fy=image_scale,
                    interpolation=interpolation,
                )
            images_read += 1
            try:
                corner_sets, marker_ids = cube.detect(image)
            except Exception:
                continue
            if marker_ids is None:
                continue

            object_points, image_points = [], []
            min_aspect = (
                min_aspect_gripper if camera == gripper_cam_idx
                else min_aspect_fixed
            )
            for corners, raw_marker_id in zip(corner_sets, marker_ids):
                marker_id = int(np.asarray(raw_marker_id).reshape(-1)[0])
                if not cube.model.has_marker(marker_id):
                    continue
                detections += 1
                native_points = (
                    np.asarray(corners, dtype=np.float64).reshape(4, 2)
                    / image_scale
                )
                try:
                    ordered_points = np.asarray(
                        cube.model.reorder_image_corners(
                            marker_id, native_points),
                        dtype=np.float64,
                    ).reshape(4, 2)
                except Exception:
                    ordered_points = native_points
                if _marker_aspect_ratio(ordered_points) < min_aspect:
                    aspect_rejections += 1
                    continue
                marker_object_points = _object_corners(cube, marker_id)
                if marker_object_points is None:
                    object_corner_failures += 1
                    continue
                object_points.append(marker_object_points)
                image_points.append(ordered_points)

            if object_points:
                observations.append(CornerObservation(
                    cam=camera,
                    event=event,
                    set_idx=(int(set_index) if set_index is not None else None),
                    object_points=np.concatenate(object_points, axis=0),
                    image_points=np.concatenate(image_points, axis=0),
                    err_hint_px=(
                        max_err_gripper if camera == gripper_cam_idx
                        else max_err_fixed
                    ),
                    grasp_idx=grasp,
                ))

    reason = ""
    if not observations:
        if images_read == 0:
            reason = (
                "0 corner observations because no images could be read "
                f"({images_missing} missing/unreadable rgb paths)"
            )
        elif detections == 0:
            reason = "0 corner observations because no AprilTags were detected"
        elif object_corner_failures >= max(1, detections - aspect_rejections):
            reason = (
                "0 corner observations because cube-model 3D corners were "
                "unavailable"
            )
        else:
            reason = (
                "0 corner observations after filtering: "
                f"{aspect_rejections} aspect-rejected, "
                f"{object_corner_failures}/{detections} missing object corners"
            )
    return observations, reason
