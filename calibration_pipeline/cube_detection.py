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
    pnp_reprojection_rmse_px: float
    pnp_inlier_fraction: float
    pnp_solver: str
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


def _is_planar(object_points: np.ndarray) -> bool:
    """Return whether the 3-D support is numerically confined to one plane."""
    points = np.asarray(object_points, dtype=np.float64).reshape(-1, 3)
    if len(points) < 4:
        return True
    singular_values = np.linalg.svd(
        points - np.mean(points, axis=0), compute_uv=False)
    scale = max(float(singular_values[0]), 1e-12)
    return bool(float(singular_values[-1]) <= 1e-7 * scale)


def _pnp_quality(
    object_points: np.ndarray,
    image_points: np.ndarray,
    K: np.ndarray,
    D: np.ndarray,
    threshold_px: float,
) -> Optional[Tuple[float, float, str]]:
    """Estimate a measurement-only pose and score every detected corner.

    Planar support uses IPPE and explicitly chooses the positive-depth solution
    with the lowest all-corner error.  Non-planar support uses RANSAC only to
    initialize the pose; acceptance is still based on the RMSE of *all* input
    corners, so an outlier cannot disappear from the shared quality mask.
    """
    obj = np.asarray(object_points, dtype=np.float64).reshape(-1, 3)
    img = np.asarray(image_points, dtype=np.float64).reshape(-1, 2)
    K = np.asarray(K, dtype=np.float64).reshape(3, 3)
    D = np.asarray(D, dtype=np.float64)
    threshold_px = float(threshold_px)
    if (len(obj) < 4 or obj.shape[0] != img.shape[0]
            or not np.all(np.isfinite(obj)) or not np.all(np.isfinite(img))
            or not np.isfinite(threshold_px) or threshold_px <= 0.0):
        return None

    candidates = []
    if _is_planar(obj):
        try:
            result = cv2.solvePnPGeneric(
                obj, img, K, D, flags=cv2.SOLVEPNP_IPPE)
            count, rvecs, tvecs = int(result[0]), result[1], result[2]
            if count > 0:
                candidates.extend(
                    (np.asarray(rvec, dtype=np.float64).reshape(3, 1),
                     np.asarray(tvec, dtype=np.float64).reshape(3, 1),
                     "IPPE")
                    for rvec, tvec in zip(rvecs, tvecs)
                )
        except cv2.error:
            candidates = []
    else:
        try:
            ok, rvec, tvec, inliers = cv2.solvePnPRansac(
                obj, img, K, D,
                iterationsCount=200,
                reprojectionError=threshold_px,
                confidence=0.999,
                flags=cv2.SOLVEPNP_EPNP,
            )
            if ok:
                solver = "RANSAC-EPNP"
                if inliers is not None and len(inliers) >= 4:
                    indices = np.asarray(inliers, dtype=np.int64).reshape(-1)
                    try:
                        rvec, tvec = cv2.solvePnPRefineLM(
                            obj[indices], img[indices], K, D, rvec, tvec)
                        solver += "+LM"
                    except (AttributeError, cv2.error):
                        pass
                candidates.append((rvec, tvec, solver))
        except cv2.error:
            pass
        if not candidates:
            try:
                has_sqpnp = hasattr(cv2, "SOLVEPNP_SQPNP")
                fallback_flag = (
                    cv2.SOLVEPNP_SQPNP if has_sqpnp else cv2.SOLVEPNP_EPNP)
                ok, rvec, tvec = cv2.solvePnP(
                    obj, img, K, D, flags=fallback_flag)
                if ok:
                    candidates.append((
                        rvec, tvec, "SQPNP" if has_sqpnp else "EPNP"))
            except cv2.error:
                pass

    scored = []
    homogeneous = np.column_stack([obj, np.ones(len(obj), dtype=np.float64)])
    for rvec, tvec, solver in candidates:
        rotation, _ = cv2.Rodrigues(rvec)
        transform = np.eye(4, dtype=np.float64)
        transform[:3, :3] = rotation
        transform[:3, 3] = np.asarray(tvec, dtype=np.float64).reshape(3)
        depths = (transform @ homogeneous.T).T[:, 2]
        if not np.all(np.isfinite(depths)) or np.any(depths <= 0.0):
            continue
        projected, _ = cv2.projectPoints(obj, rvec, tvec, K, D)
        errors = np.linalg.norm(projected.reshape(-1, 2) - img, axis=1)
        if not np.all(np.isfinite(errors)):
            continue
        rmse = float(np.sqrt(np.mean(np.square(errors))))
        inlier_fraction = float(np.mean(errors <= threshold_px))
        scored.append((rmse, inlier_fraction, solver))
    return min(scored, key=lambda item: item[0]) if scored else None


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
) -> Tuple[List[CornerObservation], dict]:
    """Detect native-pixel cube corners with one shared pre-fit PnP gate."""
    image_scale = float(image_scale)
    if not np.isfinite(image_scale) or image_scale <= 0.0:
        raise ValueError("image_scale must be finite and positive")
    observations: List[CornerObservation] = []
    images_read = images_missing = detections = 0
    object_corner_failures = aspect_rejections = 0
    pnp_failures = pnp_error_rejections = 0
    pnp_accepted_rmse = []
    pnp_solver_counts: Dict[str, int] = {}

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
                object_array = np.concatenate(object_points, axis=0)
                image_array = np.concatenate(image_points, axis=0)
                max_error = (
                    max_err_gripper if camera == gripper_cam_idx
                    else max_err_fixed
                )
                pnp_quality = _pnp_quality(
                    object_array, image_array, K_map[camera], D_map[camera],
                    max_error)
                if pnp_quality is None:
                    pnp_failures += 1
                    continue
                pnp_rmse, pnp_inlier_fraction, pnp_solver = pnp_quality
                if pnp_rmse > float(max_error):
                    pnp_error_rejections += 1
                    continue
                pnp_accepted_rmse.append(float(pnp_rmse))
                pnp_solver_counts[pnp_solver] = pnp_solver_counts.get(pnp_solver, 0) + 1
                observations.append(CornerObservation(
                    cam=camera,
                    event=event,
                    set_idx=(int(set_index) if set_index is not None else None),
                    object_points=object_array,
                    image_points=image_array,
                    pnp_reprojection_rmse_px=float(pnp_rmse),
                    pnp_inlier_fraction=float(pnp_inlier_fraction),
                    pnp_solver=str(pnp_solver),
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
                f"{object_corner_failures}/{detections} missing object corners, "
                f"{pnp_failures} PnP-invalid, "
                f"{pnp_error_rejections} PnP-RMSE-rejected"
            )
    diagnostics = {
        "status": "ok" if observations else "empty",
        "reason": reason,
        "quality_contract": {
            "selection_stage": "before_split_and_before_any_calibration_fit",
            "model_output_used": False,
            "metric": "sqrt(mean_over_corners(||projected-measured||_2^2))",
            "all_detected_corners_scored": True,
            "max_rmse_px_by_role": {
                "fixed": float(max_err_fixed),
                "gripper": float(max_err_gripper),
            },
            "planar_solver": "IPPE_positive_depth_best_all_corner_RMSE",
            "nonplanar_solver": (
                "RANSAC_EPNP_plus_optional_LM_initialization_then_"
                "all_corner_RMSE"),
        },
        "counts": {
            "images_read": int(images_read),
            "images_missing_or_unreadable": int(images_missing),
            "detected_markers": int(detections),
            "aspect_rejections": int(aspect_rejections),
            "missing_object_corner_rejections": int(object_corner_failures),
            "pnp_failures": int(pnp_failures),
            "pnp_rmse_rejections": int(pnp_error_rejections),
            "accepted_observations": int(len(observations)),
        },
        "accepted_pnp_rmse_px": {
            "min": (float(np.min(pnp_accepted_rmse))
                    if pnp_accepted_rmse else None),
            "median": (float(np.median(pnp_accepted_rmse))
                       if pnp_accepted_rmse else None),
            "max": (float(np.max(pnp_accepted_rmse))
                    if pnp_accepted_rmse else None),
        },
        "accepted_solver_counts": dict(sorted(pnp_solver_counts.items())),
    }
    return observations, diagnostics
