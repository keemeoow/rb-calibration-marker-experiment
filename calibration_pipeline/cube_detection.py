"""AprilTag-cube raw-corner observations used by the reprojection solver."""

from __future__ import annotations

import os
from collections import Counter, defaultdict
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
    marker_ids: Tuple[int, ...] = ()
    observed_faces: Tuple[str, ...] = ()
    observed_face_count: int = 0
    noncoplanar_face_count: int = 0
    is_planar: bool = True
    positive_depth_candidate_count: int = 0
    pnp_candidate_count: int = 0
    quality_tier: str = "unknown"


@dataclass(frozen=True)
class PnPQuality:
    rmse_px: float
    inlier_fraction: float
    solver: str
    positive_depth_candidate_count: int
    candidate_count: int


@dataclass
class DetectionCandidate:
    method: str
    detected_marker_ids: Tuple[int, ...]
    marker_ids: Tuple[int, ...]
    object_points: np.ndarray
    image_points: np.ndarray
    support: Dict[str, Any]
    pnp_quality: Optional[PnPQuality]
    status: str
    aspect_rejections: int = 0
    object_corner_failures: int = 0


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


def _support_metadata(cube: AprilTagCubeTarget,
                      marker_ids: List[int],
                      object_points: np.ndarray) -> Dict[str, Any]:
    """Describe one event-camera cube observation before calibration fitting."""
    # Preserve detector/correspondence block order.  Downstream diagnostics use
    # marker_ids[k] to identify object_points[4*k:4*k+4]; sorting here silently
    # broke that association whenever the detector returned a different order.
    ids = tuple(dict.fromkeys(int(marker_id) for marker_id in marker_ids))
    faces = tuple(sorted({
        str(cube.model.marker_face_name(marker_id)) for marker_id in ids
    }))
    planar = _is_planar(object_points)
    face_count = len(faces)
    if not planar and face_count >= 2:
        tier = "nonplanar_multiface"
    elif len(ids) >= 2:
        tier = "planar_multimarker"
    elif len(ids) == 1:
        tier = "single_marker"
    else:
        tier = "none"
    return {
        "marker_ids": ids,
        "observed_faces": faces,
        "observed_face_count": int(face_count),
        # For the cube, this is the number of distinct physical faces jointly
        # supporting a rank-3 point set. A coplanar pair such as IDs 0+1 is 0.
        "noncoplanar_face_count": int(face_count if not planar else 0),
        "is_planar": bool(planar),
        "quality_tier": tier,
    }


def _pnp_quality_details(
    object_points: np.ndarray,
    image_points: np.ndarray,
    K: np.ndarray,
    D: np.ndarray,
    threshold_px: float,
) -> Optional[PnPQuality]:
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
    positive_depth_candidate_count = 0
    homogeneous = np.column_stack([obj, np.ones(len(obj), dtype=np.float64)])
    for rvec, tvec, solver in candidates:
        rotation, _ = cv2.Rodrigues(rvec)
        transform = np.eye(4, dtype=np.float64)
        transform[:3, :3] = rotation
        transform[:3, 3] = np.asarray(tvec, dtype=np.float64).reshape(3)
        depths = (transform @ homogeneous.T).T[:, 2]
        if not np.all(np.isfinite(depths)) or np.any(depths <= 0.0):
            continue
        positive_depth_candidate_count += 1
        projected, _ = cv2.projectPoints(obj, rvec, tvec, K, D)
        errors = np.linalg.norm(projected.reshape(-1, 2) - img, axis=1)
        if not np.all(np.isfinite(errors)):
            continue
        rmse = float(np.sqrt(np.mean(np.square(errors))))
        inlier_fraction = float(np.mean(errors <= threshold_px))
        scored.append((rmse, inlier_fraction, solver))
    if not scored:
        return None
    rmse, inlier_fraction, solver = min(scored, key=lambda item: item[0])
    return PnPQuality(
        rmse_px=float(rmse),
        inlier_fraction=float(inlier_fraction),
        solver=str(solver),
        positive_depth_candidate_count=int(positive_depth_candidate_count),
        candidate_count=int(len(candidates)),
    )


def _pnp_quality(
    object_points: np.ndarray,
    image_points: np.ndarray,
    K: np.ndarray,
    D: np.ndarray,
    threshold_px: float,
) -> Optional[Tuple[float, float, str]]:
    """Backward-compatible compact PnP-quality result."""
    details = _pnp_quality_details(
        object_points, image_points, K, D, threshold_px)
    if details is None:
        return None
    return details.rmse_px, details.inlier_fraction, details.solver


def _evaluate_detection_candidate(
    cube: AprilTagCubeTarget,
    corner_sets: List[np.ndarray],
    marker_ids: Optional[np.ndarray],
    *,
    method: str,
    image_scale: float,
    min_aspect: float,
    K: np.ndarray,
    D: np.ndarray,
    max_error: float,
) -> DetectionCandidate:
    empty_support = {
        "marker_ids": (),
        "observed_faces": (),
        "observed_face_count": 0,
        "noncoplanar_face_count": 0,
        "is_planar": None,
        "quality_tier": "none",
    }
    if marker_ids is None:
        return DetectionCandidate(
            method=method,
            detected_marker_ids=(),
            marker_ids=(),
            object_points=np.empty((0, 3), dtype=np.float64),
            image_points=np.empty((0, 2), dtype=np.float64),
            support=empty_support,
            pnp_quality=None,
            status="no_markers_detected",
        )

    object_points, image_points = [], []
    detected_marker_ids: List[int] = []
    used_marker_ids: List[int] = []
    aspect_rejections = object_corner_failures = 0
    for corners, raw_marker_id in zip(corner_sets, marker_ids):
        marker_id = int(np.asarray(raw_marker_id).reshape(-1)[0])
        if not cube.model.has_marker(marker_id):
            continue
        detected_marker_ids.append(marker_id)
        native_points = (
            np.asarray(corners, dtype=np.float64).reshape(4, 2)
            / float(image_scale)
        )
        try:
            ordered_points = np.asarray(
                cube.model.reorder_image_corners(marker_id, native_points),
                dtype=np.float64,
            ).reshape(4, 2)
        except Exception:
            ordered_points = native_points
        if _marker_aspect_ratio(ordered_points) < float(min_aspect):
            aspect_rejections += 1
            continue
        marker_object_points = _object_corners(cube, marker_id)
        if marker_object_points is None:
            object_corner_failures += 1
            continue
        object_points.append(marker_object_points)
        image_points.append(ordered_points)
        used_marker_ids.append(marker_id)

    if not object_points:
        status = (
            "no_configured_markers" if not detected_marker_ids
            else "no_usable_markers_after_filtering"
        )
        return DetectionCandidate(
            method=method,
            detected_marker_ids=tuple(sorted(set(detected_marker_ids))),
            marker_ids=(),
            object_points=np.empty((0, 3), dtype=np.float64),
            image_points=np.empty((0, 2), dtype=np.float64),
            support=empty_support,
            pnp_quality=None,
            status=status,
            aspect_rejections=aspect_rejections,
            object_corner_failures=object_corner_failures,
        )

    object_array = np.concatenate(object_points, axis=0)
    image_array = np.concatenate(image_points, axis=0)
    support = _support_metadata(cube, used_marker_ids, object_array)
    pnp_quality = _pnp_quality_details(
        object_array, image_array, K, D, max_error)
    if pnp_quality is None:
        status = "pnp_invalid"
    elif float(pnp_quality.rmse_px) > float(max_error):
        status = "pnp_rmse_rejected"
    else:
        status = "accepted"
    return DetectionCandidate(
        method=method,
        detected_marker_ids=tuple(sorted(set(detected_marker_ids))),
        marker_ids=tuple(support["marker_ids"]),
        object_points=object_array,
        image_points=image_array,
        support=support,
        pnp_quality=pnp_quality,
        status=status,
        aspect_rejections=aspect_rejections,
        object_corner_failures=object_corner_failures,
    )


def _is_accepted_core_candidate(candidate: DetectionCandidate) -> bool:
    return bool(
        candidate.status == "accepted"
        and candidate.support["quality_tier"] == "nonplanar_multiface"
        and int(candidate.support["noncoplanar_face_count"]) >= 2
        and candidate.pnp_quality is not None
        and int(candidate.pnp_quality.positive_depth_candidate_count) >= 1
    )


def _detection_candidate_rank(candidate: DetectionCandidate):
    accepted = candidate.status == "accepted"
    core = _is_accepted_core_candidate(candidate)
    tier_rank = {
        "none": 0,
        "single_marker": 1,
        "planar_multimarker": 2,
        "nonplanar_multiface": 3,
    }.get(str(candidate.support["quality_tier"]), 0)
    rmse = (
        float(candidate.pnp_quality.rmse_px)
        if candidate.pnp_quality is not None
        else float("inf")
    )
    inlier_fraction = (
        float(candidate.pnp_quality.inlier_fraction)
        if candidate.pnp_quality is not None
        else 0.0
    )
    return (
        int(core),
        int(accepted),
        tier_rank,
        int(candidate.support["noncoplanar_face_count"]),
        len(candidate.marker_ids),
        inlier_fraction,
        -rmse,
    )


def _summarize_quality_by_event(records: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
    grouped: Dict[int, List[Dict[str, Any]]] = defaultdict(list)
    for record in records:
        grouped[int(record["event_id"])].append(record)
    output = []
    for event_id, event_records in sorted(grouped.items()):
        accepted = [record for record in event_records if record.get("pnp_accepted")]
        tier_cameras = {
            tier: sorted(int(record["camera_id"]) for record in accepted
                         if record.get("quality_tier") == tier)
            for tier in ("nonplanar_multiface", "planar_multimarker", "single_marker")
        }
        output.append({
            "event_id": int(event_id),
            "camera_ids": sorted(int(record["camera_id"]) for record in event_records),
            "pnp_accepted_camera_ids": sorted(
                int(record["camera_id"]) for record in accepted),
            "core_multiface_camera_ids": tier_cameras["nonplanar_multiface"],
            "planar_multimarker_camera_ids": tier_cameras["planar_multimarker"],
            "single_marker_camera_ids": tier_cameras["single_marker"],
            "observed_marker_ids": sorted({
                int(marker_id) for record in accepted
                for marker_id in record.get("marker_ids", [])
            }),
            "observed_faces": sorted({
                str(face) for record in accepted
                for face in record.get("observed_faces", [])
            }),
            "has_core_multiface_observation": bool(
                tier_cameras["nonplanar_multiface"]),
        })
    return output


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
    redetection_attempt_count = redetection_selected_count = 0
    recovered_core_count = 0
    selected_detection_method_counts: Dict[str, int] = {}
    quality_records: List[Dict[str, Any]] = []

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
            quality_record: Dict[str, Any] = {
                "event_id": int(event),
                "camera_id": int(camera),
                "set_idx": (int(set_index) if set_index is not None else None),
                "is_gripper_camera": bool(camera == gripper_cam_idx),
                "detected_marker_ids": [],
                "marker_ids": [],
                "observed_faces": [],
                "observed_face_count": 0,
                "noncoplanar_face_count": 0,
                "is_planar": None,
                "quality_tier": "none",
                "pnp_rmse_px": None,
                "pnp_inlier_fraction": None,
                "pnp_solver": None,
                "positive_depth_candidate_count": 0,
                "pnp_candidate_count": 0,
                "pnp_accepted": False,
                "detection_method": "default",
                "redetection_attempted": False,
                "redetection_selected": False,
                "recovered_core_observation": False,
                "detection_candidate_count": 1,
                "status": "pending",
            }
            relative_path = camera_info.get("rgb_path", "")
            if not relative_path:
                images_missing += 1
                quality_record["status"] = "missing_rgb_path"
                quality_records.append(quality_record)
                continue
            image = cv2.imread(os.path.join(root, relative_path))
            if image is None:
                images_missing += 1
                quality_record["status"] = "unreadable_image"
                quality_records.append(quality_record)
                continue
            if image_scale != 1.0:
                interpolation = (
                    cv2.INTER_AREA if image_scale < 1.0 else cv2.INTER_CUBIC)
                image = cv2.resize(
                    image, None, fx=image_scale, fy=image_scale,
                    interpolation=interpolation,
                )
            images_read += 1
            min_aspect = (
                min_aspect_gripper if camera == gripper_cam_idx
                else min_aspect_fixed
            )
            max_error = (
                max_err_gripper if camera == gripper_cam_idx
                else max_err_fixed
            )
            try:
                corner_sets, marker_ids = cube.detect(image)
            except Exception:
                quality_record["status"] = "detection_error"
                quality_records.append(quality_record)
                continue
            baseline = _evaluate_detection_candidate(
                cube, corner_sets, marker_ids,
                method="default",
                image_scale=image_scale,
                min_aspect=min_aspect,
                K=K_map[camera],
                D=D_map[camera],
                max_error=max_error,
            )
            candidates = [baseline]
            redetection_attempted = False
            recovery_max_error = None
            recovery_method = getattr(
                cube, "detect_recovery_candidates", None)
            if (not _is_accepted_core_candidate(baseline)
                    and callable(recovery_method)):
                redetection_attempted = True
                redetection_attempt_count += 1
                # Rescue measurements must be clearly good, not merely slip
                # below the looser 5 px gripper-camera baseline threshold.
                recovery_max_error = min(float(max_error), 3.0)
                try:
                    recovery_detections = recovery_method(image)
                except Exception:
                    recovery_detections = []
                for method, recovery_corners, recovery_ids in recovery_detections:
                    candidates.append(_evaluate_detection_candidate(
                        cube, recovery_corners, recovery_ids,
                        method=str(method),
                        image_scale=image_scale,
                        min_aspect=min_aspect,
                        K=K_map[camera],
                        D=D_map[camera],
                        max_error=recovery_max_error,
                    ))

            selected_candidate = max(
                candidates, key=_detection_candidate_rank)
            redetection_selected = selected_candidate.method != "default"
            recovered_core = bool(
                redetection_selected
                and _is_accepted_core_candidate(selected_candidate)
                and not _is_accepted_core_candidate(baseline)
            )
            if redetection_selected:
                redetection_selected_count += 1
            if recovered_core:
                recovered_core_count += 1
            selected_detection_method_counts[selected_candidate.method] = (
                selected_detection_method_counts.get(
                    selected_candidate.method, 0) + 1
            )

            detections += len(selected_candidate.detected_marker_ids)
            aspect_rejections += int(selected_candidate.aspect_rejections)
            object_corner_failures += int(
                selected_candidate.object_corner_failures)
            support = selected_candidate.support
            pnp_quality = selected_candidate.pnp_quality
            quality_record.update({
                "detected_marker_ids": list(
                    selected_candidate.detected_marker_ids),
                "marker_ids": list(selected_candidate.marker_ids),
                "observed_faces": list(support["observed_faces"]),
                "observed_face_count": support["observed_face_count"],
                "noncoplanar_face_count": support["noncoplanar_face_count"],
                "is_planar": support["is_planar"],
                "quality_tier": support["quality_tier"],
                "detection_method": selected_candidate.method,
                "redetection_attempted": bool(redetection_attempted),
                "redetection_selected": bool(redetection_selected),
                "recovered_core_observation": bool(recovered_core),
                "redetection_max_rmse_px": recovery_max_error,
                "detection_candidate_count": int(len(candidates)),
                "baseline_detected_marker_ids": list(
                    baseline.detected_marker_ids),
                "baseline_marker_ids": list(baseline.marker_ids),
                "baseline_quality_tier": baseline.support["quality_tier"],
                "baseline_pnp_rmse_px": (
                    None if baseline.pnp_quality is None
                    else float(baseline.pnp_quality.rmse_px)
                ),
                "redetection_candidates": [
                    {
                        "method": candidate.method,
                        "marker_ids": list(candidate.marker_ids),
                        "quality_tier": candidate.support["quality_tier"],
                        "pnp_rmse_px": (
                            None if candidate.pnp_quality is None
                            else float(candidate.pnp_quality.rmse_px)
                        ),
                        "status": candidate.status,
                    }
                    for candidate in candidates
                ],
                "status": selected_candidate.status,
            })
            if pnp_quality is not None:
                quality_record.update({
                    "pnp_rmse_px": float(pnp_quality.rmse_px),
                    "pnp_inlier_fraction": float(
                        pnp_quality.inlier_fraction),
                    "pnp_solver": str(pnp_quality.solver),
                    "positive_depth_candidate_count": int(
                        pnp_quality.positive_depth_candidate_count),
                    "pnp_candidate_count": int(
                        pnp_quality.candidate_count),
                })

            if selected_candidate.status == "pnp_invalid":
                pnp_failures += 1
            elif selected_candidate.status == "pnp_rmse_rejected":
                pnp_error_rejections += 1
            elif selected_candidate.status == "accepted" and pnp_quality is not None:
                pnp_rmse = float(pnp_quality.rmse_px)
                pnp_solver = str(pnp_quality.solver)
                pnp_accepted_rmse.append(pnp_rmse)
                pnp_solver_counts[pnp_solver] = (
                    pnp_solver_counts.get(pnp_solver, 0) + 1)
                quality_record["pnp_accepted"] = True
                observations.append(CornerObservation(
                    cam=camera,
                    event=event,
                    set_idx=(int(set_index) if set_index is not None else None),
                    object_points=selected_candidate.object_points,
                    image_points=selected_candidate.image_points,
                    pnp_reprojection_rmse_px=pnp_rmse,
                    pnp_inlier_fraction=float(pnp_quality.inlier_fraction),
                    pnp_solver=pnp_solver,
                    grasp_idx=grasp,
                    marker_ids=support["marker_ids"],
                    observed_faces=support["observed_faces"],
                    observed_face_count=support["observed_face_count"],
                    noncoplanar_face_count=support["noncoplanar_face_count"],
                    is_planar=support["is_planar"],
                    positive_depth_candidate_count=int(
                        pnp_quality.positive_depth_candidate_count),
                    pnp_candidate_count=int(pnp_quality.candidate_count),
                    quality_tier=support["quality_tier"],
                ))
            quality_records.append(quality_record)

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
            "support_unit": "event_id_x_camera_id",
            "core_cube_observation": (
                "observed_face_count>=2_and_nonplanar_face_count>=2_and_"
                "is_planar=false_and_positive_depth_candidate_count>=1"),
            "planar_multimarker_label": (
                "two_or_more_marker_ids_but_rank2_object_points;_"
                "IDs_0+1_on_+Z_are_the_canonical_example"),
            "positive_depth_candidate_count": (
                "number_of_PnP_candidates_with_every_object_corner_at_z>0"),
            "offline_redetection": (
                "default_detection_is_preserved_when_core_valid;_otherwise_"
                "subpixel_relaxed_upscale_and_unsharp_candidates_are_ranked_"
                "only_after_cube_model_PnP_and_all_corner_RMSE_scoring"),
            "offline_redetection_max_rmse_px": 3.0,
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
            "redetection_attempts": int(redetection_attempt_count),
            "redetection_selected": int(redetection_selected_count),
            "recovered_core_observations": int(recovered_core_count),
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
        "selected_detection_method_counts": dict(sorted(
            selected_detection_method_counts.items())),
        "accepted_quality_tier_counts": dict(sorted(Counter(
            observation.quality_tier for observation in observations).items())),
        "observation_quality_by_event_camera": quality_records,
        "event_quality_summary": _summarize_quality_by_event(quality_records),
    }
    return observations, diagnostics
