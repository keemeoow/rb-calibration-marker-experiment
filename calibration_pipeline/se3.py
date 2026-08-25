"""Minimal SE(3) averaging and capture-meta pose loading.

These functions support initialization only.  Final calibration is performed
by :mod:`calibration_pipeline.reprojection` from raw image corners.
"""

from __future__ import annotations

from typing import Any, Dict, List, Optional, Tuple

import numpy as np
from scipy.spatial.transform import Rotation

from calibration_pipeline.apriltag_cube import inv_T
from calibration_pipeline.runtime import (
    get_capture_set_cube_center_transform_raw,
    get_capture_set_index,
    rotation_error_deg,
)
from capture_pipeline.robot import euler_deg_to_matrix


def _se3_residual(transform: np.ndarray,
                  rotation_scale_m_per_rad: float = 0.05) -> np.ndarray:
    residual = np.zeros(6, dtype=np.float64)
    residual[:3] = transform[:3, 3]
    residual[3:] = (
        Rotation.from_matrix(transform[:3, :3]).as_rotvec()
        * float(rotation_scale_m_per_rad)
    )
    return residual


def _weighted_average(transforms: List[np.ndarray],
                      weights: Optional[List[float]] = None) -> np.ndarray:
    if not transforms:
        raise ValueError("SE(3) average received no transforms")
    weight_array = (
        np.ones(len(transforms), dtype=np.float64)
        if weights is None
        else np.maximum(np.asarray(weights, dtype=np.float64), 1e-12)
    )
    weight_array /= weight_array.sum() + 1e-12
    translation = np.sum(
        np.stack([transform[:3, 3] for transform in transforms])
        * weight_array[:, None],
        axis=0,
    )
    rotation_sum = np.sum(
        np.stack([transform[:3, :3] for transform in transforms])
        * weight_array[:, None, None],
        axis=0,
    )
    left, _, right_t = np.linalg.svd(rotation_sum)
    rotation = left @ right_t
    if np.linalg.det(rotation) < 0:
        left[:, -1] *= -1.0
        rotation = left @ right_t
    result = np.eye(4, dtype=np.float64)
    result[:3, :3] = rotation
    result[:3, 3] = translation
    return result


def robust_se3_average(
    transforms: List[np.ndarray],
    weights: Optional[List[float]] = None,
    max_iters: int = 5,
    k_mad: float = 2.5,
) -> Tuple[np.ndarray, Dict[str, float]]:
    """MAD-trimmed SE(3) initializer with translation/rotation diagnostics."""
    if not transforms:
        raise ValueError("robust_se3_average received no transforms")
    resolved_weights = [1.0] * len(transforms) if weights is None else weights
    inliers = np.arange(len(transforms), dtype=int)
    average = _weighted_average(transforms, resolved_weights)

    for _ in range(max_iters):
        errors = []
        for index in inliers:
            residual = _se3_residual(inv_T(average) @ transforms[index])
            errors.append(float(
                np.linalg.norm(residual[:3]) * 1000.0
                + np.linalg.norm(residual[3:]) * 1000.0
            ))
        errors = np.asarray(errors, dtype=np.float64)
        median = float(np.median(errors))
        mad = float(np.median(np.abs(errors - median)) + 1e-12)
        keep = errors <= median + k_mad * 1.4826 * mad
        if keep.sum() < max(3, int(0.4 * len(inliers))):
            break
        new_inliers = inliers[keep]
        if len(new_inliers) == len(inliers):
            break
        inliers = new_inliers
        average = _weighted_average(
            [transforms[index] for index in inliers],
            [resolved_weights[index] for index in inliers],
        )

    selected = [transforms[index] for index in inliers]
    average = _weighted_average(
        selected, [resolved_weights[index] for index in inliers])
    translation_errors = [
        float(np.linalg.norm(transform[:3, 3] - average[:3, 3]) * 1000.0)
        for transform in selected
    ]
    rotation_errors = [
        rotation_error_deg(transform[:3, :3], average[:3, :3])
        for transform in selected
    ]
    return average, {
        "num_total": int(len(transforms)),
        "num_inliers": int(len(inliers)),
        "inlier_ratio": float(len(inliers) / max(1, len(transforms))),
        "translation_std_mm": float(np.std(translation_errors)),
        "rotation_std_deg": float(np.std(rotation_errors)),
    }


def _parse_pose6(value: Any) -> Optional[List[float]]:
    if value is None:
        return None
    if isinstance(value, list) and len(value) == 6:
        try:
            return [float(item) for item in value]
        except (TypeError, ValueError):
            return None
    if isinstance(value, dict):
        keys = ("x", "y", "z", "rz", "ry", "rx")
        if all(key in value for key in keys):
            return [float(value[key]) for key in keys]
        for key in ("robot_pose_6dof", "tcp_pose_6dof", "pose_6dof", "pose"):
            parsed = _parse_pose6(value.get(key))
            if parsed is not None:
                return parsed
    return None


def _parse_transform(value: Any) -> Optional[np.ndarray]:
    if value is None:
        return None
    if isinstance(value, list):
        array = np.asarray(value, dtype=np.float64)
        if array.shape == (4, 4):
            return array
        if array.size == 16:
            return array.reshape(4, 4)
    if isinstance(value, dict):
        for key in ("T_B_G", "robot_pose_matrix_4x4", "matrix", "transform"):
            parsed = _parse_transform(value.get(key))
            if parsed is not None:
                return parsed
    return None


def _pose6_to_transform(pose: List[float]) -> np.ndarray:
    # Project convention: [x,y,z mm, rz,ry,rx deg].
    return euler_deg_to_matrix(*[float(value) for value in pose])


def load_nominal_set_cube_transforms(meta: Dict[str, Any]) -> Dict[int, np.ndarray]:
    transforms: Dict[int, np.ndarray] = {}
    for capture in meta.get("captures", []):
        set_index = get_capture_set_index(capture)
        if set_index is None or set_index in transforms:
            continue
        transform = get_capture_set_cube_center_transform_raw(capture)
        if transform is not None:
            transforms[int(set_index)] = np.asarray(
                transform, dtype=np.float64).reshape(4, 4)
    return transforms


def load_robot_poses_from_meta(meta: Dict[str, Any]) -> Dict[int, np.ndarray]:
    """Load canonical ``T_base_gripper``; stored 4x4 matrices take priority."""
    transforms: Dict[int, np.ndarray] = {}
    for capture in meta.get("captures", []):
        event = int(capture.get("event_id", -1))
        if event < 0:
            continue
        transform = _parse_transform(capture.get(
            "canonical_robot_pose_matrix_4x4"))
        if transform is None:
            transform = _parse_transform(capture.get("robot_pose_matrix_4x4"))
        if transform is None:
            transform = _parse_transform(capture.get(
                "capture_gripper_pose_matrix_4x4",
                capture.get("capture_pose_matrix_4x4"),
            ))
        if transform is None:
            pose = None
            for key in (
                "robot_pose_6dof", "capture_gripper_pose_6dof",
                "capture_pose_6dof", "tcp_pose_6dof", "pose_6dof",
                "robot_pose",
            ):
                pose = _parse_pose6(capture.get(key))
                if pose is not None:
                    break
            if pose is not None:
                transform = _pose6_to_transform(pose)
        if transform is not None:
            transforms[event] = np.asarray(transform, dtype=np.float64)
    return transforms
