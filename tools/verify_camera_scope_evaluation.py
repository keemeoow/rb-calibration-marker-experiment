#!/usr/bin/env python3
"""Verify both pre-GT camera-consistency evaluation contracts.

The exact synthetic scene checks Cross-view Pixel Transfer and
fixed-gripper cube consistency evaluation independently.
"""

from __future__ import annotations

import copy
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from calibration_pipeline.apriltag_cube import inv_T
from calibration_pipeline.path_evaluation import (
    FIXED_TO_FIXED_CROSS_TARGET_CONTRACT,
    GRIPPER_TO_FIXED_CROSS_TARGET_CONTRACT,
    _is_planar_points,
    build_fixed_to_fixed_cross_target_mask,
    build_gripper_to_fixed_cross_target_mask,
    evaluate_fixed_to_fixed_cross_target,
    evaluate_gripper_to_fixed_cross_target,
    validate_fixed_to_fixed_cross_target_mask,
    validate_gripper_to_fixed_cross_target_mask,
)
from calibration_pipeline.reprojection import PixelObs, project_points


def _transform(translation) -> np.ndarray:
    transform = np.eye(4, dtype=np.float64)
    transform[:3, 3] = np.asarray(translation, dtype=np.float64)
    return transform


def _observation(marker: str, camera: int, event: int, points: np.ndarray,
                 T_base_camera: np.ndarray, T_base_target: np.ndarray,
                 K: np.ndarray, D: np.ndarray) -> PixelObs:
    return PixelObs(
        marker=marker,
        cam=int(camera),
        event=int(event),
        set_idx=5,
        object_points=np.asarray(points, dtype=np.float64),
        image_points=project_points(
            inv_T(T_base_camera) @ T_base_target, points, K, D),
    )


def _assert_target_metrics(result: dict, pairs: int, directions: int,
                           label: str) -> None:
    for target in ("board", "cube"):
        metrics = result["by_target"][target]
        if metrics["n_pairs"] != pairs or metrics["n_directions"] != directions:
            raise AssertionError(f"{label}/{target}: pair population mismatch")
        if metrics["cross_view_pixel_transfer_rmse_px"] > 1e-4:
            raise AssertionError(f"{label}/{target}: exact pixel transfer failed")
        if metrics["pose_consistency_translation_rmse_mm"] > 1e-4:
            raise AssertionError(f"{label}/{target}: exact pose consistency failed")


def main() -> None:
    K = np.array([
        [800.0, 0.0, 640.0],
        [0.0, 805.0, 360.0],
        [0.0, 0.0, 1.0],
    ])
    D = np.zeros(5, dtype=np.float64)
    K_map = {camera: K.copy() for camera in (0, 1, 2)}
    D_map = {camera: D.copy() for camera in (0, 1, 2)}
    cameras = {
        0: _transform((0.0, 0.0, 0.0)),
        1: _transform((0.30, 0.02, 0.0)),
    }
    points_by_target = {
        "board": np.array([
            [-0.12, -0.08, 0.0], [0.0, -0.08, 0.0], [0.12, -0.08, 0.0],
            [-0.12, 0.0, 0.0], [0.0, 0.0, 0.0], [0.12, 0.0, 0.0],
            [-0.12, 0.08, 0.0], [0.0, 0.08, 0.0], [0.12, 0.08, 0.0],
        ], dtype=np.float64),
        "cube": np.array([
            [-0.05, -0.05, -0.05], [-0.05, -0.05, 0.05],
            [-0.05, 0.05, -0.05], [-0.05, 0.05, 0.05],
            [0.05, -0.05, -0.05], [0.05, -0.05, 0.05],
            [0.05, 0.05, -0.05], [0.05, 0.05, 0.05],
        ], dtype=np.float64),
    }
    target_poses = {
        "board": _transform((0.03, -0.04, 1.4)),
        "cube": _transform((-0.08, 0.06, 1.2)),
    }
    event_by_target = {"board": 10, "cube": 20}
    gripper_camera = 2
    T_gripper_camera = _transform((0.04, -0.02, 0.08))
    robot_T = {
        10: _transform((0.10, -0.12, 0.03)),
        20: _transform((-0.05, -0.10, 0.02)),
    }
    observations = []
    for target, points in points_by_target.items():
        event = event_by_target[target]
        for camera in sorted(cameras):
            observations.append(_observation(
                target, camera, event, points, cameras[camera],
                target_poses[target], K_map[camera], D_map[camera]))
        observations.append(_observation(
            target, gripper_camera, event, points,
            robot_T[event] @ T_gripper_camera, target_poses[target],
            K_map[gripper_camera], D_map[gripper_camera]))

    fixed_mask = build_fixed_to_fixed_cross_target_mask(
        observations, cameras, K_map, D_map, set_filter=[5])
    fixed_exact = evaluate_fixed_to_fixed_cross_target(
        observations, cameras, K_map, D_map, fixed_mask)
    _assert_target_metrics(fixed_exact, 1, 2, "Cross-view Pixel Transfer")

    perturbed_cameras = {
        camera: transform.copy() for camera, transform in cameras.items()}
    perturbed_cameras[1][0, 3] += 0.02
    fixed_changed = evaluate_fixed_to_fixed_cross_target(
        observations, perturbed_cameras, K_map, D_map, fixed_mask)
    for target in ("board", "cube"):
        metrics = fixed_changed["by_target"][target]
        if metrics["pose_consistency_translation_rmse_mm"] < 19.9:
            raise AssertionError(
                f"Cross-view Pixel Transfer/{target}: perturbation undetected")
        if metrics["n_output_rejected"] != 0:
            raise AssertionError(
                f"Cross-view Pixel Transfer/{target}: output was rejected")

    gripper_mask = build_gripper_to_fixed_cross_target_mask(
        observations, cameras, gripper_camera, K_map, D_map, set_filter=[5])
    gripper_exact = evaluate_gripper_to_fixed_cross_target(
        observations, cameras, T_gripper_camera, robot_T,
        K_map, D_map, gripper_mask)
    _assert_target_metrics(gripper_exact, 2, 4, "Fixed-Gripper")

    perturbed_handeye = T_gripper_camera.copy()
    perturbed_handeye[0, 3] += 0.02
    gripper_changed = evaluate_gripper_to_fixed_cross_target(
        observations, cameras, perturbed_handeye, robot_T,
        K_map, D_map, gripper_mask)
    for target in ("board", "cube"):
        metrics = gripper_changed["by_target"][target]
        if metrics["pose_consistency_translation_rmse_mm"] < 19.9:
            raise AssertionError(f"Fixed-Gripper/{target}: perturbation undetected")
        if metrics["n_output_rejected"] != 0:
            raise AssertionError(f"Fixed-Gripper/{target}: output was rejected")

    for mask, validator in (
            (fixed_mask, validate_fixed_to_fixed_cross_target_mask),
            (gripper_mask, validate_gripper_to_fixed_cross_target_mask)):
        tampered = copy.deepcopy(mask)
        tampered["pairs"].pop()
        try:
            validator(tampered)
        except ValueError:
            pass
        else:
            raise AssertionError("tampered frozen evaluation mask was accepted")

    arbitrary_plane = np.array([
        [0.1, -0.1, -0.1], [0.1, 0.1, -0.1],
        [0.1, 0.1, 0.1], [0.1, -0.1, 0.1],
    ])
    if not _is_planar_points(arbitrary_plane):
        raise AssertionError("an arbitrarily oriented plane was misclassified")

    if any(FIXED_TO_FIXED_CROSS_TARGET_CONTRACT[key] is not False for key in (
            "uses_shared_base_target_pose", "uses_robot_fk",
            "uses_gripper_camera", "uses_external_ground_truth")):
        raise AssertionError(
            "Cross-view Pixel Transfer dependency contract is incorrect")
    if (GRIPPER_TO_FIXED_CROSS_TARGET_CONTRACT["uses_robot_fk"] is not True
            or GRIPPER_TO_FIXED_CROSS_TARGET_CONTRACT[
                "uses_shared_base_target_pose"] is not False
            or GRIPPER_TO_FIXED_CROSS_TARGET_CONTRACT[
                "uses_external_ground_truth"] is not False):
        raise AssertionError("Fixed-Gripper dependency contract is incorrect")

    print("[PASS] Cross-view Pixel Transfer and "
          "fixed-gripper cube consistency contracts")


if __name__ == "__main__":
    main()
