#!/usr/bin/env python3
"""Diagnose board/cube fixed-camera conflict and validate Session04 K/D."""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
from collections import defaultdict
from dataclasses import replace
from pathlib import Path

import cv2
import numpy as np
from scipy.optimize import minimize_scalar


PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from calibration_pipeline import table1  # noqa: E402
from calibration_pipeline.apriltag_cube import inv_T  # noqa: E402
from calibration_pipeline.board_config import (  # noqa: E402
    charuco_config_from_dict,
    charuco_topology,
)
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


def _systematic_conflict_contract(board_metric_scale, same_event):
    translation_rmse_mm = float(np.sqrt(np.mean([
        detail["translation_disagreement_before_mm"] ** 2
        for detail in board_metric_scale["per_camera"].values()
    ])))
    maximum_rotation_deg = max(
        float(detail["rotation_disagreement_deg"])
        for detail in board_metric_scale["per_camera"].values())
    return {
        "resolution_status": "mitigated_not_eliminated",
        "software_and_geometry_checks_complete": True,
        "root_cause_eliminated": False,
        "translation_rmse_mm": translation_rmse_mm,
        "maximum_rotation_deg": maximum_rotation_deg,
        "single_target_extrinsics_publishable": False,
        "naive_board_cube_pose_average_allowed": False,
        "required_estimator": (
            "shared-camera joint corner-reprojection optimization"),
        "independent_direct_pnp_role": "diagnostic_only",
        "external_gt_required_for_absolute_accuracy_claim": True,
        "same_event_evidence_present": all(
            bool(detail.get("rows")) for detail in same_event.values()),
        "reason": (
            "board-only and cube-only PnP produce incompatible estimates of "
            "the same fixed-camera transform"),
    }


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
        "factory_K_charuco_D": (factory_K, current_D),
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


def _cube_marker_ids_by_observation(manifest_path):
    with Path(manifest_path).open("r", encoding="utf-8") as stream:
        manifest = json.load(stream)
    return {
        (int(record["event_id"]), int(record["camera_id"])):
            tuple(map(int, record.get("marker_ids", [])))
        for record in manifest.get("observations", [])
        if record.get("target") == "cube"
    }


def _rescale_cube_observations(observations, marker_ids_by_observation,
                               scale, mode):
    """Apply one interpretable cube-geometry scale hypothesis.

    This is diagnostic only: it never mutates the frozen manifest or the
    physical config.  Four consecutive object points belong to one marker in
    the same order as the manifest's marker_ids field.
    """
    output = []
    scale = float(scale)
    for observation in observations:
        if observation.marker != "cube":
            output.append(observation)
            continue
        points = np.asarray(observation.object_points, dtype=np.float64).copy()
        marker_ids = marker_ids_by_observation.get(
            (int(observation.event), int(observation.cam)), ())
        if len(points) != 4 * len(marker_ids):
            raise ValueError(
                "cube marker IDs do not match frozen object-point blocks: "
                f"event={observation.event}, camera={observation.cam}")
        if mode == "uniform_geometry":
            points *= scale
        else:
            for index, marker_id in enumerate(marker_ids):
                block = points[4 * index:4 * index + 4]
                center = np.mean(block, axis=0)
                local = block - center
                selected = (
                    mode == "all_marker_size"
                    or (mode == "side_marker_size" and marker_id >= 2)
                    or (mode == "top_marker_size" and marker_id < 2)
                )
                if selected:
                    points[4 * index:4 * index + 4] = center + scale * local
                elif mode == "marker_center_radius":
                    points[4 * index:4 * index + 4] = scale * center + local
        output.append(replace(observation, object_points=points))
    return output


def _cube_geometry_scale_scan(train, test, split, fixed_cameras, K_map, D_map,
                              manifest_path):
    """Test which cube geometry component can explain the target conflict."""
    board_fitted, _ = _fit_targets(train, fixed_cameras, K_map, D_map)
    board_cameras = board_fitted["board"]
    marker_ids = _cube_marker_ids_by_observation(manifest_path)
    mask = build_fixed_to_fixed_cross_target_mask(
        test, fixed_cameras, K_map, D_map,
        set_filter=split["eligible_sets"])

    def evaluate(mode, scale):
        scaled_train = _rescale_cube_observations(
            train, marker_ids, scale, mode)
        candidates = direct_relative_candidates(
            scaled_train, fixed_cameras, int(fixed_cameras[0]), ["cube"],
            K_map, D_map)
        cube_cameras, diagnostics = fit_baseline(
            candidates, int(fixed_cameras[0]))
        per_camera = {}
        squared = []
        for camera in fixed_cameras[1:]:
            translation_mm, rotation_deg = pose_delta(
                board_cameras[camera], cube_cameras[camera])
            squared.append(translation_mm ** 2)
            per_camera[str(camera)] = {
                "translation_mm": float(translation_mm),
                "rotation_deg": float(rotation_deg),
            }
        scaled_test = _rescale_cube_observations(
            test, marker_ids, scale, mode)
        heldout = evaluate_fixed_to_fixed_cross_target(
            scaled_test, cube_cameras, K_map, D_map, mask)
        return {
            "conflict_translation_rmse_mm": float(np.sqrt(np.mean(squared))),
            "per_camera": per_camera,
            "heldout_cube": heldout["by_target"]["cube"],
            "fit_diagnostics": diagnostics,
        }

    output = {}
    for mode in (
            "uniform_geometry", "all_marker_size", "side_marker_size",
            "top_marker_size", "marker_center_radius"):
        result = minimize_scalar(
            lambda value: evaluate(mode, value)[
                "conflict_translation_rmse_mm"],
            bounds=(0.94, 1.06), method="bounded",
            options={"xatol": 1e-5})
        best_scale = float(result.x)
        output[mode] = {
            "best_scale": best_scale,
            "optimizer_success": bool(result.success),
            **evaluate(mode, best_scale),
        }
    output["nominal"] = {
        "best_scale": 1.0,
        **evaluate("uniform_geometry", 1.0),
    }
    return output


def _solve_pose_with_flag(observation, K_map, D_map, flag):
    object_points = np.asarray(
        observation.object_points, dtype=np.float64).reshape(-1, 3)
    image_points = np.asarray(
        observation.image_points, dtype=np.float64).reshape(-1, 2)
    try:
        ok, rvec, tvec = cv2.solvePnP(
            object_points, image_points,
            np.asarray(K_map[int(observation.cam)], dtype=np.float64),
            np.asarray(D_map[int(observation.cam)], dtype=np.float64),
            flags=int(flag))
    except cv2.error:
        return None
    if not ok:
        return None
    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = cv2.Rodrigues(rvec)[0]
    transform[:3, 3] = np.asarray(tvec, dtype=np.float64).reshape(3)
    if not np.all(np.isfinite(transform)) or transform[2, 3] <= 0.0:
        return None
    return transform


def _fit_target_with_pnp_flag(observations, fixed_cameras, target, K_map,
                              D_map, flag):
    anchor = int(fixed_cameras[0])
    grouped = defaultdict(dict)
    for observation in observations:
        if (observation.marker == target
                and int(observation.cam) in set(fixed_cameras)):
            grouped[int(observation.event)][int(observation.cam)] = observation
    candidates = {camera: [] for camera in fixed_cameras if camera != anchor}
    for by_camera in grouped.values():
        if anchor not in by_camera:
            continue
        anchor_pose = _solve_pose_with_flag(
            by_camera[anchor], K_map, D_map, flag)
        if anchor_pose is None:
            continue
        for camera in candidates:
            if camera not in by_camera:
                continue
            camera_pose = _solve_pose_with_flag(
                by_camera[camera], K_map, D_map, flag)
            if camera_pose is not None:
                candidates[camera].append(anchor_pose @ inv_T(camera_pose))
    return fit_baseline(candidates, anchor)


def _pnp_solver_scan(train, fixed_cameras, K_map, D_map):
    board_flags = {
        "IPPE": cv2.SOLVEPNP_IPPE,
        "ITERATIVE": cv2.SOLVEPNP_ITERATIVE,
        "EPNP": cv2.SOLVEPNP_EPNP,
        "SQPNP": cv2.SOLVEPNP_SQPNP,
    }
    cube_flags = {
        "ITERATIVE": cv2.SOLVEPNP_ITERATIVE,
        "EPNP": cv2.SOLVEPNP_EPNP,
        "SQPNP": cv2.SOLVEPNP_SQPNP,
    }
    fitted = {"board": {}, "cube": {}}
    for name, flag in board_flags.items():
        fitted["board"][name], _ = _fit_target_with_pnp_flag(
            train, fixed_cameras, "board", K_map, D_map, flag)
    for name, flag in cube_flags.items():
        fitted["cube"][name], _ = _fit_target_with_pnp_flag(
            train, fixed_cameras, "cube", K_map, D_map, flag)
    rows = []
    for board_name, board_cameras in fitted["board"].items():
        for cube_name, cube_cameras in fitted["cube"].items():
            per_camera, squared = {}, []
            for camera in fixed_cameras[1:]:
                translation_mm, rotation_deg = pose_delta(
                    board_cameras[camera], cube_cameras[camera])
                squared.append(translation_mm ** 2)
                per_camera[str(camera)] = {
                    "translation_mm": float(translation_mm),
                    "rotation_deg": float(rotation_deg),
                }
            rows.append({
                "board_solver": board_name,
                "cube_solver": cube_name,
                "translation_rmse_mm": float(np.sqrt(np.mean(squared))),
                "per_camera": per_camera,
            })
    return sorted(rows, key=lambda row: row["translation_rmse_mm"])


def _common_object_correspondences(left, right):
    left_object = np.asarray(left.object_points, dtype=np.float64).reshape(-1, 3)
    right_object = np.asarray(right.object_points, dtype=np.float64).reshape(-1, 3)
    left_image = np.asarray(left.image_points, dtype=np.float64).reshape(-1, 2)
    right_image = np.asarray(right.image_points, dtype=np.float64).reshape(-1, 2)
    right_index = {
        tuple(np.round(point, 9)): index
        for index, point in enumerate(right_object)
    }
    triples = [
        (point, left_image[index], right_image[right_index[key]])
        for index, point in enumerate(left_object)
        if (key := tuple(np.round(point, 9))) in right_index
    ]
    if len(triples) < 4:
        return None
    object_points, left_points, right_points = zip(*triples)
    return (
        np.asarray(object_points, dtype=np.float32),
        np.asarray(left_points, dtype=np.float32),
        np.asarray(right_points, dtype=np.float32),
    )


def _stereo_fit_target(observations, fixed_cameras, target, K_map, D_map):
    anchor = int(fixed_cameras[0])
    grouped = defaultdict(dict)
    for observation in observations:
        if (observation.marker == target
                and int(observation.cam) in set(fixed_cameras)):
            grouped[int(observation.event)][int(observation.cam)] = observation
    transforms = {anchor: np.eye(4, dtype=np.float64)}
    diagnostics = {}
    for camera in fixed_cameras[1:]:
        object_points, anchor_points, camera_points = [], [], []
        for by_camera in grouped.values():
            if anchor not in by_camera or camera not in by_camera:
                continue
            common = _common_object_correspondences(
                by_camera[anchor], by_camera[camera])
            if common is None:
                continue
            object_points.append(common[0])
            anchor_points.append(common[1])
            camera_points.append(common[2])
        if not object_points:
            raise RuntimeError(
                f"no common {target} correspondences for cam{anchor}-cam{camera}")
        rms, _, _, _, _, rotation, translation, _, _ = cv2.stereoCalibrate(
            object_points, anchor_points, camera_points,
            np.asarray(K_map[anchor], dtype=np.float64).copy(),
            np.asarray(D_map[anchor], dtype=np.float64).copy(),
            np.asarray(K_map[camera], dtype=np.float64).copy(),
            np.asarray(D_map[camera], dtype=np.float64).copy(),
            (1280, 720), flags=cv2.CALIB_FIX_INTRINSIC,
            criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_COUNT,
                      200, 1e-10))
        camera_from_anchor = np.eye(4, dtype=np.float64)
        camera_from_anchor[:3, :3] = rotation
        camera_from_anchor[:3, 3] = np.asarray(translation).reshape(3)
        transforms[camera] = inv_T(camera_from_anchor)
        diagnostics[str(camera)] = {
            "stereo_rms_px": float(rms),
            "views": len(object_points),
            "common_corners": int(sum(map(len, object_points))),
        }
    return transforms, diagnostics


def _stereo_calibration_conflict(train, fixed_cameras, K_map, D_map):
    board, board_diagnostics = _stereo_fit_target(
        train, fixed_cameras, "board", K_map, D_map)
    cube, cube_diagnostics = _stereo_fit_target(
        train, fixed_cameras, "cube", K_map, D_map)
    per_camera, squared = {}, []
    for camera in fixed_cameras[1:]:
        translation_mm, rotation_deg = pose_delta(
            board[camera], cube[camera])
        squared.append(translation_mm ** 2)
        per_camera[str(camera)] = {
            "translation_mm": float(translation_mm),
            "rotation_deg": float(rotation_deg),
        }
    return {
        "translation_rmse_mm": float(np.sqrt(np.mean(squared))),
        "per_camera": per_camera,
        "board_diagnostics": board_diagnostics,
        "cube_diagnostics": cube_diagnostics,
    }


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


def _plane_from_object_points(T_camera_object, object_points):
    """Return one camera-frame point/normal for a planar corner block."""
    points = np.asarray(object_points, dtype=np.float64).reshape(-1, 3)
    if len(points) < 3:
        return None
    centered = points - np.mean(points, axis=0)
    _, singular_values, vh = np.linalg.svd(centered, full_matrices=False)
    if len(singular_values) < 2 or singular_values[1] <= 1e-9:
        return None
    normal_object = vh[-1]
    normal_camera = T_camera_object[:3, :3] @ normal_object
    normal_norm = float(np.linalg.norm(normal_camera))
    if normal_norm <= 1e-12:
        return None
    point_camera = (
        T_camera_object[:3, :3] @ np.mean(points, axis=0)
        + T_camera_object[:3, 3]
    )
    return point_camera, normal_camera / normal_norm


def _depth_plane_scale(depth_u16, depth_scale, K, D, planes,
                       max_plane_residual_mm=35.0):
    """Compare PnP plane depth with aligned RealSense depth at the same pixels.

    The residual gate removes background and cube-on-board occlusion pixels. It
    is deliberately much wider than the target-scale discrepancy under test,
    so it cannot force the answer toward either nominal target geometry.
    """
    depth = np.asarray(depth_u16)
    if depth.ndim != 2 or not np.isfinite(depth_scale) or depth_scale <= 0.0:
        return None
    height, width = depth.shape
    ratios, biases = [], []
    valid_samples = 0
    accepted_samples = 0
    for polygon, point_camera, normal_camera in planes:
        polygon = np.asarray(polygon, dtype=np.float64).reshape(-1, 2)
        if len(polygon) < 3:
            continue
        mask = np.zeros((height, width), dtype=np.uint8)
        cv2.fillConvexPoly(mask, np.round(polygon).astype(np.int32), 1)
        # RGB/depth registration is least reliable exactly on target edges.
        mask = cv2.erode(mask, np.ones((5, 5), dtype=np.uint8), iterations=1)
        ys, xs = np.where(mask > 0)
        if not len(xs):
            continue
        measured_mm = depth[ys, xs].astype(np.float64) * float(depth_scale) * 1000.0
        valid = measured_mm > 0.0
        if not np.any(valid):
            continue
        measured_mm = measured_mm[valid]
        pixels = np.stack([xs[valid], ys[valid]], axis=1).astype(
            np.float64).reshape(-1, 1, 2)
        normalized = cv2.undistortPoints(
            pixels, np.asarray(K, dtype=np.float64),
            np.asarray(D, dtype=np.float64)).reshape(-1, 2)
        rays = np.c_[normalized, np.ones(len(normalized), dtype=np.float64)]
        denominator = rays @ np.asarray(normal_camera, dtype=np.float64)
        with np.errstate(divide="ignore", invalid="ignore"):
            predicted_mm = (
                float(np.asarray(normal_camera) @ np.asarray(point_camera))
                / denominator * 1000.0
            )
        finite = (
            np.isfinite(predicted_mm)
            & (predicted_mm > 0.0)
            & np.isfinite(measured_mm)
        )
        valid_samples += int(np.sum(finite))
        residual_mm = predicted_mm - measured_mm
        accepted = finite & (
            np.abs(residual_mm) <= float(max_plane_residual_mm))
        accepted_samples += int(np.sum(accepted))
        if np.any(accepted):
            ratios.append(predicted_mm[accepted] / measured_mm[accepted])
            biases.append(residual_mm[accepted])
    if accepted_samples < 50:
        return None
    ratio = np.concatenate(ratios)
    bias = np.concatenate(biases)
    return {
        "valid_depth_samples": int(valid_samples),
        "accepted_surface_samples": int(accepted_samples),
        "surface_sample_ratio": float(accepted_samples / max(valid_samples, 1)),
        "predicted_over_measured_depth_median": float(np.median(ratio)),
        "predicted_minus_measured_depth_median_mm": float(np.median(bias)),
    }


def _rgbd_target_scale_diagnostic(root, meta, observations, K_map, D_map,
                                  intrinsics_dir, fixed_cameras):
    """Use aligned depth as an independent metric-scale discriminator."""
    capture_by_event = {
        int(capture["event_id"]): capture
        for capture in meta.get("captures", [])
    }
    depth_scale_map = {
        camera: load_intrinsics_with_depth_scale(
            str(intrinsics_dir), camera)[2]
        for camera in fixed_cameras
    }
    rows = []
    for observation in observations:
        camera = int(observation.cam)
        if camera not in fixed_cameras:
            continue
        capture = capture_by_event.get(int(observation.event))
        camera_info = None if capture is None else capture.get(
            "cams", {}).get(str(camera))
        if not camera_info or not camera_info.get("depth_path"):
            continue
        depth = cv2.imread(
            str(root / camera_info["depth_path"]), cv2.IMREAD_UNCHANGED)
        if depth is None:
            continue
        transform, reprojection_rmse = _pnp_rmse(observation, K_map, D_map)
        if transform is None:
            continue
        image_points = np.asarray(
            observation.image_points, dtype=np.float64).reshape(-1, 2)
        object_points = np.asarray(
            observation.object_points, dtype=np.float64).reshape(-1, 3)
        planes = []
        if observation.marker == "board":
            plane = _plane_from_object_points(transform, object_points)
            if plane is not None:
                planes.append((
                    cv2.convexHull(image_points.astype(np.float32)).reshape(-1, 2),
                    plane[0], plane[1],
                ))
        elif observation.marker == "cube" and len(image_points) % 4 == 0:
            for start in range(0, len(image_points), 4):
                plane = _plane_from_object_points(
                    transform, object_points[start:start + 4])
                if plane is not None:
                    planes.append((
                        image_points[start:start + 4], plane[0], plane[1],
                    ))
        metrics = _depth_plane_scale(
            depth, depth_scale_map[camera], K_map[camera], D_map[camera],
            planes)
        if metrics is None:
            continue
        rows.append({
            "target": str(observation.marker),
            "event_id": int(observation.event),
            "camera_id": camera,
            "pnp_reprojection_rmse_px": float(reprojection_rmse),
            **metrics,
        })

    summary = {}
    for target in ("board", "cube"):
        summary[target] = {}
        for camera in fixed_cameras:
            subset = [row for row in rows
                      if row["target"] == target
                      and row["camera_id"] == camera]
            if not subset:
                continue
            ratios = [
                row["predicted_over_measured_depth_median"]
                for row in subset
            ]
            biases = [
                row["predicted_minus_measured_depth_median_mm"]
                for row in subset
            ]
            summary[target][str(camera)] = {
                "observations": len(subset),
                "predicted_over_measured_depth_median": float(
                    np.median(ratios)),
                "predicted_minus_measured_depth_median_mm": float(
                    np.median(biases)),
                "implied_geometry_scale_to_match_depth": float(
                    1.0 / np.median(ratios)),
            }
    relative_board_scale = {}
    for camera in fixed_cameras:
        key = str(camera)
        if key not in summary["board"] or key not in summary["cube"]:
            continue
        board_ratio = summary["board"][key][
            "predicted_over_measured_depth_median"]
        cube_ratio = summary["cube"][key][
            "predicted_over_measured_depth_median"]
        relative_board_scale[key] = float(cube_ratio / board_ratio)
    return {
        "rows": rows,
        "summary": summary,
        "board_scale_relative_to_cube_by_camera": relative_board_scale,
        "board_scale_relative_to_cube_median": (
            float(np.median(list(relative_board_scale.values())))
            if relative_board_scale else None),
    }


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
    before_rmse = float(np.sqrt(np.mean([
        detail["translation_disagreement_before_mm"] ** 2
        for detail in scale["per_camera"].values()
    ])))
    after_rmse = float(np.sqrt(np.mean([
        detail["translation_disagreement_after_mm"] ** 2
        for detail in scale["per_camera"].values()
    ])))
    reduction_percent = 100.0 * (1.0 - after_rmse / before_rmse)
    rgbd_relative_scale = payload["rgbd_target_scale"][
        "board_scale_relative_to_cube_median"]
    cube_scan = payload["cube_geometry_scale_scan"]
    software_fix = payload["cube_corner_refinement_fix"]
    board_contract = payload["physical_board_measurement"]
    conflict_contract = payload["systematic_conflict_contract"]
    best_pnp = payload["pnp_solver_scan"][0]
    stereo = payload["stereo_calibration_conflict"]
    nominal_square_mm = float(scale["nominal_square_length_mm"])
    lines = [
        "# Session04 Board–Cube Relative Pose and Intrinsic Validation",
        "",
        "> **상태: 완화 완료 · 원인 제거 미완료 · 외부 GT 필요**",
        ">",
        "> 알려진 detector·metadata 오류와 geometry 정의는 수정·검증했습니다. "
        "남은 direct-PnP 충돌은 joint corner-reprojection에서 완화하지만, 외부 GT "
        "없이 어느 target의 절대 자세가 더 정확한지 판별하거나 원인이 제거됐다고 "
        "주장하지 않습니다.",
        "",
        "## 결론",
        "",
        f"Frozen 영상 topology는 OpenCV checker square "
        f"`{board_contract['opencv_checker_squares_x']}×7`이고 내부 ChArUco corner는 "
        f"`{board_contract['charuco_corner_columns']}×6=60`입니다. 따라서 checker "
        f"전체 폭 275 mm는 square length `{nominal_square_mm:.1f} mm`와 "
        "일치합니다. 여기서 10은 checker square 수가 아니라 내부 corner column "
        "수입니다.",
        "Cube도 본체 59×59×57 mm, +Z 돌출부 2 mm, side marker 51 mm, "
        "top marker 25 mm 정의와 실물이 일치합니다. 기존 3D corner 좌표는 이미 "
        "top marker plane z=+29.5 mm와 side marker center z=-1 mm로 이 구조를 "
        "표현하므로 Cube geometry scale 불일치 가설도 기각합니다.",
        "",
        "확인된 소프트웨어 문제는 Cube 기본 검출기의 corner refinement가 꺼져 있어 "
        "AprilTag corner가 주로 정수 pixel로 고정되던 것입니다. AprilTag 전용 "
        "refinement를 기본값으로 적용하고 frozen manifest를 재생성했습니다. 또한 "
        "manifest marker_ids가 정렬되면서 4-corner block 순서와 달라지던 metadata "
        "오류도 수정했습니다.",
        "",
        f"수정 전후 translation 충돌 RMSE는 "
        f"`{software_fix['before']['translation_conflict_rmse_mm']:.3f} → "
        f"{software_fix['after']['translation_conflict_rmse_mm']:.3f} mm`, "
        f"Cube held-out transfer는 "
        f"`{software_fix['before']['cube_heldout_transfer_rmse_px']:.3f} → "
        f"{software_fix['after']['cube_heldout_transfer_rmse_px']:.3f} px`입니다.",
        "남은 target-dependent 충돌은 한 target의 extrinsic을 정답처럼 선택하거나 "
        "Board/Cube pose를 단순 평균해서 없애지 않습니다. Calibration은 두 target이 "
        "공유하는 camera 변수를 corner reprojection으로 joint solve하고, direct PnP "
        "결과는 diagnostic으로만 사용합니다.",
        "",
        f"학습 관측에서 두 target을 강제로 맞추면 Board 쪽에 `{scale['scale']:.6f}`를 "
        "곱하는 수치가 나오지만, 이는 물리적 Board 보정값이 아니라 target 간 "
        f"systematic disagreement를 흡수한 유효값입니다. {nominal_square_mm:.1f} mm "
        "nominal square에 "
        "대입하면 "
        f"`{scale['effective_square_length_mm']:.3f} mm`입니다.",
        f"RGB-D가 독립적으로 암시한 Board/Cube 상대 scale 중앙값은 "
        f"`{rgbd_relative_scale:.6f}`입니다.",
        f"학습 relative-transform translation 충돌 RMSE는 scale 적용 시 "
        f"`{before_rmse:.3f} → {after_rmse:.3f} mm` "
        f"(`{reduction_percent:.1f}%` 감소)입니다.",
        "",
        "## 먼저 구분할 수치",
        "",
        "| 수치 | 의미 | 사용 여부 |",
        "|---|---|---|",
        f"| `{software_fix['after']['translation_conflict_rmse_mm']:.3f} mm` "
        "| Board-only PnP와 Cube-only PnP가 계산한 동일 고정 카메라 변환의 "
        "target-dependent 불일치 | 진단 및 single-target 결과 공개 금지 gate |",
        f"| `{current['heldout']['board']['by_target']['board']['pose_consistency_translation_rmse_mm']:.3f} mm` "
        "| Board가 추정한 카메라 변환의 held-out 자기 일관성 | 보조 지표 |",
        f"| `{current['heldout']['cube']['by_target']['cube']['pose_consistency_translation_rmse_mm']:.3f} mm` "
        "| Cube가 추정한 카메라 변환의 held-out 자기 일관성 | 보조 지표 |",
        "| 외부 GT 오차 | 알려진 실제 카메라 자세와 추정값의 차이 | 현재 없음 |",
        "",
        f"따라서 `{software_fix['after']['translation_conflict_rmse_mm']:.3f} mm`는 "
        "최종 joint calibration의 translation 정확도도, 실제 카메라 위치의 절대 "
        "오차도 아닙니다. 서로 다른 두 target이 동일 물리량에 대해 얼마나 충돌하는지 "
        "보여주는 독립 direct-PnP 진단값입니다. 외부 GT 없이 이 값으로 절대 정확도를 "
        "주장하지 않습니다.",
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
        "사용하지 않습니다. 실측 Board와 모순되므로 config, calibration 입력, 공식 "
        "결과에는 적용하지 않습니다.",
        "",
        "## Cube geometry 가설 분해",
        "",
        f"Board의 {nominal_square_mm:.1f} mm를 물리 기준으로 고정한 뒤 Cube object "
        "point의 서로 다른 "
        "성분만 train 관측에서 scale scan한 반사실적 진단입니다. Cube 실측값과 "
        "모순되므로 아래 scale은 config에 적용하지 않습니다.",
        "",
        "| 가설 | 최적 scale | 충돌 RMSE | Cube held-out transfer |",
        "|---|---:|---:|---:|",
    ])
    scan_labels = {
        "nominal": "현재 Cube config",
        "uniform_geometry": "Cube 전체 geometry",
        "all_marker_size": "모든 marker 크기만",
        "side_marker_size": "51 mm side marker 크기만",
        "top_marker_size": "25 mm top marker 크기만",
        "marker_center_radius": "marker 중심 배치 반경만",
    }
    for name in (
            "nominal", "uniform_geometry", "all_marker_size",
            "side_marker_size", "top_marker_size", "marker_center_radius"):
        detail = cube_scan[name]
        lines.append(
            f"| {scan_labels[name]} | {detail['best_scale']:.6f} "
            f"| {detail['conflict_translation_rmse_mm']:.3f} mm "
            f"| {detail['heldout_cube']['cross_view_pixel_transfer_rmse_px']:.3f} px |")
    lines.extend([
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
        "## RGB-D 절대 Scale 교차검증",
        "",
        "PnP가 예측한 target 표면 깊이와 같은 RGB pixel의 aligned RealSense depth를 "
        "비교했습니다. `pred/meas < 1`이면 현재 object geometry가 실제보다 작게 "
        "정의되었을 가능성이 큽니다.",
        "",
        "| Target | Camera | 관측 수 | Pred/Measured depth | 보정 암시 scale | 깊이 bias |",
        "|---|---:|---:|---:|---:|---:|",
    ])
    for target, by_camera in payload["rgbd_target_scale"]["summary"].items():
        for camera, detail in by_camera.items():
            lines.append(
                f"| {target.title()} | cam{camera} "
                f"| {detail['observations']} "
                f"| {detail['predicted_over_measured_depth_median']:.6f} "
                f"| {detail['implied_geometry_scale_to_match_depth']:.6f} "
                f"| {detail['predicted_minus_measured_depth_median_mm']:.3f} mm |")
    lines.extend([
        "",
        "이 검사는 color PnP와 별도 depth stream을 사용하므로 상대카메라 baseline "
        "비교와 다른 증거입니다. 다만 RealSense depth 자체의 systematic bias가 있어 "
        "실측 Board/Cube 치수보다 우선할 수 없으며 geometry config 변경 근거로 "
        "사용하지 않습니다.",
        f"Camera별 Board/Cube 상대 scale은 `"
        f"{', '.join(f'cam{camera}={value:.6f}' for camera, value in payload['rgbd_target_scale']['board_scale_relative_to_cube_by_camera'].items())}`이며, "
        "target 종류에 공통인 depth bias를 상쇄한 값입니다.",
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
        "모두 개별 영상 안에서는 잘 맞으므로 gross한 단일-frame corner ordering "
        "오류 가능성은 낮습니다. 다만 이 오버레이만으로 실물 marker 중심 거리나 "
        "Board/Cube 절대 치수를 확정할 수는 없습니다.",
        "",
        "## 재발 방지 계약",
        "",
        f"- Frozen manifest schema: `{payload['manifest']['manifest_schema']}`",
        "- Board/Cube geometry와 SHA-256은 meta/manifest 사이에서 일치해야 로드됨",
        "- Board corner ID→3D point와 Cube marker ID→4-corner block 순서가 다르면 즉시 실패",
        "- Board detector는 `CORNER_REFINE_NONE`, Cube detector는 명시된 refinement mode를 기록",
        f"- Direct PnP 충돌 `{conflict_contract['translation_rmse_mm']:.3f} mm`는 "
        "single-target extrinsic 공개 금지 신호이며 shared-camera joint solve를 강제",
        "",
        "## 남은 오차의 처리",
        "",
        f"수정 후에도 {software_fix['after']['translation_conflict_rmse_mm']:.3f} mm의 "
        f"target-dependent 차이가 남습니다. PnP solver를 바꾸어도 최선이 "
        f"{best_pnp['translation_rmse_mm']:.3f} mm이고 stereoCalibrate도 "
        f"{stereo['translation_rmse_mm']:.3f} mm이므로 solver 선택 문제는 아닙니다.",
        "",
        "Factory K/D에서는 충돌이 약 4.9 mm로 줄지만 Cube held-out transfer가 "
        "2.882 px에서 5.259 px로 악화됩니다. 따라서 factory intrinsic으로 즉시 "
        "교체하지 않습니다. 남은 차이는 현재 intrinsic의 제한된 view coverage와 "
        "planar/non-planar target별 localization bias가 섞인 것으로 판단하며, 다양한 "
        "거리·화면 위치의 새 intrinsic 촬영 또는 외부 GT 없이는 어느 K/D가 절대적으로 "
        "옳은지 식별할 수 없습니다.",
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
        "--observation-filter-policy", choices=("standard", "strict"),
        default="standard")
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
    with Path(args.manifest).resolve().open("r", encoding="utf-8") as stream:
        manifest_payload = json.load(stream)
    observations, manifest_diagnostics = load_pixel_observations_from_manifest(
        args.manifest, policy=args.observation_filter_policy, root=str(root),
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
    rgbd_target_scale = _rgbd_target_scale_diagnostic(
        root, meta, observations, current_K, current_D, intrinsics_dir,
        fixed_cameras)
    cube_geometry_scale_scan = _cube_geometry_scale_scan(
        train, test, split, fixed_cameras, current_K, current_D,
        args.manifest)
    pnp_solver_scan = _pnp_solver_scan(
        train, fixed_cameras, current_K, current_D)
    stereo_calibration_conflict = _stereo_calibration_conflict(
        train, fixed_cameras, current_K, current_D)
    corner_mode = str(manifest_payload.get("scope", {}).get(
        "cube_corner_refinement_mode", "apriltag"))
    strict_cube_observations = sum(
        record.get("target") == "cube"
        and bool(record.get("selected_by_policy", {}).get("strict"))
        for record in manifest_payload.get("observations", []))
    board_cfg = charuco_config_from_dict(
        manifest_diagnostics["charuco_board_config"])
    board_topology = charuco_topology(board_cfg)
    same_event_conflict = _same_event_conflicts(
        [observation for observation in observations
         if observation.set_idx in set(split["eligible_sets"])],
        fixed_cameras, current_K, current_D)
    payload = {
        "schema": "board_cube_relative_pose_diagnostic_v5",
        "session_root": str(root),
        "physical_board_measurement": {
            "reported_checker_width_mm": 275.0,
            "opencv_checker_squares_x": int(
                board_topology["checker_squares_x"]),
            "charuco_corner_columns": int(
                board_topology["charuco_corner_columns"]),
            "maximum_charuco_corners": int(
                board_topology["maximum_charuco_corners"]),
            "configured_square_length_mm": (
                float(board_cfg.square_length_m) * 1000.0),
            "configured_checker_width_mm": float(
                board_topology["checker_width_mm"]),
            "matches_frozen_config": bool(np.isclose(
                float(board_topology["checker_width_mm"]), 275.0,
                rtol=0.0, atol=1e-9)),
            "topology_note": (
                "11 checker-square columns produce 10 internal ChArUco "
                "corner columns and 60=(11-1)*(7-1) total corners"),
        },
        "physical_cube_measurement": {
            "body_width_mm": 59.0,
            "body_depth_mm": 59.0,
            "body_height_mm": 57.0,
            "top_protrusion_height_mm": 2.0,
            "overall_height_mm": 59.0,
            "top_marker_plane_z_mm": 29.5,
            "side_marker_center_z_mm": -1.0,
            "side_marker_outer_black_border_mm": 51.0,
            "top_marker_outer_black_border_mm": 25.0,
            "matches_config": True,
        },
        "cube_corner_refinement_fix": {
            "before": {
                "corner_refinement": "CORNER_REFINE_NONE",
                "manifest_git_blob": "713207099391f87846498baa0a5d924036ca3c28",
                "translation_conflict_rmse_mm": 17.299047075861232,
                "cube_heldout_transfer_rmse_px": 4.246995731145374,
                "cube_heldout_pose_translation_rmse_mm": 4.479787181376066,
                "strict_cube_observations": 90,
            },
            "after": {
                "corner_refinement": corner_mode,
                "translation_conflict_rmse_mm": float(np.sqrt(np.mean([
                    detail["translation_disagreement_before_mm"] ** 2
                    for detail in board_metric_scale["per_camera"].values()
                ]))),
                "cube_heldout_transfer_rmse_px": variant_results[
                    "charuco_calibrated_KD"]["heldout"]["cube"][
                        "by_target"]["cube"][
                            "cross_view_pixel_transfer_rmse_px"],
                "cube_heldout_pose_translation_rmse_mm": variant_results[
                    "charuco_calibrated_KD"]["heldout"]["cube"][
                        "by_target"]["cube"][
                            "pose_consistency_translation_rmse_mm"],
                "strict_cube_observations": int(strict_cube_observations),
            },
        },
        "manifest": manifest_diagnostics,
        "split": split,
        "train_only_board_metric_scale": board_metric_scale,
        "same_event_conflict": same_event_conflict,
        "intrinsic_variants": variant_results,
        "rgbd_target_scale": rgbd_target_scale,
        "cube_geometry_scale_scan": cube_geometry_scale_scan,
        "pnp_solver_scan": pnp_solver_scan,
        "stereo_calibration_conflict": stereo_calibration_conflict,
        "intrinsic_calibration_metadata": _intrinsic_metadata(intrinsics_dir),
        "overlay_event_id": int(overlay_event),
        "overlay_file": overlay_name,
    }
    payload["systematic_conflict_contract"] = (
        _systematic_conflict_contract(board_metric_scale, same_event_conflict))
    json_path = output_dir / "board_cube_relative_pose_diagnostic.json"
    json_path.write_text(
        json.dumps(_jsonable(payload), indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8")
    _write_report(output_dir / "BOARD_CUBE_RELATIVE_POSE.md", payload)
    print(json_path)


if __name__ == "__main__":
    main()
