"""Model-independent path-consistency evaluation for calibration ablations.

The evaluation population is frozen before a calibration row is fitted.  A
cube observation may be excluded only because it is absent from the detector
quality mask or because its image/object points do not admit a finite PnP
pose.  Calibrated camera/target predictions never decide which pairs remain.

Two contracts intentionally coexist:

* the legacy cube path also contains the FK-dependent eye-in-hand closure used
  by Table 1;
* the fixed-camera cross-target path evaluates board and cube without a shared
  target pose, robot FK, or gripper camera, but remains method-specific and is
  reported only as supplementary held-out consistency;
* the gripper-to-fixed path uses the same visual targets but evaluates through
  robot FK and the calibrated hand-eye transform.

They are internal diagnostics before independent external GT; neither is
absolute physical accuracy or an independent relative-pose reference.
"""

from __future__ import annotations

import hashlib
import json
from collections import defaultdict
from itertools import combinations
from typing import Dict, Iterable, Mapping, Optional, Sequence

import cv2
import numpy as np
from scipy.spatial.transform import Rotation

from calibration_pipeline import se3 as cp
from calibration_pipeline.apriltag_cube import inv_T
from calibration_pipeline.reprojection import PixelObs, pose_delta, project_points


MASK_SCHEMA = "model_independent_path_evaluation_mask_v2"
CROSS_TARGET_MASK_SCHEMA = "fixed_to_fixed_cross_target_mask_v1"
GRIPPER_TO_FIXED_MASK_SCHEMA = "fk_dependent_gripper_to_fixed_cross_target_mask_v2"
CROSS_TARGETS = ("board", "cube")

FIXED_TO_FIXED_CROSS_TARGET_CONTRACT = {
    "name": "fixed_to_fixed_board_cube_cross_target",
    "targets": list(CROSS_TARGETS),
    "per_camera_pose": (
        "T_base_target_from_camera = T_base_camera @ "
        "T_camera_target_measurement_only_PnP"),
    "pose_metrics": (
        "per_target_pairwise_translation_RMSE_mm_and_SO3_geodesic_RMSE_deg"),
    "pixel_metric": (
        "per_target_bidirectional_component_wise_pixel_transfer_RMSE_px"),
    "uses_shared_base_target_pose": False,
    "uses_robot_fk": False,
    "uses_gripper_camera": False,
    "uses_external_ground_truth": False,
    "absolute_accuracy_metric": False,
    "reporting_tier": "supplementary",
    "may_rank_methods_before_external_gt": False,
    "ranking_role_before_external_gt": (
        "supplementary_method_specific_heldout_consistency"),
    "interpretation": (
        "computed with each method's fitted fixed-camera poses; useful as a "
        "held-out self-consistency diagnostic, not an independent baseline"),
}

GRIPPER_TO_FIXED_CROSS_TARGET_CONTRACT = {
    "name": "fk_dependent_gripper_to_fixed_board_cube_cross_target",
    "targets": list(CROSS_TARGETS),
    "fixed_camera_pose": "T_base_camera = calibrated_T_base_fixed_camera",
    "gripper_camera_pose": (
        "T_base_gripper_camera_event = robot_FK_T_base_gripper_event @ "
        "calibrated_T_gripper_camera"),
    "per_camera_target_pose": (
        "T_base_target_from_camera = T_base_camera_event @ "
        "T_camera_target_measurement_only_PnP"),
    "pose_metrics": (
        "per_target_fixed_vs_gripper_translation_RMSE_mm_and_"
        "SO3_geodesic_RMSE_deg"),
    "pixel_metric": (
        "per_target_bidirectional_fixed_gripper_pixel_transfer_RMSE_px"),
    "pair_population": (
        "each_heldout_gripper_event_paired_with_the_first_saved_fixed_camera_"
        "anchor_from_the_same_set"),
    "fixed_anchor_usage": (
        "one_fixed_residual_in_optimization; measurement_reference_reused_"
        "across_same_set_gripper_events_only_for_evaluation"),
    "aggregation": (
        "pair_components_to_event_RMSE_then_event_to_set_RMSE_then_equal_"
        "weight_RMSE_across_sets"),
    "uses_shared_base_target_pose": False,
    "uses_robot_fk": True,
    "uses_gripper_camera": True,
    "uses_external_ground_truth": False,
    "absolute_accuracy_metric": False,
    "ranking_role_before_external_gt": "full_system_internal_chain_metric",
    "interpretation": (
        "visual target observations evaluate the combined fixed-camera, "
        "hand-eye, and robot-FK chain; FK error cannot be separated here"),
}

# Canonical definition requested for the paper/report.  e_cross is the usual
# inter-camera target-pose consistency: each fixed camera independently turns
# its measured cube pose into the robot-base frame, then every
# predeclared camera pair is compared.  It never uses robot FK, the gripper
# camera, a nominal cube position, or an external/implicit ground truth.
E_CROSS_CONTRACT = {
    "name": "fixed_camera_cube_pose_consistency",
    "translation_metric": "pairwise_cube_center_distance_RMSE_mm",
    "rotation_metric": "pairwise_SO3_geodesic_RMSE_deg",
    "per_camera_pose": "T_base_cube_from_camera = T_base_camera @ T_camera_cube_PnP",
    "pair_population": "predeclared_heldout_fixed_camera_pairs",
    "uses_robot_fk": False,
    "uses_gripper_camera": False,
    "uses_nominal_or_ground_truth_cube_pose": False,
    "absolute_accuracy_metric": False,
    "reporting_tier": "supplementary",
    "may_rank_methods_before_external_gt": False,
    "role": "method_specific_heldout_consistency",
}

E_CROSS_PIXEL_TRANSFER_CONTRACT = {
    "name": "bidirectional_fixed_camera_cube_pixel_transfer",
    "source_pose": "cube_PnP_from_source_camera_measurement_only",
    "transfer": (
        "T_destination_cube = inverse(T_base_destination) @ "
        "T_base_source @ T_source_cube_PnP"),
    "destination_points": "measured_destination_cube_object_points",
    "metric": "component_wise_pixel_RMSE_over_both_pair_directions",
    "uses_robot_fk": False,
    "uses_gripper_camera": False,
    "uses_shared_target_pose": False,
    "uses_external_ground_truth": False,
    "absolute_accuracy_metric": False,
    "reporting_tier": "supplementary",
    "may_rank_methods_before_external_gt": False,
    "role": "method_specific_heldout_consistency",
}


def _is_planar_points(object_points: np.ndarray) -> bool:
    """Classify an arbitrarily oriented 3-D point set by numerical rank."""
    points = np.asarray(object_points, dtype=np.float64).reshape(-1, 3)
    if len(points) < 4:
        return True
    singular_values = np.linalg.svd(
        points - np.mean(points, axis=0), compute_uv=False)
    scale = max(float(singular_values[0]), 1e-12)
    return bool(float(singular_values[-1]) <= 1e-7 * scale)


def solve_observed_pose(obs: PixelObs, K_map, D_map) -> Optional[np.ndarray]:
    """Return ``T_cam_target`` from measured corners only, or ``None``."""
    obj = np.asarray(obs.object_points, dtype=np.float64).reshape(-1, 3)
    img = np.asarray(obs.image_points, dtype=np.float64).reshape(-1, 2)
    if len(obj) < 4:
        return None
    # A side-face marker is planar even though its object-frame z coordinates
    # vary.  Rank/SVD is invariant to face orientation; a z-range test is not.
    planar = _is_planar_points(obj)
    flag = cv2.SOLVEPNP_IPPE if planar else cv2.SOLVEPNP_ITERATIVE
    try:
        ok, rv, tv = cv2.solvePnP(
            obj, img, np.asarray(K_map[int(obs.cam)]),
            np.asarray(D_map[int(obs.cam)]), flags=flag)
    except Exception:
        return None
    if not ok:
        return None
    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = cv2.Rodrigues(rv)[0]
    transform[:3, 3] = np.asarray(tv, dtype=np.float64).reshape(3)
    if not np.all(np.isfinite(transform)) or float(transform[2, 3]) <= 0.0:
        return None
    return transform


def fixed_camera_cube_pose_in_base(
        T_base_camera: np.ndarray, T_camera_cube_pnp: np.ndarray) -> np.ndarray:
    """Compose one fixed camera's measured cube pose into the base frame."""
    return (np.asarray(T_base_camera, dtype=np.float64)
            @ np.asarray(T_camera_cube_pnp, dtype=np.float64))


def fixed_camera_target_pair_disagreement(
        T_base_target_left: np.ndarray,
        T_base_target_right: np.ndarray) -> tuple[float, float]:
    """Return pairwise target-position and orientation disagreement."""
    left = np.asarray(T_base_target_left, dtype=np.float64)
    right = np.asarray(T_base_target_right, dtype=np.float64)
    translation_mm = float(
        np.linalg.norm(left[:3, 3] - right[:3, 3]) * 1000.0)
    relative_rotation = left[:3, :3].T @ right[:3, :3]
    rotation_deg = float(np.degrees(np.linalg.norm(
        Rotation.from_matrix(relative_rotation).as_rotvec())))
    return translation_mm, rotation_deg


def fixed_camera_cube_pair_disagreement(
        T_base_cube_left: np.ndarray,
        T_base_cube_right: np.ndarray) -> tuple[float, float]:
    """Backward-compatible cube-specific wrapper."""
    return fixed_camera_target_pair_disagreement(
        T_base_cube_left, T_base_cube_right)


def pairwise_rmse(values: Sequence[tuple[float, float]]) -> tuple[Optional[float], Optional[float]]:
    """Aggregate predeclared pair disagreements without output-dependent gates."""
    if not values:
        return None, None
    numeric = np.asarray(values, dtype=np.float64)
    return (
        float(np.sqrt(np.mean(np.square(numeric[:, 0])))),
        float(np.sqrt(np.mean(np.square(numeric[:, 1])))),
    )


def _scalar_rmse(values: Sequence[float]) -> Optional[float]:
    if not values:
        return None
    return float(np.sqrt(np.mean(np.square(np.asarray(values, dtype=np.float64)))))


def _event_then_set_aggregate(
        event_rows: Sequence[Mapping], metric_fields: Sequence[str]) -> tuple[dict, list[dict]]:
    """Give each set equal final weight after event-level aggregation.

    Reusing one fixed-camera anchor for several wrist-camera events creates
    correlated pairs.  Aggregating pair components within an event, then
    events within a set, prevents the repeated anchor from masquerading as an
    independent fixed-camera capture.
    """
    by_set: Dict[int, list[Mapping]] = defaultdict(list)
    for row in event_rows:
        by_set[int(row["set"])].append(row)
    set_rows = []
    for set_index, rows in sorted(by_set.items()):
        summary = {
            "set": int(set_index),
            "n_events": len(rows),
            "events": sorted(int(row["event"]) for row in rows),
        }
        for field in metric_fields:
            summary[field] = _scalar_rmse([
                float(row[field]) for row in rows if row.get(field) is not None])
        set_rows.append(summary)
    overall = {
        field: _scalar_rmse([
            float(row[field]) for row in set_rows if row.get(field) is not None])
        for field in metric_fields
    }
    return overall, set_rows


def observation_id(obs: PixelObs) -> str:
    if obs.marker != "cube" or obs.set_idx is None:
        raise ValueError("path evaluation IDs are defined only for set-labelled cube observations")
    return f"cube:event={int(obs.event)}:set={int(obs.set_idx)}:cam={int(obs.cam)}"


def cross_target_observation_id(obs: PixelObs) -> str:
    """Stable ID for a held-out board/cube observation in one fixed camera."""
    marker = str(obs.marker)
    if marker not in CROSS_TARGETS:
        raise ValueError(
            "cross-target IDs are defined only for board/cube observations")
    set_text = "none" if obs.set_idx is None else str(int(obs.set_idx))
    return (
        f"{marker}:event={int(obs.event)}:set={set_text}:cam={int(obs.cam)}")


def _sha256_json(value: Mapping) -> str:
    encoded = json.dumps(
        value, sort_keys=True, separators=(",", ":"),
        allow_nan=False).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def _source_observation_sha256(observations: Sequence[PixelObs]) -> str:
    digest = hashlib.sha256()
    for obs in sorted(observations, key=observation_id):
        digest.update(observation_id(obs).encode("utf-8"))
        for values in (obs.object_points, obs.image_points):
            array = np.ascontiguousarray(values, dtype="<f8")
            digest.update(str(array.shape).encode("ascii"))
            digest.update(array.tobytes())
    return digest.hexdigest()


def _cross_target_source_sha256(observations: Sequence[PixelObs]) -> str:
    digest = hashlib.sha256()
    for obs in sorted(observations, key=cross_target_observation_id):
        digest.update(cross_target_observation_id(obs).encode("utf-8"))
        for values in (obs.object_points, obs.image_points):
            array = np.ascontiguousarray(values, dtype="<f8")
            digest.update(str(array.shape).encode("ascii"))
            digest.update(array.tobytes())
    return digest.hexdigest()


def build_fixed_to_fixed_cross_target_mask(
        observations: Sequence[PixelObs], fixed_camera_ids: Iterable[int],
        K_map, D_map, set_filter: Optional[Sequence[int]] = None,
        required_targets: Sequence[str] = CROSS_TARGETS) -> dict:
    """Freeze board/cube fixed-camera pairs without a shared target pose.

    Pair selection uses only the pre-fit detector population and
    measurement-only PnP validity.  It never consults a calibrated transform,
    robot FK, gripper camera, shared target pose, or external GT.
    """
    fixed_cameras = tuple(sorted({int(ci) for ci in fixed_camera_ids}))
    if len(fixed_cameras) < 2:
        raise RuntimeError("cross-target evaluation needs at least two fixed cameras")
    allowed_sets = (
        None if set_filter is None else {int(set_index) for set_index in set_filter})
    required = tuple(str(target) for target in required_targets)
    unknown = sorted(set(required) - set(CROSS_TARGETS))
    if unknown:
        raise ValueError(f"unknown required cross targets: {unknown}")

    candidates = []
    for obs in observations:
        if str(obs.marker) not in CROSS_TARGETS:
            continue
        if int(obs.cam) not in fixed_cameras:
            continue
        if allowed_sets is not None:
            if obs.set_idx is None or int(obs.set_idx) not in allowed_sets:
                continue
        candidates.append(obs)

    by_id: Dict[str, PixelObs] = {}
    for obs in candidates:
        key = cross_target_observation_id(obs)
        if key in by_id:
            raise ValueError(f"duplicate cross-target observation ID: {key}")
        by_id[key] = obs

    valid_ids, invalid_ids = [], []
    for key in sorted(by_id):
        if solve_observed_pose(by_id[key], K_map, D_map) is None:
            invalid_ids.append(key)
        else:
            valid_ids.append(key)

    grouped: Dict[tuple[str, int, Optional[int]], list[str]] = defaultdict(list)
    for key in valid_ids:
        obs = by_id[key]
        grouped[(str(obs.marker), int(obs.event),
                 None if obs.set_idx is None else int(obs.set_idx))].append(key)

    pairs = []
    for (target, event, set_index), ids in sorted(
            grouped.items(), key=lambda item: (
                item[0][0], item[0][1],
                -1 if item[0][2] is None else item[0][2])):
        ordered = sorted(ids, key=lambda key: int(by_id[key].cam))
        for left, right in combinations(ordered, 2):
            pairs.append({
                "target": target,
                "event": event,
                "set": set_index,
                "left_observation_id": left,
                "right_observation_id": right,
            })

    support = {}
    for target in CROSS_TARGETS:
        target_candidates = [
            key for key in sorted(by_id) if str(by_id[key].marker) == target]
        target_valid = [
            key for key in valid_ids if str(by_id[key].marker) == target]
        target_pairs = [pair for pair in pairs if pair["target"] == target]
        support[target] = {
            "candidate_observations": len(target_candidates),
            "valid_observations": len(target_valid),
            "pnp_invalid_observations": len(target_candidates) - len(target_valid),
            "pairs": len(target_pairs),
            "directions": 2 * len(target_pairs),
        }

    missing = [target for target in required if not support[target]["pairs"]]
    if missing:
        raise RuntimeError(
            "held-out fixed-camera observations do not support cross-target "
            f"pairs for {missing}")

    body = {
        "artifact_schema": CROSS_TARGET_MASK_SCHEMA,
        "split_role": "heldout_event_grouped_set_stratified",
        "selection_basis": (
            "frozen_detector_population_and_measurement_only_PnP_validity"),
        "selection_timing_contract": (
            "build_once_without_any_method_output_then_reuse_for_every_method"),
        "model_output_used_for_selection": False,
        "output_dependent_pose_gate": None,
        "fixed_camera_ids": list(fixed_cameras),
        "set_ids": None if allowed_sets is None else sorted(allowed_sets),
        "candidate_observation_ids": sorted(by_id),
        "valid_observation_ids": valid_ids,
        "pnp_invalid_observation_ids": invalid_ids,
        "source_observation_sha256": _cross_target_source_sha256(candidates),
        "pairs": pairs,
        "support_by_target": support,
        "required_targets": list(required),
        "metric_contract": FIXED_TO_FIXED_CROSS_TARGET_CONTRACT,
    }
    body["evaluation_mask_sha256"] = _sha256_json(body)
    validate_fixed_to_fixed_cross_target_mask(body)
    return body


def validate_fixed_to_fixed_cross_target_mask(mask: Mapping) -> None:
    if mask.get("artifact_schema") != CROSS_TARGET_MASK_SCHEMA:
        raise ValueError("unknown fixed-to-fixed mask schema")
    if mask.get("model_output_used_for_selection") is not False:
        raise ValueError("cross-target selection must be model independent")
    if mask.get("output_dependent_pose_gate") is not None:
        raise ValueError("output-dependent cross-target gates are forbidden")
    if mask.get("metric_contract") != FIXED_TO_FIXED_CROSS_TARGET_CONTRACT:
        raise ValueError("fixed-to-fixed metric contract mismatch")
    pairs = list(mask.get("pairs", []))
    targets = {str(pair.get("target")) for pair in pairs}
    if not targets.issubset(set(CROSS_TARGETS)):
        raise ValueError("cross-target mask contains an unknown target")
    required = {str(target) for target in mask.get("required_targets", [])}
    if not required.issubset(set(CROSS_TARGETS)) or not required.issubset(targets):
        raise ValueError("cross-target mask lacks required board/cube pair support")
    valid_ids = set(map(str, mask.get("valid_observation_ids", [])))
    for pair in pairs:
        left = str(pair.get("left_observation_id", ""))
        right = str(pair.get("right_observation_id", ""))
        if not left or not right or left == right:
            raise ValueError("invalid cross-target pair identity")
        if left not in valid_ids or right not in valid_ids:
            raise ValueError("cross-target pair references an undeclared observation")
    support = mask.get("support_by_target", {})
    for target in CROSS_TARGETS:
        expected_pairs = sum(str(pair.get("target")) == target for pair in pairs)
        if int(support.get(target, {}).get("pairs", -1)) != expected_pairs:
            raise ValueError(f"{target}: cross-target pair support mismatch")
    unhashed = dict(mask)
    expected = str(unhashed.pop("evaluation_mask_sha256", ""))
    if not expected or _sha256_json(unhashed) != expected:
        raise ValueError("fixed-to-fixed mask SHA-256 mismatch")


def evaluate_fixed_to_fixed_cross_target(
        observations: Sequence[PixelObs], cams, K_map, D_map,
        mask: Mapping) -> dict:
    """Evaluate fixed-camera board/cube transfer without a target reference."""
    validate_fixed_to_fixed_cross_target_mask(mask)
    camera_ids = set(map(int, mask["fixed_camera_ids"]))
    missing_cameras = sorted(camera_ids - set(map(int, cams)))
    if missing_cameras:
        raise ValueError(
            f"calibration lacks cross-target fixed cameras {missing_cameras}")

    by_id = {}
    for obs in observations:
        if str(obs.marker) not in CROSS_TARGETS or int(obs.cam) not in camera_ids:
            continue
        key = cross_target_observation_id(obs)
        if key in by_id:
            raise ValueError(f"duplicate cross-target observation ID: {key}")
        by_id[key] = obs
    declared = list(mask["valid_observation_ids"])
    missing_observations = sorted(set(declared) - set(by_id))
    if missing_observations:
        raise ValueError(
            "evaluation input is missing frozen cross-target observations "
            f"{missing_observations[:5]}")

    target_in_base = {}
    for key in declared:
        obs = by_id[key]
        T_camera_target = solve_observed_pose(obs, K_map, D_map)
        if T_camera_target is None:
            raise RuntimeError(f"prevalidated PnP became invalid: {key}")
        target_in_base[key] = (
            np.asarray(cams[int(obs.cam)], dtype=np.float64) @ T_camera_target)

    accumulators = {
        target: {"pose": [], "pixel_squared": [], "rows": []}
        for target in CROSS_TARGETS
    }
    overall_pixel_squared = []
    for pair in mask["pairs"]:
        target = str(pair["target"])
        left_id, right_id = (
            pair["left_observation_id"], pair["right_observation_id"])
        dt, dr = fixed_camera_target_pair_disagreement(
            target_in_base[left_id], target_in_base[right_id])
        accumulators[target]["pose"].append((dt, dr))
        direction_rows = []
        for source_id, destination_id in (
                (left_id, right_id), (right_id, left_id)):
            destination = by_id[destination_id]
            destination_camera = int(destination.cam)
            T_destination_target = (
                inv_T(np.asarray(cams[destination_camera], dtype=np.float64))
                @ target_in_base[source_id])
            prediction = project_points(
                T_destination_target, destination.object_points,
                K_map[destination_camera], D_map[destination_camera])
            measured = np.asarray(
                destination.image_points, dtype=np.float64).reshape(-1, 2)
            if prediction.shape != measured.shape or not np.all(np.isfinite(prediction)):
                raise RuntimeError(
                    "invalid fixed-to-fixed cross-view transfer: "
                    f"{source_id} -> {destination_id}")
            squared = np.square(prediction - measured).reshape(-1)
            accumulators[target]["pixel_squared"].extend(squared.tolist())
            overall_pixel_squared.extend(squared.tolist())
            direction_rows.append({
                "source_observation_id": source_id,
                "destination_observation_id": destination_id,
                "rmse_px": float(np.sqrt(np.mean(squared))),
                "n_corners": int(len(prediction)),
            })
        accumulators[target]["rows"].append({
            **dict(pair),
            "translation_mm": dt,
            "rotation_deg": dr,
            "pixel_transfer_directions": direction_rows,
        })

    by_target = {}
    for target in CROSS_TARGETS:
        values = accumulators[target]["pose"]
        translation_rmse, rotation_rmse = pairwise_rmse(values)
        pixel_squared = accumulators[target]["pixel_squared"]
        by_target[target] = {
            "pose_consistency_translation_rmse_mm": translation_rmse,
            "pose_consistency_rotation_rmse_deg": rotation_rmse,
            "cross_view_pixel_transfer_rmse_px": (
                None if not pixel_squared else
                float(np.sqrt(np.mean(pixel_squared)))),
            "n_pairs": len(values),
            "n_directions": 2 * len(values),
            "n_output_rejected": 0,
            "per_pair": accumulators[target]["rows"],
        }

    return {
        "applicable": True,
        "evaluation_mask_sha256": mask["evaluation_mask_sha256"],
        "metric_contract": FIXED_TO_FIXED_CROSS_TARGET_CONTRACT,
        "model_dependent_gating": False,
        "output_dependent_pose_gate": None,
        "by_target": by_target,
        "overall_pixel_transfer_rmse_px": (
            None if not overall_pixel_squared else
            float(np.sqrt(np.mean(overall_pixel_squared)))),
        "overall_note": (
            "pooled board+cube pixel transfer is descriptive only; use each "
            "target component for comparison"),
        "n_output_rejected": 0,
    }


def build_gripper_to_fixed_cross_target_mask(
        observations: Sequence[PixelObs], fixed_camera_ids: Iterable[int],
        gripper_cam_idx: int, K_map, D_map,
        set_filter: Optional[Sequence[int]] = None,
        required_targets: Sequence[str] = CROSS_TARGETS,
        fixed_anchor_observations: Optional[Sequence[PixelObs]] = None,
        event_roles: Optional[Mapping[int, str]] = None) -> dict:
    """Pair each held-out wrist view with its set's first fixed-camera view.

    ``observations`` supplies the evaluation gripper views.  Fixed anchors are
    selected from ``fixed_anchor_observations`` before PnP validation by the
    earliest event for each (target, set, fixed camera), so a later/better
    frame can never be selected using output quality.  Robot FK and calibrated
    transforms remain absent from mask construction.
    """
    fixed_cameras = tuple(sorted({int(ci) for ci in fixed_camera_ids}))
    gripper = int(gripper_cam_idx)
    if not fixed_cameras:
        raise RuntimeError("gripper-to-fixed evaluation needs a fixed camera")
    if gripper in fixed_cameras:
        raise ValueError("gripper camera cannot also be a fixed camera")
    allowed_sets = (
        None if set_filter is None else {int(index) for index in set_filter})
    required = tuple(str(target) for target in required_targets)
    unknown = sorted(set(required) - set(CROSS_TARGETS))
    if unknown:
        raise ValueError(f"unknown required gripper-to-fixed targets: {unknown}")

    roles = {int(event): str(role) for event, role in (event_roles or {}).items()}
    anchor_source = (
        observations if fixed_anchor_observations is None
        else fixed_anchor_observations)
    gripper_candidates = []
    for obs in observations:
        if str(obs.marker) not in CROSS_TARGETS:
            continue
        if int(obs.cam) != gripper:
            continue
        if allowed_sets is not None:
            if obs.set_idx is None or int(obs.set_idx) not in allowed_sets:
                continue
        gripper_candidates.append(obs)

    anchor_groups: Dict[tuple[str, int, int], list[PixelObs]] = defaultdict(list)
    for obs in anchor_source:
        if str(obs.marker) not in CROSS_TARGETS or int(obs.cam) not in fixed_cameras:
            continue
        if obs.set_idx is None:
            continue
        if allowed_sets is not None and int(obs.set_idx) not in allowed_sets:
            continue
        anchor_groups[(str(obs.marker), int(obs.set_idx), int(obs.cam))].append(obs)
    selected_anchors = [
        min(group, key=lambda obs: (int(obs.event), cross_target_observation_id(obs)))
        for _, group in sorted(anchor_groups.items())
    ]

    by_id: Dict[str, PixelObs] = {}
    for obs in list(selected_anchors) + list(gripper_candidates):
        key = cross_target_observation_id(obs)
        if key in by_id:
            if by_id[key] is obs:
                continue
            raise ValueError(f"duplicate gripper-to-fixed observation ID: {key}")
        by_id[key] = obs

    valid_ids, invalid_ids = [], []
    for key in sorted(by_id):
        if solve_observed_pose(by_id[key], K_map, D_map) is None:
            invalid_ids.append(key)
        else:
            valid_ids.append(key)

    valid = set(valid_ids)
    fixed_anchor_ids = sorted(
        cross_target_observation_id(obs) for obs in selected_anchors)
    gripper_ids = sorted(
        cross_target_observation_id(obs) for obs in gripper_candidates)
    anchors_by_target_set: Dict[tuple[str, int], list[str]] = defaultdict(list)
    for key in fixed_anchor_ids:
        if key in valid:
            obs = by_id[key]
            anchors_by_target_set[(str(obs.marker), int(obs.set_idx))].append(key)

    pairs = []
    for gripper_id in gripper_ids:
        if gripper_id not in valid:
            continue
        wrist = by_id[gripper_id]
        target = str(wrist.marker)
        set_index = int(wrist.set_idx)
        for fixed_id in sorted(
                anchors_by_target_set.get((target, set_index), []),
                key=lambda key: int(by_id[key].cam)):
            anchor = by_id[fixed_id]
            pairs.append({
                "target": target,
                "event": int(wrist.event),
                "gripper_event": int(wrist.event),
                "fixed_anchor_event": int(anchor.event),
                "set": set_index,
                "fixed_camera_id": int(anchor.cam),
                "fixed_anchor_split_role": roles.get(
                    int(anchor.event), "unspecified"),
                "gripper_split_role": roles.get(
                    int(wrist.event), "evaluation"),
                "fixed_observation_id": fixed_id,
                "gripper_observation_id": gripper_id,
            })

    support = {}
    for target in CROSS_TARGETS:
        target_candidates = [
            key for key in sorted(by_id) if str(by_id[key].marker) == target]
        target_valid = [
            key for key in valid_ids if str(by_id[key].marker) == target]
        target_pairs = [pair for pair in pairs if pair["target"] == target]
        support[target] = {
            "candidate_observations": len(target_candidates),
            "valid_observations": len(target_valid),
            "pnp_invalid_observations": len(target_candidates) - len(target_valid),
            "pairs": len(target_pairs),
            "directions": 2 * len(target_pairs),
            "events": sorted({int(pair["event"]) for pair in target_pairs}),
            "sets": sorted({int(pair["set"]) for pair in target_pairs}),
            "fixed_anchor_events": sorted({
                int(pair["fixed_anchor_event"]) for pair in target_pairs}),
            "fixed_anchor_observations": len({
                str(pair["fixed_observation_id"]) for pair in target_pairs}),
            "gripper_observations": len({
                str(pair["gripper_observation_id"]) for pair in target_pairs}),
        }
    missing = [target for target in required if not support[target]["pairs"]]
    if missing:
        raise RuntimeError(
            "held-out observations do not support gripper-to-fixed pairs for "
            f"{missing}")

    body = {
        "artifact_schema": GRIPPER_TO_FIXED_MASK_SCHEMA,
        "split_role": "heldout_gripper_event_with_same_set_first_fixed_anchor",
        "fixed_anchor_selection": (
            "earliest_event_per_target_set_fixed_camera_before_PnP_validation"),
        "pairing": "same_set_fixed_anchor_times_each_evaluation_gripper_event",
        "aggregation": (
            "pair_components_to_event_RMSE_then_event_to_set_RMSE_then_"
            "equal_weight_RMSE_across_sets"),
        "selection_basis": (
            "frozen_detector_population_and_measurement_only_PnP_validity"),
        "selection_timing_contract": (
            "build_once_without_any_method_output_then_reuse_for_every_method"),
        "model_output_used_for_selection": False,
        "robot_fk_used_for_selection": False,
        "output_dependent_pose_gate": None,
        "fixed_camera_ids": list(fixed_cameras),
        "gripper_camera_id": gripper,
        "set_ids": None if allowed_sets is None else sorted(allowed_sets),
        "candidate_observation_ids": sorted(by_id),
        "fixed_anchor_observation_ids": fixed_anchor_ids,
        "gripper_observation_ids": gripper_ids,
        "valid_observation_ids": valid_ids,
        "pnp_invalid_observation_ids": invalid_ids,
        "source_observation_sha256": _cross_target_source_sha256(
            list(by_id.values())),
        "pairs": pairs,
        "support_by_target": support,
        "required_targets": list(required),
        "metric_contract": GRIPPER_TO_FIXED_CROSS_TARGET_CONTRACT,
    }
    body["evaluation_mask_sha256"] = _sha256_json(body)
    validate_gripper_to_fixed_cross_target_mask(body)
    return body


def validate_gripper_to_fixed_cross_target_mask(mask: Mapping) -> None:
    if mask.get("artifact_schema") != GRIPPER_TO_FIXED_MASK_SCHEMA:
        raise ValueError("unknown gripper-to-fixed cross-target mask schema")
    if (mask.get("model_output_used_for_selection") is not False
            or mask.get("robot_fk_used_for_selection") is not False):
        raise ValueError(
            "gripper-to-fixed pair selection must be model independent")
    if mask.get("output_dependent_pose_gate") is not None:
        raise ValueError("output-dependent gripper-to-fixed gates are forbidden")
    if mask.get("metric_contract") != GRIPPER_TO_FIXED_CROSS_TARGET_CONTRACT:
        raise ValueError("gripper-to-fixed metric contract mismatch")
    fixed_cameras = {int(ci) for ci in mask.get("fixed_camera_ids", [])}
    gripper = int(mask.get("gripper_camera_id", -1))
    if not fixed_cameras or gripper in fixed_cameras:
        raise ValueError("invalid gripper/fixed camera partition")
    pairs = list(mask.get("pairs", []))
    targets = {str(pair.get("target")) for pair in pairs}
    required = {str(target) for target in mask.get("required_targets", [])}
    if (not targets.issubset(set(CROSS_TARGETS))
            or not required.issubset(targets)):
        raise ValueError("gripper-to-fixed mask lacks required target support")
    valid_ids = set(map(str, mask.get("valid_observation_ids", [])))
    fixed_anchor_ids = set(map(
        str, mask.get("fixed_anchor_observation_ids", [])))
    gripper_ids = set(map(str, mask.get("gripper_observation_ids", [])))
    for pair in pairs:
        fixed_id = str(pair.get("fixed_observation_id", ""))
        gripper_id = str(pair.get("gripper_observation_id", ""))
        if (not fixed_id or not gripper_id or fixed_id == gripper_id
                or fixed_id not in valid_ids or gripper_id not in valid_ids):
            raise ValueError("invalid gripper-to-fixed pair identity")
        if fixed_id not in fixed_anchor_ids or gripper_id not in gripper_ids:
            raise ValueError("gripper-to-fixed pair violates anchor/view roles")
        if int(pair.get("event", -1)) != int(pair.get("gripper_event", -2)):
            raise ValueError("gripper-to-fixed event is not the wrist event")
        if int(pair.get("fixed_camera_id", -1)) not in fixed_cameras:
            raise ValueError("gripper-to-fixed pair has a non-fixed anchor")
    support = mask.get("support_by_target", {})
    for target in CROSS_TARGETS:
        expected_pairs = sum(str(pair.get("target")) == target for pair in pairs)
        if int(support.get(target, {}).get("pairs", -1)) != expected_pairs:
            raise ValueError(
                f"{target}: gripper-to-fixed pair support mismatch")
    unhashed = dict(mask)
    expected = str(unhashed.pop("evaluation_mask_sha256", ""))
    if not expected or _sha256_json(unhashed) != expected:
        raise ValueError("gripper-to-fixed evaluation mask SHA-256 mismatch")


def evaluate_gripper_to_fixed_cross_target(
        observations: Sequence[PixelObs], cams, gtc, robot_T,
        K_map, D_map, mask: Mapping) -> dict:
    """Evaluate board/cube gripper-to-fixed transfer through FK+hand-eye."""
    validate_gripper_to_fixed_cross_target_mask(mask)
    fixed_cameras = set(map(int, mask["fixed_camera_ids"]))
    gripper = int(mask["gripper_camera_id"])
    missing_cameras = sorted(fixed_cameras - set(map(int, cams)))
    if missing_cameras:
        raise ValueError(
            f"calibration lacks gripper-to-fixed cameras {missing_cameras}")

    allowed_cameras = fixed_cameras | {gripper}
    by_id = {}
    for obs in observations:
        if str(obs.marker) not in CROSS_TARGETS:
            continue
        if int(obs.cam) not in allowed_cameras:
            continue
        key = cross_target_observation_id(obs)
        if key in by_id:
            raise ValueError(f"duplicate gripper-to-fixed observation ID: {key}")
        by_id[key] = obs
    declared = list(mask["valid_observation_ids"])
    missing_observations = sorted(set(declared) - set(by_id))
    if missing_observations:
        raise ValueError(
            "evaluation input is missing frozen gripper-to-fixed observations "
            f"{missing_observations[:5]}")

    def base_camera_pose(obs: PixelObs) -> np.ndarray:
        camera = int(obs.cam)
        if camera == gripper:
            event = int(obs.event)
            if event not in robot_T:
                raise KeyError(f"robot FK missing for gripper event {event}")
            return (np.asarray(robot_T[event], dtype=np.float64)
                    @ np.asarray(gtc, dtype=np.float64))
        return np.asarray(cams[camera], dtype=np.float64)

    target_in_base = {}
    camera_pose_by_id = {}
    for key in declared:
        obs = by_id[key]
        T_camera_target = solve_observed_pose(obs, K_map, D_map)
        if T_camera_target is None:
            raise RuntimeError(
                f"prevalidated gripper-to-fixed PnP became invalid: {key}")
        camera_pose_by_id[key] = base_camera_pose(obs)
        target_in_base[key] = camera_pose_by_id[key] @ T_camera_target

    accumulators = {
        target: {"events": defaultdict(
            lambda: {"pose": [], "pixel_squared": [], "pairs": 0}),
                 "rows": []}
        for target in CROSS_TARGETS
    }
    for pair in mask["pairs"]:
        target = str(pair["target"])
        fixed_id = pair["fixed_observation_id"]
        gripper_id = pair["gripper_observation_id"]
        dt, dr = fixed_camera_target_pair_disagreement(
            target_in_base[fixed_id], target_in_base[gripper_id])
        event_key = (int(pair["set"]), int(pair["gripper_event"]))
        event_accumulator = accumulators[target]["events"][event_key]
        event_accumulator["pose"].append((dt, dr))
        event_accumulator["pairs"] += 1
        direction_rows = []
        for source_id, destination_id in (
                (fixed_id, gripper_id), (gripper_id, fixed_id)):
            destination = by_id[destination_id]
            destination_camera = int(destination.cam)
            T_destination_target = (
                inv_T(camera_pose_by_id[destination_id])
                @ target_in_base[source_id])
            prediction = project_points(
                T_destination_target, destination.object_points,
                K_map[destination_camera], D_map[destination_camera])
            measured = np.asarray(
                destination.image_points, dtype=np.float64).reshape(-1, 2)
            if prediction.shape != measured.shape or not np.all(np.isfinite(prediction)):
                raise RuntimeError(
                    "invalid gripper-to-fixed pixel transfer: "
                    f"{source_id} -> {destination_id}")
            squared = np.square(prediction - measured).reshape(-1)
            event_accumulator["pixel_squared"].extend(squared.tolist())
            direction_rows.append({
                "source_observation_id": source_id,
                "destination_observation_id": destination_id,
                "rmse_px": float(np.sqrt(np.mean(squared))),
                "n_corners": int(len(prediction)),
            })
        accumulators[target]["rows"].append({
            **dict(pair),
            "translation_mm": dt,
            "rotation_deg": dr,
            "pixel_transfer_directions": direction_rows,
        })

    by_target = {}
    for target in CROSS_TARGETS:
        event_rows = []
        for (set_index, event), values in sorted(
                accumulators[target]["events"].items()):
            translation_rmse, rotation_rmse = pairwise_rmse(values["pose"])
            pixel_rmse = (
                None if not values["pixel_squared"] else
                float(np.sqrt(np.mean(values["pixel_squared"]))))
            event_rows.append({
                "set": int(set_index),
                "event": int(event),
                "n_pairs": int(values["pairs"]),
                "pose_consistency_translation_rmse_mm": translation_rmse,
                "pose_consistency_rotation_rmse_deg": rotation_rmse,
                "cross_view_pixel_transfer_rmse_px": pixel_rmse,
            })
        fields = (
            "pose_consistency_translation_rmse_mm",
            "pose_consistency_rotation_rmse_deg",
            "cross_view_pixel_transfer_rmse_px",
        )
        overall, set_rows = _event_then_set_aggregate(event_rows, fields)
        pair_rows = accumulators[target]["rows"]
        by_target[target] = {
            **overall,
            "aggregation": (
                "pair_components_to_event_RMSE_then_event_to_set_RMSE_then_"
                "equal_weight_RMSE_across_sets"),
            "n_pairs": len(pair_rows),
            "n_directions": 2 * len(pair_rows),
            "n_events": len(event_rows),
            "n_sets": len(set_rows),
            "n_output_rejected": 0,
            "per_pair": pair_rows,
            "per_event": event_rows,
            "per_set": set_rows,
        }
    return {
        "applicable": True,
        "evaluation_mask_sha256": mask["evaluation_mask_sha256"],
        "metric_contract": GRIPPER_TO_FIXED_CROSS_TARGET_CONTRACT,
        "model_dependent_gating": False,
        "output_dependent_pose_gate": None,
        "by_target": by_target,
        "n_output_rejected": 0,
        "interpretation": (
            "visual cross-view residual through the calibrated hand-eye and "
            "robot FK chain; it is not an FK-free or absolute-accuracy metric"),
    }


def build_frozen_path_evaluation_mask(
        observations: Sequence[PixelObs], fixed_camera_ids: Iterable[int],
        gripper_cam_idx: int, K_map, D_map,
        set_filter: Sequence[int], *, require_cross: bool = True,
        require_e2e: bool = True,
        fixed_anchor_observations: Optional[Sequence[PixelObs]] = None,
        event_roles: Optional[Mapping[int, str]] = None) -> dict:
    """Freeze held-out cube paths with one fixed anchor per set/camera."""
    fixed_cameras = tuple(sorted({int(ci) for ci in fixed_camera_ids}))
    gripper = int(gripper_cam_idx)
    allowed_sets = {int(set_index) for set_index in set_filter}
    roles = {int(event): str(role) for event, role in (event_roles or {}).items()}
    allowed_cameras = set(fixed_cameras) | {gripper}
    evaluation_candidates = [
        obs for obs in observations
        if obs.marker == "cube" and obs.set_idx is not None
        and int(obs.set_idx) in allowed_sets and int(obs.cam) in allowed_cameras
    ]
    anchor_source = (
        observations if fixed_anchor_observations is None
        else fixed_anchor_observations)
    anchor_groups: Dict[tuple[int, int], list[PixelObs]] = defaultdict(list)
    for obs in anchor_source:
        if (obs.marker != "cube" or obs.set_idx is None
                or int(obs.set_idx) not in allowed_sets
                or int(obs.cam) not in fixed_cameras):
            continue
        anchor_groups[(int(obs.set_idx), int(obs.cam))].append(obs)
    selected_anchors = [
        min(group, key=lambda obs: (int(obs.event), observation_id(obs)))
        for _, group in sorted(anchor_groups.items())
    ]
    by_id: Dict[str, PixelObs] = {}
    for obs in list(evaluation_candidates) + list(selected_anchors):
        key = observation_id(obs)
        if key in by_id:
            if by_id[key] is obs:
                continue
            raise ValueError(f"duplicate path-evaluation observation ID: {key}")
        by_id[key] = obs

    valid_ids, pnp_invalid_ids = [], []
    for key in sorted(by_id):
        if solve_observed_pose(by_id[key], K_map, D_map) is None:
            pnp_invalid_ids.append(key)
        else:
            valid_ids.append(key)

    valid = set(valid_ids)
    evaluation_ids = {
        observation_id(obs) for obs in evaluation_candidates}
    fixed_anchor_ids = sorted(observation_id(obs) for obs in selected_anchors)
    valid_by_event: Dict[tuple[int, int], list[str]] = defaultdict(list)
    for key in sorted(valid & evaluation_ids):
        obs = by_id[key]
        valid_by_event[(int(obs.event), int(obs.set_idx))].append(key)

    cross_pairs = []
    for (event, set_index), ids in sorted(valid_by_event.items()):
        fixed_ids = sorted(
            (key for key in ids if int(by_id[key].cam) in fixed_cameras),
            key=lambda key: int(by_id[key].cam))
        for left, right in combinations(fixed_ids, 2):
            cross_pairs.append({
                "event": event, "set": set_index,
                "left_observation_id": left, "right_observation_id": right,
            })

    anchors_by_set: Dict[int, list[str]] = defaultdict(list)
    for key in fixed_anchor_ids:
        if key in valid:
            anchors_by_set[int(by_id[key].set_idx)].append(key)
    e2e_units = []
    for key in sorted(valid & evaluation_ids):
        obs = by_id[key]
        if int(obs.cam) != gripper:
            continue
        fixed_ids = sorted(
            anchors_by_set.get(int(obs.set_idx), []),
            key=lambda anchor_id: int(by_id[anchor_id].cam))
        if fixed_ids:
            e2e_units.append({
                "event": int(obs.event),
                "gripper_event": int(obs.event),
                "set": int(obs.set_idx),
                "fixed_anchor_events": sorted({
                    int(by_id[anchor_id].event) for anchor_id in fixed_ids}),
                "fixed_anchor_split_roles": sorted({
                    roles.get(int(by_id[anchor_id].event), "unspecified")
                    for anchor_id in fixed_ids}),
                "gripper_split_role": roles.get(
                    int(obs.event), "evaluation"),
                "fixed_observation_ids": fixed_ids,
                "eih_observation_id": key,
            })

    body = {
        "artifact_schema": MASK_SCHEMA,
        "split_role": "heldout_gripper_event_with_same_set_first_fixed_anchor",
        "fixed_anchor_selection": (
            "earliest_event_per_set_fixed_camera_before_PnP_validation"),
        "selection_basis": (
            "detector_quality_mask_and_measurement_only_PnP_validity_before_model_fit"),
        "model_output_used_for_selection": False,
        "output_dependent_pose_gate": None,
        "gripper_camera_id": gripper,
        "fixed_camera_ids": list(fixed_cameras),
        "set_ids": sorted(allowed_sets),
        "candidate_cube_observation_ids": sorted(by_id),
        "fixed_anchor_observation_ids": fixed_anchor_ids,
        "evaluation_observation_ids": sorted(evaluation_ids),
        "valid_cube_observation_ids": valid_ids,
        "pnp_invalid_observation_ids": pnp_invalid_ids,
        "source_observation_sha256": _source_observation_sha256(
            list(by_id.values())),
        "cross_pairs": cross_pairs,
        "e2e_units": e2e_units,
        "aggregation": {
            "e_cross": "RMSE_over_all_predeclared_fixed_camera_pairs",
            "e_e2e": (
                "fixed_bundle_to_event_then_event_to_set_then_equal_set_"
                "weighted_RMSE"),
        },
        "e_cross_contract": E_CROSS_CONTRACT,
        "e_cross_pixel_transfer_contract": E_CROSS_PIXEL_TRANSFER_CONTRACT,
        "required_metric_support": {
            "e_cross": bool(require_cross),
            "e_e2e": bool(require_e2e),
        },
    }
    body["evaluation_mask_sha256"] = _sha256_json(body)
    validate_frozen_path_evaluation_mask(body)
    missing = []
    if require_cross and not cross_pairs:
        missing.append("e_cross")
    if require_e2e and not e2e_units:
        missing.append("e_e2e")
    if missing:
        raise RuntimeError(
            "held-out observations do not support required path metrics: "
            + ", ".join(missing))
    return body


def validate_frozen_path_evaluation_mask(mask: Mapping) -> None:
    if mask.get("artifact_schema") != MASK_SCHEMA:
        raise ValueError("unknown path-evaluation mask schema")
    if mask.get("model_output_used_for_selection") is not False:
        raise ValueError("path-evaluation selection must be model independent")
    if mask.get("output_dependent_pose_gate") is not None:
        raise ValueError("output-dependent translation/rotation gates are forbidden")
    contract = mask.get("e_cross_contract", {})
    if contract != E_CROSS_CONTRACT:
        raise ValueError("e_cross is not the canonical fixed-camera cube consistency metric")
    if mask.get("e_cross_pixel_transfer_contract") != E_CROSS_PIXEL_TRANSFER_CONTRACT:
        raise ValueError("cross-view pixel transfer contract mismatch")
    valid_ids = set(map(str, mask.get("valid_cube_observation_ids", [])))
    fixed_anchor_ids = set(map(
        str, mask.get("fixed_anchor_observation_ids", [])))
    evaluation_ids = set(map(
        str, mask.get("evaluation_observation_ids", [])))
    for pair in mask.get("cross_pairs", []):
        if (str(pair.get("left_observation_id")) not in evaluation_ids
                or str(pair.get("right_observation_id")) not in evaluation_ids):
            raise ValueError("e_cross pair is not from the evaluation population")
    for unit in mask.get("e2e_units", []):
        fixed_ids = set(map(str, unit.get("fixed_observation_ids", [])))
        eih_id = str(unit.get("eih_observation_id", ""))
        if (not fixed_ids or not fixed_ids.issubset(fixed_anchor_ids)
                or not fixed_ids.issubset(valid_ids)
                or eih_id not in evaluation_ids or eih_id not in valid_ids):
            raise ValueError("e_e2e unit violates set-anchor roles")
        if int(unit.get("event", -1)) != int(unit.get("gripper_event", -2)):
            raise ValueError("e_e2e unit event is not the gripper event")
    unhashed = dict(mask)
    expected = str(unhashed.pop("evaluation_mask_sha256", ""))
    if not expected or _sha256_json(unhashed) != expected:
        raise ValueError("path-evaluation mask SHA-256 mismatch")


def not_applicable_path_metrics(mask: Mapping, reason: str) -> dict:
    validate_frozen_path_evaluation_mask(mask)
    return {
        "applicable": False,
        "reason": str(reason),
        "evaluation_mask_sha256": mask["evaluation_mask_sha256"],
        "model_dependent_gating": False,
        "output_dependent_pose_gate": None,
        "e_cross_translation_rmse_mm": None,
        "e_cross_rotation_rmse_deg": None,
        "fixed_camera_cube_position_consistency_rmse_mm": None,
        "fixed_camera_cube_rotation_consistency_rmse_deg": None,
        "cross_view_pixel_transfer_rmse_px": None,
        "e_e2e_translation_rmse_mm": None,
        "e_e2e_rotation_rmse_deg": None,
        "n_cross_pairs": 0,
        "n_cross_view_directions": 0,
        "n_e2e_units": 0,
        "n_e2e_events": 0,
        "n_e2e_sets": 0,
    }


def evaluate_paths_with_frozen_mask(
        observations: Sequence[PixelObs], cams, gtc, robot_T,
        gripper_cam_idx: int, K_map, D_map, mask: Mapping) -> dict:
    """Evaluate every predeclared unit; never reject a unit by its error."""
    validate_frozen_path_evaluation_mask(mask)
    gripper = int(gripper_cam_idx)
    if gripper != int(mask["gripper_camera_id"]):
        raise ValueError("gripper camera does not match the frozen evaluation mask")
    missing_cameras = sorted(set(map(int, mask["fixed_camera_ids"])) - set(map(int, cams)))
    if missing_cameras:
        raise ValueError(
            f"calibration row lacks frozen-mask cameras {missing_cameras}")

    by_id = {}
    for obs in observations:
        if obs.marker != "cube" or obs.set_idx is None:
            continue
        key = observation_id(obs)
        if key in by_id:
            raise ValueError(f"duplicate path-evaluation observation ID: {key}")
        by_id[key] = obs
    declared_ids = list(mask["valid_cube_observation_ids"])
    missing_observations = sorted(set(declared_ids) - set(by_id))
    if missing_observations:
        raise ValueError(
            "row input is missing frozen-mask observations "
            f"{missing_observations[:5]}")

    target_in_base = {}
    predicted_by_set: Dict[int, list[np.ndarray]] = defaultdict(list)
    for key in declared_ids:
        obs = by_id[key]
        T_cam_target = solve_observed_pose(obs, K_map, D_map)
        if T_cam_target is None:
            raise RuntimeError(f"prevalidated PnP observation became invalid: {key}")
        if int(obs.cam) == gripper:
            if int(obs.event) not in robot_T:
                raise KeyError(f"robot FK missing for held-out event {obs.event}")
            transform = np.asarray(robot_T[int(obs.event)]) @ np.asarray(gtc) @ T_cam_target
        else:
            transform = fixed_camera_cube_pose_in_base(
                cams[int(obs.cam)], T_cam_target)
        target_in_base[key] = transform
        predicted_by_set[int(obs.set_idx)].append(transform)

    cross_values = []
    cross_view_pixel_squared = []
    cross_rows, e2e_rows = [], []
    for pair in mask["cross_pairs"]:
        left_id = pair["left_observation_id"]
        right_id = pair["right_observation_id"]
        dt, dr = fixed_camera_cube_pair_disagreement(
            target_in_base[left_id], target_in_base[right_id])
        cross_values.append((dt, dr))
        direction_rows = []
        for source_id, destination_id in (
                (left_id, right_id), (right_id, left_id)):
            destination = by_id[destination_id]
            destination_camera = int(destination.cam)
            T_destination_cube = (
                inv_T(np.asarray(cams[destination_camera], dtype=np.float64))
                @ target_in_base[source_id]
            )
            prediction = project_points(
                T_destination_cube,
                destination.object_points,
                K_map[destination_camera],
                D_map[destination_camera],
            )
            measured = np.asarray(
                destination.image_points, dtype=np.float64).reshape(-1, 2)
            if prediction.shape != measured.shape or not np.all(np.isfinite(prediction)):
                raise RuntimeError(
                    "invalid cross-view pixel transfer in predeclared pair: "
                    f"{source_id} -> {destination_id}")
            squared = np.square(prediction - measured).reshape(-1)
            cross_view_pixel_squared.extend(squared.tolist())
            direction_rows.append({
                "source_observation_id": source_id,
                "destination_observation_id": destination_id,
                "rmse_px": float(np.sqrt(np.mean(squared))),
                "n_corners": int(len(prediction)),
            })
        cross_rows.append({
            **dict(pair),
            "translation_mm": dt,
            "rotation_deg": dr,
            "pixel_transfer_directions": direction_rows,
        })
    for unit in mask["e2e_units"]:
        fixed = [target_in_base[key] for key in unit["fixed_observation_ids"]]
        fixed_average = cp.robust_se3_average(fixed, None)[0]
        dt, dr = pose_delta(fixed_average, target_in_base[unit["eih_observation_id"]])
        e2e_rows.append({**dict(unit), "translation_mm": dt, "rotation_deg": dr})

    e2e_by_event: Dict[tuple[int, int], list[Mapping]] = defaultdict(list)
    for row in e2e_rows:
        e2e_by_event[(int(row["set"]), int(row["event"]))].append(row)
    e2e_event_rows = []
    for (set_index, event), rows in sorted(e2e_by_event.items()):
        e2e_event_rows.append({
            "set": int(set_index),
            "event": int(event),
            "n_units": len(rows),
            "translation_mm": _scalar_rmse([
                float(row["translation_mm"]) for row in rows]),
            "rotation_deg": _scalar_rmse([
                float(row["rotation_deg"]) for row in rows]),
        })
    e2e_overall, e2e_set_rows = _event_then_set_aggregate(
        e2e_event_rows, ("translation_mm", "rotation_deg"))

    cross_translation_rmse, cross_rotation_rmse = pairwise_rmse(cross_values)

    return {
        "applicable": True,
        "evaluation_mask_sha256": mask["evaluation_mask_sha256"],
        "model_dependent_gating": False,
        "output_dependent_pose_gate": None,
        "e_cross_definition": E_CROSS_CONTRACT,
        "e_cross_pixel_transfer_definition": E_CROSS_PIXEL_TRANSFER_CONTRACT,
        "e_cross_translation_rmse_mm": cross_translation_rmse,
        "e_cross_rotation_rmse_deg": cross_rotation_rmse,
        "fixed_camera_cube_position_consistency_rmse_mm": cross_translation_rmse,
        "fixed_camera_cube_rotation_consistency_rmse_deg": cross_rotation_rmse,
        "cross_view_pixel_transfer_rmse_px": (
            None if not cross_view_pixel_squared else
            float(np.sqrt(np.mean(cross_view_pixel_squared)))),
        "e_e2e_translation_rmse_mm": e2e_overall["translation_mm"],
        "e_e2e_rotation_rmse_deg": e2e_overall["rotation_deg"],
        "e_e2e_aggregation": (
            "fixed_bundle_to_event_then_event_to_set_then_equal_set_"
            "weighted_RMSE"),
        "n_cross_pairs": len(cross_values),
        "n_cross_view_directions": 2 * len(cross_values),
        "n_e2e_units": len(e2e_rows),
        "n_e2e_events": len(e2e_event_rows),
        "n_e2e_sets": len(e2e_set_rows),
        "n_output_rejected": 0,
        "per_cross_pair": cross_rows,
        "per_e2e_unit": e2e_rows,
        "per_e2e_event": e2e_event_rows,
        "per_e2e_set": e2e_set_rows,
        "predicted_by_set": dict(predicted_by_set),
    }
