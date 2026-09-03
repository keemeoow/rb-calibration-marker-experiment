"""Canonical Table 1 serialization and frozen held-out evaluation.

Only distorted-image pixel reprojection is reported.  Reprojection residuals
are not converted to millimetres: doing so from predicted depth creates a
view-dependent approximation, not an independently measured 3D error.
"""

from __future__ import annotations

from collections import defaultdict
import hashlib
import json
from typing import Any, Dict, Mapping, Sequence

import numpy as np

from calibration_pipeline.apriltag_cube import inv_T
from calibration_pipeline.path_evaluation import (
    evaluate_gripper_to_fixed_cross_target,
    evaluate_paths_with_frozen_mask,
    evaluate_fixed_to_fixed_cross_target,
)
from calibration_pipeline.reprojection import PixelObs, PoseState, project_points


REPROJECTION_METRIC_CONTRACT = {
    "primary_field": "rmse_px",
    "domain": "distorted_native_image_pixels",
    "residual": "predicted_corner_uv_minus_measured_corner_uv",
    "aggregation": "component_wise_RMSE_sqrt_sum_du2_dv2_over_2N",
    "parameters_frozen_during_evaluation": True,
    "model_dependent_observation_rejection": False,
    "millimetre_reprojection_conversion_reported": False,
}

# Corner-pooled RMSE weights a set by however many corners it happened to
# expose, so one densely detected placement can outvote the rest.  The same
# event-then-set order the cross-target paths already use gives every placement
# one vote instead.
SET_EQUAL_WEIGHT_REPROJECTION_CONTRACT = {
    "name": "set_equal_weight_heldout_reprojection",
    "aggregation": (
        "corner_components_to_event_RMSE_then_event_to_set_RMSE_then_"
        "equal_weight_RMSE_across_sets"),
    "role": "pooling_bias_control_reported_beside_corner_pooled_rmse_px",
    "unit": "distorted_native_image_pixels",
    "requires_set_index_on_every_observation": True,
    "computed_on_the_same_frozen_population": True,
    "may_rank_methods_before_external_gt": False,
}

# Only the slices named by ``EVALUATION_COMPARISON_CONTRACT`` carry the full
# per-set breakdown; every other slice keeps the scalar so the artifact does
# not grow a per-set table for all 32 camera/role combinations.
SET_EQUAL_WEIGHT_DETAIL_KEYS = ("overall", "board", "cube")

REFERENCE_DEPENDENT_REPROJECTION_CONTRACT = {
    "name": "shared_train_target_pose_reprojection_diagnostic",
    "uses_same_heldout_fixed_camera_corners": True,
    "uses_shared_train_target_pose": True,
    "reference_is_external_ground_truth": False,
    "reference_is_neutral_across_calibration_methods": False,
    "role_before_external_gt": "secondary_reference_dependent_diagnostic",
    "may_rank_methods_before_external_gt": False,
    "limitation": (
        "the board and cube base-frame poses are train-derived internal "
        "references and may be closer to methods that share their assumptions"),
}


def jsonable(value: Any) -> Any:
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.integer):
        return int(value)
    if isinstance(value, np.floating):
        return float(value)
    if isinstance(value, dict):
        return {str(key): jsonable(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [jsonable(item) for item in value]
    return value


def serialize_state(state: PoseState) -> dict:
    return {
        "T_base_Ci": {
            str(camera): transform.tolist()
            for camera, transform in sorted(state.cams.items())
        },
        "T_gripper_cam": state.gtc.tolist(),
        "T_base_board": None if state.board is None else state.board.tolist(),
        "T_base_cube_by_set": {
            str(set_index): transform.tolist()
            for set_index, transform in sorted(state.cubes.items())
        },
    }


def deserialize_state(raw: Mapping) -> PoseState:
    return PoseState(
        cams={
            int(camera): np.asarray(transform, dtype=np.float64)
            for camera, transform in raw["T_base_Ci"].items()
        },
        gtc=np.asarray(raw["T_gripper_cam"], dtype=np.float64),
        board=(
            None
            if raw.get("T_base_board") is None
            else np.asarray(raw["T_base_board"], dtype=np.float64)
        ),
        cubes={
            int(set_index): np.asarray(transform, dtype=np.float64)
            for set_index, transform in raw.get(
                "T_base_cube_by_set", {}).items()
        },
    )


def canonical_json_sha256(payload: Mapping) -> str:
    encoded = json.dumps(
        jsonable(payload), sort_keys=True, separators=(",", ":"),
        allow_nan=False,
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def state_sha256(state: PoseState) -> str:
    return canonical_json_sha256(serialize_state(state))


def observations_sha256(observations: Sequence[PixelObs]) -> str:
    """Order-independent digest of the exact calibrated-corner population."""
    records = []
    for observation in observations:
        record = hashlib.sha256()
        identity = {
            "marker": str(observation.marker),
            "cam": int(observation.cam),
            "event": int(observation.event),
            "set_idx": (None if observation.set_idx is None
                        else int(observation.set_idx)),
            "grasp_idx": (None if observation.grasp_idx is None
                          else int(observation.grasp_idx)),
        }
        record.update(json.dumps(
            identity, sort_keys=True, separators=(",", ":")).encode("utf-8"))
        for values in (observation.object_points, observation.image_points):
            array = np.ascontiguousarray(values, dtype="<f8")
            record.update(str(array.shape).encode("ascii"))
            record.update(array.tobytes())
        records.append(record.digest())
    digest = hashlib.sha256()
    digest.update(str(len(records)).encode("ascii"))
    for record in sorted(records):
        digest.update(record)
    return digest.hexdigest()


def set_equal_weight_rmse(
    event_squared: Mapping[tuple[int, int], Sequence[float]],
) -> dict:
    """Collapse corners within an event, events within a set, then sets.

    Each level takes the mean square and the final value is its square root, so
    the result is the RMS of set RMSEs.  This is the ordering
    ``_event_then_set_aggregate`` already applies to the cross-target paths;
    reusing it keeps one placement worth one vote in both places.
    """
    event_mean_squares: Dict[int, list[float]] = defaultdict(list)
    events: Dict[int, list[int]] = defaultdict(list)
    corners: Dict[int, int] = defaultdict(int)
    for (set_index, event), values in sorted(event_squared.items()):
        event_mean_squares[set_index].append(float(np.mean(values)))
        events[set_index].append(int(event))
        # Two residual components (du, dv) per corner.
        corners[set_index] += len(values) // 2
    per_set = []
    for set_index in sorted(event_mean_squares):
        set_mean_square = float(np.mean(event_mean_squares[set_index]))
        per_set.append({
            "set": int(set_index),
            "n_events": len(event_mean_squares[set_index]),
            "events": sorted(events[set_index]),
            "n_corners": int(corners[set_index]),
            "rmse_px": float(np.sqrt(set_mean_square)),
            "mean_square_px2": set_mean_square,
        })
    overall = float(np.sqrt(np.mean(
        [row["mean_square_px2"] for row in per_set])))
    return {"rmse_px": overall, "per_set": per_set}


def pixel_reprojection_metrics(
    observations: Sequence[PixelObs],
    state: PoseState,
    robot_T: Mapping[int, np.ndarray],
    K_map: Mapping[int, np.ndarray],
    D_map: Mapping[int, np.ndarray],
    gripper_cam_idx: int,
) -> dict:
    """Evaluate every supplied corner in native distorted-image pixels.

    The observation list is the evaluation population.  Invalid/missing model
    state fails the evaluation instead of silently removing observations based
    on a fitted method's output.
    """
    squared_residuals: Dict[str, list[float]] = defaultdict(list)
    counts: Dict[str, Dict[str, int]] = defaultdict(
        lambda: {"observations": 0, "corners": 0})
    # key -> (set, event) -> residual components, for the equal-weight pooling.
    by_set_event: Dict[str, Dict[tuple[int, int], list[float]]] = defaultdict(
        lambda: defaultdict(list))
    without_set_index: set[str] = set()
    gripper = int(gripper_cam_idx)
    for observation in observations:
        camera_id = int(observation.cam)
        role = "eih" if camera_id == gripper else "e2h"
        camera_key = f"cam_{camera_id}"
        keys = (
            "overall",
            observation.marker,
            role,
            f"{observation.marker}_{role}",
            camera_key,
            f"{observation.marker}_{camera_key}",
            f"{role}_{camera_key}",
            f"{observation.marker}_{role}_{camera_key}",
        )
        if observation.marker == "board":
            target = state.board
        elif observation.marker == "cube" and observation.set_idx is not None:
            target = state.cubes.get(int(observation.set_idx))
        else:
            target = None
        if target is None:
            raise RuntimeError(
                "evaluation observation has no corresponding frozen target "
                f"pose: event={observation.event}, cam={camera_id}, "
                f"marker={observation.marker}, set={observation.set_idx}")

        if role == "eih":
            if int(observation.event) not in robot_T:
                raise RuntimeError(
                    f"evaluation event {observation.event} has no robot FK")
            T_base_camera = (
                np.asarray(robot_T[int(observation.event)], dtype=np.float64)
                @ state.gtc
            )
        else:
            if camera_id not in state.cams:
                raise RuntimeError(
                    f"evaluation camera {camera_id} is not registered")
            T_base_camera = state.cams[camera_id]

        T_camera_target = inv_T(T_base_camera) @ target
        prediction = project_points(
            T_camera_target,
            observation.object_points,
            K_map[camera_id],
            D_map[camera_id],
        )
        measured = np.asarray(
            observation.image_points, dtype=np.float64).reshape(-1, 2)
        if prediction.shape != measured.shape or not np.all(np.isfinite(prediction)):
            raise RuntimeError(
                "non-finite or shape-mismatched pixel projection in the fixed "
                f"evaluation population: event={observation.event}, cam={camera_id}")
        residual_squared = np.square(prediction - measured).reshape(-1)
        components = residual_squared.tolist()
        set_index = observation.set_idx
        for key in keys:
            squared_residuals[key].extend(components)
            counts[key]["observations"] += 1
            counts[key]["corners"] += len(prediction)
            if set_index is None:
                without_set_index.add(key)
            else:
                by_set_event[key][
                    (int(set_index), int(observation.event))].extend(components)

    result = {}
    for key, values in sorted(squared_residuals.items()):
        entry = {
            "rmse_px": float(np.sqrt(np.mean(values))),
            "n_observations": int(counts[key]["observations"]),
            "n_corners": int(counts[key]["corners"]),
        }
        if key in without_set_index:
            # Refuse to compute rather than pool a subset: dropping the
            # set-less corners would silently reweight the population.
            entry["set_equal_weight_rmse_px"] = None
            entry["set_equal_weight_unsupported_reason"] = (
                "observation_without_set_index_in_this_population")
        else:
            aggregate = set_equal_weight_rmse(by_set_event[key])
            entry["set_equal_weight_rmse_px"] = aggregate["rmse_px"]
            entry["n_sets"] = len(aggregate["per_set"])
            entry["n_events"] = sum(
                int(row["n_events"]) for row in aggregate["per_set"])
            if key in SET_EQUAL_WEIGHT_DETAIL_KEYS:
                entry["set_equal_weight_per_set"] = aggregate["per_set"]
        result[key] = entry
    result["metric_contract"] = dict(REPROJECTION_METRIC_CONTRACT)
    result["set_equal_weight_contract"] = dict(
        SET_EQUAL_WEIGHT_REPROJECTION_CONTRACT)
    result["unsupported"] = []
    return result


def fixed_camera_board_cube_groups(
    test_observations: Sequence[PixelObs], fixed_camera_ids: Sequence[int]
) -> dict[str, list[PixelObs]]:
    """Return held-out board/cube corners on an explicit camera intersection."""
    cameras = {int(camera) for camera in fixed_camera_ids}

    def selected(marker: str | None = None) -> list[PixelObs]:
        return [
            observation
            for observation in test_observations
            if int(observation.cam) in cameras
            and observation.marker in {"board", "cube"}
            and (marker is None or observation.marker == marker)
        ]

    groups = {
        "overall": selected(),
        "board": selected("board"),
        "cube": selected("cube"),
    }
    if not all(groups.values()):
        raise RuntimeError("fixed-camera board/cube evaluation group is empty")
    return groups


def evaluate_internal_run(
    raw_transforms: Mapping,
    evaluation_fixed_cameras: Sequence[int],
    observation_groups: Mapping[str, Sequence[PixelObs]],
    shared_train_board_pose: np.ndarray,
    shared_train_cube_poses: Mapping[int, np.ndarray],
    all_test_observations: Sequence[PixelObs],
    robot_T: Mapping[int, np.ndarray],
    K_map: Mapping[int, np.ndarray],
    D_map: Mapping[int, np.ndarray],
    gripper_cam_idx: int,
    path_evaluation_mask: Mapping,
    fixed_to_fixed_cross_target_mask: Mapping,
    gripper_to_fixed_cross_target_mask: Mapping,
) -> dict:
    """Evaluate one frozen calibration before independent external GT exists.

    Fixed-to-fixed isolates relative fixed-camera calibration without FK.
    Gripper-to-fixed uses the same visual cross-view idea but necessarily passes
    through robot FK and hand-eye.  Shared-target reprojection and the legacy
    cube path remain explicitly labelled diagnostics.
    """
    stored = deserialize_state(raw_transforms)
    cameras = {int(camera) for camera in evaluation_fixed_cameras}
    evaluation_state = PoseState(
        cams={camera: stored.cams[camera] for camera in sorted(cameras)},
        gtc=stored.gtc,
        board=np.asarray(shared_train_board_pose, dtype=np.float64),
        cubes={
            int(set_index): np.asarray(transform, dtype=np.float64)
            for set_index, transform in shared_train_cube_poses.items()
        },
    )
    reference_dependent = {}
    for label in ("overall", "board", "cube"):
        metrics = pixel_reprojection_metrics(
            observation_groups[label], evaluation_state, robot_T, K_map,
            D_map, gripper_cam_idx)
        overall = metrics["overall"]
        reference_dependent[label] = {
            "rmse_px": float(overall["rmse_px"]),
            "n_observations": int(overall["n_observations"]),
            "n_corners": int(overall["n_corners"]),
            "by_camera": {
                str(camera): {
                    "rmse_px": float(metrics[f"cam_{camera}"]["rmse_px"])
                }
                for camera in sorted(cameras)
            },
        }
    reference_dependent["metric_contract"] = dict(
        REFERENCE_DEPENDENT_REPROJECTION_CONTRACT)

    fixed_to_fixed = evaluate_fixed_to_fixed_cross_target(
        all_test_observations,
        evaluation_state.cams,
        K_map,
        D_map,
        fixed_to_fixed_cross_target_mask,
    )
    gripper_to_fixed = evaluate_gripper_to_fixed_cross_target(
        all_test_observations,
        evaluation_state.cams,
        evaluation_state.gtc,
        robot_T,
        K_map,
        D_map,
        gripper_to_fixed_cross_target_mask,
    )
    legacy_path = evaluate_paths_with_frozen_mask(
        all_test_observations,
        evaluation_state.cams,
        evaluation_state.gtc,
        robot_T,
        gripper_cam_idx,
        K_map,
        D_map,
        path_evaluation_mask,
    )
    legacy_path.pop("predicted_by_set", None)
    return {
        "fixed_to_fixed": fixed_to_fixed,
        "gripper_to_fixed": gripper_to_fixed,
        "reference_dependent_reprojection": reference_dependent,
        "legacy_fk_dependent_cube_path": legacy_path,
        "external_ground_truth_used": False,
        "absolute_accuracy_evaluated": False,
    }
