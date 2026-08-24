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
from calibration_pipeline.path_evaluation import evaluate_paths_with_common_mask
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
        for key in keys:
            squared_residuals[key].extend(residual_squared.tolist())
            counts[key]["observations"] += 1
            counts[key]["corners"] += len(prediction)

    result = {
        key: {
            "rmse_px": float(np.sqrt(np.mean(values))),
            "n_observations": int(counts[key]["observations"]),
            "n_corners": int(counts[key]["corners"]),
        }
        for key, values in sorted(squared_residuals.items())
    }
    result["metric_contract"] = dict(REPROJECTION_METRIC_CONTRACT)
    result["unsupported"] = []
    return result


def common_target_observation_groups(
    test_observations: Sequence[PixelObs], common_fixed_cameras: Sequence[int]
) -> dict[str, list[PixelObs]]:
    """Return one fixed-camera board/cube held-out population for all methods."""
    cameras = {int(camera) for camera in common_fixed_cameras}

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
        raise RuntimeError("common-target held-out evaluation group is empty")
    return groups


def evaluate_common_target_run(
    raw_transforms: Mapping,
    common_fixed_cameras: Sequence[int],
    observation_groups: Mapping[str, Sequence[PixelObs]],
    common_board_pose: np.ndarray,
    common_cube_poses: Mapping[int, np.ndarray],
    all_test_observations: Sequence[PixelObs],
    robot_T: Mapping[int, np.ndarray],
    K_map: Mapping[int, np.ndarray],
    D_map: Mapping[int, np.ndarray],
    gripper_cam_idx: int,
    path_evaluation_mask: Mapping,
) -> dict:
    """Evaluate frozen calibration on shared pixel and path-consistency data."""
    stored = deserialize_state(raw_transforms)
    cameras = {int(camera) for camera in common_fixed_cameras}
    evaluation_state = PoseState(
        cams={camera: stored.cams[camera] for camera in sorted(cameras)},
        gtc=stored.gtc,
        board=np.asarray(common_board_pose, dtype=np.float64),
        cubes={
            int(set_index): np.asarray(transform, dtype=np.float64)
            for set_index, transform in common_cube_poses.items()
        },
    )
    output = {}
    for label in ("overall", "board", "cube"):
        metrics = pixel_reprojection_metrics(
            observation_groups[label], evaluation_state, robot_T, K_map,
            D_map, gripper_cam_idx)
        overall = metrics["overall"]
        output[label] = {
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
    path = evaluate_paths_with_common_mask(
        all_test_observations,
        evaluation_state.cams,
        evaluation_state.gtc,
        robot_T,
        gripper_cam_idx,
        K_map,
        D_map,
        path_evaluation_mask,
    )
    path.pop("predicted_by_set", None)
    output["common_path"] = path
    return output
