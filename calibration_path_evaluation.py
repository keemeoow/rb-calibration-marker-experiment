"""Model-independent path-consistency evaluation for calibration ablations.

The evaluation population is frozen before a calibration row is fitted.  A
cube observation may be excluded only because it is absent from the detector
quality mask or because its image/object points do not admit a finite PnP
pose.  Calibrated camera/target predictions never decide which pairs remain.
"""

from __future__ import annotations

import hashlib
import json
from collections import defaultdict
from itertools import combinations
from typing import Dict, Iterable, Mapping, Optional, Sequence

import cv2
import numpy as np

import CP_common as cp
from calibration_reprojection_backend import PixelObs, pose_delta


MASK_SCHEMA = "model_independent_path_evaluation_mask_v1"


def solve_observed_pose(obs: PixelObs, K_map, D_map) -> Optional[np.ndarray]:
    """Return ``T_cam_target`` from measured corners only, or ``None``."""
    obj = np.asarray(obs.object_points, dtype=np.float64).reshape(-1, 3)
    img = np.asarray(obs.image_points, dtype=np.float64).reshape(-1, 2)
    if len(obj) < 4:
        return None
    planar = float(np.ptp(obj[:, 2])) < 1e-9
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


def observation_id(obs: PixelObs) -> str:
    if obs.marker != "cube" or obs.set_idx is None:
        raise ValueError("path evaluation IDs are defined only for set-labelled cube observations")
    return f"cube:event={int(obs.event)}:set={int(obs.set_idx)}:cam={int(obs.cam)}"


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


def build_common_path_evaluation_mask(
        observations: Sequence[PixelObs], fixed_camera_ids: Iterable[int],
        gripper_cam_idx: int, K_map, D_map,
        set_filter: Sequence[int], *, require_cross: bool = True,
        require_e2e: bool = True) -> dict:
    """Freeze held-out path units without consulting a fitted model output."""
    fixed_cameras = tuple(sorted({int(ci) for ci in fixed_camera_ids}))
    gripper = int(gripper_cam_idx)
    allowed_sets = {int(set_index) for set_index in set_filter}
    allowed_cameras = set(fixed_cameras) | {gripper}
    candidates = [
        obs for obs in observations
        if obs.marker == "cube" and obs.set_idx is not None
        and int(obs.set_idx) in allowed_sets and int(obs.cam) in allowed_cameras
    ]
    by_id: Dict[str, PixelObs] = {}
    for obs in candidates:
        key = observation_id(obs)
        if key in by_id:
            raise ValueError(f"duplicate path-evaluation observation ID: {key}")
        by_id[key] = obs

    valid_ids, pnp_invalid_ids = [], []
    for key in sorted(by_id):
        if solve_observed_pose(by_id[key], K_map, D_map) is None:
            pnp_invalid_ids.append(key)
        else:
            valid_ids.append(key)

    valid_by_event: Dict[tuple[int, int], list[str]] = defaultdict(list)
    for key in valid_ids:
        obs = by_id[key]
        valid_by_event[(int(obs.event), int(obs.set_idx))].append(key)

    cross_pairs = []
    e2e_units = []
    for (event, set_index), ids in sorted(valid_by_event.items()):
        fixed_ids = sorted(
            (key for key in ids if int(by_id[key].cam) in fixed_cameras),
            key=lambda key: int(by_id[key].cam))
        eih_ids = sorted(
            (key for key in ids if int(by_id[key].cam) == gripper))
        for left, right in combinations(fixed_ids, 2):
            cross_pairs.append({
                "event": event, "set": set_index,
                "left_observation_id": left, "right_observation_id": right,
            })
        for eih_id in eih_ids:
            if fixed_ids:
                e2e_units.append({
                    "event": event, "set": set_index,
                    "fixed_observation_ids": fixed_ids,
                    "eih_observation_id": eih_id,
                })

    body = {
        "artifact_schema": MASK_SCHEMA,
        "split_role": "heldout_event_grouped_set_stratified",
        "selection_basis": (
            "detector_quality_mask_and_measurement_only_PnP_validity_before_model_fit"),
        "model_output_used_for_selection": False,
        "output_dependent_pose_gate": None,
        "gripper_camera_id": gripper,
        "fixed_camera_ids": list(fixed_cameras),
        "set_ids": sorted(allowed_sets),
        "candidate_cube_observation_ids": sorted(by_id),
        "valid_cube_observation_ids": valid_ids,
        "pnp_invalid_observation_ids": pnp_invalid_ids,
        "source_observation_sha256": _source_observation_sha256(candidates),
        "cross_pairs": cross_pairs,
        "e2e_units": e2e_units,
        "aggregation": {
            "e_cross": "RMSE_over_all_predeclared_fixed_camera_pairs",
            "e_e2e": (
                "RMSE_over_predeclared_events_after_SE3_average_of_each_exact_"
                "fixed_camera_bundle"),
        },
        "required_metric_support": {
            "e_cross": bool(require_cross),
            "e_e2e": bool(require_e2e),
        },
    }
    body["evaluation_mask_sha256"] = _sha256_json(body)
    validate_common_path_evaluation_mask(body)
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


def validate_common_path_evaluation_mask(mask: Mapping) -> None:
    if mask.get("artifact_schema") != MASK_SCHEMA:
        raise ValueError("unknown path-evaluation mask schema")
    if mask.get("model_output_used_for_selection") is not False:
        raise ValueError("path-evaluation selection must be model independent")
    if mask.get("output_dependent_pose_gate") is not None:
        raise ValueError("output-dependent translation/rotation gates are forbidden")
    unhashed = dict(mask)
    expected = str(unhashed.pop("evaluation_mask_sha256", ""))
    if not expected or _sha256_json(unhashed) != expected:
        raise ValueError("path-evaluation mask SHA-256 mismatch")


def not_applicable_path_metrics(mask: Mapping, reason: str) -> dict:
    validate_common_path_evaluation_mask(mask)
    return {
        "applicable": False,
        "reason": str(reason),
        "evaluation_mask_sha256": mask["evaluation_mask_sha256"],
        "model_dependent_gating": False,
        "output_dependent_pose_gate": None,
        "e_cross_translation_rmse_mm": None,
        "e_cross_rotation_rmse_deg": None,
        "e_e2e_translation_rmse_mm": None,
        "e_e2e_rotation_rmse_deg": None,
        "n_cross_pairs": 0,
        "n_e2e_units": 0,
    }


def evaluate_paths_with_common_mask(
        observations: Sequence[PixelObs], cams, gtc, robot_T,
        gripper_cam_idx: int, K_map, D_map, mask: Mapping) -> dict:
    """Evaluate every predeclared unit; never reject a unit by its error."""
    validate_common_path_evaluation_mask(mask)
    gripper = int(gripper_cam_idx)
    if gripper != int(mask["gripper_camera_id"]):
        raise ValueError("gripper camera does not match common evaluation mask")
    missing_cameras = sorted(set(map(int, mask["fixed_camera_ids"])) - set(map(int, cams)))
    if missing_cameras:
        raise ValueError(f"calibration row lacks common-mask cameras {missing_cameras}")

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
            f"row input is missing common-mask observations {missing_observations[:5]}")

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
            transform = np.asarray(cams[int(obs.cam)]) @ T_cam_target
        target_in_base[key] = transform
        predicted_by_set[int(obs.set_idx)].append(transform)

    cross_values, e2e_values = [], []
    cross_rows, e2e_rows = [], []
    for pair in mask["cross_pairs"]:
        dt, dr = pose_delta(
            target_in_base[pair["left_observation_id"]],
            target_in_base[pair["right_observation_id"]])
        cross_values.append((dt, dr))
        cross_rows.append({**dict(pair), "translation_mm": dt, "rotation_deg": dr})
    for unit in mask["e2e_units"]:
        fixed = [target_in_base[key] for key in unit["fixed_observation_ids"]]
        fixed_average = cp.robust_se3_average(fixed, None)[0]
        dt, dr = pose_delta(fixed_average, target_in_base[unit["eih_observation_id"]])
        e2e_values.append((dt, dr))
        e2e_rows.append({**dict(unit), "translation_mm": dt, "rotation_deg": dr})

    def rms(values, index):
        return (None if not values else
                float(np.sqrt(np.mean(np.square([value[index] for value in values])))))

    return {
        "applicable": True,
        "evaluation_mask_sha256": mask["evaluation_mask_sha256"],
        "model_dependent_gating": False,
        "output_dependent_pose_gate": None,
        "e_cross_translation_rmse_mm": rms(cross_values, 0),
        "e_cross_rotation_rmse_deg": rms(cross_values, 1),
        "e_e2e_translation_rmse_mm": rms(e2e_values, 0),
        "e_e2e_rotation_rmse_deg": rms(e2e_values, 1),
        "n_cross_pairs": len(cross_values),
        "n_e2e_units": len(e2e_values),
        "n_output_rejected": 0,
        "per_cross_pair": cross_rows,
        "per_e2e_unit": e2e_rows,
        "predicted_by_set": dict(predicted_by_set),
    }
