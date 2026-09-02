"""Cube event-camera observation quality and selection-policy tests."""

import json
from pathlib import Path

import numpy as np

from calibration_pipeline.apriltag_cube import AprilTagCubeTarget
from calibration_pipeline.cube_config import load_cube_config_from_meta
from calibration_pipeline.cube_detection import _support_metadata
from calibration_pipeline.observations import load_cube_pixel_observations
from calibration_pipeline.runtime import load_intrinsics_with_depth_scale


ROOT = Path(__file__).resolve().parents[1]
SESSION_ROOT = ROOT / "data" / "session04" / "calib_train"
INTRINSICS_ROOT = ROOT / "intrinsics"


def _target():
    cfg, _ = load_cube_config_from_meta(str(SESSION_ROOT))
    return AprilTagCubeTarget(cfg)


def test_top_tag_pair_is_labelled_planar_not_two_face():
    target = _target()
    points = np.concatenate([
        target.model.marker_corners_in_rig(0),
        target.model.marker_corners_in_rig(1),
    ])
    support = _support_metadata(target, [0, 1], points)
    assert support == {
        "marker_ids": (0, 1),
        "observed_faces": ("+Z",),
        "observed_face_count": 1,
        "noncoplanar_face_count": 0,
        "is_planar": True,
        "quality_tier": "planar_multimarker",
    }


def test_two_differently_oriented_faces_are_core_nonplanar():
    target = _target()
    points = np.concatenate([
        target.model.marker_corners_in_rig(0),
        target.model.marker_corners_in_rig(2),
    ])
    support = _support_metadata(target, [0, 2], points)
    assert support["observed_faces"] == ("+X", "+Z")
    assert support["observed_face_count"] == 2
    assert support["noncoplanar_face_count"] == 2
    assert support["is_planar"] is False
    assert support["quality_tier"] == "nonplanar_multiface"


def test_support_marker_ids_preserve_corner_block_order():
    target = _target()
    points = np.concatenate([
        target.model.marker_corners_in_rig(2),
        target.model.marker_corners_in_rig(0),
    ])
    support = _support_metadata(target, [2, 0], points)
    assert support["marker_ids"] == (2, 0)


def test_session04_core_policy_excludes_single_face_observations():
    with (SESSION_ROOT / "meta.json").open("r", encoding="utf-8") as stream:
        meta = json.load(stream)
    target = _target()
    K_map, D_map = {}, {}
    for camera in meta["cam_indices"]:
        K_map[camera], D_map[camera], _ = load_intrinsics_with_depth_scale(
            str(INTRINSICS_ROOT), camera)

    observations, diagnostics = load_cube_pixel_observations(
        str(SESSION_ROOT), meta, target, K_map, D_map,
        meta["cam_indices"], meta["gripper_cam_idx"],
        exclude_gripped=True,
        observation_policy="core_multiface",
    )

    assert observations
    comparison = diagnostics["available_observation_policy_comparison"]
    assert comparison["single_marker"] > 0
    assert comparison["core_multiface"] < comparison["legacy"]
    assert diagnostics["selected_quality_tier_counts"] == {
        "nonplanar_multiface": len(observations)
    }
    selected_records = [
        record for record in diagnostics["observation_quality_by_event_camera"]
        if record["selected_for_calibration"]
    ]
    assert len(selected_records) == len(observations)
    assert all(record["observed_face_count"] >= 2 for record in selected_records)
    assert all(record["noncoplanar_face_count"] >= 2 for record in selected_records)
    assert all(record["is_planar"] is False for record in selected_records)
    assert all(record["positive_depth_candidate_count"] >= 1
               for record in selected_records)

    recovered = [
        record for record in selected_records
        if record.get("recovered_core_observation")
    ]
    assert {(record["event_id"], record["camera_id"])
            for record in recovered} == {
                (6, 3), (9, 2), (12, 3), (22, 2), (25, 2), (29, 2),
                (42, 1), (47, 2), (48, 3), (63, 2), (77, 2),
            }
    assert all(record["pnp_rmse_px"] <= 3.0 for record in recovered)
    assert all(record["detection_method"] != "default"
               for record in recovered)

    edge_cases_by_event = {
        record["event_id"]: record
        for record in diagnostics["observation_quality_by_event_camera"]
        if record["event_id"] in (11, 15)
    }
    assert set(edge_cases_by_event) == {11, 15}
    assert edge_cases_by_event[11]["pnp_accepted"] is True
    assert edge_cases_by_event[11]["pnp_rmse_px"] > 3.0
    assert edge_cases_by_event[15]["pnp_accepted"] is False
