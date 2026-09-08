from __future__ import annotations

import copy
import json
from pathlib import Path

import pytest

from capture_pipeline.waypoint_safety import (
    PROTOCOL_COMPOSITE_RIG_45,
    validate_waypoint_semantics,
)
from capture_pipeline.capture import (
    evaluate_transport_integrity,
    load_and_validate_rig_geometry,
)
from tools.create_capture_pose_plan import build_template


def _valid_plan() -> dict:
    plan = build_template()
    plan["template_only"] = False
    plan["target_rig_id"] = "composite_rig_001"
    plan["rig_geometry_file"] = "composite_rig_geometry.json"
    plan["rig_geometry_sha256"] = "a" * 64
    plan["safe_joints_empty"] = [0, 1, 2, 3, 4, 5]
    plan["safe_joints_gripped"] = [10, 11, 12, 13, 14, 15]
    for idx, placement in enumerate(plan["placements"]):
        placement["place_approach_joints"] = [idx + axis for axis in range(6)]
        placement["place_approach_tcp"] = [100, 200, 300, 0, 0, 0]
        placement["place_tcp"] = [100, 200, 250, 0, 0, 0]
    for idx, waypoint in enumerate(plan["waypoints"]):
        waypoint["capture_joints"] = [idx + axis for axis in range(6)]
    return plan


def test_composite_protocol_accepts_exact_15_20_10_plan() -> None:
    plan = _valid_plan()

    assert plan["capture_protocol"] == PROTOCOL_COMPOSITE_RIG_45
    assert validate_waypoint_semantics(plan) is True
    assert sum(wp["phase"] == "P1_MOVING_RIG" for wp in plan["waypoints"]) == 15
    assert sum(wp["phase"] == "P2_PICK_PLACE" for wp in plan["waypoints"]) == 20
    assert sum(wp["phase"] == "P3_STATIONARY_RIG" for wp in plan["waypoints"]) == 10


def test_composite_protocol_rejects_missing_event() -> None:
    plan = _valid_plan()
    plan["waypoints"].pop()

    with pytest.raises(ValueError, match="exactly 45"):
        validate_waypoint_semantics(plan)


def test_composite_protocol_rejects_phase_or_target_state_relabeling() -> None:
    plan = _valid_plan()
    plan["waypoints"][15]["target_state"] = "gripped"

    with pytest.raises(ValueError, match="target_state"):
        validate_waypoint_semantics(plan)


def test_composite_protocol_rejects_untaught_pose() -> None:
    plan = _valid_plan()
    plan["waypoints"][0]["capture_joints"] = None

    with pytest.raises(ValueError, match="capture_joints"):
        validate_waypoint_semantics(plan)


def test_composite_protocol_rejects_z_lift_only() -> None:
    plan = copy.deepcopy(_valid_plan())
    plan["safe_pose_mode"] = "z_lift_only"

    with pytest.raises(ValueError, match="z_lift_only is not allowed"):
        validate_waypoint_semantics(plan)


def test_composite_protocol_rejects_nonvertical_place_approach() -> None:
    plan = _valid_plan()
    plan["placements"][0]["place_approach_tcp"][0] += 10

    with pytest.raises(ValueError, match="x,y must match"):
        validate_waypoint_semantics(plan)


def test_transport_integrity_does_not_depend_on_marker_detection() -> None:
    frames = {
        0: {"ts_ms": 1000.0, "ok": False, "n_markers": 0},
        1: {"ts_ms": 1040.0, "ok": False, "n_markers": 0},
    }

    result = evaluate_transport_integrity(frames, [0, 1], 120.0)

    assert result["pass"] is True


def test_transport_integrity_rejects_missing_camera() -> None:
    result = evaluate_transport_integrity({0: {"ts_ms": 1000.0}}, [0, 1], 120.0)

    assert result["pass"] is False
    assert result["missing_frame_camera_ids"] == [1]


def test_rig_geometry_requires_two_valid_se3_matrices(tmp_path: Path) -> None:
    geometry = {
        "schema_version": "composite_rig_geometry_v1",
        "template_only": False,
        "target_rig_id": "composite_rig_001",
        "translation_unit": "meter",
        "T_rig_board": [
            [1, 0, 0, 0.0],
            [0, 1, 0, 0.0],
            [0, 0, 1, 0.0],
            [0, 0, 0, 1.0],
        ],
        "T_rig_cube": [
            [1, 0, 0, 0.1],
            [0, 1, 0, 0.0],
            [0, 0, 1, 0.0],
            [0, 0, 0, 1.0],
        ],
    }
    path = tmp_path / "rig.json"
    path.write_text(json.dumps(geometry), encoding="utf-8")

    assert load_and_validate_rig_geometry(path, "composite_rig_001") == geometry
