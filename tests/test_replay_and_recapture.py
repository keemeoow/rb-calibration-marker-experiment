from __future__ import annotations

import argparse
import json

import pytest

from calibration_pipeline.table1 import (
    build_composite_placement_split,
    selected_composite_protocol_captures,
)
from capture_pipeline.waypoint_safety import PROTOCOL_SAVED_POSE_REPLAY
from zeus_gello_calibration.replay_and_recapture import (
    P2_CAMERA_JOINTS,
    P2_GRASP_REF_POSE,
    build_combined_event_plan,
    directory_has_files,
    load_saved_joints,
    require_subdir_name,
)


def _write_robot_json(root, index: str, joints, pose=None) -> None:
    capture_dir = root / index
    capture_dir.mkdir(parents=True)
    payload = {"joints": joints, "pose": pose or [1, 2, 3, 4, 5, 6]}
    (capture_dir / "robot.json").write_text(json.dumps(payload))


def test_load_saved_joints_uses_numeric_capture_order(tmp_path) -> None:
    capture_root = tmp_path / "capture"
    _write_robot_json(capture_root, "010", [10, 11, 12, 13, 14, 15])
    _write_robot_json(capture_root, "002", [2, 3, 4, 5, 6, 7])
    (capture_root / "notes").mkdir()

    items = load_saved_joints(capture_root)

    assert [item["index"] for item in items] == [2, 10]


def test_load_saved_joints_rejects_invalid_six_axis_value(tmp_path) -> None:
    capture_root = tmp_path / "capture"
    _write_robot_json(capture_root, "000", [1, 2, 3])

    with pytest.raises(ValueError, match="숫자 6개"):
        load_saved_joints(capture_root)


@pytest.mark.parametrize("value", ["", ".", "..", "../outside", "nested/output"])
def test_replay_output_must_be_one_subdirectory(value) -> None:
    with pytest.raises(argparse.ArgumentTypeError):
        require_subdir_name(value)

    assert require_subdir_name("capture_replayed_auto") == "capture_replayed_auto"


def test_directory_has_files_distinguishes_empty_output(tmp_path) -> None:
    output = tmp_path / "capture_replayed_auto"
    output.mkdir()
    assert not directory_has_files(output)

    (output / "000").mkdir()
    assert directory_has_files(output)


def test_combined_plan_preserves_every_saved_pose_and_uses_one_p2_view() -> None:
    def state(index, offset):
        return {
            "index": index,
            "joints": [offset + axis for axis in range(6)],
            "pose": [offset + axis + 10 for axis in range(6)],
            "src_dir": f"source/{index:03d}",
        }

    sources = {
        1: {"states": [state(0, 0), state(1, 10)]},
        2: {"states": [state(0, 20), state(1, 30), state(2, 40)]},
        3: {"states": [state(0, 50)]},
    }

    events = build_combined_event_plan(sources)

    assert len(events) == 6
    assert [event["capture_index"] for event in events] == list(range(6))
    assert [event["phase"] for event in events] == [
        "P1_MOVING_RIG", "P1_MOVING_RIG",
        "P2_PICK_PLACE", "P2_PICK_PLACE", "P2_PICK_PLACE",
        "P3_STATIONARY_RIG",
    ]
    p2_events = [event for event in events if event["phase"] == "P2_PICK_PLACE"]
    assert [event["source_pose"] for event in p2_events] == [
        sources[2]["states"][index]["pose"] for index in range(3)
    ]
    for event in p2_events:
        assert event["placement_pose"][:2] == event["source_pose"][:2]
        assert event["placement_pose"][3] == event["source_pose"][3]
        assert event["placement_pose"][2] == P2_GRASP_REF_POSE[2]
        assert event["placement_pose"][4:] == P2_GRASP_REF_POSE[4:]
    assert all(event["capture_joints"] == P2_CAMERA_JOINTS for event in p2_events)
    assert all(event["view_index"] == 0 for event in p2_events)


def test_saved_pose_replay_uses_dynamic_count_and_placement_split() -> None:
    captures = []
    event_id = 0
    for phase, count in (
        ("P1_MOVING_RIG", 2),
        ("P2_PICK_PLACE", 3),
        ("P3_STATIONARY_RIG", 1),
    ):
        for phase_index in range(count):
            is_p2 = phase == "P2_PICK_PLACE"
            captures.append({
                "event_id": event_id,
                "protocol_version": PROTOCOL_SAVED_POSE_REPLAY,
                "selected_for_analysis": True,
                "planned_event_id": f"{phase}:{phase_index}",
                "phase": phase,
                "placement_id": f"PLACEMENT_{phase_index:03d}" if is_p2 else None,
                "view_index": 0 if is_p2 else phase_index,
                "set_index": phase_index if is_p2 else 3 if phase.startswith("P3") else None,
            })
            event_id += 1
    meta = {
        "capture_config": {
            "capture_protocol": PROTOCOL_SAVED_POSE_REPLAY,
            "expected_event_count": 6,
            "expected_phase_counts": {
                "P1_MOVING_RIG": 2,
                "P2_PICK_PLACE": 3,
                "P3_STATIONARY_RIG": 1,
            },
            "p2_views_per_placement": 1,
        },
        "captures": captures,
    }

    assert len(selected_composite_protocol_captures(meta)) == 6
    split = build_composite_placement_split(meta, fraction=1 / 3, seed=7)

    assert len(split["train_placement_ids"]) == 2
    assert len(split["heldout_placement_ids"]) == 1
    assert set(split["train_events"]).isdisjoint(split["test_events"])
    assert len(split["always_train_events"]) == 3
