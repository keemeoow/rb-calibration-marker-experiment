from __future__ import annotations

import copy
import json
from pathlib import Path
import sys

import pytest

from capture_pipeline.waypoint_safety import (
    PROTOCOL_COMPOSITE_RIG_45,
    validate_waypoint_semantics,
)
from capture_pipeline.capture import (
    evaluate_transport_integrity,
    load_and_validate_rig_geometry,
    main as capture_main,
    resolve_capture_roots,
)
from tools.create_capture_pose_plan import build_template
from zeus_gello_calibration.paths import ZEUS_DATA_ROOT


def _valid_plan() -> dict:
    plan = build_template()
    plan["template_only"] = False
    plan["target_rig_id"] = "composite_rig_001"
    plan["rig_geometry_file"] = "composite_rig_geometry.json"
    plan["rig_geometry_sha256"] = "a" * 64
    plan["safe_joints_empty"] = [0, 1, 2, 3, 4, 5]
    plan["safe_joints_gripped"] = [10, 11, 12, 13, 14, 15]
    for idx, placement in enumerate(plan["placements"]):
        x = 100 + (idx % 5) * 30
        y = 200 + (idx // 5) * 120
        yaw = idx * 5
        placement["place_approach_joints"] = [100 + idx * 2 + axis for axis in range(6)]
        placement["place_approach_tcp"] = [x, y, 300, yaw, 0, 0]
        placement["place_tcp"] = [x, y, 250, yaw, 0, 0]
    for idx, waypoint in enumerate(plan["waypoints"]):
        waypoint["capture_joints"] = [idx * 2 + axis for axis in range(6)]
        if idx < 15:
            waypoint["capture_flange_pose_6dof_mm_deg"] = [
                (idx % 5) * 30,
                (idx // 5) * 60,
                400 + (idx % 3) * 40,
                idx * 4,
                -15 + (idx % 3) * 15,
                -15 + (idx // 5) * 15,
            ]
        elif idx < 35:
            local = idx - 15
            placement_idx, view_idx = divmod(local, 2)
            waypoint["capture_flange_pose_6dof_mm_deg"] = [
                100 + (placement_idx % 5) * 30 + view_idx * 40,
                200 + (placement_idx // 5) * 120,
                350 + view_idx * 10,
                placement_idx * 5,
                -10 + view_idx * 20,
                -5 + view_idx * 10,
            ]
        else:
            local = idx - 35
            waypoint["capture_flange_pose_6dof_mm_deg"] = [
                (local % 5) * 25,
                (local // 5) * 100,
                400 + (local % 2) * 60,
                local * 5,
                -10 + (local % 2) * 20,
                -10 + (local // 5) * 20,
            ]
    return plan


def test_composite_protocol_accepts_exact_15_20_10_plan() -> None:
    plan = _valid_plan()

    assert plan["capture_protocol"] == PROTOCOL_COMPOSITE_RIG_45
    assert validate_waypoint_semantics(plan) is True
    assert sum(wp["phase"] == "P1_MOVING_RIG" for wp in plan["waypoints"]) == 15
    assert sum(wp["phase"] == "P2_PICK_PLACE" for wp in plan["waypoints"]) == 20
    assert sum(wp["phase"] == "P3_STATIONARY_RIG" for wp in plan["waypoints"]) == 10
    assert plan["placements"][7]["placement_id"] == "PLACEMENT_07"
    assert plan["waypoints"][29]["planned_event_id"] == "P2_PLACEMENT_07_VIEW_0"


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


def test_composite_protocol_rejects_missing_flange_pose() -> None:
    plan = _valid_plan()
    plan["waypoints"][0]["capture_flange_pose_6dof_mm_deg"] = None

    with pytest.raises(ValueError, match="capture_flange_pose_6dof_mm_deg"):
        validate_waypoint_semantics(plan)


def test_composite_protocol_rejects_duplicate_joint_pose() -> None:
    plan = _valid_plan()
    plan["waypoints"][1]["capture_joints"] = list(
        plan["waypoints"][0]["capture_joints"])

    with pytest.raises(ValueError, match="repeat the same taught joint pose"):
        validate_waypoint_semantics(plan)


def test_composite_protocol_rejects_insufficient_p1_xyz_coverage() -> None:
    plan = _valid_plan()
    for waypoint in plan["waypoints"][:15]:
        waypoint["capture_flange_pose_6dof_mm_deg"][2] = 400

    with pytest.raises(ValueError, match="p1_translation_span_xyz_mm"):
        validate_waypoint_semantics(plan)


def test_composite_protocol_rejects_near_identical_p2_view_pair() -> None:
    plan = _valid_plan()
    plan["waypoints"][16]["capture_flange_pose_6dof_mm_deg"] = list(
        plan["waypoints"][15]["capture_flange_pose_6dof_mm_deg"])

    with pytest.raises(ValueError, match="p2_min_view_translation_mm"):
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


def test_final_capture_cli_rejects_missing_pose_plan_before_camera_import(
    monkeypatch: pytest.MonkeyPatch,
    capsys: pytest.CaptureFixture[str],
) -> None:
    monkeypatch.setattr(
        sys,
        "argv",
        [
            "03_capture.py",
            "--data_root",
            str(ZEUS_DATA_ROOT),
            "--intrinsics_dir",
            "intrinsics",
        ],
    )

    with pytest.raises(SystemExit) as error:
        capture_main()

    assert error.value.code == 2
    assert "--waypoints_file" in capsys.readouterr().err


def test_final_capture_cli_rejects_missing_robot_mode_before_camera_import(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    capsys: pytest.CaptureFixture[str],
) -> None:
    plan_path = tmp_path / "capture_waypoints.json"
    plan_path.write_text(json.dumps(_valid_plan()), encoding="utf-8")
    monkeypatch.setattr(
        sys,
        "argv",
        [
            "03_capture.py",
            "--data_root",
            str(ZEUS_DATA_ROOT),
            "--intrinsics_dir",
            "intrinsics",
            "--waypoints_file",
            str(plan_path),
        ],
    )

    with pytest.raises(SystemExit) as error:
        capture_main()

    assert error.value.code == 2
    assert "requires --use_robot --manual_robot" in capsys.readouterr().err


def test_final_capture_cli_requires_session_label_before_camera_import(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    capsys: pytest.CaptureFixture[str],
) -> None:
    plan_path = tmp_path / "capture_waypoints.json"
    plan_path.write_text(json.dumps(_valid_plan()), encoding="utf-8")
    monkeypatch.setattr(
        sys,
        "argv",
        [
            "03_capture.py",
            "--data_root",
            str(ZEUS_DATA_ROOT),
            "--intrinsics_dir",
            "intrinsics",
            "--waypoints_file",
            str(plan_path),
            "--use_robot",
            "--manual_robot",
        ],
    )

    with pytest.raises(SystemExit) as error:
        capture_main()

    assert error.value.code == 2
    assert "requires --session_label" in capsys.readouterr().err


def test_final_capture_cli_rejects_noncanonical_data_root_before_camera_import(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    capsys: pytest.CaptureFixture[str],
) -> None:
    plan_path = tmp_path / "capture_waypoints.json"
    plan_path.write_text(json.dumps(_valid_plan()), encoding="utf-8")
    monkeypatch.setattr(
        sys,
        "argv",
        [
            "03_capture.py",
            "--data_root",
            str(Path(__file__).resolve().parents[1] / "data"),
            "--session_label",
            "wrong_root",
            "--intrinsics_dir",
            "intrinsics",
            "--waypoints_file",
            str(plan_path),
            "--use_robot",
            "--manual_robot",
        ],
    )

    with pytest.raises(SystemExit) as error:
        capture_main()

    assert error.value.code == 2
    assert "must be inside" in capsys.readouterr().err


def test_explicit_zeus_data_root_option_is_authoritative() -> None:
    root = Path(__file__).resolve().parents[1]
    requested = root / "zeus_gello_calibration" / "data"

    resolved_data, resolved_capture = resolve_capture_roots(requested)

    assert Path(resolved_data) == requested
    assert resolved_capture is None


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
