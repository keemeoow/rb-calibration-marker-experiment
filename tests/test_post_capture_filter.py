"""Frozen post-capture observation manifest contract tests."""

import copy
import json
from pathlib import Path

import numpy as np

from calibration_pipeline.observations import (
    POST_CAPTURE_MANIFEST_SCHEMA,
    load_pixel_observations_from_manifest,
)
from calibration_pipeline.board_config import (
    charuco_config_from_dict,
    charuco_topology,
)
from calibration_pipeline import table1
from calibration_pipeline.path_evaluation import (
    _event_then_set_aggregate,
    build_frozen_path_evaluation_mask,
    build_gripper_to_fixed_cross_target_mask,
)
from calibration_pipeline.runtime import load_intrinsics_with_depth_scale


ROOT = Path(__file__).resolve().parents[1]
SESSION_ROOT = ROOT / "data" / "session04" / "calib_train"
INTRINSICS_ROOT = ROOT / "intrinsics"
MANIFEST = (
    ROOT / "data" / "session04" / "calib_out" / "capture_filter"
    / "Step2b_observation_manifest.json"
)


def test_session04_standard_frozen_population_and_source_hashes():
    observations, diagnostics = load_pixel_observations_from_manifest(
        str(MANIFEST),
        policy="standard",
        root=str(SESSION_ROOT),
        intrinsics_dir=str(INTRINSICS_ROOT),
        validate_sources=True,
        # The manifest records the absolute paths of the machine that captured
        # it, so the suite only runs in that checkout without this.  Every
        # recorded SHA-256 is still verified below.
        allow_relocated_root=True,
    )

    assert diagnostics["manifest_schema"] == POST_CAPTURE_MANIFEST_SCHEMA
    assert diagnostics["source"] == "post_capture_frozen_manifest"
    assert diagnostics["source_hashes_validated"] is True
    board_cfg = charuco_config_from_dict(
        diagnostics["charuco_board_config"])
    topology = charuco_topology(board_cfg)
    assert topology["checker_squares_x"] == 11
    assert topology["checker_squares_y"] == 7
    assert topology["charuco_corner_columns"] == 10
    assert topology["charuco_corner_rows"] == 6
    assert topology["maximum_charuco_corners"] == 60
    assert np.isclose(topology["checker_width_mm"], 275.0)
    assert np.isclose(topology["checker_height_mm"], 175.0)
    assert diagnostics["n_cube_observations"] == 99
    assert diagnostics["n_board_observations"] == 147
    assert len(observations) == 246


def _write_manifest(tmp_path, payload):
    path = tmp_path / "manifest.json"
    path.write_text(json.dumps(payload), encoding="utf-8")
    return path


def test_manifest_rejects_board_config_or_topology_drift(tmp_path):
    payload = json.loads(MANIFEST.read_text(encoding="utf-8"))

    changed_config = copy.deepcopy(payload)
    changed_config["source"]["charuco_board_config"][
        "square_length_m"] = 0.0275
    path = _write_manifest(tmp_path, changed_config)
    with np.testing.assert_raises_regex(ValueError, "config SHA-256 mismatch"):
        load_pixel_observations_from_manifest(
            str(path), policy="standard", validate_sources=False)

    changed_corner = copy.deepcopy(payload)
    board_record = next(
        record for record in changed_corner["observations"]
        if record["target"] == "board"
        and record["selected_by_policy"]["standard"])
    board_record["object_points"][0][0] += 0.0025
    path = _write_manifest(tmp_path, changed_corner)
    with np.testing.assert_raises_regex(ValueError, "ChArUco topology"):
        load_pixel_observations_from_manifest(
            str(path), policy="standard", validate_sources=False)


def test_manifest_rejects_cube_marker_block_reordering(tmp_path):
    payload = json.loads(MANIFEST.read_text(encoding="utf-8"))
    cube_record = next(
        record for record in payload["observations"]
        if record["target"] == "cube"
        and len(record.get("marker_ids", [])) >= 2
        and record["selected_by_policy"]["standard"])
    cube_record["marker_ids"] = list(reversed(cube_record["marker_ids"]))
    path = _write_manifest(tmp_path, payload)
    with np.testing.assert_raises_regex(ValueError, "marker order"):
        load_pixel_observations_from_manifest(
            str(path), policy="standard", validate_sources=False)


def test_session04_strict_population_is_a_standard_subset():
    standard, _ = load_pixel_observations_from_manifest(
        str(MANIFEST), policy="standard", validate_sources=False)
    strict, diagnostics = load_pixel_observations_from_manifest(
        str(MANIFEST), policy="strict", validate_sources=False)

    identity = lambda observation: (
        observation.marker, observation.event, observation.cam)
    assert {identity(observation) for observation in strict} <= {
        identity(observation) for observation in standard}
    assert diagnostics["n_cube_observations"] == 97
    assert diagnostics["n_board_observations"] == 146
    assert len(strict) == 243


def test_board_metric_scale_is_opt_in_diagnostic_only():
    assert table1.parse_args([]).align_board_metric_scale is False
    observations, _ = load_pixel_observations_from_manifest(
        str(MANIFEST), policy="standard", validate_sources=False)
    split = table1.build_event_split(
        observations, gripper=2, fraction=0.2,
        seed=20260731, min_train_eih_cube_events=3)
    train_events = set(split["train_events"])
    eligible = set(split["eligible_sets"])
    train = [observation for observation in observations
             if observation.set_idx in eligible
             and observation.event in train_events]
    K_map, D_map = {}, {}
    for camera in (0, 1, 2, 3):
        K_map[camera], D_map[camera], _ = load_intrinsics_with_depth_scale(
            str(INTRINSICS_ROOT), camera)

    result = table1.estimate_train_board_metric_scale(
        train, gripper_cam_idx=2, K_map=K_map, D_map=D_map)

    assert result["heldout_observations_used"] is False
    assert result["robot_fk_used"] is False
    assert 1.005 < result["scale"] < 1.015
    assert 25.1 < result["effective_square_length_mm"] < 25.4
    for camera in ("1", "3"):
        detail = result["per_camera"][camera]
        assert (detail["translation_disagreement_after_mm"]
                < detail["translation_disagreement_before_mm"])


def test_session04_set_first_fixed_anchor_covers_every_heldout_gripper_event():
    observations, _ = load_pixel_observations_from_manifest(
        str(MANIFEST), policy="standard", validate_sources=False)
    split = table1.build_event_split(
        observations, gripper=2, fraction=0.2,
        seed=20260731, min_train_eih_cube_events=3)
    eligible = set(split["eligible_sets"])
    train_events = set(split["train_events"])
    heldout_events = set(split["test_events"])
    pool = [observation for observation in observations
            if observation.set_idx in eligible]
    heldout = [observation for observation in pool
               if observation.event in heldout_events]
    K_map, D_map = {}, {}
    for camera in (0, 1, 2, 3):
        K_map[camera], D_map[camera], _ = load_intrinsics_with_depth_scale(
            str(INTRINSICS_ROOT), camera)
    roles = {
        **{event: "train" for event in train_events},
        **{event: "heldout" for event in heldout_events},
    }

    mask = build_gripper_to_fixed_cross_target_mask(
        heldout, (0, 1, 3), 2, K_map, D_map,
        set_filter=sorted(eligible), fixed_anchor_observations=pool,
        event_roles=roles)
    for target, expected_pairs in (("board", 23), ("cube", 27)):
        support = mask["support_by_target"][target]
        assert support["events"] == sorted(heldout_events)
        assert support["sets"] == sorted(eligible)
        assert support["gripper_observations"] == 9
        assert support["pairs"] == expected_pairs
    assert {pair["gripper_split_role"] for pair in mask["pairs"]} == {
        "heldout"}

    legacy = build_frozen_path_evaluation_mask(
        heldout, (0, 1, 3), 2, K_map, D_map, sorted(eligible),
        fixed_anchor_observations=pool, event_roles=roles)
    assert len(legacy["e2e_units"]) == 9
    assert {unit["event"] for unit in legacy["e2e_units"]} == heldout_events
    assert all(unit["fixed_anchor_events"] for unit in legacy["e2e_units"])


def test_set_hierarchical_aggregation_does_not_overweight_reused_anchor():
    event_rows = [
        {"set": 0, "event": 1, "error": 1.0},
        {"set": 0, "event": 2, "error": 3.0},
        {"set": 1, "event": 3, "error": 10.0},
    ]
    overall, per_set = _event_then_set_aggregate(event_rows, ("error",))
    assert np.isclose(per_set[0]["error"], np.sqrt(5.0))
    assert np.isclose(overall["error"], np.sqrt((5.0 + 100.0) / 2.0))
