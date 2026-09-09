from __future__ import annotations

import math
from pathlib import Path

import numpy as np
import pytest

from calibration_pipeline.schema import (
    EVALUATION_COMPARISON_CONTRACT,
    MAIN_ABLATION_CONDITIONS,
)
from tools.sync_table1_canonical_data import (
    _combined_cross_view_run,
    _data_warnings,
    _load,
    _markdown,
    _method_rows,
)


ROOT = Path(__file__).resolve().parents[1]
TABLE1_JSON = ROOT / "CP_result/session04/calib_result_table1/table1_methods.json"
CROSS_JSON = (
    ROOT / "CP_result/session04/cross_target_evaluation/"
    "cross_target_evaluation.json")
MARKER_JSON = (
    ROOT / "CP_result/session04/marker_system_end_to_end/"
    "marker_system_end_to_end.json")


def test_validation_requested_comparison_contracts_are_registered():
    contracts = EVALUATION_COMPARISON_CONTRACT

    assert contracts["A0_to_B3"]["rows"] == ("A0", "B3")
    assert contracts["A0_to_B3"]["components"] == (
        "external_gt.cube", "heldout_reprojection.cube")
    assert contracts["A2_to_A4"]["rows"] == ("A2", "A4")
    assert contracts["A2_to_A4"]["components"] == (
        "external_gt.cube",
        "heldout_reprojection.cube",
    )


def test_a3_a4_labels_are_neutral_before_external_gt():
    labels = {condition.row: condition.label
              for condition in MAIN_ABLATION_CONDITIONS}

    assert labels["A3"] == "raw-FK hard fixed"
    assert labels["A4"] == "corrected-FK soft factor"


def test_table1_markdown_groups_tiers_and_surfaces_data_warnings():
    table1 = _load(TABLE1_JSON)
    cross = _load(CROSS_JSON)
    marker = _load(MARKER_JSON)
    rows = _method_rows(table1, cross)

    markdown = _markdown(
        rows, marker, detailed=True,
        data_warnings=_data_warnings(table1, cross),
        session_label="Session04")

    assert "## Current Data Warnings" in markdown
    assert "## Final Protocol Lock" in markdown
    assert "dropped sets `0, 1, 2, 3`" in markdown
    assert "내부 preflight 결과" in markdown
    assert "composite rig 45-event" in markdown
    assert "legacy Session04 내부 데이터" in markdown
    assert "## Final Comparison Table" in markdown
    assert "### 평가지표 (한글로): 설명, 평가 지표 낸 방법" in markdown
    assert "train cube 관측으로 set별 evaluation pose" in markdown
    assert "## Matched Contrast Decision Table" in markdown
    assert "A0 -> B3" in markdown
    assert "A2 -> A4" in markdown
    assert "## Metric Decision Matrix" in markdown
    assert "External cube TRE / rotation / P95 / failure" in markdown
    assert "Train Cube RMSE px" in markdown
    assert "| Train RMSE px |" not in markdown
    assert "Heldout Cube RMSE" in markdown
    assert "## Cross-view Camera Consistency" in markdown
    assert "External GT Task" in markdown
    assert "Internal-Only Stopping Point" not in markdown
    assert "Required Next Experiment" not in markdown
    assert "A3 (raw-FK hard fixed)" in markdown
    assert "A4 (corrected-FK soft factor)" in markdown
    assert "A5 (vision-aligned FK hard fixed)" in markdown
    assert "Ours (raw-FK-fixed target)" not in markdown
    assert "Ours (corrected-FK factor)" not in markdown
    assert "Set-equal-weight Held-out RMSE" not in markdown
    assert "Board/Cube Held-out" not in markdown


def test_board_only_rows_report_cube_evaluation_without_heldout_leakage():
    table1 = _load(TABLE1_JSON)
    cross = _load(CROSS_JSON)
    rows = {row["method"]: row for row in _method_rows(table1, cross)}

    for method in ("A0", "B3"):
        run = table1["rows"][method]["runs"][0]
        cube_eval = run["cube_evaluation_reprojection"]
        assert cube_eval["camera_and_hand_eye_frozen"] is True
        assert cube_eval["heldout_cube_observations_used_for_pose_fit"] is False
        assert cube_eval["heldout"]["cube"]["n_corners"] == 236
        assert run["heldout_reprojection"].get("cube") is None
        assert rows[method]["all_cube_reprojection_rmse_px"] is not None
        assert rows[method]["heldout_cube_reprojection_rmse_px"] is not None


def test_final_train_metric_uses_one_cube_population_for_every_row():
    table1 = _load(TABLE1_JSON)
    cross = _load(CROSS_JSON)
    rows = _method_rows(table1, cross)

    assert {row["train_cube_n_corners"] for row in rows} == {724}
    for row in rows:
        run = table1["rows"][row["method"]]["runs"][0]
        expected = run["cube_evaluation_reprojection"]["train"]["cube"]
        assert expected["n_corners"] == 724
        assert row["train_cube_reprojection_rmse_px"] is not None


def test_combined_cross_view_pools_raw_pair_directions_and_corners():
    cross = _load(CROSS_JSON)
    supports = set()
    for method, runs in cross["per_run"].items():
        for run in runs:
            combined = _combined_cross_view_run(run)
            supports.add((
                combined["n_pairs"], combined["n_directions"],
                combined["n_destination_corners"],
                combined["n_fixed_to_fixed_pairs"],
                combined["n_gripper_to_fixed_pairs"],
                combined["n_train_fixed_anchor_pairs"],
            ))
    assert supports == {(36, 72, 904, 9, 27, 18)}
    a0 = _combined_cross_view_run(cross["per_run"]["A0"][0])
    assert a0["cross_view_pixel_transfer_rmse_px"] == pytest.approx(
        7.152744, abs=1e-6)


def test_set_equal_weight_gives_every_set_one_vote():
    """A dense placement must not outvote a sparse one."""
    from calibration_pipeline.evaluation import set_equal_weight_rmse

    # set 0: 100 corners at 1 px; set 1: a single corner at 9 px.
    aggregate = set_equal_weight_rmse({
        (0, 10): [1.0] * 200,
        (1, 20): [81.0] * 2,
    })

    pooled = math.sqrt(np.mean([1.0] * 200 + [81.0] * 2))
    assert pooled == pytest.approx(1.3387, abs=1e-4)
    assert aggregate["rmse_px"] == pytest.approx(math.sqrt((1.0 + 81.0) / 2))
    assert [row["n_corners"] for row in aggregate["per_set"]] == [100, 1]


def test_set_equal_weight_collapses_events_before_sets():
    from calibration_pipeline.evaluation import set_equal_weight_rmse

    aggregate = set_equal_weight_rmse({
        (0, 1): [1.0] * 200,
        (0, 2): [81.0] * 2,
        (1, 3): [4.0] * 2,
    })

    set_zero = aggregate["per_set"][0]
    assert set_zero["n_events"] == 2
    assert set_zero["rmse_px"] == pytest.approx(math.sqrt((1.0 + 81.0) / 2))
    assert aggregate["rmse_px"] == pytest.approx(math.sqrt((41.0 + 4.0) / 2))


def test_heldout_artifact_reports_both_poolings_on_every_target():
    table1 = _load(TABLE1_JSON)
    eligible = table1["protocol"]["split"]["eligible_sets"]

    for method, row in table1["rows"].items():
        heldout = row["runs"][0]["heldout_reprojection"]
        for target in ("overall", "board", "cube"):
            entry = heldout.get(target)
            if entry is None:
                continue
            assert entry["set_equal_weight_rmse_px"] is not None, (
                f"{method}/{target} lost the equal-weight aggregate")
            per_set = entry["set_equal_weight_per_set"]
            assert [item["set"] for item in per_set] == eligible
            # One held-out event per set in this protocol.
            assert all(item["n_events"] == 1 for item in per_set)


def test_pooled_and_equal_weight_agree_only_through_support():
    """The two poolings differ, so the report must never present one as both."""
    table1 = _load(TABLE1_JSON)
    cube = table1["rows"]["A2"]["runs"][0]["heldout_reprojection"]["cube"]

    assert cube["rmse_px"] < cube["set_equal_weight_rmse_px"]
    assert cube["n_corners"] == 236


def test_report_uses_cube_only_final_metric_set():
    markdown = (
        ROOT / "CP_result/session04/calib_result_table1/TABLE1_RESULTS.md").read_text()

    assert "Heldout Cube RMSE" in markdown
    assert "ALL Cube RMSE" in markdown
    assert "Train Cube RMSE" in markdown
    assert "External cube TRE / rotation / P95 / failure" in markdown
    assert "Board/Cube Held-out" not in markdown
    assert "Set-equal-weight Held-out RMSE" not in markdown


def test_relocated_manifest_keeps_every_hash_check():
    from calibration_pipeline.observations import _relocation

    relocate, recorded_prefix, local_prefix = _relocation(
        "/Users/woo/Documents/GitHub/Robot-Lab/repo/data/session04/calib_train",
        "/home/jysim/checkout/repo/data/session04/calib_train")

    assert recorded_prefix == "/Users/woo/Documents/GitHub/Robot-Lab"
    assert local_prefix == "/home/jysim/checkout"
    # Intrinsics live outside the session root and must relocate too.
    assert relocate("/Users/woo/Documents/GitHub/Robot-Lab/repo/intrinsics/cam0.npz") == (
        "/home/jysim/checkout/repo/intrinsics/cam0.npz")
    # An unrelated absolute path is left alone rather than silently rewritten.
    assert relocate("/opt/data/other.json") == "/opt/data/other.json"


def test_relocation_refuses_unrelated_roots():
    from calibration_pipeline.observations import _relocation

    with pytest.raises(ValueError, match="shares no trailing path"):
        _relocation("/a/b/c", "/x/y/z")
