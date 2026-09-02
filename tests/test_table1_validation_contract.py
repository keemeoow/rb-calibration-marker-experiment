from __future__ import annotations

from pathlib import Path

from calibration_pipeline.schema import (
    EVALUATION_COMPARISON_CONTRACT,
    MAIN_ABLATION_CONDITIONS,
)
from tools.sync_table1_canonical_data import (
    _data_warnings,
    _load,
    _markdown,
    _method_rows,
)


ROOT = Path(__file__).resolve().parents[1]
TABLE1_JSON = ROOT / "CP_result/session04/late_table1/table1_methods.json"
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
        "heldout_reprojection.board",)
    assert contracts["A2_to_A4"]["rows"] == ("A2", "A4")
    assert contracts["A2_to_A4"]["components"] == (
        "heldout_reprojection.board",
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
    assert "dropped sets `0, 1, 2, 3`" in markdown
    assert "10.8077 mm translation RMSE" in markdown
    assert "### Confirmatory Internal" in markdown
    assert "### Preflight" in markdown
    assert "### Post-hoc Diagnostics" in markdown
    assert "A3 (raw-FK hard fixed)" in markdown
    assert "A4 (corrected-FK soft factor)" in markdown
    assert "Ours (raw-FK-fixed target)" not in markdown
    assert "Ours (corrected-FK factor)" not in markdown
