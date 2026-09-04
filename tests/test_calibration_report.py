from __future__ import annotations

import csv
import json
from pathlib import Path

from calibration_pipeline.report import METHOD_ORDER, write_report


ROOT = Path(__file__).resolve().parents[1]
SESSION04_TABLE1 = (
    ROOT / "CP_result/session04/late_table1/table1_methods.json")


def test_report_contains_every_final_calibration_matrix(tmp_path):
    result = write_report(SESSION04_TABLE1, tmp_path)

    matrices = json.loads(
        (tmp_path / "calibration_matrices.json").read_text(encoding="utf-8"))
    assert tuple(matrices["rows"]) == METHOD_ORDER
    assert matrices["representative_seed"] == 0
    for method in METHOD_ORDER:
        runs = matrices["rows"][method]["runs"]
        assert [run["seed"] for run in runs] == [0, 1, 2]
        assert all(run["converged"] for run in runs)
        for run in runs:
            transforms = run["transforms"]
            assert set(transforms["T_base_Ci"]) == {"0", "1", "3"}
            assert len(transforms["T_gripper_cam"]) == 4

    with (tmp_path / "calibration_summary.csv").open(
            encoding="utf-8-sig", newline="") as stream:
        summary = list(csv.DictReader(stream))
    assert len(summary) == len(METHOD_ORDER)
    assert sum(int(row["converged_runs"]) for row in summary) == 27
    assert sum(int(row["prune_refit_attempts"]) for row in summary) == 15
    assert sum(int(row["prune_refit_rollbacks"]) for row in summary) == 15

    assert not (tmp_path / "CALIBRATION_RESULTS.md").exists()
    assert result["rows"] == 9
    assert "markdown" not in result
