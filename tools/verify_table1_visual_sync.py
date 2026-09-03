#!/usr/bin/env python3
"""Verify that current machine-readable results and reports are synchronized."""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
import re
import sys


ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from calibration_pipeline.cross_target import validate_result_contract
from calibration_pipeline.marker_system import validate_end_to_end_contract
from tools.sync_table1_canonical_data import (
    CANONICAL_LABEL_OVERRIDES,
    METHOD_ORDER,
    _csv_rows,
    _method_rows,
)


TABLE1_JSON = ROOT / "CP_result/session04/late_table1/table1_methods.json"
CROSS_JSON = (
    ROOT / "CP_result/session04/cross_target_evaluation/"
    "cross_target_evaluation.json")
MARKER_JSON = (
    ROOT / "CP_result/session04/marker_system_end_to_end/"
    "marker_system_end_to_end.json")
CANONICAL_CSV = ROOT / "CP_result/session04/late_table1/table1_results.csv"
REPORTS = (
    ROOT / "CP_result/session04/late_table1/TABLE1_RESULTS.md",
    ROOT / "CP_result/session04/late_table1/TABLE1_INTERACTIVE.html",
)


def _load_json(path: Path) -> dict:
    with path.open() as handle:
        return json.load(handle)


def _compare_csv(expected: list[dict], canonical_csv: Path) -> None:
    with canonical_csv.open(newline="") as handle:
        actual = list(csv.DictReader(handle))
    if [row["method"] for row in actual] != list(METHOD_ORDER):
        raise AssertionError("canonical CSV method order is incorrect")
    if len(actual) != len(expected):
        raise AssertionError("canonical CSV row count is incorrect")
    for actual_row, expected_row in zip(actual, expected):
        if set(actual_row) != set(expected_row):
            raise AssertionError(
                f"{expected_row['method']}: canonical CSV fields drifted")
        for field, expected_value in expected_row.items():
            actual_value = actual_row[field]
            if expected_value is None:
                if actual_value:
                    raise AssertionError(
                        f"{expected_row['method']}/{field}: expected empty value")
            elif isinstance(expected_value, (int, float)):
                if not math.isclose(
                        float(actual_value), float(expected_value),
                        rel_tol=1e-12, abs_tol=1e-12):
                    raise AssertionError(
                        f"{expected_row['method']}/{field}: numeric drift")
            elif actual_value != str(expected_value):
                raise AssertionError(
                    f"{expected_row['method']}/{field}: text drift")


def _verify_reports(table1: dict, marker: dict,
                    reports: tuple[Path, ...]) -> None:
    forbidden = re.compile(r"(?i)(?<![a-z])common(?![a-z])|reference[_ -]free|공통")
    required = (
        "Pre-GT Internal Evaluation (외부 GT 전 내부 평가)",
        "Current Data Warnings (현재 데이터 경고)",
        "Internal-Only Claim Envelope (현재 가능한 최대 결론)",
        "Confirmatory Internal (확증 내부)",
        "Preflight (예비실험)",
        "Post-hoc Diagnostics (사후 원인 진단)",
        "Matched Contrast Decision Table (비교실험 구성 확정표)",
        "Metric Decision Matrix (평가지표 판정표)",
        "Exploratory Paired Set Bootstrap CI (탐색적 paired set bootstrap CI)",
        "Set-equal-weight Held-out RMSE (set 동일가중 홀드아웃)",
        "Scheduled External GT Task (다음주 예정 태스크)",
        "A — Fixed-to-Fixed",
        "Gripper-to-Fixed (그리퍼카메라–고정카메라 간)",
        "Terminology (용어 설명)",
        "Reference-dependent Reprojection (기준 의존 재투영)",
    )
    labels = [
        CANONICAL_LABEL_OVERRIDES.get(
            method, table1["rows"][method]["condition"]["label"])
        for method in METHOD_ORDER
    ] + [row["label"] for row in marker["summary"]]
    for path in reports:
        text = path.read_text()
        match = forbidden.search(text)
        if match:
            raise AssertionError(
                f"{path.name}: stale evaluation term {match.group(0)!r}")
        for phrase in required:
            if phrase not in text:
                raise AssertionError(f"{path.name}: missing {phrase!r}")
        if "Required Next Experiment (다음 필수 실험)" in text:
            raise AssertionError(
                f"{path.name}: external-GT next experiment must not be required")
        if "Internal-Only Stopping Point (현재 종료 지점)" in text:
            raise AssertionError(
                f"{path.name}: current state must be scheduled, not a stopping point")
        for label in labels:
            if label not in text:
                raise AssertionError(f"{path.name}: missing result label {label!r}")


def _verify_provenance(table1: dict, cross: dict, marker: dict) -> None:
    table_protocol = table1["protocol"]
    cross_protocol = cross["protocol"]
    marker_protocol = marker["protocol"]
    if table_protocol["split"] != cross_protocol["split"]:
        raise AssertionError("Table 1 and camera-scope split differ")
    if table_protocol["split"] != marker_protocol["split"]:
        raise AssertionError("Table 1 and marker-system split differ")
    source = table_protocol["source_data_provenance"]
    if source != cross_protocol["source_data_provenance"]:
        raise AssertionError("camera-scope source-data provenance differs")
    if source != marker_protocol["source_data_provenance"]:
        raise AssertionError("marker-system source-data provenance differs")
    if cross_protocol.get("external_ground_truth_used") is not False:
        raise AssertionError("camera-scope result incorrectly claims external GT")
    if marker_protocol.get("external_ground_truth_used") is not False:
        raise AssertionError("marker-system result incorrectly claims external GT")


def parse_args(argv=None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Verify synchronized Table 1 JSON/CSV/Markdown/HTML artifacts")
    parser.add_argument("--table1", default=str(TABLE1_JSON))
    parser.add_argument("--cross", default=str(CROSS_JSON))
    parser.add_argument("--marker", default=str(MARKER_JSON))
    parser.add_argument("--csv", default=str(CANONICAL_CSV))
    parser.add_argument("--late_report", default=str(REPORTS[0]))
    parser.add_argument("--html", default=str(REPORTS[1]))
    return parser.parse_args(argv)


def main(argv=None) -> None:
    args = parse_args(argv)
    table1_json = Path(args.table1)
    cross_json = Path(args.cross)
    marker_json = Path(args.marker)
    canonical_csv = Path(args.csv)
    reports = tuple(map(Path, (args.late_report, args.html)))
    paths = (table1_json, cross_json, marker_json, canonical_csv, *reports)
    missing = [str(path) for path in paths if not path.is_file()]
    if missing:
        raise FileNotFoundError(f"required artifacts are missing: {missing}")
    table1 = _load_json(table1_json)
    cross = _load_json(cross_json)
    marker = _load_json(marker_json)
    validate_result_contract(cross)
    validate_end_to_end_contract(marker)
    _verify_provenance(table1, cross, marker)
    _compare_csv(_csv_rows(_method_rows(table1, cross)), canonical_csv)
    _verify_reports(table1, marker, reports)
    print("[PASS] Current JSON, CSV, Markdown, and HTML are synchronized")


if __name__ == "__main__":
    main()
