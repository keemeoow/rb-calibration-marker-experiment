#!/usr/bin/env python3
"""Rebuild the canonical Table 1 CSV and synchronize HTML data arrays.

The executable source is one unified A0-A4/B1-B3 runner artifact.
  * common cross-target/path metrics: cross-target evaluator
  * end-to-end marker systems: modality-specific marker runner

Markdown prose remains hand-authored, and ``verify_table1_visual_sync.py``
checks its displayed values against the CSV rebuilt here.
"""

from __future__ import annotations

import csv
import json
from pathlib import Path
import re
from typing import Any, Iterable


ROOT = Path(__file__).resolve().parents[1]
TABLE1_JSON = ROOT / "CP_result/session02/late_table1/table1_methods.json"
CROSS_CSV = ROOT / "CP_result/session02/cross_target_evaluation/cross_target_evaluation.csv"
MARKER_CSV = ROOT / "CP_result/session02/marker_system_end_to_end/marker_system_end_to_end.csv"
MAIN_CSV = ROOT / "CP_result/session02/late_table1/table1_results.csv"
HTML = ROOT / "_TABLE1_INTERACTIVE.html"

METHODS = ("A0", "A1", "A2", "A3", "A4", "A5", "B1", "B2", "B3")
RUNNABLE_METHODS = frozenset({"A0", "A1", "A2", "A3", "A4", "B1", "B2", "B3"})

FIELDS = (
    "method", "status", "converged_runs", "total_runs",
    "n_registered_fixed_cameras",
    "train_overall_rmse_px", "train_board_rmse_px", "train_cube_rmse_px",
    "test_overall_rmse_px", "test_board_rmse_px", "test_cube_rmse_px",
    "cross_view_pixel_transfer_rmse_px",
    "e_cross_translation_rmse_mm", "e_cross_rotation_rmse_deg",
    "e_e2e_translation_rmse_mm", "e_e2e_rotation_rmse_deg",
    "cross_target_overall_rmse_px", "cross_target_board_rmse_px",
    "cross_target_cube_rmse_px", "confirmatory_ready",
)

HTML_MAIN_KEYS = {
    "nReg": "n_registered_fixed_cameras",
    "trainOverallPx": "train_overall_rmse_px",
    "trainBoardPx": "train_board_rmse_px",
    "trainCubePx": "train_cube_rmse_px",
    "testOverallPx": "test_overall_rmse_px",
    "testBoardPx": "test_board_rmse_px",
    "testCubePx": "test_cube_rmse_px",
    "crossViewPx": "cross_view_pixel_transfer_rmse_px",
    "crossT": "e_cross_translation_rmse_mm",
    "crossR": "e_cross_rotation_rmse_deg",
    "e2eT": "e_e2e_translation_rmse_mm",
    "e2eR": "e_e2e_rotation_rmse_deg",
    "crossOverallPx": "cross_target_overall_rmse_px",
    "crossBoardPx": "cross_target_board_rmse_px",
    "crossCubePx": "cross_target_cube_rmse_px",
}

HTML_MARKER_KEYS = {
    "convergedRuns": "converged_runs",
    "totalRuns": "total_runs",
    "nReg": "n_registered_fixed_cameras",
    "ownHeldoutPx": "own_heldout_overall_rmse_px_mean",
    "commonOverallPx": "common_target_overall_rmse_px_mean",
    "commonBoardPx": "common_target_board_rmse_px_mean",
    "commonCubePx": "common_target_cube_rmse_px_mean",
    "crossViewPx": "cross_view_pixel_transfer_rmse_px_mean",
    "crossT": "e_cross_translation_rmse_mm_mean",
    "crossR": "e_cross_rotation_rmse_deg_mean",
    "e2eT": "e_e2e_translation_rmse_mm_mean",
    "e2eR": "e_e2e_rotation_rmse_deg_mean",
}


def nested(payload: dict, keys: Iterable[str]) -> Any:
    current: Any = payload
    for key in keys:
        if not isinstance(current, dict):
            return None
        current = current.get(key)
    return current


def mean(runs: list[dict], *keys: str) -> float | None:
    values = [nested(run, keys) for run in runs]
    values = [float(value) for value in values if isinstance(value, (int, float))]
    return None if not values else sum(values) / len(values)


def formatted(value: Any, digits: int = 4) -> str:
    return "" if value is None else f"{float(value):.{digits}f}"


def read_csv(path: Path, key: str = "method") -> dict[str, dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as handle:
        return {row[key]: row for row in csv.DictReader(handle)}


def build_main_rows() -> list[dict[str, str]]:
    result = json.loads(TABLE1_JSON.read_text(encoding="utf-8"))
    cross = read_csv(CROSS_CSV)
    output: list[dict[str, str]] = []
    for method in METHODS:
        if method == "A5":
            row = {field: "" for field in FIELDS}
            row.update({
                "method": "A5", "status": "not_run", "converged_runs": "0",
                "total_runs": "0", "confirmatory_ready": "false",
            })
            output.append(row)
            continue
        if method in RUNNABLE_METHODS:
            runs = result["rows"][method]["runs"]
            factor_ready = bool(result["protocol"]["fk_factor"]["covariance"].get(
                "confirmatory_ready", False))
            status = (
                "preflight_simulation_prior"
                if method in {"A4", "B1", "B2"} and not factor_ready
                else "complete"
            )
        else:
            raise AssertionError(method)
        row = {field: "" for field in FIELDS}
        row.update({
            "method": method,
            "status": status,
            "converged_runs": str(sum(bool(run.get("converged")) for run in runs)),
            "total_runs": str(len(runs)),
            "n_registered_fixed_cameras": str(len(runs[0]["transforms"]["T_base_Ci"])),
            "confirmatory_ready": "false",
        })
        for prefix, section in (("train", "train_reprojection"),
                                ("test", "heldout_reprojection")):
            for group in ("overall", "board", "cube"):
                row[f"{prefix}_{group}_rmse_px"] = formatted(
                    mean(runs, section, group, "rmse_px"))
        cross_row = cross[method]
        for main_key, cross_key in {
            "e_cross_translation_rmse_mm": "common_path_e_cross_translation_rmse_mm_mean",
            "e_cross_rotation_rmse_deg": "common_path_e_cross_rotation_rmse_deg_mean",
            "e_e2e_translation_rmse_mm": "common_path_e_e2e_translation_rmse_mm_mean",
            "e_e2e_rotation_rmse_deg": "common_path_e_e2e_rotation_rmse_deg_mean",
            "cross_view_pixel_transfer_rmse_px":
                "common_path_cross_view_pixel_transfer_rmse_px_mean",
            "cross_target_overall_rmse_px": "shared_target_overall_rmse_px_mean",
            "cross_target_board_rmse_px": "shared_target_board_rmse_px_mean",
            "cross_target_cube_rmse_px": "shared_target_cube_rmse_px_mean",
        }.items():
            row[main_key] = formatted(cross_row.get(cross_key) or None)
        output.append(row)
    return output


def write_main_csv(rows: list[dict[str, str]]) -> None:
    with MAIN_CSV.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=FIELDS, lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)


def replace_array_values(source: str, array_name: str,
                         rows: dict[str, dict[str, str]], mapping: dict[str, str],
                         id_property: str = "id",
                         id_pattern: str = r"[A-Z][0-9]") -> str:
    array_match = re.search(
        rf"(const\s+{re.escape(array_name)}\s*=\s*\[)(.*?)(\n\s*\];)",
        source, re.S)
    if not array_match:
        raise RuntimeError(f"HTML array not found: {array_name}")
    body = array_match.group(2)
    objects = list(re.finditer(r"\{.*?\}", body, re.S))
    for object_match in reversed(objects):
        block = object_match.group(0)
        id_match = re.search(
            rf'\b{re.escape(id_property)}:\s*"({id_pattern})"', block)
        if not id_match or id_match.group(1) not in rows:
            continue
        row = rows[id_match.group(1)]
        for html_key, csv_key in mapping.items():
            raw = row.get(csv_key, "")
            replacement = "null" if raw in {"", None} else str(raw)
            block, count = re.subn(
                rf"(\b{re.escape(html_key)}:\s*)(null|-?\d+(?:\.\d+)?(?:[eE][+-]?\d+)?)",
                rf"\g<1>{replacement}", block, count=1)
            if count != 1:
                raise RuntimeError(
                    f"{array_name}.{id_match.group(1)} is missing {html_key}")
        body = body[:object_match.start()] + block + body[object_match.end():]
    return (source[:array_match.start()] + array_match.group(1) + body
            + array_match.group(3) + source[array_match.end():])


def sync_html(main_rows: list[dict[str, str]]) -> None:
    marker_rows = read_csv(MARKER_CSV, key="system")
    source = HTML.read_text(encoding="utf-8")
    source = replace_array_values(
        source, "methods",
        {row["method"]: row for row in main_rows if row["method"] != "A5"},
        HTML_MAIN_KEYS)
    source = replace_array_values(
        source, "markerSystems", marker_rows, HTML_MARKER_KEYS,
        id_property="system", id_pattern=r"[a-z_]+")
    HTML.write_text(source, encoding="utf-8")


def main() -> None:
    rows = build_main_rows()
    write_main_csv(rows)
    sync_html(rows)
    print("Synced canonical Table 1 CSV and HTML data arrays from executable artifacts.")


if __name__ == "__main__":
    main()
