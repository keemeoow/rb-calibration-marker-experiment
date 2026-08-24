#!/usr/bin/env python3
"""Verify the two human-facing Table 1 views against canonical artifacts."""

from __future__ import annotations

import csv
import hashlib
import json
from pathlib import Path
import re


ROOT = Path(__file__).resolve().parents[1]
MAIN_CSV = ROOT / "CP_result/session02/late_table1/table1_results.csv"
CROSS_CSV = ROOT / "CP_result/session02/cross_target_evaluation/cross_target_evaluation.csv"
CROSS_JSON = ROOT / "CP_result/session02/cross_target_evaluation/cross_target_evaluation.json"
MARKER_CSV = ROOT / "CP_result/session02/marker_system_end_to_end/marker_system_end_to_end.csv"
MARKER_JSON = ROOT / "CP_result/session02/marker_system_end_to_end/marker_system_end_to_end.json"
TABLE1_RESULT = ROOT / "CP_result/session02/late_table1/table1_methods.json"
SHARED_BASELINE = ROOT / "CP_result/session02/late_table1/shared_train_only_baseline.json"
MARKDOWN = ROOT / "CP_result/session02/late_table1/TABLE1_RESULTS.md"
SESSION_SUMMARY = ROOT / "session02_result_table1.md"
HTML = ROOT / "_TABLE1_INTERACTIVE.html"

MAIN_KEYS = {
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

MARKER_KEYS = {
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

CROSS_KEYS = {
    "cross_target_overall_rmse_px": "shared_target_overall_rmse_px_mean",
    "cross_target_board_rmse_px": "shared_target_board_rmse_px_mean",
    "cross_target_cube_rmse_px": "shared_target_cube_rmse_px_mean",
    "cross_view_pixel_transfer_rmse_px":
        "common_path_cross_view_pixel_transfer_rmse_px_mean",
    "e_cross_translation_rmse_mm":
        "common_path_e_cross_translation_rmse_mm_mean",
    "e_cross_rotation_rmse_deg":
        "common_path_e_cross_rotation_rmse_deg_mean",
    "e_e2e_translation_rmse_mm":
        "common_path_e_e2e_translation_rmse_mm_mean",
    "e_e2e_rotation_rmse_deg":
        "common_path_e_e2e_rotation_rmse_deg_mean",
}


def read_csv(path: Path, key: str = "method") -> dict[str, dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))
    if any(None in row or any(value is None for value in row.values()) for row in rows):
        raise AssertionError(f"CSV row-width mismatch: {path}")
    return {row[key]: row for row in rows}


def number(value: str | None) -> float | None:
    return None if value in {None, "", "null"} else float(value)


def assert_same(label: str, actual: float | None, expected: float | None,
                digits: int = 4) -> None:
    if actual is None or expected is None:
        if actual is not None or expected is not None:
            raise AssertionError(f"{label}: {actual!r} != {expected!r}")
        return
    if abs(actual - expected) > 0.5 * 10.0 ** (-digits) + 1e-12:
        raise AssertionError(f"{label}: {actual} != {expected}")


def parse_js_array(source: str, name: str, id_property: str = "id",
                   id_pattern: str = r"[A-Z][0-9]") -> dict[str, dict[str, float | None]]:
    match = re.search(rf"const\s+{re.escape(name)}\s*=\s*\[(.*?)\n\s*\];", source, re.S)
    if not match:
        raise AssertionError(f"HTML array not found: {name}")
    output = {}
    for item in re.finditer(r"\{(.*?)\}", match.group(1), re.S):
        body = item.group(1)
        identifier = re.search(
            rf'\b{re.escape(id_property)}:\s*"({id_pattern})"', body)
        if not identifier:
            continue
        output[identifier.group(1)] = {
            key: number(raw)
            for key, raw in re.findall(
                r"\b([A-Za-z]\w*):\s*(null|-?\d+(?:\.\d+)?(?:[eE][+-]?\d+)?)",
                body,
            )
        }
    return output


def section(markdown: str, heading: str) -> str:
    match = re.search(
        rf"^##\s+{re.escape(heading)}\s*$\n(.*?)(?=^##\s|\Z)",
        markdown, re.M | re.S)
    if not match:
        raise AssertionError(f"Markdown section not found: {heading}")
    return match.group(1)


def table_rows(block: str, pattern: str) -> dict[str, list[str]]:
    rows = {}
    for line in block.splitlines():
        if not re.match(rf"^\|\s*({pattern})\s*\|", line):
            continue
        cells = [cell.strip() for cell in line.strip().strip("|").split("|")]
        rows[cells[0]] = cells
    return rows


def cell_values(cell: str) -> list[float | None]:
    output = []
    normalized = cell.replace("**", "").replace("`", "").replace("−", "-")
    for token in normalized.split("/"):
        token = token.strip().split("±", 1)[0]
        match = re.search(r"-?\d+(?:\.\d+)?", token)
        output.append(None if token in {"", "—", "-"} or match is None else float(match.group(0)))
    return output


def verify_session_summary(
        main: dict[str, dict[str, str]],
        marker: dict[str, dict[str, str]],
        markdown: str) -> None:
    """Verify the standalone session02 result report against canonical CSVs."""
    rows = table_rows(section(markdown, "2. 조건별 현재 결과"), r"(?:A|B)[0-9]")
    if set(rows) != set(main):
        raise AssertionError("Session summary method set differs from canonical CSV")
    columns = (
        (2, ("test_overall_rmse_px", "test_board_rmse_px", "test_cube_rmse_px")),
        (3, ("cross_target_overall_rmse_px", "cross_target_board_rmse_px",
             "cross_target_cube_rmse_px")),
        (4, ("cross_view_pixel_transfer_rmse_px",)),
        (5, ("e_cross_translation_rmse_mm", "e_cross_rotation_rmse_deg")),
        (6, ("e_e2e_translation_rmse_mm", "e_e2e_rotation_rmse_deg")),
    )
    for method, cells in rows.items():
        for index, keys in columns:
            actual = cell_values(cells[index])
            expected = [number(main[method][key]) for key in keys]
            if actual == [None] and all(value is None for value in expected):
                actual = [None] * len(expected)
            if len(actual) != len(expected):
                raise AssertionError(
                    f"Session summary {method} column width mismatch")
            for key, left, right in zip(keys, actual, expected):
                assert_same(f"Session summary {method}.{key}", left, right)

    marker_rows = table_rows(
        section(markdown, "4. Marker-system End-to-End 결과"),
        r"(?:board-only|cube-only|board\+cube)")
    marker_aliases = {
        "board-only": "board_only",
        "cube-only": "cube_only",
        "board+cube": "board_cube",
    }
    if set(marker_rows) != set(marker_aliases):
        raise AssertionError("Session summary marker-system set mismatch")
    marker_columns = (
        (1, ("own_heldout_overall_rmse_px_mean",)),
        (2, ("common_target_overall_rmse_px_mean",
             "common_target_board_rmse_px_mean",
             "common_target_cube_rmse_px_mean")),
        (3, ("cross_view_pixel_transfer_rmse_px_mean",)),
        (4, ("e_cross_translation_rmse_mm_mean", "e_cross_rotation_rmse_deg_mean")),
        (5, ("e_e2e_translation_rmse_mm_mean", "e_e2e_rotation_rmse_deg_mean")),
    )
    for label, cells in marker_rows.items():
        canonical = marker[marker_aliases[label]]
        for index, keys in marker_columns:
            actual = cell_values(cells[index])
            expected = [number(canonical[key]) for key in keys]
            if len(actual) != len(expected):
                raise AssertionError(
                    f"Session summary {label} column width mismatch")
            for key, left, right in zip(keys, actual, expected):
                assert_same(f"Session summary {label}.{key}", left, right)

    delta_specs = {
        "A0 → B3": ("A0", "B3", "test_board_rmse_px"),
        "A0 → A1": ("A0", "A1", "test_board_rmse_px"),
        "A1 → A2": ("A1", "A2", "test_overall_rmse_px"),
        "B1 → A4": ("B1", "A4", "test_overall_rmse_px"),
        "A2 → A3": ("A2", "A3", "test_overall_rmse_px"),
        "A2 → A4": ("A2", "A4", "test_overall_rmse_px"),
        "A3 → A4": ("A3", "A4", "test_overall_rmse_px"),
        "B2 → A4": ("B2", "A4", "test_cube_rmse_px"),
        "B3 → A2": ("B3", "A2", "test_board_rmse_px"),
    }
    delta_rows: dict[str, list[str]] = {}
    for line in section(markdown, "3. 비교쌍별 변화량과 해석").splitlines():
        if not line.startswith("|"):
            continue
        cells = [cell.strip() for cell in line.strip().strip("|").split("|")]
        for label in delta_specs:
            if cells[0].startswith(label):
                delta_rows[label] = cells
    if set(delta_rows) != set(delta_specs):
        raise AssertionError("Session summary comparison-pair set mismatch")
    delta_columns = (
        (2, ("cross_target_overall_rmse_px",)),
        (3, ("cross_view_pixel_transfer_rmse_px",)),
        (4, ("e_cross_translation_rmse_mm", "e_cross_rotation_rmse_deg")),
        (5, ("e_e2e_translation_rmse_mm", "e_e2e_rotation_rmse_deg")),
    )
    for label, (left_id, right_id, test_key) in delta_specs.items():
        cells = delta_rows[label]
        actual_test = cell_values(cells[1])[0]
        expected_test = number(main[right_id][test_key]) - number(main[left_id][test_key])
        assert_same(f"Session summary {label}.{test_key} delta", actual_test, expected_test)
        for index, keys in delta_columns:
            actual = cell_values(cells[index])
            expected = [number(main[right_id][key]) - number(main[left_id][key])
                        for key in keys]
            for key, observed, computed in zip(keys, actual, expected):
                assert_same(f"Session summary {label}.{key} delta", observed, computed)

    for disclosure in (
        "FK, eye-in-hand 카메라, 정답 cube pose를 사용하지 않는다",
        "내부 경로 일관성이지 외부 절대 정확도는 아니다",
        "독립 반복실험이나 통계적 유의성 검정으로 해석하면 안 된다",
    ):
        if disclosure not in markdown:
            raise AssertionError(f"Session summary disclosure missing: {disclosure}")


def verify_html(main: dict[str, dict[str, str]], marker: dict[str, dict[str, str]],
                html: str) -> int:
    embedded = parse_js_array(html, "methods")
    expected_methods = set(main) - {"A5"}
    if set(embedded) != expected_methods:
        raise AssertionError("HTML method set differs from canonical CSV")
    checks = 0
    for method in expected_methods:
        for html_key, csv_key in MAIN_KEYS.items():
            assert_same(
                f"HTML {method}.{html_key}", embedded[method].get(html_key),
                number(main[method][csv_key]))
            checks += 1

    embedded_marker = parse_js_array(
        html, "markerSystems", "system", r"[a-z_]+")
    if set(embedded_marker) != set(marker):
        raise AssertionError("HTML marker-system set differs from canonical CSV")
    for system in marker:
        for html_key, csv_key in MARKER_KEYS.items():
            assert_same(
                f"HTML marker {system}.{html_key}",
                embedded_marker[system].get(html_key), number(marker[system][csv_key]), 9)
            checks += 1
    forbidden = (
        "fullFitMethods", "full_data_fit",
        "전체 데이터 캘리브레이션 · no split",
        "A4a", "A4b", "Supplementary · A4 factor 분해",
    )
    for text in forbidden:
        if text in html:
            raise AssertionError(f"obsolete full-data view remains in HTML: {text}")
    for text in (
        "고정카메라 간 큐브 위치 일관성",
        "FK·손목카메라·외부 GT는 사용하지 않습니다",
        "마커 시스템 end-to-end 비교",
    ):
        if text not in html:
            raise AssertionError(f"HTML disclosure missing: {text}")
    return checks


def verify_markdown(main: dict[str, dict[str, str]], marker: dict[str, dict[str, str]],
                    markdown: str) -> None:
    rows = table_rows(section(markdown, "Main Table 전체 결과"), r"(?:A|B)[0-9]")
    if set(rows) != set(main):
        raise AssertionError("Markdown method set differs from canonical CSV")
    columns = (
        (4, ("train_overall_rmse_px", "train_board_rmse_px", "train_cube_rmse_px")),
        (5, ("test_overall_rmse_px", "test_board_rmse_px", "test_cube_rmse_px")),
        (6, ("cross_view_pixel_transfer_rmse_px",)),
        (7, ("e_cross_translation_rmse_mm", "e_cross_rotation_rmse_deg")),
        (8, ("e_e2e_translation_rmse_mm", "e_e2e_rotation_rmse_deg")),
    )
    for method, cells in rows.items():
        for index, keys in columns:
            actual = cell_values(cells[index])
            expected = [number(main[method][key]) for key in keys]
            if actual == [None] and all(value is None for value in expected):
                actual = [None] * len(expected)
            if len(actual) != len(expected):
                raise AssertionError(f"Markdown {method} column width mismatch")
            for key, left, right in zip(keys, actual, expected):
                assert_same(f"Markdown {method}.{key}", left, right)

    marker_rows = table_rows(
        section(markdown, "End-to-end marker-system 비교"),
        r"(?:board_only|cube_only|board_cube)")
    if set(marker_rows) != set(marker):
        raise AssertionError("Markdown marker-system set differs from canonical CSV")
    if "전체 데이터 재적합 — 보조 in-sample 진단" in markdown:
        raise AssertionError("obsolete full-data section remains in Markdown")
    for text in (
        "일반적인 고정카메라 간 동일 물체 pose consistency와 동일",
        "e_cross`는 이 목적함수에 들어가지 않는 **평가 전용 지표",
        "A0~A4/B1~B3를 한 runner에서 실행",
    ):
        if text not in markdown:
            raise AssertionError(f"Markdown disclosure missing: {text}")


def verify_cross(main: dict[str, dict[str, str]], cross: dict[str, dict[str, str]]) -> None:
    if set(cross) != set(main) - {"A5"}:
        raise AssertionError("Cross-target CSV method set mismatch")
    populations = set()
    for method, row in cross.items():
        for main_key, cross_key in CROSS_KEYS.items():
            assert_same(
                f"Cross CSV {method}.{main_key}", number(main[method][main_key]),
                number(row[cross_key]))
        assert_same(
            f"e_cross alias {method}",
            number(row["fixed_camera_cube_position_consistency_rmse_mm_mean"]),
            number(row["common_path_e_cross_translation_rmse_mm_mean"]), 9)
        population = (
            int(float(row["n_cross_pairs"])),
            int(float(row["n_cross_view_directions"])),
            int(float(row["n_e2e_units"])),
        )
        populations.add(population)
        if (population[0] <= 0 or population[1] != 2 * population[0]
                or population[2] <= 0):
            raise AssertionError(f"{method}: invalid path population {population}")
    if len(populations) != 1:
        raise AssertionError("methods do not share one path population")


def load_authenticated_baseline(path: Path) -> dict:
    payload = json.loads(path.read_text(encoding="utf-8"))
    unhashed = dict(payload)
    expected = str(unhashed.pop("artifact_sha256", ""))
    encoded = json.dumps(unhashed, sort_keys=True, separators=(",", ":")).encode()
    if hashlib.sha256(encoded).hexdigest() != expected:
        raise AssertionError("canonical shared baseline hash mismatch")
    if payload.get("heldout_information_used") is not False:
        raise AssertionError("canonical baseline used held-out information")
    provenance = payload.get("source_data_provenance", {})
    file_entries = [provenance.get("meta_json", {})]
    file_entries.extend(provenance.get("intrinsics", {}).values())
    if provenance.get("pose_convention_manifest"):
        file_entries.append(provenance["pose_convention_manifest"])
    for entry in file_entries:
        source = Path(str(entry.get("path", "")))
        expected_source_hash = str(entry.get("sha256", ""))
        if (not source.is_file()
                or hashlib.sha256(source.read_bytes()).hexdigest()
                != expected_source_hash):
            raise AssertionError(f"canonical result is stale: source changed: {source}")
    implementation = provenance.get("implementation_sha256", {})
    if not implementation:
        raise AssertionError("canonical baseline lacks implementation provenance")
    for relative, expected_source_hash in implementation.items():
        source = ROOT / relative
        if (not source.is_file()
                or hashlib.sha256(source.read_bytes()).hexdigest()
                != expected_source_hash):
            raise AssertionError(
                f"canonical result is stale: implementation changed: {relative}")
    populations = provenance.get("observation_populations", {})
    if set(populations) != {"eligible", "train", "heldout"}:
        raise AssertionError("canonical baseline lacks observation provenance")
    return payload


def verify_json_contracts() -> None:
    baseline = load_authenticated_baseline(SHARED_BASELINE)
    result = json.loads(TABLE1_RESULT.read_text(encoding="utf-8"))
    cross = json.loads(CROSS_JSON.read_text(encoding="utf-8"))
    marker = json.loads(MARKER_JSON.read_text(encoding="utf-8"))
    baseline_sha = baseline["artifact_sha256"]
    expected_rows = {"A0", "A1", "A2", "A3", "A4", "B1", "B2", "B3"}
    if set(result.get("rows", {})) != expected_rows:
        raise AssertionError("unified result contains obsolete or missing main rows")
    if result["protocol"]["shared_train_only_baseline"]["sha256"] != baseline_sha:
        raise AssertionError("unified result does not authenticate canonical baseline")
    if (result["protocol"].get("source_data_provenance")
            != baseline.get("source_data_provenance")):
        raise AssertionError("unified result source provenance differs from baseline")
    pose_convention = result["protocol"].get("pose_convention", {})
    if pose_convention.get("status") != "normalized_and_validated":
        raise AssertionError("canonical Table 1 pose convention was not normalized")
    if baseline.get("pose_convention") != pose_convention:
        raise AssertionError("unified result pose convention differs from baseline")
    for name, payload in (("cross-target", cross), ("marker-system", marker)):
        if (payload.get("protocol", {}).get("source_data_provenance")
                != baseline.get("source_data_provenance")):
            raise AssertionError(f"{name} source provenance differs from baseline")
        if payload.get("protocol", {}).get("pose_convention") != pose_convention:
            raise AssertionError(f"{name} pose convention differs from Table 1")
    if Path(result["protocol"]["shared_train_only_baseline"]["path"]).resolve() != SHARED_BASELINE:
        raise AssertionError("unified result points to a duplicate baseline")
    if result["protocol"].get("one_runner_for_all_executable_rows") is not True:
        raise AssertionError("Table 1 rows are not declared as one-runner output")
    pnp_contract = result["protocol"].get("cube_detection", {}).get(
        "quality_contract", {})
    if (pnp_contract.get("selection_stage")
            != "before_split_and_before_any_calibration_fit"
            or pnp_contract.get("model_output_used") is not False
            or pnp_contract.get("all_detected_corners_scored") is not True):
        raise AssertionError("cube PnP quality mask is not a common pre-fit mask")
    fk_contract = result["protocol"].get("fk_factor", {}).get(
        "mathematical_contract", {})
    if (fk_contract.get("cube_pose_is_optimization_variable") is not True
            or fk_contract.get("hard_gate_or_pose_replacement") is not False
            or fk_contract.get("external_ground_truth_used") is not False):
        raise AssertionError("corrected-FK factor contract drift")
    mask_sha = result["protocol"]["model_independent_path_evaluation_mask"][
        "evaluation_mask_sha256"]
    cross_protocol = cross["protocol"]["common_path_evaluation"]
    if cross_protocol["evaluation_mask_sha256"] != mask_sha:
        raise AssertionError("cross-target path mask differs from core")
    definition = cross_protocol.get("e_cross_definition", {})
    if (definition.get("uses_robot_fk") is not False
            or definition.get("uses_gripper_camera") is not False
            or definition.get("uses_nominal_or_ground_truth_cube_pose") is not False):
        raise AssertionError("e_cross contract includes a forbidden FK/gripper dependency")
    if definition.get("translation_metric") != "pairwise_cube_center_distance_RMSE_mm":
        raise AssertionError("e_cross is not declared as fixed-camera cube position consistency")
    pixel_definition = cross_protocol.get("e_cross_pixel_transfer_definition", {})
    if (pixel_definition.get("uses_robot_fk") is not False
            or pixel_definition.get("uses_gripper_camera") is not False
            or pixel_definition.get("uses_shared_target_pose") is not False
            or pixel_definition.get("name") !=
            "bidirectional_fixed_camera_cube_pixel_transfer"):
        raise AssertionError("cross-view pixel metric contract is not measurement-only")
    serialized = json.dumps({"table1": result, "cross": cross,
                             "marker": marker}, sort_keys=True)
    if "rmse_image_plane_mm" in serialized:
        raise AssertionError("removed image-plane mm reprojection remains in artifacts")
    if marker["protocol"].get(
            "same_split_raw_detections_K_D_solver_seeds_and_evaluation") is not True:
        raise AssertionError("marker-system evaluation contract mismatch")


def main() -> None:
    main_rows = read_csv(MAIN_CSV)
    cross_rows = read_csv(CROSS_CSV)
    marker_rows = read_csv(MARKER_CSV, "system")
    markdown = MARKDOWN.read_text(encoding="utf-8")
    session_summary = SESSION_SUMMARY.read_text(encoding="utf-8")
    html = HTML.read_text(encoding="utf-8")
    checks = verify_html(main_rows, marker_rows, html)
    verify_markdown(main_rows, marker_rows, markdown)
    verify_session_summary(main_rows, marker_rows, session_summary)
    verify_cross(main_rows, cross_rows)
    verify_json_contracts()
    expected_intrinsics = hashlib.sha256((ROOT / "intrinsics/cam1.npz").read_bytes()).hexdigest()
    if expected_intrinsics not in markdown:
        raise AssertionError("Markdown intrinsics hash mismatch")
    print(
        "OK: Markdown reports and interactive HTML are synchronized to 3 canonical CSV "
        f"artifacts ({checks} embedded numeric fields); one authenticated baseline."
    )


if __name__ == "__main__":
    main()
