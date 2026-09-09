#!/usr/bin/env python3
"""Generate the canonical CSV, Markdown, and HTML from current result JSON.

Only the v8 set-anchor camera-scope evaluation and v5 marker-system evaluation are
accepted. This prevents older metric definitions from entering a new report.
"""

from __future__ import annotations

import argparse
import csv
from html import escape
import json
from pathlib import Path
from statistics import fmean

METHOD_ORDER = ("A0", "A1", "A2", "A3", "A4", "A5", "B1", "B2", "B3")
# Label-only migrations for numerical artifacts that predate corrected prose.
CANONICAL_LABEL_OVERRIDES = {
    "A3": "FK hard fixed",
    "A4": "corrected-FK soft factor",
    "A5": "corrected-FK hard fixed (VISION-aligned)",
}
SYSTEM_ORDER = ("board_only", "cube_only", "board_cube")
FINAL_TARGET = "cube"
TARGETS = ("board", "cube")
SCOPE_FIELDS = (
    "cross_view_pixel_transfer_rmse_px",
    "pose_consistency_translation_rmse_mm",
    "pose_consistency_rotation_rmse_deg",
)
ROOT = Path(__file__).resolve().parents[1]


def _load(path: Path) -> dict:
    with path.open() as handle:
        return json.load(handle)


def _mean(values):
    numeric = [float(value) for value in values if value is not None]
    return None if not numeric else fmean(numeric)


def _fmt(value, digits: int = 4) -> str:
    if value is None or value == "":
        return "N/A"
    return f"{float(value):.{digits}f}"


def _minimum(rows, key: str):
    values = [row.get(key) for row in rows]
    numeric = [float(value) for value in values if value not in (None, "")]
    return None if not numeric else min(numeric)


def _weighted_rmse(*entries: tuple[float | None, int | None]) -> float | None:
    total_weight = 0
    total_square = 0.0
    for value, weight in entries:
        if value in (None, "") or weight in (None, ""):
            continue
        weight_int = int(weight)
        if weight_int <= 0:
            continue
        total_weight += weight_int
        total_square += float(value) ** 2 * weight_int
    if total_weight == 0:
        return None
    return (total_square / total_weight) ** 0.5


def _combined_cross_view_run(run: dict) -> dict:
    """Pool the frozen cube pair population directly across both camera scopes."""
    pairs = []
    support_signature = []
    scope_counts = {}
    for scope in ("fixed_to_fixed", "gripper_to_fixed"):
        scope_pairs = run[scope]["by_target"][FINAL_TARGET]["per_pair"]
        scope_counts[scope] = len(scope_pairs)
        for pair in scope_pairs:
            if scope == "fixed_to_fixed":
                endpoints = (
                    pair["left_observation_id"], pair["right_observation_id"])
            else:
                endpoints = (
                    pair["fixed_observation_id"], pair["gripper_observation_id"])
            support_signature.append((scope, *endpoints))
            pairs.append(pair)

    pixel_entries = []
    n_directions = 0
    n_destination_corners = 0
    for pair in pairs:
        for direction in pair["pixel_transfer_directions"]:
            n_corners = int(direction["n_corners"])
            pixel_entries.append((direction["rmse_px"], n_corners))
            n_directions += 1
            n_destination_corners += n_corners

    gripper_pairs = run["gripper_to_fixed"]["by_target"][FINAL_TARGET][
        "per_pair"]
    train_anchor_pairs = sum(
        pair.get("fixed_anchor_split_role") == "train"
        for pair in gripper_pairs)
    heldout_anchor_pairs = sum(
        pair.get("fixed_anchor_split_role") == "heldout"
        for pair in gripper_pairs)
    return {
        "cross_view_pixel_transfer_rmse_px": _weighted_rmse(*pixel_entries),
        "pose_consistency_translation_rmse_mm": _weighted_rmse(*(
            (pair["translation_mm"], 1) for pair in pairs)),
        "pose_consistency_rotation_rmse_deg": _weighted_rmse(*(
            (pair["rotation_deg"], 1) for pair in pairs)),
        "n_pairs": len(pairs),
        "n_directions": n_directions,
        "n_destination_corners": n_destination_corners,
        "n_fixed_to_fixed_pairs": scope_counts["fixed_to_fixed"],
        "n_gripper_to_fixed_pairs": scope_counts["gripper_to_fixed"],
        "n_train_fixed_anchor_pairs": train_anchor_pairs,
        "n_heldout_fixed_anchor_pairs": heldout_anchor_pairs,
        "support_signature": tuple(support_signature),
    }


def _combined_cross_view_metrics(cross: dict, method: str) -> dict:
    combined = [
        _combined_cross_view_run(run)
        for run in cross["per_run"][method]
    ]
    support_keys = (
        "n_pairs", "n_directions", "n_destination_corners",
        "n_fixed_to_fixed_pairs", "n_gripper_to_fixed_pairs",
        "n_train_fixed_anchor_pairs", "n_heldout_fixed_anchor_pairs",
        "support_signature",
    )
    reference = tuple(combined[0][key] for key in support_keys)
    if any(tuple(run[key] for key in support_keys) != reference
           for run in combined[1:]):
        raise ValueError(f"{method}: combined cross-view support drifted across seeds")
    return {
        "cross_view_pixel_transfer_rmse_px": _mean([
            run["cross_view_pixel_transfer_rmse_px"] for run in combined]),
        "pose_consistency_translation_rmse_mm": _mean([
            run["pose_consistency_translation_rmse_mm"] for run in combined]),
        "pose_consistency_rotation_rmse_deg": _mean([
            run["pose_consistency_rotation_rmse_deg"] for run in combined]),
        **{key: combined[0][key] for key in support_keys},
    }


def _fmt_best(value, best, digits: int = 4, html: bool = False) -> str:
    """Bold a displayed minimum, including values tied after rounding."""
    formatted = _fmt(value, digits)
    if best is None or formatted == "N/A" or formatted != _fmt(best, digits):
        return formatted
    return (f"<strong>{formatted}</strong>" if html
            else f"**{formatted}**")


def _status_label(value: str) -> str:
    return {
        "complete": "Current data available (현재 데이터 있음)",
        "preflight_simulation_prior": (
            "Current data available; measured FK covariance pending "
            "(현재 데이터 있음; FK covariance 측정 대기)"),
        "freeze_before_external_gt": (
            "Current data available; freeze before External GT scoring "
            "(현재 데이터 있음; External GT 채점 전 고정 필요)"),
    }.get(value, value)


def _display_label(method: str, source_label: str) -> str:
    return CANONICAL_LABEL_OVERRIDES.get(method, source_label)


def _result_sections(rows: list[dict]) -> list[tuple[str, str, list[dict]]]:
    return [
        (
            "Final A0-A5/B1-B3 (최종 단일 비교 구성)",
            "모든 행은 같은 cube heldout / External cube GT 평가 대상에서만 비교한다.",
            rows,
        ),
    ]


def _reprojection_entry(run: dict, split: str, target: str) -> dict | None:
    if target == FINAL_TARGET:
        cube_eval = run.get("cube_evaluation_reprojection")
        if cube_eval:
            entry = cube_eval.get(split, {}).get(FINAL_TARGET)
            if entry is not None:
                return entry
    return run[f"{split}_reprojection"].get(target)


def _reprojection_mean(runs: list[dict], split: str,
                       target: str) -> float | None:
    return _mean([
        None if _reprojection_entry(run, split, target) is None
        else _reprojection_entry(run, split, target)["rmse_px"]
        for run in runs
    ])


def _reprojection_field_mean(runs: list[dict], split: str, target: str,
                             field: str) -> float | None:
    return _mean([
        None if _reprojection_entry(run, split, target) is None
        else _reprojection_entry(run, split, target).get(field)
        for run in runs
    ])


def _corner_count(runs: list[dict], split: str, target: str) -> int | None:
    """Corner counts are frozen by the split, so every seed reports the same."""
    for run in runs:
        entry = _reprojection_entry(run, split, target)
        if entry is not None:
            return int(entry["n_corners"])
    return None


def _combined_reprojection_mean(
        runs: list[dict], target: str,
        splits: tuple[str, ...] = ("train", "heldout")) -> float | None:
    """Average split-combined RMSE across seeds using corner support weights."""
    per_run = []
    for run in runs:
        total_weight = 0
        total_square = 0.0
        for split in splits:
            entry = _reprojection_entry(run, split, target)
            if entry is None or entry.get("rmse_px") is None:
                continue
            weight = int(entry.get("n_corners", 0))
            total_weight += weight
            total_square += float(entry["rmse_px"]) ** 2 * weight
        if total_weight > 0:
            per_run.append((total_square / total_weight) ** 0.5)
    return _mean(per_run)


def _per_set_mean_squares(runs: list[dict], split: str,
                          target: str) -> dict[int, float]:
    by_set: dict[int, list[float]] = {}
    for run in runs:
        entry = _reprojection_entry(run, split, target)
        if entry is None:
            continue
        per_set = entry.get("set_equal_weight_per_set")
        if not per_set:
            continue
        for item in per_set:
            by_set.setdefault(int(item["set"]), []).append(
                float(item["mean_square_px2"]))
    return {
        set_index: fmean(values)
        for set_index, values in sorted(by_set.items())
        if values
    }


def _primary_objective_diagnostic(run: dict) -> tuple[str | None, dict | None]:
    """Return the coupled/factor stage used for objective-block reporting."""
    stages = run.get("stages", {})
    for name in (
            "joint_eih_e2h",
            "stage1_eih_with_fk_factor",
            "stage1_eih",
            "stage2_e2h"):
        diagnostic = stages.get(name)
        if isinstance(diagnostic, dict) and diagnostic.get(
                "objective_block_costs") is not None:
            return name, diagnostic
    return None, None


def _session_root(table1: dict) -> Path | None:
    dataset = str(table1.get("protocol", {}).get("dataset", ""))
    if not dataset:
        return None
    path = Path(dataset)
    if not path.is_absolute():
        path = ROOT / path
    if path.name == "calib_train":
        return path.parent
    return path


def _load_optional_json(path: Path) -> dict:
    if not path.is_file():
        return {}
    with path.open() as handle:
        return json.load(handle)


def _data_warnings(table1: dict, cross: dict) -> dict:
    protocol = cross.get("protocol", {})
    split = protocol.get("split", {})
    support = protocol.get("support", {})
    dropped = split.get("dropped_sets", {})
    warnings = {
        "eligible_sets": split.get("eligible_sets", []),
        "dropped_sets": sorted(dropped, key=lambda value: int(value)),
        "support": support,
        "evaluation_fixed_camera_intersection": protocol.get(
            "evaluation_fixed_camera_intersection", []),
    }
    session_root = _session_root(table1)
    if session_root is None:
        return warnings

    board_cube = _load_optional_json(
        session_root / "calib_out/verify/board_cube_relative_pose/"
        "board_cube_relative_pose_diagnostic.json")
    conflict = board_cube.get("systematic_conflict_contract", {})
    if conflict:
        warnings["board_cube_conflict"] = {
            "translation_rmse_mm": conflict.get("translation_rmse_mm"),
            "maximum_rotation_deg": conflict.get("maximum_rotation_deg"),
            "status": conflict.get("resolution_status"),
        }

    cube_quality = _load_optional_json(
        session_root / "calib_out/verify/cube_observation_quality/"
        "cube_observation_quality.json")
    diagnostics = cube_quality.get("diagnostics", {})
    if diagnostics:
        warnings["cube_quality"] = {
            "counts": diagnostics.get("counts", {}),
            "selected_quality_tier_counts": diagnostics.get(
                "selected_quality_tier_counts", {}),
        }
    return warnings


def _markdown_data_warnings(data_warnings: dict) -> str:
    support = data_warnings.get("support", {})
    overall = support.get("overall", {})
    board = support.get("board", {})
    cube = support.get("cube", {})
    cameras = ", ".join(str(value) for value in data_warnings.get(
        "evaluation_fixed_camera_intersection", [])) or "N/A"
    dropped = ", ".join(data_warnings.get("dropped_sets", [])) or "none"
    eligible = data_warnings.get("eligible_sets", [])
    conflict = data_warnings.get("board_cube_conflict", {})
    quality = data_warnings.get("cube_quality", {})
    counts = quality.get("counts", {})
    selected = quality.get("selected_quality_tier_counts", {})
    lines = [
        "## Current Data Warnings (현재 데이터 경고)",
        "",
        "> 아래 수치는 기존 `data/session04`를 재평가한 **내부 preflight 결과**다. "
        "새 `CAPTURE_PROTOCOL.md`의 composite rig 45-event 촬영 결과가 아니므로, "
        "최종 논문 수치로 확정하지 않는다.",
        "",
        "| 점검 항목 | 현재 데이터 | 결과 해석에 미치는 영향 |",
        "| --- | --- | --- |",
        f"| 평가 support | fixed cameras `{cameras}`; overall "
        f"{overall.get('n_observations', 'N/A')} obs / "
        f"{overall.get('n_corners', 'N/A')} corners; board "
        f"{board.get('n_observations', 'N/A')} / "
        f"{board.get('n_corners', 'N/A')}, cube "
        f"{cube.get('n_observations', 'N/A')} / "
        f"{cube.get('n_corners', 'N/A')} | 현재 촬영에서 공통으로 관측된 범위만 평가 |",
        f"| Split support | {len(eligible)} eligible sets; dropped sets "
        f"`{dropped}` | 제외 set을 숨기지 않고 모든 방법에 동일 적용 |",
    ]
    if counts:
        lines.append(
            "| Cube detection | "
            f"{counts.get('images_read', 'N/A')} images read, "
            f"{counts.get('accepted_observations', 'N/A')} accepted PnP observations, "
            f"{selected.get('nonplanar_multiface', 'N/A')} core multiface selected, "
            f"{counts.get('pnp_rmse_rejections', 'N/A')} PnP-RMSE rejections | "
            "동일 frozen 품질 규칙을 모든 방법에 적용 |")
    if conflict:
        lines.append(
            "| Board-Cube conflict | direct PnP disagreement: "
            f"{_fmt(conflict.get('translation_rmse_mm'))} mm translation RMSE "
            f"/ {_fmt(conflict.get('maximum_rotation_deg'))} deg max rotation | "
            "joint solve가 완화할 수는 있지만 원인을 제거하지는 못함 |")
    return "\n".join(lines)


def _method_rows(table1: dict, cross: dict) -> list[dict]:
    cross_rows = {row["method"]: row for row in cross["summary"]}
    rows = []
    for method in METHOD_ORDER:
        source = table1["rows"][method]
        runs = source["runs"]
        cross_row = cross_rows[method]
        row = {
            "method": method,
            # Numerical artifacts created before a label-only schema correction
            # may still contain stale prose such as A3="Ours (full)".
            "label": _display_label(method, source["condition"]["label"]),
            "target_set": source["condition"]["target_set"],
            "optimization": source["condition"].get(
                "optimization_label",
                ("sequential_frozen_stage"
                 if source["condition"]["unified"] == "seq"
                 else "unified_joint_optimization")),
            "cube_pose_handling": source["condition"]["fk_to_cube"],
            "board_pose_handling": source["condition"]["fk_to_board"],
            "status": cross_row["status"],
            "converged_runs": sum(bool(run["converged"]) for run in runs),
            "total_runs": len(runs),
            "n_registered_fixed_cameras": source["n_registered_cams"],
        }
        objective_diagnostics = [
            _primary_objective_diagnostic(run) for run in runs]
        objective_stage = next((name for name, _ in objective_diagnostics
                                if name is not None), None)
        diagnostics = [diagnostic for _, diagnostic in objective_diagnostics
                       if diagnostic is not None]
        row["objective_stage"] = objective_stage
        if diagnostics:
            first_blocks = diagnostics[0]["objective_block_costs"]
            row["n_visual_residual_components"] = int(
                first_blocks["visual"]["n_residual_components"])
            row["n_fk_factor_blocks"] = int(
                first_blocks["fk"]["n_factor_blocks"])
            row["n_fk_residual_components"] = int(
                first_blocks["fk"]["n_residual_components"])
            row["final_visual_robust_cost_mean"] = _mean([
                diagnostic["objective_block_costs"]["visual"][
                    "final_robust_cost"] for diagnostic in diagnostics])
            row["final_fk_robust_cost_mean"] = _mean([
                diagnostic["objective_block_costs"]["fk"][
                    "final_robust_cost"] for diagnostic in diagnostics])
            row["final_fk_robust_cost_fraction_mean"] = _mean([
                diagnostic["objective_block_costs"]["fk"].get(
                    "fraction_of_total_robust_cost", 0.0)
                for diagnostic in diagnostics])
        else:
            row.update({
                "n_visual_residual_components": None,
                "n_fk_factor_blocks": None,
                "n_fk_residual_components": None,
                "final_visual_robust_cost_mean": None,
                "final_fk_robust_cost_mean": None,
                "final_fk_robust_cost_fraction_mean": None,
            })
        for split in ("train", "heldout"):
            for target in ("overall", "board", "cube"):
                row[f"{split}_{target}_reprojection_rmse_px"] = (
                    _reprojection_mean(runs, split, target))
                row[f"{split}_{target}_n_corners"] = _corner_count(
                    runs, split, target)
        row["all_cube_reprojection_rmse_px"] = _combined_reprojection_mean(
            runs, FINAL_TARGET)
        for target in ("overall", "board", "cube"):
            row[f"heldout_{target}_set_equal_weight_rmse_px"] = (
                _reprojection_field_mean(
                    runs, "heldout", target, "set_equal_weight_rmse_px"))
            row[f"heldout_{target}_n_corners"] = _corner_count(
                runs, "heldout", target)
            row[f"heldout_{target}_set_mean_square_px2"] = (
                _per_set_mean_squares(runs, "heldout", target))
        for scope in ("fixed_to_fixed", "gripper_to_fixed"):
            for target in TARGETS:
                for field in SCOPE_FIELDS:
                    row[f"{scope}_{target}_{field}"] = cross_row[
                        f"{scope}_{target}_{field}_mean"]
        combined_cross_view = _combined_cross_view_metrics(cross, method)
        row["cross_view_cube_pixel_transfer_rmse_px"] = combined_cross_view[
            "cross_view_pixel_transfer_rmse_px"]
        row["cam_common_cube_translation_rmse_mm"] = combined_cross_view[
            "pose_consistency_translation_rmse_mm"]
        row["cam_common_cube_rotation_rmse_deg"] = combined_cross_view[
            "pose_consistency_rotation_rmse_deg"]
        row["cross_view_cube_n_pairs"] = combined_cross_view["n_pairs"]
        row["cross_view_cube_n_directions"] = combined_cross_view[
            "n_directions"]
        row["cross_view_cube_n_destination_corners"] = combined_cross_view[
            "n_destination_corners"]
        row["cross_view_cube_support"] = (
            f"{combined_cross_view['n_pairs']} pairs "
            f"({combined_cross_view['n_fixed_to_fixed_pairs']} fixed-fixed + "
            f"{combined_cross_view['n_gripper_to_fixed_pairs']} fixed-gripper; "
            f"{combined_cross_view['n_train_fixed_anchor_pairs']} train-anchor), "
            f"{combined_cross_view['n_directions']} directions / "
            f"{combined_cross_view['n_destination_corners']} destination-corners")
        for target in ("overall", "board", "cube"):
            row[f"reference_dependent_{target}_reprojection_rmse_px"] = (
                cross_row[
                    f"reference_dependent_{target}_reprojection_rmse_px_mean"])
        rows.append(row)
    return rows


def _final_train_target(row: dict) -> str:
    if row["method"] in {"A0", "B3"}:
        return "board only (same composite-rig events)"
    if row["method"] == "B2":
        return "cube only"
    return "board+cube"


def _canonical_pose_method(row: dict) -> str:
    """Map compatibility identifiers to the canonical publication terminology."""
    return {
        "estimated": "VISION",
        "raw-FK-fixed": "FK hard fixed",
        "corrected-FK-factor": "corrected-FK soft factor",
        "vision-aligned-FK-fixed": (
            "corrected-FK hard fixed (VISION-aligned)"),
    }.get(row["cube_pose_handling"], row["cube_pose_handling"])


def _final_pose_handling(row: dict) -> str:
    if row["method"] in {"A0", "B3"}:
        return "board pose=VISION; cube=evaluation only"
    return f"cube pose={_canonical_pose_method(row)}"


def _design_target(row: dict) -> str:
    if row["method"] in {"A0", "B3"}:
        return "Board only (동일 composite-rig events)"
    if row["method"] == "B2":
        return "Cube only"
    return "Board + Cube"


def _design_optimization(row: dict) -> str:
    if row["optimization"] == "sequential_frozen_stage":
        return "Sequential (stage별 frozen)"
    return "Unified joint optimization"


def _design_pose_handling(row: dict) -> str:
    if row["method"] in {"A0", "B3"}:
        return "VISION (board pose free); Cube: evaluation only"
    pose_method = _canonical_pose_method(row)
    if pose_method == "VISION":
        return "VISION (cube pose free)"
    return pose_method


def _experiment_design_table(rows: list[dict]) -> str:
    focus = {
        "A0": "Board-only sequential baseline",
        "A1": "A0 대비 cube 관측 추가 효과",
        "A2": "A1 대비 unified feedback 효과",
        "A3": "A2 대비 FK hard fixed 효과",
        "A4": "A2 대비 corrected-FK soft factor 효과",
        "A5": "A3/A4 대비 corrected-FK hard fixed 효과",
        "B1": "A4와 같은 corrected-FK soft factor에서 sequential 효과",
        "B2": "A4 대비 board residual 제거 효과",
        "B3": "A2 대비 cube residual 제거; A0/B3 구조 대조",
    }
    lines = [
        "### 비교실험 구성",
        "",
        "| Method (방법) | Calibration 입력 target | 최적화 구조 | FK / target pose 처리 | 직접 검증하는 질문 |",
        "| --- | --- | --- | --- | --- |",
    ]
    for row in rows:
        lines.append(
            f"| {row['method']} ({row['label']}) | {_design_target(row)} | "
            f"{_design_optimization(row)} | {_design_pose_handling(row)} | "
            f"{focus[row['method']]} |")
    lines.extend([
        "",
        "> A0/B3는 calibration objective에서 cube를 완전히 가린다. 다만 "
        "평가 때는 다른 행과 동일하게 calibration을 frozen하고 train cube 관측으로 "
        "set별 evaluation pose만 맞춘 뒤 cube-only 지표를 계산한다.",
    ])
    return "\n".join(lines)


def _reprojection_result_table(rows: list[dict]) -> str:
    heldout_best = _minimum(rows, "heldout_cube_reprojection_rmse_px")
    lines = [
        "### Cube 재투영 결과",
        "",
        "모든 값은 초기화 seed 3회의 평균이다. 굵은 값은 현재 내부 "
        "Heldout Cube RMSE 최솟값이며, External GT 기반 최종 순위가 아니다.",
        "",
        "| Method (방법) | Train Cube RMSE px | ALL Cube RMSE px | "
        "Heldout Cube RMSE px | Heldout-Train px | Convergence |",
        "| --- | ---: | ---: | ---: | ---: | ---: |",
    ]
    for row in rows:
        train = row["train_cube_reprojection_rmse_px"]
        heldout = row["heldout_cube_reprojection_rmse_px"]
        gap = None if train is None or heldout is None else heldout - train
        lines.append(
            f"| {row['method']} ({row['label']}) | {_fmt(train)} | "
            f"{_fmt(row['all_cube_reprojection_rmse_px'])} | "
            f"{_fmt_best(heldout, heldout_best)} | "
            f"{_fmt(gap)} | {row['converged_runs']}/{row['total_runs']} |")
    lines.extend([
        "",
        "> `Heldout-Train`은 heldout RMSE에서 train RMSE를 뺀 진단값이다. "
        "양수면 heldout 오차가 더 크다는 뜻이지만, 음수라고 새 위치 일반화가 "
        "증명되는 것은 아니다.",
        "",
        "> `Convergence 3/3`은 세 seed에서 solver 종료 조건을 충족했다는 "
        "뜻일 뿐, 절대 정확도나 전역 최적해를 보장하지 않는다. A4의 measured "
        "FK covariance는 아직 대기 중이며, A5는 External GT 채점 전에 방법과 "
        "alignment artifact를 고정해야 한다.",
    ])
    return "\n".join(lines)


def _result_snapshot(rows: list[dict]) -> str:
    by_method = {row["method"]: row for row in rows}
    a2 = by_method["A2"]
    a3 = by_method["A3"]
    a4 = by_method["A4"]
    a5 = by_method["A5"]
    b2 = by_method["B2"]
    return "\n".join([
        "## Result at a Glance (결과 한눈에 보기)",
        "",
        "| 핵심 질문 | 현재 Session04 내부 결과 | 허용되는 해석 |",
        "| --- | --- | --- |",
        f"| 현재 내부 후보는 무엇인가 | A5: Heldout {_fmt(a5['heldout_cube_reprojection_rmse_px'])} px, "
        f"Cross-view {_fmt(a5['cross_view_cube_pixel_transfer_rmse_px'])} px, "
        f"Cam-common {_fmt(a5['cam_common_cube_translation_rmse_mm'])} mm / "
        f"{_fmt(a5['cam_common_cube_rotation_rmse_deg'])} deg | heldout과 두 camera-consistency "
        "지표에서 모두 최소인 최종 후보. 물리 정확도 1위 확정은 External GT 이후 |",
        f"| FK hard fixed는 유효한가 | A2 {_fmt(a2['heldout_cube_reprojection_rmse_px'])} "
        f"-> A3 {_fmt(a3['heldout_cube_reprojection_rmse_px'])} px | 현재 데이터에서는 "
        f"{a3['heldout_cube_reprojection_rmse_px'] - a2['heldout_cube_reprojection_rmse_px']:+.4f} "
        "px 악화되어 채택 근거가 없음 |",
        f"| corrected-FK soft factor 이득은 큰가 | A2 {_fmt(a2['heldout_cube_reprojection_rmse_px'])} "
        f"-> A4 {_fmt(a4['heldout_cube_reprojection_rmse_px'])} px | 개선은 "
        f"{a2['heldout_cube_reprojection_rmse_px'] - a4['heldout_cube_reprojection_rmse_px']:.4f} px로 "
        "작아 External GT 없이 우수성을 주장하기 어려움 |",
        f"| Train 최소가 최종 우수성을 뜻하는가 | B2 Train "
        f"{_fmt(b2['train_cube_reprojection_rmse_px'])} px, Heldout "
        f"{_fmt(b2['heldout_cube_reprojection_rmse_px'])} px | 아니오. Train RMSE는 "
        "동일 관측에 대한 in-sample fit 진단 |",
        "| 최종 결론이 확정됐는가 | External cube GT: pending | 현재는 A5를 "
        "사전 고정할 근거까지이며 최종 물리 순위는 미확정 |",
    ])


def _objective_block_table(rows: list[dict]) -> str:
    lines = [
        "## Objective Block Diagnostics (목적함수 블록 진단)",
        "",
        "| Method (방법) | FK 처리 | Visual residual components "
        "(시각 잔차 수) | FK blocks / components (FK 블록/잔차 수) | "
        "Visual robust cost (시각 비용) | FK robust cost (FK 비용) | "
        "FK cost fraction (FK 비용 비율) |",
        "| --- | --- | ---: | ---: | ---: | ---: | ---: |",
    ]
    for row in rows:
        if row["method"] not in {"A2", "A3", "A4", "A5", "B1", "B2"}:
            continue
        fk_description = _canonical_pose_method(row)
        if row["method"] in {"A3", "A5"}:
            fk_description += " (hard constant; residual 없음)"
        fraction = row["final_fk_robust_cost_fraction_mean"]
        fraction_text = (
            "N/A" if fraction is None else f"{100.0 * float(fraction):.3f}%")
        lines.append(
            f"| {row['method']} | {fk_description} | "
            f"{row['n_visual_residual_components']} | "
            f"{row['n_fk_factor_blocks']} / "
            f"{row['n_fk_residual_components']} | "
            f"{_fmt(row['final_visual_robust_cost_mean'], 2)} | "
            f"{_fmt(row['final_fk_robust_cost_mean'], 2)} | "
            f"{fraction_text} |")
    lines.extend([
        "",
        "> 이 비율은 최종 목적함수 값의 분해다. 각 항의 Jacobian과 변수 "
        "연결 구조가 다르므로, FK cost 비율을 파라미터 영향력 비율로 "
        "해석하면 안 된다.",
    ])
    return "\n".join(lines)


def _write_csv(path: Path, rows: list[dict]) -> None:
    csv_rows = _csv_rows(rows)
    with path.open("w", newline="") as handle:
        writer = csv.DictWriter(
            handle, fieldnames=list(csv_rows[0]), lineterminator="\n")
        writer.writeheader()
        writer.writerows(csv_rows)


def _csv_rows(rows: list[dict]) -> list[dict]:
    return [
        {
            "method": row["method"],
            "label": row["label"],
            "calibration_train_target": _final_train_target(row),
            "optimization": row["optimization"],
            "fk_target_pose_handling": _final_pose_handling(row),
            "train_cube_reprojection_rmse_px": row[
                "train_cube_reprojection_rmse_px"],
            "all_cube_reprojection_rmse_px": row["all_cube_reprojection_rmse_px"],
            "heldout_cube_reprojection_rmse_px": row[
                "heldout_cube_reprojection_rmse_px"],
            "heldout_cube_n_corners": row["heldout_cube_n_corners"],
            "cross_view_cube_pixel_transfer_rmse_px": row[
                "cross_view_cube_pixel_transfer_rmse_px"],
            "cam_common_cube_translation_rmse_mm": row[
                "cam_common_cube_translation_rmse_mm"],
            "cam_common_cube_rotation_rmse_deg": row[
                "cam_common_cube_rotation_rmse_deg"],
            "cross_view_cube_support": row["cross_view_cube_support"],
            "cross_view_cube_n_pairs": row["cross_view_cube_n_pairs"],
            "cross_view_cube_n_directions": row[
                "cross_view_cube_n_directions"],
            "cross_view_cube_n_destination_corners": row[
                "cross_view_cube_n_destination_corners"],
            "external_cube_gt_status": "pending",
            "converged_runs": row["converged_runs"],
            "total_runs": row["total_runs"],
            "data_status": _status_label(row["status"]),
            "n_visual_residual_components": row["n_visual_residual_components"],
            "n_fk_factor_blocks": row["n_fk_factor_blocks"],
            "n_fk_residual_components": row["n_fk_residual_components"],
            "final_visual_robust_cost_mean": row["final_visual_robust_cost_mean"],
            "final_fk_robust_cost_mean": row["final_fk_robust_cost_mean"],
            "final_fk_robust_cost_fraction_mean": row[
                "final_fk_robust_cost_fraction_mean"],
        }
        for row in rows
    ]


def _scope_table(rows: list[dict], scope: str = "cube_cross_view") -> str:
    _ = scope
    fields = (
        ("cross_view_cube_pixel_transfer_rmse_px", "Cross-view Cube px"),
        ("cam_common_cube_translation_rmse_mm", "Cam-common Cube mm"),
        ("cam_common_cube_rotation_rmse_deg", "Cam-common Cube deg"),
    )
    lines = [
        "### Cross-view Camera Consistency (카메라 간 일관성 결과, cube-only)",
        "",
        "고정카메라 pair와 고정카메라-그리퍼카메라 pair의 원시 오차를 같은 "
        "frozen mask에서 직접 pooling한다. px는 destination-corner 수로, "
        "mm/deg는 pair 수로 가중한다. 별도 pair-type 순위는 만들지 않는다.",
        "",
        "| Method (방법) | Cross-view Cube px | Cam-common Cube mm | "
        "Cam-common Cube deg | Support |",
        "| --- | ---: | ---: | ---: | --- |",
    ]
    best = {
        key: _minimum(rows, key)
        for key, _label in fields
    }
    for row in rows:
        values = []
        for key, _label in fields:
            values.append(_fmt_best(row[key], best[key]))
        lines.append("| " + " | ".join(
            [row["method"], *values, row["cross_view_cube_support"]]) + " |")
    return "\n".join(lines)


def _implementation_audit() -> str:
    return "\n".join([
        "## Code-consistency Audit (코드 일치성 검증)",
        "",
        "### 카메라 간 Relative Pose",
        "",
        "메인 A0–A5·B1–B3 optimizer에는 camera-to-camera transform을 "
        "추정·평균·연결하는 함수, observation, residual, objective term이 "
        "**0개**다. 카메라는 오직 shared target-pose variables "
        "(공유 타깃 자세 변수)를 통해 결합된다.",
        "",
        "$$T_{C_iC_j}=T_{BC_i}^{-1}T_{BC_j}$$",
        "",
        "위 transform은 solve 이후 camera pose에서 유도하는 값이다. 단, "
        "저장소 전체에 relative-pose 계산이 0개인 것은 아니다. 평가 A는 "
        "방법별 supplementary held-out consistency를 사후 계산하고, 평가 B는 "
        "메인 추정값과 독립적인 OpenCV direct relative-pose baseline을 계산한다.",
        "",
        "> **판정:** ‘cube pose로 대표 camera-relative pose를 만든 뒤 "
        "optimizer에 통합한다’는 서술은 코드와 불일치한다.",
        "",
        "### 3항 Weighted-sum Loss",
        "",
        "현재 목적함수의 additive term은 최대 **2개**다.",
        "",
        "- A0·A1·A2·A3·A5·B3: robust visual reprojection **1항**",
        "- A4·B1·B2: robust visual reprojection + whitened robust corrected-FK soft factor "
        "**2항**",
        "- `pose_error`와 `FK_constraint`: 서로 다른 두 항이 아니라 동일한 "
        "corrected-FK soft factor의 두 표현",
        "- `w1`, `w2`, `w3`: 사용하지 않음",
        "- 상대 scale: visual pixel `f_scale`과 FK covariance whitening "
        "`Sigma^(-1/2)`로 결정",
        "",
        "> **판정:** `w1·reprojection + w2·pose_error + w3·FK_constraint`는 "
        "현재 코드에 없는 부정확한 서술이다.",
        "",
        "### A3의 FK hard fixed 의미",
        "",
        r"$$T_{B\,cube}(s)=F_s^{raw}T_{cube\ center\rightarrow object}^{mech}$$",
        "",
        "A3가 고정하는 cube pose는 set별 controller FK pose에 영상과 "
        "무관하게 사전 등록한 mechanical frame map `R_y(180°)`를 적용한 "
        "pose다. cube-center 원점 이동은 0이고, `Delta_train`이나 corrected-FK "
        "artifact를 사용하지 않는다. A3 최종 optimizer에서는 이 pose를 "
        "상수로 고정하고 visual reprojection 1항만 최소화한다.",
        "",
        "> **판정:** A3는 FK hard constraint이지만 external GT는 "
        "아니다. tool4/CAD frame 정의 오차가 그대로 결과에 들어간다.",
        "",
        "### A5의 corrected-FK hard fixed (VISION-aligned) 의미",
        "",
        r"$$T_{B\,cube}(s)=F_s^{raw}\Delta_{train}$$",
        "",
        "A5는 board와 held-out을 제외한 train eye-in-hand cube 영상으로 "
        "추정한 `Delta_train`을 적용한 뒤 set별 cube pose를 상수로 "
        "고정한다. A4와 동일한 corrected-FK artifact를 사용하지만 A4처럼 "
        "covariance factor로 완화하지 않는다.",
        "",
        "> **판정:** A5는 train-only corrected-FK를 hard fixed로 쓰는 최종 후보 "
        "방법으로 둘 수 있다. 단, External GT 공개 전에 alignment artifact와 "
        "평가 코드를 frozen해야 한다.",
    ])


def _delta_text(first, second) -> str:
    if first is None or second is None:
        return "N/A"
    return f"{_fmt(first)} -> {_fmt(second)} ({float(second) - float(first):+.4f})"


def _heldout_pair(by_method: dict[str, dict], first: str, second: str,
                  targets: tuple[str, ...]) -> str:
    values = []
    for target in targets:
        key = f"heldout_{target}_reprojection_rmse_px"
        values.append(
            f"{target.capitalize()} {_delta_text(by_method[first][key], by_method[second][key])}")
    return "; ".join(values)


def _contrast_definitions() -> list[tuple[str, str, str, str, str, tuple[str, ...], str]]:
    return [
        (
            "Final protocol",
            "A0",
            "B3",
            "A0 -> B3",
            "단일 target에서 sequential과 unified가 사실상 같아지는가",
            ("cube",),
            "구조 구현의 negative control이다. 현재 0.0004 px 차이로 기대한 동등성을 지지한다.",
        ),
        (
            "Final protocol",
            "A0",
            "A1",
            "A0 -> A1",
            "같은-event board-only baseline에 cube train 관측을 추가하면 cube 평가가 개선되는가",
            ("cube",),
            "현재는 0.0171 px 악화로 내부 개선 근거가 없다. External GT로 재판정한다.",
        ),
        (
            "Final protocol",
            "A1",
            "A2",
            "A1 -> A2",
            "VISION 조건에서 unified feedback이 도움이 되는가",
            ("cube",),
            "현재 0.0978 px 개선으로 unified feedback을 약하게 지지한다.",
        ),
        (
            "Final protocol",
            "B3",
            "A2",
            "B3 -> A2",
            "unified 구조에서 cube residual이 최종 cube 평가에 필요한가",
            ("cube",),
            "현재 0.0803 px 개선으로 cube residual의 내부 이득을 약하게 지지한다.",
        ),
        (
            "Final protocol",
            "A2",
            "A3",
            "A2 -> A3",
            "VISION cube pose를 FK hard fixed로 바꾸면 어떤가",
            ("cube",),
            "현재 3.1239 px 악화되어 FK hard fixed를 반박한다.",
        ),
        (
            "Final protocol",
            "B1",
            "A4",
            "B1 -> A4",
            "같은 corrected-FK soft factor에서 sequential과 unified 중 무엇이 나은가",
            ("cube",),
            "현재 0.1084 px 개선으로 unified 구조를 약하게 지지한다.",
        ),
        (
            "Final protocol",
            "A2",
            "A4",
            "A2 -> A4",
            "Unified VISION에 corrected-FK soft factor를 추가하면 이득이 있는가",
            ("cube",),
            "현재 개선은 0.0174 px로 작아 corrected-FK 우수성 근거로 부족하다.",
        ),
        (
            "Final protocol",
            "B2",
            "A4",
            "B2 -> A4",
            "corrected-FK soft factor 조건에서 board residual이 cube 보정에 도움 되는가",
            ("cube",),
            "현재 0.8821 px 개선으로 board residual의 내부 이득을 지지한다.",
        ),
        (
            "Final protocol",
            "A3",
            "A5",
            "A3 -> A5",
            "FK hard fixed와 corrected-FK hard fixed의 차이는 무엇인가",
            ("cube",),
            "현재 3.3019 px 개선으로 raw frame mismatch 보정 필요성을 지지한다.",
        ),
        (
            "Final protocol",
            "A4",
            "A5",
            "A4 -> A5",
            "같은 corrected-FK를 soft factor와 hard fixed로 쓰면 무엇이 달라지는가",
            ("cube",),
            "현재 0.1606 px 개선으로 A5를 External GT 전 고정할 후보로 둔다.",
        ),
    ]


def _matched_contrast_records(rows: list[dict]) -> list[tuple[str, str, str, str, str, str]]:
    by_method = {row["method"]: row for row in rows}
    records = []
    for tier, first, second, label, question, targets, decision in _contrast_definitions():
        metric = "External cube GT + heldout cube RMSE"
        result = _heldout_pair(by_method, first, second, targets)
        records.append((tier, label, question, metric, result, decision))
    return records


def _matched_contrast_table(rows: list[dict]) -> str:
    contrasts = _matched_contrast_records(rows)
    lines = [
        "## Matched Contrast Decision Table (비교실험 구성 확정표)",
        "",
        "최종 비교는 아래 contrast만 사용한다. 모든 heldout 평가는 cube만 "
        "보며, External GT가 들어오면 같은 cube pose list에서 paired "
        "comparison으로 판정한다.",
        "",
        "> 검증 질문은 새 45-event 최종 설계 기준이다. 아래 Session04 delta는 "
        "대응되는 legacy preflight이며, 새 촬영 후 같은 contrast를 다시 계산해야 한다.",
        "",
        "음수 delta는 두 번째 방법의 Heldout Cube RMSE가 개선됐다는 뜻이다.",
        "",
        "| Direct Contrast (직접 비교) | Question (검증 질문) | "
        "Session04 Heldout Cube 결과 | Decision (판정) |",
        "| --- | --- | --- | --- |",
    ]
    for _tier, contrast, question, _metric, result, decision in contrasts:
        lines.append(
            f"| {contrast} | {question} | {result} | {decision} |")
    lines.extend([
        "",
        "> A5는 External GT 공개 전에 방법·파라미터·alignment artifact가 "
        "frozen이면 최종 후보로 비교할 수 있다. GT를 본 뒤 A5를 정의하면 "
        "사후 진단으로만 남긴다.",
    ])
    return "\n".join(lines)


def _first_support(rows: list[dict], target: str) -> int | None:
    for row in rows:
        value = row.get(f"heldout_{target}_n_corners")
        if value is not None:
            return int(value)
    return None


def _first_split_support(rows: list[dict], split: str, target: str) -> int | None:
    for row in rows:
        value = row.get(f"{split}_{target}_n_corners")
        if value is not None:
            return int(value)
    return None


def _metric_decision_records(rows: list[dict],
                             data_warnings: dict) -> list[tuple[str, str, str, str, str]]:
    cube_corners = _first_support(rows, "cube")
    train_cube_corners = _first_split_support(rows, "train", "cube")
    all_cube_corners = (
        None if train_cube_corners is None or cube_corners is None
        else train_cube_corners + cube_corners)
    cameras = ", ".join(str(value) for value in data_warnings.get(
        "evaluation_fixed_camera_intersection", [])) or "N/A"
    return [
        (
            "External cube TRE / rotation / P95 / failure",
            "최종 물리 정확도 주 지표",
            "GT 공개 전에 저장한 blind cube pose와 동일 pose ID의 독립 GT를 paired 비교한다. "
            "translation norm, SO(3) geodesic rotation, P95, 사전 정의 failure rate를 계산한다.",
            "모든 방법에 같은 GT pose list와 failure threshold를 적용한다. GT 측정 "
            "uncertainty floor보다 작은 차이는 주장하지 않는다.",
            "Pending; External GT 추가 후 산출",
        ),
        (
            "Heldout Cube RMSE px",
            "External GT 전 내부 보조 지표",
            "train cube만으로 set별 evaluation pose를 맞춘 뒤 calibration과 pose를 "
            "frozen하고, 미사용 heldout cube corner를 재투영한다.",
            "모든 방법에 같은 cube corner와 split을 쓰고 test-time refit을 금지한다. "
            "같은 set의 다른 event이므로 새 위치 일반화나 물리 GT는 아니다.",
            f"동일 heldout cube {cube_corners or 'N/A'} corners",
        ),
        (
            "Cross-view pixel transfer RMSE",
            "카메라 간 pixel 일관성 보조 지표",
            "한 카메라의 cube PnP pose를 상대 카메라로 전달하고 양방향 "
            "destination-corner squared error를 직접 pooling해 px RMSE를 계산한다.",
            "모든 방법에 동일한 양방향 pair mask를 쓴다. fixed-gripper pair에는 "
            "Hand-Eye/FK와 train fixed-anchor가 섞이며 공통 systematic error를 검출하지 못한다.",
            rows[0].get("cross_view_cube_support", f"fixed cameras {cameras}"),
        ),
        (
            "Cam-common Obj-Cam consistency mm/deg",
            "카메라 간 3D pose 일관성 보조 지표",
            "같은 frozen cube pair에서 두 camera 경로가 계산한 `T_base_cube`의 "
            "translation norm과 SO(3) rotation 차이를 pair-pooled RMSE로 계산한다.",
            "Cross-view px와 동일 pair discrepancy의 다른 단위 표현이므로 독립 "
            "증거가 아니며 공통 systematic error를 검출하지 못한다.",
            rows[0].get("cross_view_cube_support", "same frozen cube pair mask"),
        ),
        (
            "ALL Cube RMSE px",
            "전체 fit sanity check",
            "train cube로 맞춘 set별 evaluation pose와 frozen calibration을 "
            "train+heldout cube corner 전체에 적용해 corner-pooled RMSE를 계산한다.",
            "모든 방법에 같은 cube 모집단을 쓰지만 train과 heldout을 섞고 train이 "
            "약 75%를 차지하므로 일반화 순위 지표가 아니다.",
            f"train+heldout cube {all_cube_corners or 'N/A'} corners "
            f"({train_cube_corners or 'N/A'} + {cube_corners or 'N/A'})",
        ),
        (
            "Train Cube RMSE px",
            "Train-split fit 진단",
            "frozen calibration에서 train cube로 set별 evaluation pose를 맞춘 뒤 "
            "같은 train cube corner를 재투영한다.",
            "모든 방법에 같은 cube 모집단을 쓰지만 pose를 맞춘 관측을 다시 채점하는 "
            "in-sample 값이므로 방법 순위 지표가 아니다.",
            f"동일 train cube {train_cube_corners or 'N/A'} corners",
        ),
    ]


def _metric_decision_table(rows: list[dict], data_warnings: dict) -> str:
    metric_rows = _metric_decision_records(rows, data_warnings)
    lines = [
        "## Metric Decision Matrix (평가지표 판정표)",
        "",
        "### 평가지표 (한글로): 설명, 평가 지표 낸 방법",
        "",
        "| Metric (평가지표) | 역할 | 계산 방법 | 공정성 통제와 한계 | "
        "Current Support (현재 근거) |",
        "| --- | --- | --- | --- | --- |",
    ]
    for metric, tier, use, limit, support_text in metric_rows:
        lines.append(
            f"| {metric} | {tier} | {use} | {limit} | {support_text} |")
    lines.extend([
        "",
        "### 공통 계산 규칙",
        "",
        "$$RMSE_{px}=\\sqrt{\\frac{1}{2N}\\sum_k((u_k-\\hat u_k)^2+(v_k-\\hat v_k)^2)}$$",
        "",
        "$$T^{B,(i)}_{cube}=T^B_{C_i}T^{C_i}_{cube,\\mathrm{PnP}},\\qquad "
        "T^{B,(g)}_{cube}(e)=T^B_G(e)T^G_{C_g}T^{C_g}_{cube,\\mathrm{PnP}}$$",
        "",
        "$$e_t=\\lVert t_{pred}-t_{GT}\\rVert_2,\\qquad "
        "e_R=\\cos^{-1}((\\operatorname{tr}(R_{GT}^{T}R_{pred})-1)/2)$$",
        "",
        "- 카메라와 hand-eye transform은 방법별 calibration 종료 후 frozen한다.",
        "- Heldout cube corner는 calibration, evaluation-pose fit, threshold 선택에 쓰지 않는다.",
        "- 내부 지표는 같은 support를 직접 pooling하고, seed 3개 평균은 반복 실험 표본으로 해석하지 않는다.",
        "- px, mm, deg는 서로 합치지 않고 각각 별도 열로 보고한다.",
    ])
    return "\n".join(lines)


def _internal_only_claim_envelope() -> str:
    return "\n".join([
        "## Final Protocol Lock (최종 단일 기준)",
        "",
        "| 항목 | 최종 고정 기준 | 공정성 이유 |",
        "| --- | --- | --- |",
        "| 비교 행 | A0~A5, B1~B3 한 벌만 사용 | 같은 row 정의를 모든 데이터와 문서에서 유지 |",
        "| 촬영 예산 | 새 촬영은 composite rig 45 planned events와 동일 pose ID 사용 | 검출 marker 수가 아니라 raw capture opportunity를 동일하게 통제 |",
        "| Marker ablation | 같은 raw image에서 row별 board/cube observation만 사전 정의대로 masking | A0/B3에 유리한 별도 board 촬영을 추가하지 않음 |",
        "| 평가 target | heldout, cross-view, External GT 모두 cube-only | 모든 row를 동일한 실제 3D target으로 평가 |",
        "| 최종 주 지표 | External cube TRE / rotation / P95 / failure | 내부 재투영 오차로 물리 정확도를 확정하지 않음 |",
        "| 내부 보조 지표 | Heldout Cube, Cross-view, Cam-common; Train/ALL은 fit 진단 | 같은 support와 frozen transform으로 계산 |",
        "| A5 해석 | External GT 공개 전에 방법, 파라미터, alignment artifact 고정 | GT를 본 뒤 방법을 선택하는 사후 편향 방지 |",
        "",
        "> 이 표는 **새 최종 촬영의 계약**이다. 아래 결과 수치는 이 계약 적용 전 "
        "legacy Session04 내부 데이터이므로 설계 검증용 preflight로만 사용한다.",
    ])


def _markdown(rows: list[dict], marker: dict, detailed: bool,
              data_warnings: dict | None = None,
              session_label: str = "Session") -> str:
    _ = marker
    data_warnings = data_warnings or {}
    title = (
        f"# {session_label} Calibration Evaluation (캘리브레이션 평가)"
        if detailed else f"# {session_label} Table 1 Results (표 1 결과)")
    lines = [
        title,
        "",
        "> Status: Final protocol before External GT. 비교 행은 A0~A5, "
        "B1~B3 한 벌만 사용하고, heldout 평가는 항상 cube만 본다.",
        "",
        _result_snapshot(rows),
        "",
        _markdown_data_warnings(data_warnings),
        "",
        _internal_only_claim_envelope(),
        "",
        "## Final Comparison Table (최종 ABLATION_TEST)",
        "",
        _experiment_design_table(rows),
        "",
        _reprojection_result_table(rows),
        "",
        _scope_table(rows),
        "",
        _metric_decision_table(rows, data_warnings),
        "",
        _matched_contrast_table(rows),
        "",
        _objective_block_table(rows),
        "",
        "## Terminology (용어 설명)",
        "",
        "- **$T^B_{C_i}$, Base-to-Fixed-Camera Transform "
        "(베이스–고정카메라 변환)**: 고정카메라 외부 파라미터.",
        "- **$T^G_{C_g}$, Hand–Eye Transform (핸드–아이 변환)**: "
        "그리퍼에서 그리퍼카메라로의 변환.",
        "- **$T^B_G(e)$, Robot FK Pose (이벤트별 로봇 순기구학 자세)**: "
        "이벤트 $e$의 베이스–그리퍼 변환이며 평가 중 고정 입력이다.",
        "- **PnP, Perspective-n-Point (3D–2D 자세 추정)**: 3D 표적점과 "
        "2D 영상점으로 카메라–표적 자세를 계산한다.",
        "- **RMSE, Root Mean Squared Error (평균제곱근오차)**: 잔차 "
        "제곱 평균의 제곱근. px, mm, deg는 서로 합치지 않는다.",
        "- **External cube GT**: GT 공개 전 저장한 blind prediction과 "
        "독립 cube GT pose를 비교하는 최종 주 지표.",
    ]
    if detailed:
        lines.extend([
            "",
            "## External GT Task (다음주 예정 태스크)",
            "",
            "Independent External GT가 들어오면 모든 row의 cube pose prediction을 "
            "같은 GT cube pose list와 비교한다. 최종 결과는 Translation Error, "
            "Rotation Error, P95, Failure Rate로 산출한다.",
        ])
    return "\n".join(lines) + "\n"


def _html_data_warnings(data_warnings: dict) -> str:
    support = data_warnings.get("support", {})
    overall = support.get("overall", {})
    board = support.get("board", {})
    cube = support.get("cube", {})
    cameras = ", ".join(str(value) for value in data_warnings.get(
        "evaluation_fixed_camera_intersection", [])) or "N/A"
    dropped = ", ".join(data_warnings.get("dropped_sets", [])) or "none"
    eligible = data_warnings.get("eligible_sets", [])
    items = [
        f"Evaluation support: fixed cameras <code>{escape(cameras)}</code>, "
        f"overall {overall.get('n_observations', 'N/A')} obs / "
        f"{overall.get('n_corners', 'N/A')} corners; board "
        f"{board.get('n_observations', 'N/A')} / "
        f"{board.get('n_corners', 'N/A')}, cube "
        f"{cube.get('n_observations', 'N/A')} / "
        f"{cube.get('n_corners', 'N/A')}.",
        f"Split support: {len(eligible)} eligible sets; dropped sets "
        f"<code>{escape(dropped)}</code>.",
    ]
    quality = data_warnings.get("cube_quality", {})
    counts = quality.get("counts", {})
    selected = quality.get("selected_quality_tier_counts", {})
    if counts:
        items.append(
            "Cube detection: "
            f"{counts.get('images_read', 'N/A')} images read, "
            f"{counts.get('accepted_observations', 'N/A')} accepted PnP observations, "
            f"{selected.get('nonplanar_multiface', 'N/A')} core multiface selected, "
            f"{counts.get('pnp_rmse_rejections', 'N/A')} PnP-RMSE rejections.")
    conflict = data_warnings.get("board_cube_conflict", {})
    if conflict:
        items.append(
            "Board-Cube conflict: direct PnP disagreement is "
            f"{_fmt(conflict.get('translation_rmse_mm'))} mm translation RMSE "
            f"and {_fmt(conflict.get('maximum_rotation_deg'))} deg max rotation; "
            "joint solve mitigates it but does not remove the cause.")
    return (
        '<section class="panel warning"><h2>Current Data Warnings '
        '(현재 데이터 경고)</h2><ul>'
        + "".join(f"<li>{item}</li>" for item in items)
        + "</ul></section>"
    )


def _html_matched_contrast(rows: list[dict]) -> str:
    body = []
    for tier, contrast, question, metric, result, decision in _matched_contrast_records(rows):
        body.append(
            "<tr>"
            f"<td>{escape(tier)}</td>"
            f"<td>{escape(contrast)}</td>"
            f"<td>{escape(question)}</td>"
            f"<td>{escape(metric)}</td>"
            f"<td>{escape(result)}</td>"
            f"<td>{escape(decision)}</td></tr>")
    return f"""
<section class="panel"><h2>Matched Contrast Decision Table (비교실험 구성 확정표)</h2>
<p>모든 행을 하나의 전체 순위로 세우지 않고, 한 번에 한 요소만 달라지는 contrast만 해석합니다.</p>
<div class="table-wrap"><table>
<thead><tr><th>Tier (구분)</th><th>Direct Contrast (직접 비교)</th>
<th>Question (검증 질문)</th><th>Primary Metric (주 지표)</th>
<th>Session04 Result</th><th>Decision (판정)</th></tr></thead>
<tbody>{''.join(body)}</tbody></table></div></section>"""


def _html_metric_decision(rows: list[dict], data_warnings: dict) -> str:
    body = []
    for metric, tier, use, limit, support in _metric_decision_records(
            rows, data_warnings):
        body.append(
            "<tr>"
            f"<td>{escape(metric)}</td>"
            f"<td>{escape(tier)}</td>"
            f"<td>{escape(use)}</td>"
            f"<td>{escape(limit)}</td>"
            f"<td>{escape(support)}</td></tr>")
    return f"""
<section class="panel"><h2>Metric Decision Matrix (평가지표 판정표)</h2>
<div class="table-wrap"><table>
<thead><tr><th>Metric (지표)</th><th>Tier (등급)</th><th>Use (사용법)</th>
<th>Limit (제한)</th><th>Current Support (현재 근거)</th></tr></thead>
<tbody>{''.join(body)}</tbody></table></div></section>"""


def _html_korean_metric_method() -> str:
    body = []
    for metric, description, method in [
        (
            "External cube TRE / rotation / P95 / failure",
            "최종 물리 정확도 지표",
            "GT 공개 전에 각 방법의 cube pose prediction을 저장하고, 다음주 독립 External cube GT와 같은 pose list에서 translation, rotation, P95, failure를 계산한다.",
        ),
        (
            "ALL Cube RMSE px",
            "전체 cube 영상에 대한 fit sanity check",
            "카메라/hand-eye를 고정하고 train cube로 set별 T_base_cube를 맞춘 뒤 train 724 + heldout 236 cube corner를 합친다. Train이 75.4%이므로 일반화 순위에는 쓰지 않는다.",
        ),
        (
            "Train Cube RMSE px",
            "동일 train cube 모집단의 fit 진단",
            "모든 row에서 calibration을 frozen하고 train cube로 set별 evaluation pose를 맞춘 뒤 동일한 724개 train cube corner를 재투영한다. In-sample fit이므로 순위용 지표가 아니다.",
        ),
        (
            "Heldout Cube RMSE px",
            "External GT 전 내부 보조 지표",
            "ALL Cube와 같은 train-only cube pose source를 사용하되 calibration과 pose fit에 쓰지 않은 동일한 236개 heldout cube corner를 corner-pooled RMSE로 계산한다.",
        ),
        (
            "Cross-view pixel transfer RMSE px",
            "카메라 간 pixel 일관성",
            "동일한 36개 pair(9 fixed-fixed + 27 fixed-gripper)의 72개 방향, 904개 destination-corner를 직접 pooling한다. fixed-gripper 중 18개는 train fixed-anchor를 쓴다.",
        ),
        (
            "Cam-common Obj-Cam consistency mm/deg",
            "카메라 간 3D pose 일관성",
            "같은 frozen pair의 두 경로가 만든 T_base_cube 차이를 36개 pair에 직접 pooling한다. fixed-gripper에는 Hand-Eye/FK가 포함되며 Cross-view px와 독립 증거가 아니다.",
        ),
    ]:
        body.append(
            "<tr>"
            f"<td>{escape(metric)}</td>"
            f"<td>{escape(description)}</td>"
            f"<td>{escape(method)}</td></tr>")
    return f"""
<section class="panel"><h2>평가지표 (한글로): 설명, 평가 지표 낸 방법</h2>
<div class="table-wrap"><table>
<thead><tr><th>평가지표</th><th>설명</th><th>평가 지표 낸 방법</th></tr></thead>
<tbody>{''.join(body)}</tbody></table></div>
<p>A0/B3는 calibration 단계에서는 cube를 쓰지 않습니다. Cube train 관측은 카메라/hand-eye를 다시 맞추지 않고, cube RMSE 계산을 위한 set별 evaluation pose만 맞추는 데 사용합니다.</p>
</section>"""


def _html(rows: list[dict], marker: dict,
          data_warnings: dict | None = None,
          session_label: str = "Session") -> str:
    _ = marker
    data_warnings = data_warnings or {}
    cube_best = _minimum(rows, "heldout_cube_reprojection_rmse_px")

    method_rows = []
    for row in rows:
        cam_common_mm_deg = (
            f"{_fmt(row['cam_common_cube_translation_rmse_mm'])} / "
            f"{_fmt(row['cam_common_cube_rotation_rmse_deg'])}"
        )
        method_rows.append(
            "<tr>"
            f"<td>{escape(row['method'])}</td>"
            f"<td>{escape(row['label'])}</td>"
            f"<td>{escape(_final_train_target(row))}</td>"
            f"<td>{escape(row['optimization'])}</td>"
            f"<td>{escape(_final_pose_handling(row))}</td>"
            f"<td>{_fmt(row['train_cube_reprojection_rmse_px'])}</td>"
            f"<td>{_fmt(row['all_cube_reprojection_rmse_px'])}</td>"
            f"<td>{_fmt_best(row['heldout_cube_reprojection_rmse_px'], cube_best, html=True)}</td>"
            f"<td>{_fmt(row['cross_view_cube_pixel_transfer_rmse_px'])}</td>"
            f"<td>{escape(cam_common_mm_deg)}</td>"
            "<td>Pending</td>"
            f"<td>{row['converged_runs']}/{row['total_runs']}</td>"
            f"<td>{escape(_status_label(row['status']))}</td></tr>")

    consistency_rows = []
    consistency_fields = (
        "cross_view_cube_pixel_transfer_rmse_px",
        "cam_common_cube_translation_rmse_mm",
        "cam_common_cube_rotation_rmse_deg",
    )
    best = {
        key: _minimum(rows, key)
        for key in consistency_fields
    }
    for row in rows:
        values = []
        for key in consistency_fields:
            values.append(_fmt_best(row[key], best[key], html=True))
        consistency_rows.append(
            "<tr>"
            f"<td>{escape(row['method'])}</td>"
            + "".join(f"<td>{value}</td>" for value in values)
            + f"<td>{escape(row['cross_view_cube_support'])}</td>"
            + "</tr>")
    return f"""<!doctype html>
<html lang="ko"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>{escape(session_label)} Calibration Evaluation (캘리브레이션 평가)</title>
<style>
:root{{--ink:#18212f;--muted:#627083;--line:#dce3ea;--paper:#f5f7fa;--card:#fff;--blue:#2066c7;--teal:#087f78}}
*{{box-sizing:border-box}} body{{margin:0;background:var(--paper);color:var(--ink);font:15px/1.55 system-ui,-apple-system,sans-serif}}
main{{max-width:1180px;margin:auto;padding:32px 20px 72px}} h1{{font-size:30px;margin:0 0 8px}} h2{{font-size:20px;margin:0 0 16px}} h3{{font-size:16px;margin:20px 0 6px}}
.subtitle{{color:var(--muted);margin-bottom:22px}} .badge{{display:inline-block;background:#fff3cd;color:#755600;border:1px solid #efd582;border-radius:999px;padding:5px 11px;font-weight:700}}
.panel{{background:var(--card);border:1px solid var(--line);border-radius:8px;padding:20px;margin-top:18px;box-shadow:0 4px 14px #25364d0d}}
.table-wrap{{overflow:auto}} table{{border-collapse:collapse;width:100%;min-width:680px}} th,td{{padding:9px 10px;border-bottom:1px solid var(--line);text-align:right;white-space:nowrap}} th:first-child,td:first-child{{text-align:left}} th{{color:var(--muted);font-size:12px}}
.method-section p{{color:var(--muted);margin:0 0 8px}} .warning{{border-left:4px solid #c05621}} .warning li{{margin:6px 0}} .note{{border-left:4px solid var(--teal);padding-left:14px}} code{{background:#edf2f7;padding:2px 5px;border-radius:5px}}
@media(max-width:760px){{.grid{{grid-template-columns:1fr}} main{{padding:22px 12px 50px}}}}
</style></head><body><main>
<span class="badge">Final protocol before External GT</span>
<h1>{escape(session_label)} Calibration Evaluation (캘리브레이션 평가)</h1>
<p class="subtitle">비교 행은 A0~A5, B1~B3 한 벌만 사용합니다. Heldout 평가는 항상 cube만 보며, 최종 순위는 External cube GT로 정합니다.</p>
{_html_data_warnings(data_warnings or {})}
<section class="panel note"><h2>Final Protocol Lock (최종 단일 기준)</h2>
<p>A0/B3는 calibration 단계에서는 board-only로 유지합니다. Cube 평가지표는 모든 row에서 train cube 관측으로 set별 evaluation pose만 맞춘 뒤, calibration 결과를 frozen한 상태로 계산합니다.</p></section>
<section class="panel"><h2>Final Comparison Table (최종 ABLATION_TEST)</h2>
<div class="table-wrap"><table>
<thead><tr><th>Method</th><th>Label</th><th>Calibration train target</th><th>Optimization</th><th>FK / target-pose 처리</th><th>Train Cube RMSE px</th><th>ALL Cube RMSE px</th><th>Heldout Cube RMSE px</th><th>Cross-view Cube px</th><th>Cam-common Cube mm/deg</th><th>External GT TRE/Rot/P95/Fail</th><th>Convergence</th><th>Data status</th></tr></thead>
<tbody>{''.join(method_rows)}</tbody></table></div></section>
{_html_korean_metric_method()}
{_html_matched_contrast(rows)}
{_html_metric_decision(rows, data_warnings)}
<section class="panel"><h2>Cross-view Camera Consistency (cube-only)</h2>
<div class="table-wrap"><table>
<thead><tr><th>Method</th><th>Cross-view Cube px</th><th>Cam-common Cube mm</th><th>Cam-common Cube deg</th><th>Support</th></tr></thead>
<tbody>{''.join(consistency_rows)}</tbody></table></div></section>
<section class="panel note"><h2>Interpretation (해석)</h2><p>Cross-view pixel transfer와 Cam-common Obj-Cam consistency는 같은 frozen pair discrepancy를 px와 mm/deg로 나타낸 상관된 보조 지표입니다. Fixed-gripper pair에는 Hand-Eye, Robot FK, train fixed-anchor가 섞이고 공통 systematic error는 잡지 못하므로 최종 주장은 External cube GT로만 결정합니다.</p></section>
<section class="panel note"><h2>External GT Task (다음주 예정 태스크)</h2><p>Independent External GT가 들어오면 모든 row의 cube pose prediction을 같은 GT cube pose list와 비교해 Translation Error, Rotation Error, P95, Failure Rate를 산출합니다.</p></section>
<section class="panel"><h2>Terminology (용어 설명)</h2><ul>
<li><b>PnP, Perspective-n-Point (3D–2D 자세 추정)</b>: 영상 코너로 카메라–표적 자세를 계산합니다.</li>
<li><b>FK, Forward Kinematics (순기구학)</b>: 이벤트별 Base-to-Gripper Transform (베이스–그리퍼 변환)을 계산합니다.</li>
<li><b>Hand–Eye Transform (핸드–아이 변환)</b>: Gripper-to-Camera Transform (그리퍼–카메라 변환)입니다.</li>
<li><b>RMSE, Root Mean Squared Error (평균제곱근오차)</b>: px, mm, deg 단위를 분리해 해석합니다.</li>
<li><b>External cube GT</b>: GT 공개 전 저장한 blind prediction과 독립 cube GT pose를 비교하는 최종 주 지표입니다.</li>
</ul></section>
</main></body></html>"""


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Regenerate current CSV/Markdown/HTML evaluation artifacts")
    parser.add_argument(
        "--table1", default="ABLATION_TEST_result/session04/ABLATION_TEST_table1/ABLATION_TEST_table1_methods.json")
    parser.add_argument(
        "--cross", default=(
            "ABLATION_TEST_result/session04/cross_target_evaluation/"
            "cross_target_evaluation.json"))
    parser.add_argument(
        "--marker", default=(
            "ABLATION_TEST_result/session04/marker_system_end_to_end/"
            "marker_system_end_to_end.json"))
    parser.add_argument(
        "--late_dir", default="ABLATION_TEST_result/session04/ABLATION_TEST_table1")
    parser.add_argument(
        "--html",
        default="ABLATION_TEST_result/session04/ABLATION_TEST_table1/ABLATION_TEST_TABLE1_INTERACTIVE.html")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    table1_path, cross_path, marker_path = map(
        Path, (args.table1, args.cross, args.marker))
    table1, cross, marker = map(_load, (table1_path, cross_path, marker_path))
    if cross.get("artifact_schema") != "internal_heldout_evaluation_v8":
        raise RuntimeError("cross-target result is not the current v8 schema")
    if marker.get("artifact_schema") != "marker_system_end_to_end_v5":
        raise RuntimeError("marker-system result is not the current v5 schema")
    metric_scale = table1.get("protocol", {}).get("board_metric_scale", {})
    if metric_scale.get("enabled") is not False or float(
            metric_scale.get("scale", 1.0)) != 1.0:
        raise RuntimeError(
            "canonical reports require nominal metric geometry; inferred "
            "scale alignment belongs in a separate diagnostic output")
    if tuple(table1.get("rows", {})) != METHOD_ORDER:
        raise RuntimeError("Table 1 method order or support is incomplete")
    if tuple(row["system"] for row in marker.get("summary", [])) != SYSTEM_ORDER:
        raise RuntimeError("marker-system support is incomplete")

    dataset = str(table1.get("protocol", {}).get("dataset", ""))
    session_name = next(
        (part for part in reversed(Path(dataset).parts)
         if part.lower().startswith("session")),
        "session",
    )
    session_label = session_name[0].upper() + session_name[1:]

    rows = _method_rows(table1, cross)
    data_warnings = _data_warnings(table1, cross)
    late_dir = Path(args.late_dir)
    late_dir.mkdir(parents=True, exist_ok=True)
    _write_csv(late_dir / "ABLATION_TEST_table1_results.csv", rows)
    (late_dir / "ABLATION_TEST_TABLE1_RESULTS.md").write_text(
        _markdown(
            rows, marker, detailed=True, data_warnings=data_warnings,
            session_label=session_label))
    Path(args.html).write_text(
        _html(rows, marker, data_warnings=data_warnings,
              session_label=session_label))
    print("[DONE] Generated current CSV, Markdown, and HTML artifacts")


if __name__ == "__main__":
    main()
