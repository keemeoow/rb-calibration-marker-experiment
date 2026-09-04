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
    "A3": "raw-FK hard fixed",
    "A4": "corrected-FK soft factor",
    "A5": "vision-aligned FK hard fixed",
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


def _reprojection_mean(runs: list[dict], split: str,
                       target: str) -> float | None:
    key = f"{split}_reprojection"
    return _mean([
        None if run[key].get(target) is None
        else run[key][target]["rmse_px"]
        for run in runs
    ])


def _reprojection_field_mean(runs: list[dict], split: str, target: str,
                             field: str) -> float | None:
    key = f"{split}_reprojection"
    return _mean([
        None if run[key].get(target) is None
        else run[key][target].get(field)
        for run in runs
    ])


def _corner_count(runs: list[dict], split: str, target: str) -> int | None:
    """Corner counts are frozen by the split, so every seed reports the same."""
    for run in runs:
        entry = run[f"{split}_reprojection"].get(target)
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
            entry = run[f"{split}_reprojection"].get(target)
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
        entry = run[f"{split}_reprojection"].get(target)
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
        f"- Evaluation support: fixed cameras `{cameras}`, overall "
        f"{overall.get('n_observations', 'N/A')} obs / "
        f"{overall.get('n_corners', 'N/A')} corners; board "
        f"{board.get('n_observations', 'N/A')} / "
        f"{board.get('n_corners', 'N/A')}, cube "
        f"{cube.get('n_observations', 'N/A')} / "
        f"{cube.get('n_corners', 'N/A')}.",
        f"- Split support: {len(eligible)} eligible sets; dropped sets `{dropped}`.",
    ]
    if counts:
        lines.append(
            "- Cube detection: "
            f"{counts.get('images_read', 'N/A')} images read, "
            f"{counts.get('accepted_observations', 'N/A')} accepted PnP observations, "
            f"{selected.get('nonplanar_multiface', 'N/A')} core multiface selected, "
            f"{counts.get('pnp_rmse_rejections', 'N/A')} PnP-RMSE rejections.")
    if conflict:
        lines.append(
            "- Board-Cube conflict: direct PnP disagreement is "
            f"{_fmt(conflict.get('translation_rmse_mm'))} mm translation RMSE "
            f"and {_fmt(conflict.get('maximum_rotation_deg'))} deg max rotation; "
            "joint solve mitigates it but does not remove the cause.")
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
        row["cross_view_cube_pixel_transfer_rmse_px"] = _weighted_rmse(
            (
                row["fixed_to_fixed_cube_cross_view_pixel_transfer_rmse_px"],
                cross_row.get("n_fixed_to_fixed_cube_directions"),
            ),
            (
                row["gripper_to_fixed_cube_cross_view_pixel_transfer_rmse_px"],
                cross_row.get("n_gripper_to_fixed_cube_directions"),
            ),
        )
        row["cam_common_cube_translation_rmse_mm"] = _weighted_rmse(
            (
                row["fixed_to_fixed_cube_pose_consistency_translation_rmse_mm"],
                cross_row.get("n_fixed_to_fixed_cube_pairs"),
            ),
            (
                row["gripper_to_fixed_cube_pose_consistency_translation_rmse_mm"],
                cross_row.get("n_gripper_to_fixed_cube_pairs"),
            ),
        )
        row["cam_common_cube_rotation_rmse_deg"] = _weighted_rmse(
            (
                row["fixed_to_fixed_cube_pose_consistency_rotation_rmse_deg"],
                cross_row.get("n_fixed_to_fixed_cube_pairs"),
            ),
            (
                row["gripper_to_fixed_cube_pose_consistency_rotation_rmse_deg"],
                cross_row.get("n_gripper_to_fixed_cube_pairs"),
            ),
        )
        row["cross_view_cube_support"] = (
            f"{cross_row.get('n_fixed_to_fixed_cube_pairs', 0)} "
            f"fixed-camera pairs + "
            f"{cross_row.get('n_gripper_to_fixed_cube_pairs', 0)} "
            "fixed-gripper pairs")
        for target in ("overall", "board", "cube"):
            row[f"reference_dependent_{target}_reprojection_rmse_px"] = (
                cross_row[
                    f"reference_dependent_{target}_reprojection_rmse_px_mean"])
        rows.append(row)
    return rows


def _final_train_target(row: dict) -> str:
    if row["method"] in {"A0", "B3"}:
        return "board-on-gripper only"
    if row["method"] == "B2":
        return "cube only"
    return "board+cube"


def _final_pose_handling(row: dict) -> str:
    if row["method"] in {"A0", "B3"}:
        return f"board pose={row['board_pose_handling']}; cube heldout only"
    return f"cube pose={row['cube_pose_handling']}"


def _optimization_result_table(rows: list[dict], cube_best) -> str:
    lines = [
        "| Method (방법) | Calibration train target | Optimization | "
        "FK / target-pose 처리 | Train RMSE px | ALL Cube RMSE px | "
        "Heldout Cube RMSE px | Cross-view Cube px | Cam-common Cube "
        "mm/deg | External cube GT | Convergence | Data status |",
        "| --- | --- | --- | --- | ---: | ---: | ---: | ---: | ---: | --- | ---: | --- |",
    ]
    for row in rows:
        cube = row["heldout_cube_reprojection_rmse_px"]
        cube_text = _fmt_best(cube, cube_best)
        cam_common_mm_deg = (
            f"{_fmt(row['cam_common_cube_translation_rmse_mm'])} / "
            f"{_fmt(row['cam_common_cube_rotation_rmse_deg'])}"
        )
        lines.append(
            f"| {row['method']} ({row['label']}) | {_final_train_target(row)} | "
            f"{row['optimization']} | {_final_pose_handling(row)} | "
            f"{_fmt(row['train_overall_reprojection_rmse_px'])} | "
            f"{_fmt(row['all_cube_reprojection_rmse_px'])} | "
            f"{cube_text} | "
            f"{_fmt(row['cross_view_cube_pixel_transfer_rmse_px'])} | "
            f"{cam_common_mm_deg} | Pending | "
            f"{row['converged_runs']}/{row['total_runs']} | "
            f"{_status_label(row['status'])} |")
    return "\n".join(lines)


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
        fk_description = row["cube_pose_handling"]
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
            "train_reprojection_rmse_px": row["train_overall_reprojection_rmse_px"],
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
        "## Cross-view Camera Consistency (cube-only)",
        "",
        "고정카메라 pair와 고정카메라↔그리퍼카메라 pair를 같은 cross-view "
        "cube consistency 지표 안에서 함께 집계한다. 별도 pair-type 순위 "
        "지표는 만들지 않는다.",
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
        "- A4·B1·B2: robust visual reprojection + whitened robust FK factor "
        "**2항**",
        "- `pose_error`와 `FK_constraint`: 서로 다른 두 항이 아니라 동일한 "
        "FK factor의 두 표현",
        "- `w1`, `w2`, `w3`: 사용하지 않음",
        "- 상대 scale: visual pixel `f_scale`과 FK covariance whitening "
        "`Sigma^(-1/2)`로 결정",
        "",
        "> **판정:** `w1·reprojection + w2·pose_error + w3·FK_constraint`는 "
        "현재 코드에 없는 부정확한 서술이다.",
        "",
        "### A3의 raw-FK-fixed 의미",
        "",
        r"$$T_{B\,cube}(s)=F_s^{raw}T_{cube\ center\rightarrow object}^{mech}$$",
        "",
        "A3가 고정하는 cube pose는 set별 controller raw FK pose에 영상과 "
        "무관하게 사전 등록한 mechanical frame map `R_y(180°)`를 적용한 "
        "pose다. cube-center 원점 이동은 0이고, `Delta_train`이나 aligned FK "
        "artifact를 사용하지 않는다. A3 최종 optimizer에서는 이 pose를 "
        "상수로 고정하고 visual reprojection 1항만 최소화한다.",
        "",
        "> **판정:** A3는 pure raw-FK hard constraint이지만 external GT는 "
        "아니다. tool4/CAD frame 정의 오차가 그대로 결과에 들어간다.",
        "",
        "### A5의 vision-aligned-FK-fixed 의미",
        "",
        r"$$T_{B\,cube}(s)=F_s^{raw}\Delta_{train}$$",
        "",
        "A5는 board와 held-out을 제외한 train eye-in-hand cube 영상으로 "
        "추정한 `Delta_train`을 적용한 뒤 set별 cube pose를 상수로 "
        "고정한다. A4와 동일한 aligned-FK artifact를 사용하지만 A4처럼 "
        "covariance factor로 완화하지 않는다.",
        "",
        "> **판정:** A5는 train-only vision-aligned FK를 쓰는 최종 후보 "
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
            "board-on-gripper only에서 sequential과 unified의 차이는 무엇인가",
            ("cube",),
            "External cube GT에서 최종 판정한다. 현재 데이터에 cube heldout이 없으면 N/A로 둔다.",
        ),
        (
            "Final protocol",
            "A0",
            "A1",
            "A0 -> A1",
            "board-on-gripper baseline에 cube train 관측을 추가하면 cube 평가가 개선되는가",
            ("cube",),
            "External cube GT와 heldout cube RMSE로 판정한다.",
        ),
        (
            "Final protocol",
            "A1",
            "A2",
            "A1 -> A2",
            "Vision-only 조건에서 unified feedback이 도움이 되는가",
            ("cube",),
            "External cube GT와 heldout cube RMSE로 판정한다.",
        ),
        (
            "Final protocol",
            "B3",
            "A2",
            "B3 -> A2",
            "unified 구조에서 cube residual이 최종 cube 평가에 필요한가",
            ("cube",),
            "External cube GT와 heldout cube RMSE로 판정한다.",
        ),
        (
            "Final protocol",
            "A2",
            "A3",
            "A2 -> A3",
            "Vision-estimated cube pose를 raw-FK hard fixed로 바꾸면 어떤가",
            ("cube",),
            "raw FK hard fixed가 실제 cube 정합을 높이는지 External GT로 확인한다.",
        ),
        (
            "Final protocol",
            "B1",
            "A4",
            "B1 -> A4",
            "같은 soft FK factor에서 sequential과 unified 중 무엇이 나은가",
            ("cube",),
            "External cube GT와 heldout cube RMSE로 판정한다.",
        ),
        (
            "Final protocol",
            "A2",
            "A4",
            "A2 -> A4",
            "Unified vision-only에 soft FK factor를 추가하면 이득이 있는가",
            ("cube",),
            "soft FK factor의 최종 이득은 External cube GT로 판정한다.",
        ),
        (
            "Final protocol",
            "B2",
            "A4",
            "B2 -> A4",
            "Soft FK 조건에서 board residual이 cube 보정에 도움 되는가",
            ("cube",),
            "External cube GT와 heldout cube RMSE로 판정한다.",
        ),
        (
            "Final protocol",
            "A3",
            "A5",
            "A3 -> A5",
            "Raw FK hard fixed와 vision-aligned FK hard fixed의 차이는 무엇인가",
            ("cube",),
            "A5가 GT 공개 전에 frozen method이면 최종 후보로 판정 가능하다.",
        ),
        (
            "Final protocol",
            "A4",
            "A5",
            "A4 -> A5",
            "같은 aligned FK를 soft factor와 hard fixed로 쓰면 무엇이 달라지는가",
            ("cube",),
            "A5가 GT 공개 전에 frozen method이면 최종 후보로 판정 가능하다.",
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
        "| Tier (구분) | Direct Contrast (직접 비교) | Question (검증 질문) | "
        "Primary Metric (주 지표) | Session04 Result | Decision (판정) |",
        "| --- | --- | --- | --- | --- | --- |",
    ]
    for tier, contrast, question, metric, result, decision in contrasts:
        lines.append(
            f"| {tier} | {contrast} | {question} | {metric} | {result} | "
            f"{decision} |")
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


def _metric_decision_records(rows: list[dict],
                             data_warnings: dict) -> list[tuple[str, str, str, str, str]]:
    cube_corners = _first_support(rows, "cube")
    support = data_warnings.get("support", {})
    cube = support.get("cube", {})
    cameras = ", ".join(str(value) for value in data_warnings.get(
        "evaluation_fixed_camera_intersection", [])) or "N/A"
    return [
        (
            "External cube TRE / rotation / P95 / failure",
            "Final primary metric",
            "독립 External GT cube pose와 blind prediction을 비교해 최종 순위를 정함",
            "GT 측정계 uncertainty floor보다 작은 차이는 주장하지 않는다.",
            "pending; External GT 추가 후 산출",
        ),
        (
            "ALL Cube RMSE px",
            "Fit sanity check",
            "train+heldout 전체 cube evaluation data에 frozen calibration을 적용",
            "train과 heldout을 섞으므로 일반화 지표가 아니다.",
            f"cube {cube.get('n_observations', 'N/A')} obs / "
            f"{cube.get('n_corners', cube_corners or 'N/A')} corners",
        ),
        (
            "Train reprojection RMSE",
            "Solver diagnostic",
            "수렴/적합 상태 확인",
            "학습 관측에 대한 fit이므로 방법 우월성 지표가 아니다.",
            "row별 train residual",
        ),
        (
            "Heldout Cube RMSE px",
            "Internal support metric",
            "미사용 cube event corner에 frozen transform을 적용해 재투영",
            "같은 set의 다른 event라 새 위치 일반화나 물리 GT가 아니다.",
            f"heldout cube {cube_corners or 'N/A'} corners",
        ),
        (
            "Cross-view pixel transfer RMSE",
            "Supplementary camera consistency",
            "한 카메라 PnP pose를 다른 카메라로 전달해 cube corner px 오차 계산",
            "모든 고정카메라에 함께 존재하는 systematic error와 절대 "
            "물리 오차를 검출하지 못한다.",
            f"fixed cameras {cameras}; gripper camera pair 포함",
        ),
        (
            "Cam-common Obj-Cam consistency mm/deg",
            "Supplementary camera consistency",
            "두 카메라가 계산한 cube object pose 차이를 mm/deg로 집계",
            "공통 계통오차는 검출하지 못하며 외부 GT 순위용이 아니다.",
            "fixed-camera pair와 fixed-gripper pair를 cube-only로 함께 집계",
        ),
    ]


def _metric_decision_table(rows: list[dict], data_warnings: dict) -> str:
    metric_rows = _metric_decision_records(rows, data_warnings)
    lines = [
        "## Metric Decision Matrix (평가지표 판정표)",
        "",
        "| Metric (지표) | Tier (등급) | Use (사용법) | Limit (제한) | "
        "Current Support (현재 근거) |",
        "| --- | --- | --- | --- | --- |",
    ]
    for metric, tier, use, limit, support_text in metric_rows:
        lines.append(
            f"| {metric} | {tier} | {use} | {limit} | {support_text} |")
    return "\n".join(lines)


def _internal_only_claim_envelope() -> str:
    return "\n".join([
        "## Final Protocol Lock (최종 단일 기준)",
        "",
        "| 항목 | 최종 기준 | 제외한 것 |",
        "| --- | --- | --- |",
        "| 비교 행 | A0~A5, B1~B3만 사용 | A6, 별도 board-only FK 변형, marker-system 별도 순위 |",
        "| Heldout target | 항상 cube만 평가 | Board heldout, board와 cube를 섞은 pooled overall ranking |",
        "| 최종 주 지표 | External cube TRE / rotation / P95 / failure | 내부 px만으로 물리 순위 확정 |",
        "| 보조 내부 지표 | ALL Cube, Train, Heldout Cube, Cross-view camera consistency | pair type별 값을 별도 순위 지표로 분리 |",
        "| A5 해석 | External GT 공개 전에 frozen이면 최종 후보 | GT를 본 뒤 정의한 사후 선택 |",
        "",
        "현재 Session04 artifact는 이전 촬영 구성에서 생성된 값이므로, 최종 "
        "board-on-gripper A0/B3 cube 평가가 없으면 해당 칸은 N/A로 둔다.",
    ])


def _markdown(rows: list[dict], marker: dict, detailed: bool,
              data_warnings: dict | None = None,
              session_label: str = "Session") -> str:
    _ = marker
    data_warnings = data_warnings or {}
    title = (
        f"# {session_label} Calibration Evaluation (캘리브레이션 평가)"
        if detailed else f"# {session_label} Table 1 Results (표 1 결과)")
    cube_best = _minimum(rows, "heldout_cube_reprojection_rmse_px")
    lines = [
        title,
        "",
        "> Status: Final protocol before External GT. 비교 행은 A0~A5, "
        "B1~B3 한 벌만 사용하고, heldout 평가는 항상 cube만 본다.",
        "",
        _markdown_data_warnings(data_warnings),
        "",
        _internal_only_claim_envelope(),
        "",
        "## Final Comparison Table (최종 비교실험표)",
        "",
        "> 굵은 값은 현재 artifact에서 관측된 Heldout Cube RMSE 최솟값이다. "
        "External GT가 들어오기 전에는 최종 물리 순위로 해석하지 않는다.",
        "",
        _optimization_result_table(rows, cube_best),
        "",
        _matched_contrast_table(rows),
        "",
        _metric_decision_table(rows, data_warnings),
        "",
        "> `Convergence 3/3`은 서로 다른 초기화 seed 3회 모두에서 "
        "SciPy solver가 `success=True`로 종료됐다는 뜻이다. Sequential "
        "행은 두 stage가 모두 성공해야 하며, B1은 stage 1과 모든 fixed-camera "
        "stage 2가 성공해야 1회 수렴으로 센다. 이는 solver 종료 조건 충족을 "
        "뜻할 뿐, 절대 정확도나 전역 최적해를 보장하지 않는다.",
        "",
        _scope_table(rows),
        "",
        _objective_block_table(rows),
        "",
        "## Calculation (계산 방식)",
        "",
        "최종 평가는 Target $O=cube$만 사용한다.",
        "",
        "$$T^{B,(i)}_O=T^B_{C_i}T^{C_i}_{O,\\mathrm{PnP}}$$",
        "",
        "$$T^B_{C_g}(e)=T^B_G(e)T^G_{C_g}$$",
        "",
        "$$T^{B,(g)}_O(e)=T^B_G(e)T^G_{C_g}"
        "T^{C_g}_{O,\\mathrm{PnP}}$$",
        "",
        "Cross-view pixel transfer는 한 카메라의 측정 PnP pose를 다른 "
        "카메라로 전달해 cube corner pixel error를 계산한다. 고정카메라 "
        "pair와 고정카메라↔그리퍼카메라 pair를 같은 보조 지표로 함께 "
        "집계하고, 별도 pair-type 순위 지표는 만들지 않는다.",
        "",
        "Heldout Cube RMSE는 미사용 cube event corner에 frozen transform을 "
        "적용해 계산한다.",
        "",
        "$$RMSE_{px}=\\sqrt{\\frac{1}{2N}\\sum_k((u_k-\\hat u_k)^2+(v_k-\\hat v_k)^2)}$$",
        "",
        "ALL Cube RMSE는 train cube와 heldout cube를 같은 방식으로 계산한 뒤 "
        "corner 수로 가중해 합친 fit sanity check다.",
        "",
        "## Interpretation Limit (해석 한계)",
        "",
        "Cross-view pixel transfer와 Cam-common Obj-Cam consistency는 "
        "방법별 추정값에 의존하므로 공통 systematic error를 검출하지 못한다. "
        "따라서 최종 주장은 External cube GT로만 결정한다.",
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
            f"<td>{_fmt(row['train_overall_reprojection_rmse_px'])}</td>"
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
<p>A0/B3의 board-only 방법은 cube 포즈 다양성만큼 board를 gripper에 붙여 촬영하는 최종 capture를 전제로 합니다. 현재 Session04 artifact에 해당 cube 평가가 없으면 N/A로 유지합니다.</p></section>
<section class="panel"><h2>Final Comparison Table (최종 비교실험표)</h2>
<div class="table-wrap"><table>
<thead><tr><th>Method</th><th>Label</th><th>Calibration train target</th><th>Optimization</th><th>FK / target-pose 처리</th><th>Train RMSE px</th><th>ALL Cube RMSE px</th><th>Heldout Cube RMSE px</th><th>Cross-view Cube px</th><th>Cam-common Cube mm/deg</th><th>External cube GT</th><th>Convergence</th><th>Data status</th></tr></thead>
<tbody>{''.join(method_rows)}</tbody></table></div></section>
{_html_matched_contrast(rows)}
{_html_metric_decision(rows, data_warnings)}
<section class="panel"><h2>Cross-view Camera Consistency (cube-only)</h2>
<div class="table-wrap"><table>
<thead><tr><th>Method</th><th>Cross-view Cube px</th><th>Cam-common Cube mm</th><th>Cam-common Cube deg</th><th>Support</th></tr></thead>
<tbody>{''.join(consistency_rows)}</tbody></table></div></section>
<section class="panel note"><h2>Interpretation (해석)</h2><p>Cross-view pixel transfer와 Cam-common Obj-Cam consistency는 카메라 간 cube 일관성 보조 지표입니다. 공통 systematic error는 잡지 못하므로 최종 주장은 External cube GT로만 결정합니다.</p></section>
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
        "--table1", default="CP_result/session04/late_table1/table1_methods.json")
    parser.add_argument(
        "--cross", default=(
            "CP_result/session04/cross_target_evaluation/"
            "cross_target_evaluation.json"))
    parser.add_argument(
        "--marker", default=(
            "CP_result/session04/marker_system_end_to_end/"
            "marker_system_end_to_end.json"))
    parser.add_argument(
        "--late_dir", default="CP_result/session04/late_table1")
    parser.add_argument(
        "--html",
        default="CP_result/session04/late_table1/TABLE1_INTERACTIVE.html")
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
    _write_csv(late_dir / "table1_results.csv", rows)
    (late_dir / "TABLE1_RESULTS.md").write_text(
        _markdown(
            rows, marker, detailed=True, data_warnings=data_warnings,
            session_label=session_label))
    Path(args.html).write_text(
        _html(rows, marker, data_warnings=data_warnings,
              session_label=session_label))
    print("[DONE] Generated current CSV, Markdown, and HTML artifacts")


if __name__ == "__main__":
    main()
