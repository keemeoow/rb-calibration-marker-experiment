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
}
SYSTEM_ORDER = ("board_only", "cube_only", "board_cube")
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


def _fmt_best(value, best, digits: int = 4, html: bool = False) -> str:
    """Bold a displayed minimum, including values tied after rounding."""
    formatted = _fmt(value, digits)
    if best is None or formatted == "N/A" or formatted != _fmt(best, digits):
        return formatted
    return (f"<strong>{formatted}</strong>" if html
            else f"**{formatted}**")


def _status_label(value: str) -> str:
    return {
        "complete": "Complete (완료)",
        "preflight_simulation_prior": (
            "Preflight — Simulation Prior (예비실험 — 시뮬레이션 사전값)"),
        "posthoc_diagnostic": "Post-hoc Diagnostic (사후 원인 진단)",
    }.get(value, value)


def _display_label(method: str, source_label: str) -> str:
    return CANONICAL_LABEL_OVERRIDES.get(method, source_label)


def _result_sections(rows: list[dict]) -> list[tuple[str, str, list[dict]]]:
    return [
        (
            "Confirmatory Internal (확증 내부)",
            "코드 내부 ablation과 calibration 안정성 검증에 쓰는 Complete 행이다.",
            [row for row in rows if row["status"] == "complete"],
        ),
        (
            "Preflight (예비실험)",
            "Simulation prior FK covariance를 쓰므로 물리 우월성 주장에는 쓰지 않는다.",
            [row for row in rows if row["status"] == "preflight_simulation_prior"],
        ),
        (
            "Post-hoc Diagnostics (사후 원인 진단)",
            "결과 해석 뒤 원인을 분리하기 위한 진단 행이며 메인 순위에서 제외한다.",
            [row for row in rows if row["status"] == "posthoc_diagnostic"],
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
    entry = runs[0][f"{split}_reprojection"].get(target)
    return None if entry is None else int(entry["n_corners"])


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
        for target in ("overall", "board", "cube"):
            row[f"heldout_{target}_set_equal_weight_rmse_px"] = (
                _reprojection_field_mean(
                    runs, "heldout", target, "set_equal_weight_rmse_px"))
            row[f"heldout_{target}_n_corners"] = _corner_count(
                runs, "heldout", target)
        for scope in ("fixed_to_fixed", "gripper_to_fixed"):
            for target in TARGETS:
                for field in SCOPE_FIELDS:
                    row[f"{scope}_{target}_{field}"] = cross_row[
                        f"{scope}_{target}_{field}_mean"]
        for target in ("overall", "board", "cube"):
            row[f"reference_dependent_{target}_reprojection_rmse_px"] = (
                cross_row[
                    f"reference_dependent_{target}_reprojection_rmse_px_mean"])
        rows.append(row)
    return rows


def _optimization_result_table(
        rows: list[dict], board_best, cube_best,
        highlight_complete_best: bool) -> str:
    lines = [
        "| Method (방법) | 기여도2 - Marker Set (마커 구성) | 기여도1 - "
        "Optimization (최적화) | 기여도3 - Cube Pose (큐브 자세 처리) | Train Overall "
        "(학습 전체 px) | Own Held-out Overall (자체 홀드아웃 전체 px) | "
        "Board/Cube Held-out (보드/큐브 홀드아웃 px) | Convergence (수렴) | "
        "Status (상태) |",
        "| --- | --- | --- | --- | ---: | ---: | ---: | ---: | --- |",
    ]
    for row in rows:
        board = row["heldout_board_reprojection_rmse_px"]
        cube = row["heldout_cube_reprojection_rmse_px"]
        board_text = (_fmt_best(board, board_best)
                      if highlight_complete_best else _fmt(board))
        cube_text = (_fmt_best(cube, cube_best)
                     if highlight_complete_best else _fmt(cube))
        lines.append(
            f"| {row['method']} ({row['label']}) | {row['target_set']} | "
            f"{row['optimization']} | {row['cube_pose_handling']} | "
            f"{_fmt(row['train_overall_reprojection_rmse_px'])} | "
            f"{_fmt(row['heldout_overall_reprojection_rmse_px'])} | "
            f"{board_text} / {cube_text} | "
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
    with path.open("w", newline="") as handle:
        writer = csv.DictWriter(
            handle, fieldnames=list(rows[0]), lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)


def _scope_table(rows: list[dict], scope: str) -> str:
    scope_label = (
        "A — Fixed-to-Fixed 보조 Held-out 일관성" if scope == "fixed_to_fixed"
        else "Gripper-to-Fixed (그리퍼카메라–고정카메라 간)")
    lines = [
        f"### {scope_label}",
        "",
        "| Method (방법) | Board Pixel (보드 px) | Board Translation "
        "(보드 이동 mm) | Board Rotation (보드 회전 deg) | Cube Pixel "
        "(큐브 px) | Cube Translation (큐브 이동 mm) | Cube Rotation "
        "(큐브 회전 deg) |",
        "| --- | ---: | ---: | ---: | ---: | ---: | ---: |",
    ]
    best = {
        (target, field): _minimum(rows, f"{scope}_{target}_{field}")
        for target in TARGETS for field in SCOPE_FIELDS
    }
    for row in rows:
        values = []
        for target in TARGETS:
            for field in SCOPE_FIELDS:
                key = f"{scope}_{target}_{field}"
                values.append(_fmt_best(
                    row[key], best[(target, field)]))
        lines.append("| " + " | ".join(
            [row["method"], *values]) + " |")
    return "\n".join(lines)


def _marker_table(marker: dict) -> str:
    by_system = {row["system"]: row for row in marker["summary"]}
    marker_rows = list(by_system.values())
    best = {
        key: _minimum(marker_rows, key)
        for key in (
            "fixed_to_fixed_board_cross_view_pixel_transfer_rmse_px_mean",
            "fixed_to_fixed_cube_cross_view_pixel_transfer_rmse_px_mean",
            "gripper_to_fixed_board_cross_view_pixel_transfer_rmse_px_mean",
            "gripper_to_fixed_cube_cross_view_pixel_transfer_rmse_px_mean",
        )
    }
    lines = [
        "### Marker-system End-to-End (마커 시스템 전체 경로)",
        "",
        "| System (시스템) | Own Held-out (자체 홀드아웃 px) | "
        "Fixed-to-Fixed Board/Cube (고정카메라 간 보드/큐브 px) | "
        "Gripper-to-Fixed Board/Cube (그리퍼카메라–고정카메라 간 "
        "보드/큐브 px) | Convergence (수렴) |",
        "| --- | ---: | ---: | ---: | ---: |",
    ]
    for system in SYSTEM_ORDER:
        row = by_system[system]
        fixed_board = "fixed_to_fixed_board_cross_view_pixel_transfer_rmse_px_mean"
        fixed_cube = "fixed_to_fixed_cube_cross_view_pixel_transfer_rmse_px_mean"
        gripper_board = "gripper_to_fixed_board_cross_view_pixel_transfer_rmse_px_mean"
        gripper_cube = "gripper_to_fixed_cube_cross_view_pixel_transfer_rmse_px_mean"
        lines.append(
            f"| {row['label']} | {_fmt(row['own_heldout_overall_rmse_px_mean'])} | "
            f"{_fmt_best(row[fixed_board], best[fixed_board])} / "
            f"{_fmt_best(row[fixed_cube], best[fixed_cube])} | "
            f"{_fmt_best(row[gripper_board], best[gripper_board])} / "
            f"{_fmt_best(row[gripper_cube], best[gripper_cube])} | "
            f"{row['converged_runs']}/{row['total_runs']} |")
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
        "> **판정:** A5는 이전 A3 결과의 원인을 분리하는 post-hoc "
        "diagnostic이다. 독립 실측 correction이나 external GT가 아니다.",
    ])


def _set_equal_weight_table(rows: list[dict]) -> str:
    """Report the pooling-bias control beside the corner-pooled number.

    Corner-pooled RMSE lets whichever placement exposed the most corners speak
    loudest.  Showing both, with the corner counts that drive the difference,
    keeps the reader from reading a support artefact as an accuracy change.
    """
    lines = [
        "| Method (방법) | Board pooled / set-equal (보드 px) | "
        "Cube pooled / set-equal (큐브 px) | "
        "Overall pooled / set-equal (전체 px) | Status (상태) |",
        "| --- | ---: | ---: | ---: | --- |",
    ]
    for row in rows:
        cells = []
        for target in ("board", "cube", "overall"):
            pooled = row[f"heldout_{target}_reprojection_rmse_px"]
            equal = row[f"heldout_{target}_set_equal_weight_rmse_px"]
            cells.append(f"{_fmt(pooled)} / {_fmt(equal)}")
        lines.append(
            f"| {row['method']} ({row['label']}) | " + " | ".join(cells)
            + f" | {_status_label(row['status'])} |")
    return "\n".join(lines)


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


def _matched_contrast_records(rows: list[dict]) -> list[tuple[str, str, str, str, str, str]]:
    by_method = {row["method"]: row for row in rows}
    return [
        (
            "Confirmatory internal",
            "A0 <-> B3",
            "Board-only에서 sequential freeze와 unified feedback 차이가 있는가",
            "held-out board px",
            _heldout_pair(by_method, "A0", "B3", ("board",)),
            "동률. Board-only에서는 통합 자체가 추가 이득을 만들지 않는다.",
        ),
        (
            "Confirmatory internal",
            "A0 -> A1",
            "순차법에 cube residual을 추가하면 board 성능이 좋아지는가",
            "held-out board px, N_reg",
            _heldout_pair(by_method, "A0", "A1", ("board",))
            + f"; N_reg {by_method['A0']['n_registered_fixed_cameras']} -> "
            f"{by_method['A1']['n_registered_fixed_cameras']}",
            "Board는 미세 악화. 순차 구조에서는 cube 추가 이득이 보이지 않는다.",
        ),
        (
            "Confirmatory internal",
            "A1 -> A2",
            "Vision-only 조건에서 unified feedback이 도움이 되는가",
            "held-out board/cube px",
            _heldout_pair(by_method, "A1", "A2", ("board", "cube")),
            "두 target 모두 개선. 현재 내부 확증에서 가장 강한 긍정 contrast다.",
        ),
        (
            "Confirmatory internal",
            "B3 -> A2",
            "Unified 조건에서 cube residual이 board calibration에도 도움이 되는가",
            "held-out board px",
            _heldout_pair(by_method, "B3", "A2", ("board",)),
            "Board shared component가 개선. 단 marker-system 전체 성능 주장은 아니다.",
        ),
        (
            "Confirmatory internal",
            "A2 -> A3",
            "Vision-estimated cube pose를 raw-FK hard fixed로 바꾸면 어떤가",
            "held-out board/cube px",
            _heldout_pair(by_method, "A2", "A3", ("board", "cube")),
            "특히 cube가 크게 악화. raw FK를 GT로 해석하면 안 된다.",
        ),
        (
            "Preflight",
            "B1 -> A4",
            "같은 soft FK factor에서 sequential과 unified 중 무엇이 나은가",
            "held-out board/cube px",
            _heldout_pair(by_method, "B1", "A4", ("board", "cube")),
            "통합 개선 경향. measured covariance 전까지는 preflight다.",
        ),
        (
            "Preflight",
            "A2 -> A4",
            "Unified vision-only에 soft FK factor를 추가하면 이득이 있는가",
            "held-out board/cube px",
            _heldout_pair(by_method, "A2", "A4", ("board", "cube")),
            "사실상 동률. A4는 방법 확장 후보지만 현재 우월성 주장은 금지.",
        ),
        (
            "Preflight",
            "B2 -> A4",
            "Soft FK 조건에서 board residual이 cube 보정에 도움 되는가",
            "held-out cube px",
            _heldout_pair(by_method, "B2", "A4", ("cube",)),
            "Cube가 개선. board residual은 soft-FK cube 추정에 도움이 된다.",
        ),
        (
            "Post-hoc",
            "A3 <-> A5, A4 <-> A5",
            "Raw/aligned FK, soft/hard 원인을 분리할 수 있는가",
            "internal metrics only",
            "A3->A5 "
            + _heldout_pair(by_method, "A3", "A5", ("board", "cube"))
            + "; A4->A5 "
            + _heldout_pair(by_method, "A4", "A5", ("board", "cube")),
            "A5는 원인 진단 전용. 독립 correction 또는 물리 순위가 아니다.",
        ),
    ]


def _matched_contrast_table(rows: list[dict]) -> str:
    contrasts = _matched_contrast_records(rows)
    lines = [
        "## Matched Contrast Decision Table (비교실험 구성 확정표)",
        "",
        "모든 행을 하나의 전체 순위로 세우지 않고, 한 번에 한 요소만 달라지는 "
        "contrast만 해석한다.",
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
        "> 현재 메인 결론은 A2다. A4는 measured FK covariance가 들어오기 전까지 "
        "방법 확장 후보이고, A5는 post-hoc 원인 진단이다.",
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
    # A0/B3 are board-only and A2 is cube-only, so no single row carries both
    # counts.  The split is frozen, so the first row that evaluated a target
    # reports the same support every other row of that target sees.
    board_corners = _first_support(rows, "board")
    cube_corners = _first_support(rows, "cube")
    support = data_warnings.get("support", {})
    overall = support.get("overall", {})
    cameras = ", ".join(str(value) for value in data_warnings.get(
        "evaluation_fixed_camera_intersection", [])) or "N/A"
    return [
        (
            "Train reprojection RMSE",
            "Solver diagnostic",
            "수렴/적합 상태 확인",
            "학습 관측에 대한 fit이므로 방법 우월성 지표가 아니다.",
            "row별 train residual",
        ),
        (
            "Own-marker held-out RMSE",
            "Primary internal pixel metric",
            "matched contrast의 board/cube별 주 지표",
            "같은 set의 다른 event라 새 위치 일반화나 물리 GT가 아니다.",
            f"Board {board_corners} corners, Cube {cube_corners} corners",
        ),
        (
            "Pooled overall RMSE",
            "Secondary summary",
            "같은 marker population 내부에서만 참고",
            "Board corner 지지도가 커서 전체값이 board에 치우친다.",
            "전체 순위 금지",
        ),
        (
            "Set-equal-weight RMSE",
            "Exploratory support-bias check",
            "corner-pooled 값 옆에 병기",
            "n=9 sets라 CI/유의성 주장은 아직 약하다.",
            "corner -> event -> set -> set 동일가중",
        ),
        (
            "Fixed-to-Fixed Board/Cube",
            "Supplementary FK-free subsystem metric",
            "고정카메라 상대 일관성 진단",
            "모든 고정카메라에 함께 존재하는 systematic error와 절대 "
            "물리 오차를 검출하지 못한다.",
            f"fixed cameras {cameras}; {overall.get('n_observations', 'N/A')} obs",
        ),
        (
            "Gripper-to-Fixed Board/Cube",
            "Supplementary FK-dependent closure metric",
            "전체 chain 내부 진단",
            "FK와 Hand-Eye가 섞이며 fixed anchor 일부는 train 관측이다.",
            "mixed train-anchor/held-out internal closure",
        ),
        (
            "Reference-dependent reprojection",
            "Secondary diagnostic",
            "공유 target pose 기준의 보조 확인",
            "reference가 fitted target이므로 ranking 지표가 아니다.",
            "cross-target v8 artifact",
        ),
        (
            "Seed mean +/- std",
            "Stability diagnostic",
            "3개 초기화 perturbation 안정성 확인",
            "독립 실험 표본이 아니라 통계적 반복으로 해석하지 않는다.",
            "27/27 converged",
        ),
        (
            "External TRE/rotation/P95/failure",
            "Future final primary metric",
            "blind external GT 확보 후 최종 물리 정확도",
            "현재 Session04에는 GT가 없어 계산 불가.",
            "future capture required",
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


def _markdown(rows: list[dict], marker: dict, detailed: bool,
              data_warnings: dict | None = None,
              session_label: str = "Session") -> str:
    data_warnings = data_warnings or {}
    title = (
        f"# {session_label} Calibration Evaluation (캘리브레이션 평가)"
        if detailed else f"# {session_label} Table 1 Results (표 1 결과)")
    lines = [
        title,
        "",
        "> Status: Pre-GT Internal Evaluation (외부 GT 전 내부 평가). "
        "이 문서는 External GT (외부 정답)를 사용한 절대 정확도 순위를 "
        "제시하지 않는다.",
        "",
        _markdown_data_warnings(data_warnings),
        "",
        "## Evaluation Decision (평가 구성 결정)",
        "",
        "- A — Fixed-to-Fixed/e_cross는 각 방법이 추정한 카메라 자세로 "
        "계산하는 방법별 Supplementary Held-out Consistency (보조 홀드아웃 "
        "일관성)다. Robot FK는 쓰지 않지만 독립 기준선이나 순위 지표는 아니다.",
        "- B — OpenCV Relative-pose Baseline은 Main-method Transform, Robot "
        "FK, Hand–Eye, Shared Target Pose를 쓰지 않는 Independent Reference "
        "(독립 기준선)이며 별도 결과 파일로 보고한다.",
        "- Gripper-to-Fixed (그리퍼카메라–고정카메라 간)는 실제 Board/Cube "
        "Image Corners (보드/큐브 영상 코너)를 사용하지만, 예측 경로에는 "
        "Robot FK와 Hand–Eye (핸드–아이 변환)가 포함된다.",
        "- 각 set의 최초 고정카메라 관측은 최적화에서 한 번만 사용한다. "
        "Gripper-to-Fixed 평가는 이 fixed anchor를 같은 set의 모든 held-out "
        "gripper Event와 연결하고, Event→set→set 동일가중 순서로 집계한다.",
        "- 공식 결과는 물리 config의 nominal metric scale `1.0`만 사용한다. "
        "데이터에서 추정한 scale은 별도 diagnostic 결과로만 허용한다.",
        "- Board (보드)와 Cube (큐브)는 모두 촬영 원본에서 평가한다. "
        "캘리브레이션에 사용한 마커 종류와 평가 표적 종류를 동일시하지 않는다.",
        "- Reference-dependent Reprojection (기준 의존 재투영)은 Secondary "
        "Diagnostic (보조 진단)이며 방법 순위에 사용하지 않는다.",
        "- 현재 결론의 대표 행은 A2다. A4는 measured FK covariance 전 "
        "preflight이고, A5는 이전 A3의 성능 원인을 설명하는 post-hoc "
        "diagnostic이다. 실제 물리 순위는 External GT 이후 결정한다.",
        "",
        _matched_contrast_table(rows),
        "",
        _metric_decision_table(rows, data_warnings),
        "",
        _implementation_audit(),
        "",
        "## Table 1 Optimization Results (표 1 최적화 결과)",
        "",
        "> **굵은 값**은 `Complete` 행 중 Board/Cube별 held-out RMSE "
        "최솟값이다. Preflight와 post-hoc 행은 수치가 더 낮아도 "
        "확증 결과로 강조하지 않는다. Train/Own "
        "Overall은 marker population이 달라 전체 최솟값을 강조하지 않는다.",
    ]
    complete_rows = [row for row in rows if row["status"] == "complete"]
    board_best = _minimum(complete_rows, "heldout_board_reprojection_rmse_px")
    cube_best = _minimum(complete_rows, "heldout_cube_reprojection_rmse_px")
    for title, note, section_rows in _result_sections(rows):
        if not section_rows:
            continue
        lines.extend([
            "",
            f"### {title}",
            "",
            note,
            "",
            _optimization_result_table(
                section_rows, board_best, cube_best,
                highlight_complete_best=section_rows[0]["status"] == "complete"),
        ])
    board_corners = _first_support(rows, "board")
    cube_corners = _first_support(rows, "cube")
    board_share = (
        "N/A" if not board_corners or not cube_corners
        else f"{100.0 * board_corners / (board_corners + cube_corners):.1f}%")
    lines.extend([
        "",
        "### Set-equal-weight Held-out RMSE (set 동일가중 홀드아웃)",
        "",
        f"Held-out corner 지지도는 Board `{board_corners}` / Cube "
        f"`{cube_corners}`이므로 corner-pooled Overall은 Board가 약 "
        f"`{board_share}`를 차지한다. 아래 표는 같은 관측을 corner-pooled와 "
        "`corner → event → set → set 동일가중` 두 방식으로 집계한 값이다. "
        "두 값의 차이는 정확도 변화가 아니라 set별 corner 지지도 불균형의 "
        "크기이며, 방법 간 방향이 달라지는 행은 corner 지지도에 의존하는 "
        "결론이므로 단독으로 해석하지 않는다.",
        "",
        _set_equal_weight_table(rows),
        "",
        "> Set 동일가중 값은 corner-pooled 값을 대체하지 않는 보조 지표이며, "
        "`n=9 sets`이므로 이 차이만으로 유의성을 주장하지 않는다.",
        "",
        "> `Convergence 3/3`은 서로 다른 초기화 seed 3회 모두에서 "
        "SciPy solver가 `success=True`로 종료됐다는 뜻이다. Sequential "
        "행은 두 stage가 모두 성공해야 하며, B1은 stage 1과 모든 fixed-camera "
        "stage 2가 성공해야 1회 수렴으로 센다. 이는 solver 종료 조건 충족을 "
        "뜻할 뿐, 절대 정확도나 전역 최적해를 보장하지 않는다.",
        "",
        _objective_block_table(rows),
        "",
        "## Camera-scope Diagnostics (카메라 범위 진단)",
        "",
        "> **굵은 값**은 각 target·단위 열의 최솟값이다. A의 "
        "Fixed-to-Fixed는 보조 일관성 지표이므로 굵은 값이 절대 정확도 "
        "순위를 뜻하지 않는다.",
        "",
        _scope_table(rows, "fixed_to_fixed"),
        "",
        _scope_table(rows, "gripper_to_fixed"),
        "",
        _marker_table(marker),
        "",
        "## Calculation (계산 방식)",
        "",
        "For Target $O\\in\\{board,cube\\}$ (표적 $O$):",
        "",
        "$$T^{B,(i)}_O=T^B_{C_i}T^{C_i}_{O,\\mathrm{PnP}}$$",
        "",
        "$$T^B_{C_g}(e)=T^B_G(e)T^G_{C_g}$$",
        "",
        "$$T^{B,(g)}_O(e)=T^B_G(e)T^G_{C_g}"
        "T^{C_g}_{O,\\mathrm{PnP}}$$",
        "",
        "Pixel Transfer RMSE (픽셀 전달 평균제곱근오차)는 한 카메라의 "
        "측정 PnP 자세를 다른 카메라로 옮겨 실제 검출 코너와 비교한다. "
        "Translation/Rotation Consistency (이동/회전 일관성)는 두 경로로 "
        "얻은 $T^B_O$의 차이를 mm/deg로 계산한다. Gripper-to-Fixed의 "
        "최종값은 pair 성분을 Event RMSE로, Event를 set RMSE로 집계한 뒤 "
        "set별 동일 가중치로 계산한다.",
        "",
        "Held-out reprojection의 기본값은 corner-pooled RMSE "
        "$\\sqrt{\\frac{1}{2N}\\sum(du^2+dv^2)}$이다. 같은 관측을 "
        "`corner → Event RMSE → set RMSE → set별 동일 가중치` 순서로 다시 "
        "집계한 값을 Set-equal-weight로 병기한다. 두 값은 corner 지지도가 "
        "set마다 같을 때만 일치하므로, 차이는 정확도가 아니라 지지도 "
        "불균형의 크기를 뜻한다.",
        "",
        "## Interpretation Limit (해석 한계)",
        "",
        "A의 Fixed-to-Fixed는 방법별 추정값에 의존하고 모든 고정카메라에 "
        "함께 존재하는 Systematic Error (계통 오차)를 검출할 수 없으므로 "
        "보조 일관성 진단으로만 해석한다. Gripper-to-Fixed는 Hand–Eye Error "
        "(핸드–아이 오차)와 FK Error (순기구학 오차)를 분리할 수 없다. "
        "따라서 두 범위는 함께 보고하되 External Absolute Accuracy "
        "(외부 절대 정확도)로 부르지 않는다.",
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
        "- **Reference-dependent Reprojection (기준 의존 재투영)**: "
        "학습 표적 자세에 의존하는 보조 진단으로 External GT가 아니다.",
    ])
    if detailed:
        lines.extend([
            "",
            "## Required Next Experiment (다음 필수 실험)",
            "",
            "Independent External GT (독립 외부 정답)가 확정되면 Blind "
            "Position Holdout (비공개 위치 홀드아웃)으로 Translation Error "
            "(이동 오차), Rotation Error (회전 오차), P95, Failure Rate "
            "(실패율)를 다시 계산한다. 그 전에는 내부 지표만 유지한다.",
        ])
    return "\n".join(lines) + "\n"


def _html_scope_table(rows: list[dict], scope: str) -> str:
    title = (
        "A — Fixed-to-Fixed Supplementary Consistency"
        if scope == "fixed_to_fixed"
        else "Gripper-to-Fixed (그리퍼카메라–고정카메라 간)")
    body = []
    best = {
        (target, field): _minimum(rows, f"{scope}_{target}_{field}")
        for target in TARGETS for field in SCOPE_FIELDS
    }
    for row in rows:
        cells = [escape(row["method"])]
        for target in TARGETS:
            for field in SCOPE_FIELDS:
                key = f"{scope}_{target}_{field}"
                cells.append(_fmt_best(
                    row[key], best[(target, field)], html=True))
        body.append("<tr>" + "".join(
            f"<td>{value}</td>" for value in cells) + "</tr>")
    return f"""
    <section class="panel scope" data-scope="{scope}">
      <h2>{title}</h2>
      <div class="table-wrap"><table>
        <thead><tr><th>Method (방법)</th><th>Board px</th><th>Board mm</th>
        <th>Board deg</th><th>Cube px</th><th>Cube mm</th><th>Cube deg</th></tr></thead>
        <tbody>{''.join(body)}</tbody>
      </table></div>
    </section>"""


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


def _html_method_section(
        title: str, note: str, section_rows: list[dict],
        board_best, cube_best) -> str:
    status = section_rows[0]["status"] if section_rows else ""
    highlight = status == "complete"
    body = []
    for row in section_rows:
        board = row["heldout_board_reprojection_rmse_px"]
        cube = row["heldout_cube_reprojection_rmse_px"]
        board_text = (_fmt_best(board, board_best, html=True)
                      if highlight else _fmt(board))
        cube_text = (_fmt_best(cube, cube_best, html=True)
                     if highlight else _fmt(cube))
        body.append(
            "<tr>"
            f"<td>{escape(row['method'])}</td><td>{escape(row['label'])}</td>"
            f"<td>{escape(row['target_set'])}</td>"
            f"<td>{escape(row['optimization'])}</td>"
            f"<td>{escape(row['cube_pose_handling'])}</td>"
            f"<td>{_fmt(row['heldout_overall_reprojection_rmse_px'])}</td>"
            f"<td>{board_text}</td>"
            f"<td>{cube_text}</td>"
            f"<td>{row['converged_runs']}/{row['total_runs']}</td>"
            f"<td>{escape(_status_label(row['status']))}</td></tr>")
    return f"""
    <section class="method-section">
      <h3>{escape(title)}</h3>
      <p>{escape(note)}</p>
      <div class="table-wrap"><table>
        <thead><tr><th>Method (방법)</th><th>Label (설명)</th>
        <th>Marker Set</th><th>Optimization</th><th>Cube Pose</th>
        <th>Own Held-out Overall px</th><th>Board px</th><th>Cube px</th>
        <th>Convergence (수렴)</th><th>Status (상태)</th></tr></thead>
        <tbody>{''.join(body)}</tbody>
      </table></div>
    </section>"""


def _html_set_equal_weight(rows: list[dict]) -> str:
    board_corners = _first_support(rows, "board")
    cube_corners = _first_support(rows, "cube")
    share = (
        "N/A" if not board_corners or not cube_corners
        else f"{100.0 * board_corners / (board_corners + cube_corners):.1f}%")
    body = []
    for row in rows:
        cells = []
        for target in ("board", "cube", "overall"):
            pooled = row[f"heldout_{target}_reprojection_rmse_px"]
            equal = row[f"heldout_{target}_set_equal_weight_rmse_px"]
            cells.append(f"<td>{_fmt(pooled)} / {_fmt(equal)}</td>")
        body.append(
            "<tr>"
            f"<td>{escape(row['method'])}</td>"
            f"<td>{escape(row['label'])}</td>"
            + "".join(cells)
            + f"<td>{escape(_status_label(row['status']))}</td></tr>")
    return f"""
<section class="panel"><h2>Set-equal-weight Held-out RMSE (set 동일가중 홀드아웃)</h2>
<p>Held-out corner 지지도는 Board <code>{board_corners}</code> / Cube
<code>{cube_corners}</code>이므로 corner-pooled Overall은 Board가 약
<code>{share}</code>를 차지합니다. 각 칸은 <code>corner-pooled / set 동일가중</code>이며,
두 값의 차이는 정확도 변화가 아니라 set별 corner 지지도 불균형의 크기입니다.
<code>n=9 sets</code>이므로 이 차이만으로 유의성을 주장하지 않습니다.</p>
<div class="table-wrap"><table>
<thead><tr><th>Method (방법)</th><th>Label (설명)</th>
<th>Board pooled / set-equal px</th><th>Cube pooled / set-equal px</th>
<th>Overall pooled / set-equal px</th><th>Status (상태)</th></tr></thead>
<tbody>{''.join(body)}</tbody></table></div></section>"""


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
    data_warnings = data_warnings or {}
    complete_rows = [row for row in rows if row["status"] == "complete"]
    board_best = _minimum(complete_rows, "heldout_board_reprojection_rmse_px")
    cube_best = _minimum(complete_rows, "heldout_cube_reprojection_rmse_px")
    method_sections = [
        _html_method_section(title, note, section_rows, board_best, cube_best)
        for title, note, section_rows in _result_sections(rows)
        if section_rows
    ]
    marker_rows = []
    by_system = {row["system"]: row for row in marker["summary"]}
    marker_values = list(by_system.values())
    marker_best = {
        key: _minimum(marker_values, key)
        for key in (
            "fixed_to_fixed_board_cross_view_pixel_transfer_rmse_px_mean",
            "fixed_to_fixed_cube_cross_view_pixel_transfer_rmse_px_mean",
            "gripper_to_fixed_board_cross_view_pixel_transfer_rmse_px_mean",
            "gripper_to_fixed_cube_cross_view_pixel_transfer_rmse_px_mean",
        )
    }
    for system in SYSTEM_ORDER:
        row = by_system[system]
        fixed_board = "fixed_to_fixed_board_cross_view_pixel_transfer_rmse_px_mean"
        fixed_cube = "fixed_to_fixed_cube_cross_view_pixel_transfer_rmse_px_mean"
        gripper_board = "gripper_to_fixed_board_cross_view_pixel_transfer_rmse_px_mean"
        gripper_cube = "gripper_to_fixed_cube_cross_view_pixel_transfer_rmse_px_mean"
        marker_rows.append(
            "<tr>"
            f"<td>{escape(row['label'])}</td>"
            f"<td>{_fmt(row['own_heldout_overall_rmse_px_mean'])}</td>"
            f"<td>{_fmt_best(row[fixed_board], marker_best[fixed_board], html=True)} / "
            f"{_fmt_best(row[fixed_cube], marker_best[fixed_cube], html=True)}</td>"
            f"<td>{_fmt_best(row[gripper_board], marker_best[gripper_board], html=True)} / "
            f"{_fmt_best(row[gripper_cube], marker_best[gripper_cube], html=True)}</td>"
            f"<td>{row['converged_runs']}/{row['total_runs']}</td></tr>")
    return f"""<!doctype html>
<html lang="ko"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>{escape(session_label)} Calibration Evaluation (캘리브레이션 평가)</title>
<style>
:root{{--ink:#18212f;--muted:#627083;--line:#dce3ea;--paper:#f5f7fa;--card:#fff;--blue:#2066c7;--teal:#087f78}}
*{{box-sizing:border-box}} body{{margin:0;background:var(--paper);color:var(--ink);font:15px/1.55 system-ui,-apple-system,sans-serif}}
main{{max-width:1180px;margin:auto;padding:32px 20px 72px}} h1{{font-size:30px;margin:0 0 8px}} h2{{font-size:20px;margin:0 0 16px}} h3{{font-size:16px;margin:20px 0 6px}}
.subtitle{{color:var(--muted);margin-bottom:22px}} .badge{{display:inline-block;background:#fff3cd;color:#755600;border:1px solid #efd582;border-radius:999px;padding:5px 11px;font-weight:700}}
.grid{{display:grid;grid-template-columns:1fr 1fr;gap:18px}} .panel{{background:var(--card);border:1px solid var(--line);border-radius:14px;padding:20px;margin-top:18px;box-shadow:0 4px 14px #25364d0d}}
.controls{{display:flex;gap:8px;margin-top:18px}} button{{border:1px solid var(--line);background:white;border-radius:9px;padding:9px 13px;cursor:pointer;font-weight:650}} button.active{{background:var(--blue);color:white;border-color:var(--blue)}}
.table-wrap{{overflow:auto}} table{{border-collapse:collapse;width:100%;min-width:680px}} th,td{{padding:9px 10px;border-bottom:1px solid var(--line);text-align:right;white-space:nowrap}} th:first-child,td:first-child{{text-align:left}} th{{color:var(--muted);font-size:12px}}
.method-section p{{color:var(--muted);margin:0 0 8px}} .warning{{border-left:4px solid #c05621}} .warning li{{margin:6px 0}} .note{{border-left:4px solid var(--teal);padding-left:14px}} code{{background:#edf2f7;padding:2px 5px;border-radius:5px}}
@media(max-width:760px){{.grid{{grid-template-columns:1fr}} main{{padding:22px 12px 50px}}}}
</style></head><body><main>
<span class="badge">Pre-GT Internal Evaluation (외부 GT 전 내부 평가)</span>
<h1>{escape(session_label)} Calibration Evaluation (캘리브레이션 평가)</h1>
<p class="subtitle">A2는 현재 검증된 대표 행, A4는 measured-covariance 전 preflight, A5는 post-hoc diagnostic입니다. Table 1의 굵은 값은 Complete 행만 대상으로 하며 실제 물리 순위는 External GT 이후 결정합니다.</p>
{_html_data_warnings(data_warnings or {})}
<section class="panel"><h2>Table 1 Optimization Results (표 1 최적화 결과)</h2>{''.join(method_sections)}</section>
{_html_matched_contrast(rows)}
{_html_metric_decision(rows, data_warnings)}
{_html_set_equal_weight(rows)}
<div class="controls"><button class="active" data-show="all">Both Scopes (두 범위)</button><button data-show="fixed_to_fixed">Fixed-to-Fixed</button><button data-show="gripper_to_fixed">Gripper-to-Fixed</button></div>
<div class="grid">{_html_scope_table(rows, 'fixed_to_fixed')}{_html_scope_table(rows, 'gripper_to_fixed')}</div>
<section class="panel"><h2>Marker-system End-to-End (마커 시스템 전체 경로)</h2><div class="table-wrap"><table>
<thead><tr><th>System (시스템)</th><th>Own Held-out px</th><th>Fixed-to-Fixed Board/Cube px</th><th>Gripper-to-Fixed Board/Cube px</th><th>Convergence (수렴)</th></tr></thead>
<tbody>{''.join(marker_rows)}</tbody></table></div></section>
<section class="panel note"><h2>Interpretation (해석)</h2><p>Fixed-to-Fixed는 Robot FK 없이 고정카메라 부분을 평가합니다. Gripper-to-Fixed는 set 최초 fixed anchor를 같은 set의 모든 held-out gripper Event와 연결하고 Event→set→set 동일가중 순서로 집계합니다. 실제 영상 코너를 사용하지만 <code>T^B_G(e)T^G_C</code> 경로 때문에 Robot FK와 Hand–Eye에 의존합니다. 공식 표는 nominal metric scale 1.0만 사용하며, 두 지표 모두 External Absolute Accuracy (외부 절대 정확도)가 아닙니다.</p></section>
<section class="panel"><h2>Terminology (용어 설명)</h2><ul>
<li><b>PnP, Perspective-n-Point (3D–2D 자세 추정)</b>: 영상 코너로 카메라–표적 자세를 계산합니다.</li>
<li><b>FK, Forward Kinematics (순기구학)</b>: 이벤트별 Base-to-Gripper Transform (베이스–그리퍼 변환)을 계산합니다.</li>
<li><b>Hand–Eye Transform (핸드–아이 변환)</b>: Gripper-to-Camera Transform (그리퍼–카메라 변환)입니다.</li>
<li><b>RMSE, Root Mean Squared Error (평균제곱근오차)</b>: px, mm, deg 단위를 분리해 해석합니다.</li>
<li><b>Reference-dependent Reprojection (기준 의존 재투영)</b>: 학습 표적 자세를 쓰는 보조 진단이며 순위 지표가 아닙니다.</li>
</ul></section>
<script>document.querySelectorAll('button[data-show]').forEach(b=>b.onclick=()=>{{document.querySelectorAll('button').forEach(x=>x.classList.remove('active'));b.classList.add('active');const s=b.dataset.show;document.querySelectorAll('.scope').forEach(p=>p.style.display=s==='all'||p.dataset.scope===s?'block':'none')}});</script>
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
