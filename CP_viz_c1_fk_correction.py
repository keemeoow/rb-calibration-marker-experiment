#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Generate the real-data C1/C3 paper figures from recorded result files.

No metric is recomputed from images and no uncertainty is synthesized.  C1 values
come from ``CP_result/C1/joint_ablation_summary.csv`` and C3 values come from
``CP_result/C3/ablation_summary.json``.

The C1 implementation fits the two post-corrections independently to the same raw
training predictions (``learn_fk_rigid`` and ``learn_fk_ridge`` are sibling calls in
``CP_C1_unified_vs_independent.py``).  Consequently Figure 1 presents a parallel
Raw -> SE(3) / Raw -> Ridge comparison, never an SE(3) -> Ridge sequence.

Run:
    PYTHONPATH= python CP_viz_c1_fk_correction.py
"""
from __future__ import annotations

import csv
import json
from pathlib import Path
from typing import Iterable

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

from figure_style import (
    INK, METHOD_COLORS as SHARED_METHOD_COLORS, MUTED, PAPER, SERIES_COLORS,
    apply_paper_style, clean_axis, save_figure,
)

ROOT = Path(__file__).resolve().parent
C1_CSV = ROOT / "CP_result" / "C1" / "joint_ablation_summary.csv"
C3_JSON = ROOT / "CP_result" / "C3" / "ablation_summary.json"
FIG_DIR = ROOT / "CP_result" / "figures"

OUT_C1 = FIG_DIR / "fig_CP_C1_fk_correction.png"
OUT_INTERNAL = FIG_DIR / "fig_CP_C1_internal_metrics.png"
OUT_LINK = FIG_DIR / "fig_CP_C1_C3_interpretation.png"
OUT_PROVENANCE = FIG_DIR / "fig_CP_C1_real_data_provenance.json"

METHODS = ["independent", "unified_joint", "joint_fk_fixed"]
METHOD_COLORS = SHARED_METHOD_COLORS
SERIES = [
    ("downstream_trans_rmse_mm", "Raw calibration prediction\n(no additional FK post-correction)\n캘리브레이션 원출력 · 추가 FK 후보정 없음", SERIES_COLORS["raw"]),
    ("downstream_se3_trans_rmse_mm", "+ FK-supervised SE(3) alignment", SERIES_COLORS["se3"]),
    ("downstream_fk_trans_rmse_mm", "+ FK-supervised Ridge correction [1,x,y]", SERIES_COLORS["ridge"]),
]
TARGET_MM = 5.0


def _setup_style() -> None:
    apply_paper_style()


def _clean_axis(ax, grid: bool = True) -> None:
    clean_axis(ax, grid_axis="y" if grid else None)


def _load_c1() -> list[dict[str, str]]:
    with C1_CSV.open(newline="", encoding="utf-8") as f:
        rows = list(csv.DictReader(f))
    by_method = {r["method"]: r for r in rows}
    missing_methods = [m for m in METHODS if m not in by_method]
    required = {
        "consistency_trans_rmse_mm", "consistency_rot_rmse_deg",
        "grip_align_trans_rmse_mm", "downstream_trans_rmse_mm",
        "downstream_se3_trans_rmse_mm", "downstream_fk_trans_rmse_mm",
        "train_sets", "test_sets",
    }
    missing_columns = sorted(required - set(rows[0]) if rows else required)
    if missing_methods or missing_columns:
        raise SystemExit(
            f"C1 source is incomplete; missing methods={missing_methods}, columns={missing_columns}"
        )
    return [by_method[m] for m in METHODS]


def _load_c3() -> dict[str, dict]:
    with C3_JSON.open(encoding="utf-8") as f:
        rows = json.load(f)

    def select(method: str, prior: str) -> dict:
        matches = [r for r in rows if r.get("method") == method and r.get("prior_mode") == prior]
        if len(matches) != 1 or matches[0].get("test_prior_trans_rmse_mm") is None:
            raise SystemExit(f"C3 source row unavailable or ambiguous: {method} / {prior}")
        return matches[0]

    return {
        "vision_only": select("03_pose_consistency_opt", "without_robot_cube_prior"),
        "fk_prior": select("03_pose_consistency_opt", "with_robot_cube_prior"),
        "post_correction": select("05_fk_prior_correction", "with_robot_cube_prior"),
    }


def _f(row: dict, key: str) -> float:
    value = row.get(key)
    if value in (None, ""):
        raise SystemExit(f"Required measured value is absent: {key}")
    return float(value)


def _pct_drop(before: float, after: float) -> float:
    return (1.0 - after / before) * 100.0


def _save(fig, path: Path) -> None:
    save_figure(fig, path)
    print(f"[DONE] {path}")


def figure_1(rows: list[dict[str, str]]) -> None:
    """C1 held-out agreement: verified parallel raw/SE(3)/Ridge branches."""
    fig = plt.figure(figsize=(14.4, 8.3), layout="constrained")
    gs = fig.add_gridspec(2, 2, width_ratios=[1.62, 1.0], height_ratios=[1.0, 0.17])
    ax = fig.add_subplot(gs[0, 0])
    info = fig.add_subplot(gs[0, 1])
    foot = fig.add_subplot(gs[1, :])
    info.axis("off")
    foot.axis("off")

    width = 0.24
    x = list(range(len(rows)))
    raw = [_f(r, "downstream_trans_rmse_mm") for r in rows]
    for j, (column, label, color) in enumerate(SERIES):
        values = [_f(r, column) for r in rows]
        bars = ax.bar([v + (j - 1) * width for v in x], values, width=width,
                      color=color, edgecolor="white", linewidth=0.7, zorder=3, label=label)
        for i, (bar, value) in enumerate(zip(bars, values)):
            ax.text(bar.get_x() + bar.get_width() / 2, value + 0.27, f"{value:.2f}",
                    ha="center", va="bottom", fontsize=9.6, fontweight="semibold",
                    bbox=dict(facecolor=PAPER, edgecolor="none", pad=0.5))
            if j > 0:
                ax.text(bar.get_x() + bar.get_width() / 2, max(0.35, value - 0.48),
                        f"−{_pct_drop(raw[i], value):.1f}%",
                        ha="center", va="top", fontsize=7.8, color="white", fontweight="semibold")

    ax.axhline(TARGET_MM, color="#8A4F2D", lw=1.25, ls=(0, (5, 3)), zorder=2)
    ax.text(2.43, TARGET_MM + 0.15, "5 mm FK-proxy agreement target",
            ha="right", va="bottom", fontsize=9, color="#8A4F2D")
    ax.set_xticks(x, METHODS)
    ax.set_ylabel("Held-out RMSE vs FK proxy (mm)")
    ax.set_ylim(0, max(raw) * 1.23)
    ax.set_title("C1 — Held-out Cube Position Agreement with FK Proxy", loc="left",
                 fontsize=16, fontweight="semibold", pad=30)
    ax.text(0, 1.025, "캘리브레이션 구조 및 FK-supervised 후보정에 따른 held-out 위치 RMSE",
            transform=ax.transAxes, ha="left", va="bottom", fontsize=11, color=MUTED)
    _clean_axis(ax)
    ax.legend(loc="upper right", frameon=False, fontsize=9.1, labelspacing=0.8)

    info.text(0.0, 0.98, "Calibration structures", fontsize=12.5, fontweight="semibold", va="top")
    method_text = (
        "independent\n"
        "Fixed cameras solved independently using FK cube references;\n"
        "gripper-camera hand–eye estimated separately; no joint exchange.\n\n"
        "unified_joint\n"
        "Jointly optimizes fixed-camera extrinsics, gripper-camera hand–eye,\n"
        "and free per-set cube poses; FK is a soft anchor.\n\n"
        "joint_fk_fixed\n"
        "Jointly optimizes fixed-camera extrinsics and gripper-camera hand–eye;\n"
        "per-set cube poses are fixed to FK values."
    )
    info.text(0.0, 0.91, method_text, fontsize=9.4, va="top", linespacing=1.33)
    info.text(0.0, 0.49, "C1 compares calibration structures, not FK versus no-FK systems.",
              fontsize=10.3, fontweight="semibold", va="top", wrap=True)
    info.text(0.0, 0.43, "C1은 FK 사용 여부가 아니라 FK를 포함한 캘리브레이션 구조의 차이를 비교한다.",
              fontsize=9.5, color=MUTED, va="top", wrap=True)
    info.text(0.0, 0.32, "Verified correction topology", fontsize=11.2,
              fontweight="semibold", va="top")
    info.text(0.0, 0.265,
              "Raw → FK-supervised SE(3)\nRaw → FK-supervised Ridge [1,x,y]\n\n"
              "Both corrections are independently learned from the raw train-set\n"
              "predictions. Percentages therefore use Raw as the only baseline.",
              fontsize=9.2, va="top", linespacing=1.38)

    conclusion = (
        "FK-supervised Ridge post-correction reduced held-out disagreement with the FK proxy "
        "from 11.5–13.4 mm to 2.5–2.8 mm on this split.\n"
        "해당 split에서 FK-supervised Ridge 후보정은 FK proxy 대비 held-out 불일치를 "
        "11.5–13.4 mm에서 2.5–2.8 mm로 감소시켰다."
    )
    foot.text(0.0, 0.78, conclusion, fontsize=10.2, va="top", linespacing=1.35,
              bbox=dict(boxstyle="round,pad=0.55", facecolor="#F3F6F8", edgecolor="#CCD5DC"))
    foot.text(0.0, 0.02,
              "Real-data evaluation uses the robot FK cube center as a proxy, not an independent physical ground truth.  "
              "Reported improvements quantify FK-proxy agreement, not absolute physical accuracy.",
              fontsize=8.8, color=MUTED, va="bottom")
    _save(fig, OUT_C1)


def figure_2(rows: list[dict[str, str]]) -> None:
    """C1 internal metrics, kept separate from held-out evaluation."""
    metrics = [
        ("consistency_trans_rmse_mm", "Translation consistency", "mm"),
        ("consistency_rot_rmse_deg", "Rotation consistency", "deg"),
        ("grip_align_trans_rmse_mm", "Gripper alignment", "mm"),
    ]
    fig, axes = plt.subplots(1, 3, figsize=(14.2, 5.8), layout="constrained")
    fig.suptitle("Internal subsystem consistency after calibration", fontsize=16,
                 fontweight="semibold")
    for ax, (key, title, unit) in zip(axes, metrics):
        values = [_f(r, key) for r in rows]
        bars = ax.bar(range(3), values, color=[METHOD_COLORS[m] for m in METHODS],
                      width=0.65, zorder=3)
        for bar, value in zip(bars, values):
            ax.text(bar.get_x() + bar.get_width() / 2, value + max(values) * 0.025,
                    f"{value:.2f}", ha="center", va="bottom", fontsize=10,
                    fontweight="semibold")
        ax.set_xticks(range(3), METHODS, rotation=18, ha="right")
        ax.set_title(f"{title} ({unit})", fontsize=12)
        ax.set_ylim(0, max(values) * 1.18)
        _clean_axis(ax)

    fig.text(0.5, -0.03,
             "unified_joint provides the best internal alignment, but the margin is modest.  "
             "Better train/internal consistency does not automatically imply better held-out position prediction.\n"
             "Raw held-out RMSE differs by only 0.14 mm between unified_joint (11.67 mm) and "
             "joint_fk_fixed (11.53 mm); this split does not establish clearly superior held-out generalization.",
             ha="center", va="top", fontsize=9.4, color=MUTED, linespacing=1.4)
    _save(fig, OUT_INTERNAL)


def _bar_panel(ax, labels: Iterable[str], values: Iterable[float], colors: Iterable[str],
               title: str, ylabel: str) -> None:
    labels, values, colors = list(labels), list(values), list(colors)
    bars = ax.bar(range(len(values)), values, color=colors, width=0.62, zorder=3)
    for bar, value in zip(bars, values):
        ax.text(bar.get_x() + bar.get_width() / 2, value + max(values) * 0.025,
                f"{value:.2f}", ha="center", va="bottom", fontsize=10,
                fontweight="semibold")
    ax.set_xticks(range(len(labels)), labels)
    ax.set_ylim(0, max(values) * 1.2)
    ax.set_ylabel(ylabel)
    ax.set_title(title, loc="left", fontsize=12.5, fontweight="semibold")
    _clean_axis(ax)


def figure_3(rows: list[dict[str, str]], c3: dict[str, dict]) -> None:
    """Directionally connect C1 and C3 while preserving their metric definitions."""
    fig = plt.figure(figsize=(14.5, 9.2), layout="constrained")
    gs = fig.add_gridspec(2, 2, height_ratios=[1.0, 0.92])
    ax1 = fig.add_subplot(gs[0, 0])
    ax2 = fig.add_subplot(gs[0, 1])
    limits = fig.add_subplot(gs[1, :])
    limits.axis("off")
    fig.suptitle("C1–C3 interpretation — solve-time FK prior vs post-correction",
                 fontsize=16, fontweight="semibold")

    # Separate C1 and C3 axes: their units share 'mm' but their samples and estimands differ.
    c1_raw = [_f(r, "downstream_trans_rmse_mm") for r in rows]
    c1_ridge = [_f(r, "downstream_fk_trans_rmse_mm") for r in rows]
    width = 0.34
    x = range(3)
    for offset, values, label, color in [
        (-width / 2, c1_raw, "Raw calibration prediction\n(no additional FK post-correction)", "#AEB8C2"),
        (width / 2, c1_ridge, "+ FK-supervised Ridge", "#1F5A94"),
    ]:
        bars = ax1.bar([i + offset for i in x], values, width=width, color=color,
                       label=label, zorder=3)
        for bar, value in zip(bars, values):
            ax1.text(bar.get_x() + bar.get_width() / 2, value + 0.23, f"{value:.2f}",
                     ha="center", va="bottom", fontsize=9.3)
    ax1.set_xticks(list(x), METHODS, rotation=12, ha="right")
    ax1.set_ylim(0, max(c1_raw) * 1.2)
    ax1.set_ylabel("Held-out RMSE vs FK proxy (mm)")
    ax1.set_title("C1 · set-level · fixed + gripper cameras", loc="left",
                  fontsize=12.5, fontweight="semibold")
    ax1.legend(frameon=False, fontsize=9)
    _clean_axis(ax1)

    c3_values = [_f(c3[k], "test_prior_trans_rmse_mm")
                 for k in ("vision_only", "fk_prior", "post_correction")]
    _bar_panel(
        ax2,
        ["no-fk-prior\nvision-only", "fk-prior", "fk-prior\n+ post-correction"],
        c3_values,
        ["#6D7D8B", "#C46A4A", "#2F7D67"],
        "C3 · event-level · fixed cameras only",
        "Held-out prior translation RMSE (mm)",
    )
    ax2.text(0.03, 0.93,
             f"solve-time prior: {c3_values[0]:.2f} → {c3_values[1]:.2f} mm (worse)\n"
             f"post-correction: {c3_values[1]:.2f} → {c3_values[2]:.2f} mm\n"
             f"{_pct_drop(c3_values[0], c3_values[2]):.1f}% below vision-only on this split",
             transform=ax2.transAxes, va="top", fontsize=9.1,
             bbox=dict(boxstyle="round,pad=0.4", facecolor="#F5F7F8", edgecolor="#D2D9DE"))

    limits.text(0.0, 0.98,
                "Forcing the FK proxy into the solve did not improve held-out performance, whereas using it as "
                "post-correction supervision was beneficial on this split.",
                fontsize=11.2, fontweight="semibold", va="top")
    limits.text(0.0, 0.86,
                "C1 and C3 are shown on separate axes because their metric definitions and aggregation units differ; "
                "only the direction of the post-correction result is linked.",
                fontsize=9.5, color=MUTED, va="top")

    limitations = (
        "Evaluation protocol and limitations\n"
        "• Real-data evaluation uses the robot FK cube center as a proxy, not an independent physical ground truth.\n"
        "• Single split: 9 training sets / 4 held-out sets.\n"
        "• FK rotation convention was corrected from the observed 179.8° mismatch; robust averaging was enabled.\n"
        "• Reported improvements measure agreement with the FK proxy and do not, by themselves, establish absolute physical accuracy.\n"
        "• Ridge captures position-dependent residuals relative to the FK proxy. The observed dependence may combine camera calibration, "
        "robot FK, and grasp-repeatability effects."
    )
    follow_up = (
        "Required follow-up validation\n"
        "• repeated hold-out or leave-one-set-out validation\n"
        "• mean ± standard deviation\n"
        "• cross-validated R² for Ridge residual prediction\n"
        "• permutation test\n"
        "• evaluation against independent external GT\n"
        "• coefficient stability across splits"
    )
    limits.text(0.0, 0.70, limitations, fontsize=9.2, va="top", linespacing=1.45,
                bbox=dict(boxstyle="round,pad=0.6", facecolor="#F5F7F8", edgecolor="#CCD5DC"))
    limits.text(0.70, 0.70, follow_up, fontsize=9.2, va="top", linespacing=1.45,
                bbox=dict(boxstyle="round,pad=0.6", facecolor="#FFF9ED", edgecolor="#DFCDAA"))
    limits.text(0.0, 0.04,
                "No error bars, p-values, residual distributions, or external-ground-truth claims are shown because "
                "those measurements are absent from the source result files.",
                fontsize=8.8, color=MUTED, va="bottom")
    _save(fig, OUT_LINK)


def write_provenance(rows: list[dict[str, str]], c3: dict[str, dict]) -> None:
    payload = {
        "figure_1_source": str(C1_CSV.relative_to(ROOT)),
        "figure_2_source": str(C1_CSV.relative_to(ROOT)),
        "figure_3_sources": [str(C1_CSV.relative_to(ROOT)), str(C3_JSON.relative_to(ROOT))],
        "c1_source_columns": {
            "raw": "downstream_trans_rmse_mm",
            "se3": "downstream_se3_trans_rmse_mm",
            "ridge": "downstream_fk_trans_rmse_mm",
            "internal_translation": "consistency_trans_rmse_mm",
            "internal_rotation": "consistency_rot_rmse_deg",
            "gripper_alignment": "grip_align_trans_rmse_mm",
        },
        "se3_ridge_topology": "parallel: raw->SE(3), raw->Ridge",
        "topology_evidence": {
            "file": "CP_C1_unified_vs_independent.py",
            "description": "learn_fk_ridge and learn_fk_rigid are sibling fits; downstream_rmse receives W or T_rigid independently",
        },
        "train_sets": rows[0]["train_sets"],
        "test_sets": rows[0]["test_sets"],
        "c3_selected_rows": {
            key: {"method": value["method"], "prior_mode": value["prior_mode"],
                  "source_column": "test_prior_trans_rmse_mm"}
            for key, value in c3.items()
        },
        "residual_plot": "omitted: source summaries contain no per-set x/y/z residual records",
        "uncertainty": "not shown: unavailable in source files",
    }
    OUT_PROVENANCE.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(f"[DONE] {OUT_PROVENANCE}")


def main() -> None:
    _setup_style()
    c1 = _load_c1()
    c3 = _load_c3()
    figure_1(c1)
    figure_2(c1)
    figure_3(c1, c3)
    write_provenance(c1, c3)


if __name__ == "__main__":
    main()
