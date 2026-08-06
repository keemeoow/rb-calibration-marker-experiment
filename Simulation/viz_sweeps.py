#!/usr/bin/env python3
"""
sweep 곡선 그림 — run_sweeps.py 의 JSON(코너/FK 축)에서 지표별 패널로 7방식 곡선 렌더.
dataviz 원칙 적용: 색맹안전(Okabe-Ito), y축 캡(붕괴는 주석), Ours 강조, 직접 끝라벨.

  python viz_sweeps.py --json results/tables/sweep_corner.json
  python viz_sweeps.py --json results/tables/sweep_fk.json
"""
import sys, os, json, argparse
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

FIG_DIR = os.path.join(os.path.dirname(__file__), "results", "figures")

# Okabe-Ito 색맹 안전 팔레트 + 선 스타일로 FK 방식 인코딩(solid=corr, dashed=none, dotted=fixed)
STYLE = {
    "EXP1": ("#0072B2", "-",  3.0, "o", "Ours"),                 # 파랑 굵게
    "EXP2": ("#E69F00", "-",  1.6, "s", "−unified"),             # 주황 (corr)
    "EXP3": ("#009E73", "-",  1.6, "^", "−board"),               # 초록 (corr)
    "EXP4": ("#D55E00", "--", 1.6, "v", "−FK"),                  # 주홍 (none, dashed)
    "EXP5": ("#CC79A7", "--", 1.6, "D", "−FK−unified"),          # 자주 (none, dashed)
    "EXP6": ("#000000", ":",  1.6, "P", "−cube (board-only)"),   # 검정 (none, dotted)
    "EXP7": ("#56B4E9", "-.", 2.0, "X", "FK-fixed"),             # 하늘 (fixed, dash-dot)
}
ORDER = ["EXP4", "EXP5", "EXP2", "EXP3", "EXP7", "EXP1"]   # Ours 마지막(맨 위)
# 각 지표: (key, 제목, 단위). y캡은 데이터에서 자동(붕괴 EXP6 제외한 최대의 1.15배).
PANELS = [
    ("e_task_mm", "Held-out cube prediction  e_task", "mm"),
    ("gTc_mm", "Hand-eye  gTc", "mm"),
    ("e_X_mm", "Camera+hand-eye  e_X", "mm"),
    ("e_cross_mm", "Cross-camera consistency  e_cross", "mm"),
    ("e_reproj_px", "Unified reproj (cube+board)", "px"),
]
# 붕괴로 간주할 방식(항상 축 밖 처리) — 캘리브 실패해 값이 발산
COLLAPSE = {"EXP6"}


def _auto_cap(curves, key):
    """붕괴(EXP6) 제외한 방식들의 최대값 → 그 1.15배를 y캡으로."""
    mx = 0.0
    for name in STYLE:
        if name in COLLAPSE:
            continue
        ys = [y for y in curves[name][key] if y is not None]
        if ys:
            mx = max(mx, max(ys))
    return mx * 1.15 if mx > 0 else 1.0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--json", required=True)
    args = ap.parse_args()
    blob = json.load(open(args.json))
    axis, unit = blob["axis"], blob["unit"]
    levels, curves = blob["levels"], blob["curves"]
    XLAB = {"corner": "corner noise  σ (px)", "fk": "FK error (mm)",
            "intrinsic": "intrinsic error (relative)", "outlier": "outlier rate"}
    xlabel = XLAB.get(axis, f"{axis} ({unit})")
    # 실측 지점 수직선 (해당 축에서 실제 값)
    X0 = {"corner": 0.3, "intrinsic": None, "outlier": None, "fk": None}
    x0 = X0.get(axis)

    fig, axes = plt.subplots(2, 3, figsize=(17, 9.5))
    axes = axes.flat
    for ax, (key, title, yu) in zip(axes, PANELS):
        cap = _auto_cap(curves, key)
        offscale = []
        for name in ORDER:
            col, ls, lw, mk, lab = STYLE[name]
            ys = curves[name][key]
            xs = [x for x, y in zip(levels, ys) if y is not None]
            yv = [y for y in ys if y is not None]
            if not yv:
                continue
            # 값이 캡을 크게 넘으면(붕괴) 축 밖 → 주석. 아니면 그린다.
            if np.median(yv) > cap:
                offscale.append((lab, max(yv)))
                continue
            z = 5 if name == "EXP1" else 2
            ax.plot(xs, yv, color=col, ls=ls, lw=lw, marker=mk, ms=6,
                    zorder=z, label=lab, alpha=(1.0 if name == "EXP1" else 0.85),
                    clip_on=True)
            # 직접 끝 라벨 (마지막 점)
            if name == "EXP1":
                ax.annotate(lab, (xs[-1], min(yv[-1], cap * 0.98)), fontsize=9,
                            fontweight="bold", color=col, va="center",
                            xytext=(6, 0), textcoords="offset points")
        ax.set_ylim(0, cap)
        ax.set_xlabel(xlabel, fontsize=9)
        ax.set_ylabel(yu, fontsize=9)
        ax.set_title(title, fontsize=11, fontweight="bold", loc="left")
        # 방향 표시: 이 6개 지표는 모두 낮을수록 좋음
        ax.text(1.0, 1.015, "↓ lower = better", transform=ax.transAxes,
                fontsize=8.5, color="#2a8a55", ha="right", va="bottom",
                fontweight="bold")
        ax.grid(axis="y", alpha=0.25, lw=0.7)
        ax.spines[["top", "right"]].set_visible(False)
        if x0 is not None:
            ax.axvline(x0, color="gray", ls="--", alpha=0.4, lw=0.8, zorder=1)
            ax.text(x0, cap * 0.97, " real σ", fontsize=7.5, color="gray", va="top")
        # 축 밖(붕괴) 주석
        if offscale:
            txt = "off-scale (collapse):\n" + "\n".join(
                f"  {l} →{v:.0f}{yu}" for l, v in offscale)
            ax.text(0.98, 0.02, txt, transform=ax.transAxes, fontsize=7.5,
                    ha="right", va="bottom", color="#555",
                    bbox=dict(boxstyle="round,pad=0.3", fc="#f4f4f4", ec="#ccc"))

    # 마지막 칸: 방식별 상세 표 (색선 + FK방식 / 통합·독립 / 마커)
    lax = axes[5]
    lax.axis("off")
    # 각 방식의 세 축 정보 (configs.py 와 일치)
    INFO = {   # name: (FK, solve, markers)
        "EXP1": ("FK-corr", "unified", "cube+board"),
        "EXP2": ("FK-corr", "separate", "cube+board"),
        "EXP3": ("FK-corr", "unified", "cube-only"),
        "EXP4": ("no-FK",   "unified", "cube+board"),
        "EXP5": ("no-FK",   "separate", "cube+board"),
        "EXP6": ("no-FK",   "unified", "board-only"),
        "EXP7": ("FK-fixed", "unif=sep", "cube+board"),
    }
    order = ["EXP1", "EXP2", "EXP3", "EXP4", "EXP5", "EXP6", "EXP7"]
    # 표 헤더
    lax.text(0.02, 0.96, "Method", fontsize=9, fontweight="bold", transform=lax.transAxes)
    lax.text(0.34, 0.96, "FK", fontsize=9, fontweight="bold", transform=lax.transAxes)
    lax.text(0.55, 0.96, "solve", fontsize=9, fontweight="bold", transform=lax.transAxes)
    lax.text(0.76, 0.96, "markers", fontsize=9, fontweight="bold", transform=lax.transAxes)
    lax.plot([0.02, 0.98], [0.925, 0.925], color="#999", lw=0.8, transform=lax.transAxes)
    y = 0.86
    for n in order:
        col, ls, lw, mk, lab = STYLE[n]
        fk, solve, mkr = INFO[n]
        bold = (n == "EXP1")
        # 색선 샘플
        lax.plot([0.02, 0.10], [y, y], color=col, ls=ls,
                 lw=(2.8 if bold else 1.7), marker=mk, ms=6,
                 transform=lax.transAxes, clip_on=False)
        w = "bold" if bold else "normal"
        lax.text(0.13, y, f"{n}" + (" ★" if bold else ""), fontsize=9,
                 fontweight=w, va="center", transform=lax.transAxes, color=col)
        lax.text(0.34, y, fk, fontsize=8.5, va="center", transform=lax.transAxes, fontweight=w)
        lax.text(0.55, y, solve, fontsize=8.5, va="center", transform=lax.transAxes, fontweight=w)
        lax.text(0.76, y, mkr, fontsize=8.5, va="center", transform=lax.transAxes, fontweight=w)
        y -= 0.115
    lax.text(0.02, -0.04,
             "line style:  solid = FK-corr,  dashed = no-FK,  dash-dot = FK-fixed",
             fontsize=7.5, color="#666", transform=lax.transAxes, style="italic")

    bg = blob["meta"].get("bg_intrinsic", 0), blob["meta"].get("bg_outlier", 0)
    bgtxt = ""
    if bg[0] or bg[1]:
        bgtxt = f"   [background noise: intrinsic {bg[0]:.0%}, outlier {bg[1]:.0%}]"
    fig.suptitle(
        f"Noise sweep — {XLAB.get(axis, axis)}   |   "
        f"7 methods, {blob['meta']['seeds']} seeds × {blob['meta']['pairs']} holdout pairs   "
        f"(lower = better){bgtxt}",
        fontsize=13, fontweight="bold")
    fig.tight_layout(rect=[0, 0, 1, 0.96])
    os.makedirs(FIG_DIR, exist_ok=True)
    out = os.path.join(FIG_DIR, f"sweep_{axis}.png")
    fig.savefig(out, dpi=130, bbox_inches="tight")
    print(f"[저장] {out}")


if __name__ == "__main__":
    main()
