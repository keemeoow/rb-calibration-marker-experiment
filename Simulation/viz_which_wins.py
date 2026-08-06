#!/usr/bin/env python3
"""
which-wins 시각화 — 4방법이 어떤 조건에서 이기나.
  fig_ww_sweeps.png : FK / 계통 / 랜덤 3패널 (e_task, 4방법)     [Fig1~3]
  fig_ww_grid.png   : FK오차 × 계통노이즈 승자 히트맵            [Fig4]
  fig_ww_metrics.png: 3 sweep × 4 지표 보충                      [supplementary]
dataviz: Okabe-Ito 색맹안전, 선스타일 이중부호, y캡+축밖주석, ↓lower=better.
  python viz_which_wins.py
"""
import os, json
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Patch

TBL = os.path.join(os.path.dirname(__file__), "results", "tables")
FIG = os.path.join(os.path.dirname(__file__), "results", "figures")

# (색, 선스타일, 굵기, 마커, 라벨)  — 색맹안전 Okabe-Ito
STYLE = {
    "fixed": ("#56B4E9", "-.", 1.8, "X", "fixed-FK"),
    "noFK":  ("#D55E00", "--", 1.8, "v", "no-FK"),
    "ours":  ("#0072B2", "-",  3.0, "o", "ours (de-bias + gate)"),
}
ORDER = ["fixed", "noFK", "ours"]     # ours 마지막(위)


def _cap(curves, key, exclude_hi=True):
    """관심 방법(ours/fixed) 최대의 1.25배를 y캡으로. no-FK 폭주는 축 밖."""
    base = ["fixed", "ours"] if exclude_hi else list(STYLE)
    mx = 0.0
    for n in base:
        ys = [y for y in curves[n][key] if y is not None]
        if ys:
            mx = max(mx, max(ys))
    return mx * 1.25 if mx > 0 else 1.0


def _plot_panel(ax, blob, key, title, unit, cap=None):
    curves = blob["methods"]
    levels = blob["levels"]
    cap = cap if cap is not None else _cap(curves, key)
    offscale = []
    for n in ORDER:
        col, ls, lw, mk, lab = STYLE[n]
        ys = curves[n][key]
        xs = [x for x, y in zip(levels, ys) if y is not None]
        yv = [y for y in ys if y is not None]
        if not yv:
            continue
        if np.median(yv) > cap:
            offscale.append((lab, col, max(yv)))
            continue
        z = 6 if n == "ours" else 3
        ax.plot(xs, yv, color=col, ls=ls, lw=lw, marker=mk, ms=6, zorder=z,
                label=lab, alpha=1.0 if n == "ours" else 0.9, clip_on=True)
    ax.set_ylim(0, cap)
    ax.set_title(title, fontsize=11, fontweight="bold", loc="left")
    ax.set_xlabel(blob["xlabel"], fontsize=9)
    ax.set_ylabel(unit, fontsize=9)
    ax.text(1.0, 1.015, "↓ lower = better", transform=ax.transAxes, fontsize=8.5,
            color="#2a8a55", ha="right", va="bottom", fontweight="bold")
    ax.grid(axis="y", alpha=0.25, lw=0.7)
    ax.spines[["top", "right"]].set_visible(False)
    if offscale:
        txt = "off-scale:\n" + "\n".join(f"  {l} →{v:.0f}{unit}" for l, c, v in offscale)
        ax.text(0.97, 0.03, txt, transform=ax.transAxes, fontsize=7.5, ha="right",
                va="bottom", color="#a03", bbox=dict(boxstyle="round,pad=0.3",
                fc="#fff3f0", ec="#e0b0a0"))


def fig_sweeps():
    axes_defs = [("fk", "(1) FK error sweep  (clean camera, 0.3px)"),
                 ("sys", "(2) Systematic camera-noise sweep  (perfect FK)"),
                 ("rand", "(3) Random pixel-noise sweep  (perfect FK)")]
    fig, axs = plt.subplots(1, 3, figsize=(16.5, 5.2))
    for ax, (axis, title) in zip(axs, axes_defs):
        blob = json.load(open(os.path.join(TBL, f"ww_{axis}.json")))
        _plot_panel(ax, blob, "e_task_mm", title, "mm")
    # 공용 범례
    handles = [plt.Line2D([0], [0], color=STYLE[n][0], ls=STYLE[n][1],
               lw=(3 if n == "ours" else 1.9), marker=STYLE[n][3], ms=7,
               label=STYLE[n][4]) for n in ORDER]
    fig.legend(handles=[h for h in handles], loc="upper center", ncol=4,
               frameon=False, fontsize=10.5, bbox_to_anchor=(0.5, 1.06))
    meta = blob["meta"]
    fig.suptitle(f"Held-out cube prediction e_task  |  4 methods  "
                 f"({meta['seeds']} seeds × {meta['pairs']} holdout pairs)",
                 y=1.10, fontsize=12.5, fontweight="bold")
    fig.tight_layout(rect=[0, 0, 1, 1.0])
    os.makedirs(FIG, exist_ok=True)
    out = os.path.join(FIG, "fig_ww_sweeps.png")
    fig.savefig(out, dpi=130, bbox_inches="tight")
    print(f"[저장] {out}")


def fig_grid():
    blob = json.load(open(os.path.join(TBL, "ww_grid.json")))
    fkL, syL = blob["x_levels"], blob["y_levels"]
    winner, margin = blob["winner"], blob["margin"]
    color = {n: STYLE[n][0] for n in STYLE}
    fig, ax = plt.subplots(figsize=(8.6, 6.4))
    nx, ny = len(fkL), len(syL)
    for xi in range(nx):
        for yi in range(ny):
            w = winner[xi][yi]
            if w is None:
                continue
            ax.add_patch(plt.Rectangle((yi - 0.5, xi - 0.5), 1, 1,
                         facecolor=color[w], edgecolor="white", lw=2, alpha=0.92))
            et = blob["methods"][w]["e_task_mm"][xi][yi]
            mg = margin[xi][yi]
            lab = STYLE[w][4].split(" ")[0]
            txt = f"{lab}\n{et:.1f}mm"
            if mg is not None:
                txt += f"\n(+{mg:.1f})"
            ax.text(yi, xi, txt, ha="center", va="center", fontsize=8.5,
                    color="white", fontweight="bold")
    ax.set_xticks(range(ny)); ax.set_xticklabels([f"{s:.0%}" if s else "0" for s in syL])
    ax.set_yticks(range(nx)); ax.set_yticklabels([f"{f:.0f}" for f in fkL])
    ax.set_xlabel("Systematic camera noise (intrinsic; outlier=x5)  ->", fontsize=10)
    ax.set_ylabel("FK error (mm)  ->", fontsize=10)
    ax.set_xlim(-0.5, ny - 0.5); ax.set_ylim(-0.5, nx - 0.5)
    ax.set_title("Winner map - lowest held-out e_task per condition\n(cell: method / e_task / margin to 2nd mm)",
                 fontsize=12, fontweight="bold", loc="left")
    ax.set_aspect("equal")
    legend = [Patch(facecolor=color[n], label=STYLE[n][4]) for n in ORDER]
    ax.legend(handles=legend, loc="upper left", bbox_to_anchor=(1.02, 1.0),
              frameon=False, fontsize=9.5, title="method")
    fig.tight_layout()
    out = os.path.join(FIG, "fig_ww_grid.png")
    fig.savefig(out, dpi=130, bbox_inches="tight")
    print(f"[저장] {out}")


def fig_metrics():
    METS = [("e_task_mm", "e_task", "mm"), ("gTc_mm", "hand-eye gTc", "mm"),
            ("e_X_mm", "camera e_X", "mm"), ("e_reproj_px", "reproj", "px")]
    AX = [("fk", "FK error"), ("sys", "systematic noise"), ("rand", "random px")]
    fig, axs = plt.subplots(len(AX), len(METS), figsize=(18, 11))
    for ri, (axis, atitle) in enumerate(AX):
        blob = json.load(open(os.path.join(TBL, f"ww_{axis}.json")))
        for ci, (key, mt, unit) in enumerate(METS):
            _plot_panel(axs[ri][ci], blob, key, f"{atitle} · {mt}", unit)
    handles = [plt.Line2D([0], [0], color=STYLE[n][0], ls=STYLE[n][1],
               lw=(3 if n == "ours" else 1.9), marker=STYLE[n][3], ms=7,
               label=STYLE[n][4]) for n in ORDER]
    fig.legend(handles=handles, loc="upper center", ncol=4, frameon=False,
               fontsize=11, bbox_to_anchor=(0.5, 1.02))
    fig.tight_layout(rect=[0, 0, 1, 0.985])
    out = os.path.join(FIG, "fig_ww_metrics.png")
    fig.savefig(out, dpi=120, bbox_inches="tight")
    print(f"[저장] {out}")


if __name__ == "__main__":
    fig_sweeps()
    fig_grid()
    fig_metrics()
