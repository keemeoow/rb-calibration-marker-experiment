#!/usr/bin/env python3
"""
paper_sim.json → 그림 A(노이즈 sweep 4패널) + 그림 B(승자 히트맵) + 표1·1b(markdown).
7방법(EXP1~7). dataviz: Okabe-Ito, FK방식 선스타일, y캡+축밖주석.
  python viz_paper_sim.py
"""
import os, json
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Patch

TBL = os.path.join(os.path.dirname(__file__), "results", "tables")
FIG = os.path.join(os.path.dirname(__file__), "results", "figures")

# (색, 선스타일, 굵기, 마커, 짧은라벨) — solid=corr, dashed=none, dashdot=fixed
STYLE = {
    "EXP1": ("#0072B2", "-",  3.0, "o", "Ours"),
    "EXP2": ("#E69F00", "-",  1.6, "s", "-unified"),
    "EXP3": ("#009E73", "-",  1.6, "^", "-board"),
    "EXP4": ("#D55E00", "--", 1.6, "v", "-FK"),
    "EXP5": ("#CC79A7", "--", 1.6, "D", "-FK-unified"),
    "EXP6": ("#000000", ":",  1.6, "P", "-cube(board-only)"),
    "EXP7": ("#56B4E9", "-.", 2.0, "X", "fixed-FK"),
}
ORDER = ["EXP4", "EXP5", "EXP2", "EXP3", "EXP7", "EXP6", "EXP1"]
COLLAPSE = {"EXP6"}
AXES = [("sigma", "marker corner sigma (px)"), ("sys", "systematic intrinsic err"),
        ("fk", "FK error (mm, random)"), ("outl", "outlier rate")]


def _cap(blob, keys):
    mx = 0.0
    for name in STYLE:
        if name in COLLAPSE:
            continue
        ys = [blob["results"][k][name]["e_task_mm"] for k in keys]
        ys = [y for y in ys if y is not None]
        if ys:
            mx = max(mx, max(ys))
    return mx * 1.2 if mx > 0 else 1.0


def fig_A(blob):
    lay = blob["layout"]["figA"]
    fig, axs = plt.subplots(1, 4, figsize=(19, 5))
    for ax, (axis, xlabel) in zip(axs, AXES):
        levels = lay[axis]["levels"]; keys = lay[axis]["keys"]
        cap = _cap(blob, keys); off = []
        for name in ORDER:
            col, ls, lw, mk, lab = STYLE[name]
            ys = [blob["results"][k][name]["e_task_mm"] for k in keys]
            xs = [x for x, y in zip(levels, ys) if y is not None]
            yv = [y for y in ys if y is not None]
            if not yv:
                continue
            if np.median(yv) > cap:
                off.append((lab, max(yv))); continue
            z = 6 if name == "EXP1" else 2
            ax.plot(xs, yv, color=col, ls=ls, lw=lw, marker=mk, ms=6, zorder=z,
                    label=lab, alpha=1.0 if name == "EXP1" else 0.85)
        ax.set_ylim(0, cap)
        ax.set_xlabel(xlabel, fontsize=9); ax.set_ylabel("e_task (mm, GT)", fontsize=9)
        ax.set_title(xlabel.split("(")[0].strip(), fontsize=11, fontweight="bold", loc="left")
        ax.text(1.0, 1.015, "(lower is better)", transform=ax.transAxes, fontsize=8,
                color="#2a8a55", ha="right", va="bottom", fontweight="bold")
        ax.grid(axis="y", alpha=0.25); ax.spines[["top", "right"]].set_visible(False)
        if off:
            txt = "off-scale:\n" + "\n".join(f"  {l} ->{v:.0f}" for l, v in off)
            ax.text(0.97, 0.03, txt, transform=ax.transAxes, fontsize=7, ha="right",
                    va="bottom", color="#a03", bbox=dict(boxstyle="round,pad=0.3", fc="#fff3f0", ec="#e0b0a0"))
    handles = [plt.Line2D([0], [0], color=STYLE[n][0], ls=STYLE[n][1],
               lw=(3 if n == "EXP1" else 1.7), marker=STYLE[n][3], ms=6,
               label=f"{n} {STYLE[n][4]}") for n in ["EXP1","EXP2","EXP3","EXP4","EXP5","EXP6","EXP7"]]
    fig.legend(handles=handles, loc="upper center", ncol=7, frameon=False, fontsize=9.5,
               bbox_to_anchor=(0.5, 1.07))
    m = blob["meta"]
    fig.suptitle(f"Figure A. e_task (GT) vs noise, 7 methods  ({m['protocol']}, {m['seeds']} seeds)",
                 y=1.12, fontsize=13, fontweight="bold")
    fig.tight_layout(rect=[0, 0, 1, 1.0])
    os.makedirs(FIG, exist_ok=True)
    out = os.path.join(FIG, "fig_paperA_sweeps.png")
    fig.savefig(out, dpi=130, bbox_inches="tight"); print(f"[저장] {out}")


def fig_B(blob):
    lay = blob["layout"]["figB"]
    SYS, OUT = lay["sys"], lay["out"]
    color = {n: STYLE[n][0] for n in STYLE}
    # EXP6(-cube)은 캘리브 붕괴(e_X/reproj)라 e_task 승자에서 제외
    CAND = ["EXP1", "EXP2", "EXP3", "EXP4", "EXP5", "EXP7"]
    fig, ax = plt.subplots(figsize=(8.4, 6.6))
    for xi in range(len(SYS)):
        for yi in range(len(OUT)):
            ck = lay["cells"][f"{xi}_{yi}"]
            cell = {n: blob["results"][ck][n]["e_task_mm"] for n in CAND}
            valid = {n: v for n, v in cell.items() if v is not None}
            if not valid:
                continue
            srt = sorted(valid.items(), key=lambda kv: kv[1])
            w, wv = srt[0]; margin = (srt[1][1] - wv) if len(srt) > 1 else 0
            ax.add_patch(plt.Rectangle((xi - 0.5, yi - 0.5), 1, 1, facecolor=color[w],
                         edgecolor="white", lw=2, alpha=0.92))
            ax.text(xi, yi, f"{STYLE[w][4]}\n{wv:.1f}\n(+{margin:.1f})", ha="center",
                    va="center", fontsize=8, color="white", fontweight="bold")
    ax.set_xticks(range(len(SYS))); ax.set_xticklabels([f"{s:.0%}" if s else "0" for s in SYS])
    ax.set_yticks(range(len(OUT))); ax.set_yticklabels([f"{o:.0%}" if o else "0" for o in OUT])
    ax.set_xlabel("systematic intrinsic err ->", fontsize=10)
    ax.set_ylabel("outlier rate (marker misdetection) ->", fontsize=10)
    ax.set_xlim(-0.5, len(SYS)-0.5); ax.set_ylim(-0.5, len(OUT)-0.5)
    ax.set_title("Figure B. Winner map - FK error fixed at 0 (accurate FK)\n"
                 "x = systematic noise, y = outlier rate; cell: method / e_task / margin"
                 "  (EXP6 excluded: calib collapses)",
                 fontsize=10.5, fontweight="bold", loc="left"); ax.set_aspect("equal")
    legend = [Patch(facecolor=color[n], label=f"{n} {STYLE[n][4]}") for n in CAND]
    ax.legend(handles=legend, loc="upper left", bbox_to_anchor=(1.02, 1.0),
              frameon=False, fontsize=9, title="method")
    fig.tight_layout()
    out = os.path.join(FIG, "fig_paperB_heatmap.png")
    fig.savefig(out, dpi=130, bbox_inches="tight"); print(f"[저장] {out}")


def tables(blob):
    R = blob["results"]; lay = blob["layout"]
    METH = blob["methods"]; LAB = dict(zip(METH, blob["method_labels"]))
    # 표 1: realistic 조건, 지표별
    rk = lay["table"]["realistic"]
    lines = ["## 표 1 — 시뮬 · 현실 종합 조건 (GT)\n",
             f"*{blob['meta']['protocol']}, {blob['meta']['seeds']} seeds. sigma0.3 + 계통2% + FK≈0 + 오검출5%.*",
             "*EXP6(-cube)은 e_X/reproj/cross 붕괴 → e_task 낮은 건 그리퍼예측 착시(캘리브 실패).*\n",
             "| 방법 | e_task mm | e_task deg | e_X mm | **cam→base 병진 mm (bTf)** | reproj px | cross mm |",
             "|---|--:|--:|--:|--:|--:|--:|"]
    for m in METH:
        r = R[rk][m]
        def g(k): return f"{r[k]:.2f}" if r.get(k) is not None else "—"
        lines.append(f"| {m} {LAB[m]} | {g('e_task_mm')} | {g('e_task_deg')} | "
                     f"{g('e_X_mm')} | {g('bTf_mm')} | {g('e_reproj_px')} | {g('e_cross_mm')} |")
    # 표 1b: e_task 조건별
    lines += ["\n## 표 1b — 시뮬 · 조건별 e_task (mm, GT)\n",
              "| 방법 | 이상적 | 현실종합 | +FK오차 | +오검출 |", "|---|--:|--:|--:|--:|"]
    order = ["ideal", "realistic", "fk_err", "outlier"]
    for m in METH:
        vals = []
        for c in order:
            v = R[lay["table"][c]][m]["e_task_mm"]
            vals.append(f"{v:.2f}" if v is not None else "—")
        lines.append(f"| {m} {LAB[m]} | " + " | ".join(vals) + " |")
    txt = "\n".join(lines)
    out = os.path.join(TBL, "TABLE_1_1b.md")
    open(out, "w").write(txt); print(f"[저장] {out}\n"); print(txt)


if __name__ == "__main__":
    blob = json.load(open(os.path.join(TBL, "paper_sim.json")))
    fig_A(blob); fig_B(blob); tables(blob)
