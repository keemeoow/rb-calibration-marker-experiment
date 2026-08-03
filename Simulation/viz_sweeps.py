#!/usr/bin/env python3
"""
sweep 곡선 그림 — run_sweeps.py 의 JSON(코너/FK 축)에서 지표별 패널로 7방식 곡선 렌더.

  python viz_sweeps.py --json results/tables/sweep_corner.json
  python viz_sweeps.py --json results/tables/sweep_fk.json
지표 6개(e_X, e_task, gTc, e_cross, reproj, N_reg)를 각 패널로. 7방식 곡선 + Ours 굵게.
"""
import sys, os, json, argparse
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

FIG_DIR = os.path.join(os.path.dirname(__file__), "results", "figures")

# 7방식 색/마커 (범례는 영문 — matplotlib 기본 폰트에 한글 없어 깨짐)
STYLE = {
    "EXP1": ("#4c72b0", "o", "EXP1 Ours (unified+FKcorr+cube&board)"),
    "EXP2": ("#dd8452", "s", "EXP2 -unified"),
    "EXP3": ("#55a868", "^", "EXP3 -board (cube-only)"),
    "EXP4": ("#c44e52", "v", "EXP4 -FK"),
    "EXP5": ("#8172b3", "D", "EXP5 -FK-unified"),
    "EXP6": ("#937860", "P", "EXP6 -cube (board-only)"),
    "EXP7": ("#da8bc3", "X", "EXP7 FK-fixed"),
}
PANELS = [
    ("e_X_mm", "Camera+hand-eye e_X (mm)"),
    ("e_task_mm", "Held-out cube e_task (mm)"),
    ("gTc_mm", "Hand-eye gTc (mm)"),
    ("e_cross_mm", "Cross-camera e_cross (mm)"),
    ("e_reproj_px", "Unified reproj (px, cube+board)"),
    ("N_reg", "Registered cameras N_reg"),
]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--json", required=True)
    ap.add_argument("--logy", action="store_true", help="y축 로그(붕괴 케이스 대비)")
    args = ap.parse_args()
    blob = json.load(open(args.json))
    axis = blob["axis"]; unit = blob["unit"]; levels = blob["levels"]; curves = blob["curves"]
    xlabel = f"corner noise σ ({unit})" if axis == "corner" else f"FK noise ({unit})"

    fig, axes = plt.subplots(2, 3, figsize=(16, 9))
    for ax, (key, title) in zip(axes.flat, PANELS):
        for name, (col, mk, lab) in STYLE.items():
            ys = curves[name][key]
            xs = [x for x, y in zip(levels, ys) if y is not None]
            yv = [y for y in ys if y is not None]
            if not yv:
                continue
            ax.plot(xs, yv, marker=mk, color=col, ms=5,
                    linewidth=(3 if name == "EXP1" else 1.6),
                    zorder=(3 if name == "EXP1" else 2), label=lab)
        ax.set_xlabel(xlabel); ax.set_ylabel(title.split("(")[-1].rstrip(")"))
        ax.set_title(title, fontsize=10, fontweight="bold")
        ax.grid(alpha=0.3)
        if args.logy and key in ("e_X_mm", "e_reproj_px", "e_cross_mm"):
            ax.set_yscale("log")
        if axis == "corner":
            ax.axvline(0.3, color="gray", ls="--", alpha=0.4)
    axes.flat[0].legend(fontsize=7.5, ncol=1, loc="upper left")

    fig.suptitle(
        f"Noise sweep ({'corner σ px' if axis=='corner' else 'FK mm'}) — 7 methods × all metrics\n"
        f"corner-level sim, real cameras,  {blob['meta']['seeds']} seeds × "
        f"{blob['meta']['pairs']} holdout pairs  (lower=better; N_reg higher=better)",
        fontsize=12, y=1.0)
    fig.tight_layout(rect=[0, 0, 1, 0.95])
    os.makedirs(FIG_DIR, exist_ok=True)
    out = os.path.join(FIG_DIR, f"sweep_{axis}.png")
    fig.savefig(out, dpi=120, bbox_inches="tight")
    print(f"[저장] {out}")


if __name__ == "__main__":
    main()
