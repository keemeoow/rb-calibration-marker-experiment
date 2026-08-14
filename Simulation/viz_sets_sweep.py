#!/usr/bin/env python3
"""sets_sweep.json → 촬영 셋 수 vs e_task 곡선 (대표 4방식). 표도 markdown 반환."""
import os, json
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

TBL = os.path.join(os.path.dirname(__file__), "results", "tables")
FIG = os.path.join(os.path.dirname(__file__), "results", "figures")
STYLE = {"Ours": ("#0072B2", "-", 3.0, "o"), "fixed-FK": ("#56B4E9", "-.", 2.0, "X"),
         "no-FK": ("#D55E00", "--", 1.8, "v"), "-unified(indep)": ("#E69F00", "-", 1.8, "s")}


def main():
    b = json.load(open(os.path.join(TBL, "sets_sweep.json")))
    sets, curves = b["sets"], b["curves"]
    fig, ax = plt.subplots(figsize=(8.2, 5.2))
    for name, (col, ls, lw, mk) in STYLE.items():
        if name not in curves:
            continue
        ys = curves[name]
        xs = [x for x, y in zip(sets, ys) if y is not None]
        yv = [y for y in ys if y is not None]
        ax.plot(xs, yv, color=col, ls=ls, lw=lw, marker=mk, ms=8, label=name,
                alpha=1.0 if name == "Ours" else 0.85, zorder=5 if name == "Ours" else 2)
    ax.set_xlabel("number of capture sets (cube positions)", fontsize=11)
    ax.set_ylabel("e_task (mm, GT)  -  lower is better", fontsize=11)
    ax.set_title(f"Performance vs #capture-sets\n{b['cond']}, gripped {b['gripped']}",
                 fontsize=12, fontweight="bold", loc="left")
    ax.grid(alpha=0.25); ax.spines[["top", "right"]].set_visible(False)
    ax.legend(fontsize=10, frameon=False)
    fig.tight_layout()
    os.makedirs(FIG, exist_ok=True)
    out = os.path.join(FIG, "fig_sets_sweep.png")
    fig.savefig(out, dpi=140, bbox_inches="tight"); print(f"[저장] {out}")

    # markdown 표
    lines = ["| 방식 \\ 셋 수 | " + " | ".join(str(s) for s in sets) + " |",
             "|---|" + "|".join("--:" for _ in sets) + "|"]
    for name in ["Ours", "fixed-FK", "no-FK", "-unified(indep)"]:
        if name not in curves:
            continue
        vals = [f"{v:.1f}" if v is not None else "—" for v in curves[name]]
        lines.append(f"| {name} | " + " | ".join(vals) + " |")
    md = "\n".join(lines)
    print("\n" + md)
    return md


if __name__ == "__main__":
    md = main()
    # SIM_RESULTS.md 의 플레이스홀더 치환
    p = os.path.join(os.path.dirname(__file__), "SIM_RESULTS.md")
    if os.path.exists(p):
        txt = open(p).read().replace("<!-- SETS_TABLE -->", md)
        open(p, "w").write(txt)
        print("\n[SIM_RESULTS.md 표 삽입 완료]")
