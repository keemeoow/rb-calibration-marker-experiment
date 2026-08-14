#!/usr/bin/env python3
"""Fig A — 코너 노이즈 강건성 곡선 (run_figA.py 의 JSON 에서 렌더)."""
import os, json, argparse
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

FIG_DIR = os.path.join(os.path.dirname(__file__), "results", "figures")
COLORS = {"EXP1": "#4c72b0", "EXP4": "#c44e52", "EXP7": "#dd8452", "EXP3": "#55a868"}
NAMES = {"EXP1": "Ours (corr)", "EXP4": "no-FK", "EXP7": "FK-fixed", "EXP3": "cube-only"}
MARK = {"EXP1": "o", "EXP4": "s", "EXP7": "^", "EXP3": "D"}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--json", default=os.path.join(os.path.dirname(__file__),
                                                   "results", "tables", "figA.json"))
    args = ap.parse_args()
    blob = json.load(open(args.json))
    sig = blob["sigmas"]; curves = blob["curves"]

    fig, axes = plt.subplots(1, 2, figsize=(12, 4.8))
    for ax, key, title in [(axes[0], "e_X_mm", "Camera+hand-eye error e_X"),
                           (axes[1], "e_task_mm", "Held-out cube prediction e_task")]:
        for name, c in curves.items():
            ys = c[key]
            ax.plot(sig, ys, marker=MARK[name], color=COLORS[name], ms=6,
                    label=NAMES[name], zorder=(3 if name == "EXP1" else 2),
                    linewidth=(3 if name == "EXP1" else 2))
        ax.set_xlabel("corner noise σ (px)")
        ax.set_ylabel("error (mm)")
        ax.set_title(f"{title}\n(↓ better)", fontsize=11, fontweight="bold")
        ax.grid(alpha=0.3); ax.legend(fontsize=9)
        ax.axvline(0.3, color="gray", ls="--", alpha=0.5)
        ax.text(0.3, ax.get_ylim()[1]*0.9, " real ~0.3px", fontsize=8, color="gray")

    fig.suptitle("Fig A — Corner-noise robustness (real AprilTag cube / ChArUco board)\n"
                 f"corner-level sim,  {blob['meta']['seeds']} seeds  (lower = better)",
                 fontsize=11, y=1.02)
    fig.tight_layout(rect=[0, 0, 1, 0.93])
    os.makedirs(FIG_DIR, exist_ok=True)
    p = os.path.join(FIG_DIR, "figA_corner_noise.png")
    fig.savefig(p, dpi=130, bbox_inches="tight")
    print(f"[저장] {p}")


if __name__ == "__main__":
    main()
