#!/usr/bin/env python3
"""fig_sim_scene.png 의 왼쪽(3D 리그) 패널만 단독 이미지로 → fig_sim_scene_3d.png."""
import os, sys
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import viz_scene as V
from core.scene import SimScene

FIG = os.path.join(os.path.dirname(__file__), "results", "figures")
sc = SimScene(seed=0, n_sets=8, n_events_per_set=6, sigma_px=0.3)

fig = plt.figure(figsize=(8.6, 7.6))
ax = fig.add_subplot(111, projection="3d")
V.draw_rig(ax, sc, elev=25, azim=-72,
           title="Simulation rig — real camera layout + marker geometry")
handles = [
    Line2D([0], [0], color=V.C_FIX, lw=2.5, label="Fixed cameras (eye-to-hand) x3"),
    Line2D([0], [0], color=V.C_GRIP, lw=2.5, marker="o", label="Gripper camera (eye-in-hand) path"),
    Line2D([0], [0], color=V.C_CUBE, lw=2.5, label="Cube (re-placed per set) x8"),
    Line2D([0], [0], color=V.C_BOARD, lw=2.5, label="ChArUco board 11x7 (on table)"),
]
fig.legend(handles=handles, loc="lower center", ncol=2, frameon=False, fontsize=9.5,
           bbox_to_anchor=(0.5, -0.01))
fig.tight_layout(rect=[0, 0.06, 1, 1])
os.makedirs(FIG, exist_ok=True)
out = os.path.join(FIG, "fig_sim_scene_3d.png")
fig.savefig(out, dpi=140, bbox_inches="tight")
print(f"[저장] {out}")
