#!/usr/bin/env python3
"""
논문용 시뮬레이션 씬 그림 — 채워진 큐브 박스 · 카메라 절두체 · 체커보드 보드.
  좌: 3D 리그 (고정 카메라 3대 · 그리퍼 카메라 · 큐브 8 · 보드 · base 프레임)
  우: 고정 카메라가 보는 2D 투영 (체커보드 + 큐브 마커, 픽셀노이즈)
출력: results/figures/fig_sim_scene_paper.{png,pdf}
  python viz_scene_paper.py [--seed 0]
"""
import os, sys, argparse
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import cv2
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from core.scene import SimScene, _CUBE, _BOARD
from core.se3 import inv_T
from core.targets import BOARD_SQUARES_X, BOARD_SQUARES_Y, BOARD_SQUARE_M, CUBE_HALF_M
from core.project import DEFAULT_K, DEFAULT_DIST

FIG = os.path.join(os.path.dirname(__file__), "results", "figures")
plt.rcParams.update({"font.size": 11, "axes.titlesize": 13, "font.family": "DejaVu Sans"})

C_FIX = "#0072B2"    # 고정 카메라
C_GRIP = "#D55E00"   # 그리퍼 카메라
C_CUBE = "#00915f"   # 큐브
C_CUBE_HI = "#00b377"


def _ap(T, P):
    P = np.asarray(P, float)
    return (T[:3, :3] @ P.T).T + T[:3, 3]


def draw_table(ax, center, half=0.6):
    x0, y0 = center[0], center[1]
    sq = [[x0 - half, y0 - half, 0], [x0 + half, y0 - half, 0],
          [x0 + half, y0 + half, 0], [x0 - half, y0 + half, 0]]
    ax.add_collection3d(Poly3DCollection([sq], facecolor="#f2efe9",
                        edgecolor="#d8d2c4", lw=1.0, alpha=0.55, zsort="min"))


def draw_checkerboard(ax, T_bb):
    sx, sy, sq = BOARD_SQUARES_X, BOARD_SQUARES_Y, BOARD_SQUARE_M
    faces, cols = [], []
    for jy in range(sy):
        for jx in range(sx):
            x = jx * sq - sx * sq / 2
            y = jy * sq - sy * sq / 2
            quad = np.array([[x, y, 0], [x + sq, y, 0],
                             [x + sq, y + sq, 0], [x, y + sq, 0]])
            faces.append(_ap(T_bb, quad))
            cols.append("#2b2b2b" if (jx + jy) % 2 == 0 else "#fcfcfc")
    ax.add_collection3d(Poly3DCollection(faces, facecolor=cols, edgecolor="#888",
                        lw=0.3, alpha=0.98))


def draw_cube_box(ax, T_bo, hi=False):
    h = CUBE_HALF_M
    c = np.array([[-h, -h, -h], [h, -h, -h], [h, h, -h], [-h, h, -h],
                  [-h, -h, h], [h, -h, h], [h, h, h], [-h, h, h]])
    cb = _ap(T_bo, c)
    F = [[0, 1, 2, 3], [4, 5, 6, 7], [0, 1, 5, 4],
         [2, 3, 7, 6], [1, 2, 6, 5], [0, 3, 7, 4]]
    faces = [cb[f] for f in F]
    if hi:
        ax.add_collection3d(Poly3DCollection(faces, facecolor=C_CUBE_HI,
                            edgecolor="#04452f", lw=1.0, alpha=0.92))
        mk = [_ap(T_bo, cor) for _, cor, _ in _CUBE.all_corners()]
        ax.add_collection3d(Poly3DCollection(mk, facecolor="#0a0a0a",
                            edgecolor="#e8e8e8", lw=0.6, alpha=0.95))
    else:   # 비강조: 흐리게 (겹침 완화)
        ax.add_collection3d(Poly3DCollection(faces, facecolor=C_CUBE,
                            edgecolor="#2b7a5a", lw=0.4, alpha=0.22))


def draw_frustum(ax, T_bc, color, scale=0.07, label=None, lw=1.4, loff=(0, 0, 0.03)):
    o = T_bc[:3, 3]
    w = scale * 0.75
    far = _ap(T_bc, np.array([[w, w, scale], [w, -w, scale],
                              [-w, -w, scale], [-w, w, scale]]))
    sides = [[o, far[i], far[(i + 1) % 4]] for i in range(4)]
    ax.add_collection3d(Poly3DCollection(sides, facecolor=color, edgecolor=color,
                        lw=lw, alpha=0.16))
    ax.add_collection3d(Poly3DCollection([far], facecolor=color, edgecolor=color,
                        lw=lw, alpha=0.30))
    ax.scatter(*o, color=color, s=45, depthshade=False, edgecolor="white", lw=0.8, zorder=6)
    if label:
        ax.text(o[0] + loff[0], o[1] + loff[1], o[2] + loff[2], label, color=color,
                fontsize=10, fontweight="bold", ha="center")


def draw_frame(ax, T, L=0.09, lw=2.2):
    o = T[:3, 3]
    for k, col in zip(range(3), ["#d62728", "#2ca02c", "#1f77b4"]):
        d = T[:3, k] * L
        ax.quiver(o[0], o[1], o[2], d[0], d[1], d[2], color=col, lw=lw, arrow_length_ratio=0.18)


def _shift(T, off):
    T2 = np.array(T, float, copy=True)
    T2[:3, 3] = T2[:3, 3] + np.asarray(off, float)
    return T2


def draw_rig(ax, sc, viz_off=(0, 0, 0)):
    viz_off = np.asarray(viz_off, float)
    center = sc.bTboard[:3, 3]
    draw_table(ax, center)
    draw_checkerboard(ax, sc.bTboard)
    for i, s in enumerate(sc.sets):
        draw_cube_box(ax, _shift(sc.bTo[s], viz_off), hi=(i == 0))
    LOFF = {0: (0.10, 0, 0.03), 1: (-0.02, 0, -0.10), 2: (-0.14, 0, 0.06)}
    for ci in sc.fixed_cam_ids:
        draw_frustum(ax, sc.bTf[ci], C_FIX, label=f"fixed C{ci}",
                     loff=LOFF.get(ci, (0, 0, 0.03)))
    # 그리퍼 경로 + 대표 절두체 2개(흐리게) — 큐브 따라 같은 오프셋(eye-in-hand 관계 유지)
    gpos = np.array([(sc.bTg[e] @ sc.gTc)[:3, 3] + viz_off for e in sc.events])
    ax.plot(gpos[:, 0], gpos[:, 1], gpos[:, 2], color=C_GRIP, lw=0.9, alpha=0.4)
    reps = sc.events[::max(1, len(sc.events) // 2)][:2]
    for j, e in enumerate(reps):
        draw_frustum(ax, _shift(sc.bTg[e] @ sc.gTc, viz_off), C_GRIP, scale=0.05, lw=1.0,
                     label="gripper cam" if j == 0 else None, loff=(0.16, 0.02, 0.07))
    draw_frame(ax, np.eye(4))
    ax.text(0.02, 0.02, -0.05, "robot base", fontsize=10, fontweight="bold", ha="left")

    allp = np.vstack([gpos, [sc.bTf[ci][:3, 3] for ci in sc.fixed_cam_ids],
                      [sc.bTo[s][:3, 3] + viz_off for s in sc.sets], [[0, 0, 0]], [center]])
    mn, mx = allp.min(0), allp.max(0)
    c = (mn + mx) / 2
    r = (mx - mn).max() * 0.62
    ax.set_xlim(c[0] - r, c[0] + r); ax.set_ylim(c[1] - r, c[1] + r)
    ax.set_zlim(-0.02, max(0.02, c[2] + r))
    ax.set_box_aspect((1, 1, 0.62))
    ax.view_init(elev=26, azim=-68)
    ax.set_xlabel("x (m)", labelpad=2); ax.set_ylabel("y (m)", labelpad=2)
    ax.set_zlabel("z (m)", labelpad=1)
    ax.xaxis.pane.set_alpha(0.03); ax.yaxis.pane.set_alpha(0.03); ax.zaxis.pane.set_alpha(0.03)
    ax.grid(True, alpha=0.25)
    ax.set_title("(a) Simulation rig — real camera layout & marker geometry",
                 loc="left", fontweight="bold")


def draw_camera_view(ax, sc, ci=0, viz_off=(0, 0, 0)):
    viz_off = np.asarray(viz_off, float)
    T_cb = inv_T(sc.bTf[ci])
    rng = np.random.default_rng(123)
    sx, sy, sq = BOARD_SQUARES_X, BOARD_SQUARES_Y, BOARD_SQUARE_M

    def proj(T_c_local, pts3d):
        pc = _ap(T_c_local, pts3d)
        if np.any(pc[:, 2] <= 0):
            return None
        img, _ = cv2.projectPoints(pts3d.astype(float),
                                   cv2.Rodrigues(T_c_local[:3, :3])[0],
                                   T_c_local[:3, 3].astype(float), DEFAULT_K, DEFAULT_DIST)
        return img.reshape(-1, 2)

    # 체커보드 사각형 투영
    Tcb = T_cb @ sc.bTboard
    from matplotlib.patches import Polygon as P2
    for jy in range(sy):
        for jx in range(sx):
            x = jx * sq - sx * sq / 2; y = jy * sq - sy * sq / 2
            quad = np.array([[x, y, 0], [x + sq, y, 0], [x + sq, y + sq, 0], [x, y + sq, 0]])
            ip = proj(Tcb, quad)
            if ip is None:
                continue
            col = "#2b2b2b" if (jx + jy) % 2 == 0 else "#fbfbfb"
            ax.add_patch(P2(ip, closed=True, facecolor=col, edgecolor="#999", lw=0.3))
    # 보드 코너(검출점) 노이즈
    ip = proj(Tcb, _BOARD.corners3d)
    if ip is not None:
        n = ip + rng.normal(0, sc.sigma_px, ip.shape)
        ax.scatter(n[:, 0], n[:, 1], s=14, color="#e69f00", zorder=5,
                   edgecolor="#7a5600", lw=0.4, label="detected corners")
    # 큐브 (첫 set) 보이는 면 마커 채우기 — 그림용 오프셋 적용(보드와 분리)
    Tcc = T_cb @ _shift(sc.bTo[sc.sets[0]], viz_off)
    for mid, corners3d, normal in _CUBE.all_corners():
        if (Tcc[:3, :3] @ normal)[2] > -0.15:
            continue
        ip = proj(Tcc, corners3d)
        if ip is None:
            continue
        q = ip + rng.normal(0, sc.sigma_px, ip.shape)
        ax.add_patch(P2(q, closed=True, facecolor=C_CUBE, edgecolor="#04452f",
                        lw=1.0, alpha=0.85))
    ax.add_patch(plt.Rectangle((0, 0), 640, 480, fill=False, ec="#333", lw=1.6))
    ax.set_xlim(-15, 655); ax.set_ylim(495, -15)
    ax.set_aspect("equal")
    ax.set_xlabel("u (px)"); ax.set_ylabel("v (px)")
    ax.legend(loc="upper right", fontsize=9, frameon=True, facecolor="white", framealpha=0.9)
    ax.set_title(f"(b) What fixed camera C{ci} observes  (640x480, noise {sc.sigma_px}px)",
                 loc="left", fontweight="bold")
    ax.set_facecolor("#eef1f4")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seed", type=int, default=0)
    args = ap.parse_args()
    sc = SimScene(seed=args.seed, n_sets=8, n_events_per_set=6, sigma_px=0.3)

    # 그림 전용 오프셋: 큐브(와 그리퍼 카메라)를 보드 옆으로 옮겨 겹침 제거 (sim 미변경)
    VIZ_OFF = (0.32, 0.0, 0.0)

    fig = plt.figure(figsize=(15.5, 6.6))
    gs = fig.add_gridspec(1, 2, width_ratios=[1.45, 1], wspace=0.12)
    ax1 = fig.add_subplot(gs[0], projection="3d")
    draw_rig(ax1, sc, viz_off=VIZ_OFF)
    ax2 = fig.add_subplot(gs[1])
    draw_camera_view(ax2, sc, ci=0)   # (b)는 실제 위치(프레임 안) — 오프셋 미적용

    handles = [Line2D([0], [0], color=C_FIX, lw=6, alpha=0.5, label="Fixed cameras (eye-to-hand) x3"),
               Line2D([0], [0], color=C_GRIP, lw=6, alpha=0.5, label="Gripper camera (eye-in-hand)"),
               Line2D([0], [0], color=C_CUBE, lw=6, alpha=0.6, label="AprilTag cube (per set) x8"),
               Line2D([0], [0], color="#2b2b2b", lw=6, label="ChArUco board 11x7")]
    fig.legend(handles=handles, loc="lower center", ncol=4, frameon=False,
               fontsize=10.5, bbox_to_anchor=(0.5, -0.02))
    fig.tight_layout(rect=[0, 0.04, 1, 1])
    os.makedirs(FIG, exist_ok=True)
    for ext in ("png", "pdf"):
        out = os.path.join(FIG, f"fig_sim_scene_paper.{ext}")
        fig.savefig(out, dpi=220 if ext == "png" else None, bbox_inches="tight")
        print(f"[저장] {out}")


if __name__ == "__main__":
    main()
