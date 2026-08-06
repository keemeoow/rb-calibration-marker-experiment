#!/usr/bin/env python3
"""
시뮬레이션 씬 시각화 — 실제 SimScene 기하를 그대로 3D로 그린다.
  좌: 3D 리그(고정 카메라 3대 · 그리퍼 카메라 경로 · 큐브 set별 · 보드)
  우상: 위에서 본 3D (bird's-eye)
  우하: 고정 카메라 0번이 실제로 보는 2D 투영 (보드+큐브 코너 + 픽셀노이즈)
  python viz_scene.py [--seed 0]
"""
import os, sys, argparse
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import cv2
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from core.scene import SimScene, _CUBE, _BOARD
from core.se3 import inv_T
from core.project import DEFAULT_K, DEFAULT_DIST

FIG = os.path.join(os.path.dirname(__file__), "results", "figures")

C_FIX = "#0072B2"    # 고정 카메라 (파랑)
C_GRIP = "#D55E00"   # 그리퍼 카메라 (주홍)
C_CUBE = "#009E73"   # 큐브 (초록)
C_BOARD = "#8a6d3b"  # 보드 (갈색)


def _apply(T, pts):
    """(N,3) 로컬 → base."""
    pts = np.asarray(pts, float)
    return (T[:3, :3] @ pts.T).T + T[:3, 3]


def _polyline(ax, P, **kw):
    """(N,3) 점열을 3D 선으로 (열 단위로 명시 전달 — 언패킹 버그 방지)."""
    P = np.asarray(P, float)
    ax.plot(P[:, 0], P[:, 1], P[:, 2], **kw)


def draw_cam(ax, T_bc, scale=0.06, color="k", label=None, lw=1.5):
    """base←cam 변환 T_bc 로 카메라 원뿔(광축 +Z) 그리기."""
    o = T_bc[:3, 3]
    w = scale * 0.7
    far = np.array([[w, w, scale], [w, -w, scale], [-w, -w, scale], [-w, w, scale]])
    farb = _apply(T_bc, far)
    for c in farb:
        _polyline(ax, np.vstack([o, c]), color=color, lw=lw, alpha=0.9)
    _polyline(ax, np.vstack([farb, farb[0]]), color=color, lw=lw, alpha=0.9)
    ax.scatter(*o, color=color, s=40, depthshade=False)
    if label:
        ax.text(*o, "  " + label, color=color, fontsize=9, fontweight="bold")


def draw_cube(ax, T_bo, color=C_CUBE, alpha=0.85):
    """큐브 6면 마커 quad 를 base 에 그린다."""
    for mid, corners3d, normal in _CUBE.all_corners():
        q = _apply(T_bo, corners3d)
        _polyline(ax, np.vstack([q, q[0]]), color=color, lw=1.2, alpha=alpha)


def draw_board(ax, T_bb, color=C_BOARD):
    """보드 내부 코너 격자 + 외곽."""
    pts = _apply(T_bb, _BOARD.corners3d)
    ax.scatter(pts[:, 0], pts[:, 1], pts[:, 2], color=color, s=6, alpha=0.7)
    # 외곽 사각형
    mn, mx = _BOARD.corners3d.min(0), _BOARD.corners3d.max(0)
    rect = np.array([[mn[0], mn[1], 0], [mx[0], mn[1], 0],
                     [mx[0], mx[1], 0], [mn[0], mx[1], 0]])
    rb = _apply(T_bb, rect)
    _polyline(ax, np.vstack([rb, rb[0]]), color=color, lw=2, alpha=0.9)


def set_axes_equal(ax, pts):
    pts = np.asarray(pts)
    c = pts.mean(0)
    r = np.abs(pts - c).max() * 1.1
    ax.set_xlim(c[0] - r, c[0] + r)
    ax.set_ylim(c[1] - r, c[1] + r)
    ax.set_zlim(max(0, c[2] - r), c[2] + r)


def draw_rig(ax, sc, elev, azim, title):
    allpts = []
    # base 원점 + 축
    ax.scatter(0, 0, 0, color="k", s=60, marker="s")
    ax.text(0, 0, 0, "  robot base", fontsize=9, fontweight="bold")
    for v, c in zip(np.eye(3) * 0.1, ["r", "g", "b"]):
        ax.plot(*zip([0, 0, 0], v), color=c, lw=1.5)
    # 고정 카메라
    for ci in sc.fixed_cam_ids:
        draw_cam(ax, sc.bTf[ci], color=C_FIX, label=f"fixed C{ci}")
        allpts.append(sc.bTf[ci][:3, 3])
    # 그리퍼 카메라 경로 (event 마다 카메라 = bTg@gTc)
    gpos = np.array([(sc.bTg[e] @ sc.gTc)[:3, 3] for e in sc.events])
    ax.scatter(gpos[:, 0], gpos[:, 1], gpos[:, 2], color=C_GRIP, s=12, alpha=0.5)
    draw_cam(ax, sc.bTg[sc.events[0]] @ sc.gTc, color=C_GRIP, label="gripper cam", lw=1.8)
    allpts.append(gpos.reshape(-1, 3))
    # 큐브 (set 마다)
    for s in sc.sets:
        draw_cube(ax, sc.bTo[s])
        allpts.append(sc.bTo[s][:3, 3])
    # 보드
    draw_board(ax, sc.bTboard)
    allpts.append(sc.bTboard[:3, 3])
    pts = np.vstack([np.atleast_2d(p) for p in allpts])
    set_axes_equal(ax, np.vstack([pts, [[0, 0, 0]]]))
    ax.view_init(elev=elev, azim=azim)
    ax.set_xlabel("x"); ax.set_ylabel("y"); ax.set_zlabel("z")
    ax.set_title(title, fontsize=11, fontweight="bold")


def draw_camera_view(ax, sc, ci=0):
    """고정 카메라 ci 가 보는 2D 투영 (보드 + 큐브 코너, 픽셀노이즈 포함)."""
    T_cb = inv_T(sc.bTf[ci])   # cam←base
    rng = np.random.default_rng(123)

    def proj(T_c_local, pts3d):
        pc = _apply(T_c_local, pts3d)
        if np.any(pc[:, 2] <= 0):
            return None
        img, _ = cv2.projectPoints(pts3d.astype(float), cv2.Rodrigues(T_c_local[:3, :3])[0],
                                   T_c_local[:3, 3].astype(float), DEFAULT_K, DEFAULT_DIST)
        return img.reshape(-1, 2)

    # 보드
    ip = proj(T_cb @ sc.bTboard, _BOARD.corners3d)
    if ip is not None:
        n = ip + rng.normal(0, sc.sigma_px, ip.shape)
        ax.scatter(n[:, 0], n[:, 1], s=10, color=C_BOARD, label="board corners")
    # 큐브 (첫 set) — 보이는 면만
    T_c_cube = T_cb @ sc.bTo[sc.sets[0]]
    for mid, corners3d, normal in _CUBE.all_corners():
        nrm_c = T_c_cube[:3, :3] @ normal
        if nrm_c[2] > -0.2:      # 카메라 쪽 안 향하면 스킵(대략적 가시성)
            continue
        ip = proj(T_c_cube, corners3d)
        if ip is None:
            continue
        q = ip + rng.normal(0, sc.sigma_px, ip.shape)
        qq = np.vstack([q, q[0]])
        ax.plot(qq[:, 0], qq[:, 1], color=C_CUBE, lw=1.5)
    ax.add_patch(plt.Rectangle((0, 0), 640, 480, fill=False, ec="gray", lw=1.5))
    ax.set_xlim(-20, 660); ax.set_ylim(500, -20)   # 이미지 좌표(위→아래)
    ax.set_aspect("equal")
    ax.set_title(f"What fixed camera C{ci} sees  (640x480, noise={sc.sigma_px}px)",
                 fontsize=11, fontweight="bold")
    ax.set_xlabel("u (px)"); ax.set_ylabel("v (px)")
    ax.legend(loc="upper right", fontsize=8, frameon=False)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seed", type=int, default=0)
    args = ap.parse_args()
    sc = SimScene(seed=args.seed, n_sets=8, n_events_per_set=6, sigma_px=0.3)

    fig = plt.figure(figsize=(16, 8))
    ax1 = fig.add_subplot(1, 2, 1, projection="3d")
    draw_rig(ax1, sc, elev=25, azim=-72, title="Simulation rig (3D perspective)")
    ax2 = fig.add_subplot(2, 2, 2, projection="3d")
    draw_rig(ax2, sc, elev=89, azim=-90, title="Top-down (bird's-eye)")
    ax3 = fig.add_subplot(2, 2, 4)
    draw_camera_view(ax3, sc, ci=0)

    # 범례 (좌 3D)
    from matplotlib.lines import Line2D
    handles = [Line2D([0], [0], color=C_FIX, lw=2, label="Fixed cameras (eye-to-hand) x3"),
               Line2D([0], [0], color=C_GRIP, lw=2, marker="o", label="Gripper camera (eye-in-hand) path"),
               Line2D([0], [0], color=C_CUBE, lw=2, label="Cube (re-placed per set) x8"),
               Line2D([0], [0], color=C_BOARD, lw=2, label="ChArUco board 11x7 (fixed on table)")]
    fig.legend(handles=handles, loc="lower center", ncol=4, frameon=False, fontsize=10,
               bbox_to_anchor=(0.5, -0.01))
    fig.suptitle(f"SimScene (seed={args.seed}) - real camera layout + real marker geometry",
                 fontsize=13, fontweight="bold")
    fig.tight_layout(rect=[0, 0.03, 1, 0.97])
    os.makedirs(FIG, exist_ok=True)
    out = os.path.join(FIG, "fig_sim_scene.png")
    fig.savefig(out, dpi=130, bbox_inches="tight")
    print(f"[저장] {out}")


if __name__ == "__main__":
    main()
