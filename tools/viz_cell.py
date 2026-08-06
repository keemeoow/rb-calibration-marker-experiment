# tools/viz_cell.py
"""Interactive 3D view of the robot cell in the base frame.

Ties the three offline modules together in one window so a pose can be judged by
eye before the robot ever moves:

  - robot_kin      arm posture from joint angles
  - cell_scene     obstacles, live clearance readout (red = would collide)
  - view_model     which cube markers each camera sees from this pose

Modes:
    # drag six joint sliders, watch clearance and marker visibility update
    python tools/viz_cell.py --session data/session

    # start from one recorded capture
    python tools/viz_cell.py --session data/session --event 42

    # replay every executed pose as an animation
    python tools/viz_cell.py --session data/session --replay

    # animate the joint-space straight line between two poses (what rb.move does)
    python tools/viz_cell.py --session data/session --path-from 0 --path-to 120

Rotate with the mouse; the window is a normal matplotlib 3D axes.
"""

from __future__ import annotations

import argparse
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import matplotlib.pyplot as plt                                  # noqa: E402
from matplotlib.widgets import Slider, Button                     # noqa: E402

from robot_comm import euler_deg_to_matrix                        # noqa: E402
from tools.cell_scene import Box, Cylinder, HalfSpace, load_scene  # noqa: E402
from tools.robot_kin import RobotKinematics, fit_tool_transform    # noqa: E402
from tools.view_model import CubeViewModel, load_camera_rig        # noqa: E402

LINK_COLOR = "#2b6cb0"
LINK_COLOR_BAD = "#c53030"
CAM_COLORS = {0: "#38a169", 1: "#d69e2e", 2: "#805ad5", 3: "#dd6b20"}


# -----------------------------------------------------------------------------
# static geometry
# -----------------------------------------------------------------------------

def draw_frame(ax, T, length=0.08, lw=1.6, label=None):
    o = T[:3, 3]
    for k, c in enumerate("rgb"):
        d = T[:3, k] * length
        ax.plot(*zip(o, o + d), color=c, lw=lw)
    if label:
        ax.text(*o, f" {label}", fontsize=8)


def draw_box(ax, b: Box, color="#718096", alpha=0.25):
    signs = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)])
    corners = b.center + (signs * b.half_extents) @ b.R.T
    edges = [(0, 1), (0, 2), (0, 4), (1, 3), (1, 5), (2, 3),
             (2, 6), (3, 7), (4, 5), (4, 6), (5, 7), (6, 7)]
    for i, j in edges:
        ax.plot(*zip(corners[i], corners[j]), color=color, lw=1.0, alpha=0.9)


def draw_cylinder(ax, c: Cylinder, color="#718096"):
    th = np.linspace(0, 2 * np.pi, 24)
    x = c.center_xy[0] + c.radius * np.cos(th)
    y = c.center_xy[1] + c.radius * np.sin(th)
    for z in (c.z_min, c.z_max):
        ax.plot(x, y, np.full_like(x, z), color=color, lw=1.0, alpha=0.9)
    for k in range(0, 24, 6):
        ax.plot([x[k], x[k]], [y[k], y[k]], [c.z_min, c.z_max], color=color, lw=1.0, alpha=0.9)


def draw_floor(ax, h: HalfSpace, extent, color="#a0aec0"):
    (x0, x1), (y0, y1) = extent
    xx, yy = np.meshgrid([x0, x1], [y0, y1])
    ax.plot_surface(xx, yy, np.full_like(xx, h.z_min), color=color, alpha=0.12, shade=False)


def draw_camera(ax, T_base_cam, K, w, h, color, label, depth=0.22):
    """Frustum: image corners back-projected to ``depth`` metres."""
    Kinv = np.linalg.inv(np.asarray(K, dtype=float))
    corners_px = np.array([[0, 0, 1], [w, 0, 1], [w, h, 1], [0, h, 1]], dtype=float)
    rays = (Kinv @ corners_px.T).T
    rays = rays / rays[:, 2:3] * depth
    pts = (T_base_cam[:3, :3] @ rays.T).T + T_base_cam[:3, 3]
    o = T_base_cam[:3, 3]
    for p in pts:
        ax.plot(*zip(o, p), color=color, lw=0.9, alpha=0.8)
    for i in range(4):
        ax.plot(*zip(pts[i], pts[(i + 1) % 4]), color=color, lw=0.9, alpha=0.8)
    ax.text(*o, f" {label}", color=color, fontsize=8)


def draw_cube(ax, T_base_cube, side=0.059, color="#1a202c"):
    d = side / 2.0
    signs = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]) * d
    pts = (T_base_cube[:3, :3] @ signs.T).T + T_base_cube[:3, 3]
    edges = [(0, 1), (0, 2), (0, 4), (1, 3), (1, 5), (2, 3),
             (2, 6), (3, 7), (4, 5), (4, 6), (5, 7), (6, 7)]
    for i, j in edges:
        ax.plot(*zip(pts[i], pts[j]), color=color, lw=1.4)
    draw_frame(ax, T_base_cube, length=0.05, lw=1.0)


# -----------------------------------------------------------------------------
# viewer
# -----------------------------------------------------------------------------

class CellViewer:
    def __init__(self, kin, scene, rig, view, T_base_cube, q0, executed=None):
        self.kin, self.scene, self.rig, self.view = kin, scene, rig, view
        self.T_base_cube = T_base_cube
        self.q = np.asarray(q0, dtype=float).copy()
        self.executed = executed

        self.fig = plt.figure(figsize=(13, 8))
        self.ax = self.fig.add_subplot(111, projection="3d")
        self.fig.subplots_adjust(left=0.02, right=0.72, bottom=0.24, top=0.98)

        self.extent = ((-0.9, 0.6), (-0.3, 1.2))
        self._draw_static()

        self.dynamic = []
        self.txt = self.fig.text(0.74, 0.94, "", va="top", family="monospace", fontsize=9)

        self.sliders = []
        for i in range(6):
            axs = self.fig.add_axes([0.08, 0.17 - i * 0.028, 0.55, 0.02])
            s = Slider(axs, f"J{i+1}", float(kin.joint_min_deg[i]), float(kin.joint_max_deg[i]),
                       valinit=float(self.q[i]), valfmt="%6.1f")
            s.on_changed(self._on_slider)
            self.sliders.append(s)

        if executed is not None and len(executed):
            axe = self.fig.add_axes([0.78, 0.10, 0.18, 0.03])
            self.ev = Slider(axe, "event", 0, len(executed) - 1, valinit=0, valstep=1, valfmt="%3d")
            self.ev.on_changed(self._on_event)
            axb = self.fig.add_axes([0.78, 0.04, 0.08, 0.04])
            self.btn = Button(axb, "replay")
            self.btn.on_clicked(self._replay)

        self._redraw()

    # -- drawing ----------------------------------------------------------

    def _draw_static(self):
        ax = self.ax
        draw_frame(ax, np.eye(4), length=0.15, lw=2.2, label="base")
        for obs in self.scene.obstacles:
            if isinstance(obs, Box):
                draw_box(ax, obs)
            elif isinstance(obs, Cylinder):
                draw_cylinder(ax, obs)
            elif isinstance(obs, HalfSpace):
                draw_floor(ax, obs, self.extent)
        for ci, cam in self.rig.items():
            if cam.is_gripper:
                continue
            draw_camera(ax, cam.T_base_cam, cam.K, cam.width, cam.height,
                        CAM_COLORS.get(ci, "#4a5568"), f"cam{ci}")
        if self.T_base_cube is not None:
            draw_cube(ax, self.T_base_cube)

        ax.set_xlim(*self.extent[0])
        ax.set_ylim(*self.extent[1])
        ax.set_zlim(0.0, 1.0)
        ax.set_box_aspect((self.extent[0][1] - self.extent[0][0],
                           self.extent[1][1] - self.extent[1][0], 1.0))
        ax.set_xlabel("x [m]")
        ax.set_ylabel("y [m]")
        ax.set_zlabel("z [m]")
        ax.view_init(elev=24, azim=-125)

    def _redraw(self):
        for h in self.dynamic:
            h.remove()
        self.dynamic = []

        res = self.scene.check(self.q)
        color = LINK_COLOR if res["ok"] else LINK_COLOR_BAD
        for a, b, r in self.scene.link_segments(self.q):
            (h,) = self.ax.plot(*zip(a, b), color=color, lw=max(2.0, r * 90.0),
                                solid_capstyle="round", alpha=0.85)
            self.dynamic.append(h)

        T_tool = self.kin.fk_tool(self.q)
        for k, c in enumerate("rgb"):
            o, d = T_tool[:3, 3], T_tool[:3, k] * 0.07
            (h,) = self.ax.plot(*zip(o, o + d), color=c, lw=1.8)
            self.dynamic.append(h)

        lines = ["J   = " + " ".join(f"{v:7.1f}" for v in self.q),
                 "TCP = " + " ".join(f"{v:7.1f}" for v in T_tool[:3, 3] * 1000) + "  mm",
                 ""]
        if res["clearance_m"] is None:
            lines.append("clearance : JOINT LIMIT VIOLATION")
        else:
            lines.append(f"clearance : {res['clearance_m']*1000:7.1f} mm"
                         f"  {'OK' if res['ok'] else 'COLLIDE'}")
            if not res["ok"]:
                lines.append(f"            {res['reason']}")
        lines.append(f"manipulability: {self.kin.manipulability(self.q):.4f}")

        if self.T_base_cube is not None:
            T_bg = T_tool
            lines += ["", "predicted marker visibility"]
            for ci in sorted(self.rig):
                cam = self.rig[ci]
                obs = self.view.observe(cam, self.T_base_cube, T_bg)
                tag = "grip" if cam.is_gripper else "fixed"
                faces = ",".join(sorted(obs["faces"])) or "-"
                n_side = len(obs["side_faces"])
                mark = "**" if n_side >= 2 else ("* " if obs["n_visible"] >= 2 else "  ")
                lines.append(f" {mark}cam{ci} ({tag:5s}): {obs['n_visible']} "
                             f"ids={obs['ids']} {faces}")
                # sight line: solid when this camera would give a multi-marker PnP
                o = cam.pose_in_base(T_bg)[:3, 3]
                style = dict(color=CAM_COLORS.get(ci, "#4a5568"))
                if obs["n_visible"] >= 2:
                    style.update(lw=1.6, ls="-", alpha=0.9)
                elif obs["n_visible"] == 1:
                    style.update(lw=0.8, ls=":", alpha=0.5)
                else:
                    continue
                (h,) = self.ax.plot(*zip(o, self.T_base_cube[:3, 3]), **style)
                self.dynamic.append(h)
            lines.append("   (** = two side faces -> best cube pose)")

        self.txt.set_text("\n".join(lines))
        self.fig.canvas.draw_idle()

    # -- callbacks --------------------------------------------------------

    def _on_slider(self, _):
        self.q = np.array([s.val for s in self.sliders], dtype=float)
        self._redraw()

    def _on_event(self, _):
        q = self.executed[int(self.ev.val)]
        for i, s in enumerate(self.sliders):
            s.eventson = False
            s.set_val(float(np.clip(q[i], s.valmin, s.valmax)))
            s.eventson = True
        self.q = np.asarray(q, dtype=float)
        self._redraw()

    def _replay(self, _):
        for k in range(0, len(self.executed), 2):
            self.ev.set_val(k)
            plt.pause(0.04)


# -----------------------------------------------------------------------------

def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--session", default="data/session")
    ap.add_argument("--scene", default="tools/scene_cell.example.json")
    ap.add_argument("--event", type=int, default=None, help="start from this capture's joints")
    ap.add_argument("--set", type=int, default=0, help="which set's cube placement to draw")
    ap.add_argument("--replay", action="store_true", help="animate all executed poses on open")
    ap.add_argument("--path-from", type=int, default=None)
    ap.add_argument("--path-to", type=int, default=None)
    args = ap.parse_args()

    with open(os.path.join(args.session, "meta.json"), "r") as f:
        captures = json.load(f)["captures"]
    joints = np.array([c["capture_robot_joints_6dof"] for c in captures], dtype=float)
    tools = np.array([euler_deg_to_matrix(*c["robot_pose_6dof"]) for c in captures])

    kin = RobotKinematics()
    _, _, stats = fit_tool_transform(kin, joints, tools)
    print(f"[kin] flange->tool fit: {stats['pos_rms_mm']:.2f} mm rms")

    scene = load_scene(args.scene, kin=kin)
    rig = load_camera_rig(args.session)
    view = CubeViewModel()

    cube_path = os.path.join(args.session, "calib_out", "internal_runtime",
                             f"T_base_O_set{args.set}.npy")
    T_base_cube = np.load(cube_path) if os.path.exists(cube_path) else None
    if T_base_cube is None:
        print(f"[viz] no {cube_path}; drawing without the cube")

    q0 = joints[args.event] if args.event is not None else joints[0]

    if args.path_from is not None and args.path_to is not None:
        res = scene.check_path(joints[args.path_from], joints[args.path_to])
        print(f"[path] {args.path_from} -> {args.path_to}: ok={res['ok']} "
              f"min clearance {(res['min_clearance_m'] or 0)*1000:.1f} mm ({res['reason']})")
        steps = np.linspace(0, 1, 40)
        traj = np.array([joints[args.path_from] + s * (joints[args.path_to] - joints[args.path_from])
                         for s in steps])
        viewer = CellViewer(kin, scene, rig, view, T_base_cube, traj[0], executed=traj)
    else:
        viewer = CellViewer(kin, scene, rig, view, T_base_cube, q0, executed=joints)

    if args.replay:
        viewer._replay(None)
    plt.show()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
