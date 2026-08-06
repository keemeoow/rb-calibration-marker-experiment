# tools/cell_scene.py
"""Collision model of the robot cell, good enough to certify generated poses.

Deliberately not MoveIt: the arm is 6 capsules, the cell is a handful of boxes
and cylinders, and every check is a point-to-primitive distance. That is a few
hundred microseconds per pose, so screening 10k candidates is a second of CPU.

The model is only as good as its measurements, so it ships with one guard rail:
``validate_against_executed`` replays joint poses that the robot has already
driven safely (data/session has 232 of them). If the model calls any of those a
collision, the model is wrong - inflated radii or a mis-measured obstacle - and
must be fixed before it is trusted to clear a *new* pose.

Scene units: meters, base frame, same convention as calib_out/T_base_*.npy.
"""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

from tools.robot_kin import RobotKinematics


# -----------------------------------------------------------------------------
# Primitives
# -----------------------------------------------------------------------------

# Links whose distance to a given obstacle is a pose-independent constant (the
# base column and the shoulder both sit on the J1 axis) would otherwise pin the
# reported clearance to that constant and hide what the arm is actually doing.
# Obstacles carry their own exemption list so this stays explicit per obstacle.

@dataclass
class Box:
    name: str
    center: np.ndarray      # (3,) base frame
    half_extents: np.ndarray  # (3,) half size along local axes
    R: np.ndarray = field(default_factory=lambda: np.eye(3))
    exempt_links: Sequence[int] = field(default_factory=tuple)

    def distance_to_points(self, pts: np.ndarray) -> np.ndarray:
        local = (pts - self.center) @ self.R
        outside = np.maximum(np.abs(local) - self.half_extents, 0.0)
        d_out = np.linalg.norm(outside, axis=1)
        d_in = np.max(np.abs(local) - self.half_extents, axis=1)
        return np.where(d_out > 0, d_out, d_in)


@dataclass
class Cylinder:
    """Axis-aligned along +Z of the base frame (typical for stands and legs)."""
    name: str
    center_xy: np.ndarray   # (2,)
    z_min: float
    z_max: float
    radius: float
    exempt_links: Sequence[int] = field(default_factory=tuple)

    def distance_to_points(self, pts: np.ndarray) -> np.ndarray:
        dr = np.linalg.norm(pts[:, :2] - self.center_xy, axis=1) - self.radius
        dz = np.maximum(self.z_min - pts[:, 2], pts[:, 2] - self.z_max)
        both = np.maximum(dr, 0.0) ** 2 + np.maximum(dz, 0.0) ** 2
        outside = np.sqrt(both)
        return np.where((dr > 0) | (dz > 0), outside, np.maximum(dr, dz))


@dataclass
class HalfSpace:
    """Everything below ``z_min`` is forbidden - the floor / table top."""
    name: str
    z_min: float
    exempt_links: Sequence[int] = field(default_factory=tuple)

    def distance_to_points(self, pts: np.ndarray) -> np.ndarray:
        return pts[:, 2] - self.z_min


# -----------------------------------------------------------------------------
# Robot body model
# -----------------------------------------------------------------------------

# Capsule radius per link, meters. Link i spans frame i-1 origin -> frame i origin
# (frame 0 = base). Values are a first estimate; measure the real arm and shrink.
DEFAULT_LINK_RADII_M = [0.11, 0.09, 0.09, 0.07, 0.06, 0.06]
DEFAULT_TOOL_RADIUS_M = 0.075   # gripper + wrist camera envelope
DEFAULT_SAMPLES_PER_LINK = 8

# Link pairs that may touch by construction and must not be self-collision tested.
# Index 6 is the tool stub (flange -> TCP), which is collinear with the wrist.
SELF_COLLISION_EXEMPT = {(0, 1), (1, 2), (2, 3), (3, 4), (4, 5), (5, 6),
                         (0, 2), (1, 3), (2, 4), (3, 5), (4, 6), (3, 6)}


@dataclass
class CellScene:
    kin: RobotKinematics
    obstacles: List[object] = field(default_factory=list)
    link_radii_m: Sequence[float] = field(default_factory=lambda: list(DEFAULT_LINK_RADII_M))
    tool_radius_m: float = DEFAULT_TOOL_RADIUS_M
    samples_per_link: int = DEFAULT_SAMPLES_PER_LINK
    min_clearance_m: float = 0.03
    # Links whose pose never changes (the base column sits on the mounting plate
    # and legitimately intersects the table surface) are not env-collision tested.
    env_exempt_links: Sequence[int] = field(default_factory=lambda: [0])

    # -- geometry -------------------------------------------------------------

    def link_segments(self, q_deg: Sequence[float]) -> List[Tuple[np.ndarray, np.ndarray, float]]:
        """[(p_start, p_end, radius)] for the six links plus the tool stub."""
        frames = self.kin.fk_all_frames(q_deg)
        pts = [np.zeros(3)] + [T[:3, 3] for T in frames]
        segs = [(pts[i], pts[i + 1], float(self.link_radii_m[i])) for i in range(6)]
        tool_tip = (frames[-1] @ self.kin.T_flange_tool)[:3, 3]
        segs.append((pts[6], tool_tip, float(self.tool_radius_m)))
        return segs

    def _sample_segment(self, a: np.ndarray, b: np.ndarray) -> np.ndarray:
        s = np.linspace(0.0, 1.0, self.samples_per_link).reshape(-1, 1)
        return a + s * (b - a)

    # -- checks ---------------------------------------------------------------

    def clearance(self, q_deg: Sequence[float]) -> Tuple[float, str]:
        """Smallest signed distance between the arm and any obstacle."""
        worst, who = np.inf, "none"
        exempt = set(int(i) for i in self.env_exempt_links)
        for i, (a, b, r) in enumerate(self.link_segments(q_deg)):
            if i in exempt:
                continue
            pts = self._sample_segment(a, b)
            for obs in self.obstacles:
                if i in set(int(x) for x in getattr(obs, "exempt_links", ())):
                    continue
                d = float(np.min(obs.distance_to_points(pts))) - r
                if d < worst:
                    worst, who = d, f"link{i}:{obs.name}"
        return worst, who

    def self_clearance(self, q_deg: Sequence[float]) -> Tuple[float, str]:
        segs = self.link_segments(q_deg)
        worst, who = np.inf, "none"
        for i in range(len(segs)):
            for j in range(i + 1, len(segs)):
                if (i, j) in SELF_COLLISION_EXEMPT or j - i <= 1:
                    continue
                pi = self._sample_segment(segs[i][0], segs[i][1])
                pj = self._sample_segment(segs[j][0], segs[j][1])
                d = float(np.min(np.linalg.norm(pi[:, None, :] - pj[None, :, :], axis=2)))
                d -= segs[i][2] + segs[j][2]
                if d < worst:
                    worst, who = d, f"link{i}-link{j}"
        return worst, who

    def check(self, q_deg: Sequence[float]) -> Dict:
        if not self.kin.within_limits(q_deg):
            return {"ok": False, "reason": "joint_limits", "clearance_m": None}
        env_d, env_who = self.clearance(q_deg)
        self_d, self_who = self.self_clearance(q_deg)
        d = min(env_d, self_d)
        ok = d >= self.min_clearance_m
        return {
            "ok": bool(ok),
            "reason": "" if ok else ("environment:" + env_who if env_d <= self_d else "self:" + self_who),
            "clearance_m": float(d),
            "env_clearance_m": float(env_d),
            "self_clearance_m": float(self_d),
        }

    def check_path(self, q_from: Sequence[float], q_to: Sequence[float],
                   n_steps: int = 25) -> Dict:
        """Joint-space straight line, densely sampled - what rb.move() actually does."""
        q0, q1 = np.asarray(q_from, float), np.asarray(q_to, float)
        worst, worst_reason, worst_i = np.inf, "", -1
        for i, s in enumerate(np.linspace(0.0, 1.0, n_steps)):
            res = self.check(q0 + s * (q1 - q0))
            if res["clearance_m"] is None:
                return {"ok": False, "reason": f"joint_limits@step{i}", "min_clearance_m": None}
            if res["clearance_m"] < worst:
                worst, worst_reason, worst_i = res["clearance_m"], res["reason"], i
        return {
            "ok": bool(worst >= self.min_clearance_m),
            "min_clearance_m": float(worst),
            "reason": worst_reason,
            "worst_step": worst_i,
        }

    # -- guard rail -----------------------------------------------------------

    def validate_against_executed(self, joints_deg: np.ndarray,
                                  verbose: bool = True) -> Dict:
        """Every pose the robot has already driven must come out collision-free."""
        joints_deg = np.asarray(joints_deg, dtype=float)
        clearances, failures = [], []
        for idx, q in enumerate(joints_deg):
            res = self.check(q)
            clearances.append(res["clearance_m"] if res["clearance_m"] is not None else -np.inf)
            if not res["ok"]:
                failures.append((idx, res["reason"], res["clearance_m"]))
        clearances = np.asarray(clearances, dtype=float)
        out = {
            "n": int(len(joints_deg)),
            "n_flagged": len(failures),
            "min_clearance_m": float(np.min(clearances)),
            "p05_clearance_m": float(np.percentile(clearances, 5)),
            "failures": failures[:20],
        }
        if verbose:
            print(f"[scene] executed poses: {out['n']}, flagged as colliding: {out['n_flagged']}")
            print(f"[scene] min clearance {out['min_clearance_m']*1000:.1f} mm, "
                  f"p05 {out['p05_clearance_m']*1000:.1f} mm")
            if failures:
                print("[scene] MODEL IS TOO CONSERVATIVE - these poses ran safely in reality:")
                for idx, reason, c in failures[:10]:
                    print(f"        pose {idx}: {reason} ({(c or 0)*1000:.1f} mm)")
        return out


# -----------------------------------------------------------------------------
# Scene I/O
# -----------------------------------------------------------------------------

def load_scene(path: str, kin: Optional[RobotKinematics] = None) -> CellScene:
    """Build a CellScene from a JSON description (see scene_cell.example.json)."""
    with open(path, "r") as f:
        spec = json.load(f)
    obstacles: List[object] = []
    for o in spec.get("obstacles", []):
        kind = o["type"]
        if kind == "box":
            R = np.asarray(o.get("R", np.eye(3).tolist()), dtype=float).reshape(3, 3)
            obstacles.append(Box(o["name"], np.asarray(o["center"], float),
                                 np.asarray(o["half_extents"], float), R,
                                 tuple(o.get("exempt_links", ()))))
        elif kind == "cylinder_z":
            obstacles.append(Cylinder(o["name"], np.asarray(o["center_xy"], float),
                                      float(o["z_min"]), float(o["z_max"]), float(o["radius"]),
                                      tuple(o.get("exempt_links", ()))))
        elif kind == "floor":
            obstacles.append(HalfSpace(o["name"], float(o["z_min"]),
                                       tuple(o.get("exempt_links", ()))))
        else:
            raise ValueError(f"unknown obstacle type: {kind}")
    scene = CellScene(kin=kin or RobotKinematics(), obstacles=obstacles)
    if "link_radii_m" in spec:
        scene.link_radii_m = list(spec["link_radii_m"])
    if "tool_radius_m" in spec:
        scene.tool_radius_m = float(spec["tool_radius_m"])
    if "min_clearance_m" in spec:
        scene.min_clearance_m = float(spec["min_clearance_m"])
    return scene
