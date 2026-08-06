# tools/robot_kin.py
"""ZEUS ZRA0515P forward/inverse kinematics from the controller's HWDEF DH table.

Units: joints in degrees, lengths in meters (matching the rest of the project;
the DH table below is transcribed in mm and converted on load).

Convention: standard Denavit-Hartenberg, one row per axis,
    A_i = Rz(q_i + offset_i) @ Tz(d_i) @ Tx(a_i) @ Rx(alpha_i)
    T_base_flange = A_1 @ ... @ A_6

The controller reports a *tool* pose, not the flange pose. Fit the constant
flange->tool transform once with ``fit_tool_transform`` against recorded
(joints, tcp) pairs, then FK is directly comparable to ``robot_pose_6dof``.

IK is numeric (damped least squares with multiple random restarts). It is used
offline to generate capture poses, not in a control loop, so a few ms per solve
is irrelevant and it sidesteps the fact that this arm has an offset wrist
(d5 != 0) and therefore no textbook closed-form solution.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Optional, Sequence, Tuple

import numpy as np

# -----------------------------------------------------------------------------
# HWDEF.json DH table (/etc/mcs/HWDEF.json on the robot controller).
# a, d in mm; alpha, theta_offset in degrees.
# Replace with the values read off the controller if the robot is recalibrated.
# -----------------------------------------------------------------------------
DH_A_MM = [0.0, 490.253058740, 0.0, 0.0, 0.0, 0.0]
DH_D_MM = [139.917805809, 0.0, 0.001688990, 370.499718171, 99.998311010, 70.082194192]
DH_ALPHA_DEG = [90.0, 0.0, 90.0, -90.0, 90.0, 0.0]
DH_THETA_OFFSET_DEG = [90.034713401, 90.079301407, 90.225817819,
                       -0.367560603, -0.255824596, 0.0]

# Joint travel used for sampling/IK. Defaults are the range actually exercised in
# data/session (232 executed poses) padded by JOINT_PAD_DEG. They are deliberately
# conservative: widen only after checking the controller's own limits.
OBSERVED_JOINT_MIN_DEG = np.array([-9.6, -76.9, -150.2, -138.7, -136.6, -359.4])
OBSERVED_JOINT_MAX_DEG = np.array([83.3, 34.1, -17.9, 75.8, -19.2, 352.4])
JOINT_PAD_DEG = np.array([20.0, 15.0, 15.0, 30.0, 30.0, 0.0])


@dataclass
class RobotKinematics:
    a_m: np.ndarray = field(default_factory=lambda: np.asarray(DH_A_MM) / 1000.0)
    d_m: np.ndarray = field(default_factory=lambda: np.asarray(DH_D_MM) / 1000.0)
    alpha_deg: np.ndarray = field(default_factory=lambda: np.asarray(DH_ALPHA_DEG, dtype=float))
    theta_offset_deg: np.ndarray = field(
        default_factory=lambda: np.asarray(DH_THETA_OFFSET_DEG, dtype=float))
    joint_min_deg: np.ndarray = field(
        default_factory=lambda: OBSERVED_JOINT_MIN_DEG - JOINT_PAD_DEG)
    joint_max_deg: np.ndarray = field(
        default_factory=lambda: OBSERVED_JOINT_MAX_DEG + JOINT_PAD_DEG)
    # Constant flange->tool transform; identity until fit_tool_transform() is run.
    T_flange_tool: np.ndarray = field(default_factory=lambda: np.eye(4))

    # -- forward kinematics ---------------------------------------------------

    def link_transform(self, axis: int, q_deg: float) -> np.ndarray:
        theta = np.deg2rad(q_deg + self.theta_offset_deg[axis])
        alpha = np.deg2rad(self.alpha_deg[axis])
        ct, st = np.cos(theta), np.sin(theta)
        ca, sa = np.cos(alpha), np.sin(alpha)
        a, d = self.a_m[axis], self.d_m[axis]
        return np.array([
            [ct, -st * ca,  st * sa, a * ct],
            [st,  ct * ca, -ct * sa, a * st],
            [0.0,      sa,       ca,      d],
            [0.0,     0.0,      0.0,    1.0],
        ], dtype=np.float64)

    def fk_all_frames(self, q_deg: Sequence[float]) -> List[np.ndarray]:
        """[T_base_1, ..., T_base_6]; frame i sits at the origin of link i."""
        frames = []
        T = np.eye(4)
        for i in range(6):
            T = T @ self.link_transform(i, float(q_deg[i]))
            frames.append(T.copy())
        return frames

    def fk_flange(self, q_deg: Sequence[float]) -> np.ndarray:
        return self.fk_all_frames(q_deg)[-1]

    def fk_tool(self, q_deg: Sequence[float]) -> np.ndarray:
        return self.fk_flange(q_deg) @ self.T_flange_tool

    # -- inverse kinematics ---------------------------------------------------

    def _pose_error(self, T_cur: np.ndarray, T_goal: np.ndarray) -> np.ndarray:
        """6-vector [translation_m, rotation_rad] error, current -> goal."""
        e_t = T_goal[:3, 3] - T_cur[:3, 3]
        R_err = T_goal[:3, :3] @ T_cur[:3, :3].T
        angle = np.arccos(np.clip((np.trace(R_err) - 1.0) / 2.0, -1.0, 1.0))
        if angle < 1e-9:
            e_r = np.zeros(3)
        else:
            axis = np.array([R_err[2, 1] - R_err[1, 2],
                             R_err[0, 2] - R_err[2, 0],
                             R_err[1, 0] - R_err[0, 1]]) / (2.0 * np.sin(angle))
            e_r = axis * angle
        return np.concatenate([e_t, e_r])

    def jacobian(self, q_deg: Sequence[float], tool: bool = True) -> np.ndarray:
        """Geometric Jacobian in base frame, columns per joint (rad-based)."""
        frames = self.fk_all_frames(q_deg)
        T_end = frames[-1] @ self.T_flange_tool if tool else frames[-1]
        p_end = T_end[:3, 3]
        J = np.zeros((6, 6))
        z_prev = np.array([0.0, 0.0, 1.0])
        p_prev = np.zeros(3)
        for i in range(6):
            J[:3, i] = np.cross(z_prev, p_end - p_prev)
            J[3:, i] = z_prev
            z_prev = frames[i][:3, 2]
            p_prev = frames[i][:3, 3]
        return J

    def ik(self, T_goal: np.ndarray, q_seed_deg: Sequence[float],
           tool: bool = True, max_iter: int = 120,
           tol_pos_m: float = 1e-5, tol_rot_rad: float = 1e-5,
           damping: float = 1e-3) -> Optional[np.ndarray]:
        """Damped least squares IK from one seed. Returns joints (deg) or None."""
        q = np.asarray(q_seed_deg, dtype=np.float64).copy()
        for _ in range(max_iter):
            T_cur = self.fk_tool(q) if tool else self.fk_flange(q)
            err = self._pose_error(T_cur, T_goal)
            if np.linalg.norm(err[:3]) < tol_pos_m and np.linalg.norm(err[3:]) < tol_rot_rad:
                q_wrapped = self._wrap_into_limits(q)
                return q_wrapped if q_wrapped is not None else None
            J = self.jacobian(q, tool=tool)
            JT = J.T
            dq = JT @ np.linalg.solve(J @ JT + damping * np.eye(6), err)
            step = np.rad2deg(dq)
            # Trust region: never take a step larger than 20 deg on any joint.
            scale = min(1.0, 20.0 / max(np.max(np.abs(step)), 1e-12))
            q = q + step * scale
        return None

    def ik_multi(self, T_goal: np.ndarray, n_seeds: int = 40,
                 rng: Optional[np.random.Generator] = None,
                 extra_seeds: Sequence[Sequence[float]] = (),
                 tool: bool = True, dedup_deg: float = 2.0) -> List[np.ndarray]:
        """Collect distinct IK branches by restarting from many seeds."""
        rng = rng or np.random.default_rng(0)
        seeds = [np.asarray(s, dtype=float) for s in extra_seeds]
        lo, hi = self.joint_min_deg, self.joint_max_deg
        seeds += [rng.uniform(lo, hi) for _ in range(n_seeds)]
        found: List[np.ndarray] = []
        for seed in seeds:
            q = self.ik(T_goal, seed, tool=tool)
            if q is None:
                continue
            if any(np.max(np.abs(q - f)) < dedup_deg for f in found):
                continue
            found.append(q)
        return found

    def _wrap_into_limits(self, q_deg: np.ndarray) -> Optional[np.ndarray]:
        """Fold each joint by +-360 into its limit band; None if impossible."""
        q = np.asarray(q_deg, dtype=float).copy()
        for i in range(6):
            for _ in range(4):
                if q[i] < self.joint_min_deg[i]:
                    q[i] += 360.0
                elif q[i] > self.joint_max_deg[i]:
                    q[i] -= 360.0
                else:
                    break
            if not (self.joint_min_deg[i] <= q[i] <= self.joint_max_deg[i]):
                return None
        return q

    def within_limits(self, q_deg: Sequence[float]) -> bool:
        q = np.asarray(q_deg, dtype=float)
        return bool(np.all(q >= self.joint_min_deg) and np.all(q <= self.joint_max_deg))

    def manipulability(self, q_deg: Sequence[float]) -> float:
        """Yoshikawa index; near-zero means near a singularity."""
        J = self.jacobian(q_deg)
        return float(np.sqrt(max(np.linalg.det(J @ J.T), 0.0)))


# -----------------------------------------------------------------------------
# Fitting the constant flange->tool transform from recorded (joints, tcp) pairs
# -----------------------------------------------------------------------------

def fit_tool_transform(kin: RobotKinematics,
                       joints_deg: np.ndarray,
                       T_base_tool: Sequence[np.ndarray],
                       fit_scale: bool = False) -> Tuple[np.ndarray, float, dict]:
    """Least-squares fit of T_flange_tool (and optionally an isotropic DH scale).

    Returns (T_flange_tool, scale, residual_stats). With ``fit_scale`` the DH
    translations are multiplied by ``scale`` before fitting, which is the direct
    test for a kinematic scale error in the controller's model.
    """
    from scipy.optimize import least_squares  # local import: optional dependency

    joints_deg = np.asarray(joints_deg, dtype=float)
    T_meas = np.asarray(T_base_tool, dtype=float)
    base_a, base_d = kin.a_m.copy(), kin.d_m.copy()

    def unpack(p):
        t = p[:3]
        rvec = p[3:6]
        theta = np.linalg.norm(rvec)
        if theta < 1e-12:
            R = np.eye(3)
        else:
            k = rvec / theta
            K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
            R = np.eye(3) + np.sin(theta) * K + (1 - np.cos(theta)) * (K @ K)
        T = np.eye(4)
        T[:3, :3], T[:3, 3] = R, t
        s = p[6] if fit_scale else 1.0
        return T, s

    def residuals(p):
        T_ft, s = unpack(p)
        kin.a_m, kin.d_m = base_a * s, base_d * s
        kin.T_flange_tool = T_ft
        out = []
        for q, Tm in zip(joints_deg, T_meas):
            e = kin._pose_error(kin.fk_tool(q), Tm)
            out.append(np.concatenate([e[:3] * 1000.0, np.rad2deg(e[3:])]))
        return np.concatenate(out)

    p0 = np.zeros(7 if fit_scale else 6)
    if fit_scale:
        p0[6] = 1.0
    sol = least_squares(residuals, p0, method="lm", max_nfev=4000)
    T_ft, s = unpack(sol.x)
    kin.a_m, kin.d_m = base_a * s, base_d * s
    kin.T_flange_tool = T_ft

    r = residuals(sol.x).reshape(-1, 6)
    stats = {
        "pos_rms_mm": float(np.sqrt(np.mean(np.sum(r[:, :3] ** 2, axis=1)))),
        "pos_max_mm": float(np.max(np.linalg.norm(r[:, :3], axis=1))),
        "rot_rms_deg": float(np.sqrt(np.mean(np.sum(r[:, 3:] ** 2, axis=1)))),
        "rot_max_deg": float(np.max(np.linalg.norm(r[:, 3:], axis=1))),
        "n": int(len(joints_deg)),
    }
    return T_ft, float(s), stats
