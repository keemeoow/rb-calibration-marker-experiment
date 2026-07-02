# experiment_config.py
"""Experiment-level transforms for the two robot-camera calibration studies.

This module is deliberately kept SEPARATE from ``config.py``:

  * ``config.CubeConfig``  -> the rigid AprilTag target geometry ONLY
                              (marker IDs, sizes, centers, faces). Reusable.
  * ``experiment_config``  -> how that cube relates to the robot/camera in a
                              SPECIFIC experiment (grasp geometry, fixed-target
                              pose, and what is treated as constant vs. optimized).

Both experiments reuse the SAME ``CubeConfig`` (see ``make_first_experiment`` /
``make_second_experiment``) so the accuracy comparison is fair.

--------------------------------------------------------------------------------
Units & conventions
--------------------------------------------------------------------------------
  * All translations are METERS. All poses are 4x4 homogeneous transforms
    (numpy float64), rotation in the top-left 3x3, translation in the last col.
  * Transform naming ``T_A_B`` maps a point from frame B into frame A:
        p_A = T_A_B @ p_B
    so ``T_A_C = T_A_B @ T_B_C``.

Frame names
  B  = robot base
  G  = the end frame the robot FK returns. THIS IS AMBIGUOUS ON PURPOSE:
       you MUST declare whether your FK returns the FLANGE or a defined TCP,
       via ``GraspConfig.gripper_frame``, and make ``T_G_O`` consistent with it.
  O  = cube object frame  (center of the 59mm cube; see config.CubeConfig)
  C  = camera frame

--------------------------------------------------------------------------------
Experiment 1 (GRIPPER_HELD): gripper holds the cube
--------------------------------------------------------------------------------
    T_B_O_i = T_B_G_i @ T_G_O          (FK  ×  grasp geometry)
    T_C_O_i  from AprilTag PnP
    fixed camera:  T_B_C = T_B_O_i @ inv(T_C_O_i)   (averaged over poses)
  -> uses GraspConfig.T_G_O.

--------------------------------------------------------------------------------
Experiment 2 (FIXED_TARGET): cube is placed and fixed, gripper/camera move
--------------------------------------------------------------------------------
    T_B_O ~= T_B_G_i @ T_G_C @ T_C_O_i     (T_B_O is constant)
  -> DOES NOT use T_G_O at all (structurally: FixedTargetConfig has no such
     field). Estimates the eye-in-hand extrinsic T_G_C, optionally also T_B_O.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import List, Optional, Tuple

import numpy as np

from config import CubeConfig, get_default_cube_config


# =============================================================================
# 4x4 transform helpers (meters + radians)
# =============================================================================
def identity_T() -> np.ndarray:
    return np.eye(4, dtype=np.float64)


def make_T(R: np.ndarray, t) -> np.ndarray:
    """Assemble a 4x4 from a 3x3 rotation and a 3-vector translation (meters)."""
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = np.asarray(R, dtype=np.float64).reshape(3, 3)
    T[:3, 3] = np.asarray(t, dtype=np.float64).reshape(3)
    return T


def T_from_translation_rpy(
    xyz_m: Tuple[float, float, float],
    rpy_rad: Tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> np.ndarray:
    """Build a pose from a translation (m) and intrinsic X-Y-Z (roll-pitch-yaw)
    rotation in radians. Handy for typing measured grasp geometry by hand."""
    rx, ry, rz = (float(a) for a in rpy_rad)
    cx, sx = np.cos(rx), np.sin(rx)
    cy, sy = np.cos(ry), np.sin(ry)
    cz, sz = np.cos(rz), np.sin(rz)
    Rx = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]], dtype=np.float64)
    Ry = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]], dtype=np.float64)
    Rz = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]], dtype=np.float64)
    return make_T(Rz @ Ry @ Rx, xyz_m)


def is_valid_rigid_T(T, tol: float = 1e-6) -> bool:
    """True iff T is a proper 4x4 rigid transform (orthonormal R, det +1)."""
    try:
        T = np.asarray(T, dtype=np.float64).reshape(4, 4)
    except Exception:
        return False
    R = T[:3, :3]
    if not np.allclose(R.T @ R, np.eye(3), atol=tol):
        return False
    if not np.isclose(np.linalg.det(R), 1.0, atol=tol):
        return False
    return np.allclose(T[3, :], [0, 0, 0, 1], atol=tol)


class ExperimentMode(str, Enum):
    GRIPPER_HELD = "gripper_held"   # 1차: cube in gripper; uses FK + T_G_O
    FIXED_TARGET = "fixed_target"   # 2차: cube fixed; estimates T_G_C


class GripperFrame(str, Enum):
    """Which frame the robot FK actually returns. Pick ONE and be consistent."""
    FLANGE = "flange"   # tool0 / mechanical flange
    TCP = "tcp"         # a defined tool center point
    GRIPPER = "gripper" # gripper body frame, if your controller reports that


# =============================================================================
# Experiment 1 (GRIPPER_HELD): grasp geometry
# =============================================================================
@dataclass
class GraspConfig:
    """Rigid transform from the robot end frame (G) to the cube object frame (O).

    Only used by Experiment 1. This is grasp geometry, NOT cube geometry, which
    is exactly why it lives here and not in CubeConfig.
    """

    # Which frame the FK returns. Make T_G_O consistent with this choice.
    gripper_frame: GripperFrame = GripperFrame.FLANGE

    # -------------------------------------------------------------------------
    # T_G_O : end-frame (G) -> cube object frame (O), meters.
    #         p_G = T_G_O @ p_O.
    #
    # TODO(measure): fill this from the physical grasp. How to obtain it:
    #   (a) CAD route: from the gripper CAD + the cube STL, read the pose of the
    #       cube center (O; center of the 59mm cube, +Z up, +X toward ID 4) in
    #       the flange/TCP frame you selected above. Enter translation (m) and
    #       an X-Y-Z roll/pitch/yaw via T_from_translation_rpy(...).
    #   (b) Calibration route: run Experiment 1 once with an approximate value,
    #       then solve for T_G_O that best satisfies
    #           T_B_O_i = T_B_G_i @ T_G_O   across all poses,
    #       and paste the refined 4x4 here.
    # Leave as identity + is_placeholder=True until measured; validate() warns.
    # -------------------------------------------------------------------------
    T_G_O: np.ndarray = field(default_factory=identity_T)
    is_placeholder: bool = True

    def validate(self) -> Tuple[bool, List[str]]:
        problems: List[str] = []
        if not is_valid_rigid_T(self.T_G_O):
            problems.append("[grasp] T_G_O is not a proper rigid 4x4 transform")
        if self.is_placeholder:
            problems.append("[grasp] T_G_O is still the TODO placeholder (identity); measure it")
        return (len(problems) == 0), problems

    def T_B_O(self, T_B_G: np.ndarray) -> np.ndarray:
        """Cube pose in base for one FK sample: T_B_O = T_B_G @ T_G_O."""
        return np.asarray(T_B_G, dtype=np.float64).reshape(4, 4) @ np.asarray(self.T_G_O, dtype=np.float64).reshape(4, 4)


# =============================================================================
# Experiment 2 (FIXED_TARGET): cube fixed in the world/base
# =============================================================================
@dataclass
class FixedTargetConfig:
    """The cube is a fixed calibration target. NO grasp transform exists here.

    T_B_O is either a known constant prior or an optimized variable:
      * optimize_cube_pose=True  -> T_B_O_prior is only a starting guess (or None)
      * optimize_cube_pose=False -> T_B_O_prior is held fixed as a hard constant.
    Experiment 2 estimates the eye-in-hand extrinsic T_G_C in both cases.
    """

    # Optional prior / constant for T_B_O (base -> cube object frame), meters.
    # None means "no prior; initialize from data during optimization".
    T_B_O_prior: Optional[np.ndarray] = None
    optimize_cube_pose: bool = True

    def validate(self) -> Tuple[bool, List[str]]:
        problems: List[str] = []
        if self.T_B_O_prior is not None and not is_valid_rigid_T(self.T_B_O_prior):
            problems.append("[fixed] T_B_O_prior is not a proper rigid 4x4 transform")
        if not self.optimize_cube_pose and self.T_B_O_prior is None:
            problems.append("[fixed] optimize_cube_pose=False requires a constant T_B_O_prior")
        return (len(problems) == 0), problems


# =============================================================================
# Experiment container
# =============================================================================
@dataclass
class ExperimentConfig:
    """Ties an experiment mode to the shared cube geometry and its transforms.

    Exactly one of ``grasp`` / ``fixed_target`` is populated, matching ``mode``.
    """

    name: str
    mode: ExperimentMode
    cube: CubeConfig
    grasp: Optional[GraspConfig] = None            # set iff mode == GRIPPER_HELD
    fixed_target: Optional[FixedTargetConfig] = None  # set iff mode == FIXED_TARGET

    def validate(self, strict_grasp: bool = False) -> Tuple[bool, List[str]]:
        """Structural + geometric validation.

        strict_grasp=True treats a placeholder T_G_O as a hard failure (use once
        you have measured it); default keeps it a soft warning so the pipeline
        can be wired up before the grasp is known.
        """
        # geometry-aware cube validation lives with the model
        from apriltag_cube import validate_cube_config
        ok, problems = validate_cube_config(self.cube)

        if self.mode == ExperimentMode.GRIPPER_HELD:
            if self.fixed_target is not None:
                problems.append("[mode] GRIPPER_HELD must not carry a fixed_target config")
            if self.grasp is None:
                problems.append("[mode] GRIPPER_HELD requires a GraspConfig")
            else:
                g_ok, g_problems = self.grasp.validate()
                if strict_grasp:
                    problems.extend(g_problems)
                else:
                    problems.extend(p for p in g_problems if "placeholder" not in p)
        elif self.mode == ExperimentMode.FIXED_TARGET:
            if self.grasp is not None:
                problems.append("[mode] FIXED_TARGET must not carry a grasp config (no T_G_O)")
            if self.fixed_target is None:
                problems.append("[mode] FIXED_TARGET requires a FixedTargetConfig")
            else:
                problems.extend(self.fixed_target.validate()[1])

        return (len(problems) == 0), problems


# =============================================================================
# Factories - both experiments share the SAME cube geometry
# =============================================================================
def make_first_experiment(cube: Optional[CubeConfig] = None,
                          grasp: Optional[GraspConfig] = None) -> ExperimentConfig:
    """Experiment 1: cube held by gripper, FK-based (uses T_G_O)."""
    return ExperimentConfig(
        name="exp1_gripper_held",
        mode=ExperimentMode.GRIPPER_HELD,
        cube=cube or get_default_cube_config(),
        grasp=grasp or GraspConfig(),
    )


def make_second_experiment(cube: Optional[CubeConfig] = None,
                           fixed_target: Optional[FixedTargetConfig] = None) -> ExperimentConfig:
    """Experiment 2: cube fixed as a calibration target (no T_G_O)."""
    return ExperimentConfig(
        name="exp2_fixed_target",
        mode=ExperimentMode.FIXED_TARGET,
        cube=cube or get_default_cube_config(),
        fixed_target=fixed_target or FixedTargetConfig(),
    )


def assert_same_cube(exp_a: ExperimentConfig, exp_b: ExperimentConfig) -> bool:
    """Guard that a fair comparison uses identical cube geometry in both runs."""
    from cube_config_utils import cube_configs_equivalent
    return cube_configs_equivalent(exp_a.cube, exp_b.cube)


if __name__ == "__main__":
    shared_cube = get_default_cube_config()
    exp1 = make_first_experiment(cube=shared_cube)
    exp2 = make_second_experiment(cube=shared_cube)

    print("Frame convention: T_A_B maps points from B to A (p_A = T_A_B @ p_B).")
    print(f"exp1: mode={exp1.mode.value}  gripper_frame={exp1.grasp.gripper_frame.value}")
    print(f"      T_B_O_i = T_B_G_i @ T_G_O   (T_G_O placeholder={exp1.grasp.is_placeholder})")
    print(f"exp2: mode={exp2.mode.value}  optimize_cube_pose={exp2.fixed_target.optimize_cube_pose}")
    print(f"      T_B_O ~= T_B_G_i @ T_G_C @ T_C_O_i   (no T_G_O used)")

    ok1, p1 = exp1.validate()
    ok2, p2 = exp2.validate()
    print(f"\nexp1 valid (soft grasp): {ok1}  {p1}")
    print(f"exp2 valid:              {ok2}  {p2}")
    print(f"same cube geometry:      {assert_same_cube(exp1, exp2)}")
