# tools/validate_scene.py
"""Check a cell collision model against poses the robot has already driven safely.

Any executed pose the model calls a collision means the model is wrong - too-fat
link capsules, an obstacle placed where there is none, or min_clearance set
higher than the robot actually works at. Shrink until this reports zero, then
the model can be trusted to clear poses the robot has *not* driven.

    python tools/validate_scene.py --session data/session --scene tools/scene_cell.json
"""

from __future__ import annotations

import argparse
import collections
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from robot_comm import euler_deg_to_matrix                      # noqa: E402
from tools.cell_scene import load_scene                          # noqa: E402
from tools.robot_kin import RobotKinematics, fit_tool_transform  # noqa: E402


def load_executed(session: str):
    with open(os.path.join(session, "meta.json"), "r") as f:
        captures = json.load(f)["captures"]
    joints = np.array([c["capture_robot_joints_6dof"] for c in captures], dtype=float)
    tools = np.array([euler_deg_to_matrix(*c["robot_pose_6dof"]) for c in captures])
    gripped = np.array([bool(c.get("cube_gripped")) for c in captures])
    return captures, joints, tools, gripped


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--session", default="data/session")
    ap.add_argument("--scene", default="tools/scene_cell.example.json")
    ap.add_argument("--exclude-placement", action="store_true",
                    help="skip cube-placement poses, which intentionally approach the table")
    args = ap.parse_args()

    captures, joints, tools, gripped = load_executed(args.session)

    kin = RobotKinematics()
    _, scale, stats = fit_tool_transform(kin, joints, tools)
    print(f"[kin] flange->tool fit over {stats['n']} poses: "
          f"{stats['pos_rms_mm']:.2f} mm rms / {stats['rot_rms_deg']:.3f} deg rms")

    if args.exclude_placement:
        keep = gripped
        print(f"[kin] excluding {int((~keep).sum())} placement poses")
        joints = joints[keep]

    scene = load_scene(args.scene, kin=kin)
    report = scene.validate_against_executed(joints)

    if report["n_flagged"]:
        reasons = collections.Counter(r for _, r, _ in report["failures"])
        print("\n[scene] flagged-reason histogram (first 20 failures):")
        for reason, n in reasons.most_common():
            print(f"        {n:4d}  {reason}")
        print("\n[scene] FAIL - fix the model before generating poses with it.")
        return 1

    print("\n[scene] OK - model clears every executed pose.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
