#!/usr/bin/env python3
"""Add d6-roll clones to a taught A-pose pool without going back to the robot.

`test_robot_teaching_waypoint.py` fails the pose pool when the relative-rotation
axes collapse onto a plane (3rd singular value < 0.40). That is the classical
`AX=XB` degeneracy: hand-eye rotation about the missing axis is unconstrained,
which hits the C1-C4 classical baselines hardest since they have nothing else to
lean on. Teaching yaw-varied viewpoints does not fix it -- only rotating the
camera about its own optical axis does.

That rotation is exactly joint 6. The tool frame (`settool(3, 0, 0, z, 0, 0, 0)`)
is a pure translation along the J6 axis, so rolling d6 leaves the TCP *position*
untouched and only spins the camera. A roll clone is therefore computable exactly:

    capture_joints : same, with d6 += roll
    capture_tcp    : same xyz, orientation R -> R @ Rz(roll)

Nothing is estimated, so this is not a substitute for teaching a genuinely new
viewpoint -- it is the same viewpoint seen with the camera rolled, which is
precisely the missing degree of freedom.

The camera sits off the J6 axis (`settool(2, 0, 35, 330, ...)`), so a rolled
clone translates the camera ~70mm and shifts the target in frame. The optical
axis direction does not change and marker detection is rotation-invariant, so the
cube stays detectable -- but confirm it in the dry-run before the real capture.

    # see what a roll would do, write nothing
    python tools/add_roll_clone_poses.py --poses data/session02/calib_train/capture_poses_001.json

    # write the augmented pool
    python tools/add_roll_clone_poses.py \
        --poses data/session02/calib_train/capture_poses_001.json \
        --source 8,9,10,2,15 --roll 180 \
        --output data/session02/calib_train/capture_poses_002.json
"""
import argparse
import itertools
import json
import math
import os
import sys

import numpy as np

SV3_FAIL = 0.40   # test_robot_teaching_waypoint.py pose threshold
SV3_TARGET = 0.45  # aim above the threshold so gate-rejected frames cannot undo it


def euler_to_R(rz, ry, rx):
    """ZYX extrinsic, matching robot_comm.euler_deg_to_matrix."""
    a, b, c = math.radians(rz), math.radians(ry), math.radians(rx)
    Rz = np.array([[math.cos(a), -math.sin(a), 0], [math.sin(a), math.cos(a), 0], [0, 0, 1.0]])
    Ry = np.array([[math.cos(b), 0, math.sin(b)], [0, 1.0, 0], [-math.sin(b), 0, math.cos(b)]])
    Rx = np.array([[1.0, 0, 0], [0, math.cos(c), -math.sin(c)], [0, math.sin(c), math.cos(c)]])
    return Rz @ Ry @ Rx


def R_to_euler(R):
    """Inverse of euler_to_R. Gimbal lock (|ry|=90) is not reachable here: the
    poses all look roughly downward, far from the singularity."""
    if abs(R[2, 0]) > 0.9999:
        raise ValueError("pose is at gimbal lock (ry = +/-90 deg); cannot invert")
    ry = math.degrees(math.asin(-R[2, 0]))
    rz = math.degrees(math.atan2(R[1, 0], R[0, 0]))
    rx = math.degrees(math.atan2(R[2, 1], R[2, 2]))
    return rz, ry, rx


def _log_SO3(R):
    c = np.clip((np.trace(R) - 1) / 2.0, -1, 1)
    a = math.acos(c)
    if a < 1e-8:
        return np.zeros(3)
    return a / (2 * math.sin(a)) * np.array(
        [R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])


def rot_axis_sv3(Rs):
    """3rd singular value of the pairwise relative-rotation axes — the gate metric."""
    axes = []
    for i, j in itertools.combinations(range(len(Rs)), 2):
        v = _log_SO3(Rs[i] @ Rs[j].T)
        n = np.linalg.norm(v)
        if n > 1e-3:
            axes.append(v / n)
    if len(axes) < 3:
        return 0.0
    return float(np.linalg.svd(np.array(axes), compute_uv=False)[2] / math.sqrt(len(axes)))


def pose_R(p):
    t = p["capture_tcp"]
    return euler_to_R(t[3], t[4], t[5])


def clone(p, roll, new_index):
    """One roll clone. TCP position and joints 1-5 are untouched by a d6 rotation."""
    Rz = euler_to_R(roll, 0.0, 0.0)
    rz, ry, rx = R_to_euler(pose_R(p) @ Rz)
    t = p["capture_tcp"]
    j = list(p["capture_joints"])
    j[5] = float(j[5]) + float(roll)
    out = {
        "pose_index": int(new_index),
        "capture_joints": [round(float(v), 3) for v in j],
        "capture_tcp": [round(float(t[0]), 3), round(float(t[1]), 3), round(float(t[2]), 3),
                        round(rz, 6), round(ry, 6), round(rx, 6)],
        "derived_from_pose_index": int(p.get("pose_index", -1)),
        "derived_by": "d6_roll_clone",
        "derived_roll_deg": float(roll),
    }
    # Carried through unchanged: the combiner never reads it, and a rolled clone
    # observes the same cube from the same point.
    if "cube_center_6dof" in p:
        out["cube_center_6dof"] = list(p["cube_center_6dof"])
    return out


def greedy_plan(poses, roll, target):
    """Fewest source poses whose clones lift sv3 to `target`, best-first."""
    Rs = [pose_R(p) for p in poses]
    cur, chosen = list(Rs), []
    for _ in range(len(poses)):
        best = None
        for i in range(len(Rs)):
            if i in chosen:
                continue
            s = rot_axis_sv3(cur + [Rs[i] @ euler_to_R(roll, 0.0, 0.0)])
            if best is None or s > best[0]:
                best = (s, i)
        if best is None:
            break
        cur.append(Rs[best[1]] @ euler_to_R(roll, 0.0, 0.0))
        chosen.append(best[1])
        if best[0] >= target:
            break
    return chosen, rot_axis_sv3(cur)


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--poses", required=True, help="taught capture_poses JSON")
    ap.add_argument("--source", default=None,
                    help="comma-separated pose_index values to clone. Omit to let "
                         "the tool pick the fewest that reach the target.")
    ap.add_argument("--roll", type=float, default=180.0,
                    help="d6 rotation in degrees (default 180; 150-210 all pass)")
    ap.add_argument("--target_sv3", type=float, default=SV3_TARGET)
    ap.add_argument("--d6_limit_deg", type=float, nargs=2, default=(-360.0, 360.0),
                    metavar=("LO", "HI"),
                    help="reject clones whose d6 leaves this range (default +/-360, "
                         "the usual J6 travel; narrow it if this robot is limited)")
    ap.add_argument("--output", default=None,
                    help="write the augmented pool here (omit to only report)")
    args = ap.parse_args()

    with open(args.poses) as f:
        data = json.load(f)
    poses = data.get("capture_poses")
    if not isinstance(poses, list) or len(poses) < 3:
        sys.exit("[ERROR] capture_poses must hold at least 3 taught poses")

    # A round-trip check costs nothing and catches a euler-convention mismatch
    # before it silently produces poses the robot would move to incorrectly.
    for p in poses[:3]:
        t = p["capture_tcp"]
        rz, ry, rx = R_to_euler(euler_to_R(t[3], t[4], t[5]))
        if max(abs(rz - t[3]), abs(ry - t[4]), abs(rx - t[5])) > 1e-6:
            sys.exit("[ERROR] euler round-trip mismatch; convention differs from "
                     "robot_comm.euler_deg_to_matrix. Do not trust the output.")

    by_index = {int(p.get("pose_index", i)): i for i, p in enumerate(poses)}
    sv3_before = rot_axis_sv3([pose_R(p) for p in poses])
    print("taught poses: {}   sv3 = {:.3f}   (FAIL below {:.2f}, target {:.2f})".format(
        len(poses), sv3_before, SV3_FAIL, args.target_sv3))

    d6s = [float(p["capture_joints"][5]) for p in poses]
    lo, hi = args.d6_limit_deg
    print("  d6 taught range [{:.1f}, {:.1f}]   allowed [{:.1f}, {:.1f}]".format(
        min(d6s), max(d6s), lo, hi))

    if sv3_before >= args.target_sv3:
        print("\nAlready at target. Nothing to add.")
        return 0

    if args.source:
        try:
            want = [int(x) for x in args.source.split(",") if x.strip()]
        except ValueError:
            sys.exit("[ERROR] --source must be comma-separated integers")
        missing = [w for w in want if w not in by_index]
        if missing:
            sys.exit("[ERROR] pose_index not in pool: {}".format(missing))
        picked = [by_index[w] for w in want]
    else:
        picked, _ = greedy_plan(poses, args.roll, args.target_sv3)
        print("\n(no --source given; picked the fewest that reach the target)")

    print("\nroll {:+.0f} deg on {} pose(s):".format(args.roll, len(picked)))
    new_entries, kept = [], []
    next_index = max(by_index) + 1
    for i in picked:
        src = poses[i]
        src_d6 = float(src["capture_joints"][5])
        d6 = src_d6 + float(args.roll)
        if not (lo <= d6 <= hi):
            print("  #{:<3} -> (skipped)  d6 {:8.2f} -> {:8.2f}   <-- OUT OF RANGE "
                  "[{:.0f}, {:.0f}]".format(src.get("pose_index"), src_d6, d6, lo, hi))
            continue
        c = clone(src, args.roll, next_index)
        print("  #{:<3} -> #{:<3}  d6 {:8.2f} -> {:8.2f}".format(
            src.get("pose_index"), c["pose_index"], src_d6, d6))
        new_entries.append(c)
        kept.append(i)
        next_index += 1

    if not new_entries:
        sys.exit("\n[ERROR] every clone left the d6 range. Try the opposite sign "
                 "(--roll {:+.0f}) or widen --d6_limit_deg.".format(-args.roll))

    merged = list(poses) + new_entries
    sv3_after = rot_axis_sv3([pose_R(p) for p in merged])
    ok = sv3_after >= SV3_FAIL
    print("\nafter: {} poses   sv3 {:.3f} -> {:.3f}   {}".format(
        len(merged), sv3_before, sv3_after,
        "PASS" if ok else "STILL FAILING — add more or use a different --roll"))
    if ok and sv3_after < args.target_sv3:
        print("  [!] above the {:.2f} threshold but below the {:.2f} target; a few "
              "gate-rejected frames could push it back under.".format(SV3_FAIL, args.target_sv3))

    if args.output is None:
        print("\n(--output not given; nothing written)")
        return 0 if ok else 1

    if os.path.abspath(args.output) == os.path.abspath(args.poses):
        sys.exit("[ERROR] refusing to overwrite the taught pool in place. "
                 "Write to a new file.")
    os.makedirs(os.path.dirname(os.path.abspath(args.output)) or ".", exist_ok=True)
    out = dict(data)
    out["capture_poses"] = merged
    out["roll_clone_fill"] = {
        "source": os.path.abspath(args.poses),
        "roll_deg": float(args.roll),
        "n_taught": len(poses),
        "n_synthesised": len(new_entries),
        "sv3_before": sv3_before,
        "sv3_after": sv3_after,
        "method": "clone pose, rotate d6 only (TCP position invariant)",
    }
    with open(args.output, "w") as f:
        json.dump(out, f, indent=2)
    print("[ok] wrote {}".format(args.output))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
