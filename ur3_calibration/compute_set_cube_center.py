#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ur3_calibration/compute_set_cube_center.py -- derive session2's set_cube_center_6dof.

Pass 2 needs, for each of session2's 12 floor placements, "the flange pose
when the cube was actually set down at that spot" (T_base_flange_at_placement)
combined with Pass 1's fitted T_gripper_cube (ur3_calibration/
pass1_grasp_offset.json) to get T_base_cube_taught[s] = T_base_flange_at_
placement[s] @ T_gripper_cube.

T_base_flange_at_placement is NOT in session2's poses.json (those are the
camera-observation TCP poses recorded while teaching the sweep, not the
actual place targets) -- it is reconstructed by calling
session2_pick_and_place.compute_ordered_targets() with the same inputs the
real robot run used (grasp_flange_pose.json's rotation/z, session2's
poses.json for x/y/yaw), which is exactly how build_session2_steps()
computed each place target for the real pick-and-place execution. Its
returned list is already in delta_yaw-sorted order, which is also the actual
capture order (see convert_to_meta.py's docstring) -- so items[i]["target"]
is the flange pose for capture index i / set_index (i+1).

Cross-check performed here (see printed output and the report): the
resulting T_base_cube_taught[s]'s x/y should closely match Pass 1's
independent vision-only T_base_cube_by_set_vision_only[s] (fit purely from
image corners, no FK/grasp-offset involved) -- these are two fully
independent pose estimates of the same 12 physical placements, so close
agreement is strong evidence both the T_gripper_cube fit and this
reconstruction are correct; large disagreement would mean stop and
reconsider (see the report for the actual numbers, which agree to within a
few mm).

Encoding: schema.py's FK_FIXED_CONTRACT states A3's fixed pose is
T_base_cube[s] = T_base_fk_raw[s] @ T_cube_center_tag_object_mechanical,
where the mechanical map (RAW_FK_CUBE_CENTER_TO_OBJECT, a diag(-1,1,-1)
rotation) is applied UNCONDITIONALLY by table1.py to whatever meta.json
stores in set_cube_center_6dof, regardless of dataset/robot -- it encodes a
fixed relationship between "the raw taught pose format" and the true cube
object frame. We did not go through that "raw taught pose" format at all --
T_base_cube_taught computed here already IS the true cube pose. Since the
mechanical map is a rotation matrix that is its own inverse (checked: M@M=I
for this specific diag(-1,1,-1) case), pre-composing it once here exactly
cancels table1.py's later re-application:
    stored := T_base_cube_taught[s] @ RAW_FK_CUBE_CENTER_TO_OBJECT
    table1.py later computes: stored @ RAW_FK_CUBE_CENTER_TO_OBJECT
                             = T_base_cube_taught[s] @ M @ M
                             = T_base_cube_taught[s]   (correct)
This is stored both as set_cube_center_6dof (mm + ZYX-Euler-degrees, matching
capture_pipeline.robot.euler_deg_to_matrix's exact decomposition -- verified
by a 2000-sample round-trip test) and as canonical_set_cube_center_matrix_4x4
(the exact matrix, which calibration_pipeline.runtime.
get_capture_set_cube_center_transform_raw() prefers when present, avoiding
any Euler round-trip precision loss).

Usage:
  python ur3_calibration/compute_set_cube_center.py \\
      --pass1-json ur3_calibration/pass1_grasp_offset.json \\
      --out ur3_calibration/set_cube_center_pass2.json
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from session2_pick_and_place import compute_ordered_targets, load_pose  # noqa: E402
from capture_poses import session_path  # noqa: E402
from convert_to_meta import tcp_pose_to_matrix, matrix_to_pose6_zyx_deg  # noqa: E402
from calibration_pipeline.schema import RAW_FK_CUBE_CENTER_TO_OBJECT  # noqa: E402

MECHANICAL_MAP = np.asarray(RAW_FK_CUBE_CENTER_TO_OBJECT, dtype=np.float64)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--pass1-json", default=str(Path(__file__).resolve().parent / "pass1_grasp_offset.json"))
    ap.add_argument("--out", default=str(Path(__file__).resolve().parent / "set_cube_center_pass2.json"))
    args = ap.parse_args()

    assert np.allclose(MECHANICAL_MAP @ MECHANICAL_MAP, np.eye(4), atol=1e-12), (
        "RAW_FK_CUBE_CENTER_TO_OBJECT is expected to be its own inverse")

    pass1 = json.loads(Path(args.pass1_json).read_text())
    T_gripper_cube = np.asarray(pass1["T_gripper_cube"], dtype=np.float64)
    vision_only = {
        int(s): np.asarray(T, dtype=np.float64)
        for s, T in pass1["T_base_cube_by_set_vision_only"].items()
    }

    grasp_pose = load_pose(Path(__file__).resolve().parent / "data" / "grasp_flange_pose.json")
    session_poses = session_path(Path(__file__).resolve().parent / "data", 2)
    items = compute_ordered_targets(session_poses, grasp_pose[3:6], grasp_pose[2])
    assert len(items) == 12

    output = {}
    print(f"{'set':>3} {'taught_xyz_mm':>28} {'vision_xyz_mm':>28} {'xy_diff_mm':>10}")
    for i, item in enumerate(items):
        set_index = i + 1
        T_base_flange = tcp_pose_to_matrix(item["target"])
        T_base_cube_taught = T_base_flange @ T_gripper_cube
        stored_matrix = T_base_cube_taught @ MECHANICAL_MAP
        pose6 = matrix_to_pose6_zyx_deg(stored_matrix)

        # Round-trip sanity: euler_deg_to_matrix(pose6) @ MECHANICAL_MAP must
        # reproduce T_base_cube_taught exactly (this is precisely what
        # table1.py's load_nominal_set_cube_transforms + A3's
        # mechanical_frame_map application will later compute).
        from capture_pipeline.robot import euler_deg_to_matrix
        reconstructed = euler_deg_to_matrix(*pose6) @ MECHANICAL_MAP
        assert np.allclose(reconstructed, T_base_cube_taught, atol=1e-9), (
            f"set {set_index}: encode/decode round-trip mismatch")

        taught_xyz_mm = (T_base_cube_taught[:3, 3] * 1000.0)
        vision_xyz_mm = (vision_only[set_index][:3, 3] * 1000.0) if set_index in vision_only else None
        xy_diff = (
            float(np.linalg.norm(taught_xyz_mm[:2] - vision_xyz_mm[:2]))
            if vision_xyz_mm is not None else float("nan"))
        print(f"{set_index:>3} {np.round(taught_xyz_mm, 1).tolist()!s:>28} "
              f"{(np.round(vision_xyz_mm, 1).tolist() if vision_xyz_mm is not None else None)!s:>28} "
              f"{xy_diff:>10.2f}")

        output[str(set_index)] = {
            "pose6": pose6,
            "matrix4x4": stored_matrix.tolist(),
            "T_base_cube_taught_4x4": T_base_cube_taught.tolist(),
            "cross_check_xy_diff_vs_vision_only_mm": xy_diff,
        }

    Path(args.out).write_text(json.dumps(output, indent=2))
    print(f"\nwrote {args.out}")
    diffs = [v["cross_check_xy_diff_vs_vision_only_mm"] for v in output.values()]
    print(f"cross-check xy diff vs vision-only: mean={np.mean(diffs):.2f}mm "
          f"max={np.max(diffs):.2f}mm (over {len(diffs)} sets)")


if __name__ == "__main__":
    main()
