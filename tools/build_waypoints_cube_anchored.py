#!/usr/bin/env python3
"""Waypoints anchored to each cube placement, so teaching does not scale with placements.

Teaching cost with this generator is O(sets + poses) instead of O(sets x poses):

    per session : one B grip-sweep pool (recgrip), taught once
    per placement: recset only - one command where the cube is being put down anyway
    phase A      : nothing taught at all; viewpoints are solved for each placement

Two things this does that ``build_waypoints_from_pool.py`` does not:

A_placement - solved, not reused. The existing A pool is a set of generic
    workspace viewpoints: their camera axes converge on the workspace centre
    (median miss 27 mm), not on any one cube, so each placement's cube lands
    40-104 mm off-axis. Here each viewpoint is solved so the *camera* axis hits
    that placement's cube centre, over an explicit distance/elevation/azimuth
    grid, then run through IK seeded from a taught pose.

B_eyetohand - re-anchored per pose, not against one global reference set.
    ``recgrip`` records ``cube_center_6dof`` while the cube is gripped, so for B
    that field really is the cube centre and each taught pose carries its own
    reference. ``--b_ref_set`` collapses all of them onto one set's centre.

The camera axis is taken from the measured ``T_gripper_cam``, NOT assumed to be
the tool z-axis. On this rig the camera looks along the tool's -y axis, so
aiming tool z at the cube (what ``generate_capture_waypoints.py`` does) points
the camera roughly 90 degrees away from the target.

Every generated pose is screened before it can reach the robot: joint limits,
manipulability, cell collision at the pose, and collision along the joint-space
straight line from the safe pose - the path ``rb.move()`` actually drives.

    python tools/build_waypoints_cube_anchored.py \
        --grip data/session/grip_poses_001.json \
        --sets data/session/capture_sets_001.json \
        --gripper_cam data/session/calib_out/T_gripper_cam.npy \
        --scene tools/scene_cell.json \
        --safe_joints_empty d1,d2,d3,d4,d5,d6 \
        --safe_joints_gripped d1,d2,d3,d4,d5,d6 \
        --output data/recapture_20260806/session_00/capture_waypoints.json

Output schema is unchanged, so server/robot_calb.py consumes it as-is.
"""

from __future__ import annotations

import argparse
import json
import os
import random
import sys
from typing import Dict, List, Optional, Sequence

import numpy as np

_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)

from robot_comm import euler_deg_to_matrix                       # noqa: E402
from waypoint_safety import (                                    # noqa: E402
    validate_joint_vector,
    validate_safe_joint_config,
    validate_waypoint_semantics,
)
from tools.cell_scene import load_scene                          # noqa: E402
from tools.robot_kin import RobotKinematics, fit_tool_transform  # noqa: E402


def matrix_to_euler_deg(T: np.ndarray) -> List[float]:
    """Inverse of robot_comm.euler_deg_to_matrix: [x,y,z,rz,ry,rx] in mm/deg."""
    R = T[:3, :3]
    sy = float(np.hypot(R[0, 0], R[1, 0]))
    if sy < 1e-9:  # gimbal lock: pitch at +-90, fold roll into yaw
        rz = float(np.arctan2(-R[0, 1], R[1, 1]))
        ry = float(np.arctan2(-R[2, 0], sy))
        rx = 0.0
    else:
        rz = float(np.arctan2(R[1, 0], R[0, 0]))
        ry = float(np.arctan2(-R[2, 0], sy))
        rx = float(np.arctan2(R[2, 1], R[2, 2]))
    return [round(float(T[0, 3]) * 1000.0, 3),
            round(float(T[1, 3]) * 1000.0, 3),
            round(float(T[2, 3]) * 1000.0, 3),
            round(np.rad2deg(rz), 3), round(np.rad2deg(ry), 3), round(np.rad2deg(rx), 3)]


def _load(path: str, key: str) -> List[dict]:
    if not os.path.exists(path):
        sys.exit(f"[ERROR] 파일 없음: {path}")
    with open(path) as f:
        data = json.load(f)
    items = data.get(key)
    if not isinstance(items, list) or not items:
        sys.exit(f"[ERROR] {path} 에 '{key}' 리스트가 비어있음")
    return items


def _joint_csv(value: str):
    try:
        return validate_joint_vector([float(x.strip()) for x in value.split(",")],
                                     "safe joints")
    except (TypeError, ValueError) as exc:
        raise argparse.ArgumentTypeError(str(exc))


def _csv_floats(value: str) -> List[float]:
    return [float(x.strip()) for x in value.split(",") if x.strip()]


def camera_lookat_tool_pose(cube_xyz_m: np.ndarray, T_gripper_cam: np.ndarray,
                            distance_m: float, elevation_deg: float,
                            azimuth_deg: float, roll_deg: float) -> np.ndarray:
    """Tool pose placing the *camera* at the given spherical offset, aimed at the cube.

    Solved for the camera and then converted back through the measured
    gripper->camera extrinsic. Aiming the tool frame instead would miss by
    whatever that extrinsic's rotation is - about 90 degrees on this rig.
    """
    az, el = np.deg2rad(azimuth_deg), np.deg2rad(elevation_deg)
    offset = np.array([np.cos(el) * np.cos(az), np.cos(el) * np.sin(az), np.sin(el)])
    p_cam = cube_xyz_m + distance_m * offset

    z_axis = cube_xyz_m - p_cam                      # camera looks down its +z
    z_axis /= np.linalg.norm(z_axis)
    # World -z as the up reference keeps the image roughly table-aligned; it
    # degenerates only when looking straight down, hence the fallback.
    up = np.array([0.0, 0.0, -1.0])
    if abs(float(np.dot(up, z_axis))) > 0.98:
        up = np.array([0.0, 1.0, 0.0])
    x_axis = np.cross(up, z_axis)
    x_axis /= np.linalg.norm(x_axis)
    y_axis = np.cross(z_axis, x_axis)

    R_cam = np.column_stack([x_axis, y_axis, z_axis])
    if abs(roll_deg) > 1e-9:                          # spin about the optical axis
        c, s = np.cos(np.deg2rad(roll_deg)), np.sin(np.deg2rad(roll_deg))
        R_cam = R_cam @ np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])

    T_base_cam = np.eye(4)
    T_base_cam[:3, :3], T_base_cam[:3, 3] = R_cam, p_cam
    return T_base_cam @ np.linalg.inv(T_gripper_cam)


class PoseScreener:
    """Joint limits -> manipulability -> pose collision -> path collision."""

    def __init__(self, kin: RobotKinematics, scene, safe_joints: Sequence[float],
                 min_manipulability: float, max_seed_dev_deg: float,
                 tol_pos_mm: float, tol_rot_deg: float):
        self.kin = kin
        self.scene = scene
        self.safe = np.asarray(safe_joints, dtype=float)
        self.min_manip = min_manipulability
        self.max_seed_dev = max_seed_dev_deg
        self.tol_pos_mm = tol_pos_mm
        self.tol_rot_deg = tol_rot_deg
        self.rejects: Dict[str, int] = {}

    def _reject(self, reason: str) -> None:
        self.rejects[reason] = self.rejects.get(reason, 0) + 1

    def solve(self, T_goal: np.ndarray, seeds: Sequence[Sequence[float]]) -> Optional[np.ndarray]:
        """Best screened IK solution for T_goal, or None with the reason counted."""
        best, best_clear = None, -np.inf
        saw_ik = saw_seed = saw_limits = saw_manip = saw_pose = False

        for seed in seeds:
            q = self.kin.ik(T_goal, seed)
            if q is None:
                continue
            saw_ik = True

            # Stay on the arm branch a human actually drove. A far-away IK
            # branch can be geometrically valid and still swing the elbow
            # through somewhere nobody has ever watched it go.
            if float(np.max(np.abs(q - np.asarray(seed, dtype=float)))) > self.max_seed_dev:
                continue
            saw_seed = True

            if not self.kin.within_limits(q):
                continue
            saw_limits = True

            err = self.kin._pose_error(self.kin.fk_tool(q), T_goal)
            if (np.linalg.norm(err[:3]) * 1000.0 > self.tol_pos_mm
                    or np.rad2deg(np.linalg.norm(err[3:])) > self.tol_rot_deg):
                continue

            if self.kin.manipulability(q) < self.min_manip:
                continue
            saw_manip = True

            res = self.scene.check(q)
            if not res["ok"]:
                continue
            saw_pose = True

            path = self.scene.check_path(self.safe, q)
            if not path["ok"]:
                continue

            # Among survivors prefer the roomiest, not merely the first found.
            clear = min(res["clearance_m"], path["min_clearance_m"])
            if clear > best_clear:
                best, best_clear = q, clear

        if best is not None:
            return best
        for flag, reason in ((saw_pose, "path_collision"), (saw_manip, "pose_collision"),
                             (saw_limits, "singular"), (saw_seed, "joint_limits"),
                             (saw_ik, "off_seed_branch"), (True, "ik_failed")):
            if flag:
                self._reject(reason)
                break
        return None


def farthest_point_select(candidates: List[dict], n: int, keys: Sequence[str],
                          angular_keys: Sequence[str] = ()) -> List[dict]:
    """Greedy max-spread subset, so a truncated list stays diverse.

    Taking the first n of a grid would silently collapse the azimuth or
    elevation range - exactly the diversity the calibration needs.

    Angular keys are embedded on the unit circle first. Treating them as plain
    numbers makes 0 and 315 degrees look 315 apart when they are 45 apart, which
    is enough to make the selection pile up on one side of the cube.
    """
    if len(candidates) <= n:
        return list(candidates)
    angular = set(angular_keys)
    cols = []
    for k in keys:
        v = np.array([float(c[k]) for c in candidates], dtype=float)
        if k in angular:
            cols.append(np.cos(np.deg2rad(v)))
            cols.append(np.sin(np.deg2rad(v)))
        else:
            cols.append(v)
    feats = np.column_stack(cols)
    span = np.ptp(feats, axis=0)
    span[span < 1e-9] = 1.0
    feats = feats / span

    chosen = [int(np.argmax(np.linalg.norm(feats - feats.mean(axis=0), axis=1)))]
    while len(chosen) < n:
        d = np.min(np.linalg.norm(feats[:, None, :] - feats[None, chosen, :], axis=2), axis=1)
        d[chosen] = -np.inf
        chosen.append(int(np.argmax(d)))
    return [candidates[i] for i in chosen]


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--grip", default="./grip_poses.json", help="B 그립-스윕 풀 (recgrip)")
    ap.add_argument("--sets", default="./capture_sets.json", help="큐브 배치 (recset)")
    ap.add_argument("--poses", default=None,
                    help="A 풀 (recpose). IK seed 와 --a_mode reuse 에만 사용")
    ap.add_argument("--gripper_cam", default="data/session/calib_out/T_gripper_cam.npy",
                    help="측정된 gripper->camera 외부파라미터 (A 조준에 필수)")
    ap.add_argument("--scene", default="tools/scene_cell.json", help="셀 충돌 모델")
    ap.add_argument("--output", default="./data/session/capture_waypoints.json")

    ap.add_argument("--safe_joints_empty", type=_joint_csv, required=True)
    ap.add_argument("--safe_joints_gripped", type=_joint_csv, required=True)

    ap.add_argument("--a_mode", choices=["lookat", "reuse"], default="lookat",
                    help="lookat: 배치마다 큐브 정조준 뷰포인트를 푼다(기본). "
                         "reuse: 기존 A 풀을 그대로 쓴다(현행 동작)")
    ap.add_argument("--n_per_set", type=int, default=5)
    ap.add_argument("--n_grip_per_set", type=int, default=10)

    ap.add_argument("--distances_mm", type=_csv_floats, default=[350.0, 450.0],
                    help="큐브 중심에서 카메라까지 거리 후보")
    ap.add_argument("--elevations_deg", type=_csv_floats, default=[35.0, 55.0, 75.0])
    ap.add_argument("--azimuths_deg", type=_csv_floats,
                    default=[0.0, 45.0, 90.0, 135.0, 180.0, 225.0, 270.0, 315.0])
    ap.add_argument("--rolls_deg", type=_csv_floats, default=[0.0, 90.0],
                    help="광축 중심 회전 — eye-in-hand 회전 다양성")

    ap.add_argument("--min_manipulability", type=float, default=0.01)
    ap.add_argument("--max_seed_dev_deg", type=float, default=45.0,
                    help="티칭된 seed 자세에서 관절이 이만큼 넘게 벗어나면 기각")
    ap.add_argument("--ik_tol_pos_mm", type=float, default=1.0)
    ap.add_argument("--ik_tol_rot_deg", type=float, default=0.5)
    ap.add_argument("--b_move", choices=["tcp", "joint"], default="tcp",
                    help="tcp: 현행대로 line 이동(경로는 충돌검사 불가). "
                         "joint: IK 로 풀어 관절 이동(경로까지 검사)")

    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--allow_short_sets", action="store_true",
                    help="일부 set 이 목표 수에 못 미쳐도 기록 (paired 설계가 깨짐)")
    ap.add_argument("--preview", action="store_true", help="파일을 쓰지 않고 요약만")
    args = ap.parse_args()

    if args.n_per_set <= 0 or args.n_grip_per_set <= 0:
        sys.exit("[ERROR] paired 실험은 모든 set 에 A/B 가 모두 필요 (둘 다 1 이상)")

    grip = _load(args.grip, "grip_poses")
    sets = _load(args.sets, "capture_sets")
    poses = _load(args.poses, "capture_poses") if args.poses else []

    for i, p in enumerate(grip):
        for key in ("capture_tcp", "capture_joints", "cube_center_6dof"):
            if not isinstance(p.get(key), list) or len(p[key]) != 6:
                sys.exit(f"[ERROR] {args.grip} grip_poses[{i}].{key} 가 6-벡터가 아님")
    for i, s in enumerate(sets):
        for key in ("place_joints", "set_cube_center_6dof"):
            if not isinstance(s.get(key), list) or len(s[key]) != 6:
                sys.exit(f"[ERROR] {args.sets} capture_sets[{i}].{key} 가 6-벡터가 아님")

    # --- kinematics: fit flange->tool against every taught (joints, tcp) pair ---
    kin = RobotKinematics()
    taught = [p for p in (list(poses) + list(grip))
              if isinstance(p.get("capture_joints"), list) and isinstance(p.get("capture_tcp"), list)]
    if len(taught) < 6:
        sys.exit(f"[ERROR] flange->tool 피팅에 (joints,tcp) 쌍 6개 이상 필요, {len(taught)}개뿐")
    _, scale, stats = fit_tool_transform(
        kin,
        np.array([p["capture_joints"] for p in taught], dtype=float),
        np.array([euler_deg_to_matrix(*p["capture_tcp"]) for p in taught]))
    print(f"[kin] flange->tool fit over {stats['n']} taught poses: "
          f"{stats['pos_rms_mm']:.2f} mm rms / {stats['rot_rms_deg']:.3f} deg rms")
    if stats["pos_rms_mm"] > 5.0:
        # IK is only as good as this fit. A bad fit means generated joint
        # targets do not land where the geometry says, so stop here.
        sys.exit(f"[ERROR] flange->tool 잔차 {stats['pos_rms_mm']:.2f} mm 가 너무 큼. "
                 f"DH 테이블/툴 정의가 이 로봇과 안 맞으므로 IK 결과를 신뢰할 수 없음.")

    scene = load_scene(args.scene, kin=kin)
    seeds_empty = [p["capture_joints"] for p in poses] or [list(args.safe_joints_empty)]

    screener_a = PoseScreener(kin, scene, args.safe_joints_empty,
                              args.min_manipulability, args.max_seed_dev_deg,
                              args.ik_tol_pos_mm, args.ik_tol_rot_deg)
    screener_b = PoseScreener(kin, scene, args.safe_joints_gripped,
                              args.min_manipulability, args.max_seed_dev_deg,
                              args.ik_tol_pos_mm, args.ik_tol_rot_deg)

    T_gripper_cam = None
    if args.a_mode == "lookat":
        if not os.path.exists(args.gripper_cam):
            sys.exit(f"[ERROR] --gripper_cam 없음: {args.gripper_cam}\n"
                     f"        lookat 은 측정된 카메라 외부파라미터가 있어야 조준할 수 있음 "
                     f"(--a_mode reuse 로 우회 가능)")
        T_gripper_cam = np.load(args.gripper_cam)
        cam_axis = T_gripper_cam[:3, 2]
        print(f"[cam] gripper->camera z-axis in tool frame = {np.round(cam_axis, 3).tolist()}")
        if float(abs(cam_axis[2])) > 0.9:
            print("      (tool z 와 거의 일치)")
        else:
            print("      tool z 와 어긋나 있으므로 tool 이 아니라 카메라를 조준함")

    rng = random.Random(None if args.seed < 0 else args.seed)
    grid = [(d, e, a, r)
            for d in args.distances_mm for e in args.elevations_deg
            for a in args.azimuths_deg for r in args.rolls_deg]
    print(f"[plan] sets={len(sets)}  A per set={args.n_per_set} "
          f"(from a {len(grid)}-pose grid)  B per set={args.n_grip_per_set} "
          f"(from {len(grip)} taught)\n")

    waypoints: List[dict] = []
    capture_index = 0
    short_sets: List[str] = []

    for si, s in enumerate(sets):
        set_index = int(s.get("set_index", si))
        place_joints = [float(x) for x in s["place_joints"]]
        set_cc = [float(x) for x in s["set_cube_center_6dof"]]
        cube_xyz_m = np.array(set_cc[:3], dtype=float) / 1000.0

        # --- Phase B: re-anchor each taught sweep pose by its own recorded cube centre ---
        b_solved = []
        for p in grip:
            tcp = [float(x) for x in p["capture_tcp"]]
            ref = [float(x) for x in p["cube_center_6dof"]]
            b_tcp = [round(set_cc[i] + (tcp[i] - ref[i]), 3) for i in range(3)] + tcp[3:]
            entry = {"capture_tcp": b_tcp, "pose_index": p.get("pose_index")}
            if args.b_move == "joint":
                q = screener_b.solve(euler_deg_to_matrix(*b_tcp), [p["capture_joints"]])
                if q is None:
                    continue
                entry["capture_joints"] = [round(float(v), 4) for v in q]
            b_solved.append(entry)

        b_sel = (rng.sample(b_solved, args.n_grip_per_set)
                 if len(b_solved) >= args.n_grip_per_set else list(b_solved))

        # --- Phase A: solve a camera-aimed viewpoint per grid cell, then screen ---
        a_solved = []
        if args.a_mode == "lookat":
            for (dist_mm, el, az, roll) in grid:
                T_goal = camera_lookat_tool_pose(cube_xyz_m, T_gripper_cam,
                                                 dist_mm / 1000.0, el, az, roll)
                q = screener_a.solve(T_goal, seeds_empty)
                if q is None:
                    continue
                a_solved.append({
                    "capture_joints": [round(float(v), 4) for v in q],
                    "capture_tcp": matrix_to_euler_deg(T_goal),
                    "distance_mm": dist_mm, "elevation_deg": el,
                    "azimuth_deg": az, "roll_deg": roll,
                })
            a_sel = farthest_point_select(
                a_solved, args.n_per_set,
                ["azimuth_deg", "elevation_deg", "distance_mm", "roll_deg"],
                angular_keys=["azimuth_deg", "roll_deg"])
        else:
            for p in poses:
                entry = {"capture_joints": [float(x) for x in p["capture_joints"]],
                         "pose_index": p.get("pose_index")}
                if isinstance(p.get("capture_tcp"), list):
                    entry["capture_tcp"] = [float(x) for x in p["capture_tcp"]]
                a_solved.append(entry)
            a_sel = (rng.sample(a_solved, args.n_per_set)
                     if len(a_solved) >= args.n_per_set else list(a_solved))

        mark = ""
        if len(a_sel) < args.n_per_set or len(b_sel) < args.n_grip_per_set:
            short_sets.append(f"set {set_index}: A {len(a_sel)}/{args.n_per_set}, "
                              f"B {len(b_sel)}/{args.n_grip_per_set}")
            mark = "   <-- SHORT"
        print(f"  set {set_index:2d}: A {len(a_sel)}/{args.n_per_set} "
              f"(solved {len(a_solved)}/{len(grid) if args.a_mode == 'lookat' else len(poses)})  "
              f"B {len(b_sel)}/{args.n_grip_per_set} (solved {len(b_solved)}/{len(grip)}){mark}")

        for entry in b_sel:
            wp = {"capture_index": capture_index, "set_index": set_index,
                  "place_joints": place_joints, "set_cube_center_6dof": set_cc,
                  "capture_block": "B_eyetohand", "cube_gripped": True}
            wp.update(entry)
            waypoints.append(wp)
            capture_index += 1

        for entry in a_sel:
            wp = {"capture_index": capture_index, "set_index": set_index,
                  "place_joints": place_joints, "set_cube_center_6dof": set_cc,
                  "capture_block": "A_placement", "cube_gripped": False}
            wp.update(entry)
            waypoints.append(wp)
            capture_index += 1

    for name, sc in (("A", screener_a), ("B", screener_b)):
        if sc.rejects:
            order = sorted(sc.rejects.items(), key=lambda kv: -kv[1])
            print(f"\n[{name}] rejected: " + ", ".join(f"{k}={v}" for k, v in order))

    n_b = sum(1 for w in waypoints if w["capture_block"] == "B_eyetohand")
    n_a = sum(1 for w in waypoints if w["capture_block"] == "A_placement")
    print(f"\n[INFO] 총 waypoints: {len(waypoints)}  (B: {n_b}, A: {n_a})")

    if short_sets:
        print("\n[WARN] 목표 수에 못 미친 set:")
        for line in short_sets:
            print(f"        {line}")
        if not args.allow_short_sets:
            # An unbalanced A/B design across sets quietly breaks the paired
            # comparison downstream, so refuse to write it by default.
            sys.exit("[ERROR] set 별 A/B 개수가 불균형하면 paired 설계가 깨짐. "
                     "그리드를 넓히거나(--distances_mm/--elevations_deg) "
                     "--n_per_set 을 낮추거나 --allow_short_sets 로 승인할 것.")

    if args.b_move == "tcp":
        print("\n[NOTE] B 는 TCP line 이동이라 경로 충돌검사가 적용되지 않았습니다 "
              "(끝점만 검사). 경로까지 검사하려면 --b_move joint.")

    out = {
        "safe_joints_empty": args.safe_joints_empty,
        "safe_joints_gripped": args.safe_joints_gripped,
        "set_joints": [float(x) for x in sets[0]["place_joints"]],
        "set_tcp": ([float(x) for x in sets[0]["place_tcp"]]
                    if isinstance(sets[0].get("place_tcp"), list) else None),
        "set_cube_center": [float(x) for x in sets[0]["set_cube_center_6dof"]],
        "waypoints": waypoints,
        "_meta": {
            "generator": "tools/build_waypoints_cube_anchored.py",
            "a_mode": args.a_mode, "b_move": args.b_move,
            "n_sets": len(sets), "n_per_set_A": args.n_per_set,
            "n_grip_per_set_B": args.n_grip_per_set,
            "grid": {"distances_mm": args.distances_mm,
                     "elevations_deg": args.elevations_deg,
                     "azimuths_deg": args.azimuths_deg,
                     "rolls_deg": args.rolls_deg},
            "gripper_cam": args.gripper_cam if args.a_mode == "lookat" else None,
            "scene": args.scene,
            "flange_tool_fit": {"pos_rms_mm": round(stats["pos_rms_mm"], 3),
                                "rot_rms_deg": round(stats["rot_rms_deg"], 4),
                                "n": stats["n"]},
            "screen": {"min_manipulability": args.min_manipulability,
                       "max_seed_dev_deg": args.max_seed_dev_deg},
            "seed": None if args.seed < 0 else args.seed,
            "total_captures": len(waypoints),
            "total_B_eyetohand": n_b, "total_A_placement": n_a,
        },
    }
    validate_safe_joint_config(out)
    validate_waypoint_semantics(out)

    if args.preview:
        print("\n[PREVIEW] 파일 미기록.")
        return 0

    out_dir = os.path.dirname(os.path.abspath(args.output))
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)
    with open(args.output, "w") as f:
        json.dump(out, f, indent=2)
    print(f"[OK] Wrote {args.output}")
    print("     -> 서버 'start' 시 PC 가 전송. set 마다 B(그립스윕) -> 큐브내림 -> A(placement).")
    return 0


if __name__ == "__main__":
    sys.exit(main())
