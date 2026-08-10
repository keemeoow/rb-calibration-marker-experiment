#!/usr/bin/env python3
"""티칭한 grip 자세를 배치 위치들로 옮겨 B 스윕 포즈를 합성한다.

왜 합성해도 되는가. B 웨이포인트는 관절이 아니라 TCP 로 실행되고(서버가
``rb.line`` 으로 +Z 접근 후 하강), 자세는 그대로 두고 위치만 평행이동하는 것은
``build_waypoints_from_pool.py`` 가 per_set_AB 레이아웃에서 이미 하던 일이다.
바꾸는 것은 "어디로 옮기느냐"뿐이다.

왜 배치 위치를 앵커로 쓰는가. 그 XY 는 로봇이 실제로 큐브를 내려놓은 자리이고,
고정 카메라가 그 자리의 큐브를 본다는 것은 A 프레임이 이미 증명한다. 임의의
좌표를 지어내는 것보다 훨씬 안전한 출발점이다. 높이만 티칭된 grip 이 이미 쓴
범위 안에서 들어올린다.

무엇이 검증되고 무엇이 안 되는가.
  검증함  : IK 해 존재, 관절 리미트, manipulability(특이점 근접), 큐브 최소 높이
  검증 못함: 충돌. ``tools/scene_cell.json`` 이 없어 셀 형상을 모른다.
            생성된 포즈는 반드시 저속 dry-run 으로 전 경로를 확인한 뒤 쓸 것.

사용:
  python tools/synth_grip_poses.py \
      --grip data/sessionNN/teaching/grip_poses_001.json \
      --sets data/sessionNN/teaching/capture_sets_001.json \
      --output data/sessionNN/teaching/grip_poses_002.json \
      --heights 110,190 --per_position 1
"""
import argparse
import json
import os
import sys

import numpy as np

_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)
sys.path.insert(0, os.path.join(_REPO_ROOT, "tools"))

from robot_comm import euler_deg_to_matrix
from robot_kin import RobotKinematics, fit_tool_transform

# 큐브 밑면이 테이블에 닿지 않도록 하는 큐브중심 최소 높이. 배치 상태의 중심이
# 43.6 mm 이므로 그보다 확실히 위여야 "들고 있는" 상태다.
MIN_CUBE_CENTER_Z_MM = 80.0
# 특이점 근처를 배제한다. 티칭된 포즈들의 실측 분포에서 하한을 잡는다.
MANIPULABILITY_FLOOR_FRAC = 0.5


def _fit_kinematics(pose_files):
    """티칭 포즈의 (관절, TCP) 쌍으로 flange->tool 을 맞춰 IK 를 프레임 일관되게 만든다."""
    J, T = [], []
    for path, key in pose_files:
        if not os.path.exists(path):
            continue
        for p in json.load(open(path))[key]:
            if isinstance(p.get("capture_joints"), list) and isinstance(p.get("capture_tcp"), list):
                J.append([float(v) for v in p["capture_joints"]])
                T.append(euler_deg_to_matrix(*[float(v) for v in p["capture_tcp"]]))
    if len(J) < 6:
        sys.exit("[ERROR] tool 적합에 쓸 (관절, TCP) 쌍이 부족하다")
    kin = RobotKinematics()
    T_ft, _, stats = fit_tool_transform(kin, np.asarray(J), np.asarray(T), fit_scale=False)
    kin.T_flange_tool = T_ft
    print(f"[INFO] flange->tool 적합: 잔차 pos_rms {stats['pos_rms_mm']:.2f} mm / "
          f"rot_rms {stats['rot_rms_deg']:.2f}°  (n={stats['n']})")
    return kin, np.asarray(J)


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--grip", required=True, help="티칭된 grip 포즈 (recgrip 출력)")
    ap.add_argument("--sets", required=True, help="큐브 배치 (recset 출력)")
    ap.add_argument("--poses", help="A 포즈. 있으면 tool 적합 표본으로 함께 쓴다")
    ap.add_argument("--output", required=True, help="합성 결과 grip 포즈 JSON")
    ap.add_argument("--heights", default="110,190",
                    help="큐브중심을 올릴 높이(mm) 쉼표 구분 (기본 110,190)")
    ap.add_argument("--per_position", type=int, default=1,
                    help="배치 XY 하나 · 높이 하나당 만들 포즈 수 (기본 1)")
    ap.add_argument("--keep_taught", dest="keep_taught", action="store_true", default=True,
                    help="티칭 원본 포즈를 결과 앞쪽에 유지 (기본 유지)")
    ap.add_argument("--no_keep_taught", dest="keep_taught", action="store_false",
                    help="합성 포즈만 남긴다")
    ap.add_argument("--n_orientations", type=int, default=0,
                    help="합성에 쓸 자세 개수. 티칭 풀에서 서로 가장 먼 것들을 고른 뒤 "
                         "위치들에 돌려 쓴다. 자세 하나가 여러 위치에 반복되어야 k "
                         "추정용 pure-translation 쌍이 생기므로, 이 값이 작을수록 "
                         "쌍은 많아지고 자세 다양성은 줄어든다. 0 이면 티칭 자세 전부")
    ap.add_argument("--max_poses", type=int, default=0,
                    help="결과 총 포즈 수 상한 (0 이면 제한 없음)")
    args = ap.parse_args()

    taught = json.load(open(args.grip))["grip_poses"]
    sets = json.load(open(args.sets))["capture_sets"]
    heights = [float(x) for x in args.heights.split(",")]

    kin, taught_joints = _fit_kinematics([
        (args.grip, "grip_poses"),
        (args.poses, "capture_poses") if args.poses else (None, None),
    ] if args.poses else [(args.grip, "grip_poses")])

    # 티칭된 포즈들의 manipulability 로 하한을 정한다: 사람이 실제로 쓴 자세들보다
    # 확연히 나쁜 자세는 만들지 않는다.
    taught_manip = np.array([kin.manipulability(q) for q in taught_joints])
    manip_floor = float(np.median(taught_manip) * MANIPULABILITY_FLOOR_FRAC)
    print(f"[INFO] manipulability 하한 {manip_floor:.4g} "
          f"(티칭 중앙값 {np.median(taught_manip):.4g} 의 {MANIPULABILITY_FLOOR_FRAC:.0%})")

    # 각 티칭 포즈가 들고 있던 큐브중심 -> TCP 오프셋. recgrip 은 그립 상태에서
    # cube_center_6dof 를 기록하므로 이 필드가 곧 그 포즈의 큐브 기준점이다.
    offsets = []
    for p in taught:
        tcp = np.asarray([float(v) for v in p["capture_tcp"][:3]])
        cc = np.asarray([float(v) for v in p["cube_center_6dof"][:3]])
        offsets.append(tcp - cc)

    # 합성에 돌려 쓸 자세 선택. 적게 고를수록 같은 자세가 여러 위치에 반복되어
    # k 추정용 pure-translation 쌍이 늘고, 대신 자세 다양성은 줄어든다. 고를 때는
    # 서로 가장 먼 것부터(farthest-point) 집어 남은 다양성을 최대한 지킨다.
    ori_idx = list(range(len(taught)))
    if 0 < args.n_orientations < len(taught):
        R = [euler_deg_to_matrix(*[float(v) for v in p["capture_tcp"]])[:3, :3]
             for p in taught]

        def rot_deg(i, j):
            M = R[i].T @ R[j]
            return float(np.degrees(np.arccos(
                np.clip((np.trace(M) - 1.0) / 2.0, -1.0, 1.0))))

        ori_idx = [0]
        while len(ori_idx) < args.n_orientations:
            best = max((i for i in range(len(taught)) if i not in ori_idx),
                       key=lambda i: min(rot_deg(i, s) for s in ori_idx))
            ori_idx.append(best)
        gaps = [min(rot_deg(i, j) for j in ori_idx if j != i) for i in ori_idx]
        print(f"[INFO] 합성 자세 {len(ori_idx)}개 선택 (티칭 {len(taught)}개 중), "
              f"서로 최소 상대회전 {min(gaps):.0f}°")

    out, rejected = [], []
    if args.keep_taught:
        for i, p in enumerate(taught):
            q = dict(p)
            q["pose_index"] = len(out)
            q["origin"] = "taught"
            out.append(q)

    n_ori = len(ori_idx)
    made = 0
    stop = False
    for si, s in enumerate(sets):
        if stop:
            break
        cc_xy = [float(v) for v in s["set_cube_center_6dof"][:2]]
        for hi, h in enumerate(heights):
            if h < MIN_CUBE_CENTER_Z_MM:
                sys.exit(f"[ERROR] 높이 {h} 는 최소 큐브중심 높이 "
                         f"{MIN_CUBE_CENTER_Z_MM} 미만이다")
            for k in range(args.per_position):
                if args.max_poses and len(out) >= args.max_poses:
                    stop = True
                    break
                # 자세를 위치에 골고루 흩되, 높이마다 위상을 어긋내 같은 자세가
                # 서로 다른 위치에서 반복되게 한다(=k 쌍이 생기는 지점).
                oi = ori_idx[(si + hi * (n_ori // 2 + 1) + k * 3) % n_ori]
                src = taught[oi]
                target_cc = np.array([cc_xy[0], cc_xy[1], h])
                tcp_xyz = target_cc + offsets[oi]
                rot = [float(v) for v in src["capture_tcp"][3:]]
                tcp6 = [float(tcp_xyz[0]), float(tcp_xyz[1]), float(tcp_xyz[2])] + rot

                T_goal = euler_deg_to_matrix(*tcp6)
                sols = kin.ik_multi(T_goal, n_seeds=24,
                                    extra_seeds=[src["capture_joints"]])
                sols = [q for q in sols if kin.within_limits(q)
                        and kin.manipulability(q) >= manip_floor]
                if not sols:
                    rejected.append((si, h, oi, "IK 해 없음 또는 리미트/특이점"))
                    continue
                # 티칭 자세에서 가장 적게 움직이는 해를 고른다.
                seed = np.asarray(src["capture_joints"], dtype=float)
                q = min(sols, key=lambda x: float(np.max(np.abs(x - seed))))
                out.append({
                    "capture_joints": [float(v) for v in q],
                    "pose_index": len(out),
                    "capture_tcp": tcp6,
                    "cube_center_6dof": [float(target_cc[0]), float(target_cc[1]),
                                         float(target_cc[2])] + rot,
                    "origin": "synthesised",
                    "source": {"set_index": int(s.get("set_index", si)),
                               "grip_pose_index": int(src.get("pose_index", oi)),
                               "height_mm": h},
                })
                made += 1

    print(f"[INFO] 합성 {made}개, 기각 {len(rejected)}개, 총 {len(out)}개")
    for si, h, oi, why in rejected[:10]:
        print(f"  [기각] set {si} h={h:.0f} ori={oi}: {why}")

    payload = {
        "grip_poses": out,
        "provenance": {
            "generator": "tools/synth_grip_poses.py",
            "taught_source": os.path.abspath(args.grip),
            "sets_source": os.path.abspath(args.sets),
            "n_taught": len(taught) if args.keep_taught else 0,
            "n_synthesised": made,
            "n_rejected": len(rejected),
            "heights_mm": heights,
            "method": "keep taught orientation, translate cube centre to each "
                      "placement XY at the given heights; IK screened for joint "
                      "limits and manipulability",
            "screened": ["ik_solution", "joint_limits", "manipulability",
                         "min_cube_centre_height"],
            "NOT_screened": ["cell_collision", "path_collision"],
            "required_before_use": "저속 dry-run 으로 전 경로 확인",
        },
    }
    os.makedirs(os.path.dirname(os.path.abspath(args.output)) or ".", exist_ok=True)
    with open(args.output, "w") as handle:
        json.dump(payload, handle, indent=1)
    print(f"[OK] Wrote {args.output}")
    print("[WARN] 충돌 검사는 하지 않았다. 저속 dry-run 전에는 실촬영에 쓰지 말 것.")


if __name__ == "__main__":
    main()
