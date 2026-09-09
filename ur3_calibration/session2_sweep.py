#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ur3_calibration/session2_sweep.py -- session2 데이터셋의 x,y,rz(yaw)만 이용해 스윕

session2_floor_board_dual_cam/poses.json 은 프리드라이브로 기록한 값이라
z(높이)와 rx,ry(tilt)가 포즈마다 조금씩 흔들려 있다. 이 스크립트는 그
z, rx, ry 를 무시하고 grasp_flange_pose.json 자세의 높이(z)와 기울기로
고정한 뒤, 각 session2 포즈에서는 x, y 위치와 yaw만 뽑아 15개의 새 목표
자세를 만든다.

주의 -- 로봇의 rx,ry,rz는 오일러각이 아니라 rotation vector(axis-angle)라서
"rz만 바꾸고 rx,ry는 그대로 둔다"를 성분 그대로 하면 안 된다(물리적으로
다른 회전이 되어버린다). 그렇다고 Euler ZYX로 3분해했다가 재조합하는 것도
위험하다 -- tilt(pitch)가 0/180 근처(지금 grasp 자세가 정확히 이 경우)면
yaw와 roll이 뒤섞이는 짐벌락이 생기고, session2 poses.json의 rx,ry가
포즈마다 크게 요동치는 것도 이 현상 때문이다. 대신 회전행렬에서 "월드
Z축 기준 상대 yaw"만 직접 뽑아(raw_yaw, 짐벌락 없음) grasp 자세를 그
델타만큼 Z축으로 돌리는 방식으로 목표 자세를 합성한다(tilt는 항상 grasp
자세 그대로 유지됨).

이동 순서는 기록된 순서가 아니라 delta_yaw 오름차순으로 재정렬한다 --
같은 15개 자세를 방문하되, 인접 자세 간 회전 변화를 최소화해서 moveL의
rotation-vector 선형보간이 큰 원호를 그리며 휘도는 것을 막기 위함이다.

*** 안전 경고 ***
그래도 delta_yaw가 반대쪽으로 넘어가는 지점에서는 rotvec 축이 뒤집히며
여전히 큰 변화가 남을 수 있다([WARN]으로 표시됨). 반드시:
  1) --dry-run(기본값)으로 먼저 계산된 목표/경고를 확인하고
  2) 로봇 옆에서 비상정지 버튼을 잡을 수 있는 상태로
  3) 기본 동작은 한 포즈씩 Enter로 진행(--no-step 으로 끄기 전까지)
하도록 만들었다. 그리퍼는 건드리지 않는다(로봇 이동만 테스트).

사용법:
  python session2_sweep.py                      # dry-run: 목표 계산/출력만
  python session2_sweep.py --execute             # 실제 이동, 한 포즈씩 Enter로 확인
  python session2_sweep.py --execute --no-step   # (검증 후) 연속 실행
"""

import argparse
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

import rtde_control

from capture_poses import session_path

ROBOT_IP_DEFAULT = "192.168.1.101"
GRASP_POSE_DEFAULT = Path(__file__).resolve().parent / "data" / "grasp_flange_pose.json"
SESSION_ID = 2

MOVE_SPEED_DEFAULT = 0.05   # m/s -- 회전 보간 위험 때문에 기본값을 낮게 잡음
MOVE_ACCEL_DEFAULT = 0.20   # m/s^2


def raw_yaw(rotvec):
    """회전행렬에서 월드 Z축 기준 yaw만 직접 뽑는다 (짐벌락 없음).

    R = Rz(yaw) @ R_tilt 로 두면 R[0,0]+i*R[1,0] = (A+iB)*e^{i*yaw}
    (A,B는 R_tilt에만 의존하는 상수)이므로, atan2(R[1,0],R[0,0])는
    R_tilt가 무엇이든 yaw + const(R_tilt) 를 안정적으로 돌려준다.
    두 자세의 R_tilt가 같다면(여기서는 둘 다 grasp 기준으로 맞출 것이므로)
    이 값들의 차이가 곧 그 둘 사이의 상대 yaw다.
    """
    R = Rotation.from_rotvec(rotvec).as_matrix()
    return float(np.arctan2(R[1, 0], R[0, 0]))


def wrap_pi(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi


def build_target(x, y, z_fixed, delta_yaw, grasp_rotvec):
    """grasp 자세를 월드 Z축으로 delta_yaw만큼 돌린 자세 (tilt는 grasp 그대로)."""
    rz = Rotation.from_euler("z", delta_yaw)
    r_grasp = Rotation.from_rotvec(grasp_rotvec)
    rotvec = (rz * r_grasp).as_rotvec()
    return [x, y, z_fixed, float(rotvec[0]), float(rotvec[1]), float(rotvec[2])]


def fmt_pose(pose):
    return (
        f"xyz=({pose[0] * 1000:.1f},{pose[1] * 1000:.1f},{pose[2] * 1000:.1f})mm  "
        f"rotvec=({pose[3]:.3f},{pose[4]:.3f},{pose[5]:.3f})"
    )


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--robot-ip", default=ROBOT_IP_DEFAULT)
    ap.add_argument("--grasp-pose", default=str(GRASP_POSE_DEFAULT))
    ap.add_argument(
        "--session-poses",
        default=str(session_path(Path(__file__).resolve().parent / "data", SESSION_ID)),
    )
    ap.add_argument("--speed", type=float, default=MOVE_SPEED_DEFAULT)
    ap.add_argument("--accel", type=float, default=MOVE_ACCEL_DEFAULT)
    ap.add_argument("--execute", action="store_true", help="실제로 이동 (없으면 dry-run)")
    ap.add_argument("--no-step", action="store_true", help="한 포즈씩 Enter로 확인하지 않고 연속 실행")
    args = ap.parse_args()

    grasp = json.loads(Path(args.grasp_pose).read_text())["tcp_pose"]
    z_fixed = grasp[2]
    grasp_rotvec = grasp[3:6]
    yaw_grasp = raw_yaw(grasp_rotvec)

    session = json.loads(Path(args.session_poses).read_text())
    poses = session["poses"]

    items = []
    for orig_idx, p in enumerate(poses):
        x, y, _z, rx, ry, rz = p["tcp_pose"]
        yaw_i = raw_yaw([rx, ry, rz])
        delta_yaw = wrap_pi(yaw_i - yaw_grasp)
        target = build_target(x, y, z_fixed, delta_yaw, grasp_rotvec)
        items.append({"orig_index": orig_idx, "delta_yaw": delta_yaw, "target": target})

    # 인접 자세 간 회전 변화를 줄이기 위해 delta_yaw 오름차순으로 방문 순서 재정렬.
    items.sort(key=lambda it: it["delta_yaw"])
    targets = [it["target"] for it in items]

    print(f"고정 z={z_fixed * 1000:.1f}mm  grasp rotvec={tuple(round(v, 3) for v in grasp_rotvec)}\n")
    for order, it in enumerate(items):
        print(f"[{order + 1:2d}/{len(items)}] (원본 #{it['orig_index'] + 1:2d}, "
              f"delta_yaw={np.degrees(it['delta_yaw']):6.1f}deg) {fmt_pose(it['target'])}")

    for i in range(1, len(targets)):
        d = np.array(targets[i][3:6]) - np.array(targets[i - 1][3:6])
        if np.linalg.norm(d) > 1.0:
            print(f"  [WARN] {i}->{i + 1} 구간 rotvec 변화가 큽니다 "
                  f"(|delta|={np.linalg.norm(d):.2f}) -- moveL 중 TCP가 크게 휘돌 수 있어 특히 주의.")

    if not args.execute:
        print("\n(dry-run) 실제로 움직이지 않았습니다. --execute 를 주면 이동합니다.")
        return

    rtde_c = rtde_control.RTDEControlInterface(args.robot_ip)
    try:
        for i, t in enumerate(targets):
            if not args.no_step:
                cmd = input(
                    f"\n[{i + 1}/{len(targets)}] 다음 목표로 이동: {fmt_pose(t)}\n"
                    f"Enter=이동 / q=중단 > "
                ).strip().lower()
                if cmd == "q":
                    print("중단했습니다.")
                    break
            else:
                print(f"이동 {i + 1}/{len(targets)} -> {fmt_pose(t)}")
            rtde_c.moveL(t, args.speed, args.accel)
    finally:
        rtde_c.stopScript()
        rtde_c.disconnect()


if __name__ == "__main__":
    main()
