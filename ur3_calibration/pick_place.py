#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ur3_calibration/pick_place.py — 바닥에 고정된 큐브를 집고/내려놓기

save_grasp_flange_pose.py 로 저장해둔 grasp_flange_pose.json(그리퍼가 큐브
위치까지 내려간 순간의 flange 자세 — 큐브의 3D 좌표가 아니라 그때의 로봇
flange 자세/조인트 값)를 목표(grasp pose)로 삼아:

  pick  : (그리퍼 열기) -> approach 높이로 이동(xy/방향 정렬) -> 수직 하강
          -> 그리퍼 닫기 -> 수직 상승(approach 높이로 복귀)
  place : approach 높이로 이동 -> 수직 하강 -> 그리퍼 열기 -> 수직 상승

approach 높이에서 먼저 xy/방향을 맞추고 마지막 구간은 순수 수직 이동만
하도록 두 단계로 나눈다 (대각선 하강 금지 — 이 프로젝트의 Zeus
grasp_target.py 와 동일한 규칙).

큐브 위치가 하나로 고정되어 있으므로 pick/place 모두 같은
grasp_flange_pose.json을 목표로 사용한다. 안전을 위해 --execute 를 주지 않으면 실제로 움직이지
않고 계획(pose 값)만 출력하는 dry-run으로 동작한다.

사용법:
  python pick_place.py pick  --dry-run          # 계획만 확인 (기본값)
  python pick_place.py pick  --execute          # 실제 실행 (재확인 프롬프트 있음)
  python pick_place.py place --execute --yes    # 재확인 없이 실행 (스크립트용)
"""

import argparse
import json
import time
from pathlib import Path

from gripper import RobotiqGripper

import rtde_control
import rtde_receive

ROBOT_IP_DEFAULT = "192.168.1.101"
GRASP_POSE_DEFAULT = Path(__file__).resolve().parent / "data" / "grasp_flange_pose.json"

MOVE_SPEED = 0.10       # m/s, approach 이동
MOVE_ACCEL = 0.30       # m/s^2
DESCEND_SPEED = 0.03    # m/s, 수직 하강/상승 (더 느리게)
DESCEND_ACCEL = 0.20    # m/s^2
GRIP_SETTLE_S = 1.0


def load_grasp_pose(path: Path) -> list:
    data = json.loads(path.read_text())
    return list(data["tcp_pose"])


def approach_pose(grasp_pose: list, offset_mm: float) -> list:
    pose = list(grasp_pose)
    pose[2] += offset_mm / 1000.0
    return pose


def fmt_pose(pose: list) -> str:
    return (
        f"xyz=({pose[0] * 1000:.1f},{pose[1] * 1000:.1f},{pose[2] * 1000:.1f})mm  "
        f"rotvec=({pose[3]:.3f},{pose[4]:.3f},{pose[5]:.3f})rad"
    )


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("action", choices=["pick", "place"])
    ap.add_argument("--robot-ip", default=ROBOT_IP_DEFAULT)
    ap.add_argument("--grasp-pose", default=str(GRASP_POSE_DEFAULT))
    ap.add_argument("--approach-mm", type=float, default=80.0, help="grasp pose 위 approach 높이 (기본 80mm)")
    ap.add_argument("--open-pos", type=int, default=0, help="그리퍼 열림 값 0-255 (기본 0=완전개방)")
    ap.add_argument("--close-pos", type=int, default=255, help="그리퍼 닫힘 값 0-255 (큐브 두께에 맞춰 조정)")
    ap.add_argument("--execute", action="store_true", help="실제로 로봇을 움직인다 (없으면 dry-run)")
    ap.add_argument("--yes", action="store_true", help="--execute 시 재확인 프롬프트 생략")
    args = ap.parse_args()

    grasp = load_grasp_pose(Path(args.grasp_pose))
    approach = approach_pose(grasp, args.approach_mm)

    print(f"=== {args.action} ===")
    print(f"grasp pose   : {fmt_pose(grasp)}")
    print(f"approach pose: {fmt_pose(approach)}  (+{args.approach_mm:.0f}mm)")
    print(f"gripper open={args.open_pos} close={args.close_pos}")

    if not args.execute:
        print("\n(dry-run) 실제로 움직이지 않았습니다. --execute 를 주면 실행합니다.")
        return

    if not args.yes:
        if input("\n실제로 로봇을 움직입니다. 계속하려면 'go' 입력: ").strip().lower() != "go":
            print("취소했습니다.")
            return

    rtde_c = rtde_control.RTDEControlInterface(args.robot_ip)
    rtde_r = rtde_receive.RTDEReceiveInterface(args.robot_ip)
    gripper = RobotiqGripper(args.robot_ip)
    try:
        if args.action == "pick":
            print("그리퍼 열기...")
            gripper.set_pos(args.open_pos)
            time.sleep(GRIP_SETTLE_S)
            print("approach 높이로 이동...")
            rtde_c.moveL(approach, MOVE_SPEED, MOVE_ACCEL)
            print("수직 하강...")
            rtde_c.moveL(grasp, DESCEND_SPEED, DESCEND_ACCEL)
            print("그리퍼 닫기...")
            gripper.set_pos(args.close_pos)
            time.sleep(GRIP_SETTLE_S)
            print("수직 상승 (approach 높이로 복귀)...")
            rtde_c.moveL(approach, DESCEND_SPEED, DESCEND_ACCEL)
        else:
            print("approach 높이로 이동...")
            rtde_c.moveL(approach, MOVE_SPEED, MOVE_ACCEL)
            print("수직 하강...")
            rtde_c.moveL(grasp, DESCEND_SPEED, DESCEND_ACCEL)
            print("그리퍼 열기...")
            gripper.set_pos(args.open_pos)
            time.sleep(GRIP_SETTLE_S)
            print("수직 상승 (approach 높이로 복귀)...")
            rtde_c.moveL(approach, DESCEND_SPEED, DESCEND_ACCEL)
        print(f"\n{args.action} 완료. 현재 TCP: {fmt_pose(list(rtde_r.getActualTCPPose()))}")
    finally:
        gripper.close()
        rtde_c.stopScript()
        rtde_c.disconnect()
        rtde_r.disconnect()


if __name__ == "__main__":
    main()
