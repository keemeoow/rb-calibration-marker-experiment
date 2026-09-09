#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ur3_calibration/capture_poses.py — UR3 캘리브레이션용 목표 자세 수집

세 세션으로 나눠 로봇 자세를 저장한다:
  1: handheld_fixed_cam      — 그리퍼로 마커보드를 들고 움직이며 고정 카메라들이 촬영 (eye-to-hand)
  2: floor_board_dual_cam    — 마커보드를 바닥에 두고 로봇이 접근, 고정 카메라 + 그리퍼 카메라 촬영
  3: wrist_motion_gripper_cam — 손목만 움직이며 바닥 마커보드를 그리퍼 카메라로 촬영 (eye-in-hand)

카메라 촬영은 이후 단계에서 붙일 예정이므로, 지금은 ur_rtde로 읽은
로봇 자세(TCP pose + joint angles)만 세션별 JSON으로 저장한다.

로봇은 티치펜던트의 프리드라이브(freedrive) 버튼으로 손으로 옮기고,
터미널에서 Enter를 눌러 그 순간의 자세를 기록한다.

사용법:
  python capture_poses.py --session 1
  python capture_poses.py --session 2 --num-poses 15
  python capture_poses.py --session 3 --reset

키: Enter = 현재 자세 저장 / d = 마지막 저장 삭제 / q = 세션 조기 종료
"""

import argparse
import json
import time
from pathlib import Path

import numpy as np
import rtde_receive

ROBOT_IP_DEFAULT = "192.168.1.101"

SESSIONS = {
    1: {
        "name": "handheld_fixed_cam",
        "description": "그리퍼로 마커보드를 들고 움직이며 고정 카메라들이 촬영 (eye-to-hand)",
    },
    2: {
        "name": "floor_board_dual_cam",
        "description": "마커보드를 바닥에 두고 로봇이 접근, 고정 카메라 + 그리퍼 카메라 촬영",
    },
    3: {
        "name": "wrist_motion_gripper_cam",
        "description": "손목만 움직이며 바닥 마커보드를 그리퍼 카메라로 촬영 (eye-in-hand)",
    },
}


def session_path(out_root: Path, session_id: int) -> Path:
    info = SESSIONS[session_id]
    return out_root / f"session{session_id}_{info['name']}" / "poses.json"


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument("--session", type=int, required=True, choices=sorted(SESSIONS), help="세션 번호 (1/2/3)")
    ap.add_argument("--robot-ip", default=ROBOT_IP_DEFAULT)
    ap.add_argument("--num-poses", type=int, default=15, help="목표 자세 개수 (기본 15)")
    ap.add_argument(
        "--out-root",
        default=str(Path(__file__).resolve().parent / "data"),
        help="세션 폴더가 생성될 상위 경로",
    )
    ap.add_argument("--reset", action="store_true", help="기존 세션 파일 비우고 새로 시작")
    args = ap.parse_args()

    info = SESSIONS[args.session]
    out_file = session_path(Path(args.out_root), args.session)
    out_file.parent.mkdir(parents=True, exist_ok=True)

    poses = []
    if out_file.exists() and not args.reset:
        poses = json.loads(out_file.read_text())["poses"]
        print(f"기존 {len(poses)}개 로드됨 (--reset 으로 초기화 가능)")

    print(f"=== 세션 {args.session}: {info['name']} ===")
    print(info["description"])
    print(f"목표 {args.num_poses}개, 로봇: {args.robot_ip}")
    print("로봇을 프리드라이브로 원하는 자세로 옮긴 뒤: Enter=저장 / d=마지막 삭제 / q=조기 종료\n")

    r = rtde_receive.RTDEReceiveInterface(args.robot_ip)

    def save():
        payload = {
            "session_id": args.session,
            "session_name": info["name"],
            "description": info["description"],
            "robot_ip": args.robot_ip,
            "pose_convention": "tcp_pose=[x,y,z,rx,ry,rz] meter+rotvec(rad); joints in rad and deg",
            "poses": poses,
        }
        out_file.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n")

    try:
        while len(poses) < args.num_poses:
            cmd = input(f"[{len(poses)}/{args.num_poses}] > ").strip().lower()
            if cmd == "q":
                break
            if cmd == "d":
                if poses:
                    poses.pop()
                    save()
                    print(f"  삭제됨 (남은 {len(poses)}개)")
                continue
            joints = list(r.getActualQ())
            tcp = list(r.getActualTCPPose())
            poses.append({
                "index": len(poses),
                "timestamp": time.strftime("%Y-%m-%dT%H:%M:%S"),
                "joint_radians": joints,
                "joint_degrees": [float(np.degrees(v)) for v in joints],
                "tcp_pose": tcp,
            })
            save()
            print(
                f"  저장 #{len(poses)}/{args.num_poses}: "
                f"xyz=({tcp[0] * 1000:.0f},{tcp[1] * 1000:.0f},{tcp[2] * 1000:.0f})mm"
            )
    except (KeyboardInterrupt, EOFError):
        pass
    finally:
        r.disconnect()

    print(f"\n완료 — {out_file} 에 {len(poses)}개 저장됨")
    if len(poses) < args.num_poses:
        print(f"(목표 {args.num_poses}개 중 {len(poses)}개만 저장됨 — 이어서 하려면 같은 명령 다시 실행)")


if __name__ == "__main__":
    main()
