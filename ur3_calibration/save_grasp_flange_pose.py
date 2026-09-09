#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ur3_calibration/save_grasp_flange_pose.py — 큐브를 집는 순간의 flange 자세 저장

이 스크립트는 "큐브의 3D 좌표"를 저장하는 게 아니라, 그 순간의
로봇 flange(TCP) 자세와 조인트 값을 그대로 저장한다. 로봇을 프리드라이브로
그리퍼가 큐브를 무는 위치(바닥까지 내려간 자세)로 옮긴 뒤 Enter를 누르면
그 순간의 flange 자세가 grasp_flange_pose.json 으로 저장되고, pick_place.py가
이 자세를 그대로 다시 목표로 삼아 반복 재현한다.

이 자세는 세션 2(바닥에 큐브를 두고 고정 카메라 + 그리퍼 카메라로 촬영)
전용이다 — 세션 1(손으로 들고 이동), 세션 3(손목만 움직임)은 이 그립 자세를
쓰지 않는다.

사용법:
  python save_grasp_flange_pose.py
  python save_grasp_flange_pose.py --robot-ip 192.168.1.101
"""

import argparse
import json
import time
from pathlib import Path

import numpy as np
import rtde_receive

from capture_poses import SESSIONS

ROBOT_IP_DEFAULT = "192.168.1.101"
OUT_DEFAULT = Path(__file__).resolve().parent / "data" / "grasp_flange_pose.json"
TARGET_SESSION_ID = 2


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--robot-ip", default=ROBOT_IP_DEFAULT)
    ap.add_argument("--out", default=str(OUT_DEFAULT))
    args = ap.parse_args()

    out_path = Path(args.out)
    out_path.parent.mkdir(parents=True, exist_ok=True)

    info = SESSIONS[TARGET_SESSION_ID]
    print(f"=== 이 자세는 세션 {TARGET_SESSION_ID}: {info['name']} 전용입니다 ===")
    print(info["description"])
    print("(세션 1/3은 촬영 목적이 달라 이 그립 자세를 쓰지 않습니다.)\n")

    r = rtde_receive.RTDEReceiveInterface(args.robot_ip)
    print("로봇을 프리드라이브로 큐브를 집을/놓을 위치(바닥까지 내려간 자세)로 옮긴 뒤 Enter.")
    print("q = 저장 안 하고 종료\n")

    try:
        while True:
            cmd = input("> ").strip().lower()
            if cmd == "q":
                print("저장하지 않고 종료합니다.")
                return
            joints = list(r.getActualQ())
            tcp = list(r.getActualTCPPose())
            payload = {
                "timestamp": time.strftime("%Y-%m-%dT%H:%M:%S"),
                "robot_ip": args.robot_ip,
                "pose_convention": "tcp_pose=[x,y,z,rx,ry,rz] meter+rotvec(rad); joints in rad and deg",
                "joint_radians": joints,
                "joint_degrees": [float(np.degrees(v)) for v in joints],
                "tcp_pose": tcp,
            }
            out_path.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n")
            print(
                f"저장됨 -> {out_path}\n"
                f"  xyz=({tcp[0] * 1000:.1f},{tcp[1] * 1000:.1f},{tcp[2] * 1000:.1f})mm"
            )
            again = input("다시 저장하려면 Enter, 끝내려면 q: ").strip().lower()
            if again == "q":
                break
    except (KeyboardInterrupt, EOFError):
        pass
    finally:
        r.disconnect()


if __name__ == "__main__":
    main()
