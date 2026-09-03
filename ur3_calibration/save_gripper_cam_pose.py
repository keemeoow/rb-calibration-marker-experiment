#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ur3_calibration/save_gripper_cam_pose.py -- 그리퍼 카메라 고정 촬영 위치 저장

세션2 pick-and-place 중, 큐브를 놓고 나면(그리퍼가 비면) 항상 이 자세로
돌아와서 그리퍼 카메라로 촬영한다. 큐브 자체는 매번 다른 (x,y,yaw)에
놓이지만, 촬영 위치 자체는 고정이라 카메라가 넓게 바닥을 내려다보는
자세 하나만 있으면 된다.

로봇을 원하는 촬영 자세로 옮긴 뒤 Enter를 누르면 그 순간의 flange 자세가
gripper_cam_pose.json 으로 저장된다.

사용법:
  python save_gripper_cam_pose.py
"""

import argparse
import json
import time
from pathlib import Path

import numpy as np
import rtde_receive

ROBOT_IP_DEFAULT = "192.168.1.101"
OUT_DEFAULT = Path(__file__).resolve().parent / "data" / "gripper_cam_pose.json"


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--robot-ip", default=ROBOT_IP_DEFAULT)
    ap.add_argument("--out", default=str(OUT_DEFAULT))
    args = ap.parse_args()

    out_path = Path(args.out)
    out_path.parent.mkdir(parents=True, exist_ok=True)

    r = rtde_receive.RTDEReceiveInterface(args.robot_ip)
    print("로봇을 그리퍼 카메라 고정 촬영 자세로 옮긴 뒤 Enter. q = 저장 안 하고 종료\n")

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
