#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ur3_calibration/save_cube_pose.py — 바닥에 큐브를 놓는(=집는) 고정 위치 저장

세션 2의 pick_place.py 가 사용할 큐브의 고정 그립 위치를 저장한다.
로봇을 프리드라이브로 "지금 큐브를 놓을/집을 위치"(그리퍼가 큐브를 문 채
바닥까지 내려간 자세)로 옮긴 뒤 Enter를 누르면 그 순간의 TCP pose가
cube_pose.json 으로 저장된다. 이 자세의 Z 값이 곧 큐브의 바닥 높이(그립 높이)로
pick_place.py 의 하강 목표가 된다.

사용법:
  python save_cube_pose.py
  python save_cube_pose.py --robot-ip 192.168.1.101
"""

import argparse
import json
import time
from pathlib import Path

import numpy as np
import rtde_receive

ROBOT_IP_DEFAULT = "192.168.1.101"
OUT_DEFAULT = Path(__file__).resolve().parent / "data" / "cube_pose.json"


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--robot-ip", default=ROBOT_IP_DEFAULT)
    ap.add_argument("--out", default=str(OUT_DEFAULT))
    args = ap.parse_args()

    out_path = Path(args.out)
    out_path.parent.mkdir(parents=True, exist_ok=True)

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
