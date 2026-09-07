#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""zeus_gello_calibration/replay_and_recapture.py -- 저장된 세션의 joint 자세로
로봇을 직접(자동) 다시 움직여서 pose/joint - 이미지 동기화 오차를 없애고 재촬영

GELLO 텔레옵으로 촬영할 때는 SPACE를 누르는 순간에도 사람 손이 계속 움직이고
있을 수 있어서, "그 순간 읽은 pose"와 "그 순간 찍힌 이미지"가 미세하게 어긋날
(시간 지연으로 인한 위치 offset) 위험이 있다. 이 스크립트는 그 위험을 없애기
위해, 이미 저장된 capture/<idx>/robot.json 의 joint 값으로 **로봇을 완전히
멈춰 세운 뒤** 촬영한다(움직이는 도중에 찍는 게 아니라 도착 후 정지 상태에서
촬영 -> pose와 이미지가 확실히 같은 순간을 가리킴).

*** 이 스크립트는 실제로 로봇을 자동으로 움직인다(movej). ***
지금까지 이 프로젝트의 Zeus 쪽 스크립트들은 전부 GELLO 텔레옵(사람이 직접
조종)이거나 get_state()만 읽는 읽기 전용이었는데, 이건 처음으로 자동 이동
명령을 보낸다. 원래 GELLO로 자유롭게 움직이며 찍은 16개 자세를 순서 그대로
재생하는 거라 연속된 자세끼리는 원래도 사람이 부드럽게 이어서 움직인
경로이긴 하지만, **그래도 처음 실행은 반드시 로봇 옆에서 비상정지에 손이
닿는 상태로, 저속(--jnt-speed 낮게)으로 스텝별 확인(--no-step 없이)하며
진행할 것.**

큐브가 계속 그리퍼에 물려있어야 한다(원래 세션1 촬영 때와 같은 방식으로
쥔 상태) -- 이 스크립트는 그리퍼를 전혀 건드리지 않는다.

기본은 기존 capture/<idx>/ 폴더를 덮어쓰지 않고 별도 폴더
(capture_replayed/<idx>/)에 저장한다 -- 원본과 비교해보고 정말 이걸로
바꿀지 판단할 수 있게. --overwrite 를 주면 원래 capture/<idx>/ 를 그대로
덮어쓴다.

사용법:
  python replay_and_recapture.py --session 1                     # dry-run: 계획만 출력
  python replay_and_recapture.py --session 1 --execute            # 스텝별 확인하며 실행
  python replay_and_recapture.py --session 1 --execute --no-step  # (검증 후) 연속 실행
  python replay_and_recapture.py --session 1 --execute --overwrite  # 원본 폴더에 덮어쓰기
"""

import argparse
import json
import sys
import time
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from capture_pipeline.camera import RealSenseCamera  # noqa: E402
from robot.backends.zeus_client import ZeusClient  # noqa: E402

from capture_session import (  # noqa: E402
    SESSIONS, load_camera_labels, connect_cameras, stop_cameras, LiveView,
    grab_frames, write_capture, read_robot_state,
    ROBOT_IP_DEFAULT, ROBOT_PORT_DEFAULT, DEVICE_MAP_DEFAULT,
)

JNT_SPEED_DEFAULT = 10.0   # GELLO 텔레옵과 동일한 "실기 테스트로 정한" 기본값
OVERLAP_DEFAULT = 0.0      # 블렌딩 없이 매번 완전히 멈춰야 정확한 정지 후 촬영이 됨
SETTLE_S = 0.3             # movej 리턴 직후 잔진동/카메라 버퍼 안정화 대기


def load_saved_joints(capture_root: Path) -> list:
    items = []
    for d in sorted(p for p in capture_root.iterdir() if p.is_dir()):
        robot_json = d / "robot.json"
        if not robot_json.exists():
            continue
        data = json.loads(robot_json.read_text())
        items.append({"index": int(d.name), "joints": data["joints"], "src_dir": d})
    return items


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--session", type=int, required=True, choices=sorted(SESSIONS))
    ap.add_argument("--robot-ip", default=ROBOT_IP_DEFAULT)
    ap.add_argument("--robot-port", type=int, default=ROBOT_PORT_DEFAULT)
    ap.add_argument("--device-map", default=str(DEVICE_MAP_DEFAULT))
    ap.add_argument("--out-root", default=str(Path(__file__).resolve().parent / "data"))
    ap.add_argument("--jnt-speed", type=float, default=JNT_SPEED_DEFAULT)
    ap.add_argument("--overlap", type=float, default=OVERLAP_DEFAULT)
    ap.add_argument("--overwrite", action="store_true",
                    help="원래 capture/<idx>/ 폴더에 덮어씀 (기본은 capture_replayed/<idx>/에 따로 저장)")
    ap.add_argument("--execute", action="store_true", help="실제로 이동/촬영 (없으면 dry-run)")
    ap.add_argument("--no-step", action="store_true", help="스텝마다 Enter로 확인하지 않고 연속 실행")
    ap.add_argument("--no-cam-reset", action="store_true")
    ap.add_argument("--no-preview", action="store_true")
    args = ap.parse_args()

    info = SESSIONS[args.session]
    session_dir = Path(args.out_root) / f"session{args.session}_{info['name']}"
    capture_root = session_dir / "capture"
    out_root = capture_root if args.overwrite else session_dir / "capture_replayed"

    items = load_saved_joints(capture_root)
    if not items:
        print(f"[ERROR] {capture_root} 에 저장된 캡처가 없습니다.")
        return

    print(f"=== 세션 {args.session}: {info['name']} 재생+재촬영 ===")
    print(f"원본: {capture_root}  ({len(items)}개)")
    print(f"저장 위치: {out_root}{' (원본 덮어씀)' if args.overwrite else ' (원본은 그대로 둠)'}")
    print(f"jnt_speed={args.jnt_speed}  overlap={args.overlap}\n")
    print("큐브가 세션1 촬영 때와 같은 방식으로 그리퍼에 그대로 물려있어야 합니다 "
          "(이 스크립트는 그리퍼를 건드리지 않습니다).\n")

    for it in items:
        print(f"  #{it['index']:03d}  joints(deg)={[round(v, 1) for v in it['joints']]}")

    if not args.execute:
        print(f"\n(dry-run) 총 {len(items)}개 이동+촬영 계획. --execute 를 주면 실행합니다.")
        return

    labels = load_camera_labels(Path(args.device_map))
    cams, used_labels = connect_cameras(labels, no_reset=args.no_cam_reset)
    view = None
    if not args.no_preview:
        view = LiveView(cams, used_labels, window_name="replay_and_recapture (q/ESC=닫기)")
        view.start()

    rb = ZeusClient(args.robot_ip, args.robot_port)
    rb.connect()
    print("\n*** 실제 로봇이 자동으로 움직입니다. 비상정지에 손이 닿는 상태인지 확인하세요. ***")
    if not args.no_step:
        if input("계속하려면 'go' 입력: ").strip().lower() != "go":
            print("취소했습니다.")
            rb.close()
            if view is not None:
                view.stop()
            stop_cameras(cams)
            return

    try:
        for it in items:
            desc = f"[{it['index']:03d}/{len(items)}] movej -> {[round(v, 1) for v in it['joints']]}"
            if not args.no_step:
                cmd = input(f"\n{desc}\nEnter=이동+촬영 / q=중단 > ").strip().lower()
                if cmd == "q":
                    print("중단했습니다.")
                    break
            else:
                print(desc)

            rb.movej(it["joints"], jnt_speed=args.jnt_speed, overlap=args.overlap)
            time.sleep(SETTLE_S)  # 잔진동 + 카메라 버퍼가 새 프레임으로 채워질 시간

            if view is not None:
                view.show()

            robot_state = read_robot_state(rb, {"capture_index": it["index"], "replayed_from": str(it["src_dir"])})
            frames = grab_frames(cams, used_labels)
            out_dir = out_root / f"{it['index']:03d}"
            write_capture(frames, out_dir, robot_state)
    except (KeyboardInterrupt, EOFError):
        pass
    finally:
        rb.close()
        if view is not None:
            view.stop()
        stop_cameras(cams)

    print(f"\n완료 -- {out_root} 확인해보세요.")


if __name__ == "__main__":
    main()
