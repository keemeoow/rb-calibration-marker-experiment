#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""zeus_gello_calibration/capture_frames.py -- 카메라 4대 RGB + depth 를 N장 찍어 저장.
(외부 GT 촬영용. 로봇은 움직이지 않고, 연결되어 있으면 촬영 순간 pose만 같이 기록.)

저장: <out-root>/<timestamp>/<NNN>/cam_<label>.png, cam_<label>_depth.png, robot.json

사용법:
  python capture_frames.py --shots 5                    # Enter 누를 때마다 1장, 5장
  python capture_frames.py --shots 5 --no-robot         # 로봇 서버 없이 사진만
  python capture_frames.py --shots 5 --no-cam-reset     # USB 리셋 생략 (리셋이 카메라를 떨어뜨릴 때)
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT))

from robot.backends.zeus_client import ZeusClient  # noqa: E402

from capture_session import (  # noqa: E402
    DEVICE_MAP_DEFAULT, ROBOT_IP_DEFAULT, ROBOT_PORT_DEFAULT, LiveView,
    connect_cameras, grab_frames, load_camera_labels, read_robot_state, stop_cameras, write_capture,
)

OUT_ROOT_DEFAULT = REPO_ROOT / "zeus_gello_calibration" / "gt_compare_captures"


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--shots", type=int, default=5)
    ap.add_argument("--out-root", default=str(OUT_ROOT_DEFAULT))
    ap.add_argument("--device-map", default=str(DEVICE_MAP_DEFAULT))
    ap.add_argument("--robot-ip", default=ROBOT_IP_DEFAULT)
    ap.add_argument("--robot-port", type=int, default=ROBOT_PORT_DEFAULT)
    ap.add_argument("--no-robot", action="store_true", help="로봇 서버에 접속하지 않음 (pose 기록 생략)")
    ap.add_argument("--no-cam-reset", action="store_true")
    ap.add_argument("--no-preview", action="store_true")
    args = ap.parse_args()

    out_dir = Path(args.out_root) / time.strftime("%Y%m%d_%H%M%S")
    labels = load_camera_labels(Path(args.device_map))
    cams, used_labels = connect_cameras(labels, no_reset=args.no_cam_reset)
    view = None
    if not args.no_preview:
        view = LiveView(cams, used_labels, window_name="capture_frames (q/ESC=닫기)")
        view.start()

    rb = None
    if not args.no_robot:
        try:
            rb = ZeusClient(args.robot_ip, args.robot_port)
            rb.connect()
            print("로봇 서버 연결됨 (읽기 전용, pose 기록용).")
        except Exception as exc:
            print(f"[WARN] 로봇 서버 연결 실패, pose 없이 진행: {exc}")
            rb = None

    print(f"저장 폴더: {out_dir}\n{args.shots}장 촬영. Enter=촬영 / q=종료\n")
    n = 0
    try:
        while n < args.shots:
            if view is not None:
                # 미리보기 갱신하면서 키 대기 (SPACE=촬영)
                key = view.show(wait_ms=30, status=f"[{n}/{args.shots}] SPACE=촬영  q/ESC=종료")
                if view._closed:
                    break
                if key != 32:
                    continue
            else:
                cmd = input(f"[{n}/{args.shots}] > ").strip().lower()
                if cmd == "q":
                    break
            if rb is not None:
                robot_state = read_robot_state(rb, {"capture_index": n, "note": "capture_frames: 로봇 이동 없음"})
            else:
                robot_state = {"timestamp": time.strftime("%Y-%m-%dT%H:%M:%S"), "pose": None, "joints": None,
                               "capture_index": n, "note": "capture_frames: 로봇 미접속"}
            frames = grab_frames(cams, used_labels)
            write_capture(frames, out_dir / f"{n:03d}", robot_state)
            if robot_state.get("pose"):
                print(f"  pose_mm_deg={[round(v, 2) for v in robot_state['pose']]}")
            n += 1
    except (KeyboardInterrupt, EOFError):
        pass
    finally:
        if rb is not None:
            rb.close()
        if view is not None:
            view.stop()
        stop_cameras(cams)
    print(f"\n완료 -- {out_dir} 에 {n}장 저장됨")


if __name__ == "__main__":
    main()
