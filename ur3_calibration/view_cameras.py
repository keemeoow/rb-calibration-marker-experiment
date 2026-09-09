#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ur3_calibration/view_cameras.py — 연결된 카메라 전체를 실시간으로 띄워본다

지금 연결된 카메라(그리퍼 카메라 1대 + 고정 카메라 2대, RealSense) 전부를
자동으로 찾아서 RGB 라이브 화면을 창으로 띄운다. 촬영/저장은 하지 않고
확인용이다.

해상도는 이 프로젝트의 기존 Zeus 촬영 표준인 1280x720@15fps 를 그대로
쓴다 (capture_pipeline/camera.py, capture_pipeline/export_intrinsics.py 참고 —
1280x1024가 아니라 1280x720이 프로젝트 표준).

사용법:
  python view_cameras.py
  python view_cameras.py --width 1280 --height 720 --fps 15
  q 또는 ESC = 종료
"""

import argparse
import sys
import time
from pathlib import Path

import cv2

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from capture_pipeline.camera import RealSenseCamera  # noqa: E402


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--width", type=int, default=1280)
    ap.add_argument("--height", type=int, default=720)
    ap.add_argument("--fps", type=int, default=15)
    ap.add_argument("--no-reset", action="store_true", help="시작 전 하드웨어 리셋 생략 (빠르게 켜기)")
    args = ap.parse_args()

    if not args.no_reset:
        RealSenseCamera.reset_all_devices()

    devices = RealSenseCamera.list_devices()
    if not devices:
        print("연결된 RealSense 카메라를 찾지 못했습니다 (rs-enumerate-devices 로 확인해보세요).")
        return
    print(f"카메라 {len(devices)}대 발견:")
    for serial, name in devices.items():
        print(f"  {serial}  {name}")

    cams = {}
    for serial in devices:
        cam = RealSenseCamera(
            serial,
            width=args.width,
            height=args.height,
            fps=args.fps,
            use_color=True,
            use_depth=False,
            lock_color_exposure=False,
        )
        cam.start()
        cams[serial] = cam
        cv2.namedWindow(serial, cv2.WINDOW_NORMAL)

    print("\nq 또는 ESC 를 누르면 종료합니다.")
    try:
        while True:
            for serial, cam in cams.items():
                color, _depth, _ts = cam.get_latest()
                if color is not None:
                    cv2.imshow(serial, color)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
            if all(cv2.getWindowProperty(s, cv2.WND_PROP_VISIBLE) < 1 for s in cams):
                break
            time.sleep(0.005)
    finally:
        for cam in cams.values():
            cam.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
