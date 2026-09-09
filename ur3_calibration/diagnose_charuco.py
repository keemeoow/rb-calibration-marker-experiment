#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ur3_calibration/diagnose_charuco.py -- 카메라가 보는 raw ArUco 마커 ID를 그대로 출력

02_calibrate_intrinsics.py 에서 "코너 0"만 계속 뜨는 건 대부분 코너 개수
임계값(--min_corners) 문제가 아니다. calibration_pipeline/charuco.py 의
_filter_board_markers()가 감지된 마커 ID를 board_id_set(=marker_id_start
부터 (squares_x*squares_y)//2 개)으로 걸러내는데, 실제 보드의 마커 ID가
그 범위 밖이면 전부 걸러져서 detect()가 (None, None, 0, None, None)을
반환한다 -- min_corners를 낮추거나 지워도 그랩할 게 없다.

이 스크립트는 그 필터링 없이, 카메라가 실제로 보는 raw 마커 ID를
여러 dictionary로 다 시도해서 그대로 보여준다. 어떤 dictionary가 맞는지,
마커 ID가 몇 번부터 시작하는지(marker_id_start) 바로 알 수 있다.

사용법:
  python diagnose_charuco.py                       # 첫 카메라, 흔한 dictionary 다 시도
  python diagnose_charuco.py --serial 039422061216 # 카메라 지정
  python diagnose_charuco.py --dictionary DICT_5X5_250  # 이거 하나만 시도
"""

import argparse
import sys
import time
from pathlib import Path

import cv2

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from capture_pipeline.camera import RealSenseCamera  # noqa: E402

DICTS_TO_TRY = [
    "DICT_4X4_50", "DICT_4X4_100", "DICT_4X4_250", "DICT_4X4_1000",
    "DICT_5X5_50", "DICT_5X5_100", "DICT_5X5_250", "DICT_5X5_1000",
    "DICT_6X6_50", "DICT_6X6_100", "DICT_6X6_250", "DICT_6X6_1000",
    "DICT_7X7_50", "DICT_7X7_100", "DICT_7X7_250", "DICT_7X7_1000",
]


def grab_one_frame(serial):
    RealSenseCamera.reset_all_devices()
    devices = RealSenseCamera.list_devices()
    if not devices:
        raise RuntimeError("연결된 RealSense 카메라가 없습니다.")
    serial = serial or sorted(devices)[0]
    print(f"카메라: {serial}  {devices.get(serial, '?')}")

    cam = RealSenseCamera(serial, width=1280, height=720, fps=15,
                           use_color=True, use_depth=False, lock_color_exposure=False)
    cam.start()
    time.sleep(1.0)
    color = None
    for _ in range(30):
        color, _depth, _ts = cam.get_latest()
        if color is not None:
            break
        time.sleep(0.1)
    cam.stop()
    if color is None:
        raise RuntimeError("프레임을 받지 못했습니다.")
    return color


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--serial", default=None, help="카메라 시리얼 (생략하면 발견된 첫 카메라)")
    ap.add_argument("--dictionary", default=None, help="이 dictionary 하나만 시도 (생략하면 여러 개 다 시도)")
    args = ap.parse_args()

    print("보드를 카메라 정면에 잘 보이게 든 상태에서 실행하세요.\n")
    color = grab_one_frame(args.serial)
    gray = cv2.cvtColor(color, cv2.COLOR_BGR2GRAY)

    dicts = [args.dictionary] if args.dictionary else DICTS_TO_TRY
    any_found = False
    for name in dicts:
        dict_id = getattr(cv2.aruco, name, None)
        if dict_id is None:
            print(f"  {name}: (알 수 없는 dictionary 이름)")
            continue
        dictionary = cv2.aruco.getPredefinedDictionary(dict_id)
        params = cv2.aruco.DetectorParameters()
        detector = cv2.aruco.ArucoDetector(dictionary, params)
        corners, ids, _ = detector.detectMarkers(gray)
        n = 0 if ids is None else len(ids)
        id_list = sorted(int(i) for i in ids.reshape(-1)) if ids is not None else []
        marker = "  <-- 감지됨! marker_id_start 후보 = " + str(min(id_list)) if n > 0 else ""
        print(f"  {name}: {n}개  ids={id_list}{marker}")
        if n > 0:
            any_found = True

    out_path = Path(__file__).resolve().parent / "diagnose_charuco_frame.png"
    cv2.imwrite(str(out_path), color)
    print(f"\n촬영한 프레임 저장 -> {out_path} (육안으로도 보드가 잘 보이는지 확인 가능)")
    if not any_found:
        print("\n어떤 dictionary로도 마커가 하나도 안 잡혔습니다 -- "
              "보드가 프레임 안에 잘 보이는지, 초점/조명이 괜찮은지, "
              "인쇄가 너무 흐리거나 반사되지 않는지 확인해주세요.")


if __name__ == "__main__":
    main()
