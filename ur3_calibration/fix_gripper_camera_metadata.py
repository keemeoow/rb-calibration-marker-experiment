#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ur3_calibration/fix_gripper_camera_metadata.py -- 그리퍼캠 메타데이터 보정

intrinsics/device_map.json 의 gripper_cam_idx/gripper_serial 이 null로 비어
있고, 각 intrinsics/camN.npz의 is_gripper 플래그도 전부 False로 저장되어
있다 (실제 촬영 스크립트 capture_run.py는 카메라를 모델명으로 자동 인식해서
라벨을 붙이지만, 그 결과가 intrinsics 캘리브레이션 산출물 쪽에는 기록되지
않았다). 이 스크립트는 그 결과를 device_map.json의 detected_now/serial_to_idx
로부터 재구성해서 채워 넣는다.

식별 규칙은 capture_run.py::is_gripper_cam_name()과 완전히 동일하다 --
그리퍼캠은 'D435'(IMU 없는 모델)이고, 'D435I'는 그리퍼캠이 아니다
('D435'가 'D435I'의 부분 문자열이라 반드시 I 유무까지 확인해야 한다).

device_map.json에 기록된 실제 장치 4대(2026-09-03 04_intrinsics 산출물 시점):
  039422061216  RealSense D415   -> 그리퍼 아님 (fixed1, cam0)
  136622073980  RealSense D435   -> 그리퍼캠   (gripper, cam1)
  243622070663  RealSense D435I  -> 그리퍼 아님 (fixed2, cam2)
  314522062542  RealSense D415   -> 그리퍼 아님 (fixed3, cam3)
고정캠 3대가 D435I 하나로 통일된 게 아니라 D415 2대 + D435I 1대로 섞여
있다는 점에 주의 (그리퍼캠 판별에는 영향 없음 -- D435 vs D435I만 본다).

원본을 절대 덮어써 잃지 않도록 intrinsics/pre_gripper_fix_backup/ 에 실행 전
device_map.json과 camN.npz 4개를 이미 백업해 두었다(이 스크립트가 그 위에서
다시 실행돼도 안전하도록 idempotent하게 짜여 있다: 이미 올바르게 채워져
있으면 그대로 통과).

사용법:
  python ur3_calibration/fix_gripper_camera_metadata.py           # 적용
  python ur3_calibration/fix_gripper_camera_metadata.py --dry-run # 계획만 출력
"""

import argparse
import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from capture_run import is_gripper_cam_name  # noqa: E402

INTRINSICS_DIR = Path(__file__).resolve().parent / "intrinsics"


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--intrinsics-dir", default=str(INTRINSICS_DIR))
    ap.add_argument("--dry-run", action="store_true")
    args = ap.parse_args()

    intrinsics_dir = Path(args.intrinsics_dir)
    device_map_path = intrinsics_dir / "device_map.json"
    device_map = json.loads(device_map_path.read_text())

    serial_to_idx = device_map["serial_to_idx"]
    detected_now = {item["serial"]: item["name"] for item in device_map["detected_now"]}

    missing = sorted(set(serial_to_idx) - set(detected_now))
    if missing:
        raise SystemExit(
            f"serial_to_idx has serials with no matching detected_now name: {missing} "
            "-- refusing to guess gripper identity without a device name.")

    gripper_serials = [s for s, name in detected_now.items() if is_gripper_cam_name(name)]
    if len(gripper_serials) != 1:
        raise SystemExit(
            f"expected exactly 1 gripper-camera serial (D435, non-I), found {gripper_serials}")
    gripper_serial = gripper_serials[0]
    gripper_idx = int(serial_to_idx[gripper_serial])

    print("Serial -> camN -> device name -> label:")
    rows = []
    for serial, idx in sorted(serial_to_idx.items(), key=lambda kv: kv[1]):
        name = detected_now[serial]
        is_grip = is_gripper_cam_name(name)
        label = "gripper" if is_grip else "fixed"
        rows.append((serial, idx, name, label, is_grip))
        print(f"  {serial}  cam{idx}  {name:18s}  -> {label}")

    if args.dry_run:
        print(f"\n(dry-run) would set gripper_cam_idx={gripper_idx}, gripper_serial={gripper_serial}")
        return

    changed = False
    if device_map.get("gripper_cam_idx") != gripper_idx or device_map.get("gripper_serial") != gripper_serial:
        device_map["gripper_cam_idx"] = gripper_idx
        device_map["gripper_serial"] = gripper_serial
        changed = True
    if changed:
        device_map_path.write_text(json.dumps(device_map, indent=2) + "\n")
        print(f"\nUpdated {device_map_path}: gripper_cam_idx={gripper_idx}, gripper_serial={gripper_serial}")
    else:
        print(f"\n{device_map_path} already correct, left unchanged.")

    for serial, idx, name, label, is_grip in rows:
        npz_path = intrinsics_dir / f"cam{idx}.npz"
        data = dict(np.load(npz_path, allow_pickle=True))
        current = bool(data["is_gripper"]) if "is_gripper" in data else None
        if current == is_grip:
            print(f"  cam{idx}.npz: is_gripper already {is_grip}, unchanged.")
            continue
        data["is_gripper"] = np.array(is_grip)
        np.savez(npz_path, **data)
        print(f"  cam{idx}.npz: is_gripper {current} -> {is_grip} (serial {data['serial']}, {name})")


if __name__ == "__main__":
    main()
