#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""zeus_gello_calibration/gt_compare_fits.py -- 큐브 사진 한 장으로 4가지 fit의
추론값(큐브 중심 base-frame 위치)을 비교한다. 로봇은 움직이지 않는다.

GT 큐브를 바닥에 두고 한 번만 촬영(카메라 3대) -> 그 이미지들에 대해
fit_통합_no-fk.json / fit_독립_no-fk.json / fit_통합_raw-fk.json /
fit_독립_raw-fk.json 각각의 카메라 extrinsics로 큐브 pose를 계산 -> 네 결과의
x,y,z,rz와 서로 간 차이(mm)를 출력한다. 검출(PnP)은 fit마다 다시 안 하고
한 번만 하고, 그 결과에 각 fit의 T_base_Ci만 다르게 적용한다(검출 자체는
fit과 무관하니까).

사용법:
  python gt_compare_fits.py
"""

import argparse
import sys
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT))

from calibration_pipeline.apriltag_cube import AprilTagCubeTarget, rodrigues_to_Rt  # noqa: E402
from calibration_pipeline.cube_config import load_cube_config_from_json_file  # noqa: E402
from robot.backends.zeus_client import T_to_pose6  # noqa: E402

from capture_session import (  # noqa: E402
    load_camera_labels, connect_cameras, stop_cameras, LiveView, grab_frames,
    DEVICE_MAP_DEFAULT,
)
from fit_grasp_offset import LOCAL_CAM_IDS, load_intrinsics_by_label  # noqa: E402
from gt_pick_test import GT_CUBE_CONFIG_PATH, FIXED_LABELS, load_fit, detect_cube_pose  # noqa: E402

DEFAULT_FITS = [
    ("통합_no-fk", REPO_ROOT / "zeus_gello_calibration" / "fit_통합_no-fk.json"),
    ("독립_no-fk", REPO_ROOT / "zeus_gello_calibration" / "fit_독립_no-fk.json"),
    ("통합_raw-fk", REPO_ROOT / "zeus_gello_calibration" / "fit_통합_raw-fk.json"),
    ("독립_raw-fk", REPO_ROOT / "zeus_gello_calibration" / "fit_독립_raw-fk.json"),
]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--device-map", default=str(DEVICE_MAP_DEFAULT))
    ap.add_argument("--zeus-intrinsics-dir", default=str(REPO_ROOT / "intrinsics"))
    ap.add_argument("--ur3-intrinsics-dir", default=str(REPO_ROOT / "ur3_calibration" / "intrinsics"))
    ap.add_argument("--gt-cube-config", default=str(GT_CUBE_CONFIG_PATH))
    ap.add_argument("--reproj-thr-px", type=float, default=10.0)
    ap.add_argument("--no-cam-reset", action="store_true")
    ap.add_argument("--no-preview", action="store_true")
    args = ap.parse_args()

    cube_cfg, src = load_cube_config_from_json_file(args.gt_cube_config)
    if cube_cfg is None:
        print(f"[ERROR] GT 큐브 config를 못 읽었습니다: {args.gt_cube_config}")
        return
    cube_target = AprilTagCubeTarget(cube_cfg)
    K_map, D_map = load_intrinsics_by_label(
        Path(args.zeus_intrinsics_dir), Path(args.ur3_intrinsics_dir), Path(args.device_map))

    labels = load_camera_labels(Path(args.device_map))
    cams, used_labels = connect_cameras(labels, no_reset=args.no_cam_reset)
    view = None
    if not args.no_preview:
        view = LiveView(cams, used_labels, window_name="gt_compare_fits (q/ESC=닫기)")
        view.start()

    try:
        input("\nGT 큐브를 바닥에 놓고 Enter를 누르면 한 장 촬영합니다 > ")
        if view is not None:
            view.show()
        frames = grab_frames(cams, used_labels)
    finally:
        if view is not None:
            view.stop()
        stop_cameras(cams)

    results = []
    for label, fit_path in DEFAULT_FITS:
        if not fit_path.is_file():
            print(f"[WARN] {fit_path} 없음, 건너뜀")
            continue
        _T_gripper_cube, T_base_cam = load_fit(fit_path)
        T_base_cube, report = detect_cube_pose(frames, K_map, D_map, T_base_cam, cube_target, args.reproj_thr_px)
        if T_base_cube is None:
            print(f"[{label}] 검출 실패: {report}")
            continue
        pose6 = T_to_pose6(T_base_cube)
        results.append((label, pose6, report))

    print(f"\n{'method':>14} {'x_mm':>10} {'y_mm':>10} {'z_mm':>10} {'rz_deg':>10}")
    for label, pose6, _ in results:
        print(f"{label:>14} {pose6[0]:>10.2f} {pose6[1]:>10.2f} {pose6[2]:>10.2f} {pose6[3]:>10.2f}")

    if len(results) >= 2:
        print("\n방식 간 위치 차이 (mm, xyz 유클리드 거리):")
        for i in range(len(results)):
            for j in range(i + 1, len(results)):
                li, pi, _ = results[i]
                lj, pj, _ = results[j]
                d = float(np.linalg.norm(np.asarray(pi[:3]) - np.asarray(pj[:3])))
                print(f"  {li} vs {lj}: {d:.3f} mm")

    for label, _pose6, report in results:
        print(f"\n[{label}] 카메라별 검출 상세:")
        for cam_label, msg in report.items():
            print(f"  [{cam_label}] {msg}")


if __name__ == "__main__":
    main()
