#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ur3_calibration/merge_pointclouds.py -- 캘리브레이션만으로 4대 카메라 depth를 하나로 합치기

COLMAP 같은 별도 정합(SfM/ICP) 알고리즘 없이, fixed_fk 캘리브레이션에서 이미
구한 카메라 위치(T_base_cam)만으로 각 카메라의 depth를 로봇 base 좌표계로
직접 옮겨서 합친다 -- 카메라 위치를 이미 알고 있으니 점군끼리 서로 맞춰볼
필요 없이 바로 겹쳐놓기만 하면 된다.

각 카메라: depth 픽셀 + intrinsics로 카메라 좌표계 3D 점 생성(backproject)
          -> T_base_cam으로 base 좌표계로 변환 -> 색은 컬러 이미지에서 가져옴.

결과를 PLY 파일로 저장 (MeshLab, CloudCompare 등으로 바로 열어볼 수 있음).
카메라별로 겹치는 영역(예: 바닥에 놓인 큐브)이 잘 겹쳐 보이면 캘리브레이션이
기하학적으로 맞다는 뜻이고, 어긋나 보이면 그만큼 오차가 있다는 뜻이다.

사용법:
  python merge_pointclouds.py --capture ur3_calibration/data/session2_floor_board_dual_cam/capture/001
  python merge_pointclouds.py --capture ... --out my_scene.ply --stride 2
"""

import argparse
import json
from pathlib import Path

import cv2
import numpy as np
from scipy.spatial.transform import Rotation

CAM_LABEL_TO_IDX = {"fixed1": 0, "gripper": 1, "fixed2": 2, "fixed3": 3}
GRIPPER_CAM_IDX = 1
FIT_PATH_DEFAULT = Path(__file__).resolve().parent / "full_fixed_fk_fit.json"
INTRINSICS_DIR_DEFAULT = Path(__file__).resolve().parent / "intrinsics"


def ur3_tcp_pose_to_matrix(tcp_pose) -> np.ndarray:
    x, y, z, rx, ry, rz = tcp_pose
    T = np.eye(4)
    T[:3, :3] = Rotation.from_rotvec([rx, ry, rz]).as_matrix()
    T[:3, 3] = [x, y, z]
    return T


def load_fit(fit_path: Path) -> dict:
    fit = json.loads(fit_path.read_text())
    T_base_cam = {int(k): np.array(v) for k, v in fit["T_base_cam"].items()}
    T_gripper_cam = np.array(fit["T_gripper_cam"])
    return T_base_cam, T_gripper_cam


def edge_mask(depth_u16: np.ndarray, grad_thresh_mm: float) -> np.ndarray:
    """RealSense의 'flying pixel' 노이즈(물체 경계에서 앞/뒤 depth가 섞여
    카메라 쪽으로 길게 뻗치는 점) 제거용. depth_u16의 raw 단위가 mm인
    카메라(depth_scale_m_per_unit=0.001)를 가정하고 그대로 Sobel gradient를
    잰다 -- 실측(session2 cam0)에서 유효 픽셀의 ~4%가 500mm 넘게 튀는 걸
    확인했고, 그게 화면에 보인 뾰족한 선들의 정체다."""
    depth_f = depth_u16.astype(np.float64)
    gx = cv2.Sobel(depth_f, cv2.CV_64F, 1, 0, ksize=3)
    gy = cv2.Sobel(depth_f, cv2.CV_64F, 0, 1, ksize=3)
    grad = np.sqrt(gx ** 2 + gy ** 2)
    return grad < grad_thresh_mm


def backproject(depth_u16: np.ndarray, color_bgr: np.ndarray, K: np.ndarray,
                 depth_scale: float, stride: int, max_depth_m: float,
                 grad_thresh_mm: float = 30.0):
    smooth = edge_mask(depth_u16, grad_thresh_mm)
    h, w = depth_u16.shape
    ys, xs = np.mgrid[0:h:stride, 0:w:stride]
    depth = depth_u16[ys, xs].astype(np.float64) * depth_scale
    valid = (depth > 0) & (depth < max_depth_m) & smooth[ys, xs]
    xs, ys, depth = xs[valid], ys[valid], depth[valid]

    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    x_cam = (xs - cx) / fx * depth
    y_cam = (ys - cy) / fy * depth
    points_cam = np.stack([x_cam, y_cam, depth], axis=1)  # (N,3)

    colors = color_bgr[ys, xs][:, ::-1].astype(np.uint8)  # BGR -> RGB
    return points_cam, colors


def transform_points(points_cam: np.ndarray, T_base_cam: np.ndarray) -> np.ndarray:
    R = T_base_cam[:3, :3]
    t = T_base_cam[:3, 3]
    return points_cam @ R.T + t


def write_ply(path: Path, points: np.ndarray, colors: np.ndarray):
    n = len(points)
    with open(path, "w") as f:
        f.write("ply\nformat ascii 1.0\n")
        f.write(f"element vertex {n}\n")
        f.write("property float x\nproperty float y\nproperty float z\n")
        f.write("property uchar red\nproperty uchar green\nproperty uchar blue\n")
        f.write("end_header\n")
        for p, c in zip(points, colors):
            f.write(f"{p[0]:.5f} {p[1]:.5f} {p[2]:.5f} {int(c[0])} {int(c[1])} {int(c[2])}\n")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--capture", required=True, help="예: ur3_calibration/data/session2.../capture/001")
    ap.add_argument("--fit", default=str(FIT_PATH_DEFAULT))
    ap.add_argument("--intrinsics-dir", default=str(INTRINSICS_DIR_DEFAULT))
    ap.add_argument("--out", default=str(Path(__file__).resolve().parent / "merged_scene.ply"))
    ap.add_argument("--stride", type=int, default=2, help="픽셀 서브샘플링 간격 (클수록 점 적고 파일 작음)")
    ap.add_argument("--max-depth-m", type=float, default=1.5, help="이 거리 넘는 depth는 버림")
    ap.add_argument("--edge-thresh-mm", type=float, default=30.0,
                    help="이웃 픽셀과 depth 차이가 이보다 크면 flying pixel로 보고 제외")
    ap.add_argument("--per-camera-color", action="store_true",
                    help="실제 색 대신 카메라별로 다른 단색을 입혀서 정합 상태를 더 쉽게 확인")
    args = ap.parse_args()

    CAMERA_FLAT_COLORS = {
        "fixed1": (255, 60, 60), "fixed2": (60, 220, 60),
        "fixed3": (60, 120, 255), "gripper": (255, 220, 40),
    }

    capture_dir = Path(args.capture)
    T_base_cam, T_gripper_cam = load_fit(Path(args.fit))
    intr_dir = Path(args.intrinsics_dir)

    robot = json.loads((capture_dir / "robot.json").read_text())
    T_base_gripper_now = ur3_tcp_pose_to_matrix(robot["tcp_pose"])

    all_points, all_colors = [], []
    for label, cam_idx in CAM_LABEL_TO_IDX.items():
        color_path = capture_dir / f"cam_{label}.png"
        depth_path = capture_dir / f"cam_{label}_depth.png"
        if not color_path.exists() or not depth_path.exists():
            print(f"[WARN] {label}: 이미지 없음, 건너뜀 ({color_path})")
            continue
        color = cv2.imread(str(color_path))
        depth = cv2.imread(str(depth_path), cv2.IMREAD_UNCHANGED)

        npz = np.load(intr_dir / f"cam{cam_idx}.npz", allow_pickle=True)
        K = npz["color_K"]
        depth_scale = float(npz["depth_scale_m_per_unit"])

        if cam_idx == GRIPPER_CAM_IDX:
            # 그리퍼캠은 고정 위치가 없다 -- 이 캡처 순간의 flange 자세로 매번 다시 계산.
            T_cam_now = T_base_gripper_now @ T_gripper_cam
        else:
            T_cam_now = T_base_cam[cam_idx]

        points_cam, colors = backproject(depth, color, K, depth_scale, args.stride, args.max_depth_m,
                                          args.edge_thresh_mm)
        if args.per_camera_color:
            colors = np.tile(np.array(CAMERA_FLAT_COLORS[label], dtype=np.uint8), (len(points_cam), 1))
        points_base = transform_points(points_cam, T_cam_now)
        print(f"  {label} (cam{cam_idx}): {len(points_base)}개 점")
        all_points.append(points_base)
        all_colors.append(colors)

    points = np.concatenate(all_points, axis=0)
    colors = np.concatenate(all_colors, axis=0)
    write_ply(Path(args.out), points, colors)
    print(f"\n합쳐진 점 {len(points)}개 -> {args.out}")
    print("(MeshLab/CloudCompare 등으로 열어서 카메라들이 겹치는 영역이 잘 맞아떨어지는지 확인)")


if __name__ == "__main__":
    main()
