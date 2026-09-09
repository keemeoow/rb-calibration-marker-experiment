#!/usr/bin/env python
"""cam1(D415)로 지금 보이는 마커 큐브의 자세를 계산해 출력한다.

경로
    카메라가 본 큐브        T_cam_obj      <- solvePnP
    베이스 기준 큐브        T_base_obj = T_base_C1 @ T_cam_obj
    플랜지 기준 큐브        T_flange_obj = inv(T_base_flange) @ T_base_obj

플랜지 기준으로 바꾸려면 그 순간의 로봇 자세가 필요하다. 세 가지 방법이 있다.
  1) 로봇 서버가 떠 있으면 자동으로 받아온다 (--robot_ip)
  2) --flange "x y z rx ry rz" 로 직접 넣는다 (mm, 도)
  3) 아무것도 없으면 베이스 기준까지만 출력한다

실행 예:
    python live_marker_pose.py
    python live_marker_pose.py --flange "300 0 400 180 0 0"
    python live_marker_pose.py --calib data/session04/calib_out_nofk
"""
import os
import sys
import json
import socket
import argparse

import numpy as np
import cv2

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from apriltag_cube import AprilTagCubeTarget
from config import get_default_cube_config


# ── 좌표 변환 도구 ────────────────────────────────────────
def inv_T(T):
    R = T[:3, :3]
    out = np.eye(4)
    out[:3, :3] = R.T
    out[:3, 3] = -R.T @ T[:3, 3]
    return out


def _R(ax, t):
    c, s = np.cos(t), np.sin(t)
    return {"X": np.array([[1, 0, 0], [0, c, -s], [0, s, c]]),
            "Y": np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]]),
            "Z": np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])}[ax]


def pose6_to_T(vals):
    """로봇 6dof -> 4x4.

    이 로봇의 6dof 는 [x, y, z, Rz, Ry, Rx] 순서다. 네 번째 값이 Z축,
    여섯 번째 값이 X축 회전이며 R = Rz @ Ry @ Rx 로 조립된다.
    session04 meta 의 capture 91개에서 기록된 4x4 와 최대 1e-6 도로 일치함을
    확인했다. 길이는 mm, 각도는 도.
    """
    x, y, z, a_z, a_y, a_x = [float(v) for v in vals]
    T = np.eye(4)
    T[:3, :3] = (_R("Z", np.deg2rad(a_z)) @ _R("Y", np.deg2rad(a_y))
                 @ _R("X", np.deg2rad(a_x)))
    T[:3, 3] = np.array([x, y, z]) / 1000.0
    return T


def T_to_pose6(T):
    """4x4 -> 로봇 6dof [x, y, z, Rz, Ry, Rx] (mm, 도). pose6_to_T 의 역."""
    R = T[:3, :3]
    sy = -R[2, 0]
    a_y = np.arcsin(np.clip(sy, -1, 1))
    if abs(sy) < 0.9999:
        a_x = np.arctan2(R[2, 1], R[2, 2])
        a_z = np.arctan2(R[1, 0], R[0, 0])
    else:
        a_x = np.arctan2(-R[1, 2], R[1, 1])
        a_z = 0.0
    return np.array([T[0, 3] * 1000, T[1, 3] * 1000, T[2, 3] * 1000,
                     np.rad2deg(a_z), np.rad2deg(a_y), np.rad2deg(a_x)])


def fmt(T, label):
    p = T_to_pose6(T)
    print(f"  {label}")
    print(f"    위치 (mm)  x {p[0]:9.2f}   y {p[1]:9.2f}   z {p[2]:9.2f}")
    print(f"    자세 (도)  Rz {p[3]:8.2f}  Ry {p[4]:8.2f}  Rx {p[5]:8.2f}")


def get_flange_from_robot(ip, port, timeout=4.0):
    """로봇 서버에서 현재 플랜지 자세를 받아온다. 실패하면 None."""
    try:
        s = socket.socket()
        s.settimeout(timeout)
        s.connect((ip, port))
        s.sendall(b"GET_POSE\n")
        buf = s.recv(4096).decode(errors="ignore")
        s.close()
        d = json.loads(buf.strip().splitlines()[-1])
        for k in ("capture_gripper_pose_6dof", "flange", "pose"):
            if k in d:
                return np.asarray(d[k], dtype=float)[:6]
    except Exception as e:
        print(f"  (로봇 서버 연결 실패: {type(e).__name__})")
    return None


# ── 본체 ──────────────────────────────────────────────────
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--calib", default=os.path.join(HERE, "data/session04/calib_out"))
    ap.add_argument("--intr", default=os.path.join(HERE, "intrinsics/cam1.npz"))
    ap.add_argument("--cube_config", default=os.path.join(
        HERE, "data/session04/calib_train/meta.json"))
    ap.add_argument("--flange", default=None,
                    help='"x y z Rz Ry Rx" (mm, 도). 로봇 티치펜던트 표시 순서 그대로')
    ap.add_argument("--robot_ip", default="192.168.0.23")
    ap.add_argument("--robot_port", type=int, default=12348)
    ap.add_argument("--save", default=os.path.join(HERE, "live_cam1_view.jpg"))
    ap.add_argument("--warmup", type=int, default=20, help="자동노출 안정화 프레임 수")
    ap.add_argument("--frames", type=int, default=10,
                    help="여러 장을 찍어 중앙값을 쓴다. 마커가 1개만 보이면 "
                         "한 장짜리 PnP 가 수 mm 씩 흔들리므로 기본 10장.")
    a = ap.parse_args()

    # 1) 내부파라미터와 외부파라미터
    z = np.load(a.intr)
    K = z["color_K"].astype(np.float64)
    D = z["color_D"].astype(np.float64).ravel()
    serial = str(z["serial"])
    W, H = int(z["color_w"]), int(z["color_h"])
    T_base_C1 = np.load(os.path.join(a.calib, "T_base_C1.npy"))
    print(f"[설정] cam1 serial {serial}  {W}x{H}  |  외부파라미터 {a.calib}")

    # 2) 큐브 정의 (촬영 때 쓴 것과 같은 값을 meta 에서 가져온다)
    root = os.path.dirname(a.cube_config)
    try:
        from cube_config_utils import load_cube_config_from_meta
        cube_cfg, src = load_cube_config_from_meta(root,
                                                   default_cfg=get_default_cube_config())
    except Exception:
        cube_cfg, src = get_default_cube_config(), "config.py 기본값"
    print(f"[큐브] 정의 출처: {src}")
    target = AprilTagCubeTarget(cube_cfg)

    # 3) 카메라에서 한 장 받기
    import pyrealsense2 as rs
    pipe = rs.pipeline()
    cfg = rs.config()
    cfg.enable_device(serial)
    cfg.enable_stream(rs.stream.color, W, H, rs.format.bgr8, 15)
    pipe.start(cfg)
    shots, bgr = [], None
    try:
        for _ in range(a.warmup):          # 자동노출이 자리잡을 때까지 버린다
            pipe.wait_for_frames()
        for _ in range(max(a.frames, 1)):
            frames = pipe.wait_for_frames()
            shots.append(np.asanyarray(frames.get_color_frame().get_data()).copy())
    finally:
        pipe.stop()
    bgr = shots[-1]
    cv2.imwrite(a.save, bgr)
    print(f"[촬영] {len(shots)}장, 마지막 장 저장 {a.save}")

    # 4) 큐브 자세 추정 — 장마다 풀고 중앙값을 쓴다
    poses, useds, reprojs = [], [], []
    for img in shots:
        r = target.solve_pnp_cube(img, K, D, min_markers=1, return_reproj=True)
        if not r[0]:
            continue
        T = np.eye(4)
        T[:3, :3] = cv2.Rodrigues(np.asarray(r[1]).reshape(3, 1))[0]
        T[:3, 3] = np.asarray(r[2]).ravel()
        poses.append(T)
        useds.append(tuple(sorted(r[3])))
        reprojs.append(r[4] if len(r) > 4 else None)
    if not poses:
        print("[검출] 큐브를 찾지 못했다. 화면 안에 있는지, 조명이 충분한지 확인.")
        return 1

    P = np.array([T[:3, 3] for T in poses])
    T_cam_obj = np.eye(4)
    T_cam_obj[:3, 3] = np.median(P, axis=0)
    # 회전은 중앙값 위치에 가장 가까운 장의 것을 쓴다(각도 평균의 모호성 회피)
    T_cam_obj[:3, :3] = poses[int(np.argmin(
        np.linalg.norm(P - T_cam_obj[:3, 3], axis=1)))][:3, :3]
    spread = P.std(axis=0) * 1000
    print(f"[안정성] 성공 {len(poses)}/{len(shots)}장 | "
          f"위치 표준편차 (mm) x {spread[0]:.2f} y {spread[1]:.2f} z {spread[2]:.2f}")
    used = useds[0]
    reproj = reprojs[0]
    rp = None
    if isinstance(reproj, dict):
        for k in ("mean_px", "mean", "rms_px", "reproj_mean_px"):
            if isinstance(reproj.get(k), (int, float)):
                rp = float(reproj[k]); break
    elif isinstance(reproj, (int, float)):
        rp = float(reproj)
    print(f"[검출] 마커 {len(used)}개 사용 (id {sorted(used)})"
          + (f", 재투영 {rp:.2f}px" if rp is not None else ""))

    # 5) 좌표계 변환
    print()
    fmt(T_cam_obj, "카메라(cam1) 기준 큐브")
    T_base_obj = T_base_C1 @ T_cam_obj
    print()
    fmt(T_base_obj, "로봇 베이스 기준 큐브")

    flange6 = None
    if a.flange:
        flange6 = np.array([float(v) for v in a.flange.replace(",", " ").split()])
        print("\n[로봇] 입력받은 플랜지 자세 사용")
    else:
        flange6 = get_flange_from_robot(a.robot_ip, a.robot_port)
        if flange6 is not None:
            print("\n[로봇] 서버에서 플랜지 자세 수신")

    if flange6 is None:
        print("\n[플랜지] 로봇 자세를 몰라 변환할 수 없다.")
        print("         --flange \"x y z rx ry rz\" 로 넣거나 로봇 서버를 띄우면 된다.")
        return 0

    T_base_flange = pose6_to_T(flange6)
    T_flange_obj = inv_T(T_base_flange) @ T_base_obj
    print()
    fmt(T_base_flange, "베이스 기준 플랜지 (입력)")
    print()
    fmt(T_flange_obj, "★ 플랜지 기준 큐브")
    print(f"\n  플랜지에서 큐브까지 거리: "
          f"{np.linalg.norm(T_flange_obj[:3, 3]) * 1000:.1f} mm")
    return 0


if __name__ == "__main__":
    sys.exit(main())
