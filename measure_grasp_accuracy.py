#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""measure_grasp_accuracy.py — 캘리브레이션 이후 "실제로 얼마나 정확히 잡는지"를
자 눈금이 있는 마커 큐브로 수치화한다. PYTHON 3. server 는 server/zeus_server.py 를
그대로 띄우면 되고(수정 불필요), 인지·좌표변환·목표계산·오차계산은 전부 여기 client 에 있다.

절차 (server/zeus_server.py 의 get_state/movel/grip 만 사용)
    1) record-init   지금 로봇 자세를 "인지 대기 자세"로 1회 저장한다.
    2) record-gt     사람이 큐브를 손으로 들어 자 눈금의 중점이 그리퍼가 실제로
                      무는 지점에 오도록 맞춘 뒤, 이 명령으로 FK 를 읽어 그 순간의
                      "진짜" 큐브 중점을 base 좌표로 계산해 저장한다(정답값, GT).
    3) run-trial     init 자세로 이동 -> 카메라로 큐브 인지(AprilTag PnP) ->
                      calibration_use.Calibration 으로 base 좌표 변환 -> 그 좌표를
                      잡으러 이동 -> grip close -> 예측 중점과 GT 중점의 차이(mm)를
                      계산해 기록한다. --n_trials 로 반복 가능.
    4) report        trials.jsonl 을 모아 평균/표준편차/RMSE 를 낸다.

좌표계·오프셋 규약은 grasp_target.py / server/c1.py 와 동일하다:
    큐브 중심 좌표는 calibration_pipeline.config.CubeConfig 정의상 이미 "큐브 중심"이라
    카메라 인지값(T_cam_obj)의 위치를 그대로 쓰면 된다. 반면 사람이 손으로 맞추는 GT 는
    로봇이 읽는 TCP(get_state, tool1 = flange 에서 97.5mm 오프셋) 좌표이므로, 거기서
    핑거팁까지(FINGERTIP_Z_MM) + 핑거팁에서 큐브중심까지(cube_side/2 - grip_depth) 만큼
    아래로 내려야 큐브 중심이 나온다. 두 계산이 얼마나 벌어지는지가 곧 오차다.

실행 예:
    python measure_grasp_accuracy.py record-init --robot_ip 192.168.0.23
    python measure_grasp_accuracy.py record-gt --label center1
    python measure_grasp_accuracy.py run-trial --n_trials 5
    python measure_grasp_accuracy.py report
"""
import argparse
import json
import os
import sys
import time
from typing import Optional

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

from robot.backends.zeus_client import ZeusClient
from calibration_pipeline.apriltag_cube import AprilTagCubeTarget
from calibration_pipeline.config import get_default_cube_config
from calibration_use import Calibration

OUT_DIR = os.path.join(HERE, "grasp_accuracy_runs")
INIT_POSE_PATH = os.path.join(OUT_DIR, "init_pose.json")
GT_PATH = os.path.join(OUT_DIR, "gt_records.jsonl")
TRIALS_PATH = os.path.join(OUT_DIR, "trials.jsonl")
IMAGES_DIR = os.path.join(OUT_DIR, "images")

DEFAULT_ROBOT_IP = "192.168.0.23"

# ── 그립 기하 상수. grasp_target.py / server/c1.py 실측값과 동일하며 CLI 로 덮어쓸 수 있다. ──
FINGERTIP_Z_MM = 115.5     # tool1(get_state) 판독값 -> 핑거팁. server/c1.py:53
CUBE_SIDE_MM = 59.0        # calibration_pipeline/config.py CubeConfig.cube_side_m
CUBE_GRIP_DEPTH_MM = 2.0   # 핑거팁이 큐브 윗면에서 얼마나 아래까지 무는지. server/c1.py 실측값


def center_offset_mm(fingertip_mm: float, cube_side_mm: float, grip_depth_mm: float) -> float:
    """tool1(get_state) z 판독값에서 이 값을 빼면 큐브 중심 z 가 나온다."""
    return fingertip_mm + cube_side_mm / 2.0 - grip_depth_mm


# ── 좌표 유틸 ─────────────────────────────────────────────────────────
def now_iso() -> str:
    return time.strftime("%Y-%m-%dT%H:%M:%S")


def ensure_out_dir():
    os.makedirs(OUT_DIR, exist_ok=True)
    os.makedirs(IMAGES_DIR, exist_ok=True)


def append_jsonl(path: str, record: dict):
    ensure_out_dir()
    with open(path, "a") as f:
        f.write(json.dumps(record) + "\n")


def read_jsonl(path: str):
    if not os.path.exists(path):
        return []
    out = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if line:
                out.append(json.loads(line))
    return out


def find_up_axis_and_yaw(T_base_obj: np.ndarray, current_rz_deg: float, symmetry_deg: float):
    """grasp_target.py 의 "어느 축이 위인지 찾고, 대칭각으로 요 스냅" 로직을 그대로 옮김."""
    R = T_base_obj[:3, :3]
    up = int(np.argmax(np.abs(R[2, :])))
    tilt_deg = float(np.rad2deg(np.arccos(min(abs(R[2, up]), 1.0))))
    horiz = [i for i in range(3) if i != up]

    azims = [np.rad2deg(np.arctan2(R[1, i], R[0, i])) for i in horiz]
    zc = np.mean([np.exp(1j * np.deg2rad(x * 4.0)) for x in azims])
    grid = np.rad2deg(np.angle(zc)) / 4.0
    spread_deg = float(abs(((azims[0] - azims[1] + 45.0) % 90.0) - 45.0))

    step = symmetry_deg
    if step <= 90.0 + 1e-9:
        base = grid
    else:
        kk = np.arange(-4, 5)
        base = float((grid + 90.0 * kk)[np.argmin(np.abs(grid + 90.0 * kk - azims[0]))])
    k = np.arange(-int(360 / step) - 1, int(360 / step) + 2)
    cands = base + step * k
    rz_snapped = float(cands[np.argmin(np.abs(cands - current_rz_deg))])
    return rz_snapped, tilt_deg, spread_deg


def grasp_target_pose6(cube_center_base_mm: np.ndarray, rz_deg: float, offset_mm: float) -> np.ndarray:
    """큐브 중심(base, mm) -> tool1 목표 pose6. Ry=0, Rx=180 (top-down)."""
    cx, cy, cz = [float(v) for v in cube_center_base_mm]
    return np.array([cx, cy, cz + offset_mm, rz_deg, 0.0, 180.0])


# ── record-init / record-gt ──────────────────────────────────────────
def cmd_record_init(args):
    ensure_out_dir()
    with ZeusClient(args.robot_ip, args.robot_port) as rb:
        state = rb.get_state()
    record = {"pose": state["pose"], "joints": state["joints"], "recorded_at": now_iso()}
    with open(INIT_POSE_PATH, "w") as f:
        json.dump(record, f, indent=2)
    print(f"[record-init] 저장: {INIT_POSE_PATH}")
    print(f"  pose (mm/deg) = {['%.2f' % v for v in record['pose']]}")
    print(f"  joints (deg)  = {['%.2f' % v for v in record['joints']]}")


def cmd_record_gt(args):
    offset = center_offset_mm(args.fingertip_z_mm, args.cube_side_mm, args.grip_depth_mm)
    with ZeusClient(args.robot_ip, args.robot_port) as rb:
        pose6 = rb.get_pose6()
    center = np.array([pose6[0], pose6[1], pose6[2] - offset])
    record = {
        "label": args.label,
        "recorded_at": now_iso(),
        "tool_pose6": pose6.tolist(),
        "center_offset_mm": offset,
        "T_base_gt_center_mm": center.tolist(),
    }
    append_jsonl(GT_PATH, record)
    print(f"[record-gt] label={args.label!r}  저장: {GT_PATH}")
    print(f"  TCP 판독값 (mm/deg) = {['%.2f' % v for v in pose6]}")
    print(f"  큐브중심 오프셋 = {offset:.2f} mm (핑거팁 {args.fingertip_z_mm} "
          f"+ 큐브변/2 {args.cube_side_mm/2:.1f} - 무는깊이 {args.grip_depth_mm})")
    print(f"  -> GT 큐브중심 (base, mm) = x {center[0]:.2f}  y {center[1]:.2f}  z {center[2]:.2f}")


def load_latest_gt(label: Optional[str]) -> dict:
    records = read_jsonl(GT_PATH)
    if label:
        records = [r for r in records if r.get("label") == label]
    if not records:
        raise SystemExit(f"[오류] GT 기록이 없다 (label={label!r}). "
                          f"먼저 `record-gt` 를 실행할 것: {GT_PATH}")
    return records[-1]


def load_init_pose() -> dict:
    if not os.path.exists(INIT_POSE_PATH):
        raise SystemExit(f"[오류] 인지 대기 자세가 없다. 먼저 `record-init` 을 실행할 것: "
                          f"{INIT_POSE_PATH}")
    with open(INIT_POSE_PATH) as f:
        return json.load(f)


# ── 인지 ──────────────────────────────────────────────────────────────
def detect_cube_center_base(args, calib: Calibration, target: AprilTagCubeTarget, trial_dir: str):
    """cam1(기본) 또는 --cam3/--fuse 로 큐브를 인지해 base 좌표 4x4 를 돌려준다.

    live_marker_pose.py 와 같은 방식: 여러 장을 찍어 위치는 중앙값, 자세는 그 중앙값에
    가장 가까운 한 장을 쓴다(회전 평균의 모호성 회피). 상세 진단은 반환 dict 에 담는다.
    """
    import cv2
    import pyrealsense2 as rs

    cams = ["cam1", "cam3"] if args.fuse else [args.cam]
    observations = {}
    diag = {}
    for cam_name in cams:
        cam_id = int(cam_name.replace("cam", ""))
        intr_path = os.path.join(HERE, "intrinsics", f"{cam_name}.npz")
        z = np.load(intr_path)
        K = z["color_K"].astype(np.float64)
        D = z["color_D"].astype(np.float64).ravel()
        serial = str(z["serial"])
        W, H = int(z["color_w"]), int(z["color_h"])

        pipe = rs.pipeline()
        cfg = rs.config()
        cfg.enable_device(serial)
        cfg.enable_stream(rs.stream.color, W, H, rs.format.bgr8, 15)
        pipe.start(cfg)
        shots = []
        try:
            for _ in range(args.warmup):
                pipe.wait_for_frames()
            for _ in range(max(args.frames, 1)):
                frames = pipe.wait_for_frames()
                shots.append(np.asanyarray(frames.get_color_frame().get_data()).copy())
        finally:
            pipe.stop()

        if trial_dir:
            cv2.imwrite(os.path.join(trial_dir, f"{cam_name}.jpg"), shots[-1])

        poses = []
        used_ids = None
        for img in shots:
            ok, rvec, tvec, used, reproj = target.solve_pnp_cube(
                img, K, D, min_markers=1, return_reproj=True)
            if not ok:
                continue
            T = np.eye(4)
            T[:3, :3] = cv2.Rodrigues(np.asarray(rvec).reshape(3, 1))[0]
            T[:3, 3] = np.asarray(tvec).ravel()
            poses.append(T)
            used_ids = used
        if not poses:
            print(f"  [{cam_name}] 큐브를 찾지 못했다 ({len(shots)}장 시도)")
            continue

        P = np.array([T[:3, 3] for T in poses])
        T_cam_obj = np.eye(4)
        T_cam_obj[:3, 3] = np.median(P, axis=0)
        T_cam_obj[:3, :3] = poses[int(np.argmin(
            np.linalg.norm(P - T_cam_obj[:3, 3], axis=1)))][:3, :3]
        spread_mm = (P.std(axis=0) * 1000).tolist()

        observations[cam_id] = T_cam_obj
        diag[cam_name] = {
            "n_success": len(poses), "n_shots": len(shots),
            "used_marker_ids": sorted(int(x) for x in used_ids) if used_ids else [],
            "position_std_mm": spread_mm,
        }
        print(f"  [{cam_name}] 성공 {len(poses)}/{len(shots)}장 | "
              f"마커 {diag[cam_name]['used_marker_ids']} | "
              f"위치 표준편차(mm) {['%.2f' % v for v in spread_mm]}")

    if not observations:
        raise SystemExit("[오류] 어느 카메라에서도 큐브를 인지하지 못했다.")

    if len(observations) > 1:
        center_base_m = calib.fuse(observations)
        T_base_obj = np.eye(4)
        T_base_obj[:3, 3] = center_base_m
        first_cam = next(iter(observations))
        T_base_obj[:3, :3] = (calib.to_base_pose(first_cam, observations[first_cam])[:3, :3])
    else:
        cam_id, T_cam_obj = next(iter(observations.items()))
        T_base_obj = calib.to_base_pose(cam_id, T_cam_obj)

    return T_base_obj, diag


# ── run-trial ─────────────────────────────────────────────────────────
def run_one_trial(args, rb: ZeusClient, calib: Calibration, target: AprilTagCubeTarget,
                  gt_center_mm: np.ndarray, trial_idx: int) -> dict:
    ensure_out_dir()
    init = load_init_pose()
    print(f"\n[trial {trial_idx}] 인지 대기 자세로 이동")
    rb.movej(init["joints"], jnt_speed=args.jnt_speed)
    cur_pose6 = rb.get_pose6()

    trial_dir = os.path.join(IMAGES_DIR, f"trial_{trial_idx:03d}")
    os.makedirs(trial_dir, exist_ok=True)

    print(f"[trial {trial_idx}] 큐브 인지 중...")
    T_base_obj_pred, cam_diag = detect_cube_center_base(args, calib, target, trial_dir)
    pred_center_mm = T_base_obj_pred[:3, 3] * 1000.0

    offset = center_offset_mm(args.fingertip_z_mm, args.cube_side_mm, args.grip_depth_mm)
    rz_snapped, tilt_deg, spread_deg = find_up_axis_and_yaw(
        T_base_obj_pred, current_rz_deg=cur_pose6[3], symmetry_deg=args.symmetry)
    if tilt_deg > 15.0:
        print(f"  [경고] 큐브가 %.1f도 기울어 보인다. 잘못 인지했을 수 있다." % tilt_deg)
    if spread_deg > 3.0:
        print(f"  [경고] 수평축 대칭성이 {spread_deg:.1f}도 어긋난다.")

    target_pose6 = grasp_target_pose6(pred_center_mm, rz_snapped, offset)
    approach_pose6 = target_pose6.copy()
    approach_pose6[2] += args.approach_z_mm

    print(f"[trial {trial_idx}] 예측 큐브중심(base,mm) = "
          f"x {pred_center_mm[0]:.2f} y {pred_center_mm[1]:.2f} z {pred_center_mm[2]:.2f}")
    print(f"[trial {trial_idx}] 목표 tool 자세 = {['%.2f' % v for v in target_pose6]}")

    rb.grip("open", timeout_s=args.grip_timeout_s)
    print(f"[trial {trial_idx}] 접근 (xy·회전만, 높이 {args.approach_z_mm}mm 위)")
    rb.movel(approach_pose6, lin_speed=args.lin_speed)
    print(f"[trial {trial_idx}] 수직 하강")
    rb.movel(target_pose6, lin_speed=args.descend_speed)

    held = rb.grip_holds_object(timeout_s=args.grip_timeout_s)
    reached_pose6 = rb.get_pose6()
    print(f"[trial {trial_idx}] grip close -> {'물체 감지됨 (성공)' if held else '아무것도 안 잡힘 (실패)'}")

    error_mm = pred_center_mm - gt_center_mm
    error_norm_mm = float(np.linalg.norm(error_mm))
    print(f"[trial {trial_idx}] 오차 (예측 - GT, base mm) = "
          f"dx {error_mm[0]:+.2f} dy {error_mm[1]:+.2f} dz {error_mm[2]:+.2f}  "
          f"|e| {error_norm_mm:.2f} mm")

    lift_pose6 = reached_pose6.copy()
    lift_pose6[2] += args.approach_z_mm
    rb.movel(lift_pose6, lin_speed=args.lin_speed)

    record = {
        "trial_idx": trial_idx,
        "timestamp": now_iso(),
        "held_object": held,
        "cam_diag": cam_diag,
        "tilt_deg": tilt_deg,
        "horizontal_symmetry_spread_deg": spread_deg,
        "T_base_pred_center_mm": pred_center_mm.tolist(),
        "T_base_gt_center_mm": gt_center_mm.tolist(),
        "target_tool_pose6": target_pose6.tolist(),
        "reached_tool_pose6": reached_pose6.tolist(),
        "error_xyz_mm": error_mm.tolist(),
        "error_norm_mm": error_norm_mm,
        "center_offset_mm": offset,
        "symmetry_deg": args.symmetry,
    }
    append_jsonl(TRIALS_PATH, record)

    if args.auto_replace and trial_idx < args.n_trials:
        print(f"[trial {trial_idx}] 다음 시도를 위해 같은 자리에 내려놓는다")
        rb.movel(target_pose6, lin_speed=args.descend_speed)
        rb.grip("open", timeout_s=args.grip_timeout_s)
        rb.movel(approach_pose6, lin_speed=args.lin_speed)

    return record


def cmd_run_trial(args):
    gt = load_latest_gt(args.gt_label)
    gt_center_mm = np.array(gt["T_base_gt_center_mm"])
    print(f"[run-trial] 사용할 GT: label={gt.get('label')!r} recorded_at={gt.get('recorded_at')}")
    print(f"  GT 큐브중심(base,mm) = {['%.2f' % v for v in gt_center_mm]}")

    calib = Calibration(calib_dir=args.calib_dir)
    target = AprilTagCubeTarget(get_default_cube_config())

    if args.n_trials > 1 and not args.auto_replace:
        raise SystemExit("[오류] --n_trials > 1 이면 --auto_replace 를 같이 줘야 한다 "
                          "(매 시도 사이에 사람이 다시 놔줄 게 아니라면).")

    records = []
    with ZeusClient(args.robot_ip, args.robot_port) as rb:
        for i in range(1, args.n_trials + 1):
            records.append(run_one_trial(args, rb, calib, target, gt_center_mm, i))
            if i < args.n_trials:
                time.sleep(args.between_s)

    errs = np.array([r["error_norm_mm"] for r in records])
    print(f"\n[run-trial] {len(records)}회 완료. |오차| 평균 {errs.mean():.2f} mm, "
          f"최대 {errs.max():.2f} mm")


# ── report ────────────────────────────────────────────────────────────
def cmd_report(args):
    records = read_jsonl(TRIALS_PATH)
    if not records:
        print(f"[report] 기록이 없다: {TRIALS_PATH}")
        return
    err = np.array([r["error_xyz_mm"] for r in records])
    norm = np.array([r["error_norm_mm"] for r in records])
    held = [r.get("held_object") for r in records]

    print(f"[report] {len(records)}회 시도  ({TRIALS_PATH})")
    print(f"  물체 감지 성공 = {sum(1 for h in held if h)}/{len(held)}")
    for i, axis in enumerate("xyz"):
        print(f"  {axis} 오차(mm)  평균 {err[:, i].mean():+7.2f}  표준편차 {err[:, i].std():6.2f}  "
              f"최대|값| {np.abs(err[:, i]).max():6.2f}")
    rmse = float(np.sqrt(np.mean(norm ** 2)))
    print(f"  |오차| 평균 {norm.mean():.2f} mm  표준편차 {norm.std():.2f}  "
          f"최대 {norm.max():.2f}  RMSE {rmse:.2f}")
    ok = norm.mean() < args.tol_mm
    print(f"  결과: {'PASS' if ok else 'FAIL'} (기준: 평균 |오차| < {args.tol_mm}mm)")


# ── CLI ───────────────────────────────────────────────────────────────
def add_common_robot_args(p):
    p.add_argument("--robot_ip", default=DEFAULT_ROBOT_IP)
    p.add_argument("--robot_port", type=int, default=12350)


def add_geometry_args(p):
    p.add_argument("--fingertip_z_mm", type=float, default=FINGERTIP_Z_MM)
    p.add_argument("--cube_side_mm", type=float, default=CUBE_SIDE_MM)
    p.add_argument("--grip_depth_mm", type=float, default=CUBE_GRIP_DEPTH_MM)


def main():
    ap = argparse.ArgumentParser(
        formatter_class=argparse.RawDescriptionHelpFormatter, description=__doc__)
    sub = ap.add_subparsers(dest="cmd", required=True)

    p = sub.add_parser("record-init", help="지금 로봇 자세를 인지 대기 자세로 저장")
    add_common_robot_args(p)
    p.set_defaults(func=cmd_record_init)

    p = sub.add_parser("record-gt", help="사람이 맞춘 큐브 위치를 GT 로 기록")
    add_common_robot_args(p)
    add_geometry_args(p)
    p.add_argument("--label", default="default")
    p.set_defaults(func=cmd_record_gt)

    p = sub.add_parser("run-trial", help="인지->이동->grasp->오차계산 을 실행")
    add_common_robot_args(p)
    add_geometry_args(p)
    p.add_argument("--calib_dir", default=os.path.join(HERE, "data/session02/calib_final_use"))
    p.add_argument("--cam", default="cam1", choices=["cam1", "cam3"])
    p.add_argument("--fuse", action="store_true", help="cam1+cam3를 함께 써서 median 융합")
    p.add_argument("--gt_label", default=None, help="쓸 GT 의 label. 생략하면 가장 최근 GT")
    p.add_argument("--warmup", type=int, default=20)
    p.add_argument("--frames", type=int, default=10)
    p.add_argument("--symmetry", type=float, default=90.0,
                   help="단면 회전 대칭 각도. 정사각 큐브는 90 (기본)")
    p.add_argument("--approach_z_mm", type=float, default=80.0,
                   help="목표 위 이만큼에서 먼저 xy·회전 정렬 후 수직 하강")
    p.add_argument("--lin_speed", type=float, default=60.0)
    p.add_argument("--descend_speed", type=float, default=25.0)
    p.add_argument("--jnt_speed", type=float, default=15.0)
    p.add_argument("--grip_timeout_s", type=float, default=3.0)
    p.add_argument("--n_trials", type=int, default=1)
    p.add_argument("--auto_replace", action="store_true",
                   help="매 시도 뒤 같은 자리에 자동으로 내려놓고 다음 시도로 넘어간다")
    p.add_argument("--between_s", type=float, default=2.0)
    p.set_defaults(func=cmd_run_trial)

    p = sub.add_parser("report", help="trials.jsonl 통계")
    p.add_argument("--tol_mm", type=float, default=5.0)
    p.set_defaults(func=cmd_report)

    args = ap.parse_args()
    args.func(args)


if __name__ == "__main__":
    main()
