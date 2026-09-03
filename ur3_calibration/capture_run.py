#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ur3_calibration/capture_run.py -- 카메라 4대 + 로봇으로 세션 1/2/3 실제 촬영

고정 카메라 3대 + 그리퍼 카메라 1대(RealSense, 총 4대)를 연결하고, 각
세션에 저장된 목표 자세를 순서대로 방문하면서 모든 카메라의 컬러 프레임과
로봇 pose/joint를 함께 저장한다.

세션 1/3: capture_poses.py로 이미 기록한 poses.json의 각 자세로 이동한 뒤
그 자리에서 촬영만 하면 된다 (그리퍼는 건드리지 않음 -- 세션1은 사람이
미리 큐브를 쥐어준 상태로 진행, 세션3은 그리퍼가 비어 있는 손목 스윕).

세션 2: session2_pick_and_place.py 와 완전히 동일한 pick-and-place
계획(build_plan/execute_plan)을 그대로 수행하되, "촬영 자리" 스텝에서
실제로 4대 카메라 + 로봇 데이터를 저장한다. 첫 pick 직전에는 사람이
큐브를 정확히 놓을 때까지 기다리는 필수 확인 지점이 그대로 포함된다.

카메라 라벨은 모델로 자동 인식한다 -- 그리퍼캠은 D435(IMU 없는 모델),
고정캠 3대는 D435I. 필요하면 --camera-label SERIAL=이름 으로 덮어쓸 수
있다.

--execute 시 4대를 2x2로 붙인 미리보기 창을 하나 띄운다(--no-preview로
끄기 가능). 로봇이 움직이는 동안에도 백그라운드 스레드에서 계속
갱신되며, 실제 저장되는 캡처와는 별개로 확인용이다.

출력: data/session{N}_.../capture/<index:03d>/
  cam_<라벨 또는 serial>.png       (컬러)
  cam_<라벨 또는 serial>_depth.png (컬러에 정렬된 16bit depth, mm, z16)
  robot.json                       (tcp_pose, joint_radians/degrees, timestamp)

사용법:
  python capture_run.py --session 1                          # dry-run
  python capture_run.py --session 1 --execute --no-step
  python capture_run.py --session 2 --execute --no-step
  python capture_run.py --session 3 --execute
  python capture_run.py --session 2 --execute --camera-label 123456789=gripper
"""

import argparse
import json
import sys
import threading
import time
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from capture_pipeline.camera import RealSenseCamera  # noqa: E402

import rtde_control
import rtde_receive

from capture_poses import SESSIONS, session_path
from session2_pick_and_place import (
    ROBOT_IP_DEFAULT, GRASP_POSE_DEFAULT, CAM_POSE_DEFAULT,
    APPROACH_MM_DEFAULT, PICK_LIFT_MM_DEFAULT, PLACE_LIFT_MM_DEFAULT,
    RELEASE_STEP_DEFAULT, IK_MAX_POS_ERROR, IK_MAX_ORI_ERROR,
    JOINT_SPEED, JOINT_ACCEL,
    load_pose, compute_ordered_targets, build_plan, mark_risky_moveL_as_moveJ,
    execute_plan, call_with_retry, fmt_pose,
)

CAM_WIDTH, CAM_HEIGHT, CAM_FPS = 1280, 720, 15
MOVE_SPEED = 0.10
MOVE_ACCEL = 0.30
SETTLE_S = 0.3
ROTATION_JUMP_THRESHOLD = 1.0


def parse_labels(pairs):
    labels = {}
    for item in pairs or []:
        if "=" in item:
            serial, name = item.split("=", 1)
            labels[serial] = name
    return labels


def is_gripper_cam_name(name: str) -> bool:
    """그리퍼 카메라는 D435(IMU 없는 모델)이고, 고정 카메라들은 D435I다.
    'D435'는 'D435I'의 부분 문자열이라 I가 붙었는지까지 확인해야 한다."""
    upper = name.upper()
    return "D435" in upper and "D435I" not in upper


def auto_labels(devices: dict, user_labels: dict) -> dict:
    """시리얼 -> 라벨. --camera-label로 준 값이 최우선, 나머지는 이름으로
    그리퍼캠(D435, non-I)을 자동 인식하고 남은 건 fixed1, fixed2... 로 붙인다."""
    labels = dict(user_labels)
    fixed_n = 1
    for serial, name in sorted(devices.items()):
        if serial in labels:
            continue
        if is_gripper_cam_name(name):
            labels[serial] = "gripper"
        else:
            labels[serial] = f"fixed{fixed_n}"
            fixed_n += 1
    return labels


def connect_cameras(no_reset=False):
    if not no_reset:
        RealSenseCamera.reset_all_devices()
    devices = RealSenseCamera.list_devices()
    if not devices:
        raise RuntimeError("연결된 RealSense 카메라가 없습니다 (rs-enumerate-devices로 확인).")
    print(f"카메라 {len(devices)}대 발견:")
    cams = {}
    for serial, name in devices.items():
        print(f"  {serial}  {name}")
        cam = RealSenseCamera(serial, width=CAM_WIDTH, height=CAM_HEIGHT, fps=CAM_FPS,
                               use_color=True, use_depth=True, align_depth_to_color=True,
                               lock_color_exposure=False)
        cam.start()
        cams[serial] = cam
    return cams, devices


def stop_cameras(cams):
    for cam in cams.values():
        cam.stop()


class LiveView:
    """4대 카메라를 2x2로 붙여 한 창에 계속 보여주는 백그라운드 미리보기.

    로봇 이동(moveL/moveJ)이 메인 스레드를 블로킹하는 동안에도 화면이
    계속 갱신되도록 별도 스레드에서 돈다. 실제 저장되는 캡처(save_capture)
    와는 무관한 확인용 라이브 뷰다.
    """

    TILE_W, TILE_H = 640, 360

    def __init__(self, cams: dict, labels: dict, window_name: str = "capture_run (q/ESC=닫기)"):
        self.cams = cams
        self.labels = labels
        self.window_name = window_name
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._loop, daemon=True)

    def start(self):
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        self._thread.start()

    def _loop(self):
        order = sorted(self.cams.keys(), key=lambda s: self.labels.get(s, s))
        while not self._stop.is_set():
            tiles = []
            for serial in order:
                color, _depth, _ts = self.cams[serial].get_latest()
                label = self.labels.get(serial, serial)
                if color is None:
                    tile = np.zeros((self.TILE_H, self.TILE_W, 3), dtype=np.uint8)
                else:
                    tile = cv2.resize(color, (self.TILE_W, self.TILE_H))
                cv2.putText(tile, label, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
                tiles.append(tile)
            while len(tiles) < 4:
                tiles.append(np.zeros((self.TILE_H, self.TILE_W, 3), dtype=np.uint8))
            grid = np.vstack([np.hstack(tiles[0:2]), np.hstack(tiles[2:4])])
            cv2.imshow(self.window_name, grid)
            key = cv2.waitKey(30) & 0xFF
            if key in (ord("q"), 27):
                self._stop.set()
        cv2.destroyWindow(self.window_name)

    def stop(self):
        self._stop.set()
        self._thread.join(timeout=2.0)


def read_robot_state(rtde_r, extra=None):
    joints = list(rtde_r.getActualQ())
    tcp = list(rtde_r.getActualTCPPose())
    state = {
        "timestamp": time.strftime("%Y-%m-%dT%H:%M:%S"),
        "tcp_pose": tcp,
        "joint_radians": joints,
        "joint_degrees": [float(np.degrees(v)) for v in joints],
    }
    if extra:
        state.update(extra)
    return state


def save_capture(cams, labels, out_dir: Path, robot_state: dict):
    """컬러 + depth(정렬된 16bit mm, z16) + 로봇 상태를 저장한다."""
    out_dir.mkdir(parents=True, exist_ok=True)
    saved = []
    for serial, cam in cams.items():
        color, depth, _ts = cam.get_latest()
        if color is None:
            print(f"  [WARN] cam {serial}: 프레임 없음, 건너뜀")
            continue
        name = labels.get(serial, serial)
        cv2.imwrite(str(out_dir / f"cam_{name}.png"), color)
        if depth is not None:
            cv2.imwrite(str(out_dir / f"cam_{name}_depth.png"), depth)
        else:
            print(f"  [WARN] cam {serial} ({name}): depth 프레임 없음")
        saved.append(name)
    (out_dir / "robot.json").write_text(json.dumps(robot_state, indent=2, ensure_ascii=False) + "\n")
    return saved


def run_simple_session(session_id, args, cams, labels):
    """세션 1/3: poses.json 순서대로 이동 후 촬영 (그리퍼는 건드리지 않음)."""
    info = SESSIONS[session_id]
    poses_path = session_path(Path(__file__).resolve().parent / "data", session_id)
    poses = json.loads(poses_path.read_text())["poses"]
    out_root = poses_path.parent / "capture"

    print(f"=== 세션 {session_id}: {info['name']} ===  {len(poses)}개 자세")
    print(info["description"])

    if not args.execute:
        for idx, p in enumerate(poses):
            print(f"[{idx + 1}/{len(poses)}] {fmt_pose(p['tcp_pose'])}")
        print("\n(dry-run) 실제로 움직이거나 촬영하지 않았습니다. --execute 를 주면 실행합니다.")
        return

    rtde_c = rtde_control.RTDEControlInterface(args.robot_ip)
    rtde_r = rtde_receive.RTDEReceiveInterface(args.robot_ip)
    time.sleep(1.0)
    try:
        last_pose = None
        for idx, p in enumerate(poses):
            if idx < args.skip_steps:
                last_pose = list(p["tcp_pose"])
                continue
            target = list(p["tcp_pose"])
            if not args.no_step:
                cmd = input(f"\n[{idx + 1}/{len(poses)}] 이동+촬영. Enter=진행 / q=중단 > ").strip().lower()
                if cmd == "q":
                    print("중단했습니다.")
                    break
            else:
                print(f"[{idx + 1}/{len(poses)}] 이동+촬영")

            risky = last_pose is not None and np.linalg.norm(
                np.array(target[3:6]) - np.array(last_pose[3:6])) > ROTATION_JUMP_THRESHOLD
            if risky:
                q_near = rtde_r.getActualQ()
                try:
                    q_target = call_with_retry(rtde_c.getInverseKinematics, target, q_near,
                                                IK_MAX_POS_ERROR, IK_MAX_ORI_ERROR)
                except RuntimeError as exc:
                    print(f"  [ERROR] IK 계산 실패 -- 중단: {exc}")
                    break
                rtde_c.moveJ(q_target, JOINT_SPEED, JOINT_ACCEL)
            else:
                rtde_c.moveL(target, MOVE_SPEED, MOVE_ACCEL)
            last_pose = target
            time.sleep(SETTLE_S)

            robot_state = read_robot_state(rtde_r, {"pose_index": idx})
            out_dir = out_root / f"{idx:03d}"
            saved = save_capture(cams, labels, out_dir, robot_state)
            print(f"  저장됨 -> {out_dir}  (카메라 {len(saved)}대: {', '.join(saved)})")
    finally:
        rtde_c.stopScript()
        rtde_c.disconnect()
        rtde_r.disconnect()


def run_session2(args, cams, labels):
    grasp_pose = load_pose(Path(args.grasp_pose))
    cam_pose = load_pose(Path(args.cam_pose))
    session_poses = session_path(Path(__file__).resolve().parent / "data", 2)
    items = compute_ordered_targets(session_poses, grasp_pose[3:6], grasp_pose[2])

    if args.release_pos is None:
        direction = -1 if args.close_pos > args.open_pos else 1
        release_pos = int(np.clip(args.close_pos + direction * RELEASE_STEP_DEFAULT, 0, 255))
    else:
        release_pos = args.release_pos

    steps = build_plan(items, grasp_pose, cam_pose, args.approach_mm, args.pick_lift_mm, args.place_lift_mm,
                        args.open_pos, args.close_pos, release_pos, args.return_home)
    mark_risky_moveL_as_moveJ(steps)

    out_root = session_poses.parent / "capture"
    capture_counter = {"n": 0}

    def on_capture(step, index, rtde_c, rtde_r):
        time.sleep(SETTLE_S)
        robot_state = read_robot_state(
            rtde_r, {"capture_index": capture_counter["n"], "step_index": index}
        )
        out_dir = out_root / f"{capture_counter['n']:03d}"
        saved = save_capture(cams, labels, out_dir, robot_state)
        print(f"  저장됨 -> {out_dir}  (카메라 {len(saved)}대: {', '.join(saved)})")
        capture_counter["n"] += 1

    if not args.execute:
        for i, step in enumerate(steps):
            print(f"[{i + 1:3d}] {step['kind']:12s} {step['desc']}")
        print(f"\n(dry-run) 총 {len(steps)}스텝. --execute 를 주면 실행합니다.")
        return

    execute_plan(steps, args.robot_ip, skip_steps=args.skip_steps, no_step=args.no_step,
                 on_capture=on_capture)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--session", type=int, required=True, choices=(1, 2, 3))
    ap.add_argument("--robot-ip", default=ROBOT_IP_DEFAULT)
    ap.add_argument("--grasp-pose", default=str(GRASP_POSE_DEFAULT))
    ap.add_argument("--cam-pose", default=str(CAM_POSE_DEFAULT))
    ap.add_argument("--approach-mm", type=float, default=APPROACH_MM_DEFAULT)
    ap.add_argument("--pick-lift-mm", type=float, default=PICK_LIFT_MM_DEFAULT)
    ap.add_argument("--place-lift-mm", type=float, default=PLACE_LIFT_MM_DEFAULT)
    ap.add_argument("--open-pos", type=int, default=0)
    ap.add_argument("--close-pos", type=int, default=150)
    ap.add_argument("--release-pos", type=int, default=None)
    ap.add_argument("--return-home", action="store_true")
    ap.add_argument("--skip-steps", type=int, default=0,
                    help="이미 실행된 스텝/포즈 수 -- 이어서 재실행할 때 그만큼 건너뜀")
    ap.add_argument("--execute", action="store_true", help="실제로 이동/촬영 (없으면 dry-run)")
    ap.add_argument("--no-step", action="store_true", help="스텝마다 Enter로 확인하지 않고 연속 실행")
    ap.add_argument("--no-cam-reset", action="store_true", help="카메라 시작 전 하드웨어 리셋 생략")
    ap.add_argument("--camera-label", action="append",
                    help="SERIAL=이름 형식으로 카메라 라벨 지정 (여러 번 사용 가능)")
    ap.add_argument("--no-preview", action="store_true", help="4대 미리보기 창을 띄우지 않음")
    args = ap.parse_args()

    user_labels = parse_labels(args.camera_label)

    cams = {}
    labels = dict(user_labels)
    view = None
    if args.execute:
        cams, devices = connect_cameras(no_reset=args.no_cam_reset)
        labels = auto_labels(devices, user_labels)
        print("카메라 라벨:")
        for serial, name in sorted(devices.items()):
            print(f"  {serial}  {name}  -> {labels[serial]}")
        if not args.no_preview:
            view = LiveView(cams, labels)
            view.start()

    try:
        if args.session in (1, 3):
            run_simple_session(args.session, args, cams, labels)
        else:
            run_session2(args, cams, labels)
    finally:
        if view is not None:
            view.stop()
        stop_cameras(cams)


if __name__ == "__main__":
    main()
