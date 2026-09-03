#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ur3_calibration/session2_pick_and_place.py -- 세션2를 실제 pick-and-place로 수행

session2_sweep.py 는 큐브를 들지 않고 팔만 움직여 경로를 검증하는 용도였다.
이 스크립트는 실제로 큐브를 집어(pick) 세션2의 15개 목표 위치에 순서대로
옮겨 놓고(place), 매번 그리퍼가 비면 항상 같은 고정 자세(gripper_cam_pose.json)
로 돌아가 그리퍼 카메라 + 고정 카메라가 동시에 촬영할 수 있게 한다.

한 번의 pick/place 사이클:
  1. 큐브가 있는 현재 위치(처음은 grasp_flange_pose.json, 이후는 직전에
     놓은 자리) 위 approach 높이로 이동 (xy/방향 정렬, 대각선 하강 금지)
  2. 수직으로 하강해 그리퍼 닫기 (pick)
  3. 수직으로 approach 높이까지 상승
  4. 목표 위치(session2 포즈에서 x,y,yaw만 사용, z/tilt는 grasp 기준 고정)
     위 approach 높이로 이동
  5. 수직으로 하강해 그리퍼 열기 (place)
  6. 수직으로 approach 높이까지 상승
  7. gripper_cam_pose.json 자세로 이동 (그리퍼 카메라 고정 촬영 위치 --
     실제 이미지 캡처 코드는 아직 없음, 자리 확인용)

approach 높이(수직 접근/후퇴 거리)는 기본 50mm(5cm) -- "잡고 놓기 직후
수직 방향은 5cm만 유지" 요청대로.

이동 순서(어느 목표를 몇 번째로 방문할지)는 session2_sweep.py와 동일하게
delta_yaw 오름차순으로 재정렬해 인접 자세 간 회전 변화를 줄인다.

*** moveL vs moveJ ***
UR moveL은 회전을 rotation-vector 성분 그대로 선형보간한다. gripper_cam_pose
는 pick/place 자세들과 orientation이 많이 달라서, 그쪽을 오가는 구간을
moveL로 하면 두 자세 사이 rotvec 차이가 커서(짐벌/wrap-around 근처) TCP가
중간에 크게 휘돌 위험이 있다. 그래서 사전 점검(check_rotation_jumps)에서
연속 목표 간 rotvec 변화가 큰 구간을 찾아, 그 구간만 moveL 대신
moveJ(getInverseKinematics로 관절해를 구해서 관절공간 이동)로 바꾼다.
moveJ는 TCP가 직선/특정 궤적을 그리지 않는 대신, 관절이 각자 매끄럽게
움직이므로 orientation이 크게 바뀌는 재배치 이동에 훨씬 안전하다.

*** 실제 하드웨어에서 물건을 집었다 놓았다 하는 스크립트다. ***
반드시 --dry-run(기본값)으로 전체 계획/경고를 먼저 확인하고, 로봇 옆에서
비상정지를 잡을 수 있는 상태로, 기본은 매 스텝마다 Enter로 진행한다.

사용법:
  python session2_pick_and_place.py                    # dry-run: 전체 계획 출력
  python session2_pick_and_place.py --execute           # 스텝마다 Enter로 확인 후 실행
  python session2_pick_and_place.py --execute --no-step # (검증 후) 연속 실행
  python session2_pick_and_place.py --execute --return-home  # 마지막에 큐브를 원위치로 복귀
"""

import argparse
import json
import time
from pathlib import Path

import numpy as np

import rtde_control
import rtde_receive
from gripper import RobotiqGripper
from capture_poses import session_path
from session2_sweep import raw_yaw, wrap_pi, build_target, fmt_pose

ROBOT_IP_DEFAULT = "192.168.1.101"
GRASP_POSE_DEFAULT = Path(__file__).resolve().parent / "data" / "grasp_flange_pose.json"
CAM_POSE_DEFAULT = Path(__file__).resolve().parent / "data" / "gripper_cam_pose.json"
SESSION_ID = 2

APPROACH_MM_DEFAULT = 50.0     # 5cm -- pick/place 직후 수직 유지 거리
PICK_LIFT_MM_DEFAULT = 1.0     # pick 하강도 이만큼 남기고 멈춤 (바닥/큐브에 세게 눌러붙지 않게)
PLACE_LIFT_MM_DEFAULT = 1.0    # place 하강을 이만큼 남기고 멈춤 (완전히 안 눌러줌, 큐브가 자연히 얹힘)
MOVE_SPEED = 0.10              # m/s -- approach/카메라 자세 등 수평 이동 (moveL)
MOVE_ACCEL = 0.30
DESCEND_SPEED = 0.03           # m/s -- pick/place 수직 하강/상승 (moveL, 더 느리게)
DESCEND_ACCEL = 0.20
JOINT_SPEED = 0.30             # rad/s -- 큰 재배치 이동 (moveJ)
JOINT_ACCEL = 0.30
ROTVEC_JUMP_THRESHOLD = 1.0    # 이 이상이면 moveL 대신 moveJ+IK 사용
RELEASE_STEP_DEFAULT = 20      # place 때 close-pos에서 이 정도만 풀어줌 (완전개방 금지)
IK_MAX_POS_ERROR = 1e-4        # m -- 기본값(1e-10)은 너무 타이트해서 먼 qnear에서 수렴 실패가 잦음
IK_MAX_ORI_ERROR = 1e-3        # rad


def call_with_retry(fn, *args, attempts=4, delay_s=0.5):
    """RuntimeError("... did not succeed!") 는 로봇 컨트롤 스크립트가
    막 시작된 직후처럼 일시적인 상태에서도 나서 몇 번 재시도해본다."""
    last_exc = None
    for attempt in range(attempts):
        try:
            return fn(*args)
        except RuntimeError as exc:
            last_exc = exc
            if attempt < attempts - 1:
                time.sleep(delay_s)
    raise last_exc


def approach_of(pose, offset_mm):
    p = list(pose)
    p[2] += offset_mm / 1000.0
    return p


def load_pose(path: Path) -> list:
    return list(json.loads(path.read_text())["tcp_pose"])


def compute_ordered_targets(session_poses_path: Path, grasp_rotvec, z_fixed):
    yaw_grasp = raw_yaw(grasp_rotvec)
    poses = json.loads(session_poses_path.read_text())["poses"]
    items = []
    for orig_idx, p in enumerate(poses):
        x, y, _z, rx, ry, rz = p["tcp_pose"]
        yaw_i = raw_yaw([rx, ry, rz])
        delta_yaw = wrap_pi(yaw_i - yaw_grasp)
        target = build_target(x, y, z_fixed, delta_yaw, grasp_rotvec)
        items.append({"orig_index": orig_idx, "delta_yaw": delta_yaw, "target": target})
    items.sort(key=lambda it: it["delta_yaw"])
    return items


def mv(pose, speed, accel, desc):
    return {"kind": "moveL", "pose": pose, "speed": speed, "accel": accel, "desc": desc}


def grip(pos, desc):
    return {"kind": "gripper", "pos": pos, "desc": desc}


def cap(desc):
    return {"kind": "capture", "desc": desc}


def build_plan(items, grasp_pose, cam_pose, approach_mm, pick_lift_mm, place_lift_mm,
               open_pos, close_pos, release_pos, return_home):
    """전체 실행 계획을 스텝(dict) 리스트로 만든다."""
    steps = []
    current = list(grasp_pose)

    def pick_place_block(label, source_pose, dest_pose):
        steps.append(grip(open_pos, f"[{label}] 그리퍼 열기"))
        steps.append(mv(approach_of(source_pose, approach_mm), MOVE_SPEED, MOVE_ACCEL,
                        f"[{label}] pick approach 이동"))
        # source_pose까지 완전히 내려가지 않고 pick_lift_mm 만큼 남기고 멈춘다 --
        # 바닥/큐브에 세게 눌러붙지 않도록.
        steps.append(mv(approach_of(source_pose, pick_lift_mm), DESCEND_SPEED, DESCEND_ACCEL,
                        f"[{label}] pick 수직 하강 (-{pick_lift_mm:.0f}mm 남기고 정지)"))
        steps.append(grip(close_pos, f"[{label}] 그리퍼 닫기 (pick)"))
        steps.append(mv(approach_of(source_pose, approach_mm), DESCEND_SPEED, DESCEND_ACCEL,
                        f"[{label}] pick 수직 상승"))
        steps.append(mv(approach_of(dest_pose, approach_mm), MOVE_SPEED, MOVE_ACCEL,
                        f"[{label}] place approach 이동"))
        # dest_pose까지 완전히 내려가지 않고 place_lift_mm 만큼 남기고 멈춘다 --
        # 큐브를 바닥에 눌러붙이지 않고 그 높이에서 놓아 자연히 얹히게 한다.
        # (다음 pick의 source_pose 등 다른 곳에서는 원래 dest_pose 그대로 쓴다.)
        steps.append(mv(approach_of(dest_pose, place_lift_mm), DESCEND_SPEED, DESCEND_ACCEL,
                        f"[{label}] place 수직 하강 (-{place_lift_mm:.0f}mm 남기고 정지)"))
        # 완전 개방이 아니라 살짝만 풀어서 블럭과의 마찰/위치 틀어짐을 줄인다.
        steps.append(grip(release_pos, f"[{label}] 그리퍼 살짝 풀기 (place)"))
        steps.append(mv(approach_of(dest_pose, approach_mm), DESCEND_SPEED, DESCEND_ACCEL,
                        f"[{label}] place 수직 상승"))
        steps.append(mv(cam_pose, MOVE_SPEED, MOVE_ACCEL,
                        f"[{label}] 그리퍼캠 고정 촬영 위치로 이동"))
        steps.append(cap(f"[{label}] 촬영 자리 (실제 캡처 코드는 아직 없음)"))

    for order, item in enumerate(items):
        label = f"{order + 1}/{len(items)} (원본 #{item['orig_index'] + 1})"
        pick_place_block(label, current, item["target"])
        current = item["target"]

    if return_home:
        pick_place_block("원위치 복귀", current, list(grasp_pose))

    return steps


def mark_risky_moveL_as_moveJ(steps, threshold=ROTVEC_JUMP_THRESHOLD):
    """연속 moveL 목표 간 rotvec 변화가 큰 스텝을 moveJ_risky로 바꾼다.

    moveJ_risky는 실행 시점에 getInverseKinematics로 관절해를 구해
    moveJ로 이동한다 (관절공간 이동이라 orientation이 크게 바뀌어도
    TCP가 예측 불가능하게 휘돌지 않는다).
    """
    warnings = []
    last_pose = None
    for i, step in enumerate(steps):
        if step["kind"] != "moveL":
            continue
        pose = step["pose"]
        if last_pose is not None:
            d = float(np.linalg.norm(np.array(pose[3:6]) - np.array(last_pose[3:6])))
            if d > threshold:
                step["kind"] = "moveJ_risky"
                step["jump"] = d
                warnings.append((i, d, step["desc"]))
        last_pose = pose
    return warnings


def print_plan(steps, grasp_pose, cam_pose, approach_mm, warnings):
    print(f"grasp(원위치)   : {fmt_pose(grasp_pose)}")
    print(f"gripper_cam_pose: {fmt_pose(cam_pose)}")
    print(f"approach 높이   : {approach_mm:.0f}mm\n")
    for i, step in enumerate(steps):
        if step["kind"] in ("moveL", "moveJ_risky"):
            tag = "moveL " if step["kind"] == "moveL" else "moveJ*"
            print(f"[{i + 1:3d}] {tag} {step['desc']:<40s} {fmt_pose(step['pose'])}")
        elif step["kind"] == "gripper":
            print(f"[{i + 1:3d}] grip   {step['desc']:<40s} pos={step['pos']}")
        else:
            print(f"[{i + 1:3d}] ----   {step['desc']}")
    print("\n(moveJ* = orientation 변화가 커서 moveL 대신 IK+moveJ로 실행되는 스텝)")
    if warnings:
        print()
        for i, d, desc in warnings:
            print(f"  [INFO] 스텝 {i + 1}: rotvec 변화 |delta|={d:.2f} -> moveJ로 전환됨 -- {desc}")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--robot-ip", default=ROBOT_IP_DEFAULT)
    ap.add_argument("--grasp-pose", default=str(GRASP_POSE_DEFAULT))
    ap.add_argument("--cam-pose", default=str(CAM_POSE_DEFAULT))
    ap.add_argument(
        "--session-poses",
        default=str(session_path(Path(__file__).resolve().parent / "data", SESSION_ID)),
    )
    ap.add_argument("--approach-mm", type=float, default=APPROACH_MM_DEFAULT)
    ap.add_argument("--pick-lift-mm", type=float, default=PICK_LIFT_MM_DEFAULT,
                    help="pick 하강을 이만큼 남기고 멈춤 (기본 1mm)")
    ap.add_argument("--place-lift-mm", type=float, default=PLACE_LIFT_MM_DEFAULT,
                    help="place 하강을 이만큼 남기고 멈춤 (기본 1mm, 큐브를 바닥에 누르지 않음)")
    ap.add_argument("--open-pos", type=int, default=0)
    ap.add_argument("--close-pos", type=int, default=150,
                    help="그리퍼 닫힘 값 0-255 (실측: 큐브를 딱 잡은 상태의 POS)")
    ap.add_argument("--release-pos", type=int, default=None,
                    help="place 때 살짝만 풀 위치 0-255 (기본: close-pos에서 RELEASE_STEP_DEFAULT만큼만 완화)")
    ap.add_argument("--execute", action="store_true", help="실제로 이동/그리퍼 조작 (없으면 dry-run)")
    ap.add_argument("--no-step", action="store_true", help="스텝마다 Enter로 확인하지 않고 연속 실행")
    ap.add_argument("--return-home", action="store_true", help="마지막에 큐브를 grasp_flange_pose 위치로 복귀")
    ap.add_argument("--skip-steps", type=int, default=0,
                    help="이미 실제로 실행된 스텝 수 -- 이어서 재실행할 때 그만큼 건너뜀")
    args = ap.parse_args()

    if args.release_pos is None:
        direction = -1 if args.close_pos > args.open_pos else 1
        release_pos = int(np.clip(args.close_pos + direction * RELEASE_STEP_DEFAULT, 0, 255))
    else:
        release_pos = args.release_pos

    grasp_pose = load_pose(Path(args.grasp_pose))
    cam_pose = load_pose(Path(args.cam_pose))
    items = compute_ordered_targets(Path(args.session_poses), grasp_pose[3:6], grasp_pose[2])
    steps = build_plan(items, grasp_pose, cam_pose, args.approach_mm, args.pick_lift_mm, args.place_lift_mm,
                        args.open_pos, args.close_pos, release_pos, args.return_home)
    warnings = mark_risky_moveL_as_moveJ(steps)
    print_plan(steps, grasp_pose, cam_pose, args.approach_mm, warnings)

    if not args.execute:
        print(f"\n(dry-run) 총 {len(steps)}스텝 계획. 실제로 움직이지 않았습니다. --execute 를 주면 실행합니다.")
        return

    if args.skip_steps:
        print(f"\n처음 {args.skip_steps}스텝은 이미 실행된 것으로 보고 건너뜁니다.")

    rtde_c = rtde_control.RTDEControlInterface(args.robot_ip)
    rtde_r = rtde_receive.RTDEReceiveInterface(args.robot_ip)
    gripper = RobotiqGripper(args.robot_ip)
    # 연결 직후 바로 IK 계열 함수를 부르면 "RTDE control script is not
    # running!" 로 실패하는 경우가 있어(컨트롤 스크립트가 아직 완전히
    # 뜨기 전) 잠깐 대기해준다.
    time.sleep(1.0)
    try:
        for i, step in enumerate(steps):
            if i < args.skip_steps:
                continue
            desc = step["desc"]
            kind = step["kind"]
            if i == args.skip_steps and kind == "moveL" and args.skip_steps > 0:
                # --skip-steps로 재개할 때는 실제 로봇이 계획이 가정한 위치가
                # 아니라 임의의 자세(예: 촬영 위치)에서 시작할 수 있다. 사전
                # rotvec 점프 점검은 "계획대로 순서대로 실행"을 가정하고
                # 계산된 것이라 이 첫 스텝에는 적용되지 않았을 수 있으니,
                # 안전하게 moveJ+IK로 강제 전환한다.
                kind = "moveJ_risky"
                print("  [INFO] 재개 지점의 첫 이동이라 안전하게 moveJ로 전환합니다.")
            if not args.no_step:
                cmd = input(f"\n[{i + 1}/{len(steps)}] {desc}\nEnter=진행 / q=중단 > ").strip().lower()
                if cmd == "q":
                    print("중단했습니다.")
                    break
            else:
                print(f"[{i + 1}/{len(steps)}] {desc}")

            if kind == "moveL":
                rtde_c.moveL(step["pose"], step["speed"], step["accel"])
            elif kind == "moveJ_risky":
                q_near = rtde_r.getActualQ()
                try:
                    has_solution = call_with_retry(
                        rtde_c.getInverseKinematicsHasSolution,
                        step["pose"], q_near, IK_MAX_POS_ERROR, IK_MAX_ORI_ERROR,
                    )
                except RuntimeError as exc:
                    print(f"  [ERROR] IK 계산이 계속 실패합니다 -- 중단합니다: {exc}")
                    break
                if not has_solution:
                    print(f"  [ERROR] 이 자세에 대한 관절해가 없습니다 -- 중단합니다: {fmt_pose(step['pose'])}")
                    break
                try:
                    q_target = call_with_retry(
                        rtde_c.getInverseKinematics,
                        step["pose"], q_near, IK_MAX_POS_ERROR, IK_MAX_ORI_ERROR,
                    )
                except RuntimeError as exc:
                    print(f"  [ERROR] IK 계산이 계속 실패합니다 -- 중단합니다: {exc}")
                    break
                rtde_c.moveJ(q_target, JOINT_SPEED, JOINT_ACCEL)
            elif step["kind"] == "gripper":
                gripper.set_pos(step["pos"])
                # 물리적으로 멈출 때까지 기다린다 (OBJ==0 은 아직 움직이는 중) --
                # 안 그러면 그리퍼가 다 닫히기/풀리기 전에 팔이 먼저 움직여버린다.
                gripper.wait_until_stopped()
            else:
                pass  # capture placeholder -- 카메라 캡처는 아직 미구현
    finally:
        gripper.close()
        rtde_c.stopScript()
        rtde_c.disconnect()
        rtde_r.disconnect()


if __name__ == "__main__":
    main()
