#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""zeus_gello_calibration/replay_and_recapture.py -- 저장된 세션의 joint 자세로
로봇을 직접(자동) 다시 움직여서 pose/joint - 이미지 동기화 오차를 없애고 재촬영

GELLO 텔레옵으로 촬영할 때는 SPACE를 누르는 순간에도 사람 손이 계속 움직이고
있을 수 있어서, "그 순간 읽은 pose"와 "그 순간 찍힌 이미지"가 미세하게 어긋날
(시간 지연으로 인한 위치 offset) 위험이 있다. 이 스크립트는 그 위험을 없애기
위해, 이미 저장된 capture/<idx>/robot.json 의 joint 값으로 **로봇을 완전히
멈춰 세운 뒤** 촬영한다(움직이는 도중에 찍는 게 아니라 도착 후 정지 상태에서
촬영 -> pose와 이미지가 확실히 같은 순간을 가리킴).

*** 이 스크립트는 실제로 로봇을 자동으로 움직인다(movej). ***
지금까지 이 프로젝트의 Zeus 쪽 스크립트들은 전부 GELLO 텔레옵(사람이 직접
조종)이거나 get_state()만 읽는 읽기 전용이었는데, 이건 처음으로 자동 이동
명령을 보낸다. 원래 GELLO로 자유롭게 움직이며 찍은 16개 자세를 순서 그대로
재생하는 거라 연속된 자세끼리는 원래도 사람이 부드럽게 이어서 움직인
경로이긴 하지만, **그래도 처음 실행은 반드시 로봇 옆에서 비상정지에 손이
닿는 상태로, 저속(--jnt-speed 낮게)으로 스텝별 확인(--no-step 없이)하며
진행할 것.** session1은 큐브를 원본과 같은 방식으로 계속 파지하고, session3은
stationary target을 원본 위치에 고정한다. release와 pick이 필요한 session2는 이
스크립트로 실행하지 않는다.

--regrasp-joints J1 J2 J3 J4 J5 J6 를 주면, 저장된 자세로 이동하기 전에
사용자 입력을 받아가며 큐브를 새로 쥐는 절차를 먼저 수행한다:
  1) Enter 입력 대기
  2) 그리퍼 열기 -> 지정한 joints로 movej
  3) "큐브를 놓아주세요" 안내 후 Enter 입력 대기
  4) 그리퍼 닫기
  이후 평소처럼 저장된 자세들로 이동+촬영을 진행한다.

기본은 기존 capture/<idx>/ 폴더를 덮어쓰지 않고 별도 폴더
(capture_replayed/<idx>/)에 저장한다. 해당 폴더가 이미 차 있으면 실행을 거부하므로
--output-subdir로 새 이름을 지정해야 한다. --overwrite-source를 주면 원래
capture/<idx>/를 덮어쓰지만, 원본 보존을 위해 사용하지 않는 것을 권장한다.

사용법:
  python replay_and_recapture.py --session 1                     # dry-run: 계획만 출력
  python replay_and_recapture.py --session 1 --execute --motion-only  # 이동 경로만 저속 검증
  python replay_and_recapture.py --session 1 --execute \\
      --output-subdir capture_replayed_step_check                 # 스텝별 확인하며 촬영
  python replay_and_recapture.py --session 1 --execute --no-step \\
      --output-subdir capture_replayed_auto                       # 검증 후 연속 촬영
  python replay_and_recapture.py --session 1 --execute --regrasp-joints  # 재파지부터 (기본 자세)
  python replay_and_recapture.py --session 1 --execute \\
      --regrasp-joints 24.48 -33.28 -111.05 -179.99 35.68 24.48    # 재파지 자세 직접 지정
"""

import argparse
import json
import math
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from robot.backends.zeus_client import ZeusClient  # noqa: E402

from zeus_gello_calibration.capture_session import (  # noqa: E402
    SESSIONS, load_camera_labels, connect_cameras, stop_cameras, LiveView,
    grab_frames, write_capture, read_robot_state,
    ROBOT_IP_DEFAULT, ROBOT_PORT_DEFAULT, DEVICE_MAP_DEFAULT,
)
from zeus_gello_calibration.paths import (  # noqa: E402
    ZEUS_DATA_ROOT,
    require_zeus_data_path,
)
from capture_pipeline.paths import resolve_dated_dir  # noqa: E402

JNT_SPEED_DEFAULT = 10.0   # GELLO 텔레옵과 동일한 "실기 테스트로 정한" 기본값
OVERLAP_DEFAULT = 0.0      # 블렌딩 없이 매번 완전히 멈춰야 정확한 정지 후 촬영이 됨
SETTLE_S = 0.3             # movej 리턴 직후 잔진동/카메라 버퍼 안정화 대기
ROBOT_TIMEOUT_DEFAULT = 30.0  # movej는 완료까지 응답 없는 blocking 방식이라 기본 10초로는 부족할 수 있음

# session1 재파지 기본 자세 (실측 지정값) -- --regrasp-joints를 값 없이 주면 이걸 씀.
REGRASP_JOINTS_DEFAULT = {
    1: [24.48, -33.28, -111.05, -179.99, 35.68, 24.48],
}


def run_regrasp_sequence(rb, joints, jnt_speed, overlap):
    """큐브를 새로 쥐는 절차: Enter -> 그리퍼 열기+이동 -> "놓아주세요" -> Enter -> 그리퍼 닫기."""
    input("\n[재파지] Enter를 누르면 그리퍼를 열고 지정 자세로 이동합니다 > ")
    rb.grip("open")
    time.sleep(SETTLE_S)
    rb.movej(joints, jnt_speed=jnt_speed, overlap=overlap)
    time.sleep(SETTLE_S)
    print(f"[재파지] 이동 완료: {[round(v, 2) for v in joints]}")
    input("[재파지] 큐브를 그리퍼 위치에 놓아주세요. 다 놓으셨으면 Enter > ")
    rb.grip("close")
    time.sleep(SETTLE_S)
    print("[재파지] 그리퍼 닫음 -- 저장된 자세로 이동+촬영을 시작합니다.\n")


def load_saved_joints(capture_root: Path) -> list:
    items = []
    if not capture_root.is_dir():
        return items
    for d in sorted(
        (p for p in capture_root.iterdir() if p.is_dir() and p.name.isdigit()),
        key=lambda p: int(p.name),
    ):
        robot_json = d / "robot.json"
        if not robot_json.exists():
            continue
        data = json.loads(robot_json.read_text())
        joints = data.get("joints")
        if (
            not isinstance(joints, list)
            or len(joints) != 6
            or any(not isinstance(value, (int, float)) or not math.isfinite(value) for value in joints)
        ):
            raise ValueError(f"{robot_json}: joints는 유한한 숫자 6개여야 합니다.")
        items.append({"index": int(d.name), "joints": joints, "src_dir": d})
    return items


def require_subdir_name(value: str) -> str:
    """Keep replay output inside the selected session directory."""
    path = Path(value)
    if value in {"", ".", ".."} or path.name != value or path.is_absolute():
        raise argparse.ArgumentTypeError("단일 폴더 이름만 허용합니다 (예: capture_replayed_auto).")
    return value


def directory_has_files(path: Path) -> bool:
    return path.is_dir() and any(path.iterdir())


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--session", type=int, required=True, choices=sorted(SESSIONS))
    ap.add_argument("--robot-ip", default=ROBOT_IP_DEFAULT)
    ap.add_argument("--robot-port", type=int, default=ROBOT_PORT_DEFAULT)
    ap.add_argument("--robot-timeout", type=float, default=ROBOT_TIMEOUT_DEFAULT)
    ap.add_argument("--device-map", default=str(DEVICE_MAP_DEFAULT))
    ap.add_argument(
        "--data-root", "--out-root",
        dest="data_root",
        default=str(ZEUS_DATA_ROOT),
        help=f"Zeus capture data root (must stay inside {ZEUS_DATA_ROOT})",
    )
    ap.add_argument(
        "--session-dir",
        default=None,
        help="재생할 session 폴더를 직접 지정. 생략하면 --data-root에서 --session에 맞는 최신 폴더를 찾음",
    )
    ap.add_argument(
        "--output-subdir",
        type=require_subdir_name,
        default="capture_replayed",
        help="재촬영 저장 폴더 이름 (기본: capture_replayed). 기존 데이터가 있으면 실행 거부",
    )
    ap.add_argument("--jnt-speed", type=float, default=JNT_SPEED_DEFAULT)
    ap.add_argument("--overlap", type=float, default=OVERLAP_DEFAULT)
    ap.add_argument("--overwrite-source", "--overwrite", dest="overwrite_source", action="store_true",
                    help="원래 capture/<idx>/ 폴더를 덮어씀. 원본 보존을 위해 사용 비권장")
    ap.add_argument("--execute", action="store_true", help="실제로 이동/촬영 (없으면 dry-run)")
    ap.add_argument("--no-step", action="store_true", help="스텝마다 Enter로 확인하지 않고 연속 실행")
    ap.add_argument("--no-cam-reset", action="store_true")
    ap.add_argument("--no-preview", action="store_true")
    ap.add_argument("--motion-only", action="store_true",
                    help="카메라를 아예 연결하지 않고 movej 이동만 수행 (충돌/경로 확인용, 촬영/저장 없음)")
    ap.add_argument("--regrasp-joints", type=float, nargs="*", default=None,
                    metavar="J",
                    help="저장된 자세로 이동하기 전에, 사용자 확인을 받아가며 이 joints에서 "
                         "큐브를 새로 쥐는 절차(그리퍼 열기->이동->대기->그리퍼 닫기)를 먼저 수행. "
                         "값 6개를 직접 주거나(J1..J6), 값 없이 --regrasp-joints만 주면 "
                         "REGRASP_JOINTS_DEFAULT[세션번호]를 씀")
    args = ap.parse_args()

    if args.session == 2 and args.execute:
        ap.error(
            "session2는 place/pick/gripper 동작이 필요한 데이터입니다. 단순 movej 재생으로는 "
            "물리 상태를 재현할 수 없으므로 session2_pick_and_place.py를 사용하세요."
        )

    if args.regrasp_joints is not None:
        if args.session != 1:
            ap.error("--regrasp-joints는 cube를 계속 파지하는 session1에서만 사용할 수 있습니다.")
        if len(args.regrasp_joints) == 0:
            if args.session not in REGRASP_JOINTS_DEFAULT:
                print(f"[ERROR] --regrasp-joints를 값 없이 줬는데 세션 {args.session}용 기본값이 없습니다. "
                      "직접 6개 값을 주세요 (J1..J6).")
                return
            args.regrasp_joints = REGRASP_JOINTS_DEFAULT[args.session]
        elif len(args.regrasp_joints) != 6:
            print(f"[ERROR] --regrasp-joints는 6개 값(J1..J6)이거나 값 없이 줘야 합니다 "
                  f"({len(args.regrasp_joints)}개 받음).")
            return

    info = SESSIONS[args.session]
    try:
        data_root = require_zeus_data_path(args.data_root, label="--data-root")
        session_dir = (
            require_zeus_data_path(args.session_dir, label="--session-dir")
            if args.session_dir
            else resolve_dated_dir(
                f"session{args.session}_{info['name']}", data_root, create=False
            )
        )
    except ValueError as exc:
        ap.error(str(exc))
    capture_root = session_dir / "capture"
    out_root = capture_root if args.overwrite_source else session_dir / args.output_subdir

    try:
        items = load_saved_joints(capture_root)
    except (OSError, ValueError, json.JSONDecodeError) as exc:
        ap.error(f"원본 robot.json을 읽을 수 없습니다: {exc}")
    if not items:
        ap.error(f"{capture_root}에 유효한 robot.json 캡처가 없습니다.")
    if (
        args.execute
        and not args.motion_only
        and not args.overwrite_source
        and directory_has_files(out_root)
    ):
        ap.error(
            f"{out_root}에 기존 결과가 있습니다. 원본을 보존하려면 "
            "--output-subdir에 새 이름을 지정하세요."
        )

    print(f"=== 세션 {args.session}: {info['name']} 재생+재촬영 ===")
    print(f"원본: {capture_root}  ({len(items)}개)")
    print(f"저장 위치: {out_root}{' (원본 덮어씀)' if args.overwrite_source else ' (원본은 그대로 둠)'}")
    print(f"jnt_speed={args.jnt_speed}  overlap={args.overlap}\n")
    if args.regrasp_joints is not None:
        print(f"--regrasp-joints 지정됨: 시작 전에 {[round(v, 2) for v in args.regrasp_joints]}에서 "
              "재파지 절차(그리퍼 열기->이동->대기->닫기)를 먼저 수행합니다.\n")
    elif args.session == 1:
        print("큐브가 세션1 촬영 때와 같은 방식으로 그리퍼에 그대로 물려있어야 합니다 "
              "(--regrasp-joints 없이는 이 스크립트가 그리퍼를 건드리지 않습니다).\n")
    else:
        print("session3의 stationary target은 원본 촬영 때와 같은 위치에 고정하고, "
              "로봇과 gripper camera만 이동해야 합니다.\n")

    for it in items:
        print(f"  #{it['index']:03d}  joints(deg)={[round(v, 1) for v in it['joints']]}")

    if not args.execute:
        print(f"\n(dry-run) 총 {len(items)}개 이동+촬영 계획. --execute 를 주면 실행합니다.")
        return

    cams, used_labels, view = [], [], None
    if args.motion_only:
        print("--motion-only: 카메라를 연결하지 않고 movej 이동만 수행합니다 (촬영/저장 없음).\n")
    else:
        labels = load_camera_labels(Path(args.device_map))
        cams, used_labels = connect_cameras(labels, no_reset=args.no_cam_reset)
        if not args.no_preview:
            view = LiveView(cams, used_labels, window_name="replay_and_recapture (q/ESC=닫기)")
            view.start()

    rb = ZeusClient(args.robot_ip, args.robot_port, timeout=args.robot_timeout)
    rb.connect()
    print("\n*** 실제 로봇이 자동으로 움직입니다. 비상정지에 손이 닿는 상태인지 확인하세요. ***")

    if args.regrasp_joints is not None:
        run_regrasp_sequence(rb, args.regrasp_joints, args.jnt_speed, args.overlap)

    prompt = (
        "전체 연속 자동 실행을 시작하려면 'go' 입력: "
        if args.no_step
        else "스텝별 실행을 시작하려면 'go' 입력: "
    )
    if input(prompt).strip().lower() != "go":
        print("취소했습니다.")
        rb.close()
        if view is not None:
            view.stop()
        stop_cameras(cams)
        return

    try:
        for it in items:
            desc = f"[{it['index']:03d}/{len(items)}] movej -> {[round(v, 1) for v in it['joints']]}"
            if not args.no_step:
                cmd = input(f"\n{desc}\nEnter=이동+촬영 / q=중단 > ").strip().lower()
                if cmd == "q":
                    print("중단했습니다.")
                    break
            else:
                print(desc)

            rb.movej(it["joints"], jnt_speed=args.jnt_speed, overlap=args.overlap)
            time.sleep(SETTLE_S)  # 잔진동 + 카메라 버퍼가 새 프레임으로 채워질 시간

            if args.motion_only:
                continue

            if view is not None:
                view.show()

            robot_state = read_robot_state(rb, {"capture_index": it["index"], "replayed_from": str(it["src_dir"])})
            frames = grab_frames(cams, used_labels)
            out_dir = out_root / f"{it['index']:03d}"
            write_capture(frames, out_dir, robot_state)
    except (KeyboardInterrupt, EOFError):
        pass
    finally:
        rb.close()
        if view is not None:
            view.stop()
        if cams:
            stop_cameras(cams)

    if args.motion_only:
        print("\n완료 -- movej 이동만 수행했습니다 (촬영/저장 없음).")
    else:
        print(f"\n완료 -- {out_root} 확인해보세요.")


if __name__ == "__main__":
    main()
