#!/usr/bin/env python3
"""비전이 준 물체 자세로 잡으러 갈 로봇 자세를 계산한다.

입력
    물체 자세    카메라 기준 또는 베이스 기준 (위치 mm + 오일러각 도)
    물체 크기    높이 mm, 또는 "가로 세로 높이"
    로봇 현재    플랜지 자세 [x y z Rz Ry Rx] (mm, 도)

출력
    목표 절대 자세와 현재 대비 상대 이동량, 그리고 테이블 높이 검산

계산은 VISION_GRASP_GUIDE.md 의 절차를 그대로 따른다.
    T_base_obj = T_base_C? @ T_cam_obj        <- 캘리브레이션 행렬
    잡을 높이  = 물체 중심 + (크기로 정한 오프셋)
    플랜지 z   = 잡을 높이 + 115.5            <- 그리퍼 길이

실행 예:
    python3 grasp_target.py \\
        --obj "-35.7 30.0 608.9 -28.7 -55.5 -151.8" --cam cam1 \\
        --size 30 \\
        --flange "-253.48 474.21 148.84 -115.00 0.00 180.00"

    # 이미 베이스 기준 좌표인 경우
    python3 grasp_target.py --obj "..." --cam base --size "40 40 30" --flange "..."
"""
import os
import sys
import argparse

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))

FINGERTIP_Z_MM = 115.5      # 플랜지 -> 손가락 끝. server/c1.py:53
TABLE_Z_MM = -30.57         # 테이블 면 추정값. 검산에만 쓴다
TABLE_TOL_MM = 10.0         # 이보다 벗어나면 좌표가 틀린 것으로 본다


# ── 회전 도구 ─────────────────────────────────────────────
def _R(ax, t):
    c, s = np.cos(t), np.sin(t)
    return {"X": np.array([[1, 0, 0], [0, c, -s], [0, s, c]]),
            "Y": np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]]),
            "Z": np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])}[ax]


def inv_T(T):
    R = T[:3, :3]
    out = np.eye(4)
    out[:3, :3] = R.T
    out[:3, 3] = -R.T @ T[:3, 3]
    return out


def euler_to_R(angles_deg, order, intrinsic):
    """order 각 글자에 angles_deg 를 순서대로 대응시켜 회전행렬을 만든다.

    외재(extrinsic)는 고정축 기준이라 뒤에서부터 곱하고, 내재(intrinsic)는
    따라 도는 축이라 앞에서부터 곱한다.
    """
    Rs = [_R(ax.upper(), np.deg2rad(a)) for ax, a in zip(order, angles_deg)]
    return Rs[0] @ Rs[1] @ Rs[2] if intrinsic else Rs[2] @ Rs[1] @ Rs[0]


def pose6_to_T(vals):
    """로봇 6dof [x, y, z, Rz, Ry, Rx] -> 4x4 (미터). R = Rz @ Ry @ Rx."""
    x, y, z, a_z, a_y, a_x = [float(v) for v in vals]
    T = np.eye(4)
    T[:3, :3] = euler_to_R([a_z, a_y, a_x], "zyx", intrinsic=True)
    T[:3, 3] = np.array([x, y, z]) / 1000.0
    return T


def T_to_pose6(T):
    """4x4 -> [x, y, z, Rz, Ry, Rx] (mm, 도). pose6_to_T 의 역."""
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


def parse6(s):
    v = [float(x) for x in str(s).replace(",", " ").split()]
    if len(v) != 6:
        raise ValueError("값 6개가 필요하다: x y z 그리고 각 3개 (%d개 받음)" % len(v))
    return v


def parse_size(s):
    """'30' -> (None, None, 30) / '40 40 30' -> (40, 40, 30). 단위 mm."""
    v = [float(x) for x in str(s).replace(",", " ").split()]
    if len(v) == 1:
        return None, None, v[0]
    if len(v) == 3:
        return v[0], v[1], v[2]
    raise ValueError("--size 는 높이 하나 또는 '가로 세로 높이' 세 개다")


# ── 본체 ──────────────────────────────────────────────────
def main():
    ap = argparse.ArgumentParser(
        formatter_class=argparse.RawDescriptionHelpFormatter,
        description=__doc__)
    ap.add_argument("--obj", required=True,
                    help='물체 자세 "x y z a b c" (mm, 도). 회전 순서는 --euler 참고')
    ap.add_argument("--cam", default="cam1",
                    help="물체 자세의 기준 좌표계: cam0|cam1|cam3|base (기본 cam1)")
    ap.add_argument("--euler", default="xyz",
                    help="물체 회전의 오일러 순서 (기본 xyz = rx ry rz 순 입력)")
    ap.add_argument("--intrinsic", action="store_true",
                    help="물체 회전이 내재 회전이면 지정 (기본은 외재)")
    ap.add_argument("--size", required=True,
                    help='물체 크기 mm. "30" 또는 "가로 세로 높이"')
    ap.add_argument("--flange", required=True,
                    help='로봇 현재 플랜지 "x y z Rz Ry Rx" (mm, 도)')
    ap.add_argument("--grip_depth", type=float, default=None,
                    help="윗면에서 얼마나 아래를 물지 (mm). 기본은 높이/2 = 중심")
    ap.add_argument("--tool", type=float, default=FINGERTIP_Z_MM,
                    help="명령에 쓸 settool 값 (기본 115.5). TCP 표시용")
    ap.add_argument("--symmetry", type=float, default=90.0,
                    help="단면 회전 대칭 각도. 정사각 90, 직사각 180 (기본 90)")
    ap.add_argument("--calib", default=os.path.join(HERE, "data/session04/calib_out"))
    ap.add_argument("--table", type=float, default=TABLE_Z_MM,
                    help="검산에 쓸 테이블 높이 mm (기본 -30.57)")
    ap.add_argument("--no_check", action="store_true", help="테이블 검산을 건너뛴다")
    a = ap.parse_args()

    # ── 1) 입력 파싱
    ov = parse6(a.obj)
    w, d, h = parse_size(a.size)
    flange6 = parse6(a.flange)

    T_obj = np.eye(4)
    T_obj[:3, :3] = euler_to_R(ov[3:6], a.euler, a.intrinsic)
    T_obj[:3, 3] = np.array(ov[:3]) / 1000.0

    # ── 2) 베이스 기준으로 옮긴다
    if a.cam.lower() in ("base", "robot"):
        T_base_obj = T_obj
        print("[좌표] 입력이 이미 베이스 기준")
    else:
        idx = a.cam.lower().replace("cam", "")
        path = os.path.join(a.calib, "T_base_C%s.npy" % idx)
        if not os.path.exists(path):
            print("[오류] %s 가 없다. 이 카메라는 캘리브레이션에 안 들어갔다." % path)
            return 1
        T_base_obj = np.load(path) @ T_obj
        print("[좌표] %s -> 베이스  (%s)" % (a.cam, os.path.relpath(path, HERE)))

    p = T_to_pose6(T_base_obj)
    cx, cy, cz = p[0], p[1], p[2]
    print()
    print("  베이스 기준 물체")
    print("    위치 (mm)  x %9.2f   y %9.2f   z %9.2f" % (cx, cy, cz))
    print("    자세 (도)  Rz %8.2f  Ry %8.2f  Rx %8.2f" % (p[3], p[4], p[5]))

    # ── 3) 어느 축이 위를 향하는지 찾는다
    R = T_base_obj[:3, :3]
    up = int(np.argmax(np.abs(R[2, :])))
    tilt = np.rad2deg(np.arccos(min(abs(R[2, up]), 1.0)))
    horiz = [i for i in range(3) if i != up]
    print()
    print("  물체 축이 베이스에서 향하는 방향")
    for i, nm in enumerate("XYZ"):
        v = R[:, i]
        mark = "  <- 위쪽" if i == up else ""
        print("    %s축  [%6.3f %6.3f %6.3f]   수직과 %5.1f도%s"
              % (nm, v[0], v[1], v[2],
                 np.rad2deg(np.arccos(min(abs(v[2]), 1.0))), mark))
    if tilt > 15.0:
        print("  [경고] 가장 수직에 가까운 축도 %.1f도 기울었다. "
              "물체가 누워 있거나 자세 추정이 틀렸을 수 있다." % tilt)

    # ── 4) 잡을 방향: 수평축 방위각에 그리퍼를 맞춘다
    #
    # 수평축 둘은 서로 직각이므로 90도로 접으면 같은 값이어야 한다. 물체가 조금
    # 기울면 투영된 둘이 어긋나는데, 한쪽만 쓰면 어느 쪽을 골랐느냐로 답이 갈린다.
    # 접어서 원형평균을 내면 그 임의성이 사라진다.
    azims = [np.rad2deg(np.arctan2(R[1, i], R[0, i])) for i in horiz]
    zc = np.mean([np.exp(1j * np.deg2rad(x * 4.0)) for x in azims])
    grid = np.rad2deg(np.angle(zc)) / 4.0          # 물체 수평 프레임의 방위각 (mod 90)
    spread = abs(((azims[0] - azims[1] + 45.0) % 90.0) - 45.0)

    step = a.symmetry
    if step <= 90.0 + 1e-9:
        base = grid
    else:
        # 직사각 단면이면 두 수평축이 다른 뜻을 가지므로 첫 축을 기준으로 잡는다.
        kk = np.arange(-4, 5)
        base = float((grid + 90.0 * kk)[np.argmin(np.abs(grid + 90.0 * kk - azims[0]))])
    k = np.arange(-int(360 / step) - 1, int(360 / step) + 2)
    cands = base + step * k
    rz_t = float(cands[np.argmin(np.abs(cands - flange6[3]))])
    if spread > 3.0:
        print("  [경고] 수평축 둘이 직각에서 %.1f도 어긋난다. 자세 추정을 확인할 것."
              % spread)

    # ── 5) 잡을 높이: 윗면에서 grip_depth 만큼 아래
    depth = a.grip_depth if a.grip_depth is not None else h / 2.0
    top = cz + h / 2.0
    bottom = cz - h / 2.0
    tip_z = top - depth
    flange_z = tip_z + FINGERTIP_Z_MM

    print()
    print("  크기 %s mm 를 반영" % (("%g x %g x %g" % (w, d, h)) if w else ("높이 %g" % h)))
    print("    윗면 %8.2f   중심 %8.2f   아랫면 %8.2f" % (top, cz, bottom))
    print("    무는 깊이 %.2f (윗면 기준)  ->  손끝 %8.2f" % (depth, tip_z))
    if depth < 0 or depth > h:
        print("  [경고] 무는 깊이가 물체 높이(%g)를 벗어난다." % h)
    elif not a.no_check and tip_z < a.table:
        print("  [경고] 손끝이 테이블(%.2f)보다 낮다." % a.table)

    # ── 6) 목표와 이동량
    target = np.array([cx, cy, flange_z, rz_t, 0.0, 180.0])
    cur = np.array(flange6, dtype=float)
    delta = target - cur
    delta[3] = (delta[3] + 180.0) % 360.0 - 180.0      # 최단 회전
    delta[5] = (delta[5] + 180.0) % 360.0 - 180.0      # +-180 표기차 제거

    print()
    print("  === 목표 플랜지 (절대) ===")
    print("    x %9.2f   y %9.2f   z %9.2f" % tuple(target[:3]))
    print("    Rz %8.2f  Ry %8.2f  Rx %8.2f" % tuple(target[3:]))
    print()
    print("  === 현재 대비 상대 이동 ===")
    print("    dx %+8.2f mm   dy %+8.2f mm   dz %+8.2f mm" % tuple(delta[:3]))
    print("    dRz %+7.2f 도   dRy %+7.2f 도   dRx %+7.2f 도" % tuple(delta[3:]))

    if abs(a.tool - 0.0) > 1e-9:
        print()
        print("  settool %.1f 로 명령할 때의 TCP z: %.2f  (xy·회전은 위와 동일)"
              % (a.tool, flange_z - a.tool))

    print()
    print("  === 움직이는 순서 (대각선 하강 금지) ===")
    print("    1) %9.2f %9.2f %9.2f  %8.2f %8.2f %8.2f   xy·회전만"
          % (target[0], target[1], cur[2], target[3], target[4], target[5]))
    print("    2) %9.2f %9.2f %9.2f  %8.2f %8.2f %8.2f   수직 하강 %.2f mm"
          % (target[0], target[1], target[2], target[3], target[4], target[5],
             cur[2] - target[2]))

    # ── 7) 테이블 검산
    if not a.no_check:
        gap = bottom - a.table
        print()
        print("  === 검산: 물체 아랫면이 테이블에 닿는가 ===")
        print("    아랫면 %.2f   테이블 %.2f   차이 %+.2f mm" % (bottom, a.table, gap))
        if abs(gap) <= TABLE_TOL_MM:
            print("    통과. 좌표가 맞을 가능성이 높다.")
        else:
            print("    [실패] %.1fmm 를 넘게 벗어났다. 내려가지 말 것." % TABLE_TOL_MM)
            print("           카메라 번호(--cam), 회전 순서(--euler), 크기(--size)를 확인.")
            return 2
    return 0


if __name__ == "__main__":
    sys.exit(main())
