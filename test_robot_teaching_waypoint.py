#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""티칭한 포즈 풀(set/grip/pose)의 캘리브레이션 다양성 검증.

서버(robot_calb.py)의 recpose/recgrip/recset 로 티칭해 PC(Step2_capture.py)에
저장된 세 파일을 읽어, 방법별로 필요한 "다양성 축"이 충분한지 정량 점검한다.

  - capture_poses.json : A_placement 뷰포인트 (그리퍼캠 eye-in-hand, gTc 관측)
  - grip_poses.json    : B_eyetohand 그립-스윕 (고정캠 eye-to-hand 관측)
  - capture_sets.json  : 큐브 테이블 배치 (XY + yaw 다양성)

회전 규약은 robot_comm.euler_deg_to_matrix 와 동일: TCP (x,y,z, rz,ry,rx[deg]),
R = Rz @ Ry @ Rx (ZYX extrinsic). 위치는 mm 그대로 사용.

사용:
  # data/session 안의 최신 세션(_NNN) 파일 자동 탐색
  python test_robot_teaching_waypoint.py --root data/session

  # 파일 직접 지정
  python test_robot_teaching_waypoint.py \
      --grip data/session/grip_poses_001.json \
      --poses data/session/capture_poses_001.json \
      --sets data/session/capture_sets_001.json

종료 코드: 모든 필수(PASS/FAIL) 항목 통과 시 0, 하나라도 FAIL 이면 1.

# 다양성 임계값 (PASS/FAIL 기준)
#
   지표                    | pose   | grip    | set
   ------------------------+--------+---------+----------------
   회전축 3번째 특이값        | >=0.40 | >=0.40  | -
   상대회전 median          | >=60   | >=50    | -            (deg)
   콘 반각 max              | >=45   | >=40    | -            (deg)
   깊이 z 스팬              | -      | >=80    | -            (mm)
   yaw range / max-gap    | -      | -       | >=300 / <=60 (deg)
"""
import argparse
import glob
import itertools
import json
import os
import re
import sys

import numpy as np


# ----------------------------------------------------------------------------
# 회전 유틸 (robot_comm.euler_deg_to_matrix 규약과 일치)
# ----------------------------------------------------------------------------
def euler_deg_to_R(rz_deg, ry_deg, rx_deg):
    rx, ry, rz = np.deg2rad([rx_deg, ry_deg, rz_deg])
    Rz = np.array([[np.cos(rz), -np.sin(rz), 0],
                   [np.sin(rz),  np.cos(rz), 0],
                   [0, 0, 1]], dtype=np.float64)
    Ry = np.array([[np.cos(ry), 0, np.sin(ry)],
                   [0, 1, 0],
                   [-np.sin(ry), 0, np.cos(ry)]], dtype=np.float64)
    Rx = np.array([[1, 0, 0],
                   [0, np.cos(rx), -np.sin(rx)],
                   [0, np.sin(rx),  np.cos(rx)]], dtype=np.float64)
    return Rz @ Ry @ Rx


def log_SO3(R):
    """회전행렬 -> 회전벡터(axis*angle, rad)."""
    c = np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0)
    th = np.arccos(c)
    if th < 1e-8:
        return np.zeros(3)
    w = np.array([R[2, 1] - R[1, 2],
                  R[0, 2] - R[2, 0],
                  R[1, 0] - R[0, 1]]) / (2.0 * np.sin(th))
    return w * th


# ----------------------------------------------------------------------------
# 파일 로드 / 자동 탐색
# ----------------------------------------------------------------------------
def _latest_session_file(root, base):
    """root 안에서 base_NNN.json 중 번호가 가장 큰 것. 없으면 base.json 폴백."""
    cands = glob.glob(os.path.join(root, "{}_[0-9]*.json".format(base)))
    best, best_n = None, -1
    for p in cands:
        m = re.search(r"_(\d+)\.json$", os.path.basename(p))
        if m and int(m.group(1)) > best_n:
            best, best_n = p, int(m.group(1))
    if best:
        return best
    plain = os.path.join(root, "{}.json".format(base))
    return plain if os.path.exists(plain) else None


def _load_list(path, key):
    if not path or not os.path.exists(path):
        return None, "파일 없음: {}".format(path)
    with open(path) as f:
        data = json.load(f)
    items = data.get(key)
    if not isinstance(items, list) or not items:
        return None, "{} 에 '{}' 리스트가 비어있음".format(path, key)
    return items, None


def _get6(item, keys):
    """item 에서 keys 후보 중 처음 발견되는 6-벡터 반환."""
    for k in keys:
        v = item.get(k)
        if isinstance(v, list) and len(v) == 6:
            return [float(x) for x in v]
    return None


# ----------------------------------------------------------------------------
# 다양성 지표 계산
# ----------------------------------------------------------------------------
def compute_metrics(poses6):
    """poses6: [[x,y,z,rz,ry,rx], ...] -> 지표 dict."""
    P = np.array([p[:3] for p in poses6], dtype=np.float64)
    Rs = [euler_deg_to_R(p[3], p[4], p[5]) for p in poses6]
    n = len(poses6)

    ext = (P.max(0) - P.min(0)) if n else np.zeros(3)

    # tool/뷰 +Z 축 (광축 근사) 의 콘 반각
    zc = np.array([R[:, 2] for R in Rs])
    mz = zc.mean(0)
    mz = mz / (np.linalg.norm(mz) + 1e-12)
    cone = np.degrees(np.arccos(np.clip(zc @ mz, -1, 1)))

    # 쌍별 상대회전 각도 + 축
    rels, axes = [], []
    for i, j in itertools.combinations(range(n), 2):
        v = log_SO3(Rs[i] @ Rs[j].T)
        a = np.linalg.norm(v)
        rels.append(np.degrees(a))
        if a > 1e-3:
            axes.append(v / a)
    rels = np.array(rels) if rels else np.array([0.0])

    # 회전축 span: 상대회전 축들의 정규화 특이값 (3번째가 0에 가까우면 회전이 동일평면/축)
    if len(axes) >= 3:
        sv = np.linalg.svd(np.array(axes), compute_uv=False) / np.sqrt(len(axes))
    else:
        sv = np.zeros(3)
    sv = np.pad(sv, (0, max(0, 3 - len(sv))))[:3]

    # yaw(rz) 커버리지
    yaw = np.sort(np.array([((p[3] + 180.0) % 360.0) - 180.0 for p in poses6]))
    if n >= 2:
        gaps = np.diff(np.concatenate([yaw, [yaw[0] + 360.0]]))
        yaw_max_gap = float(gaps.max())
        yaw_range = float(yaw.max() - yaw.min())
    else:
        yaw_max_gap, yaw_range = 360.0, 0.0

    return {
        "n": n,
        "pos_span": ext,                       # [dx,dy,dz] mm
        "z_span": float(ext[2]),
        "cone_max": float(cone.max()),
        "rel_median": float(np.median(rels)),
        "rel_max": float(rels.max()),
        "sv": sv,                              # 회전축 특이값 3개
        "yaw_range": yaw_range,
        "yaw_max_gap": yaw_max_gap,
    }


# ----------------------------------------------------------------------------
# 임계값 체크 (README/분석 기준). check: (label, value, op, thr, kind)
#   kind='fail' 실패 시 종료코드 1, kind='warn' 은 경고만.
# ----------------------------------------------------------------------------
def _cmp(val, op, thr):
    return val >= thr if op == ">=" else val <= thr


def report_pool(title, m, checks, expected_n=None):
    print("=" * 72)
    print("{}   (N={})".format(title, m["n"]))
    print("-" * 72)
    print("  position span  x={:.0f}  y={:.0f}  z={:.0f} mm"
          .format(*m["pos_span"]))
    print("  view/tool +Z cone half-angle max : {:.1f} deg".format(m["cone_max"]))
    print("  pairwise rel-rot  median={:.1f}  max={:.1f} deg"
          .format(m["rel_median"], m["rel_max"]))
    print("  rotation-axis span (singular vals): [{:.2f} {:.2f} {:.2f}]"
          .format(*m["sv"]))
    print("  yaw(rz) range={:.0f} deg  max-gap={:.0f} deg"
          .format(m["yaw_range"], m["yaw_max_gap"]))
    print("-" * 72)

    n_fail = 0
    if expected_n is not None and m["n"] != expected_n:
        print("  [WARN] N={} (예상 {}개와 다름)".format(m["n"], expected_n))
    for label, val, op, thr, kind in checks:
        ok = _cmp(val, op, thr)
        tag = "PASS" if ok else ("FAIL" if kind == "fail" else "WARN")
        if not ok and kind == "fail":
            n_fail += 1
        print("  [{}] {:<34} {:.2f} {} {:.2f}"
              .format(tag, label, val, op, thr))
    print("")
    return n_fail


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--root", default="data/session",
                    help="세션 파일 폴더 (base_NNN.json 자동 탐색)")
    ap.add_argument("--grip", default=None, help="grip_poses 파일 직접 지정")
    ap.add_argument("--poses", default=None, help="capture_poses 파일 직접 지정")
    ap.add_argument("--sets", default=None, help="capture_sets 파일 직접 지정")
    ap.add_argument("--n_set", type=int, default=10, help="예상 set 개수 (경고용)")
    ap.add_argument("--n_grip", type=int, default=13, help="예상 grip 개수 (경고용)")
    ap.add_argument("--n_pose", type=int, default=18, help="예상 pose 개수 (경고용)")
    args = ap.parse_args()

    grip_path = args.grip or _latest_session_file(args.root, "grip_poses")
    poses_path = args.poses or _latest_session_file(args.root, "capture_poses")
    sets_path = args.sets or _latest_session_file(args.root, "capture_sets")

    print("\n[FILES]")
    print("  grip  : {}".format(grip_path))
    print("  poses : {}".format(poses_path))
    print("  sets  : {}\n".format(sets_path))

    grip, e1 = _load_list(grip_path, "grip_poses")
    poses, e2 = _load_list(poses_path, "capture_poses")
    sets, e3 = _load_list(sets_path, "capture_sets")
    for e in (e1, e2, e3):
        if e:
            print("[ERROR] {}".format(e))
    if grip is None or poses is None or sets is None:
        sys.exit(2)

    # --- 구조 검증 (빌더 build_waypoints_from_pool.py 요구 필드) ---
    struct_fail = 0
    for i, p in enumerate(grip):
        if _get6(p, ["capture_tcp"]) is None:
            print("[STRUCT-FAIL] grip[{}] capture_tcp(6) 없음 (B는 TCP 앵커 필수)".format(i))
            struct_fail += 1
    for i, p in enumerate(poses):
        if _get6(p, ["capture_joints"]) is None:
            print("[STRUCT-FAIL] poses[{}] capture_joints(6) 없음".format(i))
            struct_fail += 1
    for i, s in enumerate(sets):
        if _get6(s, ["place_joints"]) is None:
            print("[STRUCT-FAIL] sets[{}] place_joints(6) 없음".format(i))
            struct_fail += 1
        if _get6(s, ["set_cube_center_6dof"]) is None:
            print("[STRUCT-FAIL] sets[{}] set_cube_center_6dof(6) 없음".format(i))
            struct_fail += 1
    if struct_fail:
        print("")

    # --- 지표: 자세는 capture_tcp(그리퍼캠/큐브 6dof), set은 큐브중점으로 평가 ---
    m_pose = compute_metrics([_get6(p, ["capture_tcp"]) or p["capture_joints"]
                              for p in poses])
    m_grip = compute_metrics([_get6(p, ["capture_tcp"]) for p in grip])
    m_set = compute_metrics([_get6(s, ["set_cube_center_6dof"]) for s in sets])

    total_fail = struct_fail

    # pose (eye-in-hand): 3축 회전 + 넓은 뷰. 기존 baseline 이 우수했으므로 유지.
    total_fail += report_pool(
        "capture_poses (A: 그리퍼캠 뷰포인트 / eye-in-hand gTc)", m_pose,
        [("rot-axis 3rd singular val", m_pose["sv"][2], ">=", 0.40, "fail"),
         ("pairwise rel-rot median deg", m_pose["rel_median"], ">=", 60.0, "fail"),
         ("view cone half-angle max deg", m_pose["cone_max"], ">=", 45.0, "fail")],
        expected_n=args.n_pose)

    # grip (eye-to-hand): 지난번 약점. 3번째 회전축 + 깊이(z) 를 특히 본다.
    total_fail += report_pool(
        "grip_poses (B: 큐브 그립 스윕 / eye-to-hand)", m_grip,
        [("rot-axis 3rd singular val", m_grip["sv"][2], ">=", 0.40, "fail"),
         ("pairwise rel-rot median deg", m_grip["rel_median"], ">=", 50.0, "fail"),
         ("tilt cone half-angle max deg", m_grip["cone_max"], ">=", 40.0, "fail"),
         ("depth z span mm", m_grip["z_span"], ">=", 80.0, "fail")],
        expected_n=args.n_grip)

    # set: yaw 전범위 + XY 확산. 평면 배치라 회전은 yaw 중심.
    total_fail += report_pool(
        "capture_sets (큐브 배치: XY + yaw)", m_set,
        [("yaw range deg", m_set["yaw_range"], ">=", 300.0, "fail"),
         ("yaw max-gap deg", m_set["yaw_max_gap"], "<=", 60.0, "fail"),
         ("XY span x mm", m_set["pos_span"][0], ">=", 150.0, "warn"),
         ("XY span y mm", m_set["pos_span"][1], ">=", 150.0, "warn")],
        expected_n=args.n_set)

    print("=" * 72)
    if total_fail == 0:
        print("RESULT: ALL PASS  ->  다양성 충분. build_waypoints_from_pool.py 로 진행 가능.")
        print("=" * 72)
        sys.exit(0)
    else:
        print("RESULT: {} FAIL 항목  ->  해당 축을 더 다양하게 재티칭 권장.".format(total_fail))
        print("  (grip 은 큐브를 앞뒤/좌우로 ±30~45deg 눕히고 z를 ±40mm 이상 흔들 것)")
        print("=" * 72)
        sys.exit(1)


if __name__ == "__main__":
    main()
