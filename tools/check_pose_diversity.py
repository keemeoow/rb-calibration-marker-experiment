#!/usr/bin/env python3
"""티칭한 A/B 포즈 풀이 각자 제약하는 양에 충분한 다양성을 갖는지 검사한다.

두 블록의 요구는 정반대다.

  A (capture_poses): 손목 카메라가 보드를 보며 `T_gripper_cam` 6-DoF 를 제약한다.
      필요한 것은 **상대회전 다양성**이다. 회전이 한 축으로만 몰리면 그 축에
      수직인 성분이 관측되지 않아, 각도를 아무리 크게 줘도 gTc 가 풀리지 않는다.

  B (grip_poses): 소비처는 `CP_common.estimate_robot_pos_scale` 하나이고, 그
      추정기는 상대회전 ≤5° 이면서 변위 ≥40 mm 인 **pure-translation 쌍**만 쓴다.
      즉 자세를 고정한 채 위치만 벌려야 하며, 자세가 다양하면 유효쌍이 0이 된다.

per_set_AB 레이아웃에서는 빌더가 B 패턴을 set 마다 평행이동시켜 주므로 자세가
다양해도 set 간 쌍이 생겼다. A_sets_plus_B_station 에서는 스테이션 한 곳에서만
찍으므로 그 보정이 사라진다. 이 검사는 스테이션 기준으로 계산한다.

사용:
  python tools/check_pose_diversity.py \
      --poses data/sessionNN/teaching/capture_poses_001.json \
      --grip  data/sessionNN/teaching/grip_poses_001.json
"""
import argparse
import itertools
import json
import os
import sys

import numpy as np

_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)
from robot_comm import euler_deg_to_matrix

# estimate_robot_pos_scale 의 쌍 선택 기준과 같은 값이어야 한다.
K_MAX_ROT_DEG = 5.0
K_MIN_DISP_MM = 40.0

# A 상대회전 축 공분산 고유값의 목표. 완전 균형은 [1/3, 1/3, 1/3]이지만 실제
# 워크스페이스에서 그렇게까지는 안 나온다. 가장 작은 축이 0.20 아래로 떨어지면
# 그 방향의 gTc 성분이 사실상 관측되지 않는다.
AXIS_MIN_EIGENVALUE = 0.20
A_MIN_PAIR_ANGLE_DEG = 30.0

# 티칭 목표치. k 는 스칼라 하나라 쌍이 아주 많을 필요는 없지만, 추정기가
# 카메라별 IQR 폭으로 신뢰도를 거르므로 한 자릿수로는 부족하다.
K_MIN_PAIRS = 20


def _load(path, key):
    if not os.path.exists(path):
        sys.exit(f"[ERROR] 파일 없음: {path}")
    with open(path) as handle:
        data = json.load(handle)
    items = data.get(key)
    if not isinstance(items, list) or not items:
        sys.exit(f"[ERROR] {path} 에 '{key}' 리스트가 비어있음")
    return items


def _poses(items):
    T = [euler_deg_to_matrix(*[float(v) for v in p["capture_tcp"]]) for p in items]
    return [t[:3, :3] for t in T], np.array([t[:3, 3] for t in T]) * 1000.0


def _rel_axis_angle(Ra, Rb):
    M = Ra.T @ Rb
    angle = np.degrees(np.arccos(np.clip((np.trace(M) - 1.0) / 2.0, -1.0, 1.0)))
    if angle < 1e-6:
        return np.array([0.0, 0.0, 1.0]), 0.0
    axis = np.array([M[2, 1] - M[1, 2], M[0, 2] - M[2, 0], M[1, 0] - M[0, 1]])
    return axis / np.linalg.norm(axis), float(angle)


def check_a(items):
    R, P = _poses(items)
    n = len(items)
    angles, axes = [], []
    for i, j in itertools.combinations(range(n), 2):
        axis, angle = _rel_axis_angle(R[i], R[j])
        angles.append(angle)
        if angle >= A_MIN_PAIR_ANGLE_DEG:
            # +v 와 -v 는 같은 축이므로 부호를 통일해야 공분산이 의미를 갖는다.
            axes.append(axis * np.sign(axis[np.argmax(np.abs(axis))]))
    angles = np.asarray(angles)
    print(f"\n=== A 포즈 (capture_poses) — 제약 대상: T_gripper_cam ===")
    print(f"  포즈 수: {n}")
    print(f"  쌍별 상대회전: 중앙값 {np.median(angles):.0f}°  "
          f"≥30° {100 * np.mean(angles >= 30):.0f}%  ≥60° {100 * np.mean(angles >= 60):.0f}%")
    ok = True
    if not axes:
        print(f"  [FAIL] 상대회전 {A_MIN_PAIR_ANGLE_DEG:.0f}° 이상인 쌍이 없다")
        return False
    A = np.asarray(axes)
    ev, evec = np.linalg.eigh(A.T @ A / len(A))
    ev, evec = ev[::-1], evec[:, ::-1]
    print(f"  회전축 균형(고유값): {np.round(ev, 2)}   목표: 최소축 ≥ {AXIS_MIN_EIGENVALUE}")
    for k in range(3):
        print(f"    축{k + 1} {ev[k]:.2f}  방향 {np.round(evec[:, k], 2)}")
    if ev[2] < AXIS_MIN_EIGENVALUE:
        print(f"  [FAIL] 세 번째 회전축이 {ev[2]:.2f} 로 빈약하다. 방향 "
              f"{np.round(evec[:, 2], 2)} 축으로 도는 포즈를 더 딸 것 "
              f"(같은 지점을 보되 손목 롤/좌우 비스듬/위아래 중 빠진 것)")
        ok = False
    else:
        print(f"  [OK] 세 회전축 모두 확보")
    span = P.max(axis=0) - P.min(axis=0)
    print(f"  TCP 위치 폭: x {span[0]:.0f}  y {span[1]:.0f}  z {span[2]:.0f} mm")
    return ok


def check_a_covers_sets(pose_items, set_items, fov_deg):
    """A 포즈 하나가 13개 배치를 모두 프레임에 담는지 위치 기하만으로 검사한다.

    A 풀은 배치마다 관절 그대로 재생되므로, 한 포즈에서 어떤 배치의 큐브가 화각
    밖으로 나가면 그 (set, pose) 캡처는 통째로 버려진다. 광축 방향을 몰라도
    ``이 시점에서 배치들이 벌어져 보이는 각도``는 계산할 수 있고, 그 각도가 화각을
    넘으면 광축을 어디로 두든 전부 담을 수 없다. 즉 이건 필요조건 검사다.

    실제로는 광축이 정확히 중앙을 겨냥하지 못하므로 여유를 둔다.
    """
    _, P = _poses(pose_items)
    C = np.array([[float(v) for v in s["set_cube_center_6dof"][:3]] for s in set_items])
    print(f"\n=== A 포즈 x 큐브 배치 커버리지 (화각 {fov_deg:.0f}° 기준) ===")
    print(f"  배치 {len(C)}곳, 영역 대각 "
          f"{np.linalg.norm(C.max(axis=0) - C.min(axis=0)):.0f}mm")
    margin = 0.6 * fov_deg
    worst, tight, fail = 0.0, [], []
    for i in range(len(P)):
        v = C - P[i]
        v = v / np.linalg.norm(v, axis=1, keepdims=True)
        spread = float(np.degrees(np.arccos(np.clip(v @ v.T, -1.0, 1.0))).max())
        dist = np.linalg.norm(C - P[i], axis=1)
        worst = max(worst, spread)
        idx = pose_items[i].get("pose_index", i)
        if spread > fov_deg:
            fail.append((idx, spread, dist.min(), dist.max()))
        elif spread > margin:
            tight.append((idx, spread, dist.min(), dist.max()))
    print(f"  배치를 모두 담는 데 필요한 각도: 최대 {worst:.0f}°  "
          f"(여유 기준 {margin:.0f}°, 화각 {fov_deg:.0f}°)")
    for label, rows in (("FAIL", fail), ("TIGHT", tight)):
        for idx, spread, dmin, dmax in rows:
            print(f"  [{label}] pose_index {idx}: {spread:.0f}° 필요, "
                  f"배치까지 {dmin:.0f}~{dmax:.0f}mm")
    if fail:
        print(f"  [FAIL] {len(fail)}개 포즈는 화각 안에 배치를 다 담을 수 없다. "
              f"배치 중심에서 더 물러나 티칭할 것")
        return False
    if tight:
        print(f"  [WARN] {len(tight)}개 포즈는 여유가 적다. 광축이 배치 중심을 "
              f"정확히 겨냥하지 못하면 가장자리 배치가 잘릴 수 있다")
    else:
        print(f"  [OK] 모든 포즈가 여유 있게 13배치를 담는다")
    return True


def check_b(items):
    R, P = _poses(items)
    n = len(items)
    pairs, near = 0, 0
    for i, j in itertools.combinations(range(n), 2):
        _, angle = _rel_axis_angle(R[i], R[j])
        disp = float(np.linalg.norm(P[i] - P[j]))
        if angle <= K_MAX_ROT_DEG:
            near += 1
            if disp >= K_MIN_DISP_MM:
                pairs += 1
    total = n * (n - 1) // 2
    print(f"\n=== B 포즈 (grip_poses) — 제약 대상: 로봇 스케일 k ===")
    print(f"  포즈 수: {n}")
    print(f"  자세가 같은 쌍(≤{K_MAX_ROT_DEG:.0f}°): {near}/{total}")
    print(f"  k 추정 유효쌍(자세 같고 변위 ≥{K_MIN_DISP_MM:.0f}mm): {pairs}/{total}   "
          f"목표 ≥ {K_MIN_PAIRS}")
    if pairs < K_MIN_PAIRS:
        print(f"  [FAIL] 유효쌍이 부족하다. 자세를 하나로 고정한 채 위치만 "
              f"≥100mm 씩 벌린 포즈를 {K_MIN_PAIRS} 쌍이 나올 만큼 딸 것 "
              f"(같은 자세 m개면 쌍은 m(m-1)/2 개: 7개→21, 10개→45)")
        return False
    print(f"  [OK] k 추정 가능")
    return True


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--poses", help="A placement 뷰포인트 풀 (recpose 출력)")
    ap.add_argument("--grip", help="B 그립-스윕 포즈 풀 (recgrip 출력)")
    ap.add_argument("--sets", help="큐브 배치 (recset 출력). 주면 A 포즈가 모든 배치를 "
                                   "화각 안에 담는지도 검사한다")
    ap.add_argument("--fov_deg", type=float, default=69.0,
                    help="컬러 카메라 수평 화각 (기본 69, RealSense D435)")
    args = ap.parse_args()
    if not args.poses and not args.grip:
        ap.error("--poses 또는 --grip 중 최소 하나가 필요하다")
    if args.sets and not args.poses:
        ap.error("--sets 는 --poses 와 함께 써야 한다")

    ok = True
    if args.poses:
        pose_items = _load(args.poses, "capture_poses")
        ok &= check_a(pose_items)
        if args.sets:
            ok &= check_a_covers_sets(
                pose_items, _load(args.sets, "capture_sets"), args.fov_deg)
    if args.grip:
        ok &= check_b(_load(args.grip, "grip_poses"))
    print()
    if not ok:
        sys.exit("[RESULT] 다양성 부족 — 위 지적대로 포즈를 보강할 것")
    print("[RESULT] 두 풀 모두 통과")


if __name__ == "__main__":
    main()
