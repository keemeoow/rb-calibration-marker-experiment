#!/usr/bin/env python3
"""촬영 포즈 풀 + 큐브 set 배치 -> capture_waypoints.json 생성.

두 가지 촬영 레이아웃을 지원한다. 어느 쪽인지는 출력 파일의 `capture_protocol`
로 선언되고, 서버가 첫 모션 전에 그 모양 그대로 검증한다.

  per_set_AB (기존)
      set 마다 B 스윕 -> 큐브 내려놓기 -> A placement.

  A_sets_plus_B_station (--b_station_set 지정 시)
      set 마다 A placement 만 찍고, 마지막에 큐브를 한 스테이션으로 옮겨
      거기서만 B 스윕을 한 번 한다. B 는 solve 에 들어가지 않고
      (CP_ablation_7row 의 exclude_gripped_cube=True) 소비처가
      CP_common.estimate_robot_pos_scale 하나뿐이므로 set 마다 반복할 이유가 없다.
      그 추정기는 상대회전이 거의 없는 pure-translation 쌍만 쓰므로, 스테이션
      포즈 중 일부는 자세를 고정한 채 위치만 벌려 티칭해야 한다.

입력 (로봇 서버의 recgrip/recpose/recset 로 기록 후 PC로 옮긴 파일):
  --grip   grip_poses.json    : {"grip_poses":[{pose_index, capture_joints[6], ...}]}
  --poses  capture_poses.json : {"capture_poses":[{pose_index, capture_joints[6], ...}]}
  --sets   capture_sets.json  : {"capture_sets":[{set_index, place_joints[6],
                                  set_cube_center_6dof[6], ...}]}

출력 (server/c1.py _run_auto_multiset 이 소비하는 포맷):
  waypoints[] 각 항목에 capture_block("B_eyetohand"|"A_placement") + cube_gripped 태그 포함.

예 (A-only sets + 마지막 B 스테이션, sets 마지막 항목이 스테이션):
  python tools/build_waypoints_from_pool.py \
      --grip ./grip_poses.json --poses ./capture_poses.json --sets ./capture_sets.json \
      --output ./data/session/capture_waypoints.json \
      --safe_joints_empty d1,d2,d3,d4,d5,d6 \
      --safe_joints_gripped d1,d2,d3,d4,d5,d6 \
      --n_per_set 6 --b_station_set 13
"""
import argparse
import json
import math
import os
import random
import sys

_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)
import numpy as np

from robot_comm import euler_deg_to_matrix
from waypoint_safety import (
    CAPTURE_PROTOCOL_KEY,
    PROTOCOL_A_SETS_B_STATION,
    PROTOCOL_PER_SET_AB,
    SAFE_MODE_KEY,
    SAFE_MODE_Z_LIFT,
    validate_joint_vector,
    validate_safe_joint_config,
    validate_waypoint_semantics,
)

# Per-set yield measured on CP_result/ablation_7row_canonical/seven_row_ablation.json
# (13 sets x 11 A views): train_eih_cube_events / A views = 0.75 mean, 0.55 worst.
# A set that lands under the split contract's minimum is dropped from the solve,
# and with it one position from an already-marginal position holdout.
A_VIEW_TO_TRAIN_EIH_MEAN = 0.75
A_VIEW_TO_TRAIN_EIH_WORST = 0.55
MIN_TRAIN_EIH_CUBE_EVENTS = 3  # CP_ablation_7row split contract

# One degree of viewing-angle change is worth about this much lateral travel at
# the ~500 mm working distance, so orientation and position enter the pose
# distance on a comparable footing.
CLUSTER_MM_PER_DEG = 10.0


def _load(path, key):
    if not os.path.exists(path):
        sys.exit(f"[ERROR] 파일 없음: {path}")
    with open(path) as f:
        data = json.load(f)
    items = data.get(key)
    if not isinstance(items, list) or not items:
        sys.exit(f"[ERROR] {path} 에 '{key}' 리스트가 비어있음")
    return items


def _check6(vec, path, label, idx):
    if not isinstance(vec, list) or len(vec) != 6:
        sys.exit(f"[ERROR] {path} {label}[{idx}] 가 6-벡터가 아님: {vec}")


def _pick(rng, pool, n, allow_repeat):
    if n <= 0:
        return []
    if n > len(pool) and not allow_repeat:
        return None  # 호출부에서 에러 처리
    if allow_repeat and n > len(pool):
        return [rng.choice(pool) for _ in range(n)]
    return rng.sample(pool, n)


def _pose_distance(a, b):
    """Viewpoint distance between two taught poses: rotation deg + scaled mm.

    The A pool is replayed as joint values at every placement, so two poses are
    interchangeable views of a set only if the wrist ends up somewhere else AND
    looking somewhere else.  Both terms matter, hence the combined metric.
    """
    Ta = euler_deg_to_matrix(*[float(v) for v in a])
    Tb = euler_deg_to_matrix(*[float(v) for v in b])
    cos = (float(np.trace(Ta[:3, :3] @ Tb[:3, :3].T)) - 1.0) / 2.0
    rot_deg = math.degrees(math.acos(max(-1.0, min(1.0, cos))))
    trans_mm = float(np.linalg.norm(Ta[:3, 3] - Tb[:3, 3])) * 1000.0
    return rot_deg + trans_mm / CLUSTER_MM_PER_DEG


def _viewpoint_clusters(poses, k):
    """Split the A pool into k viewpoint clusters, spread seeds first.

    Farthest-point seeding picks k mutually distant poses, then every remaining
    pose joins its nearest seed.  Handing each set one pose per cluster is what
    makes a small n_per_set safe: the set still spans the whole pool instead of
    whatever a random draw happened to concentrate on.
    """
    if k > len(poses):
        return None  # --allow_repeat 로 풀보다 많이 뽑는 경우 -> 호출부가 랜덤으로 폴백
    pose6 = [p["capture_tcp"] if isinstance(p.get("capture_tcp"), list) else None
             for p in poses]
    if any(v is None for v in pose6):
        return None  # 자세 정보 없음 -> 호출부가 랜덤으로 폴백
    seeds = [0]
    while len(seeds) < k:
        best, best_d = None, -1.0
        for i in range(len(poses)):
            if i in seeds:
                continue
            d = min(_pose_distance(pose6[i], pose6[s]) for s in seeds)
            if d > best_d:
                best, best_d = i, d
        seeds.append(best)
    clusters = [[s] for s in seeds]
    for i in range(len(poses)):
        if i in seeds:
            continue
        nearest = min(range(k), key=lambda c: _pose_distance(pose6[i], pose6[seeds[c]]))
        clusters[nearest].append(i)
    return clusters


def _balanced_pick(poses, clusters, n, set_ordinal):
    """One pose from each of n clusters, rotating members across sets.

    Every set gets a full spread of viewpoints, and every pool pose is used a
    near-equal number of times over the session.
    """
    chosen = []
    for c in range(n):
        members = clusters[c]
        # De-phase by cluster index so clusters do not advance in lockstep and
        # consecutive sets draw different members.  Without it, equal-sized
        # clusters make every even set an exact repeat of every other even set.
        chosen.append(poses[members[(set_ordinal + c) % len(members)]])
    return chosen


def _joint_csv(value):
    try:
        parsed = [float(x.strip()) for x in value.split(",")]
        return validate_joint_vector(parsed, "safe joints")
    except (TypeError, ValueError) as exc:
        raise argparse.ArgumentTypeError(str(exc))


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--poses", default="./capture_poses.json",
                    help="A placement 뷰포인트 풀 (recpose 출력)")
    ap.add_argument("--grip", default="./grip_poses.json",
                    help="B 그립-스윕 포즈 풀 (recgrip 출력)")
    ap.add_argument("--sets", default="./capture_sets.json",
                    help="큐브 set 배치 (recset 출력)")
    ap.add_argument("--output", default="./data/session/capture_waypoints.json",
                    help="출력 waypoint JSON (서버 소비 포맷)")
    ap.add_argument("--no_safe_joints", action="store_true",
                    help="안전자세 없이 촬영한다. 로봇은 매 전이에서 안전자세 대신 +Z 로 "
                         "리트랙트한다. 생성 파일에 safe_pose_mode=z_lift_only 로 남아 "
                         "나중에 이 세션이 안전자세 없이 찍혔음을 추적할 수 있다.")
    ap.add_argument("--safe_joints_empty", type=_joint_csv, default=None,
                    help="물리 검증한 empty-gripper safe joint 6개(쉼표 구분)")
    ap.add_argument("--safe_joints_gripped", type=_joint_csv, default=None,
                    help="물리 검증한 gripped-cube safe joint 6개(쉼표 구분)")
    ap.add_argument("--n_per_set", type=int, default=5,
                    help="set 당 A placement 포즈 수 (기본 5)")
    ap.add_argument("--n_grip_per_set", type=int, default=10,
                    help="per_set_AB 레이아웃에서 set 당 B 그립-스윕 포즈 수 (기본 10)")
    ap.add_argument("--b_station_set", type=int, default=-1,
                    help="A_sets_plus_B_station 레이아웃으로 생성한다. 값은 B 스테이션으로 "
                         "쓸 set 의 인덱스(0-based). 그 set 은 A 없이 B 만, 나머지 set 은 "
                         "B 없이 A 만 찍고, 스테이션은 항상 마지막에 실행된다. "
                         "-1 이면 기존 per_set_AB 레이아웃")
    ap.add_argument("--n_grip_at_station", type=int, default=0,
                    help="B 스테이션에서 찍을 grip 포즈 수. 0 이면 풀 전체를 티칭 순서대로 "
                         "모두 사용한다(권장: 티칭 자체를 촬영 계획으로 삼는다)")
    ap.add_argument("--a_assign", choices=("balanced", "random"), default="balanced",
                    help="set 별 A 포즈 배정 방식. balanced(기본)는 풀을 n_per_set 개 "
                         "viewpoint 클러스터로 나눠 set 마다 각 클러스터에서 하나씩 뽑는다. "
                         "random 은 기존 무작위 추출")
    ap.add_argument("--b_ref_set", type=int, default=-1,
                    help="B 스윕 패턴을 기록한 기준 set 의 인덱스(0-based). 그 set 의 "
                         "큐브중심(C_ref)을 기준으로 각 set 큐브중심으로 x,y,z 평행이동. "
                         "미지정이면 --b_station_set (없으면 0)")
    ap.add_argument("--seed", type=int, default=0,
                    help="랜덤 시드 (--a_assign random 및 per_set_AB 의 B 추출용). "
                         "-1 이면 매번 다르게")
    ap.add_argument("--allow_repeat", action="store_true",
                    help="n_*_per_set 이 풀보다 클 때 중복 허용(복원 추출)")
    ap.add_argument("--preview", action="store_true",
                    help="파일을 쓰지 않고 배정 결과만 출력")
    args = ap.parse_args()

    poses = _load(args.poses, "capture_poses")
    sets = _load(args.sets, "capture_sets")
    for i, p in enumerate(poses):
        _check6(p.get("capture_joints"), args.poses, "capture_joints", i)
    for i, s in enumerate(sets):
        _check6(s.get("place_joints"), args.sets, "place_joints", i)
        _check6(s.get("set_cube_center_6dof"), args.sets, "set_cube_center_6dof", i)

    station = args.b_station_set
    use_station = station >= 0
    protocol = PROTOCOL_A_SETS_B_STATION if use_station else PROTOCOL_PER_SET_AB
    if use_station and not (0 <= station < len(sets)):
        sys.exit(f"[ERROR] --b_station_set {station} 범위 밖 (0..{len(sets)-1})")

    b_ref_set = args.b_ref_set
    if b_ref_set < 0:
        b_ref_set = station if use_station else 0

    grip = _load(args.grip, "grip_poses")
    # B 는 TCP 로 앵커되므로 capture_tcp 가 필수 (관절은 평행이동 불가).
    for i, p in enumerate(grip):
        _check6(p.get("capture_tcp"), args.grip, "capture_tcp", i)
    if not (0 <= b_ref_set < len(sets)):
        sys.exit(f"[ERROR] --b_ref_set {b_ref_set} 범위 밖 (0..{len(sets)-1})")
    c_ref = [float(x) for x in sets[b_ref_set]["set_cube_center_6dof"]]
    print(f"[INFO] B 기준 set index={b_ref_set}  "
          f"C_ref(xyz)={[round(v, 1) for v in c_ref[:3]]}"
          + ("  (= 스테이션이므로 평행이동 0, 티칭 TCP 그대로)"
             if use_station and b_ref_set == station else ""))

    n_station_grip = args.n_grip_at_station or len(grip)
    if args.n_per_set <= 0:
        sys.exit("[ERROR] --n_per_set 은 1 이상이어야 함")
    if not use_station and args.n_grip_per_set <= 0:
        sys.exit("[ERROR] per_set_AB 레이아웃은 모든 set에 A/B가 모두 필요하므로 "
                 "--n_grip_per_set 은 1 이상이어야 함. B 를 한 번만 찍으려면 "
                 "--b_station_set 으로 A_sets_plus_B_station 레이아웃을 쓸 것")
    if args.n_per_set > len(poses) and not args.allow_repeat:
        sys.exit(f"[ERROR] --n_per_set {args.n_per_set} > A 포즈 수 {len(poses)} "
                 f"(--allow_repeat 또는 포즈 추가)")
    n_b_draw = n_station_grip if use_station else args.n_grip_per_set
    if n_b_draw > len(grip) and not args.allow_repeat:
        sys.exit(f"[ERROR] B 추출 수 {n_b_draw} > B 포즈 수 {len(grip)} "
                 f"(--allow_repeat 또는 포즈 추가)")

    rng = random.Random(None if args.seed < 0 else args.seed)
    n_a_sets = len(sets) - 1 if use_station else len(sets)
    print(f"[INFO] protocol={protocol}  A_poses={len(poses)}  B_grip_poses={len(grip)}  "
          f"sets={len(sets)}  n_per_set(A)={args.n_per_set}  "
          f"a_assign={args.a_assign}  seed={'random' if args.seed < 0 else args.seed}")

    clusters = None
    if args.a_assign == "balanced":
        clusters = _viewpoint_clusters(poses, args.n_per_set)
        if clusters is None:
            why = (f"n_per_set {args.n_per_set} > A 포즈 수 {len(poses)} (복원 추출)"
                   if args.n_per_set > len(poses)
                   else "A 풀에 capture_tcp 가 없다")
            print(f"[WARN] viewpoint 클러스터를 만들 수 없다: {why}. 랜덤 배정으로 폴백한다.")
        else:
            spread = [len(c) for c in clusters]
            print(f"[INFO] viewpoint 클러스터 {args.n_per_set}개 크기: {spread} "
                  f"(set 마다 각 클러스터에서 1개)")

    # A 뷰 수가 split 계약을 만족하는지 촬영 전에 알린다. 사후에 set 이 탈락하면
    # position holdout 에서 위치 하나가 통째로 사라진다.
    proj_mean = args.n_per_set * A_VIEW_TO_TRAIN_EIH_MEAN
    proj_worst = args.n_per_set * A_VIEW_TO_TRAIN_EIH_WORST
    print(f"[INFO] set 당 train_eih_cube_events 예상: 평균 {proj_mean:.1f}, "
          f"최악 set {proj_worst:.1f}  (계약 최소 {MIN_TRAIN_EIH_CUBE_EVENTS})")
    if proj_worst < MIN_TRAIN_EIH_CUBE_EVENTS:
        print(f"[WARN] n_per_set={args.n_per_set} 에서는 전환율이 나쁜 set 이 계약 최소치 "
              f"{MIN_TRAIN_EIH_CUBE_EVENTS} 아래로 떨어져 solve 에서 탈락할 수 있다. "
              f"탈락하면 그 위치는 position holdout 에서 사라진다. "
              f"촬영 후 set 별 등록 수를 반드시 확인할 것.")

    # 스테이션은 항상 마지막에 실행한다: 서버는 남은 set 이 있을 때만 큐브를
    # 재-그립하므로(_run_auto_multiset), 중간에 스테이션이 오면 그 뒤 배치들이
    # 큐브가 놓인 채 남는다.
    order = [i for i in range(len(sets)) if i != station] if use_station else list(range(len(sets)))
    if use_station:
        order.append(station)

    waypoints = []
    capture_index = 0
    for ordinal, si in enumerate(order):
        s = sets[si]
        set_index = int(s.get("set_index", si))
        place_joints = [float(x) for x in s["place_joints"]]
        # 로봇이 set 사이를 수평 이동할 때 목표 XY/자세로 쓴다. z 는 서버가
        # PLACE_TCP_Z_MM 으로 정규화하므로 티칭 당시 tool 값이어도 무방하다.
        place_tcp = [float(x) for x in s["place_tcp"]] if "place_tcp" in s else None
        set_cc = [float(x) for x in s["set_cube_center_6dof"]]

        if use_station:
            is_station = si == station
            b_sel = grip[:n_station_grip] if is_station else []
            a_sel = [] if is_station else None
        else:
            is_station = False
            b_sel = _pick(rng, grip, args.n_grip_per_set, args.allow_repeat)
            a_sel = None
        if a_sel is None:
            a_sel = (_balanced_pick(poses, clusters, args.n_per_set, ordinal)
                     if clusters is not None
                     else _pick(rng, poses, args.n_per_set, args.allow_repeat))

        print(f"  set_index={set_index}{' [B station]' if is_station else ''}: "
              f"B={[p.get('pose_index') for p in b_sel]}  "
              f"A={[p.get('pose_index') for p in a_sel]}")

        # --- Phase B (먼저): TCP 앵커. 기준 set 대비 (set_cc - c_ref) 만큼 x,y,z 평행이동.
        #     자세(rz,ry,rx)는 그대로. 서버는 이 capture_tcp 를 line 으로 실행. ---
        for p in b_sel:
            tcp_i = [float(x) for x in p["capture_tcp"]]
            b_tcp = [
                round(set_cc[0] + (tcp_i[0] - c_ref[0]), 3),
                round(set_cc[1] + (tcp_i[1] - c_ref[1]), 3),
                round(set_cc[2] + (tcp_i[2] - c_ref[2]), 3),
                tcp_i[3], tcp_i[4], tcp_i[5],
            ]
            waypoints.append({
                "capture_index": capture_index,
                "set_index": set_index,
                "place_joints": place_joints,
                "place_tcp": place_tcp,
                "set_cube_center_6dof": set_cc,
                "capture_block": "B_eyetohand",
                "cube_gripped": True,
                "capture_tcp": b_tcp,          # 관절 없음 -> 서버가 line 이동
                "pose_index": p.get("pose_index"),
                "b_ref_set": args.b_ref_set,
            })
            capture_index += 1

        # --- Phase A (나중): 관절값 그대로 (placement) ---
        for p in a_sel:
            wp = {
                "capture_index": capture_index,
                "set_index": set_index,
                "capture_joints": [float(x) for x in p["capture_joints"]],
                "place_joints": place_joints,
                "place_tcp": place_tcp,
                "set_cube_center_6dof": set_cc,
                "capture_block": "A_placement",
                "cube_gripped": False,
                "pose_index": p.get("pose_index"),
            }
            if isinstance(p.get("capture_tcp"), list):
                wp["capture_tcp"] = [float(x) for x in p["capture_tcp"]]
            waypoints.append(wp)
            capture_index += 1

    n_b = sum(1 for w in waypoints if w["capture_block"] == "B_eyetohand")
    n_a = sum(1 for w in waypoints if w["capture_block"] == "A_placement")
    if args.no_safe_joints:
        if args.safe_joints_empty or args.safe_joints_gripped:
            sys.exit("[ERROR] --no_safe_joints 와 --safe_joints_* 를 함께 줄 수 없다")
        print("[WARN] 안전자세 없이 생성한다 (safe_pose_mode=z_lift_only).")
        print("       로봇은 매 전이에서 안전자세 대신 +Z 로 리트랙트한다. "
              "저속 dry-run 으로 전 경로를 반드시 확인할 것.")
    elif not (args.safe_joints_empty and args.safe_joints_gripped):
        sys.exit("[ERROR] --safe_joints_empty 와 --safe_joints_gripped 가 모두 필요하다.\n"
                 "        안전자세 없이 진행하려면 --no_safe_joints 를 명시할 것.")

    first = sets[order[0]]
    out = {
        CAPTURE_PROTOCOL_KEY: protocol,
        SAFE_MODE_KEY: (SAFE_MODE_Z_LIFT if args.no_safe_joints else "taught_safe_pose"),
        "safe_joints_empty": args.safe_joints_empty,
        "safe_joints_gripped": args.safe_joints_gripped,
        "set_joints": [float(x) for x in first["place_joints"]],
        "set_tcp": ([float(x) for x in first["place_tcp"]]
                    if isinstance(first.get("place_tcp"), list) else None),
        "set_cube_center": [float(x) for x in first["set_cube_center_6dof"]],
        "waypoints": waypoints,
        "_meta": {
            "generator": "tools/build_waypoints_from_pool.py",
            "capture_protocol": protocol,
            "n_A_poses": len(poses),
            "n_B_grip_poses": len(grip),
            "n_sets": len(sets),
            "n_placement_sets": n_a_sets,
            "b_station_set": station if use_station else None,
            "n_grip_at_station": n_station_grip if use_station else None,
            "b_ref_set": b_ref_set,
            "n_per_set_A": args.n_per_set,
            "n_grip_per_set_B": None if use_station else args.n_grip_per_set,
            "a_assign": args.a_assign if clusters is not None else "random",
            "a_cluster_sizes": [len(c) for c in clusters] if clusters is not None else None,
            "seed": None if args.seed < 0 else args.seed,
            "allow_repeat": args.allow_repeat,
            "projected_train_eih_cube_per_set": {
                "mean": round(proj_mean, 2),
                "worst_set": round(proj_worst, 2),
                "contract_min": MIN_TRAIN_EIH_CUBE_EVENTS,
                "meets_contract_in_worst_set": bool(proj_worst >= MIN_TRAIN_EIH_CUBE_EVENTS),
                "rate_source": "CP_result/ablation_7row_canonical/seven_row_ablation.json",
            },
            "total_captures": len(waypoints),
            "total_B_eyetohand": n_b,
            "total_A_placement": n_a,
        },
    }
    validate_safe_joint_config(out)
    validate_waypoint_semantics(out)

    print(f"[INFO] 총 waypoints: {len(waypoints)}  (B eye-to-hand: {n_b}, A placement: {n_a})")

    if args.preview:
        print("[PREVIEW] 파일 미기록. 첫 set 의 첫 B/A waypoint:")
        for w in waypoints:
            if w["capture_block"] == "B_eyetohand":
                print("  B:", json.dumps(w, ensure_ascii=False)); break
        for w in waypoints:
            if w["capture_block"] == "A_placement":
                print("  A:", json.dumps(w, ensure_ascii=False)); break
        return

    out_dir = os.path.dirname(os.path.abspath(args.output))
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)
    with open(args.output, "w") as f:
        json.dump(out, f, indent=2)
    print(f"[OK] Wrote {args.output}")
    print("     -> 서버 'start' 시 PC가 이 파일을 전송. set마다 B(그립스윕)->큐브내림->A(placement).")


if __name__ == "__main__":
    main()
