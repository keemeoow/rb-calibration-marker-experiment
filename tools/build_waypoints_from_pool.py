#!/usr/bin/env python3
"""촬영 포즈 풀 + 큐브 set 배치 -> set별 랜덤 배정 capture_waypoints.json 생성.

set 마다 두 방법을 한 세션에 촬영하도록 두 종류 waypoint 를 함께 생성한다:
  - Phase B (block B, B_eyetohand): 큐브를 그립한 채 스윕 촬영 (method b, eye-to-hand)
        grip_poses.json 에서 --n_grip_per_set 개 랜덤 선택.
  - Phase A (block A, A_placement): 큐브를 바닥에 놓고 뷰포인트 촬영 (method a)
        capture_poses.json 에서 --n_per_set 개 랜덤 선택.
서버(_run_auto_multiset)는 set 마다 B 먼저(그립 스윕) -> 큐브 내려놓기 -> A(placement)
순서로 실행한다.

입력 (로봇 서버의 recgrip/recpose/recset 로 기록 후 PC로 옮긴 파일):
  --grip   grip_poses.json    : {"grip_poses":[{pose_index, capture_joints[6], ...}]}
  --poses  capture_poses.json : {"capture_poses":[{pose_index, capture_joints[6], ...}]}
  --sets   capture_sets.json  : {"capture_sets":[{set_index, place_joints[6],
                                  set_cube_center_6dof[6], ...}]}

출력 (server/robot_calb.py _run_auto_multiset 이 소비하는 포맷):
  waypoints[] 각 항목에 capture_block("B_eyetohand"|"A_placement") + cube_gripped 태그 포함.

예:
  python tools/build_waypoints_from_pool.py \
      --grip ./grip_poses.json --poses ./capture_poses.json --sets ./capture_sets.json \
      --output ./data/session/capture_waypoints.json \
      --n_grip_per_set 10 --n_per_set 5 --seed 0
"""
import argparse
import json
import os
import random
import sys


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


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--poses", default="./capture_poses.json",
                    help="A placement 뷰포인트 풀 (recpose 출력)")
    ap.add_argument("--grip", default="./grip_poses.json",
                    help="B 그립-스윕 포즈 풀 (recgrip 출력). n_grip_per_set=0 이면 생략")
    ap.add_argument("--sets", default="./capture_sets.json",
                    help="큐브 set 배치 (recset 출력)")
    ap.add_argument("--output", default="./data/session/capture_waypoints.json",
                    help="출력 waypoint JSON (서버 소비 포맷)")
    ap.add_argument("--n_per_set", type=int, default=5,
                    help="set 당 A placement 포즈 수 (기본 5)")
    ap.add_argument("--n_grip_per_set", type=int, default=10,
                    help="set 당 B 그립-스윕 포즈 수 (기본 10). 0 이면 B 생략")
    ap.add_argument("--b_ref_set", type=int, default=0,
                    help="B 스윕 패턴을 기록한 기준 set 의 인덱스(0-based). 그 set 의 "
                         "큐브중심(C_ref)을 기준으로 각 set 큐브중심으로 x,y,z 평행이동")
    ap.add_argument("--seed", type=int, default=0,
                    help="랜덤 시드 (재현성). -1 이면 매번 다르게")
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

    use_grip = args.n_grip_per_set > 0
    grip = []
    c_ref = None
    if use_grip:
        grip = _load(args.grip, "grip_poses")
        # B 는 TCP 로 앵커되므로 capture_tcp 가 필수 (관절은 평행이동 불가).
        for i, p in enumerate(grip):
            _check6(p.get("capture_tcp"), args.grip, "capture_tcp", i)
        if not (0 <= args.b_ref_set < len(sets)):
            sys.exit(f"[ERROR] --b_ref_set {args.b_ref_set} 범위 밖 (0..{len(sets)-1})")
        c_ref = [float(x) for x in sets[args.b_ref_set]["set_cube_center_6dof"]]
        print(f"[INFO] B 기준 set index={args.b_ref_set}  "
              f"C_ref(xyz)={[round(v, 1) for v in c_ref[:3]]}")

    if args.n_per_set < 0 or args.n_grip_per_set < 0:
        sys.exit("[ERROR] --n_per_set / --n_grip_per_set 은 0 이상")
    if args.n_per_set == 0 and not use_grip:
        sys.exit("[ERROR] A/B 둘 다 0 이면 생성할 게 없음")
    if args.n_per_set > len(poses) and not args.allow_repeat:
        sys.exit(f"[ERROR] --n_per_set {args.n_per_set} > A 포즈 수 {len(poses)} "
                 f"(--allow_repeat 또는 포즈 추가)")
    if use_grip and args.n_grip_per_set > len(grip) and not args.allow_repeat:
        sys.exit(f"[ERROR] --n_grip_per_set {args.n_grip_per_set} > B 포즈 수 {len(grip)} "
                 f"(--allow_repeat 또는 포즈 추가)")

    rng = random.Random(None if args.seed < 0 else args.seed)
    print(f"[INFO] A_poses={len(poses)}  B_grip_poses={len(grip) if use_grip else 0}  "
          f"sets={len(sets)}  n_per_set(A)={args.n_per_set}  n_grip_per_set(B)={args.n_grip_per_set}  "
          f"seed={'random' if args.seed < 0 else args.seed}")

    waypoints = []
    capture_index = 0
    for si, s in enumerate(sets):
        set_index = int(s.get("set_index", si))
        place_joints = [float(x) for x in s["place_joints"]]
        set_cc = [float(x) for x in s["set_cube_center_6dof"]]

        b_sel = _pick(rng, grip, args.n_grip_per_set, args.allow_repeat) if use_grip else []
        a_sel = _pick(rng, poses, args.n_per_set, args.allow_repeat)

        print(f"  set_index={set_index}: "
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
    out = {
        "set_joints": [float(x) for x in sets[0]["place_joints"]],
        "set_tcp": ([float(x) for x in sets[0]["place_tcp"]]
                    if isinstance(sets[0].get("place_tcp"), list) else None),
        "set_cube_center": [float(x) for x in sets[0]["set_cube_center_6dof"]],
        "waypoints": waypoints,
        "_meta": {
            "generator": "tools/build_waypoints_from_pool.py",
            "n_A_poses": len(poses),
            "n_B_grip_poses": len(grip) if use_grip else 0,
            "n_sets": len(sets),
            "n_per_set_A": args.n_per_set,
            "n_grip_per_set_B": args.n_grip_per_set,
            "seed": None if args.seed < 0 else args.seed,
            "allow_repeat": args.allow_repeat,
            "total_captures": len(waypoints),
            "total_B_eyetohand": n_b,
            "total_A_placement": n_a,
        },
    }

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
