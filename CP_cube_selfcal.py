#!/usr/bin/env python3
"""큐브 마커 배치 self-calibration — 다중 카메라 동시관측으로 T_object_marker 를 보정.

언제 쓰나
---------
큐브를 다시 인쇄하거나 스티커를 다시 붙였을 때, `config.py` 의 공칭 모델(면 중심 +
face_roll 90도 단위)이 실제 큐브와 맞는지 검증·보정하는 용도다.

**data/session 에서는 보정이 불필요한 것으로 확인됐다** — 커밋 d755ff7 의 face_roll 수정
이후, 마커별 추정 보정량은 0.4~1.5mm 에 그치고 held-out 카메라간 일치도는 2.45mm -> 2.46mm
로 변화가 없다. 즉 현재 큐브 모델은 이미 충분하며 `marker_pose_4x4` 를 채울 이유가 없다.
(이전에 면 조합별로 (+X,+Y) 14mm 같은 계통편차가 보였던 것은 큐브 모델이 아니라 C1 이
FK 큐브 prior 의 뒤집힌 회전을 카메라 등록 앵커로 쓴 탓이었다. CP_C1 의
`correct_fk_cube_rotation` 참고.)

무엇을 하는가
-------------
여러 고정 카메라가 같은 촬영에서 큐브의 서로 다른 면을 동시에 보는 것을 이용해, 재투영오차
최소화 번들조정으로 다음을 동시에 푼다:

    {카메라 상대 pose} x {촬영별 큐브 pose} x {마커별 6-DoF 보정 C_m}

게이지: 기준 카메라 = 항등, 기준 마커(가장 관측이 많은 마커) 보정 = 항등. 즉 큐브 오브젝트
프레임은 기준 마커의 공칭 pose 에 그대로 묶여 있어, 이 보정을 적용해도 큐브 좌표계 정의는
바뀌지 않는다 (`marker_center_m` 규약 유지).

held-out
--------
`--holdout_frac` / `--test_sets` 로 나누면 **train set 관측만으로 fit** 하고 test set 에서
카메라간 큐브중점 일치도를 보정 전/후로 비교해 출력한다. C1/C3 와 같은 split 을 쓰면 그쪽
held-out 지표가 오염되지 않는다.

산출물
------
`--out_json` 에 마커별 4x4 T_object_marker, 그리고 `config.py` 의 `CubeConfig.marker_pose_4x4`
에 그대로 붙여넣을 수 있는 파이썬 리터럴 블록을 함께 출력한다.

실행
----
    PYTHONPATH= python CP_cube_selfcal.py \
        --root_folder data/session --intrinsics_dir intrinsics \
        --holdout_frac 0.3 --split_seed 0 \
        --out_json CP_result/cube_selfcal/marker_poses.json
"""
from __future__ import annotations

import argparse
import collections
import json
import os
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np
from scipy.optimize import least_squares
from scipy.sparse import lil_matrix
from scipy.spatial.transform import Rotation

import CP_common as cp


# ── SE(3) helpers ─────────────────────────────────────────────────────────────
def vec_to_T(v: np.ndarray) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = Rotation.from_rotvec(np.asarray(v[:3], float)).as_matrix()
    T[:3, 3] = np.asarray(v[3:], float)
    return T


def T_to_vec(T: np.ndarray) -> np.ndarray:
    T = np.asarray(T, float)
    return np.concatenate([Rotation.from_matrix(T[:3, :3]).as_rotvec(), T[:3, 3]])


# ── 검출: 촬영 x 카메라 x 마커 단위 image corners ─────────────────────────────
def detect_marker_corners(root: str, meta: dict, cube, fixed_cam_ids: List[int],
                          exclude_gripped: bool) -> List[Tuple[int, int, int, np.ndarray]]:
    """(event_id, cam, marker_id, 4x2 image corners) 목록."""
    out = []
    for cap in meta.get("captures", []):
        eid = int(cap.get("event_id", -1))
        if eid < 0:
            continue
        if exclude_gripped and cap.get("cube_gripped"):
            continue
        for ci_str, cinfo in cap.get("cams", {}).items():
            ci = int(ci_str)
            if ci not in fixed_cam_ids or not cinfo.get("saved"):
                continue
            rgb_rel = cinfo.get("rgb_path", "")
            if not rgb_rel:
                continue
            img = cv2.imread(os.path.join(root, rgb_rel))
            if img is None:
                continue
            try:
                corners_list, ids = cube.detect(img)
            except Exception:
                continue
            if ids is None:
                continue
            for corners, mid_raw in zip(corners_list, ids):
                mid = int(np.asarray(mid_raw).reshape(-1)[0])
                if not cube.model.has_marker(mid):
                    continue
                ip = np.asarray(corners, float).reshape(4, 2)
                ip = np.asarray(cube.model.reorder_image_corners(mid, ip), float).reshape(4, 2)
                out.append((eid, ci, mid, ip))
    return out


# ── 평가: 카메라간 큐브중점 일치도 ────────────────────────────────────────────
def solve_cube_pose(pairs, ci, T_O_M, local, K_map, D_map,
                    min_markers: int = 2) -> Optional[np.ndarray]:
    """한 카메라의 마커 관측들로 T_C_O 를 PnP (주어진 마커 배치 모델 T_O_M 기준)."""
    if len({m for m, _ in pairs}) < min_markers:
        return None
    obj, img = [], []
    for m, ip in pairs:
        obj.append((T_O_M[m] @ np.hstack([local[m], np.ones((4, 1))]).T).T[:, :3])
        img.append(ip)
    obj = np.concatenate(obj).astype(np.float64)
    img = np.concatenate(img).astype(np.float64)
    ok, rv, tv = cv2.solvePnP(obj, img, K_map[ci], D_map[ci], flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok:
        return None
    ok, rv, tv = cv2.solvePnP(obj, img, K_map[ci], D_map[ci], rvec=rv, tvec=tv,
                              useExtrinsicGuess=True, flags=cv2.SOLVEPNP_ITERATIVE)
    T = np.eye(4)
    T[:3, :3] = cv2.Rodrigues(rv)[0]
    T[:3, 3] = tv.reshape(3)
    return T


def cross_camera_spread(events, by_ec, fixed_cam_ids, T_O_M, cams, local, K_map, D_map) -> np.ndarray:
    """같은 촬영을 본 카메라들이 예측한 큐브중점의 (중앙값 대비) 편차, mm."""
    devs = []
    for e in events:
        P = []
        for ci in fixed_cam_ids:
            pairs = by_ec.get((e, ci))
            if not pairs or ci not in cams:
                continue
            T = solve_cube_pose(pairs, ci, T_O_M, local, K_map, D_map)
            if T is not None:
                P.append((cams[ci] @ T)[:3, 3])
        if len(P) >= 2:
            P = np.asarray(P)
            devs.extend((np.linalg.norm(P - np.median(P, axis=0), axis=1) * 1000.0).tolist())
    return np.asarray(devs)


def fmt_stats(tag: str, v: np.ndarray) -> str:
    if not len(v):
        return f"  {tag:26s} (관측 없음)"
    return (f"  {tag:26s} n={len(v):4d}  median {np.median(v):6.2f}  RMS {np.sqrt((v ** 2).mean()):7.2f}  "
            f"p90 {np.percentile(v, 90):7.2f} mm  (<=5mm {np.mean(v <= 5) * 100:5.1f}%)")


# ── main ──────────────────────────────────────────────────────────────────────
def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--root_folder", required=True)
    ap.add_argument("--intrinsics_dir", required=True)
    ap.add_argument("--out_json", default=os.path.join("CP_result", "cube_selfcal", "marker_poses.json"))
    ap.add_argument("--gripper_cam_idx", type=int, default=None)
    ap.add_argument("--cube_config_json", type=str, default=None)
    ap.add_argument("--test_sets", type=str, default="",
                    help="held-out set 목록 (예: \"0,4,6,12\"). 비우면 --holdout_frac 사용.")
    ap.add_argument("--holdout_frac", type=float, default=0.0,
                    help="held-out 비율. 0 이면 전체 set 으로 fit (평가 지표 NA).")
    ap.add_argument("--split_seed", type=int, default=0)
    ap.add_argument("--exclude_gripped", type=lambda s: str(s).lower() not in ("0", "false", "no"),
                    default=True, help="큐브를 쥐고 찍은 촬영 제외 (기본 True)")
    ap.add_argument("--max_nfev", type=int, default=400)
    ap.add_argument("--huber_f_scale", type=float, default=2.0,
                    help="재투영 잔차의 Huber 스케일(px). 검출 이상치를 눌러준다.")
    args = ap.parse_args()

    root = args.root_folder
    with open(os.path.join(root, "meta.json"), "r") as f:
        meta = json.load(f)

    cfg, cfg_source = cp.resolve_cube_config_for_run(
        root_folder=root, calib_dir=os.path.dirname(args.out_json) or ".",
        cube_config_json=args.cube_config_json,
        default_cfg=cp.get_default_cube_config())
    cube = cp.AprilTagCubeTarget(cfg)
    print(f"[INFO] cube config source: {cfg_source}")

    all_cam_ids = sorted({int(k) for cap in meta.get("captures", [])
                          for k, v in cap.get("cams", {}).items() if v.get("saved")})
    gripper_cam_idx = args.gripper_cam_idx
    if gripper_cam_idx is None:
        gripper_cam_idx = meta.get("gripper_cam_idx")
    if gripper_cam_idx is None:
        dm = os.path.join(args.intrinsics_dir, "device_map.json")
        if os.path.exists(dm):
            with open(dm, "r") as f:
                gripper_cam_idx = json.load(f).get("gripper_cam_idx")
    if gripper_cam_idx is None:
        raise RuntimeError("gripper_cam_idx required")
    fixed_cam_ids = [c for c in all_cam_ids if c != int(gripper_cam_idx)]
    if len(fixed_cam_ids) < 2:
        raise RuntimeError("마커 배치 self-cal 은 고정 카메라 2대 이상이 필요하다 "
                           "(카메라 간 동시관측이 관측 방정식의 핵심)")

    K_map, D_map = {}, {}
    for ci in all_cam_ids:
        K_map[ci], D_map[ci], _ = cp.load_intrinsics_with_depth_scale(args.intrinsics_dir, ci)

    event_to_set = {}
    for cap in meta.get("captures", []):
        eid = int(cap.get("event_id", -1))
        if eid >= 0:
            sidx = cp.get_capture_set_index(cap)
            event_to_set[eid] = int(sidx) if sidx is not None else None

    print(f"[INFO] fixed={fixed_cam_ids}, gripper=cam{gripper_cam_idx}")
    det = detect_marker_corners(root, meta, cube, fixed_cam_ids, bool(args.exclude_gripped))
    if not det:
        raise RuntimeError("마커 검출 0개 — root_folder/intrinsics 를 확인하라")

    by_ec: Dict[Tuple[int, int], List[Tuple[int, np.ndarray]]] = collections.defaultdict(list)
    for e, ci, mid, ip in det:
        by_ec[(e, ci)].append((mid, ip))
    events = sorted({e for (e, _) in by_ec})
    mids = sorted({d[2] for d in det})
    local = {m: np.asarray(cube.model.local_corners_for(m), float).reshape(4, 3) for m in mids}
    T_O_M_nom = {m: np.asarray(cube.model.marker_pose_in_rig(m), float) for m in mids}
    print(f"[INFO] 마커관측 {len(det)}개, 촬영 {len(events)}개, 마커 {mids}")

    # train/test split (set 단위)
    all_sets = sorted({s for s in event_to_set.values() if s is not None})
    test_sets = set(cp_resolve_test_sets(all_sets, args.test_sets, args.holdout_frac, args.split_seed))
    train_ev = [e for e in events if event_to_set.get(e) not in test_sets]
    test_ev = [e for e in events if event_to_set.get(e) in test_sets]
    if test_sets:
        print(f"[INFO] split: train={len(train_ev)} events / test={len(test_ev)} events "
              f"(test_sets={sorted(test_sets)})")
    else:
        print("[INFO] split 없음 — 전체로 fit (보정 전/후 held-out 비교 불가)")
        train_ev = events

    # 카메라 상대 pose 초기값: 기준 카메라 = 항등 (base 프레임 불필요, 지표는 게이지 불변)
    ref_cam = fixed_cam_ids[0]
    pose_by_cam_event: Dict[int, Dict[int, np.ndarray]] = collections.defaultdict(dict)
    for (e, ci), pairs in by_ec.items():
        T = solve_cube_pose(pairs, ci, T_O_M_nom, local, K_map, D_map)
        if T is not None:
            pose_by_cam_event[ci][e] = T
    cams_nom = {ref_cam: np.eye(4)}
    for ci in fixed_cam_ids:
        if ci == ref_cam:
            continue
        Ts = [pose_by_cam_event[ref_cam][e] @ cp.inv_T(pose_by_cam_event[ci][e])
              for e in pose_by_cam_event[ref_cam]
              if e in pose_by_cam_event[ci] and event_to_set.get(e) not in test_sets]
        if not Ts:
            raise RuntimeError(f"cam{ci} 와 기준 cam{ref_cam} 의 동시관측이 없다")
        cams_nom[ci] = cp.robust_se3_average(Ts, None)[0]

    print("\n[보정 전] 카메라간 큐브중점 일치도")
    before_tr = cross_camera_spread(train_ev, by_ec, fixed_cam_ids, T_O_M_nom, cams_nom, local, K_map, D_map)
    print(fmt_stats("train", before_tr))
    before_te = cross_camera_spread(test_ev, by_ec, fixed_cam_ids, T_O_M_nom, cams_nom, local, K_map, D_map)
    if len(before_te):
        print(fmt_stats("TEST (held-out)", before_te))

    # ── 번들조정 ──────────────────────────────────────────────────────────────
    # 게이지: 관측이 가장 많은 마커의 보정을 항등으로 고정 -> 큐브 좌표계 정의 유지
    n_by_mid = collections.Counter(d[2] for d in det)
    gauge_mid = max(mids, key=lambda m: (n_by_mid[m], -m))
    opt_mids = [m for m in mids if m != gauge_mid]
    opt_cams = [c for c in fixed_cam_ids if c != ref_cam]
    ev_idx = {e: i for i, e in enumerate(train_ev)}
    NE, NM, NC = len(train_ev), len(opt_mids), len(opt_cams)
    print(f"\n[번들조정] 게이지 마커=id{gauge_mid} (관측 {n_by_mid[gauge_mid]}개), 기준 카메라=cam{ref_cam}")

    x0 = np.zeros(6 * (NE + NM + NC))
    for e in train_ev:
        Ts = [cams_nom[ci] @ pose_by_cam_event[ci][e]
              for ci in fixed_cam_ids if e in pose_by_cam_event[ci]]
        if Ts:
            x0[6 * ev_idx[e]:6 * ev_idx[e] + 6] = T_to_vec(cp.weighted_se3_average(Ts, None))
    for j, ci in enumerate(opt_cams):
        x0[6 * (NE + NM + j):6 * (NE + NM + j) + 6] = T_to_vec(cams_nom[ci])

    obs_list = [(ev_idx[e], ci, m, ip)
                for e in train_ev for ci in fixed_cam_ids for m, ip in by_ec.get((e, ci), [])]
    print(f"[번들조정] 마커관측 {len(obs_list)}개, 파라미터 {len(x0)}개")

    def unpack(x):
        Xe = [vec_to_T(x[6 * i:6 * i + 6]) for i in range(NE)]
        T_O_M = dict(T_O_M_nom)
        for j, m in enumerate(opt_mids):
            T_O_M[m] = T_O_M_nom[m] @ vec_to_T(x[6 * (NE + j):6 * (NE + j) + 6])
        cams = {ref_cam: np.eye(4)}
        for j, ci in enumerate(opt_cams):
            cams[ci] = vec_to_T(x[6 * (NE + NM + j):6 * (NE + NM + j) + 6])
        return Xe, T_O_M, cams

    def residual(x):
        Xe, T_O_M, cams = unpack(x)
        out = np.empty(len(obs_list) * 8)
        k = 0
        for (i, ci, m, ip) in obs_list:
            T_C_M = cp.inv_T(cams[ci]) @ Xe[i] @ T_O_M[m]
            rv = cv2.Rodrigues(T_C_M[:3, :3])[0]
            proj, _ = cv2.projectPoints(local[m], rv, T_C_M[:3, 3].reshape(3, 1), K_map[ci], D_map[ci])
            out[k:k + 8] = (proj.reshape(4, 2) - ip).ravel()
            k += 8
        return out

    sparsity = lil_matrix((len(obs_list) * 8, len(x0)), dtype=int)
    for r, (i, ci, m, _) in enumerate(obs_list):
        sparsity[8 * r:8 * r + 8, 6 * i:6 * i + 6] = 1
        if m in opt_mids:
            j = opt_mids.index(m)
            sparsity[8 * r:8 * r + 8, 6 * (NE + j):6 * (NE + j) + 6] = 1
        if ci in opt_cams:
            j = opt_cams.index(ci)
            sparsity[8 * r:8 * r + 8, 6 * (NE + NM + j):6 * (NE + NM + j) + 6] = 1

    def reproj_stats(r):
        e = np.linalg.norm(r.reshape(-1, 2), axis=1)
        return (f"RMS {np.sqrt((e ** 2).mean()):6.3f} px, median {np.median(e):5.3f} px, "
                f">5px {np.mean(e > 5) * 100:4.1f}%")

    print(f"[번들조정] 초기 재투영 {reproj_stats(residual(x0))}")
    sol = least_squares(residual, x0, jac_sparsity=sparsity, method="trf",
                        loss="huber", f_scale=float(args.huber_f_scale),
                        xtol=1e-10, ftol=1e-10, max_nfev=int(args.max_nfev), verbose=0)
    print(f"[번들조정] 최종 재투영 {reproj_stats(residual(sol.x))}  (nfev={sol.nfev})")

    _, T_O_M_fit, cams_fit = unpack(sol.x)

    print("\n[마커별 추정 보정량] (큐브 오브젝트 좌표계)")
    deltas = {}
    for m in mids:
        d = cp.inv_T(T_O_M_nom[m]) @ T_O_M_fit[m]
        rot_deg = float(np.degrees(np.linalg.norm(Rotation.from_matrix(d[:3, :3]).as_rotvec())))
        deltas[m] = (float(np.linalg.norm(d[:3, 3]) * 1000.0), rot_deg)
        mark = "  <- 게이지(고정)" if m == gauge_mid else ""
        print(f"  id{m} ({cube.model.marker_face_name(m):2s}): 이동 {deltas[m][0]:6.2f} mm "
              f"({d[0, 3] * 1000:+6.2f},{d[1, 3] * 1000:+6.2f},{d[2, 3] * 1000:+6.2f})  "
              f"회전 {rot_deg:5.2f} deg{mark}")

    print("\n[보정 후] 카메라간 큐브중점 일치도")
    print(fmt_stats("train", cross_camera_spread(train_ev, by_ec, fixed_cam_ids,
                                                 T_O_M_fit, cams_fit, local, K_map, D_map)))
    after_te = cross_camera_spread(test_ev, by_ec, fixed_cam_ids, T_O_M_fit, cams_fit, local, K_map, D_map)
    if len(after_te):
        print(fmt_stats("TEST (held-out)", after_te))
        print(f"\n  => held-out median {np.median(before_te):.2f} -> {np.median(after_te):.2f} mm, "
              f"5mm 이내 {np.mean(before_te <= 5) * 100:.1f}% -> {np.mean(after_te <= 5) * 100:.1f}%")

    out_dir = os.path.dirname(args.out_json)
    if out_dir:
        cp.ensure_dir(out_dir)
    payload = {
        "note": ("Self-calibrated T_object_marker (4x4) from multi-camera co-observation. "
                 "Paste into config.py CubeConfig.marker_pose_4x4 to override the nominal "
                 "face_roll/marker_center model."),
        "cube_config_source": cfg_source,
        "root_folder": root,
        "gauge_marker_id": int(gauge_mid),
        "ref_cam": int(ref_cam),
        "train_events": len(train_ev),
        "test_sets": sorted(test_sets),
        "reproj_median_px_final": float(np.median(np.linalg.norm(residual(sol.x).reshape(-1, 2), axis=1))),
        "marker_pose_4x4": {str(m): T_O_M_fit[m].tolist() for m in mids},
        "delta_from_nominal": {str(m): {"trans_mm": deltas[m][0], "rot_deg": deltas[m][1]} for m in mids},
    }
    with open(args.out_json, "w") as f:
        json.dump(payload, f, indent=2)
    print(f"\n[DONE] {args.out_json}")

    print("\n----- config.py CubeConfig.marker_pose_4x4 에 붙여넣을 블록 -----")
    print("    marker_pose_4x4: Dict[int, list] = field(default_factory=lambda: {")
    for m in mids:
        rows = ",\n".join("            [" + ", ".join(f"{v: .9f}" for v in row) + "]"
                          for row in T_O_M_fit[m])
        print(f"        {m}: [\n{rows},\n        ],")
    print("    })")


def cp_resolve_test_sets(available_sets, test_sets_arg: str, holdout_frac: float, split_seed: int):
    """C1/C3 의 resolve_test_sets 와 동일 규칙 (같은 split 을 재현하기 위함)."""
    avail = sorted(int(s) for s in available_sets)
    if str(test_sets_arg).strip():
        want = {int(t) for t in str(test_sets_arg).replace(";", ",").split(",") if t.strip()}
        return sorted(want & set(avail))
    if float(holdout_frac) > 0.0 and len(avail) >= 2:
        import random as _random
        n_test = max(1, int(round(len(avail) * float(holdout_frac))))
        n_test = min(n_test, len(avail) - 1)
        shuffled = list(avail)
        _random.Random(int(split_seed)).shuffle(shuffled)
        return sorted(shuffled[len(shuffled) - n_test:])
    return []


if __name__ == "__main__":
    main()
