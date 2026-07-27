#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
CP_validate_holdout.py — train/test 기반 카메라·큐브 정확도 검증 (빠른 폐형 방식)

두 가지 질문에 답한다. 무거운 bundle adjustment 없이 폐형(closed-form)으로만 계산해
CP_C3 대비 훨씬 빠르다.

  (1) 큐브 위치 정확도 (held-out)
      train set 으로만 카메라를 맞춘 뒤, 학습에 안 쓴 test set 의 큐브 위치를 그
      카메라들로 예측해 로봇 FK(set_cube_center) 대비 오차(mm)를 잰다.
      * FK 는 정답이 아니라 프록시이며, 회전(yaw)은 신뢰 불가이므로 병진만 본다.
      * 추가로 FK 를 전혀 안 쓰는 지표도 낸다: test set 에서의 카메라 간 일치도.

  (2) 카메라 위치 정확도 (split-half 안정성)
      set 을 서로소 두 그룹으로 나눠 각각 캘리브한 뒤 나온 카메라 외부파라미터를
      직접 비교한다. 절대 정확도는 외부 기준이 없어 측정 불가하므로, 이 재현성이
      표준 대용치다. 상대 외부파라미터(T_Cref_Ci)는 FK 를 전혀 쓰지 않아 가장 깨끗하다.

<<명령어>>
  python CP_validate_holdout.py --root_folder data/session --intrinsics_dir intrinsics
  # 결과 -> CP_result/validate_holdout
"""
import os
import json
import argparse
from collections import defaultdict
from typing import Dict, List, Optional

import numpy as np

import CP_common as cp


def rot_deg(Ra: np.ndarray, Rb: np.ndarray) -> float:
    return cp.rotation_error_deg(np.asarray(Ra)[:3, :3], np.asarray(Rb)[:3, :3])


def fit_cameras(obs_by_event: Dict[int, Dict[int, np.ndarray]],
                event_set: Dict[int, int],
                fk_cube: Dict[int, np.ndarray],
                train_sets: List[int],
                ref_cam: int) -> Optional[dict]:
    """train set 관측만으로 카메라를 맞춘다.

    a) 상대 외부파라미터 T_Cref_Ci : 두 카메라가 같은 이벤트에서 큐브를 볼 때
       T_Cref_Ci = T_Cref_O @ inv(T_Ci_O). FK 를 전혀 쓰지 않는다.
    b) base 등록 : ref 프레임의 큐브 위치들을 train set 의 FK 큐브 위치에 Kabsch 로
       정합(병진 기준). FK 회전은 신뢰 불가하므로 위치만 사용한다.
    """
    tr = set(int(s) for s in train_sets)
    rel_acc: Dict[int, List[np.ndarray]] = defaultdict(list)
    for eid, percam in obs_by_event.items():
        if event_set.get(eid) not in tr or ref_cam not in percam:
            continue
        T_ref_O = percam[ref_cam]
        for ci, T_ci_O in percam.items():
            if ci == ref_cam:
                continue
            rel_acc[ci].append(T_ref_O @ cp.inv_T(T_ci_O))
    T_ref_Ci = {ref_cam: np.eye(4)}
    for ci, Ts in rel_acc.items():
        if len(Ts) >= 2:
            T_ref_Ci[ci] = cp.robust_se3_average(Ts)[0]
    if len(T_ref_Ci) < 2:
        return None

    # per-event cube pose in ref frame (train only), then Kabsch to base via FK positions
    src, dst = [], []
    for eid, percam in obs_by_event.items():
        s = event_set.get(eid)
        if s not in tr or s not in fk_cube:
            continue
        Ts = [T_ref_Ci[ci] @ T_ci_O for ci, T_ci_O in percam.items() if ci in T_ref_Ci]
        if not Ts:
            continue
        src.append(cp.weighted_se3_average(Ts)[:3, 3])
        dst.append(np.asarray(fk_cube[s], float)[:3, 3])
    if len(src) < 3:
        return None
    T_base_ref, _ = cp.robust_kabsch_rigid(np.array(src), np.array(dst), max_resid_mm=60.0)
    return {"T_ref_Ci": T_ref_Ci,
            "T_base_Ci": {ci: T_base_ref @ T for ci, T in T_ref_Ci.items()},
            "T_base_ref": T_base_ref, "n_anchor": len(src)}


def evaluate(fit: dict, obs_by_event, event_set, fk_cube, test_sets) -> dict:
    """test set 에서 (a) FK 대비 큐브 위치오차, (b) FK 무관 카메라 간 일치도."""
    te = set(int(s) for s in test_sets)
    fk_err, xcam = [], []
    n_ev = 0
    for eid, percam in obs_by_event.items():
        s = event_set.get(eid)
        if s not in te:
            continue
        pts = [(fit["T_base_Ci"][ci] @ T_ci_O)[:3, 3]
               for ci, T_ci_O in percam.items() if ci in fit["T_base_Ci"]]
        if not pts:
            continue
        n_ev += 1
        mean_p = np.mean(pts, axis=0)
        if len(pts) >= 2:                       # FK-free cross-camera agreement
            xcam += [float(np.linalg.norm(p - mean_p) * 1000.0) for p in pts]
        if s in fk_cube:                        # FK-referenced position error
            fk_err.append(float(np.linalg.norm(mean_p - np.asarray(fk_cube[s], float)[:3, 3]) * 1000.0))
    f = lambda a: None if not a else float(np.sqrt(np.mean(np.square(a))))
    return {"n_test_events": n_ev,
            "cube_fk_rmse_mm": f(fk_err),
            "cube_fk_median_mm": None if not fk_err else float(np.median(fk_err)),
            "xcam_rmse_mm": f(xcam),
            "xcam_median_mm": None if not xcam else float(np.median(xcam))}


def main() -> None:
    ap = argparse.ArgumentParser(description="train/test validation of camera & cube accuracy")
    ap.add_argument("--root_folder", required=True)
    ap.add_argument("--intrinsics_dir", required=True)
    ap.add_argument("--gripper_cam_idx", type=int, default=None)
    ap.add_argument("--ref_fixed_cam_idx", type=int, default=None)
    ap.add_argument("--holdout_frac", type=float, default=0.3)
    ap.add_argument("--split_seed", type=int, default=0)
    ap.add_argument("--out_dir", default=None)
    args = ap.parse_args()

    root = args.root_folder
    out_dir = cp.ensure_dir(args.out_dir or os.path.join("CP_result", "validate_holdout"))
    with open(os.path.join(root, "meta.json"), "r") as f:
        meta = json.load(f)

    cfg, _ = cp.resolve_cube_config_for_run(root, calib_dir=out_dir, cube_config_json=None,
                                            default_cfg=cp.get_default_cube_config())
    cube = cp.AprilTagCubeTarget(cfg)
    all_cams = sorted({int(k) for c in meta.get("captures", [])
                       for k, v in c.get("cams", {}).items() if v.get("saved")})
    grip = args.gripper_cam_idx if args.gripper_cam_idx is not None else meta.get("gripper_cam_idx")
    fixed = [c for c in all_cams if c != int(grip)]
    ref_cam = args.ref_fixed_cam_idx if args.ref_fixed_cam_idx is not None else fixed[0]

    K_map, D_map = {}, {}
    for ci in all_cams:
        K_map[ci], D_map[ci], _ = cp.load_intrinsics_with_depth_scale(args.intrinsics_dir, ci)

    event_set: Dict[int, int] = {}
    gripped: Dict[int, bool] = {}
    for capm in meta.get("captures", []):
        eid = int(capm.get("event_id", -1))
        if eid < 0:
            continue
        s = cp.get_capture_set_index(capm)
        if s is not None:
            event_set[eid] = int(s)
        gripped[eid] = bool(capm.get("cube_gripped", False)) or \
            str(capm.get("capture_block", "")) == "B_eyetohand"

    fk_cube = cp.load_nominal_set_cube_transforms(meta)
    print(f"[INFO] cams={all_cams} fixed={fixed} gripper=cam{grip} ref=cam{ref_cam}; FK sets={len(fk_cube)}")
    print("[INFO] detecting cube observations (this is the slow part)...")
    pose_obs = cp.load_pose_observations(
        root=root, meta=meta, cube=cube, K_map=K_map, D_map=D_map, all_cam_ids=all_cams,
        gripper_cam_idx=int(grip), reuse_stored_cube_candidates=False,
        max_err_fixed=3.0, max_err_gripper=5.0, min_aspect_fixed=0.0,
        min_aspect_gripper=0.35, gripper_min_markers=1)

    # cube must be at its set placement -> non-gripped frames, fixed cameras only
    obs_by_event: Dict[int, Dict[int, np.ndarray]] = defaultdict(dict)
    for o in pose_obs:
        eid = int(o.event)
        if int(o.cam) not in fixed or gripped.get(eid, True):
            continue
        obs_by_event[eid][int(o.cam)] = np.asarray(o.T_C_O, float)
    sets_present = sorted({event_set[e] for e in obs_by_event if e in event_set})
    print(f"[INFO] usable events={len(obs_by_event)} over sets={sets_present}")

    import random
    shuffled = list(sets_present)
    random.Random(int(args.split_seed)).shuffle(shuffled)
    n_test = max(1, min(int(round(len(sets_present) * float(args.holdout_frac))), len(sets_present) - 3))
    test_sets = sorted(shuffled[len(shuffled) - n_test:])
    train_sets = [s for s in sets_present if s not in set(test_sets)]

    report: Dict[str, object] = {"sets": sets_present, "n_events": len(obs_by_event)}

    # ── (1) held-out cube position accuracy ──
    print(f"\n=== (1) HELD-OUT CUBE POSITION ACCURACY  train={train_sets} test={test_sets} ===")
    fit = fit_cameras(obs_by_event, event_set, fk_cube, train_sets, ref_cam)
    if fit is None:
        print("  [ERROR] could not fit cameras on train sets")
    else:
        ev = evaluate(fit, obs_by_event, event_set, fk_cube, test_sets)
        tr_ev = evaluate(fit, obs_by_event, event_set, fk_cube, train_sets)
        print(f"  anchor sets used: {fit['n_anchor']} events")
        print(f"  TEST  cube vs FK : rmse={ev['cube_fk_rmse_mm']:.2f}mm  median={ev['cube_fk_median_mm']:.2f}mm  ({ev['n_test_events']} events)")
        print(f"  TRAIN cube vs FK : rmse={tr_ev['cube_fk_rmse_mm']:.2f}mm  (in-sample reference)")
        if ev["xcam_rmse_mm"] is not None:
            print(f"  TEST  cross-camera agreement (FK-free): rmse={ev['xcam_rmse_mm']:.2f}mm  median={ev['xcam_median_mm']:.2f}mm")
        report["holdout"] = {"train_sets": train_sets, "test_sets": test_sets,
                             "test": ev, "train": tr_ev}

    # ── (2) split-half camera stability ──
    half = len(sets_present) // 2
    A, B = sets_present[:half], sets_present[half:]
    print(f"\n=== (2) SPLIT-HALF CAMERA STABILITY  A={A}  B={B} ===")
    fa, fb = (fit_cameras(obs_by_event, event_set, fk_cube, X, ref_cam) for X in (A, B))
    if fa is None or fb is None:
        print("  [ERROR] could not fit one of the halves")
    else:
        rows = []
        print(f"  {'camera':<10}{'relative (FK-free)':>26}{'base-registered':>24}")
        print(f"  {'':<10}{'dt_mm':>12}{'dr_deg':>14}{'dt_mm':>12}{'dr_deg':>12}")
        for ci in sorted(set(fa["T_ref_Ci"]) & set(fb["T_ref_Ci"])):
            rdt = float(np.linalg.norm(fa["T_ref_Ci"][ci][:3, 3] - fb["T_ref_Ci"][ci][:3, 3]) * 1000.0)
            rdr = rot_deg(fa["T_ref_Ci"][ci], fb["T_ref_Ci"][ci])
            bdt = float(np.linalg.norm(fa["T_base_Ci"][ci][:3, 3] - fb["T_base_Ci"][ci][:3, 3]) * 1000.0)
            bdr = rot_deg(fa["T_base_Ci"][ci], fb["T_base_Ci"][ci])
            print(f"  cam{ci:<7}{rdt:>12.2f}{rdr:>14.3f}{bdt:>12.2f}{bdr:>12.3f}")
            rows.append({"cam": ci, "rel_dt_mm": rdt, "rel_dr_deg": rdr,
                         "base_dt_mm": bdt, "base_dr_deg": bdr})
        print("  (relative = FK 미사용, 순수 시각 기하. base = FK 병진 앵커 포함)")
        report["split_half"] = {"A": A, "B": B, "cameras": rows}

    with open(os.path.join(out_dir, "validate_holdout.json"), "w") as f:
        json.dump(report, f, indent=2, ensure_ascii=False)
    print(f"\n[DONE] {os.path.join(out_dir, 'validate_holdout.json')}")


if __name__ == "__main__":
    main()
