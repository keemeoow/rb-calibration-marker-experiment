#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
CP_C1_unified_vs_independent.py  —  기여도 C1 독립 실험 (실데이터)

C1: Eye-in-Hand & Eye-to-Hand 를 하나로 통합(joint) 실행  vs  독립(따로) 실행.

한 번 촬영한 세션(--root_folder)의 관측을 읽어 두 방식을 같은 데이터로 풀어 비교한다.
결과는 기본적으로 CP_result/C1 에 저장된다. C2/C3 와 독립적으로 단독 실행된다.
공유 로더/기하 유틸은 CP_common 에서 가져온다.

두 서브시스템을 하나의 큐브로 연결한다:
  - eye-to-hand (고정 카메라 ci): T_base_Ci @ T_Ci_O[event]        == cube[set]
  - eye-in-hand (그리퍼 카메라):  T_base_gripper[e] @ gTc @ T_gcam_O[e] == cube[set(e)]

비교되는 방법(모두 동일 관측·동일 FK 정보를 사용, "따로 vs 동시"만 차이):
  independent   : 고정 카메라(각자 FK 큐브로 closed-form)와 그리퍼(gTc 단독 least-sq)를
                  *따로* 풀어 base 에서 조합. 서브시스템 간 정보교환 없음.
  unified_joint : 모든 관측을 하나의 비선형 최소제곱으로 {T_base_Ci, gTc, cube[set]} 동시
                  최적화. cube 는 자유변수, gauge 는 FK soft anchor 로 고정.
  joint_fk_fixed: cube 를 FK 값으로 *고정*하고 {T_base_Ci, gTc} 만 동시 최적화
                  (C3 의 "큐브중점 known" 과 C1 의 "동시" 를 겹쳐 본 참고용).

시뮬레이션 짝: Simul_test/joint_calib.py (calib_independent_aligned / calib_joint /
calib_joint_fk_fixed). 이 스크립트는 그 구조를 실데이터 관측으로 포팅한 것이다.

주의: base gauge 를 FK 로 잡으므로 유효한 결과에는 set >= 2~3 개가 필요하다(파일럿 1-set
데이터에서는 실행은 되지만 수치는 의미가 약하다).
"""
import os
import csv
import json
import argparse
from dataclasses import dataclass, asdict
from typing import Dict, List, Optional, Tuple

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

import CP_common as cp
import Step3_calibration as s3


# ── SE(3) <-> 6-vec (rotvec[3] + tvec[3]) ─────────────────────────────────────
def se3_to_vec(T: np.ndarray) -> np.ndarray:
    rv = Rotation.from_matrix(np.asarray(T)[:3, :3]).as_rotvec()
    return np.concatenate([rv, np.asarray(T)[:3, 3]])


def vec_to_se3(v: np.ndarray) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = Rotation.from_rotvec(v[:3]).as_matrix()
    T[:3, 3] = v[3:6]
    return T


def se3_residual(A: np.ndarray, B: np.ndarray) -> np.ndarray:
    """두 SE(3) 불일치 6-vec (회전 rotvec 3[rad] + 병진 3[m]). A==B 이면 0."""
    E = cp.inv_T(A) @ B
    rv = Rotation.from_matrix(E[:3, :3]).as_rotvec()
    return np.concatenate([rv, E[:3, 3]])


def rot_deg(Ra: np.ndarray, Rb: np.ndarray) -> float:
    E = np.asarray(Ra) @ np.asarray(Rb).T
    return float(np.degrees(np.linalg.norm(Rotation.from_matrix(E).as_rotvec())))


# ── Scene assembled from real capture data ────────────────────────────────────
@dataclass
class Scene:
    fixed_cam_ids: List[int]
    gripper_cam_idx: int
    obs_fixed: List[Tuple[int, int, int, np.ndarray]]   # (cam, event, set, T_cam_obj)
    obs_grip: List[Tuple[int, int, np.ndarray]]          # (event, set, T_gcam_obj)
    bTg: Dict[int, np.ndarray]                           # event -> base->gripper (FK)
    fk_cube: Dict[int, np.ndarray]                       # set -> base->cube (FK prior)
    sets: List[int]


def build_scene(pose_obs, robot_T, set_priors, fixed_cam_ids, gripper_cam_idx,
                event_to_set) -> Scene:
    obs_fixed, obs_grip = [], []
    sets = set()
    for o in pose_obs:
        sidx = o.set_idx if o.set_idx is not None else event_to_set.get(int(o.event))
        if sidx is None or int(sidx) not in set_priors:
            continue          # need an FK cube for the set to place it in base
        if int(o.event) not in robot_T:
            continue
        sidx = int(sidx)
        sets.add(sidx)
        if int(o.cam) == int(gripper_cam_idx):
            obs_grip.append((int(o.event), sidx, np.asarray(o.T_C_O, float)))
        elif int(o.cam) in fixed_cam_ids:
            obs_fixed.append((int(o.cam), int(o.event), sidx, np.asarray(o.T_C_O, float)))
    return Scene(
        fixed_cam_ids=sorted(fixed_cam_ids),
        gripper_cam_idx=int(gripper_cam_idx),
        obs_fixed=obs_fixed, obs_grip=obs_grip,
        bTg={int(e): np.asarray(T, float) for e, T in robot_T.items()},
        fk_cube={int(s): np.asarray(T, float) for s, T in set_priors.items()},
        sets=sorted(sets),
    )


# ── Independent (separate) solve ──────────────────────────────────────────────
def solve_independent(sc: Scene, max_nfev: int = 100):
    """고정 카메라: 각 cam 을 FK 큐브 기준 closed-form 평균으로 base 에 등록.
    그리퍼: gTc 를 (FK 큐브 기준) 단독 least-squares 로 추정. 서로 독립."""
    cams: Dict[int, np.ndarray] = {}
    for ci in sc.fixed_cam_ids:
        Ts = [sc.fk_cube[s] @ cp.inv_T(T_co)
              for (c, e, s, T_co) in sc.obs_fixed if c == ci]
        if Ts:
            cams[ci] = _se3_average(Ts)

    gTc = _solve_gripper_only(sc, max_nfev=max_nfev)
    return {"cams": cams, "gTc": gTc, "cube": dict(sc.fk_cube), "mode": "independent"}


def _solve_gripper_only(sc: Scene, gTc0: Optional[np.ndarray] = None, max_nfev: int = 100):
    if not sc.obs_grip:
        return None
    if gTc0 is None:
        gTc0 = np.eye(4)
    p0 = se3_to_vec(gTc0)

    def resid(p):
        gTc = vec_to_se3(p)
        r = []
        for (e, s, T_go) in sc.obs_grip:
            pred = sc.bTg[e] @ gTc @ T_go
            r.append(se3_residual(pred, sc.fk_cube[s]))
        return np.concatenate(r) if r else np.zeros(1)

    sol = least_squares(resid, p0, method="lm", max_nfev=max_nfev)
    return vec_to_se3(sol.x)


# ── Unified joint bundle adjustment ───────────────────────────────────────────
def solve_unified_joint(sc: Scene, init: dict, anchor_weight: float = 5.0,
                        max_nfev: int = 200):
    """모든 관측 동시 최적화: {T_base_Ci, gTc, cube[set]}.
    cube 는 자유변수이며 FK soft anchor(anchor_weight)로 base gauge 를 고정한다."""
    cam_ids = [ci for ci in sc.fixed_cam_ids if ci in init["cams"]]
    if not cam_ids or init.get("gTc") is None or not sc.sets:
        return {**init, "mode": "unified_joint", "cost": None,
                "fail": "insufficient init (need fixed cams + gTc + >=1 set)"}

    sets = sc.sets
    cube0 = {s: sc.fk_cube[s] for s in sets}

    p0 = [se3_to_vec(init["cams"][ci]) for ci in cam_ids]
    p0.append(se3_to_vec(init["gTc"]))
    p0 += [se3_to_vec(cube0[s]) for s in sets]
    p0 = np.concatenate(p0)

    n_cam = len(cam_ids)
    off_gtc = n_cam * 6
    off_cube = off_gtc + 6
    cube_idx = {s: off_cube + i * 6 for i, s in enumerate(sets)}

    def unpack(p):
        cams = {ci: vec_to_se3(p[i * 6:(i + 1) * 6]) for i, ci in enumerate(cam_ids)}
        gTc = vec_to_se3(p[off_gtc:off_gtc + 6])
        cube = {s: vec_to_se3(p[cube_idx[s]:cube_idx[s] + 6]) for s in sets}
        return cams, gTc, cube

    aw = float(anchor_weight)

    def resid(p):
        cams, gTc, cube = unpack(p)
        r = []
        for (ci, e, s, T_co) in sc.obs_fixed:
            if ci in cams:
                r.append(se3_residual(cams[ci] @ T_co, cube[s]))
        for (e, s, T_go) in sc.obs_grip:
            r.append(se3_residual(sc.bTg[e] @ gTc @ T_go, cube[s]))
        # FK soft anchor (gauge fixing): pull each cube[s] toward its FK prior.
        if aw > 0.0:
            for s in sets:
                r.append(aw * se3_residual(cube[s], sc.fk_cube[s]))
        return np.concatenate(r) if r else np.zeros(1)

    sol = least_squares(resid, p0, method="lm", max_nfev=max_nfev)
    cams, gTc, cube = unpack(sol.x)
    return {"cams": cams, "gTc": gTc, "cube": cube, "mode": "unified_joint",
            "cost": float(sol.cost)}


def solve_joint_fk_fixed(sc: Scene, init: dict, max_nfev: int = 200):
    """cube 를 FK 로 고정하고 {T_base_Ci, gTc} 만 동시 최적화."""
    cam_ids = [ci for ci in sc.fixed_cam_ids if ci in init["cams"]]
    if not cam_ids or init.get("gTc") is None or not sc.sets:
        return {**init, "mode": "joint_fk_fixed", "cost": None,
                "fail": "insufficient init"}
    p0 = np.concatenate([se3_to_vec(init["cams"][ci]) for ci in cam_ids]
                        + [se3_to_vec(init["gTc"])])
    off_gtc = len(cam_ids) * 6

    def unpack(p):
        cams = {ci: vec_to_se3(p[i * 6:(i + 1) * 6]) for i, ci in enumerate(cam_ids)}
        return cams, vec_to_se3(p[off_gtc:off_gtc + 6])

    def resid(p):
        cams, gTc = unpack(p)
        r = []
        for (ci, e, s, T_co) in sc.obs_fixed:
            if ci in cams:
                r.append(se3_residual(cams[ci] @ T_co, sc.fk_cube[s]))
        for (e, s, T_go) in sc.obs_grip:
            r.append(se3_residual(sc.bTg[e] @ gTc @ T_go, sc.fk_cube[s]))
        return np.concatenate(r) if r else np.zeros(1)

    sol = least_squares(resid, p0, method="lm", max_nfev=max_nfev)
    cams, gTc = unpack(sol.x)
    return {"cams": cams, "gTc": gTc, "cube": dict(sc.fk_cube),
            "mode": "joint_fk_fixed", "cost": float(sol.cost)}


def _se3_average(Ts: List[np.ndarray]) -> np.ndarray:
    Ts = [np.asarray(T, float) for T in Ts]
    t = np.mean([T[:3, 3] for T in Ts], axis=0)
    R = Rotation.from_matrix([T[:3, :3] for T in Ts]).mean().as_matrix()
    out = np.eye(4)
    out[:3, :3] = R
    out[:3, 3] = t
    return out


# ── Evaluation (base frame) ───────────────────────────────────────────────────
@dataclass
class JointResult:
    method: str
    n_fixed_obs: int
    n_grip_obs: int
    n_sets: int
    consistency_trans_rmse_mm: Optional[float]
    consistency_rot_rmse_deg: Optional[float]
    grip_align_trans_rmse_mm: Optional[float]   # gripper-only prediction vs cube (base)
    cube_pos_err_vs_fk_mm: Optional[float]
    optimizer_cost: Optional[float]
    note: str = ""


def evaluate(sc: Scene, model: dict) -> JointResult:
    cams = model.get("cams", {})
    gTc = model.get("gTc")
    cube = model.get("cube", sc.fk_cube)

    trans_e, rot_e, grip_te, cube_pe = [], [], [], []
    n_fixed = n_grip = 0
    for (ci, e, s, T_co) in sc.obs_fixed:
        if ci not in cams:
            continue
        pred = cams[ci] @ T_co
        d = se3_residual(pred, cube[s])
        rot_e.append(np.degrees(np.linalg.norm(d[:3])))
        trans_e.append(np.linalg.norm(d[3:]) * 1000.0)
        n_fixed += 1
    for (e, s, T_go) in sc.obs_grip:
        if gTc is None:
            continue
        pred = sc.bTg[e] @ gTc @ T_go
        d = se3_residual(pred, cube[s])
        rot_e.append(np.degrees(np.linalg.norm(d[:3])))
        trans_e.append(np.linalg.norm(d[3:]) * 1000.0)
        grip_te.append(np.linalg.norm(pred[:3, 3] - cube[s][:3, 3]) * 1000.0)
        n_grip += 1
    for s in sc.sets:
        if s in cube and s in sc.fk_cube:
            cube_pe.append(np.linalg.norm(cube[s][:3, 3] - sc.fk_cube[s][:3, 3]) * 1000.0)

    def rms(x):
        return float(np.sqrt(np.mean(np.square(x)))) if x else None

    return JointResult(
        method=model.get("mode", "?"),
        n_fixed_obs=n_fixed, n_grip_obs=n_grip, n_sets=len(sc.sets),
        consistency_trans_rmse_mm=rms(trans_e),
        consistency_rot_rmse_deg=rms(rot_e),
        grip_align_trans_rmse_mm=rms(grip_te),
        cube_pos_err_vs_fk_mm=rms(cube_pe),
        optimizer_cost=model.get("cost"),
        note=model.get("fail", ""),
    )


def save_model(out_dir: str, model: dict) -> None:
    d = os.path.join(out_dir, model.get("mode", "model"))
    cp.ensure_dir(d)
    for ci, T in model.get("cams", {}).items():
        np.save(os.path.join(d, f"T_base_C{ci}.npy"), np.asarray(T, float))
    if model.get("gTc") is not None:
        np.save(os.path.join(d, "T_gripper_cam.npy"), np.asarray(model["gTc"], float))
    for s, T in model.get("cube", {}).items():
        np.save(os.path.join(d, f"T_base_O_set{s}.npy"), np.asarray(T, float))


def main() -> None:
    ap = argparse.ArgumentParser(description="C1 ablation: unified joint vs independent (real data)")
    ap.add_argument("--root_folder", required=True)
    ap.add_argument("--intrinsics_dir", required=True)
    ap.add_argument("--out_dir", default=None)
    ap.add_argument("--gripper_cam_idx", type=int, default=None)
    ap.add_argument("--ref_fixed_cam_idx", type=int, default=None)
    ap.add_argument("--cube_config_json", type=str, default=None)
    ap.add_argument("--max_err_fixed", type=float, default=3.0)
    ap.add_argument("--max_err_gripper", type=float, default=5.0)
    ap.add_argument("--fixed_cube_min_aspect", type=float, default=0.0)
    ap.add_argument("--gripper_cube_min_aspect", type=float, default=0.35)
    ap.add_argument("--gripper_cube_min_markers", type=int, default=1)
    ap.add_argument("--anchor_weight", type=float, default=5.0,
                    help="unified_joint 에서 cube[set] 를 FK 로 당기는 gauge-anchor 가중치.")
    ap.add_argument("--max_nfev", type=int, default=200)
    args = ap.parse_args()

    root = args.root_folder
    out_dir = cp.ensure_dir(args.out_dir or os.path.join("CP_result", "C1"))
    with open(os.path.join(root, "meta.json"), "r") as f:
        meta = json.load(f)

    cfg, cfg_source = cp.resolve_cube_config_for_run(
        root_folder=root, calib_dir=out_dir,
        cube_config_json=args.cube_config_json,
        default_cfg=cp.get_default_cube_config())
    meta_cfg, _ = cp.load_cube_config_from_meta(root, default_cfg=cfg)
    reuse_stored = cp.cube_configs_equivalent(meta_cfg, cfg)
    cube = cp.AprilTagCubeTarget(cfg)

    all_cam_ids = sorted({
        int(k) for cap in meta.get("captures", [])
        for k, v in cap.get("cams", {}).items() if v.get("saved")})
    if not all_cam_ids:
        raise RuntimeError("No saved cameras in meta.json")

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

    fixed_cam_ids = [ci for ci in all_cam_ids if ci != int(gripper_cam_idx)]
    if len(fixed_cam_ids) < 1:
        raise RuntimeError("Need at least one fixed camera")

    K_map, D_map = {}, {}
    for ci in all_cam_ids:
        K_map[ci], D_map[ci], _ = cp.load_intrinsics_with_depth_scale(args.intrinsics_dir, ci)

    event_to_set: Dict[int, Optional[int]] = {}
    for cap in meta.get("captures", []):
        eid = int(cap.get("event_id", -1))
        if eid >= 0:
            sidx = cp.get_capture_set_index(cap)
            event_to_set[eid] = int(sidx) if sidx is not None else None

    robot_T = s3.load_robot_poses_from_meta(meta)
    set_priors = cp.load_nominal_set_cube_transforms(meta)

    pose_obs = cp.load_pose_observations(
        root=root, meta=meta, cube=cube, K_map=K_map, D_map=D_map,
        all_cam_ids=all_cam_ids, gripper_cam_idx=int(gripper_cam_idx),
        reuse_stored_cube_candidates=reuse_stored,
        max_err_fixed=float(args.max_err_fixed),
        max_err_gripper=float(args.max_err_gripper),
        min_aspect_fixed=float(args.fixed_cube_min_aspect),
        min_aspect_gripper=float(args.gripper_cube_min_aspect),
        gripper_min_markers=int(args.gripper_cube_min_markers))

    sc = build_scene(pose_obs, robot_T, set_priors, fixed_cam_ids,
                     int(gripper_cam_idx), event_to_set)

    print(f"[INFO] cube config source: {cfg_source}")
    print(f"[INFO] fixed={sc.fixed_cam_ids}, gripper=cam{sc.gripper_cam_idx}, sets={sc.sets}")
    print(f"[INFO] obs: fixed={len(sc.obs_fixed)}, gripper={len(sc.obs_grip)}, FK sets={len(sc.fk_cube)}")
    if len(sc.sets) < 3:
        print(f"[WARN] only {len(sc.sets)} set(s) with FK cube — base gauge is weakly "
              f"constrained; treat numbers as smoke-test only (need >=3 sets).")

    indep = solve_independent(sc, max_nfev=args.max_nfev)
    joint = solve_unified_joint(sc, indep, anchor_weight=float(args.anchor_weight),
                                max_nfev=args.max_nfev)
    joint_fk = solve_joint_fk_fixed(sc, indep, max_nfev=args.max_nfev)

    models = [indep, joint, joint_fk]
    results = [evaluate(sc, m) for m in models]
    for m in models:
        save_model(out_dir, m)

    rows = [asdict(r) for r in results]
    with open(os.path.join(out_dir, "joint_ablation_summary.json"), "w") as f:
        json.dump(rows, f, indent=2, ensure_ascii=False)
    with open(os.path.join(out_dir, "joint_ablation_summary.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)

    print("\n" + "=" * 96)
    print("C1 JOINT-vs-INDEPENDENT SUMMARY  (lower = better)")
    print("=" * 96)
    hdr = (f"{'method':16s} {'nFix':>5s} {'nGrp':>5s} {'sets':>4s} "
           f"{'cons_t_mm':>10s} {'cons_r_deg':>11s} {'grip_t_mm':>10s} "
           f"{'cube_vs_fk_mm':>13s} {'cost':>10s}")
    print(hdr)
    print("-" * len(hdr))

    def f(x, nd=3):
        return "NA" if x is None else f"{x:.{nd}f}"
    for r in results:
        print(f"{r.method:16s} {r.n_fixed_obs:5d} {r.n_grip_obs:5d} {r.n_sets:4d} "
              f"{f(r.consistency_trans_rmse_mm,2):>10s} {f(r.consistency_rot_rmse_deg,3):>11s} "
              f"{f(r.grip_align_trans_rmse_mm,2):>10s} {f(r.cube_pos_err_vs_fk_mm,2):>13s} "
              f"{f(r.optimizer_cost,4):>10s}")
    print(f"\n[DONE] summary: {os.path.join(out_dir, 'joint_ablation_summary.csv')}")


if __name__ == "__main__":
    main()
