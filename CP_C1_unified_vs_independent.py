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

평가(시뮬 unified_vs_independent.eval_model 과 정렬):
  - consistency / cube_vs_fk : train set 에서 서브시스템 정합·FK 대비 큐브오차.
  - 다운스트림 큐브예측(held-out): 카메라를 train set 으로만 맞춘 뒤 test set 큐브를
    base 에서 예측해 FK(정답 프록시) 대비 RMSE. `+fk` = train 잔차를 [1,x,y] 에
    Ridge 회귀해 뺀 보정판(시뮬의 C 잔차보정). --test_sets / --holdout_frac 로 활성.

시뮬레이션 짝: Simul_test/joint_calib.py (calib_independent_aligned / calib_joint /
calib_joint_fk_fixed) + unified_vs_independent.eval_model (downstream + `+fk`).
이 스크립트는 그 구조를 실데이터 관측으로 포팅한 것이다.

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


# ── FK 큐브 prior 의 회전 보정 ────────────────────────────────────────────────
# meta.json 의 set_cube_center_6dof 는 큐브 **중심 위치**는 맞지만 **자세(회전)** 가
# 실제 큐브와 어긋나 있다(data/session 에서 179.9deg — 사실상 뒤집힘). C1 은 이 prior 를
# 회전까지 포함해 카메라 등록의 앵커로 쓰기 때문에(solve_independent) base 프레임이
# 통째로 뒤집히고, 카메라간 일치도가 2mm 대에서 14mm 대로, held-out 큐브예측이 320mm 로
# 부풀려진다. C3 는 같은 문제를 initialize_base_translation_anchored 에서 이미
# "위치만 앵커, 회전은 기각" 으로 처리한다 — 여기서도 같은 진단을 쓰되, 기각에서 그치지
# 않고 **관측에서 복원한 회전으로 대체**한다(더 좋은 앵커).
#
# 보정은 train set 관측만으로 추정한 하나의 상수 회전 R_corr (큐브 로컬) 이며 모든 set 에
# 동일하게 적용된다. 위치는 FK 값 그대로 둔다 — held-out 지표(downstream_rmse)는 위치만
# 쓰므로 test set 정보가 새어 들어가지 않는다.
def correct_fk_cube_rotation(sc: Scene, pose_obs, set_priors, set_pose6, event_to_set,
                             train_sets: List[int], ref_cam: int,
                             max_rot_error_deg: float = 45.0) -> Tuple[Dict[int, np.ndarray], dict]:
    """(보정된 fk_cube, 진단) 반환. 회전 불일치가 임계값 이하면 원본을 그대로 돌려준다."""
    diag = {"applied": False, "median_rot_delta_deg": None, "n_train_sets_used": 0, "reason": ""}
    train = {int(s) for s in train_sets}
    obs_train = [o for o in pose_obs
                 if int(o.cam) in sc.fixed_cam_ids
                 and (o.set_idx if o.set_idx is not None else event_to_set.get(int(o.event))) in train]
    if len({int(o.cam) for o in obs_train}) < 2:
        diag["reason"] = "고정 카메라 동시관측 부족 — 보정 생략"
        return dict(sc.fk_cube), diag

    try:
        _, _, T_base_O_event, _, _, _ = cp.initialize_base_translation_anchored(
            pose_obs=obs_train, fixed_cam_ids=sc.fixed_cam_ids, ref_cam=int(ref_cam),
            set_priors={s: T for s, T in set_priors.items() if int(s) in train},
            set_pose6={s: p for s, p in (set_pose6 or {}).items() if int(s) in train},
            event_to_set=event_to_set,
            max_trans_error_mm=1e9,          # 위치 앵커는 진단용 임계값에 걸리지 않게
            max_rot_error_deg=float(max_rot_error_deg),
            disable_if_inconsistent=True)
    except Exception as exc:                 # 앵커 실패 시 원본 유지 (조용히 틀리지 않게)
        diag["reason"] = f"base 앵커 실패: {exc}"
        return dict(sc.fk_cube), diag

    # set 별 관측 큐브 자세 -> FK prior 자세 대비 상수 보정 R_corr 추정
    by_set: Dict[int, List[np.ndarray]] = {}
    for eid, T in T_base_O_event.items():
        s = event_to_set.get(int(eid))
        if s is not None and int(s) in train:
            by_set.setdefault(int(s), []).append(np.asarray(T, float))
    deltas, rot_errs = [], []
    for s, Ts in by_set.items():
        if int(s) not in sc.fk_cube:
            continue
        R_obs = cp.weighted_se3_average(Ts, None)[:3, :3]
        R_fk = sc.fk_cube[int(s)][:3, :3]
        deltas.append(R_fk.T @ R_obs)
        rot_errs.append(rot_deg(R_fk, R_obs))
    if len(deltas) < 2:
        diag["reason"] = "train set 이 2개 미만 — 보정 생략"
        return dict(sc.fk_cube), diag

    diag["median_rot_delta_deg"] = float(np.median(rot_errs))
    diag["n_train_sets_used"] = len(deltas)
    if diag["median_rot_delta_deg"] <= float(max_rot_error_deg):
        diag["reason"] = (f"FK prior 회전이 관측과 {diag['median_rot_delta_deg']:.1f}deg 로 일치 "
                          f"(<= {max_rot_error_deg}deg) — 보정 불필요")
        return dict(sc.fk_cube), diag

    R_corr = Rotation.from_matrix(np.asarray(deltas)).mean().as_matrix()
    spread = [rot_deg(D, R_corr) for D in deltas]
    out = {}
    for s, T in sc.fk_cube.items():          # test set 포함 전 set 에 동일 보정
        Tn = np.array(T, dtype=float, copy=True)
        Tn[:3, :3] = T[:3, :3] @ R_corr
        out[int(s)] = Tn
    diag.update(applied=True,
                corr_angle_deg=float(np.degrees(np.linalg.norm(Rotation.from_matrix(R_corr).as_rotvec()))),
                set_spread_deg=float(np.median(spread)),
                reason=(f"FK prior 회전이 관측과 median {diag['median_rot_delta_deg']:.1f}deg 어긋남 "
                        f"(> {max_rot_error_deg}deg) — train {len(deltas)}개 set 에서 복원한 "
                        f"상수 회전으로 대체"))
    return out, diag


# ── Independent (separate) solve ──────────────────────────────────────────────
def solve_independent(sc: Scene, max_nfev: int = 100, robust: bool = True):
    """고정 카메라: 각 cam 을 FK 큐브 기준 closed-form 평균으로 base 에 등록.
    그리퍼: gTc 를 (FK 큐브 기준) 단독 least-squares 로 추정. 서로 독립.

    robust=True 면 카메라별 평균을 MAD 기반 이상치 제거 평균으로 낸다. 큐브가 축퇴된
    각도로 보이는 촬영(예: 옆면 2개만, 정면에 가깝게)에서 PnP 가 뒤집힌 해를 내놓는
    일이 있고, 단순 평균은 그런 소수 관측에 카메라 pose 전체가 끌려간다."""
    cams: Dict[int, np.ndarray] = {}
    for ci in sc.fixed_cam_ids:
        Ts = [sc.fk_cube[s] @ cp.inv_T(T_co)
              for (c, e, s, T_co) in sc.obs_fixed if c == ci]
        if Ts:
            cams[ci] = cp.robust_se3_average(Ts, None)[0] if robust and len(Ts) >= 4 else _se3_average(Ts)

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


# ── Held-out (train/test) split + downstream cube prediction + FK residual ─────
# 시뮬 짝: unified_vs_independent.eval_model 의 (4) 다운스트림 큐브예측 + `+fk`(잔차
# Ridge 보정 C). 실데이터에는 GT 가 없으므로 로봇 FK 큐브를 정답 프록시로 쓴다:
# 카메라는 train set 으로만 맞추고, held-out test set 큐브 위치를 base 에서 예측해
# FK 와 비교한다. `+fk` 는 train 잔차 (예측 vs FK) 를 [1,x,y] 특징에 Ridge 회귀해 뺀다.
def _resid_feature(t: np.ndarray) -> np.ndarray:
    """큐브 base 위치 (x,y) 의 선형 특징 [1, x, y] (시뮬 abc_calib._resid_feature 와 동일)."""
    t = np.asarray(t, float).reshape(3)
    return np.array([1.0, t[0], t[1]], dtype=float)


def subset_scene(sc: Scene, keep_sets) -> Scene:
    """keep_sets 에 속한 관측만 남긴 train 전용 Scene (카메라 fit 용)."""
    keep = {int(s) for s in keep_sets}
    obs_f = [o for o in sc.obs_fixed if int(o[2]) in keep]
    obs_g = [o for o in sc.obs_grip if int(o[1]) in keep]
    events = {int(e) for (_, e, _, _) in obs_f} | {int(e) for (e, _, _) in obs_g}
    return Scene(
        fixed_cam_ids=list(sc.fixed_cam_ids),
        gripper_cam_idx=int(sc.gripper_cam_idx),
        obs_fixed=obs_f, obs_grip=obs_g,
        bTg={e: T for e, T in sc.bTg.items() if int(e) in events},
        fk_cube={s: T for s, T in sc.fk_cube.items() if int(s) in keep},
        sets=sorted(keep & set(sc.sets)),
    )


def predict_cube_base_pos(model: dict, sc: Scene, s: int,
                          robust: bool = True) -> Optional[np.ndarray]:
    """고정 카메라 + 그리퍼(gTc 경유) 관측을 base 로 올려 set s 큐브 위치(3,) 예측.

    robust=True 면 축별 중앙값을 쓴다. set 하나에 관측이 수십 개인데 그중 소수가 PnP
    뒤집힘으로 100mm 이상 튀는 경우가 있어(평균은 그대로 끌려간다) 중앙값이 안전하다."""
    cams = model.get("cams", {})
    gTc = model.get("gTc")
    Ts = [cams[ci] @ T_co for (ci, e, ss, T_co) in sc.obs_fixed
          if int(ss) == int(s) and ci in cams]
    if gTc is not None:
        Ts += [sc.bTg[e] @ gTc @ T_go for (e, ss, T_go) in sc.obs_grip
               if int(ss) == int(s) and int(e) in sc.bTg]
    if not Ts:
        return None
    if robust and len(Ts) >= 3:
        return np.median(np.array([T[:3, 3] for T in Ts]), axis=0)
    return _se3_average(Ts)[:3, 3]


def learn_fk_ridge(model: dict, sc_train: Scene, train_sets: List[int],
                   lam: float = 1e-3) -> Optional[np.ndarray]:
    """train 에서 (예측 큐브위치 vs FK) 잔차를 [1,x,y] 에 Ridge 회귀 → 계수 W (3x3)."""
    X, Y = [], []
    for s in train_sets:
        p = predict_cube_base_pos(model, sc_train, s)
        if p is None or int(s) not in sc_train.fk_cube:
            continue
        X.append(_resid_feature(p))
        Y.append(sc_train.fk_cube[int(s)][:3, 3] - p)
    if len(X) < 3:
        return None
    X = np.asarray(X, float)
    Y = np.asarray(Y, float)
    reg = float(lam) * np.eye(X.shape[1])
    reg[0, 0] = 0.0                       # 절편은 정규화하지 않음
    return np.linalg.solve(X.T @ X + reg, X.T @ Y)


def learn_fk_offset(model: dict, sc_train: Scene,
                    train_sets: List[int]) -> Optional[np.ndarray]:
    """모델1 baseline: `[1]` 만 = train 잔차 (p_FK - p̂) 의 평균 offset (3,).

    Ridge `[1,x,y]` 에서 x,y 기울기를 뺀 절편-only 판. held-out 감소가 단순 평균
    offset 때문인지(=위치 무관 bias) 진짜 위치 의존 기울기 때문인지 구분하는 기준선."""
    diffs = []
    for s in train_sets:
        p = predict_cube_base_pos(model, sc_train, s)
        if p is None or int(s) not in sc_train.fk_cube:
            continue
        diffs.append(sc_train.fk_cube[int(s)][:3, 3] - p)
    if not diffs:
        return None
    return np.mean(np.asarray(diffs, float), axis=0)


def downstream_axis_errs(model: dict, sc_eval: Scene, eval_sets: List[int],
                         W: Optional[np.ndarray] = None,
                         T_rigid: Optional[np.ndarray] = None,
                         offset: Optional[np.ndarray] = None) -> np.ndarray:
    """downstream_rmse 와 동일 예측·보정. 축별 해석용으로 signed 오차 (N,3) mm 반환.

    보정은 셋 중 하나만 준다: W(Ridge[1,x,y]) / T_rigid(SE(3)) / offset([1]). 모두
    None 이면 raw. 축별 RMS 는 이 배열을 fold 간 pool 해서 계산한다(단일 test set 이면
    fold 당 1행이라 per-fold RMS 는 절댓값과 같아져 왜곡되므로 pool 이 맞다)."""
    errs = []
    for s in eval_sets:
        p = predict_cube_base_pos(model, sc_eval, s)
        if p is None or int(s) not in sc_eval.fk_cube:
            continue
        if T_rigid is not None:
            t = T_rigid[:3, :3] @ p + T_rigid[:3, 3]
        elif offset is not None:
            t = p + offset
        elif W is not None:
            t = p + _resid_feature(p) @ W
        else:
            t = p
        errs.append((t - sc_eval.fk_cube[int(s)][:3, 3]) * 1000.0)
    return np.asarray(errs, float) if errs else np.zeros((0, 3))


def learn_fk_rigid(model: dict, sc_train: Scene,
                   train_sets: List[int]) -> Optional[np.ndarray]:
    """train 에서 (예측 큐브위치 -> FK 큐브위치) 를 강체 SE(3) 로 Kabsch 정렬한 T (4x4).

    Ridge `[1,x,y]` 는 3x3 자유변수라 회전·스케일·전단을 모두 흡수한다. 남은 오차가
    실제로 **base 프레임 정렬 잔차**(회전+평행이동)라면 자유도 6 짜리 강체변환으로도
    같은 만큼 잡혀야 하고, 그렇다면 물리적으로 해석 가능한(=촬영/앵커 개선으로 없앨 수
    있는) 오차라는 뜻이다. 둘을 나란히 재서 그걸 구분한다."""
    src, dst = [], []
    for s in train_sets:
        p = predict_cube_base_pos(model, sc_train, s)
        if p is None or int(s) not in sc_train.fk_cube:
            continue
        src.append(p)
        dst.append(sc_train.fk_cube[int(s)][:3, 3])
    if len(src) < 3:
        return None
    return cp.kabsch_rigid(np.asarray(src, float), np.asarray(dst, float))


def downstream_rmse(model: dict, sc_eval: Scene, eval_sets: List[int],
                    W: Optional[np.ndarray],
                    T_rigid: Optional[np.ndarray] = None) -> Optional[float]:
    """eval_sets 큐브 위치를 예측(+선택적 보정)해 FK 대비 RMSE(mm).

    W: Ridge `[1,x,y]` 잔차보정 (3x3).  T_rigid: 강체 SE(3) 정렬 (4x4). 둘 다 train 에서만
    학습하며 동시에 주지 않는다."""
    errs = []
    for s in eval_sets:
        p = predict_cube_base_pos(model, sc_eval, s)
        if p is None or int(s) not in sc_eval.fk_cube:
            continue
        if T_rigid is not None:
            t = T_rigid[:3, :3] @ p + T_rigid[:3, 3]
        else:
            t = p + (_resid_feature(p) @ W if W is not None else 0.0)
        errs.append(np.linalg.norm(t - sc_eval.fk_cube[int(s)][:3, 3]) * 1000.0)
    return float(np.sqrt(np.mean(np.square(errs)))) if errs else None


def resolve_test_sets(available_sets: List[int], test_sets_arg: str,
                      holdout_frac: float, split_seed: int) -> List[int]:
    """--test_sets(명시) 또는 --holdout_frac(무작위)로 held-out set 목록을 정한다."""
    avail = sorted(int(s) for s in available_sets)
    if str(test_sets_arg).strip():
        want = {int(t) for t in str(test_sets_arg).replace(";", ",").split(",") if t.strip()}
        return sorted(want & set(avail))
    if float(holdout_frac) > 0.0 and len(avail) >= 2:
        import random as _random
        n_test = max(1, int(round(len(avail) * float(holdout_frac))))
        n_test = min(n_test, len(avail) - 1)     # keep >=1 train set
        shuffled = list(avail)
        _random.Random(int(split_seed)).shuffle(shuffled)
        return sorted(shuffled[len(shuffled) - n_test:])
    return []


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
    # --- held-out downstream cube prediction (FK as GT proxy). None if no split. ---
    train_sets: Optional[str] = None
    test_sets: Optional[str] = None
    n_test_sets: Optional[int] = None
    downstream_trans_rmse_mm: Optional[float] = None      # no-fk (raw prediction)
    downstream_fk_trans_rmse_mm: Optional[float] = None    # +fk (Ridge residual corrected)
    downstream_se3_trans_rmse_mm: Optional[float] = None   # +se3 (rigid SE(3) aligned)
    fk_rigid_angle_deg: Optional[float] = None             # 그 강체보정의 회전 크기
    fk_rigid_trans_mm: Optional[float] = None
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
    # --- held-out (train/test) 다운스트림 큐브예측 (시뮬 eval_model 의 downstream_mm 짝) ---
    # 카메라는 train set 으로만 맞추고, held-out test set 큐브를 예측해 FK 대비 오차를 잰다.
    # 공정 비교의 핵심: unified 가 학습에 쓴 FK 로 그대로 평가되어 유리해지지 않도록.
    ap.add_argument("--test_sets", type=str, default="",
                    help="held-out 으로 뺄 set_index 목록(콤마 구분). 지정 시 --holdout_frac 무시.")
    ap.add_argument("--holdout_frac", type=float, default=0.0,
                    help="test 로 뺄 set 비율(0~1). 0(기본)이면 split 없이 전체 fit(다운스트림 NA).")
    ap.add_argument("--split_seed", type=int, default=0,
                    help="--holdout_frac 무작위 분할 시드(재현성).")
    ap.add_argument("--fixed_min_markers", type=int, default=2,
                    help="고정카메라 큐브자세가 써야 할 최소 마커 수(기본 2). 1개는 PnP 뒤집힘 "
                         "모호성이라 ~150도/~140mm 교차카메라 이상치의 주원인.")
    ap.add_argument("--exclude_gripped", type=lambda v: str(v).lower() not in ("0", "false", "no"),
                    default=True,
                    help="로봇이 큐브를 잡은 상태(cube_gripped)의 캡처 제외(기본 True). 그 캡처는 "
                         "큐브가 그리퍼와 함께 이동해 set_cube_center_6dof 가 큐브 위치를 뜻하지 "
                         "않고, eye-in-hand 는 타깃이 카메라와 같이 움직여 핸드아이가 퇴화한다.")
    ap.add_argument("--fk_prior_rotation", type=str, default="auto", choices=["auto", "use"],
                    help="auto(기본): meta 의 set_cube_center_6dof 회전이 관측과 크게 어긋나면 "
                         "train set 관측에서 복원한 회전으로 대체. use: 원본 회전 그대로 "
                         "(예전 동작 — base 프레임이 뒤집혀 오차가 부풀려진다).")
    ap.add_argument("--fk_prior_max_rot_deg", type=float, default=45.0,
                    help="이 각도를 넘게 어긋나면 FK prior 회전을 보정한다.")
    ap.add_argument("--robust_average", type=lambda v: str(v).lower() not in ("0", "false", "no"),
                    default=True,
                    help="카메라 등록·큐브예측 평균에 이상치 제거 사용 (기본 True).")
    ap.add_argument("--fixed_cam_solve", type=str, default="reproj", choices=["reproj", "off"],
                    help="reproj(기본): 각 모델의 고정 카메라를 큐브 pose 고정 상태에서 "
                         "재투영오차로 최종 정제(방법 04). off: 정제 생략(SE(3) 일관성 solve 에서 멈춤).")
    ap.add_argument("--ridge_lambda", type=float, default=1e-3,
                    help="`+fk` 잔차보정 Ridge 정규화 세기 (시뮬 lam 기본 1e-3).")
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
        gripper_min_markers=int(args.gripper_cube_min_markers),
        exclude_gripped=bool(args.exclude_gripped),
        fixed_min_markers=int(args.fixed_min_markers))

    sc = build_scene(pose_obs, robot_T, set_priors, fixed_cam_ids,
                     int(gripper_cam_idx), event_to_set)

    if args.exclude_gripped:
        n_gr = sum(1 for c in meta.get("captures", []) if c.get("cube_gripped"))
        print(f"[INFO] exclude_gripped=True: skipped {n_gr} captures taken while holding the cube")
    print(f"[INFO] cube config source: {cfg_source}")
    print(f"[INFO] fixed={sc.fixed_cam_ids}, gripper=cam{sc.gripper_cam_idx}, sets={sc.sets}")
    print(f"[INFO] obs: fixed={len(sc.obs_fixed)}, gripper={len(sc.obs_grip)}, FK sets={len(sc.fk_cube)}")
    dcc = cp.depth_scale_crosscheck(meta)
    if dcc.get("n"):
        pr = dcc.get("median_plane_residual_mm")
        pr_txt = "n/a" if pr is None else f"{pr:.1f}mm"
        print(f"[INFO] depth scale cross-check (diagnostic, NOT in solver): n={dcc['n']} "
              f"pred/meas median={dcc['median_pred_over_meas']:.4f} "
              f"(vision scale {dcc['implied_vision_scale_pct']:+.1f}%), plane residual median={pr_txt}")
    else:
        print(f"[INFO] depth scale cross-check: {dcc.get('reason', 'unavailable')}")
    fcc = cp.fk_scale_crosscheck(meta)
    if fcc.get("n"):
        cams_txt = " ".join(
            f"cam{ci}={d['median']:.4f}[{d['iqr_lo']:.3f},{d['iqr_hi']:.3f}]n{d['n']}"
            for ci, d in sorted(fcc["per_cam"].items())
        )
        print(f"[INFO] FK scale cross-check (diagnostic, NOT in solver): "
              f"reliable |dVis|/|dFK| median={fcc['reliable_median_vis_over_fk']:.4f} "
              f"(FK scale {fcc['implied_fk_scale_pct']:+.1f}% vs vision; "
              f"n_reliable={fcc['n_reliable']}/{fcc['n']}) | {cams_txt}")
    else:
        print(f"[INFO] FK scale cross-check: {fcc.get('reason', 'unavailable')}")
    rps = cp.robot_pos_scale()
    est = cp.estimate_robot_pos_scale(meta)
    est_txt = (f"data estimate k={est['k']:.4f} (robot short {est['implied_robot_short_pct']:+.1f}%, "
               f"n_reliable={est['n_reliable']})") if est.get("k") else f"data estimate: {est.get('reason','n/a')}"
    print(f"[INFO] robot_pos_scale applied={rps:.4f} "
          f"(CP_common.ROBOT_POS_SCALE_PINNED); {est_txt}")
    if len(sc.sets) < 3:
        print(f"[WARN] only {len(sc.sets)} set(s) with FK cube — base gauge is weakly "
              f"constrained; treat numbers as smoke-test only (need >=3 sets).")

    # ── held-out split: 카메라는 train set 으로만 fit, test set 은 다운스트림 평가에만 ──
    test_set_ids = resolve_test_sets(sc.sets, args.test_sets, args.holdout_frac,
                                     args.split_seed)
    if str(args.test_sets).strip():
        missing = sorted({int(t) for t in str(args.test_sets).replace(";", ",").split(",")
                          if t.strip()} - set(sc.sets))
        if missing:
            print(f"[WARN] --test_sets {missing} not in available sets {sc.sets}; ignored")
    train_set_ids = [s for s in sc.sets if s not in set(test_set_ids)]
    # FK 큐브 prior 의 회전 보정 (위치는 유지). train set 관측만으로 추정한다.
    prior_fix_diag = {"applied": False, "reason": "disabled"}
    if str(args.fk_prior_rotation).lower() != "use":
        ref_cam = args.ref_fixed_cam_idx if args.ref_fixed_cam_idx is not None else fixed_cam_ids[0]
        fit_sets = train_set_ids if test_set_ids else list(sc.sets)
        set_pose6 = cp.load_nominal_set_cube_pose6(meta)
        fixed_cube, prior_fix_diag = correct_fk_cube_rotation(
            sc, pose_obs, set_priors, set_pose6, event_to_set, fit_sets, int(ref_cam),
            max_rot_error_deg=float(args.fk_prior_max_rot_deg))
        if prior_fix_diag["applied"]:
            sc = Scene(fixed_cam_ids=sc.fixed_cam_ids, gripper_cam_idx=sc.gripper_cam_idx,
                       obs_fixed=sc.obs_fixed, obs_grip=sc.obs_grip, bTg=sc.bTg,
                       fk_cube=fixed_cube, sets=sc.sets)
        print(f"[C1] FK prior 회전: {prior_fix_diag['reason']}"
              + (f" (보정각 {prior_fix_diag['corr_angle_deg']:.1f}deg, "
                 f"set 간 산포 {prior_fix_diag['set_spread_deg']:.2f}deg)"
                 if prior_fix_diag["applied"] else ""))

    sc_fit = sc
    if test_set_ids:
        if not train_set_ids:
            raise RuntimeError(f"train set empty after removing test_sets={test_set_ids}")
        sc_fit = subset_scene(sc, train_set_ids)
        print(f"[C1] train/test split: train={train_set_ids} test={test_set_ids} "
              f"(fit obs: fixed={len(sc_fit.obs_fixed)} grip={len(sc_fit.obs_grip)})")
    else:
        print("[C1] no train/test split (fit on all sets; downstream metrics NA). "
              "Use --test_sets or --holdout_frac for the held-out cube-prediction comparison.")

    indep = solve_independent(sc_fit, max_nfev=args.max_nfev, robust=bool(args.robust_average))
    joint = solve_unified_joint(sc_fit, indep, anchor_weight=float(args.anchor_weight),
                                max_nfev=args.max_nfev)
    joint_fk = solve_joint_fk_fixed(sc_fit, indep, max_nfev=args.max_nfev)

    models = [indep, joint, joint_fk]

    # ── 고정 카메라를 재투영오차로 최종 정제 (방법 04, 03/robust 폴백) ──
    # Step3 STEP-D-3 와 같은 방식: 각 모델의 큐브 pose 를 base gauge 로 고정하고 고정
    # 카메라만 픽셀 재투영으로 다듬는다. 코너 관측은 한 번만 검출해 세 모델이 공유한다.
    if str(args.fixed_cam_solve) != "off":
        corner_obs_c1, _reason = cp.detect_corner_observations(
            root=root, meta=meta, cube=cube, K_map=K_map, D_map=D_map,
            all_cam_ids=fixed_cam_ids, gripper_cam_idx=int(gripper_cam_idx),
            max_err_fixed=float(args.max_err_fixed), max_err_gripper=float(args.max_err_gripper),
            min_aspect_fixed=0.0, min_aspect_gripper=0.0, exclude_gripped=bool(args.exclude_gripped))
        for m in models:
            cams = m.get("cams")
            cube_m = m.get("cube")
            if not cams or not cube_m:
                continue
            # per-event 큐브 pose(base) = 그 event 가 속한 set 의 모델 큐브 pose (train fit)
            T_bo_by_event = {int(e): np.asarray(cube_m[int(s)], float)
                             for (ci, e, s, _T) in sc_fit.obs_fixed if int(s) in cube_m}
            if not T_bo_by_event:
                continue
            refined, rdiag = s3.refine_fixed_cams_with_reprojection(
                root, meta, cube, K_map, D_map, cams, T_bo_by_event,
                list(cams.keys()), int(gripper_cam_idx), corner_obs=corner_obs_c1)
            m["cams"] = refined
            m["reproj_refine"] = rdiag

    results = [evaluate(sc_fit, m) for m in models]

    # 다운스트림 큐브예측 (held-out): raw 예측과 `+fk`(Ridge 잔차보정) 둘 다 기록.
    #   캘리브는 방식당 한 번, no-fk/+fk 는 예측단계에서만 다름 (시뮬과 동일 구조).
    if test_set_ids:
        for m, r in zip(models, results):
            W = learn_fk_ridge(m, sc_fit, train_set_ids, lam=float(args.ridge_lambda))
            r.train_sets = ",".join(str(s) for s in train_set_ids)
            r.test_sets = ",".join(str(s) for s in test_set_ids)
            r.n_test_sets = len(test_set_ids)
            r.downstream_trans_rmse_mm = downstream_rmse(m, sc, test_set_ids, None)
            r.downstream_fk_trans_rmse_mm = downstream_rmse(m, sc, test_set_ids, W)
            T_rig = learn_fk_rigid(m, sc_fit, train_set_ids)
            if T_rig is not None:
                r.downstream_se3_trans_rmse_mm = downstream_rmse(m, sc, test_set_ids, None,
                                                                 T_rigid=T_rig)
                r.fk_rigid_angle_deg = float(np.degrees(np.linalg.norm(
                    Rotation.from_matrix(T_rig[:3, :3]).as_rotvec())))
                r.fk_rigid_trans_mm = float(np.linalg.norm(T_rig[:3, 3]) * 1000.0)

    for m in models:
        save_model(out_dir, m)

    rows = [asdict(r) for r in results]
    with open(os.path.join(out_dir, "joint_ablation_summary.json"), "w") as f:
        json.dump(rows, f, indent=2, ensure_ascii=False)
    with open(os.path.join(out_dir, "fk_prior_rotation_fix.json"), "w") as f:
        json.dump(prior_fix_diag, f, indent=2, ensure_ascii=False)
    with open(os.path.join(out_dir, "joint_ablation_summary.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)

    has_test = any(r.n_test_sets for r in results)
    print("\n" + "=" * 112)
    print("C1 JOINT-vs-INDEPENDENT SUMMARY  (lower = better)"
          + ("  (+ held-out downstream cube prediction)" if has_test else ""))
    print("=" * 112)
    hdr = (f"{'method':16s} {'nFix':>5s} {'nGrp':>5s} {'sets':>4s} "
           f"{'cons_t_mm':>10s} {'cons_r_deg':>11s} {'grip_t_mm':>10s} "
           f"{'cube_vs_fk_mm':>13s} {'cost':>10s}")
    if has_test:
        hdr += f" {'|down_mm':>9s} {'down+fk_mm':>10s} {'down+se3_mm':>11s}"
    print(hdr)
    print("-" * len(hdr))

    def f(x, nd=3):
        return "NA" if x is None else f"{x:.{nd}f}"
    for r in results:
        line = (f"{r.method:16s} {r.n_fixed_obs:5d} {r.n_grip_obs:5d} {r.n_sets:4d} "
                f"{f(r.consistency_trans_rmse_mm,2):>10s} {f(r.consistency_rot_rmse_deg,3):>11s} "
                f"{f(r.grip_align_trans_rmse_mm,2):>10s} {f(r.cube_pos_err_vs_fk_mm,2):>13s} "
                f"{f(r.optimizer_cost,4):>10s}")
        if has_test:
            line += (f" {f(r.downstream_trans_rmse_mm,2):>9s} {f(r.downstream_fk_trans_rmse_mm,2):>10s}"
                     f" {f(r.downstream_se3_trans_rmse_mm,2):>11s}")
        print(line)
    if has_test:
        print("\n[C1] down_mm = held-out 큐브예측 RMSE(mm, FK 프록시 대비). "
              "down+fk_mm = train 잔차 Ridge[1,x,y] 보정 후, down+se3_mm = train 강체 SE(3) 정렬 후. "
              "둘 다 train 에서만 학습해 test 예측에 적용한다.")
        for r in results:
            if r.fk_rigid_angle_deg is not None:
                print(f"     {r.method:16s} 강체보정: 회전 {r.fk_rigid_angle_deg:.2f}deg, "
                      f"평행이동 {r.fk_rigid_trans_mm:.1f}mm")
    print(f"\n[DONE] summary: {os.path.join(out_dir, 'joint_ablation_summary.csv')}")


if __name__ == "__main__":
    main()
