#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""zeus_gello_calibration/fit_corrected_fk.py -- corrected-FK 변형들을 한 번에
만들어서 같은 데이터/같은 지표로 전부 비교한다.

목표: 카메라 간 일치도(cross-view/cam-common)와 못 본 큐브 held-out(FK 기준
GT) 둘 다 통합_no-fk / 통합_raw-fk보다 좋은 방식 찾기.

*** 손으로 정한 상수 없음 ***: FK 공분산(sigma), 보정 변환, 회귀 계수는 전부
데이터에서 추정하는 변수다. 픽셀 잔차의 robust loss(soft_l1, f_scale=2px)는
기존 solve_corner_reprojection의 canonical 설정을 그대로 쓴다(모든 baseline과
공통이라 비교 조건이 아님).

session2 큐브 pose를 다루는 방식 (anchor_s = FK_place_s @ T_gripper_cube):
  free      큐브 pose 자유 변수 (통합_no-fk)
  hard      큐브 pose = 보정된 anchor (raw-fk의 일반화)
  soft      큐브 pose 자유 변수 + 보정된 anchor 쪽으로 당기는 FK factor,
            공분산은 데이터에서 추정(EM처럼 fit -> sigma 재추정 -> refit 반복)

보정(correction) 종류 (전부 최소제곱 안의 변수):
  none      보정 없음
  R         anchor @ exp(cR)            -- 그리퍼 프레임 고정 오프셋(놓을 때 큐브가
                                          그리퍼 기준으로 일정하게 밀림)
  L         exp(cL) @ anchor            -- base 프레임 고정 오프셋
  LR        exp(cL) @ anchor @ exp(cR)
  lin       anchor @ exp(J phi_s)       -- 놓은 자세(x, y, cos rz, sin rz)에 선형인
                                          pose 의존 보정. J는 6xk 변수, 초기값은
                                          no-fk fit 잔차의 선형회귀.

조합 예: hard+R = "vision-aligned FK"(A5류), soft+none = A4(soft FK factor,
sigma 추정), soft+R = A4 + 계통 오프셋, hard+lin = pose 의존 보정 FK 고정.

평가(전부 eval_heldout_and_consistency.py와 동일 코드):
  train px / held-out px,mm,deg(FK 기준 GT, leave-one-out) /
  cross-view px, cam-common mm,deg (train-pooled + held-out)

사용법:
  python fit_corrected_fk.py                       # 전 변형
  python fit_corrected_fk.py --variants soft+none hard+R
  python fit_corrected_fk.py --skip-heldout        # train만 (빠름)
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np
from scipy.optimize import least_squares
from scipy.sparse import lil_matrix
from scipy.spatial.transform import Rotation

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT))

from calibration_pipeline.apriltag_cube import inv_T  # noqa: E402
from calibration_pipeline.fk_factor import fk_pose_residual, robustify_elementwise  # noqa: E402
from calibration_pipeline.reprojection import (  # noqa: E402
    PoseState, SolverOptions, project_points, retract, set_state_transform, state_transform, variable_keys,
)
from calibration_pipeline.table1 import estimate_board_handeye_initial  # noqa: E402
from robot.backends.zeus_client import pose6_to_T  # noqa: E402

import eval_heldout_and_consistency as ehc  # noqa: E402
from fit_calibration_methods import (  # noqa: E402
    GRIPPER_LOCAL_ID, SESSION1_DIR_DEFAULT, SESSION3_DIR_DEFAULT, export_fit_json, fk_anchor_cubes,
    init_cube_poses, load_all_data, per_corner_errors, rmse_px, solve_unified,
)
from session2_pick_and_place import SESSION2_DIR_DEFAULT  # noqa: E402


# ------------------------------------------------------------------ SE(3) 유틸
def se3_exp(v):
    v = np.asarray(v, dtype=np.float64).reshape(6)
    T = np.eye(4)
    T[:3, :3] = Rotation.from_rotvec(v[:3]).as_matrix()
    T[:3, 3] = v[3:]
    return T


def se3_log(T):
    T = np.asarray(T, dtype=np.float64)
    return np.concatenate([Rotation.from_matrix(T[:3, :3]).as_rotvec(), T[:3, 3]])


def place_features(items_by_index, set_ids):
    """놓은 자세(FK)에서 뽑은 특징 phi_s = [1, x_c, y_c, cos rz, sin rz].
    x,y는 학습 세트 평균으로 중심화(m 단위) -- 평균도 데이터에서 나온다."""
    xy = np.array([[items_by_index[s]["target"][0], items_by_index[s]["target"][1]] for s in set_ids]) / 1000.0
    center = xy.mean(axis=0)
    feats = {}
    for s in set_ids:
        t = items_by_index[s]["target"]
        rz = np.deg2rad(t[3])
        feats[s] = np.array([1.0, t[0] / 1000.0 - center[0], t[1] / 1000.0 - center[1], np.cos(rz), np.sin(rz)])
    return feats, center


# ------------------------------------------------------------- 문제 정의
class CorrectedFKProblem:
    """픽셀 재투영 잔차(canonical, elementwise soft_l1 robustified) +
    선택적 FK factor 잔차 + 보정 변수. scipy least_squares(loss='linear')."""

    def __init__(self, observations, reference_state, base_keys, robot_T, K_map, D_map, gripper_id,
                 anchors, cube_mode, corr_mode, features=None, fk_sigma=None,
                 corr_init=None, options=SolverOptions()):
        self.obs = list(observations)
        self.ref = reference_state.clone()
        self.base_keys = list(base_keys)
        self.robot_T, self.K, self.D, self.gripper = robot_T, K_map, D_map, int(gripper_id)
        self.anchors = {int(k): np.asarray(v, dtype=np.float64) for k, v in anchors.items()}
        self.cube_mode, self.corr_mode = cube_mode, corr_mode
        self.features = features or {}
        self.fk_sigma = None if fk_sigma is None else np.asarray(fk_sigma, dtype=np.float64).reshape(6)
        self.options = options
        self.scaling = options.scaling

        self.slices = {k: slice(6 * i, 6 * (i + 1)) for i, k in enumerate(self.base_keys)}
        n = 6 * len(self.base_keys)
        self.extra = {}
        if corr_mode in ("L", "LR"):
            self.extra["cL"] = slice(n, n + 6); n += 6
        if corr_mode in ("R", "LR"):
            self.extra["cR"] = slice(n, n + 6); n += 6
        if corr_mode == "lin":
            k = len(next(iter(self.features.values())))
            self.k_feat = k
            self.extra["J"] = slice(n, n + 6 * k); n += 6 * k
        self.n_params = n
        self.x0 = np.zeros(n)
        if corr_init:
            for name, val in corr_init.items():
                if name in self.extra:
                    self.x0[self.extra[name]] = np.asarray(val, dtype=np.float64).reshape(-1)

        self.set_ids_in_obs = sorted({int(o.set_idx) for o in self.obs if o.set_idx is not None and o.grasp_idx is None and o.marker == "cube"})
        self.factor_active = (cube_mode == "soft" and self.fk_sigma is not None)
        self.row_offsets = []
        row = 0
        for o in self.obs:
            n_c = len(np.asarray(o.image_points).reshape(-1, 2))
            self.row_offsets.append((row, row + 2 * n_c))
            row += 2 * n_c
        self.n_pixel_rows = row
        self.factor_rows = {}
        if self.factor_active:
            for s in self.set_ids_in_obs:
                self.factor_rows[s] = (row, row + 6)
                row += 6
        self.n_rows = row

    # --- 보정된 anchor
    def corrected_anchor(self, s, x):
        A = self.anchors[s]
        if "cL" in self.extra:
            A = se3_exp(x[self.extra["cL"]]) @ A
        if "cR" in self.extra:
            A = A @ se3_exp(x[self.extra["cR"]])
        if "J" in self.extra:
            J = x[self.extra["J"]].reshape(6, self.k_feat)
            A = A @ se3_exp(J @ self.features[s])
        return A

    def unpack(self, x):
        st = self.ref.clone()
        for k, sl in self.slices.items():
            set_state_transform(st, k, retract(state_transform(self.ref, k), x[sl], self.scaling))
        if self.cube_mode == "hard":
            for s in self.set_ids_in_obs:
                st.cubes[s] = self.corrected_anchor(s, x)
        return st

    def residual(self, x):
        st = self.unpack(x)
        out = np.empty(self.n_rows)
        for o, (r0, r1) in zip(self.obs, self.row_offsets):
            if o.marker == "board":
                target = st.board
            elif o.grasp_idx is not None:
                target = self.robot_T[int(o.event)] @ st.grasps[int(o.grasp_idx)]
            else:
                target = st.cubes[int(o.set_idx)]
            if int(o.cam) == self.gripper:
                T_bc = self.robot_T[int(o.event)] @ st.gtc
            else:
                T_bc = st.cams[int(o.cam)]
            pred = project_points(inv_T(T_bc) @ target, o.object_points, self.K[int(o.cam)], self.D[int(o.cam)])
            r = (pred - np.asarray(o.image_points).reshape(-1, 2)).reshape(-1)
            w = 1.0 if self.options.residual_weighting == "per_corner" else 1.0 / np.sqrt(len(pred))
            out[r0:r1] = robustify_elementwise(r * w, self.options.loss, self.options.f_scale_px)
        for s, (r0, r1) in self.factor_rows.items():
            out[r0:r1] = fk_pose_residual(st.cubes[s], self.corrected_anchor(s, x)) / self.fk_sigma
        return out

    def raw_pixel_residual(self, x):
        st = self.unpack(x)
        errs = []
        for o in self.obs:
            if o.marker == "board":
                target = st.board
            elif o.grasp_idx is not None:
                target = self.robot_T[int(o.event)] @ st.grasps[int(o.grasp_idx)]
            else:
                target = st.cubes[int(o.set_idx)]
            T_bc = self.robot_T[int(o.event)] @ st.gtc if int(o.cam) == self.gripper else st.cams[int(o.cam)]
            pred = project_points(inv_T(T_bc) @ target, o.object_points, self.K[int(o.cam)], self.D[int(o.cam)])
            errs.extend((pred - np.asarray(o.image_points).reshape(-1, 2)).reshape(-1).tolist())
        return np.asarray(errs)

    def sparsity(self):
        M = lil_matrix((self.n_rows, self.n_params), dtype=np.int8)
        extra_cols = [sl for sl in self.extra.values()]
        for o, (r0, r1) in zip(self.obs, self.row_offsets):
            cam_key = ("gtc", -1) if int(o.cam) == self.gripper else ("cam", int(o.cam))
            if o.marker == "board":
                tkey = ("board", -1)
            elif o.grasp_idx is not None:
                tkey = ("grasp", int(o.grasp_idx))
            else:
                tkey = ("cube", int(o.set_idx))
            for k in (cam_key, tkey):
                if k in self.slices:
                    M[r0:r1, self.slices[k]] = 1
            if tkey[0] == "cube" and self.cube_mode == "hard":
                for sl in extra_cols:
                    M[r0:r1, sl] = 1
        for s, (r0, r1) in self.factor_rows.items():
            if ("cube", s) in self.slices:
                M[r0:r1, self.slices[("cube", s)]] = 1
            for sl in extra_cols:
                M[r0:r1, sl] = 1
        return M.tocsr()

    def solve(self, max_nfev=300):
        sol = least_squares(self.residual, self.x0, method="trf", loss="linear", x_scale="jac",
                            jac_sparsity=self.sparsity(), max_nfev=max_nfev, xtol=1e-8, ftol=1e-8, gtol=1e-8)
        st = self.unpack(sol.x)
        raw = self.raw_pixel_residual(sol.x)
        corr = {name: sol.x[sl].copy() for name, sl in self.extra.items()}
        return st, sol, float(np.sqrt(np.mean(raw ** 2))), corr


# ------------------------------------------------------------- 변형 실행
def _base_setup(data, gtc_init, board_init):
    cam_init, grasp_init = data["cam_init"], data["grasp_init"]
    K_map, D_map = data["K_map"], data["D_map"]
    obs_s2 = data["obs_s2_fixed"] + data["obs_s2_gripper"]
    set_ids = sorted(data["items_by_index"])
    robot_T = {**data["robot_T_s1"], **data["robot_T_s2_gripper"], **data["robot_T_s3"]}
    anchors = fk_anchor_cubes(data["items_by_index"], grasp_init)
    cubes_vis = init_cube_poses(obs_s2, K_map, D_map, cam_init, gtc_init, robot_T, GRIPPER_LOCAL_ID, set_ids)
    observations = data["obs_s1"] + obs_s2 + data.get("obs_s2_board", []) + data["obs_s3"]
    observations = [o for o in observations if o.set_idx is None or int(o.set_idx) in anchors or o.grasp_idx is not None]
    return dict(cam_init=cam_init, grasp_init=grasp_init, K_map=K_map, D_map=D_map, set_ids=set_ids,
                robot_T=robot_T, anchors=anchors, cubes_vis=cubes_vis, observations=observations,
                gtc_init=gtc_init, board_init=board_init)


def _nofk_state(data, gtc_init, board_init):
    st, _, _ = solve_unified(data, "no_fk", gtc_init, board_init)
    return st


def residuals_vs_anchor(state, anchors):
    """no-fk fit한 큐브 vs FK anchor: r_s = log(inv(anchor_s) @ cube_s) (그리퍼 프레임 기준)."""
    return {s: fk_pose_residual(anchors[s], state.cubes[s]) for s in anchors if s in state.cubes}


def robust_sigma(residuals):
    R = np.array(list(residuals.values()))
    med = np.median(R, axis=0)
    mad = np.median(np.abs(R - med), axis=0) * 1.4826
    return np.maximum(mad, 1e-6), med


def solve_variant(data, variant, gtc_init, board_init, n_em=2, verbose=False):
    """variant = 'cube_mode+corr_mode'. 반환 (state, info)."""
    cube_mode, corr_mode = variant.split("+")
    S = _base_setup(data, gtc_init, board_init)
    anchors, set_ids = S["anchors"], S["set_ids"]
    feats, _ = place_features(data["items_by_index"], set_ids)

    # 1) no-fk fit -> 잔차 통계 (보정/sigma 초기값은 전부 여기서 데이터로 추정)
    st0 = _nofk_state(data, gtc_init, board_init)
    res0 = residuals_vs_anchor(st0, anchors)
    sigma0, med0 = robust_sigma(res0)
    corr_init = {}
    if corr_mode in ("R", "LR"):
        corr_init["cR"] = med0                    # 그리퍼 프레임 계통 오프셋 = 잔차 중앙값
    if corr_mode in ("L", "LR"):
        Lres = {s: se3_log(st0.cubes[s] @ inv_T(anchors[s])) for s in res0}
        corr_init["cL"] = robust_sigma(Lres)[1]
    if corr_mode == "lin":
        Phi = np.array([feats[s] for s in res0])
        Rm = np.array([res0[s] for s in res0])
        J, *_ = np.linalg.lstsq(Phi, Rm, rcond=None)   # (k,6): phi -> r  선형회귀
        corr_init["J"] = J.T.reshape(-1)

    keys = ["T_base_Ci", "T_gripper_cam", "T_base_board", "T_gripper_cube_by_grasp"]
    if cube_mode in ("free", "soft"):
        keys.append("T_base_cube_by_set")
    cubes0 = dict(st0.cubes) if cube_mode in ("free", "soft") else dict(anchors)
    ref = PoseState(cams=dict(st0.cams), gtc=st0.gtc.copy(), board=st0.board.copy(),
                    cubes=cubes0, grasps={0: st0.grasps[0].copy()})
    base_keys = variable_keys(keys, ref)

    fk_sigma = None
    if cube_mode == "soft":
        # 보정 후 잔차의 산포로 sigma 초기화
        fk_sigma = sigma0
    info = {"sigma_iters": []}
    st, corr = ref, corr_init
    rounds = n_em if cube_mode == "soft" else 1
    for it in range(rounds):
        prob = CorrectedFKProblem(S["observations"], ref, base_keys, S["robot_T"], S["K_map"], S["D_map"],
                                  GRIPPER_LOCAL_ID, anchors, cube_mode, corr_mode, feats, fk_sigma, corr)
        st, sol, train_px, corr = prob.solve()
        ref = PoseState(cams=dict(st.cams), gtc=st.gtc.copy(), board=st.board.copy(),
                        cubes=dict(st.cubes), grasps={0: st.grasps[0].copy()})
        if cube_mode == "soft":
            # E-step: 보정된 anchor 기준 잔차로 sigma 재추정
            x_dummy = prob.x0.copy()
            for name, sl in prob.extra.items():
                x_dummy[sl] = corr[name]
            res = {s: fk_pose_residual(prob.corrected_anchor(s, x_dummy), st.cubes[s]) for s in set_ids if s in st.cubes}
            fk_sigma, _ = robust_sigma(res)
            info["sigma_iters"].append({"translation_mm": (fk_sigma[3:] * 1000).tolist(),
                                        "rotation_deg": np.degrees(fk_sigma[:3]).tolist()})
        if verbose:
            print(f"    [{variant}] round {it}: train_px={train_px:.4f} nfev={sol.nfev} success={sol.success}")
    info.update(train_px=train_px, success=bool(sol.success), corr={k: v.tolist() for k, v in corr.items()},
                nofk_resid_median_mm=(med0[3:] * 1000).tolist(), nofk_resid_sigma_mm=(sigma0[3:] * 1000).tolist(),
                nofk_resid_median_deg=np.degrees(med0[:3]).tolist(), nofk_resid_sigma_deg=np.degrees(sigma0[:3]).tolist())
    return st, info


# ------------------------------------------------------------- 평가 래퍼
def evaluate_variant(variant, data, gtc_init, board_init, robot_T_all, K_map, D_map, set_ids, skip_heldout):
    obs_all_s2 = data["obs_s2_fixed"] + data["obs_s2_gripper"]
    st, info = solve_variant(data, variant, gtc_init, board_init, verbose=True)
    out = {"variant": variant, "train_px": info["train_px"], "info": info}
    xv, _, _ = ehc.cross_view_pixel_transfer(obs_all_s2, st.cams, st.gtc, robot_T_all, K_map, D_map, GRIPPER_LOCAL_ID)
    mm, dg, _ = ehc.cross_camera_consistency(obs_all_s2, st.cams, st.gtc, robot_T_all, K_map, D_map, GRIPPER_LOCAL_ID)
    out.update(cross_view_px=xv, cam_common_mm=mm, cam_common_deg=dg)
    if not skip_heldout:
        original = ehc.fit_frozen

        def fit_frozen_variant(method, data_fold, fk_mode, g, b):
            s, _ = solve_variant(data_fold, variant, g, b)
            return s.cams, s.gtc
        ehc.fit_frozen = fit_frozen_variant
        try:
            h_px, _, per_mm, h_cross = ehc.evaluate_heldout(variant, data, "n/a", gtc_init, board_init,
                                                            robot_T_all, K_map, D_map, set_ids)
        finally:
            ehc.fit_frozen = original
        mm_list = [v["translation_mm"] for v in per_mm.values()]
        dg_list = [v["rotation_deg"] for v in per_mm.values()]
        out.update(heldout_px=h_px, heldout_mm=float(np.mean(mm_list)), heldout_deg=float(np.mean(dg_list)),
                   heldout_mm_max=float(np.max(mm_list)), heldout_cross=h_cross, per_set_mm=per_mm,
                   heldout_mm_joint=h_cross.get("heldout_joint_translation_mean_mm", float("nan")),
                   heldout_deg_joint=h_cross.get("heldout_joint_rotation_mean_deg", float("nan")))
    return st, out


ALL_VARIANTS = ["free+none", "hard+none", "soft+none", "hard+R", "hard+L", "hard+LR", "soft+R", "hard+lin", "soft+lin"]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--session1-dir", default=str(SESSION1_DIR_DEFAULT))
    ap.add_argument("--session1-capture-subdir", default="capture_replayed")
    ap.add_argument("--session2-dir", default=str(SESSION2_DIR_DEFAULT))
    ap.add_argument("--session2-capture-subdir", default="capture_placed")
    ap.add_argument("--session3-dir", default=str(SESSION3_DIR_DEFAULT))
    ap.add_argument("--session3-capture-subdir", default="capture_replayed")
    ap.add_argument("--zeus-intrinsics-dir", default=str(REPO_ROOT / "intrinsics"))
    ap.add_argument("--ur3-intrinsics-dir", default=str(REPO_ROOT / "ur3_calibration" / "intrinsics"))
    ap.add_argument("--device-map", default=str(REPO_ROOT / "intrinsics" / "device_map.json"))
    ap.add_argument("--fit-json", default=str(REPO_ROOT / "zeus_gello_calibration" / "pass1_grasp_offset_replayed.json"))
    ap.add_argument("--fixed-min-corners", type=int, default=8)
    ap.add_argument("--cube-observation-policy", default="legacy", choices=("legacy", "core_multiface"))
    ap.add_argument("--variants", nargs="*", default=ALL_VARIANTS)
    ap.add_argument("--skip-heldout", action="store_true")
    ap.add_argument("--export-fits", action="store_true", help="fit_<variant>.json 저장 (gt_compare용)")
    ap.add_argument("--out", default=str(REPO_ROOT / "zeus_gello_calibration" / "corrected_fk_results.json"))
    args = ap.parse_args()

    data = load_all_data(args)
    K_map, D_map = data["K_map"], data["D_map"]
    set_ids = sorted(data["items_by_index"])
    robot_T_all = {**data["robot_T_s1"], **data["robot_T_s2_gripper"], **data["robot_T_s3"]}
    gtc_init, board_init, _ = estimate_board_handeye_initial(data["obs_s3"], data["robot_T_s3"], K_map, D_map, GRIPPER_LOCAL_ID)

    results = {}
    for v in args.variants:
        t0 = time.time()
        print(f"=== {v} ===")
        st, out = evaluate_variant(v, data, gtc_init, board_init, robot_T_all, K_map, D_map, set_ids, args.skip_heldout)
        out["elapsed_s"] = time.time() - t0
        results[v] = out
        if args.export_fits:
            export_fit_json(REPO_ROOT / "zeus_gello_calibration" / f"fit_cfk_{v.replace('+', '_')}.json",
                            st.grasps[0], st.cams, st.gtc)
        line = f"  train_px={out['train_px']:.4f} xview={out['cross_view_px']:.4f} camcom={out['cam_common_mm']:.3f}mm/{out['cam_common_deg']:.3f}deg"
        if not args.skip_heldout:
            line += (f" | heldout px={out['heldout_px']:.4f} mm={out['heldout_mm']:.3f} deg={out['heldout_deg']:.3f} "
                     f"max={out['heldout_mm_max']:.2f} | ho_xview={out['heldout_cross']['cross_view_cube_pixel_transfer_rmse_px']:.4f} "
                     f"ho_camcom={out['heldout_cross']['cam_common_translation_mm']:.3f}")
        print(line + f"  ({out['elapsed_s']:.0f}s)")
        Path(args.out).write_text(json.dumps(results, indent=2, default=lambda o: str(o)))

    print(f"\n{'variant':>10} {'train_px':>9} {'ho_px':>7} {'ho_mm':>7} {'ho_deg':>7} {'ho_max':>7} {'ho_joint':>8} {'xview':>7} {'camcom_mm':>10} {'camcom_deg':>11}")
    for v, r in results.items():
        if args.skip_heldout:
            print(f"{v:>10} {r['train_px']:>9.4f} {'-':>7} {'-':>7} {'-':>7} {'-':>7} {'-':>8} {r['cross_view_px']:>7.3f} {r['cam_common_mm']:>10.3f} {r['cam_common_deg']:>11.3f}")
        else:
            print(f"{v:>10} {r['train_px']:>9.4f} {r['heldout_px']:>7.3f} {r['heldout_mm']:>7.3f} {r['heldout_deg']:>7.3f} "
                  f"{r['heldout_mm_max']:>7.2f} {r.get('heldout_mm_joint', float('nan')):>8.3f} {r['cross_view_px']:>7.3f} "
                  f"{r['cam_common_mm']:>10.3f} {r['cam_common_deg']:>11.3f}")
    print(f"\nwrote {args.out}")


if __name__ == "__main__":
    main()
