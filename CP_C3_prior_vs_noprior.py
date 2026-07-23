#!/usr/bin/env python3
"""
CP_C3_prior_vs_noprior.py  —  기여도 C3 독립 실험

C3: solve 에서 로봇 큐브중점을 정답(known)으로 강한 soft 제약(FK 사용)하느냐
    vs 미지수로 두고 vision 만으로 풀고 마지막에 base 등록만 하느냐(FK 미사용).

한 번 촬영한 세션(--root_folder)의 관측을 읽어, with-prior / without-prior 를
같은 데이터로 풀어 비교한다. 결과는 기본적으로 CP_result/C3 에 저장된다.
이 파일은 C1/C2 와 독립적으로 단독 실행된다(공유 로더는 CP_common 이 재노출).

<<명령어>>
python CP_C3_prior_vs_noprior.py \
  --root_folder ./data/session \
  --intrinsics_dir ./intrinsics
  # 결과 -> CP_result/C3 (기본).  --out_dir 로 변경 가능.

  (optional)
  --prior_weight_trans (기본 30) / --prior_weight_rot (기본 0)
  --prior_weight_sweep "0,1,10,30,100" [--sweep_axis trans|rot|both]
  --test_sets "3,7"  또는  --holdout_frac 0.3 --split_seed 0  (held-out 공정 비교)
------------------------------------------------------------------------------------

<<4가지 방법 비교>>
1) Per-camera PnP mean
2) Per-camera PnP + robust SE(3) averaging
3) PnP pose-consistency optimization
4) Direct reprojection-error optimization

<<FK 정보 사용 유무>>
1) without robot-known cube prior
2) with robot-known cube prior, when set_cube_center_6dof exists in meta.json
------------------------------------------------------------------------------------

<<시뮬 Exp3 (Camera-based / FK-based / Camera+FK후보정) 3방식 정렬>>
시뮬 exp3_gtc_estimation 의 세 방식을 실데이터 held-out(FK 프록시)로 그대로 잰다:
  - without-prior              == ① Camera-based   (큐브를 미지수로 vision 만으로 추정)
  - with-prior                 == ② FK-based       (로봇 FK 큐브중점을 solve 에 강제)
  - 05_camera_fk_correction    == ③ Camera+FK후보정 (①의 예측을 train 잔차 Ridge 로 후보정)
핵심 지표 test_prior_trans_rmse_mm(held-out FK 위치오차) 로 세 방식을 비교한다.
(--test_sets 또는 --holdout_frac 로 held-out split 을 켜야 05 및 test_* 지표가 나온다.)
------------------------------------------------------------------------------------

<<결과물>>
<out_dir>/ablation_summary.csv
<out_dir>/ablation_summary.json
<out_dir>/<method>__<prior_mode>/T_base_C{i}.npy or T_ref_C{i}.npy
<out_dir>/<method>__<prior_mode>/diagnostics.json

"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
from collections import defaultdict
from dataclasses import dataclass, asdict, replace
from typing import Any, Dict, Iterable, List, Optional, Tuple

import cv2
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation as R

from apriltag_cube import AprilTagCubeTarget, inv_T, rodrigues_to_Rt
from calibration_runtime_utils import (
    copy_depth_fields,
    rotation_error_deg,
    filter_candidates_for_camera_role,
    get_capture_set_index,
    get_capture_set_cube_center_transform_raw,
    load_intrinsics_with_depth_scale,
    resolve_cube_config_for_run,
    select_primary_cube_candidate,
)
from config import get_default_cube_config
from cube_config_utils import cube_configs_equivalent, load_cube_config_from_meta
from robot_comm import euler_deg_to_matrix


# -----------------------------
# Basic SE(3) utilities
# -----------------------------


# 공유 로더/기하/지표는 CP_common 으로 물리 분리됨. 되받아 네임스페이스 유지
# (보조 스크립트의 S.<name> 및 내부 참조 호환).
from CP_common import *  # noqa: F401,F403
from CP_common import (  # 명시 재노출(정적 분석/가독성)
    CornerObs,
    PoseObs,
    T_to_pose6_mm,
    T_to_vec,
    build_ref_relative_from_pairwise,
    detect_corner_observations,
    ensure_dir,
    estimate_image_cube_pose,
    estimate_object_poses_from_cams,
    get_marker_object_corners,
    initialize_base_translation_anchored,
    initialize_ref_object_poses,
    kabsch_rigid,
    load_nominal_set_cube_pose6,
    load_nominal_set_cube_transforms,
    load_pose_observations,
    load_robot_poses_from_meta,
    make_T,
    marker_aspect_ratio,
    observations_by_cam_event,
    pose6_to_T_base_gripper,
    pose_consistency_metrics,
    prior_metrics,
    reprojection_errors,
    robust_kabsch_rigid,
    robust_se3_average,
    se3_log_residual,
    stored_cube_pose_candidates,
    try_parse_pose6,
    vec_to_T,
    weighted_se3_average,
)


@dataclass
class MethodResult:
    method: str
    prior_mode: str
    ok: bool
    message: str
    n_pose_obs: int
    n_corner_obs: int
    reproj_rmse_px: Optional[float]
    reproj_median_px: Optional[float]
    pose_trans_rmse_mm: Optional[float]
    pose_rot_rmse_deg: Optional[float]
    prior_trans_rmse_mm: Optional[float]
    prior_rot_rmse_deg: Optional[float]
    output_dir: str
    # --- optimizer accept/reject (problem 3) ---
    optimizer_success: Optional[bool] = None
    optimizer_accepted: Optional[bool] = None
    optimizer_cost_initial: Optional[float] = None
    optimizer_cost_final: Optional[float] = None
    optimizer_fallback_reason: Optional[str] = None
    # --- robot cube prior diagnostics/rejection (problem 1) ---
    prior_num_total: Optional[int] = None
    prior_num_used: Optional[int] = None
    prior_num_rejected: Optional[int] = None
    prior_median_trans_error_mm: Optional[float] = None
    prior_median_rot_error_deg: Optional[float] = None
    prior_warning: Optional[str] = None
    # --- direct reprojection corner loading (problem 2) ---
    corner_obs_reason_if_zero: Optional[str] = None
    # --- C3 prior-weight sweep: weights actually used for this row ---
    prior_weight_trans_used: Optional[float] = None
    prior_weight_rot_used: Optional[float] = None
    # --- C3 held-out (train/test) evaluation: cameras fit on train sets, metrics
    #     measured on unseen test sets. None when no split is requested. ---
    train_sets: Optional[str] = None
    test_sets: Optional[str] = None
    test_prior_trans_rmse_mm: Optional[float] = None
    test_prior_rot_rmse_deg: Optional[float] = None
    test_reproj_rmse_px: Optional[float] = None
    test_reproj_median_px: Optional[float] = None
    test_n_events: Optional[int] = None


def build_param_layout(cam_ids: List[int], event_ids: List[int], ref_cam: Optional[int]) -> Dict[str, Any]:
    cam_vars = [ci for ci in cam_ids if ref_cam is None or ci != ref_cam]
    layout = {
        "cam_vars": cam_vars,
        "event_vars": event_ids,
        "cam_slice": {},
        "event_slice": {},
        "n": 0,
    }
    k = 0
    for ci in cam_vars:
        layout["cam_slice"][ci] = slice(k, k + 6)
        k += 6
    for eid in event_ids:
        layout["event_slice"][eid] = slice(k, k + 6)
        k += 6
    layout["n"] = k
    return layout


def pack_params(T_cam: Dict[int, np.ndarray], T_obj: Dict[int, np.ndarray], layout: Dict[str, Any]) -> np.ndarray:
    x = np.zeros(layout["n"], dtype=np.float64)
    for ci, sl in layout["cam_slice"].items():
        x[sl] = T_to_vec(T_cam[ci])
    for eid, sl in layout["event_slice"].items():
        x[sl] = T_to_vec(T_obj[eid])
    return x


def unpack_params(x: np.ndarray, layout: Dict[str, Any], ref_cam: Optional[int]) -> Tuple[Dict[int, np.ndarray], Dict[int, np.ndarray]]:
    T_cam: Dict[int, np.ndarray] = {}
    if ref_cam is not None:
        T_cam[ref_cam] = np.eye(4, dtype=np.float64)
    for ci, sl in layout["cam_slice"].items():
        T_cam[ci] = vec_to_T(x[sl])
    T_obj = {eid: vec_to_T(x[sl]) for eid, sl in layout["event_slice"].items()}
    return T_cam, T_obj


def prior_residual_terms(
    T_obj: Dict[int, np.ndarray],
    event_to_set: Dict[int, Optional[int]],
    set_priors: Dict[int, np.ndarray],
    w_trans: float,
    w_rot: float,
) -> List[float]:
    """Soft prior residuals with SEPARATE translation/rotation weights.

    Because the robot cube prior's rotation is unreliable, w_rot defaults to 0
    so only the (reliable) cube-center translation pulls the solution.
    """
    res: List[float] = []
    if not set_priors or (w_trans <= 0.0 and w_rot <= 0.0):
        return res
    for eid, T in T_obj.items():
        sidx = event_to_set.get(eid)
        if sidx is None or sidx not in set_priors:
            continue
        Terr = inv_T(set_priors[sidx]) @ T
        if w_trans > 0.0:
            res.extend((Terr[:3, 3] * float(w_trans)).tolist())
        if w_rot > 0.0:
            rv = R.from_matrix(Terr[:3, :3]).as_rotvec()
            res.extend((rv * float(w_rot)).tolist())
    return res


def _finalize_opt(
    init_T_cam, init_T_obj, opt_T_cam, opt_T_obj, residual, x0, opt
) -> Tuple[Dict[int, np.ndarray], Dict[int, np.ndarray], Dict[str, Any]]:
    """Accept the optimized result only if it actually lowered the cost.

    scipy `success` only means a termination criterion was hit, not that the
    objective improved. If cost did not improve, fall back to the initializer.
    """
    c0 = float(np.mean(residual(x0) ** 2))
    c1 = float(np.mean(residual(opt.x) ** 2))
    accepted = bool(opt.success) and (c1 < c0)
    info = {
        "optimized": True,
        "optimizer_success": bool(opt.success),
        "accepted": accepted,
        "cost_initial": c0,
        "cost_final": c1,
        "nfev": int(opt.nfev),
    }
    if accepted:
        return opt_T_cam, opt_T_obj, info
    info["fallback_reason"] = (
        "final cost was not lower than initial cost" if bool(opt.success)
        else f"optimizer did not converge (status={int(opt.status)})"
    )
    return init_T_cam, init_T_obj, info


def optimize_pose_consistency(
    pose_obs: List[PoseObs],
    fixed_cam_ids: List[int],
    init_T_cam: Dict[int, np.ndarray],
    init_T_obj: Dict[int, np.ndarray],
    ref_cam: Optional[int],
    event_to_set: Dict[int, Optional[int]],
    set_priors: Optional[Dict[int, np.ndarray]],
    prior_weight_trans: float,
    prior_weight_rot: float,
) -> Tuple[Dict[int, np.ndarray], Dict[int, np.ndarray], Dict[str, Any]]:
    event_ids = sorted(init_T_obj.keys())
    cam_ids = sorted([ci for ci in fixed_cam_ids if ci in init_T_cam])
    layout = build_param_layout(cam_ids, event_ids, ref_cam=ref_cam)
    x0 = pack_params(init_T_cam, init_T_obj, layout)

    usable = [o for o in pose_obs if o.cam in cam_ids and o.event in init_T_obj]
    if len(usable) < 4:
        return init_T_cam, init_T_obj, {"optimized": False, "accepted": False,
                                        "reason": "not enough pose observations"}

    def residual(x: np.ndarray) -> np.ndarray:
        T_cam, T_obj = unpack_params(x, layout, ref_cam=ref_cam)
        res = []
        for o in usable:
            pred = inv_T(T_cam[o.cam]) @ T_obj[o.event]
            e = se3_log_residual(inv_T(o.T_C_O) @ pred)
            w = math.sqrt(min(50.0, 1.0 / max(o.err_px, 1e-6)))
            res.extend((e * w).tolist())
        if set_priors:
            res.extend(prior_residual_terms(T_obj, event_to_set, set_priors,
                                            prior_weight_trans, prior_weight_rot))
        return np.asarray(res, dtype=np.float64)

    opt = least_squares(
        residual, x0, method="trf", loss="huber", f_scale=0.003,
        max_nfev=300, xtol=1e-10, ftol=1e-10, gtol=1e-10,
    )
    T_cam, T_obj = unpack_params(opt.x, layout, ref_cam=ref_cam)
    return _finalize_opt(init_T_cam, init_T_obj, T_cam, T_obj, residual, x0, opt)


def optimize_reprojection(
    corner_obs: List[CornerObs],
    pose_obs: List[PoseObs],
    fixed_cam_ids: List[int],
    init_T_cam: Dict[int, np.ndarray],
    init_T_obj: Dict[int, np.ndarray],
    ref_cam: Optional[int],
    K_map: Dict[int, np.ndarray],
    D_map: Dict[int, np.ndarray],
    event_to_set: Dict[int, Optional[int]],
    set_priors: Optional[Dict[int, np.ndarray]],
    prior_weight_trans: float,
    prior_weight_rot: float,
    pose_regularizer_weight: float,
) -> Tuple[Dict[int, np.ndarray], Dict[int, np.ndarray], Dict[str, Any]]:
    event_ids = sorted(init_T_obj.keys())
    cam_ids = sorted([ci for ci in fixed_cam_ids if ci in init_T_cam])
    layout = build_param_layout(cam_ids, event_ids, ref_cam=ref_cam)
    x0 = pack_params(init_T_cam, init_T_obj, layout)

    usable_corners = [o for o in corner_obs if o.cam in cam_ids and o.event in init_T_obj]
    usable_poses = [o for o in pose_obs if o.cam in cam_ids and o.event in init_T_obj]
    if len(usable_corners) < 4:
        return init_T_cam, init_T_obj, {"optimized": False, "accepted": False,
                                        "reason": "not enough corner observations or cube object-corner API unavailable"}

    def residual(x: np.ndarray) -> np.ndarray:
        T_cam, T_obj = unpack_params(x, layout, ref_cam=ref_cam)
        res: List[float] = []
        for o in usable_corners:
            T_C_O = inv_T(T_cam[o.cam]) @ T_obj[o.event]
            rvec = R.from_matrix(T_C_O[:3, :3]).as_rotvec().reshape(3, 1)
            tvec = T_C_O[:3, 3].reshape(3, 1)
            proj, _ = cv2.projectPoints(o.object_points.astype(np.float64), rvec, tvec, K_map[o.cam], D_map[o.cam])
            diff = (proj.reshape(-1, 2) - o.image_points.reshape(-1, 2)).reshape(-1)
            # Pixel residual. Robust loss handles bad corners.
            res.extend(diff.tolist())
        if pose_regularizer_weight > 0.0:
            for o in usable_poses:
                pred = inv_T(T_cam[o.cam]) @ T_obj[o.event]
                e = se3_log_residual(inv_T(o.T_C_O) @ pred)
                res.extend((e * float(pose_regularizer_weight)).tolist())
        if set_priors:
            # translation prior in meters -> pixel-like scale via weight; rotation off by default
            res.extend(prior_residual_terms(T_obj, event_to_set, set_priors,
                                            prior_weight_trans, prior_weight_rot))
        return np.asarray(res, dtype=np.float64)

    opt = least_squares(
        residual, x0, method="trf", loss="huber", f_scale=2.0,
        max_nfev=500, xtol=1e-10, ftol=1e-10, gtol=1e-10,
    )
    T_cam, T_obj = unpack_params(opt.x, layout, ref_cam=ref_cam)
    return _finalize_opt(init_T_cam, init_T_obj, T_cam, T_obj, residual, x0, opt)


def save_transforms(out_dir: str, T_cam: Dict[int, np.ndarray], prior_mode: str, ref_cam: Optional[int]) -> None:
    # Both modes now output in robot BASE frame; the difference is whether the
    # robot cube-center constrains the SOLVE (with) or is used only for the final
    # base registration (without).
    ensure_dir(out_dir)
    for ci, T in sorted(T_cam.items()):
        np.save(os.path.join(out_dir, f"T_base_C{ci}.npy"), T)
    note = ("Transforms are in robot BASE frame.\n"
            + ("Robot cube-center used as a soft constraint IN the solve.\n"
               if prior_mode == "with_robot_cube_prior"
               else "Vision-only solve; robot cube-center positions used ONLY for final base registration (gauge).\n"))
    with open(os.path.join(out_dir, "coordinate_note.txt"), "w") as f:
        f.write(note)


def evaluate_and_save(
    method: str,
    prior_mode: str,
    base_out: str,
    pose_obs: List[PoseObs],
    corner_obs: List[CornerObs],
    T_cam: Dict[int, np.ndarray],
    T_obj: Dict[int, np.ndarray],
    fixed_cam_ids: List[int],
    K_map: Dict[int, np.ndarray],
    D_map: Dict[int, np.ndarray],
    event_to_set: Dict[int, Optional[int]],
    set_priors: Dict[int, np.ndarray],
    diag: Dict[str, Any],
    ref_cam: Optional[int],
    corner_obs_reason: str = "",
    prior_stats: Optional[Dict[str, Any]] = None,
    test_bundle: Optional[Dict[str, Any]] = None,
) -> MethodResult:
    out_dir = ensure_dir(os.path.join(base_out, f"{method}__{prior_mode}"))
    save_transforms(out_dir, T_cam, prior_mode, ref_cam)

    e = reprojection_errors(corner_obs, T_cam, T_obj, K_map, D_map, fixed_cam_ids)
    reproj_rmse = float(np.sqrt(np.mean(e ** 2))) if e.size else None
    reproj_med = float(np.median(e)) if e.size else None
    pose_t, pose_r = pose_consistency_metrics(pose_obs, T_cam, T_obj, fixed_cam_ids)
    prior_t, prior_r = prior_metrics(T_obj, event_to_set, set_priors)

    # ── Held-out (train/test) evaluation ──────────────────────────────────────
    # T_cam here is already in base frame (mapped via to_base). Re-estimate the cube
    # pose on each TEST-set event from these cameras (test FK NOT used in the fit),
    # then score FK position error + reprojection on the unseen sets. This is the
    # fair C3 comparison: with-prior can't win just by fitting the same FK it's scored on.
    test_prior_t = test_prior_r = test_reproj_rmse = test_reproj_med = None
    test_n_events = None
    train_sets_str = test_sets_str = None
    if test_bundle:
        tp_obs = test_bundle.get("pose_obs", [])
        tc_obs = test_bundle.get("corner_obs", [])
        te2s = test_bundle.get("event_to_set", {})
        tpriors = test_bundle.get("set_priors", {})
        train_sets_str = test_bundle.get("train_sets_str")
        test_sets_str = test_bundle.get("test_sets_str")
        T_obj_test = estimate_object_poses_from_cams(tp_obs, T_cam, fixed_cam_ids)
        test_n_events = len(T_obj_test)
        test_prior_t, test_prior_r = prior_metrics(T_obj_test, te2s, tpriors)
        te = reprojection_errors(tc_obs, T_cam, T_obj_test, K_map, D_map, fixed_cam_ids)
        test_reproj_rmse = float(np.sqrt(np.mean(te ** 2))) if te.size else None
        test_reproj_med = float(np.median(te)) if te.size else None

    # optimizer accept/reject info (present only for methods 3/4)
    opt_success = diag.get("optimizer_success")
    opt_accepted = diag.get("accepted")
    opt_c0 = diag.get("cost_initial")
    opt_c1 = diag.get("cost_final")
    opt_fallback = diag.get("fallback_reason") or diag.get("reason") if diag.get("optimized") is False else diag.get("fallback_reason")
    ps = prior_stats or {}

    diagnostics = {
        "method": method,
        "prior_mode": prior_mode,
        "n_pose_obs": len(pose_obs),
        "n_corner_obs": len(corner_obs),
        "corner_obs_reason_if_zero": corner_obs_reason if len(corner_obs) == 0 else "",
        "reproj_rmse_px": reproj_rmse,
        "reproj_median_px": reproj_med,
        "pose_trans_rmse_mm": pose_t,
        "pose_rot_rmse_deg": pose_r,
        "prior_trans_rmse_mm": prior_t,
        "prior_rot_rmse_deg": prior_r,
        "prior_stats": ps,
        "extra": diag,
    }
    with open(os.path.join(out_dir, "diagnostics.json"), "w") as f:
        json.dump(diagnostics, f, indent=2, ensure_ascii=False)

    return MethodResult(
        method=method,
        prior_mode=prior_mode,
        ok=True,
        message="ok",
        n_pose_obs=len(pose_obs),
        n_corner_obs=len(corner_obs),
        reproj_rmse_px=reproj_rmse,
        reproj_median_px=reproj_med,
        pose_trans_rmse_mm=pose_t,
        pose_rot_rmse_deg=pose_r,
        prior_trans_rmse_mm=prior_t,
        prior_rot_rmse_deg=prior_r,
        output_dir=out_dir,
        optimizer_success=opt_success,
        optimizer_accepted=opt_accepted,
        optimizer_cost_initial=opt_c0,
        optimizer_cost_final=opt_c1,
        optimizer_fallback_reason=opt_fallback,
        prior_num_total=ps.get("num_prior_total"),
        prior_num_used=ps.get("num_prior_used"),
        prior_num_rejected=ps.get("num_prior_rejected"),
        prior_median_trans_error_mm=ps.get("median_prior_trans_error_mm"),
        prior_median_rot_error_deg=ps.get("median_prior_rot_error_deg"),
        prior_warning=ps.get("reason"),
        corner_obs_reason_if_zero=corner_obs_reason if len(corner_obs) == 0 else None,
        train_sets=train_sets_str,
        test_sets=test_sets_str,
        test_prior_trans_rmse_mm=test_prior_t,
        test_prior_rot_rmse_deg=test_prior_r,
        test_reproj_rmse_px=test_reproj_rmse,
        test_reproj_median_px=test_reproj_med,
        test_n_events=test_n_events,
    )


def save_prior_diagnostics(out_dir: str, prior_rows: List[Dict[str, Any]], stats: Dict[str, Any]) -> None:
    """Write per-event robot-cube-prior vs vision diagnostics (problem 1)."""
    ensure_dir(out_dir)
    with open(os.path.join(out_dir, "prior_diagnostics.json"), "w") as f:
        json.dump({"prior_stats": stats, "events": prior_rows}, f, indent=2, ensure_ascii=False)
    if not prior_rows:
        return
    cols = [
        "event_id", "set_index", "raw_set_cube_center_6dof",
        "prior_translation_m", "prior_rotation_rotvec",
        "estimated_cube_translation_m", "estimated_cube_rotation_rotvec",
        "delta_trans_mm", "delta_rot_deg",
        "possible_unit_warning", "possible_rotation_warning", "set_used_for_anchor",
    ]
    with open(os.path.join(out_dir, "prior_diagnostics.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=cols)
        w.writeheader()
        for r in prior_rows:
            w.writerow({k: (json.dumps(r[k]) if isinstance(r[k], (list, dict)) else r[k]) for k in cols})


def write_corrected_set_cube_center(
    out_dir: str, prior_rows: List[Dict[str, Any]], set_pose6_old: Dict[int, List[float]]
) -> Dict[str, Any]:
    """Back-write a CORRECTED set_cube_center_6dof using the vision-observed cube pose.

    The original prior's translation is reliable but its rotation (yaw) is not.
    Here we replace each set's full 6-DoF with the vision-estimated cube pose in
    robot base frame (per-set mean of `estimated_cube_*` from prior diagnostics).
    These corrected priors are mutually consistent, so a future calibration run
    can use the FULL prior (translation + rotation), not translation-only.
    """
    by_set: Dict[int, List[np.ndarray]] = defaultdict(list)
    for r in prior_rows:
        T = make_T(r["estimated_cube_rotation_rotvec"], r["estimated_cube_translation_m"])
        by_set[int(r["set_index"])].append(T)
    corrected: Dict[str, Any] = {}
    for s, Ts in sorted(by_set.items()):
        Tmean = weighted_se3_average(Ts)
        new6 = T_to_pose6_mm(Tmean)
        entry = {"corrected_set_cube_center_6dof": [round(v, 4) for v in new6], "n_events": len(Ts)}
        old = set_pose6_old.get(s)
        if old is not None:
            entry["old_set_cube_center_6dof"] = old
            entry["delta_yaw_deg"] = float(((new6[3] - old[3] + 180.0) % 360.0) - 180.0)
        corrected[str(s)] = entry
    payload = {
        "note": ("Vision-derived cube pose in robot base frame. Translation matches the "
                 "original prior (~8mm); rotation REPLACES the unreliable nominal yaw. "
                 "Drop these into meta.json set_cube_center_6dof for the next run to enable a full 6-DoF prior."),
        "sets": corrected,
    }
    with open(os.path.join(out_dir, "corrected_set_cube_center_6dof.json"), "w") as f:
        json.dump(payload, f, indent=2, ensure_ascii=False)
    return payload


# ── Camera+FK-correction (시뮬 Exp3 ③: camera-based estimation + FK 후보정) ──────
# without-prior(=Camera-based) 카메라로 train 큐브위치를 추정해 FK(정답 프록시) 대비 잔차를
# [1,x,y] 특징에 Ridge 회귀(W)로 배우고, held-out test 큐브예측에 그 보정을 적용한다.
# gTc/카메라는 바꾸지 않고 최종 예측만 후보정하므로 held-out 에서만 효과가 난다.
def _resid_feature(t: np.ndarray) -> np.ndarray:
    t = np.asarray(t, dtype=np.float64).reshape(3)
    return np.array([1.0, t[0], t[1]], dtype=np.float64)


def fk_corrected_heldout(
    T_cam: Dict[int, np.ndarray],
    fixed_cam_ids: List[int],
    train_pose_obs: List[PoseObs],
    event_to_set: Dict[int, Optional[int]],
    train_priors: Dict[int, np.ndarray],
    test_bundle: Dict[str, Any],
    lam: float = 1e-3,
) -> Optional[Dict[str, Any]]:
    """Return {'test_prior_trans_rmse_mm', 'n'} after applying the FK-learned
    residual correction to held-out cube predictions, or None if unlearnable."""
    T_obj_tr = estimate_object_poses_from_cams(train_pose_obs, T_cam, fixed_cam_ids)
    X, Y = [], []
    for eid, T in T_obj_tr.items():
        s = event_to_set.get(eid)
        if s is None or s not in train_priors:
            continue
        t = T[:3, 3]
        X.append(_resid_feature(t))
        Y.append(train_priors[s][:3, 3] - t)
    if len(X) < 3:
        return None
    X = np.asarray(X, dtype=np.float64)
    Y = np.asarray(Y, dtype=np.float64)
    reg = float(lam) * np.eye(X.shape[1])
    reg[0, 0] = 0.0
    W = np.linalg.solve(X.T @ X + reg, X.T @ Y)

    tp_obs = test_bundle.get("pose_obs", [])
    te2s = test_bundle.get("event_to_set", {})
    tpriors = test_bundle.get("set_priors", {})
    T_obj_te = estimate_object_poses_from_cams(tp_obs, T_cam, fixed_cam_ids)
    errs = []
    for eid, T in T_obj_te.items():
        s = te2s.get(eid)
        if s is None or s not in tpriors:
            continue
        t = T[:3, 3] + _resid_feature(T[:3, 3]) @ W
        errs.append(float(np.linalg.norm(t - tpriors[s][:3, 3]) * 1000.0))
    if not errs:
        return None
    return {"test_prior_trans_rmse_mm": float(np.sqrt(np.mean(np.square(errs)))),
            "n": len(errs)}


def run_method_suite(
    pose_obs: List[PoseObs],
    corner_obs: List[CornerObs],
    fixed_cam_ids: List[int],
    ref_cam: int,
    K_map: Dict[int, np.ndarray],
    D_map: Dict[int, np.ndarray],
    event_to_set: Dict[int, Optional[int]],
    set_priors: Dict[int, np.ndarray],
    set_pose6: Dict[int, List[float]],
    out_dir: str,
    with_prior: bool,
    corner_obs_reason: str,
    args: argparse.Namespace,
    weight_trans_override: Optional[float] = None,
    weight_rot_override: Optional[float] = None,
    emit_closed_form: bool = True,
    method_suffix: str = "",
    test_bundle: Optional[Dict[str, Any]] = None,
) -> List[MethodResult]:
    """Run the method suite for one prior mode.

    weight_trans_override / weight_rot_override: if given, use these instead of
      args.prior_weight_{trans,rot} (used by the C3 prior-weight sweep).
    emit_closed_form: emit rows 01/02 (closed-form vision baselines). They are
      independent of the prior, so the caller emits them only ONCE (in the
      without-prior pass) to avoid the duplicate rows that inflated the summary.
    method_suffix: appended to method names so sweep rows stay distinct.
    """
    prior_mode = "with_robot_cube_prior" if with_prior else "without_robot_cube_prior"
    results: List[MethodResult] = []
    if with_prior and not set_priors:
        return [MethodResult("all", prior_mode, False, "no set_cube_center_6dof prior in meta.json",
                             len(pose_obs), len(corner_obs), None, None, None, None, None, None, out_dir)]

    # ── Establish robot BASE frame for BOTH modes ────────────────────────────────
    # The base frame comes from the robot cube-center POSITIONS (Kabsch). The
    # with/without-prior axis is NOT the coordinate frame; it is whether those
    # known positions also CONSTRAIN the solve:
    #   without prior : cube poses are UNKNOWNS (vision-only solve); the robot
    #                   positions are used ONLY for the final base registration (gauge).
    #   with prior    : the robot cube-center is the KNOWN answer fed INTO the solve
    #                   as a strong soft constraint (translation now; rotation too once
    #                   --prior_weight_rot>0, i.e. when the server reports a good yaw).
    T_base_ref = np.eye(4, dtype=np.float64)
    prior_stats: Optional[Dict[str, Any]] = None
    diag_anchor: Dict[str, Any] = {}
    if set_priors:
        T_base_ref, _, _, diag_anchor, prior_rows, prior_stats = initialize_base_translation_anchored(
            pose_obs, fixed_cam_ids, ref_cam, set_priors, set_pose6, event_to_set,
            max_trans_error_mm=float(args.prior_max_trans_error_mm),
            max_rot_error_deg=float(args.prior_max_rot_error_deg),
            disable_if_inconsistent=bool(args.disable_prior_if_inconsistent),
        )
        if with_prior:  # write prior diagnostics/corrected priors once (in the with-prior pass)
            save_prior_diagnostics(out_dir, prior_rows, prior_stats or {})
            if prior_stats:
                print(f"[WARN] {prior_stats.get('reason','')}")
            if getattr(args, "write_corrected_priors", True) and prior_rows:
                payload = write_corrected_set_cube_center(out_dir, prior_rows, set_pose6)
                yaws = [abs(v.get("delta_yaw_deg", 0.0)) for v in payload["sets"].values()]
                print(f"[INFO] wrote corrected_set_cube_center_6dof.json "
                      f"({len(payload['sets'])} sets, median |Δyaw|={np.median(yaws):.1f}deg vs old nominal)")

    def to_base(T_ref_C, T_ref_O):
        return ({ci: T_base_ref @ T for ci, T in T_ref_C.items()},
                {eid: T_base_ref @ T for eid, T in T_ref_O.items()})

    # Outputs/metrics are always in base frame; prior_stats only reported for with-prior.
    ev_prior_stats = prior_stats if with_prior else None

    def ev(method, T_cam, T_obj, diag):
        return evaluate_and_save(
            method, prior_mode, out_dir, pose_obs, corner_obs, T_cam, T_obj,
            fixed_cam_ids, K_map, D_map, event_to_set, set_priors, diag, None,
            corner_obs_reason=corner_obs_reason, prior_stats=ev_prior_stats,
            test_bundle=test_bundle,
        )

    # Vision calibration in cam-ref frame (mean and robust), then mapped to base.
    Tc_mean_ref, _ = build_ref_relative_from_pairwise(pose_obs, fixed_cam_ids, ref_cam, robust=False)
    To_mean_ref = initialize_ref_object_poses(pose_obs, Tc_mean_ref, fixed_cam_ids, ref_cam)
    Tc_rob_ref, _ = build_ref_relative_from_pairwise(pose_obs, fixed_cam_ids, ref_cam, robust=True)
    To_rob_ref = initialize_ref_object_poses(pose_obs, Tc_rob_ref, fixed_cam_ids, ref_cam)
    T_cam_mean, T_obj_mean = to_base(Tc_mean_ref, To_mean_ref)
    T_cam_rob, T_obj_rob = to_base(Tc_rob_ref, To_rob_ref)

    # 1) simple mean, 2) robust average — closed-form vision baselines (no solve to
    #    constrain), so identical for with/without prior. Emitted only once (caller
    #    passes emit_closed_form=False on the with-prior pass) to avoid duplicate rows.
    if emit_closed_form:
        results.append(ev("01_pnp_mean", T_cam_mean, T_obj_mean,
                          {**diag_anchor, "init": "vision_mean_base"}))
        results.append(ev("02_pnp_robust_se3", T_cam_rob, T_obj_rob,
                          {**diag_anchor, "init": "vision_robust_base"}))

    _base_pw_trans = args.prior_weight_trans if weight_trans_override is None else weight_trans_override
    _base_pw_rot = args.prior_weight_rot if weight_rot_override is None else weight_rot_override
    pw_trans = float(_base_pw_trans) if with_prior else 0.0
    pw_rot = float(_base_pw_rot) if with_prior else 0.0

    if with_prior:
        # Solve in cam-ref frame (ref cam fixed = clean gauge), but with the robot
        # cube-center fed in as a strong soft constraint. The prior (base frame) is
        # expressed in cam-ref so the constraint pulls cube poses toward the robot
        # answer; the result is mapped to base afterwards. rotation pull is off by
        # default (--prior_weight_rot 0) and turns on once the server reports good yaw.
        set_priors_ref = {s: inv_T(T_base_ref) @ P for s, P in set_priors.items()}
        T_cam_pose_r, T_obj_pose_r, diag_pose = optimize_pose_consistency(
            pose_obs=pose_obs, fixed_cam_ids=fixed_cam_ids,
            init_T_cam=Tc_rob_ref, init_T_obj=To_rob_ref, ref_cam=ref_cam,
            event_to_set=event_to_set, set_priors=set_priors_ref,
            prior_weight_trans=pw_trans, prior_weight_rot=pw_rot,
        )
        T_cam_pose, T_obj_pose = to_base(T_cam_pose_r, T_obj_pose_r)
        results.append(ev("03_pose_consistency_opt", T_cam_pose, T_obj_pose, diag_pose))
        T_cam_repr_r, T_obj_repr_r, diag_repr = optimize_reprojection(
            corner_obs=corner_obs, pose_obs=pose_obs, fixed_cam_ids=fixed_cam_ids,
            init_T_cam=T_cam_pose_r, init_T_obj=T_obj_pose_r, ref_cam=ref_cam,
            K_map=K_map, D_map=D_map, event_to_set=event_to_set, set_priors=set_priors_ref,
            prior_weight_trans=pw_trans, prior_weight_rot=pw_rot,
            pose_regularizer_weight=float(args.reproj_pose_regularizer_weight),
        )
        T_cam_repr, T_obj_repr = to_base(T_cam_repr_r, T_obj_repr_r)
        results.append(ev("04_direct_reprojection_opt", T_cam_repr, T_obj_repr, diag_repr))
    else:
        # Solve vision-only in cam-ref frame (ref cam fixed = clean gauge); cube poses
        # are free unknowns. Map the result to base only for output/metrics.
        T_cam_pose_r, T_obj_pose_r, diag_pose = optimize_pose_consistency(
            pose_obs=pose_obs, fixed_cam_ids=fixed_cam_ids,
            init_T_cam=Tc_rob_ref, init_T_obj=To_rob_ref, ref_cam=ref_cam,
            event_to_set=event_to_set, set_priors=None,
            prior_weight_trans=0.0, prior_weight_rot=0.0,
        )
        T_cam_pose, T_obj_pose = to_base(T_cam_pose_r, T_obj_pose_r)
        res_pose = ev("03_pose_consistency_opt", T_cam_pose, T_obj_pose, diag_pose)
        results.append(res_pose)
        T_cam_repr_r, T_obj_repr_r, diag_repr = optimize_reprojection(
            corner_obs=corner_obs, pose_obs=pose_obs, fixed_cam_ids=fixed_cam_ids,
            init_T_cam=T_cam_pose_r, init_T_obj=T_obj_pose_r, ref_cam=ref_cam,
            K_map=K_map, D_map=D_map, event_to_set=event_to_set, set_priors=None,
            prior_weight_trans=0.0, prior_weight_rot=0.0,
            pose_regularizer_weight=float(args.reproj_pose_regularizer_weight),
        )
        T_cam_repr, T_obj_repr = to_base(T_cam_repr_r, T_obj_repr_r)
        results.append(ev("04_direct_reprojection_opt", T_cam_repr, T_obj_repr, diag_repr))

        # ③ Camera+FK-correction (시뮬 Exp3): camera-based(=without-prior) 예측에
        #   FK 로 학습한 위치의존 잔차(Ridge)를 held-out 예측에만 후보정. gTc/카메라는
        #   그대로이므로 재투영/pose 지표는 03 과 동일하고 test_prior_t 만 개선된다.
        if test_bundle:
            corr = fk_corrected_heldout(
                T_cam_pose, fixed_cam_ids, pose_obs, event_to_set, set_priors,
                test_bundle, lam=float(getattr(args, "ridge_lambda", 1e-3)))
            if corr is not None:
                results.append(replace(
                    res_pose, method="05_camera_fk_correction",
                    test_prior_trans_rmse_mm=corr["test_prior_trans_rmse_mm"],
                    test_prior_rot_rmse_deg=None, test_n_events=corr["n"]))

    # Stamp the prior weights actually used (for the C3 sweep curve) and tag
    # method names with the suffix so sweep rows remain distinct in the summary.
    for r in results:
        r.prior_weight_trans_used = pw_trans
        r.prior_weight_rot_used = pw_rot
        if method_suffix:
            r.method = f"{r.method}{method_suffix}"
    return results


def write_summary(out_dir: str, results: List[MethodResult]) -> None:
    ensure_dir(out_dir)
    rows = [asdict(r) for r in results]
    with open(os.path.join(out_dir, "ablation_summary.json"), "w") as f:
        json.dump(rows, f, indent=2, ensure_ascii=False)
    fields = list(rows[0].keys()) if rows else []
    with open(os.path.join(out_dir, "ablation_summary.csv"), "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def write_sweep_curve(out_dir: str, results: List[MethodResult]) -> None:
    """Emit prior_weight_sweep.csv: one row per (method, weight) for plotting the
    C3 weight-vs-error curve. Only with-prior rows carry a swept weight."""
    ensure_dir(out_dir)
    rows = []
    for r in results:
        if r.prior_mode != "with_robot_cube_prior":
            continue
        # base method name without the __w... suffix, for grouping in a plot
        base_method = r.method.split("__w")[0]
        rows.append({
            "base_method": base_method,
            "method": r.method,
            "prior_weight_trans": r.prior_weight_trans_used,
            "prior_weight_rot": r.prior_weight_rot_used,
            "reproj_rmse_px": r.reproj_rmse_px,
            "reproj_median_px": r.reproj_median_px,
            "pose_trans_rmse_mm": r.pose_trans_rmse_mm,
            "pose_rot_rmse_deg": r.pose_rot_rmse_deg,
            "prior_trans_rmse_mm": r.prior_trans_rmse_mm,
            "prior_rot_rmse_deg": r.prior_rot_rmse_deg,
            "test_prior_trans_rmse_mm": r.test_prior_trans_rmse_mm,
            "test_prior_rot_rmse_deg": r.test_prior_rot_rmse_deg,
            "test_reproj_rmse_px": r.test_reproj_rmse_px,
            "train_sets": r.train_sets,
            "test_sets": r.test_sets,
            "optimizer_accepted": r.optimizer_accepted,
        })
    if not rows:
        return
    path = os.path.join(out_dir, "prior_weight_sweep.csv")
    with open(path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)
    print(f"[C3] wrote prior-weight sweep curve: {path} ({len(rows)} rows)")


def print_summary(results: List[MethodResult]) -> None:
    def fmt(v: Optional[float], nd: int = 3) -> str:
        return "NA" if v is None else f"{v:.{nd}f}"

    def acc(r: MethodResult) -> str:
        if r.optimizer_accepted is None:
            return "-"
        return "yes" if r.optimizer_accepted else "NO(fb)"

    has_test = any(r.test_prior_trans_rmse_mm is not None or r.test_n_events for r in results)

    print("\n" + "=" * 108)
    print("ABLATION SUMMARY" + ("  (+ held-out test columns)" if has_test else ""))
    print("=" * 108)
    header = (f"{'method':28s} {'prior':14s} {'reprj_rmse':>10s} {'reprj_med':>9s} "
              f"{'pose_t':>8s} {'pose_r':>7s} {'prior_t':>8s} {'opt_acc':>7s}")
    if has_test:
        header += f" {'|test_prior_t':>13s} {'test_reproj':>11s}"
    print(header)
    print("-" * len(header))
    for r in results:
        pm = "WITH" if r.prior_mode == "with_robot_cube_prior" else "no"
        line = (f"{r.method:28s} {pm:14s} {fmt(r.reproj_rmse_px,3):>10s} {fmt(r.reproj_median_px,3):>9s} "
                f"{fmt(r.pose_trans_rmse_mm,2):>8s} {fmt(r.pose_rot_rmse_deg,2):>7s} "
                f"{fmt(r.prior_trans_rmse_mm,2):>8s} {acc(r):>7s}")
        if has_test:
            line += f" {fmt(r.test_prior_trans_rmse_mm,2):>13s} {fmt(r.test_reproj_rmse_px,3):>11s}"
        print(line)
    if has_test:
        print("\n[C3] test_prior_t = FK position RMSE (mm) on HELD-OUT sets — the fair "
              "with-prior vs without-prior metric. test_reproj = reprojection RMSE (px) on held-out sets.")
        print("[C3] 시뮬 Exp3 3방식 짝: without-prior=Camera-based, with-prior=FK-based, "
              "05_camera_fk_correction=Camera+FK후보정(Ridge). test_prior_t 로 세 방식을 비교.")


def main() -> None:
    parser = argparse.ArgumentParser(description="Refactored calibration ablation runner")
    parser.add_argument("--root_folder", required=True)
    parser.add_argument("--intrinsics_dir", required=True)
    parser.add_argument("--out_dir", default=None)
    parser.add_argument("--gripper_cam_idx", type=int, default=None)
    parser.add_argument("--ref_fixed_cam_idx", type=int, default=None)
    parser.add_argument("--cube_config_json", type=str, default=None)
    parser.add_argument("--max_err_fixed", type=float, default=3.0)
    parser.add_argument("--max_err_gripper", type=float, default=5.0)
    parser.add_argument("--fixed_cube_min_aspect", type=float, default=0.0)
    parser.add_argument("--gripper_cube_min_aspect", type=float, default=0.35)
    parser.add_argument("--gripper_cube_min_markers", type=int, default=1)
    parser.add_argument("--reproj_pose_regularizer_weight", type=float, default=2.0,
                        help="Soft pose regularizer used during direct reprojection optimization.")
    # --- robot cube prior controls (problem 1) ---
    # The set_cube_center_6dof rotation is unreliable, so rotation weight defaults to 0.
    parser.add_argument("--prior_weight_trans", type=float, default=30.0,
                        help="Strong-soft weight pulling cube-center TRANSLATION to the robot answer in the with-prior solve.")
    parser.add_argument("--prior_weight_rot", type=float, default=0.0,
                        help="Soft weight on cube ROTATION prior. Default 0 (rotation unreliable now); set >0 for the "
                             "next capture, where the server reports a good per-set yaw, to use the full 6-DoF prior.")
    parser.add_argument("--prior_max_trans_error_mm", type=float, default=100.0,
                        help="Reject a prior set whose position residual to the Kabsch anchor exceeds this.")
    parser.add_argument("--prior_max_rot_error_deg", type=float, default=45.0,
                        help="If median prior-vs-vision rotation error exceeds this, rotation prior is rejected.")
    parser.add_argument("--disable_prior_if_inconsistent", type=lambda s: str(s).lower() not in ("0", "false", "no"),
                        default=True,
                        help="Exclude position-outlier prior sets from the base-frame anchor.")
    parser.add_argument("--write_corrected_priors", type=lambda s: str(s).lower() not in ("0", "false", "no"),
                        default=True,
                        help="Back-write a corrected set_cube_center_6dof (vision yaw, base frame) for next-run priors.")
    # --- C3 prior-weight sweep ---
    # 큐브중점 prior 의 soft-constraint 세기를 여러 값으로 쓸어 weight-vs-error 곡선을 만든다.
    # 예) --prior_weight_sweep 0,1,10,30,100  (0 은 without-prior 와 동치)
    parser.add_argument("--prior_weight_sweep", type=str, default="",
                        help="C3 sweep: 콤마로 구분한 prior weight 목록. 지정 시 with-prior suite 를 "
                             "각 weight 로 반복 실행하고 prior_weight_sweep.csv 를 추가로 남긴다. "
                             "비우면(기본) 기존처럼 --prior_weight_trans 단일 실행.")
    parser.add_argument("--sweep_axis", type=str, default="trans",
                        choices=["trans", "rot", "both"],
                        help="sweep 값이 구동할 prior weight 축: trans(기본)/rot/both. "
                             "rot 은 서버 yaw 가 신뢰될 때의 회전 prior 기여도를 본다.")
    # --- C3 held-out (train/test) split ---
    # prior 를 쓴 쪽은 학습에 쓴 FK 로 평가하면 자명히 유리하므로, 카메라는 train set 으로만
    # 맞추고 test set(학습에 안 쓴 배치)에서 FK 위치오차/재투영을 잰다. 공정 비교의 핵심.
    parser.add_argument("--test_sets", type=str, default="",
                        help="held-out 으로 뺄 set_index 목록(콤마 구분). 지정 시 --holdout_frac 무시.")
    parser.add_argument("--holdout_frac", type=float, default=0.0,
                        help="test 로 뺄 set 비율(0~1). 0(기본)이면 split 없이 전체로 fit·평가(기존 동작). "
                             "--test_sets 가 있으면 무시.")
    parser.add_argument("--split_seed", type=int, default=0,
                        help="--holdout_frac 무작위 분할 시드(재현성). 정렬된 set 을 이 시드로 섞어 뒤쪽을 test 로.")
    parser.add_argument("--ridge_lambda", type=float, default=1e-3,
                        help="Camera+FK-correction(05) 잔차보정 Ridge 정규화 세기 (시뮬 lam 기본 1e-3).")
    args = parser.parse_args()

    def _parse_sweep(s):
        vals = []
        for tok in str(s or "").replace(";", ",").split(","):
            tok = tok.strip()
            if not tok:
                continue
            try:
                vals.append(float(tok))
            except ValueError:
                raise SystemExit(f"[ERROR] invalid --prior_weight_sweep value: {tok!r}")
        return vals

    root = args.root_folder
    out_dir = ensure_dir(args.out_dir or os.path.join("CP_result", "C3"))
    with open(os.path.join(root, "meta.json"), "r") as f:
        meta = json.load(f)

    cfg, cfg_source = resolve_cube_config_for_run(
        root_folder=root,
        calib_dir=out_dir,
        cube_config_json=args.cube_config_json,
        default_cfg=get_default_cube_config(),
    )
    meta_cfg, _ = load_cube_config_from_meta(root, default_cfg=cfg)
    reuse_stored = cube_configs_equivalent(meta_cfg, cfg)
    cube = AprilTagCubeTarget(cfg)

    all_cam_ids = sorted({
        int(k) for cap in meta.get("captures", [])
        for k, v in cap.get("cams", {}).items() if v.get("saved")
    })
    if not all_cam_ids:
        raise RuntimeError("No saved cameras found in meta.json")

    gripper_cam_idx = args.gripper_cam_idx
    if gripper_cam_idx is None:
        gripper_cam_idx = meta.get("gripper_cam_idx")
    if gripper_cam_idx is None:
        dm = os.path.join(args.intrinsics_dir, "device_map.json")
        if os.path.exists(dm):
            with open(dm, "r") as f:
                gripper_cam_idx = json.load(f).get("gripper_cam_idx")
    if gripper_cam_idx is None:
        raise RuntimeError("gripper_cam_idx is required or must exist in meta/device_map.json")

    fixed_cam_ids = [ci for ci in all_cam_ids if ci != int(gripper_cam_idx)]
    if len(fixed_cam_ids) < 2:
        raise RuntimeError("Need at least two fixed cameras for this comparison")
    ref_cam = args.ref_fixed_cam_idx if args.ref_fixed_cam_idx is not None else fixed_cam_ids[0]
    if ref_cam not in fixed_cam_ids:
        raise RuntimeError(f"ref_fixed_cam_idx cam{ref_cam} is not in fixed cams: {fixed_cam_ids}")

    K_map, D_map = {}, {}
    for ci in all_cam_ids:
        K_map[ci], D_map[ci], _ = load_intrinsics_with_depth_scale(args.intrinsics_dir, ci)

    event_to_set: Dict[int, Optional[int]] = {}
    for cap in meta.get("captures", []):
        eid = int(cap.get("event_id", -1))
        if eid >= 0:
            sidx = get_capture_set_index(cap)
            event_to_set[eid] = int(sidx) if sidx is not None else None

    set_priors = load_nominal_set_cube_transforms(meta)
    set_pose6 = load_nominal_set_cube_pose6(meta)

    print(f"[INFO] cube config source: {cfg_source}")
    print(f"[INFO] all cams={all_cam_ids}, fixed={fixed_cam_ids}, gripper=cam{gripper_cam_idx}, ref=cam{ref_cam}")
    print(f"[INFO] stored cube candidates reused: {reuse_stored}")
    print(f"[INFO] robot cube priors: {len(set_priors)} sets")

    pose_obs = load_pose_observations(
        root=root,
        meta=meta,
        cube=cube,
        K_map=K_map,
        D_map=D_map,
        all_cam_ids=all_cam_ids,
        gripper_cam_idx=int(gripper_cam_idx),
        reuse_stored_cube_candidates=reuse_stored,
        max_err_fixed=float(args.max_err_fixed),
        max_err_gripper=float(args.max_err_gripper),
        min_aspect_fixed=float(args.fixed_cube_min_aspect),
        min_aspect_gripper=float(args.gripper_cube_min_aspect),
        gripper_min_markers=int(args.gripper_cube_min_markers),
    )
    # This comparison is about fixed multi-camera calibration. Gripper observations are kept out of metrics.
    fixed_pose_obs = [o for o in pose_obs if o.cam in fixed_cam_ids]

    corner_obs, corner_reason = detect_corner_observations(
        root=root,
        meta=meta,
        cube=cube,
        K_map=K_map,
        D_map=D_map,
        all_cam_ids=fixed_cam_ids,
        gripper_cam_idx=int(gripper_cam_idx),
        max_err_fixed=float(args.max_err_fixed),
        max_err_gripper=float(args.max_err_gripper),
        min_aspect_fixed=float(args.fixed_cube_min_aspect),
        min_aspect_gripper=float(args.gripper_cube_min_aspect),
    )

    print(f"[INFO] Loaded {len(fixed_pose_obs)} pose observations")
    print(f"[INFO] Loaded {len(corner_obs)} corner observations")
    if len(corner_obs) == 0:
        print(f"[WARN] Direct reprojection optimization skipped: {corner_reason}")
    else:
        print("[INFO] Direct reprojection optimization (04) will run on real marker corners.")

    # ── C3 held-out (train/test) split ────────────────────────────────────────
    def _obs_set(o):
        return o.set_idx if getattr(o, "set_idx", None) is not None else event_to_set.get(int(o.event))

    available_sets = sorted(int(s) for s in set_priors.keys())
    test_set_ids: List[int] = []
    if str(args.test_sets).strip():
        want = {int(t) for t in str(args.test_sets).replace(";", ",").split(",") if t.strip() != ""}
        test_set_ids = sorted(want & set(available_sets))
        missing = sorted(want - set(available_sets))
        if missing:
            print(f"[WARN] --test_sets {missing} not in available sets {available_sets}; ignored")
    elif float(args.holdout_frac) > 0.0:
        import random as _random
        n_test = max(1, int(round(len(available_sets) * float(args.holdout_frac))))
        n_test = min(n_test, max(0, len(available_sets) - 1))  # keep >=1 train set
        shuffled = list(available_sets)
        _random.Random(int(args.split_seed)).shuffle(shuffled)
        test_set_ids = sorted(shuffled[len(shuffled) - n_test:]) if n_test > 0 else []

    fit_pose_obs, fit_corner_obs, fit_priors = fixed_pose_obs, corner_obs, set_priors
    test_bundle = None
    if test_set_ids:
        test_set = set(test_set_ids)
        train_set_ids = [s for s in available_sets if s not in test_set]
        if not train_set_ids:
            raise RuntimeError(f"train set empty after removing test_sets={test_set_ids}; "
                               f"available={available_sets}")
        fit_pose_obs = [o for o in fixed_pose_obs if _obs_set(o) not in test_set]
        fit_corner_obs = [o for o in corner_obs if _obs_set(o) not in test_set]
        fit_priors = {s: T for s, T in set_priors.items() if int(s) not in test_set}
        test_pose_obs = [o for o in fixed_pose_obs if _obs_set(o) in test_set]
        test_corner_obs = [o for o in corner_obs if _obs_set(o) in test_set]
        test_priors = {s: T for s, T in set_priors.items() if int(s) in test_set}
        test_bundle = {
            "pose_obs": test_pose_obs,
            "corner_obs": test_corner_obs,
            "event_to_set": event_to_set,
            "set_priors": test_priors,
            "train_sets_str": ",".join(str(s) for s in train_set_ids),
            "test_sets_str": ",".join(str(s) for s in test_set_ids),
        }
        print(f"[C3] train/test split: train={train_set_ids} test={test_set_ids} "
              f"(fit pose_obs {len(fit_pose_obs)}/{len(fixed_pose_obs)}, "
              f"test pose_obs {len(test_pose_obs)})")
    else:
        print("[C3] no train/test split (fit and evaluate on all sets; "
              "test metrics will be NA). Use --test_sets or --holdout_frac for a fair C3 comparison.")

    # NOTE (C3 scope): this ablation measures the cube-center prior contribution for
    # the FIXED (eye-to-hand) cameras only — the moving gripper (eye-in-hand) camera
    # is intentionally excluded because its base extrinsic is not constant and cannot
    # enter this static ref-relative solve. The analogous hand-eye/FK-prior ablation
    # for the gripper camera lives in the joint solver (CP_Step3_joint_unified.py).
    sweep_weights = _parse_sweep(args.prior_weight_sweep)

    results: List[MethodResult] = []
    # without-prior pass: emits the closed-form baselines (01/02) exactly once.
    # Cameras are fit on TRAIN sets only (fit_*); held-out metrics use test_bundle.
    results.extend(run_method_suite(
        fit_pose_obs, fit_corner_obs, fixed_cam_ids, ref_cam, K_map, D_map,
        event_to_set, fit_priors, set_pose6, out_dir, with_prior=False,
        corner_obs_reason=corner_reason, args=args, emit_closed_form=True,
        test_bundle=test_bundle,
    ))

    if sweep_weights:
        print(f"[C3] prior-weight sweep on axis '{args.sweep_axis}': {sweep_weights}")
        for w in sweep_weights:
            wt = w if args.sweep_axis in ("trans", "both") else float(args.prior_weight_trans)
            wr = w if args.sweep_axis in ("rot", "both") else float(args.prior_weight_rot)
            suffix = f"__w{args.sweep_axis}{w:g}"
            results.extend(run_method_suite(
                fit_pose_obs, fit_corner_obs, fixed_cam_ids, ref_cam, K_map, D_map,
                event_to_set, fit_priors, set_pose6, out_dir, with_prior=True,
                corner_obs_reason=corner_reason, args=args,
                weight_trans_override=wt, weight_rot_override=wr,
                emit_closed_form=False, method_suffix=suffix,
                test_bundle=test_bundle,
            ))
    else:
        # single with-prior pass (legacy behavior); no duplicate closed-form rows.
        results.extend(run_method_suite(
            fit_pose_obs, fit_corner_obs, fixed_cam_ids, ref_cam, K_map, D_map,
            event_to_set, fit_priors, set_pose6, out_dir, with_prior=True,
            corner_obs_reason=corner_reason, args=args, emit_closed_form=False,
            test_bundle=test_bundle,
        ))

    write_summary(out_dir, results)
    if sweep_weights:
        write_sweep_curve(out_dir, results)
    print_summary(results)
    # surface optimizer fallbacks for problem 3
    for r in results:
        if r.optimizer_accepted is False:
            print(f"[INFO] {r.method} ({'WITH' if 'with' in r.prior_mode else 'no'} prior) "
                  f"optimization rejected: {r.optimizer_fallback_reason} "
                  f"(cost {r.optimizer_cost_initial:.4g} -> {r.optimizer_cost_final:.4g}); kept initializer")
    print(f"\n[DONE] summary: {os.path.join(out_dir, 'ablation_summary.csv')}")


if __name__ == "__main__":
    main()
