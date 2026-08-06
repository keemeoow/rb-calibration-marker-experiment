#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
CP_common.py — CP_C1/C2/C3 비교실험이 공유하는 로더·기하·지표 유틸.

캡처 세션(meta.json + 이미지)에서 관측을 읽고, SE(3) 기하/정합/지표를 계산하는
실험-무관 하위 계층. C1/C2/C3 진입 파일과 보조 진단 스크립트가 모두 이 모듈을 쓴다.
(과거에는 CP_Step3_compare_calibrartion.py 안에 함께 있었으나 여기로 물리 분리했다.)
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
from collections import defaultdict
from dataclasses import dataclass, asdict
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


def ensure_dir(path: str) -> str:
    os.makedirs(path, exist_ok=True)
    return path


def make_T(rotvec: np.ndarray, t: np.ndarray) -> np.ndarray:
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = R.from_rotvec(np.asarray(rotvec, dtype=np.float64)).as_matrix()
    T[:3, 3] = np.asarray(t, dtype=np.float64).reshape(3)
    return T


def T_to_vec(T: np.ndarray) -> np.ndarray:
    v = np.zeros(6, dtype=np.float64)
    v[:3] = np.asarray(T[:3, 3], dtype=np.float64)
    v[3:] = R.from_matrix(T[:3, :3]).as_rotvec()
    return v


def vec_to_T(v: np.ndarray) -> np.ndarray:
    v = np.asarray(v, dtype=np.float64).reshape(6)
    return make_T(v[3:], v[:3])


def se3_log_residual(T_err: np.ndarray, rot_scale_m_per_rad: float = 0.05) -> np.ndarray:
    """Return residual in meters: [dx,dy,dz, scaled_rotvec]."""
    r = np.zeros(6, dtype=np.float64)
    r[:3] = T_err[:3, 3]
    r[3:] = R.from_matrix(T_err[:3, :3]).as_rotvec() * float(rot_scale_m_per_rad)
    return r


def weighted_se3_average(T_list: List[np.ndarray], weights: Optional[List[float]] = None) -> np.ndarray:
    if not T_list:
        raise ValueError("weighted_se3_average got an empty T_list")
    if weights is None:
        w = np.ones(len(T_list), dtype=np.float64)
    else:
        w = np.maximum(np.asarray(weights, dtype=np.float64), 1e-12)
    w = w / (w.sum() + 1e-12)

    t = np.sum(np.stack([T[:3, 3] for T in T_list], axis=0) * w[:, None], axis=0)
    M = np.sum(np.stack([T[:3, :3] for T in T_list], axis=0) * w[:, None, None], axis=0)
    U, _, Vt = np.linalg.svd(M)
    Rm = U @ Vt
    if np.linalg.det(Rm) < 0:
        U[:, -1] *= -1.0
        Rm = U @ Vt
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = Rm
    T[:3, 3] = t
    return T


def robust_se3_average(
    T_list: List[np.ndarray],
    weights: Optional[List[float]] = None,
    max_iters: int = 5,
    k_mad: float = 2.5,
) -> Tuple[np.ndarray, Dict[str, float]]:
    if not T_list:
        raise ValueError("robust_se3_average got an empty T_list")
    if weights is None:
        weights = [1.0] * len(T_list)
    inliers = np.arange(len(T_list), dtype=int)
    T_avg = weighted_se3_average(T_list, weights)

    for _ in range(max_iters):
        errs = []
        for idx in inliers:
            e = se3_log_residual(inv_T(T_avg) @ T_list[idx])
            errs.append(float(np.linalg.norm(e[:3]) * 1000.0 + np.linalg.norm(e[3:]) * 1000.0))
        errs = np.asarray(errs, dtype=np.float64)
        med = float(np.median(errs))
        mad = float(np.median(np.abs(errs - med)) + 1e-12)
        thr = med + k_mad * 1.4826 * mad
        keep_local = errs <= thr
        if keep_local.sum() < max(3, int(0.4 * len(inliers))):
            break
        new_inliers = inliers[keep_local]
        if len(new_inliers) == len(inliers):
            break
        inliers = new_inliers
        T_avg = weighted_se3_average([T_list[i] for i in inliers], [weights[i] for i in inliers])

    T_avg = weighted_se3_average([T_list[i] for i in inliers], [weights[i] for i in inliers])
    trans = [float(np.linalg.norm(T[:3, 3] - T_avg[:3, 3]) * 1000.0) for T in [T_list[i] for i in inliers]]
    rot = [rotation_error_deg(T[:3, :3], T_avg[:3, :3]) for T in [T_list[i] for i in inliers]]
    return T_avg, {
        "num_total": int(len(T_list)),
        "num_inliers": int(len(inliers)),
        "inlier_ratio": float(len(inliers) / max(1, len(T_list))),
        "translation_std_mm": float(np.std(trans)) if trans else 0.0,
        "rotation_std_deg": float(np.std(rot)) if rot else 0.0,
    }


@dataclass
class PoseObs:
    cam: int
    event: int
    set_idx: Optional[int]
    T_C_O: np.ndarray
    err_px: float
    n_points: int
    source: str


@dataclass
class CornerObs:
    cam: int
    event: int
    set_idx: Optional[int]
    object_points: np.ndarray  # Nx3, cube/object frame
    image_points: np.ndarray   # Nx2
    err_hint_px: float


def try_parse_pose6(obj: Any) -> Optional[List[float]]:
    if obj is None:
        return None
    if isinstance(obj, list) and len(obj) == 6:
        try:
            return [float(x) for x in obj]
        except Exception:
            return None
    if isinstance(obj, dict):
        if all(k in obj for k in ["x", "y", "z", "rz", "ry", "rx"]):
            return [float(obj["x"]), float(obj["y"]), float(obj["z"]), float(obj["rz"]), float(obj["ry"]), float(obj["rx"])]
        for key in ["robot_pose_6dof", "tcp_pose_6dof", "pose_6dof", "pose"]:
            out = try_parse_pose6(obj.get(key))
            if out is not None:
                return out
    return None


# Pinned isotropic scale applied to every robot-reported TRANSLATION at load
# time.  k is a *processing* multiplier, not robot behaviour: the controller
# writes identical robot_pose_6dof values for every run, so two results that
# used different k are not comparable even though the raw data is identical.
# The shift is large (~12mm mean on the FK cube poses between 1.0 and 1.0229)
# and no downstream metric reveals it, which is exactly how one stored table
# ended up mixing both values.
#
# k is therefore pinned here instead of read per-run from the environment.  It
# stays at 1.0 — robot values untouched — until the physical dial-gauge
# measurement settles the true factor.  Vision and depth both put it near
# 1.023, but the per-camera spread is 1.014-1.029, so adopting it now would
# bake a provisional constant into every stored number.
#
# To adopt a measured value: change this one constant and regenerate every
# stored result together.  Never mix.
ROBOT_POS_SCALE_PINNED: float = 1.0


def robot_pos_scale() -> float:
    """Isotropic correction applied to every robot-reported TRANSLATION (not rotation).

    The robot under-reports Cartesian distances by ~2.3% on this arm (a kinematic
    scale error confirmed independently by both marker-vision and depth — see
    fk_scale_crosscheck / estimate_robot_pos_scale).  Correcting it is a one-line
    change to ``ROBOT_POS_SCALE_PINNED`` above, not a per-run environment knob:
    RB_ROBOT_POS_SCALE now only survives as a guard that refuses to run when it
    disagrees with the pin, so a stale shell cannot silently produce results at a
    second scale.
    """
    requested = os.environ.get("RB_ROBOT_POS_SCALE")
    if requested is not None:
        try:
            value = float(requested)
        except (TypeError, ValueError):
            raise RuntimeError(
                f"RB_ROBOT_POS_SCALE={requested!r} is not a number") from None
        if value != ROBOT_POS_SCALE_PINNED:
            raise RuntimeError(
                f"RB_ROBOT_POS_SCALE={value} conflicts with "
                f"CP_common.ROBOT_POS_SCALE_PINNED={ROBOT_POS_SCALE_PINNED}. "
                "Per-run overrides are refused because results produced at "
                "different scales are silently incomparable. Change the pinned "
                "constant and regenerate every stored result together.")
    return ROBOT_POS_SCALE_PINNED


def pose6_to_T_base_gripper(pose6: List[float]) -> np.ndarray:
    # Project convention: robot 6-DoF pose is [x,y,z (mm), rz,ry,rx (deg)] and
    # euler_deg_to_matrix returns the full 4x4 with translation in meters.
    T = euler_deg_to_matrix(*[float(v) for v in pose6])
    s = robot_pos_scale()
    if s != 1.0:
        T = np.array(T, dtype=np.float64, copy=True)
        T[:3, 3] *= s   # rotation is correct (joint angles); only the length scale is off
    return T


def T_to_pose6_mm(T: np.ndarray) -> List[float]:
    """Inverse of pose6_to_T_base_gripper: 4x4 (m) -> [x,y,z mm, rz,ry,rx deg].

    Matches euler_deg_to_matrix's R = Rz@Ry@Rx (intrinsic ZYX) convention so the
    written value round-trips through the existing capture/calibration code.
    """
    rz, ry, rx = R.from_matrix(np.asarray(T[:3, :3], dtype=np.float64)).as_euler("ZYX", degrees=True)
    t = np.asarray(T[:3, 3], dtype=np.float64) * 1000.0
    return [float(t[0]), float(t[1]), float(t[2]), float(rz), float(ry), float(rx)]


def load_nominal_set_cube_transforms(meta: Dict[str, Any]) -> Dict[int, np.ndarray]:
    priors: Dict[int, np.ndarray] = {}
    for cap in meta.get("captures", []):
        sidx = get_capture_set_index(cap)
        if sidx is None or sidx in priors:
            continue
        raw = get_capture_set_cube_center_transform_raw(cap)
        pose = try_parse_pose6(raw)
        if pose is None:
            pose = try_parse_pose6(cap.get("set_cube_center_6dof"))
        if pose is not None:
            priors[int(sidx)] = pose6_to_T_base_gripper(pose)
    return priors


def load_robot_poses_from_meta(meta: Dict[str, Any]) -> Dict[int, np.ndarray]:
    robot_T: Dict[int, np.ndarray] = {}
    for cap in meta.get("captures", []):
        eid = int(cap.get("event_id", -1))
        if eid < 0:
            continue
        pose = None
        for key in ["robot_pose_6dof", "tcp_pose_6dof", "pose_6dof", "robot_pose"]:
            pose = try_parse_pose6(cap.get(key))
            if pose is not None:
                break
        if pose is not None:
            robot_T[eid] = pose6_to_T_base_gripper(pose)
    return robot_T


def marker_aspect_ratio(img_pts: np.ndarray) -> float:
    pts = np.asarray(img_pts, dtype=np.float64).reshape(4, 2)
    lens = [np.linalg.norm(pts[(i + 1) % 4] - pts[i]) for i in range(4)]
    return float(min(lens) / max(max(lens), 1e-12))


def stored_cube_pose_candidates(
    cinfo: Dict[str, Any],
    cam_idx: int,
    gripper_cam_idx: Optional[int],
    max_err: float,
    min_markers: int,
    min_aspect: float,
) -> List[Dict[str, Any]]:
    candidates: List[Dict[str, Any]] = []
    aspect_by_marker: Dict[int, float] = {}

    for item in cinfo.get("markers", []):
        mid = int(item.get("marker_id", -1))
        corners = np.asarray(item.get("corners_2d", []), dtype=np.float64)
        aspect = None
        if corners.shape == (4, 2):
            aspect = marker_aspect_ratio(corners)
            aspect_by_marker[mid] = aspect

        for cand in item.get("pose_candidates") or []:
            err = float(cand.get("reproj_error_mean_px", 99.0))
            T44 = cand.get("T_cam_cube_4x4")
            if T44 is None or err > max_err:
                continue
            if aspect is not None and aspect < min_aspect:
                continue
            candidates.append({
                "T_C_O": np.asarray(T44, dtype=np.float64),
                "err_mean": err,
                "n_points": 4,
                "used_ids": [mid],
                "source": "stored_ippe",
                **copy_depth_fields(cand),
            })

    cpnp = cinfo.get("cube_pnp")
    if cpnp and cpnp.get("ok"):
        err = float(cpnp.get("reproj_mean_px", 99.0))
        used_ids = [int(x) for x in cpnp.get("used_ids", [])]
        T44 = cpnp.get("T_cam_cube_4x4")
        if T44 is not None and err <= max_err and len(set(used_ids)) >= max(1, int(min_markers)):
            aspects = [aspect_by_marker[mid] for mid in used_ids if mid in aspect_by_marker]
            if not aspects or min(aspects) >= min_aspect:
                candidates.append({
                    "T_C_O": np.asarray(T44, dtype=np.float64),
                    "err_mean": err,
                    "n_points": int(cpnp.get("n_points", 4 * max(1, len(set(used_ids))))),
                    "used_ids": used_ids,
                    "source": "stored_cube_pnp",
                    **copy_depth_fields(cpnp),
                })
    return candidates


def get_marker_object_corners(cube: AprilTagCubeTarget, marker_id: int) -> Optional[np.ndarray]:
    """Adapter for project-specific cube model APIs.

    Expected output order must match cube.model.reorder_image_corners(marker_id, corners).
    Add one branch here if your model exposes a different method/field name.
    """
    model = cube.model
    mid = int(marker_id)

    method_names = [
        "marker_corners_in_rig",  # this project's AprilTagCubeModel accessor (paired with reorder_image_corners)
        "get_marker_object_corners",
        "marker_object_corners",
        "get_marker_corners_3d",
        "marker_corners_3d",
        "object_corners",
        "corners_3d",
    ]
    for name in method_names:
        fn = getattr(model, name, None)
        if callable(fn):
            try:
                pts = np.asarray(fn(mid), dtype=np.float64)
                if pts.shape == (4, 3):
                    return pts
            except TypeError:
                pass
            except Exception:
                pass

    field_names = [
        "marker_corners_obj",
        "marker_corners_3d",
        "object_points_by_id",
        "corners_by_marker",
        "markers",
    ]
    for name in field_names:
        data = getattr(model, name, None)
        if isinstance(data, dict) and mid in data:
            val = data[mid]
            if isinstance(val, dict):
                for key in ["corners_3d", "object_points", "obj_pts", "points"]:
                    if key in val:
                        pts = np.asarray(val[key], dtype=np.float64)
                        if pts.shape == (4, 3):
                            return pts
            else:
                pts = np.asarray(val, dtype=np.float64)
                if pts.shape == (4, 3):
                    return pts
    return None


def detect_corner_observations(
    root: str,
    meta: Dict[str, Any],
    cube: AprilTagCubeTarget,
    K_map: Dict[int, np.ndarray],
    D_map: Dict[int, np.ndarray],
    all_cam_ids: List[int],
    gripper_cam_idx: int,
    max_err_fixed: float,
    max_err_gripper: float,
    min_aspect_fixed: float,
    min_aspect_gripper: float,
    exclude_gripped: bool = False,
    image_scale: float = 1.0,
) -> Tuple[List[CornerObs], str]:
    """Return (corner observations, reason-string-if-empty).

    The reason string makes problem 2 debuggable: it distinguishes "no images/
    detections were loaded" from "cube model 3D marker corners are unavailable".
    exclude_gripped: see load_pose_observations().
    """
    image_scale = float(image_scale)
    if not np.isfinite(image_scale) or image_scale <= 0.0:
        raise ValueError("image_scale must be finite and positive")
    obs: List[CornerObs] = []
    # counters for an actionable zero-observations reason
    n_imgs_read = n_imgs_missing = 0
    n_detections = n_obj_corner_fail = n_aspect_reject = 0
    for cap in meta.get("captures", []):
        eid = int(cap.get("event_id", -1))
        if eid < 0:
            continue
        if exclude_gripped and cap.get("cube_gripped"):
            continue
        sidx = get_capture_set_index(cap)
        for ci_str, cinfo in cap.get("cams", {}).items():
            ci = int(ci_str)
            if ci not in all_cam_ids or not cinfo.get("saved"):
                continue
            rgb_rel = cinfo.get("rgb_path", "")
            if not rgb_rel:
                n_imgs_missing += 1
                continue
            img = cv2.imread(os.path.join(root, rgb_rel))
            if img is None:
                n_imgs_missing += 1
                continue
            if image_scale != 1.0:
                interpolation = (cv2.INTER_AREA if image_scale < 1.0
                                 else cv2.INTER_CUBIC)
                img = cv2.resize(
                    img, None, fx=image_scale, fy=image_scale,
                    interpolation=interpolation)
            n_imgs_read += 1
            try:
                corners_list, ids = cube.detect(img)
            except Exception:
                continue
            if ids is None:
                continue
            obj_all, img_all = [], []
            min_aspect = min_aspect_gripper if ci == gripper_cam_idx else min_aspect_fixed
            for corners, mid_raw in zip(corners_list, ids):
                mid = int(np.asarray(mid_raw).reshape(-1)[0])
                if not cube.model.has_marker(mid):
                    continue
                n_detections += 1
                # Detection happens at the requested raster scale, but the
                # returned coordinates are mapped back to the native pixel
                # frame.  This keeps K, robust-loss thresholds, and held-out
                # RMSE units identical across resolution conditions.
                img_pts_raw = (
                    np.asarray(corners, dtype=np.float64).reshape(4, 2)
                    / image_scale)
                try:
                    img_pts = np.asarray(cube.model.reorder_image_corners(mid, img_pts_raw), dtype=np.float64).reshape(4, 2)
                except Exception:
                    img_pts = img_pts_raw
                if marker_aspect_ratio(img_pts) < min_aspect:
                    n_aspect_reject += 1
                    continue
                obj_pts = get_marker_object_corners(cube, mid)
                if obj_pts is None:
                    n_obj_corner_fail += 1
                    continue
                obj_all.append(obj_pts)
                img_all.append(img_pts)
            if obj_all:
                obs.append(CornerObs(
                    cam=ci,
                    event=eid,
                    set_idx=int(sidx) if sidx is not None else None,
                    object_points=np.concatenate(obj_all, axis=0),
                    image_points=np.concatenate(img_all, axis=0),
                    err_hint_px=max_err_gripper if ci == gripper_cam_idx else max_err_fixed,
                ))

    reason = ""
    if not obs:
        if n_imgs_read == 0:
            reason = (f"0 corner observations because no images could be read "
                      f"({n_imgs_missing} missing/unreadable rgb paths)")
        elif n_detections == 0:
            reason = "0 corner observations because no AprilTag markers were detected in any image"
        elif n_obj_corner_fail >= max(1, n_detections - n_aspect_reject):
            reason = ("0 corner observations because cube model 3D marker corners were unavailable "
                      "(adapt get_marker_object_corners() to your AprilTagCubeTarget model)")
        else:
            reason = (f"0 corner observations after filtering: {n_aspect_reject} aspect-rejected, "
                      f"{n_obj_corner_fail}/{n_detections} missing object corners")
    return obs, reason


def estimate_image_cube_pose(
    cube: AprilTagCubeTarget,
    img: np.ndarray,
    K: np.ndarray,
    D: np.ndarray,
    max_err: float,
    min_markers: int,
    min_aspect: float,
) -> List[Dict[str, Any]]:
    candidates: List[Dict[str, Any]] = []
    try:
        ok, rvec, tvec, used, reproj = cube.solve_pnp_cube(
            img, K, D,
            use_ransac=True,
            min_markers=max(1, int(min_markers)),
            reproj_thr_mean_px=float(max_err),
            return_reproj=True,
            min_aspect=float(min_aspect),
        )
        if ok and reproj and float(reproj.get("err_mean", 99.0)) <= max_err:
            candidates.append({
                "T_C_O": rodrigues_to_Rt(rvec, tvec),
                "err_mean": float(reproj["err_mean"]),
                "n_points": int(reproj.get("n_points", 4)),
                "used_ids": [int(x) for x in used],
                "source": "redetect_cube_pnp",
            })
    except Exception:
        pass
    return candidates


def load_pose_observations(
    root: str,
    meta: Dict[str, Any],
    cube: AprilTagCubeTarget,
    K_map: Dict[int, np.ndarray],
    D_map: Dict[int, np.ndarray],
    all_cam_ids: List[int],
    gripper_cam_idx: int,
    reuse_stored_cube_candidates: bool,
    max_err_fixed: float,
    max_err_gripper: float,
    min_aspect_fixed: float,
    min_aspect_gripper: float,
    gripper_min_markers: int,
    exclude_gripped: bool = False,
    fixed_min_markers: int = 1,
) -> List[PoseObs]:
    """exclude_gripped: skip captures taken while the robot HOLDS the cube.

    Those captures are unusable for calibration: the per-set `set_cube_center_6dof`
    no longer describes where the cube is (it moved with the gripper), and for the
    eye-in-hand camera the target moves *with* the camera, so there is no relative
    motion to solve hand-eye from. Mixing them in corrupts the per-set vision mean
    and hence the Kabsch robot-base anchor.

    fixed_min_markers: minimum distinct cube markers a FIXED-camera pose must use.
    A single-marker cube pose is planar-PnP flip-ambiguous (the estimated cube can
    be ~180deg rotated with its center placed ~a cube-width off), which is the
    dominant source of the ~150deg / ~140mm cross-camera outliers and the
    pose-repeatability / hand-eye verify failures. Requiring >=2 markers removes it.
    """
    obs: List[PoseObs] = []
    for cap in meta.get("captures", []):
        eid = int(cap.get("event_id", -1))
        if eid < 0:
            continue
        if exclude_gripped and cap.get("cube_gripped"):
            continue
        sidx = get_capture_set_index(cap)
        for ci_str, cinfo in cap.get("cams", {}).items():
            ci = int(ci_str)
            if ci not in all_cam_ids or not cinfo.get("saved"):
                continue
            is_grip = ci == gripper_cam_idx
            max_err = max_err_gripper if is_grip else max_err_fixed
            min_aspect = min_aspect_gripper if is_grip else min_aspect_fixed
            min_markers = gripper_min_markers if is_grip else max(1, int(fixed_min_markers))

            candidates = []
            if reuse_stored_cube_candidates:
                candidates.extend(stored_cube_pose_candidates(
                    cinfo, ci, gripper_cam_idx, max_err, min_markers, min_aspect
                ))
            rgb_rel = cinfo.get("rgb_path", "")
            if rgb_rel:
                img = cv2.imread(os.path.join(root, rgb_rel))
                if img is not None:
                    candidates = estimate_image_cube_pose(
                        cube, img, K_map[ci], D_map[ci], max_err, min_markers, min_aspect
                    ) + candidates
            candidates = filter_candidates_for_camera_role(candidates, ci, gripper_cam_idx)
            # Enforce the fixed-camera marker-count floor: a selected candidate that
            # used fewer than `min_markers` distinct markers is flip-ambiguous.
            if not is_grip and int(fixed_min_markers) > 1:
                candidates = [c for c in candidates
                              if len(set(c.get("used_ids", []))) >= int(fixed_min_markers)]
            best = select_primary_cube_candidate(candidates) if candidates else None
            if best is None:
                continue
            obs.append(PoseObs(
                cam=ci,
                event=eid,
                set_idx=int(sidx) if sidx is not None else None,
                T_C_O=np.asarray(best["T_C_O"], dtype=np.float64),
                err_px=float(best.get("err_mean", 99.0)),
                n_points=int(best.get("n_points", 4)),
                source=str(best.get("source", "unknown")),
            ))
    return obs


def observations_by_cam_event(pose_obs: List[PoseObs]) -> Dict[int, Dict[int, PoseObs]]:
    out: Dict[int, Dict[int, PoseObs]] = defaultdict(dict)
    for o in pose_obs:
        out[o.cam][o.event] = o
    return out


def build_ref_relative_from_pairwise(
    pose_obs: List[PoseObs],
    fixed_cam_ids: List[int],
    ref_cam: int,
    robust: bool,
) -> Tuple[Dict[int, np.ndarray], Dict[str, Any]]:
    by = observations_by_cam_event(pose_obs)
    T_ref_C: Dict[int, np.ndarray] = {ref_cam: np.eye(4, dtype=np.float64)}
    diag: Dict[str, Any] = {}
    for ci in fixed_cam_ids:
        if ci == ref_cam:
            continue
        common = sorted(set(by.get(ref_cam, {}).keys()) & set(by.get(ci, {}).keys()))
        Ts, ws = [], []
        for eid in common:
            T_ref_O = by[ref_cam][eid].T_C_O
            T_ci_O = by[ci][eid].T_C_O
            Ts.append(T_ref_O @ inv_T(T_ci_O))
            ws.append(1.0 / max(by[ref_cam][eid].err_px * by[ci][eid].err_px, 1e-9))
        if not Ts:
            continue
        if robust:
            T, st = robust_se3_average(Ts, ws)
        else:
            T = weighted_se3_average(Ts, None)
            st = {"num_total": len(Ts), "num_inliers": len(Ts), "inlier_ratio": 1.0}
        T_ref_C[ci] = T
        diag[f"T_ref_C{ci}"] = st
    return T_ref_C, diag


def initialize_ref_object_poses(
    pose_obs: List[PoseObs],
    T_ref_C: Dict[int, np.ndarray],
    fixed_cam_ids: List[int],
    ref_cam: int,
) -> Dict[int, np.ndarray]:
    by_event: Dict[int, List[Tuple[np.ndarray, float]]] = defaultdict(list)
    for o in pose_obs:
        if o.cam not in fixed_cam_ids or o.cam not in T_ref_C:
            continue
        # T_ref_O = T_ref_Ci * T_Ci_O
        by_event[o.event].append((T_ref_C[o.cam] @ o.T_C_O, 1.0 / max(o.err_px, 1e-9)))
    out: Dict[int, np.ndarray] = {}
    for eid, pairs in by_event.items():
        out[eid] = weighted_se3_average([p[0] for p in pairs], [p[1] for p in pairs])
    return out


def load_nominal_set_cube_pose6(meta: Dict[str, Any]) -> Dict[int, List[float]]:
    """Raw set_cube_center_6dof per capture set (for diagnostics/CSV)."""
    out: Dict[int, List[float]] = {}
    for cap in meta.get("captures", []):
        sidx = get_capture_set_index(cap)
        if sidx is None or int(sidx) in out:
            continue
        raw = get_capture_set_cube_center_transform_raw(cap)
        pose = try_parse_pose6(raw)
        if pose is None:
            pose = try_parse_pose6(cap.get("set_cube_center_6dof"))
        if pose is not None:
            out[int(sidx)] = [float(x) for x in pose]
    return out


def depth_scale_crosscheck(meta: Dict[str, Any], fixed_only: bool = True) -> Dict[str, Any]:
    """Independent scale cross-check (DIAGNOSTIC ONLY — never feeds the solver).

    Compares each cube observation's vision PnP camera-distance (which scales with
    the colour focal length) against the RealSense depth distance (independent of
    the colour intrinsics). A median PnP/depth ratio != 1 flags a vision/intrinsics
    scale error; a ratio ~1 while a scale error persists downstream points at the
    robot/FK side instead. Requires depth-populated meta (see
    AprilTagCubeTarget._depth_plane_metrics); returns {"n": 0, ...} otherwise.

    Uses the surface-to-surface scale (depth_z_scale_pred_over_meas): PnP-predicted
    plane depth vs measured depth at the SAME pixels, so the cube-centre-vs-surface
    offset cancels. plane residual is reported as a reliability guard.
    """
    scales: List[float] = []
    planes: List[float] = []
    per_cam: Dict[int, List[float]] = defaultdict(list)
    for cap in meta.get("captures", []):
        for ci_s, cinfo in (cap.get("cams") or {}).items():
            if not cinfo.get("saved"):
                continue
            if fixed_only and cinfo.get("is_gripper"):
                continue
            pnp = cinfo.get("cube_pnp") or {}
            if not (pnp.get("ok") and pnp.get("depth_valid")):
                continue
            s = pnp.get("depth_z_scale_pred_over_meas")
            if s is None or not np.isfinite(float(s)) or float(s) <= 0.0:
                continue
            scales.append(float(s))
            per_cam[int(ci_s)].append(float(s))
            pm = pnp.get("depth_plane_median_mm")
            if pm is not None:
                planes.append(float(pm))
    if not scales:
        return {"n": 0, "reason": "no depth-valid cube observations with a surface scale "
                "(reprocess capture so depth metrics populate)"}
    arr = np.asarray(scales, dtype=np.float64)
    med = float(np.median(arr))
    return {
        "n": int(arr.size),
        "median_pred_over_meas": med,
        "mean_pred_over_meas": float(arr.mean()),
        "implied_vision_scale_pct": (med - 1.0) * 100.0,
        "median_plane_residual_mm": (float(np.median(planes)) if planes else None),
        "per_cam_median": {int(k): float(np.median(v)) for k, v in per_cam.items()},
    }


def fk_scale_crosscheck(
    meta: Dict[str, Any],
    min_disp_mm: float = 100.0,
    max_reproj_px: float = 1.5,
    max_plane_mm: float = 15.0,
    max_iqr_width: float = 0.15,
) -> Dict[str, Any]:
    """Independent FK-scale cross-check (DIAGNOSTIC ONLY — never feeds the solver).

    Compares how far the cube CENTRE moves between captures as seen by vision
    (per fixed camera, from cube_pnp) vs by robot FK (capture_cube_center_6dof,
    tool-4 TCP — purely kinematic, no vision). Both track the same physical point,
    so |dVis|/|dFK| over capture pairs is rotation/translation-invariant and isolates
    a scale mismatch. ~1.0 means FK and vision distances agree; a robust ratio != 1
    while vision matches depth (see depth_scale_crosscheck) points the scale error at
    the robot/FK side (kinematics or the tool-4 cube-centre offset).

    Only vision poses with low reprojection AND depth-confirmed planarity are used.
    Per-camera IQR width flags cameras whose cube motion is mostly along the optical
    axis (monocular depth is unreliable there); the 'reliable' aggregate keeps only
    cameras with IQR width <= max_iqr_width.
    """
    from itertools import combinations
    per_cam_pts: Dict[int, List[Tuple[np.ndarray, np.ndarray]]] = defaultdict(list)
    for cap in meta.get("captures", []):
        fk = cap.get("capture_cube_center_6dof")
        if not fk or len(fk) < 3:
            continue
        p_fk = np.asarray(fk[:3], dtype=np.float64)
        for ci_s, cinfo in (cap.get("cams") or {}).items():
            if not cinfo.get("saved") or cinfo.get("is_gripper"):
                continue
            pnp = cinfo.get("cube_pnp") or {}
            if not (pnp.get("ok") and pnp.get("T_cam_cube_4x4")):
                continue
            reproj = pnp.get("reproj_mean_px")
            plane = pnp.get("depth_plane_median_mm")
            if reproj is None or float(reproj) > max_reproj_px:
                continue
            if plane is None or float(plane) > max_plane_mm:
                continue
            p_vis = np.asarray(pnp["T_cam_cube_4x4"], dtype=np.float64).reshape(4, 4)[:3, 3] * 1000.0
            per_cam_pts[int(ci_s)].append((p_vis, p_fk))

    per_cam: Dict[int, Dict[str, Any]] = {}
    reliable_ratios: List[float] = []
    all_ratios: List[float] = []
    for ci, lst in sorted(per_cam_pts.items()):
        V = np.array([v for v, _ in lst])
        F = np.array([f for _, f in lst])
        rr = [np.linalg.norm(V[i] - V[j]) / d
              for i, j in combinations(range(len(lst)), 2)
              for d in [np.linalg.norm(F[i] - F[j])] if d >= min_disp_mm]
        if not rr:
            continue
        rr = np.asarray(rr, dtype=np.float64)
        lo, hi = float(np.percentile(rr, 25)), float(np.percentile(rr, 75))
        per_cam[ci] = {"median": float(np.median(rr)), "iqr_lo": lo, "iqr_hi": hi, "n": int(rr.size)}
        all_ratios.extend(rr.tolist())
        if (hi - lo) <= max_iqr_width:
            reliable_ratios.extend(rr.tolist())
    if not all_ratios:
        return {"n": 0, "reason": "no quality-gated cube poses with FK cube-centre "
                "(reprocess capture so depth metrics populate, and capture FK cube centre)"}
    rel = np.asarray(reliable_ratios or all_ratios, dtype=np.float64)
    med = float(np.median(rel))
    return {
        "n": int(len(all_ratios)),
        "n_reliable": int(rel.size),
        "reliable_median_vis_over_fk": med,
        "implied_fk_scale_pct": (1.0 / med - 1.0) * 100.0,  # FK larger than vision if >0
        "pooled_median_vis_over_fk": float(np.median(all_ratios)),
        "per_cam": per_cam,
    }


def estimate_robot_pos_scale(
    meta: Dict[str, Any],
    max_rot_deg: float = 5.0,
    min_disp_mm: float = 40.0,
    max_reproj_px: float = 1.5,
    max_plane_mm: float = 15.0,
    max_iqr_width: float = 0.05,
) -> Dict[str, Any]:
    """Estimate the robot Cartesian scale factor k = |dVis| / |dFlange| from data.

    Offset/tool/grasp-independent: uses only capture PAIRS with near-zero relative
    flange rotation (pure translation), where the gripped cube's displacement equals
    the flange's displacement. |dVis| (vision, metrically anchored by the marker and
    confirmed by depth) over |dFlange| (robot_pose_6dof) gives k directly. This is a
    measurement, not a switch: adopting k means editing ROBOT_POS_SCALE_PINNED and
    regenerating every stored result. Robust median over cameras whose
    per-camera IQR width <= max_iqr_width (noisier cameras are reported but excluded).
    """
    from itertools import combinations
    per_cam: Dict[int, List[Tuple[np.ndarray, np.ndarray, np.ndarray]]] = defaultdict(list)
    for cap in meta.get("captures", []):
        if not cap.get("cube_gripped"):
            continue
        p6 = try_parse_pose6(cap.get("robot_pose_6dof"))
        Tf = cap.get("robot_pose_matrix_4x4")
        if p6 is None or Tf is None:
            continue
        p_fl = np.asarray(p6[:3], dtype=np.float64)
        Rf = np.asarray(Tf, dtype=np.float64).reshape(4, 4)[:3, :3]
        for ci_s, cinfo in (cap.get("cams") or {}).items():
            if not cinfo.get("saved") or cinfo.get("is_gripper"):
                continue
            pnp = cinfo.get("cube_pnp") or {}
            if not (pnp.get("ok") and pnp.get("T_cam_cube_4x4")):
                continue
            if (pnp.get("reproj_mean_px") or 99.0) > max_reproj_px:
                continue
            if (pnp.get("depth_plane_median_mm") or 99.0) > max_plane_mm:
                continue
            p_vis = np.asarray(pnp["T_cam_cube_4x4"], dtype=np.float64).reshape(4, 4)[:3, 3] * 1000.0
            per_cam[int(ci_s)].append((p_vis, p_fl, Rf))

    def rot_angle(Ra, Rb):
        return np.degrees(np.arccos(np.clip((np.trace(Ra @ Rb.T) - 1.0) / 2.0, -1.0, 1.0)))

    per_cam_med: Dict[int, Dict[str, Any]] = {}
    reliable: List[float] = []
    for ci, lst in sorted(per_cam.items()):
        V = np.array([x[0] for x in lst]); Fl = np.array([x[1] for x in lst]); Rs = [x[2] for x in lst]
        rr = [np.linalg.norm(V[i] - V[j]) / d
              for i, j in combinations(range(len(lst)), 2)
              for d in [np.linalg.norm(Fl[i] - Fl[j])]
              if d >= min_disp_mm and rot_angle(Rs[i], Rs[j]) <= max_rot_deg]
        if not rr:
            continue
        rr = np.asarray(rr, dtype=np.float64)
        lo, hi = float(np.percentile(rr, 25)), float(np.percentile(rr, 75))
        per_cam_med[ci] = {"median": float(np.median(rr)), "iqr_lo": lo, "iqr_hi": hi, "n": int(rr.size)}
        if (hi - lo) <= max_iqr_width:
            reliable.extend(rr.tolist())
    if not per_cam_med:
        return {"n": 0, "reason": "no pure-translation gripped pairs with quality-gated vision"}
    pool = reliable if reliable else [d["median"] for d in per_cam_med.values()]
    k = float(np.median(pool))
    return {
        "k": k,
        "implied_robot_short_pct": (1.0 - 1.0 / k) * 100.0,
        "n_reliable": len(reliable),
        "per_cam": per_cam_med,
    }


def kabsch_rigid(src: np.ndarray, dst: np.ndarray) -> np.ndarray:
    """Proper rigid SE(3) mapping src points -> dst points (no reflection, no scale)."""
    src = np.asarray(src, dtype=np.float64).reshape(-1, 3)
    dst = np.asarray(dst, dtype=np.float64).reshape(-1, 3)
    cs, cd = src.mean(0), dst.mean(0)
    H = (src - cs).T @ (dst - cd)
    U, _, Vt = np.linalg.svd(H)
    d = float(np.sign(np.linalg.det(Vt.T @ U.T)))
    Rm = Vt.T @ np.diag([1.0, 1.0, d]) @ U.T
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = Rm
    T[:3, 3] = cd - Rm @ cs
    return T


def robust_kabsch_rigid(
    src: np.ndarray, dst: np.ndarray, max_resid_mm: float, min_keep: int = 3, iters: int = 5
) -> Tuple[np.ndarray, np.ndarray]:
    """Kabsch with iterative outlier rejection on per-point residual (mm).

    Returns (T, keep_mask). Used to anchor the cam-ref frame into robot base
    using only the (reliable) cube-center positions.
    """
    src = np.asarray(src, dtype=np.float64).reshape(-1, 3)
    dst = np.asarray(dst, dtype=np.float64).reshape(-1, 3)
    keep = np.ones(len(src), dtype=bool)
    T = kabsch_rigid(src, dst)
    for _ in range(iters):
        pred = (T[:3, :3] @ src.T).T + T[:3, 3]
        resid_mm = np.linalg.norm(pred - dst, axis=1) * 1000.0
        new_keep = resid_mm <= float(max_resid_mm)
        if new_keep.sum() < max(min_keep, 3):
            # keep the best `min_keep` instead of collapsing
            order = np.argsort(resid_mm)
            new_keep = np.zeros_like(keep)
            new_keep[order[:max(min_keep, 3)]] = True
        if np.array_equal(new_keep, keep) and T is not None:
            keep = new_keep
            break
        keep = new_keep
        T = kabsch_rigid(src[keep], dst[keep])
    return T, keep


def initialize_base_translation_anchored(
    pose_obs: List[PoseObs],
    fixed_cam_ids: List[int],
    ref_cam: int,
    set_priors: Dict[int, np.ndarray],
    set_pose6: Dict[int, List[float]],
    event_to_set: Dict[int, Optional[int]],
    max_trans_error_mm: float,
    max_rot_error_deg: float,
    disable_if_inconsistent: bool,
) -> Tuple[np.ndarray, Dict[int, np.ndarray], Dict[int, np.ndarray], Dict[str, Any], List[Dict[str, Any]], Dict[str, Any]]:
    """Robot-base calibration anchored by the RELIABLE part of the prior only.

    Root-cause: set_cube_center_6dof has a correct cube-CENTER position but an
    UNRELIABLE orientation. So we:
      1) build the vision calibration in the cam-ref frame (excellent),
      2) estimate T_base_ref by rigid-aligning per-set cube-center POSITIONS
         (prior_base vs vision_ref) with outlier rejection,
      3) express cameras/objects in base frame: T_base_X = T_base_ref @ T_ref_X.
    The prior rotation is intentionally NOT used (it is diagnosed + reported).

    Returns (T_base_C, T_base_O_event, diag, prior_rows, prior_stats).
    """
    # 1) vision-only calibration in cam-ref frame
    T_ref_C, vdiag = build_ref_relative_from_pairwise(pose_obs, fixed_cam_ids, ref_cam, robust=True)
    T_ref_O = initialize_ref_object_poses(pose_obs, T_ref_C, fixed_cam_ids, ref_cam)

    # per-set mean vision object pose (cube is static within a set)
    by_set: Dict[int, List[np.ndarray]] = defaultdict(list)
    for eid, T in T_ref_O.items():
        s = event_to_set.get(eid)
        if s is not None and s in set_priors:
            by_set[int(s)].append(T)
    set_vis: Dict[int, np.ndarray] = {
        s: weighted_se3_average(lst) for s, lst in by_set.items() if lst
    }
    common_sets = sorted(set_vis.keys())

    diag: Dict[str, Any] = {"anchor": "translation_only_kabsch", "vision_pairwise": vdiag}
    if len(common_sets) < 3:
        # not enough sets to anchor; fall back to vision frame (ref = base)
        diag["anchor_status"] = f"insufficient_prior_sets ({len(common_sets)})"
        T_base_O_event = dict(T_ref_O)
        stats = {
            "num_prior_total": len(common_sets), "num_prior_used": 0,
            "num_prior_rejected": len(common_sets),
            "median_prior_trans_error_mm": None, "median_prior_rot_error_deg": None,
            "reason": "too few prior sets to anchor base frame; reporting cam-ref frame",
        }
        return np.eye(4), T_ref_C, T_base_O_event, diag, [], stats

    src = np.array([set_vis[s][:3, 3] for s in common_sets])          # cam-ref positions
    dst = np.array([set_priors[s][:3, 3] for s in common_sets])       # base positions
    T_base_ref, keep = robust_kabsch_rigid(src, dst, max_resid_mm=max_trans_error_mm)
    used_sets = [common_sets[i] for i in range(len(common_sets)) if keep[i]]
    rejected_sets = [s for s in common_sets if s not in used_sets]
    pred = (T_base_ref[:3, :3] @ src.T).T + T_base_ref[:3, 3]
    anchor_resid_mm = float(np.sqrt(np.mean(np.sum((pred - dst) ** 2, axis=1))) * 1000.0)
    diag["anchor_rms_mm"] = anchor_resid_mm
    diag["anchor_used_sets"] = used_sets
    diag["anchor_rejected_sets"] = rejected_sets

    # 2) express everything in base frame
    T_base_C = {ci: T_base_ref @ T_ref_C[ci] for ci in T_ref_C}
    T_base_O_event = {eid: T_base_ref @ T for eid, T in T_ref_O.items()}

    # 3) per-event prior diagnostics: prior pose vs vision-estimated pose (base frame)
    prior_rows: List[Dict[str, Any]] = []
    tr_errs: List[float] = []
    ro_errs: List[float] = []
    for eid in sorted(T_base_O_event.keys()):
        s = event_to_set.get(eid)
        if s is None or s not in set_priors:
            continue
        P = set_priors[s]
        est = T_base_O_event[eid]
        delta = inv_T(P) @ est
        dt = float(np.linalg.norm(delta[:3, 3]) * 1000.0)
        dr = float(np.degrees(np.linalg.norm(R.from_matrix(delta[:3, :3]).as_rotvec())))
        tr_errs.append(dt)
        ro_errs.append(dr)
        raw6 = set_pose6.get(s)
        max_t = max(abs(P[0, 3]), abs(P[1, 3]), abs(P[2, 3]))
        unit_warn = "translation>5m: possible unit (mm/m) error" if max_t > 5.0 else ""
        rot_warn = f"rotation prior off by {dr:.0f}deg (>{max_rot_error_deg:.0f})" if dr > max_rot_error_deg else ""
        prior_rows.append({
            "event_id": eid,
            "set_index": int(s),
            "raw_set_cube_center_6dof": raw6,
            "prior_translation_m": [float(x) for x in P[:3, 3]],
            "prior_rotation_rotvec": [float(x) for x in R.from_matrix(P[:3, :3]).as_rotvec()],
            "estimated_cube_translation_m": [float(x) for x in est[:3, 3]],
            "estimated_cube_rotation_rotvec": [float(x) for x in R.from_matrix(est[:3, :3]).as_rotvec()],
            "delta_trans_mm": dt,
            "delta_rot_deg": dr,
            "possible_unit_warning": unit_warn,
            "possible_rotation_warning": rot_warn,
            "set_used_for_anchor": bool(s in used_sets),
        })

    med_t = float(np.median(tr_errs)) if tr_errs else None
    med_r = float(np.median(ro_errs)) if ro_errs else None
    n_total = len(prior_rows)
    n_used = sum(1 for r in prior_rows if r["set_used_for_anchor"])
    rot_inconsistent = med_r is not None and med_r > max_rot_error_deg

    reason_bits = [f"translation-only anchor, rms={anchor_resid_mm:.1f}mm over {len(used_sets)} sets"]
    if rot_inconsistent:
        reason_bits.append(
            f"rotation prior REJECTED (median {med_r:.1f}deg > {max_rot_error_deg:.0f}deg): "
            "orientation in set_cube_center_6dof does not match the observed cube"
        )
    if rejected_sets and disable_if_inconsistent:
        reason_bits.append(f"position-outlier sets excluded from anchor: {rejected_sets}")
    stats = {
        "num_prior_total": n_total,
        "num_prior_used": n_used,
        "num_prior_rejected": n_total - n_used,
        "median_prior_trans_error_mm": med_t,
        "median_prior_rot_error_deg": med_r,
        "rotation_prior_used": (not rot_inconsistent),
        "translation_prior_used": True,
        "anchor_rms_mm": anchor_resid_mm,
        "reason": "; ".join(reason_bits),
    }
    diag["prior_stats"] = stats
    return T_base_ref, T_base_C, T_base_O_event, diag, prior_rows, stats


def pose_consistency_metrics(
    pose_obs: List[PoseObs],
    T_cam: Dict[int, np.ndarray],
    T_obj_event: Dict[int, np.ndarray],
    fixed_cam_ids: List[int],
) -> Tuple[Optional[float], Optional[float]]:
    trans_mm, rot_deg = [], []
    for o in pose_obs:
        if o.cam not in fixed_cam_ids or o.cam not in T_cam or o.event not in T_obj_event:
            continue
        pred = inv_T(T_cam[o.cam]) @ T_obj_event[o.event]
        Terr = inv_T(o.T_C_O) @ pred
        trans_mm.append(float(np.linalg.norm(Terr[:3, 3]) * 1000.0))
        rot_deg.append(float(np.degrees(np.linalg.norm(R.from_matrix(Terr[:3, :3]).as_rotvec()))))
    if not trans_mm:
        return None, None
    return float(np.sqrt(np.mean(np.square(trans_mm)))), float(np.sqrt(np.mean(np.square(rot_deg))))


def prior_metrics(
    T_obj_event: Dict[int, np.ndarray],
    event_to_set: Dict[int, Optional[int]],
    set_priors: Dict[int, np.ndarray],
) -> Tuple[Optional[float], Optional[float]]:
    trans_mm, rot_deg = [], []
    for eid, T in T_obj_event.items():
        sidx = event_to_set.get(eid)
        if sidx is None or sidx not in set_priors:
            continue
        Terr = inv_T(set_priors[sidx]) @ T
        trans_mm.append(float(np.linalg.norm(Terr[:3, 3]) * 1000.0))
        rot_deg.append(float(np.degrees(np.linalg.norm(R.from_matrix(Terr[:3, :3]).as_rotvec()))))
    if not trans_mm:
        return None, None
    return float(np.sqrt(np.mean(np.square(trans_mm)))), float(np.sqrt(np.mean(np.square(rot_deg))))


def estimate_object_poses_from_cams(
    pose_obs: List[PoseObs],
    T_cam: Dict[int, np.ndarray],
    fixed_cam_ids: List[int],
) -> Dict[int, np.ndarray]:
    """Given calibrated cameras (base frame) T_cam and per-event cube observations,
    estimate the per-event object pose in the SAME frame as T_cam.

    Used for held-out evaluation: cameras are fit on TRAIN sets, then the cube pose
    on each TEST-set event is triangulated from those cameras (no test FK used) and
    compared against the test FK prior. Same math as initialize_ref_object_poses but
    with whatever frame T_cam already lives in (base, after to_base())."""
    by_event: Dict[int, List[Tuple[np.ndarray, float]]] = defaultdict(list)
    for o in pose_obs:
        if o.cam not in fixed_cam_ids or o.cam not in T_cam:
            continue
        by_event[o.event].append((T_cam[o.cam] @ o.T_C_O, 1.0 / max(o.err_px, 1e-9)))
    out: Dict[int, np.ndarray] = {}
    for eid, pairs in by_event.items():
        out[eid] = weighted_se3_average([p[0] for p in pairs], [p[1] for p in pairs])
    return out


def reprojection_errors(
    corner_obs: List[CornerObs],
    T_cam: Dict[int, np.ndarray],
    T_obj_event: Dict[int, np.ndarray],
    K_map: Dict[int, np.ndarray],
    D_map: Dict[int, np.ndarray],
    fixed_cam_ids: List[int],
) -> np.ndarray:
    errs: List[float] = []
    for o in corner_obs:
        if o.cam not in fixed_cam_ids or o.cam not in T_cam or o.event not in T_obj_event:
            continue
        T_C_O = inv_T(T_cam[o.cam]) @ T_obj_event[o.event]
        rvec = R.from_matrix(T_C_O[:3, :3]).as_rotvec().reshape(3, 1)
        tvec = T_C_O[:3, 3].reshape(3, 1)
        proj, _ = cv2.projectPoints(o.object_points.astype(np.float64), rvec, tvec, K_map[o.cam], D_map[o.cam])
        diff = proj.reshape(-1, 2) - o.image_points.reshape(-1, 2)
        errs.extend(np.linalg.norm(diff, axis=1).tolist())
    return np.asarray(errs, dtype=np.float64)


# ══════════════════════════════════════════════════════════════════════════════
# 공유 고정-카메라 solver — 03(pose-consistency) / 04(direct reprojection)
# ------------------------------------------------------------------------------
# 예전에는 이 두 최적화기가 CP_C3 안에만 있었고, Step3 의 고정-카메라 등록은
# closed-form robust SE(3) 평균(= C3 의 02_pnp_robust_se3)에서 멈췄다. 실측 비교
# (data/session)에서 04(재투영오차 직접 최소화)가 재투영 RMSE 를 9.31→5.51px(-41%),
# median 1.48→0.79px(-47%) 로 낮춰 픽셀 정합이 가장 좋았다. 그래서 세 스크립트
# (Step3 / CP_C1 / CP_C3)가 모두 같은 solver 를 쓰도록 여기로 끌어올린다.
#
# 기본 정책: solve_fixed_cameras(prefer="reproj") = 04 를 먼저 시도하고, 마커 코너
# 관측이 없거나(코너 검출 불가) 최적화가 개선에 실패하면 자동으로 03 으로 폴백한다.
# 두 최적화기 모두 게이지(ref_cam)를 항등으로 고정한 cam-ref 프레임에서 풀고, 초기값은
# pairwise robust SE(3)(build_ref_relative_from_pairwise) 로 준다.

def build_param_layout(cam_ids: List[int], event_ids: List[int],
                       ref_cam: Optional[int]) -> Dict[str, Any]:
    """{cam(≠ref), event} 별 6-vec 슬롯 배치. ref_cam 은 항등으로 고정(변수 아님)."""
    cam_vars = [ci for ci in cam_ids if ref_cam is None or ci != ref_cam]
    layout = {"cam_vars": cam_vars, "event_vars": event_ids,
              "cam_slice": {}, "event_slice": {}, "n": 0}
    k = 0
    for ci in cam_vars:
        layout["cam_slice"][ci] = slice(k, k + 6); k += 6
    for eid in event_ids:
        layout["event_slice"][eid] = slice(k, k + 6); k += 6
    layout["n"] = k
    return layout


def pack_params(T_cam: Dict[int, np.ndarray], T_obj: Dict[int, np.ndarray],
                layout: Dict[str, Any]) -> np.ndarray:
    x = np.zeros(layout["n"], dtype=np.float64)
    for ci, sl in layout["cam_slice"].items():
        x[sl] = T_to_vec(T_cam[ci])
    for eid, sl in layout["event_slice"].items():
        x[sl] = T_to_vec(T_obj[eid])
    return x


def unpack_params(x: np.ndarray, layout: Dict[str, Any],
                  ref_cam: Optional[int]) -> Tuple[Dict[int, np.ndarray], Dict[int, np.ndarray]]:
    T_cam: Dict[int, np.ndarray] = {}
    if ref_cam is not None:
        T_cam[ref_cam] = np.eye(4, dtype=np.float64)
    for ci, sl in layout["cam_slice"].items():
        T_cam[ci] = vec_to_T(x[sl])
    T_obj = {eid: vec_to_T(x[sl]) for eid, sl in layout["event_slice"].items()}
    return T_cam, T_obj


def prior_residual_terms(T_obj: Dict[int, np.ndarray],
                         event_to_set: Dict[int, Optional[int]],
                         set_priors: Dict[int, np.ndarray],
                         w_trans: float, w_rot: float) -> List[float]:
    """FK 큐브중점 soft prior 잔차 (병진/회전 가중 분리).

    prior 의 회전은 신뢰 불가(뒤집힘)라 w_rot 기본 0 — 병진(큐브중심)만 당긴다."""
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
            res.extend((R.from_matrix(Terr[:3, :3]).as_rotvec() * float(w_rot)).tolist())
    return res


def _finalize_opt(init_T_cam, init_T_obj, opt_T_cam, opt_T_obj, residual, x0, opt,
                  adoption_guard: bool = True,
                  ) -> Tuple[Dict[int, np.ndarray], Dict[int, np.ndarray], Dict[str, Any]]:
    """비용이 실제로 낮아졌을 때만 최적화 결과를 채택 (아니면 초기값 유지).

    scipy `success` 는 종료조건 도달만 뜻하지 목적함수 개선을 보장하지 않는다."""
    c0 = float(np.mean(residual(x0) ** 2))
    c1 = float(np.mean(residual(opt.x) ** 2))
    improved = bool(c1 < c0)
    candidate_usable = bool(opt.success) and np.isfinite(c1)
    accepted = candidate_usable and (improved or not bool(adoption_guard))
    info = {"optimized": True, "optimizer_success": bool(opt.success),
            "accepted": accepted, "cost_initial": c0, "cost_final": c1,
            "cost_improved": improved,
            "adoption_guard_enabled": bool(adoption_guard),
            "forced_candidate_used": bool(accepted and not adoption_guard),
            "status": int(opt.status), "message": str(opt.message),
            "optimality": float(opt.optimality), "nfev": int(opt.nfev)}
    if accepted:
        return opt_T_cam, opt_T_obj, info
    info["fallback_reason"] = ("final cost was not lower than initial cost"
                               if bool(opt.success)
                               else f"optimizer did not converge (status={int(opt.status)})")
    return init_T_cam, init_T_obj, info


def optimize_pose_consistency(
    pose_obs: List[PoseObs], fixed_cam_ids: List[int],
    init_T_cam: Dict[int, np.ndarray], init_T_obj: Dict[int, np.ndarray],
    ref_cam: Optional[int], event_to_set: Dict[int, Optional[int]],
    set_priors: Optional[Dict[int, np.ndarray]],
    prior_weight_trans: float = 0.0, prior_weight_rot: float = 0.0,
    adoption_guard: bool = True, max_nfev: int = 300,
    tol: float = 1e-10,
) -> Tuple[Dict[int, np.ndarray], Dict[int, np.ndarray], Dict[str, Any]]:
    """03: 카메라·이벤트 pose 를 SE(3) pose 일관성으로 동시 최적화 (픽셀 불필요)."""
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
        res: List[float] = []
        for o in usable:
            pred = inv_T(T_cam[o.cam]) @ T_obj[o.event]
            e = se3_log_residual(inv_T(o.T_C_O) @ pred)
            w = math.sqrt(min(50.0, 1.0 / max(o.err_px, 1e-6)))
            res.extend((e * w).tolist())
        if set_priors:
            res.extend(prior_residual_terms(T_obj, event_to_set, set_priors,
                                            prior_weight_trans, prior_weight_rot))
        return np.asarray(res, dtype=np.float64)

    opt = least_squares(residual, x0, method="trf", loss="huber", f_scale=0.003,
                        max_nfev=int(max_nfev), xtol=float(tol),
                        ftol=float(tol), gtol=float(tol))
    T_cam, T_obj = unpack_params(opt.x, layout, ref_cam=ref_cam)
    return _finalize_opt(
        init_T_cam, init_T_obj, T_cam, T_obj, residual, x0, opt,
        adoption_guard=adoption_guard)


def optimize_reprojection(
    corner_obs: List[CornerObs], pose_obs: List[PoseObs], fixed_cam_ids: List[int],
    init_T_cam: Dict[int, np.ndarray], init_T_obj: Dict[int, np.ndarray],
    ref_cam: Optional[int], K_map: Dict[int, np.ndarray], D_map: Dict[int, np.ndarray],
    event_to_set: Dict[int, Optional[int]], set_priors: Optional[Dict[int, np.ndarray]],
    prior_weight_trans: float = 0.0, prior_weight_rot: float = 0.0,
    pose_regularizer_weight: float = 2.0,
    adoption_guard: bool = True, max_nfev: int = 500,
    tol: float = 1e-10,
) -> Tuple[Dict[int, np.ndarray], Dict[int, np.ndarray], Dict[str, Any]]:
    """04: 마커 코너의 픽셀 재투영오차를 직접 최소화 (pose 항으로 약하게 정규화)."""
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
            proj, _ = cv2.projectPoints(o.object_points.astype(np.float64), rvec, tvec,
                                        K_map[o.cam], D_map[o.cam])
            res.extend((proj.reshape(-1, 2) - o.image_points.reshape(-1, 2)).reshape(-1).tolist())
        if pose_regularizer_weight > 0.0:
            for o in usable_poses:
                pred = inv_T(T_cam[o.cam]) @ T_obj[o.event]
                e = se3_log_residual(inv_T(o.T_C_O) @ pred)
                res.extend((e * float(pose_regularizer_weight)).tolist())
        if set_priors:
            res.extend(prior_residual_terms(T_obj, event_to_set, set_priors,
                                            prior_weight_trans, prior_weight_rot))
        return np.asarray(res, dtype=np.float64)

    opt = least_squares(residual, x0, method="trf", loss="huber", f_scale=2.0,
                        max_nfev=int(max_nfev), xtol=float(tol),
                        ftol=float(tol), gtol=float(tol))
    T_cam, T_obj = unpack_params(opt.x, layout, ref_cam=ref_cam)
    return _finalize_opt(
        init_T_cam, init_T_obj, T_cam, T_obj, residual, x0, opt,
        adoption_guard=adoption_guard)


def solve_fixed_cameras(
    pose_obs: List[PoseObs], fixed_cam_ids: List[int], ref_cam: int,
    K_map: Optional[Dict[int, np.ndarray]] = None,
    D_map: Optional[Dict[int, np.ndarray]] = None,
    corner_obs: Optional[List[CornerObs]] = None,
    event_to_set: Optional[Dict[int, Optional[int]]] = None,
    set_priors: Optional[Dict[int, np.ndarray]] = None,
    prior_weight_trans: float = 0.0, prior_weight_rot: float = 0.0,
    pose_regularizer_weight: float = 2.0,
    prefer: str = "reproj",
    init_T_cam: Optional[Dict[int, np.ndarray]] = None,
    init_T_obj: Optional[Dict[int, np.ndarray]] = None,
) -> Tuple[Dict[int, np.ndarray], Dict[int, np.ndarray], Dict[str, Any]]:
    """고정 카메라 상대 pose 를 04(재투영)→03(pose-consistency) 폴백으로 푼다.

    반환 (T_cam, T_obj, diag). 모두 cam-ref 프레임(ref_cam=항등). diag["method"] 는
    실제로 채택된 방법("04_direct_reprojection" | "03_pose_consistency" |
    "init_robust_se3"). 초기값은 pairwise robust SE(3) 로 자동 생성(명시 init 가능).

    prefer="reproj"(기본): 코너 관측이 충분하고 최적화가 개선하면 04 채택. 코너가
    없거나 04 가 기각되면 03 을 시도하고, 그것도 기각되면 robust 초기값을 그대로 반환.
    prefer="pose": 04 를 건너뛰고 03 만.
    """
    if init_T_cam is None or init_T_obj is None:
        init_T_cam, _ = build_ref_relative_from_pairwise(pose_obs, fixed_cam_ids, ref_cam, robust=True)
        init_T_obj = initialize_ref_object_poses(pose_obs, init_T_cam, fixed_cam_ids, ref_cam)
    event_to_set = event_to_set or {}

    diag: Dict[str, Any] = {"method": "init_robust_se3", "prefer": prefer}

    can_reproj = (prefer == "reproj" and corner_obs and K_map is not None and D_map is not None)
    if can_reproj:
        T_cam, T_obj, info04 = optimize_reprojection(
            corner_obs=corner_obs, pose_obs=pose_obs, fixed_cam_ids=fixed_cam_ids,
            init_T_cam=init_T_cam, init_T_obj=init_T_obj, ref_cam=ref_cam,
            K_map=K_map, D_map=D_map, event_to_set=event_to_set, set_priors=set_priors,
            prior_weight_trans=prior_weight_trans, prior_weight_rot=prior_weight_rot,
            pose_regularizer_weight=pose_regularizer_weight)
        diag["reproj"] = info04
        if info04.get("accepted"):
            diag["method"] = "04_direct_reprojection"
            return T_cam, T_obj, diag

    # 04 를 못 썼거나 기각됨 → 03 으로 폴백
    T_cam, T_obj, info03 = optimize_pose_consistency(
        pose_obs=pose_obs, fixed_cam_ids=fixed_cam_ids,
        init_T_cam=init_T_cam, init_T_obj=init_T_obj, ref_cam=ref_cam,
        event_to_set=event_to_set, set_priors=set_priors,
        prior_weight_trans=prior_weight_trans, prior_weight_rot=prior_weight_rot)
    diag["pose"] = info03
    if info03.get("accepted"):
        diag["method"] = "03_pose_consistency"
        return T_cam, T_obj, diag

    return init_T_cam, init_T_obj, diag
