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


def pose6_to_T_base_gripper(pose6: List[float]) -> np.ndarray:
    # Project convention: robot 6-DoF pose is [x,y,z (mm), rz,ry,rx (deg)] and
    # euler_deg_to_matrix returns the full 4x4 with translation in meters.
    return euler_deg_to_matrix(*[float(v) for v in pose6])


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
) -> Tuple[List[CornerObs], str]:
    """Return (corner observations, reason-string-if-empty).

    The reason string makes problem 2 debuggable: it distinguishes "no images/
    detections were loaded" from "cube model 3D marker corners are unavailable".
    """
    obs: List[CornerObs] = []
    # counters for an actionable zero-observations reason
    n_imgs_read = n_imgs_missing = 0
    n_detections = n_obj_corner_fail = n_aspect_reject = 0
    for cap in meta.get("captures", []):
        eid = int(cap.get("event_id", -1))
        if eid < 0:
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
                img_pts_raw = np.asarray(corners, dtype=np.float64).reshape(4, 2)
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
) -> List[PoseObs]:
    obs: List[PoseObs] = []
    for cap in meta.get("captures", []):
        eid = int(cap.get("event_id", -1))
        if eid < 0:
            continue
        sidx = get_capture_set_index(cap)
        for ci_str, cinfo in cap.get("cams", {}).items():
            ci = int(ci_str)
            if ci not in all_cam_ids or not cinfo.get("saved"):
                continue
            max_err = max_err_gripper if ci == gripper_cam_idx else max_err_fixed
            min_aspect = min_aspect_gripper if ci == gripper_cam_idx else min_aspect_fixed
            min_markers = gripper_min_markers if ci == gripper_cam_idx else 1

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
