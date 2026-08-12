#!/usr/bin/env python3
"""Historical strict-none A2 versus FK-fixed A3 diagnostic.

This script predates the shared canonical backend.  It is retained to preserve
the provenance of the existing supplementary result; Main Table A2/A3 runs
must use CP_ablation_7row.py and calibration_reprojection_backend.py.

Both methods share data, initialization for common variables, optimizer,
tolerances, robust loss, and held-out sets.  The only model difference is:

* A2: one free base->cube pose per training set; no FK-to-cube term.
* A3: the same poses are fixed to aligned FK and are not variables.

Both methods use robot FK as the common hand-eye kinematic backbone.  A2 uses
aligned cube FK only after fitting as a held-out evaluation proxy; it is absent
from A2's cube-pose model and initialization.  No Ridge, SE(3), or other
post-correction is computed or applied.
"""
from __future__ import annotations

import argparse
import csv
import json
import os
import time
from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Sequence, Tuple

import cv2
import numpy as np
from scipy.optimize import least_squares
from scipy.sparse import lil_matrix
from scipy.spatial.transform import Rotation

import CP_common as cp
import Step3_calibration as s3
from apriltag_cube import AprilTagCubeTarget, inv_T
from calibration_runtime_utils import (
    get_capture_set_index,
    load_calib_dir,
    load_intrinsics_with_depth_scale,
    resolve_cube_config_for_run,
)
from charuco_utils import CharucoTarget
from config import CharucoBoardConfig, get_default_cube_config


@dataclass
class PixelObs:
    marker: str
    cam: int
    event: int
    set_idx: Optional[int]
    object_points: np.ndarray
    image_points: np.ndarray


def T_to_vec(T: np.ndarray) -> np.ndarray:
    T = np.asarray(T, dtype=np.float64)
    return np.concatenate([
        Rotation.from_matrix(T[:3, :3]).as_rotvec(),
        T[:3, 3],
    ])


def vec_to_T(v: np.ndarray) -> np.ndarray:
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = Rotation.from_rotvec(np.asarray(v[:3], dtype=np.float64)).as_matrix()
    T[:3, 3] = np.asarray(v[3:6], dtype=np.float64)
    return T


def pose_delta(A: np.ndarray, B: np.ndarray) -> Tuple[float, float]:
    E = inv_T(np.asarray(A, dtype=np.float64)) @ np.asarray(B, dtype=np.float64)
    dt = float(np.linalg.norm(E[:3, 3]) * 1000.0)
    dr = float(np.degrees(np.linalg.norm(Rotation.from_matrix(E[:3, :3]).as_rotvec())))
    return dt, dr


def project_points(T_C_O: np.ndarray, obj: np.ndarray,
                   K: np.ndarray, D: np.ndarray) -> np.ndarray:
    rvec = Rotation.from_matrix(T_C_O[:3, :3]).as_rotvec().reshape(3, 1)
    tvec = T_C_O[:3, 3].reshape(3, 1)
    proj, _ = cv2.projectPoints(np.asarray(obj, dtype=np.float64), rvec, tvec, K, D)
    return proj.reshape(-1, 2)


def solve_observed_pose(obs: PixelObs, K_map, D_map) -> Optional[np.ndarray]:
    obj = np.asarray(obs.object_points, dtype=np.float64).reshape(-1, 3)
    img = np.asarray(obs.image_points, dtype=np.float64).reshape(-1, 2)
    if len(obj) < 4:
        return None
    planar = float(np.ptp(obj[:, 2])) < 1e-9
    flag = cv2.SOLVEPNP_IPPE if planar else cv2.SOLVEPNP_ITERATIVE
    try:
        ok, rv, tv = cv2.solvePnP(obj, img, K_map[obs.cam], D_map[obs.cam], flags=flag)
    except Exception:
        return None
    if not ok:
        return None
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = cv2.Rodrigues(rv)[0]
    T[:3, 3] = np.asarray(tv, dtype=np.float64).reshape(3)
    if not np.all(np.isfinite(T)) or float(T[2, 3]) <= 0.0:
        return None
    return T


def load_board_pixel_observations(root: str, meta: dict, all_cam_ids: Sequence[int],
                                  gripper_cam_idx: int) -> List[PixelObs]:
    detector = CharucoTarget(CharucoBoardConfig())
    out: List[PixelObs] = []
    for cap in meta.get("captures", []):
        eid = int(cap.get("event_id", -1))
        sidx = get_capture_set_index(cap)
        if eid < 0:
            continue
        for ci_raw, cinfo in cap.get("cams", {}).items():
            ci = int(ci_raw)
            if ci not in all_cam_ids or not cinfo.get("saved"):
                continue
            # Metadata corner count is a cheap exact-negative filter.
            if int(cinfo.get("charuco_detect_n", 0) or 0) < 4:
                continue
            rgb_rel = cinfo.get("rgb_path", "")
            img = cv2.imread(os.path.join(root, rgb_rel)) if rgb_rel else None
            if img is None:
                continue
            ch_corners, ch_ids, n, _, _ = detector.detect(img)
            if ch_corners is None or ch_ids is None or n < 4:
                continue
            try:
                obj, pix = detector.board.matchImagePoints(ch_corners, ch_ids)
            except Exception:
                obj, pix = None, None
            if obj is None or pix is None or len(obj) < 4:
                continue
            out.append(PixelObs(
                marker="board", cam=ci, event=eid,
                set_idx=None if sidx is None else int(sidx),
                object_points=np.asarray(obj, dtype=np.float64).reshape(-1, 3),
                image_points=np.asarray(pix, dtype=np.float64).reshape(-1, 2),
            ))
    return out


def align_fk_from_train(meta: dict, train_sets: Sequence[int],
                        visual_cube_init: Dict[int, np.ndarray]):
    """Estimate the cube-center -> tag-object constant from training sets only."""
    raw_all = s3.load_nominal_set_cube_transforms(meta)
    raw_train = {int(s): raw_all[int(s)] for s in train_sets if int(s) in raw_all}
    estimated_train = {int(s): visual_cube_init[int(s)] for s in train_sets
                       if int(s) in visual_cube_init}
    delta, _, diag = s3.estimate_set_cube_prior_alignment(raw_train, estimated_train)
    if delta is None:
        raise RuntimeError("train-only FK/object-frame alignment failed")
    aligned_all = {
        int(s): np.asarray(T, dtype=np.float64) @ np.asarray(delta, dtype=np.float64)
        for s, T in raw_all.items()
    }
    return aligned_all, diag


def visual_initial_poses(pixel_obs: Sequence[PixelObs], marker: str,
                         set_ids: Sequence[int], cams: Dict[int, np.ndarray],
                         gTc: np.ndarray, robot_T: Dict[int, np.ndarray],
                         gripper_cam_idx: int, K_map, D_map) -> Dict[int, np.ndarray]:
    by_set: Dict[int, List[np.ndarray]] = {int(s): [] for s in set_ids}
    for o in pixel_obs:
        if o.marker != marker or o.set_idx not in by_set:
            continue
        T_C_O = solve_observed_pose(o, K_map, D_map)
        if T_C_O is None:
            continue
        if o.cam == gripper_cam_idx and o.event in robot_T:
            by_set[o.set_idx].append(robot_T[o.event] @ gTc @ T_C_O)
        elif o.cam in cams:
            by_set[o.set_idx].append(cams[o.cam] @ T_C_O)
    out = {}
    for s, values in by_set.items():
        if values:
            out[s] = cp.robust_se3_average(values, None)[0]
    return out


def visual_board_initial(pixel_obs: Sequence[PixelObs], cams: Dict[int, np.ndarray],
                         gTc: np.ndarray, robot_T: Dict[int, np.ndarray],
                         gripper_cam_idx: int, K_map, D_map) -> np.ndarray:
    values = []
    for o in pixel_obs:
        if o.marker != "board":
            continue
        T_C_O = solve_observed_pose(o, K_map, D_map)
        if T_C_O is None:
            continue
        if o.cam == gripper_cam_idx and o.event in robot_T:
            values.append(robot_T[o.event] @ gTc @ T_C_O)
        elif o.cam in cams:
            values.append(cams[o.cam] @ T_C_O)
    if not values:
        raise RuntimeError("board initialization unavailable")
    return cp.robust_se3_average(values, None)[0]


class JointReprojectionProblem:
    def __init__(self, mode: str, observations: Sequence[PixelObs], cam_ids: Sequence[int],
                 gripper_cam_idx: int, robot_T: Dict[int, np.ndarray], K_map, D_map,
                 fixed_cube: Dict[int, np.ndarray], init_cams: Dict[int, np.ndarray],
                 init_gTc: np.ndarray, init_board: np.ndarray,
                 init_cube: Optional[Dict[int, np.ndarray]] = None):
        if mode not in ("strict_none", "fk_fixed"):
            raise ValueError(mode)
        self.mode = mode
        self.obs = list(observations)
        self.cam_ids = [int(ci) for ci in cam_ids if int(ci) in init_cams]
        self.gripper = int(gripper_cam_idx)
        self.robot_T = robot_T
        self.K = K_map
        self.D = D_map
        self.fixed_cube = {int(s): np.asarray(T, float) for s, T in fixed_cube.items()}
        self.set_ids = sorted({int(o.set_idx) for o in self.obs
                               if o.marker == "cube" and o.set_idx is not None})
        self.slices: Dict[Tuple[str, int], slice] = {}
        k = 0
        for ci in self.cam_ids:
            self.slices[("cam", ci)] = slice(k, k + 6); k += 6
        self.slices[("gtc", -1)] = slice(k, k + 6); k += 6
        self.slices[("board", -1)] = slice(k, k + 6); k += 6
        if mode == "strict_none":
            for s in self.set_ids:
                self.slices[("cube", s)] = slice(k, k + 6); k += 6
        self.n_params = k
        chunks = [T_to_vec(init_cams[ci]) for ci in self.cam_ids]
        chunks += [T_to_vec(init_gTc), T_to_vec(init_board)]
        if mode == "strict_none":
            if init_cube is None or any(s not in init_cube for s in self.set_ids):
                raise RuntimeError("strict-none visual cube initialization incomplete")
            chunks += [T_to_vec(init_cube[s]) for s in self.set_ids]
        self.x0 = np.concatenate(chunks)
        self.row_offsets = []
        row = 0
        for o in self.obs:
            n = int(np.asarray(o.image_points).reshape(-1, 2).shape[0] * 2)
            self.row_offsets.append((row, row + n))
            row += n
        self.n_residuals = row

    def unpack(self, x):
        cams = {ci: vec_to_T(x[self.slices[("cam", ci)]]) for ci in self.cam_ids}
        gtc = vec_to_T(x[self.slices[("gtc", -1)]])
        board = vec_to_T(x[self.slices[("board", -1)]])
        if self.mode == "strict_none":
            cubes = {s: vec_to_T(x[self.slices[("cube", s)]]) for s in self.set_ids}
        else:
            cubes = {s: self.fixed_cube[s] for s in self.set_ids}
        return cams, gtc, board, cubes

    def residual(self, x):
        cams, gtc, board, cubes = self.unpack(x)
        out = []
        for o in self.obs:
            obj_pose = board if o.marker == "board" else cubes[int(o.set_idx)]
            if o.cam == self.gripper:
                T_B_C = self.robot_T[o.event] @ gtc
            else:
                T_B_C = cams[o.cam]
            pred = project_points(inv_T(T_B_C) @ obj_pose, o.object_points,
                                  self.K[o.cam], self.D[o.cam])
            out.extend((pred - o.image_points).reshape(-1).tolist())
        return np.asarray(out, dtype=np.float64)

    def sparsity(self):
        S = lil_matrix((self.n_residuals, self.n_params), dtype=np.int8)
        for o, (r0, r1) in zip(self.obs, self.row_offsets):
            key = ("gtc", -1) if o.cam == self.gripper else ("cam", o.cam)
            S[r0:r1, self.slices[key]] = 1
            obj_key = (("board", -1) if o.marker == "board"
                       else (("cube", int(o.set_idx)) if self.mode == "strict_none" else None))
            if obj_key is not None:
                S[r0:r1, self.slices[obj_key]] = 1
        return S.tocsr()


def perturb_x(problem: JointReprojectionProblem, seed: int,
              trans_mm: float, rot_deg: float) -> np.ndarray:
    if seed == 0:
        return problem.x0.copy()
    rng = np.random.default_rng(seed)
    x = problem.x0.copy()
    for sl in problem.slices.values():
        T = vec_to_T(x[sl])
        dT = np.eye(4)
        dT[:3, :3] = Rotation.from_rotvec(
            rng.normal(0.0, np.deg2rad(rot_deg), 3)).as_matrix()
        dT[:3, 3] = rng.normal(0.0, trans_mm / 1000.0, 3)
        x[sl] = T_to_vec(dT @ T)
    return x


def jacobian_diagnostics(jac, n_params: int) -> dict:
    J = jac.toarray() if hasattr(jac, "toarray") else np.asarray(jac, dtype=np.float64)
    singular = np.linalg.svd(J, compute_uv=False)
    smax = float(singular[0]) if len(singular) else 0.0
    tol = float(np.finfo(float).eps * max(J.shape) * smax)
    rank = int(np.sum(singular > tol))
    smin = float(singular[rank - 1]) if rank else 0.0
    cond_j = float(smax / smin) if smin > 0 else float("inf")
    return {
        "shape": [int(J.shape[0]), int(J.shape[1])],
        "rank": rank,
        "n_params": int(n_params),
        "nullity": int(n_params - rank),
        "rank_tolerance": tol,
        "largest_singular_value": smax,
        "smallest_identifiable_singular_value": smin,
        "jacobian_condition_number": cond_j,
        "gauss_newton_hessian_condition_number": float(cond_j ** 2),
        "rank_deficient": bool(rank < n_params),
    }


def evaluate_paths(pixel_obs: Sequence[PixelObs], cams, gtc, robot_T,
                   gripper: int, K_map, D_map, set_filter: Sequence[int]) -> dict:
    allowed = {int(s) for s in set_filter}
    by_event: Dict[int, Dict[str, List[np.ndarray]]] = {}
    for o in pixel_obs:
        if o.marker != "cube" or o.set_idx not in allowed:
            continue
        T_C_O = solve_observed_pose(o, K_map, D_map)
        if T_C_O is None:
            continue
        if o.cam == gripper and o.event in robot_T:
            T = robot_T[o.event] @ gtc @ T_C_O
            role = "eih"
        elif o.cam in cams:
            T = cams[o.cam] @ T_C_O
            role = "fixed"
        else:
            continue
        by_event.setdefault(o.event, {"fixed": [], "eih": []})[role].append(T)

    cross_raw, e2e_raw = [], []
    predicted_by_set: Dict[int, List[np.ndarray]] = {}
    event_to_set = {o.event: int(o.set_idx) for o in pixel_obs
                    if o.marker == "cube" and o.set_idx is not None}
    for eid, groups in by_event.items():
        fixed = groups["fixed"]
        for i in range(len(fixed)):
            for j in range(i + 1, len(fixed)):
                cross_raw.append(pose_delta(fixed[i], fixed[j]))
        if fixed and groups["eih"]:
            Tf = cp.robust_se3_average(fixed, None)[0]
            for Tg in groups["eih"]:
                e2e_raw.append(pose_delta(Tf, Tg))
        all_values = fixed + groups["eih"]
        if all_values and eid in event_to_set:
            predicted_by_set.setdefault(event_to_set[eid], []).extend(all_values)

    def rms(values):
        return None if not values else float(np.sqrt(np.mean(np.square(values))))
    # Historical versions selected these values with a 30 mm / 10 degree gate.
    # That gate depended on the fitted model output and changed the evaluation
    # population across methods.  Future diagnostic reruns retain every pair.
    return {
        "e_cross_translation_rmse_mm": rms([x[0] for x in cross_raw]),
        "e_cross_rotation_rmse_deg": rms([x[1] for x in cross_raw]),
        "e_e2e_translation_rmse_mm": rms([x[0] for x in e2e_raw]),
        "e_e2e_rotation_rmse_deg": rms([x[1] for x in e2e_raw]),
        "raw_e_cross_translation_rmse_mm": rms([x[0] for x in cross_raw]),
        "raw_e_cross_rotation_rmse_deg": rms([x[1] for x in cross_raw]),
        "raw_e_e2e_translation_rmse_mm": rms([x[0] for x in e2e_raw]),
        "raw_e_e2e_rotation_rmse_deg": rms([x[1] for x in e2e_raw]),
        "n_cross_pairs": len(cross_raw),
        "n_e2e_pairs": len(e2e_raw),
        "n_cross_pairs_rejected": 0,
        "n_e2e_pairs_rejected": 0,
        "metric_pose_gate": None,
        "model_dependent_gating": False,
        "predicted_by_set": predicted_by_set,
    }


def heldout_error(predicted_by_set: Dict[int, List[np.ndarray]], fk_by_set) -> dict:
    trans, rot, rows = [], [], []
    for s, values in sorted(predicted_by_set.items()):
        if s not in fk_by_set or not values:
            continue
        pred = cp.robust_se3_average(values, None)[0]
        dt, dr = pose_delta(pred, fk_by_set[s])
        trans.append(dt); rot.append(dr)
        rows.append({"set": int(s), "translation_mm": dt, "rotation_deg": dr,
                     "n_predictions": int(len(values))})
    return {
        "translation_rmse_mm": None if not trans else float(np.sqrt(np.mean(np.square(trans)))),
        "rotation_rmse_deg": None if not rot else float(np.sqrt(np.mean(np.square(rot)))),
        "n_sets": len(rows),
        "per_set": rows,
    }


def run_mode(mode: str, observations, common, args) -> dict:
    problem = JointReprojectionProblem(
        mode=mode, observations=observations,
        cam_ids=common["fixed_cam_ids"], gripper_cam_idx=common["gripper"],
        robot_T=common["robot_T"], K_map=common["K"], D_map=common["D"],
        fixed_cube=common["fk_train"], init_cams=common["init_cams"],
        init_gTc=common["init_gtc"], init_board=common["init_board"],
        init_cube=common["init_cube"],
    )
    runs = []
    for seed in range(int(args.num_inits)):
        x0 = perturb_x(problem, seed, args.init_translation_mm, args.init_rotation_deg)
        r0 = problem.residual(x0)
        started = time.perf_counter()
        sol = least_squares(
            problem.residual, x0, method="trf", loss="huber", f_scale=2.0,
            jac_sparsity=problem.sparsity(), max_nfev=int(args.max_nfev),
            xtol=float(args.tol), ftol=float(args.tol), gtol=float(args.tol),
        )
        r1 = problem.residual(sol.x)
        elapsed_s = float(time.perf_counter() - started)
        cams, gtc, board, cubes = problem.unpack(sol.x)
        paths_train = evaluate_paths(common["pixel_obs"], cams, gtc, common["robot_T"],
                                     common["gripper"], common["K"], common["D"],
                                     common["train_sets"])
        paths_test = evaluate_paths(common["pixel_obs"], cams, gtc, common["robot_T"],
                                    common["gripper"], common["K"], common["D"],
                                    common["test_sets"])
        heldout = heldout_error(paths_test.pop("predicted_by_set"), common["fk_all"])
        paths_train.pop("predicted_by_set")
        diag = jacobian_diagnostics(sol.jac, problem.n_params)
        runs.append({
            "seed": seed,
            "converged": bool(sol.success),
            "status": int(sol.status),
            "message": str(sol.message),
            "nfev": int(sol.nfev),
            "optimality": float(sol.optimality),
            "elapsed_s": elapsed_s,
            "initial_reprojection_rmse_px": float(np.sqrt(np.mean(np.square(r0)))),
            "final_reprojection_rmse_px": float(np.sqrt(np.mean(np.square(r1)))),
            "cost": float(sol.cost),
            "jacobian": diag,
            "train_path_metrics": paths_train,
            "test_path_metrics": paths_test,
            "heldout_fk_proxy": heldout,
            "transforms": {
                "cams": {str(ci): T.tolist() for ci, T in cams.items()},
                "gTc": gtc.tolist(),
                "board": board.tolist(),
                "cube": {str(s): T.tolist() for s, T in cubes.items()},
            },
        })

    nominal = runs[0]
    common_keys = [("gTc", None)] + [("cam", ci) for ci in common["fixed_cam_ids"]]
    dispersion = {}
    for kind, ci in common_keys:
        if kind == "gTc":
            T_ref = np.asarray(nominal["transforms"]["gTc"], float)
            values = [np.asarray(r["transforms"]["gTc"], float) for r in runs]
            name = "T_gripper_cam"
        else:
            T_ref = np.asarray(nominal["transforms"]["cams"][str(ci)], float)
            values = [np.asarray(r["transforms"]["cams"][str(ci)], float) for r in runs]
            name = f"T_base_C{ci}"
        ds = [pose_delta(T_ref, T) for T in values]
        dispersion[name] = {
            "translation_std_mm": float(np.std([x[0] for x in ds])),
            "translation_max_mm": float(np.max([x[0] for x in ds])),
            "rotation_std_deg": float(np.std([x[1] for x in ds])),
            "rotation_max_deg": float(np.max([x[1] for x in ds])),
        }
    return {
        "mode": mode,
        "objective": "corner_reprojection_only",
        "backbone_fk_used": True,
        "fk_to_cube": "FK-fixed" if mode == "fk_fixed" else "estimated",
        "fk_to_board": "estimated",
        "fk_in_objective": mode == "fk_fixed",  # Deprecated: means FK→cube.
        "post_correction": False,
        "n_parameters": problem.n_params,
        "n_residuals": problem.n_residuals,
        "runs": runs,
        "nominal": nominal,
        "initialization_dispersion": dispersion,
    }


def main():
    ap = argparse.ArgumentParser(description="Strict-none A2 vs FK-fixed A3 reprojection BA")
    ap.add_argument("--root_folder", default="data/session")
    ap.add_argument("--intrinsics_dir", default="intrinsics")
    ap.add_argument("--calib_dir", default="data/session/calib_out")
    ap.add_argument("--out_dir", default="CP_result/A2_strict_none")
    ap.add_argument("--test_sets", default="0,4,6,12")
    ap.add_argument("--num_inits", type=int, default=5)
    ap.add_argument("--init_translation_mm", type=float, default=5.0)
    ap.add_argument("--init_rotation_deg", type=float, default=1.0)
    ap.add_argument("--max_nfev", type=int, default=300)
    ap.add_argument("--tol", type=float, default=1e-8)
    args = ap.parse_args()

    with open(os.path.join(args.root_folder, "meta.json")) as f:
        meta = json.load(f)
    transforms = load_calib_dir(args.calib_dir)
    all_cam_ids = sorted({int(k) for c in meta.get("captures", []) for k in c.get("cams", {})})
    gripper = int(meta.get("gripper_cam_idx"))
    fixed_cam_ids = [ci for ci in all_cam_ids if ci != gripper and f"T_base_C{ci}" in transforms]
    K_map, D_map = {}, {}
    for ci in all_cam_ids:
        K_map[ci], D_map[ci], _ = load_intrinsics_with_depth_scale(args.intrinsics_dir, ci)
    robot_T = s3.load_robot_poses_from_meta(meta)
    cfg, cfg_source = resolve_cube_config_for_run(
        args.root_folder, calib_dir=args.calib_dir, default_cfg=get_default_cube_config())
    cube = AprilTagCubeTarget(cfg)
    cube_corner, cube_reason = cp.detect_corner_observations(
        root=args.root_folder, meta=meta, cube=cube, K_map=K_map, D_map=D_map,
        all_cam_ids=all_cam_ids, gripper_cam_idx=gripper,
        max_err_fixed=3.0, max_err_gripper=5.0,
        min_aspect_fixed=0.0, min_aspect_gripper=0.35,
        exclude_gripped=True,
    )
    pixel_obs = [PixelObs(
        marker="cube", cam=int(o.cam), event=int(o.event), set_idx=o.set_idx,
        object_points=np.asarray(o.object_points, float),
        image_points=np.asarray(o.image_points, float),
    ) for o in cube_corner
        if (int(o.cam) == gripper or len(np.asarray(o.object_points).reshape(-1, 3)) >= 8)]
    board_obs = load_board_pixel_observations(args.root_folder, meta, all_cam_ids, gripper)
    pixel_obs.extend(board_obs)

    all_sets = sorted({int(o.set_idx) for o in pixel_obs
                       if o.marker == "cube" and o.set_idx is not None})
    test_sets = sorted({int(x) for x in args.test_sets.split(",") if x.strip()} & set(all_sets))
    train_sets = [s for s in all_sets if s not in test_sets]
    train_events = {int(c["event_id"]) for c in meta.get("captures", [])
                    if get_capture_set_index(c) in train_sets}
    train_obs = [o for o in pixel_obs if o.event in train_events and
                 (o.marker == "board" or o.set_idx in train_sets)]
    init_cams = {ci: np.asarray(transforms[f"T_base_C{ci}"], float) for ci in fixed_cam_ids}
    init_gtc = np.asarray(transforms["T_gripper_cam"], float)
    init_board = visual_board_initial(train_obs, init_cams, init_gtc, robot_T,
                                      gripper, K_map, D_map)
    init_cube = visual_initial_poses(train_obs, "cube", train_sets, init_cams,
                                     init_gtc, robot_T, gripper, K_map, D_map)
    if any(s not in init_cube for s in train_sets):
        raise RuntimeError("visual-only cube initialization missing for training set")
    fk_all, fk_alignment_diag = align_fk_from_train(meta, train_sets, init_cube)
    if any(s not in fk_all for s in all_sets):
        raise RuntimeError("train-aligned FK missing for one or more observed sets")

    common = {
        "fixed_cam_ids": fixed_cam_ids, "gripper": gripper, "robot_T": robot_T,
        "K": K_map, "D": D_map, "fk_all": fk_all,
        "fk_train": {s: fk_all[s] for s in train_sets},
        "init_cams": init_cams, "init_gtc": init_gtc, "init_board": init_board,
        "init_cube": init_cube, "pixel_obs": pixel_obs,
        "train_sets": train_sets, "test_sets": test_sets,
    }
    result = {
        "protocol": {
            "dataset": args.root_folder,
            "cube_config_source": cfg_source,
            "train_sets": train_sets,
            "test_sets": test_sets,
            "optimizer": "scipy.least_squares(trf, huber, f_scale=2px)",
            "max_nfev": args.max_nfev, "tol": args.tol,
            "num_inits": args.num_inits,
            "init_perturbation": {"translation_mm": args.init_translation_mm,
                                  "rotation_deg": args.init_rotation_deg},
            "n_cube_observations": sum(o.marker == "cube" for o in train_obs),
            "n_board_observations": sum(o.marker == "board" for o in train_obs),
            "cube_corner_reason": cube_reason,
            "fk_object_frame_alignment": {
                "estimated_from_train_sets_only": True,
                "support": int(fk_alignment_diag.get("support", 0)),
                "T_set_cube_center_to_object": fk_alignment_diag.get(
                    "T_set_cube_center_to_object"),
            },
            "post_correction_applied": False,
            "backbone_fk_used_in_all_conditions": True,
            "pose_source_semantics": "FK fields describe target pose only",
        }
    }
    for mode in ("strict_none", "fk_fixed"):
        print(f"[RUN] {mode}")
        result[mode] = run_mode(mode, train_obs, common, args)
        n = result[mode]["nominal"]
        print(f"  converged={n['converged']} reproj={n['final_reprojection_rmse_px']:.4f}px "
              f"heldout={n['heldout_fk_proxy']['translation_rmse_mm']:.3f}mm/"
              f"{n['heldout_fk_proxy']['rotation_rmse_deg']:.3f}deg")

    os.makedirs(args.out_dir, exist_ok=True)
    json_path = os.path.join(args.out_dir, "A2_strict_none_vs_A3_fk_fixed.json")
    with open(json_path, "w") as f:
        json.dump(result, f, indent=2)
    csv_path = os.path.join(args.out_dir, "A2_strict_none_vs_A3_fk_fixed.csv")
    with open(csv_path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["method", "converged", "rank", "nullity", "jac_cond", "hessian_cond",
                    "reproj_px", "heldout_t_mm", "heldout_r_deg", "e2e_t_mm", "e2e_r_deg",
                    "cross_t_mm", "cross_r_deg"])
        for mode in ("strict_none", "fk_fixed"):
            n = result[mode]["nominal"]; j = n["jacobian"]
            h = n["heldout_fk_proxy"]; p = n["test_path_metrics"]
            w.writerow([mode, n["converged"], j["rank"], j["nullity"],
                        j["jacobian_condition_number"], j["gauss_newton_hessian_condition_number"],
                        n["final_reprojection_rmse_px"], h["translation_rmse_mm"],
                        h["rotation_rmse_deg"], p["e_e2e_translation_rmse_mm"],
                        p["e_e2e_rotation_rmse_deg"], p["e_cross_translation_rmse_mm"],
                        p["e_cross_rotation_rmse_deg"]])
    print(f"[SAVE] {json_path}")
    print(f"[SAVE] {csv_path}")


if __name__ == "__main__":
    main()
