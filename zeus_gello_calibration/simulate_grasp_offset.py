#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""zeus_gello_calibration/simulate_grasp_offset.py -- synthetic ground-truth
test of the T_gripper_cube fitting pipeline (fit_grasp_offset.py).

Question this answers: "given our ACTUAL session1 robot trajectory (the real
recorded joints/pose we drove the arm through) and our ACTUAL camera
intrinsics, if we knew the TRUE T_gripper_cube and TRUE camera extrinsics,
how accurately does our fitting algorithm recover them back from pixel
observations of known accuracy (i.e. how much does corner-detection noise
translate into T_gripper_cube error)?"

This is NOT a real-image test -- no cameras or robot are touched. It reuses:
  - the real session1 robot_T (T_base_gripper per capture) from
    data/session1_handheld_fixed_cam/capture_replayed/<idx>/robot.json
  - the real camera intrinsics (K_map, D_map) via fit_grasp_offset's loaders
  - the cube's real marker geometry (AprilTagCubeModel / config.py)
  - the SAME estimate_grasp_offset_one_camera + solve_corner_reprojection
    solver code fit_grasp_offset.py uses

...and replaces only the "detect markers in a real photo" step with:
"analytically project the cube's known corners into each camera with the
GT extrinsics, keep only markers that would actually be visible (face
normal check, mirrors real self-occlusion), add pixel noise".

Ground truth T_gripper_cube / T_base_cam default to the values already fitted
from the real replayed-capture data (pass1_grasp_offset_replayed.json) --
i.e. "assume that fit was exactly correct, then ask: could our pipeline have
recovered it from noisy-but-plausible pixel data on this same trajectory?"
This is a self-consistency/sensitivity study, not proof the real fit is
correct (it can't be, without an independent GT -- see conversation).

Usage:
  python simulate_grasp_offset.py                      # default: sweep noise levels
  python simulate_grasp_offset.py --pixel-noise-std 0.3 --trials 20
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import cv2
import numpy as np
from scipy.spatial.transform import Rotation

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT))

from calibration_pipeline import se3 as cp  # noqa: E402
from calibration_pipeline.apriltag_cube import AprilTagCubeModel, inv_T  # noqa: E402
from calibration_pipeline.config import get_default_cube_config  # noqa: E402
from calibration_pipeline.reprojection import (  # noqa: E402
    PixelObs, PoseState, SolverOptions, solve_corner_reprojection, variable_keys,
)

from fit_grasp_offset import (  # noqa: E402
    LOCAL_CAM_IDS, NOT_GRIPPER_SENTINEL, estimate_grasp_offset_one_camera,
    load_intrinsics_by_label, load_robot_T,
)

DEFAULT_SESSION_ROOT = REPO_ROOT / "zeus_gello_calibration" / "data" / "session1_handheld_fixed_cam"
DEFAULT_FIT_JSON = REPO_ROOT / "zeus_gello_calibration" / "pass1_grasp_offset_replayed.json"
IMAGE_SIZE = (1280, 720)  # RealSense color stream used for this session


def load_gt_from_fit(fit_json: Path):
    d = json.loads(fit_json.read_text())
    T_gripper_cube_gt = np.asarray(d["T_gripper_cube"], dtype=np.float64)
    T_base_cam_gt = {}
    for key, T in d["T_base_cam"].items():
        local_id = int(key.split("_", 1)[0])
        T_base_cam_gt[local_id] = np.asarray(T, dtype=np.float64)
    return T_gripper_cube_gt, T_base_cam_gt


def visible_markers(model: AprilTagCubeModel, T_C_O: np.ndarray):
    ids = sorted(model.cfg.id_to_face.keys())
    out = []
    for mid in ids:
        visible, _score = model.marker_visibility_score(mid, T_C_O)
        if visible:
            out.append(mid)
    return out


def synthesize_observations(robot_T: dict, T_gripper_cube_gt: np.ndarray,
                            T_base_cam_gt: dict, model: AprilTagCubeModel,
                            K_map: dict, D_map: dict, pixel_noise_std: float,
                            fixed_min_corners: int, rng: np.random.Generator):
    observations = []
    for event, T_bg in robot_T.items():
        T_base_cube = T_bg @ T_gripper_cube_gt
        for cam_id, T_base_cam in T_base_cam_gt.items():
            T_C_O = inv_T(T_base_cam) @ T_base_cube
            mids = visible_markers(model, T_C_O)
            if not mids:
                continue
            obj_pts, img_pts = [], []
            K, D = K_map[cam_id], D_map[cam_id]
            rvec, _ = cv2.Rodrigues(T_C_O[:3, :3])
            tvec = T_C_O[:3, 3].reshape(3, 1)
            for mid in mids:
                corners_obj = model.marker_corners_in_rig(mid)  # (4,3) object frame
                pix, _ = cv2.projectPoints(corners_obj, rvec, tvec, K, D)
                pix = pix.reshape(4, 2)
                if np.any(~np.isfinite(pix)):
                    continue
                if np.any((pix[:, 0] < 0) | (pix[:, 0] >= IMAGE_SIZE[0]) |
                          (pix[:, 1] < 0) | (pix[:, 1] >= IMAGE_SIZE[1])):
                    continue
                obj_pts.append(corners_obj)
                img_pts.append(pix)
            if not obj_pts:
                continue
            obj_pts = np.concatenate(obj_pts, axis=0)
            img_pts = np.concatenate(img_pts, axis=0)
            if len(obj_pts) < fixed_min_corners:
                continue
            if pixel_noise_std > 0.0:
                img_pts = img_pts + rng.normal(0.0, pixel_noise_std, size=img_pts.shape)
            observations.append(PixelObs(
                marker="synthetic", cam=cam_id, event=int(event), set_idx=None,
                object_points=obj_pts, image_points=img_pts, grasp_idx=0,
            ))
    return observations


def run_one_trial(robot_T, T_gripper_cube_gt, T_base_cam_gt, model, K_map, D_map,
                  pixel_noise_std, fixed_min_corners, rng):
    observations = synthesize_observations(
        robot_T, T_gripper_cube_gt, T_base_cam_gt, model, K_map, D_map,
        pixel_noise_std, fixed_min_corners, rng)

    grasp_estimates, cam_init = [], {}
    for c in sorted(T_base_cam_gt):
        cube_obs_c = [o for o in observations if int(o.cam) == c]
        if len(cube_obs_c) < 5:
            continue
        try:
            gripper_cube_c, cam_c_base, _diag = estimate_grasp_offset_one_camera(
                cube_obs_c, robot_T, K_map, D_map, c)
        except RuntimeError:
            continue
        grasp_estimates.append(gripper_cube_c)
        cam_init[c] = cam_c_base
    if not grasp_estimates:
        return None

    grasp_init, _ = cp.robust_se3_average(grasp_estimates, None)
    used_cam_ids = sorted(cam_init)
    state = PoseState(cams={c: cam_init[c] for c in used_cam_ids}, gtc=np.eye(4),
                       board=None, cubes={}, grasps={0: grasp_init})
    used_observations = [o for o in observations if int(o.cam) in cam_init]
    keys = variable_keys(["T_base_Ci", "T_gripper_cube_by_grasp"], state)
    final_state, solve_diag = solve_corner_reprojection(
        observations=used_observations, variable_keys_=keys, reference_state=state,
        robot_T=robot_T, K_map=K_map, D_map=D_map,
        gripper_cam_idx=NOT_GRIPPER_SENTINEL, options=SolverOptions(),
    )
    T_fit = final_state.grasps[0]

    err_T = inv_T(T_gripper_cube_gt) @ T_fit
    err_mm = float(np.linalg.norm(err_T[:3, 3]) * 1000.0)
    err_deg = float(np.linalg.norm(Rotation.from_matrix(err_T[:3, :3]).as_rotvec(degrees=True)))
    return {
        "n_observations": len(used_observations),
        "n_cams_used": len(used_cam_ids),
        "reprojection_rmse_px": solve_diag["train_reprojection_rmse_px"],
        "translation_error_mm": err_mm,
        "rotation_error_deg": err_deg,
    }


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--session-root", default=str(DEFAULT_SESSION_ROOT))
    ap.add_argument("--capture-subdir", default="capture_replayed")
    ap.add_argument("--fit-json", default=str(DEFAULT_FIT_JSON),
                    help="이 json의 T_gripper_cube/T_base_cam을 시뮬레이션 GT로 사용")
    ap.add_argument("--zeus-intrinsics-dir", default=str(REPO_ROOT / "intrinsics"))
    ap.add_argument("--ur3-intrinsics-dir", default=str(REPO_ROOT / "ur3_calibration" / "intrinsics"))
    ap.add_argument("--device-map", default=str(REPO_ROOT / "intrinsics" / "device_map.json"))
    ap.add_argument("--fixed-min-corners", type=int, default=8)
    ap.add_argument("--pixel-noise-std", type=float, default=None,
                    help="지정하면 그 값 하나만 --trials번 반복. 안 주면 기본 스윕 실행")
    ap.add_argument("--trials", type=int, default=20)
    ap.add_argument("--seed", type=int, default=0)
    args = ap.parse_args()

    session_root = Path(args.session_root)
    capture_dirs = sorted(p for p in (session_root / args.capture_subdir).iterdir() if p.is_dir())
    capture_indices = [int(p.name) for p in capture_dirs]
    robot_T = load_robot_T(session_root, capture_indices, args.capture_subdir)
    print(f"session root: {session_root}/{args.capture_subdir}  ({len(robot_T)} real recorded poses reused)")

    K_map, D_map = load_intrinsics_by_label(
        Path(args.zeus_intrinsics_dir), Path(args.ur3_intrinsics_dir), Path(args.device_map))

    T_gripper_cube_gt, T_base_cam_gt = load_gt_from_fit(Path(args.fit_json))
    print(f"GT T_gripper_cube (from {args.fit_json}): "
          f"t_mm={np.round(T_gripper_cube_gt[:3,3]*1000,2).tolist()} "
          f"cams={sorted(T_base_cam_gt)}")

    cube_cfg = get_default_cube_config()
    model = AprilTagCubeModel(cube_cfg)

    noise_levels = [args.pixel_noise_std] if args.pixel_noise_std is not None else \
        [0.0, 0.1, 0.3, 0.5, 1.0, 2.0]

    print(f"\n{'pixel_noise_std':>16} {'n_trials':>9} {'reproj_rmse_px':>15} "
          f"{'trans_err_mm(mean/max)':>24} {'rot_err_deg(mean/max)':>22}")
    for sigma in noise_levels:
        rng = np.random.default_rng(args.seed)
        results = []
        for _ in range(args.trials if sigma > 0.0 else 1):
            r = run_one_trial(robot_T, T_gripper_cube_gt, T_base_cam_gt, model,
                              K_map, D_map, sigma, args.fixed_min_corners, rng)
            if r is not None:
                results.append(r)
        if not results:
            print(f"{sigma:16.2f}  (모든 trial 실패)")
            continue
        trans = np.array([r["translation_error_mm"] for r in results])
        rot = np.array([r["rotation_error_deg"] for r in results])
        rmse_px = np.mean([r["reprojection_rmse_px"] for r in results])
        print(f"{sigma:16.2f} {len(results):9d} {rmse_px:15.4f} "
              f"{trans.mean():12.4f} / {trans.max():8.4f} "
              f"{rot.mean():12.4f} / {rot.max():8.4f}")


if __name__ == "__main__":
    main()
