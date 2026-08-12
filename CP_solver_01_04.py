#!/usr/bin/env python3
"""Controlled comparison of fixed-camera solvers 01--04.

All methods receive the same train-set cube observations and are evaluated on
the same held-out target sets.  Methods 03 and 04 both start from method 02;
04 is pure corner reprojection (no pose regularizer), and neither optimizer may
silently fall back to method 02.  The primary score is cross-view transfer
reprojection: a held-out cube pose is obtained from one camera measurement and
projected into a different camera.  No fitted output chooses evaluation units.

There is no external physical GT in this real session.  The reported metrics
are held-out multi-view agreement, not absolute camera-pose accuracy.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import os
import time
from collections import defaultdict
from itertools import combinations
from typing import Dict, Mapping, Sequence

import numpy as np

import CP_common as cp
from apriltag_cube import AprilTagCubeTarget, inv_T
from calibration_reprojection_backend import pose_delta, project_points
from calibration_runtime_utils import (
    get_capture_set_index,
    load_intrinsics_with_depth_scale,
    resolve_cube_config_for_run,
)
from config import get_default_cube_config
from cube_config_utils import cube_configs_equivalent, load_cube_config_from_meta


METHOD_ORDER = (
    "01_pnp_mean", "02_pnp_robust_se3",
    "03_pose_consistency", "04_direct_reprojection",
)


def _jsonable(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, (np.integer,)):
        return int(value)
    if isinstance(value, (np.floating,)):
        return float(value)
    if isinstance(value, dict):
        return {str(key): _jsonable(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_jsonable(item) for item in value]
    return value


def parse_ints(value: str):
    return tuple(int(item.strip()) for item in value.split(",") if item.strip())


def observation_set(obs, event_to_set: Mapping[int, int]):
    value = getattr(obs, "set_idx", None)
    return int(value) if value is not None else event_to_set.get(int(obs.event))


def transform_digest(cameras: Mapping[int, np.ndarray], objects: Mapping[int, np.ndarray]) -> str:
    digest = hashlib.sha256()
    for label, values in (("camera", cameras), ("object", objects)):
        for index, transform in sorted(values.items()):
            digest.update(f"{label}:{int(index)}".encode("utf-8"))
            digest.update(np.asarray(transform, dtype="<f8").tobytes())
    return digest.hexdigest()


def load_data(args) -> dict:
    with open(os.path.join(args.root_folder, "meta.json")) as handle:
        meta = json.load(handle)
    cfg, cfg_source = resolve_cube_config_for_run(
        args.root_folder, calib_dir=args.calib_dir,
        default_cfg=get_default_cube_config())
    meta_cfg, _ = load_cube_config_from_meta(args.root_folder, default_cfg=cfg)
    cube = AprilTagCubeTarget(cfg)
    all_cameras = sorted({
        int(camera) for capture in meta.get("captures", [])
        for camera, info in capture.get("cams", {}).items() if info.get("saved")})
    gripper = int(meta["gripper_cam_idx"])
    fixed_cameras = [camera for camera in all_cameras if camera != gripper]
    if len(fixed_cameras) < 2:
        raise RuntimeError("solver comparison needs at least two fixed cameras")
    ref_camera = int(args.ref_camera) if args.ref_camera is not None else fixed_cameras[0]
    if ref_camera not in fixed_cameras:
        raise ValueError(f"reference camera {ref_camera} is not fixed: {fixed_cameras}")
    K_map, D_map = {}, {}
    for camera in all_cameras:
        K_map[camera], D_map[camera], _ = load_intrinsics_with_depth_scale(
            args.intrinsics_dir, camera)
    event_to_set = {}
    for capture in meta.get("captures", []):
        event = int(capture.get("event_id", -1))
        set_index = get_capture_set_index(capture)
        if event >= 0 and set_index is not None:
            event_to_set[event] = int(set_index)
    reuse_stored = cube_configs_equivalent(meta_cfg, cfg)
    pose_obs = cp.load_pose_observations(
        root=args.root_folder,
        meta=meta,
        cube=cube,
        K_map=K_map,
        D_map=D_map,
        all_cam_ids=all_cameras,
        gripper_cam_idx=gripper,
        reuse_stored_cube_candidates=reuse_stored,
        max_err_fixed=float(args.max_err_fixed),
        max_err_gripper=float(args.max_err_gripper),
        min_aspect_fixed=float(args.fixed_cube_min_aspect),
        min_aspect_gripper=0.35,
        gripper_min_markers=1,
        exclude_gripped=True,
        fixed_min_markers=int(args.fixed_min_markers),
    )
    fixed_pose = [obs for obs in pose_obs if int(obs.cam) in fixed_cameras]
    corner_obs, corner_reason = cp.detect_corner_observations(
        root=args.root_folder,
        meta=meta,
        cube=cube,
        K_map=K_map,
        D_map=D_map,
        all_cam_ids=fixed_cameras,
        gripper_cam_idx=gripper,
        max_err_fixed=float(args.max_err_fixed),
        max_err_gripper=float(args.max_err_gripper),
        min_aspect_fixed=float(args.fixed_cube_min_aspect),
        min_aspect_gripper=0.35,
        exclude_gripped=True,
    )
    return {
        "meta": meta,
        "cube_config_source": cfg_source,
        "all_cameras": all_cameras,
        "fixed_cameras": fixed_cameras,
        "gripper": gripper,
        "ref_camera": ref_camera,
        "K_map": K_map,
        "D_map": D_map,
        "event_to_set": event_to_set,
        "pose_obs": fixed_pose,
        "corner_obs": corner_obs,
        "corner_reason": corner_reason,
    }


def build_transfer_units(pose_obs, corner_obs, fixed_cameras: Sequence[int]) -> dict:
    pose_by_key = {}
    corner_by_key = {}
    fixed = {int(camera) for camera in fixed_cameras}
    for obs in pose_obs:
        key = (int(obs.event), int(obs.cam))
        if int(obs.cam) in fixed and key not in pose_by_key:
            pose_by_key[key] = obs
    for obs in corner_obs:
        key = (int(obs.event), int(obs.cam))
        if int(obs.cam) in fixed and key not in corner_by_key:
            corner_by_key[key] = obs
    cameras_by_event = defaultdict(list)
    for event, camera in sorted(set(pose_by_key) & set(corner_by_key)):
        cameras_by_event[event].append(camera)
    ordered_transfer, unordered_cross = [], []
    for event, cameras in sorted(cameras_by_event.items()):
        cameras = sorted(set(cameras))
        for source in cameras:
            for target in cameras:
                if source != target:
                    ordered_transfer.append({
                        "event": event, "source_camera": source,
                        "target_camera": target,
                    })
        for left, right in combinations(cameras, 2):
            unordered_cross.append({
                "event": event, "left_camera": left, "right_camera": right,
            })
    body = {
        "selection_basis": "heldout_measurement_availability_before_solver_fit",
        "model_output_used_for_selection": False,
        "ordered_transfer_units": ordered_transfer,
        "unordered_cross_units": unordered_cross,
    }
    encoded = json.dumps(body, sort_keys=True, separators=(",", ":")).encode("utf-8")
    body["sha256"] = hashlib.sha256(encoded).hexdigest()
    return body


def evaluate_heldout(cameras, pose_obs, corner_obs, transfer_mask, K_map, D_map) -> dict:
    pose_by_key = {(int(obs.event), int(obs.cam)): obs for obs in pose_obs}
    corner_by_key = {(int(obs.event), int(obs.cam)): obs for obs in corner_obs}
    coordinate_residuals = []
    corner_norms = []
    per_pair = defaultdict(list)
    for unit in transfer_mask["ordered_transfer_units"]:
        event = int(unit["event"])
        source = int(unit["source_camera"])
        target = int(unit["target_camera"])
        source_pose = pose_by_key[(event, source)]
        target_corner = corner_by_key[(event, target)]
        T_ref_object = cameras[source] @ np.asarray(source_pose.T_C_O)
        T_target_object = inv_T(cameras[target]) @ T_ref_object
        prediction = project_points(
            T_target_object, target_corner.object_points,
            K_map[target], D_map[target])
        residual = prediction - np.asarray(target_corner.image_points).reshape(-1, 2)
        coordinate_residuals.extend(residual.reshape(-1).tolist())
        norms = np.linalg.norm(residual, axis=1)
        corner_norms.extend(norms.tolist())
        per_pair[f"{source}->{target}"].extend(residual.reshape(-1).tolist())
    cross_values = []
    for unit in transfer_mask["unordered_cross_units"]:
        event = int(unit["event"])
        left = int(unit["left_camera"])
        right = int(unit["right_camera"])
        T_left = cameras[left] @ np.asarray(pose_by_key[(event, left)].T_C_O)
        T_right = cameras[right] @ np.asarray(pose_by_key[(event, right)].T_C_O)
        cross_values.append(pose_delta(T_left, T_right))
    return {
        "transfer_coordinate_rmse_px": float(
            np.sqrt(np.mean(np.square(coordinate_residuals)))),
        "transfer_corner_euclidean_mean_px": float(np.mean(corner_norms)),
        "e_cross_translation_rmse_mm": float(np.sqrt(np.mean(np.square(
            [value[0] for value in cross_values])))),
        "e_cross_rotation_rmse_deg": float(np.sqrt(np.mean(np.square(
            [value[1] for value in cross_values])))),
        "n_ordered_transfer_units": len(transfer_mask["ordered_transfer_units"]),
        "n_cross_units": len(transfer_mask["unordered_cross_units"]),
        "n_corners": len(corner_norms),
        "per_camera_pair_coordinate_rmse_px": {
            pair: float(np.sqrt(np.mean(np.square(values))))
            for pair, values in sorted(per_pair.items())
        },
    }


def train_diagnostics(cameras, objects, pose_obs, corner_obs, fixed_cameras,
                      K_map, D_map) -> dict:
    errors = cp.reprojection_errors(
        corner_obs, cameras, objects, K_map, D_map, list(fixed_cameras))
    pose_t, pose_r = cp.pose_consistency_metrics(
        pose_obs, cameras, objects, list(fixed_cameras))
    return {
        "reprojection_corner_euclidean_rmse_px": (
            float(np.sqrt(np.mean(np.square(errors)))) if errors.size else None),
        "reprojection_corner_euclidean_median_px": (
            float(np.median(errors)) if errors.size else None),
        "pose_translation_rmse_mm": pose_t,
        "pose_rotation_rmse_deg": pose_r,
    }


def run_methods(data: Mapping, train_pose, train_corner, args) -> dict:
    fixed = data["fixed_cameras"]
    ref = data["ref_camera"]
    event_to_set = data["event_to_set"]
    methods = {}

    started = time.perf_counter()
    mean_cameras, _ = cp.build_ref_relative_from_pairwise(
        train_pose, fixed, ref, robust=False)
    mean_objects = cp.initialize_ref_object_poses(
        train_pose, mean_cameras, fixed, ref)
    methods["01_pnp_mean"] = {
        "status": "closed_form", "runtime_s": time.perf_counter() - started,
        "cameras": mean_cameras, "objects": mean_objects,
        "diagnostics": {"aggregation": "unweighted_SE3_mean"},
    }

    started = time.perf_counter()
    robust_cameras, robust_diag = cp.build_ref_relative_from_pairwise(
        train_pose, fixed, ref, robust=True)
    robust_objects = cp.initialize_ref_object_poses(
        train_pose, robust_cameras, fixed, ref)
    methods["02_pnp_robust_se3"] = {
        "status": "closed_form", "runtime_s": time.perf_counter() - started,
        "cameras": robust_cameras, "objects": robust_objects,
        "diagnostics": {"aggregation": "robust_SE3", "pairwise": robust_diag},
    }
    shared_initial_sha = transform_digest(robust_cameras, robust_objects)

    started = time.perf_counter()
    pose_cameras, pose_objects, pose_diag = cp.optimize_pose_consistency(
        pose_obs=train_pose,
        fixed_cam_ids=fixed,
        init_T_cam=robust_cameras,
        init_T_obj=robust_objects,
        ref_cam=ref,
        event_to_set=event_to_set,
        set_priors=None,
        prior_weight_trans=0.0,
        prior_weight_rot=0.0,
        adoption_guard=False,
        max_nfev=int(args.max_nfev),
        tol=float(args.tol),
    )
    methods["03_pose_consistency"] = {
        "status": "converged" if pose_diag.get("optimizer_success") else "unstable",
        "runtime_s": time.perf_counter() - started,
        "cameras": pose_cameras, "objects": pose_objects,
        "diagnostics": pose_diag,
        "shared_initialization_sha256": shared_initial_sha,
    }

    started = time.perf_counter()
    reproj_cameras, reproj_objects, reproj_diag = cp.optimize_reprojection(
        corner_obs=train_corner,
        pose_obs=train_pose,
        fixed_cam_ids=fixed,
        init_T_cam=robust_cameras,
        init_T_obj=robust_objects,
        ref_cam=ref,
        K_map=data["K_map"],
        D_map=data["D_map"],
        event_to_set=event_to_set,
        set_priors=None,
        prior_weight_trans=0.0,
        prior_weight_rot=0.0,
        pose_regularizer_weight=0.0,
        adoption_guard=False,
        max_nfev=int(args.max_nfev),
        tol=float(args.tol),
    )
    methods["04_direct_reprojection"] = {
        "status": "converged" if reproj_diag.get("optimizer_success") else "unstable",
        "runtime_s": time.perf_counter() - started,
        "cameras": reproj_cameras, "objects": reproj_objects,
        "diagnostics": reproj_diag,
        "shared_initialization_sha256": shared_initial_sha,
    }
    return methods


def write_outputs(result: Mapping, out_dir: str) -> None:
    os.makedirs(out_dir, exist_ok=True)
    with open(os.path.join(out_dir, "solver_01_04.json"), "w") as handle:
        json.dump(_jsonable(result), handle, indent=2)
    with open(os.path.join(out_dir, "solver_01_04.csv"), "w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow([
            "method", "status", "heldout_transfer_coordinate_rmse_px",
            "heldout_e_cross_translation_rmse_mm",
            "heldout_e_cross_rotation_rmse_deg", "train_reprojection_rmse_px",
            "runtime_s",
        ])
        for method in METHOD_ORDER:
            entry = result["methods"][method]
            held = entry.get("heldout", {})
            train = entry.get("train", {})
            writer.writerow([
                method, entry["status"], held.get("transfer_coordinate_rmse_px"),
                held.get("e_cross_translation_rmse_mm"),
                held.get("e_cross_rotation_rmse_deg"),
                train.get("reprojection_corner_euclidean_rmse_px"),
                entry.get("runtime_s"),
            ])
    lines = [
        "# Fixed-camera solver comparison 01–04",
        "",
        "All methods use identical train/test observations. Methods 03 and 04 start "
        "from the byte-identical method-02 state; 04 has no pose regularizer and no "
        "fallback. The primary metric transfers a held-out cube pose measured in one "
        "camera into another camera and reports coordinate RMSE.",
        "",
        "These are held-out real-data agreement metrics without external physical GT.",
        "",
        "| Method | status | held-out transfer (px) | e_cross (mm/°) | train reproj (px) | runtime (s) |",
        "| --- | --- | ---: | ---: | ---: | ---: |",
    ]
    for method in METHOD_ORDER:
        entry = result["methods"][method]
        held = entry.get("heldout", {})
        train = entry.get("train", {})
        transfer = held.get("transfer_coordinate_rmse_px")
        cross_t = held.get("e_cross_translation_rmse_mm")
        cross_r = held.get("e_cross_rotation_rmse_deg")
        train_reproj = train.get("reprojection_corner_euclidean_rmse_px")
        if entry["status"] == "unstable":
            transfer_text = "withheld"
            cross_text = "withheld"
        else:
            transfer_text = f"{transfer:.4f}"
            cross_text = f"{cross_t:.3f}/{cross_r:.3f}"
        lines.append(
            f"| {method} | {entry['status']} | {transfer_text} | {cross_text} | "
            f"{train_reproj:.4f} | {entry['runtime_s']:.3f} |")
    lines += [
        "",
        f"Evaluation mask: `{result['protocol']['evaluation_mask_sha256']}`; "
        f"{result['protocol']['n_ordered_transfer_units']} ordered transfers and "
        f"{result['protocol']['n_cross_units']} unordered cross-camera pairs.",
        "",
        "Train reprojection is diagnostic only. It must not replace the held-out "
        "transfer metric when ranking the solvers.",
        "",
    ]
    with open(os.path.join(out_dir, "solver_01_04.md"), "w") as handle:
        handle.write("\n".join(lines))


def parse_args():
    parser = argparse.ArgumentParser(description="Controlled solver 01–04 comparison")
    parser.add_argument("--root_folder", default="data/session")
    parser.add_argument("--intrinsics_dir", default="intrinsics")
    parser.add_argument("--calib_dir", default="data/session/calib_out")
    parser.add_argument("--out_dir", default="CP_result/solver_01_04")
    parser.add_argument("--test_sets", default="0,4,6,12")
    parser.add_argument("--ref_camera", type=int, default=None)
    parser.add_argument("--max_err_fixed", type=float, default=3.0)
    parser.add_argument("--max_err_gripper", type=float, default=5.0)
    parser.add_argument("--fixed_cube_min_aspect", type=float, default=0.0)
    parser.add_argument("--fixed_min_markers", type=int, default=2)
    parser.add_argument("--max_nfev", type=int, default=500)
    parser.add_argument("--tol", type=float, default=1e-10)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    data = load_data(args)
    test_sets = set(parse_ints(args.test_sets))
    available_sets = {int(value) for value in data["event_to_set"].values()}
    if not test_sets or not test_sets < available_sets:
        raise ValueError(
            f"test_sets must be a non-empty proper subset of {sorted(available_sets)}")
    train_pose = [
        obs for obs in data["pose_obs"]
        if observation_set(obs, data["event_to_set"]) not in test_sets]
    test_pose = [
        obs for obs in data["pose_obs"]
        if observation_set(obs, data["event_to_set"]) in test_sets]
    train_corner = [
        obs for obs in data["corner_obs"]
        if observation_set(obs, data["event_to_set"]) not in test_sets]
    test_corner = [
        obs for obs in data["corner_obs"]
        if observation_set(obs, data["event_to_set"]) in test_sets]
    transfer_mask = build_transfer_units(
        test_pose, test_corner, data["fixed_cameras"])
    if (not transfer_mask["ordered_transfer_units"]
            or not transfer_mask["unordered_cross_units"]):
        raise RuntimeError("held-out sets do not support cross-camera evaluation")
    print(f"[DATA] train pose/corner={len(train_pose)}/{len(train_corner)}; "
          f"test={len(test_pose)}/{len(test_corner)}")
    print("[RUN] 01–04 with common robust initialization for 03/04")
    methods = run_methods(data, train_pose, train_corner, args)
    for method in METHOD_ORDER:
        entry = methods[method]
        entry["train"] = train_diagnostics(
            entry["cameras"], entry["objects"], train_pose, train_corner,
            data["fixed_cameras"], data["K_map"], data["D_map"])
        if entry["status"] != "unstable":
            entry["heldout"] = evaluate_heldout(
                entry["cameras"], test_pose, test_corner,
                transfer_mask, data["K_map"], data["D_map"])
        else:
            entry["heldout"] = {}
    initial_shas = {
        methods[method].get("shared_initialization_sha256")
        for method in ("03_pose_consistency", "04_direct_reprojection")}
    if len(initial_shas) != 1:
        raise RuntimeError("03 and 04 did not share one initialization artifact")
    result = {
        "protocol": {
            "dataset": args.root_folder,
            "cube_config_source": data["cube_config_source"],
            "fixed_cameras": data["fixed_cameras"],
            "reference_camera": data["ref_camera"],
            "train_sets": sorted(available_sets - test_sets),
            "test_sets": sorted(test_sets),
            "n_train_pose_observations": len(train_pose),
            "n_train_corner_observations": len(train_corner),
            "n_test_pose_observations": len(test_pose),
            "n_test_corner_observations": len(test_corner),
            "primary_metric": "heldout_cross_view_transfer_coordinate_RMSE_px",
            "external_physical_ground_truth": False,
            "metric_interpretation": "multi_view_agreement_not_absolute_accuracy",
            "evaluation_mask_sha256": transfer_mask["sha256"],
            "n_ordered_transfer_units": len(transfer_mask["ordered_transfer_units"]),
            "n_cross_units": len(transfer_mask["unordered_cross_units"]),
            "model_output_used_for_evaluation_selection": False,
            "shared_03_04_initialization_sha256": next(iter(initial_shas)),
            "method_04_pose_regularizer_weight": 0.0,
            "optimizer_adoption_guard": False,
            "optimizer_max_nfev": int(args.max_nfev),
            "optimizer_tol": float(args.tol),
            "corner_detection_reason_if_empty": data["corner_reason"],
        },
        "evaluation_mask": transfer_mask,
        "methods": methods,
    }
    write_outputs(result, args.out_dir)
    print(f"[SAVE] {args.out_dir}/solver_01_04.{{json,csv,md}}")


if __name__ == "__main__":
    main()
