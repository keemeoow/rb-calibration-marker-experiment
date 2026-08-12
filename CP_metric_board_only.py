#!/usr/bin/env python3
"""External-GT board-only calibration on the bundled METRIC workcell.

This dataset is four-camera eye-on-base checkerboard data.  It cannot instantiate
the cube, eye-in-hand, or FK-to-cube axes of A0--B3, so the results produced here
are deliberately separate from the seven-row tables.

The runner reports four OpenCV single-camera hand-eye initializers and a Python
compatible reimplementation of the joint multi-camera corner-reprojection model

    T_cam_board[i,c] = T_cam_base[c] T_base_gripper[i] T_gripper_board.

The bundled C++ Allegro implementation is not labelled as executed unless its
binary is actually used; this file is a protocol-compatible reimplementation,
not a bit-for-bit reproduction of its Ceres solver.
"""

from __future__ import annotations

import argparse
import csv
import json
import os
import shutil
import time
from collections import defaultdict
from pathlib import Path
from typing import Dict, List, Mapping, Sequence, Tuple

import cv2
import numpy as np
from scipy.optimize import least_squares

import CP_common as cp
from apriltag_cube import inv_T
from calibration_reprojection_backend import (
    SE3Scaling,
    jacobian_diagnostics,
    pose_delta,
    project_points,
    retract,
)


METHODS = {
    "Tsai-Lenz": cv2.CALIB_HAND_EYE_TSAI,
    "Park-Martin": cv2.CALIB_HAND_EYE_PARK,
    "Horaud": cv2.CALIB_HAND_EYE_HORAUD,
    "Daniilidis": cv2.CALIB_HAND_EYE_DANIILIDIS,
}


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


def read_key_value_yaml(path: Path) -> dict:
    values = {}
    for raw in path.read_text().splitlines():
        line = raw.split("#", 1)[0].strip()
        if not line or ":" not in line:
            continue
        key, value = line.split(":", 1)
        values[key.strip()] = value.strip()
    return values


def load_intrinsics(camera_dir: Path) -> Tuple[np.ndarray, np.ndarray, dict]:
    values = read_key_value_yaml(camera_dir / "intrinsic_pars_file.yaml")
    K = np.array([
        [float(values["fx"]), 0.0, float(values["cx"])],
        [0.0, float(values["fy"]), float(values["cy"])],
        [0.0, 0.0, 1.0],
    ], dtype=np.float64)
    D = np.array([
        float(values.get("dist_k0", 0.0)),
        float(values.get("dist_k1", 0.0)),
        float(values.get("dist_px", 0.0)),
        float(values.get("dist_py", 0.0)),
        float(values.get("dist_k2", 0.0)),
        float(values.get("dist_k3", 0.0)),
        float(values.get("dist_k4", 0.0)),
        float(values.get("dist_k5", 0.0)),
    ], dtype=np.float64)
    return K, D, values


def load_transform(path: Path) -> np.ndarray:
    transform = np.loadtxt(path, dtype=np.float64)
    if transform.shape != (4, 4) or not np.all(np.isfinite(transform)):
        raise ValueError(f"invalid 4x4 transform: {path}")
    return transform


def checkerboard_points(rows: int, columns: int, size_m: float) -> np.ndarray:
    # Matches Detector::getObjectPoints in the bundled implementation.
    return np.asarray([
        [float(column * size_m), float(row * size_m), 0.0]
        for row in range(columns) for column in range(rows)
    ], dtype=np.float64)


def detect_dataset(dataset_dir: Path) -> dict:
    config = read_key_value_yaml(dataset_dir / "CalibrationInfo.yaml")
    n_cameras = int(config["number_of_cameras"])
    rows = int(config["number_of_rows"])
    columns = int(config["number_of_columns"])
    size_m = float(config["size"])
    if int(config["calibration_setup"]) != 1:
        raise ValueError("this runner is only for METRIC eye-on-base setup=1")
    object_points = checkerboard_points(rows, columns, size_m)
    pattern_size = (rows, columns)
    K_map, D_map, observations = {}, {}, []
    robot_by_event: Dict[int, np.ndarray] = {}
    pose_disagreement = []
    detection_counts = {}
    flags = (cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
             | cv2.CALIB_CB_FAST_CHECK)
    start = time.perf_counter()
    for camera in range(1, n_cameras + 1):
        camera_dir = dataset_dir / f"camera{camera}"
        K_map[camera], D_map[camera], _ = load_intrinsics(camera_dir)
        image_paths = sorted((camera_dir / "image").glob("*.png"))
        pose_paths = {path.stem: path for path in (camera_dir / "pose").glob("*.csv")}
        detected = 0
        for image_path in image_paths:
            if image_path.stem not in pose_paths:
                continue
            event = int(image_path.stem)
            robot = load_transform(pose_paths[image_path.stem])
            if event in robot_by_event:
                dt, dr = pose_delta(robot_by_event[event], robot)
                pose_disagreement.append((dt, dr))
            else:
                robot_by_event[event] = robot
            image = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
            if image is None:
                continue
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            found, corners = cv2.findChessboardCorners(gray, pattern_size, flags)
            if not found:
                continue
            corners = cv2.cornerSubPix(
                gray, corners, (11, 11), (-1, -1),
                (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER, 30, 0.1),
            ).reshape(-1, 2).astype(np.float64)
            ok, rvec, tvec, inliers = cv2.solvePnPRansac(
                object_points, corners, K_map[camera], D_map[camera],
                iterationsCount=50, reprojectionError=8.0, confidence=0.99,
                flags=cv2.SOLVEPNP_ITERATIVE)
            if not ok or inliers is None or len(inliers) != len(object_points):
                continue
            try:
                rvec, tvec = cv2.solvePnPRefineLM(
                    object_points, corners, K_map[camera], D_map[camera],
                    rvec, tvec)
            except AttributeError:
                pass
            T_cam_board = np.eye(4, dtype=np.float64)
            T_cam_board[:3, :3] = cv2.Rodrigues(rvec)[0]
            T_cam_board[:3, 3] = np.asarray(tvec).reshape(3)
            observations.append({
                "camera": camera,
                "event": event,
                "image_points": corners,
                "T_cam_board_pnp": T_cam_board,
            })
            detected += 1
        detection_counts[str(camera)] = detected
    elapsed = time.perf_counter() - start
    gt = {
        camera: load_transform(dataset_dir / "GT" / f"gt_cam{camera}.csv")
        for camera in range(1, n_cameras + 1)
    }
    return {
        "config": config,
        "n_cameras": n_cameras,
        "object_points": object_points,
        "K_map": K_map,
        "D_map": D_map,
        "robot_by_event": robot_by_event,
        "observations": observations,
        "ground_truth_T_base_cam": gt,
        "detection_counts": detection_counts,
        "detection_runtime_s": elapsed,
        "robot_pose_cross_camera_max_translation_mm": (
            max((value[0] for value in pose_disagreement), default=0.0)),
        "robot_pose_cross_camera_max_rotation_deg": (
            max((value[1] for value in pose_disagreement), default=0.0)),
    }


def handeye_camera(method: int, observations: Sequence[Mapping], robot_by_event,
                   camera: int) -> np.ndarray:
    selected = [obs for obs in observations if int(obs["camera"]) == int(camera)]
    if len(selected) < 5:
        raise RuntimeError(f"camera {camera}: only {len(selected)} checkerboard poses")
    # For eye-on-base, inv(T_base_gripper) plays the moving-gripper pose.
    base_to_gripper = [inv_T(robot_by_event[int(obs["event"])]) for obs in selected]
    target_to_camera = [np.asarray(obs["T_cam_board_pnp"]) for obs in selected]
    R, t = cv2.calibrateHandEye(
        [T[:3, :3] for T in base_to_gripper],
        [T[:3, 3].reshape(3, 1) for T in base_to_gripper],
        [T[:3, :3] for T in target_to_camera],
        [T[:3, 3].reshape(3, 1) for T in target_to_camera],
        method=method,
    )
    T_base_cam = np.eye(4, dtype=np.float64)
    T_base_cam[:3, :3] = np.asarray(R, dtype=np.float64).reshape(3, 3)
    T_base_cam[:3, 3] = np.asarray(t, dtype=np.float64).reshape(3)
    determinant = float(np.linalg.det(T_base_cam[:3, :3]))
    if not np.all(np.isfinite(T_base_cam)) or determinant <= 0.0:
        raise RuntimeError(f"camera {camera}: invalid hand-eye result det={determinant}")
    return T_base_cam


def estimate_board_offset(observations: Sequence[Mapping], robot_by_event,
                          T_base_cam: Mapping[int, np.ndarray]) -> np.ndarray:
    candidates = []
    for obs in observations:
        camera, event = int(obs["camera"]), int(obs["event"])
        if camera not in T_base_cam or event not in robot_by_event:
            continue
        # inv(G) B C = T_gripper_board.
        candidates.append(
            inv_T(robot_by_event[event]) @ T_base_cam[camera]
            @ np.asarray(obs["T_cam_board_pnp"]))
    if not candidates:
        raise RuntimeError("board-offset initialization has no observations")
    return cp.robust_se3_average(candidates, None)[0]


def reprojection_report(observations, object_points, robot_by_event,
                        T_base_cam, T_gripper_board, K_map, D_map) -> dict:
    coordinate_residuals = []
    corner_norms = []
    by_camera = defaultdict(list)
    for obs in observations:
        camera, event = int(obs["camera"]), int(obs["event"])
        if camera not in T_base_cam:
            continue
        T_cam_board = (inv_T(T_base_cam[camera]) @ robot_by_event[event]
                       @ T_gripper_board)
        prediction = project_points(
            T_cam_board, object_points, K_map[camera], D_map[camera])
        residual = prediction - np.asarray(obs["image_points"]).reshape(-1, 2)
        coordinate_residuals.extend(residual.reshape(-1).tolist())
        norms = np.linalg.norm(residual, axis=1)
        corner_norms.extend(norms.tolist())
        by_camera[camera].extend(residual.reshape(-1).tolist())
    return {
        "coordinate_rmse_px": float(np.sqrt(np.mean(np.square(coordinate_residuals)))),
        "mean_corner_euclidean_px": float(np.mean(corner_norms)),
        "n_observations": len(observations),
        "n_corners": len(corner_norms),
        "per_camera_coordinate_rmse_px": {
            str(camera): float(np.sqrt(np.mean(np.square(values))))
            for camera, values in sorted(by_camera.items())
        },
    }


def gt_report(T_base_cam: Mapping[int, np.ndarray], gt: Mapping[int, np.ndarray]) -> dict:
    per_camera = {}
    for camera in sorted(gt):
        if camera not in T_base_cam:
            continue
        dt, dr = pose_delta(np.asarray(gt[camera]), np.asarray(T_base_cam[camera]))
        per_camera[str(camera)] = {
            "translation_mm": dt, "rotation_deg": dr,
        }
    translations = [item["translation_mm"] for item in per_camera.values()]
    rotations = [item["rotation_deg"] for item in per_camera.values()]
    return {
        "translation_rms_mm": float(np.sqrt(np.mean(np.square(translations)))),
        "rotation_rms_deg": float(np.sqrt(np.mean(np.square(rotations)))),
        "translation_mean_mm": float(np.mean(translations)),
        "rotation_mean_deg": float(np.mean(rotations)),
        "translation_max_mm": float(np.max(translations)),
        "rotation_max_deg": float(np.max(rotations)),
        "per_camera": per_camera,
    }


def run_classical(data: Mapping) -> dict:
    results = {}
    for name, method in METHODS.items():
        started = time.perf_counter()
        try:
            cameras = {
                camera: handeye_camera(
                    method, data["observations"], data["robot_by_event"], camera)
                for camera in range(1, int(data["n_cameras"]) + 1)
            }
            board = estimate_board_offset(
                data["observations"], data["robot_by_event"], cameras)
            results[name] = {
                "status": "converged",
                "T_base_cam": cameras,
                "T_gripper_board": board,
                "external_gt": gt_report(cameras, data["ground_truth_T_base_cam"]),
                "train_reprojection": reprojection_report(
                    data["observations"], data["object_points"],
                    data["robot_by_event"], cameras, board,
                    data["K_map"], data["D_map"]),
                "runtime_s": time.perf_counter() - started,
            }
        except Exception as exc:
            results[name] = {
                "status": "failed", "error": str(exc),
                "runtime_s": time.perf_counter() - started,
            }
    return results


def run_joint_corner(data: Mapping, initial_cameras: Mapping[int, np.ndarray],
                     initial_board: np.ndarray, args) -> dict:
    cameras = sorted(initial_cameras)
    references_cam_base = {
        camera: inv_T(initial_cameras[camera]) for camera in cameras}
    reference_board = np.asarray(initial_board, dtype=np.float64)
    scaling = SE3Scaling(rotation_scale_rad=1.0, translation_scale_m=1.0)

    def unpack(parameters):
        T_cam_base = {}
        offset = 0
        for camera in cameras:
            T_cam_base[camera] = retract(
                references_cam_base[camera], parameters[offset:offset + 6], scaling)
            offset += 6
        board = retract(reference_board, parameters[offset:offset + 6], scaling)
        return T_cam_base, board

    def residual(parameters):
        T_cam_base, board = unpack(parameters)
        chunks = []
        for obs in data["observations"]:
            camera, event = int(obs["camera"]), int(obs["event"])
            T_cam_board = T_cam_base[camera] @ data["robot_by_event"][event] @ board
            prediction = project_points(
                T_cam_board, data["object_points"],
                data["K_map"][camera], data["D_map"][camera])
            chunks.append(
                (prediction - np.asarray(obs["image_points"]).reshape(-1, 2)).reshape(-1))
        return np.concatenate(chunks)

    x0 = np.zeros(6 * (len(cameras) + 1), dtype=np.float64)
    initial_residual = residual(x0)
    started = time.perf_counter()
    solution = least_squares(
        residual, x0, method="trf", loss="cauchy", f_scale=1.0,
        x_scale="jac", max_nfev=int(args.max_nfev),
        xtol=float(args.tol), ftol=float(args.tol), gtol=float(args.tol))
    runtime = time.perf_counter() - started
    T_cam_base, board = unpack(solution.x)
    T_base_cam = {camera: inv_T(T_cam_base[camera]) for camera in cameras}
    final_residual = residual(solution.x)
    return {
        "status": "converged" if bool(solution.success) else "unstable",
        "success": bool(solution.success),
        "solver_status": int(solution.status),
        "message": str(solution.message),
        "nfev": int(solution.nfev),
        "optimality": float(solution.optimality),
        "loss": "Cauchy(f_scale=1 px)",
        "initial_coordinate_rmse_px": float(np.sqrt(np.mean(np.square(initial_residual)))),
        "final_coordinate_rmse_px": float(np.sqrt(np.mean(np.square(final_residual)))),
        "jacobian": jacobian_diagnostics(
            solution.jac, len(x0),
            variable_keys_=tuple(
                [("T_cam_base", camera) for camera in cameras]
                + [("T_gripper_board", -1)])),
        "T_base_cam": T_base_cam,
        "T_gripper_board": board,
        "external_gt": gt_report(T_base_cam, data["ground_truth_T_base_cam"]),
        "train_reprojection": reprojection_report(
            data["observations"], data["object_points"],
            data["robot_by_event"], T_base_cam, board,
            data["K_map"], data["D_map"]),
        "runtime_s": runtime,
    }


def write_outputs(result: Mapping, out_dir: Path) -> None:
    out_dir.mkdir(parents=True, exist_ok=True)
    with (out_dir / "metric_board_only.json").open("w") as handle:
        json.dump(_jsonable(result), handle, indent=2)
    with (out_dir / "metric_board_only.csv").open("w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow([
            "method", "status", "translation_rms_mm", "rotation_rms_deg",
            "train_coordinate_rmse_px", "views", "runtime_s",
        ])
        for method, entry in result["methods"].items():
            writer.writerow([
                method, entry["status"],
                entry.get("external_gt", {}).get("translation_rms_mm"),
                entry.get("external_gt", {}).get("rotation_rms_deg"),
                entry.get("train_reprojection", {}).get("coordinate_rmse_px"),
                result["protocol"]["n_unique_robot_poses"], entry.get("runtime_s"),
            ])
    lines = [
        "# METRIC medium_workcell — board-only external-GT baseline",
        "",
        "This is a separate four-camera eye-on-base checkerboard experiment. It does "
        "not implement the cube/eih/FK→cube axes of the canonical seven-row ablation.",
        "",
        "GT metrics are RMS over the four `T_base_cam` transform errors. Reprojection "
        "is a train diagnostic, not the external-GT metric.",
        "",
        "| Method | status | e_t RMS (mm) | e_r RMS (°) | train reproj (px) | runtime (s) |",
        "| --- | --- | ---: | ---: | ---: | ---: |",
    ]
    for method, entry in result["methods"].items():
        gt = entry.get("external_gt", {})
        reproj = entry.get("train_reprojection", {})
        lines.append(
            f"| {method} | {entry['status']} | "
            f"{gt.get('translation_rms_mm', float('nan')):.4f} | "
            f"{gt.get('rotation_rms_deg', float('nan')):.4f} | "
            f"{reproj.get('coordinate_rmse_px', float('nan')):.4f} | "
            f"{entry.get('runtime_s', float('nan')):.3f} |")
    lines += [
        "",
        f"Detected views per camera: `{result['protocol']['detection_counts']}`; "
        f"unique robot poses: {result['protocol']['n_unique_robot_poses']}.",
        "",
        "`Joint corner reprojection (compatible)` reproduces the documented eye-on-base "
        "transform chain and shared board variable in Python/SciPy. It is not labelled "
        "as an execution of the bundled Allegro C++/Ceres binary because `cmake` was "
        "unavailable in this environment.",
        "",
    ]
    (out_dir / "metric_board_only.md").write_text("\n".join(lines))


def parse_args():
    parser = argparse.ArgumentParser(description="METRIC board-only external-GT runner")
    parser.add_argument(
        "--dataset_dir",
        default="[]Multi-Camera-Hand-Eye-Calibration-main/data/medium_workcell")
    parser.add_argument("--out_dir", default="CP_result/metric_board_only")
    parser.add_argument("--max_nfev", type=int, default=3000)
    parser.add_argument("--tol", type=float, default=1e-8)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    dataset_dir = Path(args.dataset_dir)
    print("[DETECT] METRIC checkerboard observations")
    data = detect_dataset(dataset_dir)
    print("  detections", data["detection_counts"])
    print("[RUN] classical eye-on-base hand-eye")
    methods = run_classical(data)
    initial_name = "Park-Martin"
    if methods[initial_name]["status"] != "converged":
        initial_name = next(
            name for name, entry in methods.items() if entry["status"] == "converged")
    print(f"[RUN] joint corner reprojection, initializer={initial_name}")
    initial = methods[initial_name]
    methods["Joint corner reprojection (compatible)"] = run_joint_corner(
        data, initial["T_base_cam"], initial["T_gripper_board"], args)
    result = {
        "protocol": {
            "dataset": str(dataset_dir),
            "dataset_role": "external_GT_board_only_eye_on_base",
            "calibration_setup": int(data["config"]["calibration_setup"]),
            "target": {
                "type": data["config"]["pattern_type"],
                "inner_corners": [
                    int(data["config"]["number_of_rows"]),
                    int(data["config"]["number_of_columns"])],
                "square_size_m": float(data["config"]["size"]),
            },
            "number_of_cameras": int(data["n_cameras"]),
            "detection_counts": data["detection_counts"],
            "n_unique_robot_poses": len(data["robot_by_event"]),
            "detection_runtime_s": data["detection_runtime_s"],
            "robot_pose_cross_camera_max_translation_mm": data[
                "robot_pose_cross_camera_max_translation_mm"],
            "robot_pose_cross_camera_max_rotation_deg": data[
                "robot_pose_cross_camera_max_rotation_deg"],
            "ground_truth": "bundled GT/gt_cam*.csv interpreted as T_base_cam",
            "headline_metric": "RMS_SE3_error_over_four_T_base_cam",
            "seven_row_factorial_compatible": False,
            "missing_axes": ["cube", "eye_in_hand", "FK_to_cube"],
            "bundled_cpp_binary_executed": False,
            "cpp_build_blocker": (
                "cmake executable unavailable in the current environment"
                if shutil.which("cmake") is None else None),
            "joint_method_label": "protocol-compatible reimplementation_not_upstream_binary",
        },
        "methods": methods,
    }
    write_outputs(result, Path(args.out_dir))
    print(f"[SAVE] {args.out_dir}/metric_board_only.{{json,csv,md}}")


if __name__ == "__main__":
    main()
