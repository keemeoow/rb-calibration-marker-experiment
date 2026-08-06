#!/usr/bin/env python3
"""Export blind-session target predictions from one frozen calibration method.

The output schema is consumed by ``CP_final_external_gt_eval.py``.  No external
GT is read here: predictions are produced only from frozen calibration
transforms, measured blind RGB corners, robot FK and frozen intrinsics.
"""

from __future__ import annotations

import argparse
import json
import os
from collections import defaultdict

import numpy as np

import CP_common as cp
import Step3_calibration as s3
from CP_ablation_7row import detect_observations
from calibration_path_evaluation import solve_observed_pose
from calibration_reprojection_backend import PoseState
from calibration_runtime_utils import load_intrinsics_with_depth_scale


def load_state(path: str, method: str, run_index: int) -> PoseState:
    with open(path) as handle:
        payload = json.load(handle)
    runs = payload.get("methods", {}).get(method)
    if not runs or int(run_index) >= len(runs):
        raise ValueError(f"method/run not found: {method}[{run_index}]")
    transforms = runs[int(run_index)]["transforms"]
    return PoseState(
        cams={int(key): np.asarray(value, dtype=np.float64)
              for key, value in transforms["T_base_Ci"].items()},
        gtc=np.asarray(transforms["T_gripper_cam"], dtype=np.float64),
        board=(None if transforms.get("T_base_board") is None else
               np.asarray(transforms["T_base_board"], dtype=np.float64)),
        cubes={int(key): np.asarray(value, dtype=np.float64)
               for key, value in transforms.get("T_base_cube_by_set", {}).items()},
    )


def pose_id_by_event(meta: dict, preferred_field: str) -> dict:
    mapping = {}
    for capture in meta.get("captures", []):
        event = int(capture["event_id"])
        value = capture.get(preferred_field)
        if value is None:
            value = capture.get("blind_pose_id")
        if value is None:
            value = capture.get("set_index")
        if value is None:
            value = event
        mapping[event] = str(value)
    return mapping


def predict(args) -> dict:
    state = load_state(args.final_methods_json, args.method, args.run_index)
    with open(os.path.join(args.blind_root, "meta.json")) as handle:
        meta = json.load(handle)
    all_cam_ids = sorted({int(ci) for cap in meta.get("captures", [])
                          for ci in cap.get("cams", {})})
    gripper = int(meta["gripper_cam_idx"])
    K_map, D_map = {}, {}
    for camera_id in all_cam_ids:
        K_map[camera_id], D_map[camera_id], _ = load_intrinsics_with_depth_scale(
            args.intrinsics_dir, camera_id)
    robot_T = s3.load_robot_poses_from_meta(meta)

    class DetectionArgs:
        root_folder = args.blind_root
        calib_dir = None
        image_scale = 1.0

    observations, config_source, detection = detect_observations(
        DetectionArgs, meta, K_map, D_map, all_cam_ids, gripper)
    event_to_pose = pose_id_by_event(meta, args.pose_id_field)
    transforms = defaultdict(list)
    view_rows = defaultdict(list)
    for observation in observations:
        if observation.marker != "cube" or int(observation.event) not in event_to_pose:
            continue
        T_cam_cube = solve_observed_pose(observation, K_map, D_map)
        if T_cam_cube is None:
            continue
        if int(observation.cam) == gripper:
            if int(observation.event) not in robot_T:
                continue
            T_base_cam = np.asarray(robot_T[int(observation.event)]) @ state.gtc
        else:
            if int(observation.cam) not in state.cams:
                continue
            T_base_cam = state.cams[int(observation.cam)]
        prediction = T_base_cam @ T_cam_cube
        pose_id = event_to_pose[int(observation.event)]
        transforms[pose_id].append(prediction)
        view_rows[pose_id].append({
            "event_id": int(observation.event),
            "camera_id": int(observation.cam),
            "role": "eih" if int(observation.cam) == gripper else "fixed",
        })

    poses = {}
    for pose_id in sorted(set(event_to_pose.values())):
        values = transforms.get(pose_id, [])
        if not values:
            poses[pose_id] = {
                "status": "failure",
                "reason": "no calibrated camera produced a valid cube PnP",
                "T_base_cube": None,
                "n_views": 0,
            }
            continue
        average, stats = cp.robust_se3_average(values, None)
        poses[pose_id] = {
            "status": "ok",
            "T_base_cube": np.asarray(average).tolist(),
            "n_views": len(values),
            "camera_ids": sorted({row["camera_id"] for row in view_rows[pose_id]}),
            "views": view_rows[pose_id],
            "aggregation": "robust_SE3_average_without_external_GT",
            "dispersion": stats,
        }
    return {
        "artifact_schema": "base_cube_pose_predictions_v1",
        "method": str(args.method),
        "run_index": int(args.run_index),
        "blind_gt_read": False,
        "calibration_source": os.path.abspath(args.final_methods_json),
        "blind_root": os.path.abspath(args.blind_root),
        "intrinsics_dir": os.path.abspath(args.intrinsics_dir),
        "cube_config_source": config_source,
        "cube_detection": detection,
        "registered_camera_count": len(state.cams) + 1,
        "poses": poses,
    }


def parse_args():
    parser = argparse.ArgumentParser(description="Predict blind target poses without reading GT")
    parser.add_argument("--final_methods_json", required=True)
    parser.add_argument("--method", required=True)
    parser.add_argument("--run_index", type=int, default=0)
    parser.add_argument("--blind_root", required=True)
    parser.add_argument("--intrinsics_dir", required=True)
    parser.add_argument("--pose_id_field", default="external_gt_pose_id")
    parser.add_argument("--output", required=True)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    output = predict(args)
    os.makedirs(os.path.dirname(os.path.abspath(args.output)), exist_ok=True)
    with open(args.output, "w") as handle:
        json.dump(output, handle, indent=2)
    print(f"[DONE] {args.output}")


if __name__ == "__main__":
    main()
