#!/usr/bin/env python3
"""Evaluate every Table 1 row on shared held-out board and cube targets.

The calibration transforms are loaded from stored runs and never refitted.
For a fair cross-target comparison, every row is scored on the same fixed-camera
held-out corners using target poses frozen from training data:

* cube: canonical train-only board-free FK artifact;
* board: canonical train-only eih-board initialization.

These target poses are shared internal references, not external physical GT.
"""

from __future__ import annotations

import argparse
import csv
import json
import os

import numpy as np

from calibration_pipeline import table1
from calibration_pipeline.schema import DEFAULT_SPLIT_SEED
from calibration_pipeline.path_evaluation import (
    E_CROSS_CONTRACT,
    E_CROSS_PIXEL_TRANSFER_CONTRACT,
)
from calibration_pipeline.evaluation import (
    canonical_json_sha256,
    common_target_observation_groups,
    evaluate_common_target_run,
    jsonable,
)


METHOD_ORDER = ("A0", "A1", "A2", "A3", "A4", "B1", "B2", "B3")


def _json_sha256(value) -> str:
    return canonical_json_sha256(value)


def load_stored_runs(path: str):
    with open(path) as handle:
        result = json.load(handle)
    missing = sorted(set(METHOD_ORDER) - set(result.get("rows", {})))
    if missing:
        raise RuntimeError(f"unified Table 1 artifact is missing rows {missing}")
    runs = {method: result["rows"][method]["runs"] for method in METHOD_ORDER}
    return (
        runs,
        result["shared_fk_cube_artifact"]["sha256"],
        result["protocol"]["split"],
    )


def _mean_std(values):
    numeric = [float(value) for value in values if value is not None]
    if not numeric:
        return None, None
    return float(np.mean(numeric)), float(np.std(numeric))


def summarize(method: str, run_results: list[dict], status: str):
    row = {"method": method, "status": status, "n_runs": len(run_results)}
    for group in ("overall", "board", "cube"):
        mean, std = _mean_std([
            result[group]["rmse_px"] for result in run_results])
        row[f"shared_target_{group}_rmse_px_mean"] = mean
        row[f"shared_target_{group}_rmse_px_std"] = std
    row["n_observations"] = run_results[0]["overall"]["n_observations"]
    row["n_corners"] = run_results[0]["overall"]["n_corners"]
    for field in (
        "e_cross_translation_rmse_mm",
        "e_cross_rotation_rmse_deg",
        "e_e2e_translation_rmse_mm",
        "e_e2e_rotation_rmse_deg",
        "cross_view_pixel_transfer_rmse_px",
    ):
        mean, std = _mean_std([
            result["common_path"].get(field) for result in run_results])
        row[f"common_path_{field}_mean"] = mean
        row[f"common_path_{field}_std"] = std
    row["fixed_camera_cube_position_consistency_rmse_mm_mean"] = row[
        "common_path_e_cross_translation_rmse_mm_mean"]
    row["fixed_camera_cube_position_consistency_rmse_mm_std"] = row[
        "common_path_e_cross_translation_rmse_mm_std"]
    row["fixed_camera_cube_rotation_consistency_rmse_deg_mean"] = row[
        "common_path_e_cross_rotation_rmse_deg_mean"]
    row["fixed_camera_cube_rotation_consistency_rmse_deg_std"] = row[
        "common_path_e_cross_rotation_rmse_deg_std"]
    row["n_cross_pairs"] = int(run_results[0]["common_path"]["n_cross_pairs"])
    row["n_cross_view_directions"] = int(
        run_results[0]["common_path"]["n_cross_view_directions"])
    row["n_e2e_units"] = int(run_results[0]["common_path"]["n_e2e_units"])
    return row


def write_outputs(result: dict, output_dir: str) -> None:
    """Write machine-readable sources consumed by the single Table 1 report."""
    os.makedirs(output_dir, exist_ok=True)
    with open(os.path.join(output_dir, "cross_target_evaluation.json"), "w") as handle:
        json.dump(jsonable(result), handle, indent=2)
    fields = list(result["summary"][0])
    with open(os.path.join(output_dir, "cross_target_evaluation.csv"), "w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields, lineterminator="\n")
        writer.writeheader()
        writer.writerows(result["summary"])


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description="Session02 cross-target held-out evaluation")
    parser.add_argument("--root_folder", default="data/session02/calib_train")
    parser.add_argument("--intrinsics_dir", default="intrinsics")
    parser.add_argument("--calib_dir", default="data/session02/calib_train/calib_out")
    parser.add_argument("--include_sets", default="5-12")
    parser.add_argument("--test_fraction", type=float, default=0.2)
    parser.add_argument("--split_seed", type=int, default=DEFAULT_SPLIT_SEED)
    parser.add_argument("--min_train_eih_cube_events", type=int, default=3)
    parser.add_argument("--image_scale", type=float, default=1.0)
    parser.add_argument("--num_inits", type=int, default=3)
    parser.add_argument("--init_translation_mm", type=float, default=5.0)
    parser.add_argument("--init_rotation_deg", type=float, default=1.0)
    parser.add_argument("--max_nfev", type=int, default=300)
    parser.add_argument("--tol", type=float, default=1e-8)
    parser.add_argument("--rotation_scale_rad", type=float, default=1.0)
    parser.add_argument("--translation_scale_m", type=float, default=1.0)
    parser.add_argument("--x_scale_mode", choices=["unit", "jac"], default="jac")
    parser.add_argument("--loss", choices=["huber", "soft_l1", "linear"], default="soft_l1")
    parser.add_argument("--f_scale_px", type=float, default=2.0)
    parser.add_argument(
        "--table1_result",
        default="CP_result/session02/late_table1/table1_methods.json")
    parser.add_argument(
        "--out_dir", default="CP_result/session02/cross_target_evaluation")
    return parser.parse_args(argv)


def main(argv=None) -> None:
    args = parse_args(argv)
    stored_runs, stored_artifact_hash, stored_split = load_stored_runs(
        args.table1_result)
    prepared = table1.prepare_ablation_data(args)
    if prepared.split != stored_split:
        raise RuntimeError("reconstructed split does not match stored results")
    if prepared.alignment_artifact["artifact_sha256"] != stored_artifact_hash:
        raise RuntimeError("reconstructed shared cube artifact does not match stored results")

    common_cameras = None
    for runs in stored_runs.values():
        for run in runs:
            cameras = {int(camera) for camera in run["transforms"]["T_base_Ci"]}
            common_cameras = cameras if common_cameras is None else common_cameras & cameras
    common_cameras = set() if common_cameras is None else common_cameras
    common_cameras.discard(prepared.gripper)
    if not common_cameras:
        raise RuntimeError("no fixed camera is registered by every method/run")

    observations = common_target_observation_groups(
        prepared.test_obs, common_cameras)

    per_run = {}
    summary = []
    for method in METHOD_ORDER:
        print(f"[CROSS-TARGET] {method}")
        per_run[method] = [
            evaluate_common_target_run(
                run["transforms"], common_cameras, observations,
                prepared.board_initial, prepared.fixed_cubes,
                prepared.test_obs, prepared.robot_T, prepared.K_map,
                prepared.D_map, prepared.gripper,
                prepared.path_evaluation_mask)
            for run in stored_runs[method]
        ]
        summary.append(summarize(
            method, per_run[method],
            "preflight_simulation_prior" if method in {"A4", "B1", "B2"} else "complete"))

    support = {}
    for group, group_observations in observations.items():
        support[group] = {
            "n_observations": len(group_observations),
            "n_corners": sum(len(observation.image_points) for observation in group_observations),
            "events": sorted({int(observation.event) for observation in group_observations}),
        }
    result = {
        "artifact_schema": "session02_common_heldout_evaluation_v3",
        "protocol": {
            "question": "frozen calibration transfer to shared held-out board and cube targets",
            "source_data_provenance": prepared.source_data_provenance,
            "pose_convention": prepared.pose_convention,
            "test_time_refit": False,
            "split": stored_split,
            "common_fixed_cameras": sorted(common_cameras),
            "eih_path_excluded": True,
            "eih_exclusion_reason": "shared target pose construction can be circular with the gripper-camera path",
            "cube_pose_source": "canonical train-only board-free FK artifact",
            "cube_pose_artifact_sha256": stored_artifact_hash,
            "board_pose_source": "canonical train-only eih-board hand-eye initialization",
            "board_pose_sha256": _json_sha256(prepared.board_initial),
            "external_ground_truth_used": False,
            "may_be_described_as_absolute_accuracy": False,
            "metric_applicability": {
                "common_target_reprojection_px": {
                    "all_methods": True,
                    "role": "secondary_internal_transfer_metric",
                    "limitation": (
                        "shared train-only target poses are internal references, "
                        "not external ground truth"),
                },
                "cross_view_pixel_transfer_px": {
                    "all_methods": True,
                    "role": "primary_common_inter_camera_pixel_metric",
                    "limitation": (
                        "internal relative consistency; source pose comes from "
                        "measurement-only PnP"),
                },
                "e_cross_pose_consistency": {
                    "all_methods": True,
                    "role": "secondary_fixed_camera_3D_consistency_metric",
                    "limitation": "cannot detect common-mode calibration error",
                },
                "e_e2e_path_closure": {
                    "all_methods": True,
                    "role": "secondary_robot_path_closure_metric",
                    "limitation": (
                        "contains robot FK and is not independent absolute accuracy"),
                },
            },
            "support": support,
            "common_path_evaluation": {
                "evaluation_mask_sha256": prepared.path_evaluation_mask[
                    "evaluation_mask_sha256"],
                "n_cross_pairs": len(prepared.path_evaluation_mask["cross_pairs"]),
                "n_e2e_units": len(prepared.path_evaluation_mask["e2e_units"]),
                "applied_to_every_method": True,
                "test_time_refit": False,
                "e_cross_definition": E_CROSS_CONTRACT,
                "e_cross_pixel_transfer_definition": (
                    E_CROSS_PIXEL_TRANSFER_CONTRACT),
            },
        },
        "summary": summary,
        "per_run": per_run,
    }
    write_outputs(result, args.out_dir)
    print(f"[DONE] {args.out_dir}/cross_target_evaluation.{{json,csv}}")


if __name__ == "__main__":
    main()
