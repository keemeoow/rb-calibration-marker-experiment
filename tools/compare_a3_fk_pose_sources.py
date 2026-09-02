#!/usr/bin/env python3
"""Compare formal A3 raw-FK-fixed and A5 vision-aligned-FK-fixed rows."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np


PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from calibration_pipeline import table1  # noqa: E402
from calibration_pipeline.schema import MAIN_ABLATION_CONDITIONS  # noqa: E402


def _mechanical_cube_center_to_object() -> np.ndarray:
    """Map controller tool4 axes to the configured cube-object axes.

    tool4 and the AprilTag model both place their origin at the cube center.
    The upright grasp has object +Y equal to tool +Y and flips +X/+Z, hence an
    exact 180-degree rotation about Y and no fitted translation.
    """
    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = np.diag([-1.0, 1.0, -1.0])
    return transform


def _summary(runs):
    def values(path):
        output = []
        for run in runs:
            value = run
            for key in path:
                value = value[key]
            output.append(float(value))
        return output

    train = values(("train_reprojection", "overall", "rmse_px"))
    heldout = values(("heldout_reprojection", "overall", "rmse_px"))
    return {
        "converged": f"{sum(bool(run['converged']) for run in runs)}/{len(runs)}",
        "train_reprojection_rmse_px_mean": float(np.mean(train)),
        "heldout_reprojection_rmse_px_mean": float(np.mean(heldout)),
        "per_seed": [{
            "seed": int(run["seed"]),
            "converged": bool(run["converged"]),
            "train_reprojection_rmse_px": train[index],
            "heldout_reprojection_rmse_px": heldout[index],
        } for index, run in enumerate(runs)],
    }


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root-folder", default="data/session04/calib_train")
    parser.add_argument("--intrinsics-dir", default="intrinsics")
    parser.add_argument(
        "--manifest",
        default=("data/session04/calib_out/capture_filter/"
                 "Step2b_observation_manifest.json"))
    parser.add_argument("--include-sets", default="4-12")
    parser.add_argument("--num-inits", type=int, default=3)
    parser.add_argument(
        "--output",
        default=("data/session04/calib_out/verify/"
                 "a3_fk_pose_source_comparison.json"))
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    runner_args = table1.parse_args([
        "--root_folder", args.root_folder,
        "--intrinsics_dir", args.intrinsics_dir,
        "--include_sets", args.include_sets,
        "--observation-manifest", args.manifest,
        "--observation-filter-policy", "standard",
        "--rows", "A3",
        "--num_inits", str(args.num_inits),
    ])
    data = table1.prepare_ablation_data(runner_args)
    conditions = {item.row: item for item in MAIN_ABLATION_CONDITIONS}
    raw_by_set = {
        int(set_index): np.asarray(transform, dtype=np.float64)
        for set_index, transform in data.alignment_artifact[
            "raw_fk_pose_by_set"].items()
        if int(set_index) in set(data.split["eligible_sets"])
    }
    mechanical_delta = _mechanical_cube_center_to_object()
    sources = {
        "A3_pure_raw_fk_with_mechanical_frame_map": conditions["A3"],
        "A5_vision_aligned_fk": conditions["A5"],
    }
    results = {}
    for name, condition in sources.items():
        initial_state, initialization = table1.make_initial_state(
            condition, data.shared_reference_state, data.fixed_cubes,
            data.aligned_fk_cubes)
        runs = [
            table1.run_condition_once(
                condition, initial_state, data.train_obs, data.test_obs,
                data.gripper, data.robot_T, data.K_map, data.D_map, seed,
                runner_args, data.path_evaluation_mask)
            for seed in range(args.num_inits)
        ]
        results[name] = {
            "initialization": initialization,
            "summary": _summary(runs),
        }
    payload = {
        "schema": "a3_a5_fk_pose_source_comparison_v2",
        "mechanical_T_cube_center_tag_object": mechanical_delta.tolist(),
        "mechanical_transform_uses_images": False,
        "vision_aligned_T_cube_center_tag_object": data.alignment_artifact[
            "T_fk_cube_center_to_tag_object"],
        "split": data.split,
        "results": results,
    }
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
    print(output.resolve())


if __name__ == "__main__":
    main()
