#!/usr/bin/env python3
"""Independently verify e_cross as fixed-camera cube-pose consistency.

The recomputation intentionally does not call ``evaluate_paths_with_common_mask``
and never reads robot FK or hand-eye.  For each frozen held-out camera pair it
computes

    T_base_cube^(c) = T_base_camera^(c) @ T_camera_cube_PnP^(c)

then aggregates pairwise cube-center distance and SO(3) geodesic distance by
RMSE.  The values must match the canonical result JSON exactly within floating
point tolerance and remain invariant to a common base-frame change.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
from types import SimpleNamespace

import numpy as np
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from calibration_pipeline import table1  # noqa: E402
from calibration_pipeline import cross_target as cross_eval  # noqa: E402
from calibration_pipeline.apriltag_cube import inv_T  # noqa: E402
from calibration_pipeline.path_evaluation import (  # noqa: E402
    E_CROSS_CONTRACT,
    E_CROSS_PIXEL_TRANSFER_CONTRACT,
    observation_id,
    solve_observed_pose,
)
from calibration_pipeline.reprojection import project_points  # noqa: E402
from calibration_pipeline.evaluation import deserialize_state  # noqa: E402


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--root_folder", default="data/session02/calib_train")
    parser.add_argument("--intrinsics_dir", default="intrinsics")
    parser.add_argument(
        "--calib_dir", default="data/session02/calib_train/calib_out")
    parser.add_argument("--include_sets", default="5-12")
    parser.add_argument(
        "--table1_result",
        default="CP_result/session02/late_table1/table1_methods.json")
    parser.add_argument(
        "--evaluation_json",
        default=("CP_result/session02/cross_target_evaluation/"
                 "cross_target_evaluation.json"))
    return parser.parse_args()


def preparation_args(args):
    return SimpleNamespace(
        root_folder=args.root_folder,
        intrinsics_dir=args.intrinsics_dir,
        calib_dir=args.calib_dir,
        include_sets=args.include_sets,
        test_fraction=0.2,
        split_seed=20260731,
        min_train_eih_cube_events=3,
        image_scale=1.0,
        num_inits=3,
        init_translation_mm=5.0,
        init_rotation_deg=1.0,
        max_nfev=300,
        tol=1e-8,
        rotation_scale_rad=1.0,
        translation_scale_m=1.0,
        x_scale_mode="jac",
        loss="soft_l1",
        f_scale_px=2.0,
    )


def pair_disagreement(left: np.ndarray, right: np.ndarray) -> tuple[float, float]:
    translation_mm = float(
        np.linalg.norm(left[:3, 3] - right[:3, 3]) * 1000.0)
    relative_rotation = left[:3, :3].T @ right[:3, :3]
    rotation_deg = float(np.degrees(np.linalg.norm(
        Rotation.from_matrix(relative_rotation).as_rotvec())))
    return translation_mm, rotation_deg


def aggregate(values: list[tuple[float, float]]) -> tuple[float, float]:
    array = np.asarray(values, dtype=np.float64)
    return tuple(float(np.sqrt(np.mean(np.square(array[:, index]))))
                 for index in (0, 1))


def main() -> None:
    args = parse_args()
    if any(E_CROSS_CONTRACT[key] for key in (
            "uses_robot_fk", "uses_gripper_camera",
            "uses_nominal_or_ground_truth_cube_pose")):
        raise AssertionError("e_cross contract includes a forbidden reference")
    if any(E_CROSS_PIXEL_TRANSFER_CONTRACT[key] for key in (
            "uses_robot_fk", "uses_gripper_camera", "uses_shared_target_pose",
            "uses_external_ground_truth")):
        raise AssertionError("pixel-transfer contract includes a forbidden reference")

    prepared = table1.prepare_ablation_data(preparation_args(args))
    runs, _, stored_split = cross_eval.load_stored_runs(args.table1_result)
    if prepared.split != stored_split:
        raise AssertionError("stored and reconstructed splits differ")
    expected = json.loads(Path(args.evaluation_json).read_text(encoding="utf-8"))
    by_id = {
        observation_id(observation): observation
        for observation in prepared.test_obs
        if observation.marker == "cube" and observation.set_idx is not None
    }
    mask = prepared.path_evaluation_mask

    # A common coordinate-frame change must not alter a relative consistency
    # metric.  This fixed transform is used only for that invariance check.
    common_frame = np.eye(4, dtype=np.float64)
    common_frame[:3, :3] = Rotation.from_rotvec([0.2, -0.1, 0.15]).as_matrix()
    common_frame[:3, 3] = [0.3, -0.2, 0.1]

    checked_runs = 0
    checked_pairs = 0
    for method, method_runs in runs.items():
        for run_index, run in enumerate(method_runs):
            state = deserialize_state(run["transforms"])
            values = []
            pixel_squared = []
            for pair in mask["cross_pairs"]:
                pair_poses = []
                camera_poses = {}
                for key in (pair["left_observation_id"],
                            pair["right_observation_id"]):
                    observation = by_id[key]
                    camera = int(observation.cam)
                    if camera == prepared.gripper:
                        raise AssertionError("e_cross pair contains the gripper camera")
                    T_camera_cube = solve_observed_pose(
                        observation, prepared.K_map, prepared.D_map)
                    if T_camera_cube is None:
                        raise AssertionError(f"predeclared PnP became invalid: {key}")
                    camera_poses[key] = T_camera_cube
                    pair_poses.append(np.asarray(state.cams[camera]) @ T_camera_cube)
                direct = pair_disagreement(*pair_poses)
                reframed = pair_disagreement(
                    common_frame @ pair_poses[0], common_frame @ pair_poses[1])
                if not np.allclose(direct, reframed, rtol=0.0, atol=1e-10):
                    raise AssertionError("e_cross depends on the chosen base frame")
                values.append(direct)
                left_id = pair["left_observation_id"]
                right_id = pair["right_observation_id"]
                for source_id, destination_id in (
                        (left_id, right_id), (right_id, left_id)):
                    source = by_id[source_id]
                    destination = by_id[destination_id]
                    T_destination_cube = (
                        inv_T(np.asarray(state.cams[int(destination.cam)]))
                        @ np.asarray(state.cams[int(source.cam)])
                        @ camera_poses[source_id]
                    )
                    prediction = project_points(
                        T_destination_cube,
                        destination.object_points,
                        prepared.K_map[int(destination.cam)],
                        prepared.D_map[int(destination.cam)],
                    )
                    measured = np.asarray(
                        destination.image_points, dtype=np.float64).reshape(-1, 2)
                    pixel_squared.extend(
                        np.square(prediction - measured).reshape(-1).tolist())
                checked_pairs += 1
            translation_rmse, rotation_rmse = aggregate(values)
            pixel_rmse = float(np.sqrt(np.mean(pixel_squared)))
            stored = expected["per_run"][method][run_index]["common_path"]
            if not np.isclose(
                    translation_rmse,
                    stored["e_cross_translation_rmse_mm"],
                    rtol=0.0, atol=1e-9):
                raise AssertionError(f"{method}[{run_index}] translation mismatch")
            if not np.isclose(
                    rotation_rmse,
                    stored["e_cross_rotation_rmse_deg"],
                    rtol=0.0, atol=1e-9):
                raise AssertionError(f"{method}[{run_index}] rotation mismatch")
            if not np.isclose(
                    pixel_rmse,
                    stored["cross_view_pixel_transfer_rmse_px"],
                    rtol=0.0, atol=1e-9):
                raise AssertionError(f"{method}[{run_index}] pixel-transfer mismatch")
            checked_runs += 1

    print(
        "OK: e_cross is pairwise fixed-camera cube-pose consistency and "
        "pixel-transfer is bidirectional measurement-only reprojection; "
        f"{checked_runs} runs, {checked_pairs} pairs, no FK/GT/gripper path")


if __name__ == "__main__":
    main()
