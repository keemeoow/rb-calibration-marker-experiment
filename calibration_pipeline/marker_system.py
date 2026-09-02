#!/usr/bin/env python3
"""End-to-end marker-system comparison with modality-specific initialization.

This runner answers a different question from the shared-initialization Table 1:

* Table 1 marker rows remove residuals/variables after one shared initializer
  and therefore measure optimization-level marker contribution.
* This runner initializes and optimizes each system using only its declared
  marker modality: board-only, cube-only, or board+cube.

All systems still share the event split, raw detections, K/D, solver options,
random seeds, held-out observations, and evaluation masks.  The cube-only
initializer is board-free but uses the preregistered train-only robot-FK cube
artifact to initialize hand-eye; its final visual objective has no FK factor.
Pre-GT reporting keeps fixed-to-fixed cross-view as supplementary,
method-specific held-out consistency without FK.  Gripper-to-fixed cross-view
evaluates the full hand-eye+FK+fixed-camera chain.  Both use board/cube image
corners and no shared target pose.  Shared-pose reprojection remains diagnostic.
"""

from __future__ import annotations

import argparse
import csv
import json
import os
from typing import Mapping, Sequence

import numpy as np

from calibration_pipeline.runtime import (
    DEFAULT_SESSION_ROOT, apply_session_defaults)

from calibration_pipeline import table1
from calibration_pipeline.schema import (
    DEFAULT_SPLIT_SEED,
    MARKER_COMPARISON_CONTRACT,
    RELATIVE_POSE_REPORTING_CONTRACT,
)
from calibration_pipeline.path_evaluation import (
    GRIPPER_TO_FIXED_CROSS_TARGET_CONTRACT,
    FIXED_TO_FIXED_CROSS_TARGET_CONTRACT,
    build_gripper_to_fixed_cross_target_mask,
    build_fixed_to_fixed_cross_target_mask,
)
from calibration_pipeline.reprojection import PoseState
from calibration_pipeline.evaluation import (
    REFERENCE_DEPENDENT_REPROJECTION_CONTRACT,
    evaluate_internal_run,
    fixed_camera_board_cube_groups,
    jsonable,
    pixel_reprojection_metrics,
    serialize_state,
    state_sha256,
)


SYSTEM_ORDER = ("board_only", "cube_only", "board_cube")
SYSTEM_LABELS = {
    "board_only": "Board-only end-to-end",
    "cube_only": "Cube-only end-to-end",
    "board_cube": "Board+Cube end-to-end",
}
SYSTEM_MARKERS = {
    "board_only": frozenset({"board"}),
    "cube_only": frozenset({"cube"}),
    "board_cube": frozenset({"board", "cube"}),
}
SYSTEM_FREE_FAMILIES = {
    "board_only": ("T_base_Ci", "T_gripper_cam", "T_base_board"),
    "cube_only": ("T_base_Ci", "T_gripper_cam", "T_base_cube_by_set"),
    "board_cube": (
        "T_base_Ci", "T_gripper_cam", "T_base_board", "T_base_cube_by_set"),
}


def _marker_observations(observations: Sequence, markers: frozenset[str],
                         gripper: int, registered_cameras=None):
    cameras = None if registered_cameras is None else {
        int(camera) for camera in registered_cameras}
    output = []
    for observation in observations:
        if observation.marker not in markers:
            continue
        if (int(observation.cam) != int(gripper)
                and cameras is not None
                and int(observation.cam) not in cameras):
            continue
        output.append(observation)
    return output


def _fixed_observations(observations: Sequence, markers: frozenset[str],
                        gripper: int):
    return [
        observation for observation in observations
        if observation.marker in markers and int(observation.cam) != int(gripper)
    ]


def build_modality_reference_states(data) -> tuple[dict[str, PoseState], dict]:
    """Create a separate train-only initialization pipeline per marker system."""
    board_cameras, board_camera_sources = table1.estimate_fixed_camera_initials(
        _fixed_observations(data.train_obs, SYSTEM_MARKERS["board_only"], data.gripper),
        data.board_initial,
        {},
        data.K_map,
        data.D_map,
        data.gripper,
    )
    if not board_cameras:
        raise RuntimeError("board-only system could not initialize a fixed camera")
    board_state = PoseState(
        cams=board_cameras,
        gtc=np.asarray(data.board_gtc, dtype=np.float64),
        board=np.asarray(data.board_initial, dtype=np.float64),
        cubes={},
    )

    # Board-free cube initialization: the artifact consumes only train eih-cube
    # corners, raw set FK, and robot poses.  FK is an initializer here, not a
    # residual or fixed target in the final cube-only visual optimization.
    cube_gtc = np.asarray(data.fixed_gtc_initial, dtype=np.float64)
    visual_cubes = table1.average_visual_target(
        data.train_obs, "cube", cube_gtc, data.robot_T,
        data.K_map, data.D_map, data.gripper)
    missing = sorted(set(data.split["eligible_sets"]) - set(visual_cubes))
    if missing:
        raise RuntimeError(
            f"cube-only visual initialization is missing sets {missing}")
    cube_cameras, cube_camera_sources = table1.estimate_fixed_camera_initials(
        _fixed_observations(data.train_obs, SYSTEM_MARKERS["cube_only"], data.gripper),
        None,
        visual_cubes,
        data.K_map,
        data.D_map,
        data.gripper,
    )
    if not cube_cameras:
        raise RuntimeError("cube-only system could not initialize a fixed camera")
    cube_state = PoseState(
        cams=cube_cameras,
        gtc=cube_gtc,
        board=None,
        cubes={int(set_index): np.asarray(transform, dtype=np.float64)
               for set_index, transform in visual_cubes.items()},
    )

    both_state = data.shared_reference_state.clone()
    states = {
        "board_only": board_state,
        "cube_only": cube_state,
        "board_cube": both_state,
    }
    diagnostics = {
        "board_only": {
            "initialization_markers": ["board"],
            "handeye_source": "train_eih_board_Charuco",
            "fixed_camera_sources": board_camera_sources,
            "reference_state_sha256": state_sha256(board_state),
            "board_information_used": True,
            "cube_information_used": False,
            "fk_cube_artifact_used_for_initialization": False,
        },
        "cube_only": {
            "initialization_markers": ["cube"],
            "handeye_source": "train_only_board_free_FK_cube_artifact",
            "fixed_camera_sources": cube_camera_sources,
            "reference_state_sha256": state_sha256(cube_state),
            "board_information_used": False,
            "cube_information_used": True,
            "fk_cube_artifact_used_for_initialization": True,
            "fk_cube_artifact_sha256": data.alignment_artifact["artifact_sha256"],
            "final_objective_has_fk_factor": False,
        },
        "board_cube": {
            "initialization_markers": ["board", "cube"],
            "handeye_source": "train_eih_board_Charuco",
            "fixed_camera_sources": data.shared_reference_diagnostics[
                "fixed_camera_sources"],
            "reference_state_sha256": state_sha256(both_state),
            "board_information_used": True,
            "cube_information_used": True,
            "fk_cube_artifact_used_for_initialization": False,
        },
    }
    return states, diagnostics


def run_system_once(system: str, initial: PoseState, data,
                    evaluation_cameras: set[int],
                    evaluation_observations: Mapping,
                    fixed_to_fixed_cross_target_mask: Mapping,
                    gripper_to_fixed_cross_target_mask: Mapping,
                    seed: int, args) -> dict:
    markers = SYSTEM_MARKERS[system]
    train = _marker_observations(
        data.train_obs, markers, data.gripper, initial.cams)
    test = _marker_observations(
        data.test_obs, markers, data.gripper, initial.cams)
    state, solver = table1.solve_stage(
        train,
        SYSTEM_FREE_FAMILIES[system],
        initial,
        data.robot_T,
        data.K_map,
        data.D_map,
        data.gripper,
        seed,
        args,
    )
    serialized = serialize_state(state)
    internal_evaluation = evaluate_internal_run(
        serialized, evaluation_cameras, evaluation_observations,
        data.board_initial, data.aligned_fk_cubes,
        list(data.train_obs) + list(data.test_obs), data.robot_T,
        data.K_map, data.D_map, data.gripper, data.path_evaluation_mask,
        fixed_to_fixed_cross_target_mask, gripper_to_fixed_cross_target_mask)
    return {
        "system": system,
        "seed": int(seed),
        "converged": bool(solver.get("success", False)),
        "objective_markers": sorted(markers),
        "solver": solver,
        "train_reprojection": pixel_reprojection_metrics(
            train, state, data.robot_T, data.K_map, data.D_map, data.gripper),
        "heldout_own_reprojection": pixel_reprojection_metrics(
            test, state, data.robot_T, data.K_map, data.D_map, data.gripper),
        "heldout_internal_evaluation": internal_evaluation,
        "transforms": serialized,
    }


def _mean_std(values):
    numeric = [float(value) for value in values if isinstance(value, (int, float))]
    if not numeric:
        return None, None
    return float(np.mean(numeric)), float(np.std(numeric))


def _nested(payload: dict, *keys):
    current = payload
    for key in keys:
        if not isinstance(current, dict):
            return None
        current = current.get(key)
    return current


def summarize(system: str, runs: list[dict], init_diag: dict) -> dict:
    row = {
        "system": system,
        "label": SYSTEM_LABELS[system],
        "initialization_markers": "+".join(init_diag["initialization_markers"]),
        "objective_markers": "+".join(sorted(SYSTEM_MARKERS[system])),
        "initialization_state_sha256": init_diag["reference_state_sha256"],
        "converged_runs": sum(bool(run["converged"]) for run in runs),
        "total_runs": len(runs),
        "n_registered_fixed_cameras": len(runs[0]["transforms"]["T_base_Ci"]),
    }
    metric_paths = {
        "own_heldout_overall_rmse_px": (
            "heldout_own_reprojection", "overall", "rmse_px"),
        "reference_dependent_overall_reprojection_rmse_px": (
            "heldout_internal_evaluation", "reference_dependent_reprojection",
            "overall", "rmse_px"),
        "reference_dependent_board_reprojection_rmse_px": (
            "heldout_internal_evaluation", "reference_dependent_reprojection",
            "board", "rmse_px"),
        "reference_dependent_cube_reprojection_rmse_px": (
            "heldout_internal_evaluation", "reference_dependent_reprojection",
            "cube", "rmse_px"),
        "fk_dependent_e2e_translation_rmse_mm": (
            "heldout_internal_evaluation", "legacy_fk_dependent_cube_path",
            "e_e2e_translation_rmse_mm"),
        "fk_dependent_e2e_rotation_rmse_deg": (
            "heldout_internal_evaluation", "legacy_fk_dependent_cube_path",
            "e_e2e_rotation_rmse_deg"),
    }
    for target in ("board", "cube"):
        for field in (
                "cross_view_pixel_transfer_rmse_px",
                "pose_consistency_translation_rmse_mm",
                "pose_consistency_rotation_rmse_deg"):
            metric_paths[f"fixed_to_fixed_{target}_{field}"] = (
                "heldout_internal_evaluation", "fixed_to_fixed",
                "by_target", target, field)
            metric_paths[f"gripper_to_fixed_{target}_{field}"] = (
                "heldout_internal_evaluation", "gripper_to_fixed",
                "by_target", target, field)
    for label, path in metric_paths.items():
        mean, std = _mean_std([_nested(run, *path) for run in runs])
        row[f"{label}_mean"] = mean
        row[f"{label}_std"] = std
    evaluation = runs[0]["heldout_internal_evaluation"]
    reference_dependent = evaluation["reference_dependent_reprojection"]["overall"]
    row["n_reference_dependent_observations"] = int(
        reference_dependent["n_observations"])
    row["n_reference_dependent_corners"] = int(reference_dependent["n_corners"])
    for target in ("board", "cube"):
        target_result = evaluation["fixed_to_fixed"][
            "by_target"][target]
        row[f"n_fixed_to_fixed_{target}_pairs"] = int(target_result["n_pairs"])
        row[f"n_fixed_to_fixed_{target}_directions"] = int(
            target_result["n_directions"])
        gripper_target = evaluation["gripper_to_fixed"][
            "by_target"][target]
        row[f"n_gripper_to_fixed_{target}_pairs"] = int(gripper_target["n_pairs"])
        row[f"n_gripper_to_fixed_{target}_directions"] = int(
            gripper_target["n_directions"])
    row["n_fk_dependent_e2e_units"] = int(
        evaluation["legacy_fk_dependent_cube_path"]["n_e2e_units"])
    return row


def validate_end_to_end_contract(result: Mapping) -> None:
    """Fail closed if a modality leaks into another system or masks drift."""
    if result.get("artifact_schema") != "marker_system_end_to_end_v5":
        raise ValueError("unexpected end-to-end marker-system schema")
    protocol = result.get("protocol", {})
    if protocol.get("same_split_raw_detections_K_D_solver_seeds_and_evaluation") is not True:
        raise ValueError("end-to-end marker systems do not share an evaluation contract")
    fixed_evaluation = protocol.get("fixed_camera_subsystem_evaluation", {})
    if fixed_evaluation.get("definition") != FIXED_TO_FIXED_CROSS_TARGET_CONTRACT:
        raise ValueError(
            "marker systems lack the fixed-to-fixed board/cube contract")
    if (fixed_evaluation.get("reporting_tier") != "supplementary"
            or fixed_evaluation.get(
                "may_rank_methods_before_external_gt") is not False):
        raise ValueError(
            "marker-system fixed-to-fixed evaluation must remain supplementary")
    if protocol.get("relative_pose_reporting") != \
            RELATIVE_POSE_REPORTING_CONTRACT:
        raise ValueError("marker-system relative-pose reporting policy drift")
    evaluation_mask_sha256 = fixed_evaluation.get("evaluation_mask_sha256")
    if not evaluation_mask_sha256:
        raise ValueError("fixed-to-fixed board/cube mask hash is missing")
    gripper_evaluation = protocol.get(
        "gripper_to_fixed_full_chain_evaluation", {})
    gripper_mask_sha256 = gripper_evaluation.get("evaluation_mask_sha256")
    if (not gripper_mask_sha256
            or gripper_evaluation.get("definition")
            != GRIPPER_TO_FIXED_CROSS_TARGET_CONTRACT):
        raise ValueError("marker systems lack the gripper-to-fixed contract")
    reference_dependent = protocol.get(
        "secondary_reference_dependent_reprojection", {})
    if (reference_dependent.get("definition")
            != REFERENCE_DEPENDENT_REPROJECTION_CONTRACT
            or reference_dependent.get("may_rank_marker_systems") is not False):
        raise ValueError("shared target-pose reprojection was not kept diagnostic")
    initialization = result.get("initialization", {})
    expected_information = {
        "board_only": (True, False),
        "cube_only": (False, True),
        "board_cube": (True, True),
    }
    state_hashes = set()
    for system, (uses_board, uses_cube) in expected_information.items():
        diagnostics = initialization.get(system, {})
        if diagnostics.get("board_information_used") is not uses_board:
            raise ValueError(f"{system}: board initialization leakage")
        if diagnostics.get("cube_information_used") is not uses_cube:
            raise ValueError(f"{system}: cube initialization leakage")
        state_hashes.add(diagnostics.get("reference_state_sha256"))
        for run in result.get("runs", {}).get(system, []):
            if set(run.get("objective_markers", ())) != set(SYSTEM_MARKERS[system]):
                raise ValueError(f"{system}: objective marker contract drift")
            internal = run.get("heldout_internal_evaluation", {})
            fixed_to_fixed = internal.get("fixed_to_fixed", {})
            if (fixed_to_fixed.get("evaluation_mask_sha256")
                    != evaluation_mask_sha256):
                raise ValueError(f"{system}: fixed-to-fixed mask drift")
            if set(fixed_to_fixed.get("by_target", {})) != {"board", "cube"}:
                raise ValueError(f"{system}: board/cube evaluation support drift")
            gripper_to_fixed = internal.get("gripper_to_fixed", {})
            if (gripper_to_fixed.get("evaluation_mask_sha256")
                    != gripper_mask_sha256):
                raise ValueError(f"{system}: gripper-to-fixed mask drift")
            if set(gripper_to_fixed.get("by_target", {})) != {"board", "cube"}:
                raise ValueError(
                    f"{system}: gripper-to-fixed target support drift")
            shared = internal.get("reference_dependent_reprojection", {})
            if shared.get("metric_contract") != REFERENCE_DEPENDENT_REPROJECTION_CONTRACT:
                raise ValueError(f"{system}: shared-reference diagnostic contract drift")
    if None in state_hashes or len(state_hashes) != len(SYSTEM_ORDER):
        raise ValueError("marker systems did not receive distinct modality initializers")


def write_outputs(result: dict, output_dir: str) -> None:
    """Write machine-readable sources consumed by the single Table 1 report."""
    os.makedirs(output_dir, exist_ok=True)
    with open(os.path.join(output_dir, "marker_system_end_to_end.json"), "w") as handle:
        json.dump(jsonable(result), handle, indent=2)
    rows = result["summary"]
    with open(os.path.join(output_dir, "marker_system_end_to_end.csv"), "w", newline="") as handle:
        writer = csv.DictWriter(
            handle, fieldnames=list(rows[0]), lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description="Modality-specific end-to-end marker-system comparison")
    parser.add_argument("--root_folder", default=DEFAULT_SESSION_ROOT)
    parser.add_argument("--intrinsics_dir", default="intrinsics")
    parser.add_argument("--calib_dir", default=None,
                        help="Default: <session>/calib_out from --root_folder.")
    parser.add_argument("--include_sets", default="5-12")
    parser.add_argument("--out_dir", default=None,
                        help="Default: CP_result/<session>/marker_system_end_to_end.")
    parser.add_argument("--test_fraction", type=float, default=0.2)
    parser.add_argument("--split_seed", type=int, default=DEFAULT_SPLIT_SEED)
    parser.add_argument("--min_train_eih_cube_events", type=int, default=3)
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
    table1.add_frame_prune_arguments(parser)
    parser.add_argument("--image_scale", type=float, default=1.0)
    parser.add_argument(
        "--observation-manifest", "--observation_manifest",
        dest="observation_manifest")
    parser.add_argument(
        "--observation-filter-policy", "--observation_filter_policy",
        dest="observation_filter_policy", choices=("standard", "strict"),
        default="standard")
    parser.add_argument(
        "--align-board-metric-scale", "--align_board_metric_scale",
        dest="align_board_metric_scale", action="store_true")
    args = parser.parse_args(argv)
    return apply_session_defaults(args, {'calib_dir': 'calib_dir', 'out_dir': 'marker_system_dir',
         'observation_manifest': 'observation_manifest'})


def main(argv=None) -> None:
    args = parse_args(argv)
    data = table1.prepare_ablation_data(args)
    states, init_diagnostics = build_modality_reference_states(data)
    evaluation_cameras = set.intersection(
        *({int(camera) for camera in state.cams} for state in states.values()))
    evaluation_cameras.discard(data.gripper)
    if not evaluation_cameras:
        raise RuntimeError("no fixed camera is initialized by all marker systems")
    evaluation_observations = fixed_camera_board_cube_groups(
        data.test_obs, evaluation_cameras)
    fixed_to_fixed_cross_target_mask = build_fixed_to_fixed_cross_target_mask(
        data.test_obs,
        evaluation_cameras,
        data.K_map,
        data.D_map,
        set_filter=data.split["eligible_sets"],
    )
    path_observations = list(data.train_obs) + list(data.test_obs)
    event_roles = {
        **{int(event): "train" for event in data.split["train_events"]},
        **{int(event): "heldout" for event in data.split["test_events"]},
    }
    gripper_to_fixed_cross_target_mask = build_gripper_to_fixed_cross_target_mask(
        data.test_obs,
        evaluation_cameras,
        data.gripper,
        data.K_map,
        data.D_map,
        set_filter=data.split["eligible_sets"],
        fixed_anchor_observations=path_observations,
        event_roles=event_roles,
    )

    runs = {system: [] for system in SYSTEM_ORDER}
    for seed in range(int(args.num_inits)):
        for system in SYSTEM_ORDER:
            print(f"[MARKER-E2E] {system} seed={seed}")
            runs[system].append(run_system_once(
                system, states[system], data, evaluation_cameras,
                evaluation_observations, fixed_to_fixed_cross_target_mask,
                gripper_to_fixed_cross_target_mask, seed, args))

    result = {
        "artifact_schema": "marker_system_end_to_end_v5",
        "experiment": "modality_specific_initialization_and_unified_optimization",
        "protocol": {
            "dataset": args.root_folder,
            "intrinsics_dir": args.intrinsics_dir,
            "source_data_provenance": data.source_data_provenance,
            "board_metric_scale": data.board_metric_scale,
            "pose_convention": data.pose_convention,
            "split": data.split,
            "same_split_raw_detections_K_D_solver_seeds_and_evaluation": True,
            "initialization_policy": "marker_modality_specific_train_only",
            "optimization_policy": "same_marker_modality_unified_visual_objective",
            "solver_options": table1.canonical_solver_options(args).to_dict(),
            "frame_prune_refit": table1.canonical_frame_prune_options(
                args).to_dict(),
            "evaluation_fixed_camera_intersection": sorted(evaluation_cameras),
            "camera_intersection_reason": (
                "all marker systems must use identical fixed-camera pairs"),
            "heldout_board_cube_support": {
                group: {
                    "n_observations": len(observations),
                    "n_corners": sum(len(obs.image_points) for obs in observations),
                }
                for group, observations in evaluation_observations.items()
            },
            "fixed_camera_subsystem_evaluation": {
                "role": "method_specific_heldout_consistency",
                "reporting_tier": "supplementary",
                "may_rank_methods_before_external_gt": False,
                "definition": FIXED_TO_FIXED_CROSS_TARGET_CONTRACT,
                "evaluation_mask_sha256": fixed_to_fixed_cross_target_mask[
                    "evaluation_mask_sha256"],
                "support_by_target": fixed_to_fixed_cross_target_mask[
                    "support_by_target"],
            },
            "gripper_to_fixed_full_chain_evaluation": {
                "role": "visual_observation_FK_dependent_full_chain_comparison",
                "definition": GRIPPER_TO_FIXED_CROSS_TARGET_CONTRACT,
                "evaluation_mask_sha256": gripper_to_fixed_cross_target_mask[
                    "evaluation_mask_sha256"],
                "support_by_target": gripper_to_fixed_cross_target_mask[
                    "support_by_target"],
                "uses_robot_fk_in_evaluation": True,
                "uses_shared_target_pose": False,
            },
            "secondary_reference_dependent_reprojection": {
                "definition": REFERENCE_DEPENDENT_REPROJECTION_CONTRACT,
                "may_rank_marker_systems": False,
            },
            "secondary_fk_dependent_path": {
                "evaluation_mask_sha256": data.path_evaluation_mask[
                    "evaluation_mask_sha256"],
                "may_be_described_as_external_GT": False,
            },
            "external_ground_truth_used": False,
            "may_be_described_as_absolute_accuracy": False,
            "relative_pose_reporting": RELATIVE_POSE_REPORTING_CONTRACT,
            "schema_contract": MARKER_COMPARISON_CONTRACT["end_to_end_system"],
            "cube_only_initialization_caveat": (
                "board-free train-only FK cube artifact initializes hand-eye; "
                "the final cube-only visual objective has no FK factor"),
        },
        "initialization": init_diagnostics,
        "runs": runs,
        "summary": [
            summarize(system, runs[system], init_diagnostics[system])
            for system in SYSTEM_ORDER
        ],
    }
    validate_end_to_end_contract(result)
    write_outputs(result, args.out_dir)
    print(f"[DONE] {args.out_dir}/marker_system_end_to_end.{{json,csv}}")


if __name__ == "__main__":
    main()
