#!/usr/bin/env python3
"""Evaluate every Table 1 row on held-out board and cube observations.

The calibration transforms are loaded from stored runs and never refitted.
Before external GT exists, two scopes are reported.  Fixed-to-fixed transfer is
a supplementary, method-specific held-out consistency diagnostic without FK.
Gripper-to-fixed transfer uses the same measured board/cube corners but
evaluates the full hand-eye+FK chain.

Reprojection against shared train-only board/cube poses is retained as a
secondary diagnostic.  Sharing a reference and observation population does
not make that reference neutral across calibration methods.
"""

from __future__ import annotations

import argparse
import csv
import json
import os
from typing import Mapping

import numpy as np

from calibration_pipeline.runtime import (
    DEFAULT_SESSION_ROOT, apply_session_defaults)

from calibration_pipeline import table1
from calibration_pipeline.schema import (
    DEFAULT_SPLIT_SEED,
    RELATIVE_POSE_REPORTING_CONTRACT,
)
from calibration_pipeline.path_evaluation import (
    E_CROSS_CONTRACT,
    E_CROSS_PIXEL_TRANSFER_CONTRACT,
    GRIPPER_TO_FIXED_CROSS_TARGET_CONTRACT,
    FIXED_TO_FIXED_CROSS_TARGET_CONTRACT,
    build_gripper_to_fixed_cross_target_mask,
    build_fixed_to_fixed_cross_target_mask,
)
from calibration_pipeline.evaluation import (
    REFERENCE_DEPENDENT_REPROJECTION_CONTRACT,
    canonical_json_sha256,
    evaluate_internal_run,
    fixed_camera_board_cube_groups,
    jsonable,
)


METHOD_ORDER = ("A0", "A1", "A2", "A3", "A4", "A5", "B1", "B2", "B3")


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
            result["reference_dependent_reprojection"][group]["rmse_px"]
            for result in run_results])
        row[f"reference_dependent_{group}_reprojection_rmse_px_mean"] = mean
        row[f"reference_dependent_{group}_reprojection_rmse_px_std"] = std
    diagnostic = run_results[0]["reference_dependent_reprojection"]["overall"]
    row["n_reference_dependent_observations"] = diagnostic["n_observations"]
    row["n_reference_dependent_corners"] = diagnostic["n_corners"]

    cross_view_fields = (
        "cross_view_pixel_transfer_rmse_px",
        "pose_consistency_translation_rmse_mm",
        "pose_consistency_rotation_rmse_deg",
    )
    for target in ("board", "cube"):
        for field in cross_view_fields:
            mean, std = _mean_std([
                result["fixed_to_fixed"]["by_target"][target][field]
                for result in run_results])
            row[f"fixed_to_fixed_{target}_{field}_mean"] = mean
            row[f"fixed_to_fixed_{target}_{field}_std"] = std
        target_result = run_results[0][
            "fixed_to_fixed"]["by_target"][target]
        row[f"n_fixed_to_fixed_{target}_pairs"] = int(target_result["n_pairs"])
        row[f"n_fixed_to_fixed_{target}_directions"] = int(
            target_result["n_directions"])

    for target in ("board", "cube"):
        for field in cross_view_fields:
            mean, std = _mean_std([
                result["gripper_to_fixed"]["by_target"][target][field]
                for result in run_results])
            row[f"gripper_to_fixed_{target}_{field}_mean"] = mean
            row[f"gripper_to_fixed_{target}_{field}_std"] = std
        target_result = run_results[0][
            "gripper_to_fixed"]["by_target"][target]
        row[f"n_gripper_to_fixed_{target}_pairs"] = int(target_result["n_pairs"])
        row[f"n_gripper_to_fixed_{target}_directions"] = int(
            target_result["n_directions"])

    for field in (
        "e_cross_translation_rmse_mm",
        "e_cross_rotation_rmse_deg",
        "e_e2e_translation_rmse_mm",
        "e_e2e_rotation_rmse_deg",
        "cross_view_pixel_transfer_rmse_px",
    ):
        mean, std = _mean_std([
            result["legacy_fk_dependent_cube_path"].get(field)
            for result in run_results])
        row[f"legacy_cube_path_{field}_mean"] = mean
        row[f"legacy_cube_path_{field}_std"] = std
    legacy = run_results[0]["legacy_fk_dependent_cube_path"]
    row["n_legacy_cube_cross_pairs"] = int(legacy["n_cross_pairs"])
    row["n_legacy_cube_cross_view_directions"] = int(
        legacy["n_cross_view_directions"])
    row["n_fk_dependent_e2e_units"] = int(legacy["n_e2e_units"])
    return row


def validate_result_contract(result: Mapping) -> None:
    """Fail closed if a reference-dependent value is promoted as neutral."""
    if result.get("artifact_schema") != "internal_heldout_evaluation_v8":
        raise ValueError("unexpected internal held-out evaluation schema")
    protocol = result.get("protocol", {})
    if protocol.get("external_ground_truth_used") is not False:
        raise ValueError("pre-GT evaluation cannot claim external ground truth")
    primary = protocol.get("fixed_to_fixed_evaluation", {})
    mask_sha256 = primary.get("evaluation_mask_sha256")
    if (not mask_sha256
            or primary.get("definition") != FIXED_TO_FIXED_CROSS_TARGET_CONTRACT):
        raise ValueError("fixed-to-fixed board/cube evaluation contract is missing")
    applicability = protocol.get("metric_applicability", {}).get(
        "fixed_to_fixed_board_cube", {})
    if (applicability.get("reporting_tier") != "supplementary"
            or applicability.get(
                "may_rank_methods_before_external_gt") is not False):
        raise ValueError(
            "fixed-to-fixed consistency was promoted above supplementary")
    if protocol.get("relative_pose_reporting") != \
            RELATIVE_POSE_REPORTING_CONTRACT:
        raise ValueError("relative-pose reporting policy drift")
    gripper_to_fixed_protocol = protocol.get("gripper_to_fixed_evaluation", {})
    gripper_mask_sha256 = gripper_to_fixed_protocol.get(
        "evaluation_mask_sha256")
    if (not gripper_mask_sha256
            or gripper_to_fixed_protocol.get("definition")
            != GRIPPER_TO_FIXED_CROSS_TARGET_CONTRACT):
        raise ValueError(
            "gripper-to-fixed board/cube evaluation contract is missing")
    dependent = protocol.get("metric_applicability", {}).get(
        "reference_dependent_reprojection_px", {})
    if dependent.get("may_rank_methods_before_external_gt") is not False:
        raise ValueError("reference-dependent reprojection was promoted for ranking")
    if protocol.get("reference_dependent_reprojection", {}).get(
            "definition") != REFERENCE_DEPENDENT_REPROJECTION_CONTRACT:
        raise ValueError("reference-dependent reprojection contract drift")
    for method, runs in result.get("per_run", {}).items():
        for run in runs:
            fixed_to_fixed = run.get("fixed_to_fixed", {})
            if fixed_to_fixed.get("evaluation_mask_sha256") != mask_sha256:
                raise ValueError(f"{method}: fixed-to-fixed evaluation mask drift")
            if set(fixed_to_fixed.get("by_target", {})) != {"board", "cube"}:
                raise ValueError(f"{method}: board/cube support drift")
            gripper_to_fixed = run.get("gripper_to_fixed", {})
            if (gripper_to_fixed.get("evaluation_mask_sha256")
                    != gripper_mask_sha256):
                raise ValueError(
                    f"{method}: gripper-to-fixed evaluation mask drift")
            if set(gripper_to_fixed.get("by_target", {})) != {"board", "cube"}:
                raise ValueError(
                    f"{method}: gripper-to-fixed target support drift")
            shared = run.get("reference_dependent_reprojection", {})
            if shared.get("metric_contract") != REFERENCE_DEPENDENT_REPROJECTION_CONTRACT:
                raise ValueError(f"{method}: shared-reference contract drift")


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
    parser = argparse.ArgumentParser(
        description=(
            "Pre-GT fixed-to-fixed and gripper-to-fixed board/cube evaluation"))
    parser.add_argument("--root_folder", default=DEFAULT_SESSION_ROOT)
    parser.add_argument("--intrinsics_dir", default="intrinsics")
    parser.add_argument("--calib_dir", default=None,
                        help="Default: <session>/calib_out from --root_folder.")
    parser.add_argument("--include_sets", default="5-12")
    parser.add_argument("--test_fraction", type=float, default=0.2)
    parser.add_argument("--split_seed", type=int, default=DEFAULT_SPLIT_SEED)
    parser.add_argument("--min_train_eih_cube_events", type=int, default=3)
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
        "--table1_result", default=None,
        help="Default: CP_result/<session>/late_table1/table1_methods.json.")
    parser.add_argument(
        "--out_dir", default=None,
        help="Default: CP_result/<session>/cross_target_evaluation.")
    parser.add_argument(
        "--allow-relocated-session-root", "--allow_relocated_session_root",
        dest="allow_relocated_session_root", action="store_true",
        help=("Replay a --observation-manifest captured in another checkout. "
              "Only the recorded absolute path prefix is remapped; every "
              "recorded SHA-256 is still verified against the local files."))
    args = parser.parse_args(argv)
    return apply_session_defaults(args, {'calib_dir': 'calib_dir', 'table1_result': 'table1_result',
         'out_dir': 'cross_target_dir',
         'observation_manifest': 'observation_manifest'})


def main(argv=None) -> None:
    args = parse_args(argv)
    stored_runs, stored_artifact_hash, stored_split = load_stored_runs(
        args.table1_result)
    prepared = table1.prepare_ablation_data(args)
    if prepared.split != stored_split:
        raise RuntimeError("reconstructed split does not match stored results")
    if prepared.alignment_artifact["artifact_sha256"] != stored_artifact_hash:
        raise RuntimeError("reconstructed shared cube artifact does not match stored results")

    evaluation_cameras = None
    for runs in stored_runs.values():
        for run in runs:
            cameras = {int(camera) for camera in run["transforms"]["T_base_Ci"]}
            evaluation_cameras = (
                cameras if evaluation_cameras is None
                else evaluation_cameras & cameras)
    evaluation_cameras = set() if evaluation_cameras is None else evaluation_cameras
    evaluation_cameras.discard(prepared.gripper)
    if not evaluation_cameras:
        raise RuntimeError("no fixed camera is registered by every method/run")

    observations = fixed_camera_board_cube_groups(
        prepared.test_obs, evaluation_cameras)
    fixed_to_fixed_mask = build_fixed_to_fixed_cross_target_mask(
        prepared.test_obs,
        evaluation_cameras,
        prepared.K_map,
        prepared.D_map,
        set_filter=prepared.split["eligible_sets"],
    )
    path_observations = list(prepared.train_obs) + list(prepared.test_obs)
    event_roles = {
        **{int(event): "train" for event in prepared.split["train_events"]},
        **{int(event): "heldout" for event in prepared.split["test_events"]},
    }
    gripper_to_fixed_mask = build_gripper_to_fixed_cross_target_mask(
        prepared.test_obs,
        evaluation_cameras,
        prepared.gripper,
        prepared.K_map,
        prepared.D_map,
        set_filter=prepared.split["eligible_sets"],
        fixed_anchor_observations=path_observations,
        event_roles=event_roles,
    )

    per_run = {}
    summary = []
    for method in METHOD_ORDER:
        print(f"[CROSS-TARGET] {method}")
        per_run[method] = [
            evaluate_internal_run(
                run["transforms"], evaluation_cameras, observations,
                prepared.board_initial, prepared.fixed_cubes,
                path_observations, prepared.robot_T, prepared.K_map,
                prepared.D_map, prepared.gripper,
                prepared.path_evaluation_mask, fixed_to_fixed_mask,
                gripper_to_fixed_mask)
            for run in stored_runs[method]
        ]
        summary.append(summarize(
            method, per_run[method],
            ("preflight_simulation_prior" if method in {"A4", "B1", "B2"}
             else "posthoc_diagnostic" if method == "A5"
             else "complete")))

    support = {}
    for group, group_observations in observations.items():
        support[group] = {
            "n_observations": len(group_observations),
            "n_corners": sum(len(observation.image_points) for observation in group_observations),
            "events": sorted({int(observation.event) for observation in group_observations}),
        }
    result = {
        "artifact_schema": "internal_heldout_evaluation_v8",
        "protocol": {
            "question": (
                "fixed-camera subsystem and gripper-to-fixed full-chain "
                "consistency on held-out board and cube before external GT"),
            "source_data_provenance": prepared.source_data_provenance,
            "board_metric_scale": prepared.board_metric_scale,
            "pose_convention": prepared.pose_convention,
            "test_time_refit": False,
            "split": stored_split,
            "evaluation_fixed_camera_intersection": sorted(evaluation_cameras),
            "camera_intersection_reason": (
                "every method must be evaluated on identical fixed-camera pairs"),
            "eye_in_hand_evaluated": True,
            "eye_in_hand_evaluation_depends_on_robot_fk": True,
            "why_two_camera_scopes_are_reported": (
                "fixed-to-fixed isolates fixed-camera calibration; "
                "gripper-to-fixed evaluates the combined hand-eye, FK, and "
                "fixed-camera chain"),
            "external_ground_truth_used": False,
            "may_be_described_as_absolute_accuracy": False,
            "relative_pose_reporting": RELATIVE_POSE_REPORTING_CONTRACT,
            "metric_applicability": {
                "fixed_to_fixed_board_cube": {
                    "all_methods": True,
                    "role": "method_specific_heldout_consistency",
                    "reporting_tier": "supplementary",
                    "may_rank_methods_before_external_gt": False,
                    "limitation": (
                        "uses each method's fitted camera poses, cannot measure "
                        "absolute physical accuracy, and cannot detect a "
                        "systematic error shared by every fixed camera"),
                },
                "gripper_to_fixed_board_cube": {
                    "all_methods": True,
                    "role": "full_system_internal_chain_metric",
                    "uses_visual_observations": True,
                    "uses_robot_fk_in_prediction": True,
                    "limitation": (
                        "cannot separate hand-eye calibration error from robot "
                        "FK error and is not independent absolute accuracy"),
                },
                "reference_dependent_reprojection_px": {
                    "all_methods": True,
                    "role": "secondary_reference_dependent_diagnostic",
                    "may_rank_methods_before_external_gt": False,
                    "limitation": (
                        "shared train-only target poses are not neutral external "
                        "references and can favor aligned assumptions"),
                },
                "legacy_cube_e_cross_pose_consistency": {
                    "all_methods": True,
                    "role": "method_specific_heldout_consistency",
                    "reporting_tier": "supplementary",
                    "may_rank_methods_before_external_gt": False,
                    "limitation": (
                        "cannot detect a systematic calibration error shared "
                        "by every fixed camera"),
                },
                "fk_dependent_e2e_path_closure": {
                    "all_methods": True,
                    "role": "secondary_robot_path_closure_metric",
                    "limitation": (
                        "contains robot FK and is not independent absolute accuracy"),
                },
            },
            "support": support,
            "fixed_to_fixed_evaluation": {
                "evaluation_mask_sha256": fixed_to_fixed_mask[
                    "evaluation_mask_sha256"],
                "selection_uses_model_output": False,
                "applied_to_every_method": True,
                "test_time_refit": False,
                "definition": FIXED_TO_FIXED_CROSS_TARGET_CONTRACT,
                "support_by_target": fixed_to_fixed_mask["support_by_target"],
            },
            "reference_dependent_reprojection": {
                "definition": REFERENCE_DEPENDENT_REPROJECTION_CONTRACT,
                "cube_pose_source": "canonical train-only board-free FK artifact",
                "cube_pose_artifact_sha256": stored_artifact_hash,
                "board_pose_source": (
                    "canonical train-only eih-board hand-eye initialization"),
                "board_pose_sha256": _json_sha256(prepared.board_initial),
            },
            "gripper_to_fixed_evaluation": {
                "evaluation_mask_sha256": gripper_to_fixed_mask[
                    "evaluation_mask_sha256"],
                "selection_uses_model_output": False,
                "selection_uses_robot_fk": False,
                "evaluation_uses_robot_fk": True,
                "applied_to_every_method": True,
                "test_time_refit": False,
                "definition": GRIPPER_TO_FIXED_CROSS_TARGET_CONTRACT,
                "support_by_target": gripper_to_fixed_mask["support_by_target"],
            },
            "legacy_cube_and_fk_path_evaluation": {
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
    validate_result_contract(result)
    write_outputs(result, args.out_dir)
    print(f"[DONE] {args.out_dir}/cross_target_evaluation.{{json,csv}}")


if __name__ == "__main__":
    main()
