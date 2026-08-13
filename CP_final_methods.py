#!/usr/bin/env python3
"""Final-method runner for A2/A3/A4a/A4b/A4c and the fair B1 baseline.

This runner is intentionally separate from the historical seven-row runner.
It implements the method contract in ``Calibration_Experiment_table.md``:

* A2 and A4 share an identical pixel-level visual objective and solver budget.
* A4a changes hard FK to a fixed-weight quadratic soft factor.
* A4b adds covariance whitening.
* A4c (= A4) adds an independently preregistered robust FK loss.
* B1 uses the same raw observations, visual loss, FK covariance and robust
  factor as A4, but freezes the eih-estimated targets before fitting each fixed
  camera independently.

Held-out reprojection/path metrics are diagnostic.  Absolute claims must use
``CP_final_external_gt_eval.py`` with independent blind external GT.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from typing import Mapping, Sequence

import numpy as np

import CP_ablation_7row as legacy
from CP_ablation_schema import MAIN_ABLATION_CONDITIONS, UNIFIED_FREE_VARIABLES
from calibration_fk_factor import (
    FKFactorSpec,
    diagonal_covariance,
    solve_factorized_fk,
    validate_covariance,
)
from calibration_path_evaluation import evaluate_paths_with_common_mask
from calibration_reprojection_backend import PoseState, variable_keys


FINAL_METHODS = ("A2", "A3", "A4a", "A4b", "A4", "B1", "B2")


def _sha256_file(path: str) -> str:
    digest = hashlib.sha256()
    with open(path, "rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def load_fk_covariances(path: str, set_ids: Sequence[int]) -> tuple[dict, dict]:
    """Load a preregistered FK covariance artifact without blind-GT leakage."""
    with open(path) as handle:
        payload = json.load(handle)
    if payload.get("artifact_schema") != "fk_factor_covariance_v1":
        raise ValueError("unknown FK covariance artifact schema")
    if payload.get("twist_order") != ["rx_rad", "ry_rad", "rz_rad", "tx_m", "ty_m", "tz_m"]:
        raise ValueError("FK covariance twist order/units do not match the A4 contract")
    if payload.get("blind_external_gt_used") is not False:
        raise ValueError("FK covariance artifact must explicitly exclude blind external GT")
    if payload.get("preregistered_before_blind_test") is not True:
        raise ValueError("FK covariance must be preregistered before blind-test evaluation")
    measurement_source = str(payload.get("measurement_source", "")).strip()
    if not measurement_source or measurement_source.startswith("REPLACE_"):
        raise ValueError("FK covariance measurement_source is still a template placeholder")
    try:
        n_repeats = int(payload.get("n_repeats", 0))
    except (TypeError, ValueError):
        n_repeats = 0
    if n_repeats < 3:
        raise ValueError("FK covariance artifact needs at least three physical repeats")
    shared = payload.get("shared_covariance_6x6")
    per_set = payload.get("per_set_covariance_6x6", {})
    covariances = {}
    for set_index in set_ids:
        raw = per_set.get(str(int(set_index)), shared)
        if raw is None:
            raise ValueError(f"FK covariance missing for set {set_index}")
        covariances[int(set_index)] = validate_covariance(np.asarray(raw, dtype=np.float64))
    provenance = {
        "path": os.path.abspath(path),
        "sha256": _sha256_file(path),
        "measurement_source": measurement_source,
        "n_repeats": n_repeats,
        "confirmatory_ready": True,
    }
    return covariances, provenance


def preflight_covariances(set_ids: Sequence[int], translation_std_mm: float,
                          rotation_std_deg: float) -> tuple[dict, dict]:
    covariance = diagonal_covariance(translation_std_mm, rotation_std_deg)
    return (
        {int(set_index): covariance.copy() for set_index in set_ids},
        {
            "path": None,
            "measurement_source": "diagnostic_cli_isotropic_placeholder",
            "translation_std_mm": float(translation_std_mm),
            "rotation_std_deg": float(rotation_std_deg),
            "confirmatory_ready": False,
            "warning": "preflight covariance cannot support the final A4 claim",
        },
    )


def _condition(row: str):
    return {condition.row: condition for condition in MAIN_ABLATION_CONDITIONS}[row]


def _common_metrics(data, state: PoseState, condition, train_obs, test_obs) -> dict:
    relevant_train = legacy.filter_observations(
        train_obs, condition, None, data.gripper, state.cams)
    relevant_test = legacy.filter_observations(
        test_obs, condition, None, data.gripper, state.cams)
    path = evaluate_paths_with_common_mask(
        data.test_obs,
        state.cams,
        state.gtc,
        data.robot_T,
        data.gripper,
        data.K_map,
        data.D_map,
        data.path_evaluation_mask,
    )
    path.pop("predicted_by_set", None)
    return {
        "train_reprojection": legacy.reprojection_metrics(
            relevant_train, state, data.robot_T, data.K_map, data.D_map, data.gripper),
        "heldout_reprojection": legacy.reprojection_metrics(
            relevant_test, state, data.robot_T, data.K_map, data.D_map, data.gripper),
        "heldout_path_metrics": path,
    }


def _solve_joint(data, initial: PoseState, spec: FKFactorSpec,
                 covariances: Mapping[int, np.ndarray], seed: int, args):
    condition = _condition("A2")
    observations = legacy.filter_observations(
        data.train_obs, condition, None, data.gripper, initial.cams)
    return solve_factorized_fk(
        observations=observations,
        variable_keys_=variable_keys(UNIFIED_FREE_VARIABLES["A2"], initial),
        reference_state=initial,
        robot_T=data.robot_T,
        K_map=data.K_map,
        D_map=data.D_map,
        gripper_cam_idx=data.gripper,
        options=legacy.canonical_solver_options(args),
        fk_targets=data.fixed_cubes,
        fk_covariances=covariances,
        fk_spec=spec,
        seed=seed,
        init_translation_mm=float(args.init_translation_mm),
        init_rotation_deg=float(args.init_rotation_deg),
    )


def _solve_hard_fk(data, initial: PoseState, seed: int, args):
    condition = _condition("A3")
    observations = legacy.filter_observations(
        data.train_obs, condition, None, data.gripper, initial.cams)
    return solve_factorized_fk(
        observations=observations,
        variable_keys_=variable_keys(UNIFIED_FREE_VARIABLES["A3"], initial),
        reference_state=initial,
        robot_T=data.robot_T,
        K_map=data.K_map,
        D_map=data.D_map,
        gripper_cam_idx=data.gripper,
        options=legacy.canonical_solver_options(args),
        fk_spec=FKFactorSpec(mode="none"),
        seed=seed,
        init_translation_mm=float(args.init_translation_mm),
        init_rotation_deg=float(args.init_rotation_deg),
    )


def _solve_fair_independent(data, initial: PoseState,
                            covariances: Mapping[int, np.ndarray], seed: int, args):
    """Fit eih targets once, then each fixed camera without joint feedback."""
    condition = _condition("A2")
    eih = legacy.filter_observations(
        data.train_obs, condition, "eih", data.gripper, initial.cams)
    stage1_families = ("T_gripper_cam", "T_base_board", "T_base_cube_by_set")
    stage1, d1 = solve_factorized_fk(
        observations=eih,
        variable_keys_=variable_keys(stage1_families, initial),
        reference_state=initial,
        robot_T=data.robot_T,
        K_map=data.K_map,
        D_map=data.D_map,
        gripper_cam_idx=data.gripper,
        options=legacy.canonical_solver_options(args),
        fk_targets=data.fixed_cubes,
        fk_covariances=covariances,
        fk_spec=FKFactorSpec(
            mode="covariance",
            loss=str(args.fk_loss),
            robust_scale=float(args.fk_robust_scale),
        ),
        seed=seed,
        init_translation_mm=float(args.init_translation_mm),
        init_rotation_deg=float(args.init_rotation_deg),
    )
    final = stage1.clone()
    stage2 = {}
    for camera_id in sorted(initial.cams):
        observations = [
            obs for obs in legacy.filter_observations(
                data.train_obs, condition, "e2h", data.gripper, initial.cams)
            if int(obs.cam) == int(camera_id)
        ]
        if not observations:
            raise RuntimeError(f"B1 has no fixed-camera observations for cam{camera_id}")
        solved, diag = solve_factorized_fk(
            observations=observations,
            variable_keys_=[("cam", int(camera_id))],
            reference_state=final,
            robot_T=data.robot_T,
            K_map=data.K_map,
            D_map=data.D_map,
            gripper_cam_idx=data.gripper,
            options=legacy.canonical_solver_options(args),
            fk_spec=FKFactorSpec(mode="none"),
            seed=seed,
            init_translation_mm=float(args.init_translation_mm),
            init_rotation_deg=float(args.init_rotation_deg),
        )
        final.cams[int(camera_id)] = solved.cams[int(camera_id)]
        stage2[f"cam{camera_id}"] = diag
    return final, {
        "backend": "fair_independent_eih_then_per_fixed_camera_v1",
        "same_visual_objective_as_A4": True,
        "same_fk_factor_as_A4": True,
        "joint_feedback_from_fixed_cameras": False,
        "stage1_eih_targets": d1,
        "stage2_independent_fixed_cameras": stage2,
        "success": bool(d1["success"] and all(item["success"] for item in stage2.values())),
    }


def _solve_cube_only_soft_fk(data, initial: PoseState,
                             covariances: Mapping[int, np.ndarray], seed: int, args):
    """B2 fair arm: remove board observations while retaining A4's soft-FK factor."""
    condition = _condition("B2")
    observations = legacy.filter_observations(
        data.train_obs, condition, None, data.gripper, initial.cams)
    cube_only = initial.clone()
    cube_only.board = None
    return solve_factorized_fk(
        observations=observations,
        variable_keys_=variable_keys(
            ("T_base_Ci", "T_gripper_cam", "T_base_cube_by_set"), cube_only),
        reference_state=cube_only,
        robot_T=data.robot_T,
        K_map=data.K_map,
        D_map=data.D_map,
        gripper_cam_idx=data.gripper,
        options=legacy.canonical_solver_options(args),
        fk_targets=data.fixed_cubes,
        fk_covariances=covariances,
        fk_spec=FKFactorSpec(
            mode="covariance",
            loss=str(args.fk_loss),
            robust_scale=float(args.fk_robust_scale),
        ),
        seed=seed,
        init_translation_mm=float(args.init_translation_mm),
        init_rotation_deg=float(args.init_rotation_deg),
    )


def run_method(method: str, data, initial_a2: PoseState, initial_a3: PoseState,
               covariances, seed: int, args) -> dict:
    if method == "A2":
        state, diagnostics = _solve_joint(
            data, initial_a2, FKFactorSpec(mode="none"), covariances, seed, args)
    elif method == "A3":
        state, diagnostics = _solve_hard_fk(data, initial_a3, seed, args)
    elif method == "A4a":
        state, diagnostics = _solve_joint(
            data,
            initial_a2,
            FKFactorSpec(
                mode="fixed_probe",
                loss="linear",
                fixed_lambda_px_per_mm=float(args.a4a_lambda_px_per_mm),
                probe_lever_mm=float(args.a4a_probe_lever_mm),
            ),
            covariances,
            seed,
            args,
        )
    elif method == "A4b":
        state, diagnostics = _solve_joint(
            data,
            initial_a2,
            FKFactorSpec(mode="covariance", loss="linear"),
            covariances,
            seed,
            args,
        )
    elif method == "A4":
        state, diagnostics = _solve_joint(
            data,
            initial_a2,
            FKFactorSpec(
                mode="covariance",
                loss=str(args.fk_loss),
                robust_scale=float(args.fk_robust_scale),
            ),
            covariances,
            seed,
            args,
        )
    elif method == "B1":
        state, diagnostics = _solve_fair_independent(
            data, initial_a2, covariances, seed, args)
    elif method == "B2":
        state, diagnostics = _solve_cube_only_soft_fk(
            data, initial_a2, covariances, seed, args)
    else:
        raise ValueError(f"unknown final method {method}")
    metrics = _common_metrics(
        data,
        state,
        (_condition("A3") if method == "A3" else
         _condition("B2") if method == "B2" else _condition("A2")),
        data.train_obs,
        data.test_obs,
    )
    return {
        "method": method,
        "seed": int(seed),
        "converged": bool(diagnostics.get("success", False)),
        "solver": diagnostics,
        **metrics,
        "transforms": legacy.serialize_state(state),
    }


def _write_outputs(result: dict, out_dir: str) -> None:
    os.makedirs(out_dir, exist_ok=True)
    output = os.path.join(out_dir, "final_methods.json")
    with open(output, "w") as handle:
        json.dump(legacy._jsonable(result), handle, indent=2)
    lines = [
        "# Final calibration methods — software result",
        "",
        f"Confirmatory-ready FK covariance: **{result['protocol']['fk_covariance']['confirmatory_ready']}**.",
        "",
        "Held-out reprojection and path consistency below are diagnostic; external-GT accuracy is evaluated separately.",
        "",
        "| Method | Runs converged | Held-out reprojection (px) | e_cross (mm) | e_e2e (mm) |",
        "| --- | ---: | ---: | ---: | ---: |",
    ]
    for method in result["methods"]:
        runs = result["methods"][method]
        converged = sum(bool(run["converged"]) for run in runs)

        def mean(path):
            values = []
            for run in runs:
                current = run
                for key in path:
                    current = current.get(key, {}) if isinstance(current, dict) else {}
                if isinstance(current, (int, float)):
                    values.append(float(current))
            return "—" if not values else f"{np.mean(values):.4f}"

        lines.append(
            f"| {method} | {converged}/{len(runs)} | "
            f"{mean(('heldout_reprojection', 'overall', 'rmse_px'))} | "
            f"{mean(('heldout_path_metrics', 'e_cross_translation_rmse_mm'))} | "
            f"{mean(('heldout_path_metrics', 'e_e2e_translation_rmse_mm'))} |"
        )
    with open(os.path.join(out_dir, "final_methods.md"), "w") as handle:
        handle.write("\n".join(lines) + "\n")


def parse_args():
    parser = argparse.ArgumentParser(
        description="Final A2/A3/A4/B1/B2 calibration-method runner")
    parser.add_argument("--root_folder", default="data/session")
    parser.add_argument("--intrinsics_dir", default="intrinsics")
    parser.add_argument("--calib_dir", default="data/session/calib_out")
    parser.add_argument("--out_dir", default="CP_result/session01/main/final_methods_preflight")
    parser.add_argument("--methods", default=",".join(FINAL_METHODS))
    parser.add_argument(
        "--include_sets", default="",
        help="Use only these set_index values (comma list/ranges, e.g. 5-12).")
    parser.add_argument("--fk_covariance_json")
    parser.add_argument("--preflight_isotropic_fk_std", default="3.0,0.3",
                        help="Diagnostic-only translation-mm,rotation-deg covariance when no artifact exists")
    parser.add_argument("--a4a_lambda_px_per_mm", type=float, default=3.0)
    parser.add_argument("--a4a_probe_lever_mm", type=float, default=29.5)
    parser.add_argument("--fk_loss", choices=["huber", "soft_l1"], default="huber")
    parser.add_argument("--fk_robust_scale", type=float, default=2.5)
    parser.add_argument("--test_fraction", type=float, default=0.2)
    parser.add_argument("--split_seed", type=int, default=20260729)
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
    parser.add_argument("--image_scale", type=float, default=1.0)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    methods = [item.strip() for item in args.methods.split(",") if item.strip()]
    unknown = sorted(set(methods) - set(FINAL_METHODS))
    if unknown:
        raise ValueError(f"unknown methods: {unknown}")
    data = legacy.prepare_ablation_data(args)
    set_ids = sorted(data.fixed_cubes)
    if args.fk_covariance_json:
        covariances, covariance_provenance = load_fk_covariances(
            args.fk_covariance_json, set_ids)
    else:
        values = [float(item) for item in args.preflight_isotropic_fk_std.split(",")]
        if len(values) != 2:
            raise ValueError("--preflight_isotropic_fk_std must be translation_mm,rotation_deg")
        covariances, covariance_provenance = preflight_covariances(
            set_ids, values[0], values[1])

    initial_a2, initial_a2_diag = legacy.make_initial_state(
        _condition("A2"), data.train_obs, data.gripper, data.robot_T,
        data.K_map, data.D_map, data.board_gtc, data.board_initial,
        data.visual_cubes, data.fixed_cubes, data.fixed_gtc_initial)
    initial_a3, initial_a3_diag = legacy.make_initial_state(
        _condition("A3"), data.train_obs, data.gripper, data.robot_T,
        data.K_map, data.D_map, data.board_gtc, data.board_initial,
        data.visual_cubes, data.fixed_cubes, data.fixed_gtc_initial)

    runs = {method: [] for method in methods}
    for seed in range(int(args.num_inits)):
        for method in methods:
            print(f"[FINAL] {method} seed={seed}")
            runs[method].append(run_method(
                method, data, initial_a2, initial_a3, covariances, seed, args))

    result = {
        "experiment": "final_A2_A3_A4_factor_decomposition_and_fair_B1_B2",
        "protocol": {
            "dataset": args.root_folder,
            "intrinsics_dir": args.intrinsics_dir,
            "requested_set_filter": str(getattr(args, "include_sets", "")),
            "resolved_set_indices": data.split["eligible_sets"],
            "cube_config_source": data.cube_config_source,
            "visual_objective_shared_by_A2_A4_B1": True,
            "visual_loss": str(args.loss),
            "visual_f_scale_px": float(args.f_scale_px),
            "solver_budget_shared": {
                "max_nfev": int(args.max_nfev),
                "tol": float(args.tol),
                "num_inits": int(args.num_inits),
            },
            "fk_covariance": covariance_provenance,
            "fk_loss": str(args.fk_loss),
            "fk_robust_scale": float(args.fk_robust_scale),
            "external_ground_truth_used": False,
            "claim_role": "software_preflight_and_internal_diagnostics_only",
            "split": data.split,
            "path_evaluation_mask": data.path_evaluation_mask,
            "shared_fk_artifact_sha256": data.alignment_artifact["artifact_sha256"],
            "initialization": {"A2_family": initial_a2_diag, "A3": initial_a3_diag},
        },
        "methods": runs,
    }
    _write_outputs(result, args.out_dir)
    print(f"[DONE] {os.path.join(args.out_dir, 'final_methods.json')}")


if __name__ == "__main__":
    main()
