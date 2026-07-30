#!/usr/bin/env python3
"""GT-based pixel/FK-noise sweep for the canonical seven-row design.

This is a custom pixel-level simulator, not the METRIC dataset.  METRIC's
bundled medium-workcell sequence is board-only eye-on-base data and cannot
instantiate the cube, eye-in-hand, or FK-to-cube axes of A0--B3.
"""

from __future__ import annotations

import argparse
import csv
import json
import os
from types import SimpleNamespace
from typing import Mapping, Sequence

import numpy as np
from scipy.spatial.transform import Rotation

from CP_ablation_7row import solve_synthetic, synthetic_scene
from CP_ablation_schema import MAIN_ABLATION_CONDITIONS
from calibration_reprojection_backend import PixelObs, PoseState, pose_delta


ROWS = ("A0", "A1", "A2", "A3", "B1", "B2", "B3")
CONTRASTS = {
    "A1_to_A2": ("A1", "A2", "Unified_with_estimated_cube"),
    "B1_to_A3": ("B1", "A3", "Unified_with_FK_fixed_cube"),
    "A2_to_A3": ("A2", "A3", "FK_fixed_cube_pose"),
    "B2_to_A3": ("B2", "A3", "board_addition_given_FK_fixed_cube"),
}


def add_pixel_noise(observations: Sequence[PixelObs], sigma_px: float,
                    seed: int) -> list[PixelObs]:
    rng = np.random.default_rng(np.random.SeedSequence([int(seed), 71237]))
    output = []
    for observation in observations:
        image_points = np.asarray(observation.image_points, dtype=np.float64).copy()
        if float(sigma_px) > 0:
            image_points += rng.normal(0.0, float(sigma_px), image_points.shape)
        output.append(PixelObs(
            marker=observation.marker,
            cam=int(observation.cam),
            event=int(observation.event),
            set_idx=observation.set_idx,
            object_points=np.asarray(observation.object_points, dtype=np.float64).copy(),
            image_points=image_points,
        ))
    return output


def perturb_fk_cube_poses(cubes: Mapping[int, np.ndarray], trans_sigma_mm: float,
                          rot_sigma_deg: float, seed: int) -> dict[int, np.ndarray]:
    rng = np.random.default_rng(np.random.SeedSequence([int(seed), 93199]))
    output = {}
    for set_index, transform in sorted(cubes.items()):
        axis = rng.normal(size=3)
        axis /= np.linalg.norm(axis) + 1e-15
        angle = np.deg2rad(rng.normal(0.0, float(rot_sigma_deg)))
        delta = np.eye(4)
        delta[:3, :3] = Rotation.from_rotvec(axis * angle).as_matrix()
        delta[:3, 3] = rng.normal(0.0, float(trans_sigma_mm) / 1000.0, 3)
        output[int(set_index)] = delta @ np.asarray(transform, dtype=np.float64)
    return output


def calibration_gt_metric(estimate: PoseState, truth: PoseState) -> dict:
    values = []
    for camera in sorted(truth.cams):
        dt, dr = pose_delta(estimate.cams[camera], truth.cams[camera])
        values.append((f"T_base_C{camera}", dt, dr))
    dt, dr = pose_delta(estimate.gtc, truth.gtc)
    values.append(("T_gripper_cam", dt, dr))
    return {
        "translation_rmse_mm": float(np.sqrt(np.mean([value[1] ** 2 for value in values]))),
        "rotation_rmse_deg": float(np.sqrt(np.mean([value[2] ** 2 for value in values]))),
        "per_calibration_transform": [
            {"name": name, "translation_mm": dt, "rotation_deg": dr}
            for name, dt, dr in values
        ],
        "definition": "RMS_over_T_base_Ci_and_T_gripper_cam_common_to_all_rows",
    }


def target_gt_metric(estimate: PoseState, truth: PoseState) -> dict:
    values = []
    if estimate.board is not None and truth.board is not None:
        values.append(("T_base_board",) + pose_delta(estimate.board, truth.board))
    for set_index in sorted(estimate.cubes):
        if set_index in truth.cubes:
            values.append((f"T_base_cube[{set_index}]",) + pose_delta(
                estimate.cubes[set_index], truth.cubes[set_index]))
    if not values:
        return {"translation_rmse_mm": None, "rotation_rmse_deg": None,
                "per_target_transform": []}
    return {
        "translation_rmse_mm": float(np.sqrt(np.mean([value[1] ** 2 for value in values]))),
        "rotation_rmse_deg": float(np.sqrt(np.mean([value[2] ** 2 for value in values]))),
        "per_target_transform": [
            {"name": name, "translation_mm": dt, "rotation_deg": dr}
            for name, dt, dr in values
        ],
    }


def _all_stages_success(diagnostics: Mapping) -> bool:
    return bool(diagnostics) and all(
        bool(stage.get("success")) for stage in diagnostics.values())


def run_trial(observations: Sequence[PixelObs], truth: PoseState, robot_T, K, D,
              gripper: int, args, fixed_cube_poses: Mapping[int, np.ndarray],
              trial_seed: int, rows: Sequence[str] = ROWS) -> dict:
    by_row = {condition.row: condition for condition in MAIN_ABLATION_CONDITIONS}
    output = {}
    solver_args = SimpleNamespace(
        init_translation_mm=float(args.init_translation_mm),
        init_rotation_deg=float(args.init_rotation_deg),
        max_nfev=int(args.max_nfev), tol=float(args.tol),
        loss="soft_l1", f_scale_px=2.0,
        rotation_scale_rad=1.0, translation_scale_m=1.0,
        x_scale_mode="jac",
    )
    for row in rows:
        condition = by_row[row]
        estimate, train_rmse, diagnostics = solve_synthetic(
            condition, observations, truth, robot_T, K, D, gripper,
            solver_args, fixed_cube_poses=fixed_cube_poses, solver_seed=0)
        output[row] = {
            "trial_seed": int(trial_seed),
            "converged": _all_stages_success(diagnostics),
            "train_reprojection_rmse_px_diagnostic": train_rmse,
            "calibration_gt": calibration_gt_metric(estimate, truth),
            "target_gt": target_gt_metric(estimate, truth),
            "solver_diagnostics": diagnostics,
        }
    return output


def _mean_std(values):
    values = np.asarray([value for value in values if value is not None], dtype=float)
    if not len(values):
        return None, None
    return float(np.mean(values)), float(np.std(values))


def aggregate_pixel_trials(trials: Sequence[Mapping]) -> list[dict]:
    output = []
    for row in ROWS:
        runs = [trial["rows"][row] for trial in trials]
        n_converged = sum(run["converged"] for run in runs)
        diagnostic_t_mean, diagnostic_t_std = _mean_std([
            run["calibration_gt"]["translation_rmse_mm"] for run in runs])
        diagnostic_r_mean, diagnostic_r_std = _mean_std([
            run["calibration_gt"]["rotation_rmse_deg"] for run in runs])
        fully_converged = n_converged == len(runs)
        output.append({
            "row": row,
            "n_trials": len(runs),
            "n_converged": n_converged,
            "status": "converged" if fully_converged else (
                "mixed" if n_converged else "unstable"),
            "headline_metric_available": fully_converged,
            "translation_rmse_mm_mean": diagnostic_t_mean if fully_converged else None,
            "translation_rmse_mm_std": diagnostic_t_std if fully_converged else None,
            "rotation_rmse_deg_mean": diagnostic_r_mean if fully_converged else None,
            "rotation_rmse_deg_std": diagnostic_r_std if fully_converged else None,
            "diagnostic_all_runs_translation_rmse_mm_mean": diagnostic_t_mean,
            "diagnostic_all_runs_translation_rmse_mm_std": diagnostic_t_std,
            "diagnostic_all_runs_rotation_rmse_deg_mean": diagnostic_r_mean,
            "diagnostic_all_runs_rotation_rmse_deg_std": diagnostic_r_std,
        })
    return output


def aggregate_fk_trials(fk_trials: Mapping[str, Sequence[Mapping]],
                        fk_rows: Sequence[str] = ("A2", "A3", "B1", "B2")) -> dict:
    output = {}
    for key, trials in fk_trials.items():
        all_rows = []
        for row in fk_rows:
            runs = [trial["rows"][row] for trial in trials]
            n_converged = sum(run["converged"] for run in runs)
            diagnostic_t_mean, diagnostic_t_std = _mean_std([
                run["calibration_gt"]["translation_rmse_mm"] for run in runs])
            diagnostic_r_mean, diagnostic_r_std = _mean_std([
                run["calibration_gt"]["rotation_rmse_deg"] for run in runs])
            fully_converged = n_converged == len(runs)
            all_rows.append({
                "row": row, "n_trials": len(runs),
                "n_converged": n_converged,
                "status": "converged" if fully_converged else (
                    "mixed" if n_converged else "unstable"),
                "headline_metric_available": fully_converged,
                "translation_rmse_mm_mean": diagnostic_t_mean if fully_converged else None,
                "translation_rmse_mm_std": diagnostic_t_std if fully_converged else None,
                "rotation_rmse_deg_mean": diagnostic_r_mean if fully_converged else None,
                "rotation_rmse_deg_std": diagnostic_r_std if fully_converged else None,
                "diagnostic_all_runs_translation_rmse_mm_mean": diagnostic_t_mean,
                "diagnostic_all_runs_translation_rmse_mm_std": diagnostic_t_std,
                "diagnostic_all_runs_rotation_rmse_deg_mean": diagnostic_r_mean,
                "diagnostic_all_runs_rotation_rmse_deg_std": diagnostic_r_std,
            })
        output[key] = all_rows
    return output


def aggregate_effects(summary_by_level: Mapping[str, Sequence[Mapping]],
                      level_name: str) -> list[dict]:
    effects = []
    for level, rows in summary_by_level.items():
        by_row = {row["row"]: row for row in rows}
        for contrast, (first, second, interpretation) in CONTRASTS.items():
            if first not in by_row or second not in by_row:
                continue
            for metric, unit in (("translation_rmse_mm", "mm"),
                                 ("rotation_rmse_deg", "deg")):
                first_value = by_row[first][f"{metric}_mean"]
                second_value = by_row[second][f"{metric}_mean"]
                effects.append({
                    level_name: level,
                    "contrast": contrast,
                    "metric": metric,
                    "unit": unit,
                    "delta_definition": "second_minus_first",
                    "delta": None if first_value is None or second_value is None else
                        float(second_value - first_value),
                    "interpretation": interpretation,
                })
    return effects


def write_csv(path: str, rows: Sequence[Mapping]) -> None:
    fieldnames = []
    for row in rows:
        for key in row:
            if key not in fieldnames:
                fieldnames.append(key)
    with open(path, "w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames or ["empty"])
        writer.writeheader()
        writer.writerows(rows)


def write_outputs(result: Mapping, out_dir: str) -> None:
    os.makedirs(out_dir, exist_ok=True)
    with open(os.path.join(out_dir, "synthetic_7row.json"), "w") as handle:
        json.dump(result, handle, indent=2)
    pixel_rows = []
    for sigma, rows in result["pixel_sweep_summary"].items():
        for row in rows:
            pixel_rows.append({"pixel_sigma_px": sigma, **row})
    write_csv(os.path.join(out_dir, "pixel_sweep_summary.csv"), pixel_rows)
    write_csv(os.path.join(out_dir, "pixel_sweep_effects.csv"),
              result["pixel_sweep_effects"])
    fk_rows = []
    for level, rows in result["fk_noise_sweep_summary"].items():
        for row in rows:
            fk_rows.append({"fk_noise": level, **row})
    write_csv(os.path.join(out_dir, "fk_noise_sweep_summary.csv"), fk_rows)
    write_csv(os.path.join(out_dir, "fk_noise_sweep_effects.csv"),
              result["fk_noise_sweep_effects"])

    sigmas = list(result["pixel_sweep_summary"])
    by_sigma_row = {
        (sigma, row["row"]): row
        for sigma, rows in result["pixel_sweep_summary"].items() for row in rows
    }
    lines = [
        "# Custom-GT canonical seven-row synthetic sweep",
        "",
        "This is not METRIC. The headline error is RMS over calibration transforms "
        "common to every row (`T_base_Ci` and `T_gripper_cam`), evaluated against exact GT.",
        "",
        "| Row | " + " | ".join(f"sigma={sigma} px (mm/deg)" for sigma in sigmas) + " |",
        "| --- | " + " | ".join("---:" for _ in sigmas) + " |",
    ]
    for row in ROWS:
        cells = []
        for sigma in sigmas:
            value = by_sigma_row[(sigma, row)]
            if not value["headline_metric_available"]:
                cells.append(
                    f"{value['status']} ({value['n_converged']}/{value['n_trials']})")
            else:
                cells.append(
                    f"{value['translation_rmse_mm_mean']:.4f}±{value['translation_rmse_mm_std']:.4f} / "
                    f"{value['rotation_rmse_deg_mean']:.4f}±{value['rotation_rmse_deg_std']:.4f}")
        lines.append(f"| {row} | " + " | ".join(cells) + " |")
    lines += [
        "",
        "Deltas in the effect CSV are second row minus first row; negative is better.",
        "FK-noise sweep uses the stated pixel noise and perturbs the shared fixed cube poses "
        "byte-identically for B1/A3/B2 within each trial.",
        "",
    ]
    with open(os.path.join(out_dir, "synthetic_7row.md"), "w") as handle:
        handle.write("\n".join(lines))


def _parse_fk_specs(value: str) -> list[tuple[float, float]]:
    output = []
    for item in value.split(","):
        trans, rot = item.strip().split(":", 1)
        output.append((float(trans), float(rot)))
    return output


def parse_args():
    parser = argparse.ArgumentParser(description="Custom-GT canonical seven-row noise sweep")
    parser.add_argument("--out_dir", default="CP_result/synthetic_7row")
    parser.add_argument("--pixel_sigmas", default="0,0.5,1.0,2.0")
    parser.add_argument("--trials", type=int, default=10)
    parser.add_argument("--trial_seed_start", type=int, default=12000)
    parser.add_argument("--fk_noise_specs", default="0:0,1:0.1,3:0.3,5:0.5")
    parser.add_argument("--fk_sweep_pixel_sigma", type=float, default=1.0)
    parser.add_argument("--max_nfev", type=int, default=300)
    parser.add_argument("--tol", type=float, default=1e-8)
    parser.add_argument("--init_translation_mm", type=float, default=5.0)
    parser.add_argument("--init_rotation_deg", type=float, default=1.0)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if args.trials < 1:
        raise ValueError("--trials must be positive")
    pixel_sigmas = [float(value) for value in args.pixel_sigmas.split(",")]
    fk_specs = _parse_fk_specs(args.fk_noise_specs)
    observations, truth, robot_T, K, D, gripper = synthetic_scene()
    pixel_trials = {}
    for sigma in pixel_sigmas:
        key = f"{sigma:g}"
        print(f"[PIXEL] sigma={sigma:g} px", flush=True)
        trials = []
        for index in range(args.trials):
            seed = int(args.trial_seed_start + index)
            noisy = add_pixel_noise(observations, sigma, seed)
            rows = run_trial(
                noisy, truth, robot_T, K, D, gripper, args,
                fixed_cube_poses=truth.cubes, trial_seed=seed)
            trials.append({"trial_seed": seed, "rows": rows})
            print(f"  trial={index + 1}/{args.trials}", flush=True)
        pixel_trials[key] = trials
    pixel_summary = {key: aggregate_pixel_trials(value)
                     for key, value in pixel_trials.items()}

    fk_trials = {}
    fk_rows = ("A2", "A3", "B1", "B2")
    for trans_mm, rot_deg in fk_specs:
        key = f"{trans_mm:g}mm_{rot_deg:g}deg"
        print(f"[FK] sigma={trans_mm:g} mm/{rot_deg:g} deg", flush=True)
        trials = []
        for index in range(args.trials):
            seed = int(args.trial_seed_start + index)
            noisy = add_pixel_noise(observations, args.fk_sweep_pixel_sigma, seed)
            fixed = perturb_fk_cube_poses(truth.cubes, trans_mm, rot_deg, seed)
            rows = run_trial(
                noisy, truth, robot_T, K, D, gripper, args,
                fixed_cube_poses=fixed, trial_seed=seed, rows=fk_rows)
            trials.append({"trial_seed": seed, "rows": rows})
            print(f"  trial={index + 1}/{args.trials}", flush=True)
        fk_trials[key] = trials
    fk_summary = aggregate_fk_trials(fk_trials, fk_rows)

    result = {
        "protocol": {
            "schema": "canonical_custom_pixel_GT_seven_row_v1",
            "dataset": "custom_pixel_level_GT_simulator_not_METRIC",
            "metric": "RMS_GT_error_over_T_base_Ci_and_T_gripper_cam",
            "pixel_noise": "iid_isotropic_Gaussian_per_corner_coordinate",
            "pixel_sigmas_px": pixel_sigmas,
            "trials": int(args.trials),
            "trial_seeds_predeclared": [
                int(args.trial_seed_start + index) for index in range(args.trials)],
            "fixed_cube_pixel_sweep": "exact_GT_shared_by_B1_A3_B2",
            "fk_noise": (
                "isotropic_axis_angle_scalar_sigma_deg_and_iid_translation_axis_sigma_mm"),
            "fk_sweep_pixel_sigma_px": float(args.fk_sweep_pixel_sigma),
            "fk_noise_specs_mm_deg": [list(spec) for spec in fk_specs],
            "post_correction": False,
        },
        "pixel_sweep_trials": pixel_trials,
        "pixel_sweep_summary": pixel_summary,
        "pixel_sweep_effects": aggregate_effects(
            pixel_summary, "pixel_sigma_px"),
        "fk_noise_sweep_trials": fk_trials,
        "fk_noise_sweep_summary": fk_summary,
        "fk_noise_sweep_effects": aggregate_effects(
            fk_summary, "fk_noise"),
    }
    write_outputs(result, args.out_dir)
    print(f"[SAVE] {args.out_dir}/synthetic_7row.{{json,md}}", flush=True)


if __name__ == "__main__":
    main()
