#!/usr/bin/env python3
"""Pre-registered optimizer-scaling study using synthetic and train data only.

This script selects numerical parameter scaling before any canonical Table 1
run.  It never computes held-out reprojection or path metrics.  Candidates use
identical observations, initialization perturbations, loss, tolerances, and
freeze masks; only the coordinate scaling / SciPy trust-region x_scale differs.
"""
from __future__ import annotations

import argparse
import csv
import json
import os
from types import SimpleNamespace
from typing import Any, Dict, List, Sequence

import numpy as np

from CP_ablation_7row import (
    _jsonable,
    filter_observations,
    make_initial_state,
    prepare_ablation_data,
    reprojection_metrics,
    run_noise_free_sanity,
    serialize_state,
    solve_stage,
    transform_dispersion,
)
from calibration_reprojection_backend import PoseState
from CP_ablation_schema import (
    MAIN_ABLATION_CONDITIONS,
    SEQUENTIAL_STAGE_SPECS,
    UNIFIED_FREE_VARIABLES,
    validate_main_runner_contract,
)


CANDIDATES = (
    {
        "name": "legacy_unit",
        "rotation_scale_rad": 1.0,
        "translation_scale_m": 1.0,
        "x_scale_mode": "unit",
        "preference": 2,
    },
    {
        "name": "manual_L0.5m",
        "rotation_scale_rad": 1.0,
        "translation_scale_m": 0.5,
        "x_scale_mode": "unit",
        "preference": 0,
    },
    {
        "name": "scipy_x_scale_jac",
        "rotation_scale_rad": 1.0,
        "translation_scale_m": 1.0,
        "x_scale_mode": "jac",
        "preference": 1,
    },
)

OBJECTIVE_RELATIVE_TOLERANCE = 5e-3
OBJECTIVE_ABSOLUTE_TOLERANCE = 1e-12


def candidate_args(base, candidate: dict) -> SimpleNamespace:
    return SimpleNamespace(
        init_translation_mm=float(base.init_translation_mm),
        init_rotation_deg=float(base.init_rotation_deg),
        max_nfev=int(base.max_nfev),
        tol=float(base.tol),
        rotation_scale_rad=float(candidate["rotation_scale_rad"]),
        translation_scale_m=float(candidate["translation_scale_m"]),
        x_scale_mode=str(candidate["x_scale_mode"]),
        loss=str(base.loss),
        f_scale_px=float(base.f_scale_px),
    )


def run_train_only(condition, initial_state, train_obs, prepared,
                   seed: int, args) -> dict:
    gripper = prepared.gripper
    relevant = filter_observations(
        train_obs, condition, None, gripper, initial_state.cams)
    if condition.unified == "seq":
        spec = SEQUENTIAL_STAGE_SPECS[condition.row]
        eih = filter_observations(
            relevant, condition, "eih", gripper, initial_state.cams)
        state1, d1 = solve_stage(
            eih, spec.stage1_free, initial_state, prepared.robot_T,
            prepared.K_map, prepared.D_map, gripper, seed, args)
        e2h = filter_observations(
            relevant, condition, "e2h", gripper, state1.cams)
        final_state, d2 = solve_stage(
            e2h, spec.stage2_free, state1, prepared.robot_T,
            prepared.K_map, prepared.D_map, gripper, seed, args)
        stages = {"stage1_eih": d1, "stage2_e2h": d2}
    else:
        final_state, diag = solve_stage(
            relevant, UNIFIED_FREE_VARIABLES[condition.row], initial_state,
            prepared.robot_T, prepared.K_map, prepared.D_map, gripper,
            seed, args)
        stages = {"joint_eih_e2h": diag}
    metrics = reprojection_metrics(
        relevant, final_state, prepared.robot_T, prepared.K_map,
        prepared.D_map, gripper)
    total_cost = float(sum(stage["cost"] for stage in stages.values()))
    total_residuals = int(sum(stage["n_residuals"] for stage in stages.values()))
    return {
        "seed": int(seed),
        "converged": bool(all(stage["success"] for stage in stages.values())),
        "normalized_robust_cost": total_cost / max(total_residuals, 1),
        "train_reprojection_rmse_px": float(metrics["overall"]["rmse_px"]),
        "stages": stages,
        "transforms": serialize_state(final_state),
    }


def flatten_stages(candidate_result: dict) -> List[dict]:
    return [stage for row in candidate_result["rows"].values()
            for run in row["runs"] for stage in run["stages"].values()]


def median(values: Sequence[float]) -> float:
    return float(np.median(np.asarray(values, dtype=np.float64))) if values else float("inf")


def summarize_candidate(candidate_result: dict) -> dict:
    stages = flatten_stages(candidate_result)
    return {
        "synthetic_passed": bool(candidate_result["synthetic_sanity"]["passed"]),
        "n_stages": len(stages),
        "successful_stages": sum(bool(stage["success"]) for stage in stages),
        "max_nfev_stages": sum(int(stage["status"]) == 0 for stage in stages),
        "hard_failure_stages": sum(int(stage["status"]) < 0 for stage in stages),
        "rank_deficient_stages": sum(
            bool(stage["common_scaled_jacobian"]["rank_deficient"])
            for stage in stages),
        "median_nfev": median([float(stage["nfev"]) for stage in stages]),
        "median_common_scaled_gradient_inf_norm": median([
            float(stage["common_scaled_gradient_inf_norm"]) for stage in stages]),
        "maximum_common_scaled_jacobian_condition": float(max(
            stage["common_scaled_jacobian"]["jacobian_condition_number"]
            for stage in stages)),
        "termination_status_counts": {
            str(status): sum(int(stage["status"]) == status for stage in stages)
            for status in sorted({int(stage["status"]) for stage in stages})
        },
    }


def apply_objective_guard(results: Dict[str, dict], rows: Sequence[str],
                          num_inits: int) -> None:
    best = {}
    for row in rows:
        for seed in range(num_inits):
            best[(row, seed)] = min(
                float(result["rows"][row]["runs"][seed]["normalized_robust_cost"])
                for result in results.values())
    for result in results.values():
        failures = []
        for row in rows:
            for seed in range(num_inits):
                value = float(result["rows"][row]["runs"][seed]["normalized_robust_cost"])
                limit = (best[(row, seed)] * (1.0 + OBJECTIVE_RELATIVE_TOLERANCE)
                         + OBJECTIVE_ABSOLUTE_TOLERANCE)
                if not np.isfinite(value) or value > limit:
                    failures.append({
                        "row": row, "seed": seed, "value": value,
                        "best": best[(row, seed)], "limit": limit,
                    })
        result["summary"]["objective_guard_failures"] = failures
        result["summary"]["objective_guard_passed"] = not failures


def select_candidate(results: Dict[str, dict]) -> dict:
    entries = []
    for name, result in results.items():
        summary = result["summary"]
        feasible = bool(
            summary["synthetic_passed"]
            and summary["successful_stages"] == summary["n_stages"]
            and summary["hard_failure_stages"] == 0
            and summary["rank_deficient_stages"] == 0
            and summary["objective_guard_passed"]
        )
        candidate = result["candidate"]
        score = (
            0 if feasible else 1,
            -int(summary["successful_stages"]),
            int(summary["max_nfev_stages"]),
            float(summary["median_nfev"]),
            float(summary["median_common_scaled_gradient_inf_norm"]),
            int(candidate["preference"]),
        )
        entries.append((score, name, feasible))
    entries.sort()
    _, selected, feasible = entries[0]
    return {
        "status": ("selected" if feasible else "best_available_but_gates_failed"),
        "selected_candidate": selected,
        "selected_solver_options": {
            key: results[selected]["candidate"][key]
            for key in ("rotation_scale_rad", "translation_scale_m", "x_scale_mode")
        },
        "ranking": [
            {"candidate": name, "feasible": item_feasible,
             "lexicographic_score": list(score)}
            for score, name, item_feasible in entries
        ],
        "rule": [
            "synthetic gate, every real-data stage converged, no hard failure, full rank, objective within 0.5% of per-run best",
            "maximize successful stages",
            "minimize max-nfev stages",
            "minimize median nfev",
            "minimize common-coordinate gradient inf-norm",
            "tie preference: manual L=0.5m, scipy jac, legacy unit",
        ],
    }


def write_outputs(report: dict, out_dir: str) -> None:
    os.makedirs(out_dir, exist_ok=True)
    with open(os.path.join(out_dir, "convergence_study.json"), "w") as handle:
        json.dump(_jsonable(report), handle, indent=2)
    rows = []
    for name, result in report["candidates"].items():
        summary = result["summary"]
        rows.append({
            "candidate": name,
            "rotation_scale_rad": result["candidate"]["rotation_scale_rad"],
            "translation_scale_m": result["candidate"]["translation_scale_m"],
            "x_scale_mode": result["candidate"]["x_scale_mode"],
            "synthetic_passed": summary["synthetic_passed"],
            "successful_stages": summary["successful_stages"],
            "n_stages": summary["n_stages"],
            "max_nfev_stages": summary["max_nfev_stages"],
            "hard_failure_stages": summary["hard_failure_stages"],
            "rank_deficient_stages": summary["rank_deficient_stages"],
            "objective_guard_passed": summary["objective_guard_passed"],
            "median_nfev": summary["median_nfev"],
            "median_common_scaled_gradient_inf_norm": summary[
                "median_common_scaled_gradient_inf_norm"],
            "maximum_common_scaled_jacobian_condition": summary[
                "maximum_common_scaled_jacobian_condition"],
        })
    with open(os.path.join(out_dir, "convergence_study.csv"), "w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    lines = [
        "# Corner-backend convergence study",
        "",
        "Selection data: **noise-free synthetic + calibration-train observations only**. "
        "Held-out metrics were not computed.",
        "",
        f"Common loss: `{report['protocol'].get('loss', 'unknown')}` "
        f"(`f_scale={report['protocol'].get('f_scale_px', 'unknown')} px`), "
        f"max_nfev={report['protocol'].get('max_nfev', 'unknown')}, "
        f"tol={report['protocol'].get('tol', 'unknown')}.",
        "",
        f"Selection status: **{report['selection']['status']}**; candidate: "
        f"**{report['selection']['selected_candidate']}**.",
        "",
        "| candidate | synthetic | success stages | max-nfev | rank deficient | objective guard | median nfev | common grad inf | max common J cond |",
        "| --- | :---: | ---: | ---: | ---: | :---: | ---: | ---: | ---: |",
    ]
    for row in rows:
        lines.append(
            f"| {row['candidate']} | {'PASS' if row['synthetic_passed'] else 'FAIL'} | "
            f"{row['successful_stages']}/{row['n_stages']} | {row['max_nfev_stages']} | "
            f"{row['rank_deficient_stages']} | "
            f"{'PASS' if row['objective_guard_passed'] else 'FAIL'} | "
            f"{row['median_nfev']:.1f} | "
            f"{row['median_common_scaled_gradient_inf_norm']:.3g} | "
            f"{row['maximum_common_scaled_jacobian_condition']:.3g} |"
        )
    lines += [
        "",
        "The selected setting must not be promoted when status is "
        "`best_available_but_gates_failed`.",
        "",
    ]
    with open(os.path.join(out_dir, "convergence_study.md"), "w") as handle:
        handle.write("\n".join(lines))


def deserialize_state(raw: dict) -> PoseState:
    return PoseState(
        cams={int(ci): np.asarray(T, dtype=np.float64)
              for ci, T in raw["T_base_Ci"].items()},
        gtc=np.asarray(raw["T_gripper_cam"], dtype=np.float64),
        board=(None if raw["T_base_board"] is None else
               np.asarray(raw["T_base_board"], dtype=np.float64)),
        cubes={int(s): np.asarray(T, dtype=np.float64)
               for s, T in raw["T_base_cube_by_set"].items()},
    )


def diagnose_existing_report(args) -> None:
    """Re-evaluate final-state Jacobians once; never solve or score held-out data."""
    with open(args.diagnose_report) as handle:
        source = json.load(handle)
    candidate_name = (args.diagnose_candidate
                      or source.get("selection", {}).get("selected_candidate"))
    if candidate_name not in source.get("candidates", {}):
        raise ValueError(f"candidate {candidate_name!r} absent from diagnostic report")
    source_candidate = source["candidates"][candidate_name]
    candidate = source_candidate["candidate"]
    cargs = candidate_args(args, candidate)
    cargs.loss = str(source.get("protocol", {}).get("loss", cargs.loss))
    cargs.f_scale_px = float(
        source.get("protocol", {}).get("f_scale_px", cargs.f_scale_px))
    cargs.max_nfev = 1
    prepared = prepare_ablation_data(args)
    by_row = {condition.row: condition for condition in MAIN_ABLATION_CONDITIONS}
    output = {
        "source_report": args.diagnose_report,
        "candidate": candidate_name,
        "selection_data": "calibration_train_only_final_state_jacobian",
        "heldout_metrics_computed": False,
        "rows": {},
    }
    for row, entry in source_candidate["rows"].items():
        condition = by_row[row]
        output["rows"][row] = []
        for run in entry["runs"]:
            state = deserialize_state(run["transforms"])
            relevant = filter_observations(
                prepared.train_obs, condition, None, prepared.gripper,
                state.cams)
            stage_output = {}
            if condition.unified == "seq":
                spec = SEQUENTIAL_STAGE_SPECS[row]
                stage_specs = (
                    ("stage1_eih", "eih", spec.stage1_free),
                    ("stage2_e2h", "e2h", spec.stage2_free),
                )
            else:
                stage_specs = ((
                    "joint_eih_e2h", None, UNIFIED_FREE_VARIABLES[row]),)
            for stage_name, role, free in stage_specs:
                observations = filter_observations(
                    relevant, condition, role, prepared.gripper, state.cams)
                _, diag = solve_stage(
                    observations, free, state, prepared.robot_T,
                    prepared.K_map, prepared.D_map, prepared.gripper,
                    seed=0, args=cargs)
                stage_output[stage_name] = {
                    "n_parameters": diag["n_parameters"],
                    "n_residuals": diag["n_residuals"],
                    "common_scaled_gradient_inf_norm": diag[
                        "common_scaled_gradient_inf_norm"],
                    "common_scaled_jacobian": diag["common_scaled_jacobian"],
                }
            output["rows"][row].append({
                "seed": int(run["seed"]), "stages": stage_output})
    out_dir = os.path.dirname(os.path.abspath(args.diagnose_report))
    out_path = os.path.join(out_dir, "weak_direction_diagnostics.json")
    with open(out_path, "w") as handle:
        json.dump(_jsonable(output), handle, indent=2)
    print(f"[SAVE] {out_path}")


def parse_args():
    parser = argparse.ArgumentParser(
        description="Train-only convergence study for the canonical corner backend")
    parser.add_argument("--root_folder", default="data/session")
    parser.add_argument("--intrinsics_dir", default="intrinsics")
    parser.add_argument("--calib_dir", default="data/session/calib_out")
    parser.add_argument("--out_dir", default="CP_result/convergence_study")
    parser.add_argument("--rows", default="A1,A2,A3")
    parser.add_argument("--candidates", default=",".join(
        candidate["name"] for candidate in CANDIDATES),
        help="Comma-separated candidate names; use a subset for confirmation runs.")
    parser.add_argument("--test_fraction", type=float, default=0.2)
    parser.add_argument("--split_seed", type=int, default=20260729)
    parser.add_argument("--min_train_eih_cube_events", type=int, default=3)
    parser.add_argument("--num_inits", type=int, default=3)
    parser.add_argument("--init_translation_mm", type=float, default=5.0)
    parser.add_argument("--init_rotation_deg", type=float, default=1.0)
    parser.add_argument("--max_nfev", type=int, default=300)
    parser.add_argument("--tol", type=float, default=1e-8)
    parser.add_argument("--loss", choices=["huber", "soft_l1", "linear"], default="soft_l1")
    parser.add_argument("--f_scale_px", type=float, default=2.0)
    parser.add_argument("--synthetic_only", action="store_true")
    parser.add_argument("--diagnose_report",
                        help="Existing convergence_study.json whose final train Jacobians are diagnosed.")
    parser.add_argument("--diagnose_candidate",
                        help="Candidate inside --diagnose_report; defaults to its selected candidate.")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    validate_main_runner_contract()
    if args.diagnose_report:
        diagnose_existing_report(args)
        return
    by_row = {condition.row: condition for condition in MAIN_ABLATION_CONDITIONS}
    rows = [row.strip() for row in args.rows.split(",") if row.strip()]
    unknown = [row for row in rows if row not in by_row]
    if unknown:
        raise ValueError(f"unknown rows: {unknown}")
    candidate_by_name = {candidate["name"]: candidate for candidate in CANDIDATES}
    candidate_names = [
        name.strip() for name in args.candidates.split(",") if name.strip()]
    unknown_candidates = [name for name in candidate_names if name not in candidate_by_name]
    if unknown_candidates:
        raise ValueError(f"unknown candidates: {unknown_candidates}")
    selected_candidates = [candidate_by_name[name] for name in candidate_names]
    if not selected_candidates:
        raise ValueError("at least one convergence candidate is required")
    report: Dict[str, Any] = {
        "protocol": {
            "selection_data": "noise_free_synthetic_and_calibration_train_only",
            "heldout_metrics_computed": False,
            "rows": rows,
            "candidate_subset": candidate_names,
            "num_inits": int(args.num_inits),
            "max_nfev": int(args.max_nfev),
            "tol": float(args.tol),
            "loss": str(args.loss),
            "f_scale_px": float(args.f_scale_px),
            "objective_relative_tolerance": OBJECTIVE_RELATIVE_TOLERANCE,
            "common_diagnostic_coordinate": {
                "rotation_scale_rad": 1.0,
                "translation_scale_m": 0.5,
            },
        },
        "candidates": {},
    }
    print("[SYNTHETIC] candidate gates", flush=True)
    for candidate in selected_candidates:
        cargs = candidate_args(args, candidate)
        print(f"  {candidate['name']}", flush=True)
        sanity = run_noise_free_sanity(cargs)
        report["candidates"][candidate["name"]] = {
            "candidate": dict(candidate),
            "synthetic_sanity": sanity,
            "rows": {},
        }
    if args.synthetic_only:
        for result in report["candidates"].values():
            result["summary"] = {
                "synthetic_passed": result["synthetic_sanity"]["passed"]}
        os.makedirs(args.out_dir, exist_ok=True)
        with open(os.path.join(args.out_dir, "convergence_study_synthetic.json"), "w") as handle:
            json.dump(_jsonable(report), handle, indent=2)
        return

    prepared = prepare_ablation_data(args)
    report["protocol"]["split"] = prepared.split
    report["protocol"]["heldout_observation_count_not_evaluated"] = len(prepared.test_obs)
    initials = {}
    for row in rows:
        initials[row], _ = make_initial_state(
            by_row[row], prepared.train_obs, prepared.gripper,
            prepared.robot_T, prepared.K_map, prepared.D_map,
            prepared.board_gtc, prepared.board_initial,
            prepared.visual_cubes, prepared.fixed_cubes,
            prepared.fixed_gtc_initial)
    for candidate in selected_candidates:
        name = candidate["name"]
        cargs = candidate_args(args, candidate)
        print(f"[TRAIN] {name}", flush=True)
        for row in rows:
            print(f"  row={row}", flush=True)
            runs = []
            for seed in range(int(args.num_inits)):
                run = run_train_only(
                    by_row[row], initials[row], prepared.train_obs,
                    prepared, seed, cargs)
                runs.append(run)
                statuses = [stage["status"] for stage in run["stages"].values()]
                print(
                    f"    seed={seed} converged={run['converged']} "
                    f"status={statuses} train={run['train_reprojection_rmse_px']:.6f}",
                    flush=True)
            report["candidates"][name]["rows"][row] = {
                "runs": runs,
                "initialization_dispersion": transform_dispersion(runs),
            }
        report["candidates"][name]["summary"] = summarize_candidate(
            report["candidates"][name])
        os.makedirs(args.out_dir, exist_ok=True)
        with open(os.path.join(args.out_dir, "convergence_study_checkpoint.json"), "w") as handle:
            json.dump(_jsonable(report), handle, indent=2)
    apply_objective_guard(report["candidates"], rows, int(args.num_inits))
    report["selection"] = select_candidate(report["candidates"])
    report["selection"]["selected_solver_options"].update({
        "loss": str(args.loss),
        "f_scale_px": float(args.f_scale_px),
        "max_nfev": int(args.max_nfev),
        "xtol": float(args.tol),
        "ftol": float(args.tol),
        "gtol": float(args.tol),
    })
    write_outputs(report, args.out_dir)
    print(
        f"[SELECT] {report['selection']['status']}: "
        f"{report['selection']['selected_candidate']}", flush=True)


if __name__ == "__main__":
    main()
