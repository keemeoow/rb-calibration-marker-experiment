#!/usr/bin/env python3
"""Predeclared repeated event-split evaluation for the canonical seven rows.

Each split runs the unchanged ``CP_ablation_7row.py`` entry point.  The
aggregator keeps initialization variation within a split separate from the
variation of split means, and computes contrasts only on components declared
compatible in ``CP_ablation_schema.py``.
"""

from __future__ import annotations

import argparse
import csv
import json
import os
import subprocess
import sys
from collections import defaultdict
from typing import Mapping, Sequence

import numpy as np

import CP_common
from CP_ablation_7row import validate_result_evaluation_contract
from CP_ablation_schema import EVALUATION_COMPARISON_CONTRACT


DEFAULT_SPLIT_SEEDS = (20260729, 20260730, 20260731, 20260732, 20260733)
ALL_ROWS = ("A0", "A1", "A2", "A3", "B1", "B2", "B3")


def _mean_std(values: Sequence[float]) -> tuple[float | None, float | None]:
    finite = np.asarray([value for value in values if value is not None], dtype=float)
    if not len(finite):
        return None, None
    return float(np.mean(finite)), float(np.std(finite))


def _run_metric(run: Mapping, component: str) -> float | None:
    if component == "N_reg":
        raise ValueError("N_reg is stored at row level")
    if component.startswith("heldout_reprojection."):
        group = component.split(".", 1)[1]
        return run.get("heldout_reprojection", {}).get(group, {}).get("rmse_px")
    if component == "e_e2e":
        return run.get("heldout_path_metrics", {}).get(
            "e_e2e_translation_rmse_mm")
    if component == "e_cross":
        return run.get("heldout_path_metrics", {}).get(
            "e_cross_translation_rmse_mm")
    if component == "e_e2e_rotation":
        return run.get("heldout_path_metrics", {}).get(
            "e_e2e_rotation_rmse_deg")
    if component == "e_cross_rotation":
        return run.get("heldout_path_metrics", {}).get(
            "e_cross_rotation_rmse_deg")
    raise KeyError(component)


def summarize_split(result: Mapping, split_seed: int) -> list[dict]:
    validate_result_evaluation_contract(result)
    rows = []
    for row in ALL_ROWS:
        entry = result["rows"][row]
        runs = entry["runs"]
        if len(runs) < 5:
            raise ValueError(f"{row}: repeated-split protocol requires >=5 initializations")
        if not all(run.get("converged") for run in runs):
            raise ValueError(f"{row}: non-converged initialization in split {split_seed}")
        record = {
            "split_seed": int(split_seed),
            "row": row,
            "n_initializations": len(runs),
            "n_registered_cams": int(entry["n_registered_cams"]),
            "evaluation_mask_sha256": entry["path_evaluation_mask_sha256"],
        }
        for name, component in (
                ("overall_reprojection_px", "heldout_reprojection.overall"),
                ("cube_reprojection_px", "heldout_reprojection.cube"),
                ("board_reprojection_px", "heldout_reprojection.board"),
                ("e_e2e_translation_mm", "e_e2e"),
                ("e_e2e_rotation_deg", "e_e2e_rotation"),
                ("e_cross_translation_mm", "e_cross"),
                ("e_cross_rotation_deg", "e_cross_rotation")):
            mean, std = _mean_std([_run_metric(run, component) for run in runs])
            record[f"{name}_init_mean"] = mean
            record[f"{name}_init_std"] = std
        camera_ids = sorted({
            int(key.removeprefix("cam_"))
            for run in runs
            for key in run.get("heldout_reprojection", {})
            if key.startswith("cam_") and key.removeprefix("cam_").isdigit()
        })
        for camera in camera_ids:
            values = [run["heldout_reprojection"].get(
                f"cam_{camera}", {}).get("rmse_px") for run in runs]
            mean, std = _mean_std(values)
            record[f"cam_{camera}_reprojection_px_init_mean"] = mean
            record[f"cam_{camera}_reprojection_px_init_std"] = std
        rows.append(record)
    return rows


def summarize_effects(per_split_rows: Sequence[Mapping]) -> list[dict]:
    by_split_row = {
        (int(record["split_seed"]), str(record["row"])): record
        for record in per_split_rows
    }
    split_seeds = sorted({int(record["split_seed"]) for record in per_split_rows})
    component_fields = {
        "heldout_reprojection.overall": "overall_reprojection_px_init_mean",
        "heldout_reprojection.cube": "cube_reprojection_px_init_mean",
        "heldout_reprojection.board": "board_reprojection_px_init_mean",
        "e_e2e": "e_e2e_translation_mm_init_mean",
        "e_cross": "e_cross_translation_mm_init_mean",
        "N_reg": "n_registered_cams",
    }
    effect_rows = []
    for contrast, contract in EVALUATION_COMPARISON_CONTRACT.items():
        first, second = contract["rows"]
        for component in contract["components"]:
            field = component_fields[component]
            deltas = []
            per_split = []
            for split_seed in split_seeds:
                first_value = by_split_row[(split_seed, first)].get(field)
                second_value = by_split_row[(split_seed, second)].get(field)
                if first_value is None or second_value is None:
                    continue
                delta = float(second_value) - float(first_value)
                deltas.append(delta)
                per_split.append({"split_seed": split_seed, "second_minus_first": delta})
            mean, std = _mean_std(deltas)
            lower_is_better = component != "N_reg"
            effect_rows.append({
                "contrast": contrast,
                "first_row": first,
                "second_row": second,
                "component": component,
                "delta_definition": "second_minus_first",
                "lower_is_better": lower_is_better,
                "n_splits": len(deltas),
                "delta_mean": mean,
                "delta_split_std": std,
                "n_splits_second_better": sum(
                    delta < 0 if lower_is_better else delta > 0
                    for delta in deltas),
                "causal_interpretation": contract.get("causal_interpretation"),
                "interpretation": contract.get("interpretation"),
                "per_split": per_split,
            })
    return effect_rows


def aggregate_results(results: Sequence[tuple[int, Mapping]]) -> dict:
    per_split_rows = []
    split_artifacts = []
    for split_seed, result in results:
        per_split_rows.extend(summarize_split(result, split_seed))
        split_artifacts.append({
            "split_seed": int(split_seed),
            "train_events": result["protocol"]["split"]["train_events"],
            "test_events": result["protocol"]["split"]["test_events"],
            "evaluation_mask_sha256": result["protocol"]
                ["model_independent_path_evaluation_mask"]
                ["evaluation_mask_sha256"],
            "fk_artifact_sha256": result["shared_fk_cube_artifact"]["sha256"],
        })

    across_split = []
    camera_ids = sorted({
        int(key.split("_", 2)[1])
        for record in per_split_rows for key in record
        if key.startswith("cam_") and key.endswith("_reprojection_px_init_mean")
    })
    for row in ALL_ROWS:
        records = [record for record in per_split_rows if record["row"] == row]
        output = {"row": row, "n_splits": len(records)}
        metric_names = (
            "overall_reprojection_px", "cube_reprojection_px",
            "board_reprojection_px", "e_e2e_translation_mm",
            "e_e2e_rotation_deg", "e_cross_translation_mm",
            "e_cross_rotation_deg")
        for metric in metric_names:
            split_means = [record[f"{metric}_init_mean"] for record in records]
            mean, split_std = _mean_std(split_means)
            init_stds = [record[f"{metric}_init_std"] for record in records]
            init_std_mean, _ = _mean_std(init_stds)
            output[f"{metric}_mean_across_splits"] = mean
            output[f"{metric}_split_std"] = split_std
            output[f"{metric}_mean_within_split_init_std"] = init_std_mean
        for camera in camera_ids:
            metric = f"cam_{camera}_reprojection_px"
            split_means = [record.get(f"{metric}_init_mean") for record in records]
            mean, split_std = _mean_std(split_means)
            init_stds = [record.get(f"{metric}_init_std") for record in records]
            init_std_mean, _ = _mean_std(init_stds)
            output[f"{metric}_mean_across_splits"] = mean
            output[f"{metric}_split_std"] = split_std
            output[f"{metric}_mean_within_split_init_std"] = init_std_mean
        across_split.append(output)
    return {
        "protocol": {
            "schema": "canonical_ablation_repeated_event_split_v1",
            # Load-time robot translation scale.  Table 1 once mixed two values
            # across its rows; recording it here makes that checkable without
            # re-deriving the FK cube poses.
            "robot_pos_scale": CP_common.robot_pos_scale(),
            "split_seeds_predeclared": [int(seed) for seed, _ in results],
            "minimum_initializations_per_split": 5,
            "delta_definition": "second_row_minus_first_row",
            "test_data_used_for_optimizer_selection": False,
            "target_set_comparisons_use_common_components_only": True,
            "camera_ids_reported_separately": camera_ids,
        },
        "split_artifacts": split_artifacts,
        "per_split_rows": per_split_rows,
        "across_split_rows": across_split,
        "effects": summarize_effects(per_split_rows),
    }


def _write_csv(path: str, rows: Sequence[Mapping]) -> None:
    flat_rows = []
    for row in rows:
        flat_rows.append({key: value for key, value in row.items()
                          if not isinstance(value, (list, dict))})
    fieldnames = []
    for row in flat_rows:
        for key in row:
            if key not in fieldnames:
                fieldnames.append(key)
    if not fieldnames:
        fieldnames = ["empty"]
    with open(path, "w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(flat_rows)


def write_aggregate(aggregate: Mapping, out_dir: str) -> None:
    os.makedirs(out_dir, exist_ok=True)
    with open(os.path.join(out_dir, "multisplit_ablation.json"), "w") as handle:
        json.dump(aggregate, handle, indent=2)
    _write_csv(os.path.join(out_dir, "per_split_metrics.csv"),
               aggregate["per_split_rows"])
    _write_csv(os.path.join(out_dir, "effect_summary.csv"), aggregate["effects"])
    lines = [
        "# Canonical repeated event-split ablation",
        "",
        "Deltas are second row minus first row. Negative is better for all error metrics; "
        "positive is better only for N_reg. Split standard deviation is kept separate from "
        "within-split initialization standard deviation.",
        "",
        "| Contrast | Component | splits | delta mean±split std | second better |",
        "| --- | --- | ---: | ---: | ---: |",
    ]
    for effect in aggregate["effects"]:
        if effect["delta_mean"] is None:
            formatted = "—"
        else:
            formatted = f"{effect['delta_mean']:.6f}±{effect['delta_split_std']:.6f}"
        lines.append(
            f"| {effect['contrast']} | {effect['component']} | {effect['n_splits']} | "
            f"{formatted} | {effect['n_splits_second_better']}/{effect['n_splits']} |")
    camera_ids = aggregate["protocol"].get("camera_ids_reported_separately", [])
    lines += [
        "",
        "## Held-out reprojection by camera across splits",
        "",
        "Values are mean of split means ± standard deviation across split means.",
        "",
        "| Row | " + " | ".join(f"cam {camera} (px)" for camera in camera_ids) + " |",
        "| --- | " + " | ".join("---:" for _ in camera_ids) + " |",
    ]
    for row in aggregate["across_split_rows"]:
        values = []
        for camera in camera_ids:
            mean = row.get(f"cam_{camera}_reprojection_px_mean_across_splits")
            std = row.get(f"cam_{camera}_reprojection_px_split_std")
            values.append("—" if mean is None else f"{mean:.4f}±{std:.4f}")
        lines.append(f"| {row['row']} | " + " | ".join(values) + " |")
    lines += [
        "",
        "B3_to_A3 remains a whole-system reference and has no marker-only causal interpretation.",
        "No position-holdout value in this artifact is external physical ground truth.",
        "",
    ]
    with open(os.path.join(out_dir, "multisplit_ablation.md"), "w") as handle:
        handle.write("\n".join(lines))


def parse_args():
    parser = argparse.ArgumentParser(description="Repeated event-split canonical ablation")
    parser.add_argument("--root_folder", default="data/session")
    parser.add_argument("--intrinsics_dir", default="intrinsics")
    parser.add_argument("--calib_dir", default="data/session/calib_out")
    parser.add_argument("--out_dir", default="CP_result/ablation_multisplit")
    parser.add_argument(
        "--split_seeds", default=",".join(map(str, DEFAULT_SPLIT_SEEDS)))
    parser.add_argument("--num_inits", type=int, default=5)
    parser.add_argument("--max_nfev", type=int, default=300)
    parser.add_argument("--tol", type=float, default=1e-8)
    parser.add_argument("--force", action="store_true")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    split_seeds = tuple(int(value.strip()) for value in args.split_seeds.split(",")
                        if value.strip())
    if len(split_seeds) < 2 or len(set(split_seeds)) != len(split_seeds):
        raise ValueError("provide at least two unique, predeclared split seeds")
    if int(args.num_inits) < 5:
        raise ValueError("repeated-split protocol requires --num_inits >= 5")
    os.makedirs(args.out_dir, exist_ok=True)
    plan = {
        "schema": "canonical_ablation_repeated_event_split_plan_v1",
        "split_seeds_predeclared": list(split_seeds),
        "num_initializations_per_split": int(args.num_inits),
        "rows": list(ALL_ROWS),
        "optimizer": {"max_nfev": int(args.max_nfev), "tol": float(args.tol)},
    }
    with open(os.path.join(args.out_dir, "predeclared_plan.json"), "w") as handle:
        json.dump(plan, handle, indent=2)

    results = []
    for split_seed in split_seeds:
        split_dir = os.path.join(args.out_dir, f"split_{split_seed}")
        result_path = os.path.join(split_dir, "seven_row_ablation.json")
        if not args.force and os.path.exists(result_path):
            with open(result_path) as handle:
                result = json.load(handle)
            valid_seed = int(result["protocol"]["split"]["seed"]) == split_seed
            valid_inits = all(
                len(entry["runs"]) >= int(args.num_inits)
                for entry in result.get("rows", {}).values())
            if valid_seed and valid_inits and set(result.get("rows", {})) == set(ALL_ROWS):
                validate_result_evaluation_contract(result)
                print(f"[REUSE] split seed {split_seed}", flush=True)
                results.append((split_seed, result))
                continue
        command = [
            sys.executable, "-u", "CP_ablation_7row.py",
            "--root_folder", args.root_folder,
            "--intrinsics_dir", args.intrinsics_dir,
            "--calib_dir", args.calib_dir,
            "--out_dir", split_dir,
            "--split_seed", str(split_seed),
            "--num_inits", str(args.num_inits),
            "--max_nfev", str(args.max_nfev),
            "--tol", str(args.tol),
        ]
        print(f"[RUN] split seed {split_seed}", flush=True)
        subprocess.run(command, check=True)
        with open(result_path) as handle:
            result = json.load(handle)
        validate_result_evaluation_contract(result)
        results.append((split_seed, result))
        write_aggregate(aggregate_results(results), args.out_dir)
    write_aggregate(aggregate_results(results), args.out_dir)
    print(f"[SAVE] {args.out_dir}/multisplit_ablation.{{json,md}}", flush=True)


if __name__ == "__main__":
    main()
