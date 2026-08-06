#!/usr/bin/env python3
"""Controlled real-data sensitivity study for Table 4.

The physical task experiment is intentionally outside this runner.  This file
uses the canonical A0/A2/B2/A3 corner backend and changes exactly one data axis
at a time:

* global training event budget N, balanced over five fixed cube positions;
* total camera count = one eye-in-hand plus one/two/three fixed cameras;
* detector raster scale (0.5x versus native), with points mapped back to native
  pixel coordinates before optimization.

Every condition is evaluated on one frozen native-resolution held-out pool
containing the eye-in-hand camera and fixed camera 0.  Consequently neither the
test events nor the evaluation cameras change with N, camera count, or raster
scale.  Non-converged repetitions remain diagnostic and do not receive a
headline numeric value.
"""

from __future__ import annotations

import argparse
import csv
import json
import os
from collections import defaultdict
from copy import deepcopy
from typing import Dict, Iterable, List, Mapping, Optional, Sequence, Tuple

import numpy as np

import Step3_calibration as s3
from CP_ablation_7row import (
    average_visual_target,
    canonical_solver_options,
    detect_observations,
    estimate_board_handeye_initial,
    make_initial_state,
    prepare_ablation_data,
    run_condition_once,
)
from CP_ablation_schema import MAIN_ABLATION_CONDITIONS, validate_fk_alignment_artifact
from calibration_fk_cube_artifact import estimate_board_free_fk_cube_artifact
from calibration_path_evaluation import (
    build_common_path_evaluation_mask,
    validate_common_path_evaluation_mask,
)
from calibration_runtime_utils import get_capture_set_index


ROWS = ("A0", "A2", "B2", "A3")
DEFAULT_CORE_SETS = (0, 2, 6, 9, 12)
DEFAULT_VIEW_BUDGETS = (5, 10, 20, 40)
DEFAULT_CAMERA_TOTALS = (2, 3, 4)


def _jsonable(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, (np.integer,)):
        return int(value)
    if isinstance(value, (np.floating,)):
        return float(value)
    if isinstance(value, dict):
        return {str(key): _jsonable(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_jsonable(item) for item in value]
    return value


def parse_ints(value: str) -> Tuple[int, ...]:
    return tuple(int(item.strip()) for item in value.split(",") if item.strip())


def supported_eih_events_by_set(observations, gripper: int,
                                 set_ids: Sequence[int]) -> Dict[int, List[int]]:
    """Events with both board and cube eih measurements, grouped by set."""
    allowed = {int(set_index) for set_index in set_ids}
    markers: Dict[Tuple[int, int], set] = defaultdict(set)
    for obs in observations:
        if (obs.set_idx is None or int(obs.set_idx) not in allowed
                or int(obs.cam) != int(gripper)):
            continue
        markers[(int(obs.set_idx), int(obs.event))].add(str(obs.marker))
    output = {set_index: [] for set_index in sorted(allowed)}
    for (set_index, event), present in markers.items():
        if {"board", "cube"}.issubset(present):
            output[set_index].append(event)
    return {set_index: sorted(events) for set_index, events in output.items()}


def select_balanced_events(events_by_set: Mapping[int, Sequence[int]],
                           set_ids: Sequence[int], budget: int,
                           seed: int) -> List[int]:
    """Choose a deterministic nested, equal-size event subset per position."""
    set_ids = tuple(int(set_index) for set_index in set_ids)
    budget = int(budget)
    if not set_ids or budget <= 0 or budget % len(set_ids):
        raise ValueError(
            "view budget must be positive and divisible by the number of core sets")
    per_set = budget // len(set_ids)
    selected = []
    for set_index in set_ids:
        candidates = list(sorted(int(event) for event in events_by_set.get(set_index, ())))
        if len(candidates) < per_set:
            raise RuntimeError(
                f"set {set_index} has {len(candidates)} supported events; "
                f"{per_set} required for N={budget}")
        rng = np.random.default_rng(
            np.random.SeedSequence([int(seed), int(set_index), 0x51A7]))
        order = np.asarray(candidates, dtype=np.int64)
        rng.shuffle(order)
        selected.extend(int(event) for event in order[:per_set])
    return sorted(selected)


def filter_observation_pool(observations, set_ids: Iterable[int],
                            event_ids: Iterable[int], camera_ids: Iterable[int]):
    sets = {int(value) for value in set_ids}
    events = {int(value) for value in event_ids}
    cameras = {int(value) for value in camera_ids}
    return [
        obs for obs in observations
        if obs.set_idx is not None
        and int(obs.set_idx) in sets
        and int(obs.event) in events
        and int(obs.cam) in cameras
    ]


def build_raw_fk(meta: Mapping, event_ids: Sequence[int]):
    selected = {int(event) for event in event_ids}
    train_meta = dict(meta)
    train_meta["captures"] = [
        capture for capture in meta.get("captures", [])
        if int(capture.get("event_id", -1)) in selected]
    raw_fk = s3.load_nominal_set_cube_transforms(train_meta)
    source_event = {}
    for capture in train_meta["captures"]:
        set_index = get_capture_set_index(capture)
        if set_index is not None and int(set_index) not in source_event:
            source_event[int(set_index)] = int(capture["event_id"])
    return train_meta, raw_fk, source_event


def build_training_context(train_obs, meta, robot_T, K_map, D_map,
                           gripper: int, core_sets: Sequence[int], args) -> dict:
    """Build row-specific train-only initializers without cross-row leakage."""
    context = {"errors": {}}
    event_ids = sorted({int(obs.event) for obs in train_obs})
    train_meta, raw_fk, raw_source = build_raw_fk(meta, event_ids)
    missing_raw = sorted(set(core_sets) - set(raw_fk))
    if missing_raw:
        context["errors"]["raw_fk"] = f"raw FK missing for sets {missing_raw}"

    eih_board = [
        obs for obs in train_obs
        if obs.marker == "board" and int(obs.cam) == int(gripper)]
    try:
        board_gtc, board_initial, handeye_diag = estimate_board_handeye_initial(
            eih_board, robot_T, K_map, D_map, gripper)
        visual_cubes = average_visual_target(
            train_obs, "cube", board_gtc, robot_T, K_map, D_map, gripper)
        missing_visual = sorted(set(core_sets) - set(visual_cubes))
        if missing_visual:
            raise RuntimeError(
                f"visual cube initializer missing sets {missing_visual}")
        context.update({
            "board_gtc": board_gtc,
            "board_initial": board_initial,
            "handeye_diagnostics": handeye_diag,
            "visual_cubes": visual_cubes,
        })
    except Exception as exc:  # recorded as a scientific non-convergence state
        context["errors"]["estimated_pose_initialization"] = str(exc)

    if not missing_raw:
        try:
            aligned, fixed_gtc, artifact = estimate_board_free_fk_cube_artifact(
                observations=train_obs,
                raw_fk_by_set=raw_fk,
                robot_T=robot_T,
                K_map=K_map,
                D_map=D_map,
                gripper_cam_idx=gripper,
                training_set_ids=core_sets,
                options=canonical_solver_options(args),
                num_inits=int(args.artifact_num_inits),
                init_translation_mm=float(args.init_translation_mm),
                init_rotation_deg=float(args.init_rotation_deg),
                raw_fk_source_event_by_set=raw_source,
            )
            validate_fk_alignment_artifact(artifact)
            context.update({
                "fixed_cubes": {int(s): aligned[int(s)] for s in core_sets},
                "fixed_gtc_initial": fixed_gtc,
                "alignment_artifact": artifact,
            })
        except Exception as exc:  # do not replace this with a bad headline value
            context["errors"]["fk_fixed_initialization"] = str(exc)
    context["train_event_ids"] = event_ids
    context["train_meta_capture_count"] = len(train_meta.get("captures", []))
    return context


def _row_requirements_available(row: str, context: Mapping) -> Optional[str]:
    errors = context.get("errors", {})
    if row in {"A0", "A2"} and "estimated_pose_initialization" in errors:
        return errors["estimated_pose_initialization"]
    if row in {"B2", "A3"} and "fk_fixed_initialization" in errors:
        return errors["fk_fixed_initialization"]
    return None


def _metric_mean(runs: Sequence[Mapping], family: str, component: str,
                 key: str) -> Optional[float]:
    values = []
    for run in runs:
        item = run.get(family, {}).get(component, {})
        value = item.get(key)
        if value is not None and np.isfinite(float(value)):
            values.append(float(value))
    return float(np.mean(values)) if len(values) == len(runs) and values else None


def summarize_row_runs(runs: Sequence[Mapping]) -> dict:
    all_converged = bool(runs) and all(bool(run.get("converged")) for run in runs)
    path_keys = (
        "e_e2e_translation_rmse_mm", "e_e2e_rotation_rmse_deg",
        "e_cross_translation_rmse_mm", "e_cross_rotation_rmse_deg",
    )
    metrics = {
        component: _metric_mean(
            runs, "heldout_reprojection", component, "rmse_px")
        for component in ("overall", "board", "cube")
    }
    path = {}
    for key in path_keys:
        values = [run.get("heldout_path_metrics", {}).get(key) for run in runs]
        path[key] = (float(np.mean(values))
                     if values and all(value is not None for value in values)
                     else None)
    return {
        "status": "converged" if all_converged else "unstable",
        "headline_metric_available": all_converged,
        "n_converged_inits": int(sum(bool(run.get("converged")) for run in runs)),
        "n_inits": len(runs),
        "heldout_reprojection_rmse_px": metrics,
        "heldout_path_metrics": path,
        "runs": list(runs),
    }


def run_subset(axis: str, level: str, subset_seed: int,
               train_source, train_event_ids: Sequence[int],
               train_camera_ids: Sequence[int], test_obs,
               path_mask, base, core_sets: Sequence[int], args) -> dict:
    train_obs = filter_observation_pool(
        train_source, core_sets, train_event_ids, train_camera_ids)
    context = build_training_context(
        train_obs, base.meta, base.robot_T, base.K_map, base.D_map,
        base.gripper, core_sets, args)
    by_row = {condition.row: condition for condition in MAIN_ABLATION_CONDITIONS}
    output = {
        "axis": axis,
        "level": str(level),
        "subset_seed": int(subset_seed),
        "train_event_ids": list(map(int, train_event_ids)),
        "train_camera_ids": list(map(int, train_camera_ids)),
        "n_train_observations": len(train_obs),
        "train_support": {
            "board": int(sum(obs.marker == "board" for obs in train_obs)),
            "cube": int(sum(obs.marker == "cube" for obs in train_obs)),
        },
        "initialization_errors": context.get("errors", {}),
        "shared_fk_artifact_sha256": context.get(
            "alignment_artifact", {}).get("artifact_sha256"),
        "rows": {},
    }
    for row in ROWS:
        unavailable = _row_requirements_available(row, context)
        if unavailable is not None:
            output["rows"][row] = {
                "status": "unavailable",
                "headline_metric_available": False,
                "error": unavailable,
                "n_converged_inits": 0,
                "n_inits": int(args.num_inits),
            }
            continue
        condition = by_row[row]
        try:
            initial, init_diag = make_initial_state(
                condition, train_obs, base.gripper, base.robot_T,
                base.K_map, base.D_map,
                context.get("board_gtc", np.eye(4)),
                context.get("board_initial", np.eye(4)),
                context.get("visual_cubes", {}),
                context.get("fixed_cubes", {}),
                context.get("fixed_gtc_initial", np.eye(4)),
            )
            runs = [
                run_condition_once(
                    condition, initial, train_obs, test_obs,
                    base.gripper, base.robot_T, base.K_map, base.D_map,
                    init_seed, args, path_mask)
                for init_seed in range(int(args.num_inits))
            ]
            output["rows"][row] = {
                "initialization": init_diag,
                "n_registered_fixed_cameras": len(initial.cams),
                **summarize_row_runs(runs),
            }
        except Exception as exc:
            output["rows"][row] = {
                "status": "unavailable",
                "headline_metric_available": False,
                "error": str(exc),
                "n_converged_inits": 0,
                "n_inits": int(args.num_inits),
            }
    return output


def _aggregate_values(values: Sequence[Optional[float]], all_available: bool) -> dict:
    diagnostics = [float(value) for value in values
                   if value is not None and np.isfinite(float(value))]
    return {
        "mean": (float(np.mean(diagnostics))
                 if all_available and len(diagnostics) == len(values) else None),
        "std": (float(np.std(diagnostics))
                if all_available and len(diagnostics) == len(values) else None),
        "diagnostic_mean_available_runs": (
            float(np.mean(diagnostics)) if diagnostics else None),
        "n_numeric": len(diagnostics),
    }


def aggregate_results(repetitions: Sequence[Mapping]) -> List[dict]:
    groups = defaultdict(list)
    for repetition in repetitions:
        for row, result in repetition.get("rows", {}).items():
            groups[(repetition["axis"], repetition["level"], row)].append(result)
    output = []
    for (axis, level, row), results in sorted(groups.items()):
        all_available = bool(results) and all(
            result.get("status") == "converged"
            and result.get("headline_metric_available") is True
            for result in results)
        entry = {
            "axis": axis,
            "level": level,
            "row": row,
            "status": "converged" if all_available else "unstable_or_unavailable",
            "headline_metric_available": all_available,
            "n_converged_subsets": int(sum(
                result.get("status") == "converged" for result in results)),
            "n_subsets": len(results),
            "heldout_reprojection_rmse_px": {},
            "heldout_path_metrics": {},
        }
        for component in ("overall", "board", "cube"):
            values = [result.get("heldout_reprojection_rmse_px", {}).get(component)
                      for result in results]
            entry["heldout_reprojection_rmse_px"][component] = _aggregate_values(
                values, all_available)
        for metric in (
                "e_e2e_translation_rmse_mm", "e_e2e_rotation_rmse_deg",
                "e_cross_translation_rmse_mm", "e_cross_rotation_rmse_deg"):
            values = [result.get("heldout_path_metrics", {}).get(metric)
                      for result in results]
            entry["heldout_path_metrics"][metric] = _aggregate_values(
                values, all_available)
        output.append(entry)
    return output


def aggregate_paired_cube_contrasts(repetitions: Sequence[Mapping]) -> List[dict]:
    """Pair B2/A3 by identical subset before computing the board-addition delta."""
    groups = defaultdict(list)
    totals = defaultdict(int)
    for repetition in repetitions:
        key = (repetition["axis"], repetition["level"])
        totals[key] += 1
        b2 = repetition.get("rows", {}).get("B2", {})
        a3 = repetition.get("rows", {}).get("A3", {})
        if b2.get("status") != "converged" or a3.get("status") != "converged":
            continue
        b2_value = b2.get("heldout_reprojection_rmse_px", {}).get("cube")
        a3_value = a3.get("heldout_reprojection_rmse_px", {}).get("cube")
        if b2_value is None or a3_value is None:
            continue
        groups[key].append(float(a3_value) - float(b2_value))
    output = []
    for key in sorted(totals):
        values = groups.get(key, [])
        complete = len(values) == totals[key]
        output.append({
            "axis": key[0],
            "level": key[1],
            "contrast": "B2_to_A3_common_cube",
            "delta_definition": "A3_minus_B2_px",
            "headline_metric_available": complete,
            "n_paired": len(values),
            "n_subsets": totals[key],
            "delta_mean_px": float(np.mean(values)) if complete else None,
            "delta_std_px": float(np.std(values)) if complete else None,
            "n_A3_improved": int(sum(value < 0.0 for value in values)),
            "paired_deltas_px": values,
        })
    return output


def _cell(entry: Optional[Mapping], component: str = "overall") -> str:
    if entry is None:
        return "—"
    if not entry.get("headline_metric_available"):
        return (f"unstable ({entry.get('n_converged_subsets', 0)}/"
                f"{entry.get('n_subsets', 0)})")
    value = entry["heldout_reprojection_rmse_px"][component]
    if value["mean"] is None:
        return "N/A"
    return f"{value['mean']:.4f}±{value['std']:.4f}"


def write_outputs(result: Mapping, out_dir: str) -> None:
    result = dict(result)
    result["paired_cube_contrasts"] = aggregate_paired_cube_contrasts(
        result.get("repetitions", []))
    os.makedirs(out_dir, exist_ok=True)
    with open(os.path.join(out_dir, "sensitivity.json"), "w") as handle:
        json.dump(_jsonable(result), handle, indent=2)

    with open(os.path.join(out_dir, "sensitivity.csv"), "w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow([
            "axis", "level", "row", "status", "n_converged_subsets", "n_subsets",
            "overall_mean_px", "overall_std_px", "board_mean_px", "board_std_px",
            "cube_mean_px", "cube_std_px", "e_e2e_t_mean_mm", "e_cross_t_mean_mm",
        ])
        for entry in result["summary"]:
            reproj = entry["heldout_reprojection_rmse_px"]
            path = entry["heldout_path_metrics"]
            writer.writerow([
                entry["axis"], entry["level"], entry["row"], entry["status"],
                entry["n_converged_subsets"], entry["n_subsets"],
                reproj["overall"]["mean"], reproj["overall"]["std"],
                reproj["board"]["mean"], reproj["board"]["std"],
                reproj["cube"]["mean"], reproj["cube"]["std"],
                path["e_e2e_translation_rmse_mm"]["mean"],
                path["e_cross_translation_rmse_mm"]["mean"],
            ])

    lookup = {(entry["axis"], entry["level"], entry["row"]): entry
              for entry in result["summary"]}
    contrast_lookup = {
        (entry["axis"], entry["level"]): entry
        for entry in result["paired_cube_contrasts"]}
    lines = [
        "# Table 4 — controlled real-data sensitivity",
        "",
        "Primary values are native-frame held-out corner reprojection RMSE (px), "
        "mean±population-std across predeclared subset seeds. A numeric headline is "
        "withheld unless every subset and optimizer initialization converged.",
        "",
        "- N is the total number of robot capture events, balanced equally over the "
        f"fixed positions `{result['protocol']['core_sets']}`.",
        "- Camera count includes the eye-in-hand camera. Camera subsets are nested; "
        "evaluation always uses eye-in-hand plus fixed camera 0.",
        "- `resolution_low` reruns marker detection at the declared raster scale, maps "
        "corners back to native pixels, and evaluates on the same native held-out pool.",
        "- These real-data values use held-out measurement agreement, not external "
        "physical ground truth.",
        "",
        "## Row-specific overall reprojection",
        "",
        "Target sets differ across columns, so this table is a within-row sensitivity "
        "summary; it is not a cross-row accuracy ranking.",
        "",
        "| condition | A0 | A2 | B2 | A3 |",
        "| --- | ---: | ---: | ---: | ---: |",
    ]
    ordered = []
    for budget in result["protocol"]["view_budgets"]:
        ordered.append(("views", f"N={budget}", f"views N={budget}"))
    for count in result["protocol"]["camera_totals"]:
        ordered.append(("cameras", f"cams={count}", f"cams={count}"))
    ordered.extend([
        ("resolution", "low", "resolution=low"),
        ("resolution", "native", "resolution=native"),
    ])
    for axis, level, label in ordered:
        lines.append("| " + label + " | " + " | ".join(
            _cell(lookup.get((axis, level, row))) for row in ROWS) + " |")

    lines += [
        "",
        "## Common-cube comparison — B2 versus A3",
        "",
        "This is the valid board-addition comparison. Both cells use only held-out "
        "cube corners; Δ = A3−B2, so a negative value favors adding the board.",
        "",
        "| condition | B2 cube (px) | A3 cube (px) | paired Δ (px) |",
        "| --- | ---: | ---: | ---: |",
    ]
    for axis, level, label in ordered:
        b2 = lookup.get((axis, level, "B2"))
        a3 = lookup.get((axis, level, "A3"))
        contrast = contrast_lookup.get((axis, level), {})
        delta = "—"
        if contrast.get("headline_metric_available"):
            delta = (
                f"{contrast['delta_mean_px']:+.4f}±{contrast['delta_std_px']:.4f} "
                f"({contrast['n_A3_improved']}/{contrast['n_subsets']})")
        lines.append(
            f"| {label} | {_cell(b2, 'cube')} | {_cell(a3, 'cube')} | {delta} |")
    lines += [
        "",
        "Raw per-subset convergence, corner support, solver diagnostics, and artifact "
        "hashes are retained in `sensitivity.json`.",
        "",
    ]
    with open(os.path.join(out_dir, "sensitivity.md"), "w") as handle:
        handle.write("\n".join(lines))


def parse_args():
    parser = argparse.ArgumentParser(description="Canonical Table 4 sensitivity runner")
    parser.add_argument("--root_folder", default="data/session")
    parser.add_argument("--intrinsics_dir", default="intrinsics")
    parser.add_argument("--calib_dir", default="data/session/calib_out")
    parser.add_argument("--out_dir", default="CP_result/sensitivity_7row")
    parser.add_argument("--axes", default="views,cameras,resolution")
    parser.add_argument("--core_sets", default=",".join(map(str, DEFAULT_CORE_SETS)))
    parser.add_argument("--view_budgets", default=",".join(map(str, DEFAULT_VIEW_BUDGETS)))
    parser.add_argument("--camera_totals", default=",".join(map(str, DEFAULT_CAMERA_TOTALS)))
    parser.add_argument(
        "--subset_seeds", default="20260729,20260730,20260731,20260732,20260733")
    parser.add_argument("--low_resolution_scale", type=float, default=0.5)
    parser.add_argument("--test_fraction", type=float, default=0.2)
    parser.add_argument("--split_seed", type=int, default=20260729)
    parser.add_argument("--min_train_eih_cube_events", type=int, default=3)
    parser.add_argument("--num_inits", type=int, default=1)
    parser.add_argument("--artifact_num_inits", type=int, default=2)
    parser.add_argument("--init_translation_mm", type=float, default=5.0)
    parser.add_argument("--init_rotation_deg", type=float, default=1.0)
    parser.add_argument("--max_nfev", type=int, default=300)
    parser.add_argument("--tol", type=float, default=1e-8)
    parser.add_argument("--rotation_scale_rad", type=float, default=1.0)
    parser.add_argument("--translation_scale_m", type=float, default=1.0)
    parser.add_argument("--x_scale_mode", choices=["unit", "jac"], default="jac")
    parser.add_argument("--loss", choices=["huber", "soft_l1", "linear"],
                        default="soft_l1")
    parser.add_argument("--f_scale_px", type=float, default=2.0)
    parser.add_argument("--image_scale", type=float, default=1.0,
                        help=argparse.SUPPRESS)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    axes = {item.strip() for item in args.axes.split(",") if item.strip()}
    unknown_axes = axes - {"views", "cameras", "resolution"}
    if unknown_axes:
        raise ValueError(f"unknown sensitivity axes: {sorted(unknown_axes)}")
    core_sets = parse_ints(args.core_sets)
    view_budgets = parse_ints(args.view_budgets)
    camera_totals = parse_ints(args.camera_totals)
    subset_seeds = parse_ints(args.subset_seeds)
    if tuple(sorted(view_budgets)) != view_budgets:
        raise ValueError("view_budgets must be strictly ascending")
    if not 0.0 < float(args.low_resolution_scale) < 1.0:
        raise ValueError("low_resolution_scale must lie strictly between 0 and 1")

    print("[PREP] native observations and frozen split")
    args.image_scale = 1.0
    base = prepare_ablation_data(args)
    native_pool = list(base.train_obs) + list(base.test_obs)
    train_events = set(base.split["train_events"])
    test_events = set(base.split["test_events"])
    missing_sets = sorted(set(core_sets) - set(base.split["eligible_sets"]))
    if missing_sets:
        raise RuntimeError(f"core sets absent from canonical split: {missing_sets}")

    fixed_cameras = sorted({
        int(obs.cam) for obs in native_pool if int(obs.cam) != int(base.gripper)})
    if not fixed_cameras or fixed_cameras[0] != 0:
        raise RuntimeError(
            "the predeclared common evaluation fixed camera 0 is unavailable")
    for total in camera_totals:
        if total < 2 or total - 1 > len(fixed_cameras):
            raise ValueError(
                f"cams={total} cannot be formed from eih+{len(fixed_cameras)} fixed cameras")
    full_camera_ids = [base.gripper] + fixed_cameras
    common_test_camera_ids = [base.gripper, 0]
    native_train_pool = filter_observation_pool(
        native_pool, core_sets, train_events, full_camera_ids)
    native_test_obs = filter_observation_pool(
        native_pool, core_sets, test_events, common_test_camera_ids)
    path_mask = build_common_path_evaluation_mask(
        observations=native_test_obs,
        fixed_camera_ids=[0],
        gripper_cam_idx=base.gripper,
        K_map=base.K_map,
        D_map=base.D_map,
        set_filter=core_sets,
        require_cross=False,
        require_e2e=True,
    )
    validate_common_path_evaluation_mask(path_mask)
    events_by_set = supported_eih_events_by_set(
        native_train_pool, base.gripper, core_sets)

    low_train_pool = None
    if "resolution" in axes:
        print(f"[PREP] low-resolution detections scale={args.low_resolution_scale}")
        low_args = deepcopy(args)
        low_args.image_scale = float(args.low_resolution_scale)
        all_camera_ids = sorted({
            int(ci) for capture in base.meta.get("captures", [])
            for ci in capture.get("cams", {})})
        low_all, _, _ = detect_observations(
            low_args, base.meta, base.K_map, base.D_map,
            all_camera_ids, base.gripper)
        low_train_pool = filter_observation_pool(
            low_all, core_sets, train_events, full_camera_ids)

    repetitions = []
    cache = {}
    max_budget = max(view_budgets)
    for subset_seed in subset_seeds:
        selected_by_budget = {
            budget: select_balanced_events(
                events_by_set, core_sets, budget, subset_seed)
            for budget in view_budgets
        }
        scenarios = []
        if "views" in axes:
            scenarios.extend((
                "views", f"N={budget}", native_train_pool,
                selected_by_budget[budget], full_camera_ids)
                for budget in view_budgets)
        if "cameras" in axes:
            scenarios.extend((
                "cameras", f"cams={total}", native_train_pool,
                selected_by_budget[max_budget],
                [base.gripper] + fixed_cameras[:total - 1])
                for total in camera_totals)
        if "resolution" in axes:
            scenarios.extend([
                ("resolution", "low", low_train_pool,
                 selected_by_budget[max_budget], full_camera_ids),
                ("resolution", "native", native_train_pool,
                 selected_by_budget[max_budget], full_camera_ids),
            ])
        for axis, level, source, events, cameras in scenarios:
            key = (
                id(source), tuple(events), tuple(cameras), int(subset_seed))
            if key not in cache:
                print(f"[RUN] seed={subset_seed} {axis}/{level} "
                      f"events={len(events)} cams={cameras}")
                cache[key] = run_subset(
                    axis, level, subset_seed, source, events, cameras,
                    native_test_obs, path_mask, base, core_sets, args)
            repetition = deepcopy(cache[key])
            repetition["axis"] = axis
            repetition["level"] = level
            repetitions.append(repetition)
            os.makedirs(args.out_dir, exist_ok=True)
            with open(os.path.join(args.out_dir, "sensitivity_checkpoint.json"), "w") as handle:
                json.dump(_jsonable({
                    "status": "running",
                    "repetitions": repetitions,
                    "summary": aggregate_results(repetitions),
                }), handle, indent=2)

    result = {
        "protocol": {
            "schema": "canonical_real_data_sensitivity_v1",
            "dataset": args.root_folder,
            "rows": list(ROWS),
            "axes": sorted(axes),
            "core_sets": list(core_sets),
            "view_budget_definition": (
                "total_robot_capture_events_balanced_equally_over_core_sets"),
            "view_budgets": list(view_budgets),
            "camera_count_definition": "one_eih_plus_total_minus_one_fixed",
            "fixed_camera_nesting_order": fixed_cameras,
            "camera_totals": list(camera_totals),
            "resolution_conditions": {
                "low": float(args.low_resolution_scale), "native": 1.0,
                "rerun_detection": True,
                "coordinates_mapped_to_native_pixels": True,
                "sensor_recapture": False,
            },
            "subset_seeds": list(subset_seeds),
            "split_seed": int(args.split_seed),
            "test_event_ids": sorted({int(obs.event) for obs in native_test_obs}),
            "test_camera_ids": common_test_camera_ids,
            "test_resolution": "native",
            "path_evaluation_mask_sha256": path_mask["evaluation_mask_sha256"],
            "external_physical_ground_truth": False,
            "primary_metric": "heldout_native_frame_corner_reprojection_RMSE_px",
            "headline_requires_all_repetitions_converged": True,
            "solver_options": canonical_solver_options(args).to_dict(),
            "num_inits": int(args.num_inits),
            "artifact_num_inits": int(args.artifact_num_inits),
        },
        "repetitions": repetitions,
        "summary": aggregate_results(repetitions),
    }
    write_outputs(result, args.out_dir)
    # The resume checkpoint is a strict subset of sensitivity.json; drop it once
    # the final artifact is on disk so it never gets committed alongside it.
    checkpoint = os.path.join(args.out_dir, "sensitivity_checkpoint.json")
    if os.path.exists(checkpoint):
        os.remove(checkpoint)
    print(f"[SAVE] {args.out_dir}/sensitivity.{{json,csv,md}}")


if __name__ == "__main__":
    main()
