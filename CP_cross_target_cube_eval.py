#!/usr/bin/env python3
"""Cross-target held-out evaluation: reproject cube corners with every row's frozen transforms.

Motivation: Table 1's `e_reproj overall` is not comparable across rows because each
row is scored only on its own target set.  Board-only rows (A0/B3) therefore report
board corners alone, which reads as "best" while never being asked to explain a cube.

This script asks every row the same question instead:

    Using only this row's fitted `T_base_Ci`, how well does it reproject held-out
    **cube** corners whose pose comes from the shared board-free FK artifact?

Controls that make the comparison fair:

* Cube poses are injected from the split's `fixed_cubes` (the canonical board-free FK
  artifact, `board_information_used=false`, `heldout_information_used=false`).  Every
  row is scored against byte-identical cube poses, so no row is scored on a target it
  chose itself.
* Only fixed cameras registered by **every** row are scored (cam 0 and cam 1).  cam 3
  exists solely in cube-bearing rows, so including it would compare coverage, not
  accuracy.  Lost coverage is reported separately as a count, never folded into RMSE.
* The eih path is excluded.  The shared FK cube pose was built with the artifact's
  `T_gripper_cam`, so scoring an eih reprojection would favour the rows that adopt
  that same gtc.  Restricting to e2h isolates one quantity: `T_base_Ci`.

Nothing is refit.  All transforms are read back from the stored multisplit runs.
"""
from __future__ import annotations

import argparse
import json
import os
from collections import defaultdict
from typing import Dict, List, Sequence

import numpy as np

import CP_ablation_7row as ab
import CP_common
from calibration_reprojection_backend import PoseState, inv_T, project_points

ROWS = ["A0", "A1", "A2", "A3", "B1", "B2", "B3"]
DEFAULT_SPLIT_SEEDS = [20260729, 20260730, 20260731, 20260732, 20260733]

# The adopted configuration lives in the D2 artifact, not the seven-row ablation.
# It shares the split seeds, solver options and shared FK artifact, so its stored
# transforms can be scored on exactly the same corners.
D2_ARM = "A2@lam3"
D2_ROW_LABEL = "A2@λ=3"
D2_CONDITION = {"target_set": "cube+board", "unified": "U",
                "fk_to_cube": "vision-estimated + soft FK anchor (λ=3 px/mm)"}

# Deltas mirroring the causal contrasts of Table 1, plus the board-only sanity pair.
PAIRED_DELTAS = [
    ("A0", "A1"), ("A1", "A2"), ("A2", "A3"), ("B1", "A3"), ("B2", "A3"),
    ("B3", "A3"), ("B3", "A0"), ("A2", D2_ROW_LABEL), ("A3", D2_ROW_LABEL),
    ("B3", D2_ROW_LABEL),
]


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root_folder", default="data/session")
    parser.add_argument("--intrinsics_dir", default="intrinsics")
    parser.add_argument("--calib_dir", default="data/session/calib_out")
    parser.add_argument("--multisplit_dir", default="CP_result/ablation_multisplit")
    parser.add_argument("--d2_json",
                        default="CP_result/D2_anchored_event_split/D2_anchored_event_split.json",
                        help="Adopted-configuration artifact; skipped if it stores no transforms.")
    parser.add_argument("--out_dir", default="CP_result/cross_target_cube")
    parser.add_argument("--test_fraction", type=float, default=0.2)
    parser.add_argument("--split_seeds", type=int, nargs="+", default=DEFAULT_SPLIT_SEEDS)
    parser.add_argument("--min_train_eih_cube_events", type=int, default=3)
    parser.add_argument("--image_scale", type=float, default=1.0)
    return parser.parse_args()


def install_detection_cache():
    """Detection is split-independent; run it once and reuse it across split seeds."""
    original = ab.detect_observations
    cache = {}

    def cached(args, meta, K_map, D_map, all_cam_ids, gripper):
        if "value" not in cache:
            cache["value"] = original(args, meta, K_map, D_map, all_cam_ids, gripper)
        return cache["value"]

    ab.detect_observations = cached


def load_split_rows(multisplit_dir: str, seed: int, root_folder: str) -> dict:
    split_dir = os.path.join(multisplit_dir, f"split_{seed}")
    path = os.path.join(split_dir, "seven_row_ablation.json")
    with open(path) as handle:
        payload = json.load(handle)
    CP_common.assert_artifact_robot_pos_scale(
        payload, path,
        fk_cube_path=os.path.join(split_dir, "shared_board_free_fk_cube.json"),
        root_folder=root_folder)
    return payload["rows"]


def load_d2_runs(path: str, seed: int) -> List[dict]:
    """Stored runs of the adopted arm, or [] if the artifact predates transform dumping."""
    if not os.path.exists(path):
        return []
    with open(path) as handle:
        payload = json.load(handle)
    # These runs are scored side by side with the multisplit rows above; a scale
    # mismatch between the two is exactly what put them on different bases before.
    CP_common.assert_artifact_robot_pos_scale(payload, path)
    runs = payload.get("per_split", {}).get(str(seed), {}).get(D2_ARM, [])
    if not runs or "transforms" not in runs[0]:
        return []
    return runs


def state_from_transforms(transforms: dict, cubes: Dict[int, np.ndarray]) -> PoseState:
    board = transforms.get("T_base_board")
    return PoseState(
        cams={int(k): np.asarray(v, dtype=np.float64)
              for k, v in transforms["T_base_Ci"].items()},
        gtc=np.asarray(transforms["T_gripper_cam"], dtype=np.float64),
        board=None if board is None else np.asarray(board, dtype=np.float64),
        cubes={int(k): np.asarray(v, dtype=np.float64) for k, v in cubes.items()},
    )


def cube_e2h_rmse(observations: Sequence, state: PoseState, cams: Sequence[int]) -> dict:
    """RMSE over cube corners seen by the given fixed cameras, all poses frozen."""
    wanted = {int(c) for c in cams}
    squared: Dict[int, List[float]] = defaultdict(list)
    counts: Dict[int, int] = defaultdict(int)
    for obs in observations:
        cam = int(obs.cam)
        if cam not in wanted or obs.marker != "cube":
            continue
        target = state.cubes.get(int(obs.set_idx))
        base_cam = state.cams.get(cam)
        if target is None or base_cam is None:
            continue
        pred = project_points(inv_T(base_cam) @ target, obs.object_points,
                              K_MAP[cam], D_MAP[cam])
        residual = np.square(pred - np.asarray(obs.image_points).reshape(-1, 2))
        squared[cam].extend(residual.reshape(-1).tolist())
        counts[cam] += len(pred)
    pooled = [v for values in squared.values() for v in values]
    out = {"overall": float(np.sqrt(np.mean(pooled))) if pooled else None,
           "n_corners": int(sum(counts.values()))}
    for cam in sorted(wanted):
        out[f"cam_{cam}"] = (float(np.sqrt(np.mean(squared[cam])))
                             if squared[cam] else None)
        out[f"n_corners_cam_{cam}"] = int(counts[cam])
    return out


def mean_std(values: Sequence[float]):
    clean = [float(v) for v in values if v is not None]
    if not clean:
        return None, None
    return float(np.mean(clean)), float(np.std(clean))


def fmt(mean, std, digits=4):
    if mean is None:
        return "—"
    return f"{mean:.{digits}f}±{std:.{digits}f}"


def main() -> None:
    args = parse_args()
    install_detection_cache()

    global K_MAP, D_MAP
    per_split = defaultdict(dict)
    coverage = {}
    scored_cams = None
    conditions = {}
    report_rows = None

    for seed in args.split_seeds:
        split_args = argparse.Namespace(**vars(args))
        split_args.split_seed = seed
        prepared = ab.prepare_ablation_data(split_args)
        K_MAP, D_MAP = prepared.K_map, prepared.D_map
        gripper = prepared.gripper
        stored = load_split_rows(args.multisplit_dir, seed, args.root_folder)
        runs_by_row = {row: stored[row]["runs"] for row in ROWS}
        for row in ROWS:
            conditions[row] = stored[row]["condition"]
        d2_runs = load_d2_runs(args.d2_json, seed)
        if d2_runs:
            runs_by_row[D2_ROW_LABEL] = d2_runs
            conditions[D2_ROW_LABEL] = D2_CONDITION
        if report_rows is None:
            report_rows = list(runs_by_row)
        elif report_rows != list(runs_by_row):
            raise RuntimeError(f"row set changed at seed {seed}")

        registered = [set(int(c) for c in runs[0]["transforms"]["T_base_Ci"])
                      for runs in runs_by_row.values()]
        common = sorted(set.intersection(*registered) - {gripper})
        if scored_cams is None:
            scored_cams = common
        elif scored_cams != common:
            raise RuntimeError(f"scored camera set changed at seed {seed}: "
                               f"{scored_cams} vs {common}")

        cube_test = [obs for obs in prepared.test_obs
                     if obs.marker == "cube" and int(obs.cam) != gripper]
        all_cube_cams = sorted({int(obs.cam) for obs in cube_test})
        coverage[seed] = {
            "scored_cams": common,
            "cube_e2h_cams_present": all_cube_cams,
            "dropped_cams": sorted(set(all_cube_cams) - set(common)),
            "corners_scored": sum(len(obs.object_points) for obs in cube_test
                                  if int(obs.cam) in set(common)),
            "corners_dropped": sum(len(obs.object_points) for obs in cube_test
                                   if int(obs.cam) not in set(common)),
        }

        for row, runs in runs_by_row.items():
            shared, own = [], []
            for run in runs:
                transforms = run["transforms"]
                shared.append(cube_e2h_rmse(
                    cube_test, state_from_transforms(transforms, prepared.fixed_cubes),
                    common))
                if transforms["T_base_cube_by_set"]:
                    own.append(cube_e2h_rmse(
                        cube_test,
                        state_from_transforms(transforms, transforms["T_base_cube_by_set"]),
                        common))
            entry = {"shared_fk_cube": {}, "own_cube": {}}
            keys = ["overall"] + [f"cam_{c}" for c in common]
            for key in keys:
                entry["shared_fk_cube"][key] = mean_std([r[key] for r in shared])[0]
                if own:
                    entry["own_cube"][key] = mean_std([r[key] for r in own])[0]
            entry["n_corners"] = shared[0]["n_corners"]
            entry["n_init_runs"] = len(shared)
            per_split[row][seed] = entry

    summary = {}
    for row in report_rows:
        summary[row] = {}
        for pose_source in ("shared_fk_cube", "own_cube"):
            keys = ["overall"] + [f"cam_{c}" for c in scored_cams]
            block = {}
            for key in keys:
                values = [per_split[row][s][pose_source].get(key)
                          for s in args.split_seeds]
                mean, std = mean_std(values)
                block[key] = {"mean_px": mean, "std_px": std, "n_splits": len(
                    [v for v in values if v is not None])}
            summary[row][pose_source] = block

    # Paired per-split deltas. Every row saw the same corners in the same split, so
    # the split-level difference is paired and its spread is the honest error bar.
    deltas = {}
    for a, b in PAIRED_DELTAS:
        if a not in report_rows or b not in report_rows:
            continue
        block = {}
        for key in ["overall"] + [f"cam_{c}" for c in scored_cams]:
            paired = [(per_split[b][s]["shared_fk_cube"][key]
                       - per_split[a][s]["shared_fk_cube"][key])
                      for s in args.split_seeds
                      if per_split[a][s]["shared_fk_cube"].get(key) is not None
                      and per_split[b][s]["shared_fk_cube"].get(key) is not None]
            if not paired:
                continue
            block[key] = {
                "delta_mean_px": float(np.mean(paired)),
                "delta_std_px": float(np.std(paired)),
                "second_better_splits": int(sum(1 for d in paired if d < 0)),
                "n_splits": len(paired),
            }
        deltas[f"{a}_to_{b}"] = block

    os.makedirs(args.out_dir, exist_ok=True)
    payload = {
        "artifact_schema": "cross_target_cube_reprojection_v1",
        "question": ("held-out cube corner reprojection RMSE under each row's frozen "
                     "T_base_Ci, scored on cube poses shared by all rows"),
        "cube_pose_source": "shared board-free FK artifact (prepared.fixed_cubes)",
        "scored_fixed_cameras": scored_cams,
        "excluded": {"eih_path": "gtc-circular with the shared FK cube pose",
                     "unregistered_cameras": "not registered by every row"},
        "refit": False,
        "split_seeds": args.split_seeds,
        "rows": report_rows,
        "conditions": conditions,
        "coverage": coverage,
        "per_split": {row: {str(s): v for s, v in per_split[row].items()}
                      for row in report_rows},
        "summary": summary,
        "paired_deltas": deltas,
    }
    with open(os.path.join(args.out_dir, "cross_target_cube.json"), "w") as handle:
        json.dump(ab._jsonable(payload), handle, indent=2)

    cam_cols = " | ".join(f"cam {c} (px)" for c in scored_cams)
    lines = [
        "# Cross-target held-out cube reprojection",
        "",
        "Every row is scored on the **same** held-out cube corners with the **same** cube",
        "poses (shared board-free FK artifact). No refit; all transforms frozen from the",
        f"stored multisplit runs. Scored fixed cameras: {scored_cams} "
        f"(intersection over all rows; eih path excluded).",
        "",
        f"| Row | Target | U | FK→cube | cube e2h reproj (px) | {cam_cols} | own-pose cube e2h (px) |",
        "| --- | --- | :---: | --- | ---: | " + " | ".join(["---:"] * len(scored_cams))
        + " | ---: |",
    ]
    for row in report_rows:
        cond = conditions[row]
        shared = summary[row]["shared_fk_cube"]
        own = summary[row]["own_cube"]
        cells = " | ".join(fmt(shared[f"cam_{c}"]["mean_px"], shared[f"cam_{c}"]["std_px"])
                           for c in scored_cams)
        lines.append(
            f"| {row} | {cond['target_set']} | {cond['unified']} | {cond['fk_to_cube']} | "
            f"{fmt(shared['overall']['mean_px'], shared['overall']['std_px'])} | {cells} | "
            f"{fmt(own['overall']['mean_px'], own['overall']['std_px'])} |")
    lines += [
        "",
        "`±` is the standard deviation over split means; each split value is the mean of "
        "its 5 initializations.",
        "`own-pose cube e2h` re-scores the same corners with the row's own fitted cube "
        "poses and is diagnostic only — it is not comparable across rows because the "
        "target itself differs.",
        "",
        "## Paired per-split deltas (second row minus first, negative is better)",
        "",
        f"| Contrast | pooled (px) | {cam_cols} | second better |",
        "| --- | ---: | " + " | ".join(["---:"] * len(scored_cams)) + " | ---: |",
    ]
    for name, block in deltas.items():
        if "overall" not in block:
            continue
        cells = " | ".join(
            f"{block[f'cam_{c}']['delta_mean_px']:+.4f}±{block[f'cam_{c}']['delta_std_px']:.4f}"
            for c in scored_cams if f"cam_{c}" in block)
        overall = block["overall"]
        lines.append(
            f"| {name.replace('_to_', '→')} | "
            f"{overall['delta_mean_px']:+.4f}±{overall['delta_std_px']:.4f} | {cells} | "
            f"{overall['second_better_splits']}/{overall['n_splits']} |")
    lines += [
        "",
        "`second better` counts splits on the pooled metric. Deltas are paired within a "
        "split, so the spread here is the meaningful error bar, not the per-row `±` above.",
        "",
        "## Coverage dropped by the common-camera restriction",
        "",
        "| split seed | scored cams | dropped cams | corners scored | corners dropped |",
        "| --- | --- | --- | ---: | ---: |",
    ]
    for seed in args.split_seeds:
        cov = coverage[seed]
        lines.append(
            f"| {seed} | {cov['scored_cams']} | {cov['dropped_cams']} | "
            f"{cov['corners_scored']} | {cov['corners_dropped']} |")
    lines += [
        "",
        "Dropped corners are cube corners seen only by cameras that board-only rows never "
        "registered. They are a coverage loss of those rows, not an accuracy result, and "
        "are excluded from the RMSE above rather than silently averaged in.",
        "",
        "The shared FK cube pose is not external physical ground truth: it comes from "
        "train eih cube corners and raw FK, so a common error floor applies to every row. "
        "Report these values as `e_reproj^{cube|FK-fixed}` and do not read them as "
        "absolute accuracy.",
    ]
    with open(os.path.join(args.out_dir, "cross_target_cube.md"), "w") as handle:
        handle.write("\n".join(lines) + "\n")
    print("\n".join(lines))


if __name__ == "__main__":
    main()
