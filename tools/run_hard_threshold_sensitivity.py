#!/usr/bin/env python3
"""Preregistered hard rejection threshold sensitivity experiment (OFAT).

See CP_result/session04/outlier_ablation/PREREGISTRATION_HARD_THRESHOLD.md.

Each point moves exactly one of the three rejection criteria away from the
``standard`` baseline.  Rejection is applied to train events only: the held-out
observation population is frozen at the baseline so every point is scored on
the same test set.  The split and the held-out population are verified against
the baseline at every point and any drift is reported, never silently dropped.
"""

from __future__ import annotations

import argparse
import copy
import csv
import hashlib
import json
import os
import subprocess
import sys

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
BASELINE = {"rmse": 3.0, "inlier": 0.0, "corners": 4}
POINTS = [
    ("P0", 3.0, 0.00, 4, "baseline"),
    ("R1", 1.5, 0.00, 4, "cube_pnp_rmse_px"),
    ("R2", 1.0, 0.00, 4, "cube_pnp_rmse_px"),
    ("R3", 0.7, 0.00, 4, "cube_pnp_rmse_px"),
    ("C1", 3.0, 0.00, 12, "board_min_charuco_corners"),
    ("C2", 3.0, 0.00, 20, "board_min_charuco_corners"),
    ("C3", 3.0, 0.00, 28, "board_min_charuco_corners"),
    ("C4", 3.0, 0.00, 36, "board_min_charuco_corners"),
    ("I1", 3.0, 0.90, 4, "cube_min_inlier_fraction"),
    ("I2", 3.0, 1.00, 4, "cube_min_inlier_fraction"),
]
ROWS = "A0,A2,A3"
PATH_METRICS = ("cross_view_pixel_transfer_rmse_px",
                "e_cross_translation_rmse_mm",
                "e_e2e_translation_rmse_mm")


def _mean(values):
    numeric = [v for v in values if v is not None]
    return None if not numeric else sum(numeric) / len(numeric)


def _reprojection_mean(runs, split, target):
    """Same aggregation the canonical Table 1 CSV uses."""
    key = f"{split}_reprojection"
    return _mean([None if run[key].get(target) is None
                  else run[key][target]["rmse_px"] for run in runs])


def row_metrics(row):
    runs = row.get("runs") or []
    if not runs:
        return {"converged_runs": 0, "n_runs": 0}
    out = {"n_runs": len(runs),
           "converged_runs": sum(1 for r in runs if r.get("converged"))}
    for target in ("overall", "board", "cube"):
        out[f"heldout_{target}_reprojection_rmse_px"] = _reprojection_mean(
            runs, "heldout", target)
        out[f"train_{target}_reprojection_rmse_px"] = _reprojection_mean(
            runs, "train", target)
    held = runs[0]["heldout_reprojection"].get("overall") or {}
    out["n_heldout_observations"] = held.get("n_observations")
    out["n_heldout_corners"] = held.get("n_corners")
    tr = runs[0]["train_reprojection"].get("overall") or {}
    out["n_train_observations"] = tr.get("n_observations")
    out["n_train_corners"] = tr.get("n_corners")
    masks = {r["heldout_path_metrics"].get("evaluation_mask_sha256")
             for r in runs}
    out["heldout_path_mask_sha256"] = (
        masks.pop() if len(masks) == 1 else "INCONSISTENT")
    for key in PATH_METRICS:
        out[key] = _mean([r["heldout_path_metrics"].get(key) for r in runs])
    return out


def sh(cmd, log_path):
    env = dict(os.environ, RB_ROBOT_POS_SCALE="1.0", PYTHONPATH="")
    with open(log_path, "w") as log:
        proc = subprocess.run(cmd, cwd=REPO, env=env, stdout=log,
                              stderr=subprocess.STDOUT)
    if proc.returncode != 0:
        raise RuntimeError(f"command failed ({proc.returncode}): {' '.join(cmd)}\n"
                           f"see {log_path}")


def run_step2b(point_dir, rmse, inlier, corners, session_root, intrinsics_dir):
    sh([sys.executable, "04_filter_observations.py",
        "--session-root", session_root, "--intrinsics-dir", intrinsics_dir,
        "--output-dir", point_dir,
        "--strict-cube-rmse-px", str(rmse),
        "--strict-min-inlier-fraction", str(inlier),
        "--strict-board-min-corners", str(corners)],
       os.path.join(point_dir + "_step2b.log"))
    return os.path.join(point_dir, "Step2b_observation_manifest.json")


def freeze_heldout(manifest_path, baseline_manifest_path, heldout_events, out_path,
                   point_id):
    """strict := threshold selection on train events, baseline standard on held-out."""
    with open(manifest_path) as handle:
        payload = json.load(handle)
    with open(baseline_manifest_path) as handle:
        base = json.load(handle)
    base_standard = {
        str(r["observation_id"]): bool(r["selected_by_policy"]["standard"])
        for r in base["observations"]}
    heldout = {int(e) for e in heldout_events}
    n_frozen = 0
    for record in payload["observations"]:
        if int(record.get("event_id", -1)) in heldout:
            record["selected_by_policy"]["strict"] = base_standard[
                str(record["observation_id"])]
            record["reason_by_policy"]["strict"] = "heldout_frozen_at_baseline"
            n_frozen += 1
    payload["policies"]["strict"]["heldout_freeze"] = {
        "point_id": point_id,
        "rule": ("hard rejection applied to train events only; held-out events "
                 "keep the baseline standard population"),
        "n_heldout_observations_frozen": n_frozen,
        "heldout_events": sorted(heldout),
    }
    with open(out_path, "w") as handle:
        json.dump(payload, handle, indent=2)
    return out_path


def population(manifest_path, policy, events=None):
    with open(manifest_path) as handle:
        payload = json.load(handle)
    sel = [r for r in payload["observations"]
           if r["selected_by_policy"][policy]
           and (events is None or int(r["event_id"]) in events)]
    corners = sum(int(r["corner_count"]) for r in sel)
    digest = hashlib.sha256(json.dumps(
        sorted(str(r["observation_id"]) for r in sel)).encode()).hexdigest()
    return {"n_observations": len(sel), "n_corners": corners, "sha256": digest}


def run_table1(manifest, policy, out_dir, session_root, intrinsics_dir, calib_dir):
    sh([sys.executable, "05_calibrate.py",
        "--root_folder", session_root, "--intrinsics_dir", intrinsics_dir,
        "--calib_dir", calib_dir, "--include_sets", "0-12",
        "--split_seed", "20260731", "--min_train_eih_cube_events", "3",
        "--rows", ROWS, "--observation-manifest", manifest,
        "--observation-filter-policy", policy, "--out_dir", out_dir],
       out_dir + "_table1.log")
    with open(os.path.join(out_dir, "table1_methods.json")) as handle:
        return json.load(handle)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--session-root", default="data/session04/calib_train")
    parser.add_argument("--intrinsics-dir", default="intrinsics")
    parser.add_argument("--calib-dir", default="data/session04/calib_out")
    parser.add_argument("--work-dir", required=True)
    parser.add_argument("--out-dir",
                        default="CP_result/session04/outlier_ablation/hard_threshold_sensitivity")
    args = parser.parse_args()
    work = os.path.abspath(args.work_dir)
    out_dir = os.path.join(REPO, args.out_dir)
    os.makedirs(work, exist_ok=True)
    os.makedirs(out_dir, exist_ok=True)

    records, baseline_split, baseline_manifest, baseline_heldout_pop = [], None, None, None
    for point_id, rmse, inlier, corners, axis in POINTS:
        print(f"[POINT] {point_id}  rmse<={rmse} inlier>={inlier} corners>={corners}",
              flush=True)
        point_dir = os.path.join(work, point_id)
        manifest = run_step2b(point_dir, rmse, inlier, corners,
                              args.session_root, args.intrinsics_dir)
        if point_id == "P0":
            baseline_manifest = manifest
            used = manifest
        else:
            used = freeze_heldout(
                manifest, baseline_manifest, baseline_split["test_events"],
                os.path.join(work, f"{point_id}_frozen_manifest.json"), point_id)
        result = run_table1(used, "strict", os.path.join(work, f"{point_id}_table1"),
                            args.session_root, args.intrinsics_dir, args.calib_dir)
        split = result["protocol"]["split"]
        if point_id == "P0":
            baseline_split = split
            baseline_heldout_pop = population(
                used, "strict", set(split["test_events"]))
        heldout_pop = population(used, "strict", set(baseline_split["test_events"]))
        train_pop = population(used, "strict", set(baseline_split["train_events"]))
        split_ok = (split["train_events"] == baseline_split["train_events"]
                    and split["test_events"] == baseline_split["test_events"])
        heldout_ok = heldout_pop["sha256"] == baseline_heldout_pop["sha256"]
        if not split_ok:
            print(f"  [WARN] {point_id}: split drift vs baseline", flush=True)
        if not heldout_ok:
            print(f"  [WARN] {point_id}: held-out population drift", flush=True)
        for method, row in result["rows"].items():
            metrics = row_metrics(row)
            entry = {
                "point_id": point_id, "axis": axis,
                "cube_max_pnp_rmse_px": rmse, "cube_min_inlier_fraction": inlier,
                "board_min_charuco_corners": corners, "method": method,
                "split_frozen": split_ok, "heldout_population_frozen": heldout_ok,
                "manifest_train_observations": train_pop["n_observations"],
                "manifest_train_corners": train_pop["n_corners"],
                **metrics,
            }
            records.append(entry)

    csv_path = os.path.join(out_dir, "hard_threshold_sensitivity.csv")
    with open(csv_path, "w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(records[0]),
                                lineterminator="\n")
        writer.writeheader()
        writer.writerows(records)
    with open(os.path.join(out_dir, "hard_threshold_sensitivity.json"), "w") as handle:
        json.dump({"preregistration":
                   "CP_result/session04/outlier_ablation/PREREGISTRATION_HARD_THRESHOLD.md",
                   "baseline_split": baseline_split, "points": records},
                  handle, indent=2)
    print(f"[DONE] {csv_path}")


if __name__ == "__main__":
    main()
