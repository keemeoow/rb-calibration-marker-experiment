#!/usr/bin/env python3
"""Session-paired external-GT evaluation for the final calibration claim.

The input manifest contains independent GT and one prediction file per method
and session.  The runner computes TRE, rotation geodesic error, optional
ADD/ADD-S, failure rate and workspace tail errors, then performs a paired
hierarchical bootstrap (session first, paired poses second) with equal session
weight.  Missing predictions remain failures and are never silently removed.
"""

from __future__ import annotations

import argparse
import csv
import json
import os
from dataclasses import dataclass
from typing import Dict, Iterable, Mapping, Optional, Sequence

import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation


MANIFEST_SCHEMA = "external_gt_eval_manifest_v1"
POSE_SCHEMA = "base_cube_pose_predictions_v1"


def _load_json_or_inline(value, base_dir: str) -> dict:
    if isinstance(value, dict):
        return value
    path = str(value)
    if not os.path.isabs(path):
        path = os.path.join(base_dir, path)
    with open(path) as handle:
        return json.load(handle)


def _transform(value) -> np.ndarray:
    matrix = np.asarray(value, dtype=np.float64)
    if matrix.shape != (4, 4) or not np.all(np.isfinite(matrix)):
        raise ValueError("pose transform must be a finite 4x4 matrix")
    if not np.allclose(matrix[3], [0.0, 0.0, 0.0, 1.0], atol=1e-8):
        raise ValueError("pose transform has an invalid homogeneous last row")
    return matrix


def _pose_map(payload: Mapping, *, require_gt: bool) -> dict:
    if payload.get("artifact_schema") != POSE_SCHEMA:
        raise ValueError("unknown external-GT/prediction pose schema")
    output = {}
    for pose_id, entry in payload.get("poses", {}).items():
        status = str(entry.get("status", "ok"))
        transform = entry.get("T_base_cube")
        if status == "ok" and transform is not None:
            transform = _transform(transform)
        else:
            transform = None
        if require_gt and transform is None:
            raise ValueError(f"GT pose {pose_id} is missing")
        output[str(pose_id)] = {
            "status": status,
            "transform": transform,
            "strata": tuple(sorted(set(map(str, entry.get("strata", []))))),
        }
    if not output:
        raise ValueError("pose artifact is empty")
    return output


def translation_error_mm(prediction: np.ndarray, truth: np.ndarray) -> float:
    return float(np.linalg.norm(prediction[:3, 3] - truth[:3, 3]) * 1000.0)


def rotation_error_deg(prediction: np.ndarray, truth: np.ndarray) -> float:
    delta = truth[:3, :3].T @ prediction[:3, :3]
    return float(np.degrees(np.linalg.norm(Rotation.from_matrix(delta).as_rotvec())))


def _points_in_base(transform: np.ndarray, points: np.ndarray) -> np.ndarray:
    return points @ transform[:3, :3].T + transform[:3, 3]


def add_error_mm(prediction: np.ndarray, truth: np.ndarray,
                 points_m: np.ndarray, mode: str) -> float:
    predicted = _points_in_base(prediction, points_m)
    target = _points_in_base(truth, points_m)
    if mode == "ADD":
        distances = np.linalg.norm(predicted - target, axis=1)
    elif mode == "ADD-S":
        distances = cKDTree(target).query(predicted, k=1)[0]
    else:
        raise ValueError(f"unknown ADD mode {mode!r}")
    return float(np.mean(distances) * 1000.0)


@dataclass
class SessionData:
    session_id: str
    gt: dict
    predictions: dict
    registered_cameras: dict


def load_manifest(path: str) -> tuple[dict, list[SessionData]]:
    with open(path) as handle:
        manifest = json.load(handle)
    if manifest.get("artifact_schema") != MANIFEST_SCHEMA:
        raise ValueError("unknown external-GT evaluation manifest schema")
    if manifest.get("blind_gt_used_for_training") is not False:
        raise ValueError("manifest must explicitly state blind_gt_used_for_training=false")
    if manifest.get("gt_independent_of_fk_factor") is not True:
        raise ValueError("external GT must be independent of the A4 FK factor")
    base_dir = os.path.dirname(os.path.abspath(path))
    methods = tuple(map(str, manifest.get("methods", [])))
    if str(manifest.get("ours_method")) not in methods or len(methods) < 2:
        raise ValueError("manifest needs Ours and at least one baseline")
    sessions = []
    seen = set()
    for raw in manifest.get("sessions", []):
        session_id = str(raw["session_id"])
        if session_id in seen:
            raise ValueError(f"duplicate session ID {session_id}")
        seen.add(session_id)
        gt = _pose_map(_load_json_or_inline(raw["external_gt"], base_dir), require_gt=True)
        predictions = {}
        for method in methods:
            if method not in raw.get("predictions", {}):
                predictions[method] = {}
            else:
                predictions[method] = _pose_map(
                    _load_json_or_inline(raw["predictions"][method], base_dir),
                    require_gt=False,
                )
        sessions.append(SessionData(
            session_id=session_id,
            gt=gt,
            predictions=predictions,
            registered_cameras={
                str(key): int(value)
                for key, value in raw.get("registered_cameras", {}).items()
            },
        ))
    if len(sessions) < 2:
        raise ValueError("hierarchical evaluation needs at least two independent sessions")
    return manifest, sessions


def build_pose_metrics(manifest: Mapping, sessions: Sequence[SessionData]) -> dict:
    add_mode = str(manifest.get("add_mode", "none"))
    object_points = None
    if add_mode != "none":
        if add_mode not in {"ADD", "ADD-S"}:
            raise ValueError("add_mode must be none, ADD, or ADD-S")
        object_points = np.asarray(manifest.get("object_points_m"), dtype=np.float64)
        if object_points.ndim != 2 or object_points.shape[1] != 3 or len(object_points) < 4:
            raise ValueError("ADD/ADD-S requires at least four object_points_m")
    output = {}
    for session in sessions:
        per_method = {}
        for method, predictions in session.predictions.items():
            rows = {}
            for pose_id, gt_entry in session.gt.items():
                prediction = predictions.get(pose_id, {})
                transform = prediction.get("transform")
                success = transform is not None
                strata = gt_entry["strata"] or tuple(prediction.get("strata", ()))
                row = {"success": bool(success), "strata": list(strata)}
                if success:
                    truth = gt_entry["transform"]
                    row["tre_mm"] = translation_error_mm(transform, truth)
                    row["rotation_deg"] = rotation_error_deg(transform, truth)
                    if object_points is not None:
                        row["add_mm"] = add_error_mm(transform, truth, object_points, add_mode)
                rows[pose_id] = row
            per_method[method] = rows
        output[session.session_id] = per_method
    return output


def _percentile(values: Sequence[float], percentile: float) -> float:
    return float(np.percentile(np.asarray(values, dtype=np.float64), percentile))


def _paired_pose_ids(session_rows: Mapping, ours: str, baseline: str,
                     *, successes_only: bool) -> list[str]:
    ids = sorted(set(session_rows[ours]) | set(session_rows[baseline]))
    if successes_only:
        ids = [
            pose_id for pose_id in ids
            if session_rows[ours].get(pose_id, {}).get("success", False)
            and session_rows[baseline].get(pose_id, {}).get("success", False)
        ]
    return ids


def _session_difference(session_rows: Mapping, ours: str, baseline: str,
                        pose_ids: Sequence[str], metric: str) -> Optional[float]:
    if not pose_ids:
        return None
    if metric == "failure_rate":
        ours_values = [not session_rows[ours].get(pose_id, {}).get("success", False)
                       for pose_id in pose_ids]
        baseline_values = [not session_rows[baseline].get(pose_id, {}).get("success", False)
                           for pose_id in pose_ids]
        return float(np.mean(ours_values) - np.mean(baseline_values))
    if metric == "p95_tre_mm":
        ours_values = [session_rows[ours][pose_id]["tre_mm"] for pose_id in pose_ids]
        baseline_values = [session_rows[baseline][pose_id]["tre_mm"] for pose_id in pose_ids]
        return _percentile(ours_values, 95.0) - _percentile(baseline_values, 95.0)
    if metric == "worst_stratum_p95_tre_mm":
        strata = sorted({
            stratum for pose_id in pose_ids
            for stratum in session_rows[ours][pose_id].get("strata", [])
        })
        if not strata:
            return None
        differences = []
        for stratum in strata:
            selected = [
                pose_id for pose_id in pose_ids
                if stratum in session_rows[ours][pose_id].get("strata", [])
            ]
            if selected:
                differences.append(
                    _percentile([session_rows[ours][pose_id]["tre_mm"] for pose_id in selected], 95.0)
                    - _percentile([session_rows[baseline][pose_id]["tre_mm"] for pose_id in selected], 95.0)
                )
        return None if not differences else float(max(differences))
    field = {
        "mean_tre_mm": "tre_mm",
        "mean_rotation_deg": "rotation_deg",
        "mean_add_mm": "add_mm",
    }.get(metric)
    if field is None:
        raise ValueError(f"unknown paired metric {metric}")
    values = [
        session_rows[ours][pose_id][field] - session_rows[baseline][pose_id][field]
        for pose_id in pose_ids
    ]
    return None if not values else float(np.mean(values))


def paired_hierarchical_bootstrap(metrics: Mapping, ours: str, baseline: str,
                                  metric: str, repetitions: int,
                                  seed: int) -> dict:
    session_ids = sorted(metrics)
    successes_only = metric != "failure_rate"

    def draw_session(session_id: str, rng: Optional[np.random.Generator]):
        rows = metrics[session_id]
        pose_ids = _paired_pose_ids(rows, ours, baseline, successes_only=successes_only)
        if not pose_ids:
            return None
        if rng is not None:
            indices = rng.integers(0, len(pose_ids), size=len(pose_ids))
            pose_ids = [pose_ids[int(index)] for index in indices]
        return _session_difference(rows, ours, baseline, pose_ids, metric)

    observed = [draw_session(session_id, None) for session_id in session_ids]
    observed = [value for value in observed if value is not None]
    if not observed:
        return {"available": False, "reason": "no complete paired session/pose units"}
    point = float(np.mean(observed))
    rng = np.random.default_rng(int(seed))
    samples = []
    for _ in range(int(repetitions)):
        selected = rng.integers(0, len(session_ids), size=len(session_ids))
        values = [draw_session(session_ids[int(index)], rng) for index in selected]
        values = [value for value in values if value is not None]
        if values:
            samples.append(float(np.mean(values)))
    if len(samples) < max(100, int(repetitions) // 2):
        return {"available": False, "reason": "too few valid bootstrap replicates"}
    sample_array = np.asarray(samples, dtype=np.float64)
    return {
        "available": True,
        "estimate": point,
        "ci95_lower": float(np.percentile(sample_array, 2.5)),
        "ci95_upper": float(np.percentile(sample_array, 97.5)),
        "one_sided_p_delta_ge_zero": float((1 + np.sum(sample_array >= 0.0)) / (len(sample_array) + 1)),
        "n_sessions_total": len(session_ids),
        "n_sessions_with_complete_pairs": len(observed),
        "bootstrap_repetitions_valid": len(samples),
        "aggregation": "equal_session_weight_after_paired_within_session_pose_resampling",
    }


def holm_adjust(p_values: Mapping[str, float]) -> dict:
    ordered = sorted(p_values.items(), key=lambda item: item[1])
    count = len(ordered)
    adjusted = {}
    running = 0.0
    for rank, (name, value) in enumerate(ordered):
        candidate = min(1.0, float(value) * (count - rank))
        running = max(running, candidate)
        adjusted[name] = running
    return adjusted


def summarize_method(metrics: Mapping, method: str) -> dict:
    session_summaries = []
    for session_id in sorted(metrics):
        rows = metrics[session_id][method]
        all_rows = list(rows.values())
        successes = [row for row in all_rows if row.get("success")]
        entry = {
            "session_id": session_id,
            "attempted": len(all_rows),
            "successes": len(successes),
            "failure_rate": float(1.0 - len(successes) / len(all_rows)),
        }
        for field in ("tre_mm", "rotation_deg", "add_mm"):
            values = [float(row[field]) for row in successes if field in row]
            if values:
                entry[field] = {
                    "mean": float(np.mean(values)),
                    "p50": _percentile(values, 50.0),
                    "p95": _percentile(values, 95.0),
                    "max": float(np.max(values)),
                }
        session_summaries.append(entry)
    return {"sessions": session_summaries}


def evaluate(manifest: Mapping, sessions: Sequence[SessionData], metrics: Mapping) -> dict:
    ours = str(manifest["ours_method"])
    baselines = [str(method) for method in manifest["methods"] if str(method) != ours]
    bootstrap = manifest.get("bootstrap", {})
    repetitions = int(bootstrap.get("repetitions", 10000))
    seed = int(bootstrap.get("seed", 20260806))
    margins = manifest.get("margins", {})
    required_margins = ("rotation_deg", "p95_tre_mm", "failure_rate", "worst_stratum_p95_tre_mm")
    missing = [name for name in required_margins if name not in margins]
    if missing:
        raise ValueError(f"non-inferiority margins missing: {missing}")
    metric_names = [
        "mean_tre_mm", "mean_rotation_deg", "p95_tre_mm",
        "failure_rate", "worst_stratum_p95_tre_mm",
    ]
    if str(manifest.get("add_mode", "none")) != "none":
        metric_names.append("mean_add_mm")
        if "add_mm" not in margins:
            raise ValueError("ADD/ADD-S claim requires margins.add_mm")
    comparisons = {}
    raw_tre_p = {}
    for baseline_index, baseline in enumerate(baselines):
        per_metric = {}
        for metric_index, metric_name in enumerate(metric_names):
            per_metric[metric_name] = paired_hierarchical_bootstrap(
                metrics, ours, baseline, metric_name, repetitions,
                seed + baseline_index * 1000 + metric_index)
        comparisons[baseline] = per_metric
        if per_metric["mean_tre_mm"].get("available"):
            raw_tre_p[baseline] = per_metric["mean_tre_mm"]["one_sided_p_delta_ge_zero"]
    adjusted = holm_adjust(raw_tre_p)
    alpha = float(manifest.get("alpha", 0.05))
    uncertainty = manifest.get("gt_uncertainty_floor", {})
    translation_floor = float(uncertainty.get("translation_mm", 0.0))
    decisions = {}
    for baseline in baselines:
        values = comparisons[baseline]

        def upper(name):
            return values[name].get("ci95_upper", float("inf"))

        gates = {
            "tre_superiority": bool(
                upper("mean_tre_mm") < 0.0 and adjusted.get(baseline, 1.0) < alpha),
            "tre_exceeds_gt_uncertainty_floor": bool(
                -values["mean_tre_mm"].get("estimate", float("-inf")) > translation_floor),
            "rotation_noninferiority": bool(upper("mean_rotation_deg") < float(margins["rotation_deg"])),
            "p95_noninferiority": bool(upper("p95_tre_mm") < float(margins["p95_tre_mm"])),
            "failure_noninferiority": bool(upper("failure_rate") < float(margins["failure_rate"])),
            "worst_stratum_noninferiority": bool(
                upper("worst_stratum_p95_tre_mm") < float(margins["worst_stratum_p95_tre_mm"])),
        }
        if "mean_add_mm" in values:
            gates["add_contract"] = bool(upper("mean_add_mm") < float(margins["add_mm"]))
        camera_differences = []
        for session in sessions:
            if ours in session.registered_cameras and baseline in session.registered_cameras:
                camera_differences.append(
                    session.registered_cameras[ours] - session.registered_cameras[baseline])
        gates["registered_camera_count_not_lower"] = bool(
            camera_differences and min(camera_differences) >= 0)
        decisions[baseline] = {
            "holm_adjusted_tre_p": adjusted.get(baseline),
            "gates": gates,
            "claim_pass": bool(all(gates.values())),
        }
    return {
        "protocol": {
            "independent_unit": "camera_installation_session",
            "bootstrap": "paired_hierarchical_session_then_pose",
            "bootstrap_repetitions": repetitions,
            "alpha": alpha,
            "holm_family": baselines,
            "margins": margins,
            "gt_uncertainty_floor": uncertainty,
        },
        "method_summaries": {
            method: summarize_method(metrics, method) for method in manifest["methods"]},
        "comparisons_A4_minus_baseline": comparisons,
        "decisions": decisions,
        "overall_claim_pass": bool(decisions and all(item["claim_pass"] for item in decisions.values())),
    }


def write_outputs(result: Mapping, output_dir: str) -> None:
    os.makedirs(output_dir, exist_ok=True)
    with open(os.path.join(output_dir, "external_gt_evaluation.json"), "w") as handle:
        json.dump(result, handle, indent=2)
    rows = []
    for baseline, metrics in result["comparisons_A4_minus_baseline"].items():
        for metric, values in metrics.items():
            rows.append({"baseline": baseline, "metric": metric, **values})
    fields = sorted({key for row in rows for key in row}) if rows else ["baseline", "metric"]
    with open(os.path.join(output_dir, "external_gt_evaluation.csv"), "w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)
    lines = [
        "# External-GT final evaluation",
        "",
        f"Overall claim gate: **{'PASS' if result['overall_claim_pass'] else 'FAIL'}**.",
        "",
        "Negative estimates favor A4. A claim passes only when every preregistered gate passes.",
        "",
        "| Baseline | ΔTRE mean [95% CI] mm | Δrotation mean upper CI ° | ΔP95 upper CI mm | Δfailure upper CI | Claim |",
        "| --- | ---: | ---: | ---: | ---: | --- |",
    ]
    for baseline, metrics in result["comparisons_A4_minus_baseline"].items():
        tre = metrics["mean_tre_mm"]
        decision = result["decisions"][baseline]
        if tre.get("available"):
            tre_text = f"{tre['estimate']:.3f} [{tre['ci95_lower']:.3f}, {tre['ci95_upper']:.3f}]"
        else:
            tre_text = "N/A"
        lines.append(
            f"| {baseline} | {tre_text} | "
            f"{metrics['mean_rotation_deg'].get('ci95_upper', float('nan')):.3f} | "
            f"{metrics['p95_tre_mm'].get('ci95_upper', float('nan')):.3f} | "
            f"{metrics['failure_rate'].get('ci95_upper', float('nan')):.4f} | "
            f"{'PASS' if decision['claim_pass'] else 'FAIL'} |"
        )
    with open(os.path.join(output_dir, "external_gt_evaluation.md"), "w") as handle:
        handle.write("\n".join(lines) + "\n")


def parse_args():
    parser = argparse.ArgumentParser(description="Paired hierarchical external-GT evaluation")
    parser.add_argument("--manifest", required=True)
    parser.add_argument("--output_dir", default="CP_result/final_external_gt")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    manifest, sessions = load_manifest(args.manifest)
    metrics = build_pose_metrics(manifest, sessions)
    result = evaluate(manifest, sessions, metrics)
    write_outputs(result, args.output_dir)
    print(f"[DONE] {os.path.join(args.output_dir, 'external_gt_evaluation.json')}")


if __name__ == "__main__":
    main()
