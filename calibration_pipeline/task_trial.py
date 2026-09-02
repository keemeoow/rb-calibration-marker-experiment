#!/usr/bin/env python3
"""Paired robot task-trial evaluation for peg-in-hole and grasp tests.

Every session contains matched trial pairs.  A pair fixes the target pose and
experimental stratum, then records exactly one attempt for every calibration
method.  Missing attempts cannot disappear from the denominator: they must be
recorded explicitly as failures.
"""

from __future__ import annotations

import argparse
import csv
import json
import os
from typing import Mapping, Sequence

import numpy as np


MANIFEST_SCHEMA = "robot_task_trial_manifest_v1"
TASK_TYPES = frozenset({"peg_in_hole", "grasp"})
AXES = ("x", "y", "z")
_Z_95 = 1.959963984540054


def wilson_interval(successes: int, attempts: int) -> dict:
    """Return the two-sided 95% Wilson interval for a binomial rate."""
    successes = int(successes)
    attempts = int(attempts)
    if attempts <= 0 or successes < 0 or successes > attempts:
        raise ValueError("invalid success/attempt counts")
    rate = successes / attempts
    z2 = _Z_95 * _Z_95
    denominator = 1.0 + z2 / attempts
    center = (rate + z2 / (2.0 * attempts)) / denominator
    radius = (_Z_95 / denominator) * np.sqrt(
        rate * (1.0 - rate) / attempts + z2 / (4.0 * attempts * attempts))
    return {
        "rate": float(rate),
        "ci95_lower": float(
            0.0 if successes == 0 else max(0.0, center - radius)),
        "ci95_upper": float(
            1.0 if successes == attempts else min(1.0, center + radius)),
        "method": "two_sided_Wilson_score_interval",
    }


def _non_placeholder_text(value, field: str) -> str:
    text = str(value or "").strip()
    if not text or text.startswith("REPLACE_"):
        raise ValueError(f"{field} must be defined before trials")
    return text


def _contact_error(value, *, required: bool) -> dict | None:
    if value is None:
        if required:
            raise ValueError("successful trial is missing contact_error_mm")
        return None
    if not isinstance(value, Mapping) or set(value) != set(AXES):
        raise ValueError("contact_error_mm must contain exactly x, y, z")
    output = {axis: float(value[axis]) for axis in AXES}
    if not np.all(np.isfinite(list(output.values()))):
        raise ValueError("contact_error_mm must be finite")
    return output


def validate_manifest(payload: Mapping) -> dict:
    """Validate and normalize one preregistered paired task manifest."""
    if payload.get("artifact_schema") != MANIFEST_SCHEMA:
        raise ValueError("unknown robot task-trial manifest schema")
    task_type = str(payload.get("task_type", ""))
    if task_type not in TASK_TYPES:
        raise ValueError(f"task_type must be one of {sorted(TASK_TYPES)}")
    methods = tuple(map(str, payload.get("methods", [])))
    if len(methods) < 2 or len(methods) != len(set(methods)):
        raise ValueError("task trial needs at least two unique methods")
    ours = str(payload.get("ours_method", ""))
    if ours not in methods:
        raise ValueError("ours_method must be included in methods")
    required_true = (
        "preregistered_before_trials",
        "calibration_frozen_before_trials",
        "method_order_randomized",
        "same_robot_hardware_across_methods",
        "same_perception_and_target_across_methods",
        "outcomes_not_used_to_change_calibration",
    )
    invalid_flags = [name for name in required_true if payload.get(name) is not True]
    if invalid_flags:
        raise ValueError(f"task-trial contract flags must be true: {invalid_flags}")
    success_definition = _non_placeholder_text(
        payload.get("success_definition"), "success_definition")
    measurement = _non_placeholder_text(
        payload.get("contact_error_measurement"),
        "contact_error_measurement")
    margins = payload.get("preregistered_margins", {})
    minimum_success_rate = float(margins.get("minimum_success_rate"))
    maximum_p95_error = float(margins.get("maximum_p95_contact_error_mm"))
    if (not np.isfinite(minimum_success_rate)
            or not 0.0 <= minimum_success_rate <= 1.0):
        raise ValueError("minimum_success_rate must be in [0, 1]")
    if not np.isfinite(maximum_p95_error) or maximum_p95_error < 0.0:
        raise ValueError("maximum_p95_contact_error_mm must be non-negative")

    normalized_sessions = []
    seen_sessions = set()
    for raw_session in payload.get("sessions", []):
        session_id = _non_placeholder_text(
            raw_session.get("session_id"), "session_id")
        if session_id in seen_sessions:
            raise ValueError(f"duplicate session_id {session_id}")
        seen_sessions.add(session_id)
        normalized_pairs = []
        seen_pairs = set()
        for raw_pair in raw_session.get("pairs", []):
            pair_id = _non_placeholder_text(raw_pair.get("pair_id"), "pair_id")
            if pair_id in seen_pairs:
                raise ValueError(
                    f"duplicate pair_id {pair_id} in session {session_id}")
            seen_pairs.add(pair_id)
            target_id = _non_placeholder_text(
                raw_pair.get("target_id"), "target_id")
            order = tuple(map(str, raw_pair.get("execution_order", [])))
            if len(order) != len(methods) or set(order) != set(methods):
                raise ValueError(
                    f"{session_id}/{pair_id}: execution_order must contain "
                    "every method exactly once")
            raw_trials = raw_pair.get("trials", {})
            if set(raw_trials) != set(methods):
                raise ValueError(
                    f"{session_id}/{pair_id}: trials must record every method; "
                    "missing attempts must be explicit failures")
            trials = {}
            for method in methods:
                raw_trial = raw_trials[method]
                success = raw_trial.get("success")
                if not isinstance(success, bool):
                    raise ValueError(
                        f"{session_id}/{pair_id}/{method}: success must be boolean")
                failure_reason = str(raw_trial.get("failure_reason") or "").strip()
                if not success and not failure_reason:
                    raise ValueError(
                        f"{session_id}/{pair_id}/{method}: failed trial needs "
                        "failure_reason")
                trials[method] = {
                    "success": success,
                    "failure_reason": failure_reason or None,
                    "contact_error_mm": _contact_error(
                        raw_trial.get("contact_error_mm"), required=success),
                }
            normalized_pairs.append({
                "pair_id": pair_id,
                "target_id": target_id,
                "strata": sorted(set(map(str, raw_pair.get("strata", [])))),
                "execution_order": list(order),
                "trials": trials,
            })
        if not normalized_pairs:
            raise ValueError(f"session {session_id} has no trial pairs")
        normalized_sessions.append({
            "session_id": session_id,
            "pairs": normalized_pairs,
        })
    if not normalized_sessions:
        raise ValueError("task-trial manifest has no sessions")
    return {
        "artifact_schema": MANIFEST_SCHEMA,
        "task_type": task_type,
        "methods": list(methods),
        "ours_method": ours,
        "success_definition": success_definition,
        "contact_error_measurement": measurement,
        "preregistered_margins": {
            "minimum_success_rate": minimum_success_rate,
            "maximum_p95_contact_error_mm": maximum_p95_error,
        },
        "sessions": normalized_sessions,
    }


def _percentile(values: Sequence[float], percentile: float) -> float:
    return float(np.percentile(np.asarray(values, dtype=np.float64), percentile))


def _summarize_trials(trials: Sequence[Mapping]) -> dict:
    attempts = len(trials)
    successes = [trial for trial in trials if trial["success"]]
    result = {
        "attempts": attempts,
        "successes": len(successes),
        "failures": attempts - len(successes),
        "success_rate": wilson_interval(len(successes), attempts),
    }
    errors = np.asarray([
        [trial["contact_error_mm"][axis] for axis in AXES]
        for trial in successes
    ], dtype=np.float64)
    if not len(errors):
        result["contact_error"] = {
            "available": False,
            "reason": "no successful trials",
            "successful_trials_measured": 0,
        }
        return result
    norms = np.linalg.norm(errors, axis=1)
    result["contact_error"] = {
        "available": True,
        "successful_trials_measured": int(len(errors)),
        "norm_mm": {
            "mean": float(np.mean(norms)),
            "p50": _percentile(norms, 50.0),
            "p95": _percentile(norms, 95.0),
            "max": float(np.max(norms)),
        },
        "signed_bias_mm": {
            axis: float(np.mean(errors[:, index]))
            for index, axis in enumerate(AXES)
        },
        "absolute_axis_error_mm": {
            axis: {
                "mean": float(np.mean(np.abs(errors[:, index]))),
                "p95": _percentile(np.abs(errors[:, index]), 95.0),
                "max": float(np.max(np.abs(errors[:, index]))),
            }
            for index, axis in enumerate(AXES)
        },
    }
    return result


def evaluate(payload: Mapping) -> dict:
    manifest = validate_manifest(payload)
    methods = manifest["methods"]
    ours = manifest["ours_method"]
    method_trials = {method: [] for method in methods}
    session_summaries = {}
    normalized_rows = []
    for session in manifest["sessions"]:
        by_method = {method: [] for method in methods}
        for pair in session["pairs"]:
            for order_index, method in enumerate(pair["execution_order"]):
                trial = pair["trials"][method]
                row = {
                    "session_id": session["session_id"],
                    "pair_id": pair["pair_id"],
                    "target_id": pair["target_id"],
                    "strata": pair["strata"],
                    "method": method,
                    "execution_order": order_index + 1,
                    **trial,
                }
                normalized_rows.append(row)
                method_trials[method].append(row)
                by_method[method].append(row)
        session_summaries[session["session_id"]] = {
            method: _summarize_trials(by_method[method]) for method in methods}

    method_summaries = {
        method: _summarize_trials(method_trials[method]) for method in methods}
    paired = {}
    for baseline in methods:
        if baseline == ours:
            continue
        pair_rows = []
        for session in manifest["sessions"]:
            for pair in session["pairs"]:
                ours_trial = pair["trials"][ours]
                baseline_trial = pair["trials"][baseline]
                pair_rows.append((ours_trial, baseline_trial))
        both_success = [
            (ours_trial, baseline_trial)
            for ours_trial, baseline_trial in pair_rows
            if ours_trial["success"] and baseline_trial["success"]
        ]
        norm_differences = [
            float(np.linalg.norm([
                ours_trial["contact_error_mm"][axis] for axis in AXES
            ]) - np.linalg.norm([
                baseline_trial["contact_error_mm"][axis] for axis in AXES
            ]))
            for ours_trial, baseline_trial in both_success
        ]
        paired[baseline] = {
            "n_paired_trials": len(pair_rows),
            "ours_success_baseline_failure": sum(
                ours_trial["success"] and not baseline_trial["success"]
                for ours_trial, baseline_trial in pair_rows),
            "baseline_success_ours_failure": sum(
                baseline_trial["success"] and not ours_trial["success"]
                for ours_trial, baseline_trial in pair_rows),
            "both_success": len(both_success),
            "both_failure": sum(
                not ours_trial["success"] and not baseline_trial["success"]
                for ours_trial, baseline_trial in pair_rows),
            "paired_success_rate_difference": float(np.mean([
                float(ours_trial["success"]) - float(baseline_trial["success"])
                for ours_trial, baseline_trial in pair_rows
            ])),
            "both_success_contact_norm_difference_mm": (
                None if not norm_differences else {
                    "mean": float(np.mean(norm_differences)),
                    "p50": _percentile(norm_differences, 50.0),
                    "p95": _percentile(norm_differences, 95.0),
                    "interpretation": "negative_favors_ours",
                }),
        }

    margins = manifest["preregistered_margins"]
    decisions = {}
    for method, summary in method_summaries.items():
        contact = summary["contact_error"]
        decisions[method] = {
            "success_rate_gate": bool(
                summary["success_rate"]["ci95_lower"]
                >= margins["minimum_success_rate"]),
            "p95_contact_error_gate": bool(
                contact.get("available", False)
                and contact["norm_mm"]["p95"]
                <= margins["maximum_p95_contact_error_mm"]),
        }
        decisions[method]["all_preregistered_gates_pass"] = bool(
            all(decisions[method].values()))
    return {
        "artifact_schema": "robot_task_trial_evaluation_v1",
        "protocol": {
            "task_type": manifest["task_type"],
            "methods": methods,
            "ours_method": ours,
            "paired_unit": "session_id_x_pair_id",
            "missing_attempt_policy": "must_be_recorded_as_explicit_failure",
            "contact_error_units": "signed_x_y_z_millimetres",
            "success_definition": manifest["success_definition"],
            "contact_error_measurement": manifest["contact_error_measurement"],
            "preregistered_margins": margins,
            "n_sessions": len(manifest["sessions"]),
            "confirmatory_ready": bool(len(manifest["sessions"]) >= 2),
        },
        "method_summaries": method_summaries,
        "session_summaries": session_summaries,
        "paired_ours_minus_baseline": paired,
        "method_gates": decisions,
        "trials": normalized_rows,
    }


def write_outputs(result: Mapping, output_dir: str) -> None:
    os.makedirs(output_dir, exist_ok=True)
    json_path = os.path.join(output_dir, "robot_task_trial_evaluation.json")
    csv_path = os.path.join(output_dir, "robot_task_trials.csv")
    md_path = os.path.join(output_dir, "ROBOT_TASK_TRIAL_EVALUATION.md")
    with open(json_path, "w") as handle:
        json.dump(result, handle, indent=2)
    rows = []
    for trial in result["trials"]:
        error = trial.get("contact_error_mm") or {}
        rows.append({
            **{key: value for key, value in trial.items()
               if key not in {"strata", "contact_error_mm"}},
            "strata": ";".join(trial.get("strata", [])),
            "contact_error_x_mm": error.get("x"),
            "contact_error_y_mm": error.get("y"),
            "contact_error_z_mm": error.get("z"),
        })
    with open(csv_path, "w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    lines = [
        "# Robot Task-Trial Evaluation",
        "",
        f"Task: `{result['protocol']['task_type']}`  ",
        f"Confirmatory ready: `{str(result['protocol']['confirmatory_ready']).lower()}`",
        "",
        "| Method | Success / attempts | Success rate [Wilson 95% CI] | Contact norm mean / P95 mm | Gates |",
        "| --- | ---: | ---: | ---: | --- |",
    ]
    for method, summary in result["method_summaries"].items():
        rate = summary["success_rate"]
        contact = summary["contact_error"]
        error_text = (
            f"{contact['norm_mm']['mean']:.3f} / {contact['norm_mm']['p95']:.3f}"
            if contact.get("available") else "N/A")
        lines.append(
            f"| {method} | {summary['successes']} / {summary['attempts']} | "
            f"{rate['rate']:.3f} [{rate['ci95_lower']:.3f}, "
            f"{rate['ci95_upper']:.3f}] | {error_text} | "
            f"{'PASS' if result['method_gates'][method]['all_preregistered_gates_pass'] else 'FAIL'} |"
        )
    lines.extend([
        "",
        "> Contact-error statistics use successful attempts only; every failed or "
        "missing attempt remains in the success-rate denominator.",
    ])
    with open(md_path, "w") as handle:
        handle.write("\n".join(lines) + "\n")


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description="Paired peg-in-hole/grasp robot task-trial evaluation")
    parser.add_argument("--manifest", required=True)
    parser.add_argument(
        "--output_dir", default="CP_result/shared/robot_task_trial")
    return parser.parse_args(argv)


def main(argv=None) -> None:
    args = parse_args(argv)
    with open(args.manifest) as handle:
        payload = json.load(handle)
    result = evaluate(payload)
    write_outputs(result, args.output_dir)
    print(f"[DONE] {os.path.join(args.output_dir, 'robot_task_trial_evaluation.json')}")


if __name__ == "__main__":
    main()
