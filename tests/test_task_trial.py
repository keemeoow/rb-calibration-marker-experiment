from __future__ import annotations

import pytest

from calibration_pipeline.task_trial import (
    MANIFEST_SCHEMA,
    evaluate,
    validate_manifest,
    wilson_interval,
)


def _manifest():
    pairs = []
    outcomes = [
        (True, True),
        (True, False),
        (False, False),
    ]
    for index, (a2_success, a4_success) in enumerate(outcomes):
        trials = {}
        for method, success, error in (
            ("A2", a2_success, 1.0),
            ("A4", a4_success, 2.0),
        ):
            trials[method] = {
                "success": success,
                "failure_reason": None if success else "insertion_failed",
                "contact_error_mm": (
                    {"x": error, "y": 0.0, "z": 0.0}
                    if success else None),
            }
        pairs.append({
            "pair_id": f"pair_{index}",
            "target_id": f"target_{index}",
            "strata": ["center"],
            "execution_order": ["A4", "A2"] if index % 2 == 0 else ["A2", "A4"],
            "trials": trials,
        })
    return {
        "artifact_schema": MANIFEST_SCHEMA,
        "task_type": "peg_in_hole",
        "methods": ["A2", "A4"],
        "ours_method": "A2",
        "preregistered_before_trials": True,
        "calibration_frozen_before_trials": True,
        "method_order_randomized": True,
        "same_robot_hardware_across_methods": True,
        "same_perception_and_target_across_methods": True,
        "outcomes_not_used_to_change_calibration": True,
        "success_definition": "peg reaches the mechanical stop without intervention",
        "contact_error_measurement": "independent XYZ dial-indicator fixture",
        "preregistered_margins": {
            "minimum_success_rate": 0.0,
            "maximum_p95_contact_error_mm": 5.0,
        },
        "sessions": [{"session_id": "session_01", "pairs": pairs}],
    }


def test_task_trial_keeps_failures_in_denominator_and_reports_paired_result():
    result = evaluate(_manifest())

    a2 = result["method_summaries"]["A2"]
    assert a2["attempts"] == 3
    assert a2["successes"] == 2
    assert a2["success_rate"]["rate"] == pytest.approx(2.0 / 3.0)
    assert a2["contact_error"]["norm_mm"]["mean"] == pytest.approx(1.0)
    paired = result["paired_ours_minus_baseline"]["A4"]
    assert paired["n_paired_trials"] == 3
    assert paired["ours_success_baseline_failure"] == 1
    assert paired["paired_success_rate_difference"] == pytest.approx(1.0 / 3.0)
    assert result["protocol"]["confirmatory_ready"] is False


def test_manifest_rejects_a_missing_method_attempt_instead_of_dropping_it():
    manifest = _manifest()
    del manifest["sessions"][0]["pairs"][0]["trials"]["A4"]

    with pytest.raises(ValueError, match="missing attempts must be explicit failures"):
        validate_manifest(manifest)


def test_successful_trial_requires_independent_xyz_contact_error():
    manifest = _manifest()
    manifest["sessions"][0]["pairs"][0]["trials"]["A2"]["contact_error_mm"] = None

    with pytest.raises(ValueError, match="successful trial is missing"):
        validate_manifest(manifest)


def test_wilson_interval_is_bounded_for_all_successes():
    interval = wilson_interval(10, 10)

    assert interval["rate"] == 1.0
    assert 0.0 < interval["ci95_lower"] < 1.0
    assert interval["ci95_upper"] == 1.0
