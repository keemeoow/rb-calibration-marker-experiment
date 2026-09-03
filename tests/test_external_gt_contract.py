"""External-GT reporting must stay method-agnostic and include A4/A5."""

from calibration_pipeline.external_gt import MANIFEST_SCHEMA, SessionData, evaluate


def test_external_gt_contract_compares_configured_primary_to_a2_and_a5():
    methods = ("A4", "A2", "A5")
    manifest = {
        "ours_method": "A4",
        "methods": methods,
        "bootstrap": {"repetitions": 200, "seed": 7},
        "margins": {
            "rotation_deg": 1.0,
            "p95_tre_mm": 2.0,
            "failure_rate": 0.1,
            "worst_stratum_p95_tre_mm": 2.0,
        },
        "gt_uncertainty_floor": {"translation_mm": 0.01},
        "add_mode": "none",
    }
    metrics = {}
    sessions = []
    for session_index in range(2):
        session_id = f"session_{session_index}"
        metrics[session_id] = {
            method: {
                "pose_1": {
                    "success": True,
                    "tre_mm": 1.0 + float(method != "A4"),
                    "rotation_deg": 0.1 + 0.1 * float(method != "A4"),
                    "strata": ["workspace_center"],
                }
            }
            for method in methods
        }
        sessions.append(SessionData(
            session_id=session_id,
            gt={},
            predictions={},
            registered_cameras={method: 4 for method in methods},
        ))

    result = evaluate(manifest, sessions, metrics)

    assert MANIFEST_SCHEMA == "external_gt_eval_manifest_v2"
    assert result["protocol"]["ours_method"] == "A4"
    assert set(result["comparisons_ours_minus_baseline"]) == {"A2", "A5"}
    assert "comparisons_A4_minus_baseline" not in result
