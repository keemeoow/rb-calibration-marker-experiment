#!/usr/bin/env python3
"""Validate a filled composite-rig pose plan without cameras or robot hardware."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import sys

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from capture_pipeline.capture import load_and_validate_rig_geometry  # noqa: E402
from capture_pipeline.waypoint_safety import (  # noqa: E402
    PROTOCOL_COMPOSITE_RIG_45,
    validate_waypoint_semantics,
)


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("plan", help="Filled capture pose-plan JSON")
    args = parser.parse_args()

    plan_path = Path(args.plan).expanduser().resolve()
    plan = json.loads(plan_path.read_text(encoding="utf-8"))
    if plan.get("capture_protocol") != PROTOCOL_COMPOSITE_RIG_45:
        raise SystemExit(
            f"FAIL: capture_protocol must be {PROTOCOL_COMPOSITE_RIG_45}"
        )
    try:
        validate_waypoint_semantics(plan)
    except ValueError as error:
        raise SystemExit(f"FAIL: {error}") from error

    geometry_path = Path(plan["rig_geometry_file"]).expanduser()
    if not geometry_path.is_absolute():
        geometry_path = plan_path.parent / geometry_path
    geometry_path = geometry_path.resolve()
    try:
        load_and_validate_rig_geometry(geometry_path, str(plan["target_rig_id"]))
    except (OSError, ValueError, TypeError) as error:
        raise SystemExit(f"FAIL: {error}") from error
    actual_hash = _sha256(geometry_path)
    if actual_hash.lower() != str(plan["rig_geometry_sha256"]).lower():
        raise SystemExit(
            "rig geometry SHA-256 mismatch:\n"
            f"  declared: {plan['rig_geometry_sha256']}\n"
            f"  actual:   {actual_hash}"
        )

    phase_counts = {}
    for waypoint in plan["waypoints"]:
        phase = waypoint["phase"]
        phase_counts[phase] = phase_counts.get(phase, 0) + 1
    print(f"PASS: {plan_path}")
    print(f"target_rig_id: {plan['target_rig_id']}")
    print(f"rig_geometry_sha256: {actual_hash}")
    print(f"phase_counts: {phase_counts}")


if __name__ == "__main__":
    main()
