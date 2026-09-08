#!/usr/bin/env python3
"""Create a non-executable 45-event composite-rig pose-plan template."""

from __future__ import annotations

import argparse
import json
from pathlib import Path


PROTOCOL = "composite_rig_45_v1"


def build_template() -> dict:
    placements = [
        {
            "placement_id": f"S{idx:02d}",
            "place_approach_joints": None,
            "place_approach_tcp": None,
            "place_tcp": None,
        }
        for idx in range(10)
    ]

    waypoints = []
    capture_index = 0
    for idx in range(15):
        waypoints.append({
            "capture_index": capture_index,
            "planned_event_id": f"P1_{idx:02d}",
            "phase": "P1_MOVING_RIG",
            "target_state": "gripped",
            "placement_id": None,
            "view_index": idx,
            "capture_block": "B_eyetohand",
            "cube_gripped": True,
            "capture_joints": None,
        })
        capture_index += 1

    for placement_idx in range(10):
        placement_id = f"S{placement_idx:02d}"
        for view_idx in range(2):
            waypoints.append({
                "capture_index": capture_index,
                "planned_event_id": f"P2_{placement_id}_V{view_idx}",
                "phase": "P2_PICK_PLACE",
                "target_state": "released",
                "placement_id": placement_id,
                "view_index": view_idx,
                "capture_block": "A_placement",
                "cube_gripped": False,
                "capture_joints": None,
            })
            capture_index += 1

    for idx in range(10):
        waypoints.append({
            "capture_index": capture_index,
            "planned_event_id": f"P3_{idx:02d}",
            "phase": "P3_STATIONARY_RIG",
            "target_state": "stationary",
            "placement_id": "S09",
            "view_index": idx,
            "capture_block": "A_placement",
            "cube_gripped": False,
            "capture_joints": None,
        })
        capture_index += 1

    return {
        "schema_version": "capture_pose_plan_v2",
        "capture_protocol": PROTOCOL,
        "template_only": True,
        "target_rig_id": "FILL_ME",
        "rig_geometry_file": "FILL_ME/composite_rig_geometry.json",
        "rig_geometry_sha256": "FILL_ME",
        "pose_convention": "joint_deg; TCP/flange=[x,y,z,rz,ry,rx] mm/deg, RzRyRx",
        "max_transport_attempts_per_event": 3,
        "safe_joints_empty": None,
        "safe_joints_gripped": None,
        "placements": placements,
        "waypoints": waypoints,
    }


def main() -> None:
    parser = argparse.ArgumentParser(
        description=(
            "Create the 45-event pose-plan skeleton. It is intentionally invalid "
            "until every FILL_ME/null field is replaced by a taught value."
        )
    )
    parser.add_argument("--out", required=True, help="Output JSON path")
    parser.add_argument(
        "--force", action="store_true", help="Overwrite an existing output file"
    )
    args = parser.parse_args()

    output = Path(args.out).expanduser().resolve()
    if output.exists() and not args.force:
        raise SystemExit(f"refusing to overwrite existing file: {output}")
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("x" if not args.force else "w", encoding="utf-8") as handle:
        json.dump(build_template(), handle, indent=2)
        handle.write("\n")
    print(output)


if __name__ == "__main__":
    main()
