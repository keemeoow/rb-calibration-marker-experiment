#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Python 2/3-compatible validation for auto-capture safe joint poses."""

import math


SAFE_EMPTY_KEY = "safe_joints_empty"
SAFE_GRIPPED_KEY = "safe_joints_gripped"


def validate_joint_vector(value, label):
    if not isinstance(value, (list, tuple)) or len(value) != 6:
        raise ValueError("{} must be an explicit 6-element joint vector".format(label))
    out = []
    for idx, raw in enumerate(value):
        try:
            val = float(raw)
        except (TypeError, ValueError):
            raise ValueError("{}[{}] is not numeric".format(label, idx))
        if math.isnan(val) or math.isinf(val):
            raise ValueError("{}[{}] must be finite".format(label, idx))
        out.append(val)
    return out


SAFE_MODE_KEY = "safe_pose_mode"
SAFE_MODE_Z_LIFT = "z_lift_only"

# Which A/B layout the payload claims. The validator enforces the claimed shape
# exactly, so a lost or misrouted block still aborts before the first motion.
CAPTURE_PROTOCOL_KEY = "capture_protocol"
PROTOCOL_PER_SET_AB = "per_set_AB"          # every set: B sweep + A placement
PROTOCOL_A_SETS_B_STATION = "A_sets_plus_B_station"  # A-only sets, one terminal B station
PROTOCOL_COMPOSITE_RIG_45 = "composite_rig_45_v1"
CAPTURE_PROTOCOLS = (
    PROTOCOL_PER_SET_AB,
    PROTOCOL_A_SETS_B_STATION,
    PROTOCOL_COMPOSITE_RIG_45,
)

PHASE_P1 = "P1_MOVING_RIG"
PHASE_P2 = "P2_PICK_PLACE"
PHASE_P3 = "P3_STATIONARY_RIG"
TARGET_GRIPPED = "gripped"
TARGET_RELEASED = "released"
TARGET_STATIONARY = "stationary"


def _require_nonempty_text(value, label):
    if value is None or not str(value).strip():
        raise ValueError("{} must be a non-empty string".format(label))
    return str(value)


def _validate_capture_joint_waypoint(wp, label):
    """Final protocol uses taught joint poses; no inferred robot coordinates."""
    return validate_joint_vector(wp.get("capture_joints"), label + ".capture_joints")


def _validate_composite_rig_45(data, waypoints):
    """Validate the preregistered 15/20/10 composite-rig capture plan."""
    if data.get("schema_version") != "capture_pose_plan_v2":
        raise ValueError("schema_version must be capture_pose_plan_v2")
    if data.get("template_only") is not False:
        raise ValueError("template_only must be false after all taught poses are filled")
    if data.get(SAFE_MODE_KEY) == SAFE_MODE_Z_LIFT:
        raise ValueError(
            "{} requires explicit safe_joints_empty and safe_joints_gripped; "
            "z_lift_only is not allowed".format(PROTOCOL_COMPOSITE_RIG_45)
        )
    validate_joint_vector(data.get(SAFE_EMPTY_KEY), SAFE_EMPTY_KEY)
    validate_joint_vector(data.get(SAFE_GRIPPED_KEY), SAFE_GRIPPED_KEY)
    _require_nonempty_text(data.get("target_rig_id"), "target_rig_id")
    _require_nonempty_text(data.get("rig_geometry_file"), "rig_geometry_file")
    rig_hash = _require_nonempty_text(
        data.get("rig_geometry_sha256"), "rig_geometry_sha256"
    )
    if len(rig_hash) != 64 or any(ch not in "0123456789abcdefABCDEF" for ch in rig_hash):
        raise ValueError("rig_geometry_sha256 must be a 64-character hexadecimal SHA-256")
    raw_max_attempts = data.get("max_transport_attempts_per_event", 3)
    try:
        max_attempts = int(raw_max_attempts)
    except (TypeError, ValueError):
        raise ValueError("max_transport_attempts_per_event must be an integer")
    if isinstance(raw_max_attempts, bool) or float(raw_max_attempts) != max_attempts:
        raise ValueError("max_transport_attempts_per_event must be an integer")
    if max_attempts < 1 or max_attempts > 5:
        raise ValueError("max_transport_attempts_per_event must be in [1, 5]")

    placements = data.get("placements")
    if not isinstance(placements, list) or len(placements) != 10:
        raise ValueError("placements must contain exactly 10 entries")
    placement_by_id = {}
    for idx, placement in enumerate(placements):
        label = "placements[{}]".format(idx)
        if not isinstance(placement, dict):
            raise ValueError("{} must be an object".format(label))
        expected_id = "S{:02d}".format(idx)
        placement_id = placement.get("placement_id")
        if placement_id != expected_id:
            raise ValueError(
                "{}.placement_id must be {!r}".format(label, expected_id)
            )
        if placement_id in placement_by_id:
            raise ValueError("duplicate placement_id {!r}".format(placement_id))
        validate_joint_vector(
            placement.get("place_approach_joints"),
            label + ".place_approach_joints",
        )
        place_tcp = validate_joint_vector(
            placement.get("place_tcp"), label + ".place_tcp"
        )
        approach_tcp = validate_joint_vector(
            placement.get("place_approach_tcp"), label + ".place_approach_tcp"
        )
        if approach_tcp[2] - place_tcp[2] < 10.0:
            raise ValueError(
                "{}.place_approach_tcp must be at least 10mm above place_tcp".format(label)
            )
        if max(abs(approach_tcp[axis] - place_tcp[axis]) for axis in (0, 1)) > 2.0:
            raise ValueError(
                "{}.place approach/contact x,y must match within 2mm".format(label)
            )
        rotation_error = []
        for axis in (3, 4, 5):
            delta = abs(approach_tcp[axis] - place_tcp[axis]) % 360.0
            rotation_error.append(min(delta, 360.0 - delta))
        if max(rotation_error) > 2.0:
            raise ValueError(
                "{}.place approach/contact rotation must match within 2deg".format(label)
            )
        placement_by_id[placement_id] = placement

    if len(waypoints) != 45:
        raise ValueError(
            "{} requires exactly 45 waypoints, found {}".format(
                PROTOCOL_COMPOSITE_RIG_45, len(waypoints)
            )
        )

    expected = []
    for idx in range(15):
        expected.append(("P1_{:02d}".format(idx), PHASE_P1, TARGET_GRIPPED, None, idx))
    for placement_idx in range(10):
        for view_idx in range(2):
            expected.append((
                "P2_S{:02d}_V{}".format(placement_idx, view_idx),
                PHASE_P2,
                TARGET_RELEASED,
                "S{:02d}".format(placement_idx),
                view_idx,
            ))
    for idx in range(10):
        expected.append((
            "P3_{:02d}".format(idx),
            PHASE_P3,
            TARGET_STATIONARY,
            "S09",
            idx,
        ))

    seen_ids = set()
    for idx, (wp, exp) in enumerate(zip(waypoints, expected)):
        label = "waypoints[{}]".format(idx)
        if not isinstance(wp, dict):
            raise ValueError("{} must be an object".format(label))
        event_id, phase, target_state, placement_id, view_index = exp
        if wp.get("planned_event_id") != event_id:
            raise ValueError(
                "{}.planned_event_id must be {!r}".format(label, event_id)
            )
        if event_id in seen_ids:
            raise ValueError("duplicate planned_event_id {!r}".format(event_id))
        seen_ids.add(event_id)
        if wp.get("capture_index") != idx:
            raise ValueError("{}.capture_index must be {}".format(label, idx))
        if wp.get("phase") != phase:
            raise ValueError("{}.phase must be {!r}".format(label, phase))
        if wp.get("target_state") != target_state:
            raise ValueError(
                "{}.target_state must be {!r}".format(label, target_state)
            )
        if wp.get("placement_id") != placement_id:
            raise ValueError(
                "{}.placement_id must be {!r}".format(label, placement_id)
            )
        if wp.get("view_index") != view_index:
            raise ValueError(
                "{}.view_index must be {!r}".format(label, view_index)
            )
        _validate_capture_joint_waypoint(wp, label)
        expected_gripped = phase == PHASE_P1
        if wp.get("cube_gripped") is not expected_gripped:
            raise ValueError(
                "{}.cube_gripped must be {}".format(label, expected_gripped)
            )
        expected_block = "B_eyetohand" if expected_gripped else "A_placement"
        if wp.get("capture_block") != expected_block:
            raise ValueError(
                "{}.capture_block must be {!r}".format(label, expected_block)
            )
        if placement_id is not None and placement_id not in placement_by_id:
            raise ValueError(
                "{}.placement_id {!r} is not declared".format(label, placement_id)
            )
    return True


def validate_safe_joint_config(data):
    """Fail closed: empty and gripped payload safe poses are both mandatory.

    The single exception is an explicit ``safe_pose_mode: "z_lift_only"`` in the
    payload, which returns None so the executor retracts along +Z instead of
    routing through a taught safe pose. That opt-out has to be written into the
    waypoint file on purpose — a missing key still aborts — so a session shot
    without safe poses is identifiable from its own artifacts afterwards.
    """
    if not isinstance(data, dict):
        raise ValueError("waypoint payload must be an object")
    if data.get(SAFE_MODE_KEY) == SAFE_MODE_Z_LIFT:
        return None
    empty = validate_joint_vector(data.get(SAFE_EMPTY_KEY), SAFE_EMPTY_KEY)
    gripped = validate_joint_vector(data.get(SAFE_GRIPPED_KEY), SAFE_GRIPPED_KEY)
    return {SAFE_EMPTY_KEY: empty, SAFE_GRIPPED_KEY: gripped}


def shortest_joint_error_deg(actual, target):
    """Per-axis absolute angular error, accounting for equivalent +/-360 deg."""
    a = validate_joint_vector(actual, "actual_joints")
    b = validate_joint_vector(target, "target_joints")
    errors = []
    for av, bv in zip(a, b):
        delta = abs(av - bv) % 360.0
        errors.append(min(delta, 360.0 - delta))
    return errors


def validate_waypoint_semantics(data):
    """Reject mislabeled A/B records before the robot performs any motion.

    The payload must declare one of the supported layouts via
    ``capture_protocol``.  A missing key means the legacy ``per_set_AB`` shape,
    so an old file keeps its old contract.  Neither branch accepts a set whose
    blocks merely "look plausible": the claimed shape is checked exactly, so a
    dropped B station or a half-written set aborts before the robot moves.
    """
    if not isinstance(data, dict):
        raise ValueError("waypoint payload must be an object")
    protocol = data.get(CAPTURE_PROTOCOL_KEY, PROTOCOL_PER_SET_AB)
    if protocol not in CAPTURE_PROTOCOLS:
        raise ValueError(
            "unknown {} {!r} (expected one of {})".format(
                CAPTURE_PROTOCOL_KEY, protocol, ", ".join(CAPTURE_PROTOCOLS))
        )
    waypoints = data.get("waypoints")
    if not isinstance(waypoints, list) or not waypoints:
        raise ValueError("waypoints must be a non-empty list")
    if protocol == PROTOCOL_COMPOSITE_RIG_45:
        return _validate_composite_rig_45(data, waypoints)
    seen_capture_indices = set()
    blocks_by_set = {}
    set_order = []
    for idx, wp in enumerate(waypoints):
        if not isinstance(wp, dict):
            raise ValueError("waypoints[{}] must be an object".format(idx))
        block = wp.get("capture_block")
        if block not in ("A_placement", "B_eyetohand"):
            raise ValueError("waypoints[{}] has unknown capture_block {!r}".format(idx, block))
        expected = block == "B_eyetohand"
        if wp.get("cube_gripped") is not expected:
            raise ValueError(
                "waypoints[{}] block {} requires cube_gripped={}".format(idx, block, expected)
            )
        capture_index = wp.get("capture_index")
        if capture_index is None or capture_index in seen_capture_indices:
            raise ValueError("waypoints[{}] capture_index is missing or duplicated".format(idx))
        seen_capture_indices.add(capture_index)
        set_index = wp.get("set_index")
        if set_index is None:
            raise ValueError("waypoints[{}].set_index is required".format(idx))
        if set_index not in blocks_by_set:
            set_order.append(set_index)
        blocks_by_set.setdefault(set_index, set()).add(block)
        validate_joint_vector(wp.get("place_joints"), "waypoints[{}].place_joints".format(idx))
        validate_joint_vector(
            wp.get("set_cube_center_6dof"),
            "waypoints[{}].set_cube_center_6dof".format(idx),
        )
        if block == "A_placement":
            validate_joint_vector(
                wp.get("capture_joints"), "waypoints[{}].capture_joints".format(idx)
            )
        else:
            validate_joint_vector(
                wp.get("capture_tcp"), "waypoints[{}].capture_tcp".format(idx)
            )
    a_only = set(("A_placement",))
    b_only = set(("B_eyetohand",))
    if protocol == PROTOCOL_PER_SET_AB:
        required_blocks = set(("A_placement", "B_eyetohand"))
        for set_index, blocks in blocks_by_set.items():
            if blocks != required_blocks:
                raise ValueError(
                    "set_index {} must contain both A_placement and B_eyetohand".format(set_index)
                )
        return True

    # PROTOCOL_A_SETS_B_STATION: the cube is carried to one station at the end
    # and swept there once, so every other set is A-only.  The station has to be
    # last because the executor only re-grips the cube while sets remain
    # (server _run_auto_multiset), and a station placed earlier would strand the
    # remaining placements with the cube already released.
    stations = [s for s in set_order if blocks_by_set[s] == b_only]
    placements = [s for s in set_order if blocks_by_set[s] == a_only]
    if len(stations) != 1:
        raise ValueError(
            "{} requires exactly one B-only station set, found {}".format(
                PROTOCOL_A_SETS_B_STATION, len(stations))
        )
    if len(placements) != len(set_order) - 1:
        mixed = [s for s in set_order if s not in stations and s not in placements]
        raise ValueError(
            "{} requires every non-station set to be A-only; sets {} carry both "
            "blocks".format(PROTOCOL_A_SETS_B_STATION, mixed)
        )
    if not placements:
        raise ValueError(
            "{} requires at least one A-only placement set".format(PROTOCOL_A_SETS_B_STATION)
        )
    if stations[0] != set_order[-1]:
        raise ValueError(
            "{}: B station set_index {} must be the last set, but {} follows it".format(
                PROTOCOL_A_SETS_B_STATION, stations[0], set_order[-1])
        )
    return True
