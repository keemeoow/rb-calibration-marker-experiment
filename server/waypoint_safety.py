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


def validate_safe_joint_config(data):
    """Fail closed: empty and gripped payload safe poses are both mandatory."""
    if not isinstance(data, dict):
        raise ValueError("waypoint payload must be an object")
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
    """Reject mislabeled A/B records before the robot performs any motion."""
    if not isinstance(data, dict):
        raise ValueError("waypoint payload must be an object")
    waypoints = data.get("waypoints")
    if not isinstance(waypoints, list) or not waypoints:
        raise ValueError("waypoints must be a non-empty list")
    seen_capture_indices = set()
    blocks_by_set = {}
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
    required_blocks = set(("A_placement", "B_eyetohand"))
    for set_index, blocks in blocks_by_set.items():
        if blocks != required_blocks:
            raise ValueError(
                "set_index {} must contain both A_placement and B_eyetohand".format(set_index)
            )
    return True
