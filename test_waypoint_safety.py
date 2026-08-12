import unittest

from waypoint_safety import (
    CAPTURE_PROTOCOL_KEY,
    PROTOCOL_A_SETS_B_STATION,
    shortest_joint_error_deg,
    validate_safe_joint_config,
    validate_waypoint_semantics,
)


def _a_waypoint(capture_index, set_index):
    return {
        "capture_index": capture_index,
        "set_index": set_index,
        "capture_block": "A_placement",
        "cube_gripped": False,
        "capture_joints": [1, 2, 3, 4, 5, 6],
        "place_joints": [6, 5, 4, 3, 2, 1],
        "set_cube_center_6dof": [100, 200, 30, 0, 0, 0],
    }


def _b_waypoint(capture_index, set_index):
    return {
        "capture_index": capture_index,
        "set_index": set_index,
        "capture_block": "B_eyetohand",
        "cube_gripped": True,
        "capture_tcp": [100, 200, 300, 0, 90, 180],
        "place_joints": [6, 5, 4, 3, 2, 1],
        "set_cube_center_6dof": [100, 200, 30, 0, 0, 0],
    }


def _station_payload():
    """Two A-only placements followed by one B-only station."""
    return {
        CAPTURE_PROTOCOL_KEY: PROTOCOL_A_SETS_B_STATION,
        "safe_joints_empty": [0, -20, -100, 0, -50, 0],
        "safe_joints_gripped": [0, -25, -95, 0, -55, 0],
        "waypoints": [
            _a_waypoint(0, 0),
            _a_waypoint(1, 1),
            _b_waypoint(2, 9),
        ],
    }


def _valid_payload():
    return {
        "safe_joints_empty": [0, -20, -100, 0, -50, 0],
        "safe_joints_gripped": [0, -25, -95, 0, -55, 0],
        "waypoints": [
            {
                "capture_index": 0,
                "set_index": 0,
                "capture_block": "A_placement",
                "cube_gripped": False,
                "capture_joints": [1, 2, 3, 4, 5, 6],
                "place_joints": [6, 5, 4, 3, 2, 1],
                "set_cube_center_6dof": [100, 200, 30, 0, 0, 0],
            },
            {
                "capture_index": 1,
                "set_index": 0,
                "capture_block": "B_eyetohand",
                "cube_gripped": True,
                "capture_tcp": [100, 200, 300, 0, 90, 180],
                "place_joints": [6, 5, 4, 3, 2, 1],
                "set_cube_center_6dof": [100, 200, 30, 0, 0, 0],
            },
        ],
    }


class WaypointSafetyTests(unittest.TestCase):
    def test_both_payload_safe_poses_are_required(self):
        payload = _valid_payload()
        safe = validate_safe_joint_config(payload)
        self.assertEqual(len(safe["safe_joints_empty"]), 6)

        payload.pop("safe_joints_gripped")
        with self.assertRaisesRegex(ValueError, "safe_joints_gripped"):
            validate_safe_joint_config(payload)

    def test_waypoint_block_semantics_fail_closed(self):
        payload = _valid_payload()
        self.assertTrue(validate_waypoint_semantics(payload))

        payload["waypoints"][1]["cube_gripped"] = False
        with self.assertRaisesRegex(ValueError, "requires cube_gripped=True"):
            validate_waypoint_semantics(payload)

        payload = _valid_payload()
        payload["waypoints"].pop()
        with self.assertRaisesRegex(ValueError, "must contain both"):
            validate_waypoint_semantics(payload)

    def test_station_protocol_accepts_a_only_sets(self):
        self.assertTrue(validate_waypoint_semantics(_station_payload()))

    def test_station_protocol_requires_exactly_one_station(self):
        payload = _station_payload()
        payload["waypoints"] = [_a_waypoint(0, 0), _a_waypoint(1, 1)]
        with self.assertRaisesRegex(ValueError, "exactly one B-only station"):
            validate_waypoint_semantics(payload)

        payload = _station_payload()
        payload["waypoints"].insert(1, _b_waypoint(3, 5))
        with self.assertRaisesRegex(ValueError, "exactly one B-only station"):
            validate_waypoint_semantics(payload)

    def test_station_protocol_rejects_mixed_sets(self):
        payload = _station_payload()
        payload["waypoints"].insert(1, _b_waypoint(3, 0))
        with self.assertRaisesRegex(ValueError, "A-only"):
            validate_waypoint_semantics(payload)

    def test_station_must_run_last_so_the_cube_is_never_stranded(self):
        payload = _station_payload()
        payload["waypoints"] = [_a_waypoint(0, 0), _b_waypoint(2, 9), _a_waypoint(1, 1)]
        with self.assertRaisesRegex(ValueError, "must be the last set"):
            validate_waypoint_semantics(payload)

    def test_station_protocol_requires_at_least_one_placement(self):
        payload = _station_payload()
        payload["waypoints"] = [_b_waypoint(0, 9)]
        with self.assertRaisesRegex(ValueError, "at least one A-only placement"):
            validate_waypoint_semantics(payload)

    def test_unknown_protocol_is_rejected(self):
        payload = _station_payload()
        payload[CAPTURE_PROTOCOL_KEY] = "whatever_looks_fine"
        with self.assertRaisesRegex(ValueError, "unknown capture_protocol"):
            validate_waypoint_semantics(payload)

    def test_missing_protocol_keeps_the_legacy_contract(self):
        payload = _station_payload()
        payload.pop(CAPTURE_PROTOCOL_KEY)
        with self.assertRaisesRegex(ValueError, "must contain both"):
            validate_waypoint_semantics(payload)

    def test_joint_distance_handles_wrapping(self):
        error = shortest_joint_error_deg([359, 0, 0, 0, 0, 0], [-1, 0, 0, 0, 0, 0])
        self.assertAlmostEqual(error[0], 0.0)


if __name__ == "__main__":
    unittest.main()
