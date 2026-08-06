import unittest

from waypoint_safety import (
    shortest_joint_error_deg,
    validate_safe_joint_config,
    validate_waypoint_semantics,
)


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

    def test_joint_distance_handles_wrapping(self):
        error = shortest_joint_error_deg([359, 0, 0, 0, 0, 0], [-1, 0, 0, 0, 0, 0])
        self.assertAlmostEqual(error[0], 0.0)


if __name__ == "__main__":
    unittest.main()
