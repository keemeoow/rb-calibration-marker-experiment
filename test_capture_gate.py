import unittest

from capture_gate import evaluate_capture_gate


def _pnp(depth=True, plane=5.0):
    return {
        "depth_valid": depth,
        "depth_num_samples": 30 if depth else 0,
        "depth_plane_mean_mm": plane,
        "reproj_mean_px": 0.5,
    }


GATE_CONFIG = {
    "profiles": {
        "A_placement": {
            "expected_cube_gripped": False,
            "min_cams_with_cube": 2,
            "min_fixed_cams_with_cube": 1,
            "min_fixed_multimarker_cams": 1,
            "fixed_multimarker_min_markers": 2,
            "max_cube_pnp_reproj_mean_px": 2.0,
            "min_depth_samples": 20,
            "min_cube_pnp_ok_cams": 2,
            "min_fixed_cube_pnp_ok_cams": 1,
            "min_gripper_markers": 1,
            "min_gripper_charuco_corners": 8,
            "require_gripper_cube_pnp": True,
            "require_gripper_depth_valid": True,
            "max_gripper_depth_plane_mean_mm": 20.0,
            "max_capture_span_ms": 120.0,
        },
        "B_eyetohand": {
            "expected_cube_gripped": True,
            "min_cams_with_cube": 0,
            "min_fixed_cams_with_cube": 2,
            "min_fixed_multimarker_cams": 2,
            "fixed_multimarker_min_markers": 2,
            "max_cube_pnp_reproj_mean_px": 2.0,
            "min_depth_samples": 20,
            "min_cube_pnp_ok_cams": 0,
            "min_fixed_cube_pnp_ok_cams": 2,
            "min_fixed_depth_quality_cams": 1,
            "max_fixed_depth_plane_mean_mm": 20.0,
            "min_gripper_markers": 0,
            "min_gripper_charuco_corners": 0,
            "require_gripper_cube_pnp": False,
            "require_gripper_depth_valid": False,
            "max_capture_span_ms": 120.0,
        },
    }
}


class CaptureGateTests(unittest.TestCase):
    def test_a_requires_gripper_cube_board_and_fixed_multimarker(self):
        frames = {
            0: {"ok": True, "n_markers": 2, "cube_pnp": _pnp(),
                "ch_ids": list(range(8)), "ts_ms": 1000.0},
            1: {"ok": True, "n_markers": 2, "cube_pnp": _pnp(),
                "ch_ids": None, "ts_ms": 1030.0},
        }
        result = evaluate_capture_gate(
            frames, GATE_CONFIG, gripper_cam_idx=0,
            capture_block="A_placement", cube_gripped=False)
        self.assertTrue(result["pass"], result["reason"])
        self.assertEqual(result["fixed_multimarker_cams"], 1)

    def test_b_ignores_gripper_observations_and_requires_two_fixed_cameras(self):
        frames = {
            0: {"ok": False, "n_markers": 0, "cube_pnp": None,
                "ch_ids": None, "ts_ms": 2000.0},
            1: {"ok": True, "n_markers": 2, "cube_pnp": _pnp(), "ts_ms": 2020.0},
            2: {"ok": True, "n_markers": 3, "cube_pnp": _pnp(), "ts_ms": 2030.0},
        }
        result = evaluate_capture_gate(
            frames, GATE_CONFIG, gripper_cam_idx=0,
            capture_block="B_eyetohand", cube_gripped=True)
        self.assertTrue(result["pass"], result["reason"])
        self.assertEqual(result["fixed_cube_pnp_ok_cams"], 2)

        frames.pop(2)
        result = evaluate_capture_gate(
            frames, GATE_CONFIG, gripper_cam_idx=0,
            capture_block="B_eyetohand", cube_gripped=True)
        self.assertFalse(result["pass"])
        self.assertIn("fixed cube-visible cams", result["reason"])

    def test_block_payload_mismatch_and_unknown_block_fail_closed(self):
        frames = {
            1: {"ok": True, "n_markers": 2, "cube_pnp": _pnp(), "ts_ms": 0.0},
            2: {"ok": True, "n_markers": 2, "cube_pnp": _pnp(), "ts_ms": 1.0},
        }
        mismatch = evaluate_capture_gate(
            frames, GATE_CONFIG, gripper_cam_idx=0,
            capture_block="B_eyetohand", cube_gripped=False)
        self.assertFalse(mismatch["pass"])
        self.assertIn("requires cube_gripped=True", mismatch["reason"])

        unknown = evaluate_capture_gate(
            frames, GATE_CONFIG, gripper_cam_idx=0,
            capture_block="typo", cube_gripped=True)
        self.assertFalse(unknown["pass"])
        self.assertIn("unknown capture_block", unknown["reason"])

    def test_missing_timestamp_or_depth_metric_does_not_count_as_quality(self):
        frames = {
            1: {"ok": True, "n_markers": 2,
                "cube_pnp": {"depth_valid": True, "depth_num_samples": 30,
                             "reproj_mean_px": 0.5}, "ts_ms": 10.0},
            2: {"ok": True, "n_markers": 2,
                "cube_pnp": _pnp(), "ts_ms": None},
        }
        result = evaluate_capture_gate(
            frames, GATE_CONFIG, gripper_cam_idx=0,
            capture_block="B_eyetohand", cube_gripped=True)
        self.assertFalse(result["pass"])
        self.assertEqual(result["fixed_depth_quality_cams"], 1)
        self.assertIn("missing timestamps", result["reason"])

    def test_high_reprojection_and_low_depth_support_are_not_quality_pnp(self):
        bad_reproj = _pnp()
        bad_reproj["reproj_mean_px"] = 3.0
        low_depth = _pnp()
        low_depth["depth_num_samples"] = 10
        frames = {
            1: {"ok": True, "n_markers": 2, "cube_pnp": bad_reproj, "ts_ms": 0.0},
            2: {"ok": True, "n_markers": 2, "cube_pnp": low_depth, "ts_ms": 1.0},
        }
        result = evaluate_capture_gate(
            frames, GATE_CONFIG, gripper_cam_idx=0,
            capture_block="B_eyetohand", cube_gripped=True)
        self.assertFalse(result["pass"])
        self.assertEqual(result["fixed_cube_pnp_ok_cams"], 1)
        self.assertEqual(result["fixed_depth_quality_cams"], 0)


if __name__ == "__main__":
    unittest.main()
