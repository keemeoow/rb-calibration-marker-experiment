import unittest

from capture_gate import evaluate_capture_gate, resolve_camera_storage


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




class MarkerRoiQualityGateTests(unittest.TestCase):
    """The photometric gate: blown-out or smeared marker regions must not save."""

    def _frames(self, roi0=None, roi1=None):
        frames = {
            0: {"ok": True, "n_markers": 2, "cube_pnp": _pnp(),
                "ch_ids": list(range(8)), "ts_ms": 1000.0},
            1: {"ok": True, "n_markers": 2, "cube_pnp": _pnp(),
                "ch_ids": None, "ts_ms": 1030.0},
        }
        if roi0 is not None:
            frames[0]["roi_quality"] = roi0
        if roi1 is not None:
            frames[1]["roi_quality"] = roi1
        return frames

    def _run(self, frames, **overrides):
        cfg = {"profiles": {"A_placement": dict(GATE_CONFIG["profiles"]["A_placement"],
                                                **overrides)}}
        return evaluate_capture_gate(
            frames, cfg, gripper_cam_idx=0,
            capture_block="A_placement", cube_gripped=False)

    def test_clipped_marker_regions_reject_the_capture(self):
        roi = {"clip_frac_median": 0.30, "clip_frac_max": 0.40,
               "sharpness_min": 5000.0, "roi_px_min": 40.0, "n_rois": 2}
        result = self._run(self._frames(roi1=roi), max_roi_clip_frac=0.05)
        self.assertFalse(result["pass"])
        self.assertIn("clipped", result["reason"])
        self.assertIn("1", result["reason"])

    def test_one_specular_marker_does_not_reject_a_clean_camera(self):
        # median clean, max blown: a single highlight is diagnosed, not rejected.
        roi = {"clip_frac_median": 0.0, "clip_frac_max": 0.40,
               "sharpness_min": 5000.0, "roi_px_min": 40.0, "n_rois": 4}
        result = self._run(self._frames(roi1=roi), max_roi_clip_frac=0.05)
        self.assertTrue(result["pass"], result["reason"])
        self.assertEqual(result["per_camera"][1]["roi_clip_frac_max"], 0.40)

    def test_blurred_marker_regions_reject_when_threshold_is_set(self):
        roi = {"clip_frac_median": 0.0, "clip_frac_max": 0.0,
               "sharpness_min": 120.0, "roi_px_min": 40.0, "n_rois": 2}
        result = self._run(self._frames(roi1=roi), min_roi_sharpness=1000.0)
        self.assertFalse(result["pass"])
        self.assertIn("blurred", result["reason"])

    def test_thresholds_at_zero_disable_the_rules(self):
        roi = {"clip_frac_median": 0.9, "clip_frac_max": 0.9,
               "sharpness_min": 1.0, "roi_px_min": 40.0, "n_rois": 2}
        result = self._run(self._frames(roi1=roi),
                           max_roi_clip_frac=0.0, min_roi_sharpness=0.0)
        self.assertTrue(result["pass"], result["reason"])

    def test_missing_roi_quality_is_unknown_not_a_rejection(self):
        # Legacy frames carry no roi_quality; the gate must not invent a verdict.
        result = self._run(self._frames(), max_roi_clip_frac=0.05,
                           min_roi_sharpness=1000.0)
        self.assertTrue(result["pass"], result["reason"])
        self.assertIsNone(result["per_camera"][1]["roi_clip_frac_median"])


class CameraStorageTests(unittest.TestCase):
    def _a(self, already, **kw):
        opts = dict(is_placement=True, set_index=3,
                    fixed_views_already_stored=already,
                    a_fixed_cam_views_per_set=1, b_save_gripper_cam=False)
        opts.update(kw)
        return resolve_camera_storage(**opts)

    def _b(self, **kw):
        opts = dict(is_placement=False, set_index=3,
                    fixed_views_already_stored=0,
                    a_fixed_cam_views_per_set=1, b_save_gripper_cam=False)
        opts.update(kw)
        return resolve_camera_storage(**opts)

    def test_first_a_view_of_a_set_stores_the_fixed_cameras(self):
        s = self._a(0)
        self.assertTrue(s.store_fixed)
        self.assertIsNone(s.fixed_skip_reason)
        self.assertTrue(s.store_gripper)

    def test_later_a_views_drop_the_fixed_cameras_with_a_reason(self):
        s = self._a(1)
        self.assertFalse(s.store_fixed)
        self.assertIn("redundant_static_scene", s.fixed_skip_reason)
        self.assertIn("set 3", s.fixed_skip_reason)
        # The wrist camera is the whole point of the extra A views.
        self.assertTrue(s.store_gripper)

    def test_a_quota_above_one_keeps_that_many_views(self):
        self.assertTrue(self._a(1, a_fixed_cam_views_per_set=2).store_fixed)
        self.assertFalse(self._a(2, a_fixed_cam_views_per_set=2).store_fixed)

    def test_zero_quota_disables_the_reduction(self):
        self.assertTrue(self._a(99, a_fixed_cam_views_per_set=0).store_fixed)

    def test_a_capture_without_a_set_keeps_every_camera(self):
        # Nothing to call a repeat of, so nothing may be dropped as one.
        self.assertTrue(self._a(99, set_index=None).store_fixed)

    def test_b_drops_the_gripper_camera_and_keeps_the_fixed_ones(self):
        s = self._b()
        self.assertFalse(s.store_gripper)
        self.assertEqual(s.gripper_skip_reason, "gripped_cube_occludes_wrist_camera")
        self.assertTrue(s.store_fixed)

    def test_b_never_applies_the_a_side_quota(self):
        # B poses move the cube, so a B capture is never a repeat of an earlier one.
        s = self._b(fixed_views_already_stored=99)
        self.assertTrue(s.store_fixed)

    def test_b_gripper_camera_can_be_kept_on_request(self):
        s = self._b(b_save_gripper_cam=True)
        self.assertTrue(s.store_gripper)
        self.assertIsNone(s.gripper_skip_reason)


if __name__ == "__main__":
    unittest.main()
