import os
import sys
import unittest

import numpy as np

SIM_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
REPO_DIR = os.path.dirname(SIM_DIR)
sys.path.insert(0, SIM_DIR)
sys.path.insert(0, REPO_DIR)

from configs import ALL_FACTORIAL
from core.production_prior import (
    adaptive_gate_threshold,
    align_and_blend_set_priors,
    blend_rigid_transforms,
    robust_weighted_se3_average,
)
from core.se3 import inv_T, rand_se3
from core.scene import SimScene


class ProductionPriorParityTest(unittest.TestCase):
    def test_average_matches_step3(self):
        import Step3_calibration as step3

        rng = np.random.default_rng(12)
        poses = [rand_se3(rng, 0.1, 15.0) for _ in range(8)]
        weights = np.arange(1, 9, dtype=float)
        expected, expected_stats = step3.robust_weighted_se3_average(
            poses, weights, return_stats=True)
        actual, actual_stats = robust_weighted_se3_average(
            poses, weights, return_stats=True)
        np.testing.assert_allclose(actual, expected, atol=1e-12)
        self.assertEqual(actual_stats["num_inliers"], expected_stats["num_inliers"])

    def test_alignment_matches_step3_direction_and_values(self):
        import Step3_calibration as step3

        rng = np.random.default_rng(3)
        delta = rand_se3(rng, 0.02, 4.0)
        raw = {s: rand_se3(rng, 0.2, 30.0) for s in range(5)}
        visual = {s: raw[s] @ delta for s in raw}
        support = {s: s + 1 for s in raw}

        expected_delta, expected_corrected, _ = step3.estimate_set_cube_prior_alignment(
            raw, visual, {"per_set": {str(s): {"support": support[s]} for s in raw}})
        actual = align_and_blend_set_priors(raw, visual, support)
        np.testing.assert_allclose(actual.delta, expected_delta, atol=1e-12)
        for s in raw:
            np.testing.assert_allclose(actual.corrected[s], expected_corrected[s], atol=1e-12)
            np.testing.assert_allclose(inv_T(actual.corrected[s]) @ visual[s], np.eye(4), atol=1e-10)

    def test_blend_helper_matches_step3(self):
        """Step3 still blends elsewhere (gripper board alignment), so the
        helper must stay in parity even though set priors no longer use it."""
        import Step3_calibration as step3

        A = np.eye(4)
        B = np.eye(4)
        B[0, 3] = 0.01
        np.testing.assert_allclose(
            blend_rigid_transforms(A, B, 0.25),
            step3.blend_rigid_transforms(A, B, 0.25), atol=1e-12)

    def test_accepted_set_takes_corrected_prior_unchanged(self):
        """A set that clears the gate is trusted fully."""
        rng = np.random.default_rng(21)
        delta = rand_se3(rng, 0.01, 2.0)
        raw, visual = {}, {}
        for s in range(8):
            base = rand_se3(rng, 0.4, 40.0)
            raw[s] = base
            visual[s] = base @ delta
        result = align_and_blend_set_priors(raw, visual, gate_mode="fixed")
        for s in raw:
            self.assertTrue(result.diagnostics["per_set"][str(s)]["prior_accepted"])
            np.testing.assert_allclose(
                result.anchors[s], result.corrected[s], atol=1e-12)

    def test_factorial_contains_all_14_valid_cells(self):
        self.assertEqual(len(ALL_FACTORIAL), 14)
        self.assertEqual(len({cfg.name for cfg in ALL_FACTORIAL}), 14)
        for cfg in ALL_FACTORIAL:
            cfg.validate()
            if cfg.markers == ("board",):
                self.assertEqual(cfg.fk, "none")

    def test_systematic_fk_generation_has_step3_common_right_delta(self):
        scene = SimScene(seed=7, n_sets=4, n_events_per_set=2,
                         sigma_px=0.0, fk_sys_deg=12.0)
        deltas = [inv_T(scene.fk_cube[s]) @ scene.bTo[s] for s in scene.sets]
        for delta in deltas[1:]:
            np.testing.assert_allclose(delta, deltas[0], atol=1e-12)


if __name__ == "__main__":
    unittest.main()


class AdaptiveGateTest(unittest.TestCase):
    """The gate distance measures deviation from the common delta, so its
    scale belongs to the data rather than to a fixed physical tolerance."""

    def _priors(self, rng, n_sets, spread_mm, outlier_mm=None):
        raw, visual = {}, {}
        common = rand_se3(rng, 0.02, 3.0)
        for s in range(n_sets):
            base = rand_se3(rng, 0.4, 40.0)
            jitter = np.eye(4)
            offset = outlier_mm if (outlier_mm and s == 0) else spread_mm
            direction = rng.normal(0.0, 1.0, 3)
            norm = np.linalg.norm(direction)
            if norm > 0:
                direction /= norm
            jitter[:3, 3] = direction * (offset / 1000.0)
            raw[s] = base
            visual[s] = base @ common @ jitter
        return raw, visual

    def test_default_mode_is_adaptive(self):
        rng = np.random.default_rng(7)
        raw, visual = self._priors(rng, 10, 3.0)
        default = align_and_blend_set_priors(raw, visual)
        self.assertEqual(default.diagnostics["gate_mode"], "adaptive")
        self.assertLess(default.diagnostics["gate_dt_mm"], 35.0)

    def test_adaptive_tightens_on_clean_data(self):
        rng = np.random.default_rng(11)
        raw, visual = self._priors(rng, 12, 2.0)
        result = align_and_blend_set_priors(raw, visual, gate_mode="adaptive")
        self.assertLess(result.diagnostics["gate_dt_mm"], 35.0)

    def test_adaptive_rejects_set_the_fixed_gate_accepts(self):
        rng = np.random.default_rng(5)
        raw, visual = self._priors(rng, 12, 1.0, outlier_mm=25.0)
        fixed = align_and_blend_set_priors(raw, visual, gate_mode="fixed")
        adaptive = align_and_blend_set_priors(raw, visual, gate_mode="adaptive")
        self.assertTrue(fixed.diagnostics["per_set"]["0"]["prior_accepted"])
        self.assertFalse(adaptive.diagnostics["per_set"]["0"]["prior_accepted"])

    def test_vision_scatter_floors_the_threshold(self):
        """A FK deviation below vision's own scatter must not reject a set."""
        rng = np.random.default_rng(13)
        raw, visual = self._priors(rng, 8, 2.0)
        scatter = {s: (4.0, 0.8) for s in raw}
        bare = align_and_blend_set_priors(raw, visual, gate_mode="adaptive")
        floored = align_and_blend_set_priors(
            raw, visual, gate_mode="adaptive", scatter_by_set=scatter)
        self.assertLess(bare.diagnostics["gate_dt_mm"], 4.0)
        self.assertGreaterEqual(floored.diagnostics["gate_dt_mm"], 4.0)
        self.assertEqual(floored.diagnostics["gate_floor_dt_mm"], 4.0)
        accepted = [v["prior_accepted"]
                    for v in floored.diagnostics["per_set"].values()]
        self.assertTrue(all(accepted))

    def test_too_few_sets_falls_back_to_fixed(self):
        rng = np.random.default_rng(17)
        raw, visual = self._priors(rng, 3, 2.0)
        result = align_and_blend_set_priors(raw, visual, gate_mode="adaptive")
        self.assertEqual(result.diagnostics["gate_dt_mm"], 35.0)
        self.assertEqual(result.diagnostics["gate_dr_deg"], 8.0)


class GateParityWithStep3Test(unittest.TestCase):
    """적응형 gate 기준선이 실제 파이프라인과 같은 값을 내는지 확인."""

    def _prepared(self, dts, drs, scatter_t, scatter_r):
        return {
            i: {
                "dt_mm": dt, "dr_deg": dr,
                "stability": {"translation_std_mm": st, "rotation_std_deg": sr},
            }
            for i, (dt, dr, st, sr) in enumerate(zip(dts, drs, scatter_t, scatter_r))
        }

    def test_threshold_matches_step3(self):
        import Step3_calibration as step3

        rng = np.random.default_rng(31)
        for trial in range(5):
            n = int(rng.integers(6, 15))
            dts = list(rng.gamma(2.0, 1.5, n))
            drs = list(rng.gamma(2.0, 0.4, n))
            st = list(rng.gamma(2.0, 0.6, n))
            sr = list(rng.gamma(2.0, 0.15, n))
            prepared = self._prepared(dts, drs, st, sr)

            gate_dt, gate_dr, info = step3.resolve_prior_gate(
                prepared, 35.0, 8.0, gate_mode="adaptive", gate_k=2.5)

            floor_t = float(np.median(st))
            floor_r = float(np.median(sr))
            mine_dt = adaptive_gate_threshold(dts, 2.5, floor_t, 5)
            mine_dr = adaptive_gate_threshold(drs, 2.5, floor_r, 5)

            self.assertAlmostEqual(gate_dt, mine_dt, places=12, msg=f"trial {trial}")
            self.assertAlmostEqual(gate_dr, mine_dr, places=12, msg=f"trial {trial}")
            self.assertEqual(info["applied"], "adaptive")

    def test_few_sets_fall_back_in_both(self):
        import Step3_calibration as step3

        prepared = self._prepared([1.0, 2.0, 3.0], [0.1, 0.2, 0.3],
                                  [0.5] * 3, [0.1] * 3)
        gate_dt, gate_dr, info = step3.resolve_prior_gate(prepared, 35.0, 8.0)
        self.assertEqual((gate_dt, gate_dr), (35.0, 8.0))
        self.assertEqual(info["applied"], "fixed")
        self.assertIsNone(adaptive_gate_threshold([1.0, 2.0, 3.0], 2.5, 0.5, 5))


if __name__ == "__main__":
    unittest.main()
