import unittest

from calibration_pipeline.handeye_benchmark import (
    CLASSICAL_METHODS,
    ROBOT_WORLD_METHODS,
    run_synthetic_direction_contract,
)


class HandEyeBenchmarkTests(unittest.TestCase):
    def test_all_opencv_transform_directions_on_noise_free_scene(self):
        result = run_synthetic_direction_contract(seed=31)
        self.assertTrue(result["passed"])
        self.assertEqual(
            set(result["methods"]),
            set(CLASSICAL_METHODS) | set(ROBOT_WORLD_METHODS),
        )
        for method in result["methods"].values():
            self.assertLess(method["handeye_translation_error_mm"], 1e-6)
            self.assertLess(method["handeye_rotation_error_deg"], 1e-6)


if __name__ == "__main__":
    unittest.main()
