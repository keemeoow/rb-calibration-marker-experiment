import json
import tempfile
import unittest
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from CP_final_external_gt_eval import (
    build_pose_metrics,
    evaluate,
    load_manifest,
)
from calibration_fk_factor import (
    FKFactorSpec,
    diagonal_covariance,
    se3_log,
    solve_factorized_fk,
)
from calibration_reprojection_backend import PixelObs, PoseState, SolverOptions


def _transform(tx=0.0, ty=0.0, tz=1.0, rz_deg=0.0):
    output = np.eye(4)
    output[:3, :3] = Rotation.from_euler("z", rz_deg, degrees=True).as_matrix()
    output[:3, 3] = [tx, ty, tz]
    return output


class FKFactorContractTests(unittest.TestCase):
    def test_se3_log_uses_rad_and_metre_twist_order(self):
        transform = _transform(tx=0.01, ty=-0.02, tz=0.03, rz_deg=5.0)
        # _transform's default z=1 is not wanted for this local delta.
        transform[:3, 3] = [0.01, -0.02, 0.03]
        tangent = se3_log(transform)
        self.assertAlmostEqual(tangent[2], np.deg2rad(5.0), places=9)
        self.assertEqual(tangent.shape, (6,))
        self.assertTrue(np.all(np.isfinite(tangent)))

    def test_covariance_fk_factor_pulls_visual_cube_toward_fk(self):
        camera = np.eye(4)
        visual_truth = _transform(tx=0.02, tz=1.0)
        fk_target = _transform(tx=0.0, tz=1.0)
        points = np.asarray([
            [-0.03, -0.03, 0.0],
            [0.03, -0.03, 0.0],
            [0.03, 0.03, 0.0],
            [-0.03, 0.03, 0.0],
        ])
        K = np.asarray([[600.0, 0.0, 320.0], [0.0, 600.0, 240.0], [0.0, 0.0, 1.0]])
        image = np.column_stack([
            K[0, 0] * (points[:, 0] + visual_truth[0, 3]) / visual_truth[2, 3] + K[0, 2],
            K[1, 1] * points[:, 1] / visual_truth[2, 3] + K[1, 2],
        ])
        observation = PixelObs(
            marker="cube", cam=0, event=0, set_idx=0,
            object_points=points, image_points=image)
        initial = PoseState(cams={0: camera}, gtc=np.eye(4), board=None,
                            cubes={0: _transform(tx=0.01, tz=1.0)})
        common = dict(
            observations=[observation],
            variable_keys_=[("cube", 0)],
            reference_state=initial,
            robot_T={}, K_map={0: K}, D_map={0: np.zeros(5)},
            gripper_cam_idx=99,
            options=SolverOptions(loss="soft_l1", max_nfev=200),
            seed=0,
        )
        visual_state, visual_diag = solve_factorized_fk(
            **common, fk_spec=FKFactorSpec(mode="none"))
        covariance = diagonal_covariance(translation_std_mm=1.0, rotation_std_deg=0.1)
        anchored_state, anchored_diag = solve_factorized_fk(
            **common,
            fk_targets={0: fk_target}, fk_covariances={0: covariance},
            fk_spec=FKFactorSpec(mode="covariance", loss="linear"))
        visual_distance = abs(visual_state.cubes[0][0, 3] - fk_target[0, 3])
        anchored_distance = abs(anchored_state.cubes[0][0, 3] - fk_target[0, 3])
        self.assertLess(anchored_distance, visual_distance)
        self.assertEqual(visual_diag["objective_contract"]["visual_loss"],
                         anchored_diag["objective_contract"]["visual_loss"])
        self.assertEqual(visual_diag["n_fk_residuals"], 0)
        self.assertEqual(anchored_diag["n_fk_residuals"], 6)


class ExternalGTEvaluationTests(unittest.TestCase):
    def test_hierarchical_claim_gate_passes_clear_paired_improvement(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            sessions = []
            for session_index in range(3):
                gt_poses = {}
                ours_poses = {}
                baseline_poses = {}
                for pose_index in range(12):
                    pose_id = f"p{pose_index:02d}"
                    truth = _transform(tx=0.02 * pose_index, ty=0.01 * session_index, tz=0.8)
                    ours = truth.copy()
                    baseline = truth.copy()
                    ours[0, 3] += 0.001
                    baseline[0, 3] += 0.006
                    strata = ["center" if pose_index < 6 else "edge"]
                    gt_poses[pose_id] = {"T_base_cube": truth.tolist(), "strata": strata}
                    ours_poses[pose_id] = {"T_base_cube": ours.tolist()}
                    baseline_poses[pose_id] = {"T_base_cube": baseline.tolist()}
                sessions.append({
                    "session_id": f"s{session_index}",
                    "external_gt": {"artifact_schema": "base_cube_pose_predictions_v1", "poses": gt_poses},
                    "predictions": {
                        "A4": {"artifact_schema": "base_cube_pose_predictions_v1", "poses": ours_poses},
                        "A2": {"artifact_schema": "base_cube_pose_predictions_v1", "poses": baseline_poses},
                    },
                    "registered_cameras": {"A4": 3, "A2": 3},
                })
            manifest = {
                "artifact_schema": "external_gt_eval_manifest_v1",
                "methods": ["A4", "A2"],
                "ours_method": "A4",
                "blind_gt_used_for_training": False,
                "gt_independent_of_fk_factor": True,
                "add_mode": "none",
                "alpha": 0.05,
                "margins": {
                    "rotation_deg": 0.1,
                    "p95_tre_mm": 0.0,
                    "failure_rate": 0.01,
                    "worst_stratum_p95_tre_mm": 0.0,
                },
                "gt_uncertainty_floor": {"translation_mm": 0.5, "rotation_deg": 0.05},
                "bootstrap": {"repetitions": 400, "seed": 7},
                "sessions": sessions,
            }
            manifest_path = root / "manifest.json"
            manifest_path.write_text(json.dumps(manifest))
            loaded, session_data = load_manifest(str(manifest_path))
            metrics = build_pose_metrics(loaded, session_data)
            result = evaluate(loaded, session_data, metrics)
            self.assertTrue(result["overall_claim_pass"])
            comparison = result["comparisons_A4_minus_baseline"]["A2"]["mean_tre_mm"]
            self.assertLess(comparison["ci95_upper"], 0.0)
            self.assertAlmostEqual(comparison["estimate"], -5.0, places=6)


if __name__ == "__main__":
    unittest.main()
