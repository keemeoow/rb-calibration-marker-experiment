from types import SimpleNamespace
import unittest

import cv2
import numpy as np

from CP_ablation_7row import (
    build_event_split,
    filter_observations,
    perturbed_state,
    reprojection_metrics,
    run_noise_free_sanity,
    synthetic_scene,
    validate_result_evaluation_contract,
)
from CP_ablation_schema import (
    EVALUATION_COMPARISON_CONTRACT,
    FK_ALIGNMENT_SHARED_ROWS,
    MAIN_ABLATION_CONDITIONS,
    SEQUENTIAL_STAGE_SPECS,
    validate_fk_alignment_artifact,
    validate_main_runner_contract,
)
from CP_ablation_multisplit import summarize_effects
from CP_synthetic_7row import ROWS as SYNTHETIC_ROWS, aggregate_pixel_trials
from CP_sensitivity_7row import select_balanced_events
from CP_metric_board_only import handeye_camera
from CP_common import _finalize_opt
from calibration_fk_cube_artifact import estimate_board_free_fk_cube_artifact
from calibration_path_evaluation import (
    build_common_path_evaluation_mask,
    evaluate_paths_with_common_mask,
    validate_common_path_evaluation_mask,
)
from calibration_reprojection_backend import (
    SE3Scaling,
    SolverOptions,
    pose_delta,
    solve_corner_reprojection,
    variable_keys,
)
from apriltag_cube import inv_T
from Step3_calibration import (
    refine_joint_cube_board_fk,
    refine_joint_cube_board_reprojection_fk,
)


class SevenRowAblationContractTest(unittest.TestCase):
    def test_solver_comparison_can_disable_production_adoption_guard(self):
        initial_camera = {0: np.eye(4)}
        optimized_transform = np.eye(4)
        optimized_transform[0, 3] = 0.1
        optimized_camera = {0: optimized_transform}
        objects = {0: np.eye(4)}
        optimizer = SimpleNamespace(
            success=True, x=np.array([2.0]), status=2,
            message="synthetic", optimality=0.0, nfev=2)
        residual = lambda x: np.asarray([float(x[0])])

        guarded_camera, _, guarded = _finalize_opt(
            initial_camera, objects, optimized_camera, objects,
            residual, np.array([1.0]), optimizer)
        forced_camera, _, forced = _finalize_opt(
            initial_camera, objects, optimized_camera, objects,
            residual, np.array([1.0]), optimizer, adoption_guard=False)

        self.assertFalse(guarded["accepted"])
        self.assertTrue(np.array_equal(guarded_camera[0], initial_camera[0]))
        self.assertTrue(forced["accepted"])
        self.assertTrue(forced["forced_candidate_used"])
        self.assertTrue(np.array_equal(forced_camera[0], optimized_camera[0]))

    def test_metric_eye_on_base_handeye_transform_convention(self):
        observations, truth, robot_T, _, _, _ = synthetic_scene()
        T_base_cam = truth.cams[0]
        T_gripper_board = np.eye(4)
        T_gripper_board[:3, 3] = np.array([0.03, -0.02, 0.12])
        metric_observations = []
        for event, T_base_gripper in sorted(robot_T.items()):
            metric_observations.append({
                "camera": 1,
                "event": event,
                "T_cam_board_pnp": (
                    inv_T(T_base_cam) @ T_base_gripper @ T_gripper_board),
            })
        estimated = handeye_camera(
            cv2.CALIB_HAND_EYE_PARK, metric_observations, robot_T, camera=1)
        dt, dr = pose_delta(T_base_cam, estimated)
        self.assertLess(dt, 1e-3)
        self.assertLess(dr, 1e-5)

    def test_sensitivity_event_subsets_are_balanced_and_nested(self):
        events = {
            set_index: list(range(100 * set_index, 100 * set_index + 12))
            for set_index in (0, 2, 6, 9, 12)
        }
        small = select_balanced_events(events, tuple(events), budget=5, seed=17)
        medium = select_balanced_events(events, tuple(events), budget=20, seed=17)
        large = select_balanced_events(events, tuple(events), budget=40, seed=17)
        self.assertEqual(len(small), 5)
        self.assertEqual(len(medium), 20)
        self.assertEqual(len(large), 40)
        self.assertTrue(set(small).issubset(medium))
        self.assertTrue(set(medium).issubset(large))
        for selected, per_set in ((small, 1), (medium, 4), (large, 8)):
            for set_index in events:
                self.assertEqual(
                    sum(event // 100 == set_index for event in selected), per_set)

    def test_canonical_solver_defaults_match_train_only_selection(self):
        options = SolverOptions()
        self.assertEqual(options.loss, "soft_l1")
        self.assertEqual(options.f_scale_px, 2.0)
        self.assertEqual(options.x_scale_mode, "jac")
        self.assertEqual(options.scaling, SE3Scaling(1.0, 1.0))

    def test_schema_factorial_and_freeze_contract(self):
        validate_main_runner_contract()
        rows = {condition.row: condition for condition in MAIN_ABLATION_CONDITIONS}
        self.assertEqual(rows["B2"].unified, "U")
        self.assertEqual(rows["B3"].unified, "U")
        self.assertEqual(FK_ALIGNMENT_SHARED_ROWS, {"B1", "A3", "B2"})
        self.assertEqual(set(SEQUENTIAL_STAGE_SPECS), {"A0", "A1", "B1"})
        for spec in SEQUENTIAL_STAGE_SPECS.values():
            self.assertEqual(spec.stage2_free, ("T_base_Ci",))
        self.assertEqual(
            EVALUATION_COMPARISON_CONTRACT["B2_to_A3"]["components"][0],
            "heldout_reprojection.cube")
        self.assertIsNone(
            EVALUATION_COMPARISON_CONTRACT["B3_to_A3"]["causal_interpretation"])

    def test_reprojection_metrics_include_per_camera_components(self):
        observations, truth, robot_T, K, D, gripper = synthetic_scene()
        metrics = reprojection_metrics(
            observations, truth, robot_T, K, D, gripper)
        for camera in (*sorted(truth.cams), gripper):
            self.assertIn(f"cam_{camera}", metrics)
            self.assertIn(f"cube_cam_{camera}", metrics)
            self.assertGreater(metrics[f"cam_{camera}"]["n_corners"], 0)

    def test_common_path_mask_is_output_independent_and_ungated(self):
        observations, truth, robot_T, K, D, gripper = synthetic_scene()
        mask = build_common_path_evaluation_mask(
            observations=observations,
            fixed_camera_ids=sorted(truth.cams),
            gripper_cam_idx=gripper,
            K_map=K,
            D_map=D,
            set_filter=sorted(truth.cubes),
        )
        validate_common_path_evaluation_mask(mask)
        nominal = evaluate_paths_with_common_mask(
            observations, truth.cams, truth.gtc, robot_T, gripper, K, D, mask)

        bad_cameras = {camera: transform.copy()
                       for camera, transform in truth.cams.items()}
        bad_cameras[min(bad_cameras)][:3, 3] += np.array([0.5, -0.4, 0.3])
        perturbed = evaluate_paths_with_common_mask(
            observations, bad_cameras, truth.gtc, robot_T, gripper, K, D, mask)

        self.assertEqual(nominal["evaluation_mask_sha256"],
                         perturbed["evaluation_mask_sha256"])
        self.assertEqual(nominal["n_cross_pairs"], perturbed["n_cross_pairs"])
        self.assertEqual(nominal["n_e2e_units"], perturbed["n_e2e_units"])
        self.assertEqual(perturbed["n_output_rejected"], 0)
        self.assertIsNone(perturbed["output_dependent_pose_gate"])
        self.assertFalse(perturbed["model_dependent_gating"])
        self.assertGreater(perturbed["e_cross_translation_rmse_mm"], 30.0)
        self.assertGreater(perturbed["e_e2e_translation_rmse_mm"], 30.0)

        result = {
            "protocol": {
                "model_dependent_test_gating": False,
                "model_independent_path_evaluation_mask": mask,
            },
            "rows": {
                "A1": {
                    "condition": {"target_set": "cube+board"},
                    "path_evaluation_mask_sha256": mask["evaluation_mask_sha256"],
                    "runs": [{"heldout_path_metrics": perturbed}],
                },
            },
        }
        validate_result_evaluation_contract(result)
        perturbed["n_cross_pairs"] -= 1
        with self.assertRaises(ValueError):
            validate_result_evaluation_contract(result)
        perturbed["n_cross_pairs"] += 1

        tampered = dict(mask)
        tampered["output_dependent_pose_gate"] = {
            "translation_mm": 30.0, "rotation_deg": 10.0}
        with self.assertRaises(ValueError):
            validate_common_path_evaluation_mask(tampered)

    def test_event_split_has_no_event_leakage(self):
        observations, _, _, _, _, gripper = synthetic_scene()
        split = build_event_split(
            observations, gripper, fraction=0.2, seed=7,
            min_train_eih_cube_events=3,
        )
        train = set(split["train_events"])
        test = set(split["test_events"])
        self.assertTrue(train)
        self.assertTrue(test)
        self.assertFalse(train & test)
        for info in split["per_set"].values():
            self.assertGreaterEqual(len(info["train_eih_cube_events"]), 3)

    def test_multisplit_effects_use_declared_common_components(self):
        records = []
        row_offsets = {row: index for index, row in enumerate(
            ("A0", "A1", "A2", "A3", "B1", "B2", "B3"))}
        for split_seed in (11, 12):
            for row, offset in row_offsets.items():
                records.append({
                    "split_seed": split_seed,
                    "row": row,
                    "n_registered_cams": 2 + int(row not in {"A0", "B3"}),
                    "overall_reprojection_px_init_mean": 10.0 + offset,
                    "cube_reprojection_px_init_mean": 20.0 + offset,
                    "board_reprojection_px_init_mean": 30.0 + offset,
                    "e_e2e_translation_mm_init_mean": 40.0 + offset,
                    "e_cross_translation_mm_init_mean": 50.0 + offset,
                })
        effects = {(item["contrast"], item["component"]): item
                   for item in summarize_effects(records)}
        self.assertIn(("B2_to_A3", "heldout_reprojection.cube"), effects)
        self.assertNotIn(("B2_to_A3", "heldout_reprojection.overall"), effects)
        self.assertEqual(
            effects[("B2_to_A3", "heldout_reprojection.cube")]["delta_mean"],
            row_offsets["A3"] - row_offsets["B2"])
        self.assertIsNone(
            effects[("B3_to_A3", "heldout_reprojection.board")]
            ["causal_interpretation"])

    def test_synthetic_summary_withholds_nonconverged_headline_value(self):
        def run(converged):
            return {
                "converged": converged,
                "calibration_gt": {
                    "translation_rmse_mm": 12.0,
                    "rotation_rmse_deg": 1.2,
                },
            }
        trials = []
        for trial_index in range(2):
            trials.append({
                "rows": {
                    row: run(not (row == "A2" and trial_index == 1))
                    for row in SYNTHETIC_ROWS
                },
            })
        summary = {row["row"]: row for row in aggregate_pixel_trials(trials)}
        self.assertEqual(summary["A2"]["status"], "mixed")
        self.assertFalse(summary["A2"]["headline_metric_available"])
        self.assertIsNone(summary["A2"]["translation_rmse_mm_mean"])
        self.assertEqual(
            summary["A2"]["diagnostic_all_runs_translation_rmse_mm_mean"], 12.0)
        self.assertEqual(summary["A3"]["status"], "converged")
        self.assertEqual(summary["A3"]["translation_rmse_mm_mean"], 12.0)

    def test_board_free_fk_artifact_recovers_synthetic_handeye(self):
        observations, truth, robot_T, K, D, gripper = synthetic_scene()
        eih_cube = [obs for obs in observations
                    if obs.marker == "cube" and obs.cam == gripper]
        aligned, gtc, artifact = estimate_board_free_fk_cube_artifact(
            observations=eih_cube,
            raw_fk_by_set=truth.cubes,
            robot_T=robot_T,
            K_map=K,
            D_map=D,
            gripper_cam_idx=gripper,
            training_set_ids=sorted(truth.cubes),
            options=SolverOptions(max_nfev=300),
            num_inits=2,
        )
        validate_fk_alignment_artifact(artifact)
        self.assertFalse(artifact["board_information_used"])
        self.assertEqual(artifact["board_observation_count"], 0)
        self.assertFalse(artifact["heldout_information_used"])
        self.assertTrue(all(run["success"] for run in artifact["runs"]))
        self.assertTrue(all(not run["jacobian"]["rank_deficient"]
                            for run in artifact["runs"]))
        dt, dr = pose_delta(gtc, truth.gtc)
        self.assertLess(dt, 1e-3)
        self.assertLess(dr, 1e-5)
        for set_index in truth.cubes:
            dt, dr = pose_delta(aligned[set_index], truth.cubes[set_index])
            self.assertLess(dt, 1e-3)
            self.assertLess(dr, 1e-5)
        _, _, repeated = estimate_board_free_fk_cube_artifact(
            observations=eih_cube,
            raw_fk_by_set=truth.cubes,
            robot_T=robot_T,
            K_map=K,
            D_map=D,
            gripper_cam_idx=gripper,
            training_set_ids=sorted(truth.cubes),
            options=SolverOptions(max_nfev=300),
            num_inits=2,
        )
        self.assertEqual(artifact, repeated)

    def test_board_free_fk_artifact_validation_rejects_board_leak(self):
        observations, truth, robot_T, K, D, gripper = synthetic_scene()
        eih_cube = [obs for obs in observations
                    if obs.marker == "cube" and obs.cam == gripper]
        _, _, artifact = estimate_board_free_fk_cube_artifact(
            eih_cube, truth.cubes, robot_T, K, D, gripper,
            sorted(truth.cubes), options=SolverOptions(max_nfev=300),
            num_inits=1)
        artifact["board_information_used"] = True
        with self.assertRaises(ValueError):
            validate_fk_alignment_artifact(artifact)

    def test_noise_free_seq_equals_unified_and_truth(self):
        args = SimpleNamespace(
            init_translation_mm=5.0,
            init_rotation_deg=1.0,
            max_nfev=300,
            tol=1e-8,
        )
        result = run_noise_free_sanity(args)
        self.assertTrue(result["passed"])
        self.assertTrue(all(item["passed"] for item in result["pairs"].values()))
        self.assertTrue(all(item["passed"] for item in result["truth_recovery"].values()))

    def test_runner_and_step_e_corner_entrypoints_are_equivalent(self):
        observations, truth, robot_T, K, D, gripper = synthetic_scene()
        condition = {c.row: c for c in MAIN_ABLATION_CONDITIONS}["A3"]
        initial = perturbed_state(truth)
        initial.cubes = {s: T.copy() for s, T in truth.cubes.items()}
        relevant = filter_observations(
            observations, condition, None, gripper, initial.cams)
        options = SolverOptions(
            loss="soft_l1", f_scale_px=2.0, max_nfev=1000,
            xtol=1e-12, ftol=1e-12, gtol=1e-12,
            scaling=SE3Scaling(), x_scale_mode="jac",
        )
        direct, direct_diag = solve_corner_reprojection(
            relevant,
            variable_keys(("T_base_Ci", "T_gripper_cam", "T_base_board"), initial),
            initial, robot_T, K, D, gripper, options=options, seed=0)
        step_cams, step_gtc, step_diag = refine_joint_cube_board_reprojection_fk(
            corner_observations=relevant,
            robot_T=robot_T,
            K_map=K,
            D_map=D,
            fixed_cam_ids=sorted(initial.cams),
            gripper_cam_idx=gripper,
            T_base_Ci=initial.cams,
            T_gTc=initial.gtc,
            T_base_board=initial.board,
            fk_cube_by_set=initial.cubes,
            max_nfev=1000,
            tol=1e-12,
            max_delta_trans_mm=1000.0,
            max_delta_rot_deg=180.0,
        )
        self.assertTrue(direct_diag["success"])
        self.assertTrue(step_diag["adopted"])
        self.assertEqual(direct_diag["solver_options"]["loss"], "soft_l1")
        self.assertEqual(direct_diag["solver_options"]["scipy_x_scale"], "jac")
        self.assertTrue(direct_diag["common_scaled_jacobian"]["weakest_directions"])
        self.assertAlmostEqual(
            direct_diag["train_reprojection_rmse_px"],
            step_diag["train_reprojection_rmse_px"], places=12)
        for camera in sorted(direct.cams):
            dt, dr = pose_delta(direct.cams[camera], step_cams[camera])
            self.assertLess(dt, 1e-8)
            self.assertLess(dr, 1e-8)
        dt, dr = pose_delta(direct.gtc, step_gtc)
        self.assertLess(dt, 1e-8)
        self.assertLess(dr, 1e-8)

    def test_noise_free_legacy_pose_and_corner_solutions_are_close(self):
        observations, truth, robot_T, K, D, gripper = synthetic_scene()
        initial = perturbed_state(truth)
        initial.cubes = {s: T.copy() for s, T in truth.cubes.items()}
        pnp = {ci: {} for ci in (*sorted(truth.cams), gripper)}
        board = {ci: {} for ci in (*sorted(truth.cams), gripper)}
        event_sets = set()
        for obs in observations:
            T_base_cam = (robot_T[obs.event] @ truth.gtc
                          if obs.cam == gripper else truth.cams[obs.cam])
            target = truth.board if obs.marker == "board" else truth.cubes[obs.set_idx]
            T_cam_target = inv_T(T_base_cam) @ target
            if obs.marker == "cube":
                pnp[obs.cam][obs.event] = {
                    "T_C_O": T_cam_target,
                    "used_ids": [0, 1],
                    "err_mean": 0.0,
                }
            else:
                board[obs.cam][obs.event] = {
                    "T_cam_board": T_cam_target,
                    "reproj": 0.0,
                    "n_corners": len(obs.image_points),
                }
            event_sets.add((obs.event, obs.set_idx))
        meta = {"captures": [
            {"event_id": event, "set_index": set_index, "cube_gripped": False}
            for event, set_index in sorted(event_sets)
        ]}
        legacy_cams, legacy_gtc, legacy_diag = refine_joint_cube_board_fk(
            meta=meta,
            robot_T=robot_T,
            pnp_obs=pnp,
            board_obs_by_cam=board,
            fixed_cam_ids=sorted(truth.cams),
            gripper_cam_idx=gripper,
            T_base_Ci=initial.cams,
            T_gTc=initial.gtc,
            fk_cube_by_set=truth.cubes,
            max_nfev=1000,
            max_delta_trans_mm=1000.0,
            max_delta_rot_deg=180.0,
        )
        corner_cams, corner_gtc, corner_diag = refine_joint_cube_board_reprojection_fk(
            corner_observations=observations,
            robot_T=robot_T,
            K_map=K,
            D_map=D,
            fixed_cam_ids=sorted(truth.cams),
            gripper_cam_idx=gripper,
            T_base_Ci=initial.cams,
            T_gTc=initial.gtc,
            T_base_board=initial.board,
            fk_cube_by_set=truth.cubes,
            max_nfev=1000,
            tol=1e-12,
            max_delta_trans_mm=1000.0,
            max_delta_rot_deg=180.0,
        )
        self.assertTrue(legacy_diag["adopted"])
        self.assertTrue(corner_diag["adopted"])
        for camera in sorted(truth.cams):
            dt, dr = pose_delta(legacy_cams[camera], corner_cams[camera])
            self.assertLess(dt, 1e-3)
            self.assertLess(dr, 1e-5)
        dt, dr = pose_delta(legacy_gtc, corner_gtc)
        self.assertLess(dt, 1e-3)
        self.assertLess(dr, 1e-5)


if __name__ == "__main__":
    unittest.main()
