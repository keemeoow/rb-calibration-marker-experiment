from pathlib import Path

from SOTA_Simulation.evaluate import evaluate_result, noiseless_pass
from SOTA_Simulation.methods import JointReprojectionReference
from SOTA_Simulation.sota_simulation import generate_case, load_config


ROOT = Path(__file__).resolve().parent
CONFIG = ROOT / "SOTA_Simulation" / "config.example.json"


def test_board_geometry_and_observation_coverage():
    config, path = load_config(CONFIG)
    config["simulation"]["number_of_events"] = 6
    case = generate_case(config, path)
    assert case.data.board_points.shape == (60, 3)
    assert set(case.data.cameras) == {"cam0", "cam1", "cam3"}
    assert all(
        any(observation.camera == camera for observation in case.data.observations)
        for camera in case.data.cameras
    )


def test_joint_reference_recovers_noiseless_ground_truth():
    config, path = load_config(CONFIG)
    config["simulation"]["number_of_events"] = 8
    case = generate_case(config, path)
    result = JointReprojectionReference(max_nfev=250).calibrate(case.data)
    report = evaluate_result(case, result)
    assert noiseless_pass(report, config["noiseless_tolerance"]), report


def test_tsai_combined_demo_recovers_both_camera_types():
    cv2 = __import__("cv2")
    assert hasattr(cv2, "calibrateHandEye")
    from SOTA_Simulation.sota_simulation import pose_error
    from SOTA_Simulation.tsai_combined_demo import (
        build_eye_in_hand_session,
        solve_eye_to_hand,
        solve_tsai,
    )

    config, path = load_config(CONFIG)
    config["simulation"]["number_of_events"] = 14
    config["simulation"]["pixel_noise_sigma"] = 0.0
    config["simulation"]["corner_dropout_probability"] = 0.0
    config["simulation"]["camera_event_dropout_probability"] = 0.0
    case = generate_case(config, path)

    wrist = build_eye_in_hand_session(14)
    wrist_estimate = solve_tsai(
        wrist["T_base_gripper"], wrist["T_wrist_board"], eye_to_hand=False
    )
    assert pose_error(wrist_estimate, wrist["T_gripper_wrist_truth"])[0] < 1e-5

    fixed_estimates, _ = solve_eye_to_hand(case)
    assert set(fixed_estimates) == set(case.truth.T_base_camera)
    for name, estimate in fixed_estimates.items():
        translation_mm, rotation_deg = pose_error(
            estimate, case.truth.T_base_camera[name]
        )
        assert translation_mm < 1e-5, name
        assert rotation_deg < 1e-6, name


def test_noise_sweep_keeps_trajectory_fixed_and_scales_only_visual_noise():
    from SOTA_Simulation.tsai_combined_demo import build_eye_in_hand_session
    from SOTA_Simulation.tsai_noise_sweep import run_sweep, trajectory_digest

    config, path = load_config(CONFIG)
    config["simulation"]["number_of_events"] = 14
    case = generate_case(config, path)
    wrist = build_eye_in_hand_session(14)
    before = trajectory_digest([
        case.data.T_base_gripper[event]
        for event in sorted(case.data.T_base_gripper)
    ])
    records, camera_names, _ = run_sweep(case, wrist, [0.0, 1.0], 2, 123)
    after = trajectory_digest([
        case.data.T_base_gripper[event]
        for event in sorted(case.data.T_base_gripper)
    ])

    assert before == after
    assert len(records) == 2 * 2 * len(camera_names)
    zero_errors = [record["translation_error_mm"] for record in records
                   if record["noise_sigma_mm"] == 0.0]
    assert max(zero_errors) < 1e-5
    noisy_rotation_errors = [record["rotation_error_deg"] for record in records
                             if record["noise_sigma_mm"] == 1.0]
    assert min(noisy_rotation_errors) > 0.0


def test_all_opencv_hand_eye_methods_recover_noiseless_case():
    from SOTA_Simulation.sota_simulation import pose_error
    from SOTA_Simulation.tsai_combined_demo import (
        HAND_EYE_METHODS,
        build_eye_in_hand_session,
        solve_hand_eye,
    )

    wrist = build_eye_in_hand_session(14)
    for method in HAND_EYE_METHODS:
        estimate = solve_hand_eye(
            wrist["T_base_gripper"], wrist["T_wrist_board"],
            eye_to_hand=False, method=method,
        )
        translation_mm, rotation_deg = pose_error(
            estimate, wrist["T_gripper_wrist_truth"]
        )
        assert translation_mm < 1e-5, method
        assert rotation_deg < 1e-6, method


def test_system_aggregate_uses_every_camera_once_per_trial():
    from SOTA_Simulation.tsai_combined_demo import build_eye_in_hand_session
    from SOTA_Simulation.tsai_noise_sweep import (
        aggregate_system_results,
        run_sweep,
    )

    config, path = load_config(CONFIG)
    case = generate_case(config, path)
    wrist = build_eye_in_hand_session(14)
    records, camera_names, _ = run_sweep(
        case, wrist, [0.0, 1.0], trials=2, seed=17, methods=("tsai",)
    )
    integrated, summary = aggregate_system_results(
        records, camera_names, [0.0, 1.0], ("tsai",)
    )
    assert len(integrated) == 4
    assert all(record["number_of_cameras"] == 4 for record in integrated)
    assert summary["tsai"]["0.0"]["macro_translation_error_mm_mean"] < 1e-5


def test_multicam_evaluation_has_disjoint_split_and_zero_noiseless_metrics():
    from SOTA_Simulation.opencv_multicam_evaluation import (
        load_wrist_camera,
        run_evaluation,
    )
    from SOTA_Simulation.tsai_combined_demo import build_eye_in_hand_session

    config, path = load_config(CONFIG)
    case = generate_case(config, path)
    wrist = build_eye_in_hand_session(14)
    records, train, heldout = run_evaluation(
        case, wrist, load_wrist_camera(path), [0.0], trials=1, seed=3,
        methods=("tsai",), heldout_events=(2, 5, 9, 12),
    )
    assert set(train).isdisjoint(heldout)
    assert len(train) == 10 and len(heldout) == 4
    result = records[0]
    assert result["camera_pose_translation_error_mm"] < 1e-4
    assert result["heldout_translation_error_mm"] < 1e-4
    assert result["heldout_reprojection_rmse_px"] < 1e-3
