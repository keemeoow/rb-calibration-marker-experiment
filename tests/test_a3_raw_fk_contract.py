"""A3 must stay independent of the image-fitted FK alignment artifact."""

import numpy as np

from calibration_pipeline.reprojection import PoseState
from calibration_pipeline.schema import (
    FK_ALIGNMENT_SHARED_ROWS,
    FK_FIXED_CONTRACT,
    MAIN_ABLATION_CONDITIONS,
    POSE_SOURCE_ALIGNED_FK_FIXED,
    POSE_SOURCE_FK_FIXED,
    RAW_FK_CUBE_CENTER_TO_OBJECT,
    VISION_ALIGNED_FK_FIXED_CONTRACT,
)
from calibration_pipeline.table1 import make_initial_state


def test_a3_declares_pure_raw_fk_hard_constraint():
    condition = next(
        item for item in MAIN_ABLATION_CONDITIONS if item.row == "A3")
    transform = np.asarray(RAW_FK_CUBE_CENTER_TO_OBJECT, dtype=np.float64)

    assert condition.fk_to_cube == POSE_SOURCE_FK_FIXED == "raw-FK-fixed"
    assert "A3" not in FK_ALIGNMENT_SHARED_ROWS
    assert FK_FIXED_CONTRACT[
        "image_fitted_fk_to_object_transform_used"] is False
    assert FK_FIXED_CONTRACT[
        "vision_degrees_of_freedom_in_the_fixed_cube_target"] == 0
    np.testing.assert_allclose(transform[:3, 3], 0.0)
    np.testing.assert_allclose(transform[:3, :3].T @ transform[:3, :3], np.eye(3))
    np.testing.assert_allclose(np.linalg.det(transform[:3, :3]), 1.0)


def test_a3_initial_state_uses_supplied_raw_fk_poses_exactly():
    identity = np.eye(4, dtype=np.float64)
    shared = PoseState(
        cams={0: identity.copy()},
        gtc=identity.copy(),
        board=identity.copy(),
        cubes={4: identity.copy()},
    )
    raw_fixed = identity.copy()
    raw_fixed[:3, 3] = [0.1, 0.2, 0.3]
    condition = next(
        item for item in MAIN_ABLATION_CONDITIONS if item.row == "A3")

    state, diagnostics = make_initial_state(condition, shared, {4: raw_fixed})

    np.testing.assert_array_equal(state.cubes[4], raw_fixed)
    assert diagnostics["T_base_cube_by_set_source"] == (
        "raw_FK_pose_with_preregistered_mechanical_frame_map")


def test_a5_declares_train_only_vision_aligned_hard_constraint():
    condition = next(
        item for item in MAIN_ABLATION_CONDITIONS if item.row == "A5")

    assert condition.fk_to_cube == POSE_SOURCE_ALIGNED_FK_FIXED
    assert POSE_SOURCE_ALIGNED_FK_FIXED == "vision-aligned-FK-fixed"
    assert condition.supplementary is True
    assert "A5" in FK_ALIGNMENT_SHARED_ROWS
    assert VISION_ALIGNED_FK_FIXED_CONTRACT["training_information_used"] is True
    assert VISION_ALIGNED_FK_FIXED_CONTRACT["heldout_information_used"] is False
    assert VISION_ALIGNED_FK_FIXED_CONTRACT["external_ground_truth_used"] is False


def test_a5_initial_state_uses_aligned_fk_poses_and_reports_true_source():
    identity = np.eye(4, dtype=np.float64)
    shared = PoseState(
        cams={0: identity.copy()},
        gtc=identity.copy(),
        board=identity.copy(),
        cubes={4: identity.copy()},
    )
    raw_fixed = identity.copy()
    raw_fixed[:3, 3] = [0.1, 0.2, 0.3]
    aligned_fixed = identity.copy()
    aligned_fixed[:3, 3] = [-0.2, 0.4, 0.7]
    condition = next(
        item for item in MAIN_ABLATION_CONDITIONS if item.row == "A5")

    state, diagnostics = make_initial_state(
        condition, shared, {4: raw_fixed}, {4: aligned_fixed})

    np.testing.assert_array_equal(state.cubes[4], aligned_fixed)
    assert not np.array_equal(state.cubes[4], raw_fixed)
    assert diagnostics["T_base_cube_by_set_source"] == (
        "train_only_board_free_vision_aligned_FK_pose")
