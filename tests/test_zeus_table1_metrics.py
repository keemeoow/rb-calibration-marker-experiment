from __future__ import annotations

import numpy as np
import pytest

from calibration_pipeline.apriltag_cube import inv_T
from calibration_pipeline.reprojection import PixelObs, PoseState, project_points
from zeus_gello_calibration import table1_zeus as table1


def _transform(x=0.0, y=0.0, z=0.0):
    transform = np.eye(4, dtype=np.float64)
    transform[:3, 3] = [x, y, z]
    return transform


def _two_camera_fixture():
    K = np.array([[500.0, 0.0, 320.0], [0.0, 500.0, 240.0], [0.0, 0.0, 1.0]])
    D = np.zeros(5, dtype=np.float64)
    object_points = np.array([
        [-0.04, -0.04, 0.00],
        [0.04, -0.04, 0.00],
        [0.04, 0.04, 0.00],
        [-0.04, 0.04, 0.00],
        [-0.04, -0.04, 0.08],
        [0.04, -0.04, 0.08],
        [0.04, 0.04, 0.08],
        [-0.04, 0.04, 0.08],
    ])
    T_base_cube = _transform(z=1.0)
    camera_poses = {0: _transform(), 1: _transform(x=0.20)}
    observations = []
    camera_cube_poses = {}
    for camera, T_base_camera in camera_poses.items():
        T_camera_cube = inv_T(T_base_camera) @ T_base_cube
        camera_cube_poses[camera] = T_camera_cube
        observations.append(PixelObs(
            marker="cube",
            cam=camera,
            event=1000,
            set_idx=0,
            object_points=object_points,
            image_points=project_points(T_camera_cube, object_points, K, D),
        ))
    state = PoseState(
        cams=camera_poses,
        gtc=np.eye(4),
        board=None,
        cubes={},
    )
    return observations, camera_cube_poses, state, T_base_cube, {0: K, 1: K}, {0: D, 1: D}


def test_cross_view_uses_bidirectional_source_only_transfer(monkeypatch) -> None:
    observations, camera_cube_poses, state, _cube, K_map, D_map = _two_camera_fixture()
    monkeypatch.setattr(
        table1,
        "solve_observed_pose",
        lambda observation, _K, _D: camera_cube_poses[int(observation.cam)],
    )

    result = table1.cross_view_transfer_stats(
        observations, state, {}, K_map, D_map)

    assert result["rmse_px"] == pytest.approx(0.0, abs=1e-10)
    assert result["n_observations"] == 2
    assert result["n_pairs"] == 1
    assert result["n_directions"] == 2
    assert result["by_pair_type"]["fixed_fixed"]["n_pairs"] == 1
    assert result["by_pair_type"]["fixed_gripper"]["n_pairs"] == 0


def test_cube_reprojection_uses_component_wise_rmse() -> None:
    observations, _camera_cube_poses, state, cube, K_map, D_map = _two_camera_fixture()
    shifted = [
        PixelObs(
            marker=observation.marker,
            cam=observation.cam,
            event=observation.event,
            set_idx=observation.set_idx,
            object_points=observation.object_points,
            image_points=observation.image_points + np.array([2.0, 0.0]),
        )
        for observation in observations
    ]

    result = table1.cube_reprojection_stats(
        shifted, {0: cube}, state, {}, K_map, D_map)

    assert result["rmse_px"] == pytest.approx(np.sqrt(2.0))
    assert result["n_corners"] == 16
    assert result["n_residual_components"] == 32


def test_current_zeus_data_contract_enables_a5_and_leaves_a3_pending() -> None:
    assert table1.ROWS["A3"]["available"] is False
    assert table1.ROWS["A3"]["fk"] == "mechanical_fixed"
    assert table1.ROWS["A5"]["fk"] == "corrected_fixed"
    assert table1.grasp_variable_families(table1.ROWS["A5"]) == []
    assert table1.grasp_variable_families(table1.ROWS["A4"]) == [
        "T_gripper_cube_by_grasp"
    ]


def test_external_gt_placeholder_is_empty_not_zero() -> None:
    pending = table1.external_gt_pending()

    assert pending["status"] == "pending"
    assert all(value is None for key, value in pending.items() if key != "status")
