from __future__ import annotations

import numpy as np

from calibration_pipeline.reprojection import (
    FramePruneRefitOptions,
    PixelObs,
    PoseState,
    SolverOptions,
    project_points,
    select_frame_prune_subset,
)
from calibration_pipeline.table1 import run_frame_prune_refit


def _scene():
    K = {0: np.array([
        [700.0, 0.0, 320.0],
        [0.0, 700.0, 240.0],
        [0.0, 0.0, 1.0],
    ])}
    D = {0: np.zeros(5)}
    state = PoseState(
        cams={0: np.eye(4)},
        gtc=np.eye(4),
        board=np.eye(4),
        cubes={0: np.eye(4), 99: np.eye(4)},
    )
    state.board[2, 3] = 1.0
    state.cubes[0][0, 3], state.cubes[0][2, 3] = 0.15, 1.1
    state.cubes[99][0, 3], state.cubes[99][2, 3] = -0.15, 1.1
    object_points = np.array([
        [-0.04, -0.04, 0.0],
        [0.04, -0.04, 0.0],
        [0.04, 0.04, 0.0],
        [-0.04, 0.04, 0.0],
    ])
    observations = []
    for event in range(5):
        shift = np.array([20.0, 0.0]) if event == 4 else np.zeros(2)
        for marker, target in (("board", state.board), ("cube", state.cubes[0])):
            observations.append(PixelObs(
                marker=marker,
                cam=0,
                event=event,
                set_idx=None if marker == "board" else 0,
                object_points=object_points,
                image_points=project_points(target, object_points, K[0], D[0]) + shift,
            ))
    return observations, state, {}, K, D


def _options(**overrides):
    values = {
        "enabled": True,
        "mad_multiplier": 3.0,
        "minimum_rmse_px": 4.0,
        "maximum_fraction": 0.30,
        "minimum_observations_per_variable": 2,
        "minimum_relative_improvement": 1e-6,
    }
    values.update(overrides)
    return FramePruneRefitOptions(**values)


def test_prunes_board_and_cube_together_at_image_frame_level():
    observations, state, robot_T, K, D = _scene()
    kept, report = select_frame_prune_subset(
        observations,
        [("cam", 0), ("board", -1), ("cube", 0)],
        state,
        robot_T,
        K,
        D,
        2,
        SolverOptions(),
        _options(),
    )

    assert report["attempted"] is True
    assert report["n_pruned_frames"] == 1
    assert report["n_pruned_observations"] == 2
    assert report["pruned_frames"][0]["event_id"] == 4
    assert report["pruned_frames"][0]["markers"] == ["board", "cube"]
    assert all(observation.event != 4 for observation in kept)


def test_coverage_guard_blocks_removing_a_variables_only_observation():
    observations, state, robot_T, K, D = _scene()
    object_points = observations[0].object_points
    observations = [observation for observation in observations
                    if observation.event != 4]
    observations.append(PixelObs(
        marker="cube",
        cam=0,
        event=99,
        set_idx=99,
        object_points=object_points,
        image_points=project_points(
            state.cubes[99], object_points, K[0], D[0]) + np.array([20.0, 0.0]),
    ))
    kept, report = select_frame_prune_subset(
        observations,
        [("cube", 0), ("cube", 99)],
        state,
        robot_T,
        K,
        D,
        2,
        SolverOptions(),
        _options(),
    )

    assert len(kept) == len(observations)
    assert report["n_pruned_frames"] == 0
    assert report["blocked_frames"][0]["event_id"] == 99
    assert report["blocked_frames"][0]["reason"] == "coverage_guard:cube:99"


def test_refit_is_accepted_only_when_full_training_objective_improves():
    observations, first_state, robot_T, K, D = _scene()
    first_state.gtc[0, 3] = 2.0
    candidate = first_state.clone()
    candidate.gtc[0, 3] = 1.0
    first_diag = {"success": True, "nfev": 4, "elapsed_s": 0.01}
    refit_diag = {"success": True, "nfev": 3, "elapsed_s": 0.01}

    selected, diagnostics = run_frame_prune_refit(
        observations=observations,
        variable_keys_=[("cam", 0), ("board", -1), ("cube", 0)],
        first_state=first_state,
        first_diagnostics=first_diag,
        robot_T=robot_T,
        K_map=K,
        D_map=D,
        gripper_cam_idx=2,
        solver_options=SolverOptions(),
        prune_options=_options(),
        refit_solver=lambda kept: (candidate, refit_diag),
        full_objective_cost=lambda state: float(state.gtc[0, 3]),
    )

    assert selected is candidate
    assert diagnostics["frame_prune_refit"]["accepted"] is True
    assert diagnostics["frame_prune_refit"]["rolled_back"] is False
    assert diagnostics["effective_visual_residual_population"]["observations"] == 8


def test_refit_rolls_back_when_full_training_objective_does_not_improve():
    observations, first_state, robot_T, K, D = _scene()
    first_state.gtc[0, 3] = 1.0
    candidate = first_state.clone()
    candidate.gtc[0, 3] = 2.0
    first_diag = {"success": True, "nfev": 4, "elapsed_s": 0.01}
    refit_diag = {"success": True, "nfev": 3, "elapsed_s": 0.01}

    selected, diagnostics = run_frame_prune_refit(
        observations=observations,
        variable_keys_=[("cam", 0), ("board", -1), ("cube", 0)],
        first_state=first_state,
        first_diagnostics=first_diag,
        robot_T=robot_T,
        K_map=K,
        D_map=D,
        gripper_cam_idx=2,
        solver_options=SolverOptions(),
        prune_options=_options(),
        refit_solver=lambda kept: (candidate, refit_diag),
        full_objective_cost=lambda state: float(state.gtc[0, 3]),
    )

    assert selected is first_state
    report = diagnostics["frame_prune_refit"]
    assert report["accepted"] is False
    assert report["rolled_back"] is True
    assert report["rollback_reason"] == "insufficient_full_objective_improvement"
    assert diagnostics["effective_visual_residual_population"]["observations"] == 10
