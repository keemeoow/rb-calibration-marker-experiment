"""Observation-block weighting stays explicit and auditable."""

import numpy as np

from calibration_pipeline.reprojection import (
    CornerReprojectionProblem,
    PixelObs,
    PoseState,
    RESIDUAL_WEIGHT_EQUAL_OBSERVATION,
    RESIDUAL_WEIGHT_PER_CORNER,
    SolverOptions,
    project_points,
)


def _problem(weighting: str) -> CornerReprojectionProblem:
    identity = np.eye(4, dtype=np.float64)
    K = np.array(
        [[100.0, 0.0, 50.0], [0.0, 100.0, 40.0], [0.0, 0.0, 1.0]],
        dtype=np.float64,
    )
    distortion = np.zeros(5, dtype=np.float64)
    one_point = np.array([[0.0, 0.0, 1.0]], dtype=np.float64)
    four_points = np.array(
        [[-0.1, -0.1, 1.0], [0.1, -0.1, 1.0],
         [0.1, 0.1, 1.0], [-0.1, 0.1, 1.0]],
        dtype=np.float64,
    )
    observations = []
    for points in (one_point, four_points):
        predicted = project_points(identity, points, K, distortion)
        observations.append(PixelObs(
            marker="board",
            cam=0,
            event=0,
            set_idx=None,
            object_points=points,
            image_points=predicted - 1.0,
        ))
    state = PoseState(
        cams={0: identity.copy()},
        gtc=identity.copy(),
        board=identity.copy(),
        cubes={},
    )
    return CornerReprojectionProblem(
        observations=observations,
        variable_keys_=[("board", -1)],
        reference_state=state,
        robot_T={},
        K_map={0: K},
        D_map={0: distortion},
        gripper_cam_idx=2,
        residual_weighting=weighting,
    )


def test_per_corner_weighting_preserves_native_pixel_residuals():
    problem = _problem(RESIDUAL_WEIGHT_PER_CORNER)
    assert np.allclose(
        problem.residual_vector(problem.x0),
        problem.raw_residual_vector(problem.x0),
    )


def test_equal_observation_weighting_scales_by_inverse_sqrt_corner_count():
    problem = _problem(RESIDUAL_WEIGHT_EQUAL_OBSERVATION)
    raw = problem.raw_residual_vector(problem.x0)
    weighted = problem.residual_vector(problem.x0)
    first, second = problem.row_offsets
    assert np.allclose(weighted[first[0]:first[1]], raw[first[0]:first[1]])
    assert np.allclose(
        weighted[second[0]:second[1]],
        0.5 * raw[second[0]:second[1]],
    )


def test_solver_options_reject_unknown_residual_weighting():
    try:
        SolverOptions(residual_weighting="unknown").validate()
    except ValueError:
        pass
    else:
        raise AssertionError("unknown residual weighting was accepted")
