"""Gripped-cube support in the corner-reprojection backend.

The gripped block is what lets the FK axis be asked structurally: with the cube
bolted to the gripper, either FK carries it (one constant per grasp) or it does
not (six free DoF per event).  These tests pin that behaviour and, just as
importantly, pin that adding it changed nothing for placement-only data.
"""
import unittest

import numpy as np
from scipy.spatial.transform import Rotation

from apriltag_cube import inv_T
from calibration_grasp_init import attach_gripped_variables
from calibration_reprojection_backend import (
    GRIPPED_TARGET_EVENT,
    GRIPPED_TARGET_GRASP,
    CornerReprojectionProblem,
    PixelObs,
    PoseState,
    freeze_manifest,
    project_points,
    variable_keys,
)

K = np.array([[600.0, 0.0, 320.0], [0.0, 600.0, 240.0], [0.0, 0.0, 1.0]])
D = np.zeros(5)
CUBE_PTS = np.array([[-0.03, -0.03, 0.0], [0.03, -0.03, 0.0],
                     [0.03, 0.03, 0.0], [-0.03, 0.03, 0.0],
                     [-0.03, 0.0, 0.02], [0.03, 0.0, 0.02]])


def _T(rx=0.0, ry=0.0, rz=0.0, t=(0.0, 0.0, 0.0)):
    out = np.eye(4)
    out[:3, :3] = Rotation.from_euler("xyz", [rx, ry, rz], degrees=True).as_matrix()
    out[:3, 3] = t
    return out


def _world():
    """One fixed camera, one wrist camera, one grasp, three gripped events."""
    cams = {0: _T(rz=10.0, t=(0.4, -0.3, 0.6))}
    gtc = _T(ry=5.0, t=(0.02, -0.05, 0.03))
    grasp = _T(rz=15.0, t=(0.0, 0.0, 0.12))
    robot_T = {
        0: _T(rz=0.0, t=(0.30, 0.10, 0.30)),
        1: _T(rz=25.0, t=(0.35, 0.18, 0.36)),
        2: _T(ry=15.0, rz=-20.0, t=(0.26, 0.05, 0.42)),
    }
    return cams, gtc, grasp, robot_T


def _observations(cams, gtc, grasp, robot_T, gripper=9):
    obs = []
    for event, T_bg in robot_T.items():
        target = T_bg @ grasp
        for cam in (0, gripper):
            T_base_cam = cams[0] if cam == 0 else T_bg @ gtc
            pix = project_points(inv_T(T_base_cam) @ target, CUBE_PTS, K, D)
            obs.append(PixelObs(marker="cube", cam=cam, event=event, set_idx=None,
                                object_points=CUBE_PTS, image_points=pix,
                                grasp_idx=0))
    return obs


class GrippedTargetModelTests(unittest.TestCase):
    def setUp(self):
        self.cams, self.gtc, self.grasp, self.robot_T = _world()
        self.gripper = 9
        self.obs = _observations(self.cams, self.gtc, self.grasp,
                                 self.robot_T, self.gripper)
        self.state = PoseState(
            cams=dict(self.cams), gtc=self.gtc.copy(), board=None, cubes={},
            grasps={0: self.grasp.copy()},
            event_cubes={e: T @ self.grasp for e, T in self.robot_T.items()},
        )

    def _problem(self, names, model):
        return CornerReprojectionProblem(
            self.obs, variable_keys(names, self.state), self.state,
            self.robot_T, {0: K, self.gripper: K}, {0: D, self.gripper: D},
            self.gripper, gripped_target=model)

    def test_grasp_model_reproduces_the_generating_geometry(self):
        p = self._problem(["T_gripper_cube_by_grasp"], GRIPPED_TARGET_GRASP)
        self.assertLess(np.max(np.abs(p.residual_vector(p.x0))), 1e-6)

    def test_event_model_reproduces_the_generating_geometry(self):
        p = self._problem(["T_base_cube_by_event"], GRIPPED_TARGET_EVENT)
        self.assertLess(np.max(np.abs(p.residual_vector(p.x0))), 1e-6)

    def test_fk_carries_the_cube_only_in_the_grasp_model(self):
        # Perturbing the single grasp constant must move every gripped event.
        # That coupling IS the FK information; the event model must not have it.
        grasp_p = self._problem(["T_gripper_cube_by_grasp"], GRIPPED_TARGET_GRASP)
        x = grasp_p.x0.copy()
        x[grasp_p.slices[("grasp", 0)]] = [0.0, 0.0, 0.0, 0.004, 0.0, 0.0]
        moved = grasp_p.residual_vector(x).reshape(-1, 2)
        rows_per_event = len(moved) // len(self.robot_T)
        for i in range(len(self.robot_T)):
            block = moved[i * rows_per_event:(i + 1) * rows_per_event]
            self.assertGreater(np.max(np.abs(block)), 1e-3)

        event_p = self._problem(["T_base_cube_by_event"], GRIPPED_TARGET_EVENT)
        x = event_p.x0.copy()
        x[event_p.slices[("cube_event", 0)]] = [0.0, 0.0, 0.0, 0.004, 0.0, 0.0]
        residual = event_p.residual_vector(x).reshape(-1, 2)
        # Only event 0's rows may react.
        self.assertGreater(np.max(np.abs(residual[:rows_per_event])), 1e-3)
        self.assertLess(np.max(np.abs(residual[rows_per_event:])), 1e-9)

    def test_grasp_model_is_the_cheaper_parameterization(self):
        grasp_p = self._problem(["T_gripper_cube_by_grasp"], GRIPPED_TARGET_GRASP)
        event_p = self._problem(["T_base_cube_by_event"], GRIPPED_TARGET_EVENT)
        self.assertEqual(grasp_p.n_params, 6)
        self.assertEqual(event_p.n_params, 6 * len(self.robot_T))

    def test_sparsity_matches_the_active_target_model(self):
        grasp_p = self._problem(["T_gripper_cube_by_grasp"], GRIPPED_TARGET_GRASP)
        self.assertEqual(grasp_p.jacobian_sparsity().shape,
                         (grasp_p.n_residuals, 6))
        self.assertTrue((grasp_p.jacobian_sparsity().toarray() == 1).all())

    def test_missing_grasp_state_fails_loudly(self):
        state = PoseState(cams=dict(self.cams), gtc=self.gtc, board=None, cubes={})
        with self.assertRaisesRegex(RuntimeError, "no grasps"):
            variable_keys(["T_gripper_cube_by_grasp"], state)

    def test_freeze_manifest_lists_the_gripped_variables(self):
        keys = variable_keys(["T_gripper_cube_by_grasp"], self.state)
        manifest = freeze_manifest(self.state, keys)
        self.assertIn("grasp:0", manifest["free"])
        self.assertIn("cube_event:0", manifest["frozen"])


class GrippedSeedingTests(unittest.TestCase):
    def test_seed_recovers_the_grasp_and_per_event_poses(self):
        cams, gtc, grasp, robot_T = _world()
        gripper = 9
        obs = _observations(cams, gtc, grasp, robot_T, gripper)
        state = PoseState(cams=dict(cams), gtc=gtc.copy(), board=None, cubes={})
        report = attach_gripped_variables(
            state, obs, robot_T, {0: K, gripper: K}, {0: D, gripper: D}, gripper)
        self.assertEqual(report["n_grasps"], 1)
        self.assertEqual(report["n_gripped_events"], len(robot_T))
        self.assertLess(np.max(np.abs(state.grasps[0] - grasp)), 1e-6)
        for event, T_bg in robot_T.items():
            self.assertLess(
                np.max(np.abs(state.event_cubes[event] - T_bg @ grasp)), 1e-6)

    def test_single_event_grasp_is_dropped_not_silently_degenerate(self):
        cams, gtc, grasp, robot_T = _world()
        gripper = 9
        one = {0: robot_T[0]}
        obs = _observations(cams, gtc, grasp, one, gripper)
        state = PoseState(cams=dict(cams), gtc=gtc.copy(), board=None, cubes={})
        report = attach_gripped_variables(
            state, obs, one, {0: K, gripper: K}, {0: D, gripper: D}, gripper)
        self.assertEqual(state.grasps, {})
        self.assertEqual(report["dropped_grasps_under_min_events"], {0: 1})


class PlacementRegressionTests(unittest.TestCase):
    """Placement-only data must behave exactly as it did before grasps existed."""

    def test_placement_observations_are_untouched_by_the_new_field(self):
        cams, gtc, _, robot_T = _world()
        gripper = 9
        cube = _T(rz=30.0, t=(0.30, 0.12, 0.05))
        obs = []
        for event, T_bg in robot_T.items():
            for cam in (0, gripper):
                T_base_cam = cams[0] if cam == 0 else T_bg @ gtc
                pix = project_points(inv_T(T_base_cam) @ cube, CUBE_PTS, K, D)
                obs.append(PixelObs(marker="cube", cam=cam, event=event, set_idx=3,
                                    object_points=CUBE_PTS, image_points=pix))
        state = PoseState(cams=dict(cams), gtc=gtc.copy(), board=None,
                          cubes={3: cube.copy()})
        keys = variable_keys(["T_base_cube_by_set"], state)
        self.assertEqual(keys, [("cube", 3)])
        problem = CornerReprojectionProblem(
            obs, keys, state, robot_T, {0: K, gripper: K}, {0: D, gripper: D},
            gripper)
        self.assertLess(np.max(np.abs(problem.residual_vector(problem.x0))), 1e-6)
        self.assertEqual(problem.n_params, 6)

    def test_default_pixelobs_has_no_grasp(self):
        obs = PixelObs(marker="cube", cam=0, event=0, set_idx=1,
                       object_points=CUBE_PTS, image_points=CUBE_PTS[:, :2])
        self.assertIsNone(obs.grasp_idx)


if __name__ == "__main__":
    unittest.main()
