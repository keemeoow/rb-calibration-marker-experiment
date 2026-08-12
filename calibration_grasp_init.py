"""Seed the gripped-cube variables from observations, without touching held-out data.

A gripped cube is a different estimation object from a placed one.  Placed, it is
one static pose per set.  Gripped, it is bolted to the flange, so

    T_base_cube[e] = FK(q_e) @ T_gripper_cube[g]

and the only unknown is one constant per grasp.  That is what makes the gripped
block able to pose the FK question at all: FK either carries the cube (grasp
model) or it does not (a free pose per event).  A placement cannot ask this,
because its FK link is a single contact measurement taken once per set.

Both models need a starting point.  These helpers build one from per-observation
PnP on train observations only — no board pose, no held-out events, no
production calibration transforms.
"""
from __future__ import annotations

from collections import defaultdict
from typing import Dict, Mapping, Optional, Sequence, Tuple

import cv2
import numpy as np
from scipy.spatial.transform import Rotation

from apriltag_cube import inv_T
from calibration_reprojection_backend import PixelObs, PoseState

# PnP on a handful of coplanar corners is unstable; below this the observation is
# not a usable seed.  It only gates initialization, never the residual.
MIN_SEED_CORNERS = 6


def _solve_pnp(obs: PixelObs, K_map, D_map) -> Optional[np.ndarray]:
    obj = np.asarray(obs.object_points, dtype=np.float64).reshape(-1, 3)
    pix = np.asarray(obs.image_points, dtype=np.float64).reshape(-1, 2)
    if len(obj) < MIN_SEED_CORNERS or len(obj) != len(pix):
        return None
    ok, rvec, tvec = cv2.solvePnP(
        obj, pix, np.asarray(K_map[int(obs.cam)], dtype=np.float64),
        np.asarray(D_map[int(obs.cam)], dtype=np.float64),
        flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok:
        return None
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = cv2.Rodrigues(rvec)[0]
    T[:3, 3] = np.asarray(tvec, dtype=np.float64).reshape(3)
    return T


def _average_se3(transforms: Sequence[np.ndarray]) -> np.ndarray:
    """Chordal rotation mean plus translation mean — enough for a seed."""
    stacked = np.stack([np.asarray(T, dtype=np.float64) for T in transforms])
    out = np.eye(4, dtype=np.float64)
    out[:3, :3] = Rotation.from_matrix(stacked[:, :3, :3]).mean().as_matrix()
    out[:3, 3] = stacked[:, :3, 3].mean(axis=0)
    return out


def base_cube_poses_by_event(
    observations: Sequence[PixelObs],
    state: PoseState,
    robot_T: Mapping[int, np.ndarray],
    K_map: Mapping[int, np.ndarray],
    D_map: Mapping[int, np.ndarray],
    gripper_cam_idx: int,
) -> Dict[int, np.ndarray]:
    """T_base_cube per gripped event, from that event's own cube observations.

    Each observation gives ``T_cam_cube``; composing with the observing camera's
    current base pose puts it in base frame, and the per-event mean is the seed.
    Observations whose camera is not yet registered are skipped rather than
    guessed at.
    """
    per_event: Dict[int, list] = defaultdict(list)
    for obs in observations:
        if obs.marker == "board" or obs.grasp_idx is None:
            continue
        T_cam_cube = _solve_pnp(obs, K_map, D_map)
        if T_cam_cube is None:
            continue
        cam = int(obs.cam)
        if cam == int(gripper_cam_idx):
            event = int(obs.event)
            if event not in robot_T:
                continue
            T_base_cam = np.asarray(robot_T[event], dtype=np.float64) @ state.gtc
        else:
            if cam not in state.cams:
                continue
            T_base_cam = np.asarray(state.cams[cam], dtype=np.float64)
        per_event[int(obs.event)].append(T_base_cam @ T_cam_cube)
    return {event: _average_se3(mats) for event, mats in per_event.items() if mats}


def gripper_cube_by_grasp(
    observations: Sequence[PixelObs],
    state: PoseState,
    robot_T: Mapping[int, np.ndarray],
    K_map: Mapping[int, np.ndarray],
    D_map: Mapping[int, np.ndarray],
    gripper_cam_idx: int,
) -> Tuple[Dict[int, np.ndarray], Dict[int, int]]:
    """T_gripper_cube per grasp, averaged over that grasp's events.

    Returns ``(transforms, n_events_by_grasp)``.  The count is the caller's
    signal for whether a grasp is supported well enough to keep: a grasp seen at
    one event fixes nothing that the event's own free pose would not fix.
    """
    base_by_event = base_cube_poses_by_event(
        observations, state, robot_T, K_map, D_map, gripper_cam_idx)
    grasp_of_event: Dict[int, int] = {}
    for obs in observations:
        if obs.grasp_idx is None or obs.marker == "board":
            continue
        grasp_of_event[int(obs.event)] = int(obs.grasp_idx)

    per_grasp: Dict[int, list] = defaultdict(list)
    for event, T_base_cube in base_by_event.items():
        grasp = grasp_of_event.get(event)
        if grasp is None or event not in robot_T:
            continue
        T_base_gripper = np.asarray(robot_T[event], dtype=np.float64)
        per_grasp[grasp].append(inv_T(T_base_gripper) @ T_base_cube)

    transforms = {g: _average_se3(mats) for g, mats in per_grasp.items() if mats}
    counts = {g: len(mats) for g, mats in per_grasp.items()}
    return transforms, counts


def attach_gripped_variables(
    state: PoseState,
    observations: Sequence[PixelObs],
    robot_T: Mapping[int, np.ndarray],
    K_map: Mapping[int, np.ndarray],
    D_map: Mapping[int, np.ndarray],
    gripper_cam_idx: int,
    min_events_per_grasp: int = 2,
) -> dict:
    """Populate ``state.grasps`` and ``state.event_cubes``; return a diagnostic.

    Both models are seeded from the same observations so that the FK-on/off
    contrast differs only in which variables the solver is allowed to move, not
    in where it starts.  Grasps supported by fewer than ``min_events_per_grasp``
    events are reported and dropped: with one event the grasp constant and that
    event's free pose are the same six numbers, so keeping it would quietly turn
    the FK arm into the no-FK arm for that grasp.
    """
    grasps, counts = gripper_cube_by_grasp(
        observations, state, robot_T, K_map, D_map, gripper_cam_idx)
    event_cubes = base_cube_poses_by_event(
        observations, state, robot_T, K_map, D_map, gripper_cam_idx)

    dropped = {g: n for g, n in counts.items() if n < int(min_events_per_grasp)}
    for grasp in dropped:
        grasps.pop(grasp, None)

    state.grasps = {int(g): np.asarray(T, dtype=np.float64) for g, T in grasps.items()}
    state.event_cubes = {int(e): np.asarray(T, dtype=np.float64)
                         for e, T in event_cubes.items()}
    return {
        "n_grasps": len(state.grasps),
        "n_gripped_events": len(state.event_cubes),
        "events_by_grasp": {int(g): int(n) for g, n in sorted(counts.items())},
        "dropped_grasps_under_min_events": {int(g): int(n)
                                            for g, n in sorted(dropped.items())},
        "min_events_per_grasp": int(min_events_per_grasp),
        "seed_source": "per-observation PnP on the supplied (train) observations",
    }
