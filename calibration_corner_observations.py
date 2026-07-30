"""Shared construction of calibrated 2D corner observations."""

from __future__ import annotations

import os
from typing import List, Sequence, Tuple

import cv2
import numpy as np

import CP_common as cp
from calibration_reprojection_backend import PixelObs
from calibration_runtime_utils import get_capture_set_index
from charuco_utils import CharucoTarget
from config import CharucoBoardConfig


def load_board_pixel_observations(root: str, meta: dict,
                                  all_cam_ids: Sequence[int],
                                  gripper_cam_idx: int,
                                  image_scale: float = 1.0) -> List[PixelObs]:
    image_scale = float(image_scale)
    if not np.isfinite(image_scale) or image_scale <= 0.0:
        raise ValueError("image_scale must be finite and positive")
    detector = CharucoTarget(CharucoBoardConfig())
    output: List[PixelObs] = []
    allowed = {int(ci) for ci in all_cam_ids}
    for capture in meta.get("captures", []):
        event = int(capture.get("event_id", -1))
        set_index = get_capture_set_index(capture)
        if event < 0:
            continue
        for cam_raw, camera_info in capture.get("cams", {}).items():
            camera = int(cam_raw)
            if camera not in allowed or not camera_info.get("saved"):
                continue
            if int(camera_info.get("charuco_detect_n", 0) or 0) < 4:
                continue
            rgb_relative = camera_info.get("rgb_path", "")
            image = cv2.imread(os.path.join(root, rgb_relative)) if rgb_relative else None
            if image is None:
                continue
            if image_scale != 1.0:
                interpolation = (cv2.INTER_AREA if image_scale < 1.0
                                 else cv2.INTER_CUBIC)
                image = cv2.resize(
                    image, None, fx=image_scale, fy=image_scale,
                    interpolation=interpolation)
            corners, ids, count, _, _ = detector.detect(image)
            if corners is None or ids is None or count < 4:
                continue
            try:
                object_points, image_points = detector.board.matchImagePoints(corners, ids)
            except Exception:
                object_points, image_points = None, None
            if object_points is None or image_points is None or len(object_points) < 4:
                continue
            output.append(PixelObs(
                marker="board",
                cam=camera,
                event=event,
                set_idx=None if set_index is None else int(set_index),
                object_points=np.asarray(object_points, dtype=np.float64).reshape(-1, 3),
                # Preserve native-pixel units after scaled-raster detection.
                image_points=(
                    np.asarray(image_points, dtype=np.float64).reshape(-1, 2)
                    / image_scale),
            ))
    return output


def load_cube_pixel_observations(root: str, meta: dict, cube,
                                 K_map, D_map,
                                 all_cam_ids: Sequence[int],
                                 gripper_cam_idx: int,
                                 exclude_gripped: bool = True,
                                 fixed_min_corners: int = 8,
                                 image_scale: float = 1.0) -> Tuple[List[PixelObs], dict]:
    detected, reason = cp.detect_corner_observations(
        root=root,
        meta=meta,
        cube=cube,
        K_map=K_map,
        D_map=D_map,
        all_cam_ids=all_cam_ids,
        gripper_cam_idx=int(gripper_cam_idx),
        max_err_fixed=3.0,
        max_err_gripper=5.0,
        min_aspect_fixed=0.0,
        min_aspect_gripper=0.35,
        exclude_gripped=bool(exclude_gripped),
        image_scale=float(image_scale),
    )
    output = [PixelObs(
        marker="cube",
        cam=int(obs.cam),
        event=int(obs.event),
        set_idx=None if obs.set_idx is None else int(obs.set_idx),
        object_points=np.asarray(obs.object_points, dtype=np.float64).reshape(-1, 3),
        image_points=np.asarray(obs.image_points, dtype=np.float64).reshape(-1, 2),
    ) for obs in detected
        if (int(obs.cam) == int(gripper_cam_idx)
            or len(np.asarray(obs.object_points).reshape(-1, 3)) >= int(fixed_min_corners))]
    return output, reason


def load_cube_board_pixel_observations(root: str, meta: dict, cube,
                                       K_map, D_map,
                                       all_cam_ids: Sequence[int],
                                       gripper_cam_idx: int,
                                       exclude_gripped_cube: bool = True,
                                       fixed_cube_min_corners: int = 8,
                                       image_scale: float = 1.0):
    cube_observations, cube_reason = load_cube_pixel_observations(
        root, meta, cube, K_map, D_map, all_cam_ids, gripper_cam_idx,
        exclude_gripped=exclude_gripped_cube,
        fixed_min_corners=fixed_cube_min_corners,
        image_scale=float(image_scale),
    )
    board_observations = load_board_pixel_observations(
        root, meta, all_cam_ids, gripper_cam_idx,
        image_scale=float(image_scale))
    return cube_observations + board_observations, {
        "cube": cube_reason,
        "n_cube_observations": len(cube_observations),
        "n_board_observations": len(board_observations),
    }
