"""Cube-model regression tests, including the canonical session04 capture.

Run with the project environment that provides OpenCV contrib::
    /opt/anaconda3/bin/python -m pytest -q tests/test_apriltag_cube_model.py
"""

import copy
import json
from collections import Counter, defaultdict
from pathlib import Path

import cv2
import numpy as np

from calibration_pipeline.apriltag_cube import (
    AprilTagCubeModel,
    AprilTagCubeTarget,
    FACE_OUTWARD_NORMAL,
    inv_T,
    validate_cube_config,
)
from calibration_pipeline.config import CharucoBoardConfig, get_default_cube_config
from calibration_pipeline.cube_config import (
    cube_configs_equivalent,
    load_cube_config_from_meta,
)
from calibration_pipeline.runtime import (
    load_intrinsics_with_depth_scale,
    rotation_error_deg,
    validate_cube_model_against_captures,
)


ROOT = Path(__file__).resolve().parents[1]
SESSION_ROOT = ROOT / "data" / "session04" / "calib_train"
INTRINSICS_ROOT = ROOT / "intrinsics"


def _load_session04():
    with (SESSION_ROOT / "meta.json").open("r", encoding="utf-8") as stream:
        meta = json.load(stream)
    cfg, source = load_cube_config_from_meta(str(SESSION_ROOT))
    return meta, cfg, source


def _camera_maps(meta):
    K_map, D_map = {}, {}
    for camera in meta["cam_indices"]:
        K_map[camera], D_map[camera], _ = load_intrinsics_with_depth_scale(
            str(INTRINSICS_ROOT), camera
        )
    return K_map, D_map


def test_default_geometry_is_physically_self_consistent():
    cfg = get_default_cube_config()
    model = AprilTagCubeModel(cfg)

    expected = {
        0: ("+Z", 25.0, (0.0, -14.0, 29.5), 0.0),
        1: ("+Z", 25.0, (0.0, 14.0, 29.5), 0.0),
        2: ("+X", 51.0, (29.5, 0.0, -1.0), 90.0),
        3: ("+Y", 51.0, (0.0, 29.5, -1.0), 180.0),
        4: ("-X", 51.0, (-29.5, 0.0, -1.0), 270.0),
        5: ("-Y", 51.0, (0.0, -29.5, -1.0), 0.0),
    }
    expected_corners_mm = {
        0: [[12.5, -26.5, 29.5], [-12.5, -26.5, 29.5],
            [-12.5, -1.5, 29.5], [12.5, -1.5, 29.5]],
        1: [[12.5, 1.5, 29.5], [-12.5, 1.5, 29.5],
            [-12.5, 26.5, 29.5], [12.5, 26.5, 29.5]],
        2: [[29.5, 25.5, -26.5], [29.5, -25.5, -26.5],
            [29.5, -25.5, 24.5], [29.5, 25.5, 24.5]],
        3: [[-25.5, 29.5, -26.5], [25.5, 29.5, -26.5],
            [25.5, 29.5, 24.5], [-25.5, 29.5, 24.5]],
        4: [[-29.5, -25.5, -26.5], [-29.5, 25.5, -26.5],
            [-29.5, 25.5, 24.5], [-29.5, -25.5, 24.5]],
        5: [[25.5, -29.5, -26.5], [-25.5, -29.5, -26.5],
            [-25.5, -29.5, 24.5], [25.5, -29.5, 24.5]],
    }

    ok, problems = validate_cube_config(cfg)
    assert ok, problems

    assert set(cfg.marker_ids) == set(expected)

    for marker_id in cfg.marker_ids:
        pose = model.marker_pose_in_rig(marker_id)
        face = cfg.id_to_face[marker_id]
        corners = model.marker_corners_in_rig(marker_id)

        expected_face, expected_size_mm, expected_center_mm, expected_roll_deg = expected[marker_id]
        assert face == expected_face
        assert model.marker_size(marker_id) * 1000.0 == expected_size_mm
        assert np.allclose(pose[:3, 3] * 1000.0, expected_center_mm)
        assert cfg.face_roll_deg[marker_id] == expected_roll_deg
        assert np.allclose(corners * 1000.0, expected_corners_mm[marker_id])

        assert np.allclose(pose[:3, 3], cfg.marker_center_m[marker_id])
        assert np.allclose(pose[:3, 2], FACE_OUTWARD_NORMAL[face])
        assert np.allclose(
            np.linalg.norm(np.roll(corners, -1, axis=0) - corners, axis=1),
            model.marker_size(marker_id),
        )

        # Detector/model polygons are clockwise from outside the cube, hence
        # their polygon normal is opposite the outward marker normal.
        polygon_normal = np.cross(corners[1] - corners[0], corners[2] - corners[1])
        polygon_normal /= np.linalg.norm(polygon_normal)
        assert np.allclose(polygon_normal, -pose[:3, 2])


def test_id_overlap_is_only_an_error_for_the_same_dictionary():
    cfg = get_default_cube_config()
    same_dictionary_board = CharucoBoardConfig(dictionary_name=cfg.dictionary_name)
    ok, problems = validate_cube_config(cfg, charuco_cfg=same_dictionary_board)
    assert not ok
    assert any("[collision]" in problem for problem in problems)


def test_session04_frozen_config_matches_current_default():
    _, session_cfg, source = _load_session04()
    assert source == "meta"
    assert cube_configs_equivalent(session_cfg, get_default_cube_config())


def test_session04_detector_corners_are_clockwise_for_every_id():
    meta, cfg, _ = _load_session04()
    counts = Counter()

    for capture in meta["captures"]:
        for camera_info in capture.get("cams", {}).values():
            for marker in camera_info.get("markers") or []:
                marker_id = int(marker["marker_id"])
                corners = np.asarray(marker["corners_2d"], dtype=np.float64).reshape(4, 2)
                signed_area = 0.5 * np.sum(
                    corners[:, 0] * np.roll(corners[:, 1], -1)
                    - corners[:, 1] * np.roll(corners[:, 0], -1)
                )
                # Image y increases downward, so positive signed area is the
                # visually clockwise order returned by OpenCV ArUco/AprilTag.
                assert signed_area > 0.0
                counts[marker_id] += 1

    assert set(counts) == set(cfg.marker_ids)
    assert min(counts.values()) >= 20


def test_synthetic_projection_recovers_pose_for_every_marker_with_session04_intrinsics():
    _, cfg, _ = _load_session04()
    target = AprilTagCubeTarget(cfg)
    K, D, _ = load_intrinsics_with_depth_scale(str(INTRINSICS_ROOT), 0)

    # A tilted, front-facing marker pose avoids the special fronto-parallel case.
    normal_camera = np.array([0.16, -0.12, -0.98], dtype=np.float64)
    normal_camera /= np.linalg.norm(normal_camera)
    u_camera = np.cross(normal_camera, np.array([0.0, 1.0, 0.0]))
    u_camera /= np.linalg.norm(u_camera)
    v_camera = np.cross(normal_camera, u_camera)
    R_camera_marker = np.column_stack([u_camera, v_camera, normal_camera])

    for marker_id in cfg.marker_ids:
        T_camera_marker = np.eye(4, dtype=np.float64)
        T_camera_marker[:3, :3] = R_camera_marker
        T_camera_marker[:3, 3] = [0.035, -0.025, 0.65]
        expected = T_camera_marker @ inv_T(target.model.marker_pose_in_rig(marker_id))

        rvec, _ = cv2.Rodrigues(expected[:3, :3])
        image_points, _ = cv2.projectPoints(
            target.model.marker_corners_in_rig(marker_id),
            rvec,
            expected[:3, 3],
            K,
            D,
        )
        candidates = target.single_marker_ippe_candidates(
            marker_id, image_points.reshape(4, 2), K, D
        )
        assert len(candidates) == 2

        recovered = min(candidates, key=lambda candidate: candidate["rank"])["T_C_O"]
        translation_error_m = np.linalg.norm(recovered[:3, 3] - expected[:3, 3])
        rotation_error = rotation_error_deg(recovered[:3, :3], expected[:3, :3])
        assert translation_error_m < 1e-9
        assert rotation_error < 1e-3


def test_session04_multi_face_observations_confirm_face_rolls():
    meta, cfg, _ = _load_session04()
    K_map, D_map = _camera_maps(meta)

    current = validate_cube_model_against_captures(
        meta, K_map, D_map, AprilTagCubeTarget(cfg)
    )
    assert current["status"] == "ok"
    assert current["n_pairs"] >= 300
    assert current["median_deg"] < 5.0

    all_zero_cfg = copy.deepcopy(cfg)
    all_zero_cfg.face_roll_deg = {marker_id: 0.0 for marker_id in cfg.marker_ids}
    all_zero = validate_cube_model_against_captures(
        meta, K_map, D_map, AprilTagCubeTarget(all_zero_cfg)
    )
    assert all_zero["status"] == "fail"
    assert all_zero["median_deg"] > 45.0


def test_session04_fixed_camera_depth_supports_configured_marker_sizes():
    """Depth is a corroboration, not a replacement for a ruler measurement."""
    meta, cfg, _ = _load_session04()
    scale_by_id = defaultdict(list)

    for capture in meta["captures"]:
        for camera_text, camera_info in capture.get("cams", {}).items():
            if int(camera_text) == int(meta["gripper_cam_idx"]):
                continue  # cam2 has a known color/depth scale bias in this session.
            for marker in camera_info.get("markers") or []:
                scale = marker.get("depth_z_scale_pred_over_meas")
                if marker.get("depth_valid") and scale is not None and np.isfinite(scale):
                    scale_by_id[int(marker["marker_id"])].append(float(scale))

    assert set(scale_by_id) == set(cfg.marker_ids)
    for marker_id, scales in scale_by_id.items():
        assert len(scales) >= 20
        # PnP distance scales linearly with configured marker size. A median
        # predicted/measured depth ratio near 1 supports the physical size.
        assert abs(float(np.median(scales)) - 1.0) < 0.02, marker_id
