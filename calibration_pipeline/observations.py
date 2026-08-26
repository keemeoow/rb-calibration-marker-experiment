"""Shared construction of calibrated 2D corner observations."""

from __future__ import annotations

import hashlib
import json
import os
from collections import Counter, defaultdict
from typing import Dict, List, Optional, Sequence, Tuple

import cv2
import numpy as np

from calibration_pipeline.cube_detection import detect_corner_observations
from calibration_pipeline.reprojection import PixelObs
from calibration_pipeline.runtime import get_capture_set_index
from calibration_pipeline.charuco import CharucoTarget
from calibration_pipeline.config import CharucoBoardConfig


CUBE_OBSERVATION_POLICIES = ("core_multiface", "legacy")
POST_CAPTURE_MANIFEST_SCHEMA = "post_capture_observation_manifest_v1"
POST_CAPTURE_FILTER_POLICIES = ("standard", "strict")


def _file_sha256(path: str) -> str:
    digest = hashlib.sha256()
    with open(path, "rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _validate_manifest_file(entry: dict, label: str) -> None:
    path = os.path.abspath(str(entry.get("path", "")))
    expected = str(entry.get("sha256", ""))
    if not path or not expected or not os.path.isfile(path):
        raise ValueError(f"post-capture manifest {label} is unavailable: {path!r}")
    if _file_sha256(path) != expected:
        raise ValueError(f"post-capture manifest is stale: {label} changed: {path}")


def load_pixel_observations_from_manifest(
        manifest_path: str,
        policy: str = "standard",
        root: Optional[str] = None,
        intrinsics_dir: Optional[str] = None,
        allowed_event_ids: Optional[Sequence[int]] = None,
        validate_sources: bool = True) -> Tuple[List[PixelObs], dict]:
    """Load the immutable native-pixel corners selected by Step2b.

    Unlike the normal observation loader, this path never runs a detector.  It
    validates the source hashes stored by ``Step2b_capture_filter.py`` and
    reconstructs exactly the frozen corner population for ``standard`` or
    ``strict`` policy.
    """
    policy = str(policy)
    if policy not in POST_CAPTURE_FILTER_POLICIES:
        raise ValueError(
            f"policy must be one of {POST_CAPTURE_FILTER_POLICIES}")
    manifest_path = os.path.abspath(manifest_path)
    with open(manifest_path, "r", encoding="utf-8") as stream:
        payload = json.load(stream)
    if payload.get("schema") != POST_CAPTURE_MANIFEST_SCHEMA:
        raise ValueError("unknown post-capture observation manifest schema")
    if policy not in payload.get("policies", {}):
        raise ValueError(f"manifest does not define policy {policy!r}")

    source = payload.get("source", {})
    recorded_root = os.path.realpath(str(source.get("session_root", "")))
    if root is not None and recorded_root != os.path.realpath(root):
        raise ValueError(
            "post-capture manifest session root differs from --root_folder: "
            f"{recorded_root!r} != {os.path.realpath(root)!r}")
    if validate_sources:
        _validate_manifest_file(source.get("meta_json", {}), "meta.json")
        if intrinsics_dir is not None:
            for camera, entry in source.get("intrinsics", {}).items():
                current = os.path.abspath(os.path.join(
                    intrinsics_dir, f"cam{int(camera)}.npz"))
                if current != os.path.abspath(str(entry.get("path", ""))):
                    raise ValueError(
                        f"manifest cam{camera} intrinsic path differs from "
                        f"--intrinsics_dir: {current}")
                _validate_manifest_file(entry, f"cam{camera} intrinsics")

    allowed = (
        None if allowed_event_ids is None
        else {int(event) for event in allowed_event_ids}
    )
    selected_records = [
        record for record in payload.get("observations", [])
        if bool(record.get("selected_by_policy", {}).get(policy))
        and (allowed is None or int(record.get("event_id", -1)) in allowed)
    ]
    if validate_sources:
        validated_images = set()
        for record in selected_records:
            relative_path = str(record.get("image_path", ""))
            if relative_path in validated_images:
                continue
            image_entry = source.get("images", {}).get(relative_path)
            if not isinstance(image_entry, dict):
                raise ValueError(
                    f"manifest lacks image provenance for {relative_path!r}")
            _validate_manifest_file(image_entry, f"image {relative_path}")
            validated_images.add(relative_path)

    observations: List[PixelObs] = []
    seen = set()
    for record in selected_records:
        marker = str(record.get("target", ""))
        event = int(record.get("event_id", -1))
        camera = int(record.get("camera_id", -1))
        key = (marker, event, camera)
        if marker not in {"cube", "board"} or event < 0 or camera < 0:
            raise ValueError(f"invalid frozen observation identity: {key}")
        if key in seen:
            raise ValueError(f"duplicate frozen observation: {key}")
        seen.add(key)
        object_points = np.asarray(
            record.get("object_points", []), dtype=np.float64).reshape(-1, 3)
        image_points = np.asarray(
            record.get("image_points", []), dtype=np.float64).reshape(-1, 2)
        minimum = 4
        if (len(object_points) < minimum
                or len(object_points) != len(image_points)
                or not np.all(np.isfinite(object_points))
                or not np.all(np.isfinite(image_points))):
            raise ValueError(f"invalid frozen corners for observation {key}")
        set_index = record.get("set_idx")
        grasp_index = record.get("grasp_idx")
        observations.append(PixelObs(
            marker=marker,
            cam=camera,
            event=event,
            set_idx=None if set_index is None else int(set_index),
            object_points=object_points,
            image_points=image_points,
            grasp_idx=None if grasp_index is None else int(grasp_index),
        ))

    observations.sort(key=lambda item: (
        int(item.event), int(item.cam), str(item.marker)))
    counts = Counter(observation.marker for observation in observations)
    cube_records = [
        record for record in payload.get("observations", [])
        if record.get("target") == "cube"
        and (allowed is None or int(record.get("event_id", -1)) in allowed)
    ]
    diagnostics = {
        "status": "ok" if observations else "empty",
        "reason": "" if observations else "manifest policy selected 0 observations",
        "source": "post_capture_frozen_manifest",
        "manifest_path": manifest_path,
        "manifest_sha256": _file_sha256(manifest_path),
        "manifest_schema": POST_CAPTURE_MANIFEST_SCHEMA,
        "cube_config_source": str(source.get("cube_config_source", "unknown")),
        "observation_policy": policy,
        "quality_contract": payload["policies"][policy],
        "observation_quality_by_event_camera": cube_records,
        "final_observation_count": int(len(observations)),
        "n_cube_observations": int(counts.get("cube", 0)),
        "n_board_observations": int(counts.get("board", 0)),
        "source_hashes_validated": bool(validate_sources),
    }
    return observations, diagnostics


def _legacy_cube_observation_eligible(obs, gripper_cam_idx: int,
                                      fixed_min_corners: int) -> bool:
    return bool(
        int(obs.cam) == int(gripper_cam_idx)
        or len(np.asarray(obs.object_points).reshape(-1, 3))
        >= int(fixed_min_corners)
    )


def _core_multiface_eligible(obs) -> bool:
    return bool(
        int(obs.observed_face_count) >= 2
        and int(obs.noncoplanar_face_count) >= 2
        and not bool(obs.is_planar)
        and int(obs.positive_depth_candidate_count) >= 1
        and str(obs.quality_tier) == "nonplanar_multiface"
    )


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
                                 image_scale: float = 1.0,
                                 observation_policy: str = "core_multiface",
                                 ) -> Tuple[List[PixelObs], dict]:
    """Load cube corners using an explicit event-camera quality policy.

    ``core_multiface`` keeps only PnP-valid observations supported by at least
    two non-coplanar cube faces. ``legacy`` reproduces the old corner-count
    policy and is retained for planar/single-face comparison experiments.
    """
    observation_policy = str(observation_policy)
    if observation_policy not in CUBE_OBSERVATION_POLICIES:
        raise ValueError(
            f"observation_policy must be one of {CUBE_OBSERVATION_POLICIES}")
    detected, diagnostics = detect_corner_observations(
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
    legacy_selected = [
        obs for obs in detected
        if _legacy_cube_observation_eligible(
            obs, gripper_cam_idx, fixed_min_corners)
    ]
    core_selected = [
        obs for obs in legacy_selected if _core_multiface_eligible(obs)
    ]
    selected = (
        core_selected if observation_policy == "core_multiface"
        else legacy_selected
    )
    output = [PixelObs(
        marker="cube",
        cam=int(obs.cam),
        event=int(obs.event),
        set_idx=None if obs.set_idx is None else int(obs.set_idx),
        object_points=np.asarray(obs.object_points, dtype=np.float64).reshape(-1, 3),
        image_points=np.asarray(obs.image_points, dtype=np.float64).reshape(-1, 2),
        grasp_idx=None if getattr(obs, "grasp_idx", None) is None else int(obs.grasp_idx),
    ) for obs in selected]
    diagnostics = dict(diagnostics)
    selected_keys = {(int(obs.event), int(obs.cam)) for obs in selected}
    legacy_keys = {(int(obs.event), int(obs.cam)) for obs in legacy_selected}
    core_keys = {(int(obs.event), int(obs.cam)) for obs in core_selected}
    quality_records = []
    for source_record in diagnostics.get("observation_quality_by_event_camera", []):
        record = dict(source_record)
        key = (int(record["event_id"]), int(record["camera_id"]))
        record["legacy_policy_selected"] = key in legacy_keys
        record["core_multiface_selected"] = key in core_keys
        record["selected_for_calibration"] = key in selected_keys
        if not record.get("pnp_accepted"):
            reason = "pnp_not_accepted"
        elif key not in legacy_keys:
            reason = "fixed_camera_below_min_corner_count"
        elif observation_policy == "core_multiface" and key not in core_keys:
            reason = f"noncore_{record.get('quality_tier', 'unknown')}"
        else:
            reason = "selected"
        record["selection_reason"] = reason
        quality_records.append(record)
    diagnostics["observation_quality_by_event_camera"] = quality_records

    selected_by_event: Dict[int, list] = defaultdict(list)
    for obs in selected:
        selected_by_event[int(obs.event)].append(obs)
    diagnostics["selected_event_summary"] = [
        {
            "event_id": int(event),
            "camera_ids": sorted(int(obs.cam) for obs in event_observations),
            "marker_ids": sorted({
                int(marker_id) for obs in event_observations
                for marker_id in obs.marker_ids
            }),
            "observed_faces": sorted({
                str(face) for obs in event_observations
                for face in obs.observed_faces
            }),
            "observation_count": int(len(event_observations)),
        }
        for event, event_observations in sorted(selected_by_event.items())
    ]
    diagnostics["observation_policy"] = observation_policy
    diagnostics["available_observation_policy_comparison"] = {
        "pnp_accepted": int(len(detected)),
        "legacy": int(len(legacy_selected)),
        "core_multiface": int(len(core_selected)),
        "planar_multimarker": int(sum(
            obs.quality_tier == "planar_multimarker" for obs in legacy_selected)),
        "single_marker": int(sum(
            obs.quality_tier == "single_marker" for obs in legacy_selected)),
    }
    diagnostics["available_quality_tier_counts"] = dict(sorted(Counter(
        str(obs.quality_tier) for obs in detected).items()))
    diagnostics["selected_quality_tier_counts"] = dict(sorted(Counter(
        str(obs.quality_tier) for obs in selected).items()))
    diagnostics["fixed_camera_min_corners"] = int(fixed_min_corners)
    diagnostics["fixed_camera_min_corner_rejections"] = int(
        len(detected) - len(legacy_selected))
    diagnostics["core_multiface_rejections_after_legacy_gate"] = int(
        len(legacy_selected) - len(core_selected))
    diagnostics["final_observation_count"] = int(len(output))
    return output, diagnostics


def load_cube_board_pixel_observations(root: str, meta: dict, cube,
                                       K_map, D_map,
                                       all_cam_ids: Sequence[int],
                                       gripper_cam_idx: int,
                                       exclude_gripped_cube: bool = True,
                                       fixed_cube_min_corners: int = 8,
                                       image_scale: float = 1.0,
                                       cube_observation_policy: str = "core_multiface"):
    cube_observations, cube_reason = load_cube_pixel_observations(
        root, meta, cube, K_map, D_map, all_cam_ids, gripper_cam_idx,
        exclude_gripped=exclude_gripped_cube,
        fixed_min_corners=fixed_cube_min_corners,
        image_scale=float(image_scale),
        observation_policy=str(cube_observation_policy),
    )
    board_observations = load_board_pixel_observations(
        root, meta, all_cam_ids, gripper_cam_idx,
        image_scale=float(image_scale))
    return cube_observations + board_observations, {
        "cube": cube_reason,
        "n_cube_observations": len(cube_observations),
        "n_board_observations": len(board_observations),
    }
