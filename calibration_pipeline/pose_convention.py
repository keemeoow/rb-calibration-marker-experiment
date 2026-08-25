"""Normalize heterogeneous robot-pose recording conventions before calibration.

The calibration model requires every ``T_base_gripper`` and every nominal cube
centre to use one coordinate convention. A fixed tool-frame change is harmless
only when it is applied to the entire dataset; mixing two conventions makes a
single hand-eye transform physically impossible. This module applies an
explicit, versioned sidecar manifest and checks the resulting SE(3) invariants.
"""

from __future__ import annotations

import copy
import hashlib
import json
import os
from typing import Dict, Iterable, List, Mapping, Optional, Tuple

import numpy as np

from calibration_pipeline.apriltag_cube import inv_T
from calibration_pipeline.runtime import rotation_error_deg
from capture_pipeline.robot import euler_deg_to_matrix


POSE_CONVENTION_SCHEMA = "pose_convention_manifest_v1"
POSE_CONVENTION_FILENAME = "pose_convention_manifest.json"


def _sha256(path: str) -> str:
    digest = hashlib.sha256()
    with open(path, "rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _as_transform(value, label: str) -> np.ndarray:
    transform = np.asarray(value, dtype=np.float64)
    if transform.shape != (4, 4) or not np.all(np.isfinite(transform)):
        raise ValueError(f"{label} must be a finite 4x4 transform")
    if not np.allclose(transform[3], [0.0, 0.0, 0.0, 1.0], atol=1e-10):
        raise ValueError(f"{label} has an invalid homogeneous last row")
    rotation = transform[:3, :3]
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-8):
        raise ValueError(f"{label} rotation is not orthonormal")
    if not np.isclose(np.linalg.det(rotation), 1.0, atol=1e-8):
        raise ValueError(f"{label} rotation determinant is not +1")
    return transform


def _stored_robot_transform(capture: Mapping) -> Optional[np.ndarray]:
    for key in ("robot_pose_matrix_4x4", "capture_gripper_pose_matrix_4x4"):
        value = capture.get(key)
        if value is not None:
            try:
                return _as_transform(value, f"capture.{key}")
            except (TypeError, ValueError):
                pass
    for key in ("robot_pose_6dof", "capture_gripper_pose_6dof"):
        value = capture.get(key)
        if isinstance(value, list) and len(value) == 6:
            try:
                return euler_deg_to_matrix(*[float(item) for item in value])
            except (TypeError, ValueError):
                pass
    return None


def _stored_pose6_transform(capture: Mapping, key: str) -> Optional[np.ndarray]:
    value = capture.get(key)
    if not isinstance(value, list) or len(value) != 6:
        return None
    try:
        return euler_deg_to_matrix(*[float(item) for item in value])
    except (TypeError, ValueError):
        return None


def _events_from_segment(segment: Mapping) -> Iterable[int]:
    ranges = segment.get("event_ranges")
    if not isinstance(ranges, list) or not ranges:
        raise ValueError("each pose-convention segment needs non-empty event_ranges")
    for bounds in ranges:
        if (not isinstance(bounds, list) or len(bounds) != 2
                or int(bounds[1]) < int(bounds[0])):
            raise ValueError(f"invalid pose-convention event range: {bounds!r}")
        yield from range(int(bounds[0]), int(bounds[1]) + 1)


def _cluster_relative_signatures(
    captures: Iterable[Mapping],
    robot_key: str,
    cube_key: str,
    translation_tol_mm: float,
    rotation_tol_deg: float,
) -> List[dict]:
    clusters: List[dict] = []
    for capture in captures:
        robot = capture.get(robot_key)
        cube = capture.get(cube_key)
        if robot is None or cube is None:
            continue
        relative = inv_T(np.asarray(robot, dtype=np.float64)) @ np.asarray(
            cube, dtype=np.float64)
        event = int(capture.get("event_id", -1))
        assigned = None
        for cluster in clusters:
            reference = cluster["reference"]
            dt_mm = float(np.linalg.norm(
                relative[:3, 3] - reference[:3, 3]) * 1000.0)
            dr_deg = rotation_error_deg(relative[:3, :3], reference[:3, :3])
            if dt_mm <= translation_tol_mm and dr_deg <= rotation_tol_deg:
                assigned = cluster
                break
        if assigned is None:
            assigned = {"reference": relative, "events": []}
            clusters.append(assigned)
        assigned["events"].append(event)
    return clusters


def _stationarity_diagnostics(
    captures: Iterable[Mapping],
    translation_tol_mm: float,
    rotation_tol_deg: float,
) -> Dict[str, dict]:
    by_set: Dict[int, List[Tuple[int, np.ndarray]]] = {}
    for capture in captures:
        set_index = capture.get("set_index")
        transform = capture.get("canonical_set_cube_center_matrix_4x4")
        if set_index is None or transform is None:
            continue
        by_set.setdefault(int(set_index), []).append((
            int(capture.get("event_id", -1)),
            np.asarray(transform, dtype=np.float64),
        ))
    diagnostics = {}
    failures = []
    for set_index, rows in sorted(by_set.items()):
        reference = rows[0][1]
        translation = [float(np.linalg.norm(
            transform[:3, 3] - reference[:3, 3]) * 1000.0)
            for _, transform in rows]
        rotation = [rotation_error_deg(
            transform[:3, :3], reference[:3, :3])
            for _, transform in rows]
        diagnostics[str(set_index)] = {
            "events": [event for event, _ in rows],
            "max_translation_delta_mm": max(translation, default=0.0),
            "max_rotation_delta_deg": max(rotation, default=0.0),
        }
        if (max(translation, default=0.0) > translation_tol_mm
                or max(rotation, default=0.0) > rotation_tol_deg):
            failures.append(set_index)
    if failures:
        raise ValueError(
            "one set_index contains non-stationary cube-centre records after "
            f"pose normalization: sets={failures}; split placements into distinct "
            "set indices or correct the pose-convention manifest")
    return diagnostics


def apply_pose_convention_manifest(
    root_folder: str,
    meta: Mapping,
) -> Tuple[dict, dict]:
    """Return metadata normalized to one declared robot/cube frame.

    The manifest stores right-multiplication transforms:
    ``T_base_canonical = T_base_reported @ T_reported_canonical``.
    No image measurement or fitted calibration result participates in selection.
    """
    manifest_path = os.path.abspath(os.path.join(
        root_folder, POSE_CONVENTION_FILENAME))
    output = copy.deepcopy(dict(meta))
    captures = output.get("captures", [])
    if not os.path.isfile(manifest_path):
        raw_captures = []
        for capture in captures:
            robot = _stored_robot_transform(capture)
            cube = _stored_pose6_transform(capture, "capture_cube_center_6dof")
            if robot is not None and cube is not None:
                raw_captures.append({
                    "event_id": capture.get("event_id", -1),
                    "robot": robot,
                    "cube": cube,
                })
        clusters = _cluster_relative_signatures(
            raw_captures, "robot", "cube", 2.0, 0.2)
        if len(clusters) > 1:
            cluster_events = [cluster["events"] for cluster in clusters]
            raise ValueError(
                "mixed robot pose recording conventions detected from "
                "inv(T_base_robot) @ T_base_capture_cube_center; add an explicit "
                f"{POSE_CONVENTION_FILENAME}. clusters={cluster_events}")
        return output, {
            "status": "single_undeclared_convention",
            "manifest_path": None,
            "manifest_sha256": None,
            "canonical_robot_frame": "as_recorded",
            "canonical_cube_center_frame": "as_recorded",
            "normalized_events": [],
        }

    with open(manifest_path) as handle:
        manifest = json.load(handle)
    if manifest.get("schema_version") != POSE_CONVENTION_SCHEMA:
        raise ValueError(
            f"pose convention schema must be {POSE_CONVENTION_SCHEMA!r}")
    segments = manifest.get("segments")
    if not isinstance(segments, list) or not segments:
        raise ValueError("pose convention manifest needs non-empty segments")

    by_event = {}
    for segment_index, segment in enumerate(segments):
        robot_correction = _as_transform(
            segment.get("T_reported_robot_to_canonical"),
            f"segments[{segment_index}].T_reported_robot_to_canonical")
        cube_correction = _as_transform(
            segment.get("T_reported_cube_center_to_canonical"),
            f"segments[{segment_index}].T_reported_cube_center_to_canonical")
        for event in _events_from_segment(segment):
            if event in by_event:
                raise ValueError(f"pose convention event {event} appears twice")
            by_event[event] = (segment_index, robot_correction, cube_correction)

    normalized_events = []
    for capture in captures:
        event = int(capture.get("event_id", -1))
        if event not in by_event:
            raise ValueError(
                f"pose convention manifest does not cover selected event {event}")
        segment_index, robot_correction, cube_correction = by_event[event]
        robot = _stored_robot_transform(capture)
        if robot is None:
            raise ValueError(f"event {event} has no valid robot pose")
        set_cube = _stored_pose6_transform(capture, "set_cube_center_6dof")
        capture_cube = _stored_pose6_transform(
            capture, "capture_cube_center_6dof")
        capture["canonical_robot_pose_matrix_4x4"] = (
            robot @ robot_correction).tolist()
        if set_cube is not None:
            capture["canonical_set_cube_center_matrix_4x4"] = (
                set_cube @ cube_correction).tolist()
        if capture_cube is not None:
            capture["canonical_capture_cube_center_matrix_4x4"] = (
                capture_cube @ cube_correction).tolist()
        capture["pose_convention_segment"] = int(segment_index)
        normalized_events.append(event)

    canonical_rows = [capture for capture in captures
                      if capture.get("canonical_capture_cube_center_matrix_4x4")]
    clusters = _cluster_relative_signatures(
        canonical_rows,
        "canonical_robot_pose_matrix_4x4",
        "canonical_capture_cube_center_matrix_4x4",
        float(manifest.get("relative_signature_translation_tol_mm", 2.0)),
        float(manifest.get("relative_signature_rotation_tol_deg", 0.2)),
    )
    if len(clusters) != 1:
        raise ValueError(
            "pose convention normalization did not produce one robot-to-cube-centre "
            f"signature: {[cluster['events'] for cluster in clusters]}")

    stationarity_translation_tol_mm = float(
        manifest.get("set_stationarity_translation_tol_mm", 5.0))
    stationarity_rotation_tol_deg = float(
        manifest.get("set_stationarity_rotation_tol_deg", 1.0))
    stationarity = _stationarity_diagnostics(
        captures,
        stationarity_translation_tol_mm,
        stationarity_rotation_tol_deg,
    )
    signature = clusters[0]["reference"]
    return output, {
        "status": "normalized_and_validated",
        "manifest_path": manifest_path,
        "manifest_sha256": _sha256(manifest_path),
        "canonical_robot_frame": str(manifest["canonical_robot_frame"]),
        "canonical_cube_center_frame": str(
            manifest["canonical_cube_center_frame"]),
        "normalized_events": sorted(normalized_events),
        "relative_robot_to_cube_center_translation_mm": (
            signature[:3, 3] * 1000.0).tolist(),
        "relative_robot_to_cube_center_rotation_deg": rotation_error_deg(
            signature[:3, :3], np.eye(3)),
        "set_stationarity_translation_tol_mm": stationarity_translation_tol_mm,
        "set_stationarity_rotation_tol_deg": stationarity_rotation_tol_deg,
        "set_stationarity": stationarity,
    }
