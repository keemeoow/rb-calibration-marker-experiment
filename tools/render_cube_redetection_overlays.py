#!/usr/bin/env python3
"""Render baseline-versus-recovered cube detections for visual review."""

from __future__ import annotations

import argparse
import json
import os
import sys
from pathlib import Path

import cv2
import numpy as np


PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from calibration_pipeline.apriltag_cube import AprilTagCubeTarget  # noqa: E402
from calibration_pipeline.cube_config import load_cube_config_from_meta  # noqa: E402


def _configured_detections(target, corners, ids):
    if ids is None:
        return []
    return [
        (int(marker_id), np.asarray(corner, dtype=np.float64).reshape(4, 2))
        for corner, marker_id in zip(corners, np.asarray(ids).reshape(-1))
        if target.model.has_marker(int(marker_id))
    ]


def _draw_detections(image, detections, color, prefix):
    output = image
    for marker_id, points in detections:
        polygon = np.round(points).astype(np.int32)
        cv2.polylines(output, [polygon], True, color, 3, cv2.LINE_AA)
        anchor = tuple((polygon[0] + np.array([4, -5])).tolist())
        cv2.putText(
            output, f"{prefix}{marker_id}", anchor,
            cv2.FONT_HERSHEY_SIMPLEX, 0.65, color, 2, cv2.LINE_AA,
        )
    return output


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--session-root", default="data/session04/calib_train")
    parser.add_argument(
        "--quality-json",
        default=("data/session04/calib_out/verify/cube_observation_quality/"
                 "cube_observation_quality.json"),
    )
    parser.add_argument(
        "--output",
        default=("data/session04/calib_out/verify/cube_observation_quality/"
                 "redetection_recovered_core_overlay.png"),
    )
    args = parser.parse_args()

    session_root = Path(args.session_root)
    with open(session_root / "meta.json", "r", encoding="utf-8") as stream:
        meta = json.load(stream)
    with open(args.quality_json, "r", encoding="utf-8") as stream:
        report = json.load(stream)
    cfg, _ = load_cube_config_from_meta(str(session_root))
    target = AprilTagCubeTarget(cfg)
    captures = {
        int(capture["event_id"]): capture
        for capture in meta.get("captures", [])
    }
    recovered = sorted(
        (record for record in report["diagnostics"]
         ["observation_quality_by_event_camera"]
         if record.get("recovered_core_observation")),
        key=lambda record: (
            int(record["event_id"]), int(record["camera_id"])),
    )
    if not recovered:
        raise RuntimeError("No recovered core observations found")

    cell_width, image_height, header_height = 640, 360, 82
    panels = []
    for record in recovered:
        event = int(record["event_id"])
        camera = int(record["camera_id"])
        camera_info = captures[event]["cams"][str(camera)]
        image_path = session_root / camera_info["rgb_path"]
        image = cv2.imread(str(image_path))
        if image is None:
            raise FileNotFoundError(image_path)

        baseline_corners, baseline_ids = target.detect(image)
        selected_method = str(record["detection_method"])
        recovery_corners = recovery_ids = None
        for method, corners, ids in target.detect_recovery_candidates(image):
            if method == selected_method:
                recovery_corners, recovery_ids = corners, ids
                break
        if recovery_corners is None:
            raise RuntimeError(
                f"Recovery method {selected_method!r} unavailable for E{event:02d}")

        overlay = image.copy()
        _draw_detections(
            overlay,
            _configured_detections(target, baseline_corners, baseline_ids),
            (40, 40, 230),
            "B",
        )
        _draw_detections(
            overlay,
            _configured_detections(target, recovery_corners, recovery_ids),
            (30, 220, 30),
            "R",
        )
        resized = cv2.resize(
            overlay, (cell_width, image_height), interpolation=cv2.INTER_AREA)
        header = np.full(
            (header_height, cell_width, 3), (25, 25, 25), dtype=np.uint8)
        baseline_text = ",".join(
            str(value) for value in record.get("baseline_marker_ids", [])) or "none"
        recovered_text = ",".join(
            str(value) for value in record.get("marker_ids", [])) or "none"
        cv2.putText(
            header,
            (f"E{event:02d}/cam{camera}  baseline [{baseline_text}] -> "
             f"recovered [{recovered_text}]"),
            (12, 31), cv2.FONT_HERSHEY_SIMPLEX, 0.64,
            (240, 240, 240), 2, cv2.LINE_AA,
        )
        cv2.putText(
            header,
            (f"{selected_method}  RMSE {float(record['pnp_rmse_px']):.3f}px  "
             "red=B, green=R"),
            (12, 64), cv2.FONT_HERSHEY_SIMPLEX, 0.56,
            (90, 220, 255), 2, cv2.LINE_AA,
        )
        panels.append(np.vstack([header, resized]))

    contact = np.hstack(panels)
    output = Path(args.output)
    os.makedirs(output.parent, exist_ok=True)
    if not cv2.imwrite(str(output), contact):
        raise RuntimeError(f"Failed to write {output}")
    print(output)


if __name__ == "__main__":
    main()
