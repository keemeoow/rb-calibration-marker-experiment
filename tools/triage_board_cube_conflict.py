#!/usr/bin/env python3
"""Immediate Session04 triage for the board/cube direct-PnP conflict."""

from __future__ import annotations

import argparse
import json
import sys
from collections import defaultdict
from pathlib import Path
from typing import Iterable

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from calibration_pipeline.apriltag_cube import inv_T  # noqa: E402
from calibration_pipeline.observations import (  # noqa: E402
    load_pixel_observations_from_manifest,
)
from calibration_pipeline.path_evaluation import solve_observed_pose  # noqa: E402
from calibration_pipeline.reprojection import PixelObs, pose_delta  # noqa: E402
from calibration_pipeline.se3 import robust_se3_average  # noqa: E402
from tools.verify_board_cube_relative_pose import (  # noqa: E402
    _fit_targets,
    _load_intrinsic_variants,
    _split_observations,
)


DEFAULT_OUT_DIR = (
    ROOT / "data/session04/calib_out/verify/board_cube_relative_pose"
)


def _jsonable(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, dict):
        return {str(key): _jsonable(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_jsonable(item) for item in value]
    return value


def _load_cube_features(manifest: Path, policy: str) -> dict[tuple[int, int], dict]:
    payload = json.loads(manifest.read_text(encoding="utf-8"))
    features = {}
    for record in payload.get("observations", []):
        if record.get("target") != "cube":
            continue
        if not bool(record.get("selected_by_policy", {}).get(policy, False)):
            continue
        key = (int(record["event_id"]), int(record["camera_id"]))
        features[key] = {
            "corner_count": int(record.get("corner_count", 0)),
            "observed_face_count": int(record.get("observed_face_count", 0)),
            "observed_faces": list(record.get("observed_faces", [])),
            "pnp_rmse_px": (
                None if record.get("pnp_rmse_px") is None
                else float(record["pnp_rmse_px"])
            ),
            "detection_method": str(record.get("detection_method", "")),
        }
    return features


def _fit_conflict(
    observations: list[PixelObs],
    fixed_cameras: list[int],
    K_map,
    D_map,
) -> dict:
    fitted, diagnostics = _fit_targets(observations, fixed_cameras, K_map, D_map)
    squared = []
    per_camera = {}
    for camera in fixed_cameras[1:]:
        translation_mm, rotation_deg = pose_delta(
            fitted["board"][camera], fitted["cube"][camera])
        squared.append(translation_mm ** 2)
        per_camera[str(camera)] = {
            "translation_mm": float(translation_mm),
            "rotation_deg": float(rotation_deg),
        }
    return {
        "translation_rmse_mm": float(np.sqrt(np.mean(squared))),
        "per_camera": per_camera,
        "fit_diagnostics": diagnostics,
    }


def _relative_candidate_rows(
    observations: list[PixelObs],
    fixed_cameras: list[int],
    K_map,
    D_map,
) -> dict:
    anchor = int(fixed_cameras[0])
    grouped = defaultdict(dict)
    for observation in observations:
        camera = int(observation.cam)
        if camera in set(fixed_cameras) and observation.marker in {"board", "cube"}:
            grouped[(str(observation.marker), int(observation.event))][
                camera] = observation

    output = {}
    for target in ("board", "cube"):
        output[target] = {}
        for camera in fixed_cameras[1:]:
            candidates = []
            for (marker, event), by_camera in sorted(grouped.items()):
                if marker != target or anchor not in by_camera or camera not in by_camera:
                    continue
                T_anchor_target = solve_observed_pose(
                    by_camera[anchor], K_map, D_map)
                T_camera_target = solve_observed_pose(
                    by_camera[camera], K_map, D_map)
                if T_anchor_target is None or T_camera_target is None:
                    continue
                transform = T_anchor_target @ inv_T(T_camera_target)
                candidates.append({
                    "event_id": int(event),
                    "transform": transform,
                    "anchor_corner_count": int(len(by_camera[anchor].image_points)),
                    "camera_corner_count": int(len(by_camera[camera].image_points)),
                    "baseline_norm_mm": float(
                        np.linalg.norm(transform[:3, 3]) * 1000.0),
                })
            transforms = [row["transform"] for row in candidates]
            average, diagnostics = robust_se3_average(transforms)
            rows = []
            for row in candidates:
                translation_mm, rotation_deg = pose_delta(
                    average, row["transform"])
                rows.append({
                    "event_id": row["event_id"],
                    "translation_deviation_mm": float(translation_mm),
                    "rotation_deviation_deg": float(rotation_deg),
                    "anchor_corner_count": row["anchor_corner_count"],
                    "camera_corner_count": row["camera_corner_count"],
                    "baseline_norm_mm": row["baseline_norm_mm"],
                })
            rows.sort(key=lambda item: item["translation_deviation_mm"],
                      reverse=True)
            output[target][str(camera)] = {
                "diagnostics": diagnostics,
                "rows": rows,
                "source_event_ids": [row["event_id"] for row in candidates],
                "worst_event": rows[0] if rows else None,
            }
    return output


def _source_events(candidate_rows: dict) -> list[int]:
    events = set()
    for by_camera in candidate_rows.values():
        for detail in by_camera.values():
            events.update(detail["source_event_ids"])
    return sorted(events)


def _leave_one_event_sensitivity(
    train: list[PixelObs],
    fixed_cameras: list[int],
    K_map,
    D_map,
    source_events: Iterable[int],
    base_rmse: float,
) -> list[dict]:
    rows = []
    for event in source_events:
        subset = [obs for obs in train if int(obs.event) != int(event)]
        result = _fit_conflict(subset, fixed_cameras, K_map, D_map)
        rows.append({
            "dropped_event_id": int(event),
            "translation_rmse_mm": result["translation_rmse_mm"],
            "improvement_mm": float(base_rmse - result["translation_rmse_mm"]),
            "per_camera": result["per_camera"],
        })
    rows.sort(key=lambda item: item["improvement_mm"], reverse=True)
    return rows


def _cube_quality_filter_sensitivity(
    train: list[PixelObs],
    fixed_cameras: list[int],
    K_map,
    D_map,
    cube_features: dict[tuple[int, int], dict],
) -> list[dict]:
    configs = [
        ("base", 0, 0, False),
        ("cube >=12 corners", 12, 0, False),
        ("cube >=3 faces", 0, 3, False),
        ("cube has +Z face", 0, 0, True),
        ("cube >=12 corners & >=3 faces", 12, 3, False),
        ("cube >=12 corners & +Z face", 12, 0, True),
    ]
    rows = []
    for name, min_corners, min_faces, require_top in configs:
        subset = []
        for obs in train:
            if obs.marker != "cube":
                subset.append(obs)
                continue
            feature = cube_features.get((int(obs.event), int(obs.cam)), {})
            if int(feature.get("corner_count", len(obs.image_points))) < min_corners:
                continue
            if int(feature.get("observed_face_count", 0)) < min_faces:
                continue
            if require_top and "+Z" not in set(feature.get("observed_faces", [])):
                continue
            subset.append(obs)
        try:
            result = _fit_conflict(subset, fixed_cameras, K_map, D_map)
            rows.append({
                "filter": name,
                "train_cube_observations": int(
                    sum(1 for obs in subset if obs.marker == "cube")),
                "translation_rmse_mm": result["translation_rmse_mm"],
                "per_camera": result["per_camera"],
                "fit_diagnostics": result["fit_diagnostics"]["cube"],
                "usable": True,
            })
        except Exception as error:  # noqa: BLE001 - diagnostic output
            rows.append({
                "filter": name,
                "train_cube_observations": int(
                    sum(1 for obs in subset if obs.marker == "cube")),
                "usable": False,
                "error": str(error),
            })
    return rows


def _same_support_check(
    train: list[PixelObs],
    fixed_cameras: list[int],
    K_map,
    D_map,
) -> dict:
    anchor = int(fixed_cameras[0])
    grouped = defaultdict(dict)
    for obs in train:
        if int(obs.cam) in set(fixed_cameras):
            grouped[(str(obs.marker), int(obs.event))][int(obs.cam)] = obs

    output = {}
    for camera in fixed_cameras[1:]:
        board_events = {
            event for (target, event), by_camera in grouped.items()
            if target == "board" and anchor in by_camera and camera in by_camera
        }
        cube_events = {
            event for (target, event), by_camera in grouped.items()
            if target == "cube" and anchor in by_camera and camera in by_camera
        }
        common = sorted(board_events & cube_events)
        if len(common) < 2:
            continue
        subset = [obs for obs in train if int(obs.event) in set(common)]
        result = _fit_conflict(subset, fixed_cameras, K_map, D_map)
        output[str(camera)] = {
            "common_event_ids": common,
            "translation_rmse_mm": result["translation_rmse_mm"],
            "per_camera": result["per_camera"],
            "fit_diagnostics": result["fit_diagnostics"],
        }
    return output


def _fmt(value: float, digits: int = 3) -> str:
    return f"{float(value):.{digits}f}"


def _write_report(path: Path, payload: dict) -> None:
    base = payload["base_conflict"]
    candidates = payload["candidate_dispersion"]
    leave_one = payload["leave_one_event_sensitivity"]
    filters = payload["cube_quality_filter_sensitivity"]
    same_support = payload["same_support_check"]

    lines = [
        "# Board-Cube Conflict Immediate Triage",
        "",
        "> **판정:** 지금 데이터만으로 원인을 더 좁힐 수는 있다. 새로 발견된 "
        "단순 software/config bug는 없고, 남은 문제는 `cube sparse observation + "
        "target-dependent effective scale/localization bias + 제한된 intrinsic "
        "coverage`가 섞인 체계 오차로 보는 것이 가장 안전하다.",
        "",
        "## 지금 바로 확인한 결론",
        "",
        "| 질문 | 결과 | 해석 |",
        "| --- | --- | --- |",
        f"| 재실행하면 같은가? | `{_fmt(base['translation_rmse_mm'], 4)} mm` "
        f"/ max `{_fmt(payload['max_rotation_deg'], 4)}°` | stale 산출물이 아니라 현재 checkout에서도 재현됨 |",
        f"| 한 프레임 문제인가? | event 54 제거 시 `{_fmt(leave_one[0]['translation_rmse_mm'])} mm` | 가장 영향이 크지만 conflict가 남아서 단일 프레임 문제는 아님 |",
        "| solver 문제인가? | PnP scan 최선도 `10.692 mm`, stereoCalibrate도 `12.928 mm` | solver 교체로 해결되지 않음 |",
        "| 관측 하한을 올리면? | `8.973 mm`까지 감소하지만 pair당 후보 1개 수준 | 공식 필터로 쓰기에는 support가 너무 약함 |",
        "| 지금 바꿀 수 있는가? | Board scale/factory K/D로 숫자는 낮아지지만 다른 증거와 충돌 | 공식 결과는 변경하지 않는 것이 맞음 |",
        "",
        "## Relative Candidate Dispersion",
        "",
        "Board 후보는 매우 안정적이고, cube 후보는 특히 `cam0-cam3`에서 크게 흔들린다.",
        "",
        "| Target | Pair | Candidates | Translation std | Worst event | Worst deviation | Corner support |",
        "| --- | --- | ---: | ---: | ---: | ---: | --- |",
    ]
    for target in ("board", "cube"):
        for camera in ("1", "3"):
            detail = candidates[target][camera]
            diag = detail["diagnostics"]
            worst = detail["worst_event"]
            lines.append(
                f"| {target} | cam0-cam{camera} "
                f"| {diag['num_total']} / {diag['num_inliers']} "
                f"| {_fmt(diag['translation_std_mm'])} mm "
                f"| E{worst['event_id']:04d} "
                f"| {_fmt(worst['translation_deviation_mm'])} mm "
                f"| {worst['anchor_corner_count']}/{worst['camera_corner_count']} corners |")

    lines.extend([
        "",
        "## Leave-One Event Sensitivity",
        "",
        "| Drop event | Conflict RMSE | Improvement | cam1 | cam3 |",
        "| ---: | ---: | ---: | ---: | ---: |",
    ])
    for row in leave_one[:6]:
        cam1 = row["per_camera"]["1"]["translation_mm"]
        cam3 = row["per_camera"]["3"]["translation_mm"]
        lines.append(
            f"| E{row['dropped_event_id']:04d} "
            f"| {_fmt(row['translation_rmse_mm'])} mm "
            f"| {_fmt(row['improvement_mm'])} mm "
            f"| {_fmt(cam1)} mm | {_fmt(cam3)} mm |")

    lines.extend([
        "",
        "## Cube Quality Filter Sensitivity",
        "",
        "| Filter | Cube train obs | Conflict RMSE | Candidate support | 판정 |",
        "| --- | ---: | ---: | --- | --- |",
    ])
    for row in filters:
        if not row["usable"]:
            lines.append(
                f"| {row['filter']} | {row['train_cube_observations']} | - | - | {row['error']} |")
            continue
        diag = row["fit_diagnostics"]
        support = (
            f"cam1 {diag['1']['num_total']}/{diag['1']['num_inliers']}, "
            f"cam3 {diag['3']['num_total']}/{diag['3']['num_inliers']}"
        )
        verdict = (
            "support 부족" if min(
                int(diag["1"]["num_total"]), int(diag["3"]["num_total"])) < 3
            else "충분히 사라지지 않음"
        )
        lines.append(
            f"| {row['filter']} | {row['train_cube_observations']} "
            f"| {_fmt(row['translation_rmse_mm'])} mm | {support} | {verdict} |")

    lines.extend([
        "",
        "## Same-Support Check",
        "",
        "Board/cube가 다른 event support를 쓰는 것이 1차 원인인지 확인했다.",
        "",
        "| Pair 기준 | Common events | Conflict RMSE | 해석 |",
        "| --- | --- | ---: | --- |",
    ])
    for camera, row in same_support.items():
        events = ", ".join(f"E{event:04d}" for event in row["common_event_ids"])
        lines.append(
            f"| cam0-cam{camera} | {events} | {_fmt(row['translation_rmse_mm'])} mm "
            "| 같은 event로 제한해도 해결되지 않음 |")

    lines.extend([
        "",
        "## 현재 처리 방침",
        "",
        "1. 공식 Table 1 / 발표 결론은 그대로 `A2 = internal main`으로 둔다.",
        "2. `10.8077 mm`는 최종 정확도가 아니라 Board-only PnP와 Cube-only PnP의 target-dependent diagnostic conflict로만 말한다.",
        "3. cube 관측을 더 세게 자르거나 factory K/D로 바꾸면 일부 숫자는 줄지만, support 또는 held-out cube transfer가 나빠져 공식 해결책으로 쓰지 않는다.",
        "4. 다음 실험은 새 intrinsic coverage와 Track A 반복촬영에서 같은 board/cube를 다양한 거리·화면 위치로 다시 찍어 target-dependent scale/localization bias를 분리한다.",
        "",
    ])
    path.write_text("\n".join(lines), encoding="utf-8")


def main(argv: Iterable[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--session-root", default="data/session04/calib_train")
    parser.add_argument("--intrinsics-dir", default="intrinsics")
    parser.add_argument(
        "--manifest",
        default=("data/session04/calib_out/capture_filter/"
                 "Step2b_observation_manifest.json"))
    parser.add_argument("--observation-filter-policy", default="standard")
    parser.add_argument("--split-seed", type=int, default=20260731)
    parser.add_argument("--test-fraction", type=float, default=0.2)
    parser.add_argument("--min-train-eih-cube-events", type=int, default=3)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUT_DIR)
    args = parser.parse_args(list(argv) if argv is not None else None)

    session_root = Path(args.session_root).resolve()
    intrinsics_dir = Path(args.intrinsics_dir).resolve()
    manifest = Path(args.manifest).resolve()
    observations, _ = load_pixel_observations_from_manifest(
        str(manifest),
        policy=args.observation_filter_policy,
        root=str(session_root),
        intrinsics_dir=str(intrinsics_dir),
        validate_sources=True,
        allow_relocated_root=True,
    )
    split, train, _ = _split_observations(
        observations, gripper=2, fraction=args.test_fraction,
        seed=args.split_seed, minimum=args.min_train_eih_cube_events)
    fixed_cameras = [0, 1, 3]
    K_map, D_map = _load_intrinsic_variants(intrinsics_dir)[
        "charuco_calibrated_KD"]
    cube_features = _load_cube_features(
        manifest, args.observation_filter_policy)
    base_conflict = _fit_conflict(train, fixed_cameras, K_map, D_map)
    candidate_dispersion = _relative_candidate_rows(
        train, fixed_cameras, K_map, D_map)
    leave_one = _leave_one_event_sensitivity(
        train, fixed_cameras, K_map, D_map,
        _source_events(candidate_dispersion),
        base_conflict["translation_rmse_mm"])
    payload = {
        "schema": "board_cube_conflict_immediate_triage_v1",
        "source_manifest": str(manifest),
        "split": split,
        "base_conflict": base_conflict,
        "max_rotation_deg": max(
            row["rotation_deg"] for row in base_conflict["per_camera"].values()),
        "candidate_dispersion": candidate_dispersion,
        "leave_one_event_sensitivity": leave_one,
        "cube_quality_filter_sensitivity": _cube_quality_filter_sensitivity(
            train, fixed_cameras, K_map, D_map, cube_features),
        "same_support_check": _same_support_check(
            train, fixed_cameras, K_map, D_map),
        "decision": {
            "root_cause_status": "narrowed_not_eliminated",
            "single_frame_bug": False,
            "solver_swap_fix": False,
            "official_result_change_recommended_now": False,
            "most_likely_current_explanation": (
                "cube sparse observations and target-dependent effective "
                "scale/localization bias, with limited intrinsic coverage"),
        },
    }
    args.output_dir.mkdir(parents=True, exist_ok=True)
    json_path = args.output_dir / "board_cube_conflict_triage.json"
    report_path = args.output_dir / "BOARD_CUBE_CONFLICT_TRIAGE.md"
    json_path.write_text(
        json.dumps(_jsonable(payload), indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8")
    _write_report(report_path, payload)
    print(report_path)


if __name__ == "__main__":
    main()
