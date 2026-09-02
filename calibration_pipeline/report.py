"""Generate a calibration-only report from the canonical Table 1 JSON."""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
from pathlib import Path
from statistics import mean, pstdev
from typing import Any, Iterable, Mapping

from calibration_pipeline.runtime import DEFAULT_SESSION_ROOT, session_paths


METHOD_ORDER = ("A0", "A1", "A2", "A3", "A4", "A5", "B1", "B2", "B3")

MATRIX_SEMANTICS = {
    "T_base_Ci": (
        "T^B_Ci; fixed-camera coordinates to robot-base coordinates; "
        "final deployable extrinsic; 4x4 SE(3), translation in meters"),
    "T_gripper_cam": (
        "T^G_C; wrist-camera coordinates to robot-gripper coordinates; "
        "final deployable hand-eye transform; 4x4 SE(3), translation in meters"),
    "T_base_board": (
        "T^B_board; board coordinates to robot-base coordinates; optimized "
        "target pose, not a camera calibration deliverable"),
    "T_base_cube_by_set": (
        "T^B_cube(s); cube coordinates to robot-base coordinates for each set; "
        "optimized/fixed target pose, not a camera calibration deliverable"),
}


def _numbers(values: Iterable[Any]) -> list[float]:
    return [float(value) for value in values
            if isinstance(value, (int, float)) and math.isfinite(float(value))]


def _mean_std(values: Iterable[Any]) -> tuple[float | None, float | None]:
    numeric = _numbers(values)
    if not numeric:
        return None, None
    return mean(numeric), pstdev(numeric)


def _frame_prune_records(value: Any) -> list[dict]:
    records: list[dict] = []
    if isinstance(value, Mapping):
        for key, item in value.items():
            if key == "frame_prune_refit" and isinstance(item, Mapping):
                records.append(dict(item))
            else:
                records.extend(_frame_prune_records(item))
    elif isinstance(value, list):
        for item in value:
            records.extend(_frame_prune_records(item))
    return records


def _fmt(value: Any, digits: int = 4) -> str:
    if value is None:
        return "—"
    return f"{float(value):.{digits}f}"


def _matrix_text(matrix: list[list[float]]) -> str:
    rows = [
        "  [" + ", ".join(f"{float(value): .9f}" for value in row) + "]"
        for row in matrix
    ]
    return "[\n" + ",\n".join(rows) + "\n]"


def _validate(payload: dict, representative_seed: int) -> None:
    rows = payload.get("rows", {})
    if tuple(rows) != METHOD_ORDER:
        raise ValueError(
            f"Table 1 must contain rows {METHOD_ORDER}; got {tuple(rows)}")
    for method in METHOD_ORDER:
        runs = rows[method].get("runs", [])
        if not runs:
            raise ValueError(f"{method} has no calibration runs")
        seeds = {int(run.get("seed", -1)) for run in runs}
        if representative_seed not in seeds:
            raise ValueError(
                f"{method} lacks representative seed {representative_seed}")
        for run in runs:
            transforms = run.get("transforms", {})
            if not transforms.get("T_base_Ci") or transforms.get(
                    "T_gripper_cam") is None:
                raise ValueError(f"{method}/seed{run.get('seed')} lacks final transforms")


def _row_summary(method: str, row: dict,
                 representative_seed: int) -> dict:
    runs = row["runs"]
    train_mean, train_std = _mean_std(
        run["train_reprojection"]["overall"].get("rmse_px") for run in runs)
    heldout_mean, heldout_std = _mean_std(
        run["heldout_reprojection"]["overall"].get("rmse_px") for run in runs)
    prune = [record for run in runs
             for record in _frame_prune_records(run.get("stages", {}))]
    dispersion = row.get("initialization_dispersion", {})
    translation_max = max(_numbers(
        item.get("translation_max_mm") for item in dispersion.values()),
        default=None)
    rotation_max = max(_numbers(
        item.get("rotation_max_deg") for item in dispersion.values()),
        default=None)
    representative = next(
        run for run in runs if int(run["seed"]) == representative_seed)
    transform = representative["transforms"]
    heldout = representative["heldout_reprojection"]["overall"]
    train = representative["train_reprojection"]["overall"]
    condition = row["condition"]
    return {
        "method": method,
        "label": condition["label"],
        "targets": condition["target_set"],
        "optimization": condition["optimization_label"],
        "fk_to_cube": condition["fk_to_cube"],
        "converged_runs": sum(bool(run.get("converged")) for run in runs),
        "total_runs": len(runs),
        "train_rmse_px_mean": train_mean,
        "train_rmse_px_std": train_std,
        "heldout_rmse_px_mean": heldout_mean,
        "heldout_rmse_px_std": heldout_std,
        "train_observations": train.get("n_observations"),
        "train_corners": train.get("n_corners"),
        "heldout_observations": heldout.get("n_observations"),
        "heldout_corners": heldout.get("n_corners"),
        "solver_stages": len(prune),
        "prune_refit_attempts": sum(bool(record.get("selection", {}).get(
            "attempted")) for record in prune),
        "prune_refit_accepted": sum(bool(record.get("accepted")) for record in prune),
        "prune_refit_rollbacks": sum(bool(record.get("rolled_back")) for record in prune),
        "pruned_frames_considered": sum(int(record.get("selection", {}).get(
            "n_pruned_frames", 0)) for record in prune),
        "seed_dispersion_translation_max_mm": translation_max,
        "seed_dispersion_rotation_max_deg": rotation_max,
        "representative_seed": representative_seed,
        "fixed_camera_ids": ",".join(sorted(
            transform["T_base_Ci"], key=lambda value: int(value))),
        "has_board_pose": transform.get("T_base_board") is not None,
        "cube_pose_count": len(transform.get("T_base_cube_by_set", {})),
    }


def _matrix_artifact(payload: dict, source: Path,
                     representative_seed: int) -> dict:
    return {
        "artifact_schema": "calibration_matrices_v1",
        "source_table1": str(source.resolve()),
        "dataset": payload["protocol"].get("dataset"),
        "representative_seed": representative_seed,
        "representative_seed_contract": (
            "seed 0 is the unperturbed shared initialization; it is fixed "
            "before held-out evaluation and is not chosen by held-out score"),
        "matrix_semantics": MATRIX_SEMANTICS,
        "rows": {
            method: {
                "condition": payload["rows"][method]["condition"],
                "initialization_dispersion": payload["rows"][method].get(
                    "initialization_dispersion", {}),
                "runs": [
                    {
                        "seed": int(run["seed"]),
                        "converged": bool(run.get("converged")),
                        "train_reprojection_rmse_px": run[
                            "train_reprojection"]["overall"].get("rmse_px"),
                        "heldout_reprojection_rmse_px": run[
                            "heldout_reprojection"]["overall"].get("rmse_px"),
                        "transforms": run["transforms"],
                    }
                    for run in payload["rows"][method]["runs"]
                ],
            }
            for method in METHOD_ORDER
        },
    }


def _markdown(payload: dict, summaries: list[dict], matrix_artifact: dict,
              representative_seed: int) -> str:
    protocol = payload["protocol"]
    split = protocol["split"]
    provenance = protocol["source_data_provenance"]
    train = provenance["observation_populations"]["train"]
    heldout = provenance["observation_populations"]["heldout"]
    solver = protocol["solver_options"]
    prune_totals = {
        key: sum(int(row[key]) for row in summaries)
        for key in ("solver_stages", "prune_refit_attempts",
                    "prune_refit_accepted", "prune_refit_rollbacks",
                    "pruned_frames_considered")
    }
    lines = [
        "# Calibration Results",
        "",
        "이 문서는 `05_calibrate.py`의 결과만 사용한다. Cross-target, marker-system, "
        "OpenCV baseline 또는 외부 GT가 없어도 생성된다.",
        "",
        "## 실행 및 데이터 계약",
        "",
        f"- Dataset: `{protocol.get('dataset')}`",
        f"- Eligible sets: `{split.get('eligible_sets')}`",
        f"- Train: {train['observations']} observations / {train['corners']} corners / "
        f"{len(train['events'])} events",
        f"- Held-out: {heldout['observations']} observations / {heldout['corners']} corners / "
        f"{len(heldout['events'])} events",
        f"- Solver: `{solver['method']}`, loss `{solver['loss']}`, "
        f"f_scale `{solver['f_scale_px']} px`, residual weighting "
        f"`{solver['residual_weighting']}`",
        f"- Representative matrices: seed `{representative_seed}`; seed 0 is "
        "the unperturbed initialization and is not selected using held-out error.",
        "",
        "## 행렬이 생성되는 시점",
        "",
        "| 단계 | 행렬/파라미터 | 상태와 저장 위치 |",
        "| --- | --- | --- |",
        "| 01 | Factory `color_K`, `color_D`, `depth_K`, depth-to-color extrinsic, depth scale | `intrinsics/cam*.npz`; 센서에서 읽은 초기 intrinsic |",
        "| 02 | Refined `color_K`, `color_D` | 같은 `cam*.npz`를 갱신하고 factory 값은 `factory_backup/`에 보존 |",
        "| 03 | Event별 `T_base_gripper`와 raw cube-center pose | `calib_train/meta.json`; robot FK 입력이며 calibration 결과가 아님 |",
        "| 04 | Cube/board PnP pose | 검출 품질·positive-depth 판정에만 임시 사용; 최종 행렬로 저장하거나 전달하지 않음 |",
        "| 05 초기화 | `shared_reference_state`와 행별 초기 행렬 | `shared_train_only_baseline.json`; held-out을 쓰지 않은 optimizer 시작점 |",
        "| 05 FK 정렬 | `T_gripper_cam`, `T_fk_cube_center_to_tag_object`, raw/aligned set pose | `shared_board_free_fk_cube.json`; A4/A5/B1/B2용 train-only artifact |",
        "| 05 최종 | 각 행·seed의 `T_base_Ci`, `T_gripper_cam`, board/cube pose | `table1_methods.json → rows.<행>.runs[*].transforms`; fit/refit/rollback 뒤 확정 |",
        "| 06 | 새 행렬 없음 | 05 결과를 CSV/JSON/Markdown으로 정리 |",
        "",
        "## 최종 행렬의 의미",
        "",
        "- `T_base_Ci = T^B_Ci`: 고정카메라 좌표를 robot-base 좌표로 변환하는 최종 extrinsic.",
        "- `T_gripper_cam = T^G_C`: wrist-camera 좌표를 gripper 좌표로 변환하는 최종 hand-eye.",
        "- `T_base_board`, `T_base_cube_by_set`: 공동 최적화를 연결하는 target pose이며 배포용 camera calibration 행렬이 아니다.",
        "- 모든 4×4 행렬의 translation 단위는 meter다. 회전은 좌상단 3×3 rotation matrix다.",
        "",
        "## Table 1 calibration 요약",
        "",
        "| Row | Condition | Target | FK | Converged | Train px | Held-out px | "
        "Prune attempts / accepted / rollback | Seed max Δ mm / deg |",
        "| --- | --- | --- | --- | ---: | ---: | ---: | ---: | ---: |",
    ]
    for row in summaries:
        lines.append(
            f"| {row['method']} | {row['label']} | {row['targets']} | "
            f"{row['fk_to_cube']} | {row['converged_runs']}/{row['total_runs']} | "
            f"{_fmt(row['train_rmse_px_mean'])} ± {_fmt(row['train_rmse_px_std'])} | "
            f"{_fmt(row['heldout_rmse_px_mean'])} ± {_fmt(row['heldout_rmse_px_std'])} | "
            f"{row['prune_refit_attempts']} / {row['prune_refit_accepted']} / "
            f"{row['prune_refit_rollbacks']} | "
            f"{_fmt(row['seed_dispersion_translation_max_mm'], 5)} / "
            f"{_fmt(row['seed_dispersion_rotation_max_deg'], 6)} |")
    lines.extend([
        "",
        "### 행별 실제 residual 모집단 — representative seed",
        "",
        "| Row | Train observations / corners | Held-out observations / corners | "
        "Fixed cameras | Board pose | Cube set poses |",
        "| --- | ---: | ---: | --- | --- | ---: |",
    ])
    for row in summaries:
        lines.append(
            f"| {row['method']} | {row['train_observations']} / "
            f"{row['train_corners']} | {row['heldout_observations']} / "
            f"{row['heldout_corners']} | {row['fixed_camera_ids']} | "
            f"{row['has_board_pose']} | {row['cube_pose_count']} |")
    lines.extend([
        "",
        "## Frame-prune 전체 결과",
        "",
        f"- Solver stages: {prune_totals['solver_stages']}",
        f"- Prune/refit attempts: {prune_totals['prune_refit_attempts']}",
        f"- Candidate frames considered for pruning: "
        f"{prune_totals['pruned_frames_considered']}",
        f"- Accepted refits: {prune_totals['prune_refit_accepted']}",
        f"- Rollbacks: {prune_totals['prune_refit_rollbacks']}",
        "- Rollback은 오류가 아니다. 제거 전 전체 train robust objective가 개선되지 "
        "않으면 첫 fit을 유지하는 정상 동작이다.",
        "",
        f"## 대표 최종 행렬 — seed {representative_seed}",
        "",
        "아래에는 실제 배포 대상인 fixed-camera extrinsic과 hand-eye만 표시한다. "
        "모든 seed와 target pose의 정확한 값은 `calibration_matrices.json`에 있다.",
        "",
    ])
    for method in METHOD_ORDER:
        method_payload = matrix_artifact["rows"][method]
        run = next(item for item in method_payload["runs"]
                   if int(item["seed"]) == representative_seed)
        transforms = run["transforms"]
        lines.extend([
            f"### {method} — {method_payload['condition']['label']}",
            "",
            f"Converged: `{run['converged']}` · train "
            f"`{_fmt(run['train_reprojection_rmse_px'])} px` · held-out "
            f"`{_fmt(run['heldout_reprojection_rmse_px'])} px`",
            "",
            "`T_gripper_cam`:",
            "",
            "```text",
            _matrix_text(transforms["T_gripper_cam"]),
            "```",
            "",
        ])
        for camera, matrix in sorted(
                transforms["T_base_Ci"].items(), key=lambda item: int(item[0])):
            lines.extend([
                f"`T_base_C{camera}`:",
                "",
                "```text",
                _matrix_text(matrix),
                "```",
                "",
            ])
    lines.extend([
        "## 해석 제한",
        "",
        "- Held-out reprojection은 행마다 사용 marker가 다를 수 있어 모든 행의 절대 "
        "순위를 정하는 외부 정확도 지표가 아니다.",
        "- A4/B1/B2의 FK covariance가 simulation prior이면 confirmatory 물리 결과가 아니다.",
        "- 외부 GT와 robot task가 없으므로 실제 robot-base 절대 정확도는 아직 확정할 수 없다.",
        "",
    ])
    return "\n".join(lines)


def write_report(table1_path: Path, out_dir: Path,
                 representative_seed: int = 0) -> dict:
    payload = json.loads(table1_path.read_text(encoding="utf-8"))
    _validate(payload, representative_seed)
    summaries = [
        _row_summary(method, payload["rows"][method], representative_seed)
        for method in METHOD_ORDER
    ]
    matrix_artifact = _matrix_artifact(
        payload, table1_path, representative_seed)
    out_dir.mkdir(parents=True, exist_ok=True)
    csv_path = out_dir / "calibration_summary.csv"
    matrix_path = out_dir / "calibration_matrices.json"
    markdown_path = out_dir / "CALIBRATION_RESULTS.md"
    with csv_path.open("w", encoding="utf-8-sig", newline="") as stream:
        writer = csv.DictWriter(
            stream, fieldnames=list(summaries[0]), lineterminator="\n")
        writer.writeheader()
        writer.writerows(summaries)
    matrix_path.write_text(
        json.dumps(matrix_artifact, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8")
    markdown_path.write_text(
        _markdown(payload, summaries, matrix_artifact, representative_seed),
        encoding="utf-8")
    return {
        "source": str(table1_path),
        "csv": str(csv_path),
        "matrices": str(matrix_path),
        "markdown": str(markdown_path),
        "rows": len(summaries),
        "representative_seed": representative_seed,
    }


def parse_args(argv=None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root_folder", default=DEFAULT_SESSION_ROOT)
    parser.add_argument(
        "--table1", help="Default: CP_result/<session>/late_table1/table1_methods.json")
    parser.add_argument(
        "--out_dir", help="Default: CP_result/<session>/late_table1")
    parser.add_argument(
        "--representative_seed", type=int, default=0,
        help="Fixed seed to print as representative; never selected by held-out score")
    return parser.parse_args(argv)


def main(argv=None) -> None:
    args = parse_args(argv)
    paths = session_paths(args.root_folder)
    table1_path = Path(args.table1 or paths["table1_result"])
    out_dir = Path(args.out_dir or paths["table1_dir"])
    result = write_report(
        table1_path, out_dir, representative_seed=args.representative_seed)
    print(json.dumps(result, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
