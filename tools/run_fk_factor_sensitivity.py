#!/usr/bin/env python3
"""Run a preflight FK covariance-scale sensitivity for A4.

This is not a confirmatory physical-accuracy experiment.  It keeps the Session04
manifest, split, solver, and A4 row fixed, then changes only the Simulation
prior standard deviation used by the corrected-FK soft factor.  Smaller scale
means a stronger FK factor.
"""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
import sys
from statistics import fmean
from typing import Iterable

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import calibration_pipeline.table1 as table1  # noqa: E402


DEFAULT_SCALES = (0.25, 0.5, 1.0, 2.0, 4.0)
DEFAULT_OUT_DIR = ROOT / "CP_result/session04/fk_factor_sensitivity"


def _mean(values: Iterable[float | None]) -> float | None:
    numeric = [float(value) for value in values if value is not None]
    return None if not numeric else fmean(numeric)


def _metric(run: dict, split: str, target: str) -> float | None:
    value = run[f"{split}_reprojection"].get(target)
    return None if value is None else float(value["rmse_px"])


def _stage(run: dict) -> dict:
    return run["stages"]["joint_eih_e2h"]


def _cost(stage: dict, block: str, key: str) -> float | None:
    value = stage.get("objective_block_costs", {}).get(block, {}).get(key)
    return None if value is None else float(value)


def _row_summary(payload: dict, scale: float, baseline: dict) -> dict:
    row = payload["rows"]["A4"]
    runs = row["runs"]
    stages = [_stage(run) for run in runs]
    covariance = payload["protocol"]["fk_factor"]["covariance"]
    held_overall = _mean(_metric(run, "heldout", "overall") for run in runs)
    held_board = _mean(_metric(run, "heldout", "board") for run in runs)
    held_cube = _mean(_metric(run, "heldout", "cube") for run in runs)
    train_overall = _mean(_metric(run, "train", "overall") for run in runs)
    visual_cost = _mean(
        _cost(stage, "visual", "final_robust_cost") for stage in stages)
    fk_cost = _mean(_cost(stage, "fk", "final_robust_cost") for stage in stages)
    fk_fraction = _mean(
        _cost(stage, "fk", "fraction_of_total_robust_cost")
        for stage in stages)
    nfev = _mean(float(stage.get("nfev", 0)) for stage in stages)
    rolled_back = sum(
        1 for stage in stages
        if stage.get("frame_prune_refit", {}).get("rolled_back") is True)
    return {
        "row": "A4",
        "preflight_std_scale": scale,
        "approx_fk_cost_weight_multiplier": 1.0 / (scale * scale),
        "translation_std_mm": covariance["translation_std_mm"],
        "rotation_std_deg": covariance["rotation_std_deg"],
        "converged_runs": sum(1 for run in runs if run.get("converged")),
        "total_runs": len(runs),
        "heldout_overall_rmse_px": held_overall,
        "heldout_board_rmse_px": held_board,
        "heldout_cube_rmse_px": held_cube,
        "delta_vs_A2_overall_px": held_overall - baseline["overall"],
        "delta_vs_A2_board_px": held_board - baseline["board"],
        "delta_vs_A2_cube_px": held_cube - baseline["cube"],
        "train_overall_rmse_px": train_overall,
        "final_visual_robust_cost": visual_cost,
        "final_fk_robust_cost": fk_cost,
        "final_fk_robust_cost_fraction": fk_fraction,
        "mean_nfev": nfev,
        "rolled_back_runs": rolled_back,
        "table1_json": str(Path(payload["_source_json"]).relative_to(ROOT)),
    }


def _baseline_a2(path: Path) -> dict:
    payload = json.loads(path.read_text())
    runs = payload["rows"]["A2"]["runs"]
    return {
        "overall": _mean(_metric(run, "heldout", "overall") for run in runs),
        "board": _mean(_metric(run, "heldout", "board") for run in runs),
        "cube": _mean(_metric(run, "heldout", "cube") for run in runs),
    }


def _run_one(args: argparse.Namespace, scale: float) -> dict:
    original_mm = table1.SIGMA_FK_MM
    original_deg = table1.SIGMA_FK_DEG
    try:
        table1.SIGMA_FK_MM = float(original_mm) * scale
        table1.SIGMA_FK_DEG = float(original_deg) * scale
        run_dir = Path(args.out_dir) / f"scale_{scale:g}x"
        argv = [
            "--root_folder", args.root_folder,
            "--intrinsics_dir", args.intrinsics_dir,
            "--calib_dir", args.calib_dir,
            "--include_sets", args.include_sets,
            "--split_seed", str(args.split_seed),
            "--min_train_eih_cube_events", str(args.min_train_eih_cube_events),
            "--rows", "A4",
            "--num_inits", str(args.num_inits),
            "--observation-manifest", args.observation_manifest,
            "--observation-filter-policy", args.observation_filter_policy,
            "--out_dir", str(run_dir),
        ]
        if args.allow_relocated_session_root:
            argv.append("--allow-relocated-session-root")
        table1.main(argv)
    finally:
        table1.SIGMA_FK_MM = original_mm
        table1.SIGMA_FK_DEG = original_deg

    json_path = Path(args.out_dir) / f"scale_{scale:g}x" / "table1_methods.json"
    payload = json.loads(json_path.read_text())
    payload["_source_json"] = str(json_path)
    return payload


def _write(rows: list[dict], args: argparse.Namespace, baseline: dict) -> None:
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    csv_path = out_dir / "fk_factor_sensitivity.csv"
    fieldnames = list(rows[0])
    with csv_path.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames, lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)

    json_path = out_dir / "fk_factor_sensitivity.json"
    json_path.write_text(json.dumps({
        "artifact_schema": "fk_factor_covariance_scale_sensitivity_v1",
        "status": "preflight_simulation_prior_sensitivity",
        "external_ground_truth_used": False,
        "row": "A4",
        "fixed_inputs": {
            "root_folder": args.root_folder,
            "intrinsics_dir": args.intrinsics_dir,
            "observation_manifest": args.observation_manifest,
            "observation_filter_policy": args.observation_filter_policy,
            "allow_relocated_session_root": args.allow_relocated_session_root,
            "include_sets": args.include_sets,
            "split_seed": args.split_seed,
            "num_inits": args.num_inits,
        },
        "baseline_A2_heldout_rmse_px": baseline,
        "scale_semantics": (
            "preflight_std_scale multiplies both translation and rotation "
            "Simulation-prior standard deviations; smaller scale makes the "
            "FK factor stronger"),
        "rows": rows,
    }, indent=2) + "\n")

    lines = [
        "# FK Factor Sensitivity (Preflight)",
        "",
        "목적: 8/3 피드백 #4, 즉 \"카메라 관측 수가 많아 FK 항이 묻히는가\"에 "
        "답하기 위한 preflight 분석이다.",
        "",
        "이 실험은 canonical Table 1을 덮어쓰지 않는다. Session04 manifest, split, "
        "solver, A4 row를 고정하고 Simulation prior covariance의 표준편차 scale만 "
        "바꾼다. `preflight_std_scale < 1`은 FK factor를 더 강하게, `> 1`은 더 약하게 "
        "넣는다는 뜻이다.",
        "",
        "> External GT를 사용하지 않았으므로 이 결과는 물리 정확도 우월성 근거가 "
        "아니라 FK factor 영향도 점검이다.",
        "",
        f"- A2 held-out baseline: overall `{baseline['overall']:.4f}` px, "
        f"board `{baseline['board']:.4f}` px, cube `{baseline['cube']:.4f}` px",
        f"- Scales: {', '.join(str(row['preflight_std_scale']) + 'x' for row in rows)}",
        f"- Runs per scale: {args.num_inits}",
        "",
        "| Std scale | FK weight approx | trans std mm | rot std deg | "
        "A4 held-out overall | board | cube | Δoverall vs A2 | FK cost fraction | "
        "Converged |",
        "| ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |",
    ]
    for row in rows:
        lines.append(
            f"| {row['preflight_std_scale']:.2f}x | "
            f"{row['approx_fk_cost_weight_multiplier']:.2f}x | "
            f"{row['translation_std_mm']:.3f} | "
            f"{row['rotation_std_deg']:.3f} | "
            f"{row['heldout_overall_rmse_px']:.4f} | "
            f"{row['heldout_board_rmse_px']:.4f} | "
            f"{row['heldout_cube_rmse_px']:.4f} | "
            f"{row['delta_vs_A2_overall_px']:+.4f} | "
            f"{100.0 * row['final_fk_robust_cost_fraction']:.4f}% | "
            f"{row['converged_runs']}/{row['total_runs']} |")

    best = min(rows, key=lambda row: row["heldout_overall_rmse_px"])
    strongest = min(rows, key=lambda row: row["preflight_std_scale"])
    weakest = max(rows, key=lambda row: row["preflight_std_scale"])
    lines.extend([
        "",
        "## 해석",
        "",
        f"- 이 scale sweep 안에서 가장 낮은 A4 held-out overall은 "
        f"`{best['preflight_std_scale']:.2f}x`의 "
        f"`{best['heldout_overall_rmse_px']:.4f}` px다.",
        f"- 가장 강한 FK 설정 `{strongest['preflight_std_scale']:.2f}x`와 가장 약한 "
        f"설정 `{weakest['preflight_std_scale']:.2f}x` 사이의 held-out overall 차이는 "
        f"`{strongest['heldout_overall_rmse_px'] - weakest['heldout_overall_rmse_px']:+.4f}` px다.",
        "- 따라서 현재 데이터에서는 FK factor가 완전히 무시된다고 보기는 어렵지만, "
        "A2 대비 A4의 내부 held-out 차이는 매우 작아 최종 우월성 claim으로 쓰기에는 부족하다.",
        "- 이 결과는 #4에 대한 개선된 답변이다. 단순 residual 개수나 FK cost fraction만 "
        "보지 않고, covariance scale을 바꿨을 때 출력 지표가 실제로 움직이는지도 같이 본다.",
        "",
        "## 남은 것",
        "",
        "- measured FK covariance가 없으므로 여전히 preflight다.",
        "- 다음주 Independent External GT 이후 A2/A4 물리 정확도 비교로 최종 FK claim을 확정한다.",
    ])
    (out_dir / "FK_FACTOR_SENSITIVITY.md").write_text("\n".join(lines) + "\n")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--root_folder", default="data/session04/calib_train")
    parser.add_argument("--intrinsics_dir", default="intrinsics")
    parser.add_argument("--calib_dir", default="data/session04/calib_out")
    parser.add_argument("--include_sets", default="0-12")
    parser.add_argument("--split_seed", type=int, default=20260731)
    parser.add_argument("--min_train_eih_cube_events", type=int, default=3)
    parser.add_argument("--num_inits", type=int, default=3)
    parser.add_argument(
        "--observation-manifest",
        default=("data/session04/calib_out/capture_filter/"
                 "Step2b_observation_manifest.json"))
    parser.add_argument("--observation-filter-policy", default="standard")
    parser.add_argument(
        "--allow-relocated-session-root",
        action=argparse.BooleanOptionalAction,
        default=True,
        help=("Replay a manifest produced in another checkout after remapping "
              "only the recorded path prefix. Every SHA-256 is still checked."))
    parser.add_argument("--out_dir", default=str(DEFAULT_OUT_DIR))
    parser.add_argument(
        "--scales",
        default=",".join(str(value) for value in DEFAULT_SCALES),
        help="Comma-separated FK prior std scales. Smaller means stronger FK.")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    scales = [float(raw) for raw in args.scales.split(",") if raw.strip()]
    if not scales or any(scale <= 0.0 for scale in scales):
        raise ValueError("all scales must be positive")
    baseline = _baseline_a2(ROOT / "CP_result/session04/late_table1/table1_methods.json")
    rows = []
    for scale in scales:
        print(f"[SCALE] {scale:g}x", flush=True)
        payload = _run_one(args, scale)
        rows.append(_row_summary(payload, scale, baseline))
    rows.sort(key=lambda row: row["preflight_std_scale"])
    _write(rows, args, baseline)
    print(f"[DONE] {Path(args.out_dir) / 'FK_FACTOR_SENSITIVITY.md'}")


if __name__ == "__main__":
    main()
