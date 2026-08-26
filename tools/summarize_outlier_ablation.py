#!/usr/bin/env python3
"""Compare fixed-population soft-L1 and linear-loss calibration runs."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
from statistics import fmean


METHODS = ("A0", "A1", "A2", "A3", "A4", "B1", "B2", "B3")


def load(path: Path) -> dict:
    with path.open() as handle:
        return json.load(handle)


def mean_heldout(table: dict, method: str) -> float:
    return fmean(
        float(run["heldout_reprojection"]["overall"]["rmse_px"])
        for run in table["rows"][method]["runs"])


def cross_rows(payload: dict) -> dict[str, dict]:
    return {str(row["method"]): row for row in payload["summary"]}


def same_population(soft: dict, linear: dict) -> dict:
    soft_pop = soft["protocol"]["source_data_provenance"][
        "observation_populations"]
    linear_pop = linear["protocol"]["source_data_provenance"][
        "observation_populations"]
    for split in ("eligible", "train", "heldout"):
        if soft_pop[split]["sha256"] != linear_pop[split]["sha256"]:
            raise ValueError(f"{split} observation population differs")
    if soft["protocol"]["split"] != linear["protocol"]["split"]:
        raise ValueError("soft-L1 and linear runs use different splits")
    return soft_pop


def build_rows(soft_table: dict, linear_table: dict,
               soft_cross: dict, linear_cross: dict) -> list[dict]:
    soft_by_method = cross_rows(soft_cross)
    linear_by_method = cross_rows(linear_cross)
    rows = []
    for method in METHODS:
        soft_heldout = mean_heldout(soft_table, method)
        linear_heldout = mean_heldout(linear_table, method)
        soft = soft_by_method[method]
        linear = linear_by_method[method]
        rows.append({
            "method": method,
            "soft_l1_heldout_overall_rmse_px": soft_heldout,
            "linear_heldout_overall_rmse_px": linear_heldout,
            "soft_l1_change_vs_linear_percent": (
                100.0 * (soft_heldout - linear_heldout) / linear_heldout),
            "soft_l1_fixed_to_fixed_cube_transfer_rmse_px": soft[
                "fixed_to_fixed_cube_cross_view_pixel_transfer_rmse_px_mean"],
            "linear_fixed_to_fixed_cube_transfer_rmse_px": linear[
                "fixed_to_fixed_cube_cross_view_pixel_transfer_rmse_px_mean"],
            "soft_l1_gripper_to_fixed_cube_transfer_rmse_px": soft[
                "gripper_to_fixed_cube_cross_view_pixel_transfer_rmse_px_mean"],
            "linear_gripper_to_fixed_cube_transfer_rmse_px": linear[
                "gripper_to_fixed_cube_cross_view_pixel_transfer_rmse_px_mean"],
        })
    return rows


def write(rows: list[dict], populations: dict, output_dir: Path) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    csv_path = output_dir / "outlier_loss_ablation.csv"
    with csv_path.open("w", newline="") as handle:
        writer = csv.DictWriter(
            handle, fieldnames=list(rows[0]), lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)

    lines = [
        "# Outlier Loss Ablation (이상치 손실함수 대조)",
        "",
        "이 실험은 사전 품질 마스크·train/test split·관측 코너를 그대로 "
        "고정하고 최종 최적화 loss만 `soft_l1`과 `linear`로 바꿨다. "
        "따라서 차이는 최적화 중 soft weighting 효과이며, hard rejection "
        "threshold 변화 효과가 아니다.",
        "",
        f"- Train: {populations['train']['observations']} observations, "
        f"{populations['train']['corners']} corners, "
        f"SHA-256 `{populations['train']['sha256']}`",
        f"- Held-out: {populations['heldout']['observations']} observations, "
        f"{populations['heldout']['corners']} corners, "
        f"SHA-256 `{populations['heldout']['sha256']}`",
        "",
        "| Method | Held-out soft-L1 px | Held-out linear px | soft-L1 change | "
        "Fixed→Fixed cube soft/linear px | Gripper→Fixed cube soft/linear px |",
        "| --- | ---: | ---: | ---: | ---: | ---: |",
    ]
    for row in rows:
        lines.append(
            f"| {row['method']} | "
            f"{row['soft_l1_heldout_overall_rmse_px']:.4f} | "
            f"{row['linear_heldout_overall_rmse_px']:.4f} | "
            f"{row['soft_l1_change_vs_linear_percent']:+.2f}% | "
            f"{row['soft_l1_fixed_to_fixed_cube_transfer_rmse_px']:.4f} / "
            f"{row['linear_fixed_to_fixed_cube_transfer_rmse_px']:.4f} | "
            f"{row['soft_l1_gripper_to_fixed_cube_transfer_rmse_px']:.4f} / "
            f"{row['linear_gripper_to_fixed_cube_transfer_rmse_px']:.4f} |")
    improved = [
        f"{row['method']} {abs(row['soft_l1_change_vs_linear_percent']):.2f}%"
        for row in rows
        if row["soft_l1_change_vs_linear_percent"] < 0.0
    ]
    degraded = [
        f"{row['method']} {row['soft_l1_change_vs_linear_percent']:.2f}%"
        for row in rows
        if row["soft_l1_change_vs_linear_percent"] > 0.0
    ]
    lines.extend([
        "",
        "해석: held-out 전체 오차 기준으로 soft-L1이 linear보다 낮은 "
        f"방법은 {', '.join(improved) if improved else '없음'}이고, 높은 "
        f"방법은 {', '.join(degraded) if degraded else '없음'}이다. "
        "따라서 robust loss가 모든 조건을 일괄 개선한다고 주장할 수 없으며, "
        "방법별·표적별 결과를 함께 보고해야 한다.",
        "",
        "한계: 이 결과는 이미 적용된 사전 PnP 품질 마스크를 고정한 실험이다. "
        "프레임/관측 hard rejection 자체의 민감도는 threshold를 사전 등록한 "
        "별도 실험으로 확인해야 한다.",
    ])
    (output_dir / "OUTLIER_LOSS_ABLATION.md").write_text(
        "\n".join(lines) + "\n")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--soft_table",
        default="CP_result/session02/late_table1/table1_methods.json")
    parser.add_argument(
        "--linear_table",
        default=("CP_result/session02/outlier_ablation/linear_table1/"
                 "table1_methods.json"))
    parser.add_argument(
        "--soft_cross",
        default=("CP_result/session02/cross_target_evaluation/"
                 "cross_target_evaluation.json"))
    parser.add_argument(
        "--linear_cross",
        default=("CP_result/session02/outlier_ablation/linear_cross_target/"
                 "cross_target_evaluation.json"))
    parser.add_argument(
        "--out_dir", default="CP_result/session02/outlier_ablation")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    soft_table = load(Path(args.soft_table))
    linear_table = load(Path(args.linear_table))
    populations = same_population(soft_table, linear_table)
    rows = build_rows(
        soft_table, linear_table,
        load(Path(args.soft_cross)), load(Path(args.linear_cross)))
    write(rows, populations, Path(args.out_dir))
    print(f"[DONE] {args.out_dir}/OUTLIER_LOSS_ABLATION.md")


if __name__ == "__main__":
    main()
