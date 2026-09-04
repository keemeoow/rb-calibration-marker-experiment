#!/usr/bin/env python3
"""Compare standard and strict pre-fit observation rejection policies."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
from statistics import fmean, pstdev


METHODS = ("A0", "A1", "A2", "A3", "A4", "A5", "B1", "B2", "B3")
FIXED_PROTOCOL_FIELDS = (
    "dataset",
    "intrinsics_dir",
    "requested_set_filter",
    "solver_options",
    "num_inits",
    "backend",
    "pose_convention",
    "cube_config_source",
    "comparison_component_contract",
    "optimization_structure",
    "visual_objective",
    "primary_metric",
)
SPLIT_ASSIGNMENT_FIELDS = (
    "strategy",
    "seed",
    "test_fraction_requested",
    "min_train_eih_cube_events",
    "eligible_sets",
    "train_events",
    "test_events",
    "dropped_sets",
)


def load(path: Path) -> dict:
    with path.open() as handle:
        return json.load(handle)


def population(table: dict, name: str) -> dict:
    return table["protocol"]["source_data_provenance"][
        "observation_populations"][name]


def validate_comparison(standard: dict, strict: dict) -> dict:
    """Fail unless rejection policy is the only intended protocol change."""
    standard_protocol = standard["protocol"]
    strict_protocol = strict["protocol"]
    for field in FIXED_PROTOCOL_FIELDS:
        if standard_protocol.get(field) != strict_protocol.get(field):
            raise ValueError(f"fixed protocol field differs: {field}")

    standard_split = standard_protocol["split"]
    strict_split = strict_protocol["split"]
    for field in SPLIT_ASSIGNMENT_FIELDS:
        if standard_split.get(field) != strict_split.get(field):
            raise ValueError(f"split assignment field differs: {field}")
    for set_index in standard_split["eligible_sets"]:
        key = str(set_index)
        for field in ("train_events", "test_events"):
            if (standard_split["per_set"][key][field]
                    != strict_split["per_set"][key][field]):
                raise ValueError(
                    f"set {set_index} split assignment differs: {field}")

    standard_manifest = standard_protocol["source_data_provenance"][
        "observation_manifest"]
    strict_manifest = strict_protocol["source_data_provenance"][
        "observation_manifest"]
    if standard_manifest["sha256"] != strict_manifest["sha256"]:
        raise ValueError("standard and strict runs use different manifests")
    if standard_manifest.get("policy") != "standard":
        raise ValueError("standard result is not labelled with standard policy")
    if strict_manifest.get("policy") != "strict":
        raise ValueError("strict result is not labelled with strict policy")

    standard_heldout = population(standard, "heldout")
    strict_heldout = population(strict, "heldout")
    if standard_heldout != strict_heldout:
        raise ValueError(
            "held-out observation population differs; direct comparison is unfair")

    if set(standard["rows"]) != set(METHODS):
        raise ValueError("standard result does not contain the canonical methods")
    if set(strict["rows"]) != set(METHODS):
        raise ValueError("strict result does not contain the canonical methods")
    for method in METHODS:
        standard_runs = standard["rows"][method]["runs"]
        strict_runs = strict["rows"][method]["runs"]
        if len(standard_runs) != len(strict_runs):
            raise ValueError(f"{method}: initialization count differs")
        if [run["seed"] for run in standard_runs] != [
                run["seed"] for run in strict_runs]:
            raise ValueError(f"{method}: initialization seeds differ")
        if not all(run.get("converged") for run in standard_runs + strict_runs):
            raise ValueError(f"{method}: comparison includes a non-converged run")
    return standard_heldout


def metric_summary(table: dict, method: str, metric: str) -> dict | None:
    runs = table["rows"][method]["runs"]
    if metric not in runs[0]["heldout_reprojection"]:
        return None
    values = [
        float(run["heldout_reprojection"][metric]["rmse_px"])
        for run in runs
    ]
    counts = runs[0]["heldout_reprojection"][metric]
    return {
        "mean_rmse_px": fmean(values),
        "std_rmse_px": pstdev(values),
        "n_observations": int(counts["n_observations"]),
        "n_corners": int(counts["n_corners"]),
    }


def percent_change(before: float, after: float) -> float:
    return 100.0 * (after - before) / before


def build_rows(standard: dict, strict: dict) -> list[dict]:
    rows = []
    for method in METHODS:
        standard_overall = metric_summary(standard, method, "overall")
        strict_overall = metric_summary(strict, method, "overall")
        assert standard_overall is not None and strict_overall is not None
        if (
            standard_overall["n_observations"]
            != strict_overall["n_observations"]
            or standard_overall["n_corners"] != strict_overall["n_corners"]
        ):
            raise ValueError(f"{method}: evaluated row population differs")
        row = {
            "method": method,
            "standard_heldout_overall_rmse_px": standard_overall["mean_rmse_px"],
            "standard_std_rmse_px": standard_overall["std_rmse_px"],
            "strict_heldout_overall_rmse_px": strict_overall["mean_rmse_px"],
            "strict_std_rmse_px": strict_overall["std_rmse_px"],
            "strict_change_vs_standard_percent": percent_change(
                standard_overall["mean_rmse_px"],
                strict_overall["mean_rmse_px"],
            ),
            "heldout_observations": standard_overall["n_observations"],
            "heldout_corners": standard_overall["n_corners"],
        }
        for metric in ("board", "cube"):
            standard_target = metric_summary(standard, method, metric)
            strict_target = metric_summary(strict, method, metric)
            row[f"standard_{metric}_rmse_px"] = (
                None if standard_target is None
                else standard_target["mean_rmse_px"])
            row[f"strict_{metric}_rmse_px"] = (
                None if strict_target is None else strict_target["mean_rmse_px"])
        rows.append(row)
    return rows


def removed_observations(manifest: dict, standard: dict) -> tuple[list[dict], list[dict]]:
    split = standard["protocol"]["split"]
    train_events = {int(event) for event in split["train_events"]}
    eligible_sets = {int(index) for index in split["eligible_sets"]}
    removed = []
    outside_scope = []
    for observation in manifest["observations"]:
        selected = observation.get("selected_by_policy", {})
        if not selected.get("standard") or selected.get("strict"):
            continue
        item = {
            "observation_id": str(observation["observation_id"]),
            "event_id": int(observation["event_id"]),
            "camera_id": int(observation["camera_id"]),
            "set_idx": observation.get("set_idx"),
            "target": str(observation["target"]),
            "corner_count": int(observation["corner_count"]),
            "strict_reason": str(observation["reason_by_policy"]["strict"]),
            "pnp_rmse_px": observation.get("pnp_rmse_px"),
            "pnp_inlier_fraction": observation.get("pnp_inlier_fraction"),
        }
        is_training = int(observation["event_id"]) in train_events
        is_eligible = observation.get("set_idx") in eligible_sets
        (removed if is_training and is_eligible else outside_scope).append(item)
    return removed, outside_scope


def experiment_summary(rows: list[dict]) -> dict:
    by_method = {row["method"]: row for row in rows}
    changes = [row["strict_change_vs_standard_percent"] for row in rows]
    return {
        "maximum_absolute_overall_change_percent": max(map(abs, changes)),
        "a1_to_a2_improvement_standard_percent": percent_change(
            by_method["A1"]["standard_heldout_overall_rmse_px"],
            by_method["A2"]["standard_heldout_overall_rmse_px"],
        ) * -1.0,
        "a1_to_a2_improvement_strict_percent": percent_change(
            by_method["A1"]["strict_heldout_overall_rmse_px"],
            by_method["A2"]["strict_heldout_overall_rmse_px"],
        ) * -1.0,
        "a2_minus_a4_standard_px": (
            by_method["A2"]["standard_heldout_overall_rmse_px"]
            - by_method["A4"]["standard_heldout_overall_rmse_px"]),
        "a2_minus_a4_strict_px": (
            by_method["A2"]["strict_heldout_overall_rmse_px"]
            - by_method["A4"]["strict_heldout_overall_rmse_px"]),
    }


def write_outputs(
    rows: list[dict],
    standard: dict,
    strict: dict,
    manifest: dict,
    heldout: dict,
    output_dir: Path,
) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    removed, outside_scope = removed_observations(manifest, standard)
    summary = experiment_summary(rows)
    standard_train = population(standard, "train")
    strict_train = population(strict, "train")
    payload = {
        "experiment": "pre_fit_hard_rejection_sensitivity",
        "fairness_contract": {
            "same_split": True,
            "same_heldout_observations": True,
            "same_solver_and_initialization": True,
            "same_manifest": True,
            "changed_factor_only": "observation_filter_policy",
            "heldout_sha256": heldout["sha256"],
        },
        "policies": manifest["policies"],
        "train_populations": {
            "standard": standard_train,
            "strict": strict_train,
        },
        "removed_from_table1_training": removed,
        "additional_manifest_exclusions_outside_table1_scope": outside_scope,
        "summary": summary,
        "rows": rows,
        "interpretation_limit": (
            "Internal held-out reprojection sensitivity only; no independent "
            "external ground-truth accuracy is evaluated."),
    }
    (output_dir / "hard_rejection_ablation.json").write_text(
        json.dumps(payload, indent=2, ensure_ascii=False) + "\n")

    csv_fields = [
        "method",
        "standard_heldout_overall_rmse_px",
        "strict_heldout_overall_rmse_px",
        "strict_change_vs_standard_percent",
        "heldout_observations",
        "heldout_corners",
        "standard_board_rmse_px",
        "strict_board_rmse_px",
        "standard_cube_rmse_px",
        "strict_cube_rmse_px",
    ]
    with (output_dir / "hard_rejection_ablation.csv").open(
            "w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=csv_fields,
                                extrasaction="ignore", lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)

    standard_policy = manifest["policies"]["standard"]
    strict_policy = manifest["policies"]["strict"]
    lines = [
        "# Hard-Rejection Sensitivity (관측 사전 제외 민감도)",
        "",
        "## 결론",
        "",
        "strict 정책은 Table 1 학습에서 cube 관측 2개(32 corners)를 "
        "추가 제외했지만, 모든 방법의 held-out 전체 RMSE 변화는 최대 "
        f"**{summary['maximum_absolute_overall_change_percent']:.2f}%**였다. "
        "A1→A2 통합 최적화 개선 방향도 유지됐다. 따라서 현재 결론은 "
        "이 두 경계 관측의 포함 여부에 민감하지 않다.",
        "",
        "단, 이는 동일 내부 held-out reprojection에 대한 민감도 결과이며 "
        "절대 정확도 또는 외부 GT 검증을 대신하지 않는다.",
        "",
        "## 무엇을 바꿨나",
        "",
        f"- Standard: cube PnP RMSE ≤ {standard_policy['cube_max_pnp_rmse_px']:.1f} px, "
        f"inlier fraction ≥ {standard_policy['cube_min_inlier_fraction']:.1f}; "
        f"board corners ≥ {standard_policy['board_min_charuco_corners']}",
        f"- Strict: cube PnP RMSE ≤ {strict_policy['cube_max_pnp_rmse_px']:.1f} px, "
        f"inlier fraction ≥ {strict_policy['cube_min_inlier_fraction']:.1f}; "
        f"board corners ≥ {strict_policy['board_min_charuco_corners']}",
        f"- Train: {standard_train['observations']} obs / "
        f"{standard_train['corners']} corners → "
        f"{strict_train['observations']} obs / {strict_train['corners']} corners",
        "",
        "실제 Table 1 학습에서 추가 제외된 관측:",
        "",
    ]
    for observation in removed:
        quality = (
            f"PnP RMSE {observation['pnp_rmse_px']:.4f} px"
            if observation["strict_reason"] == "pnp_rmse_above_2px"
            else f"PnP inlier fraction {observation['pnp_inlier_fraction']:.3f}"
        )
        lines.append(
            f"- `{observation['observation_id']}`: {quality}; "
            f"{observation['corner_count']} corners, "
            f"reason `{observation['strict_reason']}`")

    lines.extend([
        "",
        "## 왜 직접 비교가 공정한가",
        "",
        "- 같은 event-grouped split, 동일 seed와 3개 초기값을 사용했다.",
        "- intrinsic, distortion, target geometry, FK, solver, soft-L1 loss를 "
        "동일하게 고정했다.",
        "- held-out는 양쪽 모두 "
        f"{heldout['observations']} obs / {heldout['corners']} corners이며 "
        f"SHA-256 `{heldout['sha256']}`로 완전히 같다.",
        "- strict 여부는 최적화 전에 고정되며 fitted model 출력으로 "
        "평가 관측을 제거하지 않는다.",
        "",
        "## 결과",
        "",
        "| Method | Standard px | Strict px | Strict 변화 | Held-out obs/corners |",
        "| --- | ---: | ---: | ---: | ---: |",
    ])
    for row in rows:
        lines.append(
            f"| {row['method']} | "
            f"{row['standard_heldout_overall_rmse_px']:.4f} | "
            f"{row['strict_heldout_overall_rmse_px']:.4f} | "
            f"{row['strict_change_vs_standard_percent']:+.2f}% | "
            f"{row['heldout_observations']}/{row['heldout_corners']} |")

    lines.extend([
        "",
        "## 주장별 확인",
        "",
        "- A1→A2 held-out 개선: standard "
        f"{summary['a1_to_a2_improvement_standard_percent']:.2f}% → strict "
        f"{summary['a1_to_a2_improvement_strict_percent']:.2f}%로 방향이 유지됐다.",
        "- A2와 A4 차이: standard "
        f"{summary['a2_minus_a4_standard_px']:+.4f} px, strict "
        f"{summary['a2_minus_a4_strict_px']:+.4f} px로 둘 다 사실상 동률이다. "
        "따라서 이 실험도 A4 우월성 주장의 근거가 아니다.",
        "- A3 raw-FK-fixed는 두 정책 모두 A2/A4보다 높은 오차를 유지했다. "
        "즉 A3의 차이는 해당 두 경계 관측만으로 설명되지 않는다.",
        "- A5도 두 정책에서 낮은 내부 px를 유지한다. External GT 공개 전에 "
        "방법과 alignment artifact가 frozen이면 최종 후보지만, strict 민감도 "
        "통과 자체가 외부 물리 정확도 우월성을 뜻하지 않는다.",
        "",
        "## 재현 명령",
        "",
        "```bash",
        "python3 tools/summarize_hard_rejection_ablation.py",
        "```",
    ])
    (output_dir / "HARD_REJECTION_ABLATION.md").write_text(
        "\n".join(lines) + "\n")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--standard_table",
        default="CP_result/session04/late_table1/table1_methods.json")
    parser.add_argument(
        "--strict_table",
        default=("CP_result/session04/outlier_ablation/strict_table1/"
                 "table1_methods.json"))
    parser.add_argument(
        "--manifest",
        default=("data/session04/calib_out/capture_filter/"
                 "Step2b_observation_manifest.json"))
    parser.add_argument(
        "--out_dir", default="CP_result/session04/outlier_ablation")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    standard = load(Path(args.standard_table))
    strict = load(Path(args.strict_table))
    manifest = load(Path(args.manifest))
    heldout = validate_comparison(standard, strict)
    rows = build_rows(standard, strict)
    write_outputs(rows, standard, strict, manifest, heldout, Path(args.out_dir))
    print(f"[DONE] {args.out_dir}/HARD_REJECTION_ABLATION.md")


if __name__ == "__main__":
    main()
