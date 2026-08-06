#!/usr/bin/env python3
"""D2 — evaluate the adopted anchored configuration on the canonical event split.

Why this exists
---------------
D1 selected ``A2@lambda=3 + residual correction`` as the adopted configuration,
but it measured it on a *position* hold-out with an FK-proxy position metric.
Table 1's other columns (``N_reg``, ``e_e2e``, ``e_cross``, ``e_reproj``) come
from the canonical **event** split.  A soft anchor changes the solve itself, so
those columns cannot be copied from the A2 row the way the correction rows can
-- they were marked unmeasured.  This runner fills them.

It deliberately does **not** touch ``CP_ablation_7row.py``.  That runner carries
a validated seven-row contract and a noise-free sanity gate; adding an eighth
arm would mean re-validating it and desynchronising the stored artifacts.  D2
instead reuses that runner's data preparation verbatim -- same detection, same
event split, same train-only FK artifact, same initialization, same solver
options, same pre-fit path-evaluation mask -- and only swaps in the anchored
objective from ``CP_D1_fk_correction_2x2``.  A0-B3 numbers therefore stay
byte-identical to Table 1, and the anchored rows are directly comparable.

Reference arms (``A2`` at lambda=0 and ``A3``) are run alongside so every report
carries its own reproduction of the two Table 1 rows it sits between.  If those
do not match Table 1, the anchored numbers are not comparable either and the
script says so instead of quietly reporting.

Usage
-----
    PYTHONPATH= python3 CP_D2_anchored_event_split.py \
        --root_folder data/session --intrinsics_dir intrinsics \
        --calib_dir data/session/calib_out \
        --out_dir CP_result/D2_anchored_event_split
"""

from __future__ import annotations

import argparse
import csv
import json
import os
from typing import Dict, List, Mapping, Sequence

import numpy as np

import CP_ablation_7row as ab
import CP_common
from CP_ablation_schema import MAIN_ABLATION_CONDITIONS, UNIFIED_FREE_VARIABLES
from CP_D1_fk_correction_2x2 import solve_anchored_corner_reprojection
from calibration_path_evaluation import evaluate_paths_with_common_mask
from calibration_reprojection_backend import variable_keys


CANONICAL_SPLIT_SEEDS = (20260729, 20260730, 20260731, 20260732, 20260733)


def arm_specs(lambdas: Sequence[float]) -> List[dict]:
    """Reference A2/A3 rows plus one anchored arm per weight."""
    specs = [
        {"key": "A2", "row": "A2", "anchor_lambda": 0.0,
         "label": "reference: vision-estimated, no anchor (Table 1 A2)"},
        {"key": "A3", "row": "A3", "anchor_lambda": None,
         "label": "reference: FK-fixed (Table 1 A3)"},
    ]
    for value in lambdas:
        if float(value) == 0.0:
            continue
        specs.append({
            "key": f"A2@lam{value:g}", "row": "A2", "anchor_lambda": float(value),
            "label": f"vision-estimated + soft FK anchor lam={value:g} px/mm"})
    return specs


def run_arm(spec: Mapping, data, args, seed: int) -> dict:
    conditions = {c.row: c for c in MAIN_ABLATION_CONDITIONS}
    condition = conditions[spec["row"]]
    initial_state, init_diag = ab.make_initial_state(
        condition, data.train_obs, data.gripper, data.robot_T, data.K_map, data.D_map,
        data.board_gtc, data.board_initial, data.visual_cubes, data.fixed_cubes,
        data.fixed_gtc_initial)
    train_fit = ab.filter_observations(
        data.train_obs, condition, None, data.gripper, initial_state.cams)
    test_eval = ab.filter_observations(
        data.test_obs, condition, None, data.gripper, initial_state.cams)
    state, diag = solve_anchored_corner_reprojection(
        observations=train_fit,
        variable_keys_=variable_keys(UNIFIED_FREE_VARIABLES[spec["row"]], initial_state),
        reference_state=initial_state,
        robot_T=data.robot_T, K_map=data.K_map, D_map=data.D_map,
        gripper_cam_idx=data.gripper,
        anchor_targets=({} if spec["anchor_lambda"] is None else data.fixed_cubes),
        anchor_lambda=(0.0 if spec["anchor_lambda"] is None else float(spec["anchor_lambda"])),
        anchor_lever_mm=float(args.anchor_lever_mm),
        options=ab.canonical_solver_options(args),
        seed=int(seed),
        init_translation_mm=float(args.init_translation_mm),
        init_rotation_deg=float(args.init_rotation_deg),
    )
    train_metrics = ab.reprojection_metrics(
        train_fit, state, data.robot_T, data.K_map, data.D_map, data.gripper)
    test_metrics = ab.reprojection_metrics(
        test_eval, state, data.robot_T, data.K_map, data.D_map, data.gripper)
    path = evaluate_paths_with_common_mask(
        data.test_obs, state.cams, state.gtc, data.robot_T,
        data.gripper, data.K_map, data.D_map, data.path_evaluation_mask)
    path.pop("predicted_by_set", None)
    path.pop("per_cross_pair", None)
    path.pop("per_e2e_unit", None)
    return {
        "seed": int(seed),
        "converged": bool(diag["success"]),
        "N_reg": len(state.cams),
        "heldout_reprojection_overall_px": test_metrics["overall"]["rmse_px"],
        "heldout_reprojection_cube_px": (
            test_metrics["cube"]["rmse_px"] if "cube" in test_metrics else None),
        "heldout_reprojection_board_px": (
            test_metrics["board"]["rmse_px"] if "board" in test_metrics else None),
        "train_reprojection_overall_px": train_metrics["overall"]["rmse_px"],
        "e_e2e_translation_mm": path["e_e2e_translation_rmse_mm"],
        "e_e2e_rotation_deg": path["e_e2e_rotation_rmse_deg"],
        "e_cross_translation_mm": path["e_cross_translation_rmse_mm"],
        "e_cross_rotation_deg": path["e_cross_rotation_rmse_deg"],
        "jacobian_condition": diag["jacobian"]["jacobian_condition_number"],
        "n_parameters": diag["n_parameters"],
        "nfev": diag["nfev"],
        "anchor_rms_cube_displacement_mm": diag.get("anchor_rms_cube_displacement_mm"),
        "initialization": init_diag,
        "solver_status": diag["status"],
        # Frozen pose set, so cross-target evaluation can re-score this arm on the
        # shared cube corners without refitting (CP_cross_target_cube_eval.py).
        "transforms": ab.serialize_state(state),
    }


AGGREGATED_KEYS = (
    "N_reg", "heldout_reprojection_overall_px", "heldout_reprojection_cube_px",
    "heldout_reprojection_board_px", "e_e2e_translation_mm", "e_e2e_rotation_deg",
    "e_cross_translation_mm", "e_cross_rotation_deg", "jacobian_condition", "nfev",
    "anchor_rms_cube_displacement_mm",
)


def aggregate(per_split: Mapping[int, Mapping[str, List[dict]]],
              arm_keys: Sequence[str]) -> dict:
    """Split means first, then mean +- std across split means (Table 1 convention)."""
    out = {}
    for arm in arm_keys:
        split_means: Dict[str, List[float]] = {key: [] for key in AGGREGATED_KEYS}
        converged, total = 0, 0
        for seed in sorted(per_split):
            runs = per_split[seed][arm]
            total += len(runs)
            converged += sum(1 for r in runs if r["converged"])
            for key in AGGREGATED_KEYS:
                values = [r[key] for r in runs if r[key] is not None]
                if values:
                    split_means[key].append(float(np.mean(values)))
        entry = {"n_runs": total, "n_converged": converged,
                 "n_splits": len(per_split)}
        for key, values in split_means.items():
            entry[key] = {
                "mean": float(np.mean(values)) if values else None,
                "std": float(np.std(values)) if values else None,
            }
        out[arm] = entry
    return out


def paired_vs_reference(per_split, arm_keys, reference: str, metric: str) -> dict:
    """Per-split paired delta against a reference arm, on split means."""
    result = {}
    for arm in arm_keys:
        if arm == reference:
            continue
        deltas = []
        for seed in sorted(per_split):
            def mean_of(key):
                vals = [r[metric] for r in per_split[seed][key] if r[metric] is not None]
                return float(np.mean(vals)) if vals else None
            a, b = mean_of(arm), mean_of(reference)
            if a is not None and b is not None:
                deltas.append(a - b)
        values = np.asarray(deltas, dtype=np.float64)
        if values.size == 0:
            result[arm] = {"n_splits": 0}
            continue
        std = float(values.std(ddof=1)) if values.size > 1 else 0.0
        se = std / np.sqrt(values.size) if values.size > 1 and std > 0 else None
        result[arm] = {
            "n_splits": int(values.size),
            "mean": float(values.mean()),
            "std": float(values.std()),
            "se": se,
            "t": (float(values.mean() / se) if se else None),
            "splits_improved": int((values < 0).sum()),
        }
    return result


# Table 1 values are means over the five canonical split seeds, with the spread
# across split means.  A run over fewer seeds must be judged against that spread,
# not against the mean alone -- otherwise a perfectly consistent single split
# looks like a failure.
TABLE1_REFERENCE = {
    "A2": {"e_e2e_translation_mm": (16.1955, 1.2976),
           "e_cross_translation_mm": (38.9338, 3.4075),
           "heldout_reprojection_overall_px": (4.5075, 0.4387)},
    "A3": {"e_e2e_translation_mm": (16.1881, 1.2856),
           "e_cross_translation_mm": (38.0480, 3.5117),
           "heldout_reprojection_overall_px": (4.6230, 0.4338)},
}
REFERENCE_TOLERANCE_SIGMA = 2.0


def reference_agreement(summary: Mapping, n_splits: int) -> dict:
    """Compare the reproduced A2/A3 rows against Table 1, scaled by its spread.

    Detection carries run-to-run RANSAC variation, so exact equality is not
    expected even at five seeds; the test is whether the reproduction sits
    inside the split-to-split spread Table 1 itself reports.
    """
    report = {"n_splits_run": int(n_splits),
              "tolerance_sigma": REFERENCE_TOLERANCE_SIGMA,
              "note": ("Table 1 is a five-seed mean; a run over fewer seeds is judged "
                       "against Table 1's own split spread, not against its mean."),
              "arms": {}}
    worst = 0.0
    for arm, expected in TABLE1_REFERENCE.items():
        if arm not in summary:
            continue
        checks = {}
        for key, (value, spread) in expected.items():
            got = summary[arm][key]["mean"]
            sigma = (None if got is None or spread <= 0
                     else abs(got - value) / spread)
            if sigma is not None:
                worst = max(worst, sigma)
            checks[key] = {
                "table1_mean": value, "table1_split_std": spread,
                "reproduced": got,
                "abs_diff": (None if got is None else abs(got - value)),
                "sigma": sigma,
                "within_tolerance": (None if sigma is None
                                     else bool(sigma <= REFERENCE_TOLERANCE_SIGMA)),
            }
        report["arms"][arm] = checks
    report["worst_sigma"] = worst
    report["passed"] = bool(worst <= REFERENCE_TOLERANCE_SIGMA)
    return report


def write_outputs(result: Mapping, out_dir: str) -> None:
    os.makedirs(out_dir, exist_ok=True)
    with open(os.path.join(out_dir, "D2_anchored_event_split.json"), "w") as handle:
        json.dump(ab._jsonable(result), handle, indent=2)

    summary = result["summary"]
    arm_keys = result["arm_keys"]
    with open(os.path.join(out_dir, "D2_anchored_event_split.csv"), "w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(["arm", "anchor_lambda_px_per_mm", "n_splits", "n_runs",
                         "n_converged", "N_reg",
                         "e_reproj_overall_px_mean", "e_reproj_overall_px_std",
                         "e_e2e_mm_mean", "e_e2e_mm_std",
                         "e_cross_mm_mean", "e_cross_mm_std",
                         "jacobian_condition_mean"])
        lam = {s["key"]: s["anchor_lambda"] for s in result["arm_specs"]}
        for arm in arm_keys:
            e = summary[arm]
            writer.writerow([
                arm, lam[arm], e["n_splits"], e["n_runs"], e["n_converged"],
                e["N_reg"]["mean"],
                e["heldout_reprojection_overall_px"]["mean"],
                e["heldout_reprojection_overall_px"]["std"],
                e["e_e2e_translation_mm"]["mean"], e["e_e2e_translation_mm"]["std"],
                e["e_cross_translation_mm"]["mean"], e["e_cross_translation_mm"]["std"],
                e["jacobian_condition"]["mean"]])

    def fmt(value, digits=4):
        return "—" if value is None else f"{value:.{digits}f}"

    lam = {s["key"]: s["anchor_lambda"] for s in result["arm_specs"]}
    lines = [
        "# D2 — 채택 구성(soft anchor)을 canonical 이벤트 split에서 평가",
        "",
        "Table 1의 `‖` 칸(=anchored 행의 `N_reg`/`e_e2e`/`e_cross`/`e_reproj`)을 채우기 위한 "
        "실행이다. D1은 위치 hold-out·mm 지표였고 이 표는 **Table 1과 동일한 이벤트 split·"
        "동일 지표**다.",
        "",
        f"- `CP_common.ROBOT_POS_SCALE_PINNED` = **{result['robot_pos_scale']:.4f}** "
        f"({'로봇 원본 값 그대로' if result['robot_pos_scale'] == 1.0 else '병진 보정 적용'}). "
        "아래 재현 확인은 이 값이 Table 1과 같을 때만 의미가 있다 — 게이트의 허용치가 "
        "split 표준편차라서 스케일 불일치를 단독으로는 걸러내지 못한다.",
        f"- split seeds: {result['split_seeds']}, 각 split당 {result['num_inits']} initialization",
        f"- 총 {result['summary'][arm_keys[0]]['n_runs']} run/arm, "
        f"anchor lever {result['anchor_lever_mm']:.1f} mm, λ 단위 px/mm",
        "- 데이터 준비(검출·split·train 전용 FK artifact·초기화·solver 설정·path mask)는 "
        "`CP_ablation_7row.py`의 것을 그대로 재사용한다. 목적함수의 anchor 항만 다르다.",
        "",
        "## 이벤트 split 결과 (split mean의 mean±std)",
        "",
        "| arm | λ (px/mm) | N_reg | e_reproj overall (px) | e_e2e (mm/°) | e_cross (mm) | Jac cond | 수렴 |",
        "| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |",
    ]
    for arm in arm_keys:
        e = summary[arm]
        lam_text = "∞ (hard)" if lam[arm] is None else f"{lam[arm]:g}"
        lines.append(
            f"| {arm} | {lam_text} | {fmt(e['N_reg']['mean'], 0)} | "
            f"{fmt(e['heldout_reprojection_overall_px']['mean'])}±"
            f"{fmt(e['heldout_reprojection_overall_px']['std'])} | "
            f"{fmt(e['e_e2e_translation_mm']['mean'])}±{fmt(e['e_e2e_translation_mm']['std'])} / "
            f"{fmt(e['e_e2e_rotation_deg']['mean'])}±{fmt(e['e_e2e_rotation_deg']['std'])} | "
            f"{fmt(e['e_cross_translation_mm']['mean'])}±{fmt(e['e_cross_translation_mm']['std'])} | "
            f"{fmt(e['jacobian_condition']['mean'], 1)} | "
            f"{e['n_converged']}/{e['n_runs']} |")

    agree = result["reference_agreement"]
    lines += [
        "",
        f"## Table 1 재현 확인 — {'통과' if agree['passed'] else '실패'} "
        f"(최대 {agree['worst_sigma']:.2f}σ, 허용 {agree['tolerance_sigma']:.0f}σ)",
        "",
        "A2·A3는 Table 1과 같은 조건이므로 재현되어야 한다. 어긋나면 anchored 행도 "
        "Table 1과 비교할 수 없다. 검출에 RANSAC 변동이 있어 정확한 일치는 기대하지 않으며, "
        "**Table 1이 스스로 보고한 split 간 표준편차 안에 들어오는지**로 판정한다.",
        "",
        "| arm | 지표 | Table 1 (mean±split std) | 재현값 | 차이 | σ | 판정 |",
        "| --- | --- | ---: | ---: | ---: | ---: | :---: |",
    ]
    for arm, checks in agree["arms"].items():
        for key, check in checks.items():
            mark = "○" if check["within_tolerance"] else "✗"
            lines.append(
                f"| {arm} | {key} | {fmt(check['table1_mean'])}±"
                f"{fmt(check['table1_split_std'])} | {fmt(check['reproduced'])} | "
                f"{fmt(check['abs_diff'])} | {fmt(check['sigma'], 2)} | {mark} |")

    lines += [
        "",
        "## A2(λ=0) 대비 paired delta — split 단위, 음수가 개선",
        "",
        "| arm | e_reproj overall (px) | e_cross (mm) |",
        "| --- | ---: | ---: |",
    ]
    for arm in arm_keys:
        if arm == "A2":
            continue
        r = result["paired_vs_A2"]["heldout_reprojection_overall_px"].get(arm, {})
        c = result["paired_vs_A2"]["e_cross_translation_mm"].get(arm, {})
        def cell(d):
            if not d or d.get("mean") is None:
                return "—"
            return (f"{d['mean']:+.4f}±{d['std']:.4f}, t={fmt(d.get('t'), 2)}, "
                    f"{d['splits_improved']}/{d['n_splits']}")
        lines.append(f"| {arm} | {cell(r)} | {cell(c)} |")

    lines += [
        "",
        "## 해석 규칙",
        "",
        "- 이 표의 값은 held-out **이벤트**의 재투영·경로 일치도다. D1의 위치 hold-out "
        "mm 값과 같은 줄에서 비교하지 않는다.",
        "- 외부 GT가 없으므로 `e_e2e`·`e_cross`는 내부 일관성 지표이며 절대 정확도가 아니다.",
        "- anchored 행은 `CP_ablation_7row.py`의 7행 계약에 포함되지 않는 보충 실행이다. "
        "7행의 인과 비교표에 이 행을 끼워 넣지 않는다.",
        "",
    ]
    with open(os.path.join(out_dir, "D2_anchored_event_split.md"), "w") as handle:
        handle.write("\n".join(lines))


def parse_args():
    parser = argparse.ArgumentParser(
        description="Evaluate the anchored configuration on the canonical event split")
    parser.add_argument("--root_folder", default="data/session")
    parser.add_argument("--intrinsics_dir", default="intrinsics")
    parser.add_argument("--calib_dir", default="data/session/calib_out")
    parser.add_argument("--out_dir", default="CP_result/D2_anchored_event_split")
    parser.add_argument("--lambdas", default="3",
                        help="Anchor weights in px/mm; the adopted configuration is 3.")
    parser.add_argument("--anchor_lever_mm", type=float, default=29.5)
    parser.add_argument("--split_seeds", default=",".join(str(s) for s in CANONICAL_SPLIT_SEEDS),
                        help="Canonical repeated-split seeds, matching Table 1.")
    parser.add_argument("--test_fraction", type=float, default=0.2)
    parser.add_argument("--min_train_eih_cube_events", type=int, default=3)
    parser.add_argument("--num_inits", type=int, default=3)
    parser.add_argument("--init_translation_mm", type=float, default=5.0)
    parser.add_argument("--init_rotation_deg", type=float, default=1.0)
    parser.add_argument("--max_nfev", type=int, default=300)
    parser.add_argument("--tol", type=float, default=1e-8)
    parser.add_argument("--rotation_scale_rad", type=float, default=1.0)
    parser.add_argument("--translation_scale_m", type=float, default=1.0)
    parser.add_argument("--x_scale_mode", choices=["unit", "jac"], default="jac")
    parser.add_argument("--loss", choices=["huber", "soft_l1", "linear"], default="soft_l1")
    parser.add_argument("--f_scale_px", type=float, default=2.0)
    parser.add_argument("--image_scale", type=float, default=1.0)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    lambdas = [float(x) for x in str(args.lambdas).split(",") if x.strip()]
    seeds = [int(x) for x in str(args.split_seeds).split(",") if x.strip()]
    specs = arm_specs(lambdas)
    arm_keys = [s["key"] for s in specs]
    print(f"[D2] arms: {arm_keys}")
    print(f"[D2] split seeds: {seeds}, inits per split: {args.num_inits}")

    per_split: Dict[int, Dict[str, List[dict]]] = {}
    for split_seed in seeds:
        args.split_seed = split_seed
        print(f"\n[D2] split seed {split_seed}: preparing data", flush=True)
        data = ab.prepare_ablation_data(args)
        print(f"      train {len(data.split['train_events'])} events / "
              f"test {len(data.split['test_events'])} events", flush=True)
        per_split[split_seed] = {}
        for spec in specs:
            runs = [run_arm(spec, data, args, seed) for seed in range(int(args.num_inits))]
            per_split[split_seed][spec["key"]] = runs
            mean_px = float(np.mean([r["heldout_reprojection_overall_px"] for r in runs]))
            mean_cross = float(np.mean([r["e_cross_translation_mm"] for r in runs]))
            print(f"      {spec['key']:12s} reproj {mean_px:7.4f} px  "
                  f"e_cross {mean_cross:8.4f} mm  "
                  f"conv {sum(r['converged'] for r in runs)}/{len(runs)}", flush=True)

    summary = aggregate(per_split, arm_keys)
    result = {
        "experiment": "D2_anchored_configuration_on_canonical_event_split",
        "purpose": ("fill the Table 1 cells that a soft anchor invalidates, using the "
                    "canonical event split and the canonical metrics"),
        "reuses": "CP_ablation_7row.prepare_ablation_data (detection, split, FK artifact, "
                  "initialization, solver options, path-evaluation mask)",
        "split_seeds": seeds,
        "num_inits": int(args.num_inits),
        "anchor_lever_mm": float(args.anchor_lever_mm),
        "anchor_lambdas_px_per_mm": lambdas,
        "arm_specs": specs,
        "arm_keys": arm_keys,
        "solver_options": ab.canonical_solver_options(args).to_dict(),
        # Record the load-time robot translation scale.  Results produced under
        # different values are not comparable -- the FK cube poses they anchor
        # against differ by ~12mm -- and the reference_agreement gate below is
        # too coarse to catch a scale mismatch on its own.
        "robot_pos_scale": CP_common.robot_pos_scale(),
        "summary": summary,
        "reference_agreement": reference_agreement(summary, len(seeds)),
        "paired_vs_A2": {
            metric: paired_vs_reference(per_split, arm_keys, "A2", metric)
            for metric in ("heldout_reprojection_overall_px", "e_cross_translation_mm",
                           "e_e2e_translation_mm")
        },
        "per_split": per_split,
    }
    write_outputs(result, args.out_dir)

    print("\n[D2] canonical event split (split mean ± std across splits)")
    print(f"{'arm':12s} {'N_reg':>6s} {'reproj px':>16s} {'e_e2e mm':>16s} {'e_cross mm':>16s}")
    for arm in arm_keys:
        e = summary[arm]
        print(f"{arm:12s} {e['N_reg']['mean']:6.1f} "
              f"{e['heldout_reprojection_overall_px']['mean']:9.4f}±"
              f"{e['heldout_reprojection_overall_px']['std']:.4f} "
              f"{e['e_e2e_translation_mm']['mean']:9.4f}±{e['e_e2e_translation_mm']['std']:.4f} "
              f"{e['e_cross_translation_mm']['mean']:9.4f}±{e['e_cross_translation_mm']['std']:.4f}")
    agree = result["reference_agreement"]
    verdict = "PASS" if agree["passed"] else "FAIL"
    print(f"\n[D2] Table 1 재현 확인: {verdict} "
          f"(최대 {agree['worst_sigma']:.2f}σ, 허용 {agree['tolerance_sigma']:.0f}σ)")
    for arm, checks in agree["arms"].items():
        for key, c in checks.items():
            mark = "ok " if c["within_tolerance"] else "OFF"
            print(f"  [{mark}] {arm:3s} {key:34s} table1={c['table1_mean']:.4f}"
                  f"±{c['table1_split_std']:.4f} got={c['reproduced']:.4f} "
                  f"({c['sigma']:.2f}σ)")
    if not agree["passed"]:
        print("  → 재현이 Table 1의 split 분산을 벗어났다. anchored 행을 Table 1에 "
              "넣지 말 것.")
    print(f"\n[D2] wrote {args.out_dir}")


if __name__ == "__main__":
    main()
