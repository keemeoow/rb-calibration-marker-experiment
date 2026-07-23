#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
CP_C2_cube_vs_board.py  —  기여도 C2 독립 실험 (실데이터)

C2: Graspable marker cube 유 vs 무 (baseline = ChArUco Board).

흐름(사용자 설계): 한 번 촬영 -> 한 번 캘리브레이션(Step3) -> 이 파일이 그 결과에서
필요한 데이터만 가져와 비교.
  - Step3_calibration.py 를 --target both 로 한 번 돌리면 calibration_summary.json 에
    transform_sets = {board_only, cube_only, hybrid} 세 벌의 고정-카메라 base 변환이 기록된다.
      board_only : ChArUco 보드만으로 추정 (큐브 미사용 baseline)
      cube_only  : graspable cube 만으로 추정
      hybrid     : 두 소스 융합 + refinement (큐브가 있어야 가능한 전체 파이프라인)
  - 이 파일은 그 세 벌을 읽어 동일 검증 파이프라인(Step5)으로 재투영/교차카메라/치수 오차
    등을 매겨 표로 비교하고 CP_result/C2 에 저장한다.
  - 추가로 시뮬 Exp2(exp2_board_vs_cube) 의 관측성 지표를 실데이터로 잰다: board vs cube
    가 촬영당 몇 대에 동시에 보이는지(동시관측), 그 카메라들의 시야각이 얼마나 벌어지는지
    (coverage). board(평면 ChArUco)는 마주본 카메라만, cube(6면 마커)는 어느 각도든 보이므로
    cube 가 더 많은 카메라에 더 넓은 각으로 관측됨을 수치로 보인다. → C2_observability.csv.

C1/C3 와 독립적으로 단독 실행된다.

<<선행: 캘리브레이션 1회>>
  python Step3_calibration.py --root_folder <sess> --intrinsics_dir <intr> \
      --out_dir <sess>/calib_out --target both
<<명령어>>
  python CP_C2_cube_vs_board.py --root_folder <sess> --intrinsics_dir <intr> \
      --calib_dir <sess>/calib_out
  # 결과 -> CP_result/C2 (기본).  --out_dir 로 변경 가능.
"""
import os
import csv
import json
import argparse
from collections import defaultdict
from typing import Any, Dict, List, Optional

import numpy as np

# 검증 파이프라인은 Step5 의 것을 그대로 재사용(테스트된 코드).
from Step5_export_reports import (
    build_mode_comparison_rows,
    save_mode_comparison_report,
    write_csv,
)
import CP_common as cp


def _ensure_dir(p: str) -> str:
    os.makedirs(p, exist_ok=True)
    return p


def print_table(rows: List[dict]) -> None:
    if not rows:
        print("[C2] no transform_sets to compare — 아래 안내 참고.")
        return
    cols = ["mode", "num_base_cameras", "cross_camera_mean_mm", "cube_reproj_mean_px",
            "board_reproj_mean_px", "mesh_rmse_mm", "dimension_err_mm",
            "pose_repeat_mm", "handeye_pass"]
    widths = {c: max(len(c), *(len(str(r.get(c, ""))) for r in rows)) for c in cols}
    print("\n" + "=" * (sum(widths.values()) + len(cols) * 2))
    print("C2  CUBE vs BOARD  (mode-comparison)")
    print("=" * (sum(widths.values()) + len(cols) * 2))
    print("  ".join(c.ljust(widths[c]) for c in cols))
    print("-" * (sum(widths.values()) + len(cols) * 2))
    for r in rows:
        print("  ".join(str(r.get(c, "")).ljust(widths[c]) for c in cols))


# ── Observability (시뮬 Exp2 짝: 동시관측 카메라 수 + 시야각 coverage) ───────────
# 시뮬 exp2_board_vs_cube.observability 는 촬영당 몇 대가 타깃을 동시에 보는지, 그 카메라들의
# 시야각이 얼마나 벌어지는지를 잰다. 실데이터는 meta.json 검출값에서 그대로 계산한다:
#   board = ChArUco 코너 검출 수(charuco_detect_n)로 "봤다" 판정 (평면이라 마주본 카메라만),
#   cube  = 6면 마커라 어느 각도든 → cube_visible / cube_pnp.ok 로 판정.
# 시야각 coverage 는 캘리브된 카메라 base 위치와 타깃 중심으로 관측방향 사이각을 평균.
def _load_base_cam_transforms(transform_sets: Dict[str, Any]):
    """geometry 용 카메라 base 변환(T_base_Ci). hybrid > cube_only > board_only 우선."""
    for mode in ("hybrid", "cube_only", "board_only"):
        ts = transform_sets.get(mode)
        if not ts:
            continue
        pos: Dict[int, np.ndarray] = {}
        for k, v in ts.items():
            if not k.startswith("T_base_C"):
                continue
            try:
                ci = int(k[len("T_base_C"):])
            except ValueError:
                continue
            pos[ci] = np.asarray(v, dtype=np.float64).reshape(4, 4)
        if pos:
            return mode, pos
    return None, {}


def _pairwise_coverage_deg(dirs: List[np.ndarray]) -> Optional[float]:
    if len(dirs) < 2:
        return None
    angs = [float(np.degrees(np.arccos(np.clip(float(dirs[i] @ dirs[j]), -1.0, 1.0))))
            for i in range(len(dirs)) for j in range(i + 1, len(dirs))]
    return float(np.mean(angs)) if angs else None


def compute_observability(meta: Dict[str, Any], fixed_cam_ids: List[int],
                          transform_sets: Dict[str, Any],
                          min_board_corners: int = 6) -> Dict[str, Any]:
    geom_mode, cam_T = _load_base_cam_transforms(transform_sets)
    fixed = {int(c) for c in fixed_cam_ids}
    agg = {t: {"simul": [], "coverage": [], "n_target_events": 0,
               "seen_counts": defaultdict(int)} for t in ("board", "cube")}
    n_events = 0
    for cap in meta.get("captures", []):
        if int(cap.get("event_id", -1)) < 0:
            continue
        n_events += 1
        cams = cap.get("cams", {})
        for target in ("board", "cube"):
            seen: List = []                       # (cam_id, T_cam_target_or_None)
            for ci_str, cinfo in cams.items():
                ci = int(ci_str)
                if ci not in fixed or not cinfo.get("saved"):
                    continue
                if target == "board":
                    ok = int(cinfo.get("charuco_detect_n", 0) or 0) >= int(min_board_corners)
                    T_ct = (cinfo.get("charuco") or {}).get("T_cam_board_4x4")
                else:
                    cpnp = cinfo.get("cube_pnp") or {}
                    ok = bool(cinfo.get("cube_visible")) or bool(cpnp.get("ok"))
                    T_ct = cpnp.get("T_cam_cube_4x4")
                if ok:
                    seen.append((ci, T_ct))
                    agg[target]["seen_counts"][ci] += 1
            if not seen:
                continue
            agg[target]["n_target_events"] += 1
            agg[target]["simul"].append(len(seen))
            dirs = []
            for ci, T_ct in seen:
                if T_ct is None or ci not in cam_T:
                    continue
                T_bc = cam_T[ci]
                center = (T_bc @ np.asarray(T_ct, dtype=np.float64).reshape(4, 4))[:3, 3]
                d = center - T_bc[:3, 3]
                n = float(np.linalg.norm(d))
                if n > 1e-9:
                    dirs.append(d / n)
            cov = _pairwise_coverage_deg(dirs)
            if cov is not None:
                agg[target]["coverage"].append(cov)

    out: Dict[str, Any] = {"n_events": n_events, "n_fixed_cams": len(fixed),
                           "geom_mode": geom_mode, "min_board_corners": int(min_board_corners),
                           "targets": {}}
    for t in ("board", "cube"):
        s = agg[t]
        simul, cov = s["simul"], s["coverage"]
        out["targets"][t] = {
            "n_target_events": s["n_target_events"],
            "mean_simul_observers": float(np.mean(simul)) if simul else None,
            "pct_events_2plus_observers": (float(np.mean([x >= 2 for x in simul]) * 100.0)
                                           if simul else None),
            "mean_angular_coverage_deg": float(np.mean(cov)) if cov else None,
            "per_cam_seen_events": {int(k): int(v) for k, v in sorted(s["seen_counts"].items())},
        }
    return out


def print_observability(obs: Dict[str, Any]) -> None:
    b = obs["targets"]["board"]
    c = obs["targets"]["cube"]
    print("\n" + "=" * 74)
    print(f"C2  OBSERVABILITY  (동시관측 + 시야각, 고정 {obs['n_fixed_cams']}대, "
          f"{obs['n_events']} events, geom={obs['geom_mode']})")
    print("=" * 74)

    def f(x, nd=2):
        return "NA" if x is None else f"{x:.{nd}f}"
    rows = [("타깃 관측 event 수", b["n_target_events"], c["n_target_events"], 0),
            ("동시관측(평균 대수)", b["mean_simul_observers"], c["mean_simul_observers"], 2),
            (">=2대 동시(%)", b["pct_events_2plus_observers"], c["pct_events_2plus_observers"], 1),
            ("시야각 coverage(°)", b["mean_angular_coverage_deg"], c["mean_angular_coverage_deg"], 1)]
    print(f"{'지표':<22}{'Board(ChArUco)':>18}{'Cube':>14}")
    print("-" * 74)
    for label, bv, cv, nd in rows:
        print(f"{label:<22}{f(bv, nd):>18}{f(cv, nd):>14}")
    print("-" * 74)
    if b["mean_simul_observers"] and c["mean_simul_observers"]:
        print(f"  동시관측: board {b['mean_simul_observers']:.2f} -> cube "
              f"{c['mean_simul_observers']:.2f} 대  (cube 6면 → 더 많은 카메라가 동시 관측)")


def main() -> None:
    ap = argparse.ArgumentParser(description="C2 ablation: graspable cube vs ChArUco board (real data)")
    ap.add_argument("--root_folder", required=True)
    ap.add_argument("--intrinsics_dir", required=True)
    ap.add_argument("--calib_dir", default=None,
                    help="Step3 --target both 로 생성된 캘리브 출력 폴더 "
                         "(calibration_summary.json 이 있는 곳). 기본 <root>/calib_out.")
    ap.add_argument("--cube_config_json", default=None)
    ap.add_argument("--out_dir", default=None, help="기본 CP_result/C2.")
    ap.add_argument("--min_board_corners", type=int, default=6,
                    help="관측성에서 board 를 '봤다'고 판정할 최소 ChArUco 코너 수(charuco_detect_n).")
    args = ap.parse_args()

    root = args.root_folder
    calib_dir = args.calib_dir or os.path.join(root, "calib_out")
    out_dir = _ensure_dir(args.out_dir or os.path.join("CP_result", "C2"))

    summary_path = os.path.join(calib_dir, "calibration_summary.json")
    if not os.path.exists(summary_path):
        raise SystemExit(
            f"[C2] {summary_path} 없음.\n"
            f"     먼저 캘리브레이션을 1회 실행하세요:\n"
            f"     python Step3_calibration.py --root_folder {root} "
            f"--intrinsics_dir {args.intrinsics_dir} --out_dir {calib_dir} --target both")

    with open(os.path.join(root, "meta.json"), "r") as f:
        meta = json.load(f)
    with open(summary_path, "r") as f:
        summary = json.load(f)

    transform_sets = summary.get("transform_sets") or {}
    if not transform_sets:
        raise SystemExit(
            "[C2] calibration_summary.json 에 transform_sets 가 없습니다.\n"
            "     Step3 를 --target both (기본값) + --emit_transform_sets true 로 다시 실행하세요.\n"
            "     (board_only 가 필요하면 ChArUco 보드가 고정 카메라에 보이도록 촬영해야 합니다.)")

    all_cam_ids = sorted({
        int(k) for cap in meta.get("captures", []) for k in cap.get("cams", {}).keys()})
    gripper_cam_idx = summary.get("gripper_cam_idx")
    if gripper_cam_idx is None:
        gripper_cam_idx = meta.get("gripper_cam_idx")

    cube_cfg, cube_cfg_source = cp.resolve_cube_config_for_run(
        root, calib_dir=calib_dir, cube_config_json=args.cube_config_json,
        default_cfg=cp.get_default_cube_config())

    print(f"[C2] calib_dir={calib_dir}")
    print(f"[C2] transform_sets present: {sorted(transform_sets.keys())}")
    if "board_only" not in transform_sets:
        print("[C2][WARN] board_only 없음 — ChArUco 보드가 고정 카메라에 충분히(>=3 frame/cam) "
              "보이지 않은 세션입니다. cube_only/hybrid 만 비교됩니다.")

    rows, bundles = build_mode_comparison_rows(
        summary, meta, args.intrinsics_dir, root, all_cam_ids, gripper_cam_idx,
        cube_cfg, include_meta=False)

    # 관측성(시뮬 Exp2 짝): board vs cube 동시관측 카메라 수 + 시야각 coverage
    fixed_cam_ids = [c for c in all_cam_ids
                     if gripper_cam_idx is None or int(c) != int(gripper_cam_idx)]
    observability = compute_observability(
        meta, fixed_cam_ids, transform_sets, min_board_corners=int(args.min_board_corners))

    # 저장: CP_result/C2
    md_path = os.path.join(out_dir, "C2_cube_vs_board.md")
    csv_path = os.path.join(out_dir, "C2_cube_vs_board.csv")
    json_path = os.path.join(out_dir, "C2_cube_vs_board.json")
    obs_csv_path = os.path.join(out_dir, "C2_observability.csv")
    save_mode_comparison_report(md_path, rows)
    write_csv(csv_path, rows, list(rows[0].keys()) if rows else [])
    with open(json_path, "w") as f:
        json.dump({
            "experiment": "C2_cube_vs_board",
            "calib_dir": calib_dir,
            "cube_config_source": cube_cfg_source,
            "modes": sorted(transform_sets.keys()),
            "rows": rows,
            "observability": observability,
            "verification": bundles,
        }, f, indent=2, ensure_ascii=False)

    # 관측성 CSV (target 한 줄씩)
    obs_cols = ["target", "n_target_events", "mean_simul_observers",
                "pct_events_2plus_observers", "mean_angular_coverage_deg"]
    with open(obs_csv_path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=obs_cols)
        w.writeheader()
        for t in ("board", "cube"):
            row = {"target": t}
            row.update({k: observability["targets"][t].get(k) for k in obs_cols[1:]})
            w.writerow(row)

    print_table(rows)
    print_observability(observability)
    print(f"\n[DONE] C2 results -> {out_dir}")
    print(f"       {csv_path}")
    print(f"       {obs_csv_path}")


if __name__ == "__main__":
    main()
