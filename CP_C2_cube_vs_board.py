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
from typing import Dict, List

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


def main() -> None:
    ap = argparse.ArgumentParser(description="C2 ablation: graspable cube vs ChArUco board (real data)")
    ap.add_argument("--root_folder", required=True)
    ap.add_argument("--intrinsics_dir", required=True)
    ap.add_argument("--calib_dir", default=None,
                    help="Step3 --target both 로 생성된 캘리브 출력 폴더 "
                         "(calibration_summary.json 이 있는 곳). 기본 <root>/calib_out.")
    ap.add_argument("--cube_config_json", default=None)
    ap.add_argument("--out_dir", default=None, help="기본 CP_result/C2.")
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

    # 저장: CP_result/C2
    md_path = os.path.join(out_dir, "C2_cube_vs_board.md")
    csv_path = os.path.join(out_dir, "C2_cube_vs_board.csv")
    json_path = os.path.join(out_dir, "C2_cube_vs_board.json")
    save_mode_comparison_report(md_path, rows)
    write_csv(csv_path, rows, list(rows[0].keys()) if rows else [])
    with open(json_path, "w") as f:
        json.dump({
            "experiment": "C2_cube_vs_board",
            "calib_dir": calib_dir,
            "cube_config_source": cube_cfg_source,
            "modes": sorted(transform_sets.keys()),
            "rows": rows,
            "verification": bundles,
        }, f, indent=2, ensure_ascii=False)

    print_table(rows)
    print(f"\n[DONE] C2 results -> {out_dir}")
    print(f"       {csv_path}")


if __name__ == "__main__":
    main()
