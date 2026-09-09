#!/usr/bin/env python3
"""Run all 14 valid solve × target × FK conditions.

Examples:
  python run_factorial.py --seeds 1 --splits 1
  python run_factorial.py --seeds 20 --dump results/tables/factorial.json
"""
import argparse
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from configs import ALL_FACTORIAL
from core import run_config


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seeds", type=int, default=1)
    ap.add_argument("--splits", type=int, default=1)
    ap.add_argument("--sigma_px", type=float, default=0.3)
    ap.add_argument("--fk_sys_mm", type=float, default=0.0)
    ap.add_argument("--fk_sys_deg", type=float, default=0.0)
    ap.add_argument("--fk_noise_mm", type=float, default=0.0)
    ap.add_argument("--fk_noise_deg", type=float, default=0.0)
    ap.add_argument("--intrinsic_err", type=float, default=0.0)
    ap.add_argument("--outlier_rate", type=float, default=0.0)
    ap.add_argument("--gripped_events", type=int, default=0)
    ap.add_argument("--dump", default="results/tables/factorial.json")
    args = ap.parse_args()

    rows = {}
    for cfg in ALL_FACTORIAL:
        stats = run_config(cfg, seeds=args.seeds, sigma_px=args.sigma_px,
                           n_splits=args.splits,
                           fk_sys_mm=args.fk_sys_mm,
                           fk_sys_deg=args.fk_sys_deg,
                           fk_noise_mm=args.fk_noise_mm,
                           fk_noise_deg=args.fk_noise_deg,
                           intrinsic_err=args.intrinsic_err,
                           outlier_rate=args.outlier_rate,
                           n_gripped_events=args.gripped_events)
        rows[cfg.name] = {"config": cfg.__dict__, "stats": stats}
        task = stats["e_task_mm"][0]
        print(f"{cfg.name:<34} e_task={task:.3f} mm" if task is not None
              else f"{cfg.name:<34} e_task=NA")

    out_dir = os.path.dirname(args.dump)
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)
    with open(args.dump, "w") as f:
        json.dump({"meta": {"ground_truth": "synthetic_external_gt",
                            "fk_proxy": False,
                            "seeds": args.seeds,
                            "splits": args.splits,
                            "sigma_px": args.sigma_px,
                            "fk_sys_mm": args.fk_sys_mm,
                            "fk_sys_deg": args.fk_sys_deg,
                            "fk_noise_mm": args.fk_noise_mm,
                            "fk_noise_deg": args.fk_noise_deg,
                            "intrinsic_err": args.intrinsic_err,
                            "outlier_rate": args.outlier_rate,
                            "gripped_events": args.gripped_events}, "rows": rows}, f,
                  indent=2, default=list)
    print(f"[saved] {args.dump}")


if __name__ == "__main__":
    main()
