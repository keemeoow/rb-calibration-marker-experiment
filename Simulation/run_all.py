#!/usr/bin/env python3
"""
8개 실험 전체 실행 → Table 2a (절제, 시뮬 GT 지표) 산출 + JSON 저장.

  python run_all.py --seeds 20 --dump results/tables/table2a.json
"""
import sys, os, argparse, json
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from core import run_config
from configs import ALL


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seeds", type=int, default=20)
    ap.add_argument("--sigma_px", type=float, default=0.3)
    ap.add_argument("--dump", type=str, default="results/tables/table2a.json")
    args = ap.parse_args()

    rows = {}
    print("=" * 92)
    print(" Table 2a — Synthetic ablation (GT 대비 오차, systematic 노이즈)")
    print("=" * 92)
    hdr = f"{'#':<6}{'설명':<26}{'N_reg':>7}{'e_X(mm/°)':>16}{'e_task(mm/°)':>16}{'e_cross(mm)':>13}"
    print(hdr); print("-" * 92)
    for cfg in ALL:
        st = run_config(cfg, seeds=args.seeds, sigma_px=args.sigma_px)
        rows[cfg.name] = {"config": cfg.__dict__, "stats": st}
        def g(k, d=1):
            m = st.get(k, (None,))[0]
            return f"{m:.{d}f}" if m is not None else "—"
        print(f"{cfg.name:<6}{cfg.label[:24]:<26}"
              f"{g('N_reg'):>7}"
              f"{g('e_X_mm')+'/'+g('e_X_deg',2):>16}"
              f"{g('e_task_mm')+'/'+g('e_task_deg',2):>16}"
              f"{g('e_cross_mm'):>13}")
    print("-" * 92)
    os.makedirs(os.path.dirname(args.dump), exist_ok=True)
    json.dump(rows, open(args.dump, "w"), indent=2, default=list)
    print(f"[저장] {args.dump}")


if __name__ == "__main__":
    main()
