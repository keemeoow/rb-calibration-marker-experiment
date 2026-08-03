#!/usr/bin/env python3
"""
Table 2a (코너 수준, full-seed) — 7개 실험을 CPU 병렬로 산출 + JSON 저장.
GPU 는 이 작업(scipy least_squares / cv2 solvePnP)에 도움 안 됨 → CPU 멀티프로세스 병렬.

  python run_table2a.py --seeds 20 --workers 8 --dump results/tables/table2a.json
"""
import sys, os, argparse, json
from concurrent.futures import ProcessPoolExecutor
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from core import run_config
from configs import ALL

KEYS = ["N_reg", "e_X_mm", "e_X_deg", "e_task_mm", "e_task_deg",
        "e_cross_mm", "bTf_mm", "gTc_mm", "e_reproj_px"]


def _one_seed(args):
    """(cfg_idx, seed, sigma_px, n_sets, n_events, train_size) → seed 1개의 지표 리스트."""
    cfg_idx, seed, sigma_px, n_sets, n_events, train_size, n_splits = args
    import itertools
    from core.scene import SimScene
    from core.experiment import calibrate
    from core.metrics import eval_model
    cfg = ALL[cfg_idx]
    out = {k: [] for k in KEYS}
    sc = SimScene(seed=seed, n_sets=n_sets, n_events_per_set=n_events, sigma_px=sigma_px)
    reproj = float(np.mean(list(sc.reproj.values()))) if sc.reproj else None
    splits = 0
    for test in itertools.combinations(sc.sets, 2):
        train = [s for s in sc.sets if s not in test][:train_size]
        model, W = calibrate(sc, cfg, train)
        res = eval_model(sc, model, train, list(test), W=W)
        for k in KEYS:
            if k == "e_reproj_px":
                if reproj is not None:
                    out[k].append(reproj)
            elif res.get(k) is not None:
                out[k].append(res[k])
        splits += 1
        if splits >= n_splits:
            break
    return cfg_idx, out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seeds", type=int, default=20)
    ap.add_argument("--workers", type=int, default=os.cpu_count() - 2)
    ap.add_argument("--sigma_px", type=float, default=0.3)
    ap.add_argument("--sets", type=int, default=10)
    ap.add_argument("--events", type=int, default=6)
    ap.add_argument("--train", type=int, default=8)
    ap.add_argument("--splits", type=int, default=3)
    ap.add_argument("--dump", type=str, default="results/tables/table2a.json")
    args = ap.parse_args()

    jobs = [(ci, seed, args.sigma_px, args.sets, args.events, args.train, args.splits)
            for ci in range(len(ALL)) for seed in range(args.seeds)]
    print(f"[병렬] {len(jobs)} jobs ({len(ALL)} exp × {args.seeds} seed), "
          f"{args.workers} workers")
    acc = {ci: {k: [] for k in KEYS} for ci in range(len(ALL))}
    done = 0
    with ProcessPoolExecutor(max_workers=args.workers) as ex:
        for cfg_idx, out in ex.map(_one_seed, jobs):
            for k in KEYS:
                acc[cfg_idx][k].extend(out[k])
            done += 1
            if done % 10 == 0:
                print(f"  {done}/{len(jobs)} seeds done", flush=True)

    # 집계 + 표
    rows = {}
    print("\n" + "=" * 96)
    print(" Table 2a — Corner-level ablation (실물 마커, GT 대비, full-seed)")
    print("=" * 96)
    print(f"{'#':<6}{'설명':<26}{'N_reg':>6}{'e_X(mm/°)':>15}{'e_task(mm/°)':>15}"
          f"{'e_cross':>9}{'reproj':>8}")
    print("-" * 96)
    for ci, cfg in enumerate(ALL):
        st = {k: (float(np.mean(v)), float(np.std(v)), len(v)) if v else (None, None, 0)
              for k, v in acc[ci].items()}
        rows[cfg.name] = {"config": cfg.__dict__, "stats": st}
        def g(k, d=2):
            m = st[k][0]; return f"{m:.{d}f}" if m is not None else "—"
        print(f"{cfg.name:<6}{cfg.label[:24]:<26}{g('N_reg',1):>6}"
              f"{g('e_X_mm')+'/'+g('e_X_deg'):>15}{g('e_task_mm')+'/'+g('e_task_deg'):>15}"
              f"{g('e_cross_mm'):>9}{g('e_reproj_px'):>8}")
    print("-" * 96)
    os.makedirs(os.path.dirname(args.dump), exist_ok=True)
    json.dump({"meta": {"seeds": args.seeds, "sigma_px": args.sigma_px,
                        "sets": args.sets, "events": args.events},
               "rows": rows}, open(args.dump, "w"), indent=2, default=list)
    print(f"[저장] {args.dump}")


if __name__ == "__main__":
    main()
