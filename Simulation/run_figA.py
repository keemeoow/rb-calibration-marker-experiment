#!/usr/bin/env python3
"""
Fig A — 코너 노이즈 강건성. sigma_px(코너 픽셀 노이즈)를 스윕하며 방법별 오차 곡선.
CPU 병렬. 곡선: EXP1(corr) / EXP4(none) / EXP7(fixed) / EXP3(큐브만).

  python run_figA.py --seeds 20 --workers 40 --dump results/tables/figA.json
그림: python viz_figA.py
"""
import sys, os, argparse, json, itertools
from concurrent.futures import ProcessPoolExecutor
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from configs import EXP1, EXP3, EXP4, EXP7

CURVES = [EXP1, EXP4, EXP7, EXP3]      # corr / none / fixed / cube-only
SIGMAS = [0.0, 0.25, 0.5, 1.0, 1.5, 2.0]
KEYS = ["e_X_mm", "e_task_mm", "e_reproj_px"]


def _job(a):
    ci, seed, sigma, n_sets, n_events, train, splits = a
    from core.scene import SimScene
    from core.experiment import calibrate
    from core.metrics import eval_model
    cfg = CURVES[ci]
    sc = SimScene(seed=seed, n_sets=n_sets, n_events_per_set=n_events, sigma_px=sigma)
    reproj = float(np.mean(list(sc.reproj.values()))) if sc.reproj else None
    out = {k: [] for k in KEYS}
    n = 0
    for test in itertools.combinations(sc.sets, 2):
        tr = [s for s in sc.sets if s not in test][:train]
        m, W = calibrate(sc, cfg, tr)
        r = eval_model(sc, m, tr, list(test), W=W)
        for k in KEYS:
            if k == "e_reproj_px":
                if reproj is not None: out[k].append(reproj)
            elif r.get(k) is not None: out[k].append(r[k])
        n += 1
        if n >= splits: break
    return (ci, sigma), out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seeds", type=int, default=20)
    ap.add_argument("--workers", type=int, default=os.cpu_count() - 2)
    ap.add_argument("--sets", type=int, default=10)
    ap.add_argument("--events", type=int, default=6)
    ap.add_argument("--train", type=int, default=8)
    ap.add_argument("--splits", type=int, default=2)
    ap.add_argument("--dump", type=str, default="results/tables/figA.json")
    args = ap.parse_args()

    jobs = [(ci, seed, sig, args.sets, args.events, args.train, args.splits)
            for ci in range(len(CURVES)) for sig in SIGMAS for seed in range(args.seeds)]
    print(f"[병렬] {len(jobs)} jobs, {args.workers} workers")
    acc = {(ci, sig): {k: [] for k in KEYS}
           for ci in range(len(CURVES)) for sig in SIGMAS}
    done = 0
    with ProcessPoolExecutor(max_workers=args.workers) as ex:
        for key, out in ex.map(_job, jobs):
            for k in KEYS:
                acc[key][k].extend(out[k])
            done += 1
            if done % 20 == 0:
                print(f"  {done}/{len(jobs)}", flush=True)

    curves = {}
    for ci, cfg in enumerate(CURVES):
        curves[cfg.name] = {"label": cfg.label, "sigmas": SIGMAS}
        for k in KEYS:
            curves[cfg.name][k] = [float(np.mean(acc[(ci, sig)][k]))
                                   if acc[(ci, sig)][k] else None for sig in SIGMAS]
    os.makedirs(os.path.dirname(args.dump), exist_ok=True)
    json.dump({"meta": {"seeds": args.seeds}, "sigmas": SIGMAS, "curves": curves},
              open(args.dump, "w"), indent=2)
    print(f"[저장] {args.dump}")
    # 미리보기
    print("\nsigma_px:", SIGMAS)
    for name, c in curves.items():
        print(f"  {name:<6} e_X:", [f"{v:.1f}" if v else "—" for v in c["e_X_mm"]])


if __name__ == "__main__":
    main()
