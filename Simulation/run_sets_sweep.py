#!/usr/bin/env python3
"""
촬영 셋 수 → 성능(e_task) — 대표 4방식만. 현실 조건(FK≈0+계통2%+오검출5%), gripped 40.
"셋이 많아질수록 각 방식이 얼마나 좋아지나".
  python run_sets_sweep.py --seeds 4 --workers 12
"""
import sys, os, argparse, json, itertools
from concurrent.futures import ProcessPoolExecutor
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from core import ExpConfig

CB = ("cube", "board")
METHODS = [
    ExpConfig("EXP1", fk="factor", solve="unified",     markers=CB, label="Ours"),
    ExpConfig("EXP7", fk="fixed", solve="unified",     markers=CB, label="fixed-FK"),
    ExpConfig("EXP4", fk="none",  solve="unified",     markers=CB, label="no-FK"),
    ExpConfig("EXP2", fk="factor", solve="independent", markers=CB, label="-unified(indep)"),
]
SETS = [4, 6, 8, 10, 13, 16]
COND = (0.3, 0.02, 0.0, 0.05)      # sigma, systematic, fk, outlier (realistic)
N_GRIPPED = 40


def _job(a):
    mi, seed, nsets = a
    from core.scene import SimScene
    from core.experiment import calibrate, _splits_for_seed
    from core.metrics import eval_model
    sigma, sysv, fk, outl = COND
    cfg = METHODS[mi]
    et = []
    try:
        sc = SimScene(seed=seed, n_sets=nsets, n_events_per_set=6,
                      sigma_px=sigma, fk_noise_mm=fk, fk_noise_deg=fk/10.0,
                      intrinsic_err=sysv, outlier_rate=outl, n_gripped_events=N_GRIPPED)
        train_size = max(2, nsets - 2)          # 2개는 held-out
        for test in _splits_for_seed(sc.sets, seed, 3):
            tr = [s for s in sc.sets if s not in test][:train_size]
            model, W = calibrate(sc, cfg, tr)
            res = eval_model(sc, model, tr, list(test), W=W)
            if res.get("e_task_mm") is not None:
                et.append(res["e_task_mm"])
    except Exception as ex:                    # 실패를 숨기지 않고 보고
        print(f"[FAIL] job={a}: {type(ex).__name__}: {ex}", file=sys.stderr)
    return (mi, nsets), (np.mean(et) if et else None)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seeds", type=int, default=4)
    ap.add_argument("--workers", type=int, default=12)
    args = ap.parse_args()
    jobs = [(mi, sd, ns) for mi in range(len(METHODS)) for ns in SETS for sd in range(args.seeds)]
    print(f"[sets-sweep] {len(jobs)} jobs (4방식 × {len(SETS)}셋레벨 × {args.seeds}seed), "
          f"{args.workers} workers", flush=True)
    acc = {}
    done = 0
    with ProcessPoolExecutor(max_workers=args.workers) as ex:
        for key, v in ex.map(_job, jobs):
            acc.setdefault(key, []).append(v)
            done += 1
            if done % 20 == 0:
                print(f"  {done}/{len(jobs)}", flush=True)

    def mean(mi, ns):
        vs = [x for x in acc.get((mi, ns), []) if x is not None]
        return float(np.mean(vs)) if vs else None

    curves = {METHODS[mi].label: [mean(mi, ns) for ns in SETS] for mi in range(len(METHODS))}
    out = {"sets": SETS, "metric": "e_task_mm", "cond": "realistic (FK~0, sys2%, outlier5%)",
           "gripped": N_GRIPPED, "curves": curves}
    os.makedirs("results/tables", exist_ok=True)
    json.dump(out, open("results/tables/sets_sweep.json", "w"), indent=2)
    print("[저장] results/tables/sets_sweep.json")
    print(f"\n{'방식':18s}" + "".join(f"{s:>7d}" for s in SETS))
    for m, c in curves.items():
        print(f"{m:18s}" + "".join(f"{v:7.1f}" if v is not None else "   --  " for v in c))


if __name__ == "__main__":
    main()
