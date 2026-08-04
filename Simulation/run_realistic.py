#!/usr/bin/env python3
"""
현실 시나리오 실험 — FK≈0 (거의 완벽) + 인지 계통노이즈(intrinsic) + 마커 인지정확도(코너 σ).
실제 프로토콜(13 sets × 13 eih + gripped 130). 4방법 × (계통 × 마커σ) 격자.
"현실 노이즈 하에서 ours-B가 유지되나?"
  python run_realistic.py --seeds 3 --workers 12
"""
import sys, os, argparse, json, itertools
from concurrent.futures import ProcessPoolExecutor
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from core import ExpConfig

CB = ("cube", "board")
METHODS = [
    ExpConfig("fixed", fk="fixed", solve="unified", markers=CB, label="fixed-FK"),
    ExpConfig("noFK",  fk="none",  solve="unified", markers=CB, label="no-FK"),
    ExpConfig("oursA", fk="corr",  solve="unified", markers=CB, anchor_weight=0.0, label="ours-A"),
    ExpConfig("oursB", fk="corr",  solve="unified", markers=CB, anchor_weight=0.5, label="ours-B"),
]
SYS = [0.0, 0.01, 0.02]     # 인지 계통노이즈 (intrinsic 상대오차)
SIG = [0.3, 0.6, 1.0]       # 마커 인지정확도 (코너 검출 σ px)
FK_MM = 0.0                 # FK 거의 완벽


def _job(a):
    mi, seed, si, gi, grip, n_sets, n_events, train, pairs = a
    from core.scene import SimScene
    from core.experiment import calibrate
    from core.metrics import eval_model
    sysv, sig = SYS[si], SIG[gi]
    cfg = METHODS[mi]
    et = []
    try:
        sc = SimScene(seed=seed, n_sets=n_sets, n_events_per_set=n_events,
                      sigma_px=sig, fk_noise_mm=FK_MM, fk_noise_deg=0.0,
                      intrinsic_err=sysv, outlier_rate=0.0, n_gripped_events=grip)
        n = 0
        for test in itertools.combinations(sc.sets, 2):
            tr = [s for s in sc.sets if s not in test][:train]
            model, W = calibrate(sc, cfg, tr)
            res = eval_model(sc, model, tr, list(test), W=W)
            if res.get("e_task_mm") is not None:
                et.append(res["e_task_mm"])
            n += 1
            if n >= pairs:
                break
    except Exception:
        pass
    return (mi, si, gi), (np.mean(et) if et else None)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seeds", type=int, default=3)
    ap.add_argument("--workers", type=int, default=12)
    ap.add_argument("--sets", type=int, default=13)
    ap.add_argument("--events", type=int, default=13)
    ap.add_argument("--train", type=int, default=10)
    ap.add_argument("--pairs", type=int, default=2)
    ap.add_argument("--gripped", type=int, default=130)
    args = ap.parse_args()

    jobs = [(mi, sd, si, gi, args.gripped, args.sets, args.events, args.train, args.pairs)
            for mi in range(len(METHODS)) for si in range(len(SYS))
            for gi in range(len(SIG)) for sd in range(args.seeds)]
    print(f"[realistic] {len(jobs)} jobs (FK~0, gripped {args.gripped}, "
          f"{args.sets}sets×{args.events}eih), {args.workers} workers", flush=True)

    acc = {}
    done = 0
    with ProcessPoolExecutor(max_workers=args.workers) as ex:
        for key, v in ex.map(_job, jobs):
            acc.setdefault(key, []).append(v)
            done += 1
            if done % 20 == 0:
                print(f"  {done}/{len(jobs)}", flush=True)

    def mean(mi, si, gi):
        vs = [x for x in acc.get((mi, si, gi), []) if x is not None]
        return float(np.mean(vs)) if vs else None

    print("\n" + "=" * 70)
    print("현실 시나리오 — held-out e_task (mm), FK~0, gripped 130")
    print("=" * 70)
    out = {}
    for si, sysv in enumerate(SYS):
        for gi, sig in enumerate(SIG):
            cell = {}
            for mi, cfg in enumerate(METHODS):
                cell[cfg.name] = mean(mi, si, gi)
            valid = {n: v for n, v in cell.items() if v is not None}
            win = min(valid, key=valid.get) if valid else None
            out[f"sys{sysv}_sig{sig}"] = {"cell": cell, "winner": win}
    # 표 출력
    hdr = f"{'계통/마커σ':>10s}"
    for sig in SIG:
        hdr += f" | σ={sig}"
    print(hdr)
    for mi, cfg in enumerate(METHODS):
        print(f"\n-- {cfg.label} --")
        for si, sysv in enumerate(SYS):
            row = f"  계통{sysv:>5.0%}:"
            for gi, sig in enumerate(SIG):
                v = mean(mi, si, gi)
                row += f" {v:7.2f}" if v is not None else "    -- "
            print(row)
    print("\n[승자] (셀별 e_task 최저)")
    print(f"{'계통/σ':>10s}" + "".join(f" | σ={sig:>4}" for sig in SIG))
    for si, sysv in enumerate(SYS):
        row = f"  {sysv:>8.0%}:"
        for gi, sig in enumerate(SIG):
            w = out[f"sys{sysv}_sig{sig}"]["winner"]
            row += f" | {w or '--':>8s}"
        print(row)
    os.makedirs("results/tables", exist_ok=True)
    json.dump({"SYS": SYS, "SIG": SIG, "cells": out}, open("results/tables/realistic.json", "w"), indent=2)
    print("\n[저장] results/tables/realistic.json")


if __name__ == "__main__":
    main()
