#!/usr/bin/env python3
"""
논문용 시뮬 실험 — 7방법(EXP1~7) × 노이즈 조건. 표1·1b·그림A·B 용 데이터 산출.
약식 프로토콜(10 sets × 6 eih + gripped 40, ~38% 비율). GT 기준 지표.
  python run_paper_sim.py --seeds 4 --workers 12
"""
import sys, os, argparse, json
from concurrent.futures import ProcessPoolExecutor
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from configs import ALL          # EXP1~7 (7방법)

KEYS = ["e_task_mm", "e_task_deg", "e_X_mm", "bTf_mm", "e_reproj_px", "e_cross_mm"]
N_GRIPPED = 40                   # fork 로 워커 상속

# 노이즈 축 (그림 A) — 한 축만 변화, 나머지는 baseline(σ0.3, 나머지 0)
SIGMAS = [0.3, 0.5, 1.0, 1.5, 2.0]
SYSS   = [0.0, 0.01, 0.02, 0.03]
FKS    = [0.0, 2.0, 4.0, 8.0, 16.0]
OUTS   = [0.0, 0.05, 0.10, 0.20]
# 그림 B 격자: 계통 × 오검출 (사용자 실제 조건 FK≈0 의 2D 맵)
GRID_SYS = [0.0, 0.01, 0.02, 0.03]
GRID_OUT = [0.0, 0.05, 0.10, 0.20]
# 표 조건
TABLE_CONDS = {
    "ideal":     (0.0, 0.0, 0.0, 0.0),
    "realistic": (0.3, 0.02, 0.0, 0.05),    # FK≈0(사용자 계획) + 계통2% + 오검출5%
    "fk_err":    (0.3, 0.0, 4.0, 0.0),       # FK 오차 스트레스(별도)
    "outlier":   (0.3, 0.0, 0.0, 0.10),
}


def _ckey(sigma, sysv, fk, outl):
    return f"{sigma:.3f}_{sysv:.3f}_{fk:.3f}_{outl:.3f}"


def _all_conditions():
    """필요한 모든 (sigma,sys,fk,outl) 유니크 집합 + 레이아웃."""
    conds = {}
    def add(c): conds[_ckey(*c)] = c
    layout = {"figA": {}, "figB": {"sys": GRID_SYS, "out": GRID_OUT, "cells": {}}, "table": {}}
    # 그림 A
    for s in SIGMAS: add((s, 0.0, 0.0, 0.0))
    for v in SYSS:   add((0.3, v, 0.0, 0.0))
    for f in FKS:    add((0.3, 0.0, f, 0.0))
    for o in OUTS:   add((0.3, 0.0, 0.0, o))
    layout["figA"]["sigma"] = {"levels": SIGMAS, "keys": [_ckey(s,0,0,0) for s in SIGMAS]}
    layout["figA"]["sys"]   = {"levels": SYSS,   "keys": [_ckey(0.3,v,0,0) for v in SYSS]}
    layout["figA"]["fk"]    = {"levels": FKS,    "keys": [_ckey(0.3,0,f,0) for f in FKS]}
    layout["figA"]["outl"]  = {"levels": OUTS,   "keys": [_ckey(0.3,0,0,o) for o in OUTS]}
    # 그림 B (계통 × 오검출, FK≈0)
    for xi, v in enumerate(GRID_SYS):
        for yi, o in enumerate(GRID_OUT):
            add((0.3, v, 0.0, o))
            layout["figB"]["cells"][f"{xi}_{yi}"] = _ckey(0.3, v, 0.0, o)
    # 표
    for name, c in TABLE_CONDS.items():
        add(c); layout["table"][name] = _ckey(*c)
    return conds, layout


def _job(a):
    mi, seed, cond = a
    from core.scene import SimScene
    from core.experiment import calibrate, _splits_for_seed
    from core.metrics import eval_model
    sigma, sysv, fk, outl = cond
    cfg = ALL[mi]
    acc = {k: [] for k in KEYS}
    try:
        sc = SimScene(seed=seed, n_sets=10, n_events_per_set=6,
                      sigma_px=sigma, fk_noise_mm=fk, fk_noise_deg=fk / 10.0,
                      intrinsic_err=sysv, outlier_rate=outl, n_gripped_events=N_GRIPPED)
        for test in _splits_for_seed(sc.sets, seed, 2):
            tr = [s for s in sc.sets if s not in test][:8]
            model, W = calibrate(sc, cfg, tr)
            res = eval_model(sc, model, tr, list(test), W=W)
            for k in KEYS:
                if res.get(k) is not None:
                    acc[k].append(res[k])
    except Exception as ex:                    # 실패를 숨기지 않고 보고
        print(f"[FAIL] job={a}: {type(ex).__name__}: {ex}", file=sys.stderr)
    return (mi, _ckey(*cond)), {k: (float(np.mean(v)) if v else None) for k, v in acc.items()}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seeds", type=int, default=4)
    ap.add_argument("--workers", type=int, default=12)
    ap.add_argument("--gripped", type=int, default=40)
    args = ap.parse_args()
    global N_GRIPPED
    N_GRIPPED = int(args.gripped)

    conds, layout = _all_conditions()
    jobs = [(mi, sd, conds[ck]) for mi in range(len(ALL))
            for ck in conds for sd in range(args.seeds)]
    print(f"[paper-sim] {len(jobs)} jobs (7방법 × {len(conds)}조건 × {args.seeds}seed, "
          f"gripped {N_GRIPPED}), {args.workers} workers", flush=True)

    agg = {}     # (mi, ckey) -> {metric: [vals]}
    done = 0
    with ProcessPoolExecutor(max_workers=args.workers) as ex:
        for key, d in ex.map(_job, jobs):
            slot = agg.setdefault(key, {k: [] for k in KEYS})
            for k in KEYS:
                if d.get(k) is not None:
                    slot[k].append(d[k])
            done += 1
            if done % 40 == 0:
                print(f"  {done}/{len(jobs)}", flush=True)

    def mean(mi, ck, metric):
        v = agg.get((mi, ck), {}).get(metric, [])
        return float(np.mean(v)) if v else None

    # results[ckey][method_name][metric]
    results = {}
    for ck in conds:
        results[ck] = {}
        for mi, cfg in enumerate(ALL):
            results[ck][cfg.name] = {k: mean(mi, ck, k) for k in KEYS}

    out = {"methods": [c.name for c in ALL],
           "method_labels": [c.label for c in ALL],
           "meta": {"seeds": args.seeds, "gripped": N_GRIPPED,
                    "protocol": "10 sets x 6 eih + gripped %d" % N_GRIPPED},
           "layout": layout, "results": results}
    os.makedirs("results/tables", exist_ok=True)
    json.dump(out, open("results/tables/paper_sim.json", "w"), indent=2)
    print("[저장] results/tables/paper_sim.json")
    # 미리보기 (realistic e_task)
    rk = layout["table"]["realistic"]
    print("\nrealistic e_task(mm):")
    for cfg in ALL:
        v = results[rk][cfg.name]["e_task_mm"]
        print(f"  {cfg.name:5s} {cfg.label:22s} {v:.2f}" if v else f"  {cfg.name} —")


if __name__ == "__main__":
    main()
