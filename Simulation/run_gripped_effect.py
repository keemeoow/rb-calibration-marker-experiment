#!/usr/bin/env python3
"""
gripped 효과 집중 실험 — 실제 프로토콜(13 sets × 13 eih + gripped N) 에서
4방법 × gripped{0, 130} × 대표조건 4개. "gripped 넣으면 얼마나 좋아지나 + 순위 바뀌나".
  python run_gripped_effect.py --seeds 5 --workers 12
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
    ExpConfig("ours",  fk="factor", solve="unified", markers=CB, label="Ours (FK factor)"),
    ExpConfig("corr",  fk="corr",  solve="unified", markers=CB, label="구 Ridge 후보정"),
]
# 대표 조건 (FK오차mm, 계통노이즈)
CONDS = [(0.0, 0.0), (0.0, 0.02), (8.0, 0.0), (8.0, 0.02)]
COND_LBL = ["ideal (FK0,sys0)", "systematic (FK0,sys2%)", "FK-err (FK8,sys0)", "both (FK8,sys2%)"]


def _job(a):
    mi, seed, ci_cond, grip, n_sets, n_events, train, pairs = a
    from core.scene import SimScene
    from core.experiment import calibrate, _splits_for_seed
    from core.metrics import eval_model
    fk, sys = CONDS[ci_cond]
    cfg = METHODS[mi]
    et = []
    try:
        sc = SimScene(seed=seed, n_sets=n_sets, n_events_per_set=n_events,
                      sigma_px=0.3, fk_noise_mm=fk, fk_noise_deg=fk / 10.0,
                      intrinsic_err=sys, outlier_rate=sys * 5.0, n_gripped_events=grip)
        for test in _splits_for_seed(sc.sets, seed, pairs):
            tr = [s for s in sc.sets if s not in test][:train]
            model, W = calibrate(sc, cfg, tr)
            res = eval_model(sc, model, tr, list(test), W=W)
            if res.get("e_task_mm") is not None:
                et.append(res["e_task_mm"])
    except Exception as ex:                    # 실패를 숨기지 않고 보고
        print(f"[FAIL] job={a}: {type(ex).__name__}: {ex}", file=sys.stderr)
    return (mi, ci_cond, grip), (np.mean(et) if et else None)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seeds", type=int, default=5)
    ap.add_argument("--workers", type=int, default=12)
    ap.add_argument("--sets", type=int, default=13)
    ap.add_argument("--events", type=int, default=13)
    ap.add_argument("--train", type=int, default=10)
    ap.add_argument("--pairs", type=int, default=3)
    ap.add_argument("--gripped", type=int, default=130)
    args = ap.parse_args()

    jobs = [(mi, sd, ci, grip, args.sets, args.events, args.train, args.pairs)
            for mi in range(len(METHODS)) for ci in range(len(CONDS))
            for grip in (0, args.gripped) for sd in range(args.seeds)]
    print(f"[gripped-effect] {len(jobs)} jobs (실제 {args.sets}sets×{args.events}eih, "
          f"gripped 0 vs {args.gripped}), {args.workers} workers", flush=True)

    acc = {}
    done = 0
    with ProcessPoolExecutor(max_workers=args.workers) as ex:
        for key, v in ex.map(_job, jobs):
            acc.setdefault(key, []).append(v)
            done += 1
            if done % 20 == 0:
                print(f"  {done}/{len(jobs)}", flush=True)

    def mean(mi, ci, grip):
        vs = [x for x in acc.get((mi, ci, grip), []) if x is not None]
        return float(np.mean(vs)) if vs else None

    print("\n" + "=" * 78)
    print("gripped 효과 — held-out e_task (mm), 낮을수록 좋음")
    print("=" * 78)
    out = {}
    for ci, clbl in enumerate(COND_LBL):
        print(f"\n[{clbl}]")
        print(f"  {'method':10s} {'grip=0':>9s} {'grip=130':>9s} {'개선':>9s}")
        rows = {}
        for mi, cfg in enumerate(METHODS):
            m0, m1 = mean(mi, ci, 0), mean(mi, ci, args.gripped)
            rows[cfg.name] = {"grip0": m0, "grip130": m1}
            imp = f"{(m0-m1):+.2f}" if (m0 and m1) else "--"
            print(f"  {cfg.label:10s} {(m0 or 0):9.2f} {(m1 or 0):9.2f} {imp:>9s}")
        # 순위(grip=130)
        valid = {n: r["grip130"] for n, r in rows.items() if r["grip130"] is not None}
        if valid:
            win = min(valid, key=valid.get)
            print(f"    → grip=130 승자: {win} ({valid[win]:.2f}mm)")
        out[clbl] = rows
    os.makedirs("results/tables", exist_ok=True)
    json.dump(out, open("results/tables/gripped_effect.json", "w"), indent=2)
    print("\n[저장] results/tables/gripped_effect.json")


if __name__ == "__main__":
    main()
