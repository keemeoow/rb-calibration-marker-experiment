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
    ExpConfig("ours",  fk="factor", solve="unified", markers=CB, label="Ours (FK factor)"),
    ExpConfig("corr",  fk="corr",  solve="unified", markers=CB, label="구 Ridge 후보정"),
]
SYS = [0.0, 0.01, 0.02]     # 인지 계통노이즈 (intrinsic 상대오차)
SIG = [0.3, 0.6, 1.0]       # 마커 인지정확도 (코너 검출 σ px)
FK_MM = 0.0                 # FK 거의 완벽


MKEYS = ["e_task_mm", "e_X_mm", "gTc_mm", "bTf_mm"]   # held-out + 캘리브 자체 정확도


def _job(a):
    mi, seed, si, gi, grip, n_sets, n_events, train, pairs = a
    from core.scene import SimScene
    from core.experiment import calibrate, _splits_for_seed
    from core.metrics import eval_model
    sysv, sig = SYS[si], SIG[gi]
    cfg = METHODS[mi]
    acc = {k: [] for k in MKEYS}
    try:
        sc = SimScene(seed=seed, n_sets=n_sets, n_events_per_set=n_events,
                      sigma_px=sig, fk_noise_mm=FK_MM, fk_noise_deg=0.0,
                      intrinsic_err=sysv, outlier_rate=0.0, n_gripped_events=grip)
        for test in _splits_for_seed(sc.sets, seed, pairs):
            tr = [s for s in sc.sets if s not in test][:train]
            model, W = calibrate(sc, cfg, tr)
            res = eval_model(sc, model, tr, list(test), W=W)
            for k in MKEYS:
                if res.get(k) is not None:
                    acc[k].append(res[k])
    except Exception as ex:                    # 실패를 숨기지 않고 보고
        print(f"[FAIL] job={a}: {type(ex).__name__}: {ex}", file=sys.stderr)
    return (mi, si, gi), {k: (float(np.mean(v)) if v else None) for k, v in acc.items()}


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

    acc = {}    # (mi,si,gi) -> list of metric dicts
    done = 0
    with ProcessPoolExecutor(max_workers=args.workers) as ex:
        for key, d in ex.map(_job, jobs):
            acc.setdefault(key, []).append(d)
            done += 1
            if done % 20 == 0:
                print(f"  {done}/{len(jobs)}", flush=True)

    def mean(mi, si, gi, metric):
        vs = [dd[metric] for dd in acc.get((mi, si, gi), []) if dd.get(metric) is not None]
        return float(np.mean(vs)) if vs else None

    METRIC_TITLE = {"e_X_mm": "카메라 자기위치+핸드아이 정확도 e_X",
                    "bTf_mm": "고정카메라 base 위치 bTf",
                    "gTc_mm": "hand-eye gTc",
                    "e_task_mm": "held-out 큐브예측 e_task"}
    out = {"SYS": SYS, "SIG": SIG, "metrics": {}}
    for metric in ["e_X_mm", "bTf_mm", "gTc_mm", "e_task_mm"]:
        print("\n" + "=" * 70)
        print(f"현실 시나리오 — {METRIC_TITLE[metric]} (mm), FK~0, gripped 130")
        print("=" * 70)
        mout = {}
        for mi, cfg in enumerate(METHODS):
            print(f"-- {cfg.label} --")
            for si, sysv in enumerate(SYS):
                row = f"  계통{sysv:>5.0%}:"
                for gi, sig in enumerate(SIG):
                    v = mean(mi, si, gi, metric)
                    row += f" {v:8.2f}" if v is not None else "     -- "
                print(row)
        # 승자 맵
        print(f"  [승자] " + "".join(f" σ={sig:>4}" for sig in SIG))
        for si, sysv in enumerate(SYS):
            row = f"  계통{sysv:>4.0%}:"
            for gi, sig in enumerate(SIG):
                cell = {cfg.name: mean(mi, si, gi, metric) for mi, cfg in enumerate(METHODS)}
                valid = {n: v for n, v in cell.items() if v is not None}
                w = min(valid, key=valid.get) if valid else None
                mout[f"{sysv}_{sig}"] = {"cell": cell, "winner": w}
                row += f" {w or '--':>7s}"
            print(row)
        out["metrics"][metric] = mout
    os.makedirs("results/tables", exist_ok=True)
    json.dump(out, open("results/tables/realistic.json", "w"), indent=2)
    print("\n[저장] results/tables/realistic.json")


if __name__ == "__main__":
    main()
