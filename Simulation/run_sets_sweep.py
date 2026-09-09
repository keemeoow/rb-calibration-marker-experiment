#!/usr/bin/env python3
"""
학습 set 수 → 성능 sweep. **test set 수는 항상 2로 고정**, 학습 set 만 [3,5,7,9,11] 변화.
방식은 Ours(EXP1)/no-FK(EXP4)/fixed-FK(EXP7) 3개 고정. FK 없음/있음 둘 다.
데이터가 적어지는 현실 반영: gripped 캡처도 학습 set 수에 비례 축소(89*k/11).
질문: "학습 set 이 적을 때(=vision 약할 때) FK 가 no-FK 를 이기나?"
  OMP_NUM_THREADS=1 python run_sets_sweep.py --seeds 20 --workers 16
"""
import sys, os, argparse, json
from concurrent.futures import ProcessPoolExecutor
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from configs import EXP1, EXP4, EXP7

METHODS = [EXP1, EXP4, EXP7]                     # Ours / no-FK / fixed-FK (고정)
SET_COUNTS = [3, 5, 7, 9, 11]                    # 학습 set 수
TEST_COUNT = 2                                   # held-out (항상 고정)
N_EVENTS = 11                                    # set당 eye-in-hand (실측)
GRIPPED_FULL = 89                                # k=11 기준 gripped (실측). k 에 비례 축소.
KEYS = ["e_task_mm", "e_rel_mm", "e_reproj_raw_px"]
# FK 없음(정확) vs 있음(systematic 6.6mm) — realistic 노이즈 공통
CONDS = {
    "fk_off": dict(fk_sys_mm=0.0),
    "fk_on":  dict(fk_sys_mm=6.6),
}
NOISE = dict(sigma_px=0.2, intrinsic_err=0.005, outlier_rate=0.02, outlier_px=2.0)


def _job(a):
    mi, cond_name, k, seed = a
    from core.scene import SimScene
    from core.experiment import calibrate
    from core.metrics import eval_model
    cfg = METHODS[mi]
    fk_sys = CONDS[cond_name]["fk_sys_mm"]
    gripped = max(8, round(GRIPPED_FULL * k / 11.0))     # 데이터량 비례 축소
    out = {key: None for key in KEYS}
    try:
        sc = SimScene(seed=seed, n_sets=k + TEST_COUNT, n_events_per_set=N_EVENTS,
                      fk_sys_mm=fk_sys, fk_sys_deg=fk_sys * 0.08,
                      n_gripped_events=gripped, **NOISE)
        sets = list(sc.sets)
        train, test = sets[:k], sets[k:k + TEST_COUNT]   # test 는 항상 마지막 2개
        model, W = calibrate(sc, cfg, train)
        res = eval_model(sc, model, train, test, W=W)
        for key in KEYS:
            out[key] = res.get(key)
    except Exception as e:
        import traceback
        print(f"[FAIL] {cfg.name} {cond_name} k={k} seed={seed}: {type(e).__name__}: {e}",
              file=sys.stderr, flush=True)
        traceback.print_exc()
    return (mi, cond_name, k), out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seeds", type=int, default=20)
    ap.add_argument("--workers", type=int, default=16)
    args = ap.parse_args()

    jobs = [(mi, cn, k, sd) for mi in range(len(METHODS))
            for cn in CONDS for k in SET_COUNTS for sd in range(args.seeds)]
    print(f"[sets-sweep] {len(jobs)} jobs (3방식 × {len(CONDS)}조건 × {len(SET_COUNTS)}set수 × "
          f"{args.seeds}seed, test=2 고정), {args.workers} workers", flush=True)

    agg = {}
    done = 0
    with ProcessPoolExecutor(max_workers=args.workers) as ex:
        for key, d in ex.map(_job, jobs):
            slot = agg.setdefault(key, {kk: [] for kk in KEYS})
            for kk in KEYS:
                if d.get(kk) is not None and np.isfinite(d[kk]):
                    slot[kk].append(d[kk])
            done += 1
            if done % 60 == 0:
                print(f"  {done}/{len(jobs)}", flush=True)

    def med(mi, cn, k, metric):
        v = agg.get((mi, cn, k), {}).get(metric, [])
        return float(np.median(v)) if v else None

    # results[cond][method][metric] = [값 per set_count]
    results = {}
    for cn in CONDS:
        results[cn] = {}
        for mi, cfg in enumerate(METHODS):
            results[cn][cfg.name] = {metric: [med(mi, cn, k, metric) for k in SET_COUNTS]
                                     for metric in KEYS}
    out = {"methods": [c.name for c in METHODS],
           "labels": [c.label for c in METHODS],
           "set_counts": SET_COUNTS, "test_count": TEST_COUNT,
           "conds": list(CONDS.keys()), "meta": {"seeds": args.seeds, "agg": "median"},
           "results": results}
    os.makedirs("results/tables", exist_ok=True)
    json.dump(out, open("results/tables/sets_sweep.json", "w"), indent=2)
    print("[저장] results/tables/sets_sweep.json")

    # 미리보기: e_task, 학습 set 수별 (FK 없음/있음)
    for cn in CONDS:
        print(f"\n=== {cn} : e_task(mm) — 학습 set 수 {SET_COUNTS} (test=2 고정) ===")
        for cfg in METHODS:
            vs = results[cn][cfg.name]["e_task_mm"]
            row = "  ".join(f"{v:6.2f}" if v is not None else "  —  " for v in vs)
            print(f"  {cfg.name} {cfg.label:20s} {row}")


if __name__ == "__main__":
    main()
