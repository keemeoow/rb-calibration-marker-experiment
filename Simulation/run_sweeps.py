#!/usr/bin/env python3
"""
종합 노이즈 sweep — 두 축(코너 px / FK mm) × 7방식 × 전체 지표, held-out train/test 쌍 순회.
CPU 병렬. 시뮬의 본질(노이즈 늘려가며 각 지표 평가)을 완전히 산출.

  python run_sweeps.py --axis corner --seeds 20 --workers 40 --dump results/tables/sweep_corner.json
  python run_sweeps.py --axis fk     --seeds 20 --workers 40 --dump results/tables/sweep_fk.json

held-out: seed 마다 여러 (train,test) 조합을 순회(--pairs 로 조합 수 제한)해 평균.
지표: N_reg, e_X_mm, e_X_deg, e_task_mm, e_task_deg, gTc_mm, e_cross_mm, e_reproj_px, bTf_mm.
"""
import sys, os, argparse, json
from concurrent.futures import ProcessPoolExecutor
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from configs import ALL

KEYS = ["N_reg", "e_X_mm", "e_X_deg", "e_task_mm", "e_task_deg",
        "gTc_mm", "e_cross_mm", "e_reproj_px", "bTf_mm"]

CORNER_SIGMAS = [0.0, 0.25, 0.5, 1.0, 1.5, 2.0]     # px
FK_NOISES = [0.0, 1.0, 2.0, 4.0, 8.0, 16.0]         # mm (회전은 mm/10 deg 로 비례)


def _job(a):
    """(cfg_idx, seed, axis, level, n_sets, n_events, train_size, pairs, (intr,outl))."""
    ci, seed, axis, level, n_sets, n_events, train_size, pairs, ba = a
    from core.scene import SimScene
    from core.experiment import calibrate, _splits_for_seed
    from core.metrics import eval_model
    cfg = ALL[ci]
    intr, outl = ba                                  # 배경 노이즈(intrinsic, outlier)
    kw = dict(seed=seed, n_sets=n_sets, n_events_per_set=n_events,
              intrinsic_err=intr, outlier_rate=outl)
    if axis == "fk":
        kw["sigma_px"] = 0.3
        kw["fk_noise_mm"] = level
        kw["fk_noise_deg"] = level / 10.0            # 16mm ↔ 1.6° 비례
    elif axis == "intrinsic":
        kw["sigma_px"] = 0.3
        kw["intrinsic_err"] = level                  # intrinsic 오차 자체를 sweep
    elif axis == "outlier":
        kw["sigma_px"] = 0.3
        kw["outlier_rate"] = level                   # outlier 비율 sweep
    else:                                            # corner 축
        kw["sigma_px"] = level
    sc = SimScene(**kw)
    out = {k: [] for k in KEYS}
    # held-out: seed 별로 뽑은 (train,test) 조합 순회
    for test in _splits_for_seed(sc.sets, seed, pairs):
        train = [s for s in sc.sets if s not in test][:train_size]
        model, W = calibrate(sc, cfg, train)
        res = eval_model(sc, model, train, list(test), W=W)
        for k in KEYS:
            if res.get(k) is not None:
                out[k].append(res[k])
    return (ci, level), out


INTRINSIC_ERRS = [0.0, 0.005, 0.01, 0.02, 0.03, 0.05]   # 상대오차
OUTLIER_RATES = [0.0, 0.02, 0.05, 0.10, 0.20]           # 코너 이상치 비율


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--axis", choices=["corner", "fk", "intrinsic", "outlier"], required=True)
    ap.add_argument("--seeds", type=int, default=20)
    ap.add_argument("--workers", type=int, default=os.cpu_count() - 2)
    ap.add_argument("--sets", type=int, default=10)
    ap.add_argument("--events", type=int, default=6)
    ap.add_argument("--train", type=int, default=8)
    ap.add_argument("--pairs", type=int, default=6)      # held-out 조합 수/seed
    # 배경 노이즈(sweep 축이 아닌 축은 이 값으로 고정)
    ap.add_argument("--bg_intrinsic", type=float, default=0.0)
    ap.add_argument("--bg_outlier", type=float, default=0.0)
    ap.add_argument("--dump", type=str, default=None)
    args = ap.parse_args()

    levels = {"corner": CORNER_SIGMAS, "fk": FK_NOISES,
              "intrinsic": INTRINSIC_ERRS, "outlier": OUTLIER_RATES}[args.axis]
    dump = args.dump or f"results/tables/sweep_{args.axis}.json"
    ba = (args.bg_intrinsic, args.bg_outlier)            # 배경 노이즈
    jobs = [(ci, seed, args.axis, lv, args.sets, args.events, args.train, args.pairs, ba)
            for ci in range(len(ALL)) for lv in levels for seed in range(args.seeds)]
    print(f"[{args.axis} sweep] {len(jobs)} jobs "
          f"({len(ALL)}방식 × {len(levels)}레벨 × {args.seeds}seed), {args.workers} workers",
          flush=True)

    acc = {(ci, lv): {k: [] for k in KEYS}
           for ci in range(len(ALL)) for lv in levels}
    done = 0
    with ProcessPoolExecutor(max_workers=args.workers) as ex:
        for key, out in ex.map(_job, jobs):
            for k in KEYS:
                acc[key][k].extend(out[k])
            done += 1
            if done % 25 == 0:
                print(f"  {done}/{len(jobs)}", flush=True)

    curves = {}
    for ci, cfg in enumerate(ALL):
        curves[cfg.name] = {"label": cfg.label, "levels": levels}
        for k in KEYS:
            curves[cfg.name][k] = [
                float(np.mean(acc[(ci, lv)][k])) if acc[(ci, lv)][k] else None
                for lv in levels]
    unit = {"corner": "px", "fk": "mm", "intrinsic": "rel", "outlier": "rate"}[args.axis]
    os.makedirs(os.path.dirname(dump), exist_ok=True)
    json.dump({"axis": args.axis, "levels": levels, "unit": unit,
               "meta": {"seeds": args.seeds, "pairs": args.pairs,
                        "bg_intrinsic": args.bg_intrinsic, "bg_outlier": args.bg_outlier},
               "curves": curves}, open(dump, "w"), indent=2)
    print(f"[저장] {dump}")
    # 미리보기
    for name, c in curves.items():
        print(f"  {name:<6} e_task:", [f"{v:.1f}" if v is not None else "—" for v in c["e_task_mm"]])


if __name__ == "__main__":
    main()
