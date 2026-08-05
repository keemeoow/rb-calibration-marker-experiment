#!/usr/bin/env python3
"""
논문용 시뮬 실험 — 7방법(EXP1~7) × 노이즈 조건. 표1·1b·그림A·B 용 데이터 산출.
약식 프로토콜(10 sets × 6 eih + gripped 40, ~38% 비율). GT 기준 지표.
  python run_paper_sim.py --seeds 4 --workers 12
"""
import sys, os, argparse, json, itertools
from concurrent.futures import ProcessPoolExecutor
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from configs import ALL          # EXP1~7 (7방법)

KEYS = ["e_task_mm", "e_task_deg", "e_X_mm", "gTc_mm", "e_rel_mm",
        "e_reproj_px", "e_reproj_raw_px", "e_cross_mm"]
# 실제 규모/실측 노이즈 (main 에서 args 로 설정; fork 로 워커 상속)
N_GRIPPED = 89                   # 실측 data/session: gripped 89
N_SETS = 13                      # 실측: 13 sets
N_EVENTS = 11                    # 실측: eye-in-hand 11 shots/set
N_SPLITS = 3                     # (method,cond,seed) 당 held-out split 수 (평균)
OUTLIER_PX = 2.0                 # 실측: 오검출 크기 ~1-2px (max 2.0)
BASE_SIGMA = 0.2                 # 실측: 코너 σ median 0.15~mean 0.19
SYS_FK_REAL = 6.6                # 실측 systematic FK 위치오차 median (realistic 'FK 있음')

# 노이즈 축 (그림 A) — 한 축만 변화, 나머지는 baseline(실측값). 실측 기준으로 스윕.
#   FK 는 두 종류를 분리: systematic(위치의존, 학습가능=보정 대상) vs random(제로평균, 학습불가).
SIGMAS  = [0.2, 0.3, 0.5, 1.0, 1.5]
SYSS    = [0.0, 0.005, 0.01, 0.02]        # 계통 intrinsic (실측 <1%)
FK_SYS  = [0.0, 2.0, 5.0, 6.6, 10.0]      # systematic FK mm (실측 median 6.6) — 주 FK축
FK_RAND = [0.0, 2.0, 5.0, 10.0, 16.0]     # random FK mm (대조: 보정 불가)
OUTS    = [0.0, 0.02, 0.05, 0.10]         # 오검출률 (실측 ~2%)
# 그림 B 격자: 계통 × 오검출 (FK=0)
GRID_SYS = [0.0, 0.005, 0.01, 0.02]
GRID_OUT = [0.0, 0.02, 0.05, 0.10]
# 표 조건 — cond = (sigma, sys, fk_random, fk_systematic, outlier). 실측값 기반.
#   realistic(FK 없음/정확) 과 realistic_sysfk(FK 있음/실측 systematic) 를 나란히 비교.
TABLE_CONDS = {
    "ideal":           (0.0, 0.0,   0.0, 0.0,         0.0),
    "realistic":       (0.2, 0.005, 0.0, 0.0,         0.02),   # FK 없음: σ0.2+계통0.5%+오검출2%
    "realistic_sysfk": (0.2, 0.005, 0.0, SYS_FK_REAL, 0.02),   # FK 있음: +systematic FK 6.6mm(실측)
    "fk_sys":          (0.2, 0.0,   0.0, SYS_FK_REAL, 0.0),    # systematic FK 격리
    "fk_rand":         (0.2, 0.0,   5.0, 0.0,         0.0),    # random FK 격리 (대조)
    "outlier":         (0.2, 0.0,   0.0, 0.0,         0.05),
}


def _ckey(sigma, sysv, fkr, fks, outl):
    return f"{sigma:.3f}_{sysv:.3f}_{fkr:.3f}_{fks:.3f}_{outl:.3f}"


def _all_conditions():
    """필요한 모든 (sigma,sys,fk_rand,fk_sys,outl) 유니크 집합 + 레이아웃."""
    conds = {}
    def add(c): conds[_ckey(*c)] = c
    layout = {"figA": {}, "figB": {"sys": GRID_SYS, "out": GRID_OUT, "cells": {}}, "table": {}}
    # 그림 A (비-스윕 축은 baseline=BASE_SIGMA·나머지0). FK 축은 systematic 을 주로.
    B = BASE_SIGMA
    for s in SIGMAS:  add((s, 0.0, 0.0, 0.0, 0.0))
    for v in SYSS:    add((B, v, 0.0, 0.0, 0.0))
    for f in FK_SYS:  add((B, 0.0, 0.0, f, 0.0))
    for f in FK_RAND: add((B, 0.0, f, 0.0, 0.0))
    for o in OUTS:    add((B, 0.0, 0.0, 0.0, o))
    layout["figA"]["sigma"]  = {"levels": SIGMAS,  "keys": [_ckey(s,0,0,0,0) for s in SIGMAS]}
    layout["figA"]["sys"]    = {"levels": SYSS,    "keys": [_ckey(B,v,0,0,0) for v in SYSS]}
    layout["figA"]["fk_sys"] = {"levels": FK_SYS,  "keys": [_ckey(B,0,0,f,0) for f in FK_SYS]}
    layout["figA"]["outl"]   = {"levels": OUTS,    "keys": [_ckey(B,0,0,0,o) for o in OUTS]}
    # 대조: random FK sweep (보정 불가 확인용)
    layout["fk_rand"] = {"levels": FK_RAND, "keys": [_ckey(B,0,f,0,0) for f in FK_RAND]}
    # 그림 B (계통 × 오검출, FK=0)
    for xi, v in enumerate(GRID_SYS):
        for yi, o in enumerate(GRID_OUT):
            add((B, v, 0.0, 0.0, o))
            layout["figB"]["cells"][f"{xi}_{yi}"] = _ckey(B, v, 0.0, 0.0, o)
    # 표
    for name, c in TABLE_CONDS.items():
        add(c); layout["table"][name] = _ckey(*c)
    return conds, layout


def _job(a):
    mi, seed, cond = a
    from core.scene import SimScene
    from core.experiment import calibrate
    from core.metrics import eval_model
    sigma, sysv, fkr, fks, outl = cond
    cfg = ALL[mi]
    acc = {k: [] for k in KEYS}
    try:
        sc = SimScene(seed=seed, n_sets=N_SETS, n_events_per_set=N_EVENTS,
                      sigma_px=sigma, fk_noise_mm=fkr, fk_noise_deg=fkr / 10.0,
                      fk_sys_mm=fks, fk_sys_deg=fks * 0.08,   # 실측 dr~0.5°@6.6mm 비율
                      intrinsic_err=sysv, outlier_rate=outl, outlier_px=OUTLIER_PX,
                      n_gripped_events=N_GRIPPED)
        n = 0
        for test in itertools.combinations(sc.sets, 2):
            tr = [s for s in sc.sets if s not in test][:max(2, N_SETS - 2)]
            model, W = calibrate(sc, cfg, tr)
            res = eval_model(sc, model, tr, list(test), W=W)
            for k in KEYS:
                if res.get(k) is not None:
                    acc[k].append(res[k])
            n += 1
            if n >= N_SPLITS:
                break
    except Exception as e:      # 실패 숨기지 않고 표면화 (리뷰 ⑤). 풀은 죽이지 않음.
        import traceback
        print(f"[FAIL] {cfg.name} seed={seed} cond={_ckey(*cond)}: "
              f"{type(e).__name__}: {e}", file=sys.stderr, flush=True)
        traceback.print_exc()
    # split 별 raw 값 그대로 반환 → main 에서 median·발산율 계산 (리뷰 ⑤: robust 집계)
    return (mi, _ckey(*cond)), {k: list(v) for k, v in acc.items()}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seeds", type=int, default=4)
    ap.add_argument("--workers", type=int, default=12)
    ap.add_argument("--gripped", type=int, default=89)      # 실측 data/session
    ap.add_argument("--sets", type=int, default=13)
    ap.add_argument("--events", type=int, default=11)       # 실측 eih/set
    ap.add_argument("--splits", type=int, default=3)
    args = ap.parse_args()
    global N_GRIPPED, N_SETS, N_EVENTS, N_SPLITS
    N_GRIPPED = int(args.gripped); N_SETS = int(args.sets); N_EVENTS = int(args.events)
    N_SPLITS = int(args.splits)

    conds, layout = _all_conditions()
    jobs = [(mi, sd, conds[ck]) for mi in range(len(ALL))
            for ck in conds for sd in range(args.seeds)]
    print(f"[paper-sim] {len(jobs)} jobs (7방법 × {len(conds)}조건 × {args.seeds}seed, "
          f"gripped {N_GRIPPED}), {args.workers} workers", flush=True)

    agg = {}     # (mi, ckey) -> {metric: [모든 seed×split raw vals]}
    done = 0
    with ProcessPoolExecutor(max_workers=args.workers) as ex:
        for key, d in ex.map(_job, jobs):
            slot = agg.setdefault(key, {k: [] for k in KEYS})
            for k in KEYS:
                vs = d.get(k)
                if vs:
                    slot[k].extend(vs)      # seed·split raw 값 pool (append 아님)
            done += 1
            if done % 40 == 0:
                print(f"  {done}/{len(jobs)}", flush=True)

    DIVERGE_MM = 100.0    # e_task 100mm 초과(또는 비유한) = 발산(수렴 실패)로 간주

    def stat(mi, ck, metric):
        """robust 대표값 = median (발산 1건이 평균 파괴하는 것 방지; 리뷰 ⑤).
           비유한값(inf/nan)은 median 에서 제외 (JSON·median 오염 방지)."""
        v = [x for x in agg.get((mi, ck), {}).get(metric, []) if np.isfinite(x)]
        return float(np.median(v)) if v else None

    def diverge_rate(mi, ck):
        """e_task 가 DIVERGE_MM 초과 or 비유한(inf/nan) 인 split 비율 (수렴 실패율 — 정직한 보고)."""
        v = agg.get((mi, ck), {}).get("e_task_mm", [])
        if not v:
            return None
        return float(np.mean([1.0 if (not np.isfinite(x) or x > DIVERGE_MM) else 0.0 for x in v]))

    def nsamp(mi, ck):
        return len(agg.get((mi, ck), {}).get("e_task_mm", []))

    # results[ckey][method_name][metric]  (대표값=median) + _diverge/_n 부가
    results = {}
    for ck in conds:
        results[ck] = {}
        for mi, cfg in enumerate(ALL):
            r = {k: stat(mi, ck, k) for k in KEYS}
            r["_diverge"] = diverge_rate(mi, ck)
            r["_n"] = nsamp(mi, ck)
            results[ck][cfg.name] = r

    out = {"methods": [c.name for c in ALL],
           "method_labels": [c.label for c in ALL],
           "meta": {"seeds": args.seeds, "gripped": N_GRIPPED, "splits": N_SPLITS,
                    "agg": "median", "diverge_mm": DIVERGE_MM,
                    "protocol": "%d sets x %d eih + gripped %d" % (N_SETS, N_EVENTS, N_GRIPPED)},
           "layout": layout, "results": results}
    os.makedirs("results/tables", exist_ok=True)
    json.dump(out, open("results/tables/paper_sim.json", "w"), indent=2)
    print("[저장] results/tables/paper_sim.json")
    # 미리보기: FK 없음(realistic) vs FK 있음(realistic_sysfk) — median e_task
    r0 = layout["table"]["realistic"]; r1 = layout["table"]["realistic_sysfk"]
    print("\nFK 없음(realistic) vs FK 있음(systematic 6.6mm) — median e_task mm | reproj_raw px:")
    print(f"  {'방법':22s} {'task(없음)':>10s} {'task(있음)':>10s} {'rawpx(없음)':>11s} {'rawpx(있음)':>11s}")
    for cfg in ALL:
        a = results[r0][cfg.name]; b = results[r1][cfg.name]
        def g(d, k): return f"{d[k]:.2f}" if d.get(k) is not None else "—"
        print(f"  {cfg.name} {cfg.label:18s} {g(a,'e_task_mm'):>10s} {g(b,'e_task_mm'):>10s} "
              f"{g(a,'e_reproj_raw_px'):>11s} {g(b,'e_reproj_raw_px'):>11s}")


if __name__ == "__main__":
    main()
