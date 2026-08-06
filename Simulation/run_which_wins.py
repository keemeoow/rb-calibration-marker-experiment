#!/usr/bin/env python3
"""
"어떤 조건에서 어떤 방법이 이기나" — 4방법 × (FK오차, 카메라 랜덤/계통노이즈) 전수 sweep.

4방법:
  fixed-FK : 큐브를 FK 로 하드 고정
  no-FK    : 큐브 자유, anchor 없음, 보정 없음 (순수 vision)
  ours-A   : 큐브 자유, anchor=0, 2차 보정 O
  ours-B   : 큐브 자유, anchor=0.5, 2차 보정 O

산출(JSON):
  Fig1 fk       : FK 오차 sweep (카메라 랜덤 0.3px, 계통 0)
  Fig2 sys      : 계통노이즈 sweep (FK 완벽, 랜덤 0.3px)
  Fig3 random   : 랜덤 px sweep (FK 완벽, 계통 0)
  Fig4 grid     : FK오차 × 계통노이즈 2D 승자 히트맵 (랜덤 0.3px)

  python run_which_wins.py --seeds 25 --workers 44
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
KEYS = ["e_task_mm", "gTc_mm", "e_X_mm", "e_cross_mm", "e_reproj_px"]
N_GRIPPED = 0        # gripped 캡처 수 (main 에서 설정; fork 로 워커에 상속)

# 조건 축 레벨
FK_LEVELS = [0.0, 1.0, 2.0, 4.0, 8.0, 16.0]              # mm (회전 = mm/10 deg)
SYS_LEVELS = [0.0, 0.005, 0.01, 0.02, 0.03]              # intrinsic; outlier = sys*5
RANDOM_LEVELS = [0.3, 0.5, 1.0, 1.5, 2.0]               # px
GRID_FK = [0.0, 2.0, 4.0, 8.0, 16.0]                    # 히트맵 x
GRID_SYS = [0.0, 0.01, 0.02, 0.03]                      # 히트맵 y


def _cond(fk=0.0, sys=0.0, sigma=0.3):
    """조건 → SimScene kwargs (배경노이즈 포함)."""
    return dict(sigma_px=sigma, fk_noise_mm=fk, fk_noise_deg=fk / 10.0,
                intrinsic_err=sys, outlier_rate=sys * 5.0)


def _job(a):
    """(mi, seed, cond, n_sets, n_events, train_size, pairs, tag) → (tag, mi, metriclists)."""
    mi, seed, cond, n_sets, n_events, train_size, pairs, tag = a
    from core.scene import SimScene
    from core.experiment import calibrate, _splits_for_seed
    from core.metrics import eval_model
    cfg = METHODS[mi]
    out = {k: [] for k in KEYS}
    try:
        sc = SimScene(seed=seed, n_sets=n_sets, n_events_per_set=n_events,
                      n_gripped_events=N_GRIPPED, **cond)
        for test in _splits_for_seed(sc.sets, seed, pairs):
            train = [s for s in sc.sets if s not in test][:train_size]
            model, W = calibrate(sc, cfg, train)
            res = eval_model(sc, model, train, list(test), W=W)
            for k in KEYS:
                if res.get(k) is not None:
                    out[k].append(res[k])
    except Exception as ex:      # 한 job 실패가 풀 전체를 죽이지 않게 (빈 결과 반환)
        print(f"[FAIL] job={a}: {type(ex).__name__}: {ex}", file=sys.stderr)
    return (tag, mi, out)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seeds", type=int, default=25)
    ap.add_argument("--workers", type=int, default=max(1, os.cpu_count() - 2))
    ap.add_argument("--sets", type=int, default=10)
    ap.add_argument("--events", type=int, default=6)
    ap.add_argument("--train", type=int, default=8)
    ap.add_argument("--pairs", type=int, default=6)
    ap.add_argument("--gripped", type=int, default=0,
                    help="gripped 캡처 수(큐브 들고 회전, 고정 카메라 관측). 실제 130.")
    ap.add_argument("--outdir", type=str, default="results/tables")
    args = ap.parse_args()
    global N_GRIPPED
    N_GRIPPED = int(args.gripped)      # fork 로 워커에 상속됨

    # 모든 job 을 하나의 풀에 넣는다. tag 로 어느 그림/셀인지 구분.
    jobs = []
    # Fig1 fk
    for li, lv in enumerate(FK_LEVELS):
        for mi in range(len(METHODS)):
            for sd in range(args.seeds):
                jobs.append((mi, sd, _cond(fk=lv), args.sets, args.events, args.train,
                             args.pairs, ("fk", li)))
    # Fig2 sys
    for li, lv in enumerate(SYS_LEVELS):
        for mi in range(len(METHODS)):
            for sd in range(args.seeds):
                jobs.append((mi, sd, _cond(sys=lv), args.sets, args.events, args.train,
                             args.pairs, ("sys", li)))
    # Fig3 random
    for li, lv in enumerate(RANDOM_LEVELS):
        for mi in range(len(METHODS)):
            for sd in range(args.seeds):
                jobs.append((mi, sd, _cond(sigma=lv), args.sets, args.events, args.train,
                             args.pairs, ("rand", li)))
    # Fig4 grid
    for xi, fk in enumerate(GRID_FK):
        for yi, sy in enumerate(GRID_SYS):
            for mi in range(len(METHODS)):
                for sd in range(args.seeds):
                    jobs.append((mi, sd, _cond(fk=fk, sys=sy), args.sets, args.events,
                                 args.train, args.pairs, ("grid", xi, yi)))

    print(f"[which-wins] {len(jobs)} jobs ({len(METHODS)}방법, {args.seeds}seed), "
          f"{args.workers} workers", flush=True)

    # 누적: tag -> mi -> {key: [vals]}
    acc = {}
    done = 0
    with ProcessPoolExecutor(max_workers=args.workers) as ex:
        for tag, mi, out in ex.map(_job, jobs):
            slot = acc.setdefault(tag, {})
            m = slot.setdefault(mi, {k: [] for k in KEYS})
            for k in KEYS:
                m[k].extend(out[k])
            done += 1
            if done % 200 == 0:
                print(f"  {done}/{len(jobs)}", flush=True)

    def mean(tag, mi, k):
        v = acc.get(tag, {}).get(mi, {}).get(k, [])
        return float(np.mean(v)) if v else None

    os.makedirs(args.outdir, exist_ok=True)
    meta = dict(seeds=args.seeds, sets=args.sets, events=args.events,
                train=args.train, pairs=args.pairs)

    def dump_1d(axis, levels, xlabel):
        methods = {}
        for mi, cfg in enumerate(METHODS):
            methods[cfg.name] = {"label": cfg.label,
                                 **{k: [mean((axis, li), mi, k) for li in range(len(levels))]
                                    for k in KEYS}}
        blob = {"kind": "1d", "axis": axis, "levels": levels, "xlabel": xlabel,
                "methods": methods, "meta": meta}
        p = os.path.join(args.outdir, f"ww_{axis}.json")
        json.dump(blob, open(p, "w"), indent=2)
        print(f"[저장] {p}")
        # 미리보기
        for mi, cfg in enumerate(METHODS):
            print(f"   {cfg.name:6s} e_task:",
                  [f"{mean((axis,li),mi,'e_task_mm'):.2f}" if mean((axis,li),mi,'e_task_mm') is not None else "—"
                   for li in range(len(levels))])

    dump_1d("fk", FK_LEVELS, "FK error (mm)")
    dump_1d("sys", SYS_LEVELS, "systematic camera err (intrinsic; outlier=×5)")
    dump_1d("rand", RANDOM_LEVELS, "random pixel noise σ (px)")

    # Fig4 grid + 승자
    gm = {}
    winner = [[None] * len(GRID_SYS) for _ in range(len(GRID_FK))]
    margin = [[None] * len(GRID_SYS) for _ in range(len(GRID_FK))]
    for mi, cfg in enumerate(METHODS):
        gm[cfg.name] = {"label": cfg.label,
                        "e_task_mm": [[mean(("grid", xi, yi), mi, "e_task_mm")
                                       for yi in range(len(GRID_SYS))]
                                      for xi in range(len(GRID_FK))]}
    for xi in range(len(GRID_FK)):
        for yi in range(len(GRID_SYS)):
            vals = {cfg.name: mean(("grid", xi, yi), mi, "e_task_mm")
                    for mi, cfg in enumerate(METHODS)}
            vals = {n: v for n, v in vals.items() if v is not None}
            if not vals:
                continue
            srt = sorted(vals.items(), key=lambda kv: kv[1])
            winner[xi][yi] = srt[0][0]
            if len(srt) > 1:
                margin[xi][yi] = srt[1][1] - srt[0][1]   # 2등 - 1등 (mm)
    grid = {"kind": "2d", "x_axis": "fk", "x_levels": GRID_FK,
            "y_axis": "sys", "y_levels": GRID_SYS,
            "methods": gm, "winner": winner, "margin": margin, "meta": meta}
    p = os.path.join(args.outdir, "ww_grid.json")
    json.dump(grid, open(p, "w"), indent=2)
    print(f"[저장] {p}")
    print("[승자 히트맵] 행=FK오차, 열=계통노이즈")
    print("        sys:", GRID_SYS)
    for xi, fk in enumerate(GRID_FK):
        print(f"  fk={fk:4.0f}:", [f"{winner[xi][yi] or '—':6s}" for yi in range(len(GRID_SYS))])


if __name__ == "__main__":
    main()
