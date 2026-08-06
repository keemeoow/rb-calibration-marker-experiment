#!/usr/bin/env python3
"""
Table 2a (코너 수준, full-seed) — 절제 실험을 CPU 병렬로 산출 + JSON 저장.

통계 규약:
  * 집계 단위는 **seed**. seed 안의 여러 split 은 같은 씬을 공유해 독립 표본이 아니므로
    먼저 split 을 평균한 뒤 seed 간 통계를 낸다 (n = seed 수).
  * Ours(EXP1) 대비 **paired 차이**를 같은 seed 끼리 짝지어 계산하고, seed 를 재표집하는
    paired bootstrap 으로 95% CI 를 낸다. CI 전체가 0 보다 작아야 "Ours 가 유의하게 우수".
  * N_reg 가 다른 행은 e_X/bTf 를 직접 비교하면 안 된다 (등록 대수가 적으면 낮게 보임).

  python run_table2a.py --seeds 20 --workers 8 --dump results/tables/table2a.json
"""
import sys, os, argparse, json
from concurrent.futures import ProcessPoolExecutor
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from core import KEYS
from configs import ALL

# 표에 찍을 열 (전체 KEYS 는 JSON 에 그대로 저장)
SHOW = ["N_reg", "bTf_mm", "bTf_deg", "gTc_mm", "e_task_mm", "e_task_deg",
        "reproj_test_px", "e_cross_mm"]
# Ours 대비 paired 비교를 볼 지표 (작을수록 좋음)
PAIRED = ["bTf_mm", "gTc_mm", "e_task_mm", "e_task_deg", "reproj_test_px", "e_cross_mm"]


def _one_seed(args):
    """(cfg_idx, seed, ...) → 그 seed 의 split 평균 지표 dict."""
    (cfg_idx, seed, sigma_px, n_sets, n_events, train_size, n_splits,
     fk_mm, fk_deg) = args
    from core.scene import SimScene
    from core.experiment import calibrate, _splits_for_seed
    from core.metrics import eval_model
    cfg = ALL[cfg_idx]
    sc = SimScene(seed=seed, n_sets=n_sets, n_events_per_set=n_events, sigma_px=sigma_px,
                  fk_noise_mm=fk_mm, fk_noise_deg=fk_deg)
    per_split = []
    for test in _splits_for_seed(sc.sets, seed, n_splits):
        train = [s for s in sc.sets if s not in test][:train_size]
        model, W = calibrate(sc, cfg, train)
        per_split.append(eval_model(sc, model, train, list(test), W=W))
    out = {}
    for k in KEYS:
        v = [r[k] for r in per_split if r.get(k) is not None]
        out[k] = float(np.mean(v)) if v else None
    return cfg_idx, seed, out


def paired_bootstrap(diff, n_boot=10000, seed=0):
    """seed 단위 차이 배열 → (mean, lo95, hi95). 재표집 단위는 seed."""
    d = np.asarray([x for x in diff if x is not None], float)
    if len(d) < 2:
        return None, None, None
    rng = np.random.default_rng(seed)
    bs = d[rng.integers(0, len(d), size=(n_boot, len(d)))].mean(axis=1)
    return float(d.mean()), float(np.percentile(bs, 2.5)), float(np.percentile(bs, 97.5))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seeds", type=int, default=20)
    ap.add_argument("--workers", type=int, default=max(1, (os.cpu_count() or 4) - 2))
    ap.add_argument("--sigma_px", type=float, default=0.3)
    ap.add_argument("--sets", type=int, default=10)
    ap.add_argument("--events", type=int, default=6)
    ap.add_argument("--train", type=int, default=8)
    ap.add_argument("--splits", type=int, default=3)
    ap.add_argument("--ref", type=str, default="EXP1", help="paired 비교 기준 (Ours)")
    # FK 노이즈 regime — **결과 해석이 여기에 크게 좌우된다**. fk=0 은 FK 가 완벽한
    # 비현실적 조건이며 하드 고정(EXP7)에 유리하다. 실로봇 FK 오차 수준에서도 함께 볼 것.
    ap.add_argument("--fk_noise_mm", type=float, default=0.0)
    ap.add_argument("--fk_noise_deg", type=float, default=0.0)
    ap.add_argument("--dump", type=str, default="results/tables/table2a.json")
    args = ap.parse_args()

    jobs = [(ci, seed, args.sigma_px, args.sets, args.events, args.train, args.splits,
             args.fk_noise_mm, args.fk_noise_deg)
            for ci in range(len(ALL)) for seed in range(args.seeds)]
    print(f"[병렬] {len(jobs)} jobs ({len(ALL)} exp × {args.seeds} seed), {args.workers} workers")
    print(f"[regime] sigma_px={args.sigma_px}  FK noise={args.fk_noise_mm}mm/"
          f"{args.fk_noise_deg}°  sets={args.sets}  events={args.events}")
    # per_seed[cfg_idx][seed] = 지표 dict  (paired 비교를 위해 seed 를 키로 보관)
    per_seed = {ci: {} for ci in range(len(ALL))}
    done = 0
    with ProcessPoolExecutor(max_workers=args.workers) as ex:
        for cfg_idx, seed, out in ex.map(_one_seed, jobs):
            per_seed[cfg_idx][seed] = out
            done += 1
            if done % 10 == 0:
                print(f"  {done}/{len(jobs)} seeds done", flush=True)

    def stats(ci):
        st = {}
        for k in KEYS:
            v = [per_seed[ci][s][k] for s in sorted(per_seed[ci])
                 if per_seed[ci][s].get(k) is not None]
            st[k] = (float(np.mean(v)), float(np.std(v)), len(v)) if v else (None, None, 0)
        return st

    rows = {}
    print("\n" + "=" * 108)
    print(f" Table 2a — Corner-level ablation (seed 단위 집계, n={args.seeds})")
    print("=" * 108)
    hdr = f"{'#':<6}{'설명':<26}" + "".join(f"{k.replace('_mm','').replace('_px',''):>13}" for k in SHOW)
    print(hdr)
    print("-" * len(hdr))
    for ci, cfg in enumerate(ALL):
        st = stats(ci)
        rows[cfg.name] = {"config": cfg.__dict__, "stats": st,
                          "per_seed": {str(s): per_seed[ci][s] for s in sorted(per_seed[ci])}}
        def g(k, d=2):
            m = st[k][0]
            return f"{m:.{d}f}" if m is not None else "—"
        print(f"{cfg.name:<6}{cfg.label[:24]:<26}" + "".join(f"{g(k):>13}" for k in SHOW))
    print("-" * len(hdr))
    print("N_reg 가 다른 행끼리 bTf/e_X 직접 비교 금지 (등록 대수가 적으면 낮게 보임).")

    # ---- Ours 대비 paired bootstrap ----
    ref_i = next((i for i, c in enumerate(ALL) if c.name == args.ref), None)
    paired = {}
    if ref_i is not None:
        print(f"\n{args.ref} 대비 paired 차이 (다른방법 − {args.ref}; 양수 = Ours 가 더 좋음)")
        print(f"{'#':<6}{'지표':<18}{'Δmean':>10}{'95% CI':>22}{'판정':>10}")
        print("-" * 66)
        for ci, cfg in enumerate(ALL):
            if ci == ref_i:
                continue
            # N_reg 가 다르면 커버리지가 달라 bTf/e_cross 비교가 성립하지 않는다.
            nref = stats(ref_i)["N_reg"][0]; ncur = stats(ci)["N_reg"][0]
            mismatch = (nref is not None and ncur is not None
                        and abs(nref - ncur) > 1e-9)
            if mismatch:
                print(f"{cfg.name:<6}{'(N_reg %.2f vs %.2f — 커버리지가 달라 비교 불가)' % (ncur, nref)}")
            paired[cfg.name] = {"_n_reg_mismatch": mismatch}
            for k in PAIRED:
                diff = []
                for s in sorted(per_seed[ref_i]):
                    a, b = per_seed[ci].get(s, {}).get(k), per_seed[ref_i][s].get(k)
                    if a is not None and b is not None:
                        diff.append(a - b)          # >0 이면 Ours 가 더 낮음(=좋음)
                m, lo, hi = paired_bootstrap(diff)
                paired[cfg.name][k] = {"mean": m, "lo95": lo, "hi95": hi, "n": len(diff)}
                if m is None:
                    continue
                if mismatch:
                    verdict = "비교불가"
                else:
                    verdict = "Ours 우세" if lo > 0 else ("열세" if hi < 0 else "무결론")
                print(f"{cfg.name:<6}{k:<18}{m:>10.3f}{f'[{lo:+.3f}, {hi:+.3f}]':>22}{verdict:>10}")
        print("-" * 66)
        print("CI 가 0 을 포함하면 이 표본수로는 결론 불가 (seed 를 늘려야 함).")

    os.makedirs(os.path.dirname(args.dump), exist_ok=True)
    json.dump({"meta": {"seeds": args.seeds, "sigma_px": args.sigma_px, "sets": args.sets,
                        "events": args.events, "splits": args.splits, "ref": args.ref,
                        "unit": "seed (split 은 seed 내에서 먼저 평균)"},
               "rows": rows, "paired_vs_ref": paired},
              open(args.dump, "w"), indent=2, default=list)
    print(f"[저장] {args.dump}")


if __name__ == "__main__":
    main()
