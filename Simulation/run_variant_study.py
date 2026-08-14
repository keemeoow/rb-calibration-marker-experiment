#!/usr/bin/env python3
"""corrected-FK 가 no-FK 와 fixed-FK 를 분명히 이기는 방식을 찾는다.

문제의식
  현재 corrected-FK 는 anchor 를 vision 합의에서 만들어 상수로 고정한다.
  그러면 FK 가 새로 넣어주는 정보가 거의 없어 no-FK 와 비긴다.
  FK 의 진짜 강점은 반복성, 즉 세트 사이의 상대 관계다. 이를 살리는 변형을 겨룬다.

겨루는 방식
  no-FK      FK 미사용
  fixed-FK   raw FK 로 큐브를 고정
  corr-hard  gate 통과 anchor 를 상수로 고정          (현재 방식)
  corr-soft  anchor 를 벌점으로 당기고 큐브는 자유     (bias-variance 절충)
  corr-rel   세트 쌍의 상대 변환만 FK 로 구속          (절대 편향 면역)
  corr-init  anchor 를 초기값으로만 사용               (발산 방지 효과만)
  corr-soft-w  관측이 적은 세트일수록 FK 를 더 믿는다

조건
  A. FK 계통오차 0 / 3 / 9 mm
  B. 학습 세트 4 / 6 / 11 개  (세트가 적을 때 no-FK 가 흔들리는 구간 포함)

실행:
  python3.10 run_variant_study.py --seeds 12 --workers 32
"""
import os
import sys
import json
import argparse
from concurrent.futures import ProcessPoolExecutor

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from core import ExpConfig, run_config

for _p in ("/usr/share/fonts/truetype/nanum/NanumBarunGothic.ttf",
           "/usr/share/fonts/truetype/nanum/NanumGothic.ttf"):
    if os.path.exists(_p):
        font_manager.fontManager.addfont(_p)
        plt.rcParams["font.family"] = font_manager.FontProperties(fname=_p).get_name()
        break
plt.rcParams["axes.unicode_minus"] = False

INK, INK2, MUTED = "#0b0b0b", "#52514e", "#8c8b85"
GRID, SURFACE = "#e6e5e0", "#fcfcfb"
CB = ("cube", "board")
BASE = dict(n_sets=14, n_splits=1, n_events_per_set=6, n_gripped_events=0,
            use_real_layout=True, sigma_px=0.5, intrinsic_err=0.01)

# (키, 라벨, 색, ExpConfig 인자)
VARIANTS = [
    ("noFK",   "no-FK",       "#2a78d6", dict(fk="none")),
    ("fixed",  "fixed-FK",    "#eb6834", dict(fk="fixed")),
    ("hard",   "corr-hard",   "#8c8b85", dict(fk="corr", corr_variant="hard")),
    ("soft",   "corr-soft",   "#1baf7a", dict(fk="corr", corr_variant="soft")),
    ("rel",    "corr-rel",    "#4a3aa7", dict(fk="corr", corr_variant="rel")),
    ("init",   "corr-init",   "#eda100", dict(fk="corr", corr_variant="init")),
    ("softw",  "corr-soft-w", "#e87ba4", dict(fk="corr", corr_variant="soft",
                                              corr_weight_by_support=True)),
]
FK_LEVELS = [0.0, 3.0, 9.0]
TRAIN_SIZES = [4, 6, 11]
LAMBDAS = [0.1, 0.3, 1.0]      # soft/rel 가중치 후보


def _job(spec):
    key, cfg_kw, run_kw, seeds = spec
    res = run_config(ExpConfig("m", solve="unified", markers=CB, **cfg_kw),
                     seeds=seeds, **run_kw)
    m, sd, n = res["e_task_mm"]
    return key, (m, sd, n)


def _style(ax, xlabel, ylabel, title):
    ax.set_facecolor(SURFACE); ax.figure.set_facecolor(SURFACE)
    ax.set_xlabel(xlabel, color=INK2, fontsize=9)
    ax.set_ylabel(ylabel, color=INK2, fontsize=9)
    ax.set_title(title, color=INK, fontsize=10.5, pad=8, loc="left")
    ax.grid(True, color=GRID, linewidth=0.8, zorder=0); ax.set_axisbelow(True)
    for s in ("top", "right"):
        ax.spines[s].set_visible(False)
    for s in ("left", "bottom"):
        ax.spines[s].set_color(GRID)
    ax.tick_params(colors=MUTED, labelsize=8)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seeds", type=int, default=12)
    ap.add_argument("--workers", type=int, default=32)
    ap.add_argument("--replot", action="store_true")
    a = ap.parse_args()
    outdir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "figs_real")
    os.makedirs(outdir, exist_ok=True)
    raw_path = os.path.join(outdir, "variant_raw.json")

    if a.replot:
        out = {k: tuple(v) for k, v in json.load(open(raw_path)).items()}
    else:
        jobs = []
        # 1) 람다 고르기: 세트 11개 · FK 3mm 에서 soft/rel 의 가중치를 훑는다
        for vk in ("soft", "rel"):
            for lam in LAMBDAS:
                kw = dict(fk="corr", corr_variant=vk, corr_lambda=lam)
                jobs.append((f"L|{vk}|{lam}",
                             kw, dict(BASE, fk_sys_mm=3.0, train_size=11), a.seeds))
        # 2) 본 비교: 방법 × FK 수준 × 세트 수
        for vk, _, _, cfg_kw in VARIANTS:
            for fk_mm in FK_LEVELS:
                for ts in TRAIN_SIZES:
                    jobs.append((f"M|{vk}|{fk_mm}|{ts}", cfg_kw,
                                 dict(BASE, fk_sys_mm=fk_mm, train_size=ts), a.seeds))
        print(f"총 {len(jobs)}개 설정 × seed {a.seeds}개", flush=True)
        with ProcessPoolExecutor(max_workers=a.workers) as ex:
            out = {}
            for i, (k, v) in enumerate(ex.map(_job, jobs), 1):
                out[k] = v
                if i % 15 == 0:
                    print(f"  {i}/{len(jobs)}", flush=True)
        json.dump({k: list(v) for k, v in out.items()}, open(raw_path, "w"), indent=1)

    # ── 그림: FK 수준 × 세트 수 격자 ──
    fig, axes = plt.subplots(len(TRAIN_SIZES), len(FK_LEVELS),
                             figsize=(15, 10.5), sharex=True)
    xs = np.arange(len(VARIANTS))
    for r, ts in enumerate(TRAIN_SIZES):
        for c, fk_mm in enumerate(FK_LEVELS):
            ax = axes[r][c]
            vals = [out[f"M|{vk}|{fk_mm}|{ts}"][0] for vk, _, _, _ in VARIANTS]
            errs = [out[f"M|{vk}|{fk_mm}|{ts}"][1]
                    / np.sqrt(max(out[f"M|{vk}|{fk_mm}|{ts}"][2], 1))
                    for vk, _, _, _ in VARIANTS]
            colors = [c3 for _, _, c3, _ in VARIANTS]
            ax.bar(xs, vals, yerr=errs, width=0.68, color=colors, zorder=3,
                   edgecolor=SURFACE, linewidth=2,
                   error_kw=dict(ecolor=MUTED, lw=1.2, capsize=3))
            best = int(np.argmin(vals))
            ax.annotate("최저", (xs[best], vals[best]), textcoords="offset points",
                        xytext=(0, 12), ha="center", fontsize=8.5,
                        color=INK, fontweight="bold")
            _style(ax, "", "held-out 작업 오차 (mm)" if c == 0 else "",
                   f"학습 {ts}세트 · FK 계통 {fk_mm:g}mm")
            ax.set_xticks(xs)
            ax.set_xticklabels([lab for _, lab, _, _ in VARIANTS],
                               rotation=30, ha="right", fontsize=8)
    fig.suptitle("corrected-FK 변형 겨루기 (session02 실측 배치)",
                 color=INK, fontsize=13, x=0.007, ha="left", y=0.997)
    fig.tight_layout(rect=[0, 0, 1, 0.975])
    path = os.path.join(outdir, "13_variant_bakeoff.png")
    fig.savefig(path, dpi=150, facecolor=SURFACE)
    plt.close(fig)
    print("→", path, flush=True)

    # ── 요약 ──
    L = ["# corrected-FK 변형 겨루기\n",
         "session02 실측 배치. 값은 held-out 작업 오차(mm).\n",
         "## 가중치 고르기 (세트 11개 · FK 계통 3mm)\n",
         "| 변형 | " + " | ".join(f"λ={l}" for l in LAMBDAS) + " |",
         "|---|" + "---|" * len(LAMBDAS)]
    for vk in ("soft", "rel"):
        L.append(f"| corr-{vk} | "
                 + " | ".join(f"{out[f'L|{vk}|{l}'][0]:.3f}" for l in LAMBDAS) + " |")
    L.append("")
    for ts in TRAIN_SIZES:
        L.append(f"## 학습 {ts}세트\n")
        L.append("| 방법 | " + " | ".join(f"FK {f:g}mm" for f in FK_LEVELS) + " |")
        L.append("|---|" + "---|" * len(FK_LEVELS))
        for vk, lab, _, _ in VARIANTS:
            L.append(f"| {lab} | " + " | ".join(
                f"{out[f'M|{vk}|{f}|{ts}'][0]:.2f}" for f in FK_LEVELS) + " |")
        L.append("")
    open(os.path.join(outdir, "variant_summary.md"), "w").write("\n".join(L))
    print("완료.", outdir, flush=True)


if __name__ == "__main__":
    main()
