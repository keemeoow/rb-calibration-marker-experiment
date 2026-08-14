#!/usr/bin/env python3
"""관측이 부족해질 때 세 FK 방식이 어떻게 갈리는지 본다.

앞선 실험(run_real_study.py)은 세트 14개를 카메라 3대가 전부 보는 조건이었고,
거기서는 no-FK 가 최소한 동률이었다. FK 를 쓸 이유가 보이지 않았다.
관측이 부족해지면 no-FK 의 자유변수가 부담이 되어 상황이 달라지는지 확인한다.

두 축
  H. 학습 세트 수를 3개까지 줄인다        (자유변수 대비 관측 비율 악화)
  I. 세트를 보는 고정 카메라 수를 줄인다   (큐브 자세를 결정할 정보 감소)

각각 FK 계통오차 두 조건(3mm / 9mm)에서 본다.

실행:
  python3.10 run_sets_study.py --seeds 8 --workers 32
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
plt.rcParams["mathtext.fontset"] = "dejavusans"

C_NOFK, C_FIXED, C_OURS = "#2a78d6", "#eb6834", "#1baf7a"
INK, INK2, MUTED = "#0b0b0b", "#52514e", "#8c8b85"
GRID, SURFACE = "#e6e5e0", "#fcfcfb"

CB = ("cube", "board")
BASE = dict(n_sets=14, n_splits=1, n_events_per_set=6, n_gripped_events=0,
            use_real_layout=True, sigma_px=0.5, intrinsic_err=0.01)
METHODS = [("noFK", "no-FK", C_NOFK, "none"),
           ("fixed", "fixed-FK", C_FIXED, "fixed"),
           ("ours", "corrected-FK", C_OURS, "corr")]

TRAIN_SIZES = [3, 4, 5, 6, 8, 11]
CAM_COUNTS = [1, 2, 3]
FK_LEVELS = [(3.0, "FK 계통 3mm"), (9.0, "FK 계통 9mm")]


def _job(spec):
    key, fk, kw, seeds = spec
    res = run_config(ExpConfig("m", fk=fk, solve="unified", markers=CB),
                     seeds=seeds, **kw)
    m, sd, n = res["e_task_mm"]
    return key, (m, sd, n)


def _style(ax, xlabel, ylabel, title):
    ax.set_facecolor(SURFACE)
    ax.figure.set_facecolor(SURFACE)
    ax.set_xlabel(xlabel, color=INK2, fontsize=9.5)
    ax.set_ylabel(ylabel, color=INK2, fontsize=9.5)
    ax.set_title(title, color=INK, fontsize=11, pad=9, loc="left")
    ax.grid(True, color=GRID, linewidth=0.8, zorder=0)
    ax.set_axisbelow(True)
    for s in ("top", "right"):
        ax.spines[s].set_visible(False)
    for s in ("left", "bottom"):
        ax.spines[s].set_color(GRID)
    ax.tick_params(colors=MUTED, labelsize=8.5)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seeds", type=int, default=8)
    ap.add_argument("--workers", type=int, default=32)
    ap.add_argument("--replot", action="store_true")
    a = ap.parse_args()
    outdir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "figs_real")
    os.makedirs(outdir, exist_ok=True)
    raw_path = os.path.join(outdir, "sets_raw.json")

    if a.replot:
        out = {k: tuple(v) for k, v in json.load(open(raw_path)).items()}
    else:
        jobs = []
        for fk_mm, _ in FK_LEVELS:
            for mk, _, _, fk in METHODS:
                for ts in TRAIN_SIZES:
                    kw = dict(BASE, fk_sys_mm=fk_mm, train_size=ts)
                    jobs.append((f"H|{fk_mm}|{mk}|{ts}", fk, kw, a.seeds))
                for nc in CAM_COUNTS:
                    kw = dict(BASE, fk_sys_mm=fk_mm, train_size=11,
                              max_cams_per_set=nc)
                    jobs.append((f"I|{fk_mm}|{mk}|{nc}", fk, kw, a.seeds))
        print(f"총 {len(jobs)}개 설정 × seed {a.seeds}개", flush=True)
        with ProcessPoolExecutor(max_workers=a.workers) as ex:
            out = {}
            for i, (k, v) in enumerate(ex.map(_job, jobs), 1):
                out[k] = v
                if i % 10 == 0:
                    print(f"  {i}/{len(jobs)}", flush=True)
        json.dump({k: list(v) for k, v in out.items()}, open(raw_path, "w"), indent=1)

    fig, axes = plt.subplots(2, 2, figsize=(13.5, 8.6))
    panels = [("H", TRAIN_SIZES, "학습에 쓴 세트 수 (전체 14개 중)"),
              ("I", CAM_COUNTS, "세트를 보는 고정 카메라 수")]
    for r, (fk_mm, fk_name) in enumerate(FK_LEVELS):
        for c, (tag, levels, xlabel) in enumerate(panels):
            ax = axes[r][c]
            for mk, label, color, _ in METHODS:
                ys = np.array([out[f"{tag}|{fk_mm}|{mk}|{v}"][0] for v in levels])
                sd = np.array([out[f"{tag}|{fk_mm}|{mk}|{v}"][1] for v in levels])
                n = max(out[f"{tag}|{fk_mm}|{mk}|{levels[0]}"][2], 1)
                se = sd / np.sqrt(n)
                ax.fill_between(levels, ys - se, ys + se, color=color,
                                alpha=0.13, linewidth=0, zorder=2)
                ax.plot(levels, ys, color=color, linewidth=2.0, marker="o",
                        markersize=8, markeredgecolor=SURFACE, markeredgewidth=2,
                        label=label, zorder=3)
            _style(ax, xlabel, "held-out 작업 오차 (mm)", f"{fk_name}")
            ax.set_xticks(levels)
            if r == 0 and c == 0:
                ax.legend(loc="upper right", frameon=False, fontsize=9)
    fig.suptitle("관측이 부족해질 때 세 FK 방식", color=INK, fontsize=13,
                 x=0.007, ha="left", y=0.995)
    fig.tight_layout(rect=[0, 0, 1, 0.975])
    path = os.path.join(outdir, "12_sparse_observation.png")
    fig.savefig(path, dpi=160, facecolor=SURFACE)
    plt.close(fig)
    print("→", path, flush=True)

    L = ["# 관측이 부족할 때 세 FK 방식\n",
         "session02 실측 배치. 값은 held-out 작업 오차(mm), seed 8개.\n"]
    for fk_mm, fk_name in FK_LEVELS:
        for tag, levels, xlabel in panels:
            L.append(f"## {fk_name} · {xlabel}\n")
            L.append("| 방법 | " + " | ".join(str(v) for v in levels) + " |")
            L.append("|---|" + "---|" * len(levels))
            for mk, label, _, _f in METHODS:
                ys = [out[f"{tag}|{fk_mm}|{mk}|{v}"][0] for v in levels]
                L.append(f"| {label} | " + " | ".join(f"{y:.2f}" for y in ys) + " |")
            L.append("")
    open(os.path.join(outdir, "sets_summary.md"), "w").write("\n".join(L))
    print("완료.", outdir, flush=True)


if __name__ == "__main__":
    main()
