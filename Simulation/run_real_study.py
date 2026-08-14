#!/usr/bin/env python3
"""실측 배치(session02) 위에서 세 FK 방식을 노이즈 8축으로 비교한다.

배치는 전부 실제 값이다.
  고정 카메라 3대 자세 · 그리퍼 hand-eye · 세트 14개 큐브 자세 · 이벤트 90개 로봇 자세
  (Step3_calibration.py 를 data/session02 에 돌린 결과)

노이즈 8축 = 카메라 3종 × {랜덤, 계통} + FK × {랜덤, 계통}

  카메라 - 코너 검출   랜덤: 코너마다 독립 지터        sigma_px
                       계통: 카메라별 고정 픽셀 편향    corner_bias_px
  카메라 - 코너 이상치 랜덤: 코너마다 확률적으로 튐      outlier_rate
                       계통: 한 카메라에만 이상치 집중   outlier_rate + outlier_focus_cam
  카메라 - 내부파라미터 랜덤: 프레임마다 초점거리 흔들림 intrinsic_jitter
                       계통: 잘못된 K 로 고정          intrinsic_err
  FK                   랜덤: 세트마다 제로평균 섭동      fk_noise_mm
                       계통: 모든 세트 공통 오정렬      fk_sys_mm

실행:
  python3.10 run_real_study.py --seeds 8 --workers 24
결과:
  figs_real/*.png, figs_real/raw.json, figs_real/summary.md
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
# 실측 프로토콜: 세트 14개, 이벤트 90개. 학습 11세트 / 평가 나머지.
PROTO = dict(n_sets=14, train_size=11, n_splits=1,
             n_events_per_set=6, n_gripped_events=0, use_real_layout=True)
# 축을 하나씩 흔들 때 나머지는 이 값으로 고정한다(실측에 가까운 기본 조건).
BASE = dict(sigma_px=0.5, intrinsic_err=0.01, fk_sys_mm=3.0)

METHODS = [("noFK", "no-FK", C_NOFK, "none"),
           ("fixed", "fixed-FK", C_FIXED, "fixed"),
           ("ours", "corrected-FK", C_OURS, "corr")]

# (축 키, 제목, 그룹, 노이즈 인자, 단계값)
AXES = [
    ("corner_rand", "코너 검출 · 랜덤",    "카메라",
     lambda v: dict(sigma_px=v),                       [0.2, 0.5, 0.9, 1.4, 2.0]),
    ("corner_sys",  "코너 검출 · 계통",    "카메라",
     lambda v: dict(corner_bias_px=v),                 [0.0, 0.4, 0.8, 1.4, 2.0]),
    ("outlier_rand", "코너 이상치 · 랜덤", "카메라",
     lambda v: dict(outlier_rate=v),                   [0.0, 0.05, 0.10, 0.18, 0.30]),
    ("outlier_sys",  "코너 이상치 · 계통", "카메라",
     lambda v: dict(outlier_rate=v, outlier_focus_cam=0), [0.0, 0.10, 0.25, 0.45, 0.70]),
    ("intr_rand", "내부파라미터 · 랜덤",   "카메라",
     lambda v: dict(intrinsic_jitter=v),               [0.0, 0.005, 0.01, 0.02, 0.04]),
    ("intr_sys",  "내부파라미터 · 계통",   "카메라",
     lambda v: dict(intrinsic_err=v),                  [0.0, 0.005, 0.01, 0.02, 0.04]),
    ("fk_rand", "FK · 랜덤",               "FK",
     lambda v: dict(fk_noise_mm=v, fk_noise_deg=v / 10.0), [0.0, 2.0, 5.0, 9.0, 14.0]),
    ("fk_sys",  "FK · 계통",               "FK",
     lambda v: dict(fk_sys_mm=v, fk_sys_deg=v / 10.0),     [0.0, 2.0, 5.0, 9.0, 14.0]),
]


def _job(spec):
    key, fk, run_kw, seeds = spec
    res = run_config(ExpConfig("m", fk=fk, solve="unified", markers=CB),
                     seeds=seeds, **PROTO, **run_kw)
    m, sd, n = res["e_task_mm"]
    mx, sx, _ = res["e_X_mm"]
    return key, (m, sd, n, mx, sx)


def _style(ax, xlabel, ylabel, title):
    ax.set_facecolor(SURFACE)
    ax.figure.set_facecolor(SURFACE)
    ax.set_xlabel(xlabel, color=INK2, fontsize=9)
    ax.set_ylabel(ylabel, color=INK2, fontsize=9)
    ax.set_title(title, color=INK, fontsize=10.5, pad=8, loc="left")
    ax.grid(True, color=GRID, linewidth=0.8, zorder=0)
    ax.set_axisbelow(True)
    for s in ("top", "right"):
        ax.spines[s].set_visible(False)
    for s in ("left", "bottom"):
        ax.spines[s].set_color(GRID)
    ax.tick_params(colors=MUTED, labelsize=8)


def draw(out, outdir, metric_idx=0, metric_name="held-out 작업 오차 (mm)",
         fname="10_noise_matrix_real.png"):
    fig, axes = plt.subplots(2, 4, figsize=(19, 8.4))
    for ax, (key, title, group, _, levels) in zip(axes.ravel(), AXES):
        for mk, label, color, _fk in METHODS:
            ys = np.array([out[f"{key}|{mk}|{v}"][metric_idx] for v in levels])
            sd = np.array([out[f"{key}|{mk}|{v}"][metric_idx + 1] for v in levels])
            n = max(out[f"{key}|{mk}|{levels[0]}"][2], 1)
            se = sd / np.sqrt(n)
            ax.fill_between(levels, ys - se, ys + se, color=color,
                            alpha=0.13, linewidth=0, zorder=2)
            ax.plot(levels, ys, color=color, linewidth=2.0, marker="o",
                    markersize=7, markeredgecolor=SURFACE, markeredgewidth=2,
                    label=label, zorder=3)
        _style(ax, "", metric_name if ax in (axes[0][0], axes[1][0]) else "",
               f"[{group}] {title}")
        ax.set_xticks(levels)
    axes[0][0].legend(loc="upper left", frameon=False, fontsize=8.5)
    fig.suptitle("session02 실측 배치에서 노이즈 8축 × 세 FK 방식",
                 color=INK, fontsize=13, x=0.007, ha="left", y=0.995)
    fig.tight_layout(rect=[0, 0, 1, 0.975])
    path = os.path.join(outdir, fname)
    fig.savefig(path, dpi=150, facecolor=SURFACE)
    plt.close(fig)
    return path


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seeds", type=int, default=8)
    ap.add_argument("--workers", type=int, default=24)
    ap.add_argument("--replot", action="store_true")
    a = ap.parse_args()
    outdir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "figs_real")
    os.makedirs(outdir, exist_ok=True)

    if a.replot:
        out = {k: tuple(v) for k, v in
               json.load(open(os.path.join(outdir, "raw.json"))).items()}
    else:
        jobs = []
        for key, _, _, mk_kw, levels in AXES:
            for mk, _, _, fk in METHODS:
                for v in levels:
                    run_kw = dict(BASE)
                    run_kw.update(mk_kw(v))
                    jobs.append((f"{key}|{mk}|{v}", fk, run_kw, a.seeds))
        print(f"총 {len(jobs)}개 설정 × seed {a.seeds}개", flush=True)
        with ProcessPoolExecutor(max_workers=a.workers) as ex:
            out = {}
            for i, (k, v) in enumerate(ex.map(_job, jobs), 1):
                out[k] = v
                if i % 10 == 0:
                    print(f"  {i}/{len(jobs)} 완료", flush=True)
        with open(os.path.join(outdir, "raw.json"), "w") as f:
            json.dump({k: list(v) for k, v in out.items()}, f, indent=1)

    p1 = draw(out, outdir, 0, "held-out 작업 오차 (mm)", "10_noise_matrix_real.png")
    print("→", p1, flush=True)
    p2 = draw(out, outdir, 3, "hand-eye 오차 (mm)", "11_noise_matrix_handeye.png")
    print("→", p2, flush=True)

    # 요약표
    L = ["# 실측 배치 노이즈 실험 결과\n",
         "session02 배치(카메라 3대·세트 14개·이벤트 90개)를 참값으로 고정하고,",
         "노이즈 8축을 각각 5단계로 키우며 세 방법을 비교했다. 값은 held-out 작업 오차(mm).\n"]
    for key, title, group, _, levels in AXES:
        L.append(f"## [{group}] {title}\n")
        L.append("| 방법 | " + " | ".join(str(v) for v in levels) + " | 배수 |")
        L.append("|---|" + "---|" * (len(levels) + 1))
        for mk, label, _, _fk in METHODS:
            ys = [out[f"{key}|{mk}|{v}"][0] for v in levels]
            L.append(f"| {label} | " + " | ".join(f"{y:.2f}" for y in ys)
                     + f" | {ys[-1]/max(ys[0],1e-9):.2f} |")
        L.append("")
    open(os.path.join(outdir, "summary.md"), "w").write("\n".join(L))
    print("완료.", outdir, flush=True)


if __name__ == "__main__":
    main()
