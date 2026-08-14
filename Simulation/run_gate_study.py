#!/usr/bin/env python3
"""세 가지 질문에 답하는 실험.

  E. 노이즈 종류별로 세 방법 중 무엇이 버티고 무엇이 무너지는가
  F. 하한(vision 산포)이 정말 제 역할을 하는가
  G. k=2.5 가 가장 좋은 값인가

실행:
  python3.10 run_gate_study.py --seeds 8 --workers 24
결과:
  figs/04_noise_matrix.png, figs/05_floor_ablation.png, figs/06_k_sweep.png
  figs/gate_raw.json, figs/gate_summary.md
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
from core.scene import SimScene
from core.methods import build_production_fk_anchors, solve_unified

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
STATUS_GOOD, STATUS_CRIT = "#0ca30c", "#d03b3b"

CB = ("cube", "board")
PROTO = dict(n_sets=13, train_size=11, n_splits=1,
             n_events_per_set=6, n_gripped_events=60)
BASE = dict(sigma_px=0.5, fk_sys_mm=3.0, intrinsic_err=0.01)

METHODS = [("noFK", "no-FK", C_NOFK, dict(fk="none")),
           ("fixed", "fixed-FK", C_FIXED, dict(fk="fixed")),
           ("ours", "corrected-FK", C_OURS, dict(fk="corr"))]


def _job(spec):
    key, cfg_kw, run_kw, seeds = spec
    res = run_config(ExpConfig("m", solve="unified", markers=CB, **cfg_kw),
                     seeds=seeds, **PROTO, **run_kw)
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
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    for side in ("left", "bottom"):
        ax.spines[side].set_color(GRID)
    ax.tick_params(colors=MUTED, labelsize=8.5)


# ══════════════════════════════════════════════════════════
# E. 노이즈 종류별 강건성
NOISE_AXES = [
    ("fk_sys",   "FK 계통오차 (mm)",       "fk_sys_mm",    [0.0, 2.0, 5.0, 9.0]),
    ("fk_rand",  "FK 랜덤 노이즈 (mm)",    "fk_noise_mm",  [0.0, 2.0, 5.0, 9.0]),
    ("cam_rand", "코너 검출 노이즈 (px)",  "sigma_px",     [0.2, 0.5, 0.9, 1.3]),
    ("cam_sys",  "카메라 내부파라미터 오차", "intrinsic_err", [0.0, 0.01, 0.02, 0.04]),
    ("outlier",  "코너 이상치 비율",        "outlier_rate", [0.0, 0.05, 0.10, 0.20]),
]


def experiment_e(seeds, workers, outdir, cached=None):
    jobs = []
    for axis, _, kw, levels in NOISE_AXES:
        for key, _, _, cfg_kw in METHODS:
            for v in levels:
                run_kw = dict(BASE)
                run_kw[kw] = v
                jobs.append((f"E|{axis}|{key}|{v}", cfg_kw, run_kw, seeds))
    if cached is not None:
        out = {k: tuple(v) for k, v in cached.items()}
    else:
        with ProcessPoolExecutor(max_workers=workers) as ex:
            out = dict(ex.map(_job, jobs))

    fig, axes = plt.subplots(1, 5, figsize=(19, 3.9))
    for ax, (axis, xlabel, _, levels) in zip(axes, NOISE_AXES):
        for key, label, color, _ in METHODS:
            ys = [out[f"E|{axis}|{key}|{v}"][0] for v in levels]
            ax.plot(levels, ys, color=color, linewidth=2.0, marker="o",
                    markersize=7, markeredgecolor=SURFACE, markeredgewidth=2,
                    label=label, zorder=3)
        _style(ax, xlabel, "held-out 작업 오차 (mm)" if axis == "fk_sys" else "",
               xlabel.split(" (")[0])
        ax.set_xticks(levels)
    axes[0].legend(loc="upper left", frameon=False, fontsize=8.5)
    fig.tight_layout()
    path = os.path.join(outdir, "04_noise_matrix.png")
    fig.savefig(path, dpi=160, facecolor=SURFACE)
    plt.close(fig)
    return path, out


# ══════════════════════════════════════════════════════════
# F. 하한의 역할
FLOOR_SCENARIOS = [
    ("clean",  "깨끗함\n(FK 잔차 작음)",  dict(sigma_px=0.3, fk_sys_mm=0.5, intrinsic_err=0.0)),
    ("normal", "보통",                    dict(sigma_px=0.5, fk_sys_mm=3.0, intrinsic_err=0.01)),
    ("slip",   "미끄러짐 3세트",          dict(sigma_px=0.5, fk_sys_mm=3.0, intrinsic_err=0.01,
                                              fk_slip_sets=3, fk_slip_mm=25.0, fk_slip_deg=4.0)),
]


def _reject_rate(run_kw, seeds, use_floor):
    """세트가 얼마나 탈락하는지 seed 평균."""
    rates = []
    for seed in range(seeds):
        sc = SimScene(seed=seed, n_fixed_cams=3, n_sets=13, n_events_per_set=6,
                      n_gripped_events=60, **run_kw)
        train = list(sc.sets)[:11]
        vm = solve_unified(sc, CB, "none", train, anchor_weight=0.0, fk_prior=None)
        r = build_production_fk_anchors(sc, CB, train, gate_mode="adaptive",
                                        gate_use_floor=use_floor, visual_model=vm)
        per = r.diagnostics["per_set"]
        if per:
            rates.append(sum(1 for v in per.values()
                             if not v["prior_accepted"]) / len(per))
    return float(np.mean(rates)) if rates else 0.0


def experiment_f(seeds, workers, outdir, cached=None):
    jobs = []
    for name, _, run_kw in FLOOR_SCENARIOS:
        for use_floor in (True, False):
            jobs.append((f"F|{name}|{int(use_floor)}",
                         dict(fk="corr", gate_use_floor=use_floor),
                         run_kw, seeds))
    if cached is not None:
        out = {k: tuple(v) for k, v in cached.items()}
    else:
        with ProcessPoolExecutor(max_workers=workers) as ex:
            out = dict(ex.map(_job, jobs))

    rates = {f"{name}|{int(uf)}": _reject_rate(run_kw, seeds, uf)
             for name, _, run_kw in FLOOR_SCENARIOS for uf in (True, False)}

    fig, axes = plt.subplots(1, 2, figsize=(12.5, 4.6))
    labels = [lab for _, lab, _ in FLOOR_SCENARIOS]
    xs = np.arange(len(FLOOR_SCENARIOS))
    w = 0.34

    for ax, (getter, ylabel, title, fmt) in zip(axes, [
        (lambda n, uf: rates[f"{n}|{int(uf)}"] * 100,
         "세트 탈락 비율 (%)", "하한을 꺼도 탈락 세트가 그대로다", "%.0f%%"),
        (lambda n, uf: out[f"F|{n}|{int(uf)}"][0],
         "held-out 작업 오차 (mm)", "정확도도 완전히 동일하다", "%.2f"),
    ]):
        for i, (uf, color, lab) in enumerate([(True, STATUS_GOOD, "하한 사용"),
                                              (False, STATUS_CRIT, "하한 없음")]):
            vals = [getter(n, uf) for n, _, _ in FLOOR_SCENARIOS]
            ax.bar(xs + (i - 0.5) * w, vals, width=w * 0.92, color=color,
                   zorder=3, edgecolor=SURFACE, linewidth=2, label=lab)
            for x, v in zip(xs + (i - 0.5) * w, vals):
                ax.annotate(fmt % v, (x, v), textcoords="offset points",
                            xytext=(0, 4), ha="center", fontsize=8.5, color=INK2)
        _style(ax, "", ylabel, title)
        ax.set_xticks(xs)
        ax.set_xticklabels(labels, fontsize=9)
    axes[0].legend(loc="upper left", frameon=False, fontsize=9)
    fig.tight_layout()
    path = os.path.join(outdir, "05_floor_ablation.png")
    fig.savefig(path, dpi=170, facecolor=SURFACE)
    plt.close(fig)
    return path, {"err": out, "reject": rates}


# ══════════════════════════════════════════════════════════
# G. k 민감도
K_VALUES = [1.0, 1.5, 2.0, 2.5, 3.0, 4.0, 6.0]
K_SCENARIOS = [
    ("slip3",  "미끄러짐 3세트", STATUS_CRIT,
     dict(sigma_px=0.5, fk_sys_mm=3.0, intrinsic_err=0.01,
          fk_slip_sets=3, fk_slip_mm=25.0, fk_slip_deg=4.0)),
    ("normal", "미끄러짐 없음", C_OURS,
     dict(sigma_px=0.5, fk_sys_mm=3.0, intrinsic_err=0.01)),
]


def experiment_g(seeds, workers, outdir, cached=None):
    jobs = []
    for name, _, _, run_kw in K_SCENARIOS:
        for k in K_VALUES:
            jobs.append((f"G|{name}|{k}", dict(fk="corr", gate_k=k), run_kw, seeds))
    if cached is not None:
        out = {k: tuple(v) for k, v in cached.items()}
    else:
        with ProcessPoolExecutor(max_workers=workers) as ex:
            out = dict(ex.map(_job, jobs))

    rej = {}
    for name, _, _, run_kw in K_SCENARIOS:
        for k in K_VALUES:
            rates = []
            for seed in range(seeds):
                sc = SimScene(seed=seed, n_fixed_cams=3, n_sets=13,
                              n_events_per_set=6, n_gripped_events=60, **run_kw)
                train = list(sc.sets)[:11]
                vm = solve_unified(sc, CB, "none", train, anchor_weight=0.0,
                                   fk_prior=None)
                r = build_production_fk_anchors(sc, CB, train,
                                                gate_mode="adaptive", gate_k=k,
                                                visual_model=vm)
                per = r.diagnostics["per_set"]
                if per:
                    rates.append(sum(1 for v in per.values()
                                     if not v["prior_accepted"]) / len(per))
            rej[f"{name}|{k}"] = float(np.mean(rates)) if rates else 0.0

    fig, axes = plt.subplots(1, 2, figsize=(13, 4.6))
    # 왼쪽은 seed 표준오차 밴드를 함께 그린다. 밴드를 빼면 y축이 확대되어
    # 변동이 실제보다 크게 보이고, 결론("둔감하다")과 반대로 읽힌다.
    for name, label, color, _ in K_SCENARIOS:
        ys = np.array([out[f"G|{name}|{k}"][0] for k in K_VALUES])
        sd = np.array([out[f"G|{name}|{k}"][1] for k in K_VALUES])
        se = sd / np.sqrt(max(out[f"G|{name}|{K_VALUES[0]}"][2], 1))
        axes[0].fill_between(K_VALUES, ys - se, ys + se, color=color,
                             alpha=0.15, linewidth=0, zorder=2)
        axes[0].plot(K_VALUES, ys, color=color, linewidth=2.0, marker="o",
                     markersize=8, markeredgecolor=SURFACE, markeredgewidth=2,
                     label=label, zorder=3)
        rs = [rej[f"{name}|{k}"] * 100 for k in K_VALUES]
        axes[1].plot(K_VALUES, rs, color=color, linewidth=2.0, marker="o",
                     markersize=8, markeredgecolor=SURFACE, markeredgewidth=2,
                     label=label, zorder=3)
    axes[0].annotate("음영 = seed 표준오차\n선의 오르내림이 이 안에 들어감",
                     (0.03, 0.06), xycoords="axes fraction",
                     fontsize=8.5, color=MUTED)
    for ax, ylabel, title in [
        (axes[0], "held-out 작업 오차 (mm)", "k 를 바꿔도 정확도는 변하지 않는다"),
        (axes[1], "세트 탈락 비율 (%)", "반면 몇 개를 버리는지는 크게 달라진다"),
    ]:
        _style(ax, "k  (기준선 = 중앙값 + k × 1.4826 × MAD)", ylabel, title)
        ax.set_xticks(K_VALUES)
        ax.axvline(2.5, color=MUTED, linewidth=1.2, linestyle=":", zorder=2)
        ax.annotate("현재 값 2.5", (2.5, ax.get_ylim()[1]),
                    textcoords="offset points", xytext=(4, -12),
                    color=MUTED, fontsize=8.5)
        ax.legend(loc="center right", frameon=False, fontsize=9)
    # 미끄러짐 3/11 = 27.3% 기준선
    axes[1].axhline(3 / 11 * 100, color=INK, linewidth=1.4, linestyle="--", zorder=4)
    axes[1].annotate("실제 주입한 미끄러짐 27.3%", (K_VALUES[-1], 3 / 11 * 100),
                     textcoords="offset points", xytext=(-4, 6),
                     ha="right", fontsize=8.5, color=INK)
    fig.tight_layout()
    path = os.path.join(outdir, "06_k_sweep.png")
    fig.savefig(path, dpi=170, facecolor=SURFACE)
    plt.close(fig)
    return path, {"err": out, "reject": rej}


# ══════════════════════════════════════════════════════════
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seeds", type=int, default=8)
    ap.add_argument("--workers", type=int, default=24)
    ap.add_argument("--replot", action="store_true")
    args = ap.parse_args()
    outdir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "figs")
    os.makedirs(outdir, exist_ok=True)

    cached = None
    if args.replot:
        with open(os.path.join(outdir, "gate_raw.json")) as f:
            cached = json.load(f)

    print("[E] 노이즈 종류별 강건성 ...", flush=True)
    p_e, raw_e = experiment_e(args.seeds, args.workers, outdir,
                              cached["e"] if cached else None)
    print("     →", p_e, flush=True)

    print("[F] 하한 역할 ...", flush=True)
    p_f, raw_f = experiment_f(args.seeds, args.workers, outdir,
                              cached["f"]["err"] if cached else None)
    print("     →", p_f, flush=True)

    print("[G] k 민감도 ...", flush=True)
    p_g, raw_g = experiment_g(args.seeds, args.workers, outdir,
                              cached["g"]["err"] if cached else None)
    print("     →", p_g, flush=True)

    with open(os.path.join(outdir, "gate_raw.json"), "w") as f:
        json.dump({"e": {k: list(v) for k, v in raw_e.items()},
                   "f": {"err": {k: list(v) for k, v in raw_f["err"].items()},
                         "reject": raw_f["reject"]},
                   "g": {"err": {k: list(v) for k, v in raw_g["err"].items()},
                         "reject": raw_g["reject"]}},
                  f, indent=2, ensure_ascii=False)
    print("완료.", outdir, flush=True)


if __name__ == "__main__":
    main()
