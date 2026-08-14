#!/usr/bin/env python3
"""FK 사용 방식 비교 실험 + gate 동작 시각화.

네 가지를 본다.
  A. FK 계통오차가 커질 때 세 방식이 어떻게 갈리는가
  B. 카메라 노이즈가 커질 때 세 방식이 어떻게 갈리는가
  C. gate가 실제로 어떤 세트를 걸러내는가 (세트별 판정 시각화)
  D. gate가 정확도에 실제로 도움이 되는가 (gate on/off 비교)

실행:
  python3.10 run_fk_study.py --seeds 8 --workers 24
결과:
  figs/*.png, figs/summary.md
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

# ── 한글 폰트 ─────────────────────────────────────────────
#   이름으로만 지정하면 같은 이름의 옛한글 변형이 잡혀 글리프가 깨진다.
#   파일 경로로 직접 등록한다.
_FONT_CANDIDATES = [
    "/usr/share/fonts/truetype/nanum/NanumBarunGothic.ttf",
    "/usr/share/fonts/truetype/nanum/NanumGothic.ttf",
]
for _path in _FONT_CANDIDATES:
    if os.path.exists(_path):
        font_manager.fontManager.addfont(_path)
        plt.rcParams["font.family"] = font_manager.FontProperties(
            fname=_path).get_name()
        break
plt.rcParams["axes.unicode_minus"] = False
plt.rcParams["mathtext.fontset"] = "dejavusans"  # 수식은 별도 폰트

# ── 팔레트 (dataviz 참조 팔레트, 고정 순서 슬롯 1~3) ────────
C_NOFK, C_FIXED, C_OURS = "#2a78d6", "#eb6834", "#1baf7a"
INK, INK2, MUTED = "#0b0b0b", "#52514e", "#8c8b85"
GRID, SURFACE = "#e6e5e0", "#fcfcfb"
STATUS_GOOD, STATUS_CRIT = "#0ca30c", "#d03b3b"

CB = ("cube", "board")
PROTO = dict(n_sets=13, train_size=11, n_splits=1,
             n_events_per_set=6, n_gripped_events=60)

METHODS = [
    ("noFK",  "no-FK",        C_NOFK,  dict(fk="none")),
    ("fixed", "fixed-FK",     C_FIXED, dict(fk="fixed")),
    ("ours",  "corrected-FK", C_OURS,  dict(fk="corr")),
]


def _cfg(fk, **kw):
    return ExpConfig("m", fk=fk, solve="unified", markers=CB, **kw)


def _job(spec):
    """(라벨, ExpConfig 인자, run_config 인자) -> 평균 e_task_mm, 표준편차."""
    key, cfg_kw, run_kw, seeds = spec
    res = run_config(_cfg(**cfg_kw), seeds=seeds, **PROTO, **run_kw)
    m, sd, n = res["e_task_mm"]
    mx, _, _ = res["e_X_mm"]
    return key, (m, sd, n, mx)


def _style(ax, xlabel, ylabel, title):
    ax.set_facecolor(SURFACE)
    ax.figure.set_facecolor(SURFACE)
    ax.set_xlabel(xlabel, color=INK2, fontsize=10)
    ax.set_ylabel(ylabel, color=INK2, fontsize=10)
    ax.set_title(title, color=INK, fontsize=12.5, pad=12, loc="left")
    ax.grid(True, color=GRID, linewidth=0.8, zorder=0)
    ax.set_axisbelow(True)
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    for side in ("left", "bottom"):
        ax.spines[side].set_color(GRID)
    ax.tick_params(colors=MUTED, labelsize=9)


def _line_panel(ax, xs, series, xlabel, title, xtick_labels=None):
    """series: [(라벨, 색, [평균...], [표준편차...])]"""
    for label, color, ys, sds in series:
        ax.plot(xs, ys, color=color, linewidth=2.0, marker="o",
                markersize=8, markeredgecolor=SURFACE, markeredgewidth=2,
                label=label, zorder=3)
        lo = [y - s / max(np.sqrt(8), 1) for y, s in zip(ys, sds)]
        hi = [y + s / max(np.sqrt(8), 1) for y, s in zip(ys, sds)]
        ax.fill_between(xs, lo, hi, color=color, alpha=0.12, linewidth=0, zorder=2)

    # 마지막 점에 직접 라벨. 값이 가까우면 세로로 벌려 겹침을 막는다.
    ends = sorted(((s[2][-1], s[0], s[1]) for s in series), key=lambda t: t[0])
    span = max(max(s[2]) for s in series) - min(min(s[2]) for s in series)
    min_gap = span * 0.09 if span > 0 else 1.0
    placed = []
    for value, label, color in ends:
        y = value
        if placed and y - placed[-1] < min_gap:
            y = placed[-1] + min_gap
        placed.append(y)
        ax.annotate(label, (xs[-1], value), xycoords="data",
                    xytext=(xs[-1] + (max(xs) - min(xs)) * 0.045, y),
                    textcoords="data", color=color, fontsize=9.5,
                    va="center", fontweight="bold",
                    arrowprops=dict(arrowstyle="-", color=color,
                                    linewidth=0.8, alpha=0.55,
                                    shrinkA=0, shrinkB=2))
    _style(ax, xlabel, "held-out 작업 오차 (mm)", title)
    if xtick_labels is not None:
        ax.set_xticks(xs)
        ax.set_xticklabels(xtick_labels)
    ax.set_xlim(min(xs) - (max(xs) - min(xs)) * 0.04,
                max(xs) + (max(xs) - min(xs)) * 0.30)


# ══════════════════════════════════════════════════════════
def experiment_ab(seeds, workers, outdir, precomputed=None):
    """A: FK 계통오차 sweep,  B: 카메라 노이즈 sweep."""
    FK_LEVELS = [0.0, 1.0, 2.0, 3.0, 5.0, 8.0]
    PX_LEVELS = [0.2, 0.4, 0.6, 0.9, 1.3]

    jobs = []
    for key, _, _, cfg_kw in METHODS:
        for v in FK_LEVELS:
            jobs.append((f"A|{key}|{v}", cfg_kw,
                         dict(sigma_px=0.5, fk_sys_mm=v, intrinsic_err=0.01), seeds))
        for v in PX_LEVELS:
            jobs.append((f"B|{key}|{v}", cfg_kw,
                         dict(sigma_px=v, fk_sys_mm=3.0, intrinsic_err=0.01), seeds))

    if precomputed is not None:
        out = {k: tuple(v) for k, v in precomputed.items()}
    else:
        with ProcessPoolExecutor(max_workers=workers) as ex:
            out = dict(ex.map(_job, jobs))

    fig, axes = plt.subplots(1, 2, figsize=(13.5, 4.8))
    for ax, (tag, levels, xlabel, title) in zip(axes, [
        ("A", FK_LEVELS, "FK 계통오차 크기 (mm)",
         "FK가 부정확해질수록"),
        ("B", PX_LEVELS, "코너 검출 노이즈 (px)",
         "카메라가 부정확해질수록"),
    ]):
        series = []
        for key, label, color, _ in METHODS:
            ys = [out[f"{tag}|{key}|{v}"][0] for v in levels]
            sds = [out[f"{tag}|{key}|{v}"][1] for v in levels]
            series.append((label, color, ys, sds))
        _line_panel(ax, levels, series, xlabel, title)
    fig.tight_layout()
    path = os.path.join(outdir, "01_fk_and_noise_sweep.png")
    fig.savefig(path, dpi=170, facecolor=SURFACE)
    plt.close(fig)
    return path, {k: v for k, v in out.items()}


# ══════════════════════════════════════════════════════════
def experiment_c(outdir, seed=0, slip_sets=3, slip_mm=25.0, slip_deg=4.0):
    """C: gate가 어떤 세트를 걸러내는지 이동/회전 두 축으로 그린다."""
    sc = SimScene(seed=seed, n_fixed_cams=3, n_sets=13, n_events_per_set=6,
                  n_gripped_events=60, sigma_px=0.5, fk_sys_mm=3.0,
                  intrinsic_err=0.01,
                  fk_slip_sets=slip_sets, fk_slip_mm=slip_mm, fk_slip_deg=slip_deg)
    train = list(sc.sets)[:11]
    vm = solve_unified(sc, CB, "none", train, anchor_weight=0.0, fk_prior=None)
    res = build_production_fk_anchors(sc, CB, train, gate_mode="adaptive",
                                      visual_model=vm)
    diag = res.diagnostics
    per = diag["per_set"]

    sets = sorted(int(k) for k in per)
    dts = [per[str(s)]["prior_blend_dt_mm"] for s in sets]
    drs = [per[str(s)]["prior_blend_dr_deg"] for s in sets]
    acc = [per[str(s)]["prior_accepted"] for s in sets]
    slipped = set(sc.fk_slip_sets)

    fig, axes = plt.subplots(2, 1, figsize=(11, 7.6), sharex=True)
    xs = np.arange(len(sets))
    colors = [STATUS_GOOD if a else STATUS_CRIT for a in acc]

    panels = [
        (axes[0], dts, diag["gate_dt_mm"], diag.get("gate_floor_dt_mm"),
         "이동 차이  $d_t$  (mm)", "mm"),
        (axes[1], drs, diag["gate_dr_deg"], diag.get("gate_floor_dr_deg"),
         "회전 차이  $d_R$  (도)", "도"),
    ]
    for ax, vals, thr, floor, ylabel, unit in panels:
        ax.bar(xs, vals, width=0.62, color=colors, zorder=3,
               edgecolor=SURFACE, linewidth=2)
        ax.axhline(thr, color=INK, linewidth=2.0, linestyle="--", zorder=4)
        ax.annotate(f"gate 기준선 {thr:.2f} {unit}", (len(sets) - 0.4, thr),
                    textcoords="offset points", xytext=(0, 7),
                    color=INK, fontsize=9.5, ha="right", fontweight="bold")
        if floor:
            ax.axhline(floor, color=MUTED, linewidth=1.3, linestyle=":", zorder=4)
            ax.annotate(f"하한 {floor:.2f} {unit}", (0, floor),
                        textcoords="offset points", xytext=(0, 5),
                        color=MUTED, fontsize=8.5)
        _style(ax, "", ylabel, "")
        ax.tick_params(labelbottom=True)

    axes[0].set_title("gate가 걸러낸 세트  (두 조건을 모두 만족해야 통과)",
                      color=INK, fontsize=12.5, pad=12, loc="left")
    axes[1].set_xlabel("세트 번호", color=INK2, fontsize=10)
    axes[1].set_xticks(xs)
    axes[1].set_xticklabels(sets)

    for x, s in zip(xs, sets):
        if s in slipped:
            i = sets.index(s)
            axes[0].annotate("미끄러짐 주입", (x, dts[i]),
                             textcoords="offset points", xytext=(0, 8),
                             ha="center", fontsize=8.5, color=STATUS_CRIT,
                             fontweight="bold")

    from matplotlib.patches import Patch
    axes[0].legend(handles=[
        Patch(facecolor=STATUS_GOOD, label="통과 → 보정된 FK 사용"),
        Patch(facecolor=STATUS_CRIT, label="탈락 → vision 합의 사용"),
    ], loc="upper left", frameon=False, fontsize=9.5)

    fig.tight_layout()
    path = os.path.join(outdir, "02_gate_decision.png")
    fig.savefig(path, dpi=170, facecolor=SURFACE)
    plt.close(fig)

    detail = {
        "threshold_dt_mm": diag["gate_dt_mm"], "threshold_dr_deg": diag["gate_dr_deg"],
        "floor_dt_mm": diag.get("gate_floor_dt_mm"),
        "floor_dr_deg": diag.get("gate_floor_dr_deg"),
        "slipped_sets": sorted(slipped),
        "rejected_sets": [s for s, a in zip(sets, acc) if not a],
        "per_set": {str(s): {"dt_mm": d, "dr_deg": r, "accepted": a}
                    for s, d, r, a in zip(sets, dts, drs, acc)},
    }
    return path, detail


# ══════════════════════════════════════════════════════════
def experiment_d(seeds, workers, outdir, precomputed=None):
    """D: 미끄러진 세트 수를 늘리며 gate on/off 비교."""
    SLIPS = [0, 1, 2, 3, 4]
    VARIANTS = [
        ("gate_on",  "corrected-FK (gate 사용)", C_OURS,
         dict(fk="corr", gate_mode="adaptive")),
        ("gate_off", "corrected-FK (gate 없음)", C_FIXED,
         dict(fk="corr", gate_mode="fixed",
              prior_max_dt_mm=1e9, prior_max_dr_deg=1e9)),
        ("nofk",     "no-FK",                    C_NOFK,
         dict(fk="none")),
    ]
    jobs = []
    for key, _, _, cfg_kw in VARIANTS:
        for n in SLIPS:
            jobs.append((f"D|{key}|{n}", cfg_kw,
                         dict(sigma_px=0.5, fk_sys_mm=3.0, intrinsic_err=0.01,
                              fk_slip_sets=n, fk_slip_mm=25.0, fk_slip_deg=4.0),
                         seeds))
    if precomputed is not None:
        out = {k: tuple(v) for k, v in precomputed.items()}
    else:
        with ProcessPoolExecutor(max_workers=workers) as ex:
            out = dict(ex.map(_job, jobs))

    fig, ax = plt.subplots(figsize=(8.6, 5.0))
    series = []
    for key, label, color, _ in VARIANTS:
        ys = [out[f"D|{key}|{n}"][0] for n in SLIPS]
        sds = [out[f"D|{key}|{n}"][1] for n in SLIPS]
        series.append((label, color, ys, sds))
    _line_panel(ax, SLIPS, series, "큐브가 미끄러진 세트 수 (전체 13개 중)",
                "gate가 실제로 도움이 되는가")
    fig.tight_layout()
    path = os.path.join(outdir, "03_gate_on_off.png")
    fig.savefig(path, dpi=170, facecolor=SURFACE)
    plt.close(fig)
    return path, out


# ══════════════════════════════════════════════════════════
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seeds", type=int, default=8)
    ap.add_argument("--workers", type=int, default=24)
    ap.add_argument("--outdir", default=None)
    ap.add_argument("--replot", action="store_true",
                    help="raw_results.json 을 읽어 그림만 다시 그린다")
    args = ap.parse_args()

    outdir = args.outdir or os.path.join(
        os.path.dirname(os.path.abspath(__file__)), "figs")
    os.makedirs(outdir, exist_ok=True)

    cached = None
    if args.replot:
        with open(os.path.join(outdir, "raw_results.json")) as f:
            cached = json.load(f)

    print("[A/B] FK 계통오차 · 카메라 노이즈 sweep ...", flush=True)
    p_ab, raw_ab = experiment_ab(args.seeds, args.workers, outdir,
                                 precomputed=cached["ab"] if cached else None)
    print("     →", p_ab, flush=True)

    print("[C] gate 판정 시각화 ...", flush=True)
    p_c, detail_c = experiment_c(outdir)
    print("     →", p_c, flush=True)
    print("     기준선 %.2f mm / %.2f도 | 주입 %s | 탈락 %s"
          % (detail_c["threshold_dt_mm"], detail_c["threshold_dr_deg"],
             detail_c["slipped_sets"], detail_c["rejected_sets"]), flush=True)

    print("[D] gate on/off 비교 ...", flush=True)
    p_d, raw_d = experiment_d(args.seeds, args.workers, outdir,
                              precomputed=cached["d"] if cached else None)
    print("     →", p_d, flush=True)

    with open(os.path.join(outdir, "raw_results.json"), "w") as f:
        json.dump({"ab": {k: list(v) for k, v in raw_ab.items()},
                   "c": detail_c,
                   "d": {k: list(v) for k, v in raw_d.items()}},
                  f, indent=2, ensure_ascii=False)
    print("완료. 결과는", outdir, flush=True)


if __name__ == "__main__":
    main()
