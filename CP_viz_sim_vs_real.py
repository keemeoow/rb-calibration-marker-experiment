"""
시뮬(Simul_test) ↔ 실데이터(CP_result) 나란히 비교 figure + 통합 요약표 생성.

기여도 C1/C2/C3 각각에 대해 **상단행 = 시뮬(GT 기준)**, **하단행 = CP 실데이터(FK 프록시)**
2행 figure 를 만든다. 절대 스케일이 다르므로(시뮬 mm 단위 ~1, 실데이터 ~100) 행마다 자체
스케일을 쓰고, 대신 baseline 대비 개선율(%)을 주석·별도 패널로 병기해 직접 비교한다.

입력 (재실행·재계산 없음, 이미 저장된 산출물만 읽음):
  Simul_test/figures/unified_vs_indep_data.json     — C1 시뮬
  Simul_test/figures/exp2_board_vs_cube_data.json   — C2 시뮬
  Simul_test/figures/exp3_noise_sweep_data.json     — C3 시뮬
  CP_result/C1/joint_ablation_summary.csv           — C1 실데이터
  CP_result/C2/C2_cube_vs_board.csv, C2_observability.csv
  CP_result/C3/ablation_summary.csv                 — C3 실데이터

출력:
  CP_result/figures/fig_SIMvsCP_C1_unified_vs_indep.png
  CP_result/figures/fig_SIMvsCP_C2_board_vs_cube.png
  CP_result/figures/fig_SIMvsCP_C3_gtc_estimation.png
  CP_result/SIM_vs_CP_summary.csv / .md

실행:
  PYTHONPATH= python CP_viz_sim_vs_real.py
  PYTHONPATH= python CP_viz_sim_vs_real.py --only C2

주: figure 안 텍스트는 한글 폰트 문제 회피를 위해 영문만 쓴다(Simul_test/viz_*.py 와 동일 규약).
"""
import os
import csv
import json
import argparse

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Patch

ROOT = os.path.dirname(os.path.abspath(__file__))
SIM_FIG_DIR = os.path.join(ROOT, "Simul_test", "figures")
CP_DIR = os.path.join(ROOT, "CP_result")
OUT_DIR = os.path.join(CP_DIR, "figures")

# Simul_test/viz_*.py 와 동일한 팔레트 — 두 행이 한 시스템으로 읽혀야 하므로 유지.
RED, BLUE, GREEN = "#c44e52", "#4c72b0", "#55a868"
RED_L, BLUE_L, GREEN_L = "#e17c7f", "#7a9bcc", "#8ecf9f"
NA_GRAY = "#d9d9d9"
# +fk / 후보정 변형은 색만이 아니라 hatch 로도 구분(CVD·흑백 인쇄 대비 이중 인코딩)
FK_HATCH = "//"


# --------------------------------------------------------------------------- io
def _read_csv(path):
    with open(path, newline="") as f:
        return list(csv.DictReader(f))


def _f(row, key, default=np.nan):
    """CSV 문자열 셀 → float. 빈칸/파싱실패는 default."""
    v = (row.get(key) or "").strip()
    try:
        return float(v)
    except ValueError:
        return default


def _load_json(path):
    with open(path) as f:
        return json.load(f)


# ------------------------------------------------------------------- bar helper
def _bars(ax, labels, values, colors, hatches=None, fmt="{:.2f}", ylabel=None,
          title=None, title_bg=None, note=None):
    """단일 패널 막대. 값 라벨은 막대 위, 그리드는 y축만(recessive)."""
    xs = np.arange(len(labels))
    hatches = hatches or [""] * len(labels)
    for x, v, c, h in zip(xs, values, colors, hatches):
        ax.bar(x, v, color=c, hatch=h, edgecolor="black", linewidth=0.6,
               alpha=0.9, width=0.62)
    finite = [v for v in values if np.isfinite(v)]
    top = max(finite) if finite else 1.0
    for x, v in zip(xs, values):
        if np.isfinite(v):
            ax.text(x, v + top * 0.02, fmt.format(v), ha="center", va="bottom",
                    fontsize=8, fontweight="bold", color="#333333")
    ax.set_xticks(xs)
    ax.set_xticklabels(labels, rotation=18, ha="right", fontsize=8)
    ax.set_ylim(0, top * 1.25 if top > 0 else 1)
    ax.grid(axis="y", alpha=0.25)
    ax.set_axisbelow(True)
    if ylabel:
        ax.set_ylabel(ylabel, fontsize=9)
    if title:
        ax.set_title(title, fontsize=9.5, fontweight="bold",
                     bbox=dict(boxstyle="round,pad=0.35",
                               fc=title_bg or "#eeeeee", ec="none"))
    if note:
        ax.text(0.5, 0.94, note, transform=ax.transAxes, ha="center", va="top",
                fontsize=7.5, color="#555555")


def _na_panel(ax, title, msg):
    """실데이터에서 측정 불가한 GT 전용 지표 자리."""
    ax.set_xticks([])
    ax.set_yticks([])
    for s in ax.spines.values():
        s.set_edgecolor("#cccccc")
    ax.add_patch(plt.Rectangle((0.04, 0.04), 0.92, 0.92, transform=ax.transAxes,
                               fc=NA_GRAY, ec="none", alpha=0.45))
    ax.text(0.5, 0.56, "N/A", transform=ax.transAxes, ha="center", va="center",
            fontsize=20, fontweight="bold", color="#8a8a8a")
    ax.text(0.5, 0.36, msg, transform=ax.transAxes, ha="center", va="center",
            fontsize=8, color="#666666", wrap=True)
    ax.set_title(title, fontsize=9.5, fontweight="bold",
                 bbox=dict(boxstyle="round,pad=0.35", fc="#eeeeee", ec="none"))


def _row_tag(ax, text, color):
    """행 왼쪽 라벨 (SIMULATION / REAL DATA)."""
    ax.set_ylabel(f"[{text}]\n" + (ax.get_ylabel() or ""), fontsize=9,
                  fontweight="bold", color=color)


def _save(fig, name):
    os.makedirs(OUT_DIR, exist_ok=True)
    path = os.path.join(OUT_DIR, name)
    fig.savefig(path, dpi=130, bbox_inches="tight")
    plt.close(fig)
    print(f"[저장] {path}")
    return path


def _pct(base, prop):
    """baseline 대비 변화율(%). 음수 = 개선(오차 감소)."""
    if not np.isfinite(base) or not np.isfinite(prop) or base == 0:
        return np.nan
    return (prop - base) / base * 100.0


# ------------------------------------------------------------------------- C1
def load_c1():
    sim = _load_json(os.path.join(SIM_FIG_DIR, "unified_vs_indep_data.json"))
    s = sim["data"]["systematic"]
    cp = {r["method"]: r for r in
          _read_csv(os.path.join(CP_DIR, "C1", "joint_ablation_summary.csv"))}
    return sim["meta"], s, cp


C1_LEGEND = [
    Patch(fc=RED, ec="black", lw=0.6, label="Independent"),
    Patch(fc=BLUE, ec="black", lw=0.6, label="Joint / unified (bundle adj.)"),
    Patch(fc=GREEN, ec="black", lw=0.6, label="Joint, cube fixed to FK (CP only)"),
    Patch(fc="white", ec="black", lw=0.6, hatch=FK_HATCH,
          label="+fk = Ridge residual correction at prediction time"),
]


def _c1_sim_row(axr, s):
    """C1 시뮬 4패널 (GT 기준). axr = 길이-4 axes."""
    sim_2 = ["Indep", "Joint"]
    sim_4 = ["Indep", "Indep+fk", "Joint", "Joint+fk"]
    c2 = [RED, BLUE]
    _bars(axr[0], sim_2, [s[m]["bTf_mm"][0] for m in sim_2], c2,
          ylabel="error vs GT (mm)", title="Fixed cam bTf\n(eye-to-hand)")
    _bars(axr[1], sim_2, [s[m]["gTc_mm"][0] for m in sim_2], c2,
          ylabel="error vs GT (mm)", title="Hand-eye gTc\n(eye-in-hand)")
    _bars(axr[2], sim_2, [s[m]["consistency_mm"][0] for m in sim_2], c2,
          ylabel="error (mm)", title="Shared-base\nconsistency")
    _bars(axr[3], sim_4, [s[m]["downstream_mm"][0] for m in sim_4],
          [RED, RED_L, BLUE, BLUE_L], hatches=["", FK_HATCH, "", FK_HATCH],
          ylabel="error vs GT (mm)",
          title="Downstream cube\nprediction (held-out)", title_bg="#e3f2e8")


def _c1_cp_row(axr, cp):
    """C1 CP 실데이터 4패널 (FK 프록시). bTf 는 측정 불가 → N/A."""
    m3 = ["independent", "unified_joint", "joint_fk_fixed"]
    l3 = ["Indep", "Joint\n(unified)", "Joint\n(fk-fixed)"]
    c3 = [RED, BLUE, GREEN]
    _na_panel(axr[0], "Fixed cam bTf\n(eye-to-hand)",
              "no ground-truth camera pose\nin real session")
    _bars(axr[1], l3, [_f(cp[m], "grip_align_trans_rmse_mm") for m in m3],
          c3, fmt="{:.1f}", ylabel="RMSE (mm)",
          title="Gripper alignment RMSE\n(proxy for gTc)")
    _bars(axr[2], l3, [_f(cp[m], "consistency_trans_rmse_mm") for m in m3],
          c3, fmt="{:.1f}", ylabel="RMSE (mm)", title="Shared-base\nconsistency")
    down6 = []
    for m in m3:
        down6.append(_f(cp[m], "downstream_trans_rmse_mm"))
        down6.append(_f(cp[m], "downstream_fk_trans_rmse_mm"))
    _bars(axr[3], ["Indep", "Indep+fk", "Joint", "Joint+fk", "JointFK", "JointFK+fk"],
          down6, [RED, RED_L, BLUE, BLUE_L, GREEN, GREEN_L],
          hatches=["", FK_HATCH, "", FK_HATCH, "", FK_HATCH], fmt="{:.1f}",
          ylabel="RMSE vs FK (mm)",
          title="Downstream cube\nprediction (held-out)", title_bg="#e3f2e8")


def _c1_subtitle(meta, cp):
    return (f"sim: {meta['sets']} sets, train={meta['train']}/test=2, "
            f"{meta['seeds']} seeds, systematic noise {meta['noise']}mm"), \
           (f"real: data/session, {int(_f(cp['unified_joint'], 'n_sets')) + 4} sets, "
            "train=9/test=4   (proxy = robot-FK cube center)")


def fig_c1(which="both"):
    meta, s, cp = load_c1()
    sub_s, sub_c = _c1_subtitle(meta, cp)

    if which == "sim":
        fig, ax = plt.subplots(1, 4, figsize=(17.2, 4.6))
        _c1_sim_row(ax, s)
        ax[0].set_ylabel("[SIMULATION]\n" + ax[0].get_ylabel(), fontsize=9,
                         fontweight="bold")
        fig.legend(handles=C1_LEGEND[:2] + [C1_LEGEND[3]], loc="lower center",
                   ncol=3, fontsize=9, frameon=False, bbox_to_anchor=(0.5, -0.07))
        fig.suptitle("C1 — Unified vs Independent   [SIMULATION, vs ground truth]\n"
                     + sub_s + "   (lower = better)", fontsize=12, y=1.02)
        fig.tight_layout(rect=[0, 0.04, 1, 0.9])
        return _save(fig, "fig_SIM_C1_unified_vs_indep.png")

    if which == "cp":
        fig, ax = plt.subplots(1, 4, figsize=(17.2, 4.6))
        _c1_cp_row(ax, cp)
        ax[0].set_ylabel("[REAL DATA (CP)]", fontsize=9, fontweight="bold")
        fig.legend(handles=C1_LEGEND, loc="lower center", ncol=4, fontsize=9,
                   frameon=False, bbox_to_anchor=(0.5, -0.07))
        fig.suptitle("C1 — Unified vs Independent   [REAL DATA, vs robot-FK proxy]\n"
                     + sub_c + "   (lower = better)", fontsize=12, y=1.02)
        fig.tight_layout(rect=[0, 0.04, 1, 0.9])
        return _save(fig, "fig_CP_C1_unified_vs_indep.png")

    fig, ax = plt.subplots(2, 4, figsize=(17.2, 8.6))
    _c1_sim_row(ax[0], s)
    _c1_cp_row(ax[1], cp)
    _row_tag(ax[0][0], "SIMULATION", "#333333")
    ax[1][0].set_ylabel("[REAL DATA (CP)]", fontsize=9, fontweight="bold")
    fig.legend(handles=C1_LEGEND, loc="lower center", ncol=4, fontsize=9,
               frameon=False, bbox_to_anchor=(0.5, -0.035))
    fig.suptitle(
        "C1 — Unified (joint bundle adjustment)  vs  Independent      "
        "SIMULATION (top, vs ground truth)   vs   REAL DATA (bottom, vs robot-FK proxy)\n"
        + sub_s + "      |      " + sub_c
        + "      (lower = better;  note the different y-scales per row)",
        fontsize=11.5, y=1.0)
    fig.tight_layout(rect=[0, 0.035, 1, 0.945])
    return _save(fig, "fig_SIMvsCP_C1_unified_vs_indep.png")


# ------------------------------------------------------------------------- C2
def load_c2():
    sim = _load_json(os.path.join(SIM_FIG_DIR, "exp2_board_vs_cube_data.json"))
    cp = {r["mode"]: r for r in
          _read_csv(os.path.join(CP_DIR, "C2", "C2_cube_vs_board.csv"))}
    obs = {r["target"]: r for r in
           _read_csv(os.path.join(CP_DIR, "C2", "C2_observability.csv"))}
    return sim["meta"], sim["data"], cp, obs


C2_LEGEND = [
    Patch(fc=RED, ec="black", lw=0.6, label="Board only (planar ChArUco)"),
    Patch(fc=BLUE, ec="black", lw=0.6, label="Cube (6-face marker cube)"),
    Patch(fc=GREEN, ec="black", lw=0.6, label="Hybrid: board + cube (CP only)"),
]


def _c2_sim_row(axr, s):
    sim_m = ["board", "board+cube"]
    sim_l = ["Board only", "Board + Cube"]
    c = [RED, BLUE]
    _bars(axr[0], sim_l, [s[m]["simul"][0] for m in sim_m], c,
          ylabel="cameras / shot", title="Simultaneous views\n(higher = better)")
    _bars(axr[1], sim_l, [s[m]["coverage"][0] for m in sim_m], c, fmt="{:.1f}",
          ylabel="deg", title="Viewpoint coverage\n(higher = better)")
    _bars(axr[2], sim_l, [s[m]["cam_mm"][0] for m in sim_m], c,
          ylabel="error vs GT (mm)", title="Camera pose error\n(lower = better)",
          title_bg="#e3f2e8")
    _bars(axr[3], sim_l, [s[m]["obj_mm"][0] for m in sim_m], c,
          ylabel="error vs GT (mm)", title="Target prediction error\n(lower = better)",
          title_bg="#e3f2e8")


def _c2_cp_row(axr, cp, obs):
    obs_t = ["board", "cube"]
    obs_l = ["Board only", "Cube"]
    oc = [RED, BLUE]
    cp_m = ["board_only", "cube_only", "hybrid"]
    cp_l = ["Board only", "Cube only", "Hybrid\n(board+cube)"]
    cc = [RED, BLUE, GREEN]
    pct2 = [_f(obs[t], "pct_events_2plus_observers") for t in obs_t]
    _bars(axr[0], obs_l, [_f(obs[t], "mean_simul_observers") for t in obs_t], oc,
          ylabel="cameras / shot", title="Simultaneous views\n(higher = better)",
          note=f"$\\geq$2 observers:  {pct2[0]:.1f}%  vs  {pct2[1]:.1f}%")
    _bars(axr[1], obs_l, [_f(obs[t], "mean_angular_coverage_deg") for t in obs_t], oc,
          fmt="{:.1f}", ylabel="deg", title="Viewpoint coverage\n(higher = better)",
          note="opposite sign vs sim (see summary)")
    ncam = [int(_f(cp[m], "num_base_cameras")) for m in cp_m]
    _bars(axr[2], cp_l, [_f(cp[m], "cross_camera_mean_mm") for m in cp_m], cc,
          ylabel="mm", title="Cross-camera agreement\n(proxy for cam pose err)",
          title_bg="#e3f2e8",
          note=f"registered cameras:  {ncam[0]}  /  {ncam[1]}  /  {ncam[2]}  of 4")
    _bars(axr[3], cp_l, [_f(cp[m], "pose_repeat_mm") for m in cp_m], cc,
          ylabel="mm", title="Pose repeatability\n(proxy for target pred. err)",
          title_bg="#e3f2e8")


def fig_c2(which="both"):
    meta, s, cp, obs = load_c2()
    sub_s = (f"sim: 4 fixed cameras, incidence $\\leq$ {meta['incidence']:.0f}$\\degree$, "
             f"{meta['shots']} shots/method, {meta['seeds']} seeds")
    sub_c = (f"real: data/session, {obs['board']['n_target_events']} board / "
             f"{obs['cube']['n_target_events']} cube target events")
    tail = "(gray title = observability,  green title = calibration result)"

    if which == "sim":
        fig, ax = plt.subplots(1, 4, figsize=(17.2, 4.6))
        _c2_sim_row(ax, s)
        ax[0].set_ylabel("[SIMULATION]\n" + ax[0].get_ylabel(), fontsize=9,
                         fontweight="bold")
        fig.legend(handles=C2_LEGEND[:2], loc="lower center", ncol=2, fontsize=9,
                   frameon=False, bbox_to_anchor=(0.5, -0.07))
        fig.suptitle("C2 — Board vs Cube   [SIMULATION, vs ground truth]\n"
                     + sub_s + "   " + tail, fontsize=12, y=1.02)
        fig.tight_layout(rect=[0, 0.04, 1, 0.9])
        return _save(fig, "fig_SIM_C2_board_vs_cube.png")

    if which == "cp":
        fig, ax = plt.subplots(1, 4, figsize=(17.2, 4.6))
        _c2_cp_row(ax, cp, obs)
        ax[0].set_ylabel("[REAL DATA (CP)]\n" + ax[0].get_ylabel(), fontsize=9,
                         fontweight="bold")
        fig.legend(handles=C2_LEGEND, loc="lower center", ncol=3, fontsize=9,
                   frameon=False, bbox_to_anchor=(0.5, -0.07))
        fig.suptitle("C2 — Board vs Cube   [REAL DATA, measured proxies]\n"
                     + sub_c + "   " + tail, fontsize=12, y=1.02)
        fig.tight_layout(rect=[0, 0.04, 1, 0.9])
        return _save(fig, "fig_CP_C2_board_vs_cube.png")

    fig, ax = plt.subplots(2, 4, figsize=(17.2, 8.6))
    _c2_sim_row(ax[0], s)
    _c2_cp_row(ax[1], cp, obs)
    _row_tag(ax[0][0], "SIMULATION", "#333333")
    _row_tag(ax[1][0], "REAL DATA (CP)", "#333333")
    fig.legend(handles=C2_LEGEND, loc="lower center", ncol=3, fontsize=9,
               frameon=False, bbox_to_anchor=(0.5, -0.035))
    fig.suptitle(
        "C2 — Planar Board only  vs  Marker Cube      "
        "SIMULATION (top, vs ground truth)   vs   REAL DATA (bottom, measured proxies)\n"
        + sub_s + "      |      " + sub_c + "      " + tail,
        fontsize=11.5, y=1.0)
    fig.tight_layout(rect=[0, 0.035, 1, 0.945])
    return _save(fig, "fig_SIMvsCP_C2_board_vs_cube.png")


# ------------------------------------------------------------------------- C3
# 시뮬 3방식 ↔ CP 3방식 대응 (CP_EXPERIMENTS_README.md 의 표와 동일)
C3_MAP = [
    ("Camera-based",   ("03_pose_consistency_opt", "without_robot_cube_prior"),
     "no-fk-prior",       BLUE),
    ("FK-based",       ("03_pose_consistency_opt", "with_robot_cube_prior"),
     "fk-prior",          RED),
    ("Camera+FK-corr", ("05_fk_prior_correction", "with_robot_cube_prior"),
     "fk-prior\n+correction", GREEN),
]
C3_SIM_NOISE_MM = 6.0   # 시뮬 sweep 중 C1/C2 와 동일한 운영점


def load_c3():
    sim = _load_json(os.path.join(SIM_FIG_DIR, "exp3_noise_sweep_data.json"))
    rows = _read_csv(os.path.join(CP_DIR, "C3", "ablation_summary.csv"))
    cp = {(r["method"], r["prior_mode"]): r for r in rows}
    return sim, cp


def _c3_rel_panel(a, vals, lbls, colors, hatches):
    """baseline(첫 항목) 대비 상대변화(%). 색은 방법을 따르고 방향이 좋고나쁨."""
    rel = [_pct(vals[0], v) for v in vals]
    xsr = np.arange(len(rel))
    for x, v, c, h in zip(xsr, rel, colors, hatches):
        a.bar(x, v, color=c, hatch=h, edgecolor="black", linewidth=0.6,
              alpha=0.9, width=0.62)
    span = max(abs(min(rel)), abs(max(rel))) or 1.0
    for x, v in zip(xsr, rel):
        a.text(x, v + np.sign(v or 1) * span * 0.05, f"{v:+.1f}%",
               ha="center", va="bottom" if v >= 0 else "top",
               fontsize=8.5, fontweight="bold", color="#333333")
    a.axhline(0, color="#444444", lw=1.0)
    a.set_xticks(xsr)
    a.set_xticklabels(lbls, rotation=18, ha="right", fontsize=8)
    a.set_ylim(min(rel) - span * 0.35, max(rel) + span * 0.35)
    a.grid(axis="y", alpha=0.25)
    a.set_axisbelow(True)
    a.set_ylabel("change vs baseline (%)", fontsize=9)
    a.set_title("Relative to Camera-based baseline\n(negative = better)",
                fontsize=9.5, fontweight="bold",
                bbox=dict(boxstyle="round,pad=0.35", fc="#e3f2e8", ec="none"))


def _c3_data(sim, cp):
    xs = sim["xs"]
    i6 = xs.index(C3_SIM_NOISE_MM) if C3_SIM_NOISE_MM in xs else len(xs) // 2
    names = [m[0] for m in C3_MAP]
    colors = [m[3] for m in C3_MAP]
    hatches = ["", "", FK_HATCH]
    sim_held = [sim["curve"][n]["heldout"][i6] for n in names]
    rows = [cp[m[1]] for m in C3_MAP]
    lbl = [m[2] for m in C3_MAP]
    test = [_f(r, "test_prior_trans_rmse_mm") for r in rows]
    train = [_f(r, "prior_trans_rmse_mm") for r in rows]
    return xs, names, colors, hatches, sim_held, lbl, test, train


def _c3_sim_row(axr, sim, names, colors, hatches, sim_held, xs):
    _bars(axr[0], names, sim_held, colors, hatches=hatches, fmt="{:.3f}",
          ylabel="error vs GT (mm)",
          title="Held-out cube prediction\n(lower = better)", title_bg="#e3f2e8")
    _c3_rel_panel(axr[1], sim_held, names, colors, hatches)
    a = axr[2]
    for n, c in zip(names, colors):
        a.plot(xs, sim["curve"][n]["heldout"], marker="o", color=c, lw=2, ms=7, label=n)
    a.axvline(C3_SIM_NOISE_MM, color="gray", ls="--", alpha=0.7)
    a.text(C3_SIM_NOISE_MM, a.get_ylim()[1] * 0.95, " operating\n point",
           fontsize=7.5, color="gray", va="top")
    a.set_xlabel("observation noise (mm)", fontsize=9)
    a.set_ylabel("error vs GT (mm)", fontsize=9)
    a.grid(alpha=0.25)
    a.set_axisbelow(True)
    a.legend(fontsize=8)
    a.set_title("Noise-robustness sweep\n(simulation only)", fontsize=9.5,
                fontweight="bold",
                bbox=dict(boxstyle="round,pad=0.35", fc="#eeeeee", ec="none"))


def _c3_cp_row(axr, colors, hatches, lbl, test, train):
    _bars(axr[0], lbl, test, colors, hatches=hatches, fmt="{:.2f}",
          ylabel="RMSE vs FK (mm)",
          title="Held-out cube prediction\n(lower = better)", title_bg="#e3f2e8")
    _c3_rel_panel(axr[1], test, lbl, colors, hatches)
    a = axr[2]
    w = 0.36
    xsr = np.arange(len(lbl))
    a.bar(xsr - w / 2, train, width=w, color="#bbbbbb", edgecolor="black",
          linewidth=0.6, label="train (fit to FK)")
    for x, v, c, h in zip(xsr + w / 2, test, colors, hatches):
        a.bar(x, v, width=w, color=c, hatch=h, edgecolor="black", linewidth=0.6,
              alpha=0.9)
    top = max(test + train)
    for x, v in zip(xsr - w / 2, train):
        a.text(x, v + top * 0.02, f"{v:.2f}", ha="center", va="bottom", fontsize=7.5)
    for x, v in zip(xsr + w / 2, test):
        a.text(x, v + top * 0.02, f"{v:.1f}", ha="center", va="bottom",
               fontsize=7.5, fontweight="bold")
    a.set_xticks(xsr)
    a.set_xticklabels(lbl, rotation=18, ha="right", fontsize=8)
    a.set_ylim(0, top * 1.22)
    a.grid(axis="y", alpha=0.25)
    a.set_axisbelow(True)
    a.set_ylabel("cube position RMSE vs FK (mm)", fontsize=9)
    a.legend(handles=[Patch(fc="#bbbbbb", ec="black", lw=0.6, label="train (fit to FK)"),
                      Patch(fc="white", ec="black", lw=0.6, label="held-out test")],
             fontsize=8, loc="upper left")
    a.set_title("Train vs held-out gap\n(FK overfit, real data only)", fontsize=9.5,
                fontweight="bold",
                bbox=dict(boxstyle="round,pad=0.35", fc="#eeeeee", ec="none"))


def fig_c3(which="both"):
    sim, cp = load_c3()
    xs, names, colors, hatches, sim_held, lbl, test, train = _c3_data(sim, cp)
    note = ("note: sim puts the correction on Camera-based; CP puts it on fk-prior "
            "(per project spec) — the green bars are not the same estimator")

    if which == "sim":
        fig, ax = plt.subplots(1, 3, figsize=(14.6, 4.7))
        _c3_sim_row(ax, sim, names, colors, hatches, sim_held, xs)
        ax[0].set_ylabel("[SIMULATION]\n" + ax[0].get_ylabel(), fontsize=9,
                         fontweight="bold")
        fig.suptitle("C3 — gTc estimation: Camera-based vs FK-based vs +correction   "
                     "[SIMULATION, vs ground truth]\n"
                     f"sim @ systematic noise {C3_SIM_NOISE_MM:.0f}mm, 8 sets",
                     fontsize=11.5, y=1.03)
        fig.tight_layout(rect=[0, 0, 1, 0.9])
        return _save(fig, "fig_SIM_C3_gtc_estimation.png")

    if which == "cp":
        fig, ax = plt.subplots(1, 3, figsize=(14.6, 4.7))
        _c3_cp_row(ax, colors, hatches, lbl, test, train)
        ax[0].set_ylabel("[REAL DATA (CP)]\n" + ax[0].get_ylabel(), fontsize=9,
                         fontweight="bold")
        fig.suptitle("C3 — gTc estimation: no-fk-prior vs fk-prior vs +correction   "
                     "[REAL DATA, vs robot-FK proxy]\n"
                     "real: data/session, train=9/test=4   |   " + note,
                     fontsize=11, y=1.04)
        fig.tight_layout(rect=[0, 0, 1, 0.88])
        return _save(fig, "fig_CP_C3_gtc_estimation.png")

    fig, ax = plt.subplots(2, 3, figsize=(14.6, 8.8))
    _c3_sim_row(ax[0], sim, names, colors, hatches, sim_held, xs)
    _c3_cp_row(ax[1], colors, hatches, lbl, test, train)
    _row_tag(ax[0][0], "SIMULATION", "#333333")
    _row_tag(ax[1][0], "REAL DATA (CP)", "#333333")
    fig.suptitle(
        "C3 — gTc estimation:  Camera-based (no FK prior)  vs  FK-based (FK prior)  vs  "
        "+ residual correction\n"
        "SIMULATION (top, vs ground truth)   vs   REAL DATA (bottom, vs robot-FK proxy)      |      "
        f"sim @ systematic noise {C3_SIM_NOISE_MM:.0f}mm, 8 sets   |   "
        "real: data/session, train=9/test=4\n" + note,
        fontsize=11, y=1.0)
    fig.tight_layout(rect=[0, 0, 1, 0.925])
    return _save(fig, "fig_SIMvsCP_C3_gtc_estimation.png")


# ------------------------------------------------------------------- summary
def _verdict(sim_d, cp_d):
    """시뮬 주장이 실데이터에서 재현됐는지: 방향 일치 + 효과 크기."""
    if not np.isfinite(sim_d) or not np.isfinite(cp_d) or sim_d == 0:
        return "N/A"
    ratio = cp_d / sim_d            # >0 이면 같은 방향
    if abs(cp_d) < abs(sim_d) * 0.05:
        return "X  (효과없음)"      # 부호와 무관하게 실데이터에선 사실상 변화 없음
    if ratio < 0:
        return "X  (반대)"
    if ratio < 0.25:
        return "△  (방향만)"
    return "O"


def build_summary():
    _, s1, cp1 = load_c1()
    _, s2, cp2, obs2 = load_c2()
    sim3, cp3 = load_c3()
    i6 = sim3["xs"].index(C3_SIM_NOISE_MM)

    def sim3v(n):
        return sim3["curve"][n]["heldout"][i6]

    c3r = {m[2].replace("\n", " "): cp3[m[1]] for m in C3_MAP}
    c3 = {k: _f(v, "test_prior_trans_rmse_mm") for k, v in c3r.items()}

    rows = []

    def add(contrib, metric, unit, sb, sp, cb, cp_, base_lbl, prop_lbl):
        sd, cd = _pct(sb, sp), _pct(cb, cp_)
        rows.append(dict(
            contribution=contrib, metric=metric, unit=unit,
            baseline=base_lbl, proposed=prop_lbl,
            sim_baseline=sb, sim_proposed=sp, sim_change_pct=sd,
            cp_baseline=cb, cp_proposed=cp_, cp_change_pct=cd,
            reproduced=_verdict(sd, cd)))

    # ---- C1 ----
    add("C1", "hand-eye gTc (sim: vs GT / CP: gripper-align RMSE)", "mm",
        s1["Indep"]["gTc_mm"][0], s1["Joint"]["gTc_mm"][0],
        _f(cp1["independent"], "grip_align_trans_rmse_mm"),
        _f(cp1["unified_joint"], "grip_align_trans_rmse_mm"),
        "independent", "unified_joint")
    add("C1", "shared-base consistency", "mm",
        s1["Indep"]["consistency_mm"][0], s1["Joint"]["consistency_mm"][0],
        _f(cp1["independent"], "consistency_trans_rmse_mm"),
        _f(cp1["unified_joint"], "consistency_trans_rmse_mm"),
        "independent", "unified_joint")
    add("C1", "held-out cube prediction (no correction)", "mm",
        s1["Indep"]["downstream_mm"][0], s1["Joint"]["downstream_mm"][0],
        _f(cp1["independent"], "downstream_trans_rmse_mm"),
        _f(cp1["unified_joint"], "downstream_trans_rmse_mm"),
        "independent", "unified_joint")
    add("C1", "held-out cube prediction: effect of +fk correction", "mm",
        s1["Joint"]["downstream_mm"][0], s1["Joint+fk"]["downstream_mm"][0],
        _f(cp1["unified_joint"], "downstream_trans_rmse_mm"),
        _f(cp1["unified_joint"], "downstream_fk_trans_rmse_mm"),
        "unified_joint", "unified_joint +fk")

    # ---- C2 ----
    add("C2", "simultaneous observers", "cameras/shot",
        s2["board"]["simul"][0], s2["board+cube"]["simul"][0],
        _f(obs2["board"], "mean_simul_observers"),
        _f(obs2["cube"], "mean_simul_observers"), "board", "cube")
    add("C2", "viewpoint coverage", "deg",
        s2["board"]["coverage"][0], s2["board+cube"]["coverage"][0],
        _f(obs2["board"], "mean_angular_coverage_deg"),
        _f(obs2["cube"], "mean_angular_coverage_deg"), "board", "cube")
    add("C2", "camera pose err (sim: vs GT / CP: cross-camera)", "mm",
        s2["board"]["cam_mm"][0], s2["board+cube"]["cam_mm"][0],
        _f(cp2["board_only"], "cross_camera_mean_mm"),
        _f(cp2["hybrid"], "cross_camera_mean_mm"), "board_only", "hybrid")
    add("C2", "target pred. err (sim: vs GT / CP: pose repeatability)", "mm",
        s2["board"]["obj_mm"][0], s2["board+cube"]["obj_mm"][0],
        _f(cp2["board_only"], "pose_repeat_mm"),
        _f(cp2["hybrid"], "pose_repeat_mm"), "board_only", "hybrid")

    # ---- C3 ----
    add("C3", "held-out prediction: FK prior vs camera-only", "mm",
        sim3v("Camera-based"), sim3v("FK-based"),
        c3["no-fk-prior"], c3["fk-prior"], "Camera-based", "FK-based")
    # 후보정을 얹는 대상이 서로 다르다: 시뮬 = Camera-based 위, CP = fk-prior 위(프로젝트 규약).
    # 각자의 실제 baseline 대비 효과를 비교한다.
    add("C3", "held-out prediction: effect of residual correction", "mm",
        sim3v("Camera-based"), sim3v("Camera+FK-corr"),
        c3["fk-prior"], c3["fk-prior +correction"],
        "sim: Camera-based / CP: fk-prior", "+ correction")
    add("C3", "best method vs camera-only baseline", "mm",
        sim3v("Camera-based"), min(sim3v(n) for n in
                                   ["Camera-based", "FK-based", "Camera+FK-corr"]),
        c3["no-fk-prior"], min(c3.values()), "Camera-based / no-fk-prior", "best")
    return rows


def write_summary(rows):
    os.makedirs(CP_DIR, exist_ok=True)
    cols = ["contribution", "metric", "unit", "baseline", "proposed",
            "sim_baseline", "sim_proposed", "sim_change_pct",
            "cp_baseline", "cp_proposed", "cp_change_pct", "reproduced"]
    csv_path = os.path.join(CP_DIR, "SIM_vs_CP_summary.csv")
    with open(csv_path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=cols)
        w.writeheader()
        w.writerows(rows)
    print(f"[저장] {csv_path}")

    def num(v, nd=2):
        return "—" if not np.isfinite(v) else f"{v:,.{nd}f}"

    lines = [
        "# 시뮬(Simul_test) ↔ 실데이터(CP_result) 통합 비교표",
        "",
        "`CP_viz_sim_vs_real.py` 가 두 산출물에서 직접 읽어 생성. 변화율(%)은 baseline 대비이며",
        "**음수 = 개선(오차 감소)**, 관측성 지표(동시관측·시야각)만 **양수 = 개선**이다.",
        "재현 판정: `O` 방향·크기 모두 재현, `△` 방향만 재현(효과 1/4 미만), `X` 반대 방향.",
        "",
        "| 기여 | 지표 | baseline → proposed | 시뮬 base | 시뮬 prop | 시뮬 Δ% | CP base | CP prop | CP Δ% | 재현 |",
        "|---|---|---|---:|---:|---:|---:|---:|---:|:---:|",
    ]
    for r in rows:
        lines.append(
            f"| {r['contribution']} | {r['metric']} | {r['baseline']} → {r['proposed']} "
            f"| {num(r['sim_baseline'], 3)} | {num(r['sim_proposed'], 3)} "
            f"| {num(r['sim_change_pct'], 1)}% "
            f"| {num(r['cp_baseline'])} | {num(r['cp_proposed'])} "
            f"| {num(r['cp_change_pct'], 1)}% | {r['reproduced']} |")
    lines += [
        "",
        "## 단위·프록시 주의",
        "",
        "- 시뮬은 GT 대비 절대오차(mm), CP 는 GT 가 없어 **로봇 FK 큐브중점 프록시** 대비 오차다.",
        "  절대 크기를 직접 비교하지 말고 **Δ% 와 방향**을 비교할 것.",
        "- C1 `bTf`(고정카메라 절대오차)는 실데이터에서 측정 불가 → 표·figure 에서 제외(N/A).",
        "- C1 gTc 는 CP 에서 `grip_align_trans_rmse_mm`(그리퍼 정합 RMSE) 프록시로 대체.",
        "- C2 카메라오차/타깃예측은 CP 에서 각각 `cross_camera_mean_mm`/`pose_repeat_mm` 프록시.",
        "- C3 후보정은 시뮬이 Camera-based 위에, CP 가 fk-prior 위에 얹는다(프로젝트 규약).",
        "  따라서 '후보정 효과' 행의 baseline 이 서로 다르다.",
        "",
        "생성: `PYTHONPATH= python CP_viz_sim_vs_real.py`",
    ]
    md_path = os.path.join(CP_DIR, "SIM_vs_CP_summary.md")
    with open(md_path, "w") as f:
        f.write("\n".join(lines) + "\n")
    print(f"[저장] {md_path}")
    return csv_path, md_path


FIGS = {"C1": fig_c1, "C2": fig_c2, "C3": fig_c3}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--only", choices=["C1", "C2", "C3", "summary"],
                    help="하나만 생성 (기본: 전부)")
    ap.add_argument("--which", choices=["sim", "cp", "both", "all"], default="all",
                    help="sim=시뮬만, cp=실데이터만, both=한이미지비교, all=셋다(기본)")
    args = ap.parse_args()

    variants = ["sim", "cp", "both"] if args.which == "all" else [args.which]
    targets = [args.only] if args.only in ("C1", "C2", "C3") else ["C1", "C2", "C3"]

    if args.only != "summary":
        for c in targets:
            for v in variants:
                FIGS[c](v)
    if args.only in (None, "summary"):
        write_summary(build_summary())


if __name__ == "__main__":
    main()
