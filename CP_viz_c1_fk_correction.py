#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""C1 held-out 큐브예측 오차 — FK 정보를 얼마나 쓰느냐에 따른 비교 피규어.

`CP_result/C1/joint_ablation_summary.csv` 를 읽어 세 보정 단계를 나란히 그린다
(재계산 없음 — CP_C1 을 --holdout_frac 로 한 번 돌린 뒤 실행):

    down_mm      : FK 미사용. 카메라 관측만으로 held-out set 큐브 위치를 예측.
    down+se3_mm  : train 잔차를 강체 SE(3)(회전+평행이동, 자유도 6)로 정렬.
    down+fk_mm   : train 잔차를 Ridge [1,x,y](자유도 9 — 회전·스케일·전단)로 보정.

핵심은 **강체 SE(3) 만으로 목표 5mm 안에 들어온다**는 것이다. 남은 오차가 임의의
곡선맞춤이 아니라 base 프레임 정렬 잔차(≈3°, ≈26mm)라는 뜻이고, 그건 앵커/촬영 개선으로
실제로 없앨 수 있는 종류의 오차다.

실행:  PYTHONPATH= python CP_viz_c1_fk_correction.py
"""
import csv
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.patheffects as pe
import matplotlib.pyplot as plt
from matplotlib.patches import Patch

ROOT = os.path.dirname(os.path.abspath(__file__))
CSV = os.path.join(ROOT, "CP_result", "C1", "joint_ablation_summary.csv")
OUT = os.path.join(ROOT, "CP_result", "figures", "fig_CP_C1_fk_correction.png")

# 순서형(ordinal) 파랑 램프 — "FK 정보를 더 쓸수록 진하게". 값은 dataviz 기준 팔레트의
# sequential blue 에서 step 250 / 450 / 650 (light 표면용 ordinal 하한 250 을 지킴).
RAMP = ["#86b6ef", "#2a78d6", "#104281"]
INK, INK2, MUTED = "#0b0b0b", "#52514e", "#8a8a86"
TARGET_MM = 5.0

SERIES = [
    ("downstream_trans_rmse_mm", "FK 미사용\n(카메라 관측만)"),
    ("downstream_se3_trans_rmse_mm", "+ 강체 SE(3)\n(train, 자유도 6)"),
    ("downstream_fk_trans_rmse_mm", "+ Ridge [1,x,y]\n(train, 자유도 9)"),
]
METHOD_LABEL = {
    "independent": "independent",
    "unified_joint": "unified_joint",
    "joint_fk_fixed": "joint_fk_fixed",
}


def load_rows():
    with open(CSV, newline="") as f:
        rows = list(csv.DictReader(f))
    if not rows or not rows[0].get("downstream_se3_trans_rmse_mm"):
        raise SystemExit(
            "down+se3 열이 없다. 먼저 held-out split 으로 C1 을 돌려라:\n"
            "  PYTHONPATH= python CP_C1_unified_vs_independent.py "
            "--root_folder data/session --intrinsics_dir intrinsics --holdout_frac 0.3")
    return rows


def main():
    rows = load_rows()
    methods = [r["method"] for r in rows]
    fnum = lambda v: float(v) if v not in ("", None) else float("nan")

    # AppleGothic 은 대괄호·도(°) 기호를 CJK 글리프로 대체해 버린다 — SD Gothic Neo 사용.
    plt.rcParams.update({
        "font.family": ["Apple SD Gothic Neo", "NanumGothic", "AppleGothic"],
        "axes.unicode_minus": False,
        "figure.facecolor": "#fcfcfb",
        "axes.facecolor": "#fcfcfb",
    })
    fig, ax = plt.subplots(figsize=(9.8, 5.6))

    n = len(SERIES)
    group_w, gap = 0.78, 0.02          # 인접 막대 사이 표면 간격
    bar_w = (group_w - gap * (n - 1)) / n
    baseline = {}
    for j, (col, label) in enumerate(SERIES):
        xs, vals = [], []
        for i, r in enumerate(rows):
            xs.append(i - group_w / 2 + bar_w / 2 + j * (bar_w + gap))
            vals.append(fnum(r[col]))
        bars = ax.bar(xs, vals, width=bar_w, color=RAMP[j], linewidth=0,
                      zorder=3, label=label)
        for i, (b, v) in enumerate(zip(bars, vals)):
            if j == 0:
                baseline[i] = v
            # 값 라벨은 목표선과 겹칠 수 있어 표면색 halo 를 둘러 항상 읽히게 한다.
            ax.annotate(f"{v:.2f}", (b.get_x() + b.get_width() / 2, v),
                        xytext=(0, 4), textcoords="offset points",
                        ha="center", va="bottom", fontsize=9.5, zorder=5,
                        color=INK if v > TARGET_MM else "#1c5cab",
                        fontweight="bold" if v <= TARGET_MM else "normal",
                        path_effects=[pe.withStroke(linewidth=3.2, foreground="#fcfcfb")])
            if j > 0:      # 기준(FK 미사용) 대비 감소율 — 막대 안쪽에 조용히
                ax.annotate(f"-{(1 - v / baseline[i]) * 100:.0f}%",
                            (b.get_x() + b.get_width() / 2, v),
                            xytext=(0, -13), textcoords="offset points",
                            ha="center", va="top", fontsize=8.5, zorder=5,
                            color="#ffffff")

    ax.axhline(TARGET_MM, color=MUTED, lw=1.2, ls=(0, (4, 3)), zorder=2)
    ax.annotate(f"목표 {TARGET_MM:.0f} mm", (len(rows) - 0.52, TARGET_MM),
                xytext=(-2, 5), textcoords="offset points",
                ha="right", va="bottom", fontsize=9, color=INK2, zorder=5,
                path_effects=[pe.withStroke(linewidth=3.2, foreground="#fcfcfb")])

    ax.set_xticks(range(len(rows)))
    ax.set_xticklabels([METHOD_LABEL.get(m, m) for m in methods], fontsize=10.5, color=INK)
    # 세로 회전한 한글은 읽기 나쁘다 — 축 단위를 축 위에 가로로 둔다.
    ax.set_ylim(0, max(fnum(r[SERIES[0][0]]) for r in rows) * 1.22)
    ax.annotate("held-out 큐브 위치 예측 RMSE (mm)", (0, 1), xycoords="axes fraction",
                xytext=(-42, 12), textcoords="offset points",
                ha="left", va="bottom", fontsize=9.5, color=INK2)
    ax.tick_params(axis="y", labelsize=9, colors=INK2, length=0)
    ax.tick_params(axis="x", length=0)
    ax.grid(axis="y", color="#e6e5e1", lw=0.8, zorder=0)
    ax.set_axisbelow(True)
    for side in ("top", "right", "left"):
        ax.spines[side].set_visible(False)
    ax.spines["bottom"].set_color("#d8d7d3")

    ax.legend(handles=[Patch(facecolor=RAMP[j], label=SERIES[j][1]) for j in range(n)],
              loc="upper right", frameon=False, fontsize=9, labelcolor=INK2,
              handlelength=1.1, handleheight=1.1, borderpad=0.2, labelspacing=0.7)

    test_sets = rows[0].get("test_sets", "")
    train_sets = rows[0].get("train_sets", "")
    ax.set_title("held-out 큐브 위치 예측 오차 — FK 정보를 어디까지 쓰는가",
                 fontsize=13.5, color=INK, pad=34, loc="left")

    rigid = "   ".join(
        f"{r['method']} {float(r['fk_rigid_angle_deg']):.1f}°/{float(r['fk_rigid_trans_mm']):.0f}mm"
        for r in rows if r.get("fk_rigid_angle_deg"))
    fig.text(0.008, -0.015,
             f"data/session · train set [{train_sets}] / test set [{test_sets}] · 낮을수록 좋음\n"
             f"보정은 모두 train set 에서만 학습해 held-out test 예측에 적용 — test FK 는 비교 대상으로만 쓴다.\n"
             f"추정된 강체보정 크기(회전/평행이동): {rigid}  →  남은 오차의 대부분은 base 프레임 정렬 잔차다.",
             fontsize=8.5, color=INK2, va="top", linespacing=1.6)

    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    fig.savefig(OUT, dpi=160, bbox_inches="tight", facecolor=fig.get_facecolor())
    print(f"[DONE] {OUT}")

    print("\n" + "-" * 78)
    print(f"{'method':16s} {'down':>8s} {'down+se3':>10s} {'down+fk':>9s}   (mm)")
    for r in rows:
        print(f"{r['method']:16s} {fnum(r[SERIES[0][0]]):8.2f} "
              f"{fnum(r[SERIES[1][0]]):10.2f} {fnum(r[SERIES[2][0]]):9.2f}")


if __name__ == "__main__":
    main()
