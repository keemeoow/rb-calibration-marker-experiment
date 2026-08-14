#!/usr/bin/env python3
"""FK 오차와 카메라 오차를 동시에 준 조건에서 세 방법을 평가한다.

지금까지는 한 축씩만 흔들었다. 실제 현장에서는 둘이 함께 나빠지므로
격자로 훑어 어느 영역에서 어느 방법이 유리한지 본다.

격자 3개 (각각 FK 축 × 카메라 축)
  A. FK 계통 × 코너 검출 노이즈(랜덤)
  B. FK 계통 × 내부파라미터 오차(계통)
  C. FK 랜덤 × 코너 검출 노이즈(랜덤)

평가는 중앙값을 쓴다. 세트가 적거나 노이즈가 크면 일부 seed 가 발산해
평균이 그 하나에 끌려가기 때문이다.

실행:
  python3.10 run_combined_study.py --seeds 8 --workers 32
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
from matplotlib.colors import TwoSlopeNorm, ListedColormap
from matplotlib.patches import Patch

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from core import ExpConfig, run_config

for _p in ("/usr/share/fonts/truetype/nanum/NanumBarunGothic.ttf",
           "/usr/share/fonts/truetype/nanum/NanumGothic.ttf"):
    if os.path.exists(_p):
        font_manager.fontManager.addfont(_p)
        plt.rcParams["font.family"] = font_manager.FontProperties(fname=_p).get_name()
        break
plt.rcParams["axes.unicode_minus"] = False

C_NOFK, C_FIXED, C_OURS = "#2a78d6", "#eb6834", "#1baf7a"
INK, INK2, MUTED = "#0b0b0b", "#52514e", "#8c8b85"
GRID, SURFACE = "#e6e5e0", "#fcfcfb"

CB = ("cube", "board")
BASE = dict(n_sets=14, train_size=11, n_splits=1, n_events_per_set=6,
            n_gripped_events=0, use_real_layout=True)
METHODS = [("noFK", "no-FK", C_NOFK, "none"),
           ("fixed", "fixed-FK", C_FIXED, "fixed"),
           ("ours", "corrected-FK", C_OURS, "corr")]

# (격자 키, FK축 이름/인자/값, 카메라축 이름/인자/값)
GRIDS = [
    ("A", "FK 계통오차 (mm)", "fk_sys_mm", [0.0, 3.0, 6.0, 9.0, 12.0],
     "코너 검출 노이즈 (px)", "sigma_px", [0.2, 0.5, 0.8, 1.1, 1.4]),
    ("B", "FK 계통오차 (mm)", "fk_sys_mm", [0.0, 3.0, 6.0, 9.0, 12.0],
     "내부파라미터 오차", "intrinsic_err", [0.0, 0.005, 0.01, 0.02, 0.03]),
    ("C", "FK 랜덤오차 (mm)", "fk_noise_mm", [0.0, 3.0, 6.0, 9.0, 12.0],
     "코너 검출 노이즈 (px)", "sigma_px", [0.2, 0.5, 0.8, 1.1, 1.4]),
]
FIXED_CAM_DEFAULT = dict(sigma_px=0.5, intrinsic_err=0.01)


def _job(spec):
    key, fk, kw, seeds = spec
    res = run_config(ExpConfig("m", fk=fk, solve="unified", markers=CB),
                     seeds=seeds, **kw)
    m, sd, n, med = res["e_task_mm"]
    return key, (m, sd, n, med)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seeds", type=int, default=8)
    ap.add_argument("--workers", type=int, default=32)
    ap.add_argument("--replot", action="store_true")
    a = ap.parse_args()
    outdir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "figs_real")
    os.makedirs(outdir, exist_ok=True)
    raw_path = os.path.join(outdir, "combined_raw.json")

    if a.replot:
        out = {k: tuple(v) for k, v in json.load(open(raw_path)).items()}
    else:
        jobs = []
        for gk, _, fk_arg, fk_vals, _, cam_arg, cam_vals in GRIDS:
            for mk, _, _, fk in METHODS:
                for fv in fk_vals:
                    for cv in cam_vals:
                        kw = dict(BASE)
                        kw.update(FIXED_CAM_DEFAULT)
                        kw[fk_arg] = fv
                        kw[cam_arg] = cv
                        jobs.append((f"{gk}|{mk}|{fv}|{cv}", fk, kw, a.seeds))
        print(f"총 {len(jobs)}개 설정 × seed {a.seeds}개", flush=True)
        with ProcessPoolExecutor(max_workers=a.workers) as ex:
            out = {}
            for i, (k, v) in enumerate(ex.map(_job, jobs), 1):
                out[k] = v
                if i % 25 == 0:
                    print(f"  {i}/{len(jobs)}", flush=True)
        json.dump({k: list(v) for k, v in out.items()}, open(raw_path, "w"), indent=1)

    MED = 3  # 중앙값 인덱스
    fig, axes = plt.subplots(2, 3, figsize=(16.5, 9.2))
    win_cmap = ListedColormap([C_NOFK, C_FIXED, C_OURS])
    for c, (gk, fk_name, _, fk_vals, cam_name, _, cam_vals) in enumerate(GRIDS):
        M = np.array([[[out[f"{gk}|{mk}|{fv}|{cv}"][MED] for cv in cam_vals]
                       for fv in fk_vals] for mk, _, _, _ in METHODS])
        # 위: 승자
        ax = axes[0][c]
        winner = M.argmin(axis=0)
        ax.imshow(winner, cmap=win_cmap, vmin=0, vmax=2, aspect="auto", origin="lower")
        for i in range(len(fk_vals)):
            for j in range(len(cam_vals)):
                ax.text(j, i, f"{M[winner[i, j], i, j]:.2f}", ha="center",
                        va="center", fontsize=7.5, color="white", fontweight="bold")
        ax.set_title(f"[{gk}] 어느 방법이 최저인가", color=INK, fontsize=11,
                     pad=8, loc="left")

        # 아래: corrected-FK 가 나머지 최선보다 얼마나 나은가 (음수 = corrected 우세)
        ax2 = axes[1][c]
        others = np.minimum(M[0], M[1])
        gain = M[2] - others
        lim = max(abs(gain).max(), 1e-6)
        im = ax2.imshow(gain, cmap="coolwarm", aspect="auto", origin="lower",
                        norm=TwoSlopeNorm(vcenter=0, vmin=-lim, vmax=lim))
        for i in range(len(fk_vals)):
            for j in range(len(cam_vals)):
                ax2.text(j, i, f"{gain[i, j]:+.2f}", ha="center", va="center",
                         fontsize=7.5, color=INK)
        ax2.set_title("corrected-FK − (다른 둘 중 최선), 음수면 우세",
                      color=INK, fontsize=11, pad=8, loc="left")
        fig.colorbar(im, ax=ax2, fraction=0.04, pad=0.02)

        for axx in (ax, ax2):
            axx.set_xticks(range(len(cam_vals)))
            axx.set_xticklabels([f"{v:g}" for v in cam_vals], fontsize=8)
            axx.set_yticks(range(len(fk_vals)))
            axx.set_yticklabels([f"{v:g}" for v in fk_vals], fontsize=8)
            axx.set_xlabel(cam_name, color=INK2, fontsize=9)
            axx.set_ylabel(fk_name, color=INK2, fontsize=9)
            axx.tick_params(colors=MUTED)
    axes[0][0].legend(handles=[Patch(facecolor=c3, label=lab)
                               for _, lab, c3, _ in METHODS],
                      loc="upper left", bbox_to_anchor=(0, -0.16),
                      ncol=3, frameon=False, fontsize=9)
    fig.suptitle("FK 오차와 카메라 오차를 동시에 준 조건 (값은 held-out 작업 오차 중앙값, mm)",
                 color=INK, fontsize=12.5, x=0.007, ha="left", y=0.995)
    fig.tight_layout(rect=[0, 0, 1, 0.972])
    path = os.path.join(outdir, "14_combined_grid.png")
    fig.savefig(path, dpi=150, facecolor=SURFACE)
    plt.close(fig)
    print("→", path, flush=True)

    L = ["# FK 오차 + 카메라 오차 동시 조건\n",
         "session02 실측 배치, 학습 11세트. 값은 held-out 작업 오차의 **중앙값**(mm).",
         "평균 대신 중앙값을 쓰는 이유는 일부 seed 가 발산하면 평균이 끌려가기 때문이다.\n"]
    for gk, fk_name, _, fk_vals, cam_name, _, cam_vals in GRIDS:
        L.append(f"## 격자 {gk}: {fk_name} × {cam_name}\n")
        for mk, lab, _, _f in METHODS:
            L.append(f"**{lab}**\n")
            L.append(f"| {fk_name} \\ {cam_name} | "
                     + " | ".join(f"{v:g}" for v in cam_vals) + " |")
            L.append("|---|" + "---|" * len(cam_vals))
            for fv in fk_vals:
                L.append(f"| {fv:g} | " + " | ".join(
                    f"{out[f'{gk}|{mk}|{fv}|{cv}'][MED]:.2f}" for cv in cam_vals) + " |")
            L.append("")
    open(os.path.join(outdir, "combined_summary.md"), "w").write("\n".join(L))
    print("완료.", outdir, flush=True)


if __name__ == "__main__":
    main()
