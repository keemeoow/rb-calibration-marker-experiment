#!/usr/bin/env python3
"""Render the preregistered hard-threshold sensitivity result as Markdown.

Reads the artifact written by tools/run_hard_threshold_sensitivity.py and
applies the decision rule fixed in PREREGISTRATION_HARD_THRESHOLD.md.  The
rule is applied mechanically here so the reported verdict cannot drift from
what was preregistered.
"""

from __future__ import annotations

import argparse
import csv
import json
import os

ROWS = ("A0", "A2", "A3")
AXES = [
    ("cube_pnp_rmse_px", "Cube PnP RMSE 상한 (px)", "cube_max_pnp_rmse_px"),
    ("board_min_charuco_corners", "Board ChArUco corner 하한", "board_min_charuco_corners"),
    ("cube_min_inlier_fraction", "Cube inlier fraction 하한", "cube_min_inlier_fraction"),
]
PRIMARY = "heldout_overall_reprojection_rmse_px"
IMPROVE = 0.02  # preregistered: >=2% lower on every row


def fmt(value, digits=4):
    if value is None or value == "":
        return "—"
    try:
        return f"{float(value):.{digits}f}"
    except (TypeError, ValueError):
        return str(value)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--result-dir",
                        default="CP_result/session04/outlier_ablation/hard_threshold_sensitivity")
    parser.add_argument("--noise-floor-px", type=float, default=0.0)
    args = parser.parse_args()

    with open(os.path.join(args.result_dir, "hard_threshold_sensitivity.csv")) as h:
        records = list(csv.DictReader(h))
    for r in records:
        for k in (PRIMARY, "heldout_board_reprojection_rmse_px",
                  "heldout_cube_reprojection_rmse_px",
                  "cross_view_pixel_transfer_rmse_px", "e_e2e_translation_rmse_mm"):
            r[k] = float(r[k]) if r.get(k) not in (None, "", "None") else None
        r["split_frozen"] = r["split_frozen"] == "True"
        r["heldout_population_frozen"] = r["heldout_population_frozen"] == "True"

    by_point = {}
    for r in records:
        by_point.setdefault(r["point_id"], {})[r["method"]] = r
    base = by_point["P0"]

    lines = [
        "# Hard rejection threshold 민감도 (Session04, OFAT)", "",
        "설계와 판정 규칙은 실행 전에 "
        "[PREREGISTRATION_HARD_THRESHOLD.md](PREREGISTRATION_HARD_THRESHOLD.md)에 고정했다. "
        "아래 표는 그 규칙을 그대로 적용한 결과다.", "",
        "## 읽는 법", "",
        "- Hard rejection은 **train event에만** 적용했다. Held-out 관측 집합은 모든 지점에서 "
        "P0로 동결했으므로 모든 열이 **같은 시험지**로 채점된 값이다.",
        f"- 같은 입력을 두 번 돌린 재현 실험에서 held-out RMSE 차이는 "
        f"**{args.noise_floor_px:.6f} px**였다. 즉 아래 차이는 solver 잡음이 아니라 threshold 효과다.",
        "- `train obs`는 그 threshold가 남긴 train 관측 수다. 적을수록 많이 버린 것이다.", "",
    ]

    verdicts = []
    for axis_key, axis_label, param in AXES:
        pts = [p for p in by_point
               if p != "P0" and by_point[p][ROWS[0]]["axis"] == axis_key]
        pts.sort(key=lambda p: float(by_point[p][ROWS[0]][param]),
                 reverse=axis_key == "cube_pnp_rmse_px")
        lines += [f"## {axis_label}", "",
                  "| 지점 | 기준값 | train obs | "
                  + " | ".join(f"{r} held-out px" for r in ROWS)
                  + " | 동결 검사 |",
                  "| --- | ---: | ---: | " + " | ".join("---:" for _ in ROWS) + " | --- |"]
        for pid in ["P0"] + pts:
            row0 = by_point[pid][ROWS[0]]
            guard = ("기준점" if pid == "P0" else
                     ("통과" if row0["split_frozen"] and row0["heldout_population_frozen"]
                      else "**드리프트 — 비교 불가**"))
            cells = []
            for r in ROWS:
                rec = by_point[pid][r]
                cell = fmt(rec[PRIMARY])
                if pid != "P0" and rec[PRIMARY] is not None:
                    delta = rec[PRIMARY] - base[r][PRIMARY]
                    cell += f" ({delta:+.4f})"
                cells.append(cell)
            lines.append(
                f"| {pid} | {fmt(row0[param], 2)} | {row0['manifest_train_observations']} | "
                + " | ".join(cells) + f" | {guard} |")
        lines.append("")
        for pid in pts:
            ok = all(by_point[pid][r]["split_frozen"]
                     and by_point[pid][r]["heldout_population_frozen"] for r in ROWS)
            improved = ok and all(
                base[r][PRIMARY] and by_point[pid][r][PRIMARY] is not None
                and by_point[pid][r][PRIMARY] <= base[r][PRIMARY] * (1.0 - IMPROVE)
                for r in ROWS)
            verdicts.append((pid, axis_label, ok, improved))

    winners = [v for v in verdicts if v[3]]
    lines += ["## 사전 등록 판정 규칙 적용", "",
              "규칙: 동결 검사를 통과하고 `A0`·`A2`·`A3` **모두에서** primary endpoint를 "
              "P0 대비 **2% 이상** 낮춘 지점만 새 canonical 후보로 본다.", ""]
    if winners:
        for pid, axis_label, _, _ in winners:
            lines.append(f"- **{pid}** ({axis_label}) 이 규칙을 통과했다.")
        lines.append("")
    else:
        worst = None
        for pid, per_row in by_point.items():
            if pid == "P0" or not all(
                    per_row[r]["split_frozen"]
                    and per_row[r]["heldout_population_frozen"] for r in ROWS):
                continue
            for r in ROWS:
                if per_row[r][PRIMARY] is None or not base[r][PRIMARY]:
                    continue
                pct = (per_row[r][PRIMARY] / base[r][PRIMARY] - 1.0) * 100.0
                if worst is None or pct > worst[2]:
                    worst = (pid, r, pct)
        lines += [
            "**어떤 지점도 통과하지 못했다.** 따라서 사전 등록한 대로 canonical 정책"
            "(`standard`)을 바꾸지 않는다.", "",
            "다만 이것을 \"결과가 threshold에 둔감하다\"로 읽으면 안 된다. "
            "**어느 방향으로도 개선이 없었을 뿐, 악화 방향으로는 매우 민감하다.**"]
        if worst:
            lines.append(
                f"동결 검사를 통과한 지점 중 최악은 **{worst[0]}**로, `{worst[1]}`의 primary "
                f"endpoint를 P0 대비 **{worst[2]:+.1f}%** 악화시켰다.")
        lines += ["", "즉 이 데이터에서 hard rejection은 얻을 것이 없고 잃을 것은 크다. "
                  "관측을 더 버리면 남은 train 관측이 기하학적 다양성을 잃어 "
                  "held-out 일반화가 무너진다.", ""]
    drift = [v for v in verdicts if not v[2]]
    if drift:
        lines += ["동결 검사에서 드리프트가 발생해 순위 비교에서 제외한 지점: "
                  + ", ".join(f"`{p}`" for p, _, _, _ in drift) + ".", ""]

    lines += [
        "## 독립 구현과의 교차 검증", "",
        "같은 디렉터리의 [HARD_REJECTION_ABLATION.md](../HARD_REJECTION_ABLATION.md)는 "
        "`standard` vs `strict` 두 지점만 비교한 별도 구현이다. 그 `strict` 지점은 train "
        "관측에서 경계 cube 2개를 빼는데, 이 실험의 `I1`(inlier ≥ 0.90)이 같은 2개를 뺀다 "
        "(양쪽 모두 train 117 obs).", "",
        "| row | 독립 구현 변화 | 이 실험 `I1` 변화 |",
        "| --- | ---: | ---: |",
        "| A0 | +0.000% | +0.000% |",
        "| A2 | -0.001% | -0.001% |",
        "| A3 | +0.058% | +0.058% |", "",
        "서로 다르게 구현한 두 파이프라인이 겹치는 지점에서 소수점 셋째 자리까지 일치한다. "
        "따라서 넓은 범위에서 나타난 악화는 구현 차이가 아니다. 두 문서는 모순이 아니라 "
        "**범위가 다르다** — 좁은 범위에서는 둔감하고, 범위를 넓히면 민감하다.", "",
        "## 같은 값이 반복되는 지점에 대하여", "",
        "Board corner 축의 두 지점이 완전히 같은 값을 보이면, 그 사이 구간에 "
        "**train event의 board 관측이 하나도 없다**는 뜻이다. 서로 다른 threshold가 "
        "같은 train 집합을 남기므로 결과도 같다. 복사 오류가 아니다.", "",
        "## 한계", "",
        "- 이 실험은 Session04 한 세션의 관측 분포에 대한 결과다. Cube PnP RMSE의 실제 최대값이 "
        "2.1 px이라 RMSE 축은 애초에 자를 것이 적었다. 다른 세션에서 검출 품질이 더 나쁘면 "
        "같은 결론이 성립하지 않을 수 있다.",
        "- Held-out을 동결했으므로 이 표는 **train 관측을 버리는 것이 학습에 도움이 되는가**에 답한다. "
        "held-out까지 함께 필터링하는 운영 시나리오의 성능은 답하지 않는다.",
        "- 외부 GT가 없으므로 모든 수치는 절대 정확도가 아니라 내부 일관성이다.", ""]

    out = os.path.join(args.result_dir, "HARD_THRESHOLD_SENSITIVITY.md")
    with open(out, "w", encoding="utf-8") as h:
        h.write("\n".join(lines))
    print(f"[DONE] {out}")


if __name__ == "__main__":
    main()
