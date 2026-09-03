#!/usr/bin/env python3
"""Build a focused Korean PDF deck for the 8/3 meeting feedback resolution.

The deck is intentionally shorter than the full Session04 result deck.  It
answers one presentation question: "What feedback was raised, how did we fix or
scope it, and what do the current data say?"
"""

from __future__ import annotations

import argparse
from pathlib import Path
import sys
from typing import Iterable

from PIL import Image, ImageDraw

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from tools.make_calibration_result_presentation import (  # noqa: E402
    AMBER,
    BLUE,
    GREEN,
    GRAY_BAR,
    H,
    INK,
    LINE,
    MARGIN_X,
    MUTED,
    NAVY,
    NAVY_2,
    ORANGE,
    PANEL,
    PAPER,
    PURPLE,
    RED,
    TEAL,
    W,
    bar_chart,
    card,
    content_base,
    dark_slide,
    draw_wrapped,
    fmt,
    font,
    footer,
    load_context,
    metric,
    pct_drop,
    pill,
    rounded,
    small_stat,
    table,
    text_w,
)

OUT_PDF = ROOT / "캘리브레이션_8-3_피드백_해결_시각화_발표자료.pdf"
OUT_PNG_DIR = ROOT / "CP_result/session04/feedback_resolution_slides"


def delta(ctx: dict, first: str, second: str, key: str) -> float:
    return metric(ctx, second, key) - metric(ctx, first, key)


def signed(value: float) -> str:
    return f"{value:+.4f}"


def delta_bars(
    draw: ImageDraw.ImageDraw,
    box: tuple[int, int, int, int],
    title: str,
    entries: list[tuple[str, float, str]],
    *,
    note: str,
) -> None:
    x0, y0, x1, y1 = box
    rounded(draw, box, fill=PAPER, outline=LINE, radius=15)
    draw.text((x0 + 24, y0 + 18), title, font=font(26, "bold"), fill=INK)

    chart_x0, chart_x1 = x0 + 310, x1 - 120
    axis_x = (chart_x0 + chart_x1) // 2
    row_y = y0 + 96
    row_gap = 72
    max_abs = max(abs(v) for _, v, _ in entries) * 1.18
    max_abs = max(max_abs, 0.05)

    draw.line((axis_x, y0 + 76, axis_x, y1 - 86), fill="#C8D3E0", width=3)
    draw.text((chart_x0, y1 - 78), "개선", font=font(18, "bold"), fill=TEAL)
    draw.text((axis_x + 16, y1 - 78), "악화", font=font(18, "bold"), fill=RED)

    for idx, (label, value, caption) in enumerate(entries):
        y = row_y + idx * row_gap
        draw.text((x0 + 24, y - 6), label, font=font(21, "bold"), fill=INK)
        draw_wrapped(draw, (x0 + 24, y + 22), caption, font(16), MUTED, 250, gap=4, max_lines=2)

        half_w = (chart_x1 - chart_x0) // 2 - 18
        bar_w = max(4, int(half_w * abs(value) / max_abs))
        color = TEAL if value < 0 else RED
        if value < 0:
            rect = (axis_x - bar_w, y, axis_x, y + 28)
            value_x = axis_x - bar_w - 90
        else:
            rect = (axis_x, y, axis_x + bar_w, y + 28)
            value_x = axis_x + bar_w + 16
        rounded(draw, rect, fill=color, radius=8)
        draw.text((value_x, y - 2), signed(value), font=font(19, "bold"), fill=color)

    draw_wrapped(draw, (x0 + 24, y1 - 54), note, font(19), MUTED, x1 - x0 - 48, gap=6)


def slide_cover(ctx: dict, i: int, n: int) -> Image.Image:
    del ctx
    img = Image.new("RGB", (W, H), NAVY)
    draw = ImageDraw.Draw(img)
    draw.text((100, 114), "8/3 meeting feedback", font=font(29, "bold"), fill="#8FADE0")
    draw_wrapped(
        draw,
        (100, 174),
        "피드백 → 해결 방식 → 실제 데이터",
        font(62, "black"),
        PAPER,
        1250,
        gap=14,
    )
    draw_wrapped(
        draw,
        (100, 334),
        "Session04 캘리브레이션 결과를 8/3 피드백 항목별로 재구성한 발표자료",
        font(31),
        "#CBD5E1",
        1120,
        gap=12,
    )
    rounded(draw, (100, 468, 1080, 532), fill=NAVY_2, radius=10)
    draw.text(
        (124, 482),
        "A2 = internal main | A4 = preflight | A5 = post-hoc diagnostic",
        font=font(25, "mono"),
        fill="#DCE7F7",
    )
    draw.text((100, 740), "현재 범위: Pre-GT internal evaluation", font=font(24), fill="#AAB7CA")
    draw.text((100, 782), "External GT / robot task: 다음주 예정 태스크", font=font(24, "bold"), fill="#F8D879")
    footer(draw, i, n, dark=True, label="8/3 feedback resolution")
    return img


def slide_status(ctx: dict, i: int, n: int) -> Image.Image:
    del ctx
    img, draw, y = content_base(
        i,
        n,
        "AT A GLANCE",
        "8/3 피드백 반영 상태",
        "19개 피드백을 코드·문서·산출물 기준으로 재확인했다.",
    )
    small_stat(draw, (68, y, 384, y + 138), "11", "반영 완료", GREEN)
    small_stat(draw, (420, y, 736, y + 138), "7", "부분 반영 / 일정 의존", AMBER)
    small_stat(draw, (772, y, 1088, y + 138), "1", "point-cloud 전용 평가 미구현", RED)
    small_stat(draw, (1124, y, 1564, y + 138), "다음주", "Independent External GT", BLUE)
    card(
        draw,
        (68, y + 178, 760, y + 428),
        "현재 완료로 말할 수 있는 것",
        "frame-prune/refit/rollback, 목적함수 항 분리, event-grouped split, "
        "matched contrast table, set-equal/paired bootstrap, OpenCV reference, "
        "camera-scope 진단까지 반영했다.",
        GREEN,
    )
    card(
        draw,
        (804, y + 178, 1564, y + 428),
        "다음주로 남긴 것",
        "Independent External GT, 눈금 큐브 재파지, robot task success/failure, "
        "translation/rotation/P95/failure rate는 현재 데이터로 계산하지 않는다.",
        ORANGE,
    )
    return img


def slide_feedback_clusters(ctx: dict, i: int, n: int) -> Image.Image:
    del ctx
    img, draw, y = content_base(
        i,
        n,
        "FEEDBACK MAP",
        "피드백을 6개 문제로 묶으면",
        "각 문제는 해결 방식과 현재 산출물 위치가 다르다.",
    )
    rows = [
        ["#1 #2 #18", "이상치·검출 QA", "image-level frame-prune, refit/rollback, overlay QA", "완료"],
        ["#3 #4 #9 #19", "목적함수/FK 설명", "visual 1항 또는 visual+FK 2항으로 재정의", "완료/부분"],
        ["#5 #8 #10 #11 #12", "평가지표 공정성", "matched contrast, event split, mm/deg 보조 지표", "완료"],
        ["#6", "수치가 큰 원인", "Board-Cube conflict 17.299→10.808 mm까지 낮춤", "부분"],
        ["#7", "공개 구현 대조", "OpenCV direct relative-pose reference 구현", "부분"],
        ["#14 #15 #16 #17", "robot task/GT", "Track C schema와 evaluator 준비, 실측은 다음주", "부분/미구현"],
    ]
    table(
        draw,
        72,
        y,
        [170, 300, 760, 170],
        ["피드백", "문제", "해결 방식", "상태"],
        rows,
        row_font_size=19,
        header_font_size=17,
        max_bottom=810,
    )
    return img


def slide_solution_contract(ctx: dict, i: int, n: int) -> Image.Image:
    del ctx
    img, draw, y = content_base(
        i,
        n,
        "FIX CONTRACT",
        "피드백 → 해결 방식 → 확인 자료",
        "발표에서 질문이 들어오면 이 표의 세 번째 칸으로 바로 답하면 된다.",
    )
    rows = [
        ["프레임 이상치 제거", "MAD 기반 image-level frame-prune 후 refit, 악화 시 rollback", "15개 stage가 시도 후 rollback"],
        ["목적함수 혼동", "camera-to-camera residual 없음. shared target pose로 결합", "Visual 1항 / FK rows 2항"],
        ["FK 항이 묻힘", "visual/FK cost block을 분리 보고", "A4 FK cost fraction 0.122%"],
        ["지표 편향", "Own-heldout + set-equal + paired bootstrap + camera-scope", "Board 703 / Cube 236 corners"],
        ["공개 구현 비교", "main method와 독립인 OpenCV relative-pose baseline", "별도 JSON/CSV/MD 산출물"],
        ["robot task 정확도", "Independent External GT와 paired task-trial schema 유지", "다음주 측정 예정"],
    ]
    table(
        draw,
        72,
        y,
        [320, 690, 340],
        ["피드백", "해결 방식", "현재 확인"],
        rows,
        row_font_size=20,
        header_font_size=17,
        max_bottom=810,
    )
    return img


def slide_algorithm(ctx: dict, i: int, n: int) -> Image.Image:
    del ctx
    img, draw, y = content_base(
        i,
        n,
        "ALGORITHM",
        "알고리즘 설명 최종본",
        "카메라 간 pose를 직접 평균하는 방법이 아니라, 공유 target pose를 통해 joint optimization하는 방법이다.",
    )
    card(
        draw,
        (68, y, 760, y + 208),
        "Visual-only rows",
        "A0, A1, A2, A3, A5, B3\n"
        "고정 K/D와 3D object corner → 2D native pixel reprojection residual만 사용한다.",
        BLUE,
    )
    card(
        draw,
        (804, y, 1564, y + 208),
        "Soft-FK rows",
        "A4, B1, B2\n"
        "visual reprojection에 covariance-whitened FK factor를 추가한다.",
        TEAL,
    )
    card(
        draw,
        (68, y + 248, 760, y + 446),
        "EIH/E2H 결합",
        "fixed cameras와 gripper camera는 board/cube target pose를 공유한다. "
        "그 공유 변수 때문에 서로 feedback이 생긴다.",
        GREEN,
    )
    card(
        draw,
        (804, y + 248, 1564, y + 446),
        "금지된 설명",
        "camera-to-camera residual을 objective에 넣었다거나, "
        "w1/w2/w3 3항 weighted sum을 쓴다는 설명은 현재 코드와 맞지 않는다.",
        RED,
    )
    return img


def slide_experiment_table(ctx: dict, i: int, n: int) -> Image.Image:
    del ctx
    img, draw, y = content_base(
        i,
        n,
        "EXPERIMENT DESIGN",
        "비교실험 구성 표",
        "전체 순위가 아니라, 한 번에 하나만 바뀌는 matched contrast로 해석한다.",
    )
    rows = [
        ["A0", "board", "seq", "-", "board-only baseline"],
        ["A1", "board+cube", "seq", "estimated", "cube 추가 reference"],
        ["A2", "board+cube", "unified", "estimated", "현재 internal main"],
        ["A3", "board+cube", "unified", "raw-FK hard", "FK 위험성 진단"],
        ["A4", "board+cube", "unified", "soft-FK", "preflight 후보"],
        ["A5", "board+cube", "unified", "aligned-FK hard", "post-hoc 진단"],
        ["B1", "board+cube", "seq", "soft-FK", "A4 대비 -Unified"],
        ["B2", "cube", "unified", "soft-FK", "A4 대비 -board"],
        ["B3", "board", "unified", "-", "A2 대비 -cube"],
    ]
    table(
        draw,
        86,
        y,
        [105, 205, 210, 260, 530],
        ["Row", "Marker", "Opt.", "Cube pose", "역할"],
        rows,
        row_font_size=20,
        header_font_size=17,
        max_bottom=816,
    )
    return img


def slide_metrics(ctx: dict, i: int, n: int) -> Image.Image:
    del ctx
    img, draw, y = content_base(
        i,
        n,
        "METRICS",
        "평가지표 최종 계층",
        "지표의 역할을 분리해야 FK에 유리한 숫자만 고른다는 오해를 피할 수 있다.",
    )
    rows = [
        ["Primary", "Own-marker held-out px", "matched contrast의 주 내부 지표"],
        ["Bias check", "Set-equal-weight px", "Board/Cube corner support 차이 점검"],
        ["Sensitivity", "Paired set bootstrap CI", "n=9 sets, 방향성 민감도"],
        ["Subsystem", "Fixed-to-Fixed", "FK-free 고정카메라 상대 일관성"],
        ["Closure", "Gripper-to-Fixed", "FK/Hand-Eye 포함 full-chain 내부 진단"],
        ["Next week", "TRE/Rotation/P95/Failure", "Independent External GT 후 최종 물리 정확도"],
    ]
    table(
        draw,
        100,
        y,
        [210, 440, 710],
        ["등급", "지표", "사용법"],
        rows,
        row_font_size=22,
        header_font_size=18,
        max_bottom=790,
    )
    card(
        draw,
        (100, 760, 1500, 832),
        "핵심",
        "현재 결론은 internal reprojection consistency이고, absolute robot-base accuracy는 다음주 GT 이후다.",
        ORANGE,
        title_size=22,
        body_size=21,
    )
    return img


def slide_main_result(ctx: dict, i: int, n: int) -> Image.Image:
    a1b = metric(ctx, "A1", "heldout_board_reprojection_rmse_px")
    a2b = metric(ctx, "A2", "heldout_board_reprojection_rmse_px")
    a1c = metric(ctx, "A1", "heldout_cube_reprojection_rmse_px")
    a2c = metric(ctx, "A2", "heldout_cube_reprojection_rmse_px")
    a1o = metric(ctx, "A1", "heldout_overall_reprojection_rmse_px")
    a2o = metric(ctx, "A2", "heldout_overall_reprojection_rmse_px")
    img, draw, y = content_base(
        i,
        n,
        "DATA RESULT 1",
        "A1 → A2: 현재 가장 강한 내부 결과",
        "동일 board+cube 관측에서 sequential frozen-stage를 unified joint optimization으로 바꾼 비교다.",
    )
    bar_chart(
        draw,
        (68, y, 1008, y + 448),
        "Held-out reprojection RMSE px",
        [("A1 board", a1b, GRAY_BAR), ("A2 board", a2b, BLUE), ("A1 cube", a1c, GRAY_BAR), ("A2 cube", a2c, TEAL)],
        max_value=4.6,
        note="낮을수록 좋다. A2는 board와 cube에서 모두 개선된다.",
    )
    card(draw, (1048, y, 1564, y + 190), "숫자 요약", f"Own overall {fmt(a1o)} → {fmt(a2o)} px\n{pct_drop(a1o, a2o)} 감소", GREEN)
    card(
        draw,
        (1048, y + 230, 1564, y + 448),
        "발표 문장",
        "“같은 데이터 조건에서 unified feedback을 넣으면 두 target의 held-out residual이 함께 낮아졌습니다.”",
        BLUE,
    )
    return img


def slide_cube_result(ctx: dict, i: int, n: int) -> Image.Image:
    entries = [
        (
            "A0 → A1 board",
            delta(ctx, "A0", "A1", "heldout_board_reprojection_rmse_px"),
            "순차 구조에서 cube만 추가",
        ),
        (
            "B3 → A2 board",
            delta(ctx, "B3", "A2", "heldout_board_reprojection_rmse_px"),
            "unified 구조에서 cube coupling",
        ),
        (
            "A1 → A2 cube",
            delta(ctx, "A1", "A2", "heldout_cube_reprojection_rmse_px"),
            "same marker, unified feedback",
        ),
        (
            "B2 → A4 cube",
            delta(ctx, "B2", "A4", "heldout_cube_reprojection_rmse_px"),
            "soft-FK preflight에서 board 도움",
        ),
    ]
    img, draw, y = content_base(
        i,
        n,
        "DATA RESULT 2",
        "Cube claim은 조건부로 바꾼다",
        "cube를 단순히 추가하면 항상 좋아진다는 claim은 현재 데이터와 맞지 않는다.",
    )
    delta_bars(
        draw,
        (68, y, 1088, y + 462),
        "Second - first Δ RMSE px",
        entries,
        note="음수는 두 번째 방법이 더 좋다는 뜻이다. cube 효과는 unified/soft-FK context에서 더 설득력 있게 보인다.",
    )
    card(
        draw,
        (1126, y, 1564, y + 202),
        "수정된 claim",
        "Cube는 universal improvement가 아니라, board/cube 관측을 하나의 solver에서 연결하는 3D common target이다.",
        ORANGE,
    )
    card(
        draw,
        (1126, y + 246, 1564, y + 462),
        "왜 중요한가",
        "이렇게 말해야 A0→A1의 미세 악화를 숨기지 않으면서도 cube 설계 의도를 유지할 수 있다.",
        BLUE,
    )
    return img


def slide_fk_result(ctx: dict, i: int, n: int) -> Image.Image:
    img, draw, y = content_base(
        i,
        n,
        "DATA RESULT 3",
        "FK는 hard GT가 아니라 uncertainty-aware prior",
        "A2, A3, A4, A5는 FK를 어떻게 넣을지에 대한 원인 분리다.",
    )
    entries = [
        ("A2 board", metric(ctx, "A2", "heldout_board_reprojection_rmse_px"), BLUE),
        ("A3 board", metric(ctx, "A3", "heldout_board_reprojection_rmse_px"), RED),
        ("A4 board", metric(ctx, "A4", "heldout_board_reprojection_rmse_px"), TEAL),
        ("A2 cube", metric(ctx, "A2", "heldout_cube_reprojection_rmse_px"), BLUE),
        ("A3 cube", metric(ctx, "A3", "heldout_cube_reprojection_rmse_px"), RED),
        ("A4 cube", metric(ctx, "A4", "heldout_cube_reprojection_rmse_px"), TEAL),
    ]
    bar_chart(
        draw,
        (68, y, 1040, y + 462),
        "Held-out reprojection RMSE px",
        entries,
        max_value=7.0,
        note="A3 hard-FK fixed는 특히 cube에서 크게 악화된다. A4는 A2와 거의 tie다.",
    )
    card(
        draw,
        (1080, y, 1564, y + 146),
        "A2",
        "현재 internal main. no-FK cube pose estimated.",
        BLUE,
    )
    card(
        draw,
        (1080, y + 174, 1564, y + 318),
        "A3",
        "raw FK hard fixed가 나빠짐. FK를 GT로 쓰면 위험.",
        RED,
    )
    card(
        draw,
        (1080, y + 346, 1564, y + 462),
        "A4",
        "soft-FK preflight. 우월성은 다음주 GT 필요.",
        TEAL,
    )
    return img


def slide_data_quality(ctx: dict, i: int, n: int) -> Image.Image:
    del ctx
    img, draw, y = content_base(
        i,
        n,
        "DATA QA",
        "데이터 문제는 없는가?",
        "계산/동기화 오류는 보이지 않지만, 남은 데이터 리스크는 숨기지 않는다.",
    )
    small_stat(draw, (68, y, 384, y + 138), "27/27", "모든 row·seed 수렴", GREEN)
    small_stat(draw, (420, y, 736, y + 138), "9 sets", "eligible set 4~12", BLUE)
    small_stat(draw, (772, y, 1088, y + 138), "108", "accepted cube PnP obs", GREEN)
    small_stat(draw, (1124, y, 1564, y + 138), "10.8077 mm", "Board-Cube conflict", ORANGE)
    card(
        draw,
        (68, y + 178, 760, y + 418),
        "문제 아님",
        "JSON/CSV/Markdown/HTML은 같은 canonical data에서 동기화된다. "
        "PnP failure는 0이고, cube RMSE rejection은 2건이다.",
        GREEN,
    )
    card(
        draw,
        (804, y + 178, 1564, y + 418),
        "해석 한계",
        "held-out은 n=9 sets이고 board/cube support가 다르다. "
        "Board-Cube systematic disagreement도 10.8077 mm 남아 있다.",
        ORANGE,
    )
    return img


def slide_script_next(ctx: dict, i: int, n: int) -> Image.Image:
    a1 = metric(ctx, "A1", "heldout_overall_reprojection_rmse_px")
    a2 = metric(ctx, "A2", "heldout_overall_reprojection_rmse_px")
    img, draw, y = content_base(
        i,
        n,
        "CLOSING",
        "마지막 발표 멘트",
        "이 문단이 현재 결론의 가장 안전한 버전이다.",
    )
    script = (
        f"“8/3 피드백 이후 표를 전체 순위표가 아니라 matched ablation 표로 재구성했습니다. "
        f"현재 주 지표는 같은 marker population의 held-out reprojection이고, A1→A2에서 own overall이 "
        f"{fmt(a1)} px에서 {fmt(a2)} px로 낮아져 unified feedback 효과가 가장 분명합니다. "
        f"반면 cube 효과는 조건부이고, hard FK는 악화되며, soft-FK는 아직 preflight입니다. "
        f"따라서 현 단계 결론은 A2가 internal main이고, robot-base 물리 정확도는 다음주 Independent External GT로 "
        f"Translation/Rotation/P95/Failure Rate를 산출해 확정하겠습니다.”"
    )
    rounded(draw, (92, y + 10, 1510, y + 330), fill="#EEF5FF", outline="#BED8FF", radius=18)
    draw_wrapped(draw, (132, y + 48), script, font(27, "medium"), INK, 1336, gap=11)
    card(
        draw,
        (92, y + 370, 1510, y + 474),
        "다음 액션",
        "External GT가 들어오면 이 deck의 A4/A5 해석과 physical metric slide만 갱신하면 된다.",
        BLUE,
        title_size=23,
        body_size=23,
    )
    return img


SLIDES = [
    slide_cover,
    slide_status,
    slide_feedback_clusters,
    slide_solution_contract,
    slide_algorithm,
    slide_experiment_table,
    slide_metrics,
    slide_main_result,
    slide_cube_result,
    slide_fk_result,
    slide_data_quality,
    slide_script_next,
]


def render_deck(out_pdf: Path, png_dir: Path | None = None) -> list[Image.Image]:
    ctx = load_context()
    total = len(SLIDES)
    images = [slide(ctx, idx + 1, total) for idx, slide in enumerate(SLIDES)]
    out_pdf.parent.mkdir(parents=True, exist_ok=True)
    images[0].save(out_pdf, save_all=True, append_images=images[1:], resolution=120.0, quality=95)
    if png_dir:
        png_dir.mkdir(parents=True, exist_ok=True)
        for idx, image in enumerate(images, start=1):
            image.save(png_dir / f"slide_{idx:02d}.png")
    return images


def main(argv: Iterable[str] | None = None) -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--out", type=Path, default=OUT_PDF)
    parser.add_argument("--png-dir", type=Path, default=OUT_PNG_DIR)
    args = parser.parse_args(list(argv) if argv is not None else None)
    images = render_deck(args.out, args.png_dir)
    print(f"[DONE] wrote {args.out} ({len(images)} slides)")
    if args.png_dir:
        print(f"[DONE] wrote slide PNGs to {args.png_dir}")


if __name__ == "__main__":
    main()
