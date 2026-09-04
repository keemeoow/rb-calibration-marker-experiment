#!/usr/bin/env python3
"""Build a 16:9 Korean slide PDF for the Session04 calibration result.

The reference PDF in this repository was generated as 960 x 540 pt pages.
This script renders 1600 x 900 px slides and saves them at 120 dpi, producing
the same page geometry while keeping the layout deterministic without browser
dependencies.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
from typing import Callable, Iterable

from PIL import Image, ImageDraw, ImageFont, JpegImagePlugin  # noqa: F401

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from tools.sync_table1_canonical_data import (
    _data_warnings,
    _load,
    _method_rows,
)


OUT_PDF = ROOT / "캘리브레이션_실험결과_발표자료.pdf"

TABLE1_JSON = ROOT / "CP_result/session04/late_table1/table1_methods.json"
CROSS_JSON = (
    ROOT / "CP_result/session04/cross_target_evaluation/"
    "cross_target_evaluation.json"
)
MARKER_JSON = (
    ROOT / "CP_result/session04/marker_system_end_to_end/"
    "marker_system_end_to_end.json"
)
OPENCV_JSON = (
    ROOT / "CP_result/session04/opencv_relative_baseline/"
    "opencv_relative_baseline.json"
)

W, H = 1600, 900
MARGIN_X = 68
FOOTER_Y = 858

NAVY = "#162335"
NAVY_2 = "#223148"
INK = "#161B24"
MUTED = "#536173"
SOFT_MUTED = "#7C8796"
LINE = "#D9E1EB"
PANEL = "#F4F7FB"
PAPER = "#FFFFFF"
BLUE = "#2F80ED"
BLUE_DARK = "#1F5FB9"
TEAL = "#0F766E"
GREEN = "#13966D"
AMBER = "#D49D22"
ORANGE = "#F05A28"
RED = "#D94B48"
PURPLE = "#725AC1"
GRAY_BAR = "#93A3B5"

FONT_REG = Path("/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc")
FONT_MED = Path("/usr/share/fonts/opentype/noto/NotoSansCJK-Medium.ttc")
FONT_BOLD = Path("/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc")
FONT_BLACK = Path("/usr/share/fonts/opentype/noto/NotoSansCJK-Black.ttc")
FONT_MONO = Path("/usr/share/fonts/truetype/noto/NotoSansMono-Regular.ttf")


_FONT_CACHE: dict[tuple[str, int], ImageFont.FreeTypeFont] = {}


def font(size: int, weight: str = "regular") -> ImageFont.FreeTypeFont:
    paths = {
        "regular": FONT_REG,
        "medium": FONT_MED,
        "bold": FONT_BOLD,
        "black": FONT_BLACK,
        "mono": FONT_MONO,
    }
    path = paths[weight]
    key = (str(path), size)
    if key not in _FONT_CACHE:
        _FONT_CACHE[key] = ImageFont.truetype(str(path), size=size)
    return _FONT_CACHE[key]


def bbox(draw: ImageDraw.ImageDraw, text: str, fnt: ImageFont.FreeTypeFont):
    return draw.textbbox((0, 0), text, font=fnt)


def text_w(draw: ImageDraw.ImageDraw, text: str, fnt: ImageFont.FreeTypeFont) -> int:
    box = bbox(draw, text, fnt)
    return box[2] - box[0]


def line_h(fnt: ImageFont.FreeTypeFont, gap: int = 8) -> int:
    return fnt.size + gap


def wrap_text(
    draw: ImageDraw.ImageDraw,
    text: str,
    fnt: ImageFont.FreeTypeFont,
    max_width: int,
) -> list[str]:
    lines: list[str] = []
    for para in text.split("\n"):
        if not para:
            lines.append("")
            continue
        words = para.split(" ")
        current = ""
        for word in words:
            candidate = word if not current else f"{current} {word}"
            if text_w(draw, candidate, fnt) <= max_width:
                current = candidate
                continue
            if current:
                lines.append(current)
                current = ""
            if text_w(draw, word, fnt) <= max_width:
                current = word
                continue
            chunk = ""
            for ch in word:
                candidate = chunk + ch
                if text_w(draw, candidate, fnt) <= max_width:
                    chunk = candidate
                else:
                    if chunk:
                        lines.append(chunk)
                    chunk = ch
            current = chunk
        if current:
            lines.append(current)
    return lines


def draw_wrapped(
    draw: ImageDraw.ImageDraw,
    xy: tuple[int, int],
    text: str,
    fnt: ImageFont.FreeTypeFont,
    fill: str,
    max_width: int,
    gap: int = 8,
    max_lines: int | None = None,
) -> int:
    x, y = xy
    lines = wrap_text(draw, text, fnt, max_width)
    if max_lines is not None:
        lines = lines[:max_lines]
    for line in lines:
        draw.text((x, y), line, font=fnt, fill=fill)
        y += line_h(fnt, gap)
    return y


def rounded(
    draw: ImageDraw.ImageDraw,
    box: tuple[int, int, int, int],
    fill: str,
    outline: str | None = None,
    radius: int = 14,
    width: int = 1,
) -> None:
    draw.rounded_rectangle(box, radius=radius, fill=fill, outline=outline, width=width)


def pill(
    draw: ImageDraw.ImageDraw,
    xy: tuple[int, int],
    text: str,
    fill: str,
    fg: str = PAPER,
    size: int = 20,
    pad_x: int = 16,
    pad_y: int = 7,
) -> tuple[int, int, int, int]:
    x, y = xy
    fnt = font(size, "bold")
    box = bbox(draw, text, fnt)
    tw, th = box[2] - box[0], box[3] - box[1]
    rect = (x, y, x + tw + pad_x * 2, y + th + pad_y * 2)
    rounded(draw, rect, fill=fill, radius=9)
    draw.text((x + pad_x, y + pad_y - 1), text, font=fnt, fill=fg)
    return rect


def footer(
    draw: ImageDraw.ImageDraw,
    index: int,
    total: int,
    dark: bool = False,
    label: str = "Session04 calibration result",
) -> None:
    fill = "#AAB7CA" if dark else "#738196"
    draw.text((MARGIN_X, FOOTER_Y), label, font=font(21, "regular"), fill=fill)
    page = f"{index} / {total}"
    draw.text((W - MARGIN_X - text_w(draw, page, font(21)), FOOTER_Y), page,
              font=font(21), fill=fill)


def dark_slide(
    index: int,
    total: int,
    eyebrow: str,
    title: str,
    subtitle: str,
    note: str | None = None,
) -> Image.Image:
    img = Image.new("RGB", (W, H), NAVY)
    draw = ImageDraw.Draw(img)
    draw.text((112, 300), eyebrow, font=font(27, "bold"), fill="#8FADE0")
    draw_wrapped(draw, (112, 368), title, font(58, "black"), PAPER, 1200, gap=12)
    draw_wrapped(draw, (112, 510), subtitle, font(29), "#CBD5E1", 1200, gap=11)
    if note:
        draw_wrapped(draw, (112, 640), note, font(22), "#AAB7CA", 1140, gap=9)
    footer(draw, index, total, dark=True)
    return img


def content_base(
    index: int,
    total: int,
    badge: str,
    title: str,
    subtitle: str | None = None,
) -> tuple[Image.Image, ImageDraw.ImageDraw, int]:
    img = Image.new("RGB", (W, H), PAPER)
    draw = ImageDraw.Draw(img)
    pill(draw, (MARGIN_X, 48), badge, NAVY, size=19)
    draw_wrapped(draw, (MARGIN_X, 101), title, font(44, "black"), INK, 1240, gap=10)
    y = 158
    if subtitle:
        y = draw_wrapped(draw, (MARGIN_X, y), subtitle, font(24), "#3E4856", 1330, gap=8)
    footer(draw, index, total)
    return img, draw, y + 22


def card(
    draw: ImageDraw.ImageDraw,
    box: tuple[int, int, int, int],
    title: str,
    body: str,
    accent: str = BLUE,
    title_size: int = 27,
    body_size: int = 22,
    fill: str = PANEL,
) -> None:
    x0, y0, x1, y1 = box
    rounded(draw, box, fill=fill, outline=LINE, radius=15)
    draw.rectangle((x0, y0, x0 + 6, y1), fill=accent)
    draw.text((x0 + 24, y0 + 22), title, font=font(title_size, "bold"), fill=INK)
    draw_wrapped(draw, (x0 + 24, y0 + 66), body, font(body_size), "#2C3542",
                 x1 - x0 - 52, gap=8)


def small_stat(
    draw: ImageDraw.ImageDraw,
    box: tuple[int, int, int, int],
    value: str,
    label: str,
    color: str,
) -> None:
    x0, y0, x1, y1 = box
    rounded(draw, box, fill="#F8FAFC", outline=LINE, radius=14)
    draw.text((x0 + 22, y0 + 20), value, font=font(36, "black"), fill=color)
    draw_wrapped(draw, (x0 + 22, y0 + 72), label, font(19), MUTED, x1 - x0 - 42, gap=6)


def table(
    draw: ImageDraw.ImageDraw,
    x: int,
    y: int,
    widths: list[int],
    headers: list[str],
    rows: list[list[str]],
    *,
    row_font_size: int = 19,
    header_font_size: int = 18,
    max_width_pad: int = 18,
    header_fill: str = NAVY,
    row_fill: str = "#FFFFFF",
    alt_fill: str = "#F8FAFC",
    max_bottom: int = 820,
) -> int:
    hf = font(header_font_size, "bold")
    rf = font(row_font_size, "regular")
    h = 46
    cur_x = x
    for width, header in zip(widths, headers):
        draw.rectangle((cur_x, y, cur_x + width, y + h), fill=header_fill)
        draw_wrapped(draw, (cur_x + 10, y + 10), header, hf, PAPER,
                     width - max_width_pad, gap=5, max_lines=1)
        cur_x += width
    y += h
    for i, row in enumerate(rows):
        wrapped = [
            wrap_text(draw, cell, rf, width - max_width_pad)
            for cell, width in zip(row, widths)
        ]
        row_h = max(44, max(len(lines) for lines in wrapped) * line_h(rf, 4) + 18)
        if y + row_h > max_bottom:
            break
        fill = alt_fill if i % 2 else row_fill
        cur_x = x
        for width in widths:
            draw.rectangle((cur_x, y, cur_x + width, y + row_h), fill=fill, outline=LINE)
            cur_x += width
        cur_x = x
        for lines, width in zip(wrapped, widths):
            ty = y + 9
            for line in lines[:3]:
                draw.text((cur_x + 10, ty), line, font=rf, fill=INK)
                ty += line_h(rf, 4)
            cur_x += width
        y += row_h
    return y


def bar_chart(
    draw: ImageDraw.ImageDraw,
    box: tuple[int, int, int, int],
    title: str,
    entries: list[tuple[str, float, str]],
    *,
    max_value: float | None = None,
    note: str | None = None,
) -> None:
    x0, y0, x1, y1 = box
    rounded(draw, box, fill="#FFFFFF", outline=LINE, radius=15)
    draw.text((x0 + 24, y0 + 18), title, font=font(26, "bold"), fill=INK)
    chart_x0, chart_y0 = x0 + 58, y0 + 88
    chart_x1, chart_y1 = x1 - 28, y1 - (118 if note else 46)
    draw.line((chart_x0, chart_y1, chart_x1, chart_y1), fill=LINE, width=2)
    max_value = max_value or max(v for _, v, _ in entries) * 1.18
    gap = 18
    bar_w = int((chart_x1 - chart_x0 - gap * (len(entries) - 1)) / len(entries))
    for i, (label, value, color) in enumerate(entries):
        x = chart_x0 + i * (bar_w + gap)
        bh = int((chart_y1 - chart_y0) * value / max_value)
        y = chart_y1 - bh
        draw.rounded_rectangle((x, y, x + bar_w, chart_y1), radius=8, fill=color)
        val = f"{value:.4f}"
        draw.text((x + bar_w / 2 - text_w(draw, val, font(19, "bold")) / 2,
                   y - 29), val, font=font(19, "bold"), fill=color)
        draw_wrapped(draw, (x, chart_y1 + 12), label, font(17), MUTED,
                     bar_w + 8, gap=2, max_lines=2)
    if note:
        draw_wrapped(draw, (x0 + 24, y1 - 58), note, font(19), MUTED,
                     x1 - x0 - 48, gap=6)


def arrow(
    draw: ImageDraw.ImageDraw,
    start: tuple[int, int],
    end: tuple[int, int],
    color: str = BLUE,
    width: int = 4,
) -> None:
    draw.line((*start, *end), fill=color, width=width)
    ex, ey = end
    sx, sy = start
    dx = 1 if ex >= sx else -1
    draw.polygon([(ex, ey), (ex - 18 * dx, ey - 10), (ex - 18 * dx, ey + 10)],
                 fill=color)


def load_context() -> dict:
    table1 = _load(TABLE1_JSON)
    cross = _load(CROSS_JSON)
    marker = _load(MARKER_JSON)
    rows = _method_rows(table1, cross)
    by = {row["method"]: row for row in rows}
    warnings = _data_warnings(table1, cross)
    opencv = json.loads(OPENCV_JSON.read_text()) if OPENCV_JSON.exists() else {}
    return {
        "table1": table1,
        "cross": cross,
        "marker": marker,
        "rows": rows,
        "by": by,
        "warnings": warnings,
        "opencv": opencv,
    }


def metric(ctx: dict, method: str, key: str) -> float:
    value = ctx["by"][method][key]
    if value is None:
        raise KeyError(f"{method}.{key} is None")
    return float(value)


def fmt(value: float, digits: int = 4) -> str:
    return f"{value:.{digits}f}"


def pct_drop(before: float, after: float) -> str:
    return f"{100.0 * (before - after) / before:.2f}%"


SlideFn = Callable[[dict, int, int], Image.Image]


def slide_cover(ctx: dict, i: int, n: int) -> Image.Image:
    del ctx
    img = Image.new("RGB", (W, H), NAVY)
    draw = ImageDraw.Draw(img)
    draw_wrapped(draw, (100, 126), "Session04 캘리브레이션\n결과 발표자료",
                 font(64, "black"), PAPER, 1250, gap=14)
    body = (
        "8/3 미팅 피드백을 기준으로 비교실험 구성, 평가지표, 현재 결론의 "
        "허용 범위를 정리한 설명용 자료."
    )
    draw_wrapped(draw, (100, 318), body, font(31), "#CBD5E1", 1080, gap=12)
    code = "Final rows A0-A5/B1-B3 | heldout=cube | External cube GT pending"
    rounded(draw, (100, 442, 1030, 498), fill=NAVY_2, radius=10)
    draw.text((122, 454), code, font=font(25, "mono"), fill="#DCE7F7")
    draw.text((100, 760),
              "rb-calibration-marker-experiment · split seed 20260731 · 9 eligible sets",
              font=font(22), fill="#AAB7CA")
    draw.text((100, 800),
              "Independent External GT: 다음주 예정 태스크",
              font=font(22, "bold"), fill="#F8D879")
    footer(draw, i, n, dark=True)
    return img


def slide_reading(ctx: dict, i: int, n: int) -> Image.Image:
    del ctx
    img, draw, y = content_base(
        i, n, "OVERVIEW", "이 자료를 읽는 방법",
        "목표는 숫자를 외우는 게 아니라, 질문이 들어왔을 때 결론의 범위를 정확히 말하는 것이다.",
    )
    card(draw, (68, y, 945, y + 250), "슬라이드마다 같은 4단 구조",
         "① 질문: 교수님/리뷰어가 의심할 지점\n"
         "② 구성: 어떤 row끼리 비교해야 하는지\n"
         "③ 수치: 현재 Session04에서 관측된 방향\n"
         "④ 답변: 그대로 말할 수 있는 문장", BLUE)
    card(draw, (68, y + 276, 945, y + 428), "가장 중요한 한 문장",
         "최종 표는 A0~A5/B1~B3 한 벌만 남기고, heldout과 External GT 평가를 "
         "cube target으로 통일한 ablation이다.", BLUE)
    card(draw, (980, y, 1564, y + 428), "목차",
         "Part 1 · 현재 결론\n"
         "Part 2 · 알고리즘 구조\n"
         "Part 3 · 비교실험/평가지표\n"
         "Part 4 · 결과 해석\n"
         "Part 5 · 예상 질문 답변\n"
         "Part 6 · 다음주 External GT", TEAL)
    return img


def slide_meeting_status(ctx: dict, i: int, n: int) -> Image.Image:
    del ctx
    img, draw, y = content_base(
        i, n, "8/3 MEETING", "미팅 피드백 반영 상태",
        "원본 전사 8-3_meeting.txt를 기준으로 19개 피드백을 추출했고, 8-3_meeting.md에 최신 상태를 반영했다.",
    )
    small_stat(draw, (68, y, 386, y + 138), "11건", "코드/문서/결과에 반영 완료", GREEN)
    small_stat(draw, (420, y, 738, y + 138), "7건", "코드 준비 또는 일정 의존", AMBER)
    small_stat(draw, (772, y, 1090, y + 138), "1건", "point cloud 전용 평가는 미구현", RED)
    small_stat(draw, (1124, y, 1564, y + 138), "다음주", "Independent External GT 예정", BLUE)
    card(draw, (68, y + 178, 760, y + 418), "반영 완료의 핵심",
         "frame-prune/refit/rollback, 목적함수 항 구분, 동일 split/mask, "
         "FK-free/FK-dependent 평가 분리, OpenCV reference baseline, "
         "mm/deg 보조 지표를 문서와 결과표에 반영했다.", GREEN)
    card(draw, (804, y + 178, 1564, y + 418), "아직 완료라고 말하면 안 되는 것",
         "robot task 정확도, 눈금 큐브 재파지 실측, peg-in-hole/success rate, "
         "point cloud 기반 robot-view 평가는 현재 데이터에 없거나 다음주 태스크다.", ORANGE)
    return img


def slide_part1(ctx: dict, i: int, n: int) -> Image.Image:
    del ctx
    return dark_slide(
        i, n, "PART 1 · 현재 결론", "지금 무엇을 주장할 수 있는가?",
        "External GT 전에는 A0~A5/B1~B3 모두 최종 후보 행으로 유지한다.",
        "핵심은 내부 cube 지표와 External cube GT 최종 판정을 분리하는 것이다.",
    )


def slide_claim_envelope(ctx: dict, i: int, n: int) -> Image.Image:
    a2 = metric(ctx, "A2", "heldout_cube_reprojection_rmse_px")
    a4 = metric(ctx, "A4", "heldout_cube_reprojection_rmse_px")
    img, draw, y = content_base(
        i, n, "CLAIM ENVELOPE", "현재 가능한 최대 결론",
        "외부 GT 전에는 내부 지표로 방어 가능한 말과 금지해야 할 말을 분리한다.",
    )
    card(draw, (68, y, 516, y + 270), "지금 가능",
         f"External GT 전에는 A0~A5/B1~B3를 모두 후보로 유지한다.\n"
         f"A2 heldout cube는 "
         f"{fmt(a2)} px다.", GREEN)
    card(draw, (576, y, 1024, y + 270), "지금 금지",
         "A2가 robot base에서 물리적으로 가장 정확하다고 말하지 않는다. "
         "내부 reprojection과 절대 3D 정확도는 같은 지표가 아니다.", RED)
    card(draw, (1084, y, 1564, y + 270), "A4/A5의 위치",
         f"A4 heldout cube는 {fmt(a4)} px로 A2와 사실상 tie다. "
         "A5는 External GT 공개 전에 frozen하면 최종 후보로 비교 가능하다.", BLUE)
    card(draw, (68, y + 310, 1564, y + 440), "발표에서 그대로 말할 문장",
         "“최종 표는 A0~A5/B1~B3 한 벌만 유지하고, heldout과 External GT 평가는 cube 기준으로 통일합니다. "
         "최종 채택 방법은 External cube GT 결과로 정하겠습니다.”",
         BLUE, title_size=25, body_size=24)
    return img


def slide_part2(ctx: dict, i: int, n: int) -> Image.Image:
    del ctx
    return dark_slide(
        i, n, "PART 2 · 알고리즘 구조", "코드가 실제로 무엇을 하는가?",
        "camera-to-camera residual을 넣는 방법이 아니라, 공유 target pose로 EIH/E2H를 함께 푸는 구조다.",
    )


def slide_pipeline(ctx: dict, i: int, n: int) -> Image.Image:
    del ctx
    img, draw, y = content_base(
        i, n, "PIPELINE", "01 → 06 실행 흐름",
        "현재 발표의 결과는 Session04 canonical run에서 생성된 산출물을 기준으로 한다.",
    )
    steps = [
        ("01", "factory intrinsic 저장", "cam*.npz / depth scale"),
        ("02", "ChArUco intrinsic 보정", "05에서 K,D 고정"),
        ("03", "RGB-D + robot FK 촬영", "meta.json / images"),
        ("04", "관측 manifest 고정", "raw corner + SHA-256"),
        ("05", "A0~A5/B1~B3 fit", "frame-prune/refit/rollback"),
        ("06", "결과 보고서 생성", "matrices / summary"),
    ]
    x, step_w = 78, 226
    for idx, (num, title, desc) in enumerate(steps):
        x0 = x + idx * 246
        rounded(draw, (x0, y + 60, x0 + step_w, y + 260), fill=PANEL,
                outline=LINE, radius=16)
        pill(draw, (x0 + 18, y + 82), num, BLUE if idx < 4 else TEAL, size=20)
        draw_wrapped(draw, (x0 + 18, y + 135), title, font(24, "bold"), INK,
                     step_w - 36, gap=6)
        draw_wrapped(draw, (x0 + 18, y + 202), desc, font(18), MUTED,
                     step_w - 36, gap=5)
        if idx < len(steps) - 1:
            arrow(draw, (x0 + step_w + 6, y + 160), (x0 + step_w + 34, y + 160),
                  color="#9AA7B8", width=3)
    card(draw, (92, y + 326, 1510, y + 460), "설명 포인트",
         "05는 calibration을 만들고, 06은 05 결과를 읽어 보고서만 만든다. "
         "Cross-target, marker-system, OpenCV baseline은 calibration 완료 후 선택 평가다.",
         BLUE, title_size=25, body_size=23)
    return img


def slide_objective(ctx: dict, i: int, n: int) -> Image.Image:
    del ctx
    img, draw, y = content_base(
        i, n, "OBJECTIVE", "목적함수와 연결 구조",
        "교수님 질문: reprojection error인가, 3D error인가, camera-to-camera 관계도 넣었는가?",
    )
    card(draw, (68, y, 760, y + 208), "Visual-only rows",
         "A0 · A1 · A2 · A3 · A5 · B3\n"
         "robust native-pixel corner reprojection 1항만 사용한다.", BLUE)
    card(draw, (804, y, 1564, y + 208), "FK-factor rows",
         "A4 · B1 · B2\n"
         "visual reprojection + covariance-whitened robust FK factor, 총 2항이다.", TEAL)
    card(draw, (68, y + 246, 760, y + 452), "없는 것",
         "optimizer 내부에는 camera-to-camera residual, relative-pose 평균, "
         "w1·reprojection + w2·pose_error + w3·FK_constraint 형태가 없다.", RED)
    card(draw, (804, y + 246, 1564, y + 452), "있는 것",
         "EIH와 E2H는 shared target pose와 common hand-eye를 통해 joint optimization에서 "
         "서로 feedback을 준다.", GREEN)
    return img


def slide_methods(ctx: dict, i: int, n: int) -> Image.Image:
    del ctx
    img, draw, y = content_base(
        i, n, "METHOD ROWS", "A0~A5/B1~B3는 세 축의 ablation",
        "Marker set, optimization structure, cube pose handling 중 무엇이 바뀌는지 한 번에 본다.",
    )
    headers = ["Row", "Marker", "Optimization", "Cube pose", "역할"]
    rows = [
        ["A0", "board", "seq", "없음", "baseline"],
        ["A1", "board+cube", "seq", "estimated", "cube 추가"],
        ["A2", "board+cube", "unified", "estimated", "vision-only unified 후보"],
        ["A3", "board+cube", "unified", "raw-FK hard", "FK hard stress"],
        ["A4", "board+cube", "unified", "corrected-FK soft", "FK cov 대기 후보"],
        ["A5", "board+cube", "unified", "aligned-FK hard", "GT 전 frozen 최종 후보"],
        ["B1", "board+cube", "seq", "corrected-FK soft", "A4 대비 -Unified"],
        ["B2", "cube", "unified", "corrected-FK soft", "A4 대비 -board"],
        ["B3", "board", "unified", "없음", "A2 대비 -cube"],
    ]
    table(draw, 86, y, [110, 210, 220, 300, 460], headers, rows,
          row_font_size=20, header_font_size=18, max_bottom=818)
    return img


def slide_part3(ctx: dict, i: int, n: int) -> Image.Image:
    del ctx
    return dark_slide(
        i, n, "PART 3 · 비교실험/평가지표", "한 번에 하나만 바꿔 비교한다",
        "모든 row를 전체 순위로 세우지 않고 matched contrast만 해석한다.",
    )


def slide_comparison_contract(ctx: dict, i: int, n: int) -> Image.Image:
    del ctx
    img, draw, y = content_base(
        i, n, "CONTRAST TABLE", "최종 비교실험 구성",
        "비교 행은 A0~A5, B1~B3 한 벌만 사용하고 heldout 평가는 항상 cube만 본다.",
    )
    headers = ["구분", "직접 비교", "질문", "주 지표"]
    rows = [
        ["Final", "A0 → B3", "board-on-gripper only 순차/통합 차이", "External cube GT + heldout cube"],
        ["Final", "A0 → A1", "cube train 관측 추가 효과", "External cube GT + heldout cube"],
        ["Final", "A1 → A2", "vision-only 통합 feedback 효과", "External cube GT + heldout cube"],
        ["Final", "B3 → A2", "unified에서 cube residual 필요성", "External cube GT + heldout cube"],
        ["Final", "A2 → A3", "vision cube pose 대신 raw FK hard fixed", "External cube GT + heldout cube"],
        ["Final", "B1 → A4", "soft FK에서 순차/통합 차이", "External cube GT + heldout cube"],
        ["Final", "A2 → A4", "soft FK factor 추가 효과", "External cube GT + heldout cube"],
        ["Final", "B2 → A4", "board residual의 cube 보정 기여", "External cube GT + heldout cube"],
        ["Final", "A3/A4 → A5", "aligned FK hard fixed 최종 후보성", "External cube GT + heldout cube"],
    ]
    table(draw, 68, y, [150, 190, 690, 360], headers, rows,
          row_font_size=18, header_font_size=17, max_bottom=820)
    return img


def slide_metric_matrix(ctx: dict, i: int, n: int) -> Image.Image:
    del ctx
    img, draw, y = content_base(
        i, n, "METRICS", "평가지표 최종 판정",
        "같은 숫자라도 어떤 의존성을 갖는지에 따라 주 지표, 보조 지표, 다음주 지표로 나눈다.",
    )
    rows = [
        ["External cube TRE/rot/P95/fail", "최종 주 지표", "Independent External GT 후 최종 물리 순위"],
        ["ALL Cube RMSE px", "보조", "train+heldout cube fit sanity check"],
        ["Train RMSE px", "진단", "수렴/학습 적합도 확인"],
        ["Heldout Cube RMSE px", "보조", "미사용 cube event 재투영"],
        ["Cross-view pixel transfer", "보조", "fixed/gripper camera cube px 일관성"],
        ["Cam-common Obj-Cam mm/deg", "보조", "카메라가 계산한 cube pose 차이"],
    ]
    table(draw, 88, y, [440, 220, 740], ["지표", "등급", "사용법"], rows,
          row_font_size=21, header_font_size=18, max_bottom=794)
    card(draw, (88, 760, 1510, 830), "주의",
         "Board heldout과 pooled overall은 최종 표에서 빼고, heldout 평가는 cube만 사용한다.",
         ORANGE, title_size=22, body_size=21)
    return img


def slide_data_validity(ctx: dict, i: int, n: int) -> Image.Image:
    w = ctx["warnings"]
    support = w["support"]
    img, draw, y = content_base(
        i, n, "DATA CHECK", "데이터 문제는 없는가?",
        "계산/동기화 오류는 보이지 않지만, 해석 한계는 보고서 첫 화면에 드러냈다.",
    )
    small_stat(draw, (68, y, 384, y + 138), "27/27", "모든 row·seed solver 수렴", GREEN)
    small_stat(draw, (420, y, 736, y + 138), "9 sets", "eligible set 4~12", BLUE)
    small_stat(draw, (772, y, 1088, y + 138), "0~3", "insufficient cube events로 탈락", ORANGE)
    small_stat(draw, (1124, y, 1564, y + 138),
               f"{support['overall']['n_observations']} obs",
               "camera-scope evaluation support", BLUE)
    card(draw, (68, y + 178, 760, y + 416), "치명적 검출 실패는 아님",
         "Cube detection: 117 images read, 108 accepted PnP observations, "
         "99 core multiface selected, PnP failure 0, RMSE rejection 2.", GREEN)
    card(draw, (804, y + 178, 1564, y + 416), "남은 데이터 리스크",
         "Board-Cube direct PnP disagreement가 10.8077 mm translation RMSE로 남아 있다. "
         "joint solve는 완화하지만 원인을 제거하지 않는다.", ORANGE)
    return img


def slide_part4(ctx: dict, i: int, n: int) -> Image.Image:
    del ctx
    return dark_slide(
        i, n, "PART 4 · 결과 해석", "숫자가 말하는 것과 말하지 않는 것",
        "현재 내부 cube 지표는 참고값이고 최종 승자는 External cube GT로 정한다.",
    )


def slide_a1_a2(ctx: dict, i: int, n: int) -> Image.Image:
    a1c = metric(ctx, "A1", "heldout_cube_reprojection_rmse_px")
    a2c = metric(ctx, "A2", "heldout_cube_reprojection_rmse_px")
    img, draw, y = content_base(
        i, n, "A1 → A2", "가장 강한 내부 확증 contrast",
        "동일 board+cube 관측에서 sequential frozen-stage를 unified joint optimization으로 바꾼 비교다.",
    )
    bar_chart(draw, (68, y, 1000, y + 444), "Heldout cube reprojection RMSE px",
              [("A1 cube", a1c, GRAY_BAR), ("A2 cube", a2c, TEAL)],
              max_value=4.6,
              note="낮을수록 좋다. 최종 heldout 평가는 cube만 사용한다.")
    card(draw, (1040, y, 1564, y + 202), "결론",
         f"Heldout cube {fmt(a1c)} → {fmt(a2c)} px, {pct_drop(a1c, a2c)} 감소.",
         GREEN)
    card(draw, (1040, y + 242, 1564, y + 444), "말할 문장",
         "“동일 관측·초기값·solver에서 EIH/E2H unified feedback이 내부 held-out reprojection을 낮췄습니다.”",
         BLUE)
    return img


def slide_a2_a3(ctx: dict, i: int, n: int) -> Image.Image:
    a2c = metric(ctx, "A2", "heldout_cube_reprojection_rmse_px")
    a3c = metric(ctx, "A3", "heldout_cube_reprojection_rmse_px")
    img, draw, y = content_base(
        i, n, "A2 → A3", "raw FK hard fixed는 왜 위험한가",
        "A3는 cube pose를 raw controller FK + mechanical frame map으로 상수 고정한다.",
    )
    bar_chart(draw, (68, y, 1000, y + 444), "Heldout cube reprojection RMSE px",
              [("A2 cube", a2c, TEAL), ("A3 cube", a3c, ORANGE)],
              max_value=7.0,
              note="A3는 cube held-out가 3.5958 → 6.3959 px로 크게 악화된다.")
    card(draw, (1040, y, 1564, y + 202), "의미",
         "raw FK를 GT처럼 못 박으면 tool4/CAD frame 정의 오차가 calibration 결과에 흡수된다.",
         ORANGE)
    card(draw, (1040, y + 242, 1564, y + 444), "말할 문장",
         "“이 결과는 raw FK를 독립 정답으로 쓰면 안 된다는 교수님 지적을 수치로 확인한 것입니다.”",
         BLUE)
    return img


def slide_a2_a4_a5(ctx: dict, i: int, n: int) -> Image.Image:
    vals = {
        "A2 cube": metric(ctx, "A2", "heldout_cube_reprojection_rmse_px"),
        "A4 cube": metric(ctx, "A4", "heldout_cube_reprojection_rmse_px"),
        "A5 cube": metric(ctx, "A5", "heldout_cube_reprojection_rmse_px"),
    }
    img, draw, y = content_base(
        i, n, "A2 / A4 / A5", "FK 사용 후보를 어떻게 최종 판정할 것인가",
        "현재 내부 cube 값은 참고하고, 최종 우열은 External cube GT로 정한다.",
    )
    bar_chart(draw, (68, y, 1020, y + 444), "Heldout cube reprojection RMSE px",
              [("A2 cube", vals["A2 cube"], BLUE),
               ("A4 cube", vals["A4 cube"], TEAL),
               ("A5 cube", vals["A5 cube"], PURPLE)],
              max_value=4.4,
              note="A5는 GT 공개 전에 frozen하면 최종 후보로 비교 가능하다.")
    card(draw, (1060, y, 1564, y + 130), "A2", "Vision-only unified 후보.", BLUE)
    card(draw, (1060, y + 154, 1564, y + 286), "A4", "Corrected-FK soft factor 후보.", TEAL)
    card(draw, (1060, y + 310, 1564, y + 444), "A5", "Vision-aligned FK hard fixed 후보.", PURPLE)
    return img


def slide_cube_metric_snapshot(ctx: dict, i: int, n: int) -> Image.Image:
    by = ctx["by"]
    selected = []
    for method in ("A1", "A2", "A3", "A4", "A5", "B1", "B2"):
        selected.append([
            method,
            fmt(by[method]["heldout_cube_reprojection_rmse_px"]),
            fmt(by[method]["cross_view_cube_pixel_transfer_rmse_px"]),
            f"{fmt(by[method]['cam_common_cube_translation_rmse_mm'])} / "
            f"{fmt(by[method]['cam_common_cube_rotation_rmse_deg'])}",
            "Pending",
        ])
    img, draw, y = content_base(
        i, n, "CUBE METRICS", "최종용 cube-only 지표 snapshot",
        "현재 내부 cube 지표는 보조 근거이며, 최종 주 지표는 External cube GT다.",
    )
    table(draw, 80, y, [160, 260, 310, 310, 180],
          ["Method", "Heldout cube px", "Cross-view px", "Cam-common mm/deg", "External GT"],
          selected, row_font_size=18, header_font_size=16, max_bottom=724)
    card(draw, (80, 740, 1518, 830), "해석",
         "표의 px 값이 좋아도 공통 systematic error를 잡지 못한다. 최종 채택은 External cube GT로 결정한다.",
         ORANGE, title_size=22, body_size=21)
    return img


def slide_camera_scope(ctx: dict, i: int, n: int) -> Image.Image:
    by = ctx["by"]
    rows = [
        ["A2", fmt(by["A2"]["cross_view_cube_pixel_transfer_rmse_px"]),
         f"{fmt(by['A2']['cam_common_cube_translation_rmse_mm'])} / {fmt(by['A2']['cam_common_cube_rotation_rmse_deg'])}"],
        ["A4", fmt(by["A4"]["cross_view_cube_pixel_transfer_rmse_px"]),
         f"{fmt(by['A4']['cam_common_cube_translation_rmse_mm'])} / {fmt(by['A4']['cam_common_cube_rotation_rmse_deg'])}"],
        ["A5", fmt(by["A5"]["cross_view_cube_pixel_transfer_rmse_px"]),
         f"{fmt(by['A5']['cam_common_cube_translation_rmse_mm'])} / {fmt(by['A5']['cam_common_cube_rotation_rmse_deg'])}"],
        ["OpenCV cube", "2.8820", "3.5148 / N/A"],
    ]
    img, draw, y = content_base(
        i, n, "CAMERA SCOPE", "Cube-only camera consistency",
        "카메라 간 일관성은 하나의 보조 지표 묶음으로만 보고 최종 순위는 External cube GT로 정한다.",
    )
    table(draw, 96, y, [220, 470, 470],
          ["방법", "Cross-view cube px", "Cam-common cube mm/deg"],
          rows, row_font_size=24, header_font_size=18, max_bottom=520)
    card(draw, (96, y + 306, 760, y + 520), "Cross-view pixel transfer",
         "한 카메라 PnP pose를 다른 카메라로 전달해 cube corner px 오차를 본다.",
         BLUE)
    card(draw, (812, y + 306, 1510, y + 520), "Cam-common Obj-Cam",
         "fixed/gripper camera가 계산한 cube pose 차이를 mm/deg로 보되, 외부 GT 순위용은 아니다.",
         TEAL)
    return img


def slide_opencv(ctx: dict, i: int, n: int) -> Image.Image:
    summary = ctx["opencv"].get("summary", [])
    opencv_rows = []
    for row in summary:
        opencv_rows.append([
            row["baseline"].replace("opencv_", "OpenCV "),
            fmt(row["board_cross_view_pixel_transfer_rmse_px"]),
            fmt(row["cube_cross_view_pixel_transfer_rmse_px"]),
            f"{row['n_train_relative_inliers']}/{row['n_train_relative_candidates']}",
        ])
    img, draw, y = content_base(
        i, n, "REFERENCE", "OpenCV baseline은 구현했다",
        "교수님 지적: 같은 사진을 공개 구현에도 넣어 데이터 문제인지 코드 문제인지 확인하라.",
    )
    table(draw, 92, y, [300, 260, 260, 260],
          ["Baseline", "Board px", "Cube px", "Train inliers"],
          opencv_rows, row_font_size=23, header_font_size=18, max_bottom=505)
    card(draw, (92, y + 300, 1510, y + 500), "해석",
         "OpenCV direct relative-pose baseline은 main-method transform, Robot FK, Hand-Eye, shared target pose를 쓰지 않는다. "
         "다만 이것도 external physical GT는 아니므로 SOTA 절대 정확도 비교로 말하지 않는다.",
         ORANGE)
    return img


def slide_part5(ctx: dict, i: int, n: int) -> Image.Image:
    del ctx
    return dark_slide(
        i, n, "PART 5 · 예상 질문 답변", "교수님이 물으면 이렇게 답한다",
        "방어의 핵심은 인정할 것과 주장할 것을 분리하는 것이다.",
    )


def slide_q_data(ctx: dict, i: int, n: int) -> Image.Image:
    del ctx
    img, draw, y = content_base(
        i, n, "ANSWER CARD", "Q. 데이터적인 문제는 없는가?",
        "짧게는 “계산 오류는 안 보이지만 데이터 리스크는 드러냈다”라고 답한다.",
    )
    card(draw, (68, y, 760, y + 210), "먼저 말할 것",
         "“동기화와 계산 계약은 검증했습니다. JSON/CSV/Markdown/HTML이 같은 값을 보고하고, 모든 row는 3개 seed에서 수렴했습니다.”",
         GREEN)
    card(draw, (804, y, 1564, y + 210), "바로 이어 말할 것",
         "“다만 데이터 한계는 있습니다. dropped set 0~3, n=9 held-out sets, support imbalance, 10.8077 mm board-cube disagreement를 보고서 첫 화면에 올렸습니다.”",
         ORANGE)
    card(draw, (68, y + 254, 1564, y + 446), "한 줄 결론",
         "“그래서 데이터가 깨졌다고 보기는 어렵지만, 이 데이터만으로 절대 물리 정확도까지 확정하는 것은 과합니다.”",
         BLUE, title_size=27, body_size=29)
    return img


def slide_q_confidence(ctx: dict, i: int, n: int) -> Image.Image:
    del ctx
    img, draw, y = content_base(
        i, n, "ANSWER CARD", "Q. 현재 결론은 확실한가?",
        "답은 “내부 결론은 확실, 물리 순위는 아직”이다.",
    )
    card(draw, (68, y, 520, y + 316), "확실한 것",
         "최종 비교표는 A0~A5/B1~B3 한 벌로 고정했고 heldout은 cube만 본다. "
         "A1→A2에서 cube heldout이 개선된다.", GREEN)
    card(draw, (574, y, 1026, y + 316), "조심할 것",
         "현재 Session04 내부 px는 보조 근거다. "
         "External GT가 들어오기 전에는 물리 우월성처럼 말하지 않는다.", ORANGE)
    card(draw, (1080, y, 1564, y + 316), "금지할 것",
         "A2/A4/A5 중 누가 robot base에서 최종적으로 제일 정확한지는 다음주 External GT 전에는 말하지 않는다.",
         RED)
    card(draw, (68, y + 356, 1564, y + 456), "발표 문장",
         "“현재 결론의 강도는 내부 ablation 결론입니다. 최종 물리 정확도 결론은 다음주 독립 외부 GT로 분리했습니다.”",
         BLUE, title_size=24, body_size=25)
    return img


def slide_part6(ctx: dict, i: int, n: int) -> Image.Image:
    del ctx
    return dark_slide(
        i, n, "PART 6 · 다음주 예정", "External GT로 무엇을 마무리할 것인가",
        "현재 내부 결론을 유지하고, robot-base 물리 정확도는 독립 GT로 별도 산출한다.",
    )


def slide_next_week(ctx: dict, i: int, n: int) -> Image.Image:
    del ctx
    img, draw, y = content_base(
        i, n, "NEXT WEEK", "Independent External GT 태스크",
        "다음주에 해야 할 일은 현재 표를 뒤집는 것이 아니라, 현재 결론의 물리 정확도 범위를 닫는 것이다.",
    )
    steps = [
        ["1", "GT 수집 전 고정", "측정 pose ID, strata, 비교 contrast, failure 기준을 먼저 등록"],
        ["2", "Blind prediction", "A2/A3/A4/A5 예측을 GT 공개 전에 해시로 고정"],
        ["3", "GT 측정", "RGB calibration camera, controller FK, A4/A5 artifact와 독립인 측정계 사용"],
        ["4", "채점", "Translation Error, Rotation Error, P95, Failure Rate, ADD-S"],
    ]
    x0, y0 = 92, y
    for idx, (num, title, body) in enumerate(steps):
        yy = y0 + idx * 126
        pill(draw, (x0, yy + 8), num, BLUE if idx < 2 else TEAL, size=21)
        draw.text((x0 + 72, yy + 4), title, font=font(27, "bold"), fill=INK)
        draw_wrapped(draw, (x0 + 72, yy + 43), body, font(22), MUTED, 1280, gap=7)
        if idx < len(steps) - 1:
            draw.line((x0 + 24, yy + 62, x0 + 24, yy + 128), fill=LINE, width=4)
    return img


def slide_final_script(ctx: dict, i: int, n: int) -> Image.Image:
    a1 = metric(ctx, "A1", "heldout_cube_reprojection_rmse_px")
    a2 = metric(ctx, "A2", "heldout_cube_reprojection_rmse_px")
    img, draw, y = content_base(
        i, n, "SCRIPT", "30초 발표 스크립트",
        "마지막에 이 문단만 말해도 전체 프레임이 잡힌다.",
    )
    text = (
        f"“8/3 피드백 이후 결과표를 A0~A5/B1~B3 한 벌의 final ablation 표로 고정했습니다. "
        f"heldout 평가는 항상 cube만 사용하고, 최종 주 지표는 External cube GT입니다. "
        f"현재 Session04 내부 cube heldout에서는 A1이 {fmt(a1)} px, A2가 {fmt(a2)} px로 낮아져 unified feedback의 내부 효과가 보입니다. "
        f"A4와 A5도 External GT 공개 전에 방법과 artifact를 frozen하면 최종 후보로 같이 비교합니다. "
        f"따라서 실제 robot-base 물리 정확도는 다음주 Independent External GT에서 "
        f"Translation/Rotation/P95/Failure Rate로 따로 확정하겠습니다.”"
    )
    rounded(draw, (110, y + 28, 1490, y + 422), fill="#EEF5FF", outline="#BED8FF",
            radius=18)
    draw_wrapped(draw, (150, y + 70), text, font(30, "medium"), INK, 1300, gap=13)
    return img


SLIDES: list[SlideFn] = [
    slide_cover,
    slide_reading,
    slide_meeting_status,
    slide_part1,
    slide_claim_envelope,
    slide_part2,
    slide_pipeline,
    slide_objective,
    slide_methods,
    slide_part3,
    slide_comparison_contract,
    slide_metric_matrix,
    slide_data_validity,
    slide_part4,
    slide_a1_a2,
    slide_a2_a3,
    slide_a2_a4_a5,
    slide_cube_metric_snapshot,
    slide_camera_scope,
    slide_opencv,
    slide_part5,
    slide_q_data,
    slide_q_confidence,
    slide_part6,
    slide_next_week,
    slide_final_script,
]


def render_deck(out_pdf: Path, png_dir: Path | None = None) -> list[Image.Image]:
    ctx = load_context()
    total = len(SLIDES)
    images = [slide(ctx, idx + 1, total) for idx, slide in enumerate(SLIDES)]
    out_pdf.parent.mkdir(parents=True, exist_ok=True)
    images[0].save(
        out_pdf,
        save_all=True,
        append_images=images[1:],
        resolution=120.0,
        quality=95,
    )
    if png_dir:
        png_dir.mkdir(parents=True, exist_ok=True)
        for idx, image in enumerate(images, start=1):
            image.save(png_dir / f"slide_{idx:02d}.png")
    return images


def main(argv: Iterable[str] | None = None) -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--out", type=Path, default=OUT_PDF)
    parser.add_argument("--png-dir", type=Path)
    args = parser.parse_args(list(argv) if argv is not None else None)
    images = render_deck(args.out, args.png_dir)
    print(f"[DONE] wrote {args.out} ({len(images)} slides)")


if __name__ == "__main__":
    main()
