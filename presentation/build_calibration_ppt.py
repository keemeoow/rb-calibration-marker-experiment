#!/usr/bin/env python3
"""Build a text-and-equation-first calibration presentation.

All mathematical expressions are rendered through Matplotlib mathtext and
inserted as high-resolution equation assets. Decorative camera/cube drawings
are intentionally omitted so the deck remains a readable teaching document.
"""
from pathlib import Path
import sys
from dataclasses import dataclass

import matplotlib.pyplot as plt
from PIL import Image
from pptx import Presentation
from pptx.dml.color import RGBColor
from pptx.enum.shapes import MSO_AUTO_SHAPE_TYPE
from pptx.enum.text import MSO_ANCHOR, PP_ALIGN
from pptx.util import Inches, Pt

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from figure_style import apply_paper_style, save_figure


HERE = Path(__file__).resolve().parent
OUT = HERE / "multi_camera_calibration_FK_methods.pptx"
EQ_DIR = HERE / "equations"

SLIDE_W, SLIDE_H = Inches(13.333), Inches(7.5)
PAPER = "FFFFFF"
INK = "17212B"
MUTED = "53606B"
GRID = "DCE2E7"
BORDER = "B8C0C7"
BLUE = "2374AB"
ORANGE = "D17A22"
GREEN = "2F7D67"
GRAY = "5B6B7A"
RED = "C46A4A"
PURPLE = "8064A2"
LIGHT_BLUE = "EAF3F8"
LIGHT_ORANGE = "FBF0E5"
LIGHT_GREEN = "EAF4F0"
LIGHT_GRAY = "F3F5F6"
FONT = "NanumGothic"


@dataclass(frozen=True)
class MathCell:
    """A table cell whose complete contents must be rendered as LaTeX."""

    name: str
    latex: str
    fontsize: int = 20
    color: str = INK


def math_cell(name, latex, fontsize=20, color=INK):
    return MathCell(name, latex, fontsize, color)


def rgb(value):
    return RGBColor.from_string(value)


def blank_slide(prs):
    slide = prs.slides.add_slide(prs.slide_layouts[6])
    slide.background.fill.solid()
    slide.background.fill.fore_color.rgb = rgb(PAPER)
    return slide


def add_text(slide, text, x, y, w, h, size=18, color=INK, bold=False,
             align=PP_ALIGN.LEFT, valign=MSO_ANCHOR.TOP, margin=0.05):
    box = slide.shapes.add_textbox(Inches(x), Inches(y), Inches(w), Inches(h))
    tf = box.text_frame
    tf.clear()
    tf.word_wrap = True
    tf.margin_left = tf.margin_right = Inches(margin)
    tf.margin_top = tf.margin_bottom = Inches(margin)
    tf.vertical_anchor = valign
    p = tf.paragraphs[0]
    p.alignment = align
    p.space_after = Pt(0)
    run = p.add_run()
    run.text = text
    run.font.name = FONT
    run.font.size = Pt(size)
    run.font.bold = bold
    run.font.color.rgb = rgb(color)
    return box


def add_bullets(slide, items, x, y, w, h, size=17, color=INK, spacing=10):
    box = slide.shapes.add_textbox(Inches(x), Inches(y), Inches(w), Inches(h))
    tf = box.text_frame
    tf.clear()
    tf.word_wrap = True
    tf.margin_left = Inches(0.05)
    tf.margin_right = Inches(0.03)
    for index, item in enumerate(items):
        p = tf.paragraphs[0] if index == 0 else tf.add_paragraph()
        p.text = "•  " + item
        p.font.name = FONT
        p.font.size = Pt(size)
        p.font.color.rgb = rgb(color)
        p.space_after = Pt(spacing)
    return box


def add_box(slide, x, y, w, h, fill=PAPER, line=BORDER, rounded=True):
    shape_type = (MSO_AUTO_SHAPE_TYPE.ROUNDED_RECTANGLE if rounded
                  else MSO_AUTO_SHAPE_TYPE.RECTANGLE)
    shape = slide.shapes.add_shape(shape_type, Inches(x), Inches(y), Inches(w), Inches(h))
    shape.fill.solid()
    shape.fill.fore_color.rgb = rgb(fill)
    shape.line.color.rgb = rgb(line)
    shape.line.width = Pt(1.0)
    return shape


def add_title(slide, title, kicker):
    add_text(slide, kicker.upper(), 0.75, 0.3, 4.5, 0.25, 9, BLUE, True)
    add_text(slide, title, 0.75, 0.68, 11.8, 0.62, 27, INK, True)
    mark = slide.shapes.add_shape(MSO_AUTO_SHAPE_TYPE.RECTANGLE,
                                  Inches(0.75), Inches(1.33), Inches(0.62), Inches(0.045))
    mark.fill.solid(); mark.fill.fore_color.rgb = rgb(BLUE); mark.line.fill.background()


def add_footer(slide, number):
    add_text(slide, "MULTI-CAMERA CALIBRATION", 0.75, 7.1, 4.0, 0.2, 8, MUTED, True)
    add_text(slide, f"{number:02d}", 12.0, 7.05, 0.55, 0.22, 9, MUTED, True,
             PP_ALIGN.RIGHT)


def section_label(slide, text, x, y, w, color):
    add_box(slide, x, y, w, 0.42, color, color, False)
    add_text(slide, text, x + 0.08, y + 0.06, w - 0.16, 0.25, 12, PAPER, True,
             PP_ALIGN.CENTER)


def add_table(slide, rows, col_widths, x, y, row_h=0.55, header=True,
              colors=None, font_size=13):
    colors = colors or [GRAY] * len(col_widths)
    yy = y
    for ri, row in enumerate(rows):
        xx = x
        for ci, (cell, cw) in enumerate(zip(row, col_widths)):
            fill = LIGHT_GRAY if ri == 0 and header else PAPER
            add_box(slide, xx, yy, cw, row_h, fill, GRID, False)
            if isinstance(cell, MathCell):
                add_math_label(slide, f"table_{cell.name}", cell.latex,
                               xx + 0.06, yy + 0.04, cw - 0.12, row_h - 0.08,
                               cell.fontsize, cell.color)
            else:
                add_text(slide, str(cell), xx + 0.06, yy + 0.07, cw - 0.12, row_h - 0.1,
                         font_size, colors[ci] if ri == 0 else INK,
                         ri == 0 and header, PP_ALIGN.CENTER, MSO_ANCHOR.MIDDLE)
            xx += cw
        yy += row_h


def equation_asset(name, latex, fontsize=30, color=INK, width=12.0, height=1.0):
    """Render a LaTeX-style equation using the repository figure convention."""
    EQ_DIR.mkdir(parents=True, exist_ok=True)
    path = EQ_DIR / f"{name}.png"
    apply_paper_style()
    # Keep math glyph selection deterministic across PowerPoint build hosts.
    plt.rcParams["font.family"] = ["DejaVu Sans"]
    fig = plt.figure(figsize=(width, height), facecolor=f"#{PAPER}")
    fig.text(0.5, 0.5, f"${latex}$", ha="center", va="center",
             fontsize=fontsize, color=f"#{color}")
    save_figure(fig, path, dpi=220, close=True)
    return path


def add_equation(slide, name, latex, x, y, w, h, fontsize=30,
                 color=INK, fill=PAPER, line=BORDER):
    add_box(slide, x, y, w, h, fill, line)
    asset = equation_asset(name, latex, fontsize, color, max(w, 3), max(h * 0.75, 0.55))
    with Image.open(asset) as image:
        aspect = image.width / image.height
    max_w, max_h = w - 0.24, h - 0.18
    pic_w = min(max_w, max_h * aspect)
    pic_h = pic_w / aspect
    left = x + (w - pic_w) / 2
    top = y + (h - pic_h) / 2
    slide.shapes.add_picture(str(asset), Inches(left), Inches(top),
                             width=Inches(pic_w), height=Inches(pic_h))


def add_math_label(slide, name, latex, x, y, w, h, fontsize=22, color=INK):
    """Insert a borderless LaTeX-rendered label for small/inline notation."""
    asset = equation_asset(name, latex, fontsize, color, max(w, 1.2), max(h, 0.28))
    with Image.open(asset) as image:
        aspect = image.width / image.height
    pic_w = min(w, h * aspect)
    pic_h = pic_w / aspect
    left = x + (w - pic_w) / 2
    top = y + (h - pic_h) / 2
    slide.shapes.add_picture(str(asset), Inches(left), Inches(top),
                             width=Inches(pic_w), height=Inches(pic_h))


def add_math_description(slide, name, latex, description, x, y, formula_w,
                         total_w, h=0.46, fontsize=20, text_size=16,
                         color=INK, bullet=True):
    """Place a LaTeX expression beside ordinary-language explanation."""
    if bullet:
        add_text(slide, "•", x, y + 0.04, 0.25, h, text_size, color, True)
        x += 0.27
        total_w -= 0.27
    add_math_label(slide, name, latex, x, y, formula_w, h, fontsize, color)
    add_text(slide, description, x + formula_w + 0.08, y + 0.04,
             total_w - formula_w - 0.08, h - 0.04, text_size, color)


def add_method_card(slide, x, title, description, formula, color):
    add_box(slide, x, 1.8, 3.75, 4.55, PAPER, color)
    bar = slide.shapes.add_shape(MSO_AUTO_SHAPE_TYPE.RECTANGLE,
                                 Inches(x), Inches(1.8), Inches(3.75), Inches(0.07))
    bar.fill.solid(); bar.fill.fore_color.rgb = rgb(color); bar.line.fill.background()
    add_text(slide, title, x + 0.25, 2.1, 3.2, 0.38, 19, color, True)
    add_text(slide, description, x + 0.25, 2.72, 3.2, 1.85, 15, INK)
    add_equation(slide, f"card_{title.lower().replace('-', '_')}", formula,
                 x + 0.3, 5.15, 3.15, 0.72, 23, color, PAPER, color)


def build():
    prs = Presentation()
    prs.slide_width, prs.slide_height = SLIDE_W, SLIDE_H

    # 1. Title
    slide = blank_slide(prs)
    add_text(slide, "MULTI-CAMERA CALIBRATION", 0.8, 0.72, 5.2, 0.3, 10, BLUE, True)
    add_text(slide, "여러 카메라를 하나의 좌표계로", 0.8, 1.55, 11.4, 0.68, 31, INK, True)
    add_text(slide, "No-FK · Fixed-FK · Corrected-FK",
             0.82, 2.48, 10.5, 0.46, 20, BLUE, True)
    add_text(slide, "쉬운 개념에서 시작해 목적함수와 보정 알고리즘까지",
             0.82, 3.12, 10.5, 0.4, 17, MUTED)
    add_box(slide, 0.82, 4.32, 11.3, 1.25, LIGHT_GRAY, GRID)
    add_text(slide, "발표의 질문", 1.12, 4.58, 1.6, 0.3, 13, MUTED, True)
    add_text(slide, "같은 큐브를 본 여러 카메라의 좌표를 어떻게 하나로 합칠까?",
             2.65, 4.5, 8.9, 0.5, 20, INK, True)
    add_text(slide, "그리고 로봇 FK 정보는 사용하지 않을까, 그대로 믿을까, 보정해서 사용할까?",
             2.65, 5.05, 8.9, 0.4, 15, MUTED)
    add_footer(slide, 1)

    # 2. Roadmap
    slide = blank_slide(prs); add_title(slide, "쉬운 이야기에서 상세 수식까지", "00 · Roadmap")
    stages = [
        ("1", "문제", "왜 카메라마다 답이 다른가?", BLUE),
        ("2", "언어", "좌표계와 변환행렬", GRAY),
        ("3", "원리", "같은 큐브 자세로 합의", GREEN),
        ("4", "방법", "No / Fixed / Corrected-FK", ORANGE),
        ("5", "상세", "목적함수 · gate · blend", PURPLE),
    ]
    y = 1.75
    for num, title, body, color in stages:
        add_box(slide, 1.25, y, 10.85, 0.78, PAPER, color)
        add_text(slide, num, 1.52, y + 0.18, 0.5, 0.3, 16, color, True, PP_ALIGN.CENTER)
        add_text(slide, title, 2.25, y + 0.15, 1.4, 0.32, 16, color, True)
        add_text(slide, body, 3.75, y + 0.15, 7.8, 0.35, 16, INK)
        y += 0.94
    add_footer(slide, 2)

    # 3. Problem
    slide = blank_slide(prs); add_title(slide, "같은 큐브인데 카메라마다 위치가 다르다", "01 · Why")
    add_bullets(slide, [
        "카메라 0: ‘큐브가 내 앞 50 cm에 있다.’",
        "카메라 1: ‘큐브가 내 오른쪽 30 cm에 있다.’",
        "그리퍼 카메라: ‘큐브가 내 아래 20 cm에 있다.’",
    ], 1.1, 1.75, 5.25, 2.5, 19, INK, 18)
    add_box(slide, 6.8, 1.75, 5.25, 2.5, LIGHT_BLUE, BLUE)
    add_text(slide, "왜 모두 다른가?", 7.15, 2.05, 4.55, 0.4, 21, BLUE, True)
    add_text(slide, "각 카메라가 자기 자신을 원점으로 사용하기 때문이다.",
             7.15, 2.82, 4.55, 0.95, 20, INK, True, PP_ALIGN.CENTER, MSO_ANCHOR.MIDDLE)
    add_box(slide, 1.4, 4.75, 10.5, 1.15, LIGHT_GREEN, GREEN)
    add_text(slide, "필요한 것", 1.75, 5.08, 1.4, 0.3, 14, GREEN, True)
    add_text(slide, "각 카메라의 좌표를 하나의 공통 좌표로 바꾸는 방법",
             3.1, 5.0, 8.1, 0.45, 21, INK, True)
    add_footer(slide, 3)

    # 4. Definition
    slide = blank_slide(prs); add_title(slide, "캘리브레이션의 가장 쉬운 정의", "01 · Definition")
    add_box(slide, 1.1, 1.75, 11.1, 1.1, LIGHT_BLUE, BLUE)
    add_text(slide, "Calibration = 서로 다른 좌표계 사이의 관계를 찾는 과정",
             1.45, 2.02, 10.4, 0.5, 23, BLUE, True, PP_ALIGN.CENTER)
    add_table(slide, [
        ["구분", "질문", "이 프로젝트에서"],
        ["내부 Intrinsic", "카메라 눈의 특성은?", "먼저 구해 놓고 사용"],
        ["외부 Extrinsic", "고정 카메라가 어디에 있나?", "찾아야 함"],
        ["Hand–Eye", "카메라가 그리퍼에 어떻게 붙었나?", "찾아야 함"],
    ], [2.2, 4.6, 3.5], 1.5, 3.45, 0.62, True,
        [GRAY, BLUE, GREEN], 14)
    add_text(slide, "최종 공통 언어는 로봇 베이스 좌표계",
             2.4, 6.23, 8.5, 0.38, 18, INK, True, PP_ALIGN.CENTER)
    add_footer(slide, 4)

    # 5. Coordinate frames
    slide = blank_slide(prs); add_title(slide, "좌표계 기호부터 정리한다", "02 · Coordinates")
    add_table(slide, [
        ["기호", "좌표계", "역할"],
        [math_cell("frame_b", r"\mathbf{B}"), "Robot base", "모든 결과를 표현할 최종 기준"],
        [math_cell("frame_ci", r"\mathbf{C}_{i}"), "Fixed camera i", "i번째 고정 카메라 좌표"],
        [math_cell("frame_g", r"\mathbf{G}"), "Robot gripper", "움직이는 그리퍼 좌표"],
        [math_cell("frame_cg", r"\mathbf{C}_{g}"), "Gripper camera", "그리퍼에 부착된 카메라 좌표"],
        [math_cell("frame_o", r"\mathbf{O}"), "Object / cube", "공통으로 관측하는 캘리브레이션 타깃"],
    ], [1.5, 3.2, 6.15], 1.25, 1.65, 0.68, True,
        [BLUE, GRAY, GREEN], 15)
    add_box(slide, 2.0, 6.05, 9.35, 0.65, LIGHT_GRAY, GRID)
    add_text(slide, "위 첨자 = 도착 좌표계   ·   아래 첨자 = 출발 좌표계",
             2.3, 6.22, 8.75, 0.3, 16, INK, True, PP_ALIGN.CENTER)
    add_footer(slide, 5)

    # 6. Transform
    slide = blank_slide(prs); add_title(slide, "변환행렬은 ‘돌리고 옮기는 좌표 변환기’", "02 · Transform")
    add_equation(slide, "notation", r"{}^{A}\mathbf{T}_{B}:\;B\text{ coordinates}\rightarrow A\text{ coordinates}",
                 1.05, 1.65, 11.25, 0.75, 25, BLUE, LIGHT_BLUE, BLUE)
    add_equation(slide, "transform_matrix", r"\mathbf{T}=\left[\,\mathbf{R}\;\;\mathbf{t}\;;\;\mathbf{0}^{\mathsf{T}}\;\;1\,\right]",
                 1.2, 2.9, 4.0, 2.15, 34, INK, PAPER, BORDER)
    add_math_description(slide, "rotation_space", r"\mathbf{R}\in\mathrm{SO}(3)",
                         "어느 방향을 보는지 나타내는 3×3 회전행렬",
                         5.75, 2.95, 1.55, 5.95, 0.48, 20, 15)
    add_math_description(slide, "translation_space", r"\mathbf{t}\in\mathbb{R}^{3}",
                         "어디에 있는지 나타내는 3×1 이동벡터",
                         5.75, 3.72, 1.55, 5.95, 0.48, 20, 15)
    add_text(slide, "•  행렬곱: 이동 경로를 차례로 연결",
             5.75, 4.49, 5.95, 0.4, 15, INK)
    add_equation(slide, "transform_product", r"\mathbf{T}_{1}\mathbf{T}_{2}=\left[\,\mathbf{R}_{1}\mathbf{R}_{2}\;\;\mathbf{R}_{1}\mathbf{t}_{2}+\mathbf{t}_{1}\;;\;\mathbf{0}^{\mathsf{T}}\;\;1\,\right]",
                 2.2, 5.55, 8.95, 0.9, 27, INK, LIGHT_GRAY, BORDER)
    add_footer(slide, 6)

    # 7. Fixed chain
    slide = blank_slide(prs); add_title(slide, "고정 카메라가 베이스 기준 큐브를 계산하는 경로", "03 · Fixed camera")
    add_text(slide, "베이스 → 고정 카메라 → 큐브", 1.15, 1.75, 11.0, 0.45,
             22, BLUE, True, PP_ALIGN.CENTER)
    add_equation(slide, "fixed_chain", r"{}^{B}\mathbf{T}_{C_i}\;{}^{C_i}\mathbf{T}_{O,s}={}^B\mathbf{T}_{O,s}",
                 1.25, 2.55, 10.85, 1.2, 38, BLUE, LIGHT_BLUE, BLUE)
    add_table(slide, [
        ["항", "뜻", "상태"],
        [math_cell("fixed_term_c", r"{}^{B}\mathbf{T}_{C_i}", 19), "베이스 → 고정 카메라", "찾아야 함"],
        [math_cell("fixed_term_z", r"{}^{C_i}\mathbf{T}_{O,s}", 19), "카메라가 PnP로 관측한 큐브", "측정값"],
        [math_cell("fixed_term_o", r"{}^{B}\mathbf{T}_{O,s}", 19), "베이스 기준 큐브 자세", "FK 방식에 따라 다름"],
    ], [2.2, 5.4, 3.2], 1.4, 4.45, 0.6, True,
        [BLUE, GRAY, GREEN], 14)
    add_footer(slide, 7)

    # 8. Gripper chain
    slide = blank_slide(prs); add_title(slide, "그리퍼 카메라는 움직이므로 경로가 하나 더 길다", "03 · Gripper camera")
    add_text(slide, "베이스 → 그리퍼 → 그리퍼 카메라 → 큐브",
             1.15, 1.75, 11.0, 0.45, 22, ORANGE, True, PP_ALIGN.CENTER)
    add_equation(slide, "gripper_chain", r"{}^{B}\mathbf{T}_{G,e}\;{}^{G}\mathbf{T}_{C_g}\;{}^{C_g}\mathbf{T}_{O,e}={}^B\mathbf{T}_{O,s(e)}",
                 0.9, 2.55, 11.55, 1.2, 34, ORANGE, LIGHT_ORANGE, ORANGE)
    add_table(slide, [
        ["항", "뜻", "상태"],
        [math_cell("gripper_term_g", r"\mathbf{G}_{e}={}^B\mathbf{T}_{G,e}", 17), "촬영 순간 베이스 → 그리퍼", "로봇 FK로 알고 있음"],
        [math_cell("gripper_term_x", r"\mathbf{X}={}^G\mathbf{T}_{C_g}", 17), "그리퍼 → 그리퍼 카메라", "찾아야 함"],
        [math_cell("gripper_term_z", r"\mathbf{Z}_{g,e}={} ^{C_g}\mathbf{T}_{O,e}", 17), "그리퍼 카메라의 큐브 관측", "측정값"],
    ], [2.5, 5.0, 3.3], 1.25, 4.45, 0.6, True,
        [ORANGE, GRAY, GREEN], 14)
    add_footer(slide, 8)

    # 9. Known / unknown
    slide = blank_slide(prs); add_title(slide, "알고 있는 값과 찾아야 하는 값", "03 · Known / Unknown")
    section_label(slide, "알고 있는 값 · 관측", 0.95, 1.65, 5.45, GREEN)
    add_bullets(slide, [
        "Zᵢ,ₛ = ⁽Cᵢ⁾T_O,s · 고정 카메라 PnP",
        "Z𝗀,ₑ = ⁽C𝗀⁾T_O,e · 그리퍼 카메라 PnP",
        "Gₑ = ⁽ᴮ⁾T_G,e · 촬영 순간 로봇 자세",
        "Fₛ = ⁽ᴮ⁾T_O,s^FK · raw FK 큐브 prior",
    ], 1.05, 2.35, 5.25, 3.2, 16, INK, 15)
    section_label(slide, "찾아야 하는 값 · 미지수", 6.9, 1.65, 5.45, BLUE)
    add_bullets(slide, [
        "Cᵢ = ⁽ᴮ⁾T_Cᵢ · 고정 카메라 외부파라미터",
        "X = ⁽ᴳ⁾T_C𝗀 · hand–eye 변환",
        "Oₛ = ⁽ᴮ⁾T_O,s · set별 큐브 자세",
        "Oₛ를 어떻게 다루는지가 세 FK 방식의 차이",
    ], 7.0, 2.35, 5.15, 3.2, 16, INK, 15)
    add_box(slide, 2.0, 5.75, 9.35, 0.72, LIGHT_BLUE, BLUE)
    add_text(slide, "No-FK여도 Gₑ는 사용한다. 사용하지 않는 것은 큐브 prior Fₛ이다.",
             2.2, 5.94, 8.95, 0.3, 16, BLUE, True, PP_ALIGN.CENTER)
    add_footer(slide, 9)

    # 10. Agreement
    slide = blank_slide(prs); add_title(slide, "같은 큐브를 봤다면 베이스 좌표에서 같아야 한다", "04 · Agreement")
    add_equation(slide, "fixed_prediction", r"\widehat{\mathbf{O}}^{\mathrm{fix}}_{i,s}=\mathbf{C}_{i}\mathbf{Z}_{i,s}",
                 0.95, 1.75, 5.4, 0.9, 31, BLUE, LIGHT_BLUE, BLUE)
    add_equation(slide, "gripper_prediction", r"\widehat{\mathbf{O}}^{\mathrm{grip}}_{e}=\mathbf{G}_{e}\mathbf{X}\mathbf{Z}_{g,e}",
                 6.95, 1.75, 5.4, 0.9, 31, ORANGE, LIGHT_ORANGE, ORANGE)
    add_text(slide, "두 경로의 예측", 1.2, 3.15, 2.1, 0.32, 15, MUTED, True)
    add_text(slide, "→", 4.1, 3.8, 0.7, 0.5, 28, MUTED, True, PP_ALIGN.CENTER)
    add_box(slide, 5.0, 3.35, 3.35, 1.35, LIGHT_GREEN, GREEN)
    add_text(slide, "공통 큐브 자세 Qₛ", 5.35, 3.78, 2.65, 0.4, 20, GREEN, True, PP_ALIGN.CENTER)
    add_text(slide, "←", 8.55, 3.8, 0.7, 0.5, 28, MUTED, True, PP_ALIGN.CENTER)
    add_text(slide, "최대한 같은 값", 10.05, 3.15, 2.1, 0.32, 15, MUTED, True)
    add_box(slide, 1.25, 5.25, 10.85, 0.88, LIGHT_GRAY, GRID)
    add_text(slide, "캘리브레이션 = 모든 카메라의 큐브 예측이 공통 자세 Qₛ에 모이도록 미지수를 조정",
             1.6, 5.48, 10.15, 0.38, 18, INK, True, PP_ALIGN.CENTER)
    add_footer(slide, 10)

    # 11. Simple residual
    slide = blank_slide(prs); add_title(slide, "두 자세가 얼마나 다른지를 숫자로 바꾼다", "04 · Error")
    add_equation(slide, "epsilon", r"\boldsymbol{\varepsilon}(\mathbf{A},\mathbf{B})=\log\!\left(\mathbf{A}^{-1}\mathbf{B}\right)",
                 1.2, 1.75, 10.9, 1.05, 40, BLUE, LIGHT_BLUE, BLUE)
    add_bullets(slide, [
        "A와 B가 같으면 A⁻¹B = I, 따라서 오차는 0",
        "회전이 다르면 회전오차 3개가 생김",
        "위치가 다르면 이동오차 3개가 생김",
        "최종적으로 6개의 오차 숫자를 최소화",
    ], 1.4, 3.2, 5.6, 2.35, 18, INK, 18)
    add_box(slide, 7.35, 3.2, 4.45, 2.35, LIGHT_GRAY, GRID)
    add_text(slide, "읽는 방법", 7.7, 3.52, 3.75, 0.35, 17, MUTED, True)
    add_text(slide, "“A에서 B까지 남은 상대 회전과 상대 이동을 계산한다.”",
             7.7, 4.12, 3.75, 0.95, 20, INK, True, PP_ALIGN.CENTER, MSO_ANCHOR.MIDDLE)
    add_footer(slide, 11)

    # 12. Detailed residual
    slide = blank_slide(prs); add_title(slide, "상대 오차를 행렬 내부까지 펼쳐보면", "04 · Detailed error")
    add_equation(slide, "relative_transform", r"\mathbf{A}^{-1}\mathbf{B}=\left[\,\mathbf{R}_{A}^{\mathsf{T}}\mathbf{R}_{B}\;\;\mathbf{R}_{A}^{\mathsf{T}}(\mathbf{t}_{B}-\mathbf{t}_{A})\;;\;\mathbf{0}^{\mathsf{T}}\;\;1\,\right]",
                 0.9, 1.55, 11.55, 1.45, 34, INK, PAPER, BORDER)
    add_equation(slide, "code_residual", r"\mathbf{r}(\mathbf{A},\mathbf{B})=\left(\operatorname{RotVec}(\mathbf{R}_{A}^{\mathsf{T}}\mathbf{R}_{B}),\;\mathbf{R}_{A}^{\mathsf{T}}(\mathbf{t}_{B}-\mathbf{t}_{A})\right)^{\mathsf{T}}\in\mathbb{R}^{6}",
                 0.9, 3.35, 11.55, 1.55, 33, PURPLE, "F1EDF7", PURPLE)
    add_table(slide, [
        ["잔차 성분", "개수", "단위", "뜻"],
        ["RotVec", "3", "rad", "상대 회전"],
        ["Relative translation", "3", "m", "상대 이동"],
    ], [3.1, 1.5, 1.8, 4.0], 1.45, 5.28, 0.55, True,
        [PURPLE, GRAY, GRAY, GREEN], 13)
    add_text(slide, "PPT에서는 ε=log(A⁻¹B)로 읽고, 현재 코드는 위의 6차원 r(A,B)를 사용",
             1.5, 6.88, 10.3, 0.25, 11, MUTED, False, PP_ALIGN.CENTER)
    add_footer(slide, 12)

    # 13. Common objective
    slide = blank_slide(prs); add_title(slide, "세 방법이 공유하는 vision 목적함수", "04 · Common objective")
    add_equation(slide, "evis", r"\mathcal{E}_{\mathrm{vis}}(\mathbf{C},\mathbf{X},\{\mathbf{Q}_{s}\})=\sum_{i,s}\left\|\mathbf{r}(\mathbf{C}_{i}\mathbf{Z}_{i,s},\mathbf{Q}_{s})\right\|_{2}^{2}+\sum_{e}\left\|\mathbf{r}(\mathbf{G}_{e}\mathbf{X}\mathbf{Z}_{g,e},\mathbf{Q}_{s(e)})\right\|_{2}^{2}",
                 0.55, 1.65, 12.25, 1.35, 27, INK, LIGHT_GRAY, BORDER)
    section_label(slide, "고정 카메라 항", 1.15, 3.45, 3.2, BLUE)
    add_text(slide, "CᵢZᵢ,ₛ가 공통 큐브 자세 Qₛ와 가까워지게 함",
             1.15, 4.05, 3.2, 1.0, 16, INK, False, PP_ALIGN.CENTER)
    section_label(slide, "그리퍼 카메라 항", 5.05, 3.45, 3.2, ORANGE)
    add_text(slide, "GₑXZ𝗀,ₑ가 같은 set의 Qₛ와 가까워지게 함",
             5.05, 4.05, 3.2, 1.0, 16, INK, False, PP_ALIGN.CENTER)
    section_label(slide, "세 방법의 차이", 8.95, 3.45, 3.2, GREEN)
    add_text(slide, "Qₛ를 자유변수, raw FK, 보정 anchor 중 무엇으로 두는가",
             8.95, 4.05, 3.2, 1.0, 16, INK, False, PP_ALIGN.CENTER)
    add_box(slide, 2.0, 5.65, 9.35, 0.8, LIGHT_GREEN, GREEN)
    add_text(slide, "관측식과 오차함수는 동일하다. Qₛ의 정의만 바뀐다.",
             2.35, 5.87, 8.65, 0.35, 19, GREEN, True, PP_ALIGN.CENTER)
    add_footer(slide, 13)

    # 14. Overview
    slide = blank_slide(prs); add_title(slide, "세 가지 FK 방식의 전체 그림", "05 · FK modes")
    add_method_card(slide, 0.72, "No-FK",
                    "큐브 FK prior를 사용하지 않는다. 큐브 자세도 카메라와 hand–eye와 함께 직접 찾는다.",
                    r"\mathbf{Q}_{s}=\mathbf{O}_{s}\;(\mathrm{free})", BLUE)
    add_method_card(slide, 4.79, "Fixed-FK",
                    "raw FK를 정답으로 가정한다. 큐브 자세는 움직이지 못하고 다른 미지수만 조정된다.",
                    r"\mathbf{Q}_{s}=\mathbf{F}_{s}", ORANGE)
    add_method_card(slide, 8.86, "Corrected-FK",
                    "vision으로 raw FK의 공통 오정렬을 보정하고, 검증된 정보만 섞어 anchor를 만든다.",
                    r"\mathbf{Q}_{s}=\mathbf{A}_{s}", GREEN)
    add_footer(slide, 14)

    # 15. No-FK
    slide = blank_slide(prs); add_title(slide, "No-FK · 큐브 자세도 자유변수로 함께 찾는다", "05 · Method 1")
    add_equation(slide, "nofk_objective", r"\{\widehat{\mathbf{C}}_{i},\widehat{\mathbf{X}},\widehat{\mathbf{O}}_{s}\}=\underset{\{\mathbf{C}_{i}\},\mathbf{X},\{\mathbf{O}_{s}\}}{\arg\min}\;\mathcal{E}_{\mathrm{vis}}(\mathbf{C},\mathbf{X},\{\mathbf{O}_{s}\})",
                 0.75, 1.65, 11.85, 1.25, 31, BLUE, LIGHT_BLUE, BLUE)
    add_table(slide, [
        ["변수", "움직일 수 있나?", "역할"],
        ["Cᵢ", "Yes", "고정 카메라 위치/방향"],
        ["X", "Yes", "hand–eye"],
        ["Oₛ", "Yes", "set별 큐브 자세"],
        ["Fₛ", "사용 안 함", "큐브 FK prior"],
    ], [2.2, 2.7, 5.4], 1.55, 3.35, 0.58, True,
        [BLUE, GREEN, GRAY], 14)
    add_bullets(slide, [
        "장점: raw FK systematic 오차가 카메라 캘리브레이션으로 직접 전파되지 않음",
        "주의: 미지수가 많고 관측 구성이 약하면 해가 흔들릴 수 있음",
    ], 1.4, 6.15, 10.5, 0.65, 14, INK, 5)
    add_footer(slide, 15)

    # 16. Fixed-FK
    slide = blank_slide(prs); add_title(slide, "Fixed-FK · raw FK에 큐브 자세를 못 박는다", "05 · Method 2")
    add_equation(slide, "fixed_constraint", r"\mathbf{O}_{s}=\mathbf{F}_{s}",
                 0.9, 1.65, 2.8, 0.85, 38, ORANGE, LIGHT_ORANGE, ORANGE)
    add_equation(slide, "fixed_objective", r"\{\widehat{\mathbf{C}}_{i},\widehat{\mathbf{X}}\}=\underset{\{\mathbf{C}_{i}\},\mathbf{X}}{\arg\min}\;\mathcal{E}_{\mathrm{vis}}(\mathbf{C},\mathbf{X},\{\mathbf{F}_{s}\})",
                 4.0, 1.65, 8.45, 0.85, 29, ORANGE, PAPER, ORANGE)
    add_box(slide, 1.2, 3.05, 10.9, 1.45, LIGHT_ORANGE, ORANGE)
    add_text(slide, "예시", 1.55, 3.38, 1.0, 0.3, 15, ORANGE, True)
    add_text(slide, "실제 큐브 100 mm · raw FK 110 mm",
             2.65, 3.28, 4.0, 0.38, 20, INK, True)
    add_text(slide, "→", 6.75, 3.28, 0.6, 0.4, 24, ORANGE, True, PP_ALIGN.CENTER)
    add_text(slide, "남은 10 mm를 카메라 또는 hand–eye가 흡수할 수 있음",
             7.45, 3.25, 4.15, 0.65, 17, RED, True, PP_ALIGN.CENTER)
    add_table(slide, [
        ["FK 상태", "장점 / 위험"],
        ["정확함", "미지수가 줄어 강하고 안정적인 기준"],
        ["systematic 오차", "오차가 카메라·hand–eye로 전파될 위험"],
    ], [3.0, 7.3], 1.55, 5.05, 0.58, True,
        [ORANGE, GRAY], 14)
    add_footer(slide, 16)

    # 17. Corrected intuitive
    slide = blank_slide(prs); add_title(slide, "Corrected-FK · raw FK를 고쳐서 사용한다", "05 · Method 3 · Intuition")
    steps = [
        ("① Vision", "No-FK 1차 해로 Vₛ 계산", BLUE),
        ("② Difference", "Δₛ = Fₛ⁻¹Vₛ", PURPLE),
        ("③ Correct", "Fₛᶜᵒʳʳ = FₛΔ̄", GREEN),
        ("④ Guard & Blend", "검증 후 Aₛ 생성", ORANGE),
    ]
    x = 0.75
    for title, body, color in steps:
        add_box(slide, x, 1.8, 2.78, 2.05, PAPER, color)
        add_text(slide, title, x + 0.18, 2.08, 2.42, 0.35, 16, color, True, PP_ALIGN.CENTER)
        add_text(slide, body, x + 0.2, 2.75, 2.38, 0.55, 16, INK, True, PP_ALIGN.CENTER,
                 MSO_ANCHOR.MIDDLE)
        x += 3.05
    add_equation(slide, "corr_core", r"\boldsymbol{\Delta}_{s}=\mathbf{F}_{s}^{-1}\mathbf{V}_{s},\qquad\overline{\boldsymbol{\Delta}}=\operatorname{RobustWeightedAverage}_{s}(\boldsymbol{\Delta}_{s}),\qquad\mathbf{F}_{s}^{\mathrm{corr}}=\mathbf{F}_{s}\overline{\boldsymbol{\Delta}}",
                 0.65, 4.35, 12.05, 1.15, 27, GREEN, LIGHT_GREEN, GREEN)
    add_box(slide, 1.55, 5.95, 10.25, 0.65, LIGHT_GRAY, GRID)
    add_text(slide, "안 믿기(No-FK)와 그대로 믿기(Fixed-FK) 사이: vision으로 검증하고 25%만 혼합",
             1.8, 6.12, 9.75, 0.3, 16, INK, True, PP_ALIGN.CENTER)
    add_footer(slide, 17)

    # 18. Corrected detailed
    slide = blank_slide(prs); add_title(slide, "Corrected-FK · gate와 최종 목적함수", "06 · Detailed correction")
    add_equation(slide, "gate", r"d_{t}(s)=1000\left\|\mathbf{t}(\mathbf{V}_{s})-\mathbf{t}(\mathbf{F}_{s}^{\mathrm{corr}})\right\|_{2}\leq35\,\mathrm{mm},\qquad d_{R}(s)\leq8^{\circ}",
                 0.7, 1.55, 11.95, 0.95, 27, ORANGE, LIGHT_ORANGE, ORANGE)
    add_equation(slide, "guarded_anchor", r"\mathbf{A}_{s}=\operatorname{Blend}(\mathbf{V}_{s},\mathbf{F}_{s}^{\mathrm{corr}},0.25)\;\mathrm{if\;gate\;passes};\qquad\mathbf{A}_{s}=\mathbf{V}_{s}\;\mathrm{otherwise}",
                 0.7, 2.85, 11.95, 1.45, 32, GREEN, LIGHT_GREEN, GREEN)
    add_equation(slide, "corr_objective", r"\{\widehat{\mathbf{C}}_{i},\widehat{\mathbf{X}}\}=\underset{\{\mathbf{C}_{i}\},\mathbf{X}}{\arg\min}\;\mathcal{E}_{\mathrm{vis}}(\mathbf{C},\mathbf{X},\{\mathbf{A}_{s}\})",
                 0.7, 4.65, 11.95, 0.95, 30, GREEN, PAPER, GREEN)
    add_table(slide, [
        ["구현 상세", "정책"],
        ["set 평균", "관측 수 가중 + MAD 이상치 제거"],
        ["이동 blend", "(1−α)t(Vₛ)+αt(Fₛᶜᵒʳʳ)"],
        ["회전 blend", "가중합 후 SVD로 SO(3)에 투영"],
    ], [3.0, 7.3], 1.55, 5.78, 0.32, True,
        [GREEN, GRAY], 12)
    add_footer(slide, 18)

    # 19. Other axes
    slide = blank_slide(prs); add_title(slide, "FK 방식 외의 실험 축", "07 · Experimental axes")
    section_label(slide, "Solver", 0.9, 1.6, 5.55, BLUE)
    add_table(slide, [
        ["조건", "뜻"],
        ["Unified", "고정/그리퍼 subsystem을 하나의 문제로 동시 계산"],
        ["Independent", "각 subsystem을 따로 계산한 뒤 공통 타깃으로 정렬"],
    ], [1.7, 3.85], 0.9, 2.25, 0.75, True, [BLUE, GRAY], 13)
    section_label(slide, "Target", 6.9, 1.6, 5.55, GREEN)
    add_table(slide, [
        ["조건", "뜻"],
        ["Board-only", "평면 ChArUco 보드만 사용"],
        ["Cube-only", "다면 AprilTag 큐브만 사용"],
        ["Both", "큐브와 보드 관측을 함께 사용"],
    ], [1.7, 3.85], 6.9, 2.25, 0.62, True, [GREEN, GRAY], 13)
    add_box(slide, 1.15, 5.65, 11.05, 0.82, LIGHT_ORANGE, ORANGE)
    add_text(slide, "유효 조합은 14개: board-only에는 큐브 FK prior가 없어 fixed/corr를 정의하지 않음",
             1.4, 5.88, 10.55, 0.35, 16, ORANGE, True, PP_ALIGN.CENTER)
    add_footer(slide, 19)

    # 20. Evaluation
    slide = blank_slide(prs); add_title(slide, "평가할 때 반드시 구분할 것", "08 · Evaluation")
    add_method_card(slide, 0.72, "Simulation",
                    "생성할 때 실제 카메라와 큐브 자세를 알고 있다. e_X, e_task, e_rel을 외부 ground truth와 직접 비교할 수 있다.",
                    r"\mathrm{synthetic\;external\;GT}", BLUE)
    add_method_card(slide, 4.79, "Real data",
                    "재투영 오차와 cross-camera consistency로 내부 일관성을 평가한다. 서로 다른 성능 측면이므로 지표를 구분한다.",
                    r"\mathrm{consistency}\neq\mathrm{physical\;GT}", GRAY)
    add_method_card(slide, 8.86, "FK proxy",
                    "C1 held-out 위치 평가는 로봇 FK 큐브 중심을 proxy로 사용한다. FK와의 agreement이지 절대 물리 정확도는 아니다.",
                    r"\mathrm{FK\!\!\!-proxy}\neq\mathrm{absolute\;accuracy}", ORANGE)
    add_footer(slide, 20)

    # 21. Summary
    slide = blank_slide(prs); add_title(slide, "한 장으로 다시 보는 핵심 수식", "09 · Summary")
    add_equation(slide, "summary_common", r"\mathcal{E}_{\mathrm{vis}}=\sum_{i,s}\|\mathbf{r}(\mathbf{C}_{i}\mathbf{Z}_{i,s},\mathbf{Q}_{s})\|^{2}+\sum_{e}\|\mathbf{r}(\mathbf{G}_{e}\mathbf{X}\mathbf{Z}_{g,e},\mathbf{Q}_{s(e)})\|^{2}",
                 0.6, 1.5, 12.15, 1.0, 28, INK, LIGHT_GRAY, BORDER)
    add_equation(slide, "summary_cases", r"\mathbf{Q}_{s}=\mathbf{O}_{s}\;(\mathrm{No\!\!\!-FK}),\qquad\mathbf{Q}_{s}=\mathbf{F}_{s}\;(\mathrm{Fixed\!\!\!-FK}),\qquad\mathbf{Q}_{s}=\mathbf{A}_{s}\;(\mathrm{Corrected\!\!\!-FK})",
                 0.8, 2.85, 11.75, 2.15, 35, INK, PAPER, BORDER)
    add_box(slide, 1.0, 5.45, 11.3, 0.88, LIGHT_GREEN, GREEN)
    add_text(slide, "No-FK는 직접 찾고 · Fixed-FK는 못 박고 · Corrected-FK는 고쳐서 섞는다",
             1.3, 5.7, 10.7, 0.35, 19, GREEN, True, PP_ALIGN.CENTER)
    add_text(slide, "세 방법의 관측식과 오차함수는 같고, 공통 큐브 자세 Qₛ의 정의만 다르다.",
             1.7, 6.55, 9.9, 0.32, 14, MUTED, False, PP_ALIGN.CENTER)
    add_footer(slide, 21)

    prs.core_properties.title = "멀티카메라 캘리브레이션과 세 가지 FK 방식"
    prs.core_properties.subject = "Text and LaTeX equation-first teaching deck"
    prs.core_properties.author = "rb-calibration-marker-experiment"
    prs.save(OUT)
    print(OUT)


if __name__ == "__main__":
    build()
