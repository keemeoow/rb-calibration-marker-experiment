#!/usr/bin/env python3
"""Build a focused Korean PDF deck for the 8/3 meeting feedback resolution.

The deck is intentionally shorter than the full Session04 result deck.  It
answers one presentation question: "What feedback was raised, how did we fix or
scope it, and what do the current data say?"
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
from typing import Iterable

from PIL import Image, ImageDraw, ImageOps

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
STEP2B_OVERLAY = ROOT / "data/session04/calib_out/capture_filter/Step2b_review_overlay.jpg"
REDETECTION_OVERLAY = (
    ROOT / "data/session04/calib_out/verify/cube_observation_quality/"
    "redetection_recovered_core_overlay.png"
)
CUBE_MODEL_OVERLAY = (
    ROOT / "data/session04/calib_out/verify/0826_cube_model_validation/"
    "cube_model_overlay_event00000.png"
)
BOARD_CUBE_OVERLAY = (
    ROOT / "data/session04/calib_out/verify/board_cube_relative_pose/"
    "camera1_camera3_board_cube_overlay.png"
)
ROBOT_BASE_POINTCLOUD = (
    ROOT / "CP_result/session04/robot_base_pointcloud/"
    "robot_base_pointcloud_A2_event0054.png"
)
ROBOT_BASE_POINTCLOUD_JSON = (
    ROOT / "CP_result/session04/robot_base_pointcloud/"
    "robot_base_pointcloud_diagnostic.json"
)


def load_pointcloud_payload() -> dict:
    if not ROBOT_BASE_POINTCLOUD_JSON.exists():
        return {}
    return json.loads(ROBOT_BASE_POINTCLOUD_JSON.read_text(encoding="utf-8"))


def pointcloud_method_rows() -> list[dict]:
    return load_pointcloud_payload().get("method_summary", [])


def pointcloud_method_row(method: str) -> dict:
    for row in pointcloud_method_rows():
        if row.get("method") == method:
            return row
    return {}


def fmt_mm(value) -> str:
    return "N/A" if value is None else f"{float(value):.3f} mm"


def delta(ctx: dict, first: str, second: str, key: str) -> float:
    return metric(ctx, second, key) - metric(ctx, first, key)


def signed(value: float) -> str:
    return f"{value:+.4f}"


def image_panel(
    canvas: Image.Image,
    draw: ImageDraw.ImageDraw,
    path: Path,
    box: tuple[int, int, int, int],
    *,
    fit_mode: str = "cover",
    bg: str = "#0F172A",
) -> None:
    x0, y0, x1, y1 = box
    rounded(draw, box, fill=bg, outline=LINE, radius=15)
    pad = 10
    image_box = (x0 + pad, y0 + pad, x1 - pad, y1 - pad)
    iw, ih = image_box[2] - image_box[0], image_box[3] - image_box[1]
    if not path.exists():
        draw_wrapped(draw, (x0 + 28, y0 + 28), f"Missing image: {path}", font(22), PAPER, x1 - x0 - 56)
        return
    source = Image.open(path).convert("RGB")
    if fit_mode == "contain":
        source.thumbnail((iw, ih), Image.Resampling.LANCZOS)
        px = image_box[0] + (iw - source.width) // 2
        py = image_box[1] + (ih - source.height) // 2
        canvas.paste(source, (px, py))
    else:
        source = ImageOps.fit(source, (iw, ih), method=Image.Resampling.LANCZOS)
        canvas.paste(source, (image_box[0], image_box[1]))
    draw.rectangle(image_box, outline="#CBD5E1", width=2)


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
        "Session04 캘리브레이션 결과를 8/3 피드백 항목별로 재구성하고 실제 overlay 증거를 첨부한 발표자료",
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
        "8/3 피드백을 7개 유형으로 재정리",
        "19개 피드백을 번호 순서가 아니라 문제 유형별로 묶어 발표한다.",
    )
    small_stat(draw, (68, y, 384, y + 138), "11", "반영 완료", GREEN)
    small_stat(draw, (420, y, 736, y + 138), "8", "부분 반영 / 일정 의존", AMBER)
    small_stat(draw, (772, y, 1088, y + 138), "0", "완전 미반영 없음", GREEN)
    small_stat(draw, (1124, y, 1564, y + 138), "다음주", "Independent External GT", BLUE)
    card(
        draw,
        (68, y + 178, 760, y + 428),
        "완료 중심 유형",
        "C1 관측 품질, C2 목적함수/FK 설명, C3 평가 공정성은 현재 코드·데이터·overlay로 설명 가능하다.",
        GREEN,
    )
    card(
        draw,
        (804, y + 178, 1564, y + 428),
        "제한/후속 유형",
        "C4는 기여도 표현을 제한하고, C5/C6는 진단·외부 대조로 둔다. C7 point cloud는 구현했고 물리 GT와 robot task는 다음주 예정이다.",
        ORANGE,
    )
    return img


def slide_feedback_clusters(ctx: dict, i: int, n: int) -> Image.Image:
    del ctx
    img, draw, y = content_base(
        i,
        n,
        "FEEDBACK MAP",
        "피드백을 7개 분류로 묶으면",
        "각 피드백은 하나의 primary 분류에 배정하고, 해결 방식과 남은 일을 분리한다.",
    )
    rows = [
        ["C1", "#1 #2 #18", "관측 품질 / 이상치", "frame-prune, refit/rollback, overlay QA"],
        ["C2", "#3 #4 #9 #19", "목적함수 / FK 사용", "visual/FK block 분리, A2/A4/A5 역할 확정"],
        ["C3", "#5 #8 #10 #11 #12", "평가 공정성 / 지표", "matched contrast, event split, camera scope"],
        ["C4", "#13", "기여도 / ablation", "A1/B1은 제안이 아니라 효과 분리 baseline"],
        ["C5", "#6", "큰 오차 원인 진단", "Board-Cube conflict 17.299→10.808 mm 추적"],
        ["C6", "#7", "외부 구현 대조", "OpenCV reference + frozen external package"],
        ["C7", "#14 #15 #16 #17", "물리 GT / 로봇 작업", "Track C + point-cloud diagnostic, 실측 GT는 다음주"],
    ]
    table(
        draw,
        72,
        y,
        [88, 210, 340, 710],
        ["분류", "피드백", "문제 축", "현재 처리"],
        rows,
        row_font_size=18,
        header_font_size=17,
        max_bottom=810,
    )
    return img


def slide_solution_contract(ctx: dict, i: int, n: int) -> Image.Image:
    del ctx
    img, draw, y = content_base(
        i,
        n,
        "TYPE CONTRACT",
        "유형별 해결 계약",
        "발표에서는 먼저 C분류를 말하고, 그 다음 해당 증거 슬라이드로 넘긴다.",
    )
    rows = [
        ["C1", "#1 #2 #18", "관측 품질 / 이상치", "frame-prune, refit/rollback, 실제 overlay QA"],
        ["C2", "#3 #4 #9 #19", "목적함수 / FK 사용", "visual-only 1항, soft-FK 2항, A2/A4/A5 역할 분리"],
        ["C3", "#5 #8 #10 #11 #12", "평가 공정성 / 지표", "matched contrast, event split, camera-scope, mm/deg 보조 지표"],
        ["C4", "#13", "기여도 / ablation", "A1/B1은 제안 방법이 아니라 효과 분리 baseline"],
        ["C5", "#6", "큰 오차 원인 진단", "Board-Cube conflict 10.8077 mm를 제한으로 명시"],
        ["C6", "#7", "외부 구현 대조", "OpenCV reference와 frozen external package"],
        ["C7", "#14 #15 #16 #17", "물리 GT / 로봇 작업", "Track C + point-cloud diagnostic, GT는 다음주"],
    ]
    table(
        draw,
        72,
        y,
        [84, 216, 330, 720],
        ["분류", "피드백", "문제 축", "확인 자료 / 발표 답변"],
        rows,
        row_font_size=18,
        header_font_size=17,
        max_bottom=810,
    )
    return img


def slide_evidence_observation_overlay(ctx: dict, i: int, n: int) -> Image.Image:
    del ctx
    img, draw, y = content_base(
        i,
        n,
        "C1 | REAL OVERLAY",
        "관측 품질: selected / recovered / rejected",
        "Step 04의 frozen observation manifest가 실제 RGB 위에서 어떻게 판정됐는지 보여준다.",
    )
    image_panel(img, draw, STEP2B_OVERLAY, (68, y, 982, y + 522), fit_mode="cover")
    card(
        draw,
        (1024, y, 1564, y + 150),
        "증거 역할",
        "초록은 recovered/selected, 노랑은 quarantine, 빨강은 rejected다. 검출 QA를 실제 사진 위에서 확인한다.",
        GREEN,
        title_size=25,
        body_size=21,
    )
    card(
        draw,
        (1024, y + 184, 1564, y + 338),
        "연결 유형",
        "#1 image-level frame prune, #2 refit/rollback, #18 overlay QA에 대한 시각 증거다.",
        BLUE,
        title_size=25,
        body_size=21,
    )
    card(
        draw,
        (1024, y + 372, 1564, y + 522),
        "현재 숫자",
        "Cube detection: 117 images read, 108 accepted PnP observations, 99 core multiface selected, 2 RMSE rejections.",
        ORANGE,
        title_size=25,
        body_size=20,
    )
    return img


def slide_evidence_cube_overlays(ctx: dict, i: int, n: int) -> Image.Image:
    del ctx
    img, draw, y = content_base(
        i,
        n,
        "C1+C5 | REAL OVERLAY",
        "Cube 검출과 모델 일치 확인",
        "실제 RGB 위의 cube corner, face ID, model reprojection을 확인한다.",
    )
    image_panel(img, draw, REDETECTION_OVERLAY, (68, y, 1564, y + 230), fit_mode="cover")
    image_panel(img, draw, CUBE_MODEL_OVERLAY, (68, y + 264, 960, y + 522), fit_mode="cover")
    card(
        draw,
        (1000, y + 264, 1564, y + 392),
        "Redetection evidence",
        "baseline 검출에서 부족했던 관측을 relaxed/subpixel 후보로 회복했고, RMSE와 face set을 함께 표시했다.",
        GREEN,
        title_size=24,
        body_size=20,
    )
    card(
        draw,
        (1000, y + 420, 1564, y + 522),
        "Model evidence",
        "green=detected, magenta=model reprojection, orange=normal/axis로 cube geometry가 실제 이미지와 맞는지 본다.",
        BLUE,
        title_size=24,
        body_size=19,
    )
    return img


def slide_evidence_board_cube_conflict(ctx: dict, i: int, n: int) -> Image.Image:
    del ctx
    img, draw, y = content_base(
        i,
        n,
        "C5 | REAL OVERLAY",
        "큰 오차 원인: Board-Cube conflict",
        "같은 event의 board PnP와 cube PnP를 같은 fixed camera pair에서 직접 비교한 overlay다.",
    )
    image_panel(img, draw, BOARD_CUBE_OVERLAY, (68, y, 1564, y + 340), fit_mode="cover")
    small_stat(draw, (68, y + 378, 384, y + 512), "10.8077 mm", "direct PnP translation RMSE", ORANGE)
    small_stat(draw, (420, y + 378, 736, y + 512), "0.5270°", "max rotation disagreement", ORANGE)
    card(
        draw,
        (772, y + 378, 1564, y + 512),
        "즉시 triage",
        "event 54 제거 후에도 9.599 mm가 남는다. 단일 프레임보다 cube sparse observation + localization/intrinsic bias로 설명한다.",
        RED,
        title_size=23,
        body_size=20,
    )
    return img


def slide_robot_base_pointcloud(ctx: dict, i: int, n: int) -> Image.Image:
    del ctx
    a2 = pointcloud_method_row("A2")
    obs = a2.get("observations", 24)
    img, draw, y = content_base(
        i,
        n,
        "C7 | POINT CLOUD",
        "로봇 base frame에서 본 point cloud 정합",
        "aligned depth를 calibration 목적함수에 넣지 않고, A2 transform으로 robot-base 좌표계에 올려 진단한다.",
    )
    image_panel(img, draw, ROBOT_BASE_POINTCLOUD, (68, y, 1098, y + 522), fit_mode="contain", bg=PAPER)
    small_stat(draw, (1134, y, 1564, y + 124), f"{obs} obs", "event 24/54/72 board+cube", BLUE)
    small_stat(draw, (1134, y + 150, 1564, y + 274), fmt_mm(a2.get("board_rmse_mm")), "board depth-to-plane RMSE", TEAL)
    small_stat(draw, (1134, y + 300, 1564, y + 424), fmt_mm(a2.get("cube_rmse_mm")), "cube depth-to-plane RMSE", ORANGE)
    card(
        draw,
        (1134, y + 450, 1564, y + 608),
        "해석 제한",
        "depth 기반 진단이다. external GT와 robot-contact 정확도는 다음주 실측에서 확정한다.",
        RED,
        title_size=21,
        body_size=19,
    )
    return img


def slide_robot_base_pointcloud_rows(ctx: dict, i: int, n: int) -> Image.Image:
    del ctx
    img, draw, y = content_base(
        i,
        n,
        "C7 | ROW SUMMARY",
        "point cloud도 비교실험 row별로 보면",
        "각 row의 marker 구성만 사용해 robot-base depth-to-plane residual을 산출했다.",
    )
    rows = []
    for row in pointcloud_method_rows():
        rows.append([
            row["method"],
            row["targets"],
            fmt_mm(row.get("board_rmse_mm")),
            fmt_mm(row.get("cube_rmse_mm")),
            fmt_mm(row.get("combined_rmse_mm")),
            row["role"],
        ])
    table(
        draw,
        72,
        y,
        [84, 170, 170, 170, 190, 690],
        ["Row", "Target", "Board", "Cube", "Combined", "역할"],
        rows,
        row_font_size=17,
        header_font_size=16,
        max_bottom=748,
    )
    card(
        draw,
        (72, 700, 1564, 836),
        "해석",
        "A5/B2가 낮아 보여도 post-hoc/preflight 성격이라 물리 승자가 아니다. #17의 로봇 좌표계 진단 증거로만 사용한다.",
        ORANGE,
        title_size=21,
        body_size=20,
    )
    return img


def slide_algorithm(ctx: dict, i: int, n: int) -> Image.Image:
    del ctx
    img, draw, y = content_base(
        i,
        n,
        "C2 | ALGORITHM",
        "목적함수 / FK 사용 설명",
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
        "C4 | EXPERIMENT DESIGN",
        "기여도는 ablation 표로 제한",
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
        "C3 | METRICS",
        "평가지표 공정성 계층",
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
        "C2+C4 | DATA RESULT",
        "A1 → A2: unified 효과",
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
        "C4+C5 | DATA RESULT",
        "Cube claim은 조건부로 제한",
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
        "C2 | DATA RESULT",
        "FK는 hard GT가 아니라 prior",
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
        "C1+C5+C6 | DATA QA",
        "데이터 문제와 남은 리스크",
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
        "C1-C7 | CLOSING",
        "유형별 최종 발표 멘트",
        "이 문단이 현재 결론의 가장 안전한 버전이다.",
    )
    script = (
        f"“8/3 피드백은 C1 관측 품질, C2 목적함수/FK, C3 평가 공정성, C4 ablation 포지셔닝, "
        f"C5 큰 오차 원인, C6 외부 구현 대조, C7 물리 GT/로봇 작업으로 나눠 정리했습니다. "
        f"현재 주 지표는 같은 marker population의 held-out reprojection이고, A1→A2에서 own overall이 "
        f"{fmt(a1)} px에서 {fmt(a2)} px로 낮아져 unified feedback 효과가 가장 분명합니다. "
        f"다만 cube 효과는 조건부이고, hard FK는 악화되며, soft-FK는 preflight입니다. "
        f"#17은 A0–A5/B1–B3 전체 비교 row에 대해 robot-base point-cloud diagnostic까지 구현했습니다. "
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
    slide_evidence_observation_overlay,
    slide_evidence_cube_overlays,
    slide_evidence_board_cube_conflict,
    slide_robot_base_pointcloud,
    slide_robot_base_pointcloud_rows,
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
