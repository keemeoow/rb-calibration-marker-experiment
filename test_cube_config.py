# test_cube_config.py
"""
큐브 정의 검증용 단독 테스트.

목적:
  카메라 1대만 켜서 한 프레임(또는 라이브 프리뷰)을 촬영하고,
  config.py에 정의된 AprilTag 큐브가 "물리 큐브와 실제로 일치하는지"를
  눈으로 확인한다. Step2/Step3 파이프라인 없이 큐브 정의만 빠르게 점검하는 용도.

무엇을 확인하나:
  1) (카메라 없이도) config.py 큐브 정의가 기하학적으로 자기모순이 없는지
     (마커 크기/중심/법선/경계 검사 = apriltag_cube.validate_cube_config)
  2) (카메라로) 실제 촬영 프레임에서 마커가 검출되는지, 어떤 face가 보이는지
  3) 큐브 solvePnP 결과의 재투영 오차(px)
  4) 추정된 큐브 pose로 그린 "큐브 와이어프레임 + 좌표축"이 실제 큐브에
     정확히 겹치는지 (겹치면 큐브 정의가 물리 큐브와 일치한다는 뜻)

Intrinsics:
  기본은 RealSense 장치가 factory-calibrated 값을 직접 제공하므로 별도 교정
  파일 없이 동작한다. 교정한 값을 쓰고 싶으면 --intrinsics_dir + --cam_idx 지정.

실행 명령어:

  # 연결된 RealSense 목록만 출력
  python test_cube_config.py --list

  # 라이브 프리뷰(기본, 640x480). 's' 저장, SPACE 상세리포트, 'q' 종료
  python test_cube_config.py

  # 1280x720 고해상도로 (검출 정확도↑, 다중 카메라 대역폭 부담↑)
  python test_cube_config.py --hd

  # 창 없이 한 프레임만 캡처/분석 후 저장 (헤드리스)
  python test_cube_config.py --once

  # 교정된 intrinsics 사용
  python test_cube_config.py --intrinsics_dir ../rb-ArucoCube_Robot_multi_calibration/intrinsics --cam_idx 0
"""

import os
import json
import time
import argparse
import datetime
from typing import Optional, Tuple, List

import cv2
import numpy as np

from config import get_default_cube_config
from apriltag_cube import (
    AprilTagCubeTarget,
    print_cube_sanity_check,
    validate_cube_config,
)


# ---------------------------------------------------------------------------
# Intrinsics
# ---------------------------------------------------------------------------
def load_intrinsics_from_dir(intr_dir: str, cam_idx: int) -> Tuple[np.ndarray, np.ndarray]:
    """교정된 intrinsics npz(cam{idx}.npz)에서 K, D 로드."""
    p = os.path.join(intr_dir, f"cam{cam_idx}.npz")
    if not os.path.exists(p):
        raise FileNotFoundError(f"Intrinsics not found: {p}")
    d = np.load(p, allow_pickle=True)
    K = d["color_K"].astype(np.float64)
    D = d["color_D"].astype(np.float64)
    return K, D


def color_intrinsics_from_camera(cam) -> Tuple[np.ndarray, np.ndarray]:
    """RealSense 장치가 제공하는 color 스트림 factory intrinsics에서 K, D 추출."""
    import pyrealsense2 as rs
    profile = cam.pipeline.get_active_profile()
    vsp = profile.get_stream(rs.stream.color).as_video_stream_profile()
    intr = vsp.get_intrinsics()
    K = np.array([
        [intr.fx, 0.0, intr.ppx],
        [0.0, intr.fy, intr.ppy],
        [0.0, 0.0, 1.0],
    ], dtype=np.float64)
    D = np.asarray(intr.coeffs, dtype=np.float64).reshape(-1, 1)
    return K, D


# ---------------------------------------------------------------------------
# Drawing helpers
# ---------------------------------------------------------------------------
def draw_cube_wireframe(img: np.ndarray, K: np.ndarray, D: np.ndarray,
                        rvec: np.ndarray, tvec: np.ndarray, side_m: float,
                        color=(255, 255, 0), thickness: int = 2) -> None:
    """추정된 큐브 pose로 59mm 큐브의 12개 모서리를 투영해 그린다.

    이 와이어프레임이 화면 속 실제 큐브 외곽과 겹치면 큐브 정의(치수/원점)가
    물리 큐브와 일치한다는 강한 시각적 증거가 된다.
    """
    d = float(side_m) / 2.0
    corners = np.array(
        [[x, y, z] for x in (-d, d) for y in (-d, d) for z in (-d, d)],
        dtype=np.float64,
    )
    proj, _ = cv2.projectPoints(corners, rvec, tvec, K, D)
    proj = proj.reshape(-1, 2)
    if not np.all(np.isfinite(proj)):
        return
    proj = proj.astype(int)
    for i in range(8):
        for j in range(i + 1, 8):
            # 좌표 하나만 다른 두 꼭짓점 = 큐브 모서리
            if int(np.sum(np.abs(corners[i] - corners[j]) > 1e-9)) == 1:
                cv2.line(img, tuple(proj[i]), tuple(proj[j]), color, thickness, cv2.LINE_AA)


def draw_reprojection(img: np.ndarray, reproj: dict) -> None:
    """검출 코너(빨강)와 재투영 코너(초록)를 함께 찍어 재투영 오차를 시각화."""
    if not reproj:
        return
    img_pts = np.asarray(reproj.get("img_pts")).reshape(-1, 2)
    proj2 = np.asarray(reproj.get("proj2")).reshape(-1, 2)
    for p in img_pts:
        cv2.circle(img, (int(p[0]), int(p[1])), 4, (0, 0, 255), 1, cv2.LINE_AA)   # 관측
    for p in proj2:
        cv2.circle(img, (int(p[0]), int(p[1])), 2, (0, 255, 0), -1, cv2.LINE_AA)  # 재투영


def draw_footer(img: np.ndarray, lines: List[str],
                colors: Optional[List[tuple]] = None) -> np.ndarray:
    """이미지 하단에 상태 텍스트 바를 붙여서 반환."""
    if not lines:
        return img
    h = 26 * len(lines) + 10
    footer = np.zeros((h, img.shape[1], 3), dtype=np.uint8)
    if colors is None:
        colors = [(255, 255, 255)] * len(lines)
    for i, line in enumerate(lines):
        cv2.putText(footer, line, (10, 24 + i * 26),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                    colors[min(i, len(colors) - 1)], 1, cv2.LINE_AA)
    return cv2.vconcat([img, footer])


# ---------------------------------------------------------------------------
# Analysis
# ---------------------------------------------------------------------------
def analyze_frame(cube: AprilTagCubeTarget, img: np.ndarray,
                  K: np.ndarray, D: np.ndarray,
                  max_err: float, min_aspect: float, min_markers: int) -> Tuple[np.ndarray, dict]:
    """한 프레임에서 마커 검출 + 큐브 PnP를 수행하고 오버레이 이미지를 만든다."""
    out = img.copy()
    expected_ids = sorted(int(x) for x in cube.cfg.marker_ids)

    corners_list, ids = cube.detect(img)
    detected_ids = sorted(set(int(x) for x in ids)) if ids is not None else []

    # 검출된 마커 외곽 + ID 표기
    if ids is not None and len(ids) > 0:
        cv2.aruco.drawDetectedMarkers(out, corners_list, ids.reshape(-1, 1).astype(np.int32))
        # face 이름을 각 마커 중심에 추가로 표기
        for c, mid in zip(corners_list, ids):
            mid = int(mid)
            if not cube.model.has_marker(mid):
                continue
            center = np.asarray(c, dtype=np.float64).reshape(4, 2).mean(axis=0)
            face = cube.model.marker_face_name(mid)
            cv2.putText(out, face, (int(center[0]) - 10, int(center[1]) + 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2, cv2.LINE_AA)

    ok, rvec, tvec, used, reproj = cube.solve_pnp_cube(
        img, K, D,
        use_ransac=True,
        min_markers=max(int(min_markers), 1),
        reproj_thr_mean_px=float(max_err),
        return_reproj=True,
        min_aspect=float(min_aspect),
    )

    result = {
        "expected_ids": expected_ids,
        "detected_ids": detected_ids,
        "missing_ids": [m for m in expected_ids if m not in detected_ids],
        "unknown_ids": [m for m in detected_ids if m not in expected_ids],
        "pnp_ok": bool(ok),
        "used_ids": sorted(int(x) for x in used) if used else [],
    }

    if ok and reproj is not None:
        result["reproj_err_mean_px"] = float(reproj.get("err_mean", float("nan")))
        result["reproj_err_median_px"] = float(reproj.get("err_median", float("nan")))
        result["reproj_err_p90_px"] = float(reproj.get("err_p90", float("nan")))
        result["n_points"] = int(reproj.get("n_points", 0))
        tvec_m = np.asarray(tvec, dtype=np.float64).reshape(3)
        result["t_cam_cube_mm"] = (tvec_m * 1000.0).tolist()
        result["distance_mm"] = float(np.linalg.norm(tvec_m) * 1000.0)

        draw_cube_wireframe(out, K, D, rvec, tvec, cube.cfg.cube_side_m)
        cv2.drawFrameAxes(out, K, D, rvec, tvec, float(cube.cfg.cube_side_m) * 0.5, 2)
        draw_reprojection(out, reproj)

    return out, result


def status_lines(result: dict, intr_src: str) -> Tuple[List[str], List[tuple]]:
    lines, colors = [], []
    det = result["detected_ids"]
    exp = result["expected_ids"]
    lines.append(f"intrinsics: {intr_src}")
    colors.append((200, 200, 200))
    lines.append(f"detected {len(det)}/{len(exp)} ids: {det}  missing: {result['missing_ids']}")
    colors.append((0, 255, 0) if not result["missing_ids"] else (0, 200, 255))
    if result["unknown_ids"]:
        lines.append(f"WARNING unknown ids (not in cube config): {result['unknown_ids']}")
        colors.append((0, 0, 255))
    if result["pnp_ok"]:
        lines.append(
            f"PnP OK  used={result['used_ids']}  reproj_mean={result.get('reproj_err_mean_px', float('nan')):.2f}px"
            f"  dist={result.get('distance_mm', float('nan')):.0f}mm"
        )
        err = result.get("reproj_err_mean_px", 99.0)
        colors.append((0, 255, 0) if err <= 2.0 else ((0, 200, 255) if err <= 5.0 else (0, 0, 255)))
    else:
        lines.append("PnP FAILED (마커가 안보이거나 재투영 오차 임계 초과)")
        colors.append((0, 0, 255))
    return lines, colors


def print_report(result: dict, intr_src: str) -> None:
    print("\n===== CUBE DETECTION REPORT =====")
    print(f"intrinsics source : {intr_src}")
    print(f"expected marker ids: {result['expected_ids']}")
    print(f"detected marker ids: {result['detected_ids']}")
    print(f"missing ids        : {result['missing_ids']}")
    if result["unknown_ids"]:
        print(f"[WARN] unknown ids (not in cube config): {result['unknown_ids']}")
    if result["pnp_ok"]:
        print(f"PnP: OK  used_ids={result['used_ids']}  n_points={result.get('n_points')}")
        print(f"  reproj mean/median/p90 (px): "
              f"{result.get('reproj_err_mean_px', float('nan')):.3f} / "
              f"{result.get('reproj_err_median_px', float('nan')):.3f} / "
              f"{result.get('reproj_err_p90_px', float('nan')):.3f}")
        t = result.get("t_cam_cube_mm", [float('nan')] * 3)
        print(f"  t_cam_cube (mm): [{t[0]:.1f}, {t[1]:.1f}, {t[2]:.1f}]  "
              f"distance={result.get('distance_mm', float('nan')):.1f}mm")
        err = result.get("reproj_err_mean_px", 99.0)
        verdict = "우수 (<2px)" if err <= 2.0 else ("양호 (<5px)" if err <= 5.0 else "불량 (>5px) - 큐브 정의/치수 확인 필요")
        print(f"  판정: {verdict}")
    else:
        print("PnP: FAILED")
    print("=================================\n")


# ---------------------------------------------------------------------------
# Config-only check (카메라 불필요)
# ---------------------------------------------------------------------------
def run_config_only() -> int:
    print("[1/1] config.py 큐브 정의 기하 정합성 검사\n")
    ok = print_cube_sanity_check()
    return 0 if ok else 1


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main() -> int:
    parser = argparse.ArgumentParser(
        description="AprilTag 큐브 정의 검증용 단일 카메라 촬영 테스트")
    parser.add_argument("--config-only", action="store_true",
                        help="카메라 없이 config.py 큐브 정의 정합성만 검사")
    parser.add_argument("--list", action="store_true",
                        help="연결된 RealSense 장치 목록만 출력하고 종료")
    parser.add_argument("--serial", default=None,
                        help="사용할 카메라 시리얼 (미지정 시 첫 번째 장치)")
    parser.add_argument("--intrinsics_dir", default=None,
                        help="교정된 intrinsics 폴더 (미지정 시 장치 factory 값 사용)")
    parser.add_argument("--cam_idx", type=int, default=0,
                        help="--intrinsics_dir 사용 시 cam{idx}.npz 인덱스")
    parser.add_argument("--width", type=int, default=640,
                        help="color 폭 (기본 640; 카메라 4대 연결 시 안정적)")
    parser.add_argument("--height", type=int, default=480,
                        help="color 높이 (기본 480)")
    parser.add_argument("--fps", type=int, default=15)
    parser.add_argument("--hd", action="store_true",
                        help="1280x720 고해상도 사용 (--width/--height 무시). "
                             "검출 정확도↑ 이지만 다중 카메라 USB 대역폭 부담↑")
    parser.add_argument("--once", action="store_true",
                        help="창 없이 한 프레임만 캡처/분석 후 저장하고 종료 (헤드리스)")
    parser.add_argument("--max-err", type=float, default=8.0,
                        help="큐브 PnP 재투영 오차 임계(px)")
    parser.add_argument("--min-aspect", type=float, default=0.3,
                        help="마커 종횡비 최소값(비스듬한 마커 배제)")
    parser.add_argument("--min-markers", type=int, default=1,
                        help="큐브 PnP 최소 마커 수")
    parser.add_argument("--reset", action="store_true",
                        help="시작 전 모든 RealSense 하드웨어 리셋 (bad state 복구)")
    parser.add_argument("--out", default="cube_test_out",
                        help="주석 이미지/리포트 저장 폴더")
    args = parser.parse_args()

    if args.hd:
        args.width, args.height = 1280, 720

    # 큐브 정의 검사는 항상 먼저 (카메라와 무관하게 큐브가 '제대로 만들어졌는지' 확인)
    print("========== 큐브 정의 정합성 검사 ==========")
    cfg = get_default_cube_config()
    cfg_ok, problems = validate_cube_config(cfg)
    print_cube_sanity_check(cfg)
    if not cfg_ok:
        print("[ERROR] 큐브 정의가 자기모순입니다. config.py를 먼저 고치세요:")
        for p in problems:
            print(f"  - {p}")
        return 1

    if args.config_only:
        print("\n--config-only: 카메라 단계 생략. 큐브 정의 검사 통과.")
        return 0

    # --- 카메라 단계 (pyrealsense2 필요) ---
    try:
        from camera import RealSenseCamera
    except Exception as e:
        print(f"\n[ERROR] 카메라 모듈 로드 실패 (pyrealsense2 미설치?): {e}")
        print("        큐브 정의만 검사하려면 --config-only 로 실행하세요.")
        return 1

    devices = RealSenseCamera.list_devices()
    if not devices:
        print("\n[ERROR] 연결된 RealSense 장치가 없습니다.")
        return 1

    print("\n연결된 RealSense 장치:")
    for s, name in devices.items():
        print(f"  - {s}  ({name})")
    if args.list:
        return 0

    serial = args.serial or sorted(devices.keys())[0]
    if serial not in devices:
        print(f"[ERROR] 시리얼 {serial} 을(를) 찾을 수 없습니다.")
        return 1
    print(f"\n사용 카메라: {serial} ({devices[serial]})  {args.width}x{args.height}@{args.fps}")

    if args.reset:
        RealSenseCamera.reset_all_devices()

    cube = AprilTagCubeTarget(cfg)

    cam = RealSenseCamera(
        serial=serial,
        width=args.width, height=args.height, fps=args.fps,
        use_color=True, use_depth=False, warmup_frames=10,
    )
    cam.start()

    # Intrinsics 결정
    if args.intrinsics_dir:
        K, D = load_intrinsics_from_dir(args.intrinsics_dir, args.cam_idx)
        intr_src = f"file:{args.intrinsics_dir}/cam{args.cam_idx}.npz"
    else:
        K, D = color_intrinsics_from_camera(cam)
        intr_src = "realsense-factory"
    print(f"intrinsics: {intr_src}\nK=\n{np.round(K, 2)}\nD={np.round(D.reshape(-1), 4)}")

    os.makedirs(args.out, exist_ok=True)
    stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")

    def _grab() -> Optional[np.ndarray]:
        color, _depth, _ts = cam.get_latest()
        return color

    def _save(annotated: np.ndarray, raw: np.ndarray, result: dict) -> None:
        base = os.path.join(args.out, f"cube_test_{stamp}")
        cv2.imwrite(base + "_annotated.png", annotated)
        cv2.imwrite(base + "_raw.png", raw)
        with open(base + "_report.json", "w") as f:
            json.dump({"intrinsics_source": intr_src, "serial": serial, **result},
                      f, indent=2, ensure_ascii=False)
        print(f"저장됨: {base}_annotated.png / _raw.png / _report.json")

    try:
        if args.once:
            # 창 없이 한 프레임만
            raw = None
            for _ in range(40):  # 프레임 버퍼 안정될 때까지 잠깐 대기
                raw = _grab()
                if raw is not None:
                    break
                time.sleep(0.05)
            if raw is None:
                print("[ERROR] 프레임을 받지 못했습니다.")
                return 1
            annotated, result = analyze_frame(
                cube, raw, K, D, args.max_err, args.min_aspect, args.min_markers)
            print_report(result, intr_src)
            _save(annotated, raw, result)
            return 0

        # 라이브 프리뷰 (기본)
        print("\n[라이브 프리뷰] 큐브를 카메라 앞에 놓으세요.")
        print("  s: 현재 프레임 저장   SPACE: 상세 리포트 출력   q/ESC: 종료")
        win = "cube test (single cam)"
        while True:
            raw = _grab()
            if raw is None:
                if (cv2.waitKey(30) & 0xFF) in (ord('q'), 27):
                    break
                continue
            annotated, result = analyze_frame(
                cube, raw, K, D, args.max_err, args.min_aspect, args.min_markers)
            lines, colors = status_lines(result, intr_src)
            panel = draw_footer(annotated, lines, colors)
            cv2.imshow(win, panel)
            key = cv2.waitKey(30) & 0xFF
            if key in (ord('q'), 27):
                break
            elif key == ord('s'):
                _save(annotated, raw, result)
            elif key == ord(' '):
                print_report(result, intr_src)
        cv2.destroyAllWindows()
        return 0
    finally:
        cam.stop()


if __name__ == "__main__":
    raise SystemExit(main())
