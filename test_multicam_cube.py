# test_multicam_cube.py
"""
다중 카메라 큐브 일관성 라이브 테스트.

목적:
  4대(고정 3대 + 그리퍼 1대)를 모두 켜고, 각 카메라가 "동시에" 본 AprilTag 큐브를
  캘리브레이션 외부 파라미터(T_base_Ci)로 공통 base 좌표계에 올렸을 때
  "하나의 큐브로 합쳐지는지(= 4대가 같은 위치/자세를 가리키는지)"를 실시간으로 확인한다.

  Step4_verify.test_cross_camera_consistency 와 같은 검사를, 저장 데이터가 아니라
  라이브 카메라로, 전 카메라 동시에 눈으로 보는 도구.

원리:
  카메라 i가 본 큐브 pose를 T_Ci_O 라 하면, base 좌표계에서의 큐브는
      T_base_O(i) = T_base_Ci @ T_Ci_O
  캘리브레이션이 정확하고 큐브가 하나라면 모든 고정 카메라의 T_base_O(i)가
  거의 같아야 한다. 이 산포(translation mm / rotation deg)가 곧 "합쳐짐"의 척도.

  그리퍼 카메라(이동 카메라)는 base 변환이 로봇 pose에 의존하므로 이 정지-큐브
  라이브 검사에서는 공통 프레임 병합에서 제외한다(자체 검출만 표시).

필요 입력:
  - intrinsics_dir : cam{i}.npz (교정 K,D) + device_map.json (serial->idx, gripper_cam_idx)
  - calib_dir      : Step3 산출물 T_base_C{i}.npy (고정 카메라), (선택) T_base_O.npy

실행 예:
  python test_multicam_cube.py \
    --intrinsics_dir ../rb-ArucoCube_Robot_multi_calibration/intrinsics \
    --calib_dir      ../rb-ArucoCube_Robot_multi_calibration/data/session/calib_out

  # 창 없이 한 번만 측정/저장 (헤드리스)
  python test_multicam_cube.py --once
"""

import os
import json
import time
import argparse
import datetime
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np

from config import get_default_cube_config
from apriltag_cube import AprilTagCubeTarget, inv_T, validate_cube_config, print_cube_sanity_check
from utils_pose import robust_se3_average
from calibration_runtime_utils import rotation_error_deg


DEFAULT_INTR = "../rb-ArucoCube_Robot_multi_calibration/intrinsics"
DEFAULT_CALIB = "../rb-ArucoCube_Robot_multi_calibration/data/session/calib_out"

# 카메라별 표시 색상 (BGR)
CAM_COLORS = {
    0: (0, 200, 0),      # green
    1: (255, 120, 0),    # blue
    2: (0, 140, 255),    # orange (gripper)
    3: (200, 0, 200),    # magenta
}
DEFAULT_COLOR = (0, 255, 255)


# ---------------------------------------------------------------------------
# Loading
# ---------------------------------------------------------------------------
def load_device_map(intr_dir: str) -> Tuple[Dict[str, int], Optional[int]]:
    p = os.path.join(intr_dir, "device_map.json")
    if not os.path.exists(p):
        raise FileNotFoundError(f"device_map.json not found in {intr_dir}")
    with open(p, "r") as f:
        m = json.load(f)
    serial_to_idx = {str(s): int(i) for s, i in m.get("serial_to_idx", {}).items()}
    gripper_cam_idx = m.get("gripper_cam_idx", None)
    return serial_to_idx, (int(gripper_cam_idx) if gripper_cam_idx is not None else None)


def load_intrinsics(intr_dir: str, cam_idx: int) -> Tuple[np.ndarray, np.ndarray]:
    p = os.path.join(intr_dir, f"cam{cam_idx}.npz")
    if not os.path.exists(p):
        raise FileNotFoundError(f"Intrinsics not found: {p}")
    d = np.load(p, allow_pickle=True)
    return d["color_K"].astype(np.float64), d["color_D"].astype(np.float64)


def load_base_extrinsics(calib_dir: str, cam_ids: List[int]) -> Dict[int, np.ndarray]:
    """고정 카메라별 T_base_C{i}.npy 로드 (있는 것만)."""
    out = {}
    for ci in cam_ids:
        p = os.path.join(calib_dir, f"T_base_C{ci}.npy")
        if os.path.exists(p):
            T = np.load(p).astype(np.float64)
            if T.shape == (4, 4) and np.all(np.isfinite(T)):
                out[int(ci)] = T
    return out


def load_optional_T(calib_dir: str, name: str) -> Optional[np.ndarray]:
    p = os.path.join(calib_dir, name)
    if os.path.exists(p):
        T = np.load(p).astype(np.float64)
        if T.shape == (4, 4) and np.all(np.isfinite(T)):
            return T
    return None


# ---------------------------------------------------------------------------
# Per-camera analysis
# ---------------------------------------------------------------------------
def draw_cube_overlay(out: np.ndarray, cube: AprilTagCubeTarget,
                      K: np.ndarray, D: np.ndarray,
                      corners_list, ids, rvec, tvec, reproj,
                      color=(255, 255, 0)) -> None:
    if ids is not None and len(ids) > 0:
        cv2.aruco.drawDetectedMarkers(out, corners_list, ids.reshape(-1, 1).astype(np.int32))
    if rvec is None:
        return
    # 큐브 와이어프레임 (59mm 3D 박스)
    d = float(cube.cfg.cube_side_m) / 2.0
    box = np.array([[x, y, z] for x in (-d, d) for y in (-d, d) for z in (-d, d)], np.float64)
    proj, _ = cv2.projectPoints(box, rvec, tvec, K, D)
    proj = proj.reshape(-1, 2)
    if np.all(np.isfinite(proj)):
        proj = proj.astype(int)
        for i in range(8):
            for j in range(i + 1, 8):
                if int(np.sum(np.abs(box[i] - box[j]) > 1e-9)) == 1:
                    cv2.line(out, tuple(proj[i]), tuple(proj[j]), color, 2, cv2.LINE_AA)
    cv2.drawFrameAxes(out, K, D, rvec, tvec, d, 2)
    if reproj:
        for p in np.asarray(reproj.get("img_pts")).reshape(-1, 2):
            cv2.circle(out, (int(p[0]), int(p[1])), 4, (0, 0, 255), 1, cv2.LINE_AA)
        for p in np.asarray(reproj.get("proj2")).reshape(-1, 2):
            cv2.circle(out, (int(p[0]), int(p[1])), 2, (0, 255, 0), -1, cv2.LINE_AA)


def analyze_cam(cube: AprilTagCubeTarget, img: np.ndarray, K: np.ndarray, D: np.ndarray,
                max_err: float, min_aspect: float, min_markers: int, color) -> dict:
    out = img.copy()
    corners_list, ids = cube.detect(img)
    detected_ids = sorted(set(int(x) for x in ids)) if ids is not None else []
    ok, rvec, tvec, used, reproj = cube.solve_pnp_cube(
        img, K, D, use_ransac=True, min_markers=max(int(min_markers), 1),
        reproj_thr_mean_px=float(max_err), return_reproj=True, min_aspect=float(min_aspect))
    draw_cube_overlay(out, cube, K, D, corners_list, ids,
                      rvec if ok else None, tvec if ok else None,
                      reproj if ok else None, color=color)
    T_C_O = None
    err_mean = None
    if ok and reproj is not None:
        from apriltag_cube import rodrigues_to_Rt
        T_C_O = rodrigues_to_Rt(rvec, tvec)
        err_mean = float(reproj.get("err_mean", float("nan")))
    return {
        "overlay": out,
        "detected_ids": detected_ids,
        "used_ids": sorted(int(x) for x in used) if used else [],
        "pnp_ok": bool(ok),
        "T_C_O": T_C_O,
        "reproj_err_mean_px": err_mean,
    }


# ---------------------------------------------------------------------------
# Cross-camera merge (base frame)
# ---------------------------------------------------------------------------
def compute_merge(cam_results: Dict[int, dict],
                  T_base_Ci: Dict[int, np.ndarray],
                  fixed_ids: List[int]) -> dict:
    """고정 카메라들의 큐브 pose를 base 좌표계로 올려 산포를 계산."""
    base_poses: Dict[int, np.ndarray] = {}
    for ci in fixed_ids:
        r = cam_results.get(ci)
        if r is None or not r["pnp_ok"] or r["T_C_O"] is None or ci not in T_base_Ci:
            continue
        base_poses[ci] = T_base_Ci[ci] @ r["T_C_O"]

    merge = {"base_poses": base_poses, "n": len(base_poses), "per_cam": {}}
    if len(base_poses) < 2:
        merge["ok"] = None
        return merge

    T_list = list(base_poses.values())
    T_mean = robust_se3_average(T_list)
    t_mean = T_mean[:3, 3]
    trans_errs, rot_errs = [], []
    for ci, T in base_poses.items():
        dt_mm = float(np.linalg.norm(T[:3, 3] - t_mean) * 1000.0)
        dr_deg = rotation_error_deg(T[:3, :3], T_mean[:3, :3])
        merge["per_cam"][ci] = {"dt_mm": dt_mm, "dr_deg": dr_deg,
                                "xyz_mm": (T[:3, 3] * 1000.0).tolist()}
        trans_errs.append(dt_mm)
        rot_errs.append(dr_deg)
    merge["T_mean"] = T_mean
    merge["mean_center_mm"] = (t_mean * 1000.0).tolist()
    merge["trans_spread_mean_mm"] = float(np.mean(trans_errs))
    merge["trans_spread_max_mm"] = float(np.max(trans_errs))
    merge["rot_spread_mean_deg"] = float(np.mean(rot_errs))
    merge["rot_spread_max_deg"] = float(np.max(rot_errs))
    merge["ok"] = bool(merge["trans_spread_max_mm"] < 5.0)
    return merge


# ---------------------------------------------------------------------------
# Rendering
# ---------------------------------------------------------------------------
def label_bar(w: int, text: str, color, h: int = 26) -> np.ndarray:
    bar = np.zeros((h, w, 3), np.uint8)
    cv2.putText(bar, text, (6, 19), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)
    return bar


def make_grid(cam_results: Dict[int, dict], order: List[int], gripper_idx: Optional[int],
              merge: dict, cell=(480, 360)) -> np.ndarray:
    cw, ch = cell
    cells = []
    for ci in order:
        r = cam_results.get(ci)
        color = CAM_COLORS.get(ci, DEFAULT_COLOR)
        if r is None or r["overlay"] is None:
            img = np.full((ch, cw, 3), 40, np.uint8)
            cv2.putText(img, "no frame", (cw // 2 - 60, ch // 2),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        else:
            img = cv2.resize(r["overlay"], (cw, ch))
        role = "GRIPPER" if ci == gripper_idx else "FIXED"
        parts = [f"cam{ci} [{role}]"]
        if r is not None:
            parts.append(f"ids={r.get('detected_ids', [])}")
            if r.get("pnp_ok"):
                err = r.get("reproj_err_mean_px")
                parts.append(f"reproj={err:.2f}px" if err is not None else "reproj=?")
                if ci in merge.get("per_cam", {}):
                    parts.append(f"base dt={merge['per_cam'][ci]['dt_mm']:.1f}mm")
            else:
                parts.append("no cube")
        cell_img = cv2.vconcat([label_bar(cw, "  ".join(parts), color), img])
        cells.append(cell_img)

    while len(cells) < 4:
        cells.append(np.zeros((ch + 26, cw, 3), np.uint8))
    top = cv2.hconcat([cells[0], cells[1]])
    bot = cv2.hconcat([cells[2], cells[3]])
    return cv2.vconcat([top, bot])


def render_topview(merge: dict, T_base_O_ref: Optional[np.ndarray],
                   size: int = 560, margin: int = 60) -> np.ndarray:
    """base 좌표계 XY 평면 탑뷰: 각 고정 카메라의 큐브 중심을 점으로."""
    canvas = np.full((size, size, 3), 255, np.uint8)
    base_poses = merge.get("base_poses", {})
    pts = []  # (label, x_mm, y_mm, z_mm, color)
    for ci, T in base_poses.items():
        pts.append((f"cam{ci}", T[0, 3] * 1000.0, T[1, 3] * 1000.0, T[2, 3] * 1000.0,
                    CAM_COLORS.get(ci, DEFAULT_COLOR)))
    if T_base_O_ref is not None:
        pts.append(("calib", T_base_O_ref[0, 3] * 1000.0, T_base_O_ref[1, 3] * 1000.0,
                    T_base_O_ref[2, 3] * 1000.0, (120, 120, 120)))
    z_mean = float(np.mean([p[3] for p in pts])) if pts else 0.0

    if not pts:
        cv2.putText(canvas, "no fixed-camera cube in base frame", (20, size // 2),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        return canvas

    xs = [p[1] for p in pts]
    ys = [p[2] for p in pts]
    cx, cy = np.mean(xs), np.mean(ys)
    span = max(np.ptp(xs), np.ptp(ys), 20.0) * 1.6  # 최소 20mm 창, 여유 1.6배
    scale = (size - 2 * margin) / span

    def to_px(x, y):
        px = int(margin + (x - (cx - span / 2)) * scale)
        py = int(size - margin - (y - (cy - span / 2)) * scale)  # y up
        return px, py

    # 스케일 바 (10mm)
    bar_mm = 10.0
    cv2.line(canvas, (margin, size - 25), (margin + int(bar_mm * scale), size - 25), (0, 0, 0), 2)
    cv2.putText(canvas, f"{bar_mm:.0f}mm", (margin, size - 32),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
    cv2.putText(canvas, "base frame top view (X right, Y up)  [dz = Z dev vs mean]",
                (12, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)

    # 평균점
    mpx, mpy = to_px(cx, cy)
    cv2.drawMarker(canvas, (mpx, mpy), (0, 0, 0), cv2.MARKER_CROSS, 16, 2)

    for k, (label, x, y, z, color) in enumerate(pts):
        px, py = to_px(x, y)
        if label == "calib":
            cv2.drawMarker(canvas, (px, py), color, cv2.MARKER_STAR, 16, 2)
        else:
            cv2.circle(canvas, (px, py), 7, color, -1, cv2.LINE_AA)
        # 점이 겹쳐도 라벨이 안 겹치게 세로로 분리 + Z편차(dz) 표기
        dz = z - z_mean
        txt = f"{label} dz={dz:+.1f}mm"
        tw = cv2.getTextSize(txt, cv2.FONT_HERSHEY_SIMPLEX, 0.45, 1)[0][0]
        tx = px + 10 if px + 10 + tw <= size else px - 10 - tw  # 우측 끝이면 왼쪽에
        cv2.putText(canvas, txt, (tx, py + 4 + k * 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1, cv2.LINE_AA)

    # 산포 텍스트
    y0 = size - 70
    if merge.get("ok") is not None:
        verdict = "MERGED (one cube)" if merge["ok"] else "SCATTERED - check calib"
        vcol = (0, 150, 0) if merge["ok"] else (0, 0, 255)
        cv2.putText(canvas, verdict, (12, y0), cv2.FONT_HERSHEY_SIMPLEX, 0.6, vcol, 2)
        cv2.putText(canvas,
                    f"spread trans mean/max: {merge['trans_spread_mean_mm']:.1f}/{merge['trans_spread_max_mm']:.1f} mm",
                    (12, y0 + 22), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
        cv2.putText(canvas,
                    f"spread rot  mean/max: {merge['rot_spread_mean_deg']:.2f}/{merge['rot_spread_max_deg']:.2f} deg",
                    (12, y0 + 42), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
    else:
        cv2.putText(canvas, f"need 2+ fixed cams seeing cube (have {merge.get('n', 0)})",
                    (12, y0), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255), 2)
    return canvas


def print_report(merge: dict, cam_results: Dict[int, dict], gripper_idx: Optional[int]) -> None:
    print("\n===== MULTI-CAM CUBE CONSISTENCY =====")
    for ci in sorted(cam_results.keys()):
        r = cam_results[ci]
        role = "GRIPPER" if ci == gripper_idx else "FIXED"
        s = f"  cam{ci} [{role}] ids={r['detected_ids']}"
        if r["pnp_ok"]:
            s += f" reproj={r['reproj_err_mean_px']:.2f}px"
            if ci in merge.get("per_cam", {}):
                pc = merge["per_cam"][ci]
                s += f"  base dt={pc['dt_mm']:.2f}mm dr={pc['dr_deg']:.2f}deg  xyz={[round(v,1) for v in pc['xyz_mm']]}"
        else:
            s += " (no cube)"
        print(s)
    if merge.get("ok") is not None:
        print(f"  --> fixed cams in base frame: {merge['n']}")
        print(f"      trans spread mean/max: {merge['trans_spread_mean_mm']:.2f} / {merge['trans_spread_max_mm']:.2f} mm")
        print(f"      rot   spread mean/max: {merge['rot_spread_mean_deg']:.2f} / {merge['rot_spread_max_deg']:.2f} deg")
        print(f"      판정: {'합쳐짐(one cube, <5mm)' if merge['ok'] else '흩어짐(>5mm) - 캘리브레이션/큐브 확인'}")
    else:
        print(f"  --> 고정 카메라 2대 이상이 동시에 큐브를 봐야 병합 가능 (현재 {merge.get('n', 0)}대)")
    print("======================================\n")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main() -> int:
    ap = argparse.ArgumentParser(description="다중 카메라 큐브 일관성 라이브 테스트")
    ap.add_argument("--intrinsics_dir", default=DEFAULT_INTR,
                    help="cam{i}.npz + device_map.json 폴더")
    ap.add_argument("--calib_dir", default=DEFAULT_CALIB,
                    help="Step3 산출물 T_base_C{i}.npy 폴더")
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=480)
    ap.add_argument("--fps", type=int, default=15)
    ap.add_argument("--once", action="store_true", help="창 없이 한 번 측정/저장 후 종료")
    ap.add_argument("--max-err", type=float, default=8.0)
    ap.add_argument("--min-aspect", type=float, default=0.3)
    ap.add_argument("--min-markers", type=int, default=1)
    ap.add_argument("--no-reset", action="store_true", help="시작 시 하드웨어 리셋 건너뛰기")
    ap.add_argument("--out", default="multicam_test_out")
    args = ap.parse_args()

    # 큐브 정의 정합성 먼저
    cfg = get_default_cube_config()
    cfg_ok, problems = validate_cube_config(cfg)
    print_cube_sanity_check(cfg)
    if not cfg_ok:
        print("[ERROR] 큐브 정의 자기모순:", problems)
        return 1

    intr_dir = os.path.abspath(args.intrinsics_dir)
    calib_dir = os.path.abspath(args.calib_dir)
    print(f"\nintrinsics_dir: {intr_dir}\ncalib_dir     : {calib_dir}")
    if not os.path.isdir(intr_dir):
        print(f"[ERROR] intrinsics_dir 없음: {intr_dir}")
        return 1

    serial_to_idx, gripper_idx = load_device_map(intr_dir)
    print(f"serial->idx: {serial_to_idx}  gripper_cam_idx: {gripper_idx}")

    fixed_ids = sorted(i for i in serial_to_idx.values() if i != gripper_idx)
    T_base_Ci = load_base_extrinsics(calib_dir, fixed_ids)
    T_base_O_ref = load_optional_T(calib_dir, "T_base_O.npy")
    print(f"고정 카메라: {fixed_ids}  |  T_base_C 로드됨: {sorted(T_base_Ci.keys())}"
          f"  |  T_base_O ref: {'있음' if T_base_O_ref is not None else '없음'}")
    if not T_base_Ci:
        print(f"[ERROR] {calib_dir} 에 T_base_C*.npy 가 없습니다. Step3 캘리브레이션을 먼저 수행하세요.")
        return 1

    try:
        from camera import RealSenseCamera
    except Exception as e:
        print(f"[ERROR] 카메라 모듈 로드 실패 (pyrealsense2?): {e}")
        return 1

    devices = RealSenseCamera.list_devices()
    print("\n연결된 장치:")
    for s, name in devices.items():
        idx = serial_to_idx.get(s, "?")
        print(f"  cam{idx}: {s} ({name})")

    # 매핑에 있는 & 실제 연결된 카메라만
    idx_serial = sorted(((serial_to_idx[s], s) for s in devices if s in serial_to_idx),
                        key=lambda x: x[0])
    if not idx_serial:
        print("[ERROR] device_map 과 일치하는 연결 장치가 없습니다.")
        return 1

    # intrinsics + 큐브
    cube = AprilTagCubeTarget(cfg)
    K_map, D_map = {}, {}
    for ci, _ in idx_serial:
        K_map[ci], D_map[ci] = load_intrinsics(intr_dir, ci)

    if not args.no_reset:
        RealSenseCamera.reset_all_devices()

    cams: Dict[int, RealSenseCamera] = {}
    for ci, serial in idx_serial:
        cam = RealSenseCamera(serial=serial, width=args.width, height=args.height,
                              fps=args.fps, use_color=True, use_depth=False, warmup_frames=10)
        cam.start()
        cams[ci] = cam
    print(f"\n{len(cams)}대 카메라 시작 완료. 큐브를 고정 카메라들이 함께 보이는 위치에 두세요.")

    os.makedirs(args.out, exist_ok=True)
    stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    order = sorted(cams.keys())

    def grab_synced() -> Dict[int, Optional[np.ndarray]]:
        ref_ci = fixed_ids[0] if fixed_ids and fixed_ids[0] in cams else order[0]
        _, _, ref_ts = cams[ref_ci].get_latest()
        frames = {}
        for ci, cam in cams.items():
            if ref_ts is not None:
                c, _, _ = cam.get_at(ref_ts)
            else:
                c, _, _ = cam.get_latest()
            frames[ci] = c
        return frames

    def measure() -> Tuple[Dict[int, dict], dict]:
        frames = grab_synced()
        cam_results = {}
        for ci in order:
            img = frames.get(ci)
            if img is None:
                cam_results[ci] = {"overlay": None, "detected_ids": [], "used_ids": [],
                                   "pnp_ok": False, "T_C_O": None, "reproj_err_mean_px": None}
                continue
            cam_results[ci] = analyze_cam(cube, img, K_map[ci], D_map[ci],
                                          args.max_err, args.min_aspect, args.min_markers,
                                          CAM_COLORS.get(ci, DEFAULT_COLOR))
        merge = compute_merge(cam_results, T_base_Ci, fixed_ids)
        return cam_results, merge

    def save(cam_results, merge):
        base = os.path.join(args.out, f"multicam_{stamp}")
        grid = make_grid(cam_results, order, gripper_idx, merge)
        top = render_topview(merge, T_base_O_ref)
        cv2.imwrite(base + "_grid.png", grid)
        cv2.imwrite(base + "_topview.png", top)
        report = {
            "n_fixed_in_base": merge.get("n", 0),
            "trans_spread_mean_mm": merge.get("trans_spread_mean_mm"),
            "trans_spread_max_mm": merge.get("trans_spread_max_mm"),
            "rot_spread_mean_deg": merge.get("rot_spread_mean_deg"),
            "rot_spread_max_deg": merge.get("rot_spread_max_deg"),
            "merged": merge.get("ok"),
            "per_cam": {str(k): v for k, v in merge.get("per_cam", {}).items()},
        }
        with open(base + "_report.json", "w") as f:
            json.dump(report, f, indent=2, ensure_ascii=False)
        print(f"저장됨: {base}_grid.png / _topview.png / _report.json")

    try:
        if args.once:
            for _ in range(40):
                cam_results, merge = measure()
                if merge.get("n", 0) >= 2 or any(r["pnp_ok"] for r in cam_results.values()):
                    break
                time.sleep(0.1)
            print_report(merge, cam_results, gripper_idx)
            save(cam_results, merge)
            return 0

        print("\n[라이브] s: 저장   SPACE: 리포트 출력   q/ESC: 종료")
        while True:
            cam_results, merge = measure()
            grid = make_grid(cam_results, order, gripper_idx, merge)
            top = render_topview(merge, T_base_O_ref)
            cv2.imshow("cameras (2x2)", grid)
            cv2.imshow("base frame top view", top)
            key = cv2.waitKey(30) & 0xFF
            if key in (ord('q'), 27):
                break
            elif key == ord('s'):
                save(cam_results, merge)
            elif key == ord(' '):
                print_report(merge, cam_results, gripper_idx)
        cv2.destroyAllWindows()
        return 0
    finally:
        for cam in cams.values():
            cam.stop()


if __name__ == "__main__":
    raise SystemExit(main())
