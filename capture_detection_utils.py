from typing import Dict, Optional, Sequence

import cv2
import numpy as np


def filter_marker_ids(corners_list, ids, allowed_ids: Sequence[int]):
    if ids is None or len(ids) == 0:
        return [], None
    allowed = set(int(x) for x in allowed_ids)
    filt_corners = []
    filt_ids = []
    ids_flat = np.asarray(ids).reshape(-1)
    for corners, mid in zip(corners_list, ids_flat):
        if int(mid) in allowed:
            filt_corners.append(corners)
            filt_ids.append(int(mid))
    if not filt_ids:
        return [], None
    return filt_corners, np.asarray(filt_ids, dtype=np.int32)


def marker_roi_quality(bgr: np.ndarray,
                       quads: Sequence[np.ndarray],
                       clip_low: int = 2,
                       clip_high: int = 253) -> Dict[str, Optional[float]]:
    """Sharpness and clipping measured on the marker regions only.

    Whole-frame statistics gate on the wrong thing: a blown-out background or a
    blurred table costs nothing, while a crisp background can hide one smeared
    marker. Only the pixels the corner solver actually consumes matter, so each
    detected quad is measured on its own and the worst one decides the frame.

    ``sharpness`` is Laplacian variance over the quad's bounding box. It is not
    comparable across cameras — a marker imaged larger spreads its edge energy
    over more pixels — which is why the gate threshold is per camera and starts
    disabled until pilot data fixes it. ``clip_frac`` (fraction of pixels pinned
    at black or white) is scale-free and can be gated immediately.

    Returns worst-case and median values plus ``n_rois``; all-None when there is
    nothing to measure, which callers must treat as "unknown", not "good".
    """
    empty = {"sharpness_min": None, "sharpness_median": None,
             "clip_frac_median": None, "clip_frac_max": None,
             "roi_px_min": None, "n_rois": 0}
    if bgr is None or not len(quads):
        return empty

    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY) if bgr.ndim == 3 else bgr
    h, w = gray.shape[:2]

    sharps, clips, sizes = [], [], []
    for quad in quads:
        pts = np.asarray(quad, dtype=np.float32).reshape(-1, 2)
        if pts.shape[0] < 3:
            continue
        x0 = max(int(np.floor(pts[:, 0].min())), 0)
        y0 = max(int(np.floor(pts[:, 1].min())), 0)
        x1 = min(int(np.ceil(pts[:, 0].max())) + 1, w)
        y1 = min(int(np.ceil(pts[:, 1].max())) + 1, h)
        if x1 - x0 < 4 or y1 - y0 < 4:
            continue
        roi = gray[y0:y1, x0:x1]
        sharps.append(float(cv2.Laplacian(roi, cv2.CV_64F).var()))
        clips.append(float(np.mean((roi <= clip_low) | (roi >= clip_high))))
        sizes.append(float(min(x1 - x0, y1 - y0)))

    if not sharps:
        return empty
    return {
        "sharpness_min": min(sharps),
        "sharpness_median": float(np.median(sharps)),
        # Gate on the median, diagnose with the max: one specular highlight on
        # one marker should not discard a capture whose other markers are clean,
        # but a camera whose typical marker is blown out has no usable corners.
        "clip_frac_median": float(np.median(clips)),
        "clip_frac_max": max(clips),
        "roi_px_min": min(sizes),
        "n_rois": len(sharps),
    }


def _expand_quad(corners, pad_px: float) -> np.ndarray:
    pts = np.asarray(corners, dtype=np.float32).reshape(-1, 2)
    if pts.shape[0] != 4:
        return np.round(pts).astype(np.int32)
    if float(pad_px) <= 0:
        return np.round(pts).astype(np.int32)
    center = np.mean(pts, axis=0)
    radii = np.linalg.norm(pts - center[None, :], axis=1)
    mean_radius = float(np.mean(radii)) if radii.size > 0 else 1.0
    scale = 1.0 + float(pad_px) / max(mean_radius, 1.0)
    expanded = center[None, :] + (pts - center[None, :]) * scale
    return np.round(expanded).astype(np.int32)


def mask_board_marker_regions(bgr: np.ndarray,
                              board_marker_corners,
                              pad_px: float = 6.0,
                              fill_value: int = 127) -> np.ndarray:
    if bgr is None:
        return bgr
    if board_marker_corners is None or len(board_marker_corners) == 0:
        return bgr.copy()
    masked = bgr.copy()
    fill_color = (int(fill_value), int(fill_value), int(fill_value))
    for corners in board_marker_corners:
        quad = _expand_quad(corners, pad_px)
        if quad.shape[0] >= 3:
            cv2.fillConvexPoly(masked, quad, fill_color, lineType=cv2.LINE_AA)
    return masked


def detect_cube_markers_in_frame(bgr: np.ndarray,
                                 cube,
                                 cube_ids: Sequence[int],
                                 charuco=None,
                                 is_gripper: bool = False,
                                 board_mask_pad_px: float = 6.0) -> Dict[str, object]:
    board_mkr_corners = None
    board_mkr_ids = None
    ch_corners = None
    ch_ids = None
    charuco_detect_n = 0
    board_mask_applied = False

    cube_img = bgr
    if charuco is not None:
        # ChArUco 검출은 카메라 종류와 무관하게 수행 (보드-전용 비교실험을 위해
        # 고정카메라도 보드를 인식/저장). 단, 보드 마커 영역 마스킹은 그리퍼에서만:
        # 고정카메라의 큐브 검출 입력을 기존 큐브-전용 파이프라인과 동일하게 유지해
        # 대조군을 보존하기 위함 (보드 검출은 순수 '추가' 동작).
        try:
            ch_corners, ch_ids, charuco_detect_n, board_mkr_corners, board_mkr_ids = charuco.detect(bgr)
        except Exception:
            ch_corners, ch_ids, charuco_detect_n, board_mkr_corners, board_mkr_ids = None, None, 0, None, None
        if is_gripper and board_mkr_corners is not None and len(board_mkr_corners) > 0:
            cube_img = mask_board_marker_regions(bgr, board_mkr_corners, pad_px=board_mask_pad_px)
            board_mask_applied = True

    raw_corners, raw_ids = cube.detect(cube_img)
    raw_ids_list = [] if raw_ids is None else [int(x) for x in np.asarray(raw_ids).reshape(-1)]
    corners, ids = filter_marker_ids(raw_corners, raw_ids, cube_ids)
    filtered_ids_list = [] if ids is None else [int(x) for x in np.asarray(ids).reshape(-1)]

    return {
        "cube_image": cube_img,
        "corners": corners,
        "ids": ids,
        "raw_ids": raw_ids_list,
        "filtered_ids": filtered_ids_list,
        "board_mkr_corners": board_mkr_corners,
        "board_mkr_ids": board_mkr_ids,
        "ch_corners": ch_corners,
        "ch_ids": ch_ids,
        "charuco_detect_n": int(charuco_detect_n),
        "board_mask_applied": bool(board_mask_applied),
    }
