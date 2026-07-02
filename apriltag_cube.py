# apriltag_cube.py
"""AprilTag cube geometry, detection, and solvePnP for the 59mm marker cube.

This file intentionally keeps only the calibration-critical path:
  detect tags -> build 2D/3D correspondences -> solve PnP -> report reprojection.
Depth-based pose scoring and legacy meta inference were removed to keep the
cube definition easy to audit.
"""

from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Tuple

import cv2
import numpy as np

from config import CubeConfig, get_default_cube_config


def rodrigues_to_Rt(rvec, tvec) -> np.ndarray:
    """OpenCV rvec/tvec -> 4x4 T_camera_object."""
    R, _ = cv2.Rodrigues(np.asarray(rvec, dtype=np.float64).reshape(3, 1))
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = R
    T[:3, 3] = np.asarray(tvec, dtype=np.float64).reshape(3)
    return T


def inv_T(T: np.ndarray) -> np.ndarray:
    """Inverse of a 4x4 rigid transform."""
    T = np.asarray(T, dtype=np.float64).reshape(4, 4)
    out = np.eye(4, dtype=np.float64)
    out[:3, :3] = T[:3, :3].T
    out[:3, 3] = -T[:3, :3].T @ T[:3, 3]
    return out


def rot_axis_angle(axis: np.ndarray, angle_rad: float) -> np.ndarray:
    axis = np.asarray(axis, dtype=np.float64).reshape(3)
    axis = axis / (np.linalg.norm(axis) + 1e-12)
    K = np.array(
        [[0.0, -axis[2], axis[1]], [axis[2], 0.0, -axis[0]], [-axis[1], axis[0], 0.0]],
        dtype=np.float64,
    )
    return np.eye(3, dtype=np.float64) + np.sin(angle_rad) * K + (1.0 - np.cos(angle_rad)) * (K @ K)


@dataclass
class MarkerObservation:
    marker_id: int
    face_name: str
    corners_2d: np.ndarray
    corners_2d_reordered: np.ndarray
    obj_corners_3d: np.ndarray
    aspect_ratio: float
    T_object_marker: np.ndarray


@dataclass
class CubeObservationSet:
    marker_observations: List[MarkerObservation]

    def image_bbox(self, pad_px: float = 0.0) -> Optional[Tuple[int, int, int, int]]:
        if not self.marker_observations:
            return None
        pts = np.concatenate([obs.corners_2d for obs in self.marker_observations], axis=0)
        return (
            int(np.floor(np.min(pts[:, 0]) - pad_px)),
            int(np.floor(np.min(pts[:, 1]) - pad_px)),
            int(np.ceil(np.max(pts[:, 0]) + pad_px)),
            int(np.ceil(np.max(pts[:, 1]) + pad_px)),
        )


def depth_metrics_to_fields(metrics: Optional[Dict[str, Any]]) -> Dict[str, Any]:
    """Compatibility helper for older CSV/log code. Depth is not used here."""
    metrics = metrics or {}
    return {
        "depth_valid": bool(metrics.get("valid", False)),
        "depth_num_samples": int(metrics.get("num_samples", 0)),
        "depth_num_markers": int(metrics.get("num_markers", 0)),
        "depth_support_marker_ids": [int(x) for x in metrics.get("support_marker_ids", [])],
        "depth_plane_mean_mm": metrics.get("plane_mean_mm"),
        "depth_plane_median_mm": metrics.get("plane_median_mm"),
        "depth_plane_max_mm": metrics.get("plane_max_mm"),
        "depth_z_mean_mm": metrics.get("z_mean_mm"),
        "depth_z_median_mm": metrics.get("z_median_mm"),
        "depth_inlier_ratio": metrics.get("inlier_ratio"),
    }


class AprilTagCubeModel:
    """3D model of the cube with per-marker sizes and centers. Units: meters."""

    def __init__(self, cfg: CubeConfig):
        self.cfg = cfg
        d = float(cfg.cube_side_m) / 2.0
        # face -> (fallback center, local +x/u axis, local +y/v axis, outward normal)
        self.face_defs = {
            "+Z": (np.array([0, 0, d]), np.array([1, 0, 0]), np.array([0, 1, 0]), np.array([0, 0, 1])),
            "-Z": (np.array([0, 0, -d]), np.array([1, 0, 0]), np.array([0, 1, 0]), np.array([0, 0, -1])),
            "+X": (np.array([d, 0, 0]), np.array([0, 0, -1]), np.array([0, 1, 0]), np.array([1, 0, 0])),
            "-X": (np.array([-d, 0, 0]), np.array([0, 0, 1]), np.array([0, 1, 0]), np.array([-1, 0, 0])),
            "+Y": (np.array([0, d, 0]), np.array([1, 0, 0]), np.array([0, 0, -1]), np.array([0, 1, 0])),
            "-Y": (np.array([0, -d, 0]), np.array([1, 0, 0]), np.array([0, 0, 1]), np.array([0, -1, 0])),
        }

    def marker_size(self, marker_id: int) -> float:
        return float(getattr(self.cfg, "marker_size_by_id", {}).get(int(marker_id), self.cfg.marker_size_m))

    def local_corners_for(self, marker_id: int) -> np.ndarray:
        s = self.marker_size(marker_id) / 2.0
        return np.array([[s, -s, 0.0], [-s, -s, 0.0], [-s, s, 0.0], [s, s, 0.0]], dtype=np.float64)

    def has_marker(self, marker_id: int) -> bool:
        mid = int(marker_id)
        return mid in getattr(self.cfg, "marker_pose_4x4", {}) or mid in self.cfg.id_to_face

    def marker_face_name(self, marker_id: int) -> str:
        mid = int(marker_id)
        return str(self.cfg.id_to_face.get(mid, f"marker_{mid}"))

    def marker_pose_in_rig(self, marker_id: int) -> np.ndarray:
        mid = int(marker_id)
        explicit = getattr(self.cfg, "marker_pose_4x4", {})
        if mid in explicit:
            return np.asarray(explicit[mid], dtype=np.float64).reshape(4, 4)

        face = self.cfg.id_to_face[mid]
        fallback_center, u, v, n = self.face_defs[face]
        center = np.asarray(getattr(self.cfg, "marker_center_m", {}).get(mid, fallback_center), dtype=np.float64).reshape(3)

        inset = float(getattr(self.cfg, "marker_inset_m", 0.0) or 0.0)
        if inset:
            center = center - inset * (n / (np.linalg.norm(n) + 1e-12))

        R_roll = rot_axis_angle(n, np.deg2rad(float(self.cfg.face_roll_deg.get(mid, 0.0))))
        u = R_roll @ u.astype(np.float64)
        v = R_roll @ v.astype(np.float64)
        n = np.cross(u, v)

        T = np.eye(4, dtype=np.float64)
        T[:3, 0] = u / (np.linalg.norm(u) + 1e-12)
        T[:3, 1] = v / (np.linalg.norm(v) + 1e-12)
        T[:3, 2] = n / (np.linalg.norm(n) + 1e-12)
        T[:3, 3] = center
        return T

    def marker_corners_in_rig(self, marker_id: int) -> np.ndarray:
        T = self.marker_pose_in_rig(marker_id)
        pts_h = np.c_[self.local_corners_for(marker_id), np.ones(4, dtype=np.float64)]
        return (T @ pts_h.T).T[:, :3].astype(np.float64)

    def reorder_image_corners(self, marker_id: int, img_pts: np.ndarray) -> np.ndarray:
        pts = np.asarray(img_pts, dtype=np.float64).reshape(4, 2)
        order = getattr(self.cfg, "corner_reorder", {}).get(int(marker_id))
        return pts[list(order)] if order is not None else pts

    def marker_visibility_score(self, marker_id: int, T_C_O: np.ndarray) -> Tuple[bool, float]:
        T_O_M = self.marker_pose_in_rig(marker_id)
        center_cam = T_C_O[:3, :3] @ T_O_M[:3, 3] + T_C_O[:3, 3]
        normal_cam = T_C_O[:3, :3] @ T_O_M[:3, 2]
        score = float(np.dot(normal_cam, -center_cam))
        return score > 0.0, score


class AprilTagCubeTarget:
    """Detect configured tags and estimate T_camera_object using solvePnP."""

    def __init__(self, cfg: CubeConfig):
        self.cfg = cfg
        self.model = AprilTagCubeModel(cfg)
        dictionary_id = getattr(cv2.aruco, cfg.dictionary_name)
        self.dictionary = cv2.aruco.getPredefinedDictionary(dictionary_id)
        try:
            self.params = cv2.aruco.DetectorParameters()
            self.detector = cv2.aruco.ArucoDetector(self.dictionary, self.params)
            self._new_api = True
        except AttributeError:
            self.params = cv2.aruco.DetectorParameters_create()
            self.detector = None
            self._new_api = False

    def detect(self, bgr: np.ndarray) -> Tuple[List[np.ndarray], Optional[np.ndarray]]:
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY) if bgr.ndim == 3 else bgr
        if self._new_api:
            corners, ids, _ = self.detector.detectMarkers(gray)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(gray, self.dictionary, parameters=self.params)
        return ([], None) if ids is None else (corners, ids.flatten().astype(int))

    def collect_observations(
        self,
        bgr: np.ndarray,
        only_ids: Optional[List[int]] = None,
        min_aspect: float = 0.0,
    ) -> CubeObservationSet:
        corners_list, ids = self.detect(bgr)
        allowed = {int(x) for x in only_ids} if only_ids is not None else None
        observations: List[MarkerObservation] = []
        if ids is None:
            return CubeObservationSet([])

        for corners, mid in zip(corners_list, ids):
            mid = int(mid)
            if not self.model.has_marker(mid) or (allowed is not None and mid not in allowed):
                continue
            raw = np.asarray(corners, dtype=np.float64).reshape(4, 2)
            img = self.model.reorder_image_corners(mid, raw)
            edge_a = float(np.linalg.norm(img[1] - img[0]))
            edge_b = float(np.linalg.norm(img[3] - img[0]))
            aspect = min(edge_a, edge_b) / (max(edge_a, edge_b) + 1e-6)
            if min_aspect > 0.0 and aspect < min_aspect:
                continue
            observations.append(MarkerObservation(
                marker_id=mid,
                face_name=self.model.marker_face_name(mid),
                corners_2d=raw,
                corners_2d_reordered=img,
                obj_corners_3d=self.model.marker_corners_in_rig(mid),
                aspect_ratio=aspect,
                T_object_marker=self.model.marker_pose_in_rig(mid),
            ))
        return CubeObservationSet(observations)

    def build_correspondences(
        self,
        corners_list,
        ids,
        min_markers: int = 1,
        only_ids: Optional[List[int]] = None,
        min_aspect: float = 0.3,
    ):
        allowed = {int(x) for x in only_ids} if only_ids is not None else None
        obj_pts, img_pts, used = [], [], []
        if ids is None:
            return None, None, []

        for corners, mid in zip(corners_list, ids):
            mid = int(mid)
            if not self.model.has_marker(mid) or (allowed is not None and mid not in allowed):
                continue
            img = self.model.reorder_image_corners(mid, np.asarray(corners).reshape(4, 2))
            edge_a = float(np.linalg.norm(img[1] - img[0]))
            edge_b = float(np.linalg.norm(img[3] - img[0]))
            aspect = min(edge_a, edge_b) / (max(edge_a, edge_b) + 1e-6)
            if min_aspect > 0.0 and aspect < min_aspect:
                continue
            obj_pts.append(self.model.marker_corners_in_rig(mid))
            img_pts.append(img)
            used.append(mid)

        if len(used) < int(min_markers):
            return None, None, used
        return (
            np.concatenate(obj_pts, axis=0).reshape(-1, 1, 3).astype(np.float64),
            np.concatenate(img_pts, axis=0).reshape(-1, 1, 2).astype(np.float64),
            used,
        )

    def single_marker_ippe_candidates(self, marker_id: int, img_pts: np.ndarray, K: np.ndarray, D: np.ndarray, **_) -> List[Dict[str, Any]]:
        """Return both IPPE solutions for a single visible marker, ranked by visibility + reprojection."""
        if not self.model.has_marker(marker_id):
            return []
        obj = self.model.marker_corners_in_rig(marker_id).reshape(-1, 1, 3)
        img = self.model.reorder_image_corners(marker_id, img_pts).reshape(-1, 1, 2)
        ret, rvecs, tvecs, reproj_errs = cv2.solvePnPGeneric(obj, img, K, D, flags=cv2.SOLVEPNP_IPPE)
        if int(ret) <= 0:
            return []

        candidates = []
        for i, (rvec, tvec) in enumerate(zip(rvecs, tvecs)):
            rvec = np.asarray(rvec, dtype=np.float64).reshape(3, 1)
            tvec = np.asarray(tvec, dtype=np.float64).reshape(3, 1)
            proj, _ = cv2.projectPoints(obj.reshape(-1, 3), rvec, tvec, K, D)
            proj = proj.reshape(-1, 2)
            err = np.linalg.norm(proj - img.reshape(-1, 2), axis=1)
            T_C_O = rodrigues_to_Rt(rvec, tvec)
            z_ok = float(tvec[2, 0]) > 0.0
            vis_ok, vis_score = self.model.marker_visibility_score(marker_id, T_C_O)
            tier = 0 if (z_ok and vis_ok) else (1 if z_ok else 2)
            err_mean = float(reproj_errs[i][0]) if reproj_errs is not None else float(np.mean(err))
            candidates.append({
                "solution_index": int(i),
                "rvec": rvec,
                "tvec": tvec,
                "proj2": proj,
                "err": err,
                "err_mean": err_mean,
                "T_C_O": T_C_O,
                "z_ok": z_ok,
                "vis_ok": vis_ok,
                "vis_score": float(vis_score),
                "visibility_tier": tier,
                "depth_metrics": {"valid": False},
                "rank": (tier, err_mean, -float(vis_score)),
            })
        return candidates

    def solve_pnp_cube(
        self,
        bgr,
        K,
        D,
        use_ransac: bool = True,
        min_markers: int = 1,
        reproj_thr_mean_px: float = 10.0,
        only_ids: Optional[List[int]] = None,
        return_reproj: bool = False,
        min_aspect: float = 0.3,
        depth_u16: Optional[np.ndarray] = None,  # kept for backward-compatible signature; ignored
        depth_scale: Optional[float] = None,    # kept for backward-compatible signature; ignored
    ):
        corners_list, ids = self.detect(bgr)
        obj_pts, img_pts, used = self.build_correspondences(corners_list, ids, min_markers, only_ids, min_aspect)
        if obj_pts is None:
            return (False, None, None, used, None) if return_reproj else (False, None, None, used)

        n_points = int(obj_pts.shape[0])
        if n_points == 4 and len(used) == 1:
            candidates = self.single_marker_ippe_candidates(used[0], img_pts.reshape(-1, 2), K, D)
            if not candidates:
                return (False, None, None, used, None) if return_reproj else (False, None, None, used)
            best = min(candidates, key=lambda x: x["rank"])
            rvec, tvec = best["rvec"], best["tvec"]
            proj2, err = best["proj2"], best["err"]
            extra = {k: best[k] for k in ("solution_index", "z_ok", "vis_ok", "vis_score", "visibility_tier", "depth_metrics")}
        else:
            flags = cv2.SOLVEPNP_ITERATIVE if n_points >= 8 else cv2.SOLVEPNP_IPPE
            if use_ransac and n_points >= 8:
                ok, rvec, tvec, _ = cv2.solvePnPRansac(
                    obj_pts, img_pts, K, D, flags=flags,
                    reprojectionError=5.0, iterationsCount=200, confidence=0.999,
                )
            else:
                ok, rvec, tvec = cv2.solvePnP(obj_pts, img_pts, K, D, flags=flags)
            if not ok:
                return (False, None, None, used, None) if return_reproj else (False, None, None, used)
            proj2, _ = cv2.projectPoints(obj_pts.reshape(-1, 3), rvec, tvec, K, D)
            proj2 = proj2.reshape(-1, 2)
            err = np.linalg.norm(proj2 - img_pts.reshape(-1, 2), axis=1)
            extra = {}

        err_mean = float(np.mean(err)) if err.size else float("inf")
        reproj = {
            "obj_pts": obj_pts,
            "img_pts": img_pts,
            "proj2": proj2,
            "err": err,
            "err_mean": err_mean,
            "err_median": float(np.median(err)) if err.size else float("inf"),
            "err_p90": float(np.percentile(err, 90)) if err.size else float("inf"),
            "n_points": int(err.size),
            "rvec": rvec,
            "tvec": tvec,
            **extra,
        }
        ok_final = bool(np.isfinite(err_mean) and err_mean <= float(reproj_thr_mean_px))
        return (ok_final, rvec, tvec, used, reproj) if return_reproj else (ok_final, rvec, tvec, used)


# Outward normal of each cube face in the object frame. A marker's local +Z
# axis (T_object_marker[:3, 2]) must equal the normal of the face it sits on.
FACE_OUTWARD_NORMAL: Dict[str, np.ndarray] = {
    "+X": np.array([1.0, 0.0, 0.0]),
    "-X": np.array([-1.0, 0.0, 0.0]),
    "+Y": np.array([0.0, 1.0, 0.0]),
    "-Y": np.array([0.0, -1.0, 0.0]),
    "+Z": np.array([0.0, 0.0, 1.0]),
    "-Z": np.array([0.0, 0.0, -1.0]),
}

# Which cartesian axis is "out of plane" for each face, and its sign.
_FACE_AXIS = {"+X": (0, +1), "-X": (0, -1), "+Y": (1, +1), "-Y": (1, -1), "+Z": (2, +1), "-Z": (2, -1)}


def charuco_marker_ids(charuco_cfg) -> set:
    """IDs an OpenCV CharucoBoard occupies: start .. start + floor(sx*sy/2) - 1."""
    count = (int(charuco_cfg.squares_x) * int(charuco_cfg.squares_y)) // 2
    start = int(getattr(charuco_cfg, "marker_id_start", 0))
    return set(range(start, start + count))


def validate_cube_config(
    cfg: Optional[CubeConfig] = None,
    charuco_cfg=None,
    tol_m: float = 1e-6,
) -> Tuple[bool, List[str]]:
    """Validate that the CubeConfig is physically self-consistent.

    Checks (each failure is appended to ``problems``):
      1. marker IDs are unique
      2. every marker size matches its face group (+Z top vs. side face)
      3. every marker center lies on its declared face plane
      4. each marker's local +Z axis equals the face outward normal
      5. no marker ID collides with the Charuco board ID range
      6. all four corners of every marker fall inside the cube face bounds

    Units are meters throughout. Returns ``(ok, problems)``.
    """
    from config import TOP_MARKER_SIZE_M, SIDE_MARKER_SIZE_M

    cfg = cfg or get_default_cube_config()
    model = AprilTagCubeModel(cfg)
    problems: List[str] = []

    d = float(cfg.cube_side_m) / 2.0
    inset = float(getattr(cfg, "marker_inset_m", 0.0) or 0.0)

    # 1) unique IDs
    ids = [int(m) for m in cfg.marker_ids]
    dupes = sorted({m for m in ids if ids.count(m) > 1})
    if dupes:
        problems.append(f"[unique] duplicate marker IDs: {dupes}")

    # 5) charuco collision (compute once)
    if charuco_cfg is None:
        try:
            from config import CharucoBoardConfig
            charuco_cfg = CharucoBoardConfig()
        except Exception:
            charuco_cfg = None
    charuco_ids = charuco_marker_ids(charuco_cfg) if charuco_cfg is not None else set()
    collide = sorted(set(ids) & charuco_ids)
    if collide:
        problems.append(
            f"[collision] cube IDs {collide} overlap Charuco range "
            f"[{min(charuco_ids)}..{max(charuco_ids)}]; raise CharucoBoardConfig.marker_id_start"
        )

    for mid in ids:
        face = cfg.id_to_face.get(mid)
        if face is None:
            problems.append(f"[face] id {mid} has no id_to_face entry")
            continue
        if face not in _FACE_AXIS:
            problems.append(f"[face] id {mid} has unknown face '{face}'")
            continue

        # 2) size matches face group
        size = model.marker_size(mid)
        expected_size = TOP_MARKER_SIZE_M if face == "+Z" else SIDE_MARKER_SIZE_M
        if abs(size - expected_size) > tol_m:
            problems.append(
                f"[size] id {mid} on face {face}: size {size:.4f}m != expected {expected_size:.4f}m"
            )

        pose = model.marker_pose_in_rig(mid)
        center = pose[:3, 3]
        normal = pose[:3, 2]

        # 3) center lies on the face plane (out-of-plane coord == +/-(d - inset))
        axis, sign = _FACE_AXIS[face]
        expected_coord = sign * (d - inset)
        if abs(center[axis] - expected_coord) > tol_m:
            problems.append(
                f"[center] id {mid} on face {face}: axis-{axis} coord {center[axis]:+.4f} "
                f"!= face plane {expected_coord:+.4f}"
            )

        # 4) marker normal == face outward normal
        if not np.allclose(normal, FACE_OUTWARD_NORMAL[face], atol=1e-6):
            problems.append(
                f"[normal] id {mid} on face {face}: marker +Z {np.round(normal, 4).tolist()} "
                f"!= face normal {FACE_OUTWARD_NORMAL[face].tolist()}"
            )

        # 6) all corners inside the cube bounding box (fit on the face)
        corners = model.marker_corners_in_rig(mid)
        if np.any(np.abs(corners) > d + tol_m):
            worst = float(np.max(np.abs(corners)))
            problems.append(
                f"[bounds] id {mid} on face {face}: a corner reaches {worst:.4f}m > half-cube {d:.4f}m"
            )

    return (len(problems) == 0), problems


def print_cube_sanity_check(cfg: Optional[CubeConfig] = None, charuco_cfg=None, tol_m: float = 1e-6) -> bool:
    cfg = cfg or get_default_cube_config()
    model = AprilTagCubeModel(cfg)
    ok, problems = validate_cube_config(cfg, charuco_cfg=charuco_cfg, tol_m=tol_m)

    print("=== AprilTag 59mm cube sanity check ===")
    print(f"cube_side_m = {cfg.cube_side_m:.6f}  dictionary = {cfg.dictionary_name}  "
          f"marker_inset_m = {getattr(cfg, 'marker_inset_m', 0.0):.4f}")
    print(f"{'id':>3} {'face':>4} {'size_m':>8}  {'center_m':>26}  {'normal':>14}")
    for mid in (int(m) for m in cfg.marker_ids):
        size = model.marker_size(mid)
        pose = model.marker_pose_in_rig(mid)
        c, n = pose[:3, 3], pose[:3, 2]
        print(f"{mid:>3} {cfg.id_to_face.get(mid, '?'):>4} {size:>8.4f}  "
              f"({c[0]:+.4f},{c[1]:+.4f},{c[2]:+.4f})  ({n[0]:+.0f},{n[1]:+.0f},{n[2]:+.0f})")
    if ok:
        print("RESULT: OK - cube config is physically self-consistent")
    else:
        print("RESULT: MISMATCH")
        for p in problems:
            print(f"  - {p}")
    return bool(ok)


if __name__ == "__main__":
    print_cube_sanity_check()
