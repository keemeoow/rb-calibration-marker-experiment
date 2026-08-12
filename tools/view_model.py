# tools/view_model.py
"""Predict, offline, which cube markers each camera will see from a given pose.

This is the geometric core of automatic capture-pose generation: it replaces
"drive the robot there and find out" with a projection of the known cube model
through the known camera calibration.

Inputs, all already in the repo:
  - intrinsics/cam{0..3}.npz        color_K / color_D / color_w / color_h
  - <session>/calib_out/T_base_C*.npy   fixed camera extrinsics (base frame)
  - <session>/calib_out/T_gripper_cam.npy  eye-in-hand extrinsic
  - config.CubeConfig               cube marker geometry (single source of truth)

A marker counts as observable when it is front-facing, fully inside the image
with margin, large enough in pixels, and not too foreshortened - the same
conditions that make ``cube_pnp`` succeed on real data. Thresholds default to
values that reproduce the detections in data/session; see validate_view_model.py.
"""

from __future__ import annotations

import os
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence

import cv2
import numpy as np

from apriltag_cube import AprilTagCubeModel
from config import CubeConfig, get_default_cube_config


@dataclass
class VisibilityThresholds:
    min_incidence_cos: float = 0.30      # cos of angle between face normal and view ray
    image_margin_px: float = 12.0        # corners must sit this far inside the frame
    min_side_px: float = 14.0            # shortest projected edge of the tag
    min_aspect: float = 0.35             # min edge / max edge of the projected quad
    min_range_m: float = 0.20
    max_range_m: float = 2.00


@dataclass
class Camera:
    index: int
    K: np.ndarray
    D: np.ndarray
    width: int
    height: int
    is_gripper: bool
    T_base_cam: Optional[np.ndarray] = None   # fixed cams only
    T_gripper_cam: Optional[np.ndarray] = None  # gripper cam only

    def pose_in_base(self, T_base_gripper: Optional[np.ndarray] = None) -> np.ndarray:
        if not self.is_gripper:
            return self.T_base_cam
        if T_base_gripper is None:
            raise ValueError("gripper camera needs T_base_gripper")
        return T_base_gripper @ self.T_gripper_cam


def load_camera_rig(session_dir: str,
                    intrinsics_dir: str = "intrinsics",
                    gripper_cam_idx: int = 2,
                    cam_indices: Sequence[int] = (0, 1, 2, 3)) -> Dict[int, Camera]:
    """Load every camera that has both intrinsics and a usable extrinsic."""
    calib = os.path.join(session_dir, "calib_out")
    T_gc_path = os.path.join(calib, "T_gripper_cam.npy")
    T_gripper_cam = np.load(T_gc_path) if os.path.exists(T_gc_path) else None

    rig: Dict[int, Camera] = {}
    for ci in cam_indices:
        npz_path = os.path.join(intrinsics_dir, f"cam{ci}.npz")
        if not os.path.exists(npz_path):
            continue
        d = np.load(npz_path, allow_pickle=True)
        is_grip = int(ci) == int(gripper_cam_idx)
        T_bc = None
        if not is_grip:
            p = os.path.join(calib, f"T_base_C{ci}.npy")
            if not os.path.exists(p):
                continue
            T_bc = np.load(p)
        elif T_gripper_cam is None:
            continue
        rig[int(ci)] = Camera(
            index=int(ci),
            K=np.asarray(d["color_K"], dtype=np.float64),
            D=np.asarray(d["color_D"], dtype=np.float64).reshape(-1, 1),
            width=int(d["color_w"]),
            height=int(d["color_h"]),
            is_gripper=is_grip,
            T_base_cam=T_bc,
            T_gripper_cam=T_gripper_cam if is_grip else None,
        )
    return rig


def _quad_aspect(pts: np.ndarray) -> float:
    """min edge / max edge of a projected quad - matches CP_common.marker_aspect_ratio."""
    lens = [float(np.linalg.norm(pts[(i + 1) % 4] - pts[i])) for i in range(4)]
    return min(lens) / max(max(lens), 1e-12)


class CubeViewModel:
    """Projects the cube's markers into any calibrated camera."""

    def __init__(self, cube_cfg: Optional[CubeConfig] = None,
                 thresholds: Optional[VisibilityThresholds] = None):
        self.cfg = cube_cfg or get_default_cube_config()
        self.model = AprilTagCubeModel(self.cfg)
        self.th = thresholds or VisibilityThresholds()
        self.marker_ids = list(self.cfg.marker_ids)
        self._corners_obj = {mid: self.model.marker_corners_in_rig(mid) for mid in self.marker_ids}
        self._pose_obj = {mid: self.model.marker_pose_in_rig(mid) for mid in self.marker_ids}

    def observe(self, cam: Camera, T_base_cube: np.ndarray,
                T_base_gripper: Optional[np.ndarray] = None,
                occluded_ids: Sequence[int] = ()) -> Dict:
        """Predicted observation of the cube by one camera.

        Returns {"markers": [...], "n_visible": int, "faces": set, "range_m": float}
        with one entry per marker that passes every visibility test.
        """
        T_base_cam = cam.pose_in_base(T_base_gripper)
        T_cam_cube = np.linalg.inv(T_base_cam) @ T_base_cube
        R, t = T_cam_cube[:3, :3], T_cam_cube[:3, 3]
        occluded = {int(x) for x in occluded_ids}

        markers: List[Dict] = []
        for mid in self.marker_ids:
            if mid in occluded:
                continue
            T_O_M = self._pose_obj[mid]
            center_cam = R @ T_O_M[:3, 3] + t
            if center_cam[2] <= 0:
                continue
            rng = float(np.linalg.norm(center_cam))
            if not (self.th.min_range_m <= rng <= self.th.max_range_m):
                continue

            normal_cam = R @ T_O_M[:3, 2]
            view_dir = center_cam / max(rng, 1e-12)
            incidence_cos = float(-np.dot(normal_cam, view_dir))
            if incidence_cos < self.th.min_incidence_cos:
                continue

            pts_obj = self._corners_obj[mid]
            pts_cam = (R @ pts_obj.T).T + t
            if np.any(pts_cam[:, 2] <= 0):
                continue
            uv, _ = cv2.projectPoints(pts_cam.reshape(-1, 1, 3), np.zeros(3), np.zeros(3),
                                      cam.K, cam.D)
            uv = uv.reshape(4, 2)
            m = self.th.image_margin_px
            if (np.any(uv[:, 0] < m) or np.any(uv[:, 0] > cam.width - m) or
                    np.any(uv[:, 1] < m) or np.any(uv[:, 1] > cam.height - m)):
                continue

            edges = [float(np.linalg.norm(uv[(i + 1) % 4] - uv[i])) for i in range(4)]
            if min(edges) < self.th.min_side_px:
                continue
            aspect = _quad_aspect(uv)
            if aspect < self.th.min_aspect:
                continue

            markers.append({
                "marker_id": int(mid),
                "face": self.cfg.id_to_face[mid],
                "corners_2d": uv,
                "range_m": rng,
                "incidence_cos": incidence_cos,
                "aspect": aspect,
                "min_side_px": min(edges),
            })

        faces = {mk["face"] for mk in markers}
        return {
            "markers": markers,
            "n_visible": len(markers),
            "ids": sorted(mk["marker_id"] for mk in markers),
            "faces": faces,
            "side_faces": {f for f in faces if f != "+Z"},
            "range_m": float(np.linalg.norm(t)),
        }

    def observe_all(self, rig: Dict[int, Camera], T_base_cube: np.ndarray,
                    T_base_gripper: Optional[np.ndarray] = None,
                    occluded_ids: Sequence[int] = ()) -> Dict[int, Dict]:
        out = {}
        for ci, cam in rig.items():
            if cam.is_gripper and T_base_gripper is None:
                continue
            out[ci] = self.observe(cam, T_base_cube, T_base_gripper, occluded_ids)
        return out


# -----------------------------------------------------------------------------
# Observability of a *set* of poses - what actually determines calibration quality
# -----------------------------------------------------------------------------

def rotation_diversity(rotations: Sequence[np.ndarray]) -> Dict[str, float]:
    """Spread of pairwise relative-rotation axes on the sphere.

    Hand-eye calibration is only well-conditioned when the relative motions
    between poses rotate about clearly different axes by a decent angle. The
    returned ``axis_spread`` is the smallest eigenvalue of sum(a a^T) normalised
    to [0, 1]: 0 means every relative motion shares one axis (rank-deficient),
    1/3 means the axes are isotropic.
    """
    R = [np.asarray(r, dtype=float)[:3, :3] for r in rotations]
    axes, angles = [], []
    for i in range(len(R)):
        for j in range(i + 1, len(R)):
            Rij = R[j] @ R[i].T
            ang = float(np.arccos(np.clip((np.trace(Rij) - 1.0) / 2.0, -1.0, 1.0)))
            if ang < np.deg2rad(5.0):
                continue
            ax = np.array([Rij[2, 1] - Rij[1, 2], Rij[0, 2] - Rij[2, 0], Rij[1, 0] - Rij[0, 1]])
            n = np.linalg.norm(ax)
            if n < 1e-9:
                continue
            axes.append(ax / n)
            angles.append(np.rad2deg(ang))
    if len(axes) < 3:
        return {"axis_spread": 0.0, "median_angle_deg": 0.0, "n_pairs": len(axes)}
    A = np.asarray(axes)
    M = (A.T @ A) / len(A)
    return {
        "axis_spread": float(np.linalg.eigvalsh(M)[0] * 3.0),
        "median_angle_deg": float(np.median(angles)),
        "n_pairs": len(axes),
    }


def coverage_report(observations: Sequence[Dict[int, Dict]],
                    gripper_cam_idx: int = 2) -> Dict[str, float]:
    """Aggregate the metrics that data/session was weakest on."""
    n_multi = n_two_side = n_top = 0
    per_face: Dict[str, int] = {}
    aspects: List[float] = []
    for obs in observations:
        for ci, o in obs.items():
            if ci == gripper_cam_idx:
                continue
            if o["n_visible"] >= 3:
                n_multi += 1
            if len(o["side_faces"]) >= 2:
                n_two_side += 1
            if "+Z" in o["faces"]:
                n_top += 1
            for f in o["faces"]:
                per_face[f] = per_face.get(f, 0) + 1
            aspects += [mk["aspect"] for mk in o["markers"]]
    return {
        "frames_ge3_markers": n_multi,
        "frames_two_side_faces": n_two_side,
        "frames_top_visible": n_top,
        "per_face": per_face,
        "median_aspect": float(np.median(aspects)) if aspects else 0.0,
        "frac_aspect_ge_0p6": float(np.mean([a >= 0.6 for a in aspects])) if aspects else 0.0,
    }
