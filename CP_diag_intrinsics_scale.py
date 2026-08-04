"""Ad-hoc diagnostic: does the vision-vs-robot distance scale error come from the
ChArUco-recalibrated intrinsics (color_K) vs the factory intrinsics (factory_color_K)?

Method (image-free): each capture stores the 2D marker corners and the robot
gripper pose (tool3 grip point, ~13mm from the cube center). Re-run cube solvePnP
with each K set to get the cube center in the (fixed) camera frame, then fit a
similarity transform (Umeyama, scale+R+t) mapping the robot gripper trajectory
(base frame) onto the vision cube-center trajectory (camera frame). The recovered
scale s IS the vision/robot distance scale for that camera. s~=1.0 is correct;
whichever K set gives s nearest 1.0 (with small residual) is the right intrinsics.
"""
import os, sys, json
import numpy as np
import cv2

REPO = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, REPO)
from config import CubeConfig
from apriltag_cube import AprilTagCubeTarget


def build_session_cube_cfg(cc: dict) -> CubeConfig:
    """Reconstruct the cube config actually used at capture time. The session
    stores marker_size_by_id / marker_center_m as null (uniform marker on faces),
    so we must NOT inherit the config.py default's per-id 25/51mm sizes."""
    cfg = CubeConfig(
        cube_side_m=float(cc["cube_side_m"]),
        marker_size_m=float(cc["marker_size_m"]),
        dictionary_name=str(cc["dictionary_name"]),
        marker_ids=tuple(int(x) for x in cc["marker_ids"]),
    )
    cfg.id_to_face = {int(k): str(v) for k, v in cc["id_to_face"].items()}
    cfg.face_roll_deg = {int(k): float(v) for k, v in cc.get("face_roll_deg", {}).items()}
    cfg.corner_reorder = {int(k): tuple(v) for k, v in cc.get("corner_reorder", {}).items()}
    cfg.marker_pose_4x4 = {int(k): v for k, v in (cc.get("marker_pose_4x4") or {}).items()}
    cfg.marker_size_by_id = {}   # uniform -> fall back to marker_size_m
    cfg.marker_center_m = {}     # empty -> face-def centers at cube_side/2
    return cfg

SESSION = "/home/sprout/Desktop/**jiwoo/rb-ArucoCube_Robot_multi_calibration/data/session"
INTR = os.path.join(REPO, "intrinsics")
META = os.path.join(SESSION, "meta.json")

MIN_MARKERS = 2       # require >=2 markers -> 8 pts -> unambiguous non-planar PnP


def umeyama_scale(X, Y):
    """Similarity fit Y ~= s*R*X + t. X,Y: (N,3). Returns (s, rms_residual_mm)."""
    X = np.asarray(X, float); Y = np.asarray(Y, float)
    n = X.shape[0]
    muX, muY = X.mean(0), Y.mean(0)
    Xc, Yc = X - muX, Y - muY
    Sigma = (Yc.T @ Xc) / n
    U, Dsv, Vt = np.linalg.svd(Sigma)
    S = np.eye(3)
    if np.linalg.det(U) * np.linalg.det(Vt) < 0:
        S[2, 2] = -1
    R = U @ S @ Vt
    varX = (Xc ** 2).sum() / n
    s = float(np.trace(np.diag(Dsv) @ S) / varX)
    t = muY - s * R @ muX
    resid = Y - (s * (X @ R.T) + t)
    rms = float(np.sqrt((resid ** 2).sum(1).mean()))
    return s, rms


def load_K(cam, key):
    d = np.load(os.path.join(INTR, f"cam{cam}.npz"), allow_pickle=True)
    Kk = "factory_color_K" if key == "factory" else "color_K"
    Dk = "factory_color_D" if key == "factory" else "color_D"
    return d[Kk].astype(np.float64).reshape(3, 3), d[Dk].astype(np.float64).reshape(-1, 1)


def cube_tvec(target, markers, K, D):
    """Re-run cube PnP from stored raw corners. Returns tvec (m) in camera frame or None."""
    corners_list, ids = [], []
    for mk in markers:
        c = np.asarray(mk["corners_2d"], dtype=np.float64).reshape(4, 2)
        corners_list.append(c)
        ids.append(int(mk["marker_id"]))
    obj, img, used = target.build_correspondences(corners_list, ids, min_markers=MIN_MARKERS,
                                                  only_ids=None, min_aspect=0.3)
    if obj is None or len(used) < MIN_MARKERS:
        return None
    ok, rvec, tvec = cv2.solvePnP(obj, img, K, D, flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok:
        return None
    proj, _ = cv2.projectPoints(obj.reshape(-1, 3), rvec, tvec, K, D)
    err = float(np.mean(np.linalg.norm(proj.reshape(-1, 2) - img.reshape(-1, 2), axis=1)))
    if not np.isfinite(err) or err > 8.0:
        return None
    return tvec.reshape(3)


def rot_angle_deg(Ra, Rb):
    R = Ra.T @ Rb
    c = (np.trace(R) - 1.0) / 2.0
    return float(np.degrees(np.arccos(np.clip(c, -1.0, 1.0))))


def main():
    meta = json.load(open(META))
    gripper_cam = int(meta.get("gripper_cam_idx", -1))
    cam_ids = [int(c) for c in meta["cam_indices"]]
    fixed_cams = [c for c in cam_ids if c != gripper_cam]
    caps = meta["captures"]
    # IMPORTANT: this session uses a 30mm / 22mm DICT_4X4_50 cube, not the
    # config.py default (59mm AprilTag). Build the target from the session config.
    session_cfg = build_session_cube_cfg(meta["cube_config"])
    target = AprilTagCubeTarget(session_cfg)
    print(f"cube: side={session_cfg.cube_side_m*1000:.0f}mm marker={session_cfg.marker_size_m*1000:.0f}mm "
          f"dict={session_cfg.dictionary_name}\n")

    print(f"session captures: {len(caps)}  fixed cams: {fixed_cams}  gripper cam: {gripper_cam}")
    print(f"similarity(Umeyama) fit of robot gripper traj -> vision cube-center traj, >= {MIN_MARKERS} markers\n")

    Ks = {c: {k: load_K(c, k) for k in ("charuco", "factory")} for c in fixed_cams}
    for c in fixed_cams:
        (Kc, _), (Kf, _) = Ks[c]["charuco"], Ks[c]["factory"]
        print(f"cam{c}: fx charuco={Kc[0,0]:.1f}  factory={Kf[0,0]:.1f}  factory/charuco={Kf[0,0]/Kc[0,0]:.4f}")
    print()

    print(f"{'cam':5}{'N':5}{'k_charuco (rms)':22}{'k_factory (rms)':22}{'winner':9}")
    for c in fixed_cams:
        (Kc, Dc), (Kf, Df) = Ks[c]["charuco"], Ks[c]["factory"]
        Xr, Ych, Yfac = [], [], []
        for cap in caps:
            cd = cap["cams"].get(str(c), cap["cams"].get(c))
            if cd is None or not cd.get("cube_visible"):
                continue
            markers = cd.get("markers") or []
            if len([m for m in markers if "corners_2d" in m]) < MIN_MARKERS:
                continue
            T = cap.get("capture_gripper_pose_matrix_4x4") or cap.get("robot_pose_matrix_4x4")
            if T is None:
                continue
            T = np.asarray(T, dtype=np.float64).reshape(4, 4)
            tv_ch = cube_tvec(target, markers, Kc, Dc)
            tv_fac = cube_tvec(target, markers, Kf, Df)
            if tv_ch is None or tv_fac is None:
                continue
            Xr.append(T[:3, 3] * 1000.0)     # robot grip point, base frame (m->mm)
            Ych.append(tv_ch * 1000.0)       # vision cube center, cam frame (mm)
            Yfac.append(tv_fac * 1000.0)
        N = len(Xr)
        if N < 4:
            print(f"cam{c:<4}{N:<5}{'(too few)':22}")
            continue
        kc, rc = umeyama_scale(Xr, Ych)
        kf, rf = umeyama_scale(Xr, Yfac)
        win = "factory" if abs(kf - 1) < abs(kc - 1) else "charuco"
        print(f"cam{c:<4}{N:<5}{f'{kc:.4f} ({rc:.1f}mm)':22}{f'{kf:.4f} ({rf:.1f}mm)':22}{win:9}")


if __name__ == "__main__":
    main()
