"""
코너 수준 관측 생성 — 3D 마커 코너 → 2D 투영 → 픽셀 노이즈 → solvePnP → pose.
렌더링 없이 실물 마커 기하를 반영. cv2 사용.

핵심:
  1. 타깃(큐브/보드)의 각 면 마커 3D 코너를 카메라 좌표로 변환.
  2. 면 법선 vs 시선 입사각 판정 → 카메라를 향한 면만 관측(큐브 다면성/보드 평면 자연 발생).
  3. FoV 안의 코너만 K/왜곡으로 2D 투영.
  4. 코너 2D 에 가우시안 픽셀 노이즈 N(0, σ²) 주입 (실제 검출 지터).
  5. 관측된 3D-2D 대응으로 solvePnP → camera←target pose 복원.
  → 반환 pose 가 "카메라가 검출한 타깃 pose". 노이즈가 픽셀에서 자연히 전파됨.
"""
import numpy as np
import cv2


# 실측 카메라 intrinsic (intrinsics/charuco_intrinsics_report.json 4대 평균, 640x480)
DEFAULT_K = np.array([[597.5, 0, 327.3], [0, 599.7, 245.7], [0, 0, 1.0]])
DEFAULT_DIST = np.array([0.0918, -0.2228, -0.0001, 0.0058, 0.0])   # brown k1,k2,p1,p2,k3
IMAGE_W, IMAGE_H = 640, 480


def observe(target, T_cam_target, sigma_px=0.5, incidence_max_deg=75.0,
            K=DEFAULT_K, dist=DEFAULT_DIST, rng=None,
            min_markers=1, min_corners=4,
            K_pnp=None, dist_pnp=None, outlier_rate=0.0, outlier_px=15.0):
    """카메라가 타깃을 관측 → (T_cam_target_est, n_corners, reproj_px) 또는 None(미검출).

    target        : CubeTarget | BoardTarget (rig 로컬 3D 코너 제공)
    T_cam_target  : GT camera←target pose (4x4)
    sigma_px      : 코너 검출 픽셀 노이즈 표준편차 (랜덤 지터)
    incidence_max : 면 법선-시선 입사각 임계(초과 시 그 면 미검출)

    추가 관측 노이즈(실제):
      K_pnp/dist_pnp : PnP 가 쓰는 intrinsic. 참 K/dist 로 투영하되 PnP 는 이 부정확
                       K_pnp/dist_pnp 를 씀 → intrinsic 캘리브 오차 → 위치의존 systematic
                       편향(FK 후보정이 학습 가능). None 이면 참값과 동일(오차 없음).
      outlier_rate   : 각 코너가 이상치가 될 확률. outlier_px 크기의 큰 노이즈 주입.
    반환 pose 는 solvePnP 로 복원한 추정 pose (노이즈 전파됨).
    """
    if rng is None:
        rng = np.random.default_rng()
    if K_pnp is None:
        K_pnp = K
    if dist_pnp is None:
        dist_pnp = dist
    R_ct = T_cam_target[:3, :3]
    t_ct = T_cam_target[:3, 3]
    cos_inc = np.cos(np.deg2rad(incidence_max_deg))

    obj_pts, img_pts = [], []
    n_markers = 0
    for mid, corners3d, normal in target.all_corners():
        # 면 법선을 카메라 좌표로; 카메라를 향하는지 입사각 판정.
        # 면이 카메라를 향하면 법선(n_cam)이 시선(view)의 반대 → n_cam·view < 0.
        n_cam = R_ct @ normal
        c_cam = R_ct @ corners3d.mean(0) + t_ct          # 면 중심(카메라 좌표)
        view = c_cam / (np.linalg.norm(c_cam) + 1e-12)   # 카메라→면 시선
        facing = float(-(n_cam @ view))                  # 향하는 정도 (양수=정면)
        if facing < cos_inc:
            continue                                     # 너무 비스듬 → 미검출
        # 3D 코너를 카메라 좌표로
        pc = (R_ct @ corners3d.T).T + t_ct               # (4,3) camera 좌표
        if np.any(pc[:, 2] <= 1e-3):                      # 카메라 뒤 → 스킵
            continue
        # 2D 투영 (K, 왜곡)
        proj, _ = cv2.projectPoints(corners3d.reshape(-1, 1, 3),
                                    cv2.Rodrigues(R_ct)[0], t_ct.reshape(3, 1), K, dist)
        proj = proj.reshape(-1, 2)
        # FoV 판정
        inb = ((proj[:, 0] >= 0) & (proj[:, 0] < IMAGE_W) &
               (proj[:, 1] >= 0) & (proj[:, 1] < IMAGE_H))
        if not np.all(inb):
            continue
        # 픽셀 노이즈 (랜덤 지터) + 이상치(outlier)
        proj_n = proj + rng.normal(0, sigma_px, proj.shape)
        if outlier_rate > 0:
            mask = rng.random(len(proj_n)) < outlier_rate
            if np.any(mask):
                proj_n[mask] += rng.normal(0, outlier_px, (int(mask.sum()), 2))
        obj_pts.append(corners3d); img_pts.append(proj_n)
        n_markers += 1

    if n_markers < min_markers:
        return None
    obj = np.concatenate(obj_pts, 0).astype(np.float64)
    img = np.concatenate(img_pts, 0).astype(np.float64)
    if len(obj) < min_corners:
        return None

    # solvePnP: 노이즈 낀 2D 코너 + 3D 코너 → pose 복원.
    #   PnP 는 K_pnp/dist_pnp(부정확 intrinsic)를 씀 → 참값(K/dist)과 다르면 systematic 편향.
    ok, rvec, tvec = cv2.solvePnP(obj.reshape(-1, 1, 3), img.reshape(-1, 1, 2),
                                  K_pnp, dist_pnp, flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok:
        return None
    R_est, _ = cv2.Rodrigues(rvec)
    T_est = np.eye(4); T_est[:3, :3] = R_est; T_est[:3, 3] = tvec.reshape(3)

    # 재투영 오차 (px) — PnP 가 쓴 K_pnp 기준
    reproj, _ = cv2.projectPoints(obj.reshape(-1, 1, 3), rvec, tvec, K_pnp, dist_pnp)
    reproj = reproj.reshape(-1, 2)
    reproj_px = float(np.sqrt(np.mean(np.sum((reproj - img) ** 2, axis=1))))
    return T_est, len(obj), reproj_px
