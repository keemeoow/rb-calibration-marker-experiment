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


def _pnp(obj, img, K, dist, rvec=None, tvec=None):
    """평범한 solvePnP 래퍼 → (rvec, tvec) 또는 None."""
    guess = rvec is not None
    ok, rv, tv = cv2.solvePnP(obj.reshape(-1, 1, 3), img.reshape(-1, 1, 2), K, dist,
                              rvec=rvec, tvec=tvec, useExtrinsicGuess=guess,
                              flags=cv2.SOLVEPNP_ITERATIVE)
    return (rv, tv) if ok else None


def _px_residual(obj, img, K, dist, rvec, tvec):
    p, _ = cv2.projectPoints(obj.reshape(-1, 1, 3), rvec, tvec, K, dist)
    return np.linalg.norm(p.reshape(-1, 2) - img, axis=1)


def _robust_pnp(obj, img, K, dist, min_corners=4, thresh_px=3.0, robust=True,
                iters=3):
    """강건 PnP — **IRLS/trimming 방식**: 전체 코너로 풀고, 재투영 잔차가
    max(thresh_px, 3·MAD) 를 넘는 코너를 버린 뒤 재추정 (최대 iters 회).

    RANSAC 을 쓰지 않는 이유: solvePnPRansac 의 minimal 가설 solver 는 **평면 타깃
    (ChArUco 보드, 큐브 단일면)에서 퇴화**해 6점짜리 엉터리 consensus 를 돌려준다
    (실측: 보드 회전오차 median 125°). trimming 은 minimal solver 를 쓰지 않으므로
    평면/비평면 모두에서 안전하고, 이상치가 없으면 아무 코너도 버리지 않는다.

    반환 (rvec, tvec, keep_idx) 또는 None.
    """
    keep = np.arange(len(obj))
    r = _pnp(obj, img, K, dist)
    if r is None:
        return None
    rvec, tvec = r
    if not robust:
        return rvec, tvec, keep
    for _ in range(iters):
        res = _px_residual(obj, img, K, dist, rvec, tvec)
        mad = 1.4826 * float(np.median(np.abs(res - np.median(res)))) + 1e-9
        thr = max(float(thresh_px), 3.0 * mad)
        new = np.flatnonzero(res <= thr)
        if len(new) < max(min_corners, 4) or len(new) == len(keep):
            break
        keep = new
        r = _pnp(obj[keep], img[keep], K, dist, rvec, tvec)
        if r is None:
            break
        rvec, tvec = r
    return rvec, tvec, keep


def observe(target, T_cam_target, sigma_px=0.5, incidence_max_deg=75.0,
            K=DEFAULT_K, dist=DEFAULT_DIST, rng=None,
            min_markers=1, min_corners=4,
            K_pnp=None, dist_pnp=None, outlier_rate=0.0, outlier_px=15.0,
            robust_pnp=True, ransac_px=3.0):
    """카메라가 타깃을 관측 → 관측 dict 또는 None(미검출).

    target        : CubeTarget | BoardTarget (rig 로컬 3D 코너 제공)
    T_cam_target  : GT camera←target pose (4x4)
    sigma_px      : 코너 검출 픽셀 노이즈 표준편차 (랜덤 지터)
    incidence_max : 면 법선-시선 입사각 임계(초과 시 그 면 미검출)

    추가 관측 노이즈(실제):
      K_pnp/dist_pnp : PnP 가 쓰는 intrinsic. 참 K/dist 로 투영하되 PnP 는 이 부정확
                       K_pnp/dist_pnp 를 씀 → intrinsic 캘리브 오차 → 위치의존 systematic
                       편향. None 이면 참값과 동일(오차 없음).
      outlier_rate   : 각 코너가 이상치가 될 확률. outlier_px 크기의 큰 노이즈 주입.

    robust_pnp     : solvePnPRansac + 인라이어 재정제. **프론트엔드는 씬이 한 번만 돌리므로
                     모든 캘리브 방법이 동일한 pose·동일한 인라이어 코너 집합을 공유한다**
                     (이상치 실험에서 특정 방법만 강건 프론트엔드를 갖는 불공정 제거).

    반환 dict:
      T        : solvePnP 로 복원한 camera←target pose (노이즈 전파됨)
      obj/img  : 최종 pose 산출에 실제 쓰인 3D-2D 코너 대응 (인라이어). 재투영 평가용 원본.
      mids     : 코너별 마커 id (면 가시성 추적용)
      K/dist   : 이 관측이 쓴 intrinsic (재투영 평가도 동일 intrinsic 을 써야 공정)
      reproj_px: 프론트엔드 PnP 자체의 재투영 잔차 (씬 품질 지표. 방법 비교용 아님)
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

    obj_pts, img_pts, mid_pts = [], [], []
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
        pc = (R_ct @ corners3d.T).T + t_ct               # (N,3) camera 좌표
        front = pc[:, 2] > 1e-3
        if getattr(target, "partial_ok", False):
            if int(front.sum()) < 4:                     # 부분 검출 타깃: 앞쪽 코너만
                continue
            corners3d = corners3d[front]
        elif not np.all(front):                          # 마커 단위 타깃: 하나라도 뒤면 스킵
            continue
        # 2D 투영 (K, 왜곡)
        proj, _ = cv2.projectPoints(corners3d.reshape(-1, 1, 3),
                                    cv2.Rodrigues(R_ct)[0], t_ct.reshape(3, 1), K, dist)
        proj = proj.reshape(-1, 2)
        # FoV 판정.
        #   AprilTag(큐브): 태그가 온전히 보여야 디코딩됨 → 마커 단위 all-or-nothing.
        #   ChArUco(보드) : 체스판 코너는 개별 검출 → 화면 안 코너만 부분 사용.
        #   (부분 검출을 막으면 하향 27° 배치에서 고정 카메라 3대 중 1대만 보드를 얻어
        #    board-only 비교군이 부당하게 무너진다.)
        inb = ((proj[:, 0] >= 0) & (proj[:, 0] < IMAGE_W) &
               (proj[:, 1] >= 0) & (proj[:, 1] < IMAGE_H))
        if getattr(target, "partial_ok", False):
            if int(inb.sum()) < 4:
                continue
            corners3d = corners3d[inb]; proj = proj[inb]
        elif not np.all(inb):
            continue
        # 픽셀 노이즈 (랜덤 지터) + 이상치(outlier)
        proj_n = proj + rng.normal(0, sigma_px, proj.shape)
        if outlier_rate > 0:
            mask = rng.random(len(proj_n)) < outlier_rate
            if np.any(mask):
                proj_n[mask] += rng.normal(0, outlier_px, (int(mask.sum()), 2))
        obj_pts.append(corners3d); img_pts.append(proj_n)
        mid_pts.append(np.full(len(corners3d), mid, dtype=int))
        n_markers += 1

    if n_markers < min_markers:
        return None
    obj = np.concatenate(obj_pts, 0).astype(np.float64)
    img = np.concatenate(img_pts, 0).astype(np.float64)
    mids = np.concatenate(mid_pts, 0)
    if len(obj) < min_corners:
        return None

    # solvePnP: 노이즈 낀 2D 코너 + 3D 코너 → pose 복원.
    #   PnP 는 K_pnp/dist_pnp(부정확 intrinsic)를 씀 → 참값(K/dist)과 다르면 systematic 편향.
    r = _robust_pnp(obj, img, K_pnp, dist_pnp, min_corners=min_corners,
                    thresh_px=ransac_px, robust=robust_pnp)
    if r is None:
        return None
    rvec, tvec, keep = r
    obj_in = obj[keep]; img_in = img[keep]; mids_in = mids[keep]
    R_est, _ = cv2.Rodrigues(rvec)
    T_est = np.eye(4); T_est[:3, :3] = R_est; T_est[:3, 3] = tvec.reshape(3)

    # 재투영 오차 (px) — PnP 가 쓴 K_pnp 기준. **프론트엔드 자체 잔차**이며 캘리브 방법과 무관.
    reproj, _ = cv2.projectPoints(obj_in.reshape(-1, 1, 3), rvec, tvec, K_pnp, dist_pnp)
    reproj = reproj.reshape(-1, 2)
    reproj_px = float(np.sqrt(np.mean(np.sum((reproj - img_in) ** 2, axis=1))))
    return {"T": T_est, "obj": obj_in, "img": img_in, "mids": mids_in,
            "K": K_pnp, "dist": dist_pnp, "n_corners": len(obj_in),
            "reproj_px": reproj_px}


def reproject_rms(obs, T_pred):
    """관측 dict 의 원본 2D 코너 vs 예측 pose T_pred(camera←target) 재투영 → RMS px.
    관측이 쓴 K/dist 를 그대로 사용(참 intrinsic 을 쓰면 GT 누출). 카메라 뒤면 None."""
    R = T_pred[:3, :3]; t = T_pred[:3, 3]
    obj = obs["obj"]
    zc = (R @ obj.T).T[:, 2] + t[2]
    if np.any(zc <= 1e-3):
        return None
    p, _ = cv2.projectPoints(obj.reshape(-1, 1, 3), cv2.Rodrigues(R)[0],
                             t.reshape(3, 1), obs["K"], obs["dist"])
    p = p.reshape(-1, 2)
    return float(np.sqrt(np.mean(np.sum((p - obs["img"]) ** 2, axis=1))))
