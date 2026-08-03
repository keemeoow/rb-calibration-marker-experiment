"""
평가 지표 (모두 GT 대비 — 시뮬이라 정답을 앎).

  N_reg      : 등록(캘리브 성공)된 고정 카메라 수
  e_X        : 변환행렬 GT 대비 오차 (고정 bTf + 그리퍼 gTc), mm/°  ← 시뮬 핵심
  e_task     : held-out 큐브 pose 예측 오차 (위치 mm + 회전°)       ← 실전 성능
  e_cross    : 카메라 간 큐브위치 예측 일관성 (mm)
  e_reproj   : 재투영 오차 (px) — corner-level 필요, pose-level 에선 None

큐브 위치 예측: 고정 카메라 + 그리퍼(있으면)로 base 에서 예측 (median 합의).
"""
import numpy as np
import cv2
from .se3 import inv_T, se3_avg, rot_deg, trans_mm
from .targets import CubeTarget, BoardTarget
from .project import DEFAULT_K, DEFAULT_DIST

_CUBE = CubeTarget()
_BOARD = BoardTarget()


def unified_reproj(sc, model):
    """통일 재투영 오차(px) — 캘리브된 카메라로 **보드+큐브 전체**를 GT 위치에 재투영해
    GT 관측 코너와 비교. 캘리브에 어떤 마커를 썼든(보드만/큐브만) 동일 대상(보드+큐브)으로
    평가 → 공정 비교. 낮을수록 카메라 외부파라미터가 정확.

    방식: 추정 카메라 bTf_est 로 예측한 타깃 pose 를, GT 카메라가 실제 본 코너(노이즈 낀
    관측)와 재투영 비교. 즉 '추정 카메라가 관측을 얼마나 재현하나'.
    """
    cams = model["cams"]
    errs = []
    for ci in sc.fixed_cam_ids:
        if ci not in cams:
            continue
        for tgt, obs_dict in [(_CUBE, sc.obs_fix_cube), (_BOARD, sc.obs_fix_board)]:
            for s in sc.sets:
                if (ci, s) not in obs_dict:
                    continue
                # 추정 카메라로 본 타깃 pose = inv(bTf_est) @ (base 타깃위치)
                base_t = sc.bTo[s] if tgt is _CUBE else sc.bTboard
                T_pred = inv_T(cams[ci]) @ base_t          # camera_est←target
                # GT 관측 pose (노이즈 낀 solvePnP 결과)
                T_obs = obs_dict[(ci, s)]
                errs.append(_reproj_between(tgt, T_pred, T_obs))
    # 붕괴 케이스(캘리브 실패)는 이미지 밖으로 발산 → 화면 밖 상한(px)으로 클립.
    #   합리적 상한 = 이미지 대각선(~800px) — "완전 실패"를 유한값으로.
    CAP = 800.0
    v = [min(e, CAP) for e in errs if e is not None]
    return float(np.mean(v)) if v else None


def _reproj_between(target, T_pred, T_obs):
    """예측 pose T_pred 로 타깃 코너를 투영한 위치 vs 관측 pose T_obs 로 투영한 위치의
    픽셀 오차(RMS). 두 pose 가 같으면 0."""
    pts = []
    for mid, c3d, normal in target.all_corners():
        pts.append(c3d)
    obj = np.concatenate(pts, 0)
    def proj(T):
        R = T[:3, :3]; t = T[:3, 3]
        if np.any((R @ obj.T).T[:, 2] + t[2] <= 1e-3):
            return None
        p, _ = cv2.projectPoints(obj.reshape(-1, 1, 3), cv2.Rodrigues(R)[0],
                                 t.reshape(3, 1), DEFAULT_K, DEFAULT_DIST)
        return p.reshape(-1, 2)
    pa, pb = proj(T_pred), proj(T_obs)
    if pa is None or pb is None:
        return None
    return float(np.sqrt(np.mean(np.sum((pa - pb) ** 2, axis=1))))


def predict_cube_pos(sc, model, s):
    """캘리브된 카메라들로 set s 큐브 중심(base)을 예측 (축별 median)."""
    cams = model["cams"]; gTc = model.get("gTc")
    pts = []
    for ci in sc.fixed_cam_ids:
        if ci in cams and (ci, s) in sc.obs_fix_cube:
            pts.append((cams[ci] @ sc.obs_fix_cube[(ci, s)])[:3, 3])
    if gTc is not None:
        for e in sc.set_events.get(s, []):
            if e in sc.obs_grip_cube:
                pts.append((sc.bTg[e] @ gTc @ sc.obs_grip_cube[e])[:3, 3])
    if not pts:
        return None
    p = np.median(np.array(pts), axis=0)
    # 독립(indep)의 rigid 정합이 있으면 그리퍼 예측을 고정 base 로 (여기선 합의 median 사용)
    return p


def predict_cube_pose(sc, model, s):
    """set s 큐브 **pose(4x4)** 예측 — 카메라 합의(회전 포함)."""
    cams = model["cams"]; gTc = model.get("gTc")
    Ts = []
    for ci in sc.fixed_cam_ids:
        if ci in cams and (ci, s) in sc.obs_fix_cube:
            Ts.append(cams[ci] @ sc.obs_fix_cube[(ci, s)])
    if gTc is not None:
        for e in sc.set_events.get(s, []):
            if e in sc.obs_grip_cube:
                Ts.append(sc.bTg[e] @ gTc @ sc.obs_grip_cube[e])
    return se3_avg(Ts) if Ts else None


def eval_model(sc, model, train_sets, test_sets, W=None):
    """한 model 에 대해 지표 dict 반환. W: FK 후보정 계수(corr 방식만)."""
    from .methods import apply_fk_correction
    out = {}
    cams = model["cams"]

    # N_reg
    out["N_reg"] = len(cams)

    # e_X : 고정 카메라 bTf + 그리퍼 gTc GT 대비 (mm/°)
    ce = [trans_mm(cams[ci], sc.bTf[ci]) for ci in cams]
    cr = [rot_deg(cams[ci], sc.bTf[ci]) for ci in cams]
    g_mm = trans_mm(model["gTc"], sc.gTc) if model.get("gTc") is not None else None
    g_deg = rot_deg(model["gTc"], sc.gTc) if model.get("gTc") is not None else None
    # e_X = 카메라·gTc 평균 (mm, deg 각각)
    all_mm = ce + ([g_mm] if g_mm is not None else [])
    all_deg = cr + ([g_deg] if g_deg is not None else [])
    out["e_X_mm"] = float(np.mean(all_mm)) if all_mm else None
    out["e_X_deg"] = float(np.mean(all_deg)) if all_deg else None
    out["bTf_mm"] = float(np.mean(ce)) if ce else None
    out["gTc_mm"] = g_mm

    # e_task : held-out 큐브 pose 예측 오차 (위치 mm + 회전°)
    t_mm, t_deg = [], []
    for s in test_sets:
        p = predict_cube_pose(sc, model, s)
        if p is None:
            continue
        pos = apply_fk_correction(p[:3, 3], W) if W is not None else p[:3, 3]
        t_mm.append(np.linalg.norm(pos - sc.bTo[s][:3, 3]) * 1000)
        t_deg.append(rot_deg(p, sc.bTo[s]))
    out["e_task_mm"] = float(np.mean(t_mm)) if t_mm else None
    out["e_task_deg"] = float(np.mean(t_deg)) if t_deg else None

    # e_cross : 카메라 간 큐브위치 예측 일관성 (train)
    cross = []
    for s in train_sets:
        pts = [(cams[ci] @ sc.obs_fix_cube[(ci, s)])[:3, 3]
               for ci in sc.fixed_cam_ids if ci in cams and (ci, s) in sc.obs_fix_cube]
        if len(pts) >= 2:
            c = np.mean(pts, 0)
            cross.append(np.mean([np.linalg.norm(p - c) for p in pts]) * 1000)
    out["e_cross_mm"] = float(np.mean(cross)) if cross else None

    # e_reproj : 통일 재투영 — 캘리브에 뭘 썼든(보드만/큐브만) 보드+큐브 전체로 평가 (공정)
    out["e_reproj_px"] = unified_reproj(sc, model)
    return out
