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
from .se3 import inv_T, se3_avg, rot_deg, trans_mm


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

    # e_reproj : corner-level 필요 → pose-level 에선 미지원
    out["e_reproj_px"] = None
    return out
