"""
평가 지표.

GT 기반 (시뮬이므로 정답을 앎 — 논문의 '절대 정확도' 근거):
  N_reg        : 등록(캘리브 성공)된 고정 카메라 수
  e_X          : 변환행렬 GT 대비 오차 (고정 bTf + 그리퍼 gTc), mm/°
  e_task       : held-out 큐브 pose 예측 오차 (위치 mm + 회전°)     ← 실전 성능
  e_reproj_gt  : GT 타깃 pose 를 추정 카메라로 재투영 (진단용)

GT-free (실데이터에서도 그대로 계산 가능 — 시뮬/실측 비교의 다리):
  e_cross      : 카메라 간 큐브위치 예측 일관성 (mm)
  reproj_train : train set 의 원본 2D 코너 재투영 RMS (px)
  reproj_test  : **held-out set** 의 원본 2D 코너 재투영 RMS (px)  ← 헤드라인

재투영 규약 (중요):
  * 씬이 생성해 보관한 **원본 노이즈 2D 코너**(raw_*)에 직접 재투영한다.
    (이전 버전은 프론트엔드 PnP 자체 잔차 reproj_seed 를 모든 방법에 그대로 넣어
     방법별 차이가 아예 없었다.)
  * 타깃 pose 는 GT 가 아니라 **모델이 추정한 값**을 쓴다.
  * mode='loco' (leave-one-camera-out): 평가 대상 카메라를 뺀 나머지 카메라 + 그리퍼로
    타깃 base pose 를 추정한 뒤 대상 카메라에 재투영 → 카메라 간 3D 정합을 픽셀로 측정.
    한 카메라가 자기 관측을 자기가 맞추는 자명한 해를 배제한다.
  * 재투영에는 그 관측이 실제 쓴 K/dist(부정확 intrinsic)를 사용한다. 참 K 를 쓰면 GT 누출.
  * 발산(캘리브 실패)은 이미지 대각 CAP(px)으로 클립 → 유한값 + fail_rate 로 보고.
"""
import numpy as np
from .se3 import inv_T, se3_avg, rot_deg, trans_mm
from .project import reproject_rms

CAP_PX = 800.0                 # 재투영 상한(이미지 대각) — "완전 실패"를 유한값으로


# ---------------------------------------------------------------- 타깃 pose 예측
def _apply_align(T, align):
    """독립(indep) 방식의 rigid 조합 변환을 그리퍼 체인 예측에 적용."""
    return T if align is None else align @ T


def _target_base_pose(sc, model, s, ttype, exclude_cam=None):
    """모델로 set s 의 타깃 base pose 추정. exclude_cam 이 주어지면 그 카메라는 제외(LOCO)."""
    cams = model["cams"]; gTc = model.get("gTc"); align = model.get("align")
    obs_fix = sc.obs_fix_cube if ttype == "cube" else sc.obs_fix_board
    obs_grip = sc.obs_grip_cube if ttype == "cube" else sc.obs_grip_board
    Ts = []
    for ci in sc.fixed_cam_ids:
        if ci == exclude_cam or ci not in cams:
            continue
        if (ci, s) in obs_fix:
            Ts.append(cams[ci] @ obs_fix[(ci, s)])
    if gTc is not None:
        for e in sc.set_events.get(s, []):
            if e in obs_grip:
                Ts.append(_apply_align(sc.bTg[e] @ gTc @ obs_grip[e], align))
    return se3_avg(Ts) if Ts else None


def predict_cube_pos(sc, model, s):
    """캘리브된 카메라들로 set s 큐브 중심(base)을 예측 (축별 median).
    독립 방식이면 그리퍼 체인 예측에 rigid 조합(align)을 적용한다."""
    cams = model["cams"]; gTc = model.get("gTc"); align = model.get("align")
    pts = []
    for ci in sc.fixed_cam_ids:
        if ci in cams and (ci, s) in sc.obs_fix_cube:
            pts.append((cams[ci] @ sc.obs_fix_cube[(ci, s)])[:3, 3])
    if gTc is not None:
        for e in sc.set_events.get(s, []):
            if e in sc.obs_grip_cube:
                pts.append(_apply_align(sc.bTg[e] @ gTc @ sc.obs_grip_cube[e], align)[:3, 3])
    if not pts:
        return None
    return np.median(np.array(pts), axis=0)


def predict_cube_pose(sc, model, s):
    """set s 큐브 pose(4x4) 예측 — 카메라 합의(회전 포함), align 적용."""
    return _target_base_pose(sc, model, s, "cube")


# ---------------------------------------------------------------- 재투영 (GT-free)
def raw_reproj(sc, model, sets, mode="loco", targets=("cube", "board")):
    """원본 2D 코너 재투영 RMS(px). 반환 (mean, p95, fail_rate, n).

    fail_rate: CAP 에 걸린(발산) 관측 비율. mean 만 보면 실패가 가려지므로 함께 본다.
    """
    cams = model["cams"]
    vals, fails = [], 0
    for ttype in targets:
        raw_fix = sc.raw_fix_cube if ttype == "cube" else sc.raw_fix_board
        for s in sets:
            for ci in sc.fixed_cam_ids:
                if ci not in cams or (ci, s) not in raw_fix:
                    continue
                base = _target_base_pose(sc, model, s, ttype,
                                         exclude_cam=ci if mode == "loco" else None)
                if base is None:
                    continue                       # LOCO 에 필요한 다른 관측이 없음
                e = reproject_rms(raw_fix[(ci, s)], inv_T(cams[ci]) @ base)
                if e is None or not np.isfinite(e) or e >= CAP_PX:
                    vals.append(CAP_PX); fails += 1
                else:
                    vals.append(e)
    if not vals:
        return None, None, None, 0
    v = np.array(vals)
    return float(v.mean()), float(np.percentile(v, 95)), float(fails / len(v)), len(v)


def reproj_gt(sc, model):
    """진단용 GT 재투영 — GT 타깃 pose 를 추정 카메라로 관측 코너에 재투영."""
    cams = model["cams"]
    vals = []
    for ttype in ("cube", "board"):
        raw_fix = sc.raw_fix_cube if ttype == "cube" else sc.raw_fix_board
        for (ci, s), obs in raw_fix.items():
            if ci not in cams:
                continue
            base = sc.bTo[s] if ttype == "cube" else sc.bTboard
            e = reproject_rms(obs, inv_T(cams[ci]) @ base)
            vals.append(CAP_PX if (e is None or not np.isfinite(e)) else min(e, CAP_PX))
    return float(np.mean(vals)) if vals else None


# ---------------------------------------------------------------- 종합
def eval_model(sc, model, train_sets, test_sets, W=None):
    """한 model 에 대해 지표 dict 반환. W: FK 후보정 계수(구 corr 방식만)."""
    from .methods import apply_fk_correction
    out = {}
    cams = model["cams"]
    out["N_reg"] = len(cams)

    # e_X : 고정 카메라 bTf + 그리퍼 gTc GT 대비 (mm/°)
    ce = [trans_mm(cams[ci], sc.bTf[ci]) for ci in cams]
    cr = [rot_deg(cams[ci], sc.bTf[ci]) for ci in cams]
    g_mm = trans_mm(model["gTc"], sc.gTc) if model.get("gTc") is not None else None
    g_deg = rot_deg(model["gTc"], sc.gTc) if model.get("gTc") is not None else None
    all_mm = ce + ([g_mm] if g_mm is not None else [])
    all_deg = cr + ([g_deg] if g_deg is not None else [])
    # e_X 는 bTf 와 gTc 를 섞은 평균이라 해석이 모호 → 분리 지표를 항상 함께 보고
    out["e_X_mm"] = float(np.mean(all_mm)) if all_mm else None
    out["e_X_deg"] = float(np.mean(all_deg)) if all_deg else None
    out["bTf_mm"] = float(np.mean(ce)) if ce else None
    out["bTf_deg"] = float(np.mean(cr)) if cr else None
    out["gTc_mm"] = g_mm
    out["gTc_deg"] = g_deg

    # e_task : held-out 큐브 pose 예측 오차
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
    out["e_task_p95_mm"] = float(np.percentile(t_mm, 95)) if t_mm else None

    # e_cross : 카메라 간 큐브위치 예측 일관성 (train)
    cross = []
    for s in train_sets:
        pts = [(cams[ci] @ sc.obs_fix_cube[(ci, s)])[:3, 3]
               for ci in sc.fixed_cam_ids if ci in cams and (ci, s) in sc.obs_fix_cube]
        if len(pts) >= 2:
            c = np.mean(pts, 0)
            cross.append(np.mean([np.linalg.norm(p - c) for p in pts]) * 1000)
    out["e_cross_mm"] = float(np.mean(cross)) if cross else None

    # 재투영 — train / held-out 분리, 원본 코너 기준 (GT-free)
    m, p95, fr, n = raw_reproj(sc, model, train_sets, mode="loco")
    out["reproj_train_px"] = m
    m, p95, fr, n = raw_reproj(sc, model, test_sets, mode="loco")
    out["reproj_test_px"] = m
    out["reproj_test_p95_px"] = p95
    out["reproj_fail_rate"] = fr
    out["e_reproj_px"] = m                      # 헤드라인 = held-out LOCO 재투영
    out["e_reproj_gt_px"] = reproj_gt(sc, model)   # 진단용 GT 재투영
    return out
