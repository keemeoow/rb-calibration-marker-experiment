"""
캘리브 방법 — 통합(unified) / 독립(independent) × FK 사용방식 × 마커 선택.

미지수: 고정 카메라 {bTf_ci}, 그리퍼 핸드아이 gTc, 타깃 pose(큐브 cube[s], 보드 board).
공통 물리관계 (관측 = camera←target):
  고정:   bTf_ci @ obs_fix == target
  그리퍼: bTg[e] @ gTc @ obs_grip == target        (bTg = 로봇 자세, 항상 known)
base gauge 는 로봇 자세 bTg(그리퍼 체인)가 제공 → FK 큐브 prior 없이도 통합이 성립.

FK 사용방식 (fk_mode):
  none   : 타깃(큐브)을 미지수로 추정. FK 미사용.
  fixed  : 큐브 = FK 상수로 고정 → 카메라·gTc 만 최적화.
  factor : 큐브는 자유. FK 를 **공분산 가중 robust 인자**로 BA 에 추가 (= Ours).
           FK 는 관측과 같은 자격의 잔차 블록이며, 잘못된 FK 는 Huber 가 자동으로 낮춘다.
  corr   : none 으로 캘리브 후 예측 위치에 Ridge 후보정 (구 방식. 회전 미보정 → 비교군).

공정성 규약 (모든 방법에 동일 적용):
  1. 초기화는 `_bootstrap_visual` 하나만 사용 — GT·FK 를 쓰지 않고 관측과 로봇 자세만으로
     구성한다. (이전 버전은 no-FK 비교군까지 fk_cube / bTboard(GT) 로 초기화했음.)
  2. 모든 잔차는 sigma 로 whitening 되고 동일한 Huber loss·동일 solver 로 푼다.
  3. 프론트엔드(robust PnP)는 씬이 한 번만 돌려 모든 방법이 공유한다.

마커: markers ⊆ {"cube","board"}. "board" 는 FK 없음(테이블 고정) → fixed/factor/corr 은
      큐브가 있어야 성립 (board only + FK 는 불가; 상위에서 차단).
"""
import numpy as np
import cv2
from scipy.optimize import least_squares
from .se3 import (inv_T, se3_to_vec, vec_to_se3, se3_residual, se3_avg, fit_rigid)


# ---------------------------------------------------------------- 솔버 규약 (전 방법 공통)
# 잔차 whitening sigma. PnP pose 관측의 대략적 잡음 스케일 — 값 자체보다 "모든 방법이
# 같은 값을 쓴다"는 점이 중요하다. 씬의 GT 잡음에서 읽어오지 않는다(오라클 방지).
SIGMA_OBS_ROT = 0.010          # rad
SIGMA_OBS_T = 0.003            # m
# FK 인자의 사전 불확실성 — 로봇 반복정밀도 스펙에서 정하는 값. 전 실험 동결.
SIGMA_FK_DEG = 0.30
SIGMA_FK_MM = 2.0
HUBER_F_SCALE = 3.0            # whitening 후 3-sigma 밖을 감쇠
SOLVER = dict(method="trf", loss="huber", f_scale=HUBER_F_SCALE, x_scale="jac")


def _whiten(r6, s_rot=SIGMA_OBS_ROT, s_t=SIGMA_OBS_T):
    """se3_residual 6-vec 을 sigma 로 정규화 → 회전/병진이 같은 척도가 됨."""
    return np.concatenate([r6[:3] / s_rot, r6[3:] / s_t])


def _solve_ls(resid, p0, max_nfev):
    """모든 방법이 공유하는 robust 최소제곱 (동일 solver·동일 loss)."""
    return least_squares(resid, p0, max_nfev=max_nfev, **SOLVER)


# ---------------------------------------------------------------- 관측 수집
def _gather_obs(sc, markers, train_sets):
    """(kind, ci_or_e, set, target_type, T_obs) 리스트. kind: 'fix'|'grip'."""
    recs = []
    if "cube" in markers:
        for ci in sc.fixed_cam_ids:
            for s in train_sets:
                if (ci, s) in sc.obs_fix_cube:
                    recs.append(("fix", ci, s, "cube", sc.obs_fix_cube[(ci, s)]))
        for e in [e for s in train_sets for e in sc.set_events[s]]:
            if e in sc.obs_grip_cube:
                recs.append(("grip", e, sc.event_set[e], "cube", sc.obs_grip_cube[e]))
    if "board" in markers:
        for ci in sc.fixed_cam_ids:
            for s in train_sets:
                if (ci, s) in sc.obs_fix_board:
                    recs.append(("fix", ci, s, "board", sc.obs_fix_board[(ci, s)]))
        for e in [e for s in train_sets for e in sc.set_events[s]]:
            if e in sc.obs_grip_board:
                recs.append(("grip", e, sc.event_set[e], "board", sc.obs_grip_board[e]))
    return recs


# ---------------------------------------------------------------- GT-free 초기화
def _handeye_closed_form(sc, markers, train_sets):
    """모션 기반 핸드아이(Park) 로 gTc 초기값. **GT·FK 미사용**.

    같은 set 안에서 타깃은 정지 → bTg[e] @ X @ obs[e] 가 e 에 무관.
    이는 표준 eye-in-hand AX=XB 이므로 cv2.calibrateHandEye 로 닫힌형 해를 얻는다.
    보드는 전 event 에 걸쳐 정지 → 한 번에, 큐브는 set 별로 풀어 평균.
    """
    sols = []
    groups = []
    if "board" in markers:
        evs = [e for s in train_sets for e in sc.set_events[s] if e in sc.obs_grip_board]
        if len(evs) >= 3:
            groups.append((evs, sc.obs_grip_board))
    if "cube" in markers:
        for s in train_sets:
            evs = [e for e in sc.set_events[s] if e in sc.obs_grip_cube]
            if len(evs) >= 3:
                groups.append((evs, sc.obs_grip_cube))
    for evs, obs in groups:
        Rg = [sc.bTg[e][:3, :3] for e in evs]
        tg = [sc.bTg[e][:3, 3].reshape(3, 1) for e in evs]
        Rt = [obs[e][:3, :3] for e in evs]
        tt = [obs[e][:3, 3].reshape(3, 1) for e in evs]
        try:
            R, t = cv2.calibrateHandEye(Rg, tg, Rt, tt, method=cv2.CALIB_HAND_EYE_PARK)
        except cv2.error:
            continue
        if R is None or not np.all(np.isfinite(R)) or not np.all(np.isfinite(t)):
            continue
        R = _project_SO3(R)
        if R is None:                 # 회전축 다양성이 부족한 set → 퇴화 해. 버린다.
            continue
        T = np.eye(4); T[:3, :3] = R; T[:3, 3] = np.asarray(t).reshape(3)
        sols.append(T)
    return se3_avg(sols) if sols else np.eye(4)


def _project_SO3(R, max_dev=0.05):
    """calibrateHandEye 가 퇴화 입력에서 비직교/좌수 행렬을 돌려주는 경우가 있다.
    SVD 로 SO(3) 에 투영하고, 원본이 너무 어긋났거나 det<0 이면 None (해당 해 폐기)."""
    R = np.asarray(R, float)
    U, S, Vt = np.linalg.svd(R)
    if np.linalg.det(U @ Vt) < 0 or np.max(np.abs(S - 1.0)) > max_dev:
        return None
    return U @ Vt


def _bootstrap_visual(sc, markers, train_sets):
    """GT·FK 를 쓰지 않는 공통 초기화 → (cams, gTc, cube0, board0).

    순서:
      1) gTc  : 모션 기반 핸드아이 (로봇 자세 + 그리퍼 관측만).
      2) 타깃 : base gauge 는 로봇 자세가 제공 → cube[s]/board = bTg @ gTc @ obs 합의.
      3) 카메라: 위에서 얻은 base 타깃으로 bTf = target @ inv(obs) 역산.
    어떤 단계도 sc.bTo / sc.bTboard / sc.fk_cube 를 참조하지 않는다.
    """
    use_cube, use_board = "cube" in markers, "board" in markers
    gTc = _handeye_closed_form(sc, markers, train_sets)

    cube0 = {}
    if use_cube:
        for s in train_sets:
            Ts = [sc.bTg[e] @ gTc @ sc.obs_grip_cube[e]
                  for e in sc.set_events[s] if e in sc.obs_grip_cube]
            if Ts:
                cube0[s] = se3_avg(Ts)
    board0 = None
    if use_board:
        Ts = [sc.bTg[e] @ gTc @ sc.obs_grip_board[e]
              for s in train_sets for e in sc.set_events[s] if e in sc.obs_grip_board]
        if Ts:
            board0 = se3_avg(Ts)

    cams = {}
    for ci in sc.fixed_cam_ids:
        Ts = []
        if use_cube:
            Ts += [cube0[s] @ inv_T(sc.obs_fix_cube[(ci, s)])
                   for s in train_sets if s in cube0 and (ci, s) in sc.obs_fix_cube]
        if use_board and board0 is not None:
            Ts += [board0 @ inv_T(sc.obs_fix_board[(ci, s)])
                   for s in train_sets if (ci, s) in sc.obs_fix_board]
        if Ts:
            cams[ci] = se3_avg(Ts)
    return cams, gTc, cube0, board0


# ---------------------------------------------------------------- 통합(unified) BA
def solve_unified(sc, markers, fk_mode, train_sets, max_nfev=300):
    """모든 관측을 하나의 robust 비선형 최소제곱으로 동시 최적화.

    fk_mode:
      'none'   FK 항 없음 (큐브 자유)
      'fixed'  큐브를 FK 상수로 고정 (미지수에서 제외)
      'factor' 큐브 자유 + FK 를 공분산 가중 robust 잔차 블록으로 추가  ← Ours
    """
    use_cube = "cube" in markers
    use_board = "board" in markers
    # **관측이 하나도 없는 카메라는 미지수에서 제외** — 그렇지 않으면 구속되지 않은 카메라가
    # 발산한 채로 N_reg 에 잡혀 "등록 성공"으로 오보고된다 (board-only 가 대표 사례).
    cam_ids = [ci for ci in sc.fixed_cam_ids
               if (use_cube and any((ci, s) in sc.obs_fix_cube for s in train_sets))
               or (use_board and any((ci, s) in sc.obs_fix_board for s in train_sets))
               or (use_cube and any(k[0] == ci
                                    for k in getattr(sc, "obs_fix_cube_grip", {})))]
    cams0, gTc0, cube_b, board_b = _bootstrap_visual(sc, markers, train_sets)
    cube_free = use_cube and (fk_mode != "fixed")
    # gripped 관측: 고정 카메라가 그리퍼-큐브를 관측 (eye-to-hand via 로봇 모션).
    #   cube(base) = bTg_grip @ X. X(그리퍼→큐브)는 신규 미지수.
    grip_recs = [(ci, ge, T) for (ci, ge), T in getattr(sc, "obs_fix_cube_grip", {}).items()
                 if ci in cam_ids] if use_cube else []
    use_grip = len(grip_recs) > 0

    # 파라미터 레이아웃
    p0 = [se3_to_vec(cams0.get(ci, np.eye(4))) for ci in cam_ids]
    p0.append(se3_to_vec(gTc0))
    idx = {}
    off = len(cam_ids) * 6 + 6
    if cube_free:
        for s in train_sets:
            # 초기 큐브: 그리퍼 체인 추정값(GT-free). 없으면 고정 카메라 합의로 대체.
            init = cube_b.get(s)
            if init is None:
                Ts = [cams0[ci] @ sc.obs_fix_cube[(ci, s)]
                      for ci in cam_ids if ci in cams0 and (ci, s) in sc.obs_fix_cube]
                init = se3_avg(Ts) if Ts else np.eye(4)
            idx[("cube", s)] = off; off += 6; p0.append(se3_to_vec(init))
    if use_board:
        init = board_b
        if init is None:
            Ts = [cams0[ci] @ sc.obs_fix_board[(ci, s)]
                  for ci in cam_ids if ci in cams0 for s in train_sets
                  if (ci, s) in sc.obs_fix_board]
            init = se3_avg(Ts) if Ts else np.eye(4)
        idx[("board",)] = off; off += 6; p0.append(se3_to_vec(init))
    if use_grip:                                         # 그리퍼→큐브 장착 X (신규 미지수)
        Xs = [inv_T(sc.bTg_grip[ge]) @ cams0[ci] @ T for (ci, ge, T) in grip_recs if ci in cams0]
        X0 = se3_avg(Xs) if Xs else np.eye(4)
        idx[("X",)] = off; off += 6; p0.append(se3_to_vec(X0))
    p0 = np.concatenate(p0)
    recs = _gather_obs(sc, markers, train_sets)

    s_fk_rot = np.deg2rad(SIGMA_FK_DEG)
    s_fk_t = SIGMA_FK_MM / 1000.0
    use_fk_factor = (fk_mode == "factor") and cube_free

    def unpack(p):
        cams = {ci: vec_to_se3(p[i*6:(i+1)*6]) for i, ci in enumerate(cam_ids)}
        gTc = vec_to_se3(p[len(cam_ids)*6:len(cam_ids)*6+6])
        return cams, gTc

    def target_pose(p, ttype, s):
        if ttype == "board":
            return vec_to_se3(p[idx[("board",)]:idx[("board",)]+6])
        if cube_free:
            return vec_to_se3(p[idx[("cube", s)]:idx[("cube", s)]+6])
        return sc.fk_cube[s]                            # fixed: 상수

    def resid(p):
        cams, gTc = unpack(p)
        r = []
        for (kind, a, s, ttype, T_obs) in recs:
            Cs = target_pose(p, ttype, s)
            if kind == "fix":
                r.append(_whiten(se3_residual(cams[a] @ T_obs, Cs)))
            else:
                r.append(_whiten(se3_residual(sc.bTg[a] @ gTc @ T_obs, Cs)))
        # FK factor (Ours): FK 를 별도 공분산의 잔차 블록으로. Huber 가 나쁜 FK 를 감쇠.
        if use_fk_factor:
            for s in train_sets:
                if ("cube", s) in idx:
                    r.append(_whiten(se3_residual(target_pose(p, "cube", s), sc.fk_cube[s]),
                                     s_rot=s_fk_rot, s_t=s_fk_t))
        # gripped: 고정카메라 @ 관측 == bTg_grip @ X (로봇 모션 기반 eye-to-hand)
        if use_grip:
            Xm = vec_to_se3(p[idx[("X",)]:idx[("X",)]+6])
            for (ci, ge, T_obs) in grip_recs:
                if ci in cams:
                    r.append(_whiten(se3_residual(cams[ci] @ T_obs, sc.bTg_grip[ge] @ Xm)))
        return np.concatenate(r) if r else np.zeros(1)

    sol = _solve_ls(resid, p0, max_nfev)
    cams, gTc = unpack(sol.x)
    model = {"cams": cams, "gTc": gTc, "mode": f"unified/{fk_mode}", "align": None}
    if use_grip:
        model["X"] = vec_to_se3(sol.x[idx[("X",)]:idx[("X",)]+6])
    return model


# ---------------------------------------------------------------- 독립(independent)
def solve_independent(sc, markers, fk_mode, train_sets):
    """고정 카메라와 그리퍼를 *따로* 풀고 base 에서 조합(공유 타깃 rigid 정합).
       fk_mode='fixed' 면 큐브 FK 고정(각 카메라 독립 역산 = 통합과 동일)."""
    cam_ids = sc.fixed_cam_ids
    use_cube = "cube" in markers

    # --- 고정 카메라 ---
    if fk_mode == "fixed" and use_cube:
        # 큐브=FK 고정 → 각 카메라 closed-form 역산
        cams = {}
        for ci in cam_ids:
            Ts = [sc.fk_cube[s] @ inv_T(sc.obs_fix_cube[(ci, s)])
                  for s in train_sets if (ci, s) in sc.obs_fix_cube]
            if not Ts:
                continue
            cams[ci] = se3_avg(Ts)
        gTc = _handeye_to_fk(sc, train_sets)            # 그리퍼도 FK 큐브에 정합
        return {"cams": cams, "gTc": gTc, "mode": "indep/fixed", "align": None}

    # none/factor/corr: 고정 카메라는 관측만으로 합의. 그리퍼는 따로 핸드아이.
    cams0, gTc0, cube_b, board_b = _bootstrap_visual(sc, markers, train_sets)
    cams = dict(cams0)
    if use_cube:
        # 고정 카메라끼리만의 큐브 합의로 카메라 정제 (그리퍼 정보 미사용 = 독립)
        cube_c = {}
        for s in train_sets:
            Ts = [cams0[ci] @ sc.obs_fix_cube[(ci, s)]
                  for ci in cam_ids if ci in cams0 and (ci, s) in sc.obs_fix_cube]
            if Ts:
                cube_c[s] = se3_avg(Ts)
        cams = {}
        for ci in cam_ids:
            Ts = [cube_c[s] @ inv_T(sc.obs_fix_cube[(ci, s)])
                  for s in train_sets if s in cube_c and (ci, s) in sc.obs_fix_cube]
            if Ts:
                cams[ci] = se3_avg(Ts)
            elif ci in cams0:                 # 큐브는 못 봤지만 보드로는 초기화된 카메라
                cams[ci] = cams0[ci]          # (관측 없는 카메라는 아예 등록하지 않는다)
    if fk_mode == "factor" and use_cube:
        # 독립판 Ours: 고정 카메라 블록을 FK factor 와 함께 따로 BA (그리퍼 정보 미사용).
        cams = _refine_fixed_block(sc, cams, train_sets)
    # 그리퍼 핸드아이 (독립: 고정 카메라 정보 미사용)
    if use_cube:
        gTc = _handeye_freecube(sc, train_sets, gTc0,
                                fk_factor=(fk_mode == "factor"))
    else:
        gTc = _handeye_freeboard(sc, train_sets, gTc0)
    # 조합: 그리퍼가 본 큐브 vs 고정이 본 큐브를 base 에서 rigid 정합 → 그리퍼계를 고정계로
    align = _rigid_align(sc, cams, gTc, markers, train_sets)
    return {"cams": cams, "gTc": gTc, "mode": "indep/" + fk_mode, "align": align}


def _handeye_to_fk(sc, train_sets):
    """그리퍼 gTc 를 FK 큐브 절대위치에 정합 (fixed 모드 독립 핸드아이)."""
    g = []
    for e in [e for s in train_sets for e in sc.set_events[s]]:
        if e not in sc.obs_grip_cube:
            continue
        s = sc.event_set[e]
        g.append(inv_T(sc.bTg[e]) @ sc.fk_cube[s] @ inv_T(sc.obs_grip_cube[e]))
    return se3_avg(g) if g else np.eye(4)


def _fk_resid(cube_T, fk_T):
    """FK factor 잔차 블록 (공분산 가중). 통합/독립이 같은 sigma 를 쓴다."""
    return _whiten(se3_residual(cube_T, fk_T),
                   s_rot=np.deg2rad(SIGMA_FK_DEG), s_t=SIGMA_FK_MM / 1000.0)


def _refine_fixed_block(sc, cams0, train_sets, max_nfev=200):
    """독립 방식의 고정 카메라 블록만 따로 BA (카메라 + 자유 큐브 + FK factor).
       그리퍼 관측은 쓰지 않는다 — '통합하지 않음'이 이 비교군의 정의이므로."""
    cam_ids = [ci for ci in cams0]
    sets = [s for s in train_sets
            if any((ci, s) in sc.obs_fix_cube for ci in cam_ids)]
    if not cam_ids or not sets:
        return cams0
    cube0 = {}
    for s in sets:
        Ts = [cams0[ci] @ sc.obs_fix_cube[(ci, s)]
              for ci in cam_ids if (ci, s) in sc.obs_fix_cube]
        cube0[s] = se3_avg(Ts)
    p0 = np.concatenate([se3_to_vec(cams0[ci]) for ci in cam_ids]
                        + [se3_to_vec(cube0[s]) for s in sets])
    cidx = {s: len(cam_ids) * 6 + i * 6 for i, s in enumerate(sets)}

    def resid(p):
        cams = {ci: vec_to_se3(p[i*6:(i+1)*6]) for i, ci in enumerate(cam_ids)}
        r = []
        for s in sets:
            Cs = vec_to_se3(p[cidx[s]:cidx[s]+6])
            for ci in cam_ids:
                if (ci, s) in sc.obs_fix_cube:
                    r.append(_whiten(se3_residual(cams[ci] @ sc.obs_fix_cube[(ci, s)], Cs)))
            r.append(_fk_resid(Cs, sc.fk_cube[s]))
        return np.concatenate(r)

    x = _solve_ls(resid, p0, max_nfev).x
    return {ci: vec_to_se3(x[i*6:(i+1)*6]) for i, ci in enumerate(cam_ids)}


def _handeye_freecube(sc, train_sets, gTc0, fk_factor=False, max_nfev=120):
    """그리퍼만으로 gTc 추정 (GT 미사용). 큐브 위치를 미지수로 두고
       '같은 set 큐브는 이벤트 무관 상수'라는 제약으로 gTc·cube[s] 동시 최적화.
       초기값 gTc0 는 모션 기반 닫힌형 해 (통합과 동일한 초기화).
       fk_factor=True 면 통합과 **같은 sigma·같은 Huber** 의 FK 인자를 큐브에 건다."""
    cube0 = {}
    for s in train_sets:
        Ts = [sc.bTg[e] @ gTc0 @ sc.obs_grip_cube[e]
              for e in sc.set_events[s] if e in sc.obs_grip_cube]
        if Ts:
            cube0[s] = se3_avg(Ts)
    sets = list(cube0)
    if not sets:
        return gTc0
    p0 = np.concatenate([se3_to_vec(gTc0)] + [se3_to_vec(cube0[s]) for s in sets])
    cidx = {s: 6 + i * 6 for i, s in enumerate(sets)}

    def resid(p):
        gTc = vec_to_se3(p[:6])
        r = []
        for s in sets:
            Cs = vec_to_se3(p[cidx[s]:cidx[s]+6])
            for e in sc.set_events[s]:
                if e not in sc.obs_grip_cube:
                    continue
                r.append(_whiten(se3_residual(sc.bTg[e] @ gTc @ sc.obs_grip_cube[e], Cs)))
            if fk_factor:
                r.append(_fk_resid(Cs, sc.fk_cube[s]))
        return np.concatenate(r) if r else np.zeros(1)

    return vec_to_se3(_solve_ls(resid, p0, max_nfev).x[:6])


def _handeye_freeboard(sc, train_sets, gTc0, max_nfev=120):
    """보드만: gTc 와 보드 pose 를 동시 추정 (GT 미사용 순수 AX=XB).
       보드는 전 event 에 걸쳐 정지 → 보드 pose 는 단일 미지수."""
    evs = [e for s in train_sets for e in sc.set_events[s] if e in sc.obs_grip_board]
    if len(evs) < 3:
        return gTc0
    board0 = se3_avg([sc.bTg[e] @ gTc0 @ sc.obs_grip_board[e] for e in evs])
    p0 = np.concatenate([se3_to_vec(gTc0), se3_to_vec(board0)])

    def resid(p):
        gTc = vec_to_se3(p[:6]); Bd = vec_to_se3(p[6:12])
        r = [_whiten(se3_residual(sc.bTg[e] @ gTc @ sc.obs_grip_board[e], Bd)) for e in evs]
        return np.concatenate(r)

    return vec_to_se3(_solve_ls(resid, p0, max_nfev).x[:6])


def _rigid_align(sc, cams, gTc, markers, train_sets):
    """독립 조합: (그리퍼 예측 큐브) → (고정 예측 큐브) rigid 변환. 예측 시 그리퍼 체인에 적용."""
    if "cube" not in markers:
        return None
    P_grip, P_fix = [], []
    for s in train_sets:
        Tf = [cams[ci] @ sc.obs_fix_cube[(ci, s)]
              for ci in sc.fixed_cam_ids if ci in cams and (ci, s) in sc.obs_fix_cube]
        Tg = [sc.bTg[e] @ gTc @ sc.obs_grip_cube[e]
              for e in sc.set_events[s] if e in sc.obs_grip_cube]
        if Tf and Tg:
            P_fix.append(se3_avg(Tf)[:3, 3]); P_grip.append(se3_avg(Tg)[:3, 3])
    if len(P_fix) >= 3:
        R, t = fit_rigid(P_grip, P_fix)
        T = np.eye(4); T[:3, :3] = R; T[:3, 3] = t
        return T                                   # 4x4 로 반환 → 회전까지 적용 가능
    return None


# ---------------------------------------------------------------- FK 후보정 (구 corr 방식)
def _feat(t, degree=1):
    """위치 특징. degree=1: [1,x,y] (CP_C1 정합, 기본). degree=2: [1,x,y,x²,y²,xy,z]."""
    x, y, z = t[0], t[1], t[2]
    if degree >= 2:
        return np.array([1.0, x, y, x*x, y*y, x*y, z])
    return np.array([1.0, x, y])


def learn_fk_correction(sc, model, train_sets, lam=1e-3, degree=1):
    """train 에서 (예측 큐브위치 vs FK) 잔차를 특징에 Ridge 회귀 → 계수 W.
    **위치만 보정하고 회전은 보정하지 않는다** — Ours 가 아니라 비교군(구 방식)."""
    from .metrics import predict_cube_pos          # 지연 import (순환 방지)
    X, Y = [], []
    for s in train_sets:
        p = predict_cube_pos(sc, model, s)
        if p is None:
            continue
        X.append(_feat(p, degree)); Y.append(sc.fk_cube[s][:3, 3] - p)
    ncoef = len(_feat(np.zeros(3), degree))
    if len(X) < ncoef:                                 # 표본이 특징수보다 적으면 과적합
        return None
    X = np.array(X); Y = np.array(Y)
    reg = lam * np.eye(ncoef); reg[0, 0] = 0.0         # 절편 정규화 제외
    W = np.linalg.solve(X.T @ X + reg, X.T @ Y)
    return {"W": W, "degree": degree}


def apply_fk_correction(p, W):
    """예측 위치 p 에 후보정 적용. W 는 {'W','degree'} dict 또는 None."""
    if W is None:
        return p
    return p + _feat(p, W["degree"]) @ W["W"]
