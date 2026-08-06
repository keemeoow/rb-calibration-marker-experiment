"""
캘리브 방법 — 통합(unified) / 독립(independent) × FK 3-값(none/fixed/corr) × 마커 선택.

미지수: 고정 카메라 {bTf_ci}, 그리퍼 핸드아이 gTc, 타깃 pose(큐브 cube[s], 보드 board).
공통 물리관계 (관측 = camera←target):
  고정:   bTf_ci @ obs_fix == target
  그리퍼: bTg[e] @ gTc @ obs_grip == target        (bTg = 로봇 자세, 항상 known)
base gauge 는 로봇 자세 bTg(그리퍼 체인)가 제공 → FK 큐브 prior 없이도 통합 가능.

FK 3-값:
  none  : 타깃(큐브)을 미지수로 추정. FK 미사용.
  fixed : 큐브 = FK 상수로 고정 → 카메라·gTc 만 최적화 (통합=독립: 카메라 분리).
  corr  : none 으로 캘리브 후, 최종 큐브예측에 train 잔차 Ridge 후보정 (채택).

마커: markers ⊆ {"cube","board"}. "board" 는 FK 없음(테이블 고정) → fixed/corr 은
      큐브가 있어야 성립 (board only + FK 는 불가; 상위에서 차단).
"""
import numpy as np
from scipy.optimize import least_squares
from .se3 import (inv_T, se3_to_vec, vec_to_se3, se3_residual, se3_avg, fit_rigid)


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


def _handeye_freetarget(sc, train_sets, obs_g, max_nfev=60):
    """그리퍼 관측만으로 gTc 추정 (identity 초기, FK/GT 미사용). target[s] 자유변수.
       제약: bTg[e] @ gTc @ obs_g[e] == target[set(e)]. base gauge 는 로봇 자세 bTg(정당하게 known)."""
    sets = [s for s in train_sets if any(e in obs_g for e in sc.set_events[s])]
    if sum(1 for s in sets for e in sc.set_events[s] if e in obs_g) < 3:
        return np.eye(4)
    gTc0 = np.eye(4)                                   # visual: FK/GT 아닌 항등 초기
    tgt0 = {s: se3_avg([sc.bTg[e] @ gTc0 @ obs_g[e] for e in sc.set_events[s] if e in obs_g])
            for s in sets}
    p0 = np.concatenate([se3_to_vec(gTc0)] + [se3_to_vec(tgt0[s]) for s in sets])
    cidx = {s: 6 + i * 6 for i, s in enumerate(sets)}

    def resid(p):
        gTc = vec_to_se3(p[:6]); r = []
        for s in sets:
            Cs = vec_to_se3(p[cidx[s]:cidx[s] + 6])
            for e in sc.set_events[s]:
                if e in obs_g:
                    r.append(se3_residual(sc.bTg[e] @ gTc @ obs_g[e], Cs))
        return np.concatenate(r) if r else np.zeros(1)

    sol = least_squares(resid, p0, method="lm", max_nfev=max_nfev)   # init 전용 → lm(빠름)
    return vec_to_se3(sol.x[:6])


def _bootstrap(sc, markers, train_sets):
    """visual-only 초기값 (FK/GT 미사용). base gauge = 로봇 자세 bTg(정당하게 known).
       gTc: 그리퍼 관측 free-target 핸드아이(identity 초기). target[s]: 그리퍼 예측(bTg@gTc@obs).
       고정 카메라: target[s]@inv(obs_fix). fk_cube·bTboard(GT) 는 안 쓴다."""
    use_cube = "cube" in markers
    obs_g = sc.obs_grip_cube if use_cube else sc.obs_grip_board
    obs_f = sc.obs_fix_cube if use_cube else sc.obs_fix_board
    gTc = _handeye_freetarget(sc, train_sets, obs_g)
    tgt0 = {}                                          # target[s] (base) = 그리퍼 예측(관측만)
    for s in train_sets:
        T = [sc.bTg[e] @ gTc @ obs_g[e] for e in sc.set_events[s] if e in obs_g]
        if T:
            tgt0[s] = se3_avg(T)
    cams = {}                                          # 고정 카메라: visual target 으로 역산
    for ci in sc.fixed_cam_ids:
        Ts = [tgt0[s] @ inv_T(obs_f[(ci, s)]) for s in train_sets
              if s in tgt0 and (ci, s) in obs_f]
        if Ts:
            cams[ci] = se3_avg(Ts)
    return cams, gTc


def debias_fk_prior(sc, markers, train_sets):
    """real 파이프라인(estimate_set_cube_prior_alignment) 정합: raw FK 큐브 prior 의
       **상수 계통 오정렬**을 vision 으로 제거.
         T_delta_avg = robust_avg_s( inv(fk_cube[s]) @ vision_cube[s] )   (모든 set 공통 상수)
         de-biased FK[s] = fk_cube[s] @ T_delta_avg
       vision_cube = 부트스트랩(관측만) 큐브 합의. 이게 Ours corr 의 핵심(FK 를 vision 에 맞춰
       정렬 후 앵커로 사용). 추정 불가 시 raw 유지. (fixed-FK 는 raw 를 그대로 써 대조.)"""
    if "cube" not in markers:
        return sc.fk_cube
    cams0, gTc0 = _bootstrap(sc, markers, train_sets)
    deltas = []
    for s in train_sets:
        Ts = [cams0[ci] @ sc.obs_fix_cube[(ci, s)] for ci in sc.fixed_cam_ids
              if ci in cams0 and (ci, s) in sc.obs_fix_cube]
        Ts += [sc.bTg[e] @ gTc0 @ sc.obs_grip_cube[e]
               for e in sc.set_events[s] if e in sc.obs_grip_cube]
        if not Ts:
            continue
        deltas.append(inv_T(sc.fk_cube[s]) @ se3_avg(Ts))    # FK→vision delta (set별)
    if len(deltas) < 2:
        return sc.fk_cube
    T_delta = se3_avg(deltas)                                 # robust 평균 = 상수 오정렬
    return {s: sc.fk_cube[s] @ T_delta for s in sc.sets}      # 전 set de-bias


# ---------------------------------------------------------------- 통합(unified) BA
def solve_unified(sc, markers, fk_mode, train_sets, max_nfev=200, anchor_weight=0.0,
                  fk_prior=None):
    """모든 관측을 하나의 비선형 최소제곱으로 동시 최적화 (CP_C1 solve_unified_joint 정합).
       fk_mode='fixed' 면 큐브를 FK 상수로 고정(미지수 제외).
       anchor_weight>0 이면 자유 큐브를 FK prior 로 약하게 당기는 soft anchor 항 추가.
       fk_prior : 큐브 FK prior dict {s: pose}. None 이면 sc.fk_cube(raw). corr 는 de-biased
                  FK(sc.fk_cube 를 vision 으로 상수보정한 것)를 넘김 → real 파이프라인 정합."""
    if fk_prior is None:
        fk_prior = sc.fk_cube
    cam_ids = sc.fixed_cam_ids
    cams0, gTc0 = _bootstrap(sc, markers, train_sets)
    use_cube = "cube" in markers
    use_board = "board" in markers
    cube_free = use_cube and (fk_mode != "fixed")
    # gripped 관측: 고정 카메라가 그리퍼-큐브를 관측 (eye-to-hand via 로봇 모션).
    #   cube(base) = bTg_grip @ X. X(그리퍼→큐브)는 신규 미지수. train/test 무관 항상 포함
    #   (held-out 은 테이블 큐브 set 이라 gripped 는 순수 캘리브 보강).
    grip_recs = [(ci, ge, T) for (ci, ge), T in getattr(sc, "obs_fix_cube_grip", {}).items()
                 if ci in cam_ids] if use_cube else []
    use_grip = len(grip_recs) > 0

    # 파라미터 레이아웃
    p0 = [se3_to_vec(cams0.get(ci, np.eye(4))) for ci in cam_ids]
    p0.append(se3_to_vec(gTc0))
    idx = {}
    off = len(cam_ids) * 6 + 6
    if cube_free:
        cube0 = {}
        for s in train_sets:
            Ts = []
            if use_cube:
                Ts += [cams0[ci] @ sc.obs_fix_cube[(ci, s)] for ci in cam_ids if ci in cams0 and (ci,s) in sc.obs_fix_cube]
                Ts += [sc.bTg[e] @ gTc0 @ sc.obs_grip_cube[e] for e in sc.set_events[s] if e in sc.obs_grip_cube]
            cube0[s] = se3_avg(Ts) if Ts else np.eye(4)   # visual init (FK fallback 제거)
            idx[("cube", s)] = off; off += 6; p0.append(se3_to_vec(cube0[s]))
    if use_board:
        Ts = [cams0[ci] @ sc.obs_fix_board[(ci, s)]
              for ci in cam_ids if ci in cams0 for s in train_sets if (ci,s) in sc.obs_fix_board]
        Ts += [sc.bTg[e] @ gTc0 @ sc.obs_grip_board[e]
               for s in train_sets for e in sc.set_events[s] if e in sc.obs_grip_board]
        board0 = se3_avg(Ts) if Ts else np.eye(4)     # visual init (GT fallback 제거)
        idx[("board",)] = off; off += 6; p0.append(se3_to_vec(board0))
    if use_grip:                                         # 그리퍼→큐브 장착 X (신규 미지수)
        Xs = [inv_T(sc.bTg_grip[ge]) @ cams0[ci] @ T for (ci, ge, T) in grip_recs if ci in cams0]
        X0 = se3_avg(Xs) if Xs else np.eye(4)
        idx[("X",)] = off; off += 6; p0.append(se3_to_vec(X0))
    p0 = np.concatenate(p0)
    recs = _gather_obs(sc, markers, train_sets)

    def unpack(p):
        cams = {ci: vec_to_se3(p[i*6:(i+1)*6]) for i, ci in enumerate(cam_ids)}
        gTc = vec_to_se3(p[len(cam_ids)*6:len(cam_ids)*6+6])
        return cams, gTc

    def target_pose(p, ttype, s):
        if ttype == "board":
            return vec_to_se3(p[idx[("board",)]:idx[("board",)]+6])
        if cube_free:
            return vec_to_se3(p[idx[("cube", s)]:idx[("cube", s)]+6])
        return fk_prior[s]                              # fixed: FK 상수(raw 또는 de-biased)

    aw = float(anchor_weight)

    def resid(p):
        cams, gTc = unpack(p)
        r = []
        for (kind, a, s, ttype, T_obs) in recs:
            Cs = target_pose(p, ttype, s)
            if kind == "fix":
                r.append(se3_residual(cams[a] @ T_obs, Cs))
            else:
                r.append(se3_residual(sc.bTg[a] @ gTc @ T_obs, Cs))
        # FK soft anchor (gauge 고정): 자유 큐브를 FK prior 로 약하게 당김 (CP_C1 정합).
        #   corr 는 de-biased FK(fk_prior)를 씀 → 상수 오정렬 제거된 prior 로 당김.
        if aw > 0.0 and cube_free:
            for s in train_sets:
                if ("cube", s) in idx:
                    r.append(aw * se3_residual(target_pose(p, "cube", s), fk_prior[s]))
        # gripped: 고정카메라 @ 관측 == bTg_grip @ X (로봇 모션 기반 eye-to-hand)
        if use_grip:
            Xm = vec_to_se3(p[idx[("X",)]:idx[("X",)]+6])
            for (ci, ge, T_obs) in grip_recs:
                if ci in cams:
                    r.append(se3_residual(cams[ci] @ T_obs, sc.bTg_grip[ge] @ Xm))
        return np.concatenate(r) if r else np.zeros(1)

    sol = least_squares(resid, p0, method="trf", loss="huber", f_scale=0.02,
                        max_nfev=max_nfev)   # robust loss (모든 방법 동일)
    cams, gTc = unpack(sol.x)
    model = {"cams": cams, "gTc": gTc, "mode": f"unified/{fk_mode}"}
    if use_grip:
        model["X"] = vec_to_se3(sol.x[idx[("X",)]:idx[("X",)]+6])
    return model


# ---------------------------------------------------------------- 독립(independent)
def solve_independent(sc, markers, fk_mode, train_sets, fk_prior=None):
    """고정 카메라와 그리퍼를 *따로* 풀고 base 에서 조합(공유 타깃 rigid 정합).
       fk_mode='fixed' → 큐브=raw FK 고정. corr(fk_prior 주어짐) → 큐브=de-biased FK 고정.
       둘 다 각 카메라 FK prior 로 절대 역산 (align 불필요). none → visual only."""
    cam_ids = sc.fixed_cam_ids
    use_cube = "cube" in markers

    # --- FK prior 로 고정 카메라 절대 역산: fixed=raw FK, corr=de-biased FK ---
    anchor_cube = sc.fk_cube if (fk_mode == "fixed") else fk_prior
    if anchor_cube is not None and use_cube:
        cams = {}
        for ci in cam_ids:
            Ts = [anchor_cube[s] @ inv_T(sc.obs_fix_cube[(ci, s)]) for s in train_sets if (ci,s) in sc.obs_fix_cube]
            if Ts: cams[ci] = se3_avg(Ts)
        gTc = _handeye_to_fk(sc, train_sets, anchor_cube)   # 그리퍼도 같은 FK prior 에 정합
        return {"cams": cams, "gTc": gTc, "mode": "indep/" + fk_mode, "align": None}

    # none/corr: 고정 카메라는 관측 합의(FK 초기화 후 카메라 합의). 그리퍼는 따로 핸드아이.
    cams0, _ = _bootstrap(sc, markers, train_sets)
    # 큐브 합의(고정 카메라만)로 카메라 정제
    cams = cams0
    if use_cube:
        cube_c = {}
        for s in train_sets:
            Ts = [cams0[ci] @ sc.obs_fix_cube[(ci, s)] for ci in cam_ids if ci in cams0 and (ci,s) in sc.obs_fix_cube]
            if Ts:
                cube_c[s] = se3_avg(Ts)
        cams = {}
        for ci in cam_ids:
            Ts = [cube_c[s] @ inv_T(sc.obs_fix_cube[(ci, s)])
                  for s in train_sets if s in cube_c and (ci,s) in sc.obs_fix_cube]
            cams[ci] = se3_avg(Ts) if Ts else cams0[ci]
    # 그리퍼 핸드아이 (독립: 고정 정보 미사용). none/corr 은 FK/GT 미사용(visual free-target).
    obs_g = sc.obs_grip_cube if use_cube else sc.obs_grip_board
    gTc = _handeye_freetarget(sc, train_sets, obs_g)
    # 조합: 그리퍼가 본 큐브 vs 고정이 본 큐브를 base 에서 rigid 정합
    align = _rigid_align(sc, cams, gTc, markers, train_sets)
    return {"cams": cams, "gTc": gTc, "mode": "indep/" + fk_mode, "align": align}


def _handeye_to_fk(sc, train_sets, cube_prior=None):
    """그리퍼 gTc 를 FK 큐브 절대위치에 정합 (독립 핸드아이). cube_prior=raw 또는 de-biased FK."""
    if cube_prior is None:
        cube_prior = sc.fk_cube
    g = []
    for e in [e for s in train_sets for e in sc.set_events[s]]:
        if e not in sc.obs_grip_cube: continue
        s = sc.event_set[e]
        g.append(inv_T(sc.bTg[e]) @ cube_prior[s] @ inv_T(sc.obs_grip_cube[e]))
    return se3_avg(g) if g else np.eye(4)


def _rigid_align(sc, cams, gTc, markers, train_sets):
    """독립 조합: (고정 예측 큐브) vs (그리퍼 예측 큐브) 를 rigid 정합 → 그리퍼계를 고정 base 로."""
    if "cube" not in markers:
        return None
    P_grip, P_fix = [], []
    for s in train_sets:
        Tf = [cams[ci] @ sc.obs_fix_cube[(ci, s)] for ci in sc.fixed_cam_ids if ci in cams and (ci,s) in sc.obs_fix_cube]
        Tg = [sc.bTg[e] @ gTc @ sc.obs_grip_cube[e] for e in sc.set_events[s] if e in sc.obs_grip_cube]
        if Tf and Tg:
            P_fix.append(se3_avg(Tf)[:3, 3]); P_grip.append(se3_avg(Tg)[:3, 3])
    if len(P_fix) >= 3:
        return fit_rigid(P_grip, P_fix)
    return None


# ---------------------------------------------------------------- FK 후보정 (corr)
def _feat(t, degree=1):
    """위치 특징. degree=1: [1,x,y] (CP_C1 정합, 기본). degree=2: [1,x,y,x²,y²,xy,z]
    (intrinsic 편향 등 비선형 systematic 까지 학습)."""
    x, y, z = t[0], t[1], t[2]
    if degree >= 2:
        return np.array([1.0, x, y, x*x, y*y, x*y, z])
    return np.array([1.0, x, y])


def learn_fk_correction(sc, model, train_sets, lam=1e-3, degree=1):
    """train 에서 (예측 큐브위치 vs FK) 잔차를 특징에 Ridge 회귀 → 계수 W.
    degree=1: [1,x,y](CP_C1 동일). degree=2: 2차 특징(intrinsic 편향 학습 강화)."""
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
    return {"W": W, "degree": degree}                  # degree 를 함께 반환(적용 시 동일 특징)


def apply_fk_correction(p, W):
    """예측 위치 p 에 후보정 적용. W 는 {'W','degree'} dict 또는 None."""
    if W is None:
        return p
    return p + _feat(p, W["degree"]) @ W["W"]
