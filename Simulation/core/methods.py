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
                recs.append(("fix", ci, s, "cube", sc.obs_fix_cube[(ci, s)]))
        for e in [e for s in train_sets for e in sc.set_events[s]]:
            recs.append(("grip", e, sc.event_set[e], "cube", sc.obs_grip_cube[e]))
    if "board" in markers:
        for ci in sc.fixed_cam_ids:
            for s in train_sets:
                recs.append(("fix", ci, s, "board", sc.obs_fix_board[(ci, s)]))
        for e in [e for s in train_sets for e in sc.set_events[s]]:
            recs.append(("grip", e, sc.event_set[e], "board", sc.obs_grip_board[e]))
    return recs


def _bootstrap(sc, markers, train_sets):
    """초기값: 고정 카메라(FK 큐브 or 관측 합의), gTc(그리퍼 관측 합의)."""
    cams = {}
    for ci in sc.fixed_cam_ids:
        Ts = []
        if "cube" in markers:
            Ts += [sc.fk_cube[s] @ inv_T(sc.obs_fix_cube[(ci, s)]) for s in train_sets]
        if Ts:
            cams[ci] = se3_avg(Ts)
        elif "board" in markers:                      # board only: nominal = GT (gauge 근사)
            cams[ci] = sc.bTf[ci].copy()
    # gTc 초기: 그리퍼가 본 타깃을 base 로 (FK 큐브 or 보드 GT)
    g = []
    if "cube" in markers:
        for e in [e for s in train_sets for e in sc.set_events[s]]:
            s = sc.event_set[e]
            g.append(inv_T(sc.bTg[e]) @ sc.fk_cube[s] @ inv_T(sc.obs_grip_cube[e]))
    elif "board" in markers:
        for e in [e for s in train_sets for e in sc.set_events[s]]:
            g.append(inv_T(sc.bTg[e]) @ sc.bTboard @ inv_T(sc.obs_grip_board[e]))
    gTc = se3_avg(g) if g else np.eye(4)
    return cams, gTc


# ---------------------------------------------------------------- 통합(unified) BA
def solve_unified(sc, markers, fk_mode, train_sets, max_nfev=80):
    """모든 관측을 하나의 비선형 최소제곱으로 동시 최적화.
       fk_mode='fixed' 면 큐브를 FK 상수로 고정(미지수 제외)."""
    cam_ids = sc.fixed_cam_ids
    cams0, gTc0 = _bootstrap(sc, markers, train_sets)
    use_cube = "cube" in markers
    use_board = "board" in markers
    cube_free = use_cube and (fk_mode != "fixed")

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
                Ts += [cams0[ci] @ sc.obs_fix_cube[(ci, s)] for ci in cam_ids if ci in cams0]
                Ts += [sc.bTg[e] @ gTc0 @ sc.obs_grip_cube[e] for e in sc.set_events[s]]
            cube0[s] = se3_avg(Ts) if Ts else sc.fk_cube[s]
            idx[("cube", s)] = off; off += 6; p0.append(se3_to_vec(cube0[s]))
    if use_board:
        Ts = [cams0[ci] @ sc.obs_fix_board[(ci, s)]
              for ci in cam_ids if ci in cams0 for s in train_sets]
        Ts += [sc.bTg[e] @ gTc0 @ sc.obs_grip_board[e]
               for s in train_sets for e in sc.set_events[s]]
        board0 = se3_avg(Ts) if Ts else sc.bTboard
        idx[("board",)] = off; off += 6; p0.append(se3_to_vec(board0))
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
        return sc.fk_cube[s]                            # fixed: 상수

    def resid(p):
        cams, gTc = unpack(p)
        r = []
        for (kind, a, s, ttype, T_obs) in recs:
            Cs = target_pose(p, ttype, s)
            if kind == "fix":
                r.append(se3_residual(cams[a] @ T_obs, Cs))
            else:
                r.append(se3_residual(sc.bTg[a] @ gTc @ T_obs, Cs))
        return np.concatenate(r) if r else np.zeros(1)

    sol = least_squares(resid, p0, method="lm", max_nfev=max_nfev)
    cams, gTc = unpack(sol.x)
    return {"cams": cams, "gTc": gTc, "mode": f"unified/{fk_mode}"}


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
            Ts = [sc.fk_cube[s] @ inv_T(sc.obs_fix_cube[(ci, s)]) for s in train_sets]
            cams[ci] = se3_avg(Ts)
        gTc = _handeye_to_fk(sc, train_sets)            # 그리퍼도 FK 큐브에 정합
        return {"cams": cams, "gTc": gTc, "mode": "indep/fixed", "align": None}

    # none/corr: 고정 카메라는 관측 합의(FK 초기화 후 카메라 합의). 그리퍼는 따로 핸드아이.
    cams0, _ = _bootstrap(sc, markers, train_sets)
    # 큐브 합의(고정 카메라만)로 카메라 정제
    cams = cams0
    if use_cube:
        cube_c = {}
        for s in train_sets:
            Ts = [cams0[ci] @ sc.obs_fix_cube[(ci, s)] for ci in cam_ids if ci in cams0]
            if Ts:
                cube_c[s] = se3_avg(Ts)
        cams = {}
        for ci in cam_ids:
            Ts = [cube_c[s] @ inv_T(sc.obs_fix_cube[(ci, s)])
                  for s in train_sets if s in cube_c]
            cams[ci] = se3_avg(Ts) if Ts else cams0[ci]
    # 그리퍼 핸드아이 (독립: 고정 정보 미사용). none/corr 은 FK 미사용(순수 AX=XB).
    if use_cube:
        gTc = _handeye_freecube(sc, train_sets)          # FK 미사용 순수 핸드아이
    else:
        gTc = _handeye_to_board(sc, train_sets)          # 보드만
    # 조합: 그리퍼가 본 큐브 vs 고정이 본 큐브를 base 에서 rigid 정합
    align = _rigid_align(sc, cams, gTc, markers, train_sets)
    return {"cams": cams, "gTc": gTc, "mode": "indep/" + fk_mode, "align": align}


def _handeye_to_fk(sc, train_sets):
    """그리퍼 gTc 를 FK 큐브 절대위치에 정합 (fixed 모드 독립 핸드아이)."""
    g = []
    for e in [e for s in train_sets for e in sc.set_events[s]]:
        s = sc.event_set[e]
        g.append(inv_T(sc.bTg[e]) @ sc.fk_cube[s] @ inv_T(sc.obs_grip_cube[e]))
    return se3_avg(g) if g else np.eye(4)


def _handeye_freecube(sc, train_sets, max_nfev=60):
    """그리퍼만으로 gTc 추정 (FK 미사용, 순수 AX=XB). 큐브 위치를 미지수로 두고
       '같은 set 큐브는 이벤트 무관 상수'라는 제약으로 gTc·cube[s] 동시 최적화.
       base gauge 는 로봇 자세 bTg 가 제공. 자세 다양성 낮으면 gTc 병진이 약하게 구속됨
       (= 통합이 고정 카메라로 이걸 보완하는 부분)."""
    events = [e for s in train_sets for e in sc.set_events[s]]
    if len(events) < 3:
        return np.eye(4)
    # 초기: FK 큐브로 대략 (초기값일 뿐, 잔차엔 FK 미사용)
    gTc0 = _handeye_to_fk(sc, train_sets)
    cube0 = {s: se3_avg([sc.bTg[e] @ gTc0 @ sc.obs_grip_cube[e] for e in sc.set_events[s]])
             for s in train_sets if sc.set_events[s]}
    sets = [s for s in train_sets if s in cube0]
    p0 = np.concatenate([se3_to_vec(gTc0)] + [se3_to_vec(cube0[s]) for s in sets])
    cidx = {s: 6 + i * 6 for i, s in enumerate(sets)}

    def resid(p):
        gTc = vec_to_se3(p[:6])
        r = []
        for s in sets:
            Cs = vec_to_se3(p[cidx[s]:cidx[s]+6])
            for e in sc.set_events[s]:
                r.append(se3_residual(sc.bTg[e] @ gTc @ sc.obs_grip_cube[e], Cs))
        return np.concatenate(r) if r else np.zeros(1)

    sol = least_squares(resid, p0, method="lm", max_nfev=max_nfev)
    return vec_to_se3(sol.x[:6])


def _handeye_to_board(sc, train_sets):
    """보드만: 그리퍼 gTc 를 보드(자유 pose)에 대해 AX=XB 로. 여기선 GT 보드로 근사 정합."""
    g = []
    for e in [e for s in train_sets for e in sc.set_events[s]]:
        g.append(inv_T(sc.bTg[e]) @ sc.bTboard @ inv_T(sc.obs_grip_board[e]))
    return se3_avg(g) if g else np.eye(4)


def _rigid_align(sc, cams, gTc, markers, train_sets):
    """독립 조합: (고정 예측 큐브) vs (그리퍼 예측 큐브) 를 rigid 정합 → 그리퍼계를 고정 base 로."""
    if "cube" not in markers:
        return None
    P_grip, P_fix = [], []
    for s in train_sets:
        Tf = [cams[ci] @ sc.obs_fix_cube[(ci, s)] for ci in sc.fixed_cam_ids if ci in cams]
        Tg = [sc.bTg[e] @ gTc @ sc.obs_grip_cube[e] for e in sc.set_events[s]]
        if Tf and Tg:
            P_fix.append(se3_avg(Tf)[:3, 3]); P_grip.append(se3_avg(Tg)[:3, 3])
    if len(P_fix) >= 3:
        return fit_rigid(P_grip, P_fix)
    return None


# ---------------------------------------------------------------- FK 후보정 (corr)
def _feat(t):
    """위치 특징 [1, x, y]."""
    return np.array([1.0, t[0], t[1]])


def learn_fk_correction(sc, model, train_sets, lam=1e-3):
    """train 에서 (예측 큐브위치 vs FK) 잔차를 [1,x,y] Ridge 회귀 → W(3x3)."""
    from .metrics import predict_cube_pos          # 지연 import (순환 방지)
    X, Y = [], []
    for s in train_sets:
        p = predict_cube_pos(sc, model, s)
        if p is None:
            continue
        X.append(_feat(p)); Y.append(sc.fk_cube[s][:3, 3] - p)
    if len(X) < 3:
        return None
    X = np.array(X); Y = np.array(Y)
    reg = lam * np.eye(3); reg[0, 0] = 0.0             # 절편 정규화 제외
    return np.linalg.solve(X.T @ X + reg, X.T @ Y)


def apply_fk_correction(p, W):
    """예측 위치 p 에 후보정 적용."""
    return p if W is None else p + _feat(p) @ W
