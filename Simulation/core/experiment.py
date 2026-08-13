"""
통합 러너 — 실험 설정(ExpConfig) 하나를 받아 캘리브+평가 수행. 7개 실험이 공유.

ExpConfig:
  name     : 실험 이름
  fk       : "none" | "fixed" | "corr"     (FK 사용 방식)
  solve    : "unified" | "independent"     (통합 vs 따로)
  markers  : ("cube","board") | ("cube",) | ("board",)

제약(상위에서 검증): board only + FK(fixed/corr) 는 불가(보드는 FK 없음).
"""
from dataclasses import dataclass
from typing import Tuple
import itertools
import numpy as np

from .scene import SimScene
from .methods import (solve_unified, solve_independent,
                      build_production_fk_anchors, learn_fk_correction)
from .metrics import eval_model


@dataclass(frozen=True)
class ExpConfig:
    name: str
    fk: str                      # none | fixed | corr
    solve: str                   # unified | independent
    markers: Tuple[str, ...]     # ("cube","board") 등
    label: str = ""
    # Step3 production defaults. Legacy soft-anchor weight is deliberately not
    # exposed here because it is a different algorithm from production corr.
    prior_max_dt_mm: float = 35.0
    prior_max_dr_deg: float = 8.0
    # Gate threshold policy. "adaptive" derives the limits from the spread of the
    # per-set distances, which is what those distances actually measure
    # (deviation from the common delta), floored by vision's own scatter.
    # "fixed" keeps the legacy constants above and exists only for comparison.
    gate_mode: str = "adaptive"     # adaptive | fixed
    gate_k: float = 2.5
    gate_use_floor: bool = True
    # ── corrected-FK 변형 ────────────────────────────────────
    #   hard : gate 통과 anchor 를 고정 (현재 방식)
    #   soft : anchor 를 벌점으로만 당기고 큐브는 자유변수로 둔다
    #   rel  : 절대 anchor 대신 세트 쌍의 상대 변환만 FK 로 구속한다
    #   init : anchor 는 초기값으로만 쓰고 제약은 걸지 않는다
    corr_variant: str = "hard"      # hard | soft | rel | init | auto | iv
    corr_lambda: float = 0.3        # soft/rel 의 가중치
    corr_weight_by_support: bool = False   # 관측 적은 세트일수록 FK 를 더 믿는다
    post_correction: str = "none"   # none | ridge (C1 output correction, separate axis)
    fk_degree: int = 1              # Ridge feature degree when explicitly enabled

    def validate(self):
        if set(self.markers) == {"board"} and self.fk in ("fixed", "corr"):
            raise ValueError(f"{self.name}: board only + FK({self.fk}) 불가 (보드는 FK 없음)")
        if self.fk not in ("none", "fixed", "corr"):
            raise ValueError(f"bad fk={self.fk}")
        if self.solve not in ("unified", "independent"):
            raise ValueError(f"bad solve={self.solve}")
        if self.post_correction not in ("none", "ridge"):
            raise ValueError(f"bad post_correction={self.post_correction}")
        if self.gate_mode not in ("fixed", "adaptive"):
            raise ValueError(f"bad gate_mode={self.gate_mode}")
        if self.corr_variant not in ("hard", "soft", "rel", "init", "auto", "iv"):
            raise ValueError(f"bad corr_variant={self.corr_variant}")


def calibrate(sc, cfg: ExpConfig, train_sets):
    """설정대로 캘리브.
       - none  : FK 미사용 (anchor 없음). 순수 카메라 기반.
       - corr  : **Step3 production 방식** — raw FK 를 vision 으로 상수 de-bias한 뒤
                 35mm/8deg gate를 통과한 set만 alpha=0.25로 vision anchor와 blend.
                 이 set anchor를 고정한 refinement는 Step3 D-2와 같은 역할이다.
       - fixed : 큐브를 raw FK 상수로 하드 고정 (de-bias 안 함 → systematic FK 에 취약, 대조군).
       - post_correction='ridge': 위 solve와 독립인 C1 [1,x,y] 출력 후보정.
    """
    cfg.validate()
    fk_solve = cfg.fk
    aw = 0.0
    rel_w = 0.0
    aw_by_set = None
    fk_prior = None
    prior_diag = None
    if cfg.fk == "corr":
        # Step3 ordering: obtain a vision solution first, then align the nominal
        # set priors to its set-wise cube consensus before the refinement pass.
        visual_model = solve_unified(
            sc, cfg.markers, "none", train_sets,
            anchor_weight=0.0, fk_prior=None)
        aligned = build_production_fk_anchors(
            sc, cfg.markers, train_sets,
            max_prior_dt_mm=cfg.prior_max_dt_mm,
            max_prior_dr_deg=cfg.prior_max_dr_deg,
            gate_mode=cfg.gate_mode,
            gate_k=cfg.gate_k,
            gate_use_floor=cfg.gate_use_floor,
            visual_model=visual_model)
        prior_diag = aligned.diagnostics
        if aligned.anchors:
            # iv 는 gate 대신 세트별 가중치로 신뢰도를 표현하므로
            # gate 로 걸러진 anchor 대신 보정된 FK 전체를 넘긴다.
            fk_prior = (aligned.corrected if cfg.corr_variant == "iv"
                        else aligned.anchors)
            if cfg.corr_variant == "hard":
                fk_solve = "fixed"   # anchor 를 상수로 고정 (기본)
            else:
                fk_solve = "none"    # 큐브는 자유변수로 두고 방식별로 다르게 쓴다
                if cfg.corr_variant == "soft":
                    aw = cfg.corr_lambda
                elif cfg.corr_variant == "rel":
                    rel_w = cfg.corr_lambda
                # init 은 아무 제약도 걸지 않는다(anchor 는 초기화 정보로만 남음)
                if cfg.corr_variant == "iv":
                    # 상수가 하나도 없는 방식.
                    #   각 세트마다 두 값을 잰다.
                    #     sigma_s : 그 세트 vision 합의의 흔들림 (관측들의 산포)
                    #     d_s     : 보정된 FK 가 vision 합의에서 벗어난 거리
                    #   가중치 w_s = sigma_s / sqrt(sigma_s^2 + d_s^2)
                    #   FK 가 vision 의 흔들림 안에 있으면 w -> 1 (FK 를 믿는다)
                    #   FK 가 크게 벗어나면 w -> 0 (그 세트에서 FK 를 버린다)
                    #   문턱값도, 배율도, 잘라내기도 없다. gate 를 대체한다.
                    aw = 1.0
                    aw_by_set = {}
                    for s_key, v in (prior_diag or {}).get("per_set", {}).items():
                        sig = v.get("vision_scatter_mm")
                        dts_ = v.get("prior_blend_dt_mm")
                        if sig is None or dts_ is None:
                            continue
                        aw_by_set[int(s_key)] = float(
                            sig / np.hypot(sig, dts_)) if (sig or dts_) else 1.0
                elif cfg.corr_variant == "auto":
                    # FK anchor 를 얼마나 믿을지 데이터에서 정한다.
                    #   sigma_V = vision 합의 자신의 흔들림 (gate 하한으로 이미 계산됨)
                    #   sigma_F = de-bias 후 남은 FK 잔차 (gate 거리의 중앙값)
                    # 둘의 비가 곧 상대 신뢰도다. FK 잔차가 vision 흔들림보다 작으면
                    # anchor 를 강하게, 크면 약하게 당긴다. 극단에서 각각
                    # fixed-FK 와 no-FK 로 수렴한다.
                    dg = prior_diag or {}
                    sV = dg.get("gate_floor_dt_mm") or 0.0
                    dts = [v.get("prior_blend_dt_mm") for v in
                           dg.get("per_set", {}).values()
                           if v.get("prior_blend_dt_mm") is not None]
                    sF = float(np.median(dts)) if dts else 0.0
                    if sV > 0 and sF > 0:
                        aw = float(np.clip(cfg.corr_lambda * (sV / sF), 0.02, 5.0))
                    else:
                        aw = cfg.corr_lambda
                if cfg.corr_weight_by_support and cfg.corr_variant == "soft":
                    sup = (prior_diag or {}).get("per_set", {})
                    # 관측이 적은 세트일수록 FK 를 더 믿는다
                    aw_by_set = {}
                    for s_key, v in sup.items():
                        n = max(int(v.get("support", 1)), 1)
                        aw_by_set[int(s_key)] = cfg.corr_lambda * (6.0 / n)
        else:
            fk_solve = "none"   # no reliable visual alignment: do not trust raw FK silently
    if cfg.solve == "unified":
        model = solve_unified(sc, cfg.markers, fk_solve, train_sets,
                              anchor_weight=aw, fk_prior=fk_prior,
                              rel_weight=rel_w, anchor_weight_by_set=aw_by_set)
    else:
        model = solve_independent(sc, cfg.markers, fk_solve, train_sets, fk_prior=fk_prior)
    model["requested_fk_mode"] = cfg.fk
    model["prior_diagnostics"] = prior_diag
    W = (learn_fk_correction(sc, model, train_sets, degree=cfg.fk_degree)
         if cfg.post_correction == "ridge" else None)
    return model, W


def run_config(cfg: ExpConfig, seeds=20, n_sets=10, sigma_px=0.3, train_size=8,
               fk_noise_mm=0.0, fk_noise_deg=0.0, n_fixed_cams=3,
               n_events_per_set=6, n_splits=3,
               fk_sys_mm=0.0, fk_sys_deg=0.0,
               intrinsic_err=0.0, outlier_rate=0.0,
               n_gripped_events=0,
               fk_slip_sets=0, fk_slip_mm=0.0, fk_slip_deg=0.0,
               fk_sys_res_ratio=0.4,
               corner_bias_px=0.0, outlier_focus_cam=None,
               max_cams_per_set=None,
               intrinsic_jitter=0.0, use_real_layout=False):
    """한 설정을 여러 seed × (seed당 n_splits 개의 train/test holdout)로 평가.
       코너 수준(실물 마커 투영→PnP). sigma_px = 코너 픽셀 노이즈.
       n_splits: seed 당 평가할 holdout 조합 수 (전체는 느려 제한; 통계는 seed 수로 확보)."""
    cfg.validate()
    import numpy as _np
    KEYS = ["N_reg", "e_X_mm", "e_X_deg", "e_task_mm", "e_task_deg",
            "e_cross_mm", "bTf_mm", "gTc_mm", "e_reproj_px"]
    acc = {k: [] for k in KEYS}
    for seed in range(seeds):
        sc = SimScene(seed=seed, n_fixed_cams=n_fixed_cams, n_sets=n_sets,
                      n_events_per_set=n_events_per_set, sigma_px=sigma_px,
                      fk_noise_mm=fk_noise_mm, fk_noise_deg=fk_noise_deg,
                      fk_sys_mm=fk_sys_mm, fk_sys_deg=fk_sys_deg,
                      intrinsic_err=intrinsic_err, outlier_rate=outlier_rate,
                      n_gripped_events=n_gripped_events,
                      fk_slip_sets=fk_slip_sets, fk_slip_mm=fk_slip_mm,
                      fk_slip_deg=fk_slip_deg,
                      fk_sys_res_ratio=fk_sys_res_ratio,
                      corner_bias_px=corner_bias_px,
                      outlier_focus_cam=outlier_focus_cam,
                      max_cams_per_set=max_cams_per_set,
                      intrinsic_jitter=intrinsic_jitter,
                      use_real_layout=use_real_layout)
        reproj_seed = float(_np.mean(list(sc.reproj.values()))) if getattr(sc, "reproj", None) else None
        sets = sc.sets
        splits = 0
        for test_sets in itertools.combinations(sets, 2):
            rest = [s for s in sets if s not in test_sets]
            train_sets = rest[:train_size]               # 첫 조합만 (seed 로 다양성 확보)
            model, W = calibrate(sc, cfg, list(train_sets))
            res = eval_model(sc, model, list(train_sets), list(test_sets), W=W)
            for k in KEYS:
                if k == "e_reproj_px":
                    if reproj_seed is not None:
                        acc[k].append(reproj_seed)
                elif res.get(k) is not None:
                    acc[k].append(res[k])
            splits += 1
            if splits >= n_splits:
                break
    # (평균, 표준편차, 개수, 중앙값). 발산 seed 가 섞이면 평균이 크게 흔들리므로
    # 중앙값을 함께 돌려준다. 기존 호출부는 앞 세 개만 쓰므로 영향이 없다.
    return {k: (float(np.mean(v)), float(np.std(v)), len(v), float(np.median(v)))
            if v else (None, None, 0, None)
            for k, v in acc.items()}


def summarize(cfg: ExpConfig, stats: dict) -> str:
    """한 줄 요약 문자열."""
    def g(k):
        m = stats.get(k, (None,))[0]
        return f"{m:.3f}" if m is not None else "—"
    return (f"[{cfg.name}] {cfg.label}\n"
            f"   N_reg={g('N_reg')}  e_X={g('e_X_mm')}mm/{g('e_X_deg')}°  "
            f"e_task={g('e_task_mm')}mm/{g('e_task_deg')}°  e_cross={g('e_cross_mm')}mm")
