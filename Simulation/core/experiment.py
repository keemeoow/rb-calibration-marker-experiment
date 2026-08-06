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
                      debias_fk_prior)
from .metrics import eval_model


@dataclass(frozen=True)
class ExpConfig:
    name: str
    fk: str                      # none | fixed | corr
    solve: str                   # unified | independent
    markers: Tuple[str, ...]     # ("cube","board") 등
    label: str = ""
    fk_degree: int = 1           # corr 후보정 특징 차수: 1=[1,x,y](CP_C1), 2=2차(강화)
    anchor_weight: float = 5.0   # corr 1차 soft anchor 세기 (0=ours-A, >0=ours-B)

    def validate(self):
        if set(self.markers) == {"board"} and self.fk in ("fixed", "corr"):
            raise ValueError(f"{self.name}: board only + FK({self.fk}) 불가 (보드는 FK 없음)")
        if self.fk not in ("none", "fixed", "corr"):
            raise ValueError(f"bad fk={self.fk}")
        if self.solve not in ("unified", "independent"):
            raise ValueError(f"bad solve={self.solve}")


def calibrate(sc, cfg: ExpConfig, train_sets):
    """설정대로 캘리브.
       - none  : FK 미사용 (anchor 없음). 순수 카메라 기반.
       - corr  : **real 방식** — raw FK 를 vision 으로 상수 de-bias(debias_fk_prior) 후
                 그 de-biased FK 를 soft anchor 로 사용. (예전의 'FK 로 예측을 당기는' 후보정
                 Ridge 는 real 과 방향이 반대라 제거.)
       - fixed : 큐브를 raw FK 상수로 하드 고정 (de-bias 안 함 → systematic FK 에 취약, 대조군).
    """
    cfg.validate()
    fk_solve = "none" if cfg.fk == "corr" else cfg.fk    # corr 캘리브는 큐브 자유(+anchor)
    aw = cfg.anchor_weight if cfg.fk == "corr" else 0.0  # corr 만 soft anchor (0=ours-A)
    fk_prior = None
    if cfg.fk == "corr":
        fk_prior = debias_fk_prior(sc, cfg.markers, train_sets)   # FK 를 vision 에 맞춰 de-bias
    if cfg.solve == "unified":
        model = solve_unified(sc, cfg.markers, fk_solve, train_sets,
                              anchor_weight=aw, fk_prior=fk_prior)
    else:
        model = solve_independent(sc, cfg.markers, fk_solve, train_sets, fk_prior=fk_prior)
    return model, None   # W 후보정 제거 (de-bias 가 real 방식의 보정)


def run_config(cfg: ExpConfig, seeds=20, n_sets=10, sigma_px=0.3, train_size=8,
               fk_noise_mm=0.0, fk_noise_deg=0.0, n_fixed_cams=3,
               n_events_per_set=6, n_splits=3):
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
                      fk_noise_mm=fk_noise_mm, fk_noise_deg=fk_noise_deg)
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
    return {k: (float(np.mean(v)), float(np.std(v)), len(v)) if v else (None, None, 0)
            for k, v in acc.items()}


def summarize(cfg: ExpConfig, stats: dict) -> str:
    """한 줄 요약 문자열."""
    def g(k):
        m = stats.get(k, (None,))[0]
        return f"{m:.3f}" if m is not None else "—"
    return (f"[{cfg.name}] {cfg.label}\n"
            f"   N_reg={g('N_reg')}  e_X={g('e_X_mm')}mm/{g('e_X_deg')}°  "
            f"e_task={g('e_task_mm')}mm/{g('e_task_deg')}°  e_cross={g('e_cross_mm')}mm")
