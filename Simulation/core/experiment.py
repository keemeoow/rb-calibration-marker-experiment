"""
통합 러너 — 실험 설정(ExpConfig) 하나를 받아 캘리브+평가 수행. 모든 실험 스크립트가 공유.

ExpConfig:
  name     : 실험 이름
  fk       : "none" | "fixed" | "factor" | "corr"
  solve    : "unified" | "independent"
  markers  : ("cube","board") | ("cube",) | ("board",)

FK 사용방식:
  none   FK 미사용 (순수 시각).
  fixed  큐브를 FK 상수로 하드 고정.
  factor 큐브 자유 + FK 를 공분산 가중 robust 잔차 인자로 BA 에 추가.   ← **Ours**
  corr   none 으로 풀고 예측 위치에 Ridge 후보정 (구 방식. 위치만 보정 → 비교군).

Ours 는 factor 하나로 동결한다. sigma_FK / Huber f_scale 은 core.methods 의 모듈 상수이며
실험별로 바꾸지 않는다 (이전 버전은 스크립트마다 anchor_weight 가 5.0/0.5/0.0 로 달랐음).

제약(상위에서 검증): board only + FK(fixed/factor/corr) 는 불가(보드는 FK 없음).

split 규약: seed 당 n_splits 개의 held-out 조합을 **seed 별 RNG 로 추출**한다.
  (이전 버전은 항상 combinations 의 앞 n_splits 개만 써서 특정 set 조합에 편향됐다.)
  split 은 같은 씬을 공유하므로 독립 표본이 아니다 → 통계는 seed 단위로 집계할 것.
"""
from dataclasses import dataclass
from typing import Tuple
import itertools
import numpy as np

from .scene import SimScene
from .methods import (solve_unified, solve_independent, learn_fk_correction)
from .metrics import eval_model

KEYS = ["N_reg", "e_X_mm", "e_X_deg", "e_task_mm", "e_task_deg", "e_task_p95_mm",
        "e_cross_mm", "bTf_mm", "bTf_deg", "gTc_mm", "gTc_deg",
        "reproj_train_px", "reproj_test_px", "reproj_test_p95_px",
        "reproj_fail_rate", "e_reproj_px", "e_reproj_gt_px"]

FK_MODES = ("none", "fixed", "factor", "corr")


@dataclass(frozen=True)
class ExpConfig:
    name: str
    fk: str                      # none | fixed | factor | corr
    solve: str                   # unified | independent
    markers: Tuple[str, ...]     # ("cube","board") 등
    label: str = ""
    fk_degree: int = 1           # corr 후보정 특징 차수 (corr 전용)

    def validate(self):
        if set(self.markers) == {"board"} and self.fk != "none":
            raise ValueError(f"{self.name}: board only + FK({self.fk}) 불가 (보드는 FK 없음)")
        if self.fk not in FK_MODES:
            raise ValueError(f"bad fk={self.fk} (허용: {FK_MODES})")
        if self.solve not in ("unified", "independent"):
            raise ValueError(f"bad solve={self.solve}")


def calibrate(sc, cfg: ExpConfig, train_sets):
    """설정대로 캘리브 → (model, W). W 는 corr 방식에서만 non-None."""
    cfg.validate()
    fk_solve = "none" if cfg.fk == "corr" else cfg.fk    # corr 캘리브는 큐브 자유
    if cfg.solve == "unified":
        model = solve_unified(sc, cfg.markers, fk_solve, train_sets)
    else:
        model = solve_independent(sc, cfg.markers, fk_solve, train_sets)
    W = None
    if cfg.fk == "corr":
        W = learn_fk_correction(sc, model, train_sets, degree=cfg.fk_degree)
    return model, W


def _splits_for_seed(sets, seed, n_splits, test_size=2):
    """seed 별로 다른 held-out 조합을 재현 가능하게 추출."""
    combos = list(itertools.combinations(sets, test_size))
    rng = np.random.default_rng(20000 + seed)
    take = min(n_splits, len(combos))
    pick = rng.choice(len(combos), size=take, replace=False)
    return [combos[i] for i in sorted(pick)]


def run_records(cfg: ExpConfig, seeds=20, n_sets=10, sigma_px=0.3, train_size=8,
                fk_noise_mm=0.0, fk_noise_deg=0.0, n_fixed_cams=3,
                n_events_per_set=6, n_splits=3, outlier_rate=0.0, intrinsic_err=0.0,
                robust_pnp=True):
    """seed × split 단위 원자료 레코드 리스트를 반환 (paired 통계용).

    각 레코드: {"seed", "split", "test_sets", **metrics}. 실패는 삼키지 않고 예외를 올린다.
    """
    cfg.validate()
    recs = []
    for seed in range(seeds):
        sc = SimScene(seed=seed, n_fixed_cams=n_fixed_cams, n_sets=n_sets,
                      n_events_per_set=n_events_per_set, sigma_px=sigma_px,
                      fk_noise_mm=fk_noise_mm, fk_noise_deg=fk_noise_deg,
                      outlier_rate=outlier_rate, intrinsic_err=intrinsic_err,
                      robust_pnp=robust_pnp)
        for si, test_sets in enumerate(_splits_for_seed(sc.sets, seed, n_splits)):
            train_sets = [s for s in sc.sets if s not in test_sets][:train_size]
            model, W = calibrate(sc, cfg, train_sets)
            res = eval_model(sc, model, train_sets, list(test_sets), W=W)
            rec = {"seed": seed, "split": si, "test_sets": list(test_sets)}
            rec.update({k: res.get(k) for k in KEYS})
            recs.append(rec)
    return recs


def aggregate(recs, by_seed=True):
    """레코드 → {key: (mean, std, n)}.

    by_seed=True 면 seed 안에서 split 을 먼저 평균한 뒤 seed 간 통계를 낸다.
    split 은 같은 씬을 공유해 독립이 아니므로 이것이 기본값이다 (n = seed 수).
    """
    if not recs:
        return {k: (None, None, 0) for k in KEYS}
    if by_seed:
        seeds = sorted({r["seed"] for r in recs})
        units = []
        for sd in seeds:
            rs = [r for r in recs if r["seed"] == sd]
            units.append({k: (float(np.mean([r[k] for r in rs if r[k] is not None]))
                              if any(r[k] is not None for r in rs) else None)
                          for k in KEYS})
    else:
        units = recs
    out = {}
    for k in KEYS:
        v = [u[k] for u in units if u.get(k) is not None]
        out[k] = (float(np.mean(v)), float(np.std(v)), len(v)) if v else (None, None, 0)
    return out


def run_config(cfg: ExpConfig, **kw):
    """한 설정을 여러 seed × split 으로 평가 → {key: (mean, std, n)} (seed 단위 집계)."""
    by_seed = kw.pop("by_seed", True)
    return aggregate(run_records(cfg, **kw), by_seed=by_seed)


def summarize(cfg: ExpConfig, stats: dict) -> str:
    """한 줄 요약 문자열."""
    def g(k):
        m = stats.get(k, (None,))[0]
        return f"{m:.3f}" if m is not None else "—"
    return (f"[{cfg.name}] {cfg.label}\n"
            f"   N_reg={g('N_reg')}  bTf={g('bTf_mm')}mm/{g('bTf_deg')}°  "
            f"gTc={g('gTc_mm')}mm/{g('gTc_deg')}°  "
            f"e_task={g('e_task_mm')}mm/{g('e_task_deg')}°  "
            f"reproj(test)={g('reproj_test_px')}px  e_cross={g('e_cross_mm')}mm")
