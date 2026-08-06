"""
비교 실험 설정 (확정 리스트). core.ExpConfig 로 정의, 각 run_*.py 가 가져다 씀.

Ours (EXP1) = 통합 BA + 큐브·보드 동시 + **공분산 가중 robust FK factor**.
  FK 는 후처리 회귀가 아니라 BA 안의 잔차 블록이며 회전까지 함께 구속된다.
  sigma_FK / Huber f_scale 은 core.methods 모듈 상수로 **전 실험 동결** (스크립트별 조정 금지).

절제 논리 (EXP1 에서 하나씩 제거):
  EXP1 Ours (큐브+보드 · 통합 · FK factor)
   ├─ −통합       → EXP2 (따로 풀고 rigid 조합)
   ├─ −보드       → EXP3 (큐브만)
   ├─ −FK         → EXP4 (순수 시각)
   ├─ −FK −통합   → EXP5
   ├─ −큐브       → EXP6 (보드만. FK 자동 불가)
   └─ factor→고정 → EXP7 (FK 하드 고정)
  EXP8 은 FK 를 후처리 Ridge 로 쓰던 **구 방식**(위치만 보정, 회전 미보정) — 비교군.

주의: N_reg 가 다른 행끼리 e_X/bTf 를 직접 비교하면 안 된다 (EXP6 은 이 배치에서
      고정 카메라 3대 중 일부만 보드를 관측 → 등록 대수가 줄어 오차가 낮게 보임).
"""
from core import ExpConfig

CB = ("cube", "board")
C = ("cube",)
B = ("board",)

EXP1 = ExpConfig("EXP1", fk="factor", solve="unified",     markers=CB, label="Ours (통합·큐브+보드·FK factor)")
EXP2 = ExpConfig("EXP2", fk="factor", solve="independent", markers=CB, label="−통합 (따로)")
EXP3 = ExpConfig("EXP3", fk="factor", solve="unified",     markers=C,  label="−보드 (큐브만)")
EXP4 = ExpConfig("EXP4", fk="none",   solve="unified",     markers=CB, label="−FK (순수 시각)")
EXP5 = ExpConfig("EXP5", fk="none",   solve="independent", markers=CB, label="−FK −통합")
EXP6 = ExpConfig("EXP6", fk="none",   solve="unified",     markers=B,  label="−큐브 (보드만)")
EXP7 = ExpConfig("EXP7", fk="fixed",  solve="unified",     markers=CB, label="FK 하드 고정")
# 구 방식(비교군): 순수 시각 캘리브 + 예측 위치에 Ridge 후보정. 회전은 보정하지 않음.
EXP8 = ExpConfig("EXP8", fk="corr",   solve="unified",     markers=CB, label="구 방식 (Ridge 후보정)")

ALL = [EXP1, EXP2, EXP3, EXP4, EXP5, EXP6, EXP7, EXP8]

# 여러 스크립트가 공유하는 핵심 4방식 (Ours vs 주요 대안)
CORE4 = [
    ExpConfig("ours",  fk="factor", solve="unified", markers=CB, label="Ours (FK factor)"),
    ExpConfig("fixed", fk="fixed",  solve="unified", markers=CB, label="fixed-FK"),
    ExpConfig("noFK",  fk="none",   solve="unified", markers=CB, label="no-FK"),
    ExpConfig("corr",  fk="corr",   solve="unified", markers=CB, label="구 Ridge 후보정"),
]
