"""
7개 실험 설정 (확정 리스트). core.ExpConfig 로 정의, experiments/ 의 스크립트가 가져다 씀.

절제 논리 (기본 EXP1 에서 하나씩 제거):
  EXP1 Ours (큐브+보드 · 통합 · FK보정)
   ├─ −통합       → EXP2
   ├─ −보드       → EXP3 (큐브만)
   ├─ −FK보정     → EXP4 (FK없음)
   ├─ −FK −통합   → EXP5
   ├─ −큐브       → EXP6 (보드만, FK 자동 불가)
   └─ FK보정→고정 → EXP7 (fixed, 통합=독립)
"""
from core import ExpConfig

CB = ("cube", "board")
C = ("cube",)
B = ("board",)

# anchor_weight 는 모든 corr 방법에서 0.5 로 동결 (스크립트 간 불일치 제거; 리뷰 ⑤).
EXP1 = ExpConfig("EXP1", fk="corr",  solve="unified",     markers=CB, anchor_weight=0.5, label="Ours (FK보정·통합·큐브+보드)")
EXP2 = ExpConfig("EXP2", fk="corr",  solve="independent", markers=CB, anchor_weight=0.5, label="−통합 (따로)")
EXP3 = ExpConfig("EXP3", fk="corr",  solve="unified",     markers=C,  anchor_weight=0.5, label="−보드 (큐브만)")
EXP4 = ExpConfig("EXP4", fk="none",  solve="unified",     markers=CB, label="−FK (FK 안씀)")
EXP5 = ExpConfig("EXP5", fk="none",  solve="independent", markers=CB, label="−FK −통합")
EXP6 = ExpConfig("EXP6", fk="none",  solve="unified",     markers=B,  label="−큐브 (보드만)")
EXP7 = ExpConfig("EXP7", fk="fixed", solve="unified",     markers=CB, label="FK 고정 (통합=독립)")
# EXP8: Ours + 2차 후보정 특징 — 참고용(표본 적으면 과적합. 기본 sweep 에선 제외).
EXP8 = ExpConfig("EXP8", fk="corr",  solve="unified",     markers=CB, label="Ours+ (2차 후보정)", fk_degree=2)

ALL = [EXP1, EXP2, EXP3, EXP4, EXP5, EXP6, EXP7]      # 기본 7방식 (EXP8은 참고용, 별도)
