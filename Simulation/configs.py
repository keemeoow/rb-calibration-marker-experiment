"""
7개 실험 설정 (확정 리스트). core.ExpConfig 로 정의, experiments/ 의 스크립트가 가져다 씀.

절제 논리 (기본 EXP1 에서 하나씩 제거):
  EXP1 Ours (큐브+보드 · 통합 · FK보정)
   ├─ −통합       → EXP2
   ├─ −보드       → EXP3 (큐브만)
   ├─ −FK보정     → EXP4 (FK없음)
   ├─ −FK −통합   → EXP5
   ├─ −큐브       → EXP6 (보드만, FK 자동 불가)
   └─ FK보정→raw 고정 → EXP7
"""
from core import ExpConfig

CB = ("cube", "board")
C = ("cube",)
B = ("board",)

# corr는 Step3 production 기본값(alpha=.25, gate=35mm/8deg)을 ExpConfig에서 공유한다.
EXP1 = ExpConfig("EXP1", fk="corr",  solve="unified",     markers=CB, label="Ours (Step3 FK보정·통합·큐브+보드)")
EXP2 = ExpConfig("EXP2", fk="corr",  solve="independent", markers=CB, label="−통합 (따로)")
EXP3 = ExpConfig("EXP3", fk="corr",  solve="unified",     markers=C,  label="−보드 (큐브만)")
EXP4 = ExpConfig("EXP4", fk="none",  solve="unified",     markers=CB, label="−FK (FK 안씀)")
EXP5 = ExpConfig("EXP5", fk="none",  solve="independent", markers=CB, label="−FK −통합")
EXP6 = ExpConfig("EXP6", fk="none",  solve="unified",     markers=B,  label="−큐브 (보드만)")
EXP7 = ExpConfig("EXP7", fk="fixed", solve="unified",     markers=CB, label="raw FK 고정")
# C1 Ridge는 Step3 corr와 별개의 출력 후보정 축으로 명시한다.
EXP8 = ExpConfig("EXP8", fk="corr", solve="unified", markers=CB,
                 post_correction="ridge", fk_degree=1,
                 label="Ours + C1 Ridge post-correction")

ALL = [EXP1, EXP2, EXP3, EXP4, EXP5, EXP6, EXP7]      # 기본 7방식 (EXP8은 참고용, 별도)


def build_factorial_configs():
    """Return every scientifically valid solve × marker × FK combination.

    Board-only has no robot cube prior, so fixed/corr are intentionally absent.
    This produces 14 valid cells: 2 solves × (1 board-only + 3 cube-only +
    3 cube+board FK choices).
    """
    out = []
    marker_axes = [("board", B), ("cube", C), ("both", CB)]
    for solve in ("unified", "independent"):
        for marker_name, markers in marker_axes:
            fk_modes = ("none",) if markers == B else ("none", "fixed", "corr")
            for fk in fk_modes:
                name = f"{solve}__{marker_name}__{fk}"
                out.append(ExpConfig(
                    name=name, solve=solve, markers=markers, fk=fk,
                    label=f"{solve} · {marker_name} · {fk}"))
    return out


ALL_FACTORIAL = build_factorial_configs()
