# D2 — 채택 구성(soft anchor)을 canonical 이벤트 split에서 평가

Table 1의 `‖` 칸(=anchored 행의 `N_reg`/`e_e2e`/`e_cross`/`e_reproj`)을 채우기 위한 실행이다. D1은 위치 hold-out·mm 지표였고 이 표는 **Table 1과 동일한 이벤트 split·동일 지표**다.

- `CP_common.ROBOT_POS_SCALE_PINNED` = **1.0000** (로봇 원본 값 그대로). 아래 재현 확인은 이 값이 Table 1과 같을 때만 의미가 있다 — 게이트의 허용치가 split 표준편차라서 스케일 불일치를 단독으로는 걸러내지 못한다.
- split seeds: [20260729, 20260730, 20260731, 20260732, 20260733], 각 split당 3 initialization
- 총 15 run/arm, anchor lever 29.5 mm, λ 단위 px/mm
- 데이터 준비(검출·split·train 전용 FK artifact·초기화·solver 설정·path mask)는 `CP_ablation_7row.py`의 것을 그대로 재사용한다. 목적함수의 anchor 항만 다르다.

## 이벤트 split 결과 (split mean의 mean±std)

| arm | λ (px/mm) | N_reg | e_reproj overall (px) | e_e2e (mm/°) | e_cross (mm) | Jac cond | 수렴 |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| A2 | 0 | 3 | 4.5075±0.4387 | 16.1956±1.2976 / 7.7423±0.5937 | 38.9338±3.4075 | 332.9 | 15/15 |
| A3 | ∞ (hard) | 3 | 4.6230±0.4338 | 16.1879±1.2856 / 7.7087±0.5854 | 38.0480±3.5118 | 51.5 | 15/15 |
| A2@lam3 | 3 | 3 | 4.5252±0.4374 | 16.2352±1.2932 / 7.7123±0.5887 | 38.7634±3.4492 | 289.3 | 15/15 |

## Table 1 재현 확인 — 통과 (최대 0.00σ, 허용 2σ)

A2·A3는 Table 1과 같은 조건이므로 재현되어야 한다. 어긋나면 anchored 행도 Table 1과 비교할 수 없다. 검출에 RANSAC 변동이 있어 정확한 일치는 기대하지 않으며, **Table 1이 스스로 보고한 split 간 표준편차 안에 들어오는지**로 판정한다.

| arm | 지표 | Table 1 (mean±split std) | 재현값 | 차이 | σ | 판정 |
| --- | --- | ---: | ---: | ---: | ---: | :---: |
| A2 | e_e2e_translation_mm | 16.1955±1.2976 | 16.1956 | 0.0001 | 0.00 | ○ |
| A2 | e_cross_translation_mm | 38.9338±3.4075 | 38.9338 | 0.0000 | 0.00 | ○ |
| A2 | heldout_reprojection_overall_px | 4.5075±0.4387 | 4.5075 | 0.0000 | 0.00 | ○ |
| A3 | e_e2e_translation_mm | 16.1881±1.2856 | 16.1879 | 0.0002 | 0.00 | ○ |
| A3 | e_cross_translation_mm | 38.0480±3.5117 | 38.0480 | 0.0000 | 0.00 | ○ |
| A3 | heldout_reprojection_overall_px | 4.6230±0.4338 | 4.6230 | 0.0000 | 0.00 | ○ |

## A2(λ=0) 대비 paired delta — split 단위, 음수가 개선

| arm | e_reproj overall (px) | e_cross (mm) |
| --- | ---: | ---: |
| A3 | +0.1155±0.0253, t=9.13, 0/5 | -0.8858±0.1513, t=-11.71, 5/5 |
| A2@lam3 | +0.0178±0.0062, t=5.73, 0/5 | -0.1704±0.0674, t=-5.06, 5/5 |

## 해석 규칙

- 이 표의 값은 held-out **이벤트**의 재투영·경로 일치도다. D1의 위치 hold-out mm 값과 같은 줄에서 비교하지 않는다.
- 외부 GT가 없으므로 `e_e2e`·`e_cross`는 내부 일관성 지표이며 절대 정확도가 아니다.
- anchored 행은 `CP_ablation_7row.py`의 7행 계약에 포함되지 않는 보충 실행이다. 7행의 인과 비교표에 이 행을 끼워 넣지 않는다.
