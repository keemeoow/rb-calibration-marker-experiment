# A2 strict-none vs A3 FK-fixed — 동일 reprojection BA

## 판정

**A2 strict-none은 현재 `수렴 불안정`으로 보고한다.** 5개 initialization 모두 `max_nfev=300`에 도달했고 optimizer 종료조건을 만족하지 못했다. 따라서 아래 오차를 Table 1의 확정 A2 숫자로 사용하지 않는다.

공정 비교용 A3 reprojection BA도 5/5가 같은 상한에 도달했다. 이 결과는 “A2만 발산했다”는 증거가 아니라, 현재 corner-only BA와 엄격한 종료조건에서 두 모델 모두 완전 수렴하지 않았으며 A2가 더 불리한 conditioning과 초기값 민감도를 보였다는 증거다.

## 고정한 실험 조건

- 데이터: `data/session`
- train sets: 1, 2, 3, 5, 7, 8, 9, 10, 11
- held-out sets: 0, 4, 6, 12
- 공통 변수 초기값: 동일한 저장 calibration의 fixed-camera transforms와 `T_gripper_cam`
- A2 cube 초기값: FK를 사용하지 않은 visual consensus
- FK↔AprilTag object-frame 정렬: train 9개 set의 visual consensus만 사용; held-out set 사용 금지
- 목적함수: cube+board의 2D corner reprojection residual만 사용
- optimizer: SciPy `least_squares`, `trf`, Huber loss, `f_scale=2 px`
- 종료: `max_nfev=300`, `xtol=ftol=gtol=1e-8`
- initialization: nominal + 5 mm/1° Gaussian perturbation 4개, 총 5 runs
- 공통: A2/A3 모두 hand-eye 정합에 운동학 백본 `T_base_gripper=FK(q)` 사용
- A2: set별 cube pose 9개를 자유변수로 공동 추정, `FK→cube=estimated`, cube FK residual 0
- A3: 동일 set cube pose를 aligned FK에 고정
- Ridge/SE(3) post-correction: 사용하지 않음
- aligned cube FK는 A2 fitting/초기화에는 사용하지 않고 held-out 평가 proxy로만 사용

## 수렴 및 관측 가능성

| 항목 | A2 strict-none | A3 FK-fixed |
| --- | ---: | ---: |
| parameters | 84 | 30 |
| runs satisfying termination | 0/5 | 0/5 |
| Jacobian rank | 84/84 (모든 run) | 30/30 (모든 run) |
| nullity | 0 | 0 |
| Jacobian condition range | 279.8–352.2 | 56.1–131.8 |
| Gauss–Newton Hessian condition, nominal | 78,290 | 3,146 |
| nominal optimality at termination | 8.25×10⁴ | 6.29×10⁴ |
| five-run optimizer time | 562.5 s | 559.4 s |

해석:

- Nullity 0이므로 현재 데이터에서는 scale 또는 기준 프레임에 해당하는 명백한 구조적 gauge freedom이 검출되지 않았다.
- 실제 target 치수와 intrinsics가 scale을 정하고, robot base→gripper pose와 eih 관측이 base frame을 연결한다.
- 다만 A2의 nominal Hessian condition은 A3보다 약 24.9배 크다. FK로 cube 변수를 제거했을 때 수치 조건이 크게 좋아진다는 방향의 증거다.
- Full rank는 “정확하고 안정적”이라는 뜻이 아니다. 모든 run이 평가 상한까지 갔고 initialization 간 해가 달랐으므로 A2를 안정 수렴으로 판정하지 않는다.

## 참고 오차 — Table 1 확정값으로 사용 금지

| 지표, 5 init mean±std | A2 strict-none | A3 FK-fixed |
| --- | ---: | ---: |
| held-out FK-proxy translation (mm) | 4.71±0.50 | 3.35±0.42 |
| held-out FK-proxy rotation (°) | 0.79±0.14 | 0.80±0.11 |
| nominal reprojection RMSE (px) | 3.013 | 3.263 |
| nominal gated e2e (mm/°) | 9.42 / 1.74 | 8.87 / 1.77 |
| nominal gated fixed-cross (mm/°) | 12.30 / 1.63 | 9.69 / 1.68 |

A2의 training reprojection이 더 낮은 것은 cube pose 자유변수가 54개 더 많아 training pixels를 더 잘 맞출 수 있기 때문이다. 그럼에도 held-out translation은 A3보다 나쁘다. 이는 training reprojection만으로 방법 우열을 판단하면 안 된다는 사례다.

## 초기값 민감도

| transform | A2 translation std / max (mm) | A2 rotation std / max (°) | A3 translation std / max (mm) | A3 rotation std / max (°) |
| --- | ---: | ---: | ---: | ---: |
| T_gripper_cam | 1.05 / 3.18 | 0.22 / 0.63 | 1.01 / 3.12 | 0.18 / 0.49 |
| T_base_C0 | 5.27 / 15.67 | 0.73 / 2.10 | 3.62 / 10.27 | 0.64 / 1.78 |
| T_base_C1 | 3.33 / 10.45 | 0.50 / 1.35 | 2.52 / 7.35 | 0.36 / 0.95 |
| T_base_C3 | 6.80 / 21.13 | 0.60 / 1.82 | 6.99 / 21.53 | 0.59 / 1.81 |

cam3는 A2/A3 모두 큰 initialization 민감도를 보여, A2만의 문제가 아니라 해당 카메라 관측성/이상치 문제도 남아 있다.

## 산출물

- 전체 run 및 Jacobian: [`A2_strict_none_vs_A3_fk_fixed.json`](A2_strict_none_vs_A3_fk_fixed.json)
- nominal 비교 행: [`A2_strict_none_vs_A3_fk_fixed.csv`](A2_strict_none_vs_A3_fk_fixed.csv)
- 실행기: [`../../CP_A2_strict_none.py`](../../CP_A2_strict_none.py)
