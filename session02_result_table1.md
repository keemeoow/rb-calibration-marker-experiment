# Session02 Table 1 결과와 비교별 해석

이 문서는 session02의 Table 1 비교실험 결과만 분리해 정리한 결과 보고서다. 표시 수치는 세 canonical CSV를 기준으로 하며, `tools/verify_table1_visual_sync.py`로 Markdown·Interactive HTML과의 일치 여부를 검증한다.

## 바로가기

- [0. 한눈에 보는 결론](#0-한눈에-보는-결론)
- [1. 데이터 원천과 읽는 법](#1-데이터-원천과-읽는-법)
- [2. 조건별 현재 결과](#2-조건별-현재-결과)
- [3. 비교쌍별 변화량과 해석](#3-비교쌍별-변화량과-해석)
- [4. Marker-system End-to-End 결과](#4-marker-system-end-to-end-결과)
- [5. 현재 결론과 후속 검증](#5-현재-결론과-후속-검증)

## 0. 한눈에 보는 결론

> **현재 가장 강한 내부 후보는 A3(FK-fixed 통합)**다. 다만 pixel과 고정카메라 간 일관성은 좋아진 반면 `e_e2e`가 악화됐으므로, 물리적으로 가장 정확한 최종 방법이라고 확정할 단계는 아니다.

| 연구 질문 | 공정 비교 | 결과 요약 | 현재 말할 수 있는 결론 |
| --- | --- | --- | --- |
| 통합 최적화가 유효한가? | A1 → A2, B1 → A4 | 대부분 개선, primary px는 약 0.01 px 개선 | 효과 방향은 일관되지만 크기는 작음 |
| FK를 고정하면 좋은가? | A2 → A3 | pixel·`e_cross` 개선, `e_e2e` 악화 | 강한 후보이나 실측 FK 검증 필요 |
| Soft-FK가 필요한가? | A2 → A4 | 전 지표가 미세하게 개선 | Simulation prior 결과라 아직 입증되지 않음 |
| Board와 cube를 함께 써야 하는가? | B2 → A4, B3 → A2 | pixel과 path 지표 사이 trade-off | 어느 marker도 모든 지표를 지배하지 않음 |
| 최종 방법을 결정할 수 있는가? | A3 ↔ A4 + 외부 검증 | 내부 지표의 우승 조건이 다름 | FK jig와 blind external GT 후 결정 |

## 1. 데이터 원천과 읽는 법

표시 숫자는 다음 세 파일에서 읽는다.

- `CP_result/session02/late_table1/table1_results.csv`
- `CP_result/session02/cross_target_evaluation/cross_target_evaluation.csv`
- `CP_result/session02/marker_system_end_to_end/marker_system_end_to_end.csv`

모든 오차는 작을수록 좋다. 각 실행 조건은 기본 3개 초기 seed를 모두 수렴했고 고정카메라 3대를 등록했다. 다만 이 3개 seed는 같은 데이터에서 solver 초기값만 바꾼 것이므로 독립 반복실험이나 통계적 유의성 검정으로 해석하면 안 된다.

A4/B1/B2는 실측 FK covariance가 아니라 Simulation prior를 쓴 **preflight** 결과이며, A5는 correction label과 적용 규칙이 확정되지 않아 실행하지 않았다.

### 1.1 상태 표시

| 상태 | 의미 | 최종 주장 사용 여부 |
| --- | --- | --- |
| `complete` | 현재 계약대로 실행 완료 | 내부 비교에 사용 가능 |
| `preflight` | Simulation FK covariance 사용 | 경향 확인만 가능 |
| `not run` | 조건 정의 또는 실측 입력 대기 | 비교·순위에서 제외 |

### 1.2 지표를 읽는 기준

| 지표 | 단위 | 무엇을 평가하는가 | 조건 간 직접 비교 |
| --- | --- | --- | --- |
| `Test px` | px | 각 행의 held-out raw-corner reprojection | **같은 marker population끼리만** 가능 |
| `Common px` | px | 같은 18 observations/338 corners의 cross-target reprojection | 모든 실행 조건에 공통 적용 가능 |
| `Pixel transfer` | px | 한 고정카메라의 cube 관측을 다른 고정카메라로 옮긴 양방향 오차 | 모든 실행 조건에 공통 적용 가능 |
| `e_cross` | mm/deg | 동일 cube의 고정카메라별 pose 상호 일관성 | 모든 실행 조건에 공통 적용 가능 |
| `e_e2e` | mm/deg | eye-in-hand와 eye-to-hand 경로의 cube pose 불일치 | 모든 실행 조건에 공통 적용 가능 |

`e_cross`는 FK, eye-in-hand 카메라, 정답 cube pose를 사용하지 않는다. 즉 일반적인 fixed-camera pose consistency와 같은 정의다. `e_e2e`는 내부 경로 일관성이지 외부 절대 정확도는 아니다.

## 2. 조건별 현재 결과

### 2.1 실험 조건 요약

| ID | Marker | 계산 방식 | Cube/FK 처리 | 역할 |
| --- | --- | --- | --- | --- |
| A0 | board | 순차 | cube 없음 | board-only 순차 baseline |
| A1 | board+cube | 순차 | vision 자유변수 | cube가 포함된 순차 baseline |
| A2 | board+cube | 통합 | vision 자유변수 | vision-only 통합법 |
| A3 | board+cube | 통합 | FK pose로 hard fixed | FK-fixed 통합법 |
| A4 | board+cube | 통합 | soft FK factor | corrected-FK 통합법 |
| A5 | 미정 | 미실행 | 실측 correction label 대기 | 향후 실험 예약 |
| B1 | board+cube | 순차 | soft FK factor | A4의 순차 대조군 |
| B2 | cube | 통합 | soft FK factor | A4에서 board residual 제거 |
| B3 | board | 통합 | cube 없음 | A2에서 cube residual 제거 |

### 2.2 전체 수치

표의 `전체/board/cube`는 항상 이 순서로 읽는다. 굵은 값은 조건이 다른 행을 무리하게 섞지 않고, 비교 가능한 범위에서만 표시했다.

| ID | 상태 | Test px 전체/board/cube | Common px 전체/board/cube | Pixel transfer px | `e_cross` mm/deg | `e_e2e` mm/deg |
| --- | --- | --- | --- | ---: | --- | --- |
| A0 | complete | 2.4829 / 2.4829 / — | 1.8082 / 1.8970 / 1.6029 | 11.3422 | 14.0135 / 1.1060 | 4.9059 / 1.2229 |
| A1 | complete | 3.1267 / 2.5103 / 4.5317 | 1.9587 / 1.8856 / 2.1058 | 10.9179 | 13.7546 / 1.2193 | 4.2459 / 1.0239 |
| A2 | complete | 3.1155 / 2.5038 / 4.5113 | 1.9108 / 1.8851 / 1.9645 | 10.5265 | 13.1259 / 1.1910 | 4.0685 / 1.0385 |
| A3 | complete | **2.4996** / 2.4951 / **2.5136** | **1.7093** / **1.8537** / **1.3514** | **9.0609** | **10.8317 / 1.0601** | 4.7905 / 1.1893 |
| A4 | preflight | 3.0959 / 2.5038 / 4.4556 | 1.9052 / 1.8835 / 1.9505 | 10.4774 | 13.0413 / 1.1899 | 4.0330 / 1.0313 |
| A5 | not run | — | — | — | — | — |
| B1 | preflight | 3.1057 / 2.5100 / 4.4724 | 1.9470 / 1.8837 / 2.0753 | 10.8253 | 13.6109 / 1.2157 | 4.1987 / 1.0184 |
| B2 | preflight | 4.4759 / — / 4.4759 | 2.5281 / 2.3287 / 2.9075 | 9.2952 | 11.9161 / 1.2866 | **3.8215 / 0.9311** |
| B3 | complete | 2.4829 / 2.4829 / — | 1.8082 / 1.8970 / 1.6030 | 11.3398 | 14.0102 / 1.1062 | 4.9051 / 1.2225 |

A3의 굵은 값은 **같은 board+cube full 조건에서** 가장 좋은 값이다. A0/B3의 Test overall 2.4829 px가 A3보다 작더라도 board corner만 포함하므로 A3의 board+cube overall과 직접 순위를 매길 수 없다. B2의 `e_e2e` 최솟값도 cube-only + Simulation-prior 조건이므로 전체 방법의 절대 우승을 의미하지 않는다.

## 3. 비교쌍별 변화량과 해석

아래 Δ는 `오른쪽 조건 − 왼쪽 조건`이다. 오차 지표이므로 **음수(−)는 개선**, **양수(+)는 악화**다. `Test same-pop`은 두 조건이 실제로 공유하는 marker component만 비교한다. `*`는 Simulation prior 기반 preflight가 포함된 비교다.

| 비교 | Test same-pop px Δ | Common px Δ | Transfer px Δ | `e_cross` mm/deg Δ | `e_e2e` mm/deg Δ | 판정 |
| --- | ---: | ---: | ---: | --- | --- | --- |
| A0 → B3: board 순차→통합 | 0.0000 (board) | 0.0000 | −0.0024 | −0.0033 / +0.0002 | −0.0008 / −0.0004 | 차이 없음 |
| A0 → A1: 순차법에 cube 추가 | +0.0274 (board) | +0.1505 | −0.4243 | −0.2589 / +0.1133 | −0.6600 / −0.1990 | 혼합 |
| A1 → A2: vision 순차→통합 | −0.0112 (overall) | −0.0479 | −0.3914 | −0.6287 / −0.0283 | −0.1774 / +0.0146 | 소폭 개선 |
| B1 → A4: soft-FK 순차→통합 | −0.0098 (overall) | −0.0418 | −0.3479 | −0.5696 / −0.0258 | −0.1657 / +0.0129 | 소폭 개선* |
| A2 → A3: vision→FK-fixed | −0.6159 (overall) | −0.2015 | −1.4656 | −2.2942 / −0.1309 | **+0.7220 / +0.1508** | pixel/cross 개선, e2e 악화 |
| A2 → A4: vision→soft-FK | −0.0196 (overall) | −0.0056 | −0.0491 | −0.0846 / −0.0011 | −0.0355 / −0.0072 | 미세 개선* |
| A3 → A4: hard→soft FK | +0.5963 (overall) | +0.1959 | +1.4165 | +2.2096 / +0.1298 | **−0.7575 / −0.1580** | 지표 간 trade-off* |
| B2 → A4: board residual 추가 | −0.0203 (cube) | −0.6229 | +1.1822 | +1.1252 / −0.0967 | +0.2115 / +0.1002 | pixel/path 혼합* |
| B3 → A2: cube residual 추가 | +0.0209 (board) | +0.1026 | −0.8133 | −0.8843 / +0.0848 | −0.8366 / −0.1840 | path translation 개선 |

### 3.1 A0 ↔ B3 — Board-only 순차와 통합

Held-out board는 모두 2.4829 px이고 common overall도 1.8082 px로 사실상 같다. 나머지 차이도 0.0033 mm 이하이므로 현재 데이터에서는 board-only 통합 solver의 측정 가능한 이득이 없다.

**발표 표현:** “Board-only 조건에서는 순차와 통합이 같은 최적점에 도달해 통합 이득이 관측되지 않았습니다.”

### 3.2 A0 → A1 — 순차법에서 cube 추가

같은 board held-out은 0.0274 px, common overall은 0.1505 px 악화된다. 반면 pixel transfer, `e_cross` translation, `e_e2e` translation/rotation은 개선되고 `e_cross` rotation은 악화된다. Cube는 영상 적합 전체를 지배적으로 개선하기보다 전역 translation과 eih–e2h 경로 연결을 보강하는 trade-off를 보인다.

**발표 표현:** “Cube 추가는 전체 pixel 적합보다 카메라 간 translation과 robot 경로 연결에 기여했습니다.”

### 3.3 A1 → A2 — Vision-only 통합 효과

Test overall은 0.0112 px, common overall은 0.0479 px 개선된다. Pixel transfer와 `e_cross` translation/rotation, `e_e2e` translation도 개선되지만 `e_e2e` rotation은 0.0146° 악화된다. 통합 효과의 방향은 대체로 일관되지만 primary px 개선폭은 작다.

**발표 표현:** “Vision-only에서도 통합 효과는 여러 지표에서 같은 방향이지만, held-out pixel 개선폭은 작습니다.”

### 3.4 B1 → A4 — Soft-FK 통합 효과

Test overall 0.0098 px, common overall 0.0418 px, pixel transfer 0.3479 px가 개선되고 translation 지표도 개선된다. A1→A2와 유사한 패턴이므로 통합 효과가 vision-only에만 국한되지는 않는다는 내부 근거가 된다. 다만 둘 다 Simulation covariance를 사용한 preflight라 최종 일반성 근거는 아니다.

**발표 표현:** “Soft-FK에서도 통합 효과의 방향은 재현됐지만, 현재는 Simulation prior 기반 예비 결과입니다.”

### 3.5 A2 → A3 — FK-fixed 효과

A3는 Test/common px, pixel transfer, `e_cross`에서 크게 개선되어 현재 full board+cube 조건의 강한 내부 후보가 된다. 그러나 `e_e2e` translation/rotation은 0.7220 mm/0.1508° 악화된다. Hard FK가 고정카메라와 cube 영상 관계는 강하게 정렬하지만 robot FK와 eye-in-hand를 포함한 전체 경로와는 더 불일치할 수 있음을 뜻한다.

**발표 표현:** “A3가 pixel과 고정카메라 일관성에서는 가장 좋지만, 전체 robot 경로 지표는 악화돼 실측 FK 검증 전에는 최종법으로 확정하지 않았습니다.”

### 3.6 A2 → A4 — Soft corrected-FK factor 효과

표시 지표는 모두 개선 방향이지만 변화량은 작다. 현재 Simulation prior는 visual 해를 FK 방향으로 약하게 이동시키는 수준이며, 이 수치만으로 FK residual correction의 필요성을 입증할 수 없다.

**발표 표현:** “Soft-FK는 일관된 미세 개선을 보였지만, 실측 covariance가 없어 필요성을 입증한 결과로 보지는 않습니다.”

### 3.7 A3 ↔ A4 — Hard fixed와 soft factor

A3는 pixel 및 고정카메라 상대 일관성이 좋고, A4는 `e_e2e`가 좋다. Hard FK는 fixed-camera/cube 관계를 강제하고 soft FK는 robot/eye-in-hand 경로와 절충한다. 최종 선택은 내부 지표 가중치가 아니라 독립 물리 GT와 실제 FK repeatability로 결정해야 한다.

**발표 표현:** “Hard와 soft FK의 우승 지표가 다르므로 외부 물리 GT로 최종 선택하겠습니다.”

### 3.8 B2 → A4 — Board residual 기여

Board 추가는 same-pop Test cube와 common target pixel 적합, `e_cross` rotation을 개선한다. 반대로 pixel transfer와 translation/eih–e2h path 일부는 악화한다. Board 병용의 필요성은 px 도메인에서 강하지만 모든 path metric에서 우월하다고 말할 수는 없다.

**발표 표현:** “Board는 공통 target의 pixel 적합을 크게 개선하지만 translation/path 지표에는 trade-off가 있습니다.”

### 3.9 B3 → A2 — Cube residual 기여

Cube 추가는 held-out board와 common overall, `e_cross` rotation을 악화하지만 pixel transfer, `e_cross` translation, `e_e2e` translation/rotation은 개선한다. Cube의 주된 효과는 전역 translation 및 eih–e2h 경로 연결 강화로 해석하는 것이 타당하다.

**발표 표현:** “Cube의 핵심 기여는 pixel 전체 개선이 아니라 고정카메라 translation과 eih–e2h 경로 연결 강화입니다.”

## 4. Marker-system End-to-End 결과

이 표는 shared all-marker baseline에서 residual만 제거한 B2/B3와 다르다. 각 시스템이 초기화부터 해당 modality만 사용하므로 실제 marker 구성의 end-to-end 비교에 사용한다.

| 시스템 | Own held-out px | Common px 전체/board/cube | Pixel transfer px | `e_cross` mm/deg | `e_e2e` mm/deg |
| --- | ---: | --- | ---: | --- | --- |
| board-only | **2.4829** | **1.8083** / 1.8970 / **1.6032** | 11.3397 | 14.0100 / **1.1062** | 4.9044 / 1.2222 |
| cube-only | 4.5346 | 2.5710 / 2.3583 / 2.9739 | **9.4462** | **12.1463** / 1.2721 | **3.9188 / 0.9427** |
| board+cube | 3.1155 | 1.9108 / **1.8851** / 1.9645 | 10.5265 | 13.1259 / 1.1910 | 4.0685 / 1.0385 |

Own held-out는 marker population이 달라 직접 순위를 매기면 안 된다. 같은 338 common corners에서는 board-only가 overall/cube px, board+cube가 board px, cube-only가 pixel transfer와 translation/eih–e2h path에서 각각 가장 좋다. 즉 한 시스템이 모든 지표를 지배하지 않으며, board는 공통 내부 reference에 대한 pixel 적합, cube는 카메라 상대 translation과 robot 경로 일관성, board+cube는 두 성질 사이의 절충으로 나타난다.

## 5. 현재 결론과 후속 검증

1. A1→A2와 B1→A4에서 통합 효과의 방향은 대체로 일관되지만 primary px 개선폭은 약 0.01 px로 작다.
2. A3는 full board+cube 조건에서 Test/common px, pixel transfer, `e_cross`가 가장 좋다.
3. A3의 `e_e2e`는 A2/A4보다 나쁘므로 hard FK가 물리적으로 가장 정확하다는 결론은 아직 낼 수 없다.
4. A4는 A2보다 전반적으로 조금 좋지만 Simulation prior이고 개선폭도 작아 soft FK 보정의 필요성은 아직 입증되지 않았다.
5. Cube는 전역 translation/eih–e2h 연결을, board는 common pixel 적합을 주로 개선하지만 어느 쪽도 모든 지표를 동시에 개선하지 않는다.
6. FK repeatability 실측과 blind external-GT 전에는 A3/A4/B2 또는 특정 marker system을 최종 우승자로 선언하지 않는다.

다음 실험에서는 눈금 cube jig로 FK repeatability와 bias를 직접 측정하고, 사전 등록한 실측 covariance로 A4/B1/B2를 재실행한다. 이후 캘리브레이션에 사용하지 않은 위치의 external GT에서 A3와 A4를 paired 비교해 최종 방법을 결정한다.
