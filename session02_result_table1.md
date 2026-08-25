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

> **현재 inter-camera 내부 후보는 A3(FK-fixed 통합)**다. held-out/common pixel과 병진 일관성은 가장 좋지만 회전 지표에서는 다른 방법이 더 좋아, 물리적으로 가장 정확한 최종 방법이라고 확정할 단계는 아니다.

| 연구 질문 | 공정 비교 | 결과 요약 | 현재 말할 수 있는 결론 |
| --- | --- | --- | --- |
| 통합 최적화가 유효한가? | A1 → A2, B1 → A4 | pixel·translation 개선, 회전 소폭 악화 | 병진에는 유효하나 회전까지 일방 개선은 아님 |
| FK를 고정하면 좋은가? | A2 → A3 | held-out/common px와 병진 개선, 회전 혼합 | 강한 후보이나 실측 FK 검증 필요 |
| Soft-FK가 필요한가? | A2 → A4 | 대부분 미세 개선, `e_cross` 회전만 0.0029° 악화 | Simulation prior 결과라 아직 입증되지 않음 |
| Board와 cube를 함께 써야 하는가? | B2 → A4, B3 → A2 | 현재 held-out에서는 일관된 우위 없음 | 어느 marker도 모든 지표를 지배하지 않음 |
| 최종 방법을 결정할 수 있는가? | A3 ↔ A4 + 외부 검증 | 내부 지표의 우승 조건이 다름 | FK jig와 blind external GT 후 결정 |

## 1. 데이터 원천과 읽는 법

표시 숫자는 다음 세 파일에서 읽는다.

- `CP_result/session02/late_table1/table1_results.csv`
- `CP_result/session02/cross_target_evaluation/cross_target_evaluation.csv`
- `CP_result/session02/marker_system_end_to_end/marker_system_end_to_end.csv`

모든 오차는 작을수록 좋다. 각 실행 조건은 기본 3개 초기 seed를 모두 수렴했고 고정카메라 3대를 등록했다. 다만 이 3개 seed는 같은 데이터에서 solver 초기값만 바꾼 것이므로 독립 반복실험이나 통계적 유의성 검정으로 해석하면 안 된다.

A4/B1/B2는 실측 FK covariance가 아니라 Simulation prior를 쓴 **preflight** 결과이며, A5는 correction label과 적용 규칙이 확정되지 않아 실행하지 않았다.

이번 재실행은 현재 `meta.json`에 포함된 event 90–95를 반영했다. 공통 split seed는 `20260731`, held-out event는 `[29, 38, 46, 50, 53, 61, 69, 75, 92]`다. event 0–89의 tool3/177.5 mm 기록과 event 90–95의 flange/143 mm 기록은 명시적 manifest의 강체변환으로 flange/143 mm에 통일했다. 정규화 후 set 11의 cube-center 최대 차이는 0.679 mm이며, manifest와 정지성 진단도 baseline provenance에 포함된다. 과거 산출물은 이 계약과 달라 직접 수치 비교 대상으로 사용하지 않는다.

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
| `Common px` | px | 같은 12 observations/229 corners의 cross-target reprojection | 모든 실행 조건에 공통 적용 가능 |
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
| A0 | complete | 2.4675 / 2.4675 / — | 2.0193 / 2.1445 / 1.7148 | 11.7971 | 14.2795 / 0.8603 | **2.9446** / 0.9976 |
| A1 | complete | 2.9942 / 2.5090 / 4.2264 | 2.0975 / 2.2221 / 1.7962 | 12.7282 | 15.8616 / **0.8086** | 3.7887 / **0.7973** |
| A2 | complete | 2.9715 / 2.4863 / 4.2015 | 2.0628 / 2.2345 / 1.6268 | 12.0222 | 14.8495 / 0.8437 | 3.4324 / 0.8177 |
| A3 | complete | **2.5315** / 2.5019 / **2.6276** | **2.0105** / **2.2072** / **1.4945** | **9.6520** | **11.5631** / 0.8442 | **3.1686** / 0.9167 |
| A4 | preflight | 2.9525 / **2.4861** / 4.1435 | 2.0592 / 2.2310 / 1.6228 | 11.9717 | 14.7545 / 0.8466 | 3.3922 / 0.8144 |
| A5 | not run | — | — | — | — | — |
| B1 | preflight | 2.9729 / 2.5082 / 4.1624 | 2.0903 / 2.2169 / 1.7833 | 12.6368 | 15.7124 / 0.8119 | 3.7334 / 0.7944 |
| B2 | preflight | 4.2202 / — / 4.2202 | 2.9214 / 3.0008 / 2.7403 | 11.1316 | 14.0157 / 1.0872 | 4.1085 / 0.8250 |
| B3 | complete | 2.4675 / 2.4675 / — | 2.0195 / 2.1446 / 1.7152 | 11.7964 | 14.2776 / 0.8608 | 2.9440 / 0.9978 |

A3의 굵은 값은 **같은 board+cube full 조건에서** held-out/common pixel과 병진 지표가 가장 좋은 값이다. 다만 `e_cross` 회전은 A1, `e_e2e` 회전은 B1이 더 작다. marker population이 다른 A0/B3/B2의 자체 Test overall은 직접 순위를 매길 수 없다.

## 3. 비교쌍별 변화량과 해석

아래 Δ는 `오른쪽 조건 − 왼쪽 조건`이다. 오차 지표이므로 **음수(−)는 개선**, **양수(+)는 악화**다. `Test same-pop`은 두 조건이 실제로 공유하는 marker component만 비교한다. `*`는 Simulation prior 기반 preflight가 포함된 비교다.

| 비교 | Test same-pop px Δ | Common px Δ | Transfer px Δ | `e_cross` mm/deg Δ | `e_e2e` mm/deg Δ | 판정 |
| --- | ---: | ---: | ---: | --- | --- | --- |
| A0 → B3: board 순차→통합 | +0.0000 (board) | +0.0002 | −0.0007 | −0.0019 / +0.0005 | −0.0006 / +0.0002 | 차이 없음 |
| A0 → A1: 순차법에 cube 추가 | +0.0415 (board) | +0.0782 | +0.9311 | +1.5821 / −0.0517 | +0.8441 / −0.2003 | 혼합 |
| A1 → A2: vision 순차→통합 | −0.0227 (overall) | −0.0347 | −0.7060 | −1.0121 / +0.0351 | −0.3563 / +0.0204 | pixel·translation 개선 |
| B1 → A4: soft-FK 순차→통합 | −0.0204 (overall) | −0.0311 | −0.6651 | −0.9579 / +0.0347 | −0.3412 / +0.0200 | pixel·translation 개선* |
| A2 → A3: vision→FK-fixed | −0.4400 (overall) | −0.0523 | −2.3702 | −3.2864 / +0.0005 | −0.2638 / +0.0990 | pixel·병진 개선, 회전 혼합 |
| A2 → A4: vision→soft-FK | −0.0190 (overall) | −0.0036 | −0.0505 | −0.0950 / +0.0029 | −0.0402 / −0.0033 | 거의 모두 미세 개선* |
| A3 → A4: hard→soft FK | +0.4210 (overall) | +0.0487 | +2.3197 | +3.1914 / +0.0024 | +0.2236 / −0.1023 | A3 병진·pixel, A4 e2e 회전* |
| B2 → A4: board residual 추가 | −0.0767 (cube) | −0.8622 | +0.8401 | +0.7388 / −0.2406 | −0.7163 / −0.0106 | pixel/path 혼합* |
| B3 → A2: cube residual 추가 | +0.0188 (board) | +0.0433 | +0.2258 | +0.5719 / −0.0171 | +0.4884 / −0.1801 | 병진 악화, 회전 개선 |

### 3.1 A0 ↔ B3 — Board-only 순차와 통합

Held-out board 차이는 반올림 기준 0.0000 px, common overall 차이는 0.0002 px다. path 병진 차이도 최대 0.0019 mm이므로 현재 데이터에서는 board-only 통합 solver의 실질적인 이득이 없다.

**발표 표현:** “Board-only 조건에서는 순차와 통합이 같은 최적점에 도달해 통합 이득이 관측되지 않았습니다.”

### 3.2 A0 → A1 — 순차법에서 cube 추가

같은 board held-out은 0.0415 px, common overall은 0.0782 px 악화된다. Pixel transfer, `e_cross` translation, `e_e2e` translation도 악화되는 반면 두 회전 지표는 개선된다. 현재 sequential 조건에서 cube 추가는 translation을 보강했다고 말할 수 없고, rotation과 translation 사이 trade-off로 봐야 한다.

**발표 표현:** “현재 sequential 조건에서 Cube 추가는 회전 일관성 일부를 개선했지만 pixel과 translation 지표는 악화했습니다.”

### 3.3 A1 → A2 — Vision-only 통합 효과

Test overall은 0.0227 px, common overall은 0.0347 px 개선된다. Pixel transfer, `e_cross` translation, `e_e2e` translation도 개선되지만 `e_cross`/`e_e2e` rotation은 각각 0.0351°/0.0204° 악화된다. 통합 효과는 pixel과 translation 계열에서 같은 방향이고 회전에는 작은 trade-off가 있다.

**발표 표현:** “Vision-only 통합은 translation 계열을 개선했지만 회전은 소폭 악화했고, held-out pixel 차이는 매우 작았습니다.”

### 3.4 B1 → A4 — Soft-FK 통합 효과

Test overall 0.0204 px, common overall 0.0311 px, pixel transfer 0.6651 px가 개선되고 translation 지표도 개선된다. 회전 지표는 소폭 악화되어 A1→A2와 같은 패턴이다. 다만 둘 다 Simulation covariance를 사용한 preflight라 최종 일반성 근거는 아니다.

**발표 표현:** “Soft-FK에서도 통합 효과의 방향은 재현됐지만, 현재는 Simulation prior 기반 예비 결과입니다.”

### 3.5 A2 → A3 — FK-fixed 효과

A3는 Test overall 0.4400 px, common overall 0.0523 px, pixel transfer 2.3702 px, `e_cross` 병진 3.2864 mm, `e_e2e` 병진 0.2638 mm를 개선한다. 반면 `e_cross` 회전은 사실상 동일(+0.0005°)하고 `e_e2e` 회전은 0.0990° 악화된다. Hard FK는 pixel과 inter-camera translation을 강하게 정렬하지만 robot-path 회전까지 동시에 개선하지는 않는다.

**발표 표현:** “A3가 held-out/common pixel과 고정카메라 병진 일관성을 개선했지만 robot-path 회전은 악화돼 실측 FK 검증 전에는 최종법으로 확정하지 않았습니다.”

### 3.6 A2 → A4 — Soft corrected-FK factor 효과

대부분의 지표는 아주 조금 개선되지만 `e_cross` 회전은 0.0029° 악화된다. 현재 Simulation prior는 visual 해를 FK 방향으로 약하게 이동시키는 수준이며, 이 수치만으로 FK factor의 필요성을 입증할 수 없다.

**발표 표현:** “Soft-FK는 일관된 미세 개선을 보였지만, 실측 covariance가 없어 필요성을 입증한 결과로 보지는 않습니다.”

### 3.7 A3 ↔ A4 — Hard fixed와 soft factor

A3는 Test/common pixel, pixel transfer, `e_cross`, `e_e2e` 병진이 좋고, A4는 `e_e2e` 회전이 0.1023° 좋다. Hard FK가 대부분의 내부 지표에서 앞서지만 최종 선택은 실제 FK repeatability와 향후 독립 물리 GT로 결정해야 한다.

**발표 표현:** “Hard와 soft FK의 우승 지표가 다르므로 외부 물리 GT로 최종 선택하겠습니다.”

### 3.8 B2 → A4 — Board residual 기여

Board 추가는 same-pop Test cube 0.0767 px, common target 0.8622 px, `e_cross` 회전 0.2406°, `e_e2e` 병진/회전을 개선한다. 반면 pixel transfer와 `e_cross` 병진은 악화한다. 따라서 board 병용은 공통 pixel과 robot-path에는 도움이 되지만 모든 inter-camera 지표를 지배하지 않는다.

**발표 표현:** “현재 population에서 Board 추가는 공통 pixel과 robot-path를 개선했지만 pixel transfer와 고정카메라 병진 일관성은 악화했습니다.”

### 3.9 B3 → A2 — Cube residual 기여

Cube 추가는 held-out board, common overall, pixel transfer, `e_cross` 병진, `e_e2e` 병진을 악화하고 두 회전 지표는 개선한다. 현재 결과에서는 cube가 전역 translation을 강화했다는 기존 해석을 유지할 수 없다.

**발표 표현:** “현재 population에서는 Cube 추가의 일관된 개선을 확인하지 못했고 회전 지표만 좋아졌습니다.”

## 4. Marker-system End-to-End 결과

이 표는 shared all-marker baseline에서 residual만 제거한 B2/B3와 다르다. 각 시스템이 초기화부터 해당 modality만 사용하므로 실제 marker 구성의 end-to-end 비교에 사용한다.

| 시스템 | Own held-out px | Common px 전체/board/cube | Pixel transfer px | `e_cross` mm/deg | `e_e2e` mm/deg |
| --- | ---: | --- | ---: | --- | --- |
| board-only | **2.4676** | **2.0195** / **2.1445** / 1.7157 | 11.7990 | 14.2816 / 0.8605 | **2.9450** / 0.9977 |
| cube-only | 4.2799 | 2.9834 / 3.0575 / 2.8150 | **11.3110** | **14.2441** / 1.0662 | 4.2222 / 0.8321 |
| board+cube | 2.9715 | 2.0628 / 2.2345 / **1.6268** | 12.0222 | 14.8495 / **0.8437** | 3.4324 / **0.8177** |

Own held-out는 marker population이 달라 직접 순위를 매기면 안 된다. 같은 229 common corners에서는 board-only가 overall/board px, board+cube가 cube px, cube-only가 pixel transfer와 `e_cross` 병진, board-only가 `e_e2e` 병진, board+cube가 두 회전 지표에서 각각 가장 좋다. 즉 한 시스템이 모든 지표를 지배하지 않으며 외부 GT 결론도 아니다.

## 5. 현재 결론과 후속 검증

1. A1→A2와 B1→A4에서 통합은 pixel과 translation 지표를 일관되게 개선하지만 회전은 소폭 악화한다.
2. A3는 full board+cube 조건에서 held-out/common pixel, pixel transfer와 병진 지표가 가장 좋다.
3. A3의 `e_e2e` 회전은 A2/A4보다 나쁘므로 hard FK가 물리적으로 가장 정확하다는 결론은 아직 낼 수 없다.
4. A4는 A2를 대부분 미세하게 개선하지만 Simulation prior이므로 soft FK factor의 필요성은 아직 입증되지 않았다.
5. 새 held-out population에서는 cube/board 추가의 효과가 지표별로 갈리며 어느 쪽도 모든 지표를 동시에 개선하지 않는다.
6. FK repeatability 실측과 blind external-GT 전에는 A3/A4/B2 또는 특정 marker system을 최종 우승자로 선언하지 않는다.

다음 실험에서는 눈금 cube jig로 FK repeatability와 bias를 직접 측정하고, 사전 등록한 실측 covariance로 A4/B1/B2를 재실행한다. 이후 캘리브레이션에 사용하지 않은 위치의 external GT에서 A3와 A4를 paired 비교해 최종 방법을 결정한다.
