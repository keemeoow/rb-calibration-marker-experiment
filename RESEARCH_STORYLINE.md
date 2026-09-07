# Research Storyline 기준 문서

검증 기준일: 2026-09-04
현재 단계: External GT 추가 전 최종 비교 프로토콜 고정

이 문서는 논문 기여도와 발표 서사의 최상위 기준이다. 최종 비교표와 평가지표는
[CALIBRATION_EXPERIMENT_VALIDATION.md](CALIBRATION_EXPERIMENT_VALIDATION.md)를
단일 기준으로 따른다.

## 0. 현재 한 문장 결론

본 연구는 여러 고정 카메라와 그리퍼 카메라를 로봇 베이스 좌표계로 정합하는
multi-camera calibration framework를 제안한다. 최종 비교는 `A0~A5`, `B1~B3`
한 벌만 사용하고, heldout 및 External GT 평가는 항상 cube target만 본다.
현재 Session04 내부 cube 지표에서는 A5가 가장 좋아 보이지만, 최종 제안 방법은
다음주 Independent External cube GT의 TRE, rotation, P95, failure rate로 확정한다.

## 1. 최종 기여도

| 기여도 | 최종 주장 | 검증 방식 |
| --- | --- | --- |
| C1. Unified multi-camera calibration | sequential/frozen-stage보다 unified feedback이 multi-camera 정합에 유리한지 검증 | A1 -> A2, B1 -> A4 |
| C2. Graspable multi-face cube | gripper-mounted cube가 board-only 대비 최종 cube 정합을 개선하는지 검증 | A0 -> A1, B3 -> A2, B2 -> A4 |
| C3. FK-aware target-pose handling | raw FK hard fixed, corrected-FK soft factor, vision-aligned FK hard fixed 중 무엇이 실제 3D 정합에 좋은지 검증 | A2 -> A3, A2 -> A4, A3/A4 -> A5 |
| C4. Real-world validation | 내부 px가 아니라 독립 External cube GT로 최종 물리 정확도를 판정 | TRE, rotation, P95, failure rate |

## 2. 최종 비교실험 구조

| Row | 역할 | 최종 해석 |
| --- | --- | --- |
| A0 | board-on-gripper only sequential baseline | board-only 순차 기준선 |
| A1 | board-on-gripper + cube sequential | cube train 관측 추가 효과 |
| A2 | board+cube unified visual-only | vision-only unified 후보 |
| A3 | board+cube unified raw-FK hard fixed | raw FK hard constraint 후보 |
| A4 | board+cube unified corrected-FK soft factor | corrected-FK soft 후보 |
| A5 | board+cube unified vision-aligned FK hard fixed | GT 전 frozen 시 최종 후보 |
| B1 | board+cube sequential corrected-FK soft factor | A4 대비 unified 효과 제거 |
| B2 | cube-only corrected-FK soft factor | A4 대비 board residual 제거 |
| B3 | board-on-gripper only unified baseline | board-only unified 기준선 |

A0/B3의 board-only 방법은 cube 촬영 포즈 다양성만큼 board를 gripper에 붙여 촬영한다.
현재 Session04 artifact는 이전 촬영 구성이므로 A0/B3 cube heldout 값은 N/A로 남을 수 있다.

## 3. 최종 평가지표

| 지표 | 역할 | 해석 |
| --- | --- | --- |
| External cube TRE / rotation / P95 / failure | 최종 주 지표 | Independent External GT와 blind prediction 비교 |
| ALL Cube RMSE px | fit sanity check | train+heldout cube 전체 재투영 |
| Train RMSE px | 수렴 진단 | 학습 적합도이며 순위 지표가 아님 |
| Heldout Cube RMSE px | 내부 보조 지표 | 미사용 cube event 재투영 |
| Cross-view pixel transfer RMSE px | 카메라 간 pixel 일관성 | fixed-camera pair와 fixed-gripper pair를 cube-only로 함께 집계 |
| Cam-common Obj-Cam consistency mm/deg | 카메라 간 3D 일관성 | 두 카메라가 본 cube pose 차이 |

제거한 지표:

- Board heldout RMSE
- board/cube pooled overall ranking
- 별도 pair-type 순위표
- set-equal/paired bootstrap을 최종 지표로 쓰는 구조

## 4. 현재 Session04 내부 관찰

현재 내부 cube 값은 최종 결론이 아니라 External GT 전 참고값이다.

| 비교 | 현재 내부 cube 결과 | 해석 |
| --- | --- | --- |
| A1 -> A2 | 4.1402 -> 3.5958 px | unified feedback이 내부 cube residual을 낮춤 |
| A2 -> A3 | 3.5958 -> 6.3959 px | raw FK hard fixed는 현재 내부 cube에서 악화 |
| A2 -> A4 | 3.5958 -> 3.5805 px | corrected-FK soft factor는 A2와 거의 동률 |
| B1 -> A4 | 4.1182 -> 3.5805 px | soft-FK 조건에서도 unified 쪽이 낮음 |
| B2 -> A4 | 4.4827 -> 3.5805 px | board residual이 cube 보정에 도움 |
| A4 -> A5 | 3.5805 -> 3.2274 px | A5가 현재 내부 cube 지표 최저 |

## 5. A5 채택 원칙

A5는 내부 cube 지표가 가장 좋으므로 최종 후보에서 제외하지 않는다. 다만 다음 조건을
만족해야 최종 제안 방법으로 채택할 수 있다.

1. External GT 공개 전에 A5 절차와 hyperparameter를 고정한다.
2. `Delta_train` 추정에 사용한 train observation list와 artifact hash를 기록한다.
3. External GT cube pose list와 failure 기준을 GT 확인 전에 고정한다.
4. External GT에서도 A5가 TRE, rotation, P95, failure rate에서 가장 좋으면 A5를 최종 방법으로 채택한다.
5. GT를 본 뒤 A5 정의를 바꾸면 사후 진단으로만 남긴다.

## 6. 발표에서 써도 되는 문장

1. "최종 비교표는 A0~A5, B1~B3 한 벌만 사용합니다."
2. "heldout 평가는 항상 cube target만 사용합니다."
3. "현재 내부 cube 지표에서는 A5가 가장 낮지만, 최종 물리 순위는 External cube GT로 결정합니다."
4. "A5는 External GT 공개 전에 방법과 artifact를 frozen하면 최종 후보로 비교할 수 있습니다."
5. "Board heldout과 board/cube pooled overall은 최종 순위 지표에서 제거했습니다."

## 7. 피해야 할 문장

| 피해야 할 문장 | 이유 | 대체 문장 |
| --- | --- | --- |
| "내부 px 최저가 곧 최종 방법이다." | 공통 systematic error를 검출하지 못함 | "최종 방법은 External cube GT로 정한다." |
| "A5는 무조건 사후 진단이다." | GT 공개 전 frozen하면 후보 method가 될 수 있음 | "A5는 frozen 여부에 따라 후보/진단이 갈린다." |
| "Board heldout도 최종 평가에 쓴다." | 최종 heldout은 cube-only | "Board는 training/ablation 관측으로만 둔다." |
| "A6도 최종 표에 넣는다." | 최종 표는 9행만 사용 | "A0~A5, B1~B3만 사용한다." |

## 8. 다음주 External GT 이후 업데이트

1. 모든 row의 blind cube prediction hash를 확인한다.
2. 동일 cube GT pose list에서 TRE, rotation, P95, failure rate를 계산한다.
3. 내부 cube 지표와 External GT가 충돌하면 External GT를 최종 기준으로 둔다.
4. A5가 최종 최고이면 A5를 제안 방법으로 채택하고, 그렇지 않으면 가장 좋은 row를 채택한다.

## 9. 근거 문서

- [CALIBRATION_EXPERIMENT_VALIDATION.md](CALIBRATION_EXPERIMENT_VALIDATION.md): 최종 실험표와 평가지표 단일 기준
- [CP_result/session04/late_table1/TABLE1_RESULTS.md](CP_result/session04/late_table1/TABLE1_RESULTS.md): 자동 생성된 현재 결과표
- [FK_use_A2-A5.md](FK_use_A2-A5.md): A2-A5 FK 사용 방식
- [8-3_meeting.md](8-3_meeting.md): 8/3 피드백 반영 현황
- [RUN_PIPELINE.md](RUN_PIPELINE.md): 재현 파이프라인
