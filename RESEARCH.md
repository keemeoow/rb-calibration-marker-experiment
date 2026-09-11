# Research Storyline 기준 문서

검증 기준일: 2026-09-04
현재 단계: External GT 추가 전 최종 비교 프로토콜 고정

이 문서는 논문 기여도와 발표 서사의 최상위 기준이다. 최종 비교표와 평가지표는
[CALIBRATION_EXPERIMENT_VALIDATION.md](CALIBRATION_EXPERIMENT_VALIDATION.md)를
단일 기준으로 따른다.

## 용어 표기 계약

앞으로 논문, 발표, 문서와 자동 생성 결과에는 아래 세 표현만 기본 방법명으로 사용한다.

| 표준 표현 | 정확한 의미 | 세부 방식 표기 예시 |
| --- | --- | --- |
| **VISION** | cube pose에 robot FK prior를 연결하지 않고 영상 residual로 추정 | `VISION`, `VISION unified` |
| **FK** | controller FK pose를 별도 학습 보정 없이 사용 | `FK hard fixed` |
| **corrected-FK** | train 정보로 FK pose/frame을 보정해 사용 | `corrected-FK soft factor`, `corrected-FK hard fixed (VISION-aligned)` |

`VISION`도 eye-in-hand camera pose를 구성하기 위한 robot FK backbone은 사용한다.
따라서 세 이름은 **robot FK 전체의 사용 여부**가 아니라, calibration에서 cube target
pose를 어떻게 처리하는지를 구분한다. 내부 artifact의 과거 schema 문자열은 호환성을
위해 유지할 수 있지만 사용자에게 보이는 방법명으로 출력하지 않는다.

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
| C3. FK-aware target-pose handling | FK hard fixed, corrected-FK soft factor, corrected-FK hard fixed 중 무엇이 실제 3D 정합에 좋은지 검증 | A2 -> A3, A2 -> A4, A3/A4 -> A5 |
| C4. Real-world validation | 내부 px가 아니라 독립 External cube GT로 최종 물리 정확도를 판정 | TRE, rotation, P95, failure rate |

## 2. 최종 비교실험 구조

| Row | 역할 | 최종 해석 |
| --- | --- | --- |
| A0 | board-on-gripper only sequential baseline | board-only 순차 기준선 |
| A1 | board-on-gripper + cube sequential | cube train 관측 추가 효과 |
| A2 | board+cube unified VISION | VISION unified 후보 |
| A3 | board+cube unified FK hard fixed | FK hard constraint 후보 |
| A4 | board+cube unified corrected-FK soft factor | corrected-FK soft 후보 |
| A5 | board+cube unified corrected-FK hard fixed (VISION-aligned) | GT 전 frozen 시 최종 후보 |
| B1 | board+cube sequential corrected-FK soft factor | A4 대비 unified 효과 제거 |
| B2 | cube-only corrected-FK soft factor | A4 대비 board residual 제거 |
| B3 | board-on-gripper only unified baseline | board-only unified 기준선 |

A0/B3를 위해 별도 board 영상을 더 촬영하지 않는다. Planar board와 multi-face cube를
하나의 강체 composite target으로 제작하고, 모든 방법이 P1 moving rig 15 events,
P2 10 placements x 2 views 20 events, P3 stationary rig 10 events로 구성된 동일한
45개 raw capture event를 사용한다. A0/B3는 cube observation을 calibration 전체에서
masking하고 B2는 board observation을 masking한다. A1~A5/B1만 같은 영상의 두 target
residual을 모두 사용하므로 cube 추가 효과에 촬영 수 증가가 섞이지 않는다.

현재 Session04 artifact는 정적 workspace board를 사용한 legacy 촬영이다. A0/B3
calibration은 board-only로 유지하고, cube RMSE는 train cube로 set별 evaluation pose만
맞춘 뒤 frozen calibration으로 계산한다. 새 composite-target 결과와 섞어 동일한 최종
실험으로 해석하지 않는다.

## 3. 최종 평가지표

| 지표 | 역할 | 해석 |
| --- | --- | --- |
| External cube TRE mm / Rotation Error deg / P95 TRE / failure | **최종 주 지표** | Independent External GT와 blind prediction을 같은 pose ID로 비교 |
| ALL Cube RMSE px | full-data fit sanity check | 전체 placement로 한 번 fit한 뒤 같은 전체 cube를 재투영; 최종 순위에 사용하지 않음 |
| Train Cube RMSE px | train-split 진단 | leave-one-placement-out 각 fold의 train cube에서 계산; in-sample이므로 순위 지표가 아님 |
| Held-out Test Cube RMSE px | 내부 예측 보조 지표 | 해당 fold에서 제외한 cube placement를 frozen calibration으로 재투영 |
| ALL Cross-view Cube RMSE px | full-data camera-consistency 진단 | 전체 placement fit에서 source-only 양방향 pixel transfer |
| Train Cross-view Cube RMSE px | train camera-consistency 진단 | 각 fold의 train placement에서 source-only 양방향 pixel transfer |
| Held-out Test Cross-view Cube RMSE px | **내부 주 비교 지표** | calibration에서 제외한 placement에서 destination 관측을 pose 계산에 쓰지 않고 평가 |

세 Cube RMSE는 반드시 cube pose reference 출처를 함께 기록한다. 현재 Zeus pre-GT
구현은 모든 row에 같은 `FK-reference`를 사용하므로 비교 계산은 동일하지만 FK 계열에
구조적으로 유리하다. 따라서 이 값만으로 방법을 채택하지 않는다. Cross-view는 target
GT가 필요 없지만 카메라들의 공통 systematic error를 검출하지 못한다.

`ALL`은 `Train`과 `Held-out Test` 숫자를 단순 평균한 값이 아니다. 모든 placement를
사용해 별도로 한 번 fit한 descriptive 결과다. 최종 순위는 External GT만 결정한다.

제거한 지표:

- Board heldout RMSE
- board/cube pooled overall ranking
- 별도 pair-type 순위표
- Cam-common Obj-Cam mm/deg를 독립된 최종 증거로 쓰는 구조

## 4. 현재 Session04 내부 관찰

현재 내부 cube 값은 최종 결론이 아니라 External GT 전 참고값이다.

| 비교 | 현재 내부 cube 결과 | 해석 |
| --- | --- | --- |
| A1 -> A2 | 3.6938 -> 3.5960 px | unified feedback이 내부 cube residual을 낮춤 |
| A2 -> A3 | 3.5960 -> 6.7199 px | FK hard fixed는 현재 내부 cube에서 악화 |
| A2 -> A4 | 3.5960 -> 3.5786 px | corrected-FK soft factor는 A2와 거의 동률 |
| B1 -> A4 | 3.6870 -> 3.5786 px | corrected-FK soft factor 조건에서도 unified 쪽이 낮음 |
| B2 -> A4 | 4.4608 -> 3.5786 px | board residual이 cube 보정에 도움 |
| A4 -> A5 | 3.5786 -> 3.4180 px | A5가 현재 내부 cube 지표 최저 |

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
- [ABLATION_TEST_result/session04/ABLATION_TEST_table1/ABLATION_TEST_TABLE1_RESULTS.md](ABLATION_TEST_result/session04/ABLATION_TEST_table1/ABLATION_TEST_TABLE1_RESULTS.md): 자동 생성된 현재 결과표
- [FK_use_A2-A5.md](FK_use_A2-A5.md): A2-A5 FK 사용 방식
- [8-3_meeting.md](8-3_meeting.md): 8/3 피드백 반영 현황
- [RUN_PIPELINE.md](RUN_PIPELINE.md): 재현 파이프라인
