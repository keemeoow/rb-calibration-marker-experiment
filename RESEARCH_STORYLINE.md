# Research Storyline 기준 문서

검증 기준일: 2026-09-03  
현재 단계: 실제 Session04 데이터로 초기 연구 스토리라인을 검증하고, 논문 claim을 조정하는 단계

이 문서는 논문 기여도와 발표 서사의 최상위 기준이다. 초기 연구 기획에서 잡은 스토리라인을 유지하되, 실제 데이터가 바꾼 부분은 이 문서에서 먼저 수정한다.

## 0. 현재 한 문장 결론

본 연구는 여러 고정 카메라와 로봇 베이스를 하나의 좌표계로 묶는 unified calibration framework를 제안한다. 현재 실제 데이터에서 가장 강하게 확인된 결과는 **동일 marker population 안에서 sequential/independent 방식보다 unified visual feedback이 내부 held-out reprojection error를 낮춘다**는 점이다. Graspable multi-face cube와 FK prior는 중요한 설계 축이지만, cube 효과는 solver context에 따라 조건부이고, FK 효과는 현재 simulation covariance 기반 preflight 수준이므로 다음주 external GT에서 물리 정확도 claim을 확정해야 한다.

## 1. 초기 스토리라인

| 기여도 | 초기 의도 | 원래 기대한 검증 |
| --- | --- | --- |
| C1. Unified multi-camera robot-base calibration | 여러 eye-to-hand 카메라를 독립적으로 풀지 않고, 하나의 robot-base frame에 joint optimization으로 정렬 | B1 -> A4에서 independent/soft-FK 대비 unified/soft-FK가 우수 |
| C2. Graspable multi-face marker cube | 로봇이 잡을 수 있는 3D 공통 target으로 planar board의 관측 한계를 보완 | A0 -> A1, B3 -> A2에서 cube 추가가 성능 개선 |
| C3. Uncertainty-aware FK-grounded registration | FK를 완전한 GT로 고정하지 않고 uncertainty-aware prior로 사용 | A2 -> A3 -> A4 계열에서 no-FK, hard-FK, soft-FK 차이 확인 |
| C4. Real-world validation | 실제 로봇 manipulation 환경에서 물리 정확도와 task 성능 검증 | external GT translation/rotation error, task-level grasp accuracy |

## 2. 실제 데이터가 바꾼 내용

| 항목 | 실제 데이터에서 확인된 변화 | 현재 문서/발표에서 써야 할 문장 |
| --- | --- | --- |
| C1 검증 축 | A4는 simulation covariance preflight이므로 현재의 주 검증축으로 쓰기 어렵다. 대신 A1 -> A2가 현재 가장 깨끗한 내부 비교다. Own overall reprojection error가 4.0837 px -> 3.8901 px로 4.74% 감소했다. | "현재 데이터에서 가장 확실한 결과는 unified visual feedback이 동일 marker population의 내부 held-out residual을 낮춘다는 점이다." |
| C2 cube claim | A0 -> A1에서는 board held-out이 4.0530 px -> 4.0645 px로 약간 나빠졌다. 따라서 cube를 넣으면 항상 좋아진다고 말하면 안 된다. 다만 B3 -> A2에서는 board가 4.0531 px -> 3.9840 px로 좋아지고, B2 -> A4에서는 cube가 4.4827 px -> 3.5805 px로 좋아진다. | "Cube는 단독 추가만으로 항상 이득을 주는 장치가 아니라, board/cube가 unified solver 안에서 서로 제약을 주게 만드는 공통 3D target이다." |
| C3 FK claim | A2 -> A3에서 hard fixed FK는 board 3.9840 px -> 4.1025 px, cube 3.5958 px -> 6.3959 px로 나빠졌다. A2 -> A4는 거의 동률이다. A4는 simulation covariance 기반이라 최종 우수성 claim에는 부족하다. | "Raw FK를 GT처럼 강하게 고정하면 위험하며, FK는 uncertainty-aware prior로 다루어야 한다. 다만 soft-FK의 물리적 우수성은 다음주 GT로 확정한다." |
| C4 real-world validation | 현재는 independent external GT가 없어서 translation error, rotation error, P95, failure rate를 계산할 수 없다. | "External GT와 task-level 평가는 다음주 예정된 검증 단계다." |
| 최종 우승 모델 | 내부 residual 기준으로는 A2가 대표 결과, A4는 preflight 후보, A5는 post-hoc diagnostic이다. | "현재 논문 본문에서 final method로 과도하게 A4/A5를 밀지 않고, A2를 내부 검증 대표값으로 둔다." |

## 3. 현재 데이터 기준 최종 기여도

### C1. Unified Calibration Framework

현재 유지 가능한 가장 강한 기여도다.

- Claim: Independent/sequential calibration보다 unified visual feedback이 내부 held-out reprojection residual을 낮춘다.
- Evidence: A1 -> A2에서 own overall 4.0837 px -> 3.8901 px, board 4.0645 px -> 3.9840 px, cube 4.1402 px -> 3.5958 px.
- Boundary: 이것은 내부 reprojection 기준의 결과다. 아직 physical translation/rotation accuracy claim은 아니다.

### C2. Graspable Multi-Face Marker Cube

기여도는 유지하되 표현을 조건부로 바꾼다.

- Claim: Cube는 여러 시점에서 잡히는 3D 공통 target을 제공하고, unified solver 안에서 board와 cube 관측을 연결하는 추가 constraint가 된다.
- Evidence: B3 -> A2에서 board residual이 4.0531 px -> 3.9840 px로 감소한다. B2 -> A4에서는 cube residual이 4.4827 px -> 3.5805 px로 감소한다.
- Boundary: A0 -> A1이 개선되지 않으므로 "cube 추가 자체가 항상 성능을 올린다"는 claim은 제거한다.

### C3. Uncertainty-Aware FK Prior

현재는 "방법론적 필요성 + preflight 후보"로 둔다.

- Claim: FK를 hard GT로 쓰는 것은 안전하지 않으며, FK는 uncertainty-aware prior로 들어가야 한다.
- Evidence: A2 -> A3에서 hard fixed FK가 특히 cube residual을 크게 악화시킨다.
- Boundary: A4는 A2와 거의 동률이고 simulation covariance 기반이다. measured covariance와 external GT가 확보되기 전에는 soft-FK가 최종적으로 우수하다고 쓰지 않는다.

### C4. Real-World Validation

현재는 다음주 예정된 검증 항목이다.

- Claim: External GT와 manipulation task 결과로 물리 정확도를 검증할 계획이다.
- Evidence: 현재 없음. 다음주 데이터에서 translation error, rotation error, P95, failure rate를 계산한다.
- Boundary: 현재 논문/발표에서는 external GT 결과처럼 말하지 않는다.

## 4. 비교실험 구성 기준

| ID | 역할 | 현재 해석 |
| --- | --- | --- |
| A0 | Board-only baseline | planar board만 쓴 기본 내부 baseline |
| A1 | Board + cube sequential-style reference | cube를 같은 population에 추가했을 때의 reference |
| A2 | Board + cube unified no-FK | 현재 내부 검증의 대표 결과 |
| A3 | Board + cube unified hard-FK | raw FK hard constraint의 위험성을 보여주는 negative control |
| A4 | Board + cube unified soft-FK | simulation covariance 기반 preflight candidate |
| A5 | Train-vision aligned hard-FK diagnostic | post-hoc diagnostic, final winner로 쓰지 않음 |
| B1 | Independent per-camera soft-FK | A4와 비교하는 preflight independent baseline |
| B2 | Cube-only soft-FK | board 관측이 cube estimate를 도와주는지 보는 support row |
| B3 | Board-only unified-style baseline | A2와 비교해 cube/unified coupling 효과를 보는 support row |

현재 발표/논문에서 가장 중요한 비교는 다음 순서로 둔다.

1. A1 -> A2: unified feedback의 주 결과
2. B3 -> A2: board-only 대비 cube+unified coupling의 보조 결과
3. A2 -> A3: hard FK를 피해야 하는 이유
4. A2 -> A4, B1 -> A4: soft-FK preflight
5. A5: post-hoc diagnostic

## 5. 평가지표 기준

| 우선순위 | 지표 | 현재 사용 여부 | 해석 |
| --- | --- | --- | --- |
| Primary internal | Own-marker held-out reprojection error | 사용 | 각 방법이 자기 marker population에서 얼마나 잘 일반화되는지 보는 현재 핵심 지표 |
| Support-bias check | Set-equal held-out reprojection error | 사용 | board/cube support 수 차이 때문에 생기는 ranking bias 확인 |
| Scope check | Camera-scope held-out residual | 사용 | fixed cameras 0, 1, 3에서 실제 적용 scope가 깨지지 않는지 확인 |
| Data quality | cube detection acceptance, rejected observations, board-cube conflict | 사용 | 결과를 해석하기 전에 데이터 문제가 아닌지 확인 |
| Physical GT | translation error, rotation error, P95, failure rate | 다음주 예정 | 로봇 베이스 물리 정확도와 task claim 확정용 |

현재 지표로 할 수 있는 최대 결론은 "internal reprojection consistency"다. "absolute robot-base accuracy"는 다음주 external GT 이후에만 확정한다.

## 6. 발표에서 써도 되는 문장

1. "현재 실제 데이터에서 가장 타당한 결론은 A2, 즉 board+cube를 unified visual feedback으로 묶은 구성이 내부 held-out residual을 가장 안정적으로 낮춘다는 것입니다."
2. "Cube는 단순히 넣기만 하면 좋아지는 요소가 아니라, unified solver 안에서 board와 서로 제약을 주는 3D 공통 target으로 해석하는 것이 맞습니다."
3. "Hard FK는 residual을 악화시켰기 때문에 FK를 GT로 고정하는 접근은 위험합니다."
4. "Soft-FK는 설계상 필요한 방향이지만 현재 A4는 simulation covariance 기반 preflight라서, 최종 물리 정확도 claim은 다음주 external GT로 검증합니다."
5. "따라서 현 단계의 결론은 unified internal consistency는 확인했고, physical robot-base accuracy는 다음 검증 단계에서 확정한다는 것입니다."

## 7. 발표에서 피해야 할 문장

| 피해야 할 문장 | 이유 | 대체 문장 |
| --- | --- | --- |
| "Cube를 추가하면 항상 성능이 좋아진다." | A0 -> A1에서 board residual이 개선되지 않음 | "Cube 효과는 unified solver context에서 나타난다." |
| "A4가 최종 최고 방법이다." | A4는 simulation covariance preflight이고 A2와 거의 동률 | "A4는 soft-FK preflight candidate다." |
| "FK를 쓰면 정확도가 올라간다." | A3 hard-FK는 악화됨 | "FK는 uncertainty-aware prior로 제한적으로 써야 한다." |
| "로봇 베이스 물리 정확도가 검증됐다." | external GT가 아직 없음 | "물리 정확도 검증은 다음주 예정이다." |
| "A5가 최종 방법이다." | A5는 post-hoc diagnostic | "A5는 train-vision aligned diagnostic으로 해석한다." |

## 8. 다음주 External GT 이후 업데이트 규칙

다음주 external GT가 들어오면 이 문서를 먼저 업데이트한다.

1. Translation error, rotation error, P95, failure rate를 C4에 추가한다.
2. A2와 A4의 물리 정확도를 비교해 soft-FK claim을 확정하거나 보류한다.
3. Cube가 물리 정확도에도 도움이 되는지 C2 표현을 다시 조정한다.
4. Internal residual과 physical GT가 충돌하면 physical GT를 최종 ranking 기준으로 둔다.
5. 현재 internal-only conclusion은 삭제하지 않고 "pre-GT internal validation"으로 보존한다.

## 9. 근거 문서

- [CALIBRATION_EXPERIMENT_VALIDATION.md](CALIBRATION_EXPERIMENT_VALIDATION.md): 전체 실험 검증과 claim 경계
- [CP_result/session04/late_table1/TABLE1_RESULTS.md](CP_result/session04/late_table1/TABLE1_RESULTS.md): canonical Table 1 수치와 해석
- [8-3_meeting.md](8-3_meeting.md): 8/3 피드백 반영 현황
- [CALIBRATION_EXPLANATION_LATEX.md](CALIBRATION_EXPLANATION_LATEX.md): 발표/논문용 설명 문장
- [RUN_PIPELINE.md](RUN_PIPELINE.md): 재현 파이프라인
