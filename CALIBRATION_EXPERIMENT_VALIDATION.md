# Calibration 비교실험·평가지표 검증

검증 기준: Session04 `A0~A5/B1~B3` 결과 및 현재 calibration 코드  
상태: Pre-GT internal evaluation  
검증일: 2026-09-03

## 최종 판정

현재 구성은 **코드 내부 ablation과 calibration 안정성 검증에는 타당**하다.
하지만 **방법의 최종 물리 정확도나 우월성을 주장하는 비교실험으로는 아직
불충분**하다.

## 1. 현재 잘 설계된 부분

- 모든 행이 같은 frozen manifest, K/D, split, solver와 train-only 초기값을 사용한다.
- 05의 held-out 계산에는 test-time refit, 결과 기반 관측 제거, frame-prune 누수가 없다.
- 다른 marker 모집단의 pooled RMSE를 직접 비교하지 않도록 계약이 들어가 있다.
- 27/27 runs가 수렴했고 42개 solver block 모두 full-rank다. Scaled Jacobian
  condition number는 약 `18.9~697.8`이다.

비교 조건 정의는 [README의 비교 구성](README.md#8-a0a5b1b3-비교실험), 코드
계약은 [schema.py](calibration_pipeline/schema.py)에 있다.

## 2. 권장 비교 구성

모든 행을 하나의 전체 순위로 만들지 않고, 한 번에 한 요소만 달라지는 matched
contrast로 해석한다.

| 구분 | 직접 비교 | 검증 질문 | 사용할 주 지표 | 판정 |
| --- | --- | --- | --- | --- |
| 내부 확증 | A0 ↔ B3 | Board-only에서 순차/통합 차이 | held-out board px | 유효, schema 계약 반영 완료 |
| 내부 확증 | A0 → A1 | 순차법에 cube를 추가한 효과 | held-out board px, 등록 수 | 유효 |
| 내부 확증 | A1 → A2 | Vision-only 순차/통합 차이 | board/cube px 각각 | 가장 타당 |
| 내부 확증 | B3 → A2 | 통합법에서 cube residual 효과 | held-out board px | 유효 |
| 내부 확증 | A2 → A3 | Vision cube pose와 raw-FK hard fixed 차이 | board/cube px 각각 | 유효, FK를 GT로 해석 금지 |
| Preflight | B1 → A4 | 같은 soft FK factor에서 순차/통합 차이 | board/cube px 각각 | 구조는 유효, covariance가 simulation |
| Preflight | A2 → A4 | soft FK factor 추가 효과 | board/cube px 각각 | 중요 비교, schema 계약 반영 완료 |
| Preflight | B2 → A4 | Board residual의 기여 | held-out cube px | 유효, covariance 한계 |
| Post-hoc | A3 → A5, A4 → A5 | raw/aligned, soft/hard 원인 분리 | 모든 내부 지표 | 진단 전용 |

따라서 메인 결론은 현재처럼 **A2**, 방법 확장 후보는 **A4**, 원인 진단은
**A5**로 둔다. A3/A4의 `Ours` 명칭은 확증 전에는 `raw-FK hard`,
`corrected-FK soft`처럼 중립적으로 표기하는 것이 안전하다.

## 3. 현재 수치로 말할 수 있는 결론

| 비교 | Board / Cube held-out | 해석 |
| --- | --- | --- |
| A0 → A1 | `4.0530 → 4.0645` / cube 신규 | 순차법에서는 cube 추가가 board를 개선하지 않음 |
| A1 → A2 | `4.0645 → 3.9840` / `4.1402 → 3.5958` | 통합 feedback이 두 target에서 모두 개선 |
| B1 → A4 | `4.0648 → 3.9884` / `4.1182 → 3.5805` | soft FK 조건에서도 통합법 개선 경향, 단 preflight |
| A2 → A4 | `3.9840 → 3.9884` / `3.5958 → 3.5805` | Board는 미세 악화, Cube는 미세 개선; 사실상 동률 |
| A2 → A3 | Cube `3.5958 → 6.3959` | raw FK hard fixed가 현재 데이터에서는 크게 악화 |
| B2 → A4 | Cube `4.4827 → 3.5805` | soft FK 조건에서 board residual이 cube 보정에 도움 |
| A4 → A5 | Own overall `3.8899 → 3.7270` | A5가 낮지만 post-hoc이고 다른 cross-view 지표와 불일치 |

전체 수치는 [Session04 Table 1 결과](CP_result/session04/late_table1/TABLE1_RESULTS.md)에서
확인한다.

## 4. 평가지표 판정

| 지표 | 판정 | 제한 |
| --- | --- | --- |
| Train reprojection RMSE | 적합 | 수렴 진단만 가능 |
| Own-marker held-out RMSE | 조건부 적합 | 같은 set의 다른 event이므로 새 위치 일반화가 아님 |
| Pooled overall RMSE | 보조로 변경 권장 | held-out corner가 Board 703, Cube 236이라 Board가 약 75% 지배 |
| Fixed-to-Fixed | 보조 지표로 적합 | 상대 일관성만 측정하며 공통 systematic error를 검출하지 못함 |
| Gripper-to-Fixed / `e_e2e` | 내부 체인 진단만 가능 | FK가 포함되고 일부 fixed anchor가 train 관측임 |
| Seed mean ± std | 안정성 지표로만 적합 | seed 3개는 독립 실험 표본이 아님 |
| External TRE/rotation/P95/failure | 최종 주 지표로 적합 | 다음주 Independent External GT 태스크에서 산출 |

특히 Gripper-to-Fixed는 held-out gripper event에 일부 train fixed-anchor를 연결한다.
따라서 `held-out 성능`보다는 **mixed train-anchor/held-out internal closure**라고
표시하는 편이 정확하다.

## 5. 개선 우선순위

1. 완료: `A0_to_B3`, `A2_to_A4` 비교 계약을 추가하고 결과표를
   `확증 / preflight / post-hoc` 세 구역으로 분리했다.
2. 완료: corner-pooled RMSE 외에 `event → set 동일가중 RMSE`와
   paired set bootstrap CI를 추가했다. 현재 `n=9 sets`이므로 CI는 exploratory로
   표시한다.
3. 완료: per-camera/target support, dropped sets `0~3`, detection
   failure, Board–Cube 충돌 `10.808 mm`를 결과 첫 화면에 경고로 표시했다.
4. 다음주 예정: Independent External GT로 Translation Error, Rotation Error,
   P95, Failure Rate를 산출한다. 그 전에는 내부 지표 기반 결론만 유지한다.
5. 후속 촬영/측정 필요: measured FK covariance, cam0/cam1 intrinsic view 보강,
   명시적인 robot pose convention, 독립 session과 unseen-position GT를 확보한다.
6. 논문 비교 필요: A0 같은 내부 baseline 외에 Tsai/Park/Daniilidis 또는 공개
   robot-world/hand-eye 방법을 동일 입력·동일 평가로 추가한다.

가장 큰 데이터 위험은 [Board–Cube 간 10.808 mm systematic disagreement](data/session04/calib_out/verify/board_cube_relative_pose/BOARD_CUBE_RELATIVE_POSE.md)다.
현재 joint solve가 이를 완화할 뿐 원인을 제거한 것은 아니다.

## 관련 문서

- 실행 순서: [RUN_PIPELINE.md](RUN_PIPELINE.md)
- 비교실험 및 지표 정의: [README.md](README.md)
- Calibration 수식: [CALIBRATION_EXPLANATION_LATEX.md](CALIBRATION_EXPLANATION_LATEX.md)
- 상세 결과: [TABLE1_RESULTS.md](CP_result/session04/late_table1/TABLE1_RESULTS.md)
- 전체 calibration 행렬: [calibration_matrices.json](CP_result/session04/late_table1/calibration_matrices.json)

## 현재 구현 상태와 다음 태스크

비교 계약, 보고서 구조, event/set 균등 집계, paired bootstrap, 데이터 경고 표시는
현재 구현 완료 상태다. 다음 태스크는 다음주 Independent External GT 수집/평가이며,
이후에만 robot-base Translation Error, Rotation Error, P95, Failure Rate를 최종
물리 정확도 지표로 보고한다.
