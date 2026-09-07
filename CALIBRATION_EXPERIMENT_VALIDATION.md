# Calibration 비교실험·평가지표 검증

검증 기준: Session04 `A0~A5/B1~B3` 결과 및 현재 calibration 코드  
상태: Pre-GT internal evaluation  
검증일: 2026-09-04

논문 기여도와 스토리라인의 최상위 기준은 [RESEARCH_STORYLINE.md](RESEARCH_STORYLINE.md)를 따른다.

## 0. 비교실험 구성별 평가지표 상단 요약

아래 표를 먼저 보고 발표한다. 모든 row를 하나의 전체 순위로 세우지 않고,
비교실험 구성별로 **허용되는 평가지표**와 **해석 제한**을 분리한다.V

| 비교실험 구성 | 확인하려는 효과 | Primary metric | 같이 보는 보조/진단 지표 | 공정성 제한 |
| --- | --- | --- | --- | --- |
| A0 ↔ B3 | board-only에서 sequential freeze와 unified-style 구조 차이 | held-out board RMSE px | set-equal board RMSE, seed stability, Fixed-to-Fixed board | 같은 board-only population 안에서만 해석 |
| A0 → A1 | sequential 구조에서 cube 추가가 board에 주는 영향 | held-out board RMSE px, registered fixed cameras | pooled overall은 참고만, support/dropped-set 확인 | cube가 추가되지만 board 개선 여부만 직접 비교 |
| A1 → A2 | 같은 board+cube population에서 unified visual feedback 효과 | held-out board RMSE px, held-out cube RMSE px | set-equal RMSE, paired set bootstrap CI, seed stability | 현재 가장 깨끗한 internal main contrast |
| B3 → A2 | unified 구조에서 cube residual이 board calibration에 주는 영향 | held-out board RMSE px | Fixed-to-Fixed board/cube, point-cloud diagnostic | marker-system 전체 우위가 아니라 board component 개선 |
| A2 → A3 | vision-estimated cube pose와 raw-FK hard fixed 차이 | held-out board/cube RMSE px | Gripper-to-Fixed closure, point-cloud diagnostic | raw FK는 external GT가 아니며 negative control로 해석 |
| B1 → A4 | soft-FK 조건에서 sequential vs unified 차이 | held-out board/cube RMSE px | FK cost fraction, set-equal RMSE, bootstrap CI | covariance가 simulation prior라 preflight |
| A2 → A4 | unified visual-only에 soft FK factor를 추가한 효과 | held-out board/cube RMSE px | Objective block diagnostics, FK sensitivity, point-cloud diagnostic | A2와 거의 동률이면 우월성 주장 금지 |
| B2 → A4 | soft-FK 조건에서 board residual이 cube 보정에 주는 영향 | held-out cube RMSE px | Gripper-to-Fixed cube closure, point-cloud cube RMSE | cube-only와 board+cube의 target support 차이 주의 |
| A3/A4 → A5 | raw/aligned FK, soft/hard 처리의 원인 분리 | 내부 held-out RMSE, point-cloud diagnostic | Fixed-to-Fixed, Gripper-to-Fixed, FK alignment artifact audit | A5는 post-hoc diagnostic이며 main method 아님 |
| External GT 예정 | 최종 robot-base 물리 정확도와 task 성능 | TRE, rotation error, P95, failure rate | robot task success, contact/XYZ error | 다음주 Independent External GT 후 최종 판정 |

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

## 5. 평가지표 결과 산출 방식

모든 내부 지표는 같은 frozen observation manifest, camera intrinsics `K/D`,
event-grouped split, row별 최종 transform을 사용한다. Held-out 평가에서는
test-time refit, held-out frame-prune, 결과 기반 관측 제거를 하지 않는다.

### 5.1 Train reprojection RMSE

각 row/seed를 train observations로 최적화한 뒤, 최종 transform을 고정하고
train image corner를 다시 투영한다.

```text
pixel residual = observed corner - projected corner
RMSE_px = sqrt( sum(du^2 + dv^2) / (2N) )
```

이 값은 solver가 학습 관측을 얼마나 잘 맞췄는지 보는 수렴/적합 진단이다.
학습 데이터에 직접 맞춘 값이므로 방법 우월성 지표로 쓰지 않는다.

### 5.2 Own-marker held-out RMSE

각 row의 marker 구성에 해당하는 held-out observations만 사용한다. 예를 들어
A0/B3는 board만, B2는 cube만, A1/A2/A3/A4/A5/B1은 board+cube를 평가한다.
최종 transform은 train에서 이미 고정된 상태이며, held-out에서 새로 풀지 않는다.

```text
for held-out image corners:
    project 3D target corner -> image
    compute du, dv
    aggregate RMSE per target and overall
```

이 지표가 현재 main internal metric이다. 다만 held-out이 같은 set의 다른 event라서
완전히 새로운 3D 위치 일반화나 robot-base 물리 정확도를 뜻하지 않는다.

### 5.3 Pooled overall RMSE

Board와 Cube의 held-out corner residual을 한꺼번에 모아 하나의 RMSE로 계산한다.

```text
pooled overall = RMSE(all held-out board/cube corners)
```

현재 support는 Board 703 corners, Cube 236 corners라 Board가 약 75%를 차지한다.
따라서 pooled overall은 요약값으로만 쓰고, 최종 판정은 board/cube target별 수치와
matched contrast를 함께 본다.

### 5.4 Set-equal-weight RMSE

corner 수가 많은 set이나 target이 결과를 지배하지 않도록 같은 residual을
`corner -> event -> set -> set equal-weight` 순서로 다시 집계한다.

```text
event RMSE = one event의 corner residual RMSE
set RMSE = 같은 set에 속한 event RMSE 집계
set-equal RMSE = 각 set RMSE를 동일 가중 평균
```

이 값은 pooled RMSE 옆에 붙는 support-bias check다. 현재 eligible set이 9개라
통계적 유의성 주장보다 방향성 민감도 점검으로 해석한다.

### 5.5 Paired set bootstrap CI

각 matched contrast에서 같은 held-out set을 paired unit으로 묶고, set 단위로
replacement resampling을 10,000회 수행한다.

```text
delta_set = RMSE_second_method(set) - RMSE_first_method(set)
bootstrap sample = 9 sets를 replacement로 재표본추출
95% CI = bootstrap delta 분포의 2.5%, 97.5% 분위수
```

음수는 두 번째 row가 더 낮은 residual을 냈다는 뜻이다. 하지만 `n=9 sets`이므로
가설검정이 아니라 contrast 방향성이 얼마나 흔들리는지 보는 탐색 지표다.

### 5.6 Fixed-to-Fixed

고정카메라끼리 같은 held-out board/cube target을 본 경우를 사용한다. 한 카메라의
PnP target pose를 robot base로 올리고, 다른 고정카메라로 다시 투영하거나 두
robot-base target pose의 차이를 계산한다.

```text
T_base_target_from_cam_i = T_base_cam_i * T_cam_i_target(PnP)
pixel transfer = cam_i에서 얻은 target pose를 cam_j image로 재투영
translation/rotation consistency = 두 T_base_target의 SE(3) 차이
```

Robot FK를 쓰지 않으므로 FK-free subsystem check로는 좋다. 하지만 모든 고정카메라가
공유하는 systematic error는 검출하지 못하므로 보조 일관성 지표로만 둔다.

### 5.7 Gripper-to-Fixed / `e_e2e`

Gripper camera observation과 fixed camera anchor를 robot-base chain으로 연결한다.
이때 gripper camera pose는 robot FK와 hand-eye transform을 통해 계산된다.

```text
T_base_gripper_cam(event) = T_base_gripper_FK(event) * T_gripper_cam
T_base_target_from_gripper = T_base_gripper_cam * T_gripper_cam_target(PnP)
T_base_target_from_fixed = T_base_fixed_cam * T_fixed_cam_target(PnP)
compare two robot-base target poses
```

최종값은 pair 성분을 event RMSE로, event를 set RMSE로 집계한 뒤 set별 동일
가중치로 계산한다. 일부 fixed anchor가 train 관측이고 FK/Hand-Eye가 섞이므로
`held-out 성능`이 아니라 **mixed train-anchor/held-out internal closure**다.

### 5.8 Seed mean ± std

각 row는 서로 다른 초기 perturbation seed 3개로 실행된다. 표에는 seed별 결과의
평균과 표준편차를 함께 보고한다.

```text
mean = average(metric_seed0, metric_seed1, metric_seed2)
std = sample/summary dispersion across three initializations
```

이는 optimizer 안정성 확인용이다. 같은 데이터와 같은 split을 반복한 것이므로
독립 실험 표본이나 통계적 성능 차이로 해석하지 않는다.

### 5.9 Robot-base point-cloud diagnostic

8/3 피드백 #17 대응 지표다. Aligned depth를 calibration 목적함수에는 넣지 않고,
각 row의 transform으로 robot-base frame에 올린 뒤 target plane과의 거리만 본다.

```text
depth pixel inside detected target polygon -> 3D point in camera frame
T_base_point = T_base_camera * point_camera
depth-to-plane residual = distance(T_base_point, row target plane)
```

현재 A0-A5/B1-B3 전체 row에 대해 event 24/54/72의 board/cube point cloud를
생성했다. 이 지표는 3D 공간 정합의 진단 자료지만, depth 자체가 external GT가
아니므로 최종 robot task accuracy로 쓰지 않는다.

### 5.10 External TRE / rotation / P95 / failure

다음주 Independent External GT가 들어오면 최종 주 지표로 산출한다.

```text
translation error = || predicted position - GT position ||
rotation error = angle( R_pred^-1 * R_GT )
P95 = translation/contact error의 95th percentile
failure rate = task 또는 tolerance 기준 실패 비율
```

이 지표만이 최종 robot-base physical accuracy와 task-level claim을 확정할 수 있다.
내부 지표와 External GT가 충돌하면 External GT를 최종 판정 기준으로 둔다.

## 6. 개선 우선순위

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

## 7. 관련 문서

- 논문 스토리라인 기준: [RESEARCH_STORYLINE.md](RESEARCH_STORYLINE.md)
- 실행 순서: [RUN_PIPELINE.md](RUN_PIPELINE.md)
- 비교실험 및 지표 정의: [README.md](README.md)
- Calibration 수식: [CALIBRATION_EXPLANATION_LATEX.md](CALIBRATION_EXPLANATION_LATEX.md)
- 상세 결과: [TABLE1_RESULTS.md](CP_result/session04/late_table1/TABLE1_RESULTS.md)
- 전체 calibration 행렬: [calibration_matrices.json](CP_result/session04/late_table1/calibration_matrices.json)

## 8. 현재 구현 상태와 다음 태스크

비교 계약, 보고서 구조, event/set 균등 집계, paired bootstrap, 데이터 경고 표시는
현재 구현 완료 상태다. 다음 태스크는 다음주 Independent External GT 수집/평가이며,
이후에만 robot-base Translation Error, Rotation Error, P95, Failure Rate를 최종
물리 정확도 지표로 보고한다.
