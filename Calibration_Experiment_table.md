# Calibration Experiment — 최종 비교실험 명세

이 문서는 실제 셋업에서 제안 방법의 기여를 분리하고, “Ours가 3D 공간 정합에서 우수하다”는
주장을 어떤 조건에서 허용할지 정한 최종 실험 계약이다. 과거 디버깅 수치와 내부 감사 과정은
반복하지 않고 원본 산출물에 남긴다.

현재 결론은 다음과 같다.

- Unified visual calibration의 held-out pixel consistency 개선은 기존 데이터에서 확인됐다.
- Soft-FK는 hard-FK보다 합리적인 제안 방향이지만, 공분산 기반 robust factor는 아직 구현·평가되지 않았다.
- 현재 최저 셀 2.034 mm는 외부 GT가 아닌 FK-proxy translation 결과다. 절대 3D 정확도 근거가 아니다.
- 따라서 외부 GT 실험 전에는 “Ours가 가장 정확하다”고 결론 내리지 않는다.

## 방법 정의

모든 방법은 운동학 백본 `T_base_gripper = FK(q)`를 공통으로 사용한다. 실험 축의 `FK`는 이
백본의 사용 여부가 아니라, **cube pose에 FK 정보를 어떤 방식으로 주입하는가**를 뜻한다.

- `seq`: eye-in-hand와 eye-to-hand를 순차적으로 풀고 앞 단계 결과를 동결한다.
- `U-BA`: 고정·손목 카메라, hand-eye, target pose를 하나의 pixel-level bundle adjustment에서 푼다.
- `hard-FK`: cube pose를 FK pose에 고정한다.
- `soft-FK`: cube pose를 자유변수로 두고 covariance-weighted robust FK factor를 추가한다.
- `correction`: calibration 이후 held-out target pose 예측에만 적용한다. Calibration transform은 바꾸지 않는다.

제안 soft-FK residual은 다음을 기본형으로 한다.

```text
r_FK = Log(T_FK^-1 T_cube)
E_FK = rho(r_FK^T Sigma_FK^-1 r_FK)
```

`Sigma_FK`와 FK-factor robust-loss scale은 별도 반복측정 또는 training-only inner validation에서
정하고 test에서는 동결한다. 기존 상수 가중치 `lambda=3 px/mm` 결과는 **A4a fixed-weight soft-anchor
선행 실험**이며 최종 covariance-weighted robust A4 결과가 아니다.

A2와 A4는 다음 목적함수 차이 하나만 허용한다.

```text
A2: E_visual
A4: E_visual + E_FK
```

`E_visual`의 raw-corner residual, visibility mask, robust loss, outlier 기준, 초기화, multi-start 수,
iteration budget과 종료조건은 완전히 동일하게 동결한다. A2를 기존 pose-level 결과로 두고 A4만
pixel-level로 실행한 값은 `A2→A4`의 인과 비교에 사용하지 않는다.

## Table 1 — Main Ablation (실제 셋업)

Table 1에는 제안 기여를 직접 검증하는 A/B 계열만 둔다. 모든 행은 동일 source frames, frozen
intrinsics, train/test mask와 공통 평가 mask를 사용한다. 동일 optimization level의 행은 frozen corner/PnP
frontend와 solver budget도 공유하며, target subset이나 입력 표현이 다른 경우 해당 차이를 표에 공개한다.

| ID | 방법 / 검증 기여 | Target | Solve | FK→cube | Correction | 반드시 비교할 paired contrast | 현재 상태 |
| --- | --- | --- | --- | --- | --- | --- | --- |
| A0 | Board-only sequential — 평면 타깃 기준선 | board | seq | 해당 없음 | 없음 | A0→A1: cube 추가에 따른 관측성·등록률 변화 | 기존 결과 있음 |
| A1 | Cube+board sequential — cube 관측성 | cube+board | seq | vision-estimated | 없음 | A0→A1; A1→A2 | 기존 결과 있음; A1→A2용 공통 pixel frontend 재실행 필요 |
| A2 | Unified visual-only — 통합 최적화 | cube+board | U-BA | 사용 안 함; cube 자유변수 | 없음 | **A1→A2:** solve만 seq→U | 기존 pose-level 결과 있음; 최종 pixel-level 공통 backend 재실행 필요 |
| A3 | Unified hard-FK — FK 강제·안정성 | cube+board | U-BA | hard constraint | 없음 | **A2→A3:** FK factor none→hard | 기존 결과 있음; 최종 공통 pixel backend 재실행 필요 |
| **A4★** | **Unified soft-FK (Ours-core)** — 제안 캘리브레이션 원리 | **cube+board** | **joint pixel-level U-BA + robust loss** | **covariance-weighted robust factor** | 없음 | **A2→A4:** 최종 FK factor 전체 효과; **B1→A4:** independent→joint 효과 | **최종 구현·외부 GT 평가 필요** |
| A5 | Unified soft-FK + 6-DoF correction — Ours-full 후보 | cube+board | A4와 동일 | A4와 동일 | train-only SE(3) residual correction | **A4→A5:** correction만 추가 | 미구현. 통과 전에는 Ours-full로 부르지 않음 |
| **B1** | **Robust independent — primary fair baseline** | cube+board | 카메라별 독립 robust solve + rigid composition | A4와 동일한 `Sigma_FK` 정보량 | 없음 | **B1→A4:** joint optimization의 순수 효과 | 공정 arm 미실행. 기존 sequential/FK-fixed B1으로 대체 금지 |
| B2 | Cube-only unified — board 기여 | cube | A4와 동일 | A4와 동일 soft-FK | 없음 | **B2→A4:** board만 추가 | soft-FK 공정 arm 미실행 |
| B3 | Board-only unified — cube 기여 | board | A2와 동일 | 해당 없음 | 없음 | **B3→A2:** cube만 추가 | 기존 행 있음; paired delta 재산출 필요 |

### 행별 인과 해석

- `A1→A2`만 Unified의 순수 효과로 사용한다.
- `A2→A4`는 visual-only 대비 최종 covariance-weighted robust FK factor package의 전체 효과다.
- `A3→A4`는 hard-FK 대비 최종 covariance+robust soft-FK package 비교이며 순수 soft 전환 효과가 아니다.
  순수 soft constraint 효과는 supplementary의 `A3→A4a`로 판단한다.
- `A4→A5`만 correction의 추가 효과다. `A2→A5`를 correction 단순효과로 해석하지 않는다.
- Board의 효과는 `B2→A4`, cube의 효과는 `B3→A2`로 판단한다.
- `B3→A3`는 marker와 FK 조건을 동시에 바꾸므로 cube 기여의 근거로 사용하지 않는다.
- A0/A1은 관측성·전통 baseline이다. 제안 방법의 joint 효과는 정보량을 맞춘 `B1→A4`로 판단한다.

### Supplementary — A4 FK-factor decomposition

Main Table에는 최종 A4만 유지하고, A4 내부 기여는 아래 supplementary ablation으로 분리한다.
세 행 모두 `E_visual`과 solver budget은 A2/A4 공통 계약 그대로 유지하며, 아래 `Loss`는 **FK factor에
적용하는 loss만** 뜻한다.

| ID | FK residual | FK weight | FK loss | 검증 목적 | 필수 paired contrast |
| --- | --- | --- | --- | --- | --- |
| A4a | soft | fixed isotropic `lambda` | quadratic | hard constraint를 soft constraint로 바꾸는 효과 | `A3→A4a` |
| A4b | soft | `Sigma_FK^-1` | quadratic | covariance whitening의 추가 효과 | `A4a→A4b` |
| **A4c = A4** | soft | `Sigma_FK^-1` | 사전 고정 robust loss | FK outlier robustification의 추가 효과 | `A4b→A4c` |

추가 confirmatory contrast는 `A2→A4`(최종 FK factor 전체 효과), `B1→A4`(independent→joint),
`A4→A5`(calibration을 고정한 correction 추가 효과)로 유지한다. A4a의 `lambda`도 test 결과가 아니라
training-only inner validation 또는 사전 물리 기준으로 정한다.

## Classical hand-eye baselines

Classical 방법은 mixed eye-in+eye-to joint calibration이 아니므로 Main Ablation에 포함하지 않는다.
동일 pose frontend와 동일 raw observations로 hand-eye를 계산한 후, 같은 규칙으로 fixed-camera 경로를
합성해 최종 task에서 평가한다.

| ID | 방법 | 분류 | 계획 구현 | 공정 비교 조건 | 상태 |
| --- | --- | --- | --- | --- | --- |
| C1 | Tsai–Lenz | classical `AX=XB`, separable | OpenCV `calibrateHandEye` | 동일 robust pose frontend·motion pairs·평가 mask | 미실행 |
| C2 | Park–Martin | classical `AX=XB`, Lie-group | OpenCV `calibrateHandEye` | C1과 동일 | 미실행 |
| C3 | Horaud | classical `AX=XB` | OpenCV `calibrateHandEye` | C1과 동일 | 미실행 |
| C4 | Daniilidis | classical `AX=XB`, dual quaternion | OpenCV `calibrateHandEye` | C1과 동일 | 미실행 |

계획 구현의 네 method enum은 [OpenCV `calibrateHandEye` 공식 문서](https://docs.opencv.org/4.13.0/d9/d0c/group__calib3d.html)에
포함되어 있다. 라이브러리 지원 여부와 별개로 좌표계 방향·입력 motion pair·퇴화 조건은 자체 synthetic
test로 검증한 뒤 실제 결과를 생성한다.

Classical 방법은 적용 범위가 A4보다 좁다는 점을 명시한다. 그러나 동일한 외부-GT 최종 task 결과는
직접 비교할 수 있다. 네 방법은 동일 입력으로 실행 비용이 낮으므로 최종 실험에는 모두 포함하는 것을
기본안으로 한다. C1–C4는 표준 기준선이지 최신 SOTA의 대체물이 아니다. Recent multi-camera,
robot-world, iterative 또는 targetless 외부 방법은 공개 코드·입력 적합성·실행 가능성을 확인한 별도
comparison contract에서 다루며, 현재 관련 감사 내용은
[CP_EXPERIMENTS_README — SOTA protocol audit](CP_EXPERIMENTS_README.md#sota-protocol-audit)를 따른다.

## 평가 지표

### Confirmatory accuracy endpoints

| 지표 | 정의 | 보고값 | 주의 |
| --- | --- | --- | --- |
| `TRE_t` (mm) | 독립 외부 GT와 예측 target 중심의 3D 거리 | mean, P50, P95, max, 95% CI | **최우선 지표**. FK를 GT로 사용 금지 |
| `e_R` (deg) | `Log(R_GT^T R_pred)`의 geodesic angle | mean, P50, P95, max, 95% CI | 위치-only Ridge는 이 지표를 개선할 수 없음 |
| ADD / ADD-S (mm) | GT·예측 pose로 변환한 object surface point의 평균 거리 | mean, P95, 95% CI | cube 대칭을 인정하면 ADD-S 사용 |

외부 GT는 A4/A5의 FK factor, correction label, intrinsic calibration과 독립이어야 한다. 독립성이
확보되지 않으면 지표 이름에 `FK-proxy` 또는 `internal`을 붙이고 1차 지표로 사용하지 않는다.

전체 6-DoF 우월성 주장은 `TRE_t` superiority, `e_R` superiority 또는 non-inferiority, 사전 선택한
ADD/ADD-S 계약을 모두 통과하는 intersection-union gate로 판단한다. 여러 baseline에 대한 동일 endpoint
비교는 Holm 보정하며, endpoint 간 gate 순서와 alpha 사용은 분석 전에 고정한다.

### Mandatory guardrails and operational endpoints

| 지표 | 정의 | 보고값 | 역할 |
| --- | --- | --- | --- |
| Tail error | session별 P95 `TRE_t`, P95 `e_R` | paired difference와 95% CI | 평균 개선이 tail 악화를 숨기지 않는지 확인 |
| Workspace error | center/edge/near/far/height 구간별 `TRE_t` | 구간별 P50/P95, worst-stratum P95 | train 영역 안·밖을 분리 |
| Reliability | calibration/inference failure rate, `N_reg` | paired failure-rate difference, 등록 카메라 수 | 실패 run을 오차 평균에서 조용히 제외 금지 |
| Efficiency | runtime, iterations, convergence rate, 필요한 views | session별 분포 | 실용성과 수렴 안정성 |

P95, workspace, failure/coverage는 secondary라는 이름으로 완화할 수 있는 선택 지표가 아니라 최종 claim의
필수 guardrail이다.

### Diagnostic only

| 지표 | 정의 | 용도 |
| --- | --- | --- |
| Held-out raw-corner reprojection | 공통 held-out 실제 2D corner에 frozen transform을 재투영한 RMSE/median/P95 | pixel fit과 일반화 진단 |
| Cross-camera disagreement | 같은 target의 카메라별 3D 예측 산포(mm/deg) | 내부 정합 진단. 절대 정확도 아님 |
| Transform repeatability | 독립 재설치 session 간 `bTf`, `gTc` 변화(mm/deg) | 재현성 |
| Depth alignment | point-to-plane 또는 point-to-CAD distance(mm) | scale·3D geometry 교차검증 |
| Geometry/FK checks | cube dimension consistency, commanded relative-motion consistency, FK-proxy translation error | geometry·scale·운동학 오류 진단 |
| Solver diagnostics | condition number, Jacobian rank, residual distribution | 퇴화·수렴 원인 분석 |

Diagnostic 지표만으로 절대 3D 정확도 또는 외부 방법 대비 우위를 결론 내리지 않는다. Marker reprojection이
정의되지 않는 targetless 방법은 공통 외부-GT endpoint로만 직접 순위를 비교한다.

`e_X`처럼 `T_base_Ci`와 `T_gripper_cam`을 하나로 평균한 숫자는 사용하지 않는다. Simulation 또는
외부 transform GT가 있는 경우 다음을 분리 보고한다.

- 고정 카메라 `bTf`: translation mm / rotation deg
- hand-eye `gTc`: translation mm / rotation deg
- 카메라 간 상대변환: translation mm / rotation deg

## External-GT uncertainty contract

외부 GT 장비의 제조사 정격 정확도만 인용하지 않고 실제 작업 거리와 workspace에서 반복 측정한다.
Session별로 다음 성분을 별도 기록한다.

- Translation/rotation 반복측정 분산과 시간 drift
- GT rigid body·jig의 탈착/장착 반복성
- Cube object frame과 GT marker/reflector frame 사이 `T_cube_GTmarker`의 추정 불확도
- GT frame을 robot base로 옮기는 `T_base_GT` registration 불확도
- RGB capture와 GT measurement 사이의 시간 정합 오차
- 위 성분을 합성한 translation/rotation measurement uncertainty floor

GT 등록에 calibration train observation이나 A4 FK anchor를 재사용하지 않는다. 방법 간 차이가 합성
uncertainty floor와 비슷하거나 더 작으면 통계적 유의성과 별개로 “실질적으로 더 정확하다”는 표현을
사용하지 않는다.

## 통계 계약과 최종 합격 조건

- 통계 단위는 frame이나 split이 아니라 **독립 camera-installation session**이다.
- 같은 session의 동일 held-out poses에서 모든 방법을 paired 평가한다.
- 방법별 mean과 함께 P50/P95/max를 보고하되, 추론의 최상위 단위는 session으로 유지한다.
- 95% CI는 paired hierarchical bootstrap으로 계산한다.

  1. 동일 method pair가 있는 session을 paired 상태로 복원추출한다.
  2. 선택된 각 session 안에서 동일 blind pose ID를 두 방법에 공통으로 복원추출한다.
  3. 각 session의 paired metric difference를 계산하고 session에 동일 가중치를 주어 집계한다.
  4. 사전 고정한 bootstrap 반복 수와 CI 방식으로 구간을 산출한다.

- Pose만 독립 표본처럼 재표집하거나 pose가 많은 session에 더 큰 가중치를 주어 CI를 인위적으로 좁히지 않는다.
- A4의 confirmatory 비교군은 결과를 보기 전에 `A1, A2, A3, B1, C1–C4`로 고정한다. A0/B2/B3는
  관측성·기여 분석, A5는 별도 extension family로 취급한다.
- Confirmatory TRE 비교의 p-value는 위 비교군 전체에 Holm 보정한다. Rotation과 ADD/ADD-S의 검정
  family 및 hierarchical gate 순서도 결과 열람 전에 고정한다.
- 실패 run도 failure rate에 포함하며 성공 run만 골라 정확도를 비교하지 않는다.
- 한 방법이 실패해 paired TRE가 정의되지 않는 session 수를 별도로 보고한다. Claim은 충분한 complete
  session pair와 failure-rate 비열등성을 모두 만족해야 하며, 실패 session을 단순 삭제하지 않는다.
- Hyperparameter, `Sigma_FK`, robust threshold, correction model은 training-only로 선택한다.
- Pilot 5 sessions는 분산·실패율 추정과 power analysis용이며 최종 우월성 검정 표본으로 확정하지 않는다.

Ours-core A4가 우수하다고 판정하려면 다음을 모두 만족해야 한다.

1. 사전 고정한 confirmatory baseline 각각에 대해 `delta TRE = TRE_A4 - TRE_baseline`의 hierarchical
   paired bootstrap 95% CI 상한이 0보다 작고 Holm-adjusted 검정을 통과한다.
2. 회전오차가 유의하게 낮거나 `UpperCI(e_R,A4 - e_R,baseline) < m_R`을 만족한다.
3. 사전 선택한 ADD/ADD-S를 confirmatory endpoint로 주장한다면 그 사전 정의 superiority 또는
   non-inferiority 계약을 통과한다.
4. `UpperCI(P95_TRE,A4 - P95_TRE,baseline) < m_P95`를 만족한다.
5. `UpperCI(FailureRate_A4 - FailureRate_baseline) < m_fail`을 만족하고 등록 카메라 수가 줄지 않는다.
6. `UpperCI(worst-stratum P95_A4 - worst-stratum P95_baseline) < m_stratum`을 만족해 특정 workspace
   구간의 열화를 전체 평균으로 숨기지 않는다.

여기서 `m_R`, `m_P95`, `m_fail`, `m_stratum`은 task tolerance와 GT uncertainty를 근거로 결과 확인 전에
고정한다. “악화되지 않음”을 point estimate 비교로 판정하지 않는다. Margin이 0보다 크면 허용 가능한
악화량을 뜻하므로 그 공학적 근거를 함께 기록한다.

A5는 calibration transform을 변경하지 않고 held-out target prediction에만 correction을 적용한다.
A4 대비 translation, rotation, tail, failure/coverage의 같은 계약을 통과할 때만 Ours-full로 승격한다.
그렇지 않으면 A4를 최종 방법으로 유지하고 correction은 supplementary 결과로 보고한다.

## 현재 실제 데이터가 보여주는 범위

아래 값은 기존 canonical event split과 cross-target artifact의 요약이다. 외부 GT가 아니며 서로
다른 split·robot scale 값이 섞인 수치를 한 행의 절대 성능처럼 읽지 않는다.

| 행 | `N_reg` | 기존 `e_e2e` (mm/deg) | 기존 `e_cross` (mm) | 공통 cube reproj (px) | 해석 |
| --- | ---: | ---: | ---: | ---: | --- |
| A0 | 2 | — | — | 10.9130±1.0401 | Board-only는 cam3 미등록. 관측성 한계 |
| A1 | 3 | 16.3323 / 7.7466 | 39.5978 | 10.5768±1.0394 | Cube 추가 후 3대 등록 |
| A2 | 3 | 16.1955 / 7.7423 | 38.9338 | 10.5170±1.0399 | A1 대비 held-out overall reproj 5/5 split 개선 |
| A3 | 3 | 16.1881 / 7.7087 | 38.0480 | 10.5042±1.0581 | 조건수·수렴 안정성은 좋지만 외부 GT 정확도 미검증 |
| A4a fixed-weight (`lambda=3`) | 3 | 15.1168 / 7.7342 | 35.0872 | 10.4968±1.0487 | 공분산 기반 A4가 아닌 fixed-weight soft-anchor 선행 결과 |
| A5 위치-only interim | 3 | calibration 지표는 A4와 동일 | calibration 지표는 A4와 동일 | calibration 지표는 A4와 동일 | FK-proxy translation 2.034 mm (A3+Ridge). 6-DoF·절대 GT 근거 아님 |
| B3 | 2 | — | — | 10.9129±1.0401 | Cube 단순효과 paired delta 미산출 |

확인된 paired 사실만 사용한다.

- `A1→A2`: held-out overall reprojection `-0.1486±0.0216 px`, 5/5 split 개선.
- `A2→A3`: 기존 own-target overall reprojection `+0.1155±0.0253 px`, 0/5 split 개선.
- A4a fixed-weight soft anchor는 cross-camera disagreement를 줄였지만 외부 3D GT 우위를 뜻하지 않는다.
- 위치-only correction의 2.034 mm는 FK-proxy이므로 논문의 절대 정확도 헤드라인으로 사용하지 않는다.

### 결과에서 허용되는 주장

| 주장 | 필요한 근거 |
| --- | --- |
| Cube가 관측성을 높인다 | `A0→A1`의 paired coverage와 registration 결과 |
| Unified solve가 sequential보다 낫다 | 공통 pixel backend의 `A1→A2` |
| Joint solve가 independent보다 낫다 | 정보량을 맞춘 primary fair contrast `B1→A4` |
| Soft constraint가 hard-FK보다 낫다 | `A3→A4a`와 외부-GT task 결과 |
| Covariance weighting과 robust FK가 기여한다 | `A4a→A4b→A4c` supplementary 결과 |
| Ours-core가 외부 방법보다 정확하다 | 독립 외부 GT와 confirmatory CI·failure·coverage 계약 전체 통과 |
| Correction이 최종 기여다 | Calibration 고정 상태의 `A4→A5`가 동일 계약 통과 |

외부 GT 완료 전에는 “Ours가 가장 정확하다” 또는 “절대 3D 정확도 2.034 mm”라고 표현하지 않는다.

세부 provenance:

- [Canonical repeated ablation](CP_result/ablation_multisplit/multisplit_ablation.md)
- [Cross-target cube evaluation](CP_result/cross_target_cube/cross_target_cube.md)
- [Soft-anchor event split](CP_result/D2_anchored_event_split/D2_anchored_event_split.md)
- [위치 hold-out 2×2 (D1)](CP_result/D1_fk_correction_2x2/D1_fk_correction_2x2.md)
- [실험 설명과 제한](CP_EXPERIMENTS_README.md)

### 2026-08-06 robot scale 통일

D1·D2 아티팩트는 `RB_ROBOT_POS_SCALE=1.0229`로 만들어져 이 표의 나머지 행(k=1.0)과 기준이
달랐다. 두 실험을 pinned k=1.0으로 재실행해 표 전체를 하나의 scale 기준으로 통일했다.
`CP_common.ROBOT_POS_SCALE_PINNED`는 이제 유일한 k 정의점이며, 저장 아티팩트를 읽는 경로에도
scale 가드가 걸려 있다.

- **D2**: Table 1의 A2/A3를 **0.00σ**로 재현한다(k=1.0229 아티팩트는 1.51σ였다).
- **D1**: 최저 셀이 **A2@λ=3 + Ridge (1.586 mm) → A3 + Ridge (2.034 mm)**로 바뀌었다. 즉
  k=1.0에서는 채택 후보였던 soft-anchor arm이 최저 셀이 아니고, hard-FK(A3)가 최저다.
  보정 자체의 효과는 오히려 커졌다(Ridge: A2 −2.361 mm t=−3.86, A3 −2.227 mm t=−5.13, 각 12/13
  fold). FK 고정 여부는 13개 위치로 여전히 판정되지 않는다(A3−A2 @ ridge `t=−1.66`, 9/13).
- 보정 전 오차는 k=1.0에서 arm 전체가 1.2–2.6 mm 크다. FK-proxy 지표가 FK 자체를 기준으로
  삼으므로 이는 순환 논증이며, k의 물리적 확정 근거로 사용하지 않는다.

## 실행 계약

### 촬영 단계 강제사항

- `Step2_capture.py`에서 `--root_folder`를 생략하면 `data/sessionNN/calib_train`을 원자적으로 새로 만들고,
  `NN`은 기존 번호의 최댓값에 1을 더한다. 기존 번호는 빈 폴더라도 자동 재사용하거나 덮어쓰지 않는다.
- 과거 session을 의도적으로 이어 촬영할 때만 `--root_folder data/sessionNN/calib_train`을 명시한다.
- `Step2_capture.py`는 `A_placement`와 `B_eyetohand`의 관측 요구를 분리 적용한다.
- A/B 태그와 `cube_gripped` 상태가 모순되거나 알 수 없는 block이면 저장하지 않는다.
- 자동 촬영은 gate 실패 frame을 성공으로 집계하지 않으며 `force_save`는 기본 차단한다.
- Cross-camera frame 선택과 span은 공통 host-monotonic receipt clock을 사용하고, 장치별 timestamp/domain은
  진단용으로 별도 보존한다.
- `server/robot_calb.py`는 `safe_joints_empty`, `safe_joints_gripped`가 모두 없는 waypoint를 첫 모션 전에
  거부하고, 각 capture를 `safe→target→safe`로 실행한다.
- 위 코드 강제는 물리 경로의 충돌 안전성을 증명하지 않는다. 두 payload 상태의 전체 경로를 저속
  dry-run으로 검증한 뒤에만 최종 촬영한다.

### 데이터·분할

- 모든 방법은 동일 source session, camera intrinsics, robot poses, train/test session split과 blind external-GT
  poses를 사용한다.
- 방법이 요구하지 않는 target/depth 정보를 억지로 제공하지 않고, 실제 사용한 RGB/depth/robot pose,
  target 종류와 optimization level(pixel/pose)을 결과 표에 공개한다.
- Classical pose-level 방법에는 동일한 frozen PnP 결과와 motion pairs를 입력한다.
- Pixel-level 방법에는 동일한 raw corner detections와 visibility mask를 입력한다.
- Targetless 방법을 추가하면 동일 시점 raw RGB를 제공하되 texture/overlap 등 다른 operating condition을
  명시하고 공통 외부-GT endpoint로만 직접 순위를 비교한다.
- Session 전체를 train/test로 분리한다. 같은 물리 위치의 프레임이 양쪽에 섞이지 않게 한다.
- Position holdout은 center/edge/near/far/height를 stratify한다.
- 공통 카메라 교집합 정확도와 전체 카메라 coverage를 함께 보고한다.
- 추가로 등록한 카메라의 코너를 정확도 평균에서 빼거나, 실패 카메라를 조용히 제외하지 않는다.

### 공통 frontend·solver 예산

- 같은 optimization level 안에서는 동일 corner detector 결과 또는 완전히 동일한 재검출 설정을 사용한다.
- Pose-level 방법끼리는 동일 PnP-RANSAC과 marker rejection을, pixel-level 방법끼리는 동일 visual robust
  loss와 outlier threshold를 사용한다.
- A2/A4와 직접 인과 비교하는 iterative arm은 동일 multi-start 수, 최대 iteration, 종료조건을 사용한다.
  Closed-form 등 solver 구조가 다른 방법은 해당 조건이 다름을 공개하고 동일 외부-GT endpoint로 평가한다.
- 실패·예외를 누락하지 않고 failure reason과 함께 저장한다.
- Train reprojection은 진단용이고 최종 순위는 held-out/외부-GT 지표로 정한다.

### Simulation

Simulation은 실제 실험 전 sanity check와 GT transform 오차 분석에 사용한다. 최종 재실행 전 다음을
수정해야 한다.

- visual-only/independent solver에서 `fk_cube`와 GT board pose를 초기값·해에 사용하지 않는다.
- independent rigid alignment를 실제 prediction에 적용한다.
- noisy raw 2D corners와 visibility mask를 저장하고 방법별 held-out reprojection을 계산한다.
- 실물 59 mm cube, marker center/roll, 카메라별 K/D와 실제 camera pose를 사용한다.
- 예외를 숨기지 않고 seed 단위 실패율을 보고한다.
- 최소 30 independent seeds와 사전 정의한 random/stratified splits를 사용한다.
- `bTf`와 `gTc` GT 오차를 분리한다.

Simulation 결과는 실제 외부-GT 결과를 대신하지 않는다.

## 확정 전 사용자 판단 항목

아래 항목은 코드가 자동으로 결정할 수 없으며 실험 책임자가 사전에 확정해야 한다.

1. **최종 주장 범위**
   - 권장: 우선 A4를 Ours-core로 고정한다.
   - A5는 6-DoF correction이 A4 대비 외부 GT에서 통과한 후에만 최종 방법으로 채택한다.
   - 위치-only correction을 유지하면 논문 주장도 “3D translation localization”으로 제한해야 한다.

2. **독립 외부 GT 장비·방법**
   - 우선순위: laser tracker/측정암 → motion capture rigid body → 정밀 가공 3D jig.
   - Robot FK, A4 anchor에 사용한 값, correction label은 외부 GT로 재사용할 수 없다.
   - 어떤 장비를 사용할지와 장비 정확도 사양을 기록해야 한다.

3. **로봇 위치 스케일**
   - `CP_common.ROBOT_POS_SCALE_PINNED`(현재 `1.0`)과 `1.0229` 중 하나를 기존 결과에 맞춰
     선택하지 않는다. 값은 환경변수가 아니라 이 상수 하나이며, 바꾸면 저장된 결과를 전부 재생성한다.
   - 인증 길이 기준물·depth·외부 계측으로 물리적으로 확정한 뒤 모든 결과를 한 값으로 재생성한다.

4. **Non-inferiority margins**
   - `m_R`, `m_P95`, `m_fail`, `m_stratum`을 실제 task 허용오차와 GT uncertainty로 정한다.
   - 결과를 본 뒤 margin을 정하거나 pilot의 관측 효과크기를 margin으로 사용하면 안 된다.
   - Failure margin은 session 실패가 실제 운용에 미치는 비용을 반영하고, 등록 카메라 수 감소 허용 여부도
     별도로 고정한다.

5. **재촬영 규모**
   - 권장 pilot: 독립 재설치 5 sessions × session당 blind test pose 30개.
   - Pilot 분산으로 power analysis 후 최종 session 수를 확정한다.
   - Workspace center/edge/near/far/height와 cube orientation을 균형화한다.

6. **Classical baseline 범위**
   - 권장: C1–C4 모두 실행한다. 동일 입력으로 실행 비용이 작고 선택적 baseline이라는 비판을 줄인다.
   - 논문 Main Ablation에는 넣지 않고 별도 표로 보고한다.

7. **ADD와 ADD-S 선택**
   - Cube의 면 ID와 방향이 task에서 구분되면 ADD를 사용한다.
   - 대칭 방향을 동일 pose로 인정하는 task면 symmetry set을 사전 정의하고 ADD-S를 사용한다.

8. **Recent/targetless 외부 baseline 범위**
   - C1–C4를 SOTA로 부르지 않는다.
   - 추가 방법은 공개 코드, 입력 modality, GPU/의존성, target/scene overlap 적합성을 확인한 뒤 별도
     comparison contract에 사전 등록한다.

## 직접 실행해야 하는 물리 작업

- 큐브 변, marker 크기·중심·roll, ChArUco square 길이를 캘리퍼로 재측정한다.
- 카메라별 30–50장 intrinsic session을 중앙·모서리·거리·tilt ±30–45°로 다시 촬영한다.
- Exposure/gain/focus를 고정하고 target ROI sharpness 및 saturation gate를 활성화한다.
- 블록별 `capture_gate` threshold를 기존 데이터와 pilot에서 검증하고 실제 PASS frame만 저장한다.
- Cube placement는 각 면과 각 카메라의 관측 수를 균형화한다.
- Gripped cube는 위치와 roll/pitch/yaw가 다양한 30–50 자세로 촬영한다.
- 외부 GT 지그/장비로 calibration에 쓰지 않은 blind poses를 측정한다.
- 카메라 재설치 session을 반복하고 각 session의 환경·설정·config hash를 기록한다.

촬영부터 전체 결과 산출까지는
[단일 실행 체크리스트](Check_Calibration.md)를 위에서부터 순서대로 따른다.

## 남은 구현 순서

1. Block-aware gate·host-clock sync·safe-pose executor는 코드 반영 완료. 기존 데이터 replay와 저속
   physical dry-run으로 threshold/경로 검증
2. Target-ROI sharpness·clipping·marker-edge gate를 Step2 저장 조건에 구현하고 카메라별 threshold 고정
3. A2/A3/A4a/A4b/A4 공통 pixel backend와 fair B1은 `CP_final_methods.py`에 구현 완료. 실측 covariance와
   첫 자동 번호 engineering-pilot session으로 production 검증
4. Blind prediction exporter와 external-GT hierarchical evaluator는 구현 완료. 실제 GT schema·failure
   logging으로 end-to-end 검증
5. B2 공정 arm과 C1–C4 same-session classical baseline 연결
6. 필요 시 A5 6-DoF SE(3) correction 구현
7. Simulation 누출·metric 문제 수정 후 30-seed 재실행
8. 기존 데이터 face-roll/PnP 재처리로 전체 파이프라인 검증
9. 외부 GT 포함 재촬영 및 session-paired 최종 평가
10. Paired hierarchical bootstrap CI·Holm 보정·P50/P95/failure table 생성

최종 논문 문장은 9–10단계의 외부-GT 평가와 통계 계약을 통과한 이후에만 확정한다.
