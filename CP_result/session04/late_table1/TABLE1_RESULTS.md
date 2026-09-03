# Session04 Calibration Evaluation (캘리브레이션 평가)

> Status: Pre-GT Internal Evaluation (외부 GT 전 내부 평가). 이 문서는 External GT (외부 정답)를 사용한 절대 정확도 순위를 제시하지 않는다.

## Current Data Warnings (현재 데이터 경고)

- Evaluation support: fixed cameras `0, 1, 3`, overall 17 obs / 340 corners; board 8 / 232, cube 9 / 108.
- Split support: 9 eligible sets; dropped sets `0, 1, 2, 3`.
- Cube detection: 117 images read, 108 accepted PnP observations, 99 core multiface selected, 2 PnP-RMSE rejections.
- Board-Cube conflict: direct PnP disagreement is 10.8077 mm translation RMSE and 0.5270 deg max rotation; joint solve mitigates it but does not remove the cause.

## Evaluation Decision (평가 구성 결정)

- A — Fixed-to-Fixed/e_cross는 각 방법이 추정한 카메라 자세로 계산하는 방법별 Supplementary Held-out Consistency (보조 홀드아웃 일관성)다. Robot FK는 쓰지 않지만 독립 기준선이나 순위 지표는 아니다.
- B — OpenCV Relative-pose Baseline은 Main-method Transform, Robot FK, Hand–Eye, Shared Target Pose를 쓰지 않는 Independent Reference (독립 기준선)이며 별도 결과 파일로 보고한다.
- Gripper-to-Fixed (그리퍼카메라–고정카메라 간)는 실제 Board/Cube Image Corners (보드/큐브 영상 코너)를 사용하지만, 예측 경로에는 Robot FK와 Hand–Eye (핸드–아이 변환)가 포함된다.
- 각 set의 최초 고정카메라 관측은 최적화에서 한 번만 사용한다. Gripper-to-Fixed 평가는 이 fixed anchor를 같은 set의 모든 held-out gripper Event와 연결하고, Event→set→set 동일가중 순서로 집계한다.
- 공식 결과는 물리 config의 nominal metric scale `1.0`만 사용한다. 데이터에서 추정한 scale은 별도 diagnostic 결과로만 허용한다.
- Board (보드)와 Cube (큐브)는 모두 촬영 원본에서 평가한다. 캘리브레이션에 사용한 마커 종류와 평가 표적 종류를 동일시하지 않는다.
- Reference-dependent Reprojection (기준 의존 재투영)은 Secondary Diagnostic (보조 진단)이며 방법 순위에 사용하지 않는다.
- 현재 결론의 대표 행은 A2다. A4는 measured FK covariance 전 preflight이고, A5는 이전 A3의 성능 원인을 설명하는 post-hoc diagnostic이다. Independent External GT는 다음주 예정 태스크이므로 실제 물리 순위는 현재 산출하지 않는다.

## Internal-Only Claim Envelope (현재 가능한 최대 결론)

| Claim Type | Allowed Now (지금 가능) | Not Allowed Now (지금 금지) |
| --- | --- | --- |
| Main result | A2 is the strongest confirmatory internal row under matched held-out pixel contrasts. | A2 is physically most accurate in robot base. |
| Method extension | A4 is a preflight extension candidate; A2->A4 is practically tied on internal held-out px. | Corrected-FK soft factor is superior before measured FK covariance. |
| FK diagnosis | A3 shows raw-FK hard fixing hurts cube reprojection; A5 separates raw/aligned and soft/hard causes. | Raw FK or vision-aligned FK is external GT. |
| Metric scope | Report board/cube held-out px, set-equal-weight px, paired set bootstrap CI, Fixed-to-Fixed, Gripper-to-Fixed. | Merge all metrics into one final physical ranking. |
| Data risk | Surface dropped sets, support imbalance, detection failures, and 10.8077 mm Board-Cube disagreement. | Say the joint solve removed the systematic disagreement. |

Independent External GT is scheduled as a next-week task. Until then, this report stops at the strongest internally defensible evidence.

## Matched Contrast Decision Table (비교실험 구성 확정표)

모든 행을 하나의 전체 순위로 세우지 않고, 한 번에 한 요소만 달라지는 contrast만 해석한다.

| Tier (구분) | Direct Contrast (직접 비교) | Question (검증 질문) | Primary Metric (주 지표) | Session04 Result | Decision (판정) |
| --- | --- | --- | --- | --- | --- |
| Confirmatory internal | A0 <-> B3 | Board-only에서 sequential freeze와 unified feedback 차이가 있는가 | held-out board px | Board 4.0530 -> 4.0531 (+0.0001) | 동률. Board-only에서는 통합 자체가 추가 이득을 만들지 않는다. |
| Confirmatory internal | A0 -> A1 | 순차법에 cube residual을 추가하면 board 성능이 좋아지는가 | held-out board px, N_reg | Board 4.0530 -> 4.0645 (+0.0116); N_reg 3 -> 3 | Board는 미세 악화. 순차 구조에서는 cube 추가 이득이 보이지 않는다. |
| Confirmatory internal | A1 -> A2 | Vision-only 조건에서 unified feedback이 도움이 되는가 | held-out board/cube px | Board 4.0645 -> 3.9840 (-0.0805); Cube 4.1402 -> 3.5958 (-0.5443) | 두 target 모두 개선. 현재 내부 확증에서 가장 강한 긍정 contrast다. |
| Confirmatory internal | B3 -> A2 | Unified 조건에서 cube residual이 board calibration에도 도움이 되는가 | held-out board px | Board 4.0531 -> 3.9840 (-0.0690) | Board shared component가 개선. 단 marker-system 전체 성능 주장은 아니다. |
| Confirmatory internal | A2 -> A3 | Vision-estimated cube pose를 raw-FK hard fixed로 바꾸면 어떤가 | held-out board/cube px | Board 3.9840 -> 4.1025 (+0.1185); Cube 3.5958 -> 6.3959 (+2.8000) | 특히 cube가 크게 악화. raw FK를 GT로 해석하면 안 된다. |
| Preflight | B1 -> A4 | 같은 soft FK factor에서 sequential과 unified 중 무엇이 나은가 | held-out board/cube px | Board 4.0648 -> 3.9884 (-0.0764); Cube 4.1182 -> 3.5805 (-0.5378) | 통합 개선 경향. measured covariance 없이 내부 preflight로만 해석한다. |
| Preflight | A2 -> A4 | Unified vision-only에 soft FK factor를 추가하면 이득이 있는가 | held-out board/cube px | Board 3.9840 -> 3.9884 (+0.0044); Cube 3.5958 -> 3.5805 (-0.0153) | 사실상 동률. A4는 방법 확장 후보지만 현재 우월성 주장은 금지. |
| Preflight | B2 -> A4 | Soft FK 조건에서 board residual이 cube 보정에 도움 되는가 | held-out cube px | Cube 4.4827 -> 3.5805 (-0.9023) | Cube가 개선. board residual은 soft-FK cube 추정에 도움이 된다. |
| Post-hoc | A3 -> A5 | Raw FK hard fixed와 vision-aligned FK hard fixed의 차이는 무엇인가 | internal metrics only | Board 4.1025 -> 3.8804 (-0.2222); Cube 6.3959 -> 3.2274 (-3.1685) | A5는 원인 진단 전용. 독립 correction 또는 물리 순위가 아니다. |
| Post-hoc | A4 -> A5 | 같은 aligned FK를 soft factor와 hard fixed로 쓰면 무엇이 달라지는가 | internal metrics only | Board 3.9884 -> 3.8804 (-0.1080); Cube 3.5805 -> 3.2274 (-0.3531) | A5는 원인 진단 전용. 내부 px 하나로 물리 우열을 정하지 않는다. |

> 현재 메인 결론은 A2다. A4는 measured FK covariance가 들어오기 전까지 방법 확장 후보이고, A5는 post-hoc 원인 진단이다.

## Metric Decision Matrix (평가지표 판정표)

| Metric (지표) | Tier (등급) | Use (사용법) | Limit (제한) | Current Support (현재 근거) |
| --- | --- | --- | --- | --- |
| Train reprojection RMSE | Solver diagnostic | 수렴/적합 상태 확인 | 학습 관측에 대한 fit이므로 방법 우월성 지표가 아니다. | row별 train residual |
| Own-marker held-out RMSE | Primary internal pixel metric | matched contrast의 board/cube별 주 지표 | 같은 set의 다른 event라 새 위치 일반화나 물리 GT가 아니다. | Board 703 corners, Cube 236 corners |
| Pooled overall RMSE | Secondary summary | 같은 marker population 내부에서만 참고 | Board corner 지지도가 커서 전체값이 board에 치우친다. | 전체 순위 금지 |
| Set-equal-weight RMSE | Exploratory support-bias check | corner-pooled 값 옆에 병기 | n=9 sets라 CI/유의성 주장은 아직 약하다. | corner -> event -> set -> set 동일가중 |
| Fixed-to-Fixed Board/Cube | Supplementary FK-free subsystem metric | 고정카메라 상대 일관성 진단 | 모든 고정카메라에 함께 존재하는 systematic error와 절대 물리 오차를 검출하지 못한다. | fixed cameras 0, 1, 3; 17 obs |
| Gripper-to-Fixed Board/Cube | Supplementary FK-dependent closure metric | 전체 chain 내부 진단 | FK와 Hand-Eye가 섞이며 fixed anchor 일부는 train 관측이다. | mixed train-anchor/held-out internal closure |
| Reference-dependent reprojection | Secondary diagnostic | 공유 target pose 기준의 보조 확인 | reference가 fitted target이므로 ranking 지표가 아니다. | cross-target v8 artifact |
| Seed mean +/- std | Stability diagnostic | 3개 초기화 perturbation 안정성 확인 | 독립 실험 표본이 아니라 통계적 반복으로 해석하지 않는다. | 27/27 converged |
| External TRE/rotation/P95/failure | Scheduled next-week external validation | 다음주 예정 태스크 | 현재 보고서 생성 시점에는 미계산. 다음주 Independent External GT로 Translation Error, Rotation Error, P95, Failure Rate를 산출한다. | planned external validation boundary |

## Exploratory Paired Set Bootstrap CI (탐색적 paired set bootstrap CI)

각 contrast는 같은 held-out set을 paired unit으로 묶고, set을 10,000회 replacement resampling했다. 값은 `second - first` RMSE 차이이며 음수는 두 번째 방법의 내부 px가 더 낮다는 뜻이다. `n=9 sets`라 유의성 검정이 아니라 방향성 민감도 점검이다.

| Tier | Contrast | Target | Direction | Δ pooled / set-equal px | Set-bootstrap 95% CI px | Interpretation |
| --- | --- | --- | --- | ---: | ---: | --- |
| Confirmatory internal | A0 <-> B3 | board | B3 - A0 | 0.0001 / 0.0001 | [-0.0000, 0.0001] | n=9; interval crosses 0; direction is exploratory; 동률. Board-only에서는 통합 자체가 추가 이득을 만들지 않는다. |
| Confirmatory internal | A0 -> A1 | board | A1 - A0 | 0.0116 / -0.0027 | [-0.0568, 0.0523] | n=9; interval crosses 0; direction is exploratory; Board는 미세 악화. 순차 구조에서는 cube 추가 이득이 보이지 않는다. |
| Confirmatory internal | A1 -> A2 | board | A2 - A1 | -0.0805 / -0.0597 | [-0.1223, 0.0076] | n=9; interval crosses 0; direction is exploratory; 두 target 모두 개선. 현재 내부 확증에서 가장 강한 긍정 contrast다. |
| Confirmatory internal | A1 -> A2 | cube | A2 - A1 | -0.5443 / -0.7961 | [-1.6750, 0.0870] | n=9; interval crosses 0; direction is exploratory; 두 target 모두 개선. 현재 내부 확증에서 가장 강한 긍정 contrast다. |
| Confirmatory internal | B3 -> A2 | board | A2 - B3 | -0.0690 / -0.0625 | [-0.1226, 0.0002] | n=9; interval crosses 0; direction is exploratory; Board shared component가 개선. 단 marker-system 전체 성능 주장은 아니다. |
| Confirmatory internal | A2 -> A3 | board | A3 - A2 | 0.1185 / 0.1642 | [-0.3669, 0.6408] | n=9; interval crosses 0; direction is exploratory; 특히 cube가 크게 악화. raw FK를 GT로 해석하면 안 된다. |
| Confirmatory internal | A2 -> A3 | cube | A3 - A2 | 2.8000 / 2.8956 | [-0.0196, 5.4695] | n=9; interval crosses 0; direction is exploratory; 특히 cube가 크게 악화. raw FK를 GT로 해석하면 안 된다. |
| Preflight | B1 -> A4 | board | A4 - B1 | -0.0764 / -0.0563 | [-0.1139, 0.0096] | n=9; interval crosses 0; direction is exploratory; 통합 개선 경향. measured covariance 없이 내부 preflight로만 해석한다. |
| Preflight | B1 -> A4 | cube | A4 - B1 | -0.5378 / -0.7811 | [-1.6341, 0.0550] | n=9; interval crosses 0; direction is exploratory; 통합 개선 경향. measured covariance 없이 내부 preflight로만 해석한다. |
| Preflight | A2 -> A4 | board | A4 - A2 | 0.0044 / 0.0034 | [-0.0049, 0.0118] | n=9; interval crosses 0; direction is exploratory; 사실상 동률. A4는 방법 확장 후보지만 현재 우월성 주장은 금지. |
| Preflight | A2 -> A4 | cube | A4 - A2 | -0.0153 / -0.0294 | [-0.0555, 0.0060] | n=9; interval crosses 0; direction is exploratory; 사실상 동률. A4는 방법 확장 후보지만 현재 우월성 주장은 금지. |
| Preflight | B2 -> A4 | cube | A4 - B2 | -0.9023 / -0.6836 | [-2.0077, 0.6209] | n=9; interval crosses 0; direction is exploratory; Cube가 개선. board residual은 soft-FK cube 추정에 도움이 된다. |
| Post-hoc | A3 -> A5 | board | A5 - A3 | -0.2222 / -0.2179 | [-0.5426, 0.0937] | n=9; interval crosses 0; direction is exploratory; A5는 원인 진단 전용. 독립 correction 또는 물리 순위가 아니다. |
| Post-hoc | A3 -> A5 | cube | A5 - A3 | -3.1685 / -3.2827 | [-5.7887, -0.5075] | n=9; negative interval; internal improvement direction is stable; A5는 원인 진단 전용. 독립 correction 또는 물리 순위가 아니다. |
| Post-hoc | A4 -> A5 | board | A5 - A4 | -0.1080 / -0.0571 | [-0.4869, 0.3263] | n=9; interval crosses 0; direction is exploratory; A5는 원인 진단 전용. 내부 px 하나로 물리 우열을 정하지 않는다. |
| Post-hoc | A4 -> A5 | cube | A5 - A4 | -0.3531 / -0.3577 | [-1.1905, 0.4725] | n=9; interval crosses 0; direction is exploratory; A5는 원인 진단 전용. 내부 px 하나로 물리 우열을 정하지 않는다. |

> 이 표에서 0을 지나지 않는 contrast도 external physical accuracy를 증명하지 않는다. 내부 held-out set에서 방향이 덜 흔들린다는 뜻까지만 허용한다.

## Code-consistency Audit (코드 일치성 검증)

### 카메라 간 Relative Pose

메인 A0–A5·B1–B3 optimizer에는 camera-to-camera transform을 추정·평균·연결하는 함수, observation, residual, objective term이 **0개**다. 카메라는 오직 shared target-pose variables (공유 타깃 자세 변수)를 통해 결합된다.

$$T_{C_iC_j}=T_{BC_i}^{-1}T_{BC_j}$$

위 transform은 solve 이후 camera pose에서 유도하는 값이다. 단, 저장소 전체에 relative-pose 계산이 0개인 것은 아니다. 평가 A는 방법별 supplementary held-out consistency를 사후 계산하고, 평가 B는 메인 추정값과 독립적인 OpenCV direct relative-pose baseline을 계산한다.

> **판정:** ‘cube pose로 대표 camera-relative pose를 만든 뒤 optimizer에 통합한다’는 서술은 코드와 불일치한다.

### 3항 Weighted-sum Loss

현재 목적함수의 additive term은 최대 **2개**다.

- A0·A1·A2·A3·A5·B3: robust visual reprojection **1항**
- A4·B1·B2: robust visual reprojection + whitened robust FK factor **2항**
- `pose_error`와 `FK_constraint`: 서로 다른 두 항이 아니라 동일한 FK factor의 두 표현
- `w1`, `w2`, `w3`: 사용하지 않음
- 상대 scale: visual pixel `f_scale`과 FK covariance whitening `Sigma^(-1/2)`로 결정

> **판정:** `w1·reprojection + w2·pose_error + w3·FK_constraint`는 현재 코드에 없는 부정확한 서술이다.

### A3의 raw-FK-fixed 의미

$$T_{B\,cube}(s)=F_s^{raw}T_{cube\ center\rightarrow object}^{mech}$$

A3가 고정하는 cube pose는 set별 controller raw FK pose에 영상과 무관하게 사전 등록한 mechanical frame map `R_y(180°)`를 적용한 pose다. cube-center 원점 이동은 0이고, `Delta_train`이나 aligned FK artifact를 사용하지 않는다. A3 최종 optimizer에서는 이 pose를 상수로 고정하고 visual reprojection 1항만 최소화한다.

> **판정:** A3는 pure raw-FK hard constraint이지만 external GT는 아니다. tool4/CAD frame 정의 오차가 그대로 결과에 들어간다.

### A5의 vision-aligned-FK-fixed 의미

$$T_{B\,cube}(s)=F_s^{raw}\Delta_{train}$$

A5는 board와 held-out을 제외한 train eye-in-hand cube 영상으로 추정한 `Delta_train`을 적용한 뒤 set별 cube pose를 상수로 고정한다. A4와 동일한 aligned-FK artifact를 사용하지만 A4처럼 covariance factor로 완화하지 않는다.

> **판정:** A5는 이전 A3 결과의 원인을 분리하는 post-hoc diagnostic이다. 독립 실측 correction이나 external GT가 아니다.

## Table 1 Optimization Results (표 1 최적화 결과)

> **굵은 값**은 `Complete` 행 중 Board/Cube별 held-out RMSE 최솟값이다. Preflight와 post-hoc 행은 수치가 더 낮아도 확증 결과로 강조하지 않는다. Train/Own Overall은 marker population이 달라 전체 최솟값을 강조하지 않는다.

### Confirmatory Internal (확증 내부)

코드 내부 ablation과 calibration 안정성 검증에 쓰는 Complete 행이다.

| Method (방법) | 기여도2 - Marker Set (마커 구성) | 기여도1 - Optimization (최적화) | 기여도3 - Cube Pose (큐브 자세 처리) | Train Overall (학습 전체 px) | Own Held-out Overall (자체 홀드아웃 전체 px) | Board/Cube Held-out (보드/큐브 홀드아웃 px) | Convergence (수렴) | Status (상태) |
| --- | --- | --- | --- | ---: | ---: | ---: | ---: | --- |
| A0 (baseline) | board | sequential_frozen_stage | — | 3.8202 | 4.0530 | 4.0530 / N/A | 3/3 | Complete (완료) |
| A1 (+cube) | cube+board | sequential_frozen_stage | estimated | 3.7923 | 4.0837 | 4.0645 / 4.1402 | 3/3 | Complete (완료) |
| A2 (+unified) | cube+board | unified_joint_optimization | estimated | 3.7421 | 3.8901 | **3.9840** / **3.5958** | 3/3 | Complete (완료) |
| A3 (raw-FK hard fixed) | cube+board | unified_joint_optimization | raw-FK-fixed | 5.1587 | 4.7835 | 4.1025 / 6.3959 | 3/3 | Complete (완료) |
| B3 (−cube) | board | unified_joint_optimization | — | 3.8202 | 4.0531 | 4.0531 / N/A | 3/3 | Complete (완료) |

### Preflight (예비실험)

Simulation prior FK covariance를 쓰므로 물리 우월성 주장에는 쓰지 않는다.

| Method (방법) | 기여도2 - Marker Set (마커 구성) | 기여도1 - Optimization (최적화) | 기여도3 - Cube Pose (큐브 자세 처리) | Train Overall (학습 전체 px) | Own Held-out Overall (자체 홀드아웃 전체 px) | Board/Cube Held-out (보드/큐브 홀드아웃 px) | Convergence (수렴) | Status (상태) |
| --- | --- | --- | --- | ---: | ---: | ---: | ---: | --- |
| A4 (corrected-FK soft factor) | cube+board | unified_joint_optimization | corrected-FK-factor | 3.7441 | 3.8899 | 3.9884 / 3.5805 | 3/3 | Preflight — Simulation Prior (예비실험 — 시뮬레이션 사전값) |
| B1 (−Unified) | cube+board | sequential_frozen_stage | corrected-FK-factor | 3.7887 | 4.0783 | 4.0648 / 4.1182 | 3/3 | Preflight — Simulation Prior (예비실험 — 시뮬레이션 사전값) |
| B2 (−board) | cube | unified_joint_optimization | corrected-FK-factor | 3.0269 | 4.4827 | N/A / 4.4827 | 3/3 | Preflight — Simulation Prior (예비실험 — 시뮬레이션 사전값) |

### Post-hoc Diagnostics (사후 원인 진단)

결과 해석 뒤 원인을 분리하기 위한 진단 행이며 메인 순위에서 제외한다.

| Method (방법) | 기여도2 - Marker Set (마커 구성) | 기여도1 - Optimization (최적화) | 기여도3 - Cube Pose (큐브 자세 처리) | Train Overall (학습 전체 px) | Own Held-out Overall (자체 홀드아웃 전체 px) | Board/Cube Held-out (보드/큐브 홀드아웃 px) | Convergence (수렴) | Status (상태) |
| --- | --- | --- | --- | ---: | ---: | ---: | ---: | --- |
| A5 (Post-hoc diagnostic (vision-aligned FK fixed)) | cube+board | unified_joint_optimization | vision-aligned-FK-fixed | 3.9648 | 3.7270 | 3.8804 / 3.2274 | 3/3 | Post-hoc Diagnostic (사후 원인 진단) |

### Set-equal-weight Held-out RMSE (set 동일가중 홀드아웃)

Held-out corner 지지도는 Board `703` / Cube `236`이므로 corner-pooled Overall은 Board가 약 `74.9%`를 차지한다. 아래 표는 같은 관측을 corner-pooled와 `corner → event → set → set 동일가중` 두 방식으로 집계한 값이다. 두 값의 차이는 정확도 변화가 아니라 set별 corner 지지도 불균형의 크기이며, 방법 간 방향이 달라지는 행은 corner 지지도에 의존하는 결론이므로 단독으로 해석하지 않는다.

| Method (방법) | Board pooled / set-equal (보드 px) | Cube pooled / set-equal (큐브 px) | Overall pooled / set-equal (전체 px) | Status (상태) |
| --- | ---: | ---: | ---: | --- |
| A0 (baseline) | 4.0530 / 4.1922 | N/A / N/A | 4.0530 / 4.1922 | Complete (완료) |
| A1 (+cube) | 4.0645 / 4.1895 | 4.1402 / 4.8990 | 4.0837 / 4.4282 | Complete (완료) |
| A2 (+unified) | 3.9840 / 4.1298 | 3.5958 / 4.1029 | 3.8901 / 4.1369 | Complete (완료) |
| A3 (raw-FK hard fixed) | 4.1025 / 4.2940 | 6.3959 / 6.9985 | 4.7835 / 5.3355 | Complete (완료) |
| A4 (corrected-FK soft factor) | 3.9884 / 4.1332 | 3.5805 / 4.0734 | 3.8899 / 4.1339 | Preflight — Simulation Prior (예비실험 — 시뮬레이션 사전값) |
| A5 (Post-hoc diagnostic (vision-aligned FK fixed)) | 3.8804 / 4.0761 | 3.2274 / 3.7157 | 3.7270 / 4.0251 | Post-hoc Diagnostic (사후 원인 진단) |
| B1 (−Unified) | 4.0648 / 4.1895 | 4.1182 / 4.8545 | 4.0783 / 4.4164 | Preflight — Simulation Prior (예비실험 — 시뮬레이션 사전값) |
| B2 (−board) | N/A / N/A | 4.4827 / 4.7570 | 4.4827 / 4.7570 | Preflight — Simulation Prior (예비실험 — 시뮬레이션 사전값) |
| B3 (−cube) | 4.0531 / 4.1923 | N/A / N/A | 4.0531 / 4.1923 | Complete (완료) |

> Set 동일가중 값은 corner-pooled 값을 대체하지 않는 보조 지표이며, `n=9 sets`이므로 이 차이만으로 유의성을 주장하지 않는다.

> `Convergence 3/3`은 서로 다른 초기화 seed 3회 모두에서 SciPy solver가 `success=True`로 종료됐다는 뜻이다. Sequential 행은 두 stage가 모두 성공해야 하며, B1은 stage 1과 모든 fixed-camera stage 2가 성공해야 1회 수렴으로 센다. 이는 solver 종료 조건 충족을 뜻할 뿐, 절대 정확도나 전역 최적해를 보장하지 않는다.

## Objective Block Diagnostics (목적함수 블록 진단)

| Method (방법) | FK 처리 | Visual residual components (시각 잔차 수) | FK blocks / components (FK 블록/잔차 수) | Visual robust cost (시각 비용) | FK robust cost (FK 비용) | FK cost fraction (FK 비용 비율) |
| --- | --- | ---: | ---: | ---: | ---: | ---: |
| A2 | estimated | 7372 | 0 / 0 | 23950.81 | 0.00 | 0.000% |
| A3 | raw-FK-fixed (hard constant; residual 없음) | 7372 | 0 / 0 | 36683.56 | 0.00 | 0.000% |
| A4 | corrected-FK-factor | 7372 | 9 / 54 | 23960.94 | 29.28 | 0.122% |
| A5 | vision-aligned-FK-fixed (hard constant; residual 없음) | 7372 | 0 / 0 | 25788.40 | 0.00 | 0.000% |
| B1 | corrected-FK-factor | 6068 | 9 / 54 | 23343.41 | 33.96 | 0.145% |
| B2 | corrected-FK-factor | 1448 | 9 / 54 | 3348.98 | 38.57 | 1.139% |

> 이 비율은 최종 목적함수 값의 분해다. 각 항의 Jacobian과 변수 연결 구조가 다르므로, FK cost 비율을 파라미터 영향력 비율로 해석하면 안 된다.

## Camera-scope Diagnostics (카메라 범위 진단)

> **굵은 값**은 각 target·단위 열의 최솟값이다. A의 Fixed-to-Fixed는 보조 일관성 지표이므로 굵은 값이 절대 정확도 순위를 뜻하지 않는다.

### A — Fixed-to-Fixed 보조 Held-out 일관성

| Method (방법) | Board Pixel (보드 px) | Board Translation (보드 이동 mm) | Board Rotation (보드 회전 deg) | Cube Pixel (큐브 px) | Cube Translation (큐브 이동 mm) | Cube Rotation (큐브 회전 deg) |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| A0 | 1.1851 | 1.7187 | **0.1926** | 6.6803 | 10.1151 | 0.6312 |
| A1 | 2.8473 | 3.8423 | 0.3315 | 6.3173 | 9.2300 | 0.8210 |
| A2 | 3.0241 | 4.3197 | 0.4218 | 3.9997 | 5.9604 | 0.6321 |
| A3 | 2.4999 | 2.6496 | 1.7229 | 5.8563 | 9.0239 | 2.1769 |
| A4 | 3.1156 | 4.4295 | 0.4118 | 4.0375 | 5.9500 | 0.6294 |
| A5 | 4.8563 | 7.3005 | 0.4917 | **3.4706** | **4.4892** | 0.8073 |
| B1 | 2.9129 | 3.9581 | 0.3474 | 6.2871 | 9.1546 | 0.8406 |
| B2 | 4.1681 | 5.7157 | 0.3533 | 3.5610 | 4.8703 | **0.4866** |
| B3 | **1.1817** | **1.7139** | 0.1933 | 6.6759 | 10.1082 | 0.6306 |

### Gripper-to-Fixed (그리퍼카메라–고정카메라 간)

| Method (방법) | Board Pixel (보드 px) | Board Translation (보드 이동 mm) | Board Rotation (보드 회전 deg) | Cube Pixel (큐브 px) | Cube Translation (큐브 이동 mm) | Cube Rotation (큐브 회전 deg) |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| A0 | **4.5444** | 5.4810 | 0.7966 | 7.4402 | 8.1421 | 0.9779 |
| A1 | 4.8357 | 6.1045 | 0.9829 | 7.6103 | 8.6918 | 1.1164 |
| A2 | 5.2889 | 7.2536 | 1.0833 | 6.8987 | 7.6782 | 1.1296 |
| A3 | 4.8363 | **4.5645** | 1.9455 | 7.1667 | 8.0968 | 2.0591 |
| A4 | 5.3034 | 7.2305 | 1.0633 | 6.9064 | 7.7073 | 1.1144 |
| A5 | 5.6332 | 6.8618 | **0.7789** | **6.2895** | **7.2581** | **0.9110** |
| B1 | 4.8536 | 6.0976 | 0.9803 | 7.5907 | 8.6865 | 1.1161 |
| B2 | 5.4611 | 7.9540 | 1.1039 | 7.2526 | 8.1045 | 1.1786 |
| B3 | 4.5449 | 5.4827 | 0.7966 | 7.4381 | 8.1394 | 0.9777 |

### Marker-system End-to-End (마커 시스템 전체 경로)

| System (시스템) | Own Held-out (자체 홀드아웃 px) | Fixed-to-Fixed Board/Cube (고정카메라 간 보드/큐브 px) | Gripper-to-Fixed Board/Cube (그리퍼카메라–고정카메라 간 보드/큐브 px) | Convergence (수렴) |
| --- | ---: | ---: | ---: | ---: |
| Board-only end-to-end | 4.0531 | **1.1831** / 6.6780 | **4.5447** / 7.4389 | 3/3 |
| Cube-only end-to-end | 4.6037 | 3.8585 / **3.5981** | 5.3888 / 7.4576 | 3/3 |
| Board+Cube end-to-end | 3.8901 | 3.0241 / 3.9997 | 5.2889 / **6.8987** | 3/3 |

## Calculation (계산 방식)

For Target $O\in\{board,cube\}$ (표적 $O$):

$$T^{B,(i)}_O=T^B_{C_i}T^{C_i}_{O,\mathrm{PnP}}$$

$$T^B_{C_g}(e)=T^B_G(e)T^G_{C_g}$$

$$T^{B,(g)}_O(e)=T^B_G(e)T^G_{C_g}T^{C_g}_{O,\mathrm{PnP}}$$

Pixel Transfer RMSE (픽셀 전달 평균제곱근오차)는 한 카메라의 측정 PnP 자세를 다른 카메라로 옮겨 실제 검출 코너와 비교한다. Translation/Rotation Consistency (이동/회전 일관성)는 두 경로로 얻은 $T^B_O$의 차이를 mm/deg로 계산한다. Gripper-to-Fixed의 최종값은 pair 성분을 Event RMSE로, Event를 set RMSE로 집계한 뒤 set별 동일 가중치로 계산한다.

Held-out reprojection의 기본값은 corner-pooled RMSE $\sqrt{\frac{1}{2N}\sum(du^2+dv^2)}$이다. 같은 관측을 `corner → Event RMSE → set RMSE → set별 동일 가중치` 순서로 다시 집계한 값을 Set-equal-weight로 병기한다. 두 값은 corner 지지도가 set마다 같을 때만 일치하므로, 차이는 정확도가 아니라 지지도 불균형의 크기를 뜻한다.

## Interpretation Limit (해석 한계)

A의 Fixed-to-Fixed는 방법별 추정값에 의존하고 모든 고정카메라에 함께 존재하는 Systematic Error (계통 오차)를 검출할 수 없으므로 보조 일관성 진단으로만 해석한다. Gripper-to-Fixed는 Hand–Eye Error (핸드–아이 오차)와 FK Error (순기구학 오차)를 분리할 수 없다. 따라서 두 범위는 함께 보고하되 External Absolute Accuracy (외부 절대 정확도)로 부르지 않는다.

## Terminology (용어 설명)

- **$T^B_{C_i}$, Base-to-Fixed-Camera Transform (베이스–고정카메라 변환)**: 고정카메라 외부 파라미터.
- **$T^G_{C_g}$, Hand–Eye Transform (핸드–아이 변환)**: 그리퍼에서 그리퍼카메라로의 변환.
- **$T^B_G(e)$, Robot FK Pose (이벤트별 로봇 순기구학 자세)**: 이벤트 $e$의 베이스–그리퍼 변환이며 평가 중 고정 입력이다.
- **PnP, Perspective-n-Point (3D–2D 자세 추정)**: 3D 표적점과 2D 영상점으로 카메라–표적 자세를 계산한다.
- **RMSE, Root Mean Squared Error (평균제곱근오차)**: 잔차 제곱 평균의 제곱근. px, mm, deg는 서로 합치지 않는다.
- **Reference-dependent Reprojection (기준 의존 재투영)**: 학습 표적 자세에 의존하는 보조 진단으로 External GT가 아니다.

## Scheduled External GT Task (다음주 예정 태스크)

Independent External GT (독립 외부 정답)는 다음주 예정 태스크로 진행한다. 따라서 현재 문서는 내부 지표로 가능한 최대 보고서이며, 다음주 외부 GT 수집/평가 후 Translation Error, Rotation Error, P95, Failure Rate 같은 최종 물리 정확도 지표를 별도 산출한다.
