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
- 현재 결론의 대표 행은 A2다. A4는 measured FK covariance 전 preflight이고, A5는 이전 A3의 성능 원인을 설명하는 post-hoc diagnostic이다. 실제 물리 순위는 External GT 이후 결정한다.

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
| A3 (raw-FK hard fixed) | cube+board | unified_joint_optimization | raw-FK-fixed | 5.1587 | 4.7835 | 4.1026 / 6.3959 | 3/3 | Complete (완료) |
| B3 (−cube) | board | unified_joint_optimization | — | 3.8202 | 4.0530 | 4.0530 / N/A | 3/3 | Complete (완료) |

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
| A0 | 1.1851 | 1.7186 | **0.1926** | 6.6803 | 10.1150 | 0.6312 |
| A1 | 2.8468 | 3.8417 | 0.3316 | 6.3167 | 9.2291 | 0.8210 |
| A2 | 3.0243 | 4.3201 | 0.4218 | 3.9992 | 5.9598 | 0.6322 |
| A3 | 2.4999 | 2.6499 | 1.7229 | 5.8562 | 9.0238 | 2.1769 |
| A4 | 3.1156 | 4.4295 | 0.4118 | 4.0375 | 5.9500 | 0.6294 |
| A5 | 4.8563 | 7.3005 | 0.4917 | **3.4706** | **4.4892** | 0.8073 |
| B1 | 2.9129 | 3.9581 | 0.3474 | 6.2871 | 9.1546 | 0.8406 |
| B2 | 4.1681 | 5.7157 | 0.3533 | 3.5610 | 4.8703 | **0.4866** |
| B3 | **1.1829** | **1.7159** | 0.1931 | 6.6767 | 10.1095 | 0.6309 |

### Gripper-to-Fixed (그리퍼카메라–고정카메라 간)

| Method (방법) | Board Pixel (보드 px) | Board Translation (보드 이동 mm) | Board Rotation (보드 회전 deg) | Cube Pixel (큐브 px) | Cube Translation (큐브 이동 mm) | Cube Rotation (큐브 회전 deg) |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| A0 | **4.5445** | 5.4811 | 0.7966 | 7.4402 | 8.1421 | 0.9779 |
| A1 | 4.8356 | 6.1045 | 0.9829 | 7.6100 | 8.6914 | 1.1164 |
| A2 | 5.2890 | 7.2539 | 1.0833 | 6.8987 | 7.6781 | 1.1296 |
| A3 | 4.8364 | **4.5646** | 1.9454 | 7.1666 | 8.0966 | 2.0590 |
| A4 | 5.3034 | 7.2305 | 1.0633 | 6.9064 | 7.7073 | 1.1144 |
| A5 | 5.6332 | 6.8618 | **0.7790** | **6.2895** | **7.2581** | **0.9110** |
| B1 | 4.8536 | 6.0976 | 0.9803 | 7.5907 | 8.6865 | 1.1161 |
| B2 | 5.4611 | 7.9540 | 1.1039 | 7.2526 | 8.1046 | 1.1786 |
| B3 | 4.5447 | 5.4821 | 0.7966 | 7.4385 | 8.1401 | 0.9777 |

### Marker-system End-to-End (마커 시스템 전체 경로)

| System (시스템) | Own Held-out (자체 홀드아웃 px) | Fixed-to-Fixed Board/Cube (고정카메라 간 보드/큐브 px) | Gripper-to-Fixed Board/Cube (그리퍼카메라–고정카메라 간 보드/큐브 px) | Convergence (수렴) |
| --- | ---: | ---: | ---: | ---: |
| Board-only end-to-end | 4.0530 | **1.1832** / 6.6778 | **4.5447** / 7.4389 | 3/3 |
| Cube-only end-to-end | 4.6038 | 3.8587 / **3.5977** | 5.3888 / 7.4576 | 3/3 |
| Board+Cube end-to-end | 3.8901 | 3.0243 / 3.9992 | 5.2890 / **6.8987** | 3/3 |

## Calculation (계산 방식)

For Target $O\in\{board,cube\}$ (표적 $O$):

$$T^{B,(i)}_O=T^B_{C_i}T^{C_i}_{O,\mathrm{PnP}}$$

$$T^B_{C_g}(e)=T^B_G(e)T^G_{C_g}$$

$$T^{B,(g)}_O(e)=T^B_G(e)T^G_{C_g}T^{C_g}_{O,\mathrm{PnP}}$$

Pixel Transfer RMSE (픽셀 전달 평균제곱근오차)는 한 카메라의 측정 PnP 자세를 다른 카메라로 옮겨 실제 검출 코너와 비교한다. Translation/Rotation Consistency (이동/회전 일관성)는 두 경로로 얻은 $T^B_O$의 차이를 mm/deg로 계산한다. Gripper-to-Fixed의 최종값은 pair 성분을 Event RMSE로, Event를 set RMSE로 집계한 뒤 set별 동일 가중치로 계산한다.

## Interpretation Limit (해석 한계)

A의 Fixed-to-Fixed는 방법별 추정값에 의존하고 모든 고정카메라에 함께 존재하는 Systematic Error (계통 오차)를 검출할 수 없으므로 보조 일관성 진단으로만 해석한다. Gripper-to-Fixed는 Hand–Eye Error (핸드–아이 오차)와 FK Error (순기구학 오차)를 분리할 수 없다. 따라서 두 범위는 함께 보고하되 External Absolute Accuracy (외부 절대 정확도)로 부르지 않는다.

## Terminology (용어 설명)

- **$T^B_{C_i}$, Base-to-Fixed-Camera Transform (베이스–고정카메라 변환)**: 고정카메라 외부 파라미터.
- **$T^G_{C_g}$, Hand–Eye Transform (핸드–아이 변환)**: 그리퍼에서 그리퍼카메라로의 변환.
- **$T^B_G(e)$, Robot FK Pose (이벤트별 로봇 순기구학 자세)**: 이벤트 $e$의 베이스–그리퍼 변환이며 평가 중 고정 입력이다.
- **PnP, Perspective-n-Point (3D–2D 자세 추정)**: 3D 표적점과 2D 영상점으로 카메라–표적 자세를 계산한다.
- **RMSE, Root Mean Squared Error (평균제곱근오차)**: 잔차 제곱 평균의 제곱근. px, mm, deg는 서로 합치지 않는다.
- **Reference-dependent Reprojection (기준 의존 재투영)**: 학습 표적 자세에 의존하는 보조 진단으로 External GT가 아니다.

## Required Next Experiment (다음 필수 실험)

Independent External GT (독립 외부 정답)가 확정되면 Blind Position Holdout (비공개 위치 홀드아웃)으로 Translation Error (이동 오차), Rotation Error (회전 오차), P95, Failure Rate (실패율)를 다시 계산한다. 그 전에는 내부 지표만 유지한다.
