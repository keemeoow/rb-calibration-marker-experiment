# Session02 Calibration Evaluation (캘리브레이션 평가)

> Status: Pre-GT Internal Evaluation (외부 GT 전 내부 평가). 이 문서는 External GT (외부 정답)를 사용한 절대 정확도 순위를 제시하지 않는다.

## Evaluation Decision (평가 구성 결정)

- Fixed-to-Fixed (고정카메라 간)는 Robot FK (로봇 순기구학) 없이 고정카메라 부분만 평가한다.
- Gripper-to-Fixed (그리퍼카메라–고정카메라 간)는 실제 Board/Cube Image Corners (보드/큐브 영상 코너)를 사용하지만, 예측 경로에는 Robot FK와 Hand–Eye (핸드–아이 변환)가 포함된다.
- Board (보드)와 Cube (큐브)는 모두 촬영 원본에서 평가한다. 캘리브레이션에 사용한 마커 종류와 평가 표적 종류를 동일시하지 않는다.
- Reference-dependent Reprojection (기준 의존 재투영)은 Secondary Diagnostic (보조 진단)이며 방법 순위에 사용하지 않는다.

## Table 1 Optimization Results (표 1 최적화 결과)

| Method (방법) | Marker Set (마커 구성) | Optimization (최적화) | Cube Pose (큐브 자세 처리) | Train Overall (학습 전체 px) | Own Held-out Overall (자체 홀드아웃 전체 px) | Board/Cube Held-out (보드/큐브 홀드아웃 px) | Convergence (수렴) | Status (상태) |
| --- | --- | --- | --- | ---: | ---: | ---: | ---: | --- |
| A0 (baseline) | board | sequential_frozen_stage | — | 3.8654 | 2.4675 | 2.4675 / N/A | 3/3 | Complete (완료) |
| A1 (+cube) | cube+board | sequential_frozen_stage | estimated | 3.7146 | 2.9942 | 2.5090 / 4.2264 | 3/3 | Complete (완료) |
| A2 (+unified) | cube+board | unified_joint_optimization | estimated | 3.7118 | 2.9715 | 2.4863 / 4.2015 | 3/3 | Complete (완료) |
| A3 (Ours (full)) | cube+board | unified_joint_optimization | FK-fixed | 3.9662 | 2.5315 | 2.5019 / 2.6276 | 3/3 | Complete (완료) |
| A4 (Ours (corrected-FK factor)) | cube+board | unified_joint_optimization | corrected-FK-factor | 3.7115 | 2.9525 | 2.4861 / 4.1435 | 3/3 | Preflight — Simulation Prior (예비실험 — 시뮬레이션 사전값) |
| B1 (−Unified) | cube+board | sequential_frozen_stage | corrected-FK-factor | 3.7136 | 2.9729 | 2.5082 / 4.1624 | 3/3 | Preflight — Simulation Prior (예비실험 — 시뮬레이션 사전값) |
| B2 (−board) | cube | unified_joint_optimization | corrected-FK-factor | 3.0497 | 4.2202 | N/A / 4.2202 | 3/3 | Preflight — Simulation Prior (예비실험 — 시뮬레이션 사전값) |
| B3 (−cube) | board | unified_joint_optimization | — | 3.8654 | 2.4675 | 2.4675 / N/A | 3/3 | Complete (완료) |

## Objective Block Diagnostics (목적함수 블록 진단)

| Method (방법) | FK 처리 | Visual residual components (시각 잔차 수) | FK blocks / components (FK 블록/잔차 수) | Visual robust cost (시각 비용) | FK robust cost (FK 비용) | FK cost fraction (FK 비용 비율) |
| --- | --- | ---: | ---: | ---: | ---: | ---: |
| A2 | estimated | 7754 | 0 / 0 | 23447.26 | 0.00 | 0.000% |
| A3 | FK-fixed (hard constant; residual 없음) | 7754 | 0 / 0 | 25555.84 | 0.00 | 0.000% |
| A4 | corrected-FK-factor | 7754 | 8 / 48 | 23453.98 | 31.30 | 0.133% |
| B1 | corrected-FK-factor | 6474 | 8 / 48 | 22768.13 | 35.24 | 0.155% |
| B2 | corrected-FK-factor | 1688 | 8 / 48 | 3857.04 | 30.78 | 0.792% |

> 이 비율은 최종 목적함수 값의 분해다. 각 항의 Jacobian과 변수 연결 구조가 다르므로, FK cost 비율을 파라미터 영향력 비율로 해석하면 안 된다.

## Camera-scope Evaluation (카메라 범위 평가)

### Fixed-to-Fixed (고정카메라 간)

| Method (방법) | Board Pixel (보드 px) | Board Translation (보드 이동 mm) | Board Rotation (보드 회전 deg) | Cube Pixel (큐브 px) | Cube Translation (큐브 이동 mm) | Cube Rotation (큐브 회전 deg) |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| A0 | 2.9152 | 3.4548 | 0.4114 | 11.7971 | 14.2795 | 0.8603 |
| A1 | 2.7157 | 3.0380 | 0.4424 | 12.7283 | 15.8616 | 0.8086 |
| A2 | 2.7072 | 3.3930 | 0.4565 | 12.0223 | 14.8496 | 0.8437 |
| A3 | 4.2937 | 5.8991 | 0.4728 | 9.6528 | 11.5639 | 0.8442 |
| A4 | 2.7596 | 3.4694 | 0.4432 | 11.9717 | 14.7545 | 0.8466 |
| B1 | 2.7364 | 3.0878 | 0.4239 | 12.6368 | 15.7124 | 0.8119 |
| B2 | 3.6756 | 5.2159 | 0.5417 | 11.1316 | 14.0157 | 1.0872 |
| B3 | 2.9169 | 3.4568 | 0.4116 | 11.7979 | 14.2793 | 0.8607 |

### Gripper-to-Fixed (그리퍼카메라–고정카메라 간)

| Method (방법) | Board Pixel (보드 px) | Board Translation (보드 이동 mm) | Board Rotation (보드 회전 deg) | Cube Pixel (큐브 px) | Cube Translation (큐브 이동 mm) | Cube Rotation (큐브 회전 deg) |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| A0 | 3.4670 | 3.6364 | 0.4078 | 7.2054 | 8.7543 | 1.1145 |
| A1 | 3.3601 | 3.7720 | 0.5957 | 8.0735 | 9.9106 | 0.9239 |
| A2 | 3.3336 | 3.6671 | 0.6126 | 7.4688 | 9.2350 | 0.9518 |
| A3 | 4.6270 | 4.7592 | 0.5014 | 5.8096 | 7.3903 | 1.0382 |
| A4 | 3.3564 | 3.6661 | 0.5975 | 7.4038 | 9.1691 | 0.9498 |
| B1 | 3.3594 | 3.7490 | 0.5814 | 7.9733 | 9.8097 | 0.9224 |
| B2 | 3.7794 | 3.9643 | 0.7630 | 7.4269 | 9.0752 | 1.0366 |
| B3 | 3.4679 | 3.6366 | 0.4076 | 7.2055 | 8.7542 | 1.1148 |

### Marker-system End-to-End (마커 시스템 전체 경로)

| System (시스템) | Own Held-out (자체 홀드아웃 px) | Fixed-to-Fixed Board/Cube (고정카메라 간 보드/큐브 px) | Gripper-to-Fixed Board/Cube (그리퍼카메라–고정카메라 간 보드/큐브 px) | Convergence (수렴) |
| --- | ---: | ---: | ---: | ---: |
| Board-only end-to-end | 2.4676 | 2.9151 / 11.7990 | 3.4669 / 7.2070 | 3/3 |
| Cube-only end-to-end | 4.2798 | 3.6096 / 11.3110 | 3.7603 / 7.5768 | 3/3 |
| Board+Cube end-to-end | 2.9715 | 2.7072 / 12.0223 | 3.3336 / 7.4688 | 3/3 |

## Calculation (계산 방식)

For Target $O\in\{board,cube\}$ (표적 $O$):

$$T^{B,(i)}_O=T^B_{C_i}T^{C_i}_{O,\mathrm{PnP}}$$

$$T^B_{C_g}(e)=T^B_G(e)T^G_{C_g}$$

$$T^{B,(g)}_O(e)=T^B_G(e)T^G_{C_g}T^{C_g}_{O,\mathrm{PnP}}$$

Pixel Transfer RMSE (픽셀 전달 평균제곱근오차)는 한 카메라의 측정 PnP 자세를 다른 카메라로 옮겨 실제 검출 코너와 비교한다. Translation/Rotation Consistency (이동/회전 일관성)는 두 경로로 얻은 $T^B_O$의 차이를 mm/deg로 계산한다.

## Interpretation Limit (해석 한계)

Fixed-to-Fixed는 모든 고정카메라에 함께 존재하는 Systematic Error (계통 오차)를 검출할 수 없다. Gripper-to-Fixed는 Hand–Eye Error (핸드–아이 오차)와 FK Error (순기구학 오차)를 분리할 수 없다. 따라서 두 범위는 함께 보고하되 External Absolute Accuracy (외부 절대 정확도)로 부르지 않는다.

## Terminology (용어 설명)

- **$T^B_{C_i}$, Base-to-Fixed-Camera Transform (베이스–고정카메라 변환)**: 고정카메라 외부 파라미터.
- **$T^G_{C_g}$, Hand–Eye Transform (핸드–아이 변환)**: 그리퍼에서 그리퍼카메라로의 변환.
- **$T^B_G(e)$, Robot FK Pose (이벤트별 로봇 순기구학 자세)**: 이벤트 $e$의 베이스–그리퍼 변환이며 평가 중 고정 입력이다.
- **PnP, Perspective-n-Point (3D–2D 자세 추정)**: 3D 표적점과 2D 영상점으로 카메라–표적 자세를 계산한다.
- **RMSE, Root Mean Squared Error (평균제곱근오차)**: 잔차 제곱 평균의 제곱근. px, mm, deg는 서로 합치지 않는다.
- **Reference-dependent Reprojection (기준 의존 재투영)**: 학습 표적 자세에 의존하는 보조 진단으로 External GT가 아니다.

## Required Next Experiment (다음 필수 실험)

Independent External GT (독립 외부 정답)가 확정되면 Blind Position Holdout (비공개 위치 홀드아웃)으로 Translation Error (이동 오차), Rotation Error (회전 오차), P95, Failure Rate (실패율)를 다시 계산한다. 그 전에는 내부 지표만 유지한다.
