# Session04 Table 1 Results (표 1 결과)

> Status: Pre-GT Internal Evaluation (외부 GT 전 내부 평가). 이 문서는 External GT (외부 정답)를 사용한 절대 정확도 순위를 제시하지 않는다.

## Evaluation Decision (평가 구성 결정)

- Fixed-to-Fixed (고정카메라 간)는 Robot FK (로봇 순기구학) 없이 고정카메라 부분만 평가한다.
- Gripper-to-Fixed (그리퍼카메라–고정카메라 간)는 실제 Board/Cube Image Corners (보드/큐브 영상 코너)를 사용하지만, 예측 경로에는 Robot FK와 Hand–Eye (핸드–아이 변환)가 포함된다.
- Board (보드)와 Cube (큐브)는 모두 촬영 원본에서 평가한다. 캘리브레이션에 사용한 마커 종류와 평가 표적 종류를 동일시하지 않는다.
- Reference-dependent Reprojection (기준 의존 재투영)은 Secondary Diagnostic (보조 진단)이며 방법 순위에 사용하지 않는다.

## Table 1 Optimization Results (표 1 최적화 결과)

| Method (방법) | Marker Set (마커 구성) | Optimization (최적화) | Cube Pose (큐브 자세 처리) | Train Overall (학습 전체 px) | Own Held-out Overall (자체 홀드아웃 전체 px) | Board/Cube Held-out (보드/큐브 홀드아웃 px) | Convergence (수렴) | Status (상태) |
| --- | --- | --- | --- | ---: | ---: | ---: | ---: | --- |
| A0 (baseline) | board | sequential_frozen_stage | — | 6.1832 | 5.8773 | 5.8773 / N/A | 3/3 | Complete (완료) |
| A1 (+cube) | cube+board | sequential_frozen_stage | estimated | 5.9224 | 6.3602 | 5.9153 / 7.7657 | 3/3 | Complete (완료) |
| A2 (+unified) | cube+board | unified_joint_optimization | estimated | 5.7846 | 6.2143 | 5.9229 / 7.1745 | 3/3 | Complete (완료) |
| A3 (Ours (full)) | cube+board | unified_joint_optimization | FK-fixed | 5.9805 | 5.4968 | 5.7250 / 4.5724 | 3/3 | Complete (완료) |
| A4 (Ours (corrected-FK factor)) | cube+board | unified_joint_optimization | corrected-FK-factor | 5.7846 | 6.1904 | 5.9128 / 7.1088 | 3/3 | Preflight — Simulation Prior (예비실험 — 시뮬레이션 사전값) |
| B1 (−Unified) | cube+board | sequential_frozen_stage | corrected-FK-factor | 5.9154 | 6.3309 | 5.9092 / 7.6710 | 3/3 | Preflight — Simulation Prior (예비실험 — 시뮬레이션 사전값) |
| B2 (−board) | cube | unified_joint_optimization | corrected-FK-factor | 3.7132 | 6.6975 | N/A / 6.6975 | 3/3 | Preflight — Simulation Prior (예비실험 — 시뮬레이션 사전값) |
| B3 (−cube) | board | unified_joint_optimization | — | 6.1832 | 5.8772 | 5.8772 / N/A | 3/3 | Complete (완료) |

## Objective Block Diagnostics (목적함수 블록 진단)

| Method (방법) | FK 처리 | Visual residual components (시각 잔차 수) | FK blocks / components (FK 블록/잔차 수) | Visual robust cost (시각 비용) | FK robust cost (FK 비용) | FK cost fraction (FK 비용 비율) |
| --- | --- | ---: | ---: | ---: | ---: | ---: |
| A2 | estimated | 10702 | 0 / 0 | 61765.45 | 0.00 | 0.000% |
| A3 | FK-fixed (hard constant; residual 없음) | 10702 | 0 / 0 | 65338.16 | 0.00 | 0.000% |
| A4 | corrected-FK-factor | 10702 | 13 / 78 | 61807.01 | 113.09 | 0.183% |
| B1 | corrected-FK-factor | 8406 | 13 / 78 | 59283.13 | 130.83 | 0.220% |
| B2 | corrected-FK-factor | 1992 | 13 / 78 | 5931.97 | 94.07 | 1.561% |

> 이 비율은 최종 목적함수 값의 분해다. 각 항의 Jacobian과 변수 연결 구조가 다르므로, FK cost 비율을 파라미터 영향력 비율로 해석하면 안 된다.

## Camera-scope Evaluation (카메라 범위 평가)

### Fixed-to-Fixed (고정카메라 간)

| Method (방법) | Board Pixel (보드 px) | Board Translation (보드 이동 mm) | Board Rotation (보드 회전 deg) | Cube Pixel (큐브 px) | Cube Translation (큐브 이동 mm) | Cube Rotation (큐브 회전 deg) |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| A0 | 0.8795 | 1.2764 | 0.1601 | 11.4709 | 14.8581 | 1.3588 |
| A1 | 1.9699 | 2.0179 | 0.4818 | 10.7893 | 13.8134 | 1.4674 |
| A2 | 1.6976 | 1.7903 | 0.5346 | 9.6100 | 12.3958 | 1.4572 |
| A3 | 3.4871 | 4.7325 | 0.3518 | 7.1163 | 9.4860 | 1.4009 |
| A4 | 1.6960 | 1.7696 | 0.5502 | 9.5791 | 12.3691 | 1.4708 |
| B1 | 2.0291 | 2.0975 | 0.4914 | 10.7375 | 13.7652 | 1.4663 |
| B2 | 3.7844 | 4.9628 | 0.6438 | 6.9340 | 8.6410 | 1.5589 |
| B3 | 0.8802 | 1.2771 | 0.1601 | 11.4711 | 14.8589 | 1.3589 |

### Gripper-to-Fixed (그리퍼카메라–고정카메라 간)

| Method (방법) | Board Pixel (보드 px) | Board Translation (보드 이동 mm) | Board Rotation (보드 회전 deg) | Cube Pixel (큐브 px) | Cube Translation (큐브 이동 mm) | Cube Rotation (큐브 회전 deg) |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| A0 | 6.8528 | 7.1155 | 0.5435 | 9.4778 | 9.9321 | 1.1293 |
| A1 | 7.1604 | 9.0046 | 1.1072 | 9.7098 | 10.2791 | 1.4699 |
| A2 | 7.1781 | 9.3471 | 1.2727 | 9.2498 | 9.6951 | 1.5611 |
| A3 | 7.3443 | 8.2255 | 0.6178 | 8.6882 | 8.5329 | 1.1331 |
| A4 | 7.1826 | 9.3124 | 1.2648 | 9.2592 | 9.7057 | 1.5522 |
| B1 | 7.1964 | 9.0548 | 1.1018 | 9.7038 | 10.2655 | 1.4628 |
| B2 | 7.3569 | 10.8418 | 1.6608 | 8.0188 | 8.5501 | 1.9163 |
| B3 | 6.8532 | 7.1168 | 0.5434 | 9.4786 | 9.9332 | 1.1289 |

### Marker-system End-to-End (마커 시스템 전체 경로)

| System (시스템) | Own Held-out (자체 홀드아웃 px) | Fixed-to-Fixed Board/Cube (고정카메라 간 보드/큐브 px) | Gripper-to-Fixed Board/Cube (그리퍼카메라–고정카메라 간 보드/큐브 px) | Convergence (수렴) |
| --- | ---: | ---: | ---: | ---: |
| Board-only end-to-end | 5.8773 | 0.8789 / 11.4695 | 6.8526 / 9.4762 | 3/3 |
| Cube-only end-to-end | 6.6602 | 3.4723 / 7.2645 | 7.3813 / 8.2717 | 3/3 |
| Board+Cube end-to-end | 6.2143 | 1.6976 / 9.6100 | 7.1781 / 9.2498 | 3/3 |

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
