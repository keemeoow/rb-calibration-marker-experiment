# Session04 Table 1 Results (표 1 결과)

> Status: Pre-GT Internal Evaluation (외부 GT 전 내부 평가). 이 문서는 External GT (외부 정답)를 사용한 절대 정확도 순위를 제시하지 않는다.

## Evaluation Decision (평가 구성 결정)

- Fixed-to-Fixed (고정카메라 간)는 Robot FK (로봇 순기구학) 없이 고정카메라 부분만 평가한다.
- Gripper-to-Fixed (그리퍼카메라–고정카메라 간)는 실제 Board/Cube Image Corners (보드/큐브 영상 코너)를 사용하지만, 예측 경로에는 Robot FK와 Hand–Eye (핸드–아이 변환)가 포함된다.
- 각 set의 최초 고정카메라 관측은 최적화에서 한 번만 사용한다. Gripper-to-Fixed 평가는 이 fixed anchor를 같은 set의 모든 held-out gripper Event와 연결하고, Event→set→set 동일가중 순서로 집계한다.
- 공식 결과는 물리 config의 nominal metric scale `1.0`만 사용한다. 데이터에서 추정한 scale은 별도 diagnostic 결과로만 허용한다.
- Board (보드)와 Cube (큐브)는 모두 촬영 원본에서 평가한다. 캘리브레이션에 사용한 마커 종류와 평가 표적 종류를 동일시하지 않는다.
- Reference-dependent Reprojection (기준 의존 재투영)은 Secondary Diagnostic (보조 진단)이며 방법 순위에 사용하지 않는다.

## Table 1 Optimization Results (표 1 최적화 결과)

| Method (방법) | Marker Set (마커 구성) | Optimization (최적화) | Cube Pose (큐브 자세 처리) | Train Overall (학습 전체 px) | Own Held-out Overall (자체 홀드아웃 전체 px) | Board/Cube Held-out (보드/큐브 홀드아웃 px) | Convergence (수렴) | Status (상태) |
| --- | --- | --- | --- | ---: | ---: | ---: | ---: | --- |
| A0 (baseline) | board | sequential_frozen_stage | — | 3.8202 | 4.0530 | 4.0530 / N/A | 3/3 | Complete (완료) |
| A1 (+cube) | cube+board | sequential_frozen_stage | estimated | 3.7844 | 4.0206 | 4.1007 / 3.7761 | 3/3 | Complete (완료) |
| A2 (+unified) | cube+board | unified_joint_optimization | estimated | 3.7513 | 3.9178 | 4.0415 / 3.5306 | 3/3 | Complete (완료) |
| A3 (Ours (full)) | cube+board | unified_joint_optimization | FK-fixed | 3.9474 | 3.8220 | 3.9644 / 3.3702 | 3/3 | Complete (완료) |
| A4 (Ours (corrected-FK factor)) | cube+board | unified_joint_optimization | corrected-FK-factor | 3.7532 | 3.9193 | 4.0481 / 3.5151 | 3/3 | Preflight — Simulation Prior (예비실험 — 시뮬레이션 사전값) |
| B1 (−Unified) | cube+board | sequential_frozen_stage | corrected-FK-factor | 3.7851 | 4.0214 | 4.1047 / 3.7669 | 3/3 | Preflight — Simulation Prior (예비실험 — 시뮬레이션 사전값) |
| B2 (−board) | cube | unified_joint_optimization | corrected-FK-factor | 3.0882 | 4.5964 | N/A / 4.5964 | 3/3 | Preflight — Simulation Prior (예비실험 — 시뮬레이션 사전값) |
| B3 (−cube) | board | unified_joint_optimization | — | 3.8202 | 4.0530 | 4.0530 / N/A | 3/3 | Complete (완료) |

## Objective Block Diagnostics (목적함수 블록 진단)

| Method (방법) | FK 처리 | Visual residual components (시각 잔차 수) | FK blocks / components (FK 블록/잔차 수) | Visual robust cost (시각 비용) | FK robust cost (FK 비용) | FK cost fraction (FK 비용 비율) |
| --- | --- | ---: | ---: | ---: | ---: | ---: |
| A2 | estimated | 7404 | 0 / 0 | 24246.06 | 0.00 | 0.000% |
| A3 | FK-fixed (hard constant; residual 없음) | 7404 | 0 / 0 | 25913.29 | 0.00 | 0.000% |
| A4 | corrected-FK-factor | 7404 | 9 / 54 | 24257.93 | 30.94 | 0.127% |
| B1 | corrected-FK-factor | 6068 | 9 / 54 | 23566.46 | 34.83 | 0.148% |
| B2 | corrected-FK-factor | 1480 | 9 / 54 | 3593.24 | 45.56 | 1.252% |

> 이 비율은 최종 목적함수 값의 분해다. 각 항의 Jacobian과 변수 연결 구조가 다르므로, FK cost 비율을 파라미터 영향력 비율로 해석하면 안 된다.

## Camera-scope Evaluation (카메라 범위 평가)

### Fixed-to-Fixed (고정카메라 간)

| Method (방법) | Board Pixel (보드 px) | Board Translation (보드 이동 mm) | Board Rotation (보드 회전 deg) | Cube Pixel (큐브 px) | Cube Translation (큐브 이동 mm) | Cube Rotation (큐브 회전 deg) |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| A0 | 1.1851 | 1.7187 | 0.1926 | 12.6532 | 16.9975 | 1.7553 |
| A1 | 3.2635 | 4.4046 | 0.4975 | 10.9188 | 14.9617 | 1.6957 |
| A2 | 3.5459 | 4.9728 | 0.5134 | 9.5463 | 12.6073 | 1.7252 |
| A3 | 5.4698 | 7.9675 | 0.4229 | 7.5131 | 9.7526 | 1.7893 |
| A4 | 3.6300 | 5.0622 | 0.4979 | 9.5487 | 12.5754 | 1.7234 |
| B1 | 3.3373 | 4.5221 | 0.5101 | 10.8716 | 14.8919 | 1.7044 |
| B2 | 5.7922 | 8.4244 | 0.5009 | 7.0953 | 9.1767 | 1.8041 |
| B3 | 1.1835 | 1.7171 | 0.1931 | 12.6500 | 16.9932 | 1.7553 |

### Gripper-to-Fixed (그리퍼카메라–고정카메라 간)

| Method (방법) | Board Pixel (보드 px) | Board Translation (보드 이동 mm) | Board Rotation (보드 회전 deg) | Cube Pixel (큐브 px) | Cube Translation (큐브 이동 mm) | Cube Rotation (큐브 회전 deg) |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| A0 | 4.5444 | 5.4810 | 0.7966 | 10.2456 | 11.2086 | 1.5669 |
| A1 | 5.0193 | 6.4169 | 0.9736 | 9.6380 | 10.7764 | 1.6304 |
| A2 | 5.4411 | 7.2118 | 1.0295 | 8.9883 | 9.8188 | 1.6354 |
| A3 | 6.0437 | 7.3039 | 0.7528 | 7.9345 | 8.6983 | 1.4928 |
| A4 | 5.4698 | 7.2275 | 1.0129 | 8.9831 | 9.8212 | 1.6275 |
| B1 | 5.0473 | 6.4239 | 0.9741 | 9.6086 | 10.7567 | 1.6315 |
| B2 | 6.1255 | 8.6949 | 1.0881 | 8.5233 | 9.1899 | 1.6669 |
| B3 | 4.5446 | 5.4824 | 0.7967 | 10.2444 | 11.2069 | 1.5670 |

### Marker-system End-to-End (마커 시스템 전체 경로)

| System (시스템) | Own Held-out (자체 홀드아웃 px) | Fixed-to-Fixed Board/Cube (고정카메라 간 보드/큐브 px) | Gripper-to-Fixed Board/Cube (그리퍼카메라–고정카메라 간 보드/큐브 px) | Convergence (수렴) |
| --- | ---: | ---: | ---: | ---: |
| Board-only end-to-end | 4.0530 | 1.1827 / 12.6506 | 4.5448 / 10.2436 | 3/3 |
| Cube-only end-to-end | 4.7212 | 5.5635 / 7.1530 | 6.0188 / 8.7492 | 3/3 |
| Board+Cube end-to-end | 3.9178 | 3.5459 / 9.5463 | 5.4411 / 8.9883 | 3/3 |

## Calculation (계산 방식)

For Target $O\in\{board,cube\}$ (표적 $O$):

$$T^{B,(i)}_O=T^B_{C_i}T^{C_i}_{O,\mathrm{PnP}}$$

$$T^B_{C_g}(e)=T^B_G(e)T^G_{C_g}$$

$$T^{B,(g)}_O(e)=T^B_G(e)T^G_{C_g}T^{C_g}_{O,\mathrm{PnP}}$$

Pixel Transfer RMSE (픽셀 전달 평균제곱근오차)는 한 카메라의 측정 PnP 자세를 다른 카메라로 옮겨 실제 검출 코너와 비교한다. Translation/Rotation Consistency (이동/회전 일관성)는 두 경로로 얻은 $T^B_O$의 차이를 mm/deg로 계산한다. Gripper-to-Fixed의 최종값은 pair 성분을 Event RMSE로, Event를 set RMSE로 집계한 뒤 set별 동일 가중치로 계산한다.

## Interpretation Limit (해석 한계)

Fixed-to-Fixed는 모든 고정카메라에 함께 존재하는 Systematic Error (계통 오차)를 검출할 수 없다. Gripper-to-Fixed는 Hand–Eye Error (핸드–아이 오차)와 FK Error (순기구학 오차)를 분리할 수 없다. 따라서 두 범위는 함께 보고하되 External Absolute Accuracy (외부 절대 정확도)로 부르지 않는다.

## Terminology (용어 설명)

- **$T^B_{C_i}$, Base-to-Fixed-Camera Transform (베이스–고정카메라 변환)**: 고정카메라 외부 파라미터.
- **$T^G_{C_g}$, Hand–Eye Transform (핸드–아이 변환)**: 그리퍼에서 그리퍼카메라로의 변환.
- **$T^B_G(e)$, Robot FK Pose (이벤트별 로봇 순기구학 자세)**: 이벤트 $e$의 베이스–그리퍼 변환이며 평가 중 고정 입력이다.
- **PnP, Perspective-n-Point (3D–2D 자세 추정)**: 3D 표적점과 2D 영상점으로 카메라–표적 자세를 계산한다.
- **RMSE, Root Mean Squared Error (평균제곱근오차)**: 잔차 제곱 평균의 제곱근. px, mm, deg는 서로 합치지 않는다.
- **Reference-dependent Reprojection (기준 의존 재투영)**: 학습 표적 자세에 의존하는 보조 진단으로 External GT가 아니다.
