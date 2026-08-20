# 다중 고정카메라·로봇 Hand–Eye 캘리브레이션 비교실험

이 문서는 실제 데이터 캡처부터 A0~A5/B1~B3 비교실험, 공통 평가지표, 결과 파일 생성까지의 메인 설명서다. 구현의 기준은 `Run_calibration_comparison.py`와 `calibration_pipeline/`이며, 현재 최종 추정기는 모든 실행 조건에서 동일한 **raw-corner pixel reprojection 최적화**를 사용한다.

## 바로가기

- [1. 가장 짧은 실행 순서](#1-가장-짧은-실행-순서)
- [2. 시스템과 좌표계](#2-시스템과-좌표계)
- [3. 입력 데이터](#3-입력-데이터)
- [4. 전체 파이프라인](#4-전체-파이프라인)
- [5. PnP와 초기화의 역할](#5-pnp와-초기화의-역할)
- [6. 최종 재투영 최적화 원리](#6-최종-재투영-최적화-원리)
- [7. 독립과 통합 최적화의 차이](#7-독립과-통합-최적화의-차이)
- [8. A0~A5/B1~B3 비교실험](#8-a0a5b1b3-비교실험)
- [9. 마커 시스템 End-to-End 비교](#9-마커-시스템-end-to-end-비교)
- [10. 공통 Cross-target 평가](#10-공통-cross-target-평가)
- [11. 평가지표 정의와 적용 범위](#11-평가지표-정의와-적용-범위)
- [12. 어떤 조건끼리 비교해야 하는가](#12-어떤-조건끼리-비교해야-하는가)
- [13. 현재 session02 결과와 비교별 해석](#13-현재-session02-결과와-비교별-해석)
- [14. 출력 파일과 시각화 데이터 흐름](#14-출력-파일과-시각화-데이터-흐름)
- [15. 코드 구조](#15-코드-구조)
- [16. 전체 실행 명령](#16-전체-실행-명령)
- [17. 현재 해석 한계](#17-현재-해석-한계)

## 1. 가장 짧은 실행 순서

```bash
# 1. RealSense factory intrinsics 저장
python3 Step1_dump_all_intrinsics.py --out_dir intrinsics

# 1b. ChArUco 기반 intrinsics 재보정
python3 Step1b_charuco_intrinsics.py \
  --intr_dir intrinsics \
  --save_images

# 2. cube/board 영상, depth, robot FK 동시 캡처
python3 Step2_capture.py \
  --data_root data \
  --intrinsics_dir intrinsics \
  --use_robot \
  --robot_ip 192.168.0.23 \
  --robot_port 12348 \
  --show

# 3. A/B 비교실험 실행
python3 Run_calibration_comparison.py table1 \
  --root_folder data/sessionNN/calib_train \
  --intrinsics_dir intrinsics \
  --include_sets 5-12 \
  --split_seed 20260731 \
  --out_dir CP_result/sessionNN/late_table1
```

`Step3_calibration.py`는 별도 최종 solver가 아니다. `table1`과 같은 코드를 이용해 공통 train-only baseline만 먼저 생성·검증하고 싶을 때 사용한다. 따라서 Step3를 생략하고 `table1`을 바로 실행해도 계산 경로는 동일하다.

## 2. 시스템과 좌표계

| 기호 | 의미 | 최적화 여부 |
| --- | --- | --- |
| (T^B_{C_i}) | 고정카메라 (i)에서 robot base로의 외부 파라미터 | 조건에 따라 자유변수 |
| (T^G_C) | gripper에서 eye-in-hand 카메라로의 Hand–Eye transform | 조건에 따라 자유변수 |
| (T^B_G(e)) | event (e)의 robot FK가 제공한 base–gripper transform | 모든 조건에서 고정 입력 |
| (T^B_{mathrm{board}}) | 작업공간에 고정된 ChArUco board pose | board 조건에서 자유변수 |
| (T^B_{mathrm{cube}}(s)) | 배치 set (s)의 cube pose | vision 추정, FK 고정 또는 FK factor |
| (K_i,D_i) | 카메라 (i)의 intrinsic과 distortion | 사전 보정 후 항상 고정 |

중요한 용어 규칙은 다음과 같다.

- `vision` 또는 `no-FK`는 **robot FK 전체를 사용하지 않는다는 뜻이 아니다.** Eye-in-hand 카메라 pose (T^B_G(e)T^G_C)를 만들기 위해 robot FK는 모든 조건에서 사용한다.
- `vision`, `FK-fixed`, `corrected-FK`의 차이는 배치된 cube pose (T^B_{mathrm{cube}}(s))를 어떤 방식으로 다루는지에 대한 차이다.
- board는 robot에 부착된 물체가 아니므로 `FK-fixed board`라는 조건은 물리적으로 정의하지 않는다.

## 3. 입력 데이터

### 3.1 Step1/1b의 입력과 출력

| 단계 | 입력 | 출력 |
| --- | --- | --- |
| Step1 | 연결된 RealSense 장치 | `intrinsics/cam*.npz`의 factory (K,D), depth scale |
| Step1b | ChArUco board 영상 | 재보정된 `cam*.npz`, 기존 factory 값 backup |

비교실험에서는 이 (K,D)를 다시 최적화하지 않는다. 조건 간 intrinsic 자유도가 달라지는 것을 방지하기 위해 모든 행에서 상수로 고정한다.

### 3.2 Step2의 입력과 출력

Step2는 다음 정보를 같은 capture event 단위로 저장한다.

- 고정카메라와 eye-in-hand 카메라의 RGB/depth 영상
- cube AprilTag와 ChArUco board 검출 provenance
- `event_id`, `set_index`, `gripper_cam_idx`
- robot pose (T^B_G(e)) 또는 대응 6-DoF FK
- cube를 파지한 경우 `cube_gripped`, `grasp_id`
- cube geometry/configuration과 카메라별 저장 성공 여부

비교실험의 필수 입력은 다음과 같다.

```text
data/sessionNN/calib_train/
├── meta.json
└── ... RGB/depth images

intrinsics/
└── cam*.npz
```

## 4. 전체 파이프라인

```text
Step1/1b: K,D 고정
        │
Step2: RGB/depth + robot FK + meta.json
        │
        ▼
cube/board raw corner 검출
        │
        ├── PnP + Hand–Eye + robust SE(3) 평균 ── 초기값만 생성
        │
        ├── event-grouped/set-stratified train/test split
        │
        ├── board-free train-only FK–cube alignment artifact
        │
        └── 모든 Table 1 행의 shared train-only baseline
                    │
                    ▼
       A0~A4/B1~B3 raw-corner reprojection 최적화
       A5는 실측 correction label 전까지 baseline만 예약
                    │
                    ├── row-local train/test reprojection px
                    ├── frozen common-mask e_cross / pixel transfer / e_e2e
                    └── transforms + solver diagnostics
                    │
                    ▼
       cross-target / marker-system / external-GT 평가
                    │
                    ▼
       JSON → CSV → Markdown/interactive HTML 검증
```

### 4.1 관측 데이터 표현

최적화의 한 관측은 `PixelObs`로 표현된다.

- marker 종류: `board` 또는 `cube`
- camera와 event ID
- cube의 경우 set ID
- target 좌표계의 3D corner (mathbf X_n)
- native distorted image의 실제 2D corner (mathbf u_n)

board와 cube 모두 최종 solver에는 PnP pose가 아니라 **3D corner–2D pixel 대응점**으로 들어간다.

### 4.2 Train/Test 분리

기본 primary split은 `event_grouped_and_set_stratified`다.

- 같은 event의 모든 카메라 관측은 train 또는 test 한쪽에만 들어간다.
- 각 cube set에 train과 test event가 모두 남도록 나눈다.
- set별 eye-in-hand cube train event 수가 최소 기준보다 작아지지 않게 한다.
- test event는 baseline, FK alignment, 초기값 생성에 사용하지 않는다.

이 split은 새로운 cube 위치 전체를 숨기는 position holdout이 아니다. 위치 전체 holdout에서 자유 cube pose를 test-time fitting 없이 평가할 수 없기 때문에, primary pixel 평가는 같은 set 안의 held-out event를 사용한다. 새로운 위치에 대한 물리 정확도는 별도 blind external-GT 실험으로 평가한다.

## 5. PnP와 초기화의 역할

PnP는 최종 결과를 직접 결정하는 backend가 아니라 초기값과 독립 consistency 평가를 만드는 도구다.

### 5.1 관측별 PnP

측정 corner만으로 (T^{C_i}_{O})를 계산한다.

- planar board: `SOLVEPNP_IPPE`
- non-planar cube: `SOLVEPNP_ITERATIVE`
- 최소 4개 3D–2D 대응점 필요
- 최종 Table 1 solver에서는 PnP pose residual을 쓰지 않고 raw pixel corner residual을 쓴다.

### 5.2 Board 기반 Hand–Eye 초기화

Eye-in-hand board PnP와 robot FK를 이용해 다음 OpenCV Hand–Eye 후보를 모두 계산한다.

- Tsai
- Park
- Horaud
- Daniilidis

각 후보로 여러 event의 board base pose를 만들고 robust SE(3) 평균 및 pose dispersion을 계산한다. 가장 일관된 후보가 (T^G_C)와 (T^B_{mathrm{board}}) 초기값으로 선택된다.

### 5.3 Cube pose와 고정카메라 초기화

초기 Hand–Eye와 eye-in-hand cube PnP를 조합해 set별 visual cube pose를 만든 뒤 MAD 기반 robust SE(3) 평균을 적용한다.

고정카메라 초기값은 다음 변환 체인에서 얻는다.

\[
T^B_{C_i}=T^B_O\left(T^{C_i}_O\right)^{-1}
\]

board와 cube에서 얻은 후보를 카메라별로 robust SE(3) 평균한다. 이 값들은 모든 Table 1 조건의 공통 출발점일 뿐 최종 결과가 아니다.

### 5.4 Board-free FK–cube alignment

A3/A4/B1/B2가 공유하는 FK cube artifact는 다음 관계를 train eye-in-hand cube corner만으로 정렬한다.

\[
T^B_G(e)T^G_C T^C_O(e)
=T^B_{\mathrm{FK,cube,raw}}(s)T^{\mathrm{FK,cube}}_O
\]

board, 고정카메라 관측, held-out event는 사용하지 않는다. PnP/Hand–Eye 또는 global (AX=ZB)는 초기화에만 쓰고, artifact의 최종 정렬은 eye-in-hand cube raw-corner 재투영으로 다시 최적화한다.

## 6. 최종 재투영 최적화 원리

### 6.1 고정카메라 관측

고정카메라 (i)가 target (O)를 본 경우 예측 transform은 다음과 같다.

\[
\hat T^{C_i}_{O}=\left(T^B_{C_i}\right)^{-1}T^B_O
\]

### 6.2 Eye-in-hand 관측

event (e)의 eye-in-hand 카메라 pose는 robot FK와 Hand–Eye로 결정된다.

\[
T^B_C(e)=T^B_G(e)T^G_C
\]

따라서 target 예측은 다음과 같다.

\[
\hat T^C_O(e)=\left(T^B_G(e)T^G_C\right)^{-1}T^B_O
\]

### 6.3 Pixel residual

카메라별로 고정된 (K_i,D_i)를 이용해 3D corner를 distorted image에 투영한다.

\[
\hat{\mathbf u}_{i,e,n}
=\pi\!\left(K_i,D_i,\hat T^{C_i}_{O}(e)\mathbf X_n\right)
\]

\[
\mathbf r_{i,e,n}=\hat{\mathbf u}_{i,e,n}-\mathbf u_{i,e,n}
\]

모든 선택된 board/cube, eih/e2h corner residual을 하나의 벡터로 쌓아 최적화한다. 기본 solver는 다음과 같다.

- SciPy trust-region reflective `least_squares(method="trf")`
- `soft_l1` robust loss
- `f_scale=2 px`
- `x_scale="jac"`
- SE(3) left-local perturbation과 retraction
- 기본 `max_nfev=300`, `xtol=ftol=gtol=1e-8`

최종 단계에는 PnP 평균, pose-consistency 후처리 또는 Ridge 보정이 없다. RANSAC으로 결과가 나쁜 관측을 사후 제거하지도 않는다. 검출 품질과 PnP-validity 기반 공통 mask는 모델 fitting 전에 고정하고, 최적화 중 이상치 영향은 공통 `soft_l1` loss로 제한한다.

## 7. 독립과 통합 최적화의 차이

### 7.1 독립/순차 방식 `seq`

1단계는 eye-in-hand 관측만 사용한다.

\[
\min_{T^G_C,T^B_O}
\sum_{(e,n)\in\mathrm{eih}}\sum_{k\in\{u,v\}}
\rho\!\left(r_{e,n,k}\right)
\]

2단계에서는 1단계의 Hand–Eye와 target pose를 완전히 고정하고 고정카메라만 푼다.

\[
\min_{T^B_{C_1},\ldots,T^B_{C_m}}
\sum_{i=1}^{m}\sum_{(e,n)\in\mathrm{e2h}_i}\sum_{k\in\{u,v\}}
\rho\!\left(r_{i,e,n,k}\right)
\]

target과 Hand–Eye가 고정되어 카메라별 residual block은 수학적으로 분리된다. e2h 결과가 eih 변수로 되돌아가는 alternating pass는 없다.

### 7.2 통합 방식 `U`

eih와 e2h residual을 하나의 문제에 넣고 모든 선언된 자유변수를 동시에 갱신한다.

\[
\min_{\{T^B_{C_i}\},T^G_C,\{T^B_O\}}
\left[
\sum_{(e,n)\in\mathrm{eih}}\sum_{k\in\{u,v\}}\rho(r_{e,n,k})
+\sum_i\sum_{(e,n)\in\mathrm{e2h}_i}\sum_{k\in\{u,v\}}\rho(r_{i,e,n,k})
\right]
\]

공유 target pose와 Hand–Eye가 두 관측 경로를 연결하므로 고정카메라 관측이 Hand–Eye/target 추정에 피드백된다. `e_cross`는 이 목적함수의 항이 아니라 최적화 후 독립적으로 계산하는 평가지표다.

## 8. A0~A5/B1~B3 비교실험

모든 실행 가능한 행은 동일한 train/test split, (K,D), raw detection, solver option, seed와 train-only reference state에서 시작한다. 행별로 바뀌는 것은 marker residual, cube pose 처리, 자유변수 freeze mask뿐이다.

| ID | 입력 marker | 최적화 | Cube pose 처리 | 자유변수 | 핵심 출력/질문 |
| --- | --- | --- | --- | --- | --- |
| A0 | board | 순차 `seq` | cube 없음 | 1단계 (T^G_C,T^B_{board}), 2단계 (T^B_{C_i}) | board-only optimization baseline |
| A1 | board+cube | 순차 `seq` | vision 자유변수 | 1단계 (T^G_C,T^B_{board},T^B_{cube}(s)), 2단계 (T^B_{C_i}) | 같은 순차법에서 cube residual 추가 효과 |
| A2 | board+cube | 통합 `U` | vision 자유변수 | (T^B_{C_i},T^G_C,T^B_{board},T^B_{cube}(s)) | vision-only 통합 효과 |
| A3 | board+cube | 통합 `U` | board-free aligned FK pose로 hard fixed | (T^B_{C_i},T^G_C,T^B_{board}) | cube를 정확한 상수로 둘 때의 효과 |
| A4 | board+cube | 통합 `U` | covariance-whitened soft FK factor | A2와 같음 | vision과 FK 불확실성을 함께 쓰는 효과 |
| A5 | 미정 | 미실행 | 독립 실측 6-DoF correction label 필요 | 아직 정의하지 않음 | A4와 동일 baseline만 예약, 현재 `not_run` |
| B1 | board+cube | 순차 `seq` | A4와 동일 soft FK factor | 1단계 Hand–Eye/board/cube, 2단계 camera별 | 같은 FK factor에서 통합 효과 검증 |
| B2 | cube | 통합 `U` | A4와 동일 soft FK factor | (T^B_{C_i},T^G_C,T^B_{cube}(s)) | 같은 FK factor에서 board residual 제거 효과 |
| B3 | board | 통합 `U` | cube 없음 | (T^B_{C_i},T^G_C,T^B_{board}) | vision 통합 조건에서 cube residual 제거 효과 |

### 8.1 A0 — Board·순차 baseline

- 입력: board corner, (K,D), robot FK
- 초기화: board PnP → 여러 Hand–Eye 후보 → robust SE(3) 선택
- 최적화: eih board로 Hand–Eye/board를 푼 후 고정, e2h board로 고정카메라만 계산
- 출력: board 기반 camera/Hand–Eye transform, train/test board px
- 제한: cube가 없으므로 row-local `e_cross`, pixel transfer, `e_e2e`는 N/A

### 8.2 A1 — Cube 추가·순차

- 입력: A0 입력 + cube corner
- 초기화: A0와 동일 shared baseline
- 최적화: eih 단계에서 board와 set별 cube pose까지 추정한 뒤 고정카메라 단계로 진행
- 비교 목적: A0 대비 최적화 목적함수에 cube residual을 추가한 효과

### 8.3 A2 — Vision-only 통합

- 입력: A1과 동일
- 최적화: eih/e2h, 고정카메라, Hand–Eye, board/cube pose를 하나의 raw-corner 문제에서 동시 계산
- 비교 목적: A1 대비 관측 종류나 FK 처리는 그대로 두고 통합 feedback만 추가한 효과

### 8.4 A3 — FK-fixed 통합

- 입력: A2 입력 + board-free train-only aligned FK cube artifact
- 최적화: cube pose는 변수 목록에서 제거하여 완전히 고정하고 나머지만 통합 최적화
- 비교 목적: A2 대비 cube pose를 vision으로 추정하는 대신 FK 상수로 두는 효과
- 주의: FK가 실제로 정확하지 않다면 hard constraint가 결과를 편향시킬 수 있으므로 실측 FK 검증이 필요

### 8.5 A4 — Corrected-FK factor 통합

- 입력: A2 입력 + aligned FK target + set별 6×6 FK covariance
- 최적화: cube pose는 자유변수로 유지하면서 visual residual에 FK factor를 추가

\[
\mathbf r_{\mathrm{FK},s}
=L_s^{-1}
\begin{bmatrix}
\operatorname{rotvec}\!\left((T^B_{cube}(s))^{-1}\bar T^B_{cube,FK}(s)\right)\\
\operatorname{trans}\!\left((T^B_{cube}(s))^{-1}\bar T^B_{cube,FK}(s)\right)
\end{bmatrix},
\quad \Sigma_s=L_sL_s^T
\]

FK factor에는 Huber loss를 적용한다. `--fk_covariance_json`이 없으면 고정 Simulation prior를 사용하는 preflight이며 confirmatory 결과로 해석하지 않는다.

### 8.6 A5 — 독립 correction label 대기

A5는 이름만 있는 임의 solver가 아니다. 독립 실측으로 얻은 6-DoF correction label과 적용 규칙이 정해지기 전에는 실행하지 않는다. 현재 baseline artifact에는 A4와 byte-identical한 initial state만 넣어 향후 비교에서 초기값 차이가 생기지 않게 한다.

### 8.7 B1/B2/B3 — 원인 분리용 ablation

- B1↔A4: marker와 FK factor는 같고 순차/통합만 다르다.
- B2↔A4: cube와 FK factor는 같고 board residual 유무만 다르다.
- B3↔A2: board 기반 통합이라는 공통 조건에서 cube residual 유무를 본다.

B2/B3는 all-marker shared initializer에서 시작한 뒤 residual과 변수를 제거한다. 따라서 “처음부터 cube-only/board-only 시스템을 구축했을 때의 성능”이 아니라 **동일 초기조건에서 최적화 항의 기여도**를 측정한다.

## 9. 마커 시스템 End-to-End 비교

실제 마커 구성 자체를 비교하려면 다음 별도 실험을 사용한다.

```bash
python3 Run_calibration_comparison.py marker-system \
  --root_folder data/sessionNN/calib_train \
  --intrinsics_dir intrinsics \
  --include_sets 5-12 \
  --out_dir CP_result/sessionNN/marker_system_end_to_end
```

| 시스템 | 초기화에 허용되는 marker | 최종 목적함수 marker | 최종 자유변수 |
| --- | --- | --- | --- |
| `board_only` | board만 | board만 | cameras, Hand–Eye, board pose |
| `cube_only` | cube만 | cube만 | cameras, Hand–Eye, cube poses |
| `board_cube` | board+cube | board+cube | cameras, Hand–Eye, board/cube poses |

세 시스템은 split, raw detection, (K,D), solver, seed와 held-out 평가 population을 공유하지만 초기값은 modality별로 별도 생성한다. cube-only는 board-free train-only FK artifact로 Hand–Eye를 초기화하지만, 최종 목적함수에는 FK factor가 없다.

출력은 각 시스템의 own-marker held-out px와 모든 시스템에 공통인 board+cube target px, `e_cross`, pixel transfer, `e_e2e`다. 외부 GT가 아니므로 절대 정확도 주장은 할 수 없다.

## 10. 공통 Cross-target 평가

Table 1의 A0/B3처럼 목적함수에 cube가 없는 행과 cube-bearing 행은 row-local overall residual population이 다르다. 이를 보완하기 위해 모든 저장된 transform을 같은 held-out fixed-camera board+cube corner에서 다시 평가한다.

```bash
python3 Run_calibration_comparison.py cross-target \
  --root_folder data/sessionNN/calib_train \
  --intrinsics_dir intrinsics \
  --include_sets 5-12 \
  --table1_result CP_result/sessionNN/late_table1/table1_methods.json \
  --out_dir CP_result/sessionNN/cross_target_evaluation
```

평가 중 calibration transform은 다시 최적화하지 않는다. board pose는 train-only eih-board 초기값, cube pose는 train-only board-free FK artifact를 모든 방법에 공통 내부 reference로 사용한다. 따라서 조건 간 비교 population은 같지만 외부 물리 GT는 아니다.

## 11. 평가지표 정의와 적용 범위

### 11.1 Train reprojection RMSE px

\[
e_{\mathrm{train}}^{px}
=\sqrt{\frac{1}{2N}\sum_{n=1}^{N}
\left((\hat u_n-u_n)^2+(\hat v_n-v_n)^2\right)}
\]

- solver가 사용한 train corner에서 계산
- 최적화가 제대로 수렴했는지 보는 diagnostic
- 일반화 성능이나 절대 정확도의 주 지표가 아님

### 11.2 Held-out reprojection RMSE px — Primary metric

같은 식을 held-out event corner에 적용한다. 모든 transform은 train 결과로 동결하며 test-time refit이나 결과 의존적 관측 제거가 없다.

- 같은 marker population을 가진 조건끼리는 직접 비교 가능
- target 구성이 다른 조건의 pooled `overall`은 직접 비교하면 안 됨
- 재투영 오차는 px만 보고하며 예측 깊이를 곱한 임의의 mm 환산값은 사용하지 않음

### 11.3 `e_cross` — 고정카메라 간 cube pose consistency

각 고정카메라가 자기 영상 corner만으로 cube PnP를 독립 계산한다.

\[
T^{B,(i)}_{cube}=T^B_{C_i}T^{C_i}_{cube,\mathrm{PnP}}
\]

카메라 pair (i,j)에 대해 다음을 계산하고 모든 사전 고정 pair에서 RMSE를 낸다.

\[
e^{t}_{ij}=\|\mathbf t_i-\mathbf t_j\|_2\,[\mathrm{mm}]
\]

\[
e^{R}_{ij}=\left\|\log\left(R_i^TR_j\right)\right\|_2\,[\mathrm{deg}]
\]

이 mm는 재투영 오차를 mm로 변환한 값이 아니라 두 카메라가 예측한 cube 중심 사이의 실제 좌표 거리다. robot FK, gripper camera, nominal cube 정답, 외부 GT는 사용하지 않는다. 따라서 일반적인 고정카메라 간 위치/회전 일관성 지표이지만 절대 정확도는 아니다.

### 11.4 Cross-view pixel transfer RMSE px

카메라 A의 측정 corner로 얻은 cube PnP pose를 추정된 카메라 상대관계로 B에 옮긴 뒤 B의 실제 측정 corner와 비교한다.

\[
T^{C_B}_{cube}
=\left(T^B_{C_B}\right)^{-1}T^B_{C_A}T^{C_A}_{cube,\mathrm{PnP}}
\]

A→B와 B→A를 모두 계산한다. FK나 공유 cube 정답 없이 카메라 간 상대 보정을 px 도메인에서 평가하는 지표다.

### 11.5 `e_e2e` — Eye-in-hand와 Eye-to-hand 경로 일치

같은 held-out cube에 대해 다음 두 경로를 비교한다.

- 고정카메라 경로: 여러 (T^B_{C_i}T^{C_i}_{cube,\mathrm{PnP}})의 robust SE(3) 평균
- 손목카메라 경로: (T^B_G(e)T^G_C T^C_{cube,\mathrm{PnP}})

translation mm와 SO(3) rotation deg RMSE를 보고한다. robot FK가 포함되므로 전체 변환 체인의 내부 일관성 지표이며 외부 GT 정확도는 아니다.

### 11.6 수렴·등록·안정성 지표

- `converged_runs / total_runs`: 여러 초기 seed 중 solver 성공 수
- `n_registered_fixed_cameras`: 공통 조건을 만족하며 등록된 고정카메라 수
- transform dispersion: seed 간 translation/rotation 표준편차와 최댓값
- Jacobian rank/condition number: 관측가능성과 수치 안정성 diagnostic

### 11.7 External-GT 절대 정확도

물리 정확도는 calibration과 분리된 blind 데이터에서만 평가한다.

1. `blind-predict`: GT를 읽지 않고 각 방법의 (T^B_{cube}) 예측 저장
2. GT 잠금 해제
3. `external-gt`: TRE mm, rotation deg, P95, failure rate 등을 paired hierarchical bootstrap으로 비교

내부 `e_cross`/`e_e2e`가 좋아도 external-GT 오차가 반드시 작다는 뜻은 아니다.

### 11.8 조건별 지표 적용표

| 지표 | A0/B3 board-only row-local | Cube-bearing A1~A4/B1/B2 | Cross-target 전체 행 | Marker-system | 해석 |
| --- | --- | --- | --- | --- | --- |
| Train reprojection px | board만 | 선언된 marker | 공통 평가 아님 | 각 시스템 marker | fitting diagnostic |
| Held-out reprojection px | board만 | 선언된 marker | 공통 board+cube | own/common 모두 | primary internal pixel metric |
| `e_cross` mm/deg | N/A | 가능 | 전체 행 가능 | 가능 | 고정카메라 간 cube pose 일관성 |
| Pixel transfer px | N/A | 가능 | 전체 행 가능 | 가능 | 카메라 상대관계의 px 검증 |
| `e_e2e` mm/deg | N/A | 가능 | 전체 행 가능 | 가능 | FK 포함 전체 경로 일관성 |
| External TRE/rotation | blind GT 필요 | blind GT 필요 | 해당 없음 | 해당 없음 | 절대 물리 정확도 |

## 12. 어떤 조건끼리 비교해야 하는가

| 연구 질문 | 비교 | 공통으로 볼 지표 | 피해야 할 해석 |
| --- | --- | --- | --- |
| 순차법에 cube residual이 도움이 되는가 | A0 → A1 | held-out board px, 등록 수 | 서로 다른 pooled overall 직접 비교 |
| vision 조건에서 통합 feedback이 도움이 되는가 | A1 → A2 | held-out overall/board/cube px | 초기값 차이로 설명 |
| soft FK 조건에서도 통합 효과가 있는가 | B1 → A4 | held-out overall/board/cube, 공통 path | 서로 다른 FK weight 사용 |
| hard FK 고정의 효과는 무엇인가 | A2 → A3 | 동일 held-out marker px, cross-target/path | FK가 GT라고 단정 |
| soft FK factor의 효과는 무엇인가 | A2 → A4 | 동일 held-out marker px, cross-target/path | 실측 covariance 없을 때 최종 결론 |
| hard fixed와 soft factor 중 무엇이 나은가 | A3 ↔ A4 | px, common path, 이후 external GT | 내부 지표만으로 물리 정확도 단정 |
| board residual의 최적화 기여는 무엇인가 | B2 → A4 | held-out cube px, `e_cross`, pixel transfer, `e_e2e` | marker 시스템 end-to-end 성능 주장 |
| cube residual의 최적화 기여는 무엇인가 | B3 → A2 | held-out board px와 cross-target/path | B3 row-local N/A를 0으로 간주 |
| 실제 marker 시스템으로 무엇이 좋은가 | marker-system의 3개 조건 | common target px와 common path | Table 1 B2/B3로 대체 |

핵심 원칙은 **같은 관측 population을 가진 지표만 직접 비교하는 것**이다. 조건마다 marker가 다르면 해당 marker의 공통 component 또는 `cross-target` 결과를 사용한다.

## 13. 현재 session02 결과와 비교별 해석

상세 수치, 공정 비교 변화량, marker-system 결과 및 해석은 [session02_result_table1.md](session02_result_table1.md)에 분리했다. 이 문서의 표는 세 canonical CSV와 자동 대조되며 Interactive HTML과 동일한 데이터만 사용한다.

핵심 요약은 다음과 같다.

- 통합 최적화의 개선 방향은 대체로 일관되지만 held-out primary px 개선폭은 작다.
- A3는 full board+cube 조건의 pixel 및 `e_cross`에서 가장 좋지만 `e_e2e`는 악화되므로 물리 정확도의 최종 우승자로 확정할 수 없다.
- Soft-FK 조건은 아직 Simulation prior 기반 preflight이며 실측 covariance 재실행이 필요하다.
- Board와 cube는 서로 다른 지표를 개선하므로 marker-system의 절대 우승 조건도 아직 없다.

## 14. 출력 파일과 시각화 데이터 흐름

### 14.1 Table 1 원시 출력

```text
CP_result/sessionNN/late_table1/
├── shared_train_only_baseline.json
├── shared_board_free_fk_cube.json
├── table1_methods.json
├── table1_results.csv
└── TABLE1_RESULTS.md
```

- `shared_train_only_baseline.json`: split, solver, 관측 loader, 공통/행별 initial state와 SHA-256
- `shared_board_free_fk_cube.json`: board/held-out 미사용 FK–cube alignment provenance
- `table1_methods.json`: 각 행·seed의 transforms, train/test px, path metrics, solver/Jacobian diagnostics
- `table1_results.csv`: 논문/HTML용 요약 숫자의 canonical table
- `TABLE1_RESULTS.md`: 교수님 피드백, 결과표, 지표 해석 문서

기본 실행은 3개 초기값을 사용한다. seed 0은 공통 baseline 그대로이고, 나머지 seed는 각 자유 transform에 결정론적인 5 mm/1° perturbation을 준다. `table1_methods.json`에는 각 run의 원값을 보존하고, CSV/표에는 수렴 수와 metric 평균을 요약한다. 따라서 한 번의 우연한 초기값에서 얻은 숫자만 보고하지 않는다.

### 14.2 보조 평가 출력

```text
CP_result/sessionNN/
├── cross_target_evaluation/
│   ├── cross_target_evaluation.json
│   └── cross_target_evaluation.csv
└── marker_system_end_to_end/
    ├── marker_system_end_to_end.json
    └── marker_system_end_to_end.csv
```

### 14.3 시각화 동기화

```text
table1_methods.json ─┐
cross_target CSV ────┼─> tools/sync_table1_canonical_data.py
marker-system CSV ───┘             │
                                   ├─> table1_results.csv
                                   └─> _TABLE1_INTERACTIVE.html

table1_results.csv + 2 evaluation CSV + HTML + MD
        └─> tools/verify_table1_visual_sync.py
```

Markdown 결과표는 읽기 좋은 설명과 함께 유지하지만, 표시 숫자는 verifier가 세 canonical CSV 및 HTML의 숫자와 대조한다. `e_cross`는 `tools/verify_e_cross_definition.py`가 FK/GT 없는 별도 코드 경로로 다시 계산한다.

## 15. 코드 구조

| 코드 | 역할 |
| --- | --- |
| `Run_calibration_comparison.py` | 모든 실제 비교실험의 단일 사용자 진입점 |
| `Step3_calibration.py` | Table 1과 같은 코드로 baseline만 생성 |
| `calibration_pipeline/schema.py` | A/B 조건, 자유변수, 공정 비교 계약 |
| `calibration_pipeline/table1.py` | split, baseline, A/B 실행, raw result 저장 |
| `calibration_pipeline/observations.py` | board/cube raw pixel observation 구성 |
| `calibration_pipeline/se3.py` | PnP pose의 robust 평균과 meta/FK 로딩 |
| `calibration_pipeline/fk_alignment.py` | board-free train-only FK–cube alignment |
| `calibration_pipeline/reprojection.py` | 공통 raw-corner pixel solver |
| `calibration_pipeline/fk_factor.py` | covariance-whitened corrected-FK factor |
| `calibration_pipeline/evaluation.py` | frozen pixel reprojection과 공통 target 평가 |
| `calibration_pipeline/path_evaluation.py` | `e_cross`, pixel transfer, `e_e2e` |
| `calibration_pipeline/cross_target.py` | 모든 Table 1 transform의 동일 population 재평가 |
| `calibration_pipeline/marker_system.py` | modality별 초기화부터 수행하는 end-to-end 비교 |
| `calibration_pipeline/blind_prediction.py` | GT-blind pose prediction |
| `calibration_pipeline/external_gt.py` | 독립 GT 통계 평가 |
| `tools/sync_table1_canonical_data.py` | JSON/CSV에서 결과 CSV와 HTML 동기화 |
| `tools/verify_table1_visual_sync.py` | CSV·MD·HTML 숫자 및 계약 검증 |
| `tools/verify_e_cross_definition.py` | 표준 fixed-camera consistency 독립 재계산 |

## 16. 전체 실행 명령

```bash
# 선택: 공통 baseline만 먼저 생성
python3 Step3_calibration.py \
  --root_folder data/sessionNN/calib_train \
  --intrinsics_dir intrinsics \
  --include_sets 5-12 \
  --split_seed 20260731 \
  --out_dir CP_result/sessionNN/late_table1

# A0~A4/B1~B3 실행, A5 baseline 예약
python3 Run_calibration_comparison.py table1 \
  --root_folder data/sessionNN/calib_train \
  --intrinsics_dir intrinsics \
  --include_sets 5-12 \
  --split_seed 20260731 \
  --num_inits 3 \
  --out_dir CP_result/sessionNN/late_table1

# 저장된 모든 Table 1 방법의 공통 target 평가
python3 Run_calibration_comparison.py cross-target \
  --root_folder data/sessionNN/calib_train \
  --intrinsics_dir intrinsics \
  --include_sets 5-12 \
  --split_seed 20260731 \
  --table1_result CP_result/sessionNN/late_table1/table1_methods.json \
  --out_dir CP_result/sessionNN/cross_target_evaluation

# marker modality별 end-to-end 비교
python3 Run_calibration_comparison.py marker-system \
  --root_folder data/sessionNN/calib_train \
  --intrinsics_dir intrinsics \
  --include_sets 5-12 \
  --split_seed 20260731 \
  --out_dir CP_result/sessionNN/marker_system_end_to_end

# canonical CSV와 interactive HTML 갱신
# 현재 sync script의 source 경로는 canonical session02로 고정되어 있음
python3 tools/sync_table1_canonical_data.py

# 결과 무결성 검증
python3 tools/verify_table1_visual_sync.py
python3 tools/verify_e_cross_definition.py
```

현재 session02 결과는 다음에서 확인한다.

- [Session02 Table 1 결과와 비교별 해석](session02_result_table1.md)
- [Table 1 결과 및 교수님 피드백 반영](CP_result/session02/late_table1/TABLE1_RESULTS.md)
- [Interactive 결과](./_TABLE1_INTERACTIVE.html)
- [독립/통합 수식 상세](CALIBRATION_PIPELINE.md)
- [캘리브레이션 방법과 SOTA 설명](CALIBRATION_EXPLANATION_LATEX.md)
- [Simulation 추후 수정 목록](Simulation/Simulation_TO_EDIT.md)

## 17. 현재 해석 한계

1. A3의 FK-fixed가 좋은 것은 aligned FK cube pose를 hard constraint로 사용한 영향일 수 있다. 눈금 cube jig의 반복 파지 실측 없이 FK를 정답으로 주장할 수 없다.
2. A4/B1/B2는 실측 covariance 파일이 없으면 Simulation prior를 사용한 preflight다. 최종 corrected-FK 주장에는 preregistered physical covariance가 필요하다.
3. `e_cross`, pixel transfer, `e_e2e`는 중요한 내부 일관성 지표지만 external absolute accuracy가 아니다.
4. event holdout pixel 평가는 관측 일반화 검증이며 새로운 작업 위치 전체에 대한 물리 정확도를 직접 보장하지 않는다.
5. B2/B3 shared-baseline ablation과 marker-system end-to-end 비교는 연구 질문이 다르므로 같은 주장으로 섞지 않는다.
6. A5는 correction label과 적용 규칙이 확정되기 전까지 실행하지 않는다.
