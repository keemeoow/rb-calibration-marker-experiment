# 다중 고정카메라·로봇 Hand–Eye 캘리브레이션 비교실험

이 문서는 실제 데이터 캡처부터 A0~A5/B1~B3 비교실험, 평가 계약, 결과 파일 생성까지의 메인 설명서다. 구현의 기준은 root의 `01_...py`~`06_...py` 실행 파일과 `calibration_pipeline/`이며, 현재 최종 추정기는 모든 실행 조건에서 동일한 **raw-corner pixel reprojection 최적화**를 사용한다. 단계별 입력·과정·결과와 복사 가능한 전체 명령은 [`CALIBRATION_PIPELINE_RUNBOOK.md`](CALIBRATION_PIPELINE_RUNBOOK.md)를 따른다.

## 바로가기

- [Session04 canonical 결과 인덱스](CP_result/README.md)
- [Session04 Board–Cube systematic error 진단](data/session04/calib_out/verify/board_cube_relative_pose/BOARD_CUBE_RELATIVE_POSE.md)
- [문서 지도](#문서-지도)
- [1. 가장 짧은 실행 순서](#1-가장-짧은-실행-순서)
- [2. 시스템과 좌표계](#2-시스템과-좌표계)
- [3. 입력 데이터](#3-입력-데이터)
- [4. 전체 파이프라인](#4-전체-파이프라인)
- [5. PnP와 초기화의 역할](#5-pnp와-초기화의-역할)
- [6. 최종 재투영 최적화 원리](#6-최종-재투영-최적화-원리)
- [7. 순차와 통합 최적화의 차이](#7-순차와-통합-최적화의-차이)
- [8. A0~A5/B1~B3 비교실험](#8-a0a5b1b3-비교실험)
- [9. 마커 시스템 End-to-End 비교](#9-마커-시스템-end-to-end-비교)
- [10. 외부 GT 전 Cross-target 평가](#10-외부-gt-전-cross-target-평가)
- [11. 평가지표 정의와 적용 범위](#11-평가지표-정의와-적용-범위)
- [12. 어떤 조건끼리 비교해야 하는가](#12-어떤-조건끼리-비교해야-하는가)
- [13. 현재 session04 결과와 비교별 해석](#13-현재-session04-결과와-비교별-해석)
- [14. 출력 파일과 시각화 데이터 흐름](#14-출력-파일과-시각화-데이터-흐름)
- [15. 코드 구조](#15-코드-구조)
- [16. 전체 실행 명령](#16-전체-실행-명령)
- [17. 현재 해석 한계](#17-현재-해석-한계)
- [18. Terminology (용어 설명)](#18-terminology-용어-설명)

## 문서 지도

| 기능 | 기준 문서 | 관리 원칙 |
| --- | --- | --- |
| 실행·데이터·코드 계약 | `README.md` | 사람이 편집하는 저장소 진입점 |
| 수식·방법·논문 서술 | `CALIBRATION_EXPLANATION_LATEX.md` | 사람이 편집하는 이론 및 paper-ready 문서 |
| Session04 결과 | `CP_result/README.md` | 결과 인덱스; `TABLE1_RESULTS.md`와 `TABLE1_INTERACTIVE.html`은 생성기로 갱신 |
| Simulation | `Simulation/README.md` | 실행 진입점; backend 통합 계획은 `Simulation/MIGRATION_PLAN.md`, 결과는 `SIM_RESULTS.md`와 `results/` |
| Capture·검출 검증 | `data/session04/calib_out/capture_filter/CAPTURE_FILTER.md` 및 `data/session04/calib_out/verify/` | 데이터와 함께 보존하는 생성·검증 보고서; Board–Cube 계통오차 기준 문서는 `board_cube_relative_pose/BOARD_CUBE_RELATIVE_POSE.md` |

## 1. 가장 짧은 실행 순서

Session04 내부 결과를 재생성하려면 다음을 순서대로 실행한다. 경로 인자는 넘기지 않는다 —
`--calib_dir`, `--out_dir`, `--observation-manifest` 등은 모두 `--root_folder`에서 유도된다.

```bash
COMMON="--root_folder data/session04/calib_train --include_sets 0-12 \
  --min_train_eih_cube_events 3 --split_seed 20260731 --observation-filter-policy standard"

python3 05_calibrate.py                 $COMMON --num_inits 3
python3 06_make_report.py --root_folder data/session04/calib_train
```

두 명령으로 calibration과 상세 결과/전체 행렬 출력이 끝난다. Cross-target,
marker-system, OpenCV baseline은 `tools/`에 있는 선택 평가이며 메인 완료 조건이 아니다.

```bash
# 1. RealSense factory intrinsics 저장
python3 01_export_intrinsics.py --out_dir intrinsics

# 1b. ChArUco 기반 intrinsics 재보정
python3 02_calibrate_intrinsics.py \
  --intr_dir intrinsics \
  --save_images

# 2. cube/board 영상, depth, robot FK 동시 캡처
python3 03_capture.py \
  --data_root data \
  --intrinsics_dir intrinsics \
  --use_robot \
  --robot_ip 192.168.0.23 \
  --robot_port 12348 \
  --show

# 2b. board/cube geometry와 native-pixel corner를 SHA-256 manifest로 고정
python3 04_filter_observations.py \
  --session-root data/sessionNN/calib_train \
  --intrinsics-dir intrinsics

# 3. A/B 비교실험 실행
python3 05_calibrate.py \
  --root_folder data/sessionNN/calib_train \
  --intrinsics_dir intrinsics \
  --include_sets 0-12 \
  --split_seed 20260731 \
  --observation-manifest data/sessionNN/calib_out/capture_filter/Step2b_observation_manifest.json \
  --out_dir CP_result/sessionNN/late_table1
```

Shared Train-only Baseline만 확인하려면 05번에 `--baseline_only`를 추가한다. 전체 Table 1 실행에도 같은 baseline 코드가 포함되므로 보통은 05번을 한 번만 실행하면 된다.

## 2. 시스템과 좌표계

| 기호 | 의미 | 최적화 여부 |
| --- | --- | --- |
| $T^B_{C_i}$ | 고정카메라 $i$에서 robot base로의 외부 파라미터 | 조건에 따라 자유변수 |
| $T^G_C$ | eye-in-hand 카메라 좌표에서 gripper 좌표로의 Hand–Eye transform | 조건에 따라 자유변수 |
| $T^B_G(e)$ | event $e$의 robot FK가 제공한 base–gripper transform | 모든 조건에서 고정 입력 |
| $T^B_{\mathrm{board}}$ | 작업공간에 고정된 ChArUco board pose | board 조건에서 자유변수 |
| $T^B_{\mathrm{cube}}(s)$ | 배치 set $s$의 cube pose | vision 추정, raw/aligned FK 고정 또는 FK factor |
| $(K_i,D_i)$ | 카메라 $i$의 intrinsic과 distortion | 사전 보정 후 항상 고정 |

중요한 용어 규칙은 다음과 같다.

- `vision` 또는 `no-FK`는 **robot FK 전체를 사용하지 않는다는 뜻이 아니다.** Eye-in-hand 카메라 pose $T^B_G(e)T^G_C$를 만들기 위해 robot FK는 모든 조건에서 사용한다.
- `vision`, `raw-FK-fixed`, `vision-aligned-FK-fixed`, `corrected-FK factor`의 차이는 배치된 cube pose $T^B_{\mathrm{cube}}(s)$를 자유변수로 둘지, raw FK의 mechanical frame map 또는 train-vision-aligned FK에 고정할지, 또는 자유변수에 aligned-FK covariance-whitened factor를 연결할지의 차이다.
- board는 robot에 부착된 물체가 아니므로 `FK-fixed board`라는 조건은 물리적으로 정의하지 않는다.

`T^B_G(e)`의 $G$는 한 실행 안에서 반드시 같은 물리 frame이어야 한다. 현재 session02는 event 0--89가 tool3(150 mm TCP), event 90--95가 flange로 기록되어 있어, [`pose_convention_manifest.json`](data/session02/calib_train/pose_convention_manifest.json)으로 모두 flange 기준으로 정규화한다. cube-center 기록도 같은 manifest에서 legacy tool4 177.5 mm를 실제 tool4 143.0 mm 기준으로 바꾼다. 적용식은 강체 좌표변환

$$
T^B_{G,\mathrm{canonical}}
=T^B_{G,\mathrm{reported}}T^{G,\mathrm{reported}}_{G,\mathrm{canonical}}
$$

이며 픽셀 잔차나 최적화 결과는 이 선택에 사용하지 않는다. 정규화 뒤에는 `inv(T_base_robot) @ T_base_capture_cube_center`가 한 종류인지, 같은 `set_index`의 cube-center가 5 mm/1° 안에서 정지해 있는지 검사하고 실패 시 계산을 중단한다. manifest 자체도 결과 provenance SHA-256에 포함된다.

## 3. 입력 데이터

### 3.1 01/02의 입력과 출력

| 단계 | 입력 | 출력 |
| --- | --- | --- |
| 01 | 연결된 RealSense 장치 | `intrinsics/cam*.npz`의 factory $(K,D)$, depth scale |
| 02 | ChArUco board 영상 | 재보정된 `cam*.npz`, 기존 factory 값 backup |

비교실험에서는 이 $(K,D)$를 다시 최적화하지 않는다. 조건 간 intrinsic 자유도가 달라지는 것을 방지하기 위해 모든 행에서 상수로 고정한다.

### 3.2 03의 입력과 출력

03은 다음 정보를 같은 capture event 단위로 저장한다.

- 고정카메라와 eye-in-hand 카메라의 RGB/depth 영상
- cube AprilTag와 ChArUco board 검출 provenance
- `event_id`, `set_index`, `gripper_cam_idx`
- robot pose $T^B_G(e)$ 또는 대응 6-DoF FK
- cube를 파지한 경우 `cube_gripped`, `grasp_id`
- cube geometry/configuration과 카메라별 저장 성공 여부

비교실험의 필수 입력은 다음과 같다.

```text
data/sessionNN/calib_train/
├── meta.json
├── pose_convention_manifest.json  # 기록 frame이 섞인 세션만 필요
└── ... RGB/depth images

intrinsics/
└── cam*.npz
```

### 3.3 04 촬영 후 관측 고정

촬영이 끝난 뒤 [`04_filter_observations.py`](04_filter_observations.py)를
실행하면 원본을 수정하지 않고 저장된 RGB를 다시 검출한다. Cube는 기본 검출이
core 조건을 만족하지 않은 경우에만 subpixel/완화 threshold/2배 확대 후보를
평가하며, 서로 다른 방향의 non-coplanar face가 2개 이상인 관측만 calibration
핵심 관측으로 선택한다. Board도 저장 당시 숫자를 재사용하지 않고 모든 saved
RGB에서 ChArUco corner를 다시 검출한다.

```bash
python3 04_filter_observations.py \
  --session-root data/session04/calib_train \
  --intrinsics-dir intrinsics
```

기본 출력은 `data/session04/calib_out/capture_filter/`이다. 이 디렉터리의
`CAPTURE_FILTER.md`에 event별 선택/제외 이유와 재촬영 후보가 정리되고,
`Step2b_review_overlay.jpg`에서 recovered/quarantine/rejected 관측을 확인할 수
있다. `Step2b_observation_manifest.json`은 native-pixel 2D corner, 대응 3D corner,
정책 판정 및 source SHA-256을 고정한다.

Cube의 기본 AprilTag 검출은 `CORNER_REFINE_APRILTAG`를 사용한다. refinement를
끄면 정수 pixel corner의 systematic localization bias가 fixed-camera relative pose에
누적되므로 canonical manifest로 허용하지 않는다. Manifest의 `marker_ids[k]`는
`object_points[4*k:4*k+4]`와 같은 검출 순서를 보존한다.

05/Table 1에서 detector를 다시 실행하지 않고 이 관측 집합을 그대로 쓰려면
다음 옵션을 추가한다.

```bash
--observation-manifest data/session04/calib_out/capture_filter/Step2b_observation_manifest.json \
--observation-filter-policy standard
```

`strict` 정책은 cube RMSE/inlier와 board corner 수 기준을 더 강하게 적용한다.
Meta, intrinsics 또는 선택된 RGB 파일의 SHA-256이 manifest 생성 이후 바뀌면
calibration은 stale input으로 판단하고 중단한다.

공식 결과는 항상 물리 config의 nominal metric scale을 그대로 사용한다.
Session04 영상과 corner-ID topology에는 가로 checker square가 11개이고 내부
ChArUco corner column이 10개다. 275 mm가 흰 여백을 제외한 checker pattern의
전체 폭이므로 `275/11=25 mm`, 즉 `square_length_m=0.025`와 일치한다.
`--align-board-metric-scale`은 실측값이 아닌 데이터 기반 추정값을 사용하는
**별도 진단 전용 옵션**이다. 공식 `CP_result/sessionNN` 실행에는 넣지 않으며,
canonical Markdown/HTML 생성기도 scale 정렬 결과를 거부한다. 실물 치수를 측정하고
사용자가 명시적으로 승인하기 전에는 config나 공식 결과에 반영하지 않는다.

## 4. 전체 파이프라인

```text
01/02: K,D 고정
        │
03: RGB/depth + robot FK + meta.json
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
       A0~A5/B1~B3 raw-corner reprojection 1차 최적화
                    │
                    ├── train (event,camera) frame MAD prune
                    ├── 남은 train frame으로 1회 refit
                    └── 원본 train robust cost 미개선 시 1차 결과로 rollback
       A6는 독립 실측 correction label 전까지 baseline만 예약
                    │
                    ├── row-local train/test reprojection px
                    ├── FK-free Fixed-to-Fixed (고정카메라 간) Board/Cube Cross-view
                    ├── FK-dependent Gripper-to-Fixed (그리퍼카메라–고정카메라 간) Board/Cube Cross-view
                    ├── reference-dependent reprojection diagnostic
                    ├── legacy cube e_cross / FK-dependent e_e2e
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
- target 좌표계의 3D corner $\mathbf X_n$
- native distorted image의 실제 2D corner $\mathbf u_n$

board와 cube 모두 최종 solver에는 PnP pose가 아니라 **3D corner–2D pixel 대응점**으로 들어간다.

### 4.2 Train/Test 분리

기본 primary split은 `event_grouped_and_set_stratified`다.

- 같은 event의 모든 카메라 관측은 train 또는 test 한쪽에만 들어간다.
- 각 cube set에 train과 test event가 모두 남도록 나눈다.
- set별 eye-in-hand cube train event 수가 최소 기준보다 작아지지 않게 한다.
- test event는 baseline, FK alignment, 초기값 생성에 사용하지 않는다.
- set 최초 고정카메라 관측은 최적화 잔차에서 한 번만 사용한다. 같은 set의
  이후 gripper-camera 관측은 공통 `T_base_cube(set)`을 통해 이 anchor와 연결된다.

이 split은 새로운 cube 위치 전체를 숨기는 position holdout이 아니다. 위치 전체 holdout에서 자유 cube pose를 test-time fitting 없이 평가할 수 없기 때문에, primary pixel 평가는 같은 set 안의 held-out event를 사용한다. 새로운 위치에 대한 물리 정확도는 별도 blind external-GT 실험으로 평가한다.

## 5. PnP와 초기화의 역할

PnP는 최종 결과를 직접 결정하는 backend가 아니라 초기값과 독립 consistency 평가를 만드는 도구다.

### 5.1 관측별 PnP

측정 corner만으로 $T^{C_i}_{O}$를 계산한다. 역할에 따라 두 계약을 구분한다.

- 모든 방법이 공유하는 cube 입력 품질 마스크: planar support는 positive-depth `SOLVEPNP_IPPE` 후보 중 all-corner RMSE 최소 해, non-planar support는 `RANSAC-EPNP(+LM)` 초기화 후 all-corner RMSE 평가
- 품질 판정: 검출된 모든 corner의 Euclidean pixel RMSE가 고정카메라 3 px, gripper 카메라 5 px 이하여야 하며 train/test split 전에 한 번만 적용
- 초기화/경로평가: planar target은 `SOLVEPNP_IPPE`, non-planar cube는 `SOLVEPNP_ITERATIVE`
- 모든 경우 최소 4개 3D–2D 대응점과 positive depth가 필요
- 최종 Table 1 solver에서는 PnP pose residual을 쓰지 않고 raw pixel corner residual을 쓴다.

### 5.2 Board 기반 Hand–Eye 초기화

Eye-in-hand board PnP와 robot FK를 이용해 다음 OpenCV Hand–Eye 후보를 모두 계산한다.

- Tsai
- Park
- Horaud
- Daniilidis

각 후보로 여러 event의 board base pose를 만들고 robust SE(3) 평균 및 pose dispersion을 계산한다. 가장 일관된 후보가 $T^G_C$와 $T^B_{\mathrm{board}}$ 초기값으로 선택된다.

### 5.3 Cube pose와 고정카메라 초기화

초기 Hand–Eye와 eye-in-hand cube PnP를 조합해 set별 visual cube pose를 만든 뒤 MAD 기반 robust SE(3) 평균을 적용한다.

고정카메라 초기값은 다음 변환 체인에서 얻는다.

$$
T^B_{C_i}=T^B_O\left(T^{C_i}_O\right)^{-1}
$$

Board (보드)와 Cube (큐브)에서 얻은 후보를 카메라별로 Robust SE(3) Mean (강건 평균)한다. 이 값들은 모든 Table 1 조건이 공유하는 출발점일 뿐 최종 결과가 아니다.

### 5.4 Board-free FK–cube alignment

A4/A5/B1/B2가 공유하는 FK cube artifact는 다음 관계를 train eye-in-hand cube corner만으로 정렬한다. A3는 이 artifact를 사용하지 않는다. A4/B1/B2는 이를 soft factor의 중심으로 사용하고, post-hoc A5는 동일 pose를 hard constraint로 고정한다.

$$
T^B_G(e)T^G_C T^C_O(e)
=T^B_{\mathrm{FK,cube,raw}}(s)T^{\mathrm{FK,cube}}_O
$$

board, 고정카메라 관측, held-out event는 사용하지 않는다. PnP/Hand–Eye 또는 global $AX=ZB$는 초기화에만 쓰고, artifact의 최종 정렬은 eye-in-hand cube raw-corner 재투영으로 다시 최적화한다.

## 6. 최종 재투영 최적화 원리

### 6.1 고정카메라 관측

고정카메라 $i$가 target $O$를 본 경우 예측 transform은 다음과 같다.

$$
\hat T^{C_i}_{O}=\left(T^B_{C_i}\right)^{-1}T^B_O
$$

### 6.2 Eye-in-hand 관측

event $e$의 eye-in-hand 카메라 pose는 robot FK와 Hand–Eye로 결정된다.

$$
T^B_C(e)=T^B_G(e)T^G_C
$$

따라서 target 예측은 다음과 같다.

$$
\hat T^C_O(e)=\left(T^B_G(e)T^G_C\right)^{-1}T^B_O
$$

### 6.3 Pixel residual

카메라별로 고정된 $(K_i,D_i)$를 이용해 3D corner를 distorted image에 투영한다.

$$
\hat{\mathbf u}_{i,e,n}
=\pi\!\left(K_i,D_i,\hat T^{C_i}_{O}(e)\mathbf X_n\right)
$$

$$
\mathbf r_{i,e,n}=\hat{\mathbf u}_{i,e,n}-\mathbf u_{i,e,n}
$$

모든 선택된 board/cube, eih/e2h corner residual을 하나의 벡터로 쌓아 최적화한다. 기본 solver는 다음과 같다.

- SciPy trust-region reflective `least_squares(method="trf")`
- `soft_l1` robust loss
- `f_scale=2 px`
- `x_scale="jac"`
- SE(3) left-local perturbation과 retraction
- 기본 `max_nfev=300`, `xtol=ftol=gtol=1e-8`

1차 적합 뒤에는 **학습 관측에만** 한 번의 frame-prune/refit을 적용한다. 한 frame은
같은 `(event_id, camera_id)` 이미지의 board/cube 관측 전체다. Frame RMSE가
`max(4 px, median + 3 * 1.4826 * MAD)`보다 큰 후보를 높은 순서로 제거하되,
전체 frame의 30%를 넘기지 않고 각 자유 transform을 지지하는 관측을 최소 2개
남긴다. 남은 frame으로 1차 결과에서 재적합하고, **원래의 제거 전 학습 관측 전체**에
대한 동일 robust objective가 상대적으로 `1e-6`보다 많이 감소할 때만 채택한다.
실패하거나 개선되지 않으면 1차 결과로 rollback한다.

Held-out 관측은 후보 선택, refit, rollback 판정에 사용하지 않으며 절대 제거하지
않는다. 모든 단계의 선택 frame, 전후 residual population, cost, 채택 여부는 solver
diagnostics에 저장된다. 이 절차를 끄는 재현용 옵션은 `--no-frame-prune-refit`이다.
PnP Mean, Pose-consistency 후처리, Ridge Correction은 최종 단계에 사용하지 않는다.

## 7. 순차와 통합 최적화의 차이

### 7.1 순차 방식 `seq`

1단계는 eye-in-hand 관측만 사용한다.

$$
\min_{T^G_C,T^B_O}
\sum_{(e,n)\in\mathrm{eih}}\sum_{k\in\{u,v\}}
\rho\!\left(r_{e,n,k}\right)
$$

2단계에서는 1단계의 Hand–Eye와 target pose를 완전히 고정하고 고정카메라만 푼다.

$$
\min_{T^B_{C_1},\ldots,T^B_{C_m}}
\sum_{i=1}^{m}\sum_{(e,n)\in\mathrm{e2h}_i}\sum_{k\in\{u,v\}}
\rho\!\left(r_{i,e,n,k}\right)
$$

target과 Hand–Eye가 고정되어 카메라별 residual block은 수학적으로 분리된다. e2h 결과가 eih 변수로 되돌아가는 alternating pass는 없다.

### 7.2 통합 방식 `U`

eih와 e2h residual을 하나의 문제에 넣고 모든 선언된 자유변수를 동시에 갱신한다.

$$
\min_{\{T^B_{C_i}\},T^G_C,\{T^B_O\}}
\left[
\sum_{(e,n)\in\mathrm{eih}}\sum_{k\in\{u,v\}}\rho(r_{e,n,k})
+\sum_i\sum_{(e,n)\in\mathrm{e2h}_i}\sum_{k\in\{u,v\}}\rho(r_{i,e,n,k})
\right]
$$

공유 target pose와 Hand–Eye가 두 관측 경로를 연결하므로 고정카메라 관측이 Hand–Eye/target 추정에 피드백된다. `e_cross`는 이 목적함수의 항이 아니라 최적화 후 독립적으로 계산하는 평가지표다.

## 8. A0~A5/B1~B3 비교실험

모든 실행 가능한 행은 동일한 train/test split, $(K,D)$, raw detection, solver option, seed와 train-only reference state에서 시작한다. 행별로 바뀌는 것은 marker residual, cube pose 처리, 자유변수 freeze mask뿐이다.

| ID | 입력 marker | 최적화 | Cube pose 처리 | 자유변수 | 핵심 출력/질문 |
| --- | --- | --- | --- | --- | --- |
| A0 | board | 순차 `seq` | cube 없음 | 1단계 $T^G_C,T^B_{\mathrm{board}}$, 2단계 $T^B_{C_i}$ | board-only optimization baseline |
| A1 | board+cube | 순차 `seq` | vision 자유변수 | 1단계 $T^G_C,T^B_{\mathrm{board}},T^B_{\mathrm{cube}}(s)$, 2단계 $T^B_{C_i}$ | 같은 순차법에서 cube residual 추가 효과 |
| A2 | board+cube | 통합 `U` | vision 자유변수 | $T^B_{C_i},T^G_C,T^B_{\mathrm{board}},T^B_{\mathrm{cube}}(s)$ | vision-only 통합 효과 |
| A3 | board+cube | 통합 `U` | raw FK + mechanical frame map으로 hard fixed | $T^B_{C_i},T^G_C,T^B_{\mathrm{board}}$ | 영상 정렬 없는 raw FK hard constraint 효과 |
| A4 | board+cube | 통합 `U` | covariance-whitened soft FK factor | A2와 같음 | vision과 FK 불확실성을 함께 쓰는 효과 |
| A5 | board+cube | 통합 `U` | train-only vision-aligned FK로 hard fixed | $T^B_{C_i},T^G_C,T^B_{\mathrm{board}}$ | 이전 A3 성능 원인을 분리하는 post-hoc 진단 |
| A6 | 미정 | 미실행 | 독립 실측 6-DoF correction label 필요 | 아직 정의하지 않음 | A4와 동일 baseline만 예약, 현재 `not_run` |
| B1 | board+cube | 순차 `seq` | A4와 동일 soft FK factor | 1단계 Hand–Eye/board/cube, 2단계 camera별 | 같은 FK factor에서 통합 효과 검증 |
| B2 | cube | 통합 `U` | A4와 동일 soft FK factor | $T^B_{C_i},T^G_C,T^B_{\mathrm{cube}}(s)$ | 같은 FK factor에서 board residual 제거 효과 |
| B3 | board | 통합 `U` | cube 없음 | $T^B_{C_i},T^G_C,T^B_{\mathrm{board}}$ | vision 통합 조건에서 cube residual 제거 효과 |

### 8.1 A0 — Board·순차 baseline

- 입력: board corner, $(K,D)$, robot FK
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

### 8.4 A3 — raw-FK-fixed 통합

- 입력: A2 입력 + raw set-cube-center FK + 사전 등록한 $R_y(180^\circ)$ mechanical frame map
- 최적화: cube pose는 변수 목록에서 제거하여 완전히 고정하고 나머지만 통합 최적화
- 비교 목적: A2 대비 cube pose를 vision으로 추정하는 대신 FK 상수로 두는 효과
- 주의: FK가 실제로 정확하지 않다면 hard constraint가 결과를 편향시킬 수 있으므로 실측 FK 검증이 필요

### 8.5 A4 — Corrected-FK factor 통합

- 입력: A2 입력 + aligned FK target + set별 6×6 FK covariance
- 최적화: cube pose는 자유변수로 유지하면서 visual residual에 FK factor를 추가

$$
\mathbf r_{\mathrm{FK},s}
=L_s^{-1}
\begin{bmatrix}
\operatorname{rotvec}\!\left((T^B_{cube}(s))^{-1}\bar T^B_{cube,FK}(s)\right)\\
\operatorname{trans}\!\left((T^B_{cube}(s))^{-1}\bar T^B_{cube,FK}(s)\right)
\end{bmatrix},
\quad \Sigma_s=L_sL_s^T
$$

FK factor에는 Huber loss를 적용한다. `--fk_covariance_json`이 없으면 고정 Simulation prior를 사용하는 preflight이며 confirmatory 결과로 해석하지 않는다. 실측 파일은 측정 source와 estimator가 명시되어야 하고, full-rank 6D 표본 covariance의 수학적 최소 조건인 7회 이상의 독립 반복, 대칭성, positive-definiteness를 통과해야 한다.

### 8.6 A5 — vision-aligned-FK-fixed post-hoc 진단

A5는 A4와 동일한 board-free train-only aligned-FK artifact를 사용하되, set별 cube pose를 자유변수와 FK factor에서 모두 제거하고 상수로 고정한다.

$$
T^B_{\mathrm{cube}}(s)=T^B_{\mathrm{FK,cube,raw}}(s)\Delta_{\mathrm{train}}
$$

따라서 A3↔A5는 mechanical raw FK와 train-vision alignment의 차이를, A4↔A5는 동일 aligned target을 soft factor와 hard constraint로 사용하는 차이를 분리한다. `\Delta_{\mathrm{train}}`이 영상으로 적합되었고 결과 확인 후 추가한 행이므로 A5는 `posthoc_diagnostic`이며 독립 실측 correction이나 external GT로 부르지 않는다.

### 8.7 A6 — 독립 correction label 대기

A6는 독립 실측으로 얻은 6-DoF correction label과 적용 규칙이 정해지기 전에는 실행하지 않는다. 현재 baseline artifact에는 A4와 byte-identical한 initial state만 넣어 향후 비교에서 초기값 차이가 생기지 않게 한다.

### 8.8 B1/B2/B3 — 원인 분리용 ablation

- B1↔A4: marker와 FK factor는 같고 순차/통합만 다르다.
- B2↔A4: cube와 FK factor는 같고 board residual 유무만 다르다.
- B3↔A2: Board-based Unified Optimization (보드 기반 통합 최적화)이라는 동일 조건에서 Cube Residual (큐브 잔차) 유무를 본다.

B2/B3는 all-marker shared initializer에서 시작한 뒤 residual과 변수를 제거한다. 따라서 “처음부터 cube-only/board-only 시스템을 구축했을 때의 성능”이 아니라 **동일 초기조건에서 최적화 항의 기여도**를 측정한다.

## 9. 마커 시스템 End-to-End 비교

실제 마커 구성 자체를 비교하려면 다음 별도 실험을 사용한다.

```bash
python3 tools/compare_markers.py \
  --root_folder data/sessionNN/calib_train \
  --intrinsics_dir intrinsics \
  --include_sets 0-12 \
  --observation-manifest data/sessionNN/calib_out/capture_filter/Step2b_observation_manifest.json \
  --out_dir CP_result/sessionNN/marker_system_end_to_end
```

| 시스템 | 초기화에 허용되는 marker | 최종 목적함수 marker | 최종 자유변수 |
| --- | --- | --- | --- |
| `board_only` | board만 | board만 | cameras, Hand–Eye, board pose |
| `cube_only` | cube만 | cube만 | cameras, Hand–Eye, cube poses |
| `board_cube` | board+cube | board+cube | cameras, Hand–Eye, board/cube poses |

세 시스템은 split, raw detection, $(K,D)$, solver, seed와 held-out 평가 population을 공유하지만 초기값은 modality별로 별도 생성한다. cube-only는 board-free train-only FK artifact로 Hand–Eye를 초기화하지만, 최종 목적함수에는 FK factor가 없다.

보드와 큐브의 촬영 원본이 모두 있으므로, 어떤 Marker (마커)로 캘리브레이션했는지와 무관하게 두 Target (표적)을 모두 평가할 수 있다. 평가는 Fixed-to-Fixed (고정카메라 간)와 Gripper-to-Fixed (그리퍼카메라–고정카메라 간)를 함께 보고한다. 전자는 FK 없이 고정카메라 보정만 분리하고, 후자는 같은 Image Corner (영상 코너)를 사용하되 `FK × Hand–Eye`를 거쳐 전체 체인을 평가한다. Shared Target Pose Reprojection (공유 표적 자세 재투영)은 보조 진단값이며, External GT (외부 정답)가 아니므로 절대 정확도 주장은 할 수 없다.

## 10. 외부 GT 전 Cross-target 평가

Table 1의 A0/B3처럼 목적함수에 cube가 없는 행과 cube-bearing 행은 row-local residual population이 다르다. 하지만 촬영 원본에 보드와 큐브 검출값이 모두 있으면, 캘리브레이션에 쓰지 않은 target도 **평가용 관측**으로 사용할 수 있다. 저장된 각 방법의 transform을 동결한다. Fixed-to-Fixed는 같은 held-out event의 고정카메라를 비교하고, Gripper-to-Fixed는 set 최초 fixed anchor를 같은 set의 모든 held-out gripper Event와 비교한다.

```bash
python3 tools/evaluate_cross_target.py \
  --root_folder data/sessionNN/calib_train \
  --intrinsics_dir intrinsics \
  --include_sets 0-12 \
  --observation-manifest data/sessionNN/calib_out/capture_filter/Step2b_observation_manifest.json \
  --table1_result CP_result/sessionNN/late_table1/table1_methods.json \
  --out_dir CP_result/sessionNN/cross_target_evaluation
```

모든 방법에는 Identical Evaluation Population (동일 평가 모집단), 즉 같은 Camera Pair (카메라 쌍)·Event (이벤트)·Corner (코너)를 적용한다. 그러나 모집단이 같다는 사실만으로 Reference (기준값)가 공정하거나 중립적이라는 뜻은 아니다.

따라서 외부 GT가 들어오기 전에는 다음처럼 분리한다.

1. Fixed-to-Fixed (고정카메라 간): Shared Target Pose (공유 표적 자세)와 FK 없이 보드·큐브별 Pixel Transfer (픽셀 전달) 및 Pose Consistency (자세 일관성)
2. Gripper-to-Fixed (그리퍼카메라–고정카메라 간): set 최초 fixed anchor를 같은 set의 모든 held-out gripper Event와 연결해 보드·큐브별 픽셀 전달 및 자세 일관성을 계산. 움직이는 손목카메라 자세 계산에는 Robot FK (로봇 순기구학)가 들어감
3. 보조 지표: train-only board/cube pose를 reference로 쓰는 reprojection. 모든 방법의 순위를 정하는 근거로 사용하지 않음
4. Legacy 보조 지표: cube 경로를 평균한 `e_e2e`

Fixed-to-Fixed (고정카메라 간) 값이 전체 평가를 대표하지는 않는다. 이는 Camera Extrinsic (카메라 외부 파라미터)의 상대 일관성을 Robot FK와 Hand–Eye 오차에서 분리하는 **부분 지표**다. 전체 Eye-in-Hand + Eye-to-Hand (그리퍼카메라 + 고정카메라) 성능은 Gripper-to-Fixed를 함께 보고하되, 그 오차에는 FK·Hand–Eye·Fixed-camera 오차가 섞인다고 명시한다.

## 11. 평가지표 정의와 적용 범위

### 11.1 Train reprojection RMSE px

$$
e_{\mathrm{train}}^{px}
=\sqrt{\frac{1}{2N}\sum_{n=1}^{N}
\left((\hat u_n-u_n)^2+(\hat v_n-v_n)^2\right)}
$$

- solver가 사용한 train corner에서 계산
- 최적화가 제대로 수렴했는지 보는 diagnostic
- 일반화 성능이나 절대 정확도의 주 지표가 아님

### 11.2 Own-marker held-out reprojection RMSE px

같은 식을 held-out event corner에 적용한다. 모든 transform은 train 결과로 동결하며 test-time refit이나 결과 의존적 관측 제거가 없다.

- 같은 marker population을 가진 조건끼리의 주 pixel 지표로 직접 비교 가능
- target 구성이 다른 조건의 pooled `overall`은 직접 비교하면 안 됨
- 재투영 오차는 px만 보고하며 예측 깊이를 곱한 임의의 mm 환산값은 사용하지 않음

### 11.3 A — Supplementary FK-free Fixed-to-Fixed Board/Cube Cross-view

Target $O\in\{board,cube\}$에 대해 각 고정카메라가 자기 측정 corner만으로 PnP pose를 계산한다.

$$
T^{B,(i)}_O=T^B_{C_i}T^{C_i}_{O,\mathrm{PnP}}
$$

동일 event의 카메라 pair $(i,j)$에서 pose 차이와 양방향 pixel transfer를 계산한다.

$$
T^{C_j}_O=\left(T^B_{C_j}\right)^{-1}T^B_{C_i}T^{C_i}_{O,\mathrm{PnP}}
$$

- 보드와 큐브는 각각 별도로 보고한다. 서로 다른 geometry/corner 수를 하나의 pooled 순위값으로 합치지 않는다.
- shared base-frame target pose, robot FK, gripper camera, 외부 GT를 사용하지 않는다.
- 모든 방법에 동일한 사전 고정 pair와 corner를 사용하며, 결과가 큰 pair를 사후 제거하지 않는다.
- 각 방법이 추정한 $T^B_{C_i}$를 사용하므로 방법별 held-out 자기 일관성만 평가하는 보조 지표이며, 방법 순위나 독립 기준선으로 사용하지 않는다.
- Camera-relative Consistency (카메라 간 상대 일관성)는 평가하지만, 모든 고정카메라에 함께 존재하는 Systematic Error (계통 오차)와 Absolute Physical Accuracy (절대 물리 정확도)는 검출할 수 없다.
- 독립 relative-pose 기준선 B는 `opencv-relative` runner가 별도로 계산하며, 메인 방법의 camera transform, robot FK, hand–eye, shared target pose를 사용하지 않는다.

### 11.4 FK-dependent Gripper-to-Fixed (FK 의존 그리퍼카메라–고정카메라 간) Board/Cube Cross-view

Eye-in-hand 카메라 $C_g$의 event별 base pose는 다음과 같다.

$$
T^B_{C_g}(e)=T^B_G(e)T^G_{C_g}
$$

Gripper 카메라가 영상에서 PnP한 target pose를 base로 옮겨 fixed camera 결과와 비교한다.

$$
T^{B,(g)}_O(e)=T^B_G(e)T^G_{C_g}T^{C_g}_{O,\mathrm{PnP}}
$$

양방향 pixel transfer는 fixed→gripper와 gripper→fixed를 모두 계산한다. 측정 residual은 실제 보드·큐브 corner이지만, calibration prediction에는 $T^B_G(e)$가 필요하다. 따라서 이 지표는 순수 영상 관측을 사용하면서도 **FK-dependent**이며, Hand–Eye 오차와 FK 오차를 분리할 수 없다.

각 `(target, set, fixed camera)`에서 촬영 순서상 최초 관측을 fixed anchor로
사전 고정하며, 더 잘 맞는 후속 관측으로 바꾸지 않는다. 고정 관측은 최적화에서
복제하지 않는다. 평가에서는 같은 set의 각 held-out gripper Event와 pair를 만든다.
최종 RMSE는 pair 성분→Event RMSE→set RMSE→set 동일가중 RMSE 순서로 집계해,
하나의 fixed anchor가 반복 참조된 횟수를 독립 fixed 촬영 수처럼 세지 않는다.

### 11.5 Shared-target reprojection — 보조 진단값

Board는 train-only Hand–Eye 초기 pose, cube는 train-only board-free FK artifact pose를 공유 reference로 넣어 fixed-camera reprojection을 계산한다. 관측 population은 동일하지만 reference 자체가 외부 GT가 아니며 특정 방법의 가정과 더 가까울 수 있다. 따라서 이 값은 과거 결과와의 연결 및 진단 목적으로만 남기고, 외부 GT 전 방법 순위나 공정성 주장의 근거로 사용하지 않는다.

### 11.6 Legacy `e_cross` — 고정카메라 간 cube pose consistency

각 고정카메라가 자기 영상 corner만으로 cube PnP를 독립 계산한다.

$$
T^{B,(i)}_{cube}=T^B_{C_i}T^{C_i}_{cube,\mathrm{PnP}}
$$

카메라 pair $(i,j)$에 대해 다음을 계산하고 모든 사전 고정 pair에서 RMSE를 낸다.

$$
e^{t}_{ij}=\|\mathbf t_i-\mathbf t_j\|_2\,[\mathrm{mm}]
$$

$$
e^{R}_{ij}=\left\|\log\left(R_i^TR_j\right)\right\|_2\,[\mathrm{deg}]
$$

이 mm는 재투영 오차를 mm로 변환한 값이 아니라 두 카메라가 예측한 cube 중심 사이의 실제 좌표 거리다. robot FK, gripper camera, nominal cube 정답, 외부 GT는 사용하지 않는다. 따라서 일반적인 고정카메라 간 위치/회전 일관성 지표이지만 절대 정확도는 아니다.

### 11.7 Legacy cube cross-view pixel transfer RMSE px

카메라 A의 측정 corner로 얻은 cube PnP pose를 추정된 카메라 상대관계로 B에 옮긴 뒤 B의 실제 측정 corner와 비교한다.

$$
T^{C_B}_{cube}
=\left(T^B_{C_B}\right)^{-1}T^B_{C_A}T^{C_A}_{cube,\mathrm{PnP}}
$$

A→B와 B→A를 모두 계산한다. FK나 공유 cube 정답 없이 카메라 간 상대 보정을 px 도메인에서 평가하는 지표다.

현재 Fixed-to-Fixed (고정카메라 간) 지표는 같은 계산을 보드와 큐브 모두에 수행한다. 이 절은 기존 Cube-only (큐브 전용) 출력 필드의 정의를 보존하기 위한 설명이다.

### 11.8 `e_e2e` — Eye-in-hand와 Eye-to-hand 경로 일치

같은 held-out cube에 대해 다음 두 경로를 비교한다.

- 고정카메라 경로: 여러 $T^B_{C_i}T^{C_i}_{\mathrm{cube},\mathrm{PnP}}$의 robust SE(3) 평균
- 손목카메라 경로: $T^B_G(e)T^G_C T^C_{\mathrm{cube},\mathrm{PnP}}$

translation mm와 SO(3) rotation deg RMSE를 보고한다. robot FK가 포함되므로 전체 변환 체인의 내부 일관성 지표이며 외부 GT 정확도는 아니다.

### 11.9 수렴·등록·안정성 지표

- `converged_runs / total_runs`: 여러 초기 seed 중 solver 성공 수
- `n_registered_fixed_cameras`: 모든 비교 조건을 만족하며 등록된 Fixed Camera (고정카메라) 수
- transform dispersion: seed 간 translation/rotation 표준편차와 최댓값
- Jacobian rank/condition number: 관측가능성과 수치 안정성 diagnostic

### 11.10 External-GT 절대 정확도

물리 정확도는 calibration과 분리된 blind 데이터에서만 평가한다.

1. 독립 tracker/CMM/jig로 $T^B_{\mathrm{cube,GT}}$를 측정하되 RGB calibration camera, controller FK, A4 factor, A5 alignment를 GT 생성에 사용하지 않는다.
2. `blind-predict`: GT를 읽지 않고 A2/A3/A4/A5의 $T^B_{\mathrm{cube}}$ 예측을 동일 `run_index=0`으로 저장하고 해시를 동결한다.
3. GT 잠금 해제 후 `external-gt`로 TRE mm, rotation deg, ADD-S, P95, failure rate를 paired hierarchical bootstrap으로 비교한다.

코드 최소는 독립 session 2개지만 최종 실험은 5 sessions×30 blind poses를 권장한다. primary contrast는 A4↔A2, secondary contrast는 A4↔A5로 GT 공개 전에 고정한다. 구체적인 측정계·strata·파일 형식·실행 명령은 [`protocol_templates/CAPTURE_CAMPAIGN_PROTOCOL.md`](protocol_templates/CAPTURE_CAMPAIGN_PROTOCOL.md), [`external_gt_pose_TEMPLATE.json`](protocol_templates/external_gt_pose_TEMPLATE.json), [`external_gt_eval_manifest_TEMPLATE.json`](protocol_templates/external_gt_eval_manifest_TEMPLATE.json)에 있다.

내부 `e_cross`/`e_e2e`가 좋아도 external-GT 오차가 반드시 작다는 뜻은 아니다.

### 11.11 조건별 지표 적용표

| 지표 | A0/B3 board-only row-local | Cube-bearing A1~A5/B1/B2 | Cross-target 전체 행 | Marker-system | 해석 |
| --- | --- | --- | --- | --- | --- |
| Train Reprojection (학습 재투영) px | board만 | 선언된 marker | 전체 행 동일 평가는 아님 | 각 시스템 marker | Fitting Diagnostic (적합 진단) |
| Own-marker held-out reprojection px | board만 | 선언된 marker | 해당 없음 | 각 시스템 marker | 같은 marker 조건 내 주 pixel 지표 |
| Fixed-to-Fixed Board Transfer/Pose (고정카메라 간 보드 전달/자세) | row-local과 무관 | row-local과 무관 | 전체 행 가능 | 전체 시스템 가능 | FK-free Board Relative Evaluation (FK 없는 보드 상대평가) |
| Fixed-to-Fixed Cube Transfer/Pose (고정카메라 간 큐브 전달/자세) | row-local과 무관 | row-local과 무관 | 전체 행 가능 | 전체 시스템 가능 | FK-free Cube Relative Evaluation (FK 없는 큐브 상대평가) |
| Gripper-to-Fixed Board/Cube Transfer/Pose (그리퍼카메라–고정카메라 간 보드/큐브 전달·자세) | row-local과 무관 | row-local과 무관 | 전체 행 가능 | 전체 시스템 가능 | Visual Full-chain Evaluation (영상 기반 전체 체인 평가); FK-dependent |
| Shared-target reprojection px | 해당 없음 | 해당 없음 | 전체 행 가능 | 전체 시스템 가능 | reference-dependent 보조 진단; 순위 금지 |
| FK-dependent `e_e2e` mm/deg | N/A | 가능 | 전체 행 가능 | 가능 | FK 포함 전체 경로 보조 지표 |
| External TRE/rotation | blind GT 필요 | blind GT 필요 | 해당 없음 | 해당 없음 | 절대 물리 정확도 |

## 12. 어떤 조건끼리 비교해야 하는가

| Research Question (연구 질문) | Comparison (비교) | Shared Evaluation Metric (동일하게 볼 지표) | Invalid Interpretation (피해야 할 해석) |
| --- | --- | --- | --- |
| 순차법에 cube residual이 도움이 되는가 | A0 → A1 | held-out board px, 등록 수 | 서로 다른 pooled overall 직접 비교 |
| vision 조건에서 통합 feedback이 도움이 되는가 | A1 → A2 | held-out overall/board/cube px | 초기값 차이로 설명 |
| soft FK 조건에서도 통합 효과가 있는가 | B1 → A4 | Held-out Same-marker (홀드아웃 동일 마커) px, Fixed-to-Fixed와 Gripper-to-Fixed | 서로 다른 FK Weight (FK 가중치) 사용 |
| hard FK 고정의 효과는 무엇인가 | A2 → A3 | 동일 Held-out Marker (홀드아웃 마커) px, Fixed-to-Fixed와 Gripper-to-Fixed | FK가 GT라고 단정 |
| soft FK factor의 효과는 무엇인가 | A2 → A4 | 동일 Held-out Marker px, Fixed-to-Fixed와 Gripper-to-Fixed | 실측 Covariance (공분산) 없을 때 최종 결론 |
| raw FK와 vision-aligned FK hard fixed의 차이는 무엇인가 | A3 ↔ A5 | 동일 Held-out Marker px, Fixed-to-Fixed와 Gripper-to-Fixed | A5를 독립 실측 correction으로 해석 |
| aligned FK의 hard fixed와 soft factor 중 무엇이 나은가 | A4 ↔ A5 | 동일 Held-out Marker px, 두 camera scope, 이후 external GT | 내부 px 하나로 물리 정확도 순위 결정 |
| Board Residual (보드 잔차)의 최적화 기여는 무엇인가 | B2 → A4 | Held-out Cube (홀드아웃 큐브) px, Fixed-to-Fixed와 Gripper-to-Fixed | Marker System End-to-End (마커 시스템 전체 성능) 주장 |
| Cube Residual (큐브 잔차)의 최적화 기여는 무엇인가 | B3 → A2 | Held-out Board (홀드아웃 보드) px, Fixed-to-Fixed와 Gripper-to-Fixed | B3 Row-local N/A (행 내부 미적용)를 0으로 간주 |
| 실제 Marker System (마커 시스템)으로 무엇이 좋은가 | marker-system의 3개 조건 | Board/Cube Fixed-to-Fixed와 Gripper-to-Fixed | Shared Target Pose Reprojection (공유 표적 자세 재투영)으로 순위 결정 |

핵심 원칙은 **같은 Observation Population (관측 모집단)을 쓰는 것만으로 충분하지 않고, Evaluation Scope (평가 범위)와 Dependency (의존성)를 분리해야 한다**는 것이다. Fixed-to-Fixed는 FK-free Fixed-camera Subsystem Metric (FK 없는 고정카메라 부분 지표), Gripper-to-Fixed는 FK-dependent Full-chain Metric (FK 의존 전체 체인 지표)로 함께 보고한다. Shared-target Reprojection (공유 표적 재투영)은 보조값으로만 본다.

## 13. 현재 session04 결과와 비교별 해석

Canonical 결과 인덱스는 [CP_result/README.md](CP_result/README.md), 상세 자동 생성 보고서는 [CP_result/session04/late_table1/TABLE1_RESULTS.md](CP_result/session04/late_table1/TABLE1_RESULTS.md)다. 결과 문서는 Fixed-to-Fixed (고정카메라 간) 및 set-anchor Gripper-to-Fixed (그리퍼카메라–고정카메라 간) Board/Cube (보드/큐브) 평가를 분리한다.

핵심 요약은 다음과 같다.

- 동일한 cube+board marker population에서 A1→A2 own held-out overall은 `4.0837 → 3.8901 px`로 감소해 Unified feedback의 기여를 지원한다.
- A3 raw-FK-fixed의 held-out cube는 `6.3958 px`로 증가했으므로 raw tool4/mechanical pose를 외부 GT처럼 취급하지 않는다.
- A2와 A4의 own held-out overall은 `3.8901`, `3.8899 px`로 사실상 동일하다. A4/B1/B2는 Simulation Prior 기반 preflight이므로 corrected-FK factor의 우월성을 주장하지 않는다.
- A5의 own held-out overall은 `3.7270 px`로 A4보다 낮다. 그러나 Fixed-to-Fixed cube는 `3.4706 px`로 개선되는 반면 board는 `4.8563 px`로 A4의 `3.1156 px`보다 악화되어 내부 지표에서도 일관된 승자가 아니다. 또한 train 영상으로 만든 aligned FK를 hard-fixed한 post-hoc 진단이므로 이전 A3 성능의 원인을 설명할 뿐 실제 물리 정확도 우월성을 입증하지 않는다.
- 따라서 현재 확증 대표 행은 A2, 방법론적 확장 후보는 A4, 원인 진단은 A5다. A4/A5의 실제 순위는 blind external GT 이후 결정한다.
- 모든 현재 수치는 동일 marker population 내부 held-out reprojection 또는 내부 경로 일관성이다. External GT 전에는 절대 정확도 순위를 만들지 않는다.

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

- `shared_train_only_baseline.json`: Split (분할), Solver (최적화기), Observation Loader (관측 로더), Shared/Row-specific Initial State (동일/행별 초기 상태), Meta/Intrinsics/Implementation (메타/내부 파라미터/구현 파일) 및 Train/Held-out Observation Population SHA-256
- `shared_board_free_fk_cube.json`: board/held-out 미사용 FK–cube alignment provenance
- `table1_methods.json`: 각 행·seed의 transforms, train/test px, path metrics, solver/Jacobian diagnostics
- `table1_results.csv`: 논문/HTML용 요약 숫자의 canonical table
- `TABLE1_RESULTS.md`: 교수님 피드백, 결과표, 지표 해석 문서

기본 실행은 3개 Initial States (초기값)를 사용한다. Seed 0 (시드 0)은 Shared Baseline (동일 초기값) 그대로이고, 나머지 seed는 각 자유 transform에 결정론적인 5 mm/1° Perturbation (교란)을 준다. `table1_methods.json`에는 각 run의 원값을 보존하고, CSV/표에는 Converged Runs (수렴 실행 수)와 Metric Mean (지표 평균)을 요약한다. 따라서 한 번의 우연한 초기값에서 얻은 숫자만 보고하지 않는다.

### 14.2 보조 평가 출력

```text
CP_result/sessionNN/
├── cross_target_evaluation/
│   ├── cross_target_evaluation.json
│   └── cross_target_evaluation.csv
├── marker_system_end_to_end/
│   ├── marker_system_end_to_end.json
│   └── marker_system_end_to_end.csv
├── opencv_relative_baseline/
│   ├── opencv_relative_baseline.json
│   ├── opencv_relative_baseline.csv
│   └── OPENCV_RELATIVE_BASELINE.md
└── outlier_ablation/
    ├── OUTLIER_LOSS_ABLATION.md
    ├── HARD_REJECTION_ABLATION.md
    ├── outlier_loss_ablation.csv
    ├── hard_rejection_ablation.csv
    ├── hard_rejection_ablation.json
    ├── linear_table1/
    ├── linear_cross_target/
    └── strict_table1/
```

### 14.3 시각화 동기화

```text
table1_methods.json ─┐
cross_target CSV ────┼─> tools/sync_table1_canonical_data.py
marker-system CSV ───┘             │
                                   ├─> table1_results.csv
                                   └─> TABLE1_INTERACTIVE.html

table1_results.csv + 2 evaluation CSV + HTML + MD
        └─> tools/verify_table1_visual_sync.py
```

Markdown/HTML은 새 JSON/CSV만 입력으로 사용해 다시 생성한다. 이전 계산식에서 만든 Derived Results (파생 결과)는 유지하지 않는다. Fixed-to-Fixed (고정카메라 간)와 Gripper-to-Fixed (그리퍼카메라–고정카메라 간) 계산은 `tools/verify_camera_scope_evaluation.py`의 Synthetic Contract Test (합성 계약 검증)로 확인한다.

## 15. 코드 구조

| 코드 | 역할 |
| --- | --- |
| `01_...py`~`06_...py` | 사용자가 순서대로 실행하는 calibration 전용 root CLI 진입점 |
| `capture_pipeline/` | intrinsic 내보내기·ChArUco 보정·동기 촬영 구현 |
| `calibration_pipeline/filter_observations.py` | 촬영 후 전체 재검출, frozen-corner manifest와 재촬영 후보 생성 |
| `calibration_pipeline/schema.py` | A/B 조건, 자유변수, 공정 비교 계약 |
| `calibration_pipeline/table1.py` | split, baseline, A/B 실행, raw result 저장 |
| `calibration_pipeline/report.py` | 05 결과의 수렴·오차·prune 결정과 전체 행렬 보고서 생성 |
| `calibration_pipeline/observations.py` | board/cube raw pixel observation 구성 |
| `calibration_pipeline/se3.py` | PnP pose의 robust 평균과 meta/FK 로딩 |
| `calibration_pipeline/fk_alignment.py` | board-free train-only FK–cube alignment |
| `calibration_pipeline/reprojection.py` | 모든 조건에서 동일하게 쓰는 Raw-corner Pixel Solver (원시 코너 픽셀 최적화기) |
| `calibration_pipeline/fk_factor.py` | covariance-whitened corrected-FK factor |
| `calibration_pipeline/evaluation.py` | Camera-scope/Reference-dependent Internal Evaluation (카메라 범위/기준 의존 내부 평가)의 명시적 분리 |
| `calibration_pipeline/path_evaluation.py` | Fixed-to-Fixed, Gripper-to-Fixed Board/Cube Cross-view (보드/큐브 교차 시점)와 Legacy Path (기존 경로) |
| `calibration_pipeline/cross_target.py` | 모든 Table 1 transform의 두 camera scope 재평가 |
| `calibration_pipeline/marker_system.py` | modality별 초기화부터 수행하는 end-to-end 비교 |
| `calibration_pipeline/opencv_relative_baseline.py` | OpenCV PnP 기반 FK-free fixed-camera reference baseline |
| `calibration_pipeline/blind_prediction.py` | GT-blind pose prediction |
| `calibration_pipeline/external_gt.py` | 독립 GT 통계 평가 |
| `calibration_pipeline/task_trial.py` | Paired peg-in-hole/grasp success·접촉 오차 평가 |
| `tools/sync_table1_canonical_data.py` | JSON/CSV에서 결과 CSV와 HTML 동기화 |
| `tools/verify_table1_visual_sync.py` | CSV·MD·HTML 숫자 및 계약 검증 |
| `tools/verify_e_cross_definition.py` | 표준 fixed-camera consistency 독립 재계산 |
| `tools/verify_camera_scope_evaluation.py` | 두 Camera Scope (카메라 범위) 계약, Hand–Eye 민감도와 Planar PnP (평면 PnP) 검증 |
| `tools/summarize_outlier_ablation.py` | 동일 관측에서 soft-L1과 linear loss 결과 비교 |
| `tools/summarize_hard_rejection_ablation.py` | standard/strict 사전 관측 제외를 동일 held-out에서 비교 |

## 16. 전체 실행 명령

```bash
# 선택: Shared Baseline (동일 초기값)만 먼저 생성
python3 05_calibrate.py \
  --root_folder data/sessionNN/calib_train \
  --intrinsics_dir intrinsics \
  --include_sets 0-12 \
  --split_seed 20260731 \
  --observation-manifest data/sessionNN/calib_out/capture_filter/Step2b_observation_manifest.json \
  --out_dir CP_result/sessionNN/late_table1 \
  --baseline_only

# A0~A5/B1~B3 실행, A6 baseline 예약
python3 05_calibrate.py \
  --root_folder data/sessionNN/calib_train \
  --intrinsics_dir intrinsics \
  --include_sets 0-12 \
  --split_seed 20260731 \
  --num_inits 3 \
  --observation-manifest data/sessionNN/calib_out/capture_filter/Step2b_observation_manifest.json \
  --out_dir CP_result/sessionNN/late_table1

# 상세 calibration 결과와 모든 행렬 출력
python3 06_make_report.py \
  --root_folder data/sessionNN/calib_train \
  --table1 CP_result/sessionNN/late_table1/table1_methods.json \
  --out_dir CP_result/sessionNN/late_table1

# 선택 평가: 저장된 모든 방법의 외부-GT 전 board/cube 내부 평가
python3 tools/evaluate_cross_target.py \
  --root_folder data/sessionNN/calib_train \
  --intrinsics_dir intrinsics \
  --include_sets 0-12 \
  --split_seed 20260731 \
  --observation-manifest data/sessionNN/calib_out/capture_filter/Step2b_observation_manifest.json \
  --table1_result CP_result/sessionNN/late_table1/table1_methods.json \
  --out_dir CP_result/sessionNN/cross_target_evaluation

# 선택 평가: marker modality별 end-to-end 비교
python3 tools/compare_markers.py \
  --root_folder data/sessionNN/calib_train \
  --intrinsics_dir intrinsics \
  --include_sets 0-12 \
  --split_seed 20260731 \
  --observation-manifest data/sessionNN/calib_out/capture_filter/Step2b_observation_manifest.json \
  --out_dir CP_result/sessionNN/marker_system_end_to_end

# 선택 평가: OpenCV PnP 독립 FK-free relative-pose 기준선
python3 tools/opencv_baseline.py \
  --root_folder data/sessionNN/calib_train \
  --intrinsics_dir intrinsics \
  --include_sets 0-12 \
  --split_seed 20260731 \
  --observation-manifest data/sessionNN/calib_out/capture_filter/Step2b_observation_manifest.json \
  --out_dir CP_result/sessionNN/opencv_relative_baseline

# 독립 robot task trial 수집 후 success/contact-error 평가
python3 -m calibration_pipeline.task_trial \
  --manifest protocol_templates/robot_task_trial_manifest_<날짜>.json \
  --output_dir CP_result/sessionNN/robot_task_trial

# Outlier soft-weighting 대조: 같은 관측을 linear loss로 재실행
python3 05_calibrate.py \
  --root_folder data/sessionNN/calib_train \
  --intrinsics_dir intrinsics \
  --include_sets 0-12 \
  --split_seed 20260731 \
  --num_inits 3 \
  --loss linear \
  --observation-manifest data/sessionNN/calib_out/capture_filter/Step2b_observation_manifest.json \
  --out_dir CP_result/sessionNN/outlier_ablation/linear_table1
python3 tools/evaluate_cross_target.py \
  --root_folder data/sessionNN/calib_train \
  --intrinsics_dir intrinsics \
  --include_sets 0-12 \
  --split_seed 20260731 \
  --loss linear \
  --observation-manifest data/sessionNN/calib_out/capture_filter/Step2b_observation_manifest.json \
  --table1_result CP_result/sessionNN/outlier_ablation/linear_table1/table1_methods.json \
  --out_dir CP_result/sessionNN/outlier_ablation/linear_cross_target
python3 tools/summarize_outlier_ablation.py

# Hard rejection 민감도: strict 관측 정책으로 재보정 후 동일 held-out 비교
python3 05_calibrate.py \
  --root_folder data/sessionNN/calib_train \
  --intrinsics_dir intrinsics \
  --include_sets 0-12 \
  --split_seed 20260731 \
  --num_inits 3 \
  --observation-manifest data/sessionNN/calib_out/capture_filter/Step2b_observation_manifest.json \
  --observation-filter-policy strict \
  --out_dir CP_result/sessionNN/outlier_ablation/strict_table1
python3 tools/summarize_hard_rejection_ablation.py

# 선택 확장 평가의 기존 통합 Markdown/HTML 재생성 및 동기화 검증
python3 tools/sync_table1_canonical_data.py
python3 tools/verify_table1_visual_sync.py
# 계산 계약 검증
python3 tools/verify_e_cross_definition.py
python3 tools/verify_camera_scope_evaluation.py
```

현재 canonical session04 결과는 다음에서 확인한다.

- [Session04 결과 인덱스](CP_result/README.md)
- [상세 calibration 결과와 대표 행렬](CP_result/session04/late_table1/CALIBRATION_RESULTS.md)
- [Table 1 결과 및 평가 계약](CP_result/session04/late_table1/TABLE1_RESULTS.md)
- [Interactive 결과](CP_result/session04/late_table1/TABLE1_INTERACTIVE.html)
- [OpenCV FK-free reference baseline](CP_result/session04/opencv_relative_baseline/OPENCV_RELATIVE_BASELINE.md)
- [Soft-L1 vs linear outlier loss 대조](CP_result/session04/outlier_ablation/OUTLIER_LOSS_ABLATION.md)
- [Standard vs strict hard-rejection 민감도](CP_result/session04/outlier_ablation/HARD_REJECTION_ABLATION.md)
- [Corner refinement와 weighting 대조](CP_result/session04/corner_weighting_ablation/CORNER_WEIGHTING_ABLATION.md)
- [순차/통합 및 FK factor 수식 상세](CALIBRATION_EXPLANATION_LATEX.md)
- [Simulation backend migration 계획](Simulation/MIGRATION_PLAN.md)

## 17. 현재 해석 한계

1. A3는 raw FK와 mechanical frame map만 hard constraint로 사용한다. 눈금 cube jig의 반복 파지 실측 없이 FK를 정답으로 주장할 수 없다.
2. A4/B1/B2는 실측 covariance 파일이 없으면 Simulation prior를 사용한 preflight다. 최종 corrected-FK 주장에는 preregistered physical covariance가 필요하다.
3. A5는 train vision으로 정렬한 FK를 hard-fixed한 post-hoc 진단이다. 낮은 내부 px를 독립 보정이나 물리 정확도 우월성으로 주장하지 않는다.
4. Fixed-to-Fixed (고정카메라 간)와 Gripper-to-Fixed (그리퍼카메라–고정카메라 간) Board/Cube Cross-view (보드/큐브 교차 시점)는 외부 GT 전 내부 지표일 뿐 External Absolute Accuracy (외부 절대 정확도)가 아니다.
5. event holdout pixel 평가는 관측 일반화 검증이며 새로운 작업 위치 전체에 대한 물리 정확도를 직접 보장하지 않는다.
6. B2/B3 shared-baseline ablation과 marker-system end-to-end 비교는 연구 질문이 다르므로 같은 주장으로 섞지 않는다.
7. A6는 독립 correction label과 적용 규칙이 확정되기 전까지 실행하지 않는다.
8. Shared train-only target pose reprojection은 reference-dependent 보조 진단값이며, 외부 GT 전 공정한 방법 순위에 사용하지 않는다.
9. Session02에서 board 기반 상대 자세와 cube 기반 상대 자세가 camera 1에서 21.9 mm, camera 3에서 19.5 mm 다르다. FK 실험보다 먼저 cube geometry, corner ordering, target별 PnP 편향을 확인해야 한다.

직접 수행해야 하는 다음 실험은 다음과 같다.

1. 5개 독립 camera-installation session에서 session당 30개의 새 cube pose를 모든 카메라로 동시에 blind capture한다.
2. tracker/CMM/6-DoF kinematic jig로 robot base와 cube pose를 독립 측정하고 반복성으로 GT uncertainty floor를 먼저 결정한다.
3. 별도 세션에서 peg-in-hole 또는 grasp success/접촉 위치 오차를 측정한다. 이때 인식 알고리즘과 target은 모든 방법에 고정한다.
4. 후속 단계에서 MATLAB/기존 multiview toolbox 또는 COLMAP을 동일 원본과 동일 held-out split으로 별도 실행한다. 현재 캘리브레이션 완성 마일스톤에는 포함하지 않는다.

## 18. Terminology (용어 설명)

- **Fixed-to-Fixed (고정카메라 간)**: 두 고정카메라가 같은 표적을 관측했을 때의 상대 일관성 평가. Robot FK (로봇 순기구학)를 사용하지 않는다.
- **Gripper-to-Fixed (그리퍼카메라–고정카메라 간)**: 움직이는 그리퍼카메라와 고정카메라의 동일 표적 관측을 비교하는 전체 체인 평가. 영상 코너를 사용하지만 Robot FK와 Hand–Eye가 예측 경로에 포함된다.
- **Reference-dependent Reprojection (기준 의존 재투영)**: 학습 데이터로 만든 표적 자세를 기준으로 사용하는 보조 진단. External GT (외부 정답)가 아니므로 방법 순위의 근거로 쓰지 않는다.
- **PnP (3D–2D 자세 추정)**: 알려진 3D 표적점과 검출된 2D 영상점을 이용해 카메라–표적 자세를 계산하는 방법.
- **FK, Forward Kinematics (순기구학)**: 로봇 관절 상태로부터 Base-to-Gripper Transform (베이스–그리퍼 변환)을 계산하는 과정.
- **Hand–Eye Transform (핸드–아이 변환)**: Gripper-to-Camera Transform (그리퍼–카메라 변환) $T^G_C$.
- **RMSE, Root Mean Squared Error (평균제곱근오차)**: 잔차 제곱 평균의 제곱근. px, mm, deg는 서로 다른 물리량이므로 합쳐 순위를 만들지 않는다.
