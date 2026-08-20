# Table 1 비교실험 결과와 교수님 피드백 반영

이 문서는 session02의 canonical 보고서다. 표의 숫자는 `table1_results.csv`, 공통 평가는 `cross_target_evaluation.csv`, 마커 시스템 비교는 `marker_system_end_to_end.csv`에서 생성된다. 인터랙티브 HTML도 같은 CSV를 읽어 동기화하므로 Markdown과 HTML의 데이터 원천은 동일하다.

## 목차

- [핵심 수정 사항](#핵심-수정-사항)
- [재투영 오차를 px만 남긴 이유](#재투영-오차를-px만-남긴-이유)
- [Main Table 전체 결과](#main-table-전체-결과)
- [각 평가지표의 적용 가능성과 한계](#각-평가지표의-적용-가능성과-한계)
- [`e_cross` 정의와 검증](#e_cross-정의와-검증)
- [eih와 e2h의 독립·통합 최적화 로직](#eih와-e2h의-독립통합-최적화-로직)
- [아웃라이어와 동일 평가 기준](#아웃라이어와-동일-평가-기준)
- [Baseline 통일과 Step3 동일성](#baseline-통일과-step3-동일성)
- [End-to-end marker-system 비교](#end-to-end-marker-system-비교)
- [FK 실측 후 판단](#fk-실측-후-판단)
- [현재 코드와 산출물의 역할](#현재-코드와-산출물의-역할)
- [Canonical 파일](#canonical-파일)

## 핵심 수정 사항

1. 재투영 오차의 `mm` 환산값을 코드·JSON·CSV·Markdown·HTML에서 삭제했다. 재투영은 카메라 영상에서 직접 관측되는 `px`로만 보고한다.
2. 교수님이 요청한 카메라 간 픽셀 검증을 추가했다. 카메라 A의 측정 코너만으로 얻은 큐브 PnP pose를 A–B 보정관계로 카메라 B에 옮겨 B의 실제 코너와 비교하며, A→B와 B→A를 모두 평가한다.
3. `e_cross`를 FK 정답 오차가 아니라 일반적인 고정카메라 간 동일 물체 pose consistency로 재정의하고, FK·손목카메라·공유 target pose·외부 GT가 코드 경로에 없음을 독립 검증했다.
4. `Run_calibration_comparison.py table1`이 `calibration_pipeline/table1.py`의 단일 구현으로 A0~A4/B1~B3와 A5 예약 initial state를 포함한 train-only baseline을 생성한다. 두 번째 FK runner, A3 Step3 우회 호출, 분리된 core/final 결과 JSON을 삭제했다.
5. B1/B2의 스키마를 실제 코드와 동일한 corrected-FK(soft factor) 조건으로 수정했다. B1↔A4와 B2↔A4가 동일 FK factor를 공유하는 공정 비교다.
6. 모든 실행 결과는 동일 split, 원시 검출, `K,D`, solver budget, seed 및 공통 평가 mask를 사용한다. 전체 데이터 적합 결과와 train/test 결과를 같은 열에서 비교하지 않는다.

## 재투영 오차를 px만 남긴 이유

기존 `rmse_image_plane_mm`는 측정된 3차원 오차가 아니었다. 픽셀 잔차를 정규화한 뒤 **각 방법이 예측한 코너 깊이**를 곱해 영상면 길이처럼 환산한 값이었다. 따라서 같은 픽셀 오차도 카메라 초점거리와 예측 깊이에 따라 다른 mm가 되고, 서로 다른 calibration 결과가 단위 환산 기준까지 바꾼다. 깊이 방향 오차도 포함하지 않으므로 3D 위치 RMSE라고 부를 수 없다.

재투영의 직접 관측량은

\[
e_{\mathrm{reproj}}^{\mathrm{px}}
=\sqrt{\frac{1}{2N}\sum_{n=1}^{N}\left((\hat u_n-u_n)^2+(\hat v_n-v_n)^2\right)}
\]

이며, native distorted image에서 고정된 카메라별 `K_i,D_i`로 계산한다. 평가 중 parameter refit이나 결과 의존적 관측 제거는 없다. 물리 단위의 정확도가 필요하면 예측 깊이 환산이 아니라 독립 측정한 3D GT로 별도 평가해야 한다.

## Main Table 전체 결과

각 숫자는 3개 초기화의 평균이다. Train/Test의 표기 순서는 `전체 / 보드 / 큐브`이고, `e_cross`, `e_e2e`는 `mm / deg`다. A4/B1/B2는 실측 FK covariance가 아니라 사전 고정한 Simulation prior를 사용한 preflight 결과다.

| ID | 구성 | 상태 | 수렴 / 등록 | Train 재투영 px<br>전체 / 보드 / 큐브 | Test 재투영 px<br>전체 / 보드 / 큐브 | 카메라 간 픽셀 전이 px | `e_cross`<br>mm / deg | `e_e2e`<br>mm / deg | 해석 |
| --- | --- | --- | --- | --- | --- | ---: | --- | --- | --- |
| A0 | 보드 · 순차 · vision | complete | 3/3 · 3대 | 3.5745 / 3.5745 / — | 2.4829 / 2.4829 / — | 11.3422 | 14.0135 / 1.1060 | 4.9059 / 1.2229 | core |
| A1 | 큐브+보드 · 순차 · vision | complete | 3/3 · 3대 | 3.3980 / 3.5657 / 2.7210 | 3.1267 / 2.5103 / 4.5317 | 10.9179 | 13.7546 / 1.2193 | 4.2459 / 1.0239 | core |
| A2 | 큐브+보드 · 통합 · vision | complete | 3/3 · 3대 | 3.3981 / 3.5699 / 2.7020 | 3.1155 / 2.5038 / 4.5113 | 10.5265 | 13.1259 / 1.1910 | 4.0685 / 1.0385 | vision |
| A3 | 큐브+보드 · 통합 · FK-fixed | complete | 3/3 · 3대 | 3.6477 / 3.6063 / 3.7911 | 2.4996 / 2.4951 / 2.5136 | **9.0609** | **10.8317 / 1.0601** | 4.7905 / 1.1893 | hard FK |
| A4 | 큐브+보드 · 통합 · corrected-FK | preflight | 3/3 · 3대 | 3.3973 / 3.5703 / 2.6958 | 3.0959 / 2.5038 / 4.4556 | 10.4774 | 13.0413 / 1.1899 | 4.0330 / 1.0313 | soft FK |
| A5 | FK correction · 독립 실측 label | not run | 0/0 · — | — / — / — | — / — / — | — | — / — | — / — | 독립 label 없음 |
| B1 | 큐브+보드 · 독립 · corrected-FK | preflight | 3/3 · 3대 | 3.3967 / 3.5661 / 2.7115 | 3.1057 / 2.5100 / 4.4724 | 10.8253 | 13.6109 / 1.2157 | 4.1987 / 1.0184 | A4와 FK factor 동일 |
| B2 | 큐브 · 통합 · corrected-FK | preflight | 3/3 · 3대 | 2.6293 / — / 2.6293 | 4.4759 / — / 4.4759 | 9.2952 | 11.9161 / 1.2866 | **3.8215 / 0.9311** | A4와 FK factor 동일 |
| B3 | 보드 · 통합 · vision | complete | 3/3 · 3대 | 3.5745 / 3.5745 / — | 2.4829 / 2.4829 / — | 11.3398 | 14.0102 / 1.1062 | 4.9051 / 1.2225 | core |

현재 내부 지표에서는 A3가 카메라 간 픽셀 전이와 `e_cross`가 가장 작다. 그러나 B2는 `e_e2e`가 가장 작고, A0/B3는 자체 보드 Test가 더 작다. 따라서 외부 GT 없이 “A3가 절대 3D 정확도에서 최종 우수하다”고 결론 내릴 수는 없다. 특히 A4/B1/B2는 실측 covariance 전의 preflight다.

## 각 평가지표의 적용 가능성과 한계

| 지표 | 모든 A0–B3에 동일하게 계산 가능한가? | 타당한 사용 | 제한 |
| --- | --- | --- | --- |
| Train 재투영 RMSE px | 각 방법이 학습에 사용한 marker에만 가능 | solver 적합·수렴 진단 | 학습 표적 집합이 달라 전체 방법 순위에 사용 불가 |
| 자체 Test 재투영 RMSE px | 각 방법이 사용한 marker에만 가능 | 같은 표적 구성 안의 holdout 일반화 비교 | A0/B3의 overall과 B2의 overall은 관측 집합이 달라 직접 순위 비교 불가 |
| 공통 target 재투영 RMSE px | 가능: 모든 방법에 같은 18 observations/338 corners | 동결 calibration의 공통 내부 transfer 비교 | train-only 공유 target pose가 내부 기준이며 외부 GT가 아님 |
| 카메라 간 양방향 픽셀 전이 RMSE px | 가능: 9 pair/18 direction 공통 mask | **모든 방법의 1차 공통 inter-camera pixel 지표** | source PnP 노이즈를 포함하며 절대 3D 정확도는 아님 |
| `e_cross` 병진/회전 | 가능: 같은 9 fixed-camera pairs | 고정카메라 간 동일 큐브의 3D pose consistency | 모든 카메라가 함께 틀리는 common-mode 오차를 검출하지 못함 |
| `e_e2e` 병진/회전 | 가능: 같은 3 event units | 고정카메라 경로와 손목카메라+robot FK 경로의 closure | FK와 hand-eye를 포함하므로 독립 GT가 아니며 FK 계열 방법에 유리할 수 있음 |
| 등록 카메라 수 `N_reg` | 가능 | 실패·coverage 확인 | 이번 실행은 모두 3대로 같아 정확도 순위를 구분하지 못함 |
| 외부 3D GT 오차 | 현재 불가 | 최종 절대 정확도 및 FK-fixed 채택 판단 | 실측 지그/독립 GT label 취득 후에만 가능 |

교수님 피드백에 가장 직접 대응하는 공통 픽셀 지표는 “카메라 간 양방향 픽셀 전이”다. 공통 target 재투영은 모든 방법에 같은 target pose를 넣는 평가라 비교 가능하지만, 그 pose 자체가 내부 train-only 기준이다. 두 값을 구분해 보고한다.

## `e_cross` 정의와 검증

본 보고서의 `e_cross`는 **일반적인 고정카메라 간 동일 물체 pose consistency와 동일**하다. 동일 held-out event에서 카메라 `i`가 영상 관측만으로 PnP한 큐브 pose를

\[
{}^{B}\!T_{O}^{(i)} = {}^{B}\!T_{C_i}\,{}^{C_i}\!T_{O,\mathrm{PnP}}
\]

로 base frame에 옮기고, 모든 사전 고정된 카메라 쌍 `(i,j)`에 대해

\[
e_{\mathrm{cross},t}
=\sqrt{\operatorname{mean}_{(i,j)}
\left\|t({}^{B}\!T_O^{(i)})-t({}^{B}\!T_O^{(j)})\right\|_2^2}
\]

를 계산한다. 회전은 같은 쌍의 SO(3) geodesic angle RMSE다. 이 경로에는 robot FK, gripper camera, nominal/FK cube 위치, 공유 target pose, 외부 GT가 없다. 공통 base frame을 임의의 rigid transform으로 바꿔도 값이 변하지 않음을 검사했다.

카메라 간 픽셀 전이는 별도로

\[
{}^{C_j}\!T_O^{(i\rightarrow j)}
=({}^{B}\!T_{C_j})^{-1}
{}^{B}\!T_{C_i}
{}^{C_i}\!T_{O,\mathrm{PnP}}
\]

를 만든 뒤 `j` 영상의 실제 큐브 코너에 재투영한다. `(i→j, j→i)` 18방향의 `(Δu,Δv)`를 모두 pooling한 component-wise RMSE가 표의 px 값이다. 독립 검증 결과 24 runs, 216 pairs가 canonical JSON과 `1e-9` px 이내에서 일치했다.

`e_cross`는 이 목적함수에 들어가지 않는 **평가 전용 지표**다. 통합 최적화가 `e_cross`를 직접 최소화해 유리해지는 구조가 아니다.

## eih와 e2h의 독립/통합 최적화 로직

좌표계는 base `B`, gripper `G`, gripper camera `C_g`, fixed camera `C_i`, target `O`로 둔다. 이벤트 `e`의 robot FK `{}^B T_G(e)`와 카메라별 내부 파라미터 `K_i,D_i`는 상수다. 관측 코너 `X_n^O`의 픽셀 잔차는

\[
r_{i,e,n}(\theta)=
\pi\!\left(K_i,D_i,
({}^{B}\!T_{C_i}(e))^{-1}
{}^{B}\!T_O X_n^O\right)-u_{i,e,n},
\]

\[
{}^{B}\!T_{C_i}(e)=
\begin{cases}
{}^{B}\!T_{C_i}, & i\text{가 fixed camera}\;\;(e2h),\\
{}^{B}\!T_G(e)\,{}^{G}\!T_{C_g}, & i=C_g\;\;(eih).
\end{cases}
\]

### 순차·독립(A0/A1/B1)

1단계는 eih 관측만 사용한다.

\[
(\hat T_{G C_g},\hat T_O)
=\arg\min_{T_{G C_g},T_O}
\sum_{(g,e,n)\in\mathcal O_{eih}}
\rho\!\left(\|r_{g,e,n}\|^2\right)
+E_{FK}\quad(B1만)
\]

그 다음 `T_GCg`와 target pose를 완전히 동결한다. 2단계는 fixed camera별로 서로 독립인 문제를 푼다.

\[
\hat T_{B C_i}
=\arg\min_{T_{B C_i}}
\sum_{(i,e,n)\in\mathcal O_{e2h,i}}
\rho\!\left(\|r_{i,e,n}
(T_{B C_i};\hat T_{G C_g},\hat T_O)\|^2\right).
\]

카메라별 블록 사이에 공유 자유변수가 없고, e2h 결과가 eih 변수로 되돌아가는 feedback/alternating pass도 없다.

### 통합(A2/A3/A4/B2/B3)

eih와 모든 e2h raw-corner residual을 **하나의 residual vector**에 쌓아 한 번에 푼다.

\[
\hat\theta=
\arg\min_{\theta}
\left[
\sum_{\mathcal O_{eih}}\rho(\|r_{g,e,n}(\theta)\|^2)
+\sum_i\sum_{\mathcal O_{e2h,i}}\rho(\|r_{i,e,n}(\theta)\|^2)
+E_{FK}(\theta)
\right].
\]

`θ`에는 조건에 따라 모든 `T_BCi`, `T_GCg`, board pose, set별 cube pose가 포함된다. 공유 target pose와 hand-eye가 eih/e2h 경로를 결합하므로 한 카메라의 잔차가 공유 변수와 다른 카메라 extrinsic을 함께 갱신한다.

- A2: cube pose 자유변수, `E_FK=0`.
- A3: cube pose를 train-only FK artifact에 고정하므로 cube 변수 없음.
- A4: cube pose 자유변수이며 preregistered covariance로 whitening한 SE(3) FK residual을 robust soft factor로 추가.
- B2: board residual/변수만 제거하고 A4와 같은 FK factor 유지.
- B3: cube residual/변수와 FK factor 제거.

모든 시각 residual에는 동일 `soft_l1`, `f_scale=2 px`, `max_nfev=300`, tolerance와 local SE(3) parameterization을 사용한다. `K,D`는 카메라별 사전 calibration 상수이며 최적화 변수가 아니다.

## 아웃라이어와 동일 평가 기준

- BA 단계에서 homography/RANSAC으로 결과에 따라 corner를 다시 삭제하지 않는다. 모든 방법이 같은 사전 고정 observation mask를 사용한다.
- ChArUco 최소 4 corners, fixed cube 최소 8 corners, gripper cube aspect gate가 frontend mask를 만든다.
- PnP RANSAC/pose solve는 초기화 또는 평가용 측정 pose 생성에만 쓰며, BA raw-corner residual 자체를 대체하지 않는다.
- 평가 시 model-dependent depth/pose threshold가 없고 `n_output_rejected=0`이다.
- 기존 10배 차이 문제의 원인이던 full-data 결과와 train/test 결과의 혼합 표시는 제거했다. 현재 표는 고정된 event-grouped, set-stratified holdout만 사용한다.

## Baseline 통일과 Step3 동일성

`calibration_pipeline/table1.py`가 held-out을 보지 않고 단 하나의 `shared_train_only_baseline.json`을 만든 뒤 A0~A4/B1~B3를 한 runner에서 실행한다. 실행 전인 A5도 A4와 byte-identical한 pending initial state로 같은 artifact에 포함한다. split, reference state, observation mask, `K,D`, solver option, seed가 고정되고 각 행에서는 marker/FK/freeze 처리만 바뀐다. `Step3_calibration.py`도 같은 모듈의 baseline-only 경로를 호출하므로 별도 초기화 경로가 없다.

- baseline artifact 내부 SHA-256: `ab6d642159bd2eec61f573c394b23b7df8578e687825239ada8bf852be0ca949`
- shared reference state SHA-256: `7ad783925bd57c0eef257fce2cbed7c533b7d626ad51173bf80a93bc7dbb51d0`
- board-free FK artifact file SHA-256: `18ca790a65626b24160e99288b07ad72d2a2b8a58b8e84ccd4a9c7a0860c6b0b`
- intrinsics `cam1.npz` file SHA-256: `8c1fbe08d7fb862d503d06d91d63c153bfb4f04cc6751aedcc9d880714e60ca3`

A3도 다른 통합 조건과 동일하게 `calibration_pipeline.reprojection.solve_corner_reprojection`을 직접 호출한다. 차이는 train-only FK cube pose를 state에 넣고 cube variable을 freeze mask에서 제외한다는 것뿐이다. Step3는 최종 A3를 다시 풀지 않고 동일 baseline 준비만 수행한다.

동일 baseline을 사용하는 A0–B3는 “같은 초기조건에서 최적화 구성요소를 제거한 ablation”이다. 특히 A0/B3/B2를 실제 marker-only 시스템 성능으로 과대해석하지 않는다. 실제 시스템 비교는 다음처럼 modality별 초기화부터 분리했다.

## End-to-end marker-system 비교

| 시스템 | 초기화 → 목적함수 | 수렴 / 등록 | 자체 Test px | 공통 전체 / 보드 / 큐브 px | 카메라 간 픽셀 전이 px | `e_cross` mm / deg | `e_e2e` mm / deg |
| --- | --- | --- | ---: | --- | ---: | --- | --- |
| board_only | board → board | 3/3 · 3대 | 2.4829 | 1.8083 / 1.8970 / 1.6032 | 11.3397 | 14.0100 / 1.1062 | 4.9044 / 1.2222 |
| cube_only | cube → cube | 3/3 · 3대 | 4.5346 | 2.5710 / 2.3583 / 2.9739 | **9.4462** | **12.1463 / 1.2721** | **3.9188 / 0.9427** |
| board_cube | board+cube → board+cube | 3/3 · 3대 | 3.1155 | 1.9108 / 1.8851 / 1.9645 | 10.5265 | 13.1259 / 1.1910 | 4.0685 / 1.0385 |

이 내부 데이터에서는 board-only가 공통 target px, cube-only가 inter-camera pixel transfer와 경로 병진에서 각각 유리하다. 한 시스템이 모든 지표를 지배하지 않으며 외부 GT 결론도 아니다.

## FK 실측 후 판단

눈금 큐브 지그로 반복 파지 위치의 물리 편차를 독립 측정한다. 오차가 충분히 작으면 A3(FK-fixed)를 후보로 유지하고, 오차가 크면 실측 repeatability로 covariance/correction을 preregister한 뒤 A4/A5를 재실행한다. 현재 A5는 독립 6-DoF correction label이 없어 실행하지 않았고, A4/B1/B2는 Simulation prior라 최종 주장에 사용하지 않는다.

## 현재 코드와 산출물의 역할

| 파일 | 역할 |
| --- | --- |
| `Step3_calibration.py` | 비교실험 전 공통 train-only baseline 준비용 호환 진입점 |
| `Run_calibration_comparison.py` | Table 1과 모든 보조 실험의 단일 사용자 진입점 |
| `calibration_pipeline/table1.py` | A0~A4/B1~B3 실행, A5 initial state 예약, 공통 split/reference/FK artifact/baseline 생성 |
| `calibration_pipeline/schema.py` | 조건, 자유변수, 순차 freeze boundary, 공정 비교 계약 |
| `calibration_pipeline/reprojection.py` | raw distorted-pixel projection, SE(3) 변수화, robust least squares |
| `calibration_pipeline/evaluation.py` | px-only 재투영 계약, 공통 평가와 상태 직렬화의 단일 구현 |
| `calibration_pipeline/path_evaluation.py` | `e_cross`, 양방향 pixel transfer, `e_e2e`와 고정 mask |
| `calibration_pipeline/cross_target.py` | 저장된 모든 방법을 동일 338-corner/common path mask로 평가 |
| `calibration_pipeline/marker_system.py` | modality별 초기화부터 분리한 marker-system 비교 |
| `calibration_pipeline/blind_prediction.py`, `external_gt.py` | 향후 blind 외부 GT 예측/채점; 현재 Table 1 내부 순위에는 미사용 |
| `tools/sync_table1_canonical_data.py` | 실행 JSON/CSV에서 main CSV와 HTML 숫자를 동기화 |
| `tools/verify_table1_visual_sync.py` | Markdown/HTML/3 CSV/baseline 계약 및 제거된 mm 필드 검사 |
| `tools/verify_e_cross_definition.py` | FK 없이 `e_cross`와 pixel transfer를 독립 재계산 |
| `tools/verify_step3_shared_baseline.py` | Step3와 Table 1이 생성한 baseline payload/hash 동일성 확인 |

## Canonical 파일

- [Main CSV](table1_results.csv)
- [Unified Table 1 JSON](table1_methods.json)
- [Shared baseline](shared_train_only_baseline.json)
- [Board-free FK cube artifact](shared_board_free_fk_cube.json)
- [Common evaluation CSV](../cross_target_evaluation/cross_target_evaluation.csv)
- [Marker-system CSV](../marker_system_end_to_end/marker_system_end_to_end.csv)
- [Interactive HTML](../../../_TABLE1_INTERACTIVE.html)
