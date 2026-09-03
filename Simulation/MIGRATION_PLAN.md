# Simulation Backend Migration Plan

## 바로가기

- [1. 이 문서의 범위](#1-이-문서의-범위)
- [확정된 설계 결정](#확정된-설계-결정)
- [2. 현재 실제 데이터 코드와의 불일치](#2-현재-실제-데이터-코드와의-불일치)
- [3. 채택할 공통 방법](#3-채택할-공통-방법)
- [4. 통합 목적함수](#4-통합-목적함수)
- [5. 파일별 수정 목록](#5-파일별-수정-목록)
- [6. FK 모드 매핑](#6-fk-모드-매핑)
- [7. 삭제·유지 기준](#7-삭제유지-기준)
- [8. 필수 검증](#8-필수-검증)
- [9. 완료 조건](#9-완료-조건)

## 1. 이 문서의 범위

이번 실제 데이터 비교실험 리팩터링에서는 `Simulation/`의 실행 코드를 수정하지 않는다. 이 문서는 시뮬레이션을 실제 데이터 Table 1과 같은 수학적 방법·관측 단위·solver 계약으로 바꾸기 위한 후속 작업 명세다.

현재 시뮬레이션 결과는 기존 코드로 생성된 결과이므로, 아래 변경을 적용하기 전까지 새 실제 데이터 Table 1과 “동일 backend 결과”라고 주장하면 안 된다.

## 확정된 설계 결정

기존 계획·감사 문서에서 현재 코드에도 유효한 결정만 이 문서에 통합한다.

1. FK 처리는 `no-FK(vision)`, `raw-FK-fixed`, `corrected-FK factor`의 세 모드로 분리한다.
2. A3의 hard constraint는 controller raw FK에 사전 등록한 mechanical frame map만 적용하며, 영상으로 추정한 정렬값을 사용하지 않는다.
3. Board-only 조건에는 cube FK prior나 corrected-FK factor를 적용하지 않는다.
4. PnP는 초기화에만 사용하고 최종 visual objective는 raw distorted-pixel corner reprojection으로 통일한다.
5. FK 불확실성은 DH joint-noise를 가정하지 않고 cube-pose FK에 직접 가한 SE(3) perturbation으로 모델링한다.
6. `e_cross`는 held-out 평가 전용이며 최적화 목적함수에 넣지 않는다.

## 2. 현재 실제 데이터 코드와의 불일치

현재 `Simulation/core/project.py`는 3D object corner, 이미지 corner, `K`, distortion을 생성한다. 그러나 `Simulation/core/methods.py`의 최종 최적화는 이 원시 corner를 직접 쓰지 않고, 프레임별 PnP로 만든 pose를 입력으로 받아 SE(3) pose-consistency residual을 최소화한다.

따라서 현재 상태는 다음과 같다.

| 구분 | 실제 데이터 Table 1 | 현재 Simulation |
| --- | --- | --- |
| 초기화 | robust PnP/SE(3) | robust PnP/SE(3) |
| 최종 visual residual | raw corner pixel reprojection | PnP pose consistency |
| residual 단위 | px | translation/rotation 정규화값 |
| 통합 방식 | eih+e2h pixel residual 한 벡터 | eih+e2h pose residual 한 벡터 |
| FK factor | covariance-whitened SE(3) factor | 동일 개념이나 다른 visual backend와 결합 |

PnP pose를 먼저 확정하면 PnP 단계에서 손실된 corner별 정보와 비등방 관측 불확실성을 최종 solver가 복구할 수 없다. 그러므로 시뮬레이션도 PnP는 초기화에만 사용하고, 최종 해는 원시 픽셀 corner로 결정해야 한다.

## 3. 채택할 공통 방법

공통 파이프라인은 다음 한 줄로 고정한다.

> robust PnP/SE(3) 초기화 → raw distorted-corner pixel reprojection 최종 최적화

실제 데이터의 다음 구현을 시뮬레이션에서도 그대로 호출한다.

- visual problem/state/solver: `calibration_pipeline/reprojection.py`
- FK factor: `calibration_pipeline/fk_factor.py`
- 조건과 freeze mask: `calibration_pipeline/schema.py`
- px 평가 정의: `calibration_pipeline/evaluation.py`, `calibration_pipeline/path_evaluation.py`

시뮬레이션 전용 코드는 scene 생성, noise 주입, 반복 Monte Carlo, ground-truth 평가만 담당한다. projection·SE(3) 변수화·robust loss·FK factor를 별도로 재구현하지 않는다.

## 4. 통합 목적함수

이벤트 (e), 타깃 (o), 3D corner (X_{o,k}), 관측 픽셀 (u_{e,c,o,k})를 사용한다.

고정 카메라 (C_i)의 eye-to-hand 예측은

\[
\hat u^{e2h}_{e,i,o,k}
=
\pi\!\left(
K_i,D_i,
T_{C_iB}T_{Bo}X_{o,k}
\right),
\qquad T_{C_iB}=T_{BC_i}^{-1}
\]

이고, 손목 카메라 (C_h)의 eye-in-hand 예측은

\[
\hat u^{eih}_{e,h,o,k}
=
\pi\!\left(
K_h,D_h,
(T_{BG_e}T_{GC_h})^{-1}T_{Bo}X_{o,k}
\right)
\]

이다. 독립 방식은 eih를 먼저 풀어 타깃·hand-eye를 고정한 뒤 각 고정 카메라를 e2h만으로 푼다.

\[
\theta_{eih}^{*}=\arg\min \sum \rho(\|u-\hat u^{eih}\|^2),
\qquad
T_{BC_i}^{*}=\arg\min_{T_{BC_i}} \sum \rho(\|u-\hat u^{e2h}\|^2)
\]

통합 방식은 같은 변수들을 한 번에 갱신한다.

\[
\theta_U^{*}
=
\arg\min_{\theta_U}
\left[
\sum \rho(\|u-\hat u^{eih}(\theta_U)\|^2)
+
\sum \rho(\|u-\hat u^{e2h}(\theta_U)\|^2)
+
E_{FK}(\theta_U)
\right]
\]

핵심 차이는 통합 방식에서 고정 카메라 residual의 gradient가 공유 타깃 pose와 `T_gripper_camera`까지 되돌아간다는 점이다. `e_cross`는 이 목적함수에 넣지 않고 held-out 평가에만 사용한다.

## 5. 파일별 수정 목록

### `Simulation/core/project.py`

- 현재 반환하는 `obj`, `img`, `K`, `dist`, event/set/camera 정보를 보존한다.
- PnP pose `T`는 초기화 진단용으로만 표시한다.
- 실제 데이터 `PixelObs`로 무손실 변환할 수 있도록 marker 종류, camera id, event id, set id를 명시한다.

### `Simulation/core/scene.py`

- scene observation 저장 시 PnP pose만 남기지 말고 원시 3D/2D corner 배열을 끝까지 보존한다.
- eye-in-hand와 eye-to-hand 관측이 동일한 observation schema를 사용하게 한다.
- GT transform은 평가 객체에만 두고 solver 입력과 분리한다.

### `Simulation/core/methods.py`

- 현재 pose-domain `se3_residual` 기반 최종 solver를 canonical 최종법에서 제거한다.
- 생성 관측을 `calibration_pipeline.reprojection.PixelObs`로 변환한다.
- `PoseState`, `variable_keys`, `solve_corner_reprojection`을 실제 데이터와 동일하게 사용한다.
- `none`: cube pose 자유변수, FK factor 없음.
- `fixed`: cube pose를 FK 값으로 설정하고 variable key에서 제외.
- `factor`: cube pose 자유변수 + `solve_factorized_fk`의 covariance-whitened robust factor.
- 기존 pose-consistency solver는 명시적인 `pose_consistency_ablation`으로만 남기며 기본법이나 자동 fallback으로 사용하지 않는다.

### `Simulation/core/experiment.py`

- 기본 dispatch를 raw-corner reprojection 방법으로 변경한다.
- solver 실패 시 pose 방식 결과를 같은 method 이름으로 대체하지 않는다. 실패는 실패로 기록한다.
- seed, solver option, observation mask를 모든 FK 모드에서 공유한다.

### `Simulation/core/metrics.py`

- train/held-out reprojection은 component-wise pixel RMSE 계약으로 통일한다.
- GT가 있으므로 translation error(mm), rotation error(deg)는 별도 absolute-GT 지표로 유지한다.
- `e_cross`는 FK·GT·손목카메라를 쓰지 않는 고정카메라 pairwise cube-pose consistency로 계산한다.
- `e_cross`와 GT pose error를 같은 지표로 합치지 않는다.

### 실행·시각화 파일

`run_all.py`, `run_realistic.py`, `run_table2a.py`, `run_paper_sim.py`, `run_which_wins.py`와 관련 시각화 코드는 새 method 이름과 px 계약을 읽도록 변경한다. 모든 결과 Markdown은 상단 바로가기 목차를 생성해야 한다.

## 6. FK 모드 매핑

| Table 1 | Simulation 모드 | cube 변수 | FK 항 | visual 항 |
| --- | --- | --- | --- | --- |
| A2 | `none` | 자유 | 없음 | 공통 raw-corner px |
| A3 | `fixed` | 고정 | hard constraint | 공통 raw-corner px |
| A4 | `factor` | 자유 | covariance-whitened robust factor | 공통 raw-corner px |
| B1 | `factor` + sequential freeze | 1단계 자유, 이후 고정 | A4와 동일 | 공통 raw-corner px |
| B2 | `factor` + cube-only | 자유 | A4와 동일 | 공통 raw-corner px |

## 7. 삭제·유지 기준

삭제 대상은 동일 수학을 별도로 구현한 projection, optimizer, FK robustification 중복 코드다. 유지 대상은 다음뿐이다.

- synthetic geometry와 camera/robot trajectory 생성
- noise/outlier/FK perturbation 생성
- Monte Carlo 반복과 GT 평가
- 명시적으로 이름 붙인 pose-consistency ablation

자동 fallback은 금지한다. raw-corner solver 실패를 pose solver 성공으로 바꾸면 방법별 실패율과 정확도 비교가 오염된다.

## 8. 필수 검증

1. noise-free scene에서 sequential과 unified가 각각 GT를 수치 허용오차 내 복원해야 한다.
2. FK noise가 0일 때 A3 hard-FK가 GT cube pose와 일치해야 한다.
3. FK covariance를 크게 할수록 A4가 A2에 접근하고, 작게 할수록 A3에 접근해야 한다.
4. 동일 synthetic observations를 실제 데이터 adapter와 Simulation adapter에 넣었을 때 residual vector가 원소 단위로 같아야 한다.
5. `e_cross`는 공통 base-frame transform을 좌측 곱해도 불변이어야 한다.
6. 모든 방법이 동일 split, seed, K/D, raw corner mask, solver budget을 기록해야 한다.
7. pose-consistency 코드를 제거해도 canonical method 결과가 변하지 않아야 한다.

## 9. 완료 조건

- Simulation canonical method가 `CornerReprojectionProblem`을 직접 사용한다.
- 실제 데이터와 Simulation의 A2/A3/A4가 같은 변수/freeze/FK mode 계약을 사용한다.
- 최종 visual metric은 px만 사용한다.
- absolute 3D error는 synthetic GT에 대해서만 별도 열로 보고한다.
- `e_cross`는 평가 전용이며 목적함수와 분리된다.
- 기존 Simulation 결과를 새 backend로 전부 재생성하고, 결과 문서·표·그림의 숫자가 한 canonical 데이터 소스에서 동기화된다.
