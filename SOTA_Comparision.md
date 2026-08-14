# Multi-camera–robot calibration: 비교 방법 사전 안내

## 1. Overview

- **연구 목표: multi-camera–robot calibration**
  - 여러 카메라를 로봇 base 좌표계에 등록
  - 고정 카메라와 wrist camera를 하나의 robot workcell 안에서 함께 calibration
  - 카메라에서 측정한 물체 pose를 로봇이 사용할 수 있는 좌표로 변환

- **우리 시스템의 구성**
  - 작업 공간에 설치된 고정 카메라 여러 대
  - 로봇 손목에 설치된 wrist camera 한 대
  - 로봇이 잡은 marker cube와 작업대의 calibration board 사용
  - 여러 카메라의 관측을 하나의 최적화 문제로 결합

- **Hand–eye calibration과의 관계**
  - Hand–eye calibration: 전체 문제를 구성하는 기본 calibration 관계
  - Robot-world/hand–eye calibration: 로봇과 외부 카메라의 관계를 추정하는 주요 formulation
  - Multi-camera–robot calibration: 여러 관계를 함께 다루는 이 연구의 system-level 목표
  - 관련 논문 검색 및 baseline 분류에는 `hand–eye`, `robot-world/hand–eye` 용어를 그대로 사용

- **우리 방법의 핵심**
  - 카메라별 독립 calibration이 아닌 multi-camera joint calibration
  - PnP pose만 맞추는 방식이 아닌 raw corner 기반 pixel-level optimization
  - Robot FK를 완전히 정확한 값으로 고정하지 않고 불확실성을 고려
  - 여러 카메라가 관측한 cube pose를 shared latent variable로 사용

- **비교 실험에서 확인할 질문**
  1. 카메라별 독립 calibration보다 multi-camera joint calibration이 정확한가?
  2. FK를 고정하는 방식보다 FK uncertainty를 고려하는 방식이 정확한가?

- 세부 기여 및 성능 판정 기준: [`SOTA_Claim_Protocol.md`](SOTA_Claim_Protocol.md)

## 2. 기본 용어

### 2.1 카메라 설치 방식

| 구성 | 설명 |
| --- | --- |
| eye-in-hand | 카메라가 로봇 손목에 부착되어 함께 움직이는 구성 |
| eye-to-hand | 카메라가 작업 공간에 고정되어 로봇을 바라보는 구성 |

- 우리 시스템: eye-in-hand와 eye-to-hand가 함께 있는 mixed configuration
- 기존 방법의 일반적인 제약
  - 한 가지 설치 방식만 지원
  - 카메라 한 대만 지원
  - 여러 카메라에 각각 실행한 뒤 결과를 조합해야 하는 경우가 많음

### 2.2 `AX=XB`와 `AX=ZB`

- **`AX=XB`**
  - 로봇의 motion과 카메라에서 관측한 motion 사용
  - 하나의 고정 변환 `X` 추정
  - 고전적인 hand–eye calibration의 대표 형태

- **`AX=ZB`**
  - Hand–eye와 robot-world 관계에 해당하는 두 변환 동시 추정
  - `AX=XB`보다 우리 문제 설정에 가까운 형태

- **주의점**
  - 논문마다 `X`, `Y`, `Z`의 정의가 다를 수 있음
  - 문자보다 각 변환의 출발 좌표계와 도착 좌표계를 확인해야 함

### 2.3 Pose-level과 pixel-level

- **Pose-level**
  - 영상의 marker corner로 PnP pose를 먼저 계산
  - 계산된 camera/target pose와 robot pose 사이의 오차 최소화
  - 고전적인 hand–eye 방법에서 주로 사용

- **Pixel-level**
  - 검출된 corner의 pixel 좌표를 직접 사용
  - 예측 corner와 관측 corner 사이의 reprojection error 최소화
  - Bundle adjustment 계열에서 주로 사용

- **비교에서의 의미**
  - Pose-level 방법: 기본 baseline
  - Multi-camera pixel-level 방법: 우리 방법과 더 가까운 직접 비교 대상

## 3. 비교 후보

| 구분 | 방법 | 핵심 특징 | 공개 구현 |
| --- | --- | --- | --- |
| Classical | Tsai–Lenz (1989) | 회전과 이동을 순차적으로 푸는 `AX=XB` | [OpenCV](https://github.com/opencv/opencv/blob/4.x/modules/calib3d/src/calibration_handeye.cpp) |
| Classical | Park–Martin (1994) | Lie group을 이용한 `AX=XB` | [OpenCV](https://github.com/opencv/opencv/blob/4.x/modules/calib3d/src/calibration_handeye.cpp) |
| Classical | Horaud–Dornaika (1995) | 다른 회전 표현을 사용하는 `AX=XB` | [OpenCV](https://github.com/opencv/opencv/blob/4.x/modules/calib3d/src/calibration_handeye.cpp) |
| Classical | Andreff et al. (1999) | 회전과 이동을 동시에 추정하는 on-line formulation | [OpenCV](https://github.com/opencv/opencv/blob/4.x/modules/calib3d/src/calibration_handeye.cpp) |
| Classical | Daniilidis (1999) | Dual quaternion 기반 simultaneous 방법 | [OpenCV](https://github.com/opencv/opencv/blob/4.x/modules/calib3d/src/calibration_handeye.cpp) |
| Robot-world | Shah (2013) | `AX=ZB`의 두 변환을 closed-form으로 계산 | [OpenCV](https://github.com/opencv/opencv/blob/4.x/modules/calib3d/src/calibration_handeye.cpp) |
| Iterative | Tabb & Ahmad Yousef (2017) | Robot-world/hand–eye(s)의 반복 최적화 | [GitHub](https://github.com/amy-tabb/RWHEC-Tabb-AhmadYousef) |
| Multi-camera | Allegro et al. (2024) | 여러 카메라를 reprojection error로 함께 보정 | [GitHub](https://github.com/davidea97/Multi-Camera-Hand-Eye-Calibration) |
| Uncertainty-aware | Ha (2023) | 측정별 noise와 covariance를 고려 | [GitHub](https://github.com/hjhdog1/probabilisticAXYB) |
| Targetless | Calib3R (2025) | Marker 없이 RGB와 3D model 사용 | [GitHub](https://github.com/davidea97/Calib3R) |

- **Baseline과 SOTA의 구분**
  - Tsai–Lenz, Park–Martin, Horaud–Dornaika, Andreff, Daniilidis: 최신 방법이 아닌 고전 baseline
  - 최신 논문이라도 입력과 설치 조건이 다르면 직접적인 경쟁 방법으로 보기 어려움
  - 실제 비교 대상 선정 기준
    - 우리 시스템에 적용 가능
    - 공개 코드 또는 재현 가능한 구현 존재
    - 동일한 외부 기준값으로 평가 가능

## 4. 방법별 비교 관점

### 4.1 고전 hand–eye 방법

- 대상: Tsai–Lenz, Park–Martin, Horaud–Dornaika, Andreff, Daniilidis
- 구현: OpenCV `calibrateHandEye`
- 입력
  - Robot FK pose
  - Marker 관측에서 계산한 PnP pose
- 특징
  - 동일한 입력에 solver option만 변경하여 비교 가능
  - 카메라별 독립 calibration
  - Pose-level objective
- 비교 역할
  - 전통적인 calibration 성능의 기준선
  - Multi-camera joint optimization의 효과를 확인하기 위한 비교군

### 4.2 Shah (2013)

- 문제 형태: robot-world/hand–eye `AX=ZB`
- 방법: 두 미지 변환의 closed-form 추정
- 특징
  - `AX=XB`보다 우리 문제 설정에 가까움
  - Robot pose를 고정 입력으로 사용
  - Pose-level 방법
- 비교 역할: Robot-world/hand–eye 계열의 기본 baseline

### 4.3 Tabb & Ahmad Yousef (2017)

- 문제 형태: robot-world/hand–eye(s)
- 방법: 반복 최적화
- 특징
  - 여러 cost function과 rotation parameterization 제시
  - Multi-eye 문제 지원
  - 공개 코드의 입력 형식에 맞춘 data adapter 필요
- 확인할 사항
  - 우리 카메라 구성의 지원 범위
  - 사용할 cost variant
  - Pixel observation 사용 여부와 정확한 objective

### 4.4 Allegro et al. (2024)

- 입력
  - Calibration board image
  - Camera intrinsics
  - Robot pose
- 방법
  - 여러 카메라의 pose를 함께 최적화
  - Reprojection error 최소화
- 비교 역할
  - 우리 방법과 가장 가까운 공개 multi-camera baseline
  - Multi-camera joint calibration 효과의 직접 비교 대상
- 확인할 사항
  - 고정 카메라와 wrist camera의 동시 지원 여부
  - 지원하지 않는 카메라가 있을 경우 실제 평가 범위 명시
  - 원 구현의 설정값 및 수정 사항 기록

### 4.5 불확실성을 고려하는 방법

- **Ha (2023)**
  - 측정별 noise와 covariance를 반영하는 확률적 `AX=ZB`
  - Pose-level uncertainty 처리의 비교 대상

- **우리 방법과의 구분**
  - FK uncertainty 자체를 최초 기여로 주장하지 않음
  - 차별점
    - Mixed eye-in-hand/eye-to-hand 구성
    - Multi-camera pixel-level optimization
    - Cube와 board 관측의 결합
    - Shared latent cube pose에 대한 robust soft-FK factor

### 4.6 Targetless 방법

- 대상: Calib3R
- 특징
  - Calibration marker 불필요
  - RGB sequence와 3D foundation model 사용
- 우리 방법과의 차이
  - 입력 조건 자체가 다름
  - Marker 기반 방법과 동일한 조건의 직접 비교가 어려움
- 비교 역할
  - Targetless calibration의 편의성과 정확도 간 trade-off 확인
  - 주된 순위 경쟁보다 참고 실험에 가까움

## 5. 실제 데이터 적용 전 simulation 검증

### 5.1 목적

- SOTA method의 성능 순위 평가가 아닌 **구현 정확성 검증**
- 확인할 항목
  - 좌표계 방향과 transform chain
  - 단위 변환
  - Single-camera method의 camera별 반복 적용
  - Multi-camera method의 shared variable 및 camera coupling
  - 입력 adapter와 결과 변환 코드
- 기본 합격 조건
  - Noise가 없는 관측에서 ground-truth transform 복원
  - 오차가 사전에 정한 numerical tolerance 이내
  - 수치 연산 때문에 정확히 `0.0`일 필요는 없음

### 5.2 Simulation 구성

- **Calibration target**
  - Cube를 사용하지 않고 board만 사용
  - 실제 촬영에 사용할 board와 동일한 geometry 사용
  - Board의 row/column 수, marker 또는 square 크기, marker 간격을 실제 단위로 입력
  - Board frame의 원점과 축 방향을 명시

- **Robot 및 camera ground truth**
  - 고정 카메라별 extrinsic `T_base_camera_i`
  - End-effector에 고정된 board transform `T_gripper_board`
  - Pose별 robot FK `T_base_gripper(k)`
  - 카메라별 intrinsic matrix와 distortion coefficient

- **Synthetic observation 생성**
  - Board의 3D corner를 transform chain으로 각 camera frame에 변환
  - Camera intrinsic과 distortion model을 이용해 pixel로 projection
  - 실제 field of view와 image resolution 밖의 corner는 제외
  - 여러 카메라가 같은 robot pose에서 board를 관측하는 event 포함
  - 전체 camera–event observation graph가 연결되도록 구성

- **Robot trajectory**
  - Translation과 roll/pitch/yaw가 모두 변하는 pose 사용
  - 거의 같은 위치나 평행한 회전축만 반복하는 degenerate motion 방지
  - Board가 영상의 중앙뿐 아니라 가장자리와 서로 다른 depth에서도 관측되도록 구성

### 5.3 검증 순서

1. **Exact-pose test**
   - Ground-truth target pose와 robot pose를 solver에 직접 입력
   - Calibration solver 자체와 transform convention 검증
2. **Noiseless-pixel test**
   - 정확한 board corner를 pixel로 projection
   - PnP 및 data adapter를 포함한 전체 pipeline 검증
3. **Randomized recovery test**
   - Camera extrinsic, robot trajectory, solver initialization과 random seed 변경
   - 특정 scene이나 초기값에서만 맞는 구현인지 확인
   - 구현 자체의 사전 점검에만 사용하며, 방법 간 비교 실험에는 포함하지 않음
4. **Realistic-noise test**
   - Pixel noise, 일부 corner 누락, outlier, FK noise를 순서대로 추가
   - 구현 검증 통과 후 robustness와 method별 성능 차이 확인

### 5.4 Multi-camera 적용 확인

- Single-camera baseline
  - 동일한 synthetic data에서 카메라별로 각각 calibration
  - 각 `T_base_camera_i`를 ground truth와 비교
  - Camera별 결과를 robot base frame으로 조립하는 코드 검증

- Multi-camera baseline
  - 모든 camera observation을 하나의 문제에 입력
  - Camera별 extrinsic과 공통 `T_gripper_board`를 함께 추정
  - 카메라 사이의 relative transform도 ground truth와 비교
  - 일부 event에서 특정 camera의 관측을 제거해도 연결된 observation graph에서 복원되는지 확인

- **구분할 사항**
  - 카메라별 single-camera solver를 여러 번 실행하는 것: multi-camera system에 대한 독립 적용
  - Shared variable을 사용해 여러 카메라를 함께 최적화하는 것: joint multi-camera calibration
  - 두 구현을 별도 baseline으로 구분

### 5.5 이 simulation이 검증하는 범위

- 충분한 범위
  - Board-on-end-effector를 관측하는 고정 카메라 calibration
  - 정지 board를 관측하는 wrist camera eye-in-hand calibration
  - Single-camera baseline의 반복 적용과 결과 조립
  - Board 기반 joint multi-camera calibration
  - 좌표계, 단위, PnP, projection 및 adapter 구현

- 포함되지 않는 범위
  - Cube observation과 cube pose 추정
  - Cube에 연결된 soft-FK factor
  - Fixed camera와 wrist camera를 함께 묶는 전체 mixed-camera graph

- 따라서 board-only simulation은 **첫 번째 구현 검증 단계로 충분**
- 전체 시스템 검증에는 이후 eye-in-hand 또는 mixed-camera simulation이 별도로 필요
- 기존 `CP_synthetic_7row.py`의 cube/FK ablation과는 목적을 분리
- Marker를 사용하지 않는 Calib3R는 이 board-only 검증 대상에서 제외
- Uncertainty-aware method는 noiseless recovery 통과 후 noise/covariance 조건에서 추가 검증
- 공통 코드 및 adapter 형식: [`SOTA_Simulation/README.md`](SOTA_Simulation/README.md)

### 5.6 공통 noise sweep

- 모든 방법에서 아래 조건을 동일하게 사용
  - Camera 배치와 ground truth 고정
  - Eye-in-hand와 eye-to-hand는 서로 다른 robot trajectory 사용 가능
  - 각 실험군에서는 모든 비교 알고리즘에 동일한 robot/board trajectory 사용
  - Board geometry와 관측 event 수 고정
  - Solver마다 동일한 관측과 동일한 noise sample 사용
- 3D board-corner perception noise
  - `0, 1, 3, 5 mm`
  - Camera frame에서 관측된 각 board corner의 x/y/z 좌표에 Gaussian noise 적용
  - 각 level은 corner 좌표축별 standard deviation을 의미
  - Noisy corner로 rigid board pose를 다시 추정하므로 translation과 rotation error가 함께 발생
  - 3D corner detector를 가정한 구현 stress test이며 RGB ChArUco의 pixel/PnP noise와 동일하지 않음
- Calibration
  - Wrist camera eye-in-hand calibration을 독립적으로 수행
  - 각 fixed camera eye-to-hand calibration을 독립적으로 수행
  - 결과를 robot base frame에 모아 camera별 extrinsic error 평가
- 반복 평가
  - 기본 30개 random seed 사용
  - 같은 seed의 standard-normal sample을 noise level에 따라 scaling하는 paired comparison
  - Mean, standard deviation, trial distribution 및 failure를 함께 저장
- 주의
  - 위 과정은 multi-camera system에 single-camera solver를 독립 적용하는 baseline
  - Shared variable로 모든 camera를 동시에 푸는 joint multi-camera calibration과 구분
- 실행 코드: [`SOTA_Simulation/tsai_noise_sweep.py`](SOTA_Simulation/tsai_noise_sweep.py)

## 6. 비교 실험의 공통 원칙

- 모든 방법에 최대한 동일하게 적용
  - 촬영 데이터
  - Camera intrinsics와 distortion model
  - Robot pose 및 timestamp synchronization
  - 외부 평가 데이터
  - Train/validation/test split

- 원 방법의 범위 유지
  - 지원하지 않는 구성을 임의로 추가하지 않음
  - 수정이 필요한 경우 원 방법과 수정 버전을 구분
  - 원 논문에 없는 후처리 추가 시 별도 variant로 보고

- Hyperparameter 관리
  - Training/validation data에서 결정
  - Test 결과 확인 후 변경 금지

- Coverage와 failure 기록
  - 실제 평가한 카메라 수와 구성 명시
  - 지원하지 못한 카메라 명시
  - 실패한 실행의 code version, 환경, 실패 원인 기록

- 좌표계와 단위 확인
  - `base→gripper`와 `gripper→base` 구분
  - `target→camera`와 `camera→target` 구분
  - mm와 m 혼용 방지
  - 실제 데이터 적용 전 synthetic data로 convention 검증

## 7. 성능 평가

### Primary metrics

- Calibration에 사용하지 않은 held-out pose로 평가
- **Held-out chain error**
  - 추정 extrinsic으로 held-out board 관측을 robot base frame에 변환
  - Ground-truth board pose에 대한 translation [mm] 및 rotation [degree] error
- **Camera pose accuracy**
  - `wrist, cam0, cam1, cam3`의 extrinsic GT error
  - Camera별 값과 동일 가중 system macro-average를 함께 보고
- **Multi-camera registration consistency**
  - 네 카메라에서 만들 수 있는 여섯 camera pair의 relative transform error
  - Translation [mm] 및 rotation [degree]을 별도로 보고
- **Held-out reprojection RMSE**
  - Calibration에 사용하지 않은 pose의 모든 camera/board corner pixel RMSE
- Translation, rotation, pixel error를 임의 가중합한 단일 score는 사용하지 않음
- Simulation 실행 코드: [`SOTA_Simulation/opencv_multicam_evaluation.py`](SOTA_Simulation/opencv_multicam_evaluation.py)

### 함께 보고할 항목

- Mean 및 95% confidence interval
- P50/P95 error
- Failure rate
- Camera registration coverage
- 실행 시간과 필요한 영상 수

### Diagnostic metrics

- Held-out reprojection error
- 카메라 간 pose inconsistency
- 복원된 cube 치수 오차
- Solver convergence 및 condition number

- **주의점**
  - Reprojection error는 구현 점검용 지표
  - 작은 reprojection error가 정확한 외부 calibration을 보장하지 않음
  - 최종 방법 순위는 독립된 external GT로 판단

## 8. 논문과 구현을 확인할 항목

1. **문제 정의**
   - `AX=XB`, `AX=ZB` 또는 별도의 formulation
2. **입력 데이터**
   - Image, corner, PnP pose, robot FK, covariance 등
3. **지원 구성**
   - Eye-in-hand 또는 eye-to-hand
   - Single-camera 또는 multi-camera
4. **목적함수**
   - Pose error, reprojection error 또는 probabilistic objective
5. **Robot pose 처리**
   - 고정된 값 또는 불확실한 관측
6. **재현 가능성**
   - 공개 코드
   - Pretrained model
   - Dependency와 hardware requirement
   - 입력 형식 및 실행 예제

## 참고 자료

- [OpenCV hand–eye calibration documentation](https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html)
- [Tsai & Lenz, 1989](https://doi.org/10.1109/70.34770)
- [Daniilidis, 1999](https://www.cis.upenn.edu/~kostas/mypub.dir/ijrr99.pdf)
- [Shah, 2013](https://www.nist.gov/publications/solving-robot-worldhand-eye-calibration-problem-using-kronecker-product)
- [Tabb & Ahmad Yousef, 2017](https://arxiv.org/abs/1907.12425) · [code](https://github.com/amy-tabb/RWHEC-Tabb-AhmadYousef)
- [Ha, 2023](https://doi.org/10.1109/TRO.2022.3214350) · [code](https://github.com/hjhdog1/probabilisticAXYB)
- [Allegro et al., 2024](https://arxiv.org/abs/2406.11392) · [code](https://github.com/davidea97/Multi-Camera-Hand-Eye-Calibration)
- [Calib3R](https://github.com/davidea97/Calib3R)
