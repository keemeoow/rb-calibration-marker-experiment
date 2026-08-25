# Multi-camera–robot calibration: 비교 방법 사전 안내

## 목차

- [1. 비교 목적](#toc-section-1)
- [2. 공통 문제 정의](#toc-section-2)
- [3. 비교군 요약](#toc-section-3)
- [4. C 계열 — Classical hand-eye baselines](#toc-section-4)
- [5. D 계열 — Robot-world 및 iterative baselines](#toc-section-5)
- [6. D3 — 최근접 공개 multi-camera baseline](#toc-section-6)
- [7. D4 — 조건부 targetless baseline](#toc-section-7)
- [8. 2026년 이후 관련 방법 처리](#toc-section-8)
- [8.1. D5–D6 — FK uncertainty 직접 비교군](#toc-section-9)
- [9. 데이터 수집 계약](#toc-section-10)
- [10. 공정한 실행 계약](#toc-section-11)
- [11. 평가 지표](#toc-section-12)
- [12. 통계 계약](#toc-section-13)
- [13. 사전 합격 조건](#toc-section-14)
- [14. 결과 표 템플릿](#toc-section-15)
- [15. 보고 문장 계약](#toc-section-16)
- [16. 참고 문헌 및 구현](#toc-section-17)

> Version: 1.0  
> 기준일: 2026-08-06  
> 상태: 외부 GT 실험 전 사전 계약

<a id="toc-section-1"></a>

## 1. 비교 목적

- **우리 시스템의 구성**
  - 작업 공간에 설치된 고정 카메라 여러 대
  - 로봇 손목에 설치된 wrist camera 한 대
  - 로봇이 잡은 marker cube와 작업대의 calibration board 사용
  - 여러 카메라의 관측을 하나의 최적화 문제로 결합

1. 카메라별 independent calibration보다 multi-camera joint calibration이 우수한가?
2. Robot FK를 고정 관측으로 사용하는 기존 방법보다 covariance-weighted robust corrected-FK factor가 우수한가?
3. Calibration 이후의 residual correction이 독립 외부 GT에서도 추가 이득을 주는가?

- **우리 방법의 핵심**
  - 카메라별 독립 calibration이 아닌 multi-camera joint calibration
  - PnP pose만 맞추는 방식이 아닌 raw corner 기반 pixel-level optimization
  - Robot FK를 완전히 정확한 값으로 고정하지 않고 불확실성을 고려
  - 여러 카메라가 관측한 cube pose를 shared latent variable로 사용

- **비교 실험에서 확인할 질문**
  1. 카메라별 독립 calibration보다 multi-camera joint calibration이 정확한가?
  2. FK를 고정하는 방식보다 FK uncertainty를 고려하는 방식이 정확한가?

<a id="toc-section-2"></a>

## 2. 공통 문제 정의

## 2. 기본 용어

### 2.1 카메라 설치 방식

방법 간 차이는 FK와 영상 관측을 어떤 방정식 또는 목적함수로 연결하고, 어떤 pose를 고정 입력 또는 최적화 변수로 취급하는가에 있다.

<a id="toc-section-3"></a>

## 3. 비교군 요약

| ID | 방법 | 계열 | FK 구분 | Joint multi-camera | Pixel-level objective | FK uncertainty model | 실행 상태 |
| --- | --- | --- | --- | --- | --- | --- | --- |
| C1 | Tsai–Lenz | classical `AX=XB` | FK-fixed | 아니오 | 아니오 | 없음 | 필수 |
| C2 | Park–Martin | classical `AX=XB` | FK-fixed | 아니오 | 아니오 | 없음 | 필수 |
| C3 | Horaud | classical `AX=XB` | FK-fixed | 아니오 | 아니오 | 없음 | 필수 |
| C4 | Daniilidis | classical dual quaternion | FK-fixed | 아니오 | 아니오 | 없음 | 필수 |
| D1 | Shah | robot-world/hand-eye `AX=YB` | FK-fixed | 아니오 | 아니오 | 없음 | 필수 |
| D2 | Tabb–Ahmad Yousef | iterative robot-world/hand-eye(s) | FK-fixed | 지원 | cost variant에 따름 | 명시적 covariance soft factor 없음 | 필수 |
| D3 | Allegro et al. | reprojection 기반 multi-camera hand-eye | FK-fixed | 예 | 예 | robot pose를 고정 입력으로 사용 | 최우선 recent baseline |
| D4 | Calib3R | targetless 3D foundation model | FK-fixed | 예 | 2D/3D scene terms | robot motion 기반 calibration term | 조건부 |
| D5 | Ha | probabilistic `AX=YB` | corrected-FK | 아니오 | 아니오 | 측정별 noise/reliability 모델 | 필수 FK-uncertainty baseline |
| D6 | Ulrich–Hillemann | uncertainty-aware hand–eye | corrected-FK | 아니오 | 지원 구성에 따름 | robot uncertainty와 corrected robot poses | 최우선 FK-uncertainty baseline |
| Ours-core | A4 | unified cube+board U-BA | corrected-FK | 예 | 예 | covariance-weighted robust corrected-FK | 필수 |
| Ours-full 후보 | A5 | A4 + held-out 6-DoF correction | corrected-FK | 예 | A4와 동일 | A4와 동일 | 합격 시에만 |

`없음`은 해당 방법의 원 논문 또는 표준 구현에서 제안 A4와 같은 `Sigma_FK^-1` 기반 robust soft factor를 사용하지 않는다는 뜻이다. 모든 형태의 noise handling이 없다는 뜻으로 해석하지 않는다.

corrected-FK 또는 robot uncertainty 자체를 최초 기여로 주장하지 않는다. Strobl–Hirzinger(2006), Ha(2023), Ulrich–Hillemann(2024)이 stochastic 또는 uncertainty-aware hand–eye calibration을 이미 다룬다. 제안 방법의 차별점은 이러한 불확도 처리를 mixed eye-in-hand/eye-to-hand, raw pixel-level, cube+board multi-camera U-BA의 shared latent cube poses에 결합하는 데 둔다.

<a id="toc-section-4"></a>

## 4. C 계열 — Classical hand-eye baselines

### C1. Tsai–Lenz (1989)

Tsai–Lenz는 대표적인 `AX=XB` hand-eye calibration 방법이다. 상대 robot motion `A`, 상대 camera motion `B`, 고정 hand-eye transform `X`를 사용하고 rotation과 translation을 순차적으로 계산한다.

- 역할: 가장 전통적인 기준선
- 실행: OpenCV `calibrateHandEye(..., CALIB_HAND_EYE_TSAI)`
- 제한: 카메라별 independent solve이며 joint pixel-level objective가 아니다.
- 문헌: [Tsai and Lenz, 1989](https://doi.org/10.1109/70.34770)

### C2. Park–Martin (1994)

Park–Martin은 Lie group과 matrix logarithm을 이용해 `AX=XB`를 푼다.

- 역할: Tsai와 다른 rotation formulation을 갖는 classical baseline
- 실행: OpenCV `calibrateHandEye(..., CALIB_HAND_EYE_PARK)`
- 제한: C1과 동일하게 pose-level independent solve이다.

### C3. Horaud (1995)

Horaud는 rotation 표현과 hand-eye 방정식 해법이 다른 classical `AX=XB` 방법이다.

- 역할: 동일 PnP pose input에서 classical solver 선택에 따른 성능 범위 확인
- 실행: OpenCV `calibrateHandEye(..., CALIB_HAND_EYE_HORAUD)`
- 제한: Ours의 pixel-level joint BA와 목적함수가 다르다.

### C4. Daniilidis (1999)

Daniilidis는 dual quaternion으로 rotation과 translation을 하나의 algebraic formulation에서 다룬다.

- 역할: simultaneous closed-form 계열의 대표 기준선
- 실행: OpenCV `calibrateHandEye(..., CALIB_HAND_EYE_DANIILIDIS)`
- 제한: 모든 noise 조건에서 다른 classical 방법보다 우수하다고 가정하지 않는다.
- 문헌: [Daniilidis, 1999](https://www.cis.upenn.edu/~kostas/mypub.dir/ijrr99.pdf)

C1–C4는 모두 실행한 뒤 동일 외부 GT에서 가장 좋은 방법을 `best classical`로 추가 표시한다. 결과를 본 뒤 한 방법만 선택하여 보고하지 않는다.

<a id="toc-section-5"></a>

## 5. D 계열 — Robot-world 및 iterative baselines

### D1. Shah (2013)

Shah는 robot-world/hand-eye 문제를 다음 형태로 푼다.

```text
A_i X = Y B_i
```

Frame convention에 따라 두 번째 미지수는 `Y` 또는 `Z`로 표기할 수 있다. Kronecker product와 SVD를 이용하는 separable closed-form solution으로 두 미지수를 구한다.

- 역할: robot-world/hand-eye closed-form 기준선
- 실행: OpenCV `calibrateRobotWorldHandEye(..., CALIB_ROBOT_WORLD_HAND_EYE_SHAH)`
- FK 취급: 입력된 robot pose는 최적화 중 covariance를 가진 자유변수로 수정하지 않는다.
- 문헌: [Shah, 2013](https://www.nist.gov/publications/solving-robot-worldhand-eye-calibration-problem-using-kronecker-product)

### D2. Tabb & Ahmad Yousef (2017)

Tabb와 Ahmad Yousef는 `AX=ZB` robot-world/hand-eye 문제에 대해 여러 iterative cost와 rotation parameterization을 비교하고, robot-world hand-multiple-eye로 확장한다.

- 역할: closed-form이 아닌 iterative 기준선, multi-eye 지원 기준선
- 실행: 저자 공개 [코드 저장소](https://github.com/amy-tabb/RWHEC-Tabb-AhmadYousef)
- 입력: 저자 형식으로 변환한 동일 board observations와 robot poses
- 공정성: 공개 cost variant를 training-only validation에서 선택하고 test에서는 고정한다.
- 문헌: [Tabb and Ahmad Yousef, 2017](https://arxiv.org/abs/1907.12425)

논문에 없는 임의의 robustification이나 후처리를 추가하지 않는다. 추가 구현이 필요하면 원 방법과 `+refinement` 변형을 별도 행으로 보고한다.

<a id="toc-section-6"></a>

## 6. D3 — 최근접 공개 multi-camera baseline

### Allegro, Terreran & Ghidoni (2024)

Allegro et al.은 calibration board와 robot motion을 사용해 여러 카메라의 pose를 함께 최적화한다. 공통 board-to-end-effector transform과 camera 간 spatial consistency를 이용하며 reprojection error를 최소화한다.

- 역할: 제안 셋업과 가장 가까운 공개 구현 기반 recent multi-camera baseline
- 핵심 대비: 두 방법 모두 joint reprojection optimization을 사용하지만, A4는 cube pose에 covariance-weighted robust FK factor를 둔다.
- 공식 구현: [Multi-Camera-Hand-Eye-Calibration](https://github.com/davidea97/Multi-Camera-Hand-Eye-Calibration)
- 문헌: [Allegro et al., 2024](https://arxiv.org/abs/2406.11392)

#### 적용 계약

- Board-on-EE session에서 모든 고정 카메라가 가능한 한 동시에 board를 관측한다.
- 원 구현의 intrinsics, image, robot-pose 및 configuration 형식을 사용한다.
- 원 방법이 직접 지원하지 않는 mixed eye-in-hand + eye-to-hand 전체 구성을 억지로 하나의 run에 넣지 않는다.
- 지원 가능한 camera subset의 공통 외부 GT 정확도와 전체 시스템 coverage를 분리해 보고한다.
- 저자 기본값 변경, bug fix 또는 adapter patch는 commit hash와 함께 기록한다.

Allegro를 “전체 hand-eye 분야의 유일한 최신 SOTA”라고 표현하지 않는다. 본 실험에서는 `closest public multi-camera baseline`으로 정의한다.

<a id="toc-section-7"></a>

## 7. D4 — 조건부 targetless baseline

### Calib3R (2025)

Calib3R는 calibration pattern 없이 일반 RGB 장면에서 3D foundation model의 pointmaps를 만들고, robot motion과 결합한 unified optimization으로 metric-scaled reconstruction과 camera-to-robot calibration을 함께 수행한다. Single- 및 multi-camera robot setup을 지원한다.

- 역할: pattern-free calibration의 최신 operating point
- 공식 구현: [Calib3R](https://github.com/davidea97/Calib3R)
- 문헌: [Allegro et al., 2025](https://arxiv.org/abs/2509.08813)
- 자원: 공식 논문 실험은 고성능 GPU를 사용하므로 hardware와 runtime을 별도 공개한다.

이전 문서의 “공식 코드가 404/비공개”라는 문장은 삭제한다. 2026-08-06 현재 공식 저장소는 공개되어 있다.

#### 포함 조건

D4는 아래 조건을 모두 만족할 때 quantitative table에 포함한다.

1. 공식 환경을 재현하고 제공된 예제로 정상 실행한다.
2. 실험 RGB sequence에 충분한 texture, overlap 및 viewpoint diversity가 있다.
3. Camera mounting과 robot-pose convention을 원 방법에 맞게 매핑할 수 있다.
4. Calibration에 사용하지 않은 동일 외부 GT blind poses로 평가할 수 있다.

조건을 충족하지 못하면 실패를 숨기지 않고 원인과 시도한 버전·환경을 기록한다. Marker-based 방법과 입력 조건이 다르므로 D4는 primary rank의 필수 승리 조건이 아니라 `targetless convenience–accuracy trade-off`로 별도 해석한다.

<a id="toc-section-8"></a>

## 8. 2026년 이후 관련 방법 처리

2026년의 multi-camera robot-world/hand-eye dual-quaternion 방법처럼 더 최신 문헌은 related work에서 검토한다. 단, 다음을 모두 만족할 때만 재현 실험 행으로 승격한다.

- 논문과 알고리즘 세부가 충분히 공개됨
- 본 데이터의 frame convention으로 모호하지 않게 매핑 가능
- reference implementation 또는 독립적으로 검증 가능한 재현 절차가 있음
- 동일 외부 GT 평가가 가능함

출판연도만으로 D3를 대체하지 않는다.

<a id="toc-section-9"></a>

## 8.1. D5–D6 — FK uncertainty 직접 비교군

### D5. Ha (2023)

Ha는 `AX=YB` loop closure에서 개별 `A_i`, `B_i` 측정의 서로 다른 noise property와 reliability를 반영하는 probabilistic maximum-likelihood framework를 제안하고 estimation uncertainty를 산출한다.

- 역할: pose-level covariance-aware robot-world/hand-eye baseline
- 공식 구현: [probabilisticAXYB](https://github.com/hjhdog1/probabilisticAXYB)
- 문헌: [Probabilistic Framework for Hand–Eye and Robot–World Calibration](https://doi.org/10.1109/TRO.2022.3214350)
- 비교 제한: 원 방법은 raw-corner multi-camera cube+board joint BA가 아니므로 공통 외부 GT endpoint로 비교한다.

### D6. Ulrich & Hillemann (2024)

Ulrich와 Hillemann은 industrial robot의 absolute pose uncertainty를 명시적으로 모델링하고 hand–eye pose와 corrected robot poses를 추정한다. Target-based와 targetless 구성을 지원하며 robot uncertainty 자체도 보고한다.

- 역할: 제안 A4와 가장 직접적으로 겹치는 uncertainty-aware hand–eye baseline
- 문헌: [Uncertainty-Aware Hand–Eye Calibration](https://doi.org/10.1109/TRO.2023.3330609)
- 비교 제한: mixed multi-camera 전체 구성을 직접 지원하지 않으면 지원 camera subset과 전체 coverage를 분리 보고한다.

### 필수 related work. Strobl & Hirzinger (2006)

Strobl과 Hirzinger는 SE(3) stochastic model과 manipulator의 translation/rotation precision 특성에 따른 weighting을 이용한 maximum-likelihood hand–eye calibration을 제안했다. 따라서 covariance weighting의 역사적 근거로 반드시 인용한다.

- 문헌: [Optimal Hand-Eye Calibration](https://doi.org/10.1109/IROS.2006.282250)

<a id="toc-section-10"></a>

## 9. 데이터 수집 계약

### 공통 세션

| 준비물 | 사용 방법 |
| --- | --- |
| eye-in-hand | 카메라가 로봇 손목에 부착되어 함께 움직이는 구성 |
| eye-to-hand | 카메라가 작업 공간에 고정되어 로봇을 바라보는 구성 |

- 우리 시스템: eye-in-hand와 eye-to-hand가 함께 있는 mixed configuration
- 기존 방법의 일반적인 제약
  - 한 가지 설치 방식만 지원
  - 카메라 한 대만 지원
  - 여러 카메라에 각각 실행한 뒤 결과를 조합해야 하는 경우가 많음

### 2.2 `AX=XB`와 `AX=ZB`

<a id="toc-section-11"></a>

## 10. 공정한 실행 계약

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

<a id="toc-section-12"></a>

## 11. 평가 지표

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

<a id="toc-section-13"></a>

## 12. 통계 계약

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

Primary comparison family에는 Holm correction을 적용한다. 5 sessions × 30 blind poses는 pilot으로 사용하고, 최종 session 수는 pilot의 session-level variance를 이용한 power analysis로 정한다.

<a id="toc-section-14"></a>

## 13. 사전 합격 조건

수치 margin은 robot task tolerance와 외부 GT uncertainty를 기준으로 실험 전에 확정한다.

```text
UpperCI(mean(TRE_A4 - TRE_baseline)) < 0
UpperCI(P95_A4 - P95_baseline) < m_P95
UpperCI(e_R,A4 - e_R,baseline) < m_R
FailureRate_A4 - FailureRate_baseline <= m_fail
```

Ours-core 우위 주장은 다음을 모두 만족할 때만 허용한다.

1. 사전 지정한 primary baseline(B1, best classical, D3) 대비 mean `TRE_t` superiority를 통과한다.
2. Rotation error가 superiority 또는 `m_R` 이내 non-inferiority를 통과한다.
3. P95와 failure rate가 각 margin 안에 있다.
4. Worst workspace stratum에서 큰 열화가 없다.
5. 차이가 외부 GT measurement uncertainty floor보다 해석 가능한 크기다.

A5는 A4 대비 같은 계약을 translation과 rotation 모두에서 통과할 때만 Ours-full로 채택한다.

<a id="toc-section-15"></a>

## 14. 결과 표 템플릿

| Method | FK 구분 | Input | Cameras supported/evaluated | `TRE_t` mean [95% CI] ↓ | `TRE_t` P95 ↓ | `e_R` mean [95% CI] ↓ | ADD/ADD-S ↓ | Coverage ↑ | Failure ↓ | Time ↓ |
| --- | --- | --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| C1 Tsai | FK-fixed | target + FK/PnP | — | — | — | — | — | — | — | — |
| C2 Park | FK-fixed | target + FK/PnP | — | — | — | — | — | — | — | — |
| C3 Horaud | FK-fixed | target + FK/PnP | — | — | — | — | — | — | — | — |
| C4 Daniilidis | FK-fixed | target + FK/PnP | — | — | — | — | — | — | — | — |
| D1 Shah | FK-fixed | target + FK/PnP | — | — | — | — | — | — | — | — |
| D2 Tabb | FK-fixed | target + FK/images | — | — | — | — | — | — | — | — |
| D3 Allegro | FK-fixed | board + FK + images | — | — | — | — | — | — | — | — |
| D4 Calib3R | FK-fixed | targetless RGB + FK | — | — | — | — | — | — | — | — |
| D5 Ha probabilistic AXYB | corrected-FK | target poses + robot poses/covariance | — | — | — | — | — | — | — | — |
| D6 Uncertainty-Aware HEC | corrected-FK | target/images + uncertain robot poses | — | — | — | — | — | — | — | — |
| B1 Independent fair | corrected-FK | cube+board + FK + images | — | — | — | — | — | — | — | — |
| A2 Unified | no-FK(vision) | cube+board + images | — | — | — | — | — | — | — | — |
| A4 Ours-core | corrected-FK | cube+board + FK + images | — | — | — | — | — | — | — | — |
| A5 Ours-full candidate | corrected-FK | A4 + correction | — | — | — | — | — | — | — | — |

`Cameras supported/evaluated`는 반드시 채운다. 지원하지 못한 camera를 제외하고 얻은 낮은 오차와 전체 camera coverage를 혼동하지 않는다.

<a id="toc-section-16"></a>

## 15. 보고 문장 계약

외부 GT 전에는 다음 수준만 허용한다.

> Unified visual calibration은 기존 데이터의 held-out pixel consistency를 개선했다. Fixed-weight soft anchor는 유망한 선행 결과를 보였지만, covariance-weighted robust A4와 독립 외부 GT 평가는 아직 완료되지 않았다.

외부 GT와 통계 계약을 통과한 뒤에만 다음 형식의 문장을 사용한다.

> 동일한 independent sessions와 blind external-GT poses에서 A4는 사전 지정한 primary baselines 대비 translation error를 낮추면서 rotation, P95 및 failure-rate non-inferiority 조건을 충족했다.

“가장 정확하다”, “SOTA를 달성했다” 같은 포괄적 표현은 평가한 방법·셋업·workspace·GT 범위를 문장 안에서 제한할 수 있을 때만 사용한다.

<a id="toc-section-17"></a>

## 16. 참고 문헌 및 구현

- [Tsai & Lenz, *A New Technique for Fully Autonomous and Efficient 3D Robotics Hand/Eye Calibration*, 1989](https://doi.org/10.1109/70.34770)
- [Daniilidis, *Hand-Eye Calibration Using Dual Quaternions*, 1999](https://www.cis.upenn.edu/~kostas/mypub.dir/ijrr99.pdf)
- [Shah, *Solving the Robot-World/Hand-Eye Calibration Problem Using the Kronecker Product*, 2013](https://www.nist.gov/publications/solving-robot-worldhand-eye-calibration-problem-using-kronecker-product)
- [Tabb & Ahmad Yousef, *Solving the Robot-World Hand-Eye(s) Calibration Problem with Iterative Methods*, 2017](https://arxiv.org/abs/1907.12425)
- [Tabb & Ahmad Yousef official code](https://github.com/amy-tabb/RWHEC-Tabb-AhmadYousef)
- [Allegro et al., *Multi-Camera Hand-Eye Calibration for Human-Robot Collaboration in Industrial Robotic Workcells*, 2024](https://arxiv.org/abs/2406.11392)
- [Allegro et al. official multi-camera code](https://github.com/davidea97/Multi-Camera-Hand-Eye-Calibration)
- [Allegro et al., *Calib3R*, 2025](https://arxiv.org/abs/2509.08813)
- [Calib3R official code](https://github.com/davidea97/Calib3R)
- [Strobl & Hirzinger, *Optimal Hand-Eye Calibration*, 2006](https://doi.org/10.1109/IROS.2006.282250)
- [Ha, *Probabilistic Framework for Hand–Eye and Robot–World Calibration*, 2023](https://doi.org/10.1109/TRO.2022.3214350)
- [Ha official probabilistic AXYB code](https://github.com/hjhdog1/probabilisticAXYB)
- [Ulrich & Hillemann, *Uncertainty-Aware Hand–Eye Calibration*, 2024](https://doi.org/10.1109/TRO.2023.3330609)
