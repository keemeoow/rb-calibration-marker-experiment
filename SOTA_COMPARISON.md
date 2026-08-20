# SOTA 비교 실험 명세

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

이 비교는 단순히 많은 방법을 나열하기 위한 것이 아니다. 다음 세 질문을 분리해 검증한다.

1. 카메라별 independent calibration보다 multi-camera joint calibration이 우수한가?
2. Robot FK를 고정 관측으로 사용하는 기존 방법보다 covariance-weighted robust corrected-FK factor가 우수한가?
3. Calibration 이후의 residual correction이 독립 외부 GT에서도 추가 이득을 주는가?

세 번째 질문은 A5가 A4 대비 사전 정의한 translation·rotation 합격 조건을 모두 통과한 경우에만 최종 기여로 채택한다. 그 전까지 제안 방법은 A4(Ours-core)이다.

이 문서에서 `classical baseline`, `recent close baseline`, `targetless baseline`을 구분한다. 고전 방법을 SOTA라고 부르지 않으며, 출판연도가 최신이라는 이유만으로 셋업이 다른 방법을 직접 경쟁자로 간주하지 않는다.

<a id="toc-section-2"></a>

## 2. 공통 문제 정의

로봇과 카메라는 서로 다른 좌표계에서 pose를 표현한다.

- `T_base_gripper = FK(q)`: joint encoder와 robot kinematic model에서 얻은 end-effector pose
- `T_camera_target`: calibration target의 영상 관측에서 얻은 camera-target 관계
- Calibration output: camera, robot base, gripper 및 target frame 사이의 고정 변환

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
| Board-on-EE session | Classical 고정 카메라, Shah, Tabb, Allegro |
| 기존 또는 신규 eye-in-hand target session | Classical wrist camera 및 지원 방법 |
| Cube+board joint session | A2–A5와 B 계열 |
| Texture가 충분한 RGB sequence + robot poses | Calib3R |
| Calibration에 사용하지 않은 외부 GT blind poses | 모든 방법의 공통 최종 평가 |

### 촬영 최소 조건

- 동일 camera installation session 안에서 방법별 raw input을 최대한 공유한다.
- Board-on-EE는 최소 20–30개의 다양한 translation 및 roll/pitch/yaw pose로 촬영한다.
- Fixed exposure, gain, focus와 동일 intrinsics를 사용한다.
- Corner detections, raw RGB, timestamps, FK poses, visibility masks와 config hash를 저장한다.
- 독립 camera re-installation session을 반복한다.
- Train/test는 frame이 아니라 physical session과 blind workspace pose 기준으로 분리한다.

<a id="toc-section-11"></a>

## 10. 공정한 실행 계약

### 동일하게 고정할 항목

- Raw frames와 외부 GT blind poses
- Camera intrinsics와 distortion model
- Robot-pose timestamp synchronization
- Classical 방법에 입력하는 PnP poses
- Pixel-level 방법에 입력하는 corner detections
- Train/test session split
- 허용되는 initialization/multi-start budget
- 실패 판정과 time limit

### 방법별 고유 항목

- 원 논문이 요구하는 target 또는 targetless input
- 원 구현의 objective와 parameterization
- 저자 권장 hyperparameters
- 필수 dependency와 compute hardware

Hyperparameter는 training-only inner validation으로 정하고 test 및 external-GT blind set에서 동결한다. Test 결과를 보고 method-specific threshold, loss scale 또는 initialization을 바꾸지 않는다.

<a id="toc-section-12"></a>

## 11. 평가 지표

### Primary

| 지표 | 단위 | 정의 |
| --- | --- | --- |
| `TRE_t` | mm | 예측 pose와 독립 외부 GT 사이 translation error |
| `e_R` | deg | 두 rotation 사이 geodesic angle |
| ADD 또는 ADD-S | mm | Task object model의 pose discrepancy |

ADD-S는 cube symmetry를 task에서 동일 pose로 인정할 때만 사용한다. Symmetry set은 결과 확인 전에 고정한다.

### Secondary

- P50/P95 `TRE_t`와 `e_R`
- Camera registration coverage `N_reg`
- Session/camera/pose failure rate
- Calibration runtime, inference runtime와 peak memory
- Number of images required

### Diagnostic only

- Held-out reprojection error
- Cross-camera disagreement
- Reconstructed cube dimension error
- Commanded relative-motion consistency
- FK-proxy error
- Solver convergence와 condition number

Targetless D4에는 marker reprojection error가 공통 지표로 정의되지 않는다. 따라서 모든 방법의 최종 순위는 공통 외부 GT endpoint로만 정한다.

<a id="toc-section-13"></a>

## 12. 통계 계약

추론 단위는 camera-installation session이다. 동일 session의 blind poses를 독립 session처럼 세지 않는다.

1. Session을 paired 방식으로 복원추출한다.
2. 각 session 내부에서 동일 blind pose pair를 복원추출한다.
3. Method 간 paired difference를 계산한다.
4. Hierarchical bootstrap 95% CI를 산출한다.

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
