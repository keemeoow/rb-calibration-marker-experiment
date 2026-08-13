# Calibration Experiment — 최종 비교실험 명세

## 연구 목적과 최종 논문 기여도

### 연구 목적

본 연구의 목적은 **여러 고정 카메라(eye-to-hand, 로봇 외부 고정 카메라)와 그리퍼 장착 카메라
(eye-in-hand, 로봇 손목 카메라)를 하나의 로봇 기준 좌표계(robot-base frame)에 정밀하고 일관되게
등록하는 통합 캘리브레이션 방법**을 개발하는 것이다. 각 카메라를 따로 보정한 뒤 변환을 연쇄적으로
곱하는 기존 방식 대신, 모든 카메라의 실제 영상 관측, 로봇 정기구학(FK), 하나의 공유 물리 표적을
동일한 최적화 문제 안에서 결합한다.

최종적으로는 학습 기반 6차원 자세 추정기에 의존하지 않고도 다음을 달성하는 것이 목표다.

- 서로 다른 장착 방식의 카메라를 하나의 로봇 기준 좌표계에 직접 등록한다.
- 여러 카메라가 공유할 수 있고 로봇이 직접 파지·이동할 수 있는 다면 마커 큐브를 사용한다.
- 로봇 정기구학을 절대 정답으로 고정하지 않고 측정 불확실성을 가진 기하 제약으로 융합한다.
- 영상상의 재투영 오차뿐 아니라 독립 외부 기준값과 실제 로봇 작업 정확도로 유효성을 검증한다.

### 핵심 연구 질문과 검증 비교

| 연구 질문 | 핵심 비교 또는 평가 | 비교에서 바뀌는 요소 | 판정 기준 |
| --- | --- | --- | --- |
| **RQ1. 통합 최적화가 카메라별 독립 보정보다 우수한가?** | `B1→A4` | 독립 최적화 → 통합 최적화 | 동일 관측·동일 FK 정보에서 외부 기준 3차원 오차와 작업 오차 감소 |
| **RQ2. 파지 가능한 다면 큐브가 평면 보드보다 추가 기하 정보를 제공하는가?** | 주 비교 `B3→A2`, 보조 비교 `A0→A1` | 보드만 사용 → 보드+큐브 사용 | 등록률·관측 범위·조건수 개선과 외부 기준 오차 감소 |
| **RQ3. 불확실성 기반 soft-FK가 visual-only 또는 hard-FK보다 우수한가?** | `A2→A3→A4a→A4b→A4` | FK 없음 → 강제 FK → soft-FK → 공분산 가중 → 강건 손실 | 독립 외부 기준 정확도와 tail error 개선, 공분산 calibration 적합성 확보 |
| **RQ4. 제안 보정이 실제 로봇 조작 정확도를 개선하는가?** | 독립 외부 GT + blind manipulation task | 캘리브레이션 방법만 변경 | 목표 위치·회전 오차, 파지 성공률, 실패율 개선 |

`A→B`는 **방법 A에서 방법 B로 바꾼 짝 비교**를 뜻하며, 표의 오차 변화량은 별도 언급이 없으면
`B의 오차 − A의 오차`로 계산한다. 따라서 오차 지표에서는 음수가 개선이다.

### 최종 논문 기여와 현재 증거 수준

| 기여 | 제안 내용 | 핵심 검증 | session02-late의 현재 결과 | 현재 판정 |
| --- | --- | --- | --- | --- |
| **C1. 통합 로봇 기준 다중 카메라 보정** | 여러 고정 카메라와 손목 카메라를 공유 표적 기반의 하나의 최적화에서 공동 등록 | `B1→A4` | 홀드아웃 전체 재투영 RMSE `−0.0098 px`, 카메라 간 위치 불일치 `−0.5819 mm`, 고정↔손목 경로 위치 불일치 `−0.1701 mm` | 내부 지표의 소폭 개선만 확인. 독립 외부 GT 전에는 기여 입증 불가 |
| **C2. 파지 가능한 다면 마커 큐브** | 평면 보드를 넘어 여러 카메라와 로봇 조작이 공유하는 3차원 강체 표적 제공 | 주 비교 `B3→A2`; 보조 비교 `A0→A1` | 공통 보드 재투영 오차가 각각 `+0.0209 px`, `+0.0274 px`로 개선되지 않음 | 현재 안정 구간에서는 정확도·등록률 이득 미확인. 관측 범위와 외부 GT 평가 필요 |
| **C3. 불확실성 인지 정기구학 기반 등록** | FK를 공분산 가중 강건 soft constraint로 영상과 융합 | `A2→A3→A4a→A4b→A4` | hard-FK(A3)는 전체 재투영 `−0.6157 px`, 카메라 간 위치 불일치 `−2.2955 mm`이나 고정↔손목 경로는 `+0.7211 mm` 악화. A4는 A2 대비 `−0.0124 px` | 실측 `Σ_FK`가 아닌 임시값을 쓴 preflight이므로 최종 기여 미입증 |
| **C4. 실제 로봇 시스템 검증** | 재투영 오차를 넘어 독립 외부 GT와 실제 조작 작업으로 유효성 검증 | `TRE_t`, `e_R`, ADD/ADD-S, 작업 오차·파지 성공률 | 독립 외부 GT와 blind task 결과 없음 | 미평가 |

따라서 현재 논문에서 안전하게 말할 수 있는 범위는 **“통합 보정, 다면 큐브, 불확실성 기반 FK 융합을
하나의 검증 가능한 프레임워크로 구현했다”**까지다. **“제안 방법이 가장 정확하다”**, **“다면 큐브가
평면 보드보다 우수하다”**, **“soft-FK가 hard-FK보다 우수하다”**는 문장은 아래 외부-GT 및 통계 계약을
통과한 뒤에만 사용한다.

### 기호·약어·단위 정의

| 기호 또는 약어 | 한글 의미 | 이 문서에서의 정의 |
| --- | --- | --- |
| `T_A_B` 또는 `T_A^B` | 좌표변환 행렬 | 좌표계 B의 점을 좌표계 A로 표현하는 4×4 강체변환 행렬 |
| `T_base_Ci` | 고정 카메라 외부파라미터 | `i`번째 고정 카메라 좌표를 로봇 기준 좌표로 변환 |
| `T_gripper_cam` 또는 `gTc` | 손-카메라 변환 | 손목 카메라 좌표를 그리퍼 좌표로 변환하는 hand-eye 결과 |
| `T_base_gripper=FK(q)` | 로봇 정기구학 자세 | 관절값 `q`로 계산한 그리퍼의 로봇 기준 자세 |
| `T_base_cube` | 큐브의 로봇 기준 자세 | 큐브 좌표를 로봇 기준 좌표로 변환 |
| `FK` | 정기구학(Forward Kinematics) | 관절값으로 로봇 링크 또는 그리퍼 자세를 계산하는 모델 |
| `GT` | 외부 기준값(Ground Truth) | 캘리브레이션 학습과 독립적으로 측정한 기준 자세 |
| `6-DoF` | 6자유도 | 3축 위치와 3축 회전을 함께 표현한 자세 |
| `SE(3)` | 3차원 강체변환군 | 3차원 회전과 이동을 함께 다루는 수학적 공간 |
| `seq` | 순차 보정 | 손목 경로를 먼저 풀고 동결한 뒤 고정 카메라를 각각 추정 |
| `U-BA` | 통합 번들 조정(Unified Bundle Adjustment) | 카메라·hand-eye·표적 자세를 raw corner 단위로 공동 최적화 |
| `hard-FK` | 강제 FK 제약 | 큐브 자세를 FK 예측값에 고정하고 최적화하지 않음 |
| `soft-FK` | 연성 FK 제약 | 큐브 자세를 자유변수로 두되 FK와의 차이에 비용을 부여 |
| `r_FK` | FK 잔차 | FK 예측 자세와 시각 기반 큐브 자세 사이의 6차원 차이 |
| `Σ_FK` | FK 공분산 | FK 위치·회전 오차의 크기와 축 간 상관관계를 나타내는 6×6 행렬 |
| `ρ(·)` | 강건 손실 함수 | 큰 이상치의 영향을 줄이는 Huber 또는 soft-L1 함수 |
| `λ` | 고정 FK 가중치 | 공분산 대신 쓰는 사전 고정된 soft-FK 제약 강도 |
| `E_visual`, `E_FK` | 영상 비용, FK 비용 | 각각 raw-corner 재투영 잔차 비용과 FK 자세 잔차 비용 |
| `RMSE` | 평균제곱근오차 | 오차 제곱의 평균에 제곱근을 취한 값; 낮을수록 좋음 |
| `TRE_t` | 표적 위치 등록오차 | 외부 GT와 예측 표적 중심 사이의 3차원 거리(mm) |
| `e_R` | 회전 오차 | 외부 GT와 예측 회전 사이의 측지각(deg) |
| `ADD`, `ADD-S` | 물체 모델 평균 거리 | 예측·GT 자세로 옮긴 물체 표면점 간 평균 거리; `ADD-S`는 대칭 허용 |
| `e_cross` | 고정 카메라 간 불일치 | 같은 큐브를 본 고정 카메라 쌍의 로봇 기준 자세 차이(mm/deg) |
| `e_e2e` | 고정↔손목 경로 불일치 | 고정 카메라 묶음의 큐브 자세와 손목 카메라+FK 경로의 자세 차이(mm/deg) |
| `N_reg` | 등록된 고정 카메라 수 | 보정에 성공해 평가 가능한 eye-to-hand 카메라 개수 |
| `P50`, `P95` | 중앙값, 95백분위수 | 전체 오차 중 각각 50%, 95%가 이 값 이하임을 뜻함 |
| `CI` | 신뢰구간(Confidence Interval) | 반복 표집에서 모수 또는 방법 차이를 포함할 것으로 기대되는 구간 |
| `px`, `mm`, `deg` | 픽셀, 밀리미터, 도 | 각각 영상 오차, 위치 오차, 회전 오차의 단위 |

## 문서 목적과 현재 결론

이 문서는 실제 셋업에서 위 기여를 분리하고, “Ours가 3차원 공간 정합에서 우수하다”는 주장을 어떤
조건에서 허용할지 정한 최종 실험 계약이다. 과거 디버깅 수치와 내부 감사 과정은 반복하지 않고 원본
산출물에 남긴다.

현재 결론은 다음과 같다.

- Session02 안정 구간의 공통 홀드아웃 분할에서 영상 전용 통합 보정은 순차 보정 대비 전체 재투영
  RMSE를 0.0113 px 줄였다. 차이는 작으며 외부 정확도 근거는 아니다.
- 공분산 기반 강건 soft-FK와 공정 B1/B2 비교군은 구현·실행됐다. 다만 실측 `Σ_FK`가 없어
  3 mm / 0.3 deg 등방성 임시값을 쓴 **software 예비실험(preflight)** 결과다.
- A5 6-DoF correction과 독립 외부 GT 평가는 아직 없다. 따라서 `TRE_t`, `e_R`, ADD/ADD-S와
  절대 3D 정확도 우열은 판정할 수 없다.
- 현재 평가지표 설계는 보완 후 충분하지만, session02에서 실제 계산된 내부 진단 지표만으로는 네 기여를
  최종 입증하기에 부족하다.
- 외부 GT와 실측 covariance가 준비되기 전에는 “Ours가 가장 정확하다”고 결론 내리지 않는다.

## 방법 정의

모든 방법은 운동학 백본 `T_base_gripper = FK(q)`를 공통으로 사용한다. 실험 축의 `FK`는 이
백본의 사용 여부가 아니라, **cube pose에 FK 정보를 어떤 방식으로 주입하는가**를 뜻한다.

- `seq`: eye-in-hand와 eye-to-hand를 순차적으로 풀고 앞 단계 결과를 동결한다.
- `U-BA`: 고정·손목 카메라, hand-eye, target pose를 하나의 pixel-level bundle adjustment에서 푼다.
- `hard-FK`: cube pose를 FK pose에 고정한다.
- `soft-FK`: cube pose를 자유변수로 두고 covariance-weighted robust FK factor를 추가한다.
- `correction`: calibration 이후 held-out target pose 예측에만 적용한다. Calibration transform은 바꾸지 않는다.

제안 soft-FK residual은 다음을 기본형으로 한다.

```text
r_FK = Log(T_FK^-1 T_cube)
E_FK = rho(r_FK^T Sigma_FK^-1 r_FK)
```

여기서 `r_FK`는 FK 예측과 큐브 자세 사이의 6차원 자세 잔차, `Log`는 강체변환을 회전 3축과 이동
3축의 벡터로 바꾸는 로그 사상, `Σ_FK^-1`은 FK 공분산의 역행렬(정보행렬), `ρ`는 이상치 영향을
줄이는 강건 손실 함수다. 따라서 불확실성이 큰 FK 방향은 약하게, 신뢰도가 높은 방향은 강하게 반영한다.

`Sigma_FK`와 FK-factor robust-loss scale은 별도 반복측정 또는 training-only inner validation에서
정하고 test에서는 동결한다. 기존 상수 가중치 `lambda=3 px/mm` 결과는 **A4a fixed-weight soft-anchor
선행 실험**이며 최종 covariance-weighted robust A4 결과가 아니다.

A2와 A4는 다음 목적함수 차이 하나만 허용한다.

```text
A2: E_visual
A4: E_visual + E_FK
```

`E_visual`의 raw-corner residual, visibility mask, robust loss, outlier 기준, 초기화, multi-start 수,
iteration budget과 종료조건은 완전히 동일하게 동결한다. A2를 기존 pose-level 결과로 두고 A4만
pixel-level로 실행한 값은 `A2→A4`의 인과 비교에 사용하지 않는다.

## Table 1 — Main Ablation (실제 셋업)

Table 1에는 제안 기여를 직접 검증하는 A/B 계열만 둔다. 모든 행은 동일 source frames, frozen
intrinsics, train/test mask와 공통 평가 mask를 사용한다. 동일 optimization level의 행은 frozen corner/PnP
frontend와 solver budget도 공유하며, target subset이나 입력 표현이 다른 경우 해당 차이를 표에 공개한다.

| 방법 번호 | 방법 / 검증 기여 | 관측 표적 | 해결 방식 | 큐브 자세에 대한 FK 사용 | 후처리 보정 | 반드시 비교할 짝 비교 | 현재 상태 |
| --- | --- | --- | --- | --- | --- | --- | --- |
| A0 | 보드만 사용한 순차 보정 — 평면 표적 기준선 | 보드 | 순차(`seq`) | 해당 없음 | 없음 | A0→A1: 큐브 추가에 따른 관측성·등록률 변화 | session02-late 홀드아웃 실행 완료(3/3) |
| A1 | 큐브+보드 순차 보정 — 큐브 관측성 | 큐브+보드 | 순차(`seq`) | 영상으로 추정 | 없음 | A0→A1; A1→A2 | 공통 픽셀 frontend 실행 완료(3/3) |
| A2 | 영상만 사용하는 통합 보정 — 통합 최적화 | 큐브+보드 | 통합 번들 조정(`U-BA`) | 사용하지 않음; 큐브 자세는 자유변수 | 없음 | **A1→A2:** 해결 방식만 순차→통합으로 변경 | 공통 픽셀 backend 실행 완료(3/3) |
| A3 | 강제 FK 통합 보정 — FK 강제·안정성 | 큐브+보드 | 통합 번들 조정(`U-BA`) | 강제 제약(`hard constraint`) | 없음 | **A2→A3:** FK 없음→강제 FK | 공통 픽셀 backend 실행 완료(3/3) |
| **A4★** | **불확실성 기반 soft-FK 통합 보정(Ours-core)** — 제안 캘리브레이션 원리 | **큐브+보드** | **강건 픽셀 단위 통합 번들 조정** | **공분산 가중 강건 FK 제약** | 없음 | **A2→A4:** 최종 FK 제약 전체 효과; **B1→A4:** 독립→통합 효과 | software preflight 완료(3/3); 실측 `Σ_FK`·외부 GT 필요 |
| A5 | 통합 soft-FK + 6자유도 오차 보정 — Ours-full 후보 | 큐브+보드 | A4와 동일 | A4와 동일 | 학습 데이터만 사용한 SE(3) 잔차 보정 | **A4→A5:** 후처리 보정만 추가 | 미구현. 통과 전에는 Ours-full로 부르지 않음 |
| **B1** | **강건 독립 보정 — 주 공정 기준선** | 큐브+보드 | 카메라별 독립 강건 최적화 후 강체변환 합성 | A4와 동일한 `Σ_FK` 정보량 | 없음 | **B1→A4:** 통합 최적화의 순수 효과 | 공정 preflight 완료(3/3); 실측 `Σ_FK` 필요 |
| B2 | 큐브만 사용하는 통합 보정 — 보드 기여 평가 | 큐브 | A4와 동일 | A4와 동일한 soft-FK | 없음 | **B2→A4:** 보드만 추가 | 공정 preflight 완료(3/3); 실측 `Σ_FK` 필요 |
| B3 | 보드만 사용하는 통합 보정 — 큐브 기여 평가 | 보드 | A2와 동일 | 해당 없음 | 없음 | **B3→A2:** 큐브만 추가 | session02-late 짝 비교 완료(3/3) |

### 행별 인과 해석

- `A1→A2`만 Unified의 순수 효과로 사용한다.
- `A2→A4`는 visual-only 대비 최종 covariance-weighted robust FK factor package의 전체 효과다.
- `A3→A4`는 hard-FK 대비 최종 covariance+robust soft-FK package 비교이며 순수 soft 전환 효과가 아니다.
  순수 soft constraint 효과는 supplementary의 `A3→A4a`로 판단한다.
- `A4→A5`만 correction의 추가 효과다. `A2→A5`를 correction 단순효과로 해석하지 않는다.
- Board의 효과는 `B2→A4`, cube의 효과는 `B3→A2`로 판단한다.
- `B3→A3`는 marker와 FK 조건을 동시에 바꾸므로 cube 기여의 근거로 사용하지 않는다.
- A0/A1은 관측성·전통 baseline이다. 제안 방법의 joint 효과는 정보량을 맞춘 `B1→A4`로 판단한다.

### Supplementary — A4 FK-factor decomposition

Main Table에는 최종 A4만 유지하고, A4 내부 기여는 아래 supplementary ablation으로 분리한다.
세 행 모두 `E_visual`과 solver budget은 A2/A4 공통 계약 그대로 유지하며, 아래 `Loss`는 **FK factor에
적용하는 loss만** 뜻한다.

| 방법 번호 | FK 잔차 형태 | FK 가중 방식 | FK 손실 함수 | 검증 목적 | 필수 짝 비교 |
| --- | --- | --- | --- | --- | --- |
| A4a | 연성 제약(`soft`) | 고정 등방성 가중치 `λ` | 제곱 손실 | 강제 제약을 연성 제약으로 바꾸는 효과 | `A3→A4a` |
| A4b | 연성 제약(`soft`) | FK 정보행렬 `Σ_FK^-1` | 제곱 손실 | 공분산 whitening(축별 불확실성 정규화)의 추가 효과 | `A4a→A4b` |
| **A4c = A4** | 연성 제약(`soft`) | FK 정보행렬 `Σ_FK^-1` | 사전 고정 강건 손실 | FK 이상치에 대한 강건화 효과 | `A4b→A4c` |

추가 confirmatory contrast는 `A2→A4`(최종 FK factor 전체 효과), `B1→A4`(independent→joint),
`A4→A5`(calibration을 고정한 correction 추가 효과)로 유지한다. A4a의 `lambda`도 test 결과가 아니라
training-only inner validation 또는 사전 물리 기준으로 정한다.

## Classical hand-eye baselines

Classical 방법은 mixed eye-in+eye-to joint calibration이 아니므로 Main Ablation에 포함하지 않는다.
동일 pose frontend와 동일 raw observations로 hand-eye를 계산한 후, 같은 규칙으로 fixed-camera 경로를
합성해 최종 task에서 평가한다.

| ID | 방법 | 분류 | 계획 구현 | 공정 비교 조건 | 상태 |
| --- | --- | --- | --- | --- | --- |
| C1 | Tsai–Lenz | classical `AX=XB`, separable | OpenCV `calibrateHandEye` | 동일 robust pose frontend·motion pairs·평가 mask | 미실행 |
| C2 | Park–Martin | classical `AX=XB`, Lie-group | OpenCV `calibrateHandEye` | C1과 동일 | 미실행 |
| C3 | Horaud | classical `AX=XB` | OpenCV `calibrateHandEye` | C1과 동일 | 미실행 |
| C4 | Daniilidis | classical `AX=XB`, dual quaternion | OpenCV `calibrateHandEye` | C1과 동일 | 미실행 |

계획 구현의 네 method enum은 [OpenCV `calibrateHandEye` 공식 문서](https://docs.opencv.org/4.13.0/d9/d0c/group__calib3d.html)에
포함되어 있다. 라이브러리 지원 여부와 별개로 좌표계 방향·입력 motion pair·퇴화 조건은 자체 synthetic
test로 검증한 뒤 실제 결과를 생성한다.

Classical 방법은 적용 범위가 A4보다 좁다는 점을 명시한다. 그러나 동일한 외부-GT 최종 task 결과는
직접 비교할 수 있다. 네 방법은 동일 입력으로 실행 비용이 낮으므로 최종 실험에는 모두 포함하는 것을
기본안으로 한다. C1–C4는 표준 기준선이지 최신 SOTA의 대체물이 아니다. Recent multi-camera,
robot-world, iterative 또는 targetless 외부 방법은 공개 코드·입력 적합성·실행 가능성을 확인한 별도
comparison contract에서 다루며, 현재 관련 감사 내용은
[CP_EXPERIMENTS_README — SOTA protocol audit](CP_EXPERIMENTS_README.md#sota-protocol-audit)를 따른다.

## 평가 지표

### 최종 정확도 판정을 위한 핵심 지표(Confirmatory accuracy endpoints)

| 지표(기호의 한글 뜻) | 정의 | 보고값 | 주의 |
| --- | --- | --- | --- |
| `TRE_t` (표적 위치 등록오차, mm) | 독립 외부 GT와 예측 표적 중심 사이의 3차원 거리 | 평균, P50(중앙값), P95(95백분위수), 최댓값, 95% CI(신뢰구간) | **최우선 위치 지표**. FK를 GT로 사용 금지 |
| `e_R` (회전 오차, deg) | `Log(R_GT^T R_pred)`로 계산한 GT와 예측 회전 사이의 측지각 | 평균, P50, P95, 최댓값, 95% CI | 위치만 보정하는 Ridge 모델은 이 지표를 개선할 수 없음 |
| `ADD` / `ADD-S` (물체 모델 평균 거리, mm) | GT·예측 자세로 변환한 물체 표면점 사이의 평균 거리 | 평균, P95, 95% CI | 큐브 방향을 구별하면 ADD, 대칭을 허용하면 ADD-S 사용 |

외부 GT는 A4/A5의 FK factor, correction label, intrinsic calibration과 독립이어야 한다. 독립성이
확보되지 않으면 지표 이름에 `FK-proxy` 또는 `internal`을 붙이고 1차 지표로 사용하지 않는다.

전체 6-DoF 우월성 주장은 `TRE_t` superiority, `e_R` superiority 또는 non-inferiority, 사전 선택한
ADD/ADD-S 계약을 모두 통과하는 intersection-union gate로 판단한다. 여러 baseline에 대한 동일 endpoint
비교는 Holm 보정하며, endpoint 간 gate 순서와 alpha 사용은 분석 전에 고정한다.

### 실제 로봇 조작 작업 지표(Task-level manipulation endpoints)

기여 C4를 주장하려면 아래 지표를 독립 외부 GT로 측정한 blind task에서 추가 보고한다. 단순 파지 성공률만
보고하면 파지 허용오차가 캘리브레이션 오차를 가릴 수 있으므로 연속 오차와 이진 성공률을 함께 사용한다.

| 지표(기호의 한글 뜻) | 정의 | 보고값 | 판정 역할 |
| --- | --- | --- | --- |
| `e_task,t` (작업 위치 오차, mm) | 명령한 목표 물체 또는 TCP(tool center point, 도구 중심점) 위치와 외부 장비로 측정한 실제 도달 위치 사이 거리 | 평균, P50, P95, 최댓값, 95% CI | 실제 위치 작업 정확도 |
| `e_task,R` (작업 회전 오차, deg) | 목표 방향과 실제 도달 방향 사이의 측지각 | 평균, P50, P95, 최댓값, 95% CI | 실제 6자유도 방향 정확도 |
| `S_grasp` (파지 성공률, %) | 사전 정의한 접촉·들기·유지 기준을 모두 통과한 blind trial 비율 | 성공/전체 횟수, 비율, 95% CI | 작업 성공 여부; 방법별 동일 물체·자세에서 짝 비교 |
| `F_task` (작업 실패율, %) | 인식 실패, 경로 계획 실패, 충돌 위험 중단, 파지 실패를 포함한 전체 실패 비율 | 원인별 횟수와 전체 실패율, 95% CI | 성공한 trial만 골라 평가하는 편향 방지 |

### FK 불확실성 모델 검증 지표(Uncertainty calibration endpoints)

공분산을 비용함수에 사용했다는 사실만으로 “불확실성 인지” 기여가 입증되지는 않는다. 학습에 쓰지 않은
반복측정에서 `Σ_FK`가 실제 FK 오차 분포를 올바르게 설명하는지 다음 지표로 검증한다.

| 지표(기호의 한글 뜻) | 정의 | 보고값 | 판정 역할 |
| --- | --- | --- | --- |
| `d_FK² = r_FK^T Σ_FK^-1 r_FK` (제곱 마할라노비스 거리) | FK 자세 잔차를 예측 공분산으로 정규화한 6차원 거리 | 평균, P50, P95, χ²(카이제곱, 6자유도) 기준 초과율 | 공분산이 실제 오차보다 과대·과소 추정되는지 확인 |
| `Cov_50`, `Cov_95` (50%·95% 타원체 포함률) | 실제 FK 오차가 예측한 50%·95% 신뢰 타원체 안에 든 비율 | 관측 포함률과 목표 포함률의 차이, bootstrap 95% CI | 공분산 calibration의 직접 검증 |
| `NLL_FK` (FK 음의 로그우도) | 실제 FK 잔차에 대해 공분산이 부여한 확률의 음의 로그값 | 평균·P95와 기준 모델 대비 차이 | 단순히 큰 공분산을 주어 포함률만 높이는 해 방지 |
| 축별 표준화 잔차 | 회전 3축·이동 3축 잔차를 각 축 표준편차로 나눈 값 | 축별 평균, 표준편차, 상관행렬 | 편향, 축별 scale 오류, 누락된 상관관계 확인 |

### 필수 안전장치와 운용 지표(Mandatory guardrails and operational endpoints)

| 지표(한글 뜻) | 정의 | 보고값 | 역할 |
| --- | --- | --- | --- |
| Tail error(꼬리 오차) | session별 P95 `TRE_t`, P95 `e_R` | 짝 차이와 95% CI | 평균 개선이 큰 오차 사례의 악화를 숨기지 않는지 확인 |
| Workspace error(작업공간별 오차) | 중앙/가장자리/근거리/원거리/높이 구간별 `TRE_t` | 구간별 P50/P95, 최악 구간 P95 | 학습 영역 안·밖의 성능을 분리 |
| Reliability(신뢰성) | 보정·추론 실패율과 `N_reg`(등록 카메라 수) | 짝 실패율 차이와 등록 카메라 수 | 실패 run을 오차 평균에서 조용히 제외하지 못하게 함 |
| Efficiency(효율성) | 실행시간, 반복 횟수, 수렴률, 필요한 view 수 | session별 분포 | 실용성과 수렴 안정성 |

P95, workspace, failure/coverage는 secondary라는 이름으로 완화할 수 있는 선택 지표가 아니라 최종 claim의
필수 guardrail이다.

### 진단 전용 지표(Diagnostic only)

| 지표(한글 뜻) | 정의 | 용도 |
| --- | --- | --- |
| Held-out raw-corner reprojection(홀드아웃 원시 코너 재투영) | 학습에 쓰지 않은 실제 2차원 코너에 동결된 변환을 재투영한 RMSE/P50/P95 | 픽셀 적합도와 일반화 진단 |
| Cross-camera disagreement(카메라 간 불일치, `e_cross`) | 같은 표적에 대한 카메라별 3차원 예측 자세의 차이(mm/deg) | 내부 정합 진단; 절대 정확도 아님 |
| Fixed↔gripper path disagreement(고정↔손목 경로 불일치, `e_e2e`) | 고정 카메라 묶음과 손목 카메라+FK 경로가 예측한 표적 자세 차이(mm/deg) | 경로 간 폐쇄오차 진단; 절대 정확도 아님 |
| Transform repeatability(변환 반복성) | 독립 재설치 session 간 `bTf`(기준→고정카메라), `gTc`(그리퍼→손목카메라) 변화(mm/deg) | 재현성 |
| Depth alignment(깊이 정합) | 점-평면 또는 점-CAD 거리(mm) | 크기 비율과 3차원 기하 교차검증 |
| Geometry/FK checks(기하·FK 검사) | 큐브 치수 일관성, 명령 상대운동 일관성, FK-proxy 위치 오차 | 기하·scale·운동학 오류 진단 |
| Solver diagnostics(최적화 진단) | 조건수, Jacobian rank, 잔차 분포 | 퇴화·수렴 원인 분석 |

진단 지표만으로 절대 3차원 정확도 또는 외부 방법 대비 우위를 결론 내리지 않는다. 마커 재투영 오차를
정의할 수 없는 무표적(targetless) 방법은 공통 외부-GT 지표로만 직접 순위를 비교한다.

`e_X`처럼 `T_base_Ci`와 `T_gripper_cam`을 하나로 평균한 숫자는 사용하지 않는다. Simulation 또는
외부 transform GT가 있는 경우 다음을 분리 보고한다.

- 고정 카메라 `bTf`: 위치 오차(mm) / 회전 오차(deg)
- hand-eye `gTc`: 위치 오차(mm) / 회전 오차(deg)
- 카메라 간 상대변환: 위치 오차(mm) / 회전 오차(deg)

### 평가지표 충분성 판정

**결론:** 최종 실험 명세의 지표 구성은 캘리브레이션 논문의 정확도·재현성·신뢰성을 평가하기에 대체로
충분하다. 위에서 추가한 **FK 공분산 calibration 지표**와 **실제 조작 작업 지표**까지 실행하면 네 기여를
직접 검증할 수 있다. 그러나 **현재 session02 결과만으로는 평가지표가 충분하지 않다.** 지금 실제로
계산된 값은 주로 픽셀 적합도와 내부 경로 일관성이며, 절대 정확도·불확실성 적합도·작업 성공을 측정하지
못했기 때문이다.

| 평가 축 | 필요한 지표 | 명세에 정의됨 | session02에서 실제 계산됨 | 충분성 판정 |
| --- | --- | ---: | ---: | --- |
| 절대 위치 정확도 | `TRE_t`(표적 위치 등록오차) | 예 | 아니오 | **부족** — 독립 외부 GT 필요 |
| 절대 회전 정확도 | `e_R`(회전 오차) | 예 | 아니오 | **부족** — 6자유도 우월성 판정 불가 |
| 물체 전체 자세 정확도 | `ADD` 또는 `ADD-S`(물체 모델 평균 거리) | 예 | 아니오 | **부족** |
| 픽셀 일반화 | 홀드아웃 raw-corner RMSE/P50/P95 | 예 | RMSE만 계산 | **부분 충족** — P50/P95·오차 분포 추가 필요 |
| 카메라 경로 일관성 | `e_cross`, `e_e2e`(카메라 간·경로 간 불일치) | 예 | 예; 9 pairs, 3 units | **예비 진단만 가능** — 평가 단위가 너무 적음 |
| FK 불확실성 적합도 | `d_FK²`, `Cov_50/95`, `NLL_FK` | 이번 업데이트에서 추가 | 아니오; placeholder `Σ_FK` 사용 | **부족** — 기여 C3의 핵심 공백 |
| 실제 조작 성능 | `e_task,t`, `e_task,R`, `S_grasp`, `F_task` | 이번 업데이트에서 추가 | 아니오 | **부족** — 기여 C4 미평가 |
| 독립 재설치 반복성 | `bTf`, `gTc` session 간 변화 | 예 | 아니오; 독립 session 1개 | **부족** |
| 실패·등록 신뢰성 | 실패율, `N_reg` | 예 | `N_reg`와 optimizer 수렴만 계산 | **부분 충족** — 실제 capture/inference 실패율 필요 |
| 작업공간·꼬리 오차 | 구간별 P50/P95, 최악 구간 P95 | 예 | 아니오 | **부족** |
| 효율성 | 시간, 반복 횟수, 수렴률, 필요한 view 수 | 예 | 일부 solver 기록만 존재 | **부분 충족** |
| 통계적 일반화 | 독립 session 단위 짝 계층 bootstrap 95% CI | 예 | 아니오; session 1개 | **부족** — 3 multi-start는 독립 표본이 아님 |
| 외부 기준선 | C1–C4 및 사전 고정 외부 방법 | 예 | 미실행 | **부족** — 비교 우위 주장 불가 |

최종 논문 결과가 충분해지기 위한 최소 조건은 다음과 같다.

1. 실측 반복자료로 `Σ_FK`를 만들고, 별도 홀드아웃 반복측정에서 포함률·마할라노비스 거리·NLL을 검증한다.
2. 캘리브레이션과 FK 제약에 독립적인 외부 장비로 blind 6자유도 표적 자세를 측정한다.
3. 독립 카메라 재설치 session을 반복하고, 동일 blind pose를 모든 방법에 짝지어 평가한다.
4. 각 session에서 중앙/가장자리/거리/높이/회전 구간을 균형화하고 평균뿐 아니라 P95와 최악 구간을 보고한다.
5. 같은 목표 pose와 물체를 사용한 실제 파지·배치 trial에서 연속 작업 오차와 성공·실패율을 함께 보고한다.
6. C1–C4와 사전 고정한 외부 기준선을 동일 입력·동일 외부-GT 지표로 실행한다.

## 외부 기준값의 불확실성 계약(External-GT uncertainty contract)

외부 GT 장비의 제조사 정격 정확도만 인용하지 않고 실제 작업 거리와 workspace에서 반복 측정한다.
Session별로 다음 성분을 별도 기록한다.

- Translation/rotation 반복측정 분산과 시간 drift
- GT rigid body·jig의 탈착/장착 반복성
- Cube object frame과 GT marker/reflector frame 사이 `T_cube_GTmarker`의 추정 불확도
- GT frame을 robot base로 옮기는 `T_base_GT` registration 불확도
- RGB capture와 GT measurement 사이의 시간 정합 오차
- 위 성분을 합성한 translation/rotation measurement uncertainty floor

GT 등록에 calibration train observation이나 A4 FK anchor를 재사용하지 않는다. 방법 간 차이가 합성
uncertainty floor와 비슷하거나 더 작으면 통계적 유의성과 별개로 “실질적으로 더 정확하다”는 표현을
사용하지 않는다.

## 통계 계약과 최종 합격 조건

- 통계 단위는 frame이나 split이 아니라 **독립 camera-installation session**이다.
- 같은 session의 동일 held-out poses에서 모든 방법을 paired 평가한다.
- 방법별 mean과 함께 P50/P95/max를 보고하되, 추론의 최상위 단위는 session으로 유지한다.
- 95% CI는 paired hierarchical bootstrap으로 계산한다.

  1. 동일 method pair가 있는 session을 paired 상태로 복원추출한다.
  2. 선택된 각 session 안에서 동일 blind pose ID를 두 방법에 공통으로 복원추출한다.
  3. 각 session의 paired metric difference를 계산하고 session에 동일 가중치를 주어 집계한다.
  4. 사전 고정한 bootstrap 반복 수와 CI 방식으로 구간을 산출한다.

- Pose만 독립 표본처럼 재표집하거나 pose가 많은 session에 더 큰 가중치를 주어 CI를 인위적으로 좁히지 않는다.
- A4의 confirmatory 비교군은 결과를 보기 전에 `A1, A2, A3, B1, C1–C4`로 고정한다. A0/B2/B3는
  관측성·기여 분석, A5는 별도 extension family로 취급한다.
- Confirmatory TRE 비교의 p-value는 위 비교군 전체에 Holm 보정한다. Rotation과 ADD/ADD-S의 검정
  family 및 hierarchical gate 순서도 결과 열람 전에 고정한다.
- 실패 run도 failure rate에 포함하며 성공 run만 골라 정확도를 비교하지 않는다.
- 한 방법이 실패해 paired TRE가 정의되지 않는 session 수를 별도로 보고한다. Claim은 충분한 complete
  session pair와 failure-rate 비열등성을 모두 만족해야 하며, 실패 session을 단순 삭제하지 않는다.
- Hyperparameter, `Sigma_FK`, robust threshold, correction model은 training-only로 선택한다.
- Pilot 5 sessions는 분산·실패율 추정과 power analysis용이며 최종 우월성 검정 표본으로 확정하지 않는다.

Ours-core A4가 우수하다고 판정하려면 다음을 모두 만족해야 한다.

1. 사전 고정한 confirmatory baseline 각각에 대해 `delta TRE = TRE_A4 - TRE_baseline`의 hierarchical
   paired bootstrap 95% CI 상한이 0보다 작고 Holm-adjusted 검정을 통과한다.
2. 회전오차가 유의하게 낮거나 `UpperCI(e_R,A4 - e_R,baseline) < m_R`을 만족한다.
3. 사전 선택한 ADD/ADD-S를 confirmatory endpoint로 주장한다면 그 사전 정의 superiority 또는
   non-inferiority 계약을 통과한다.
4. `UpperCI(P95_TRE,A4 - P95_TRE,baseline) < m_P95`를 만족한다.
5. `UpperCI(FailureRate_A4 - FailureRate_baseline) < m_fail`을 만족하고 등록 카메라 수가 줄지 않는다.
6. `UpperCI(worst-stratum P95_A4 - worst-stratum P95_baseline) < m_stratum`을 만족해 특정 workspace
   구간의 열화를 전체 평균으로 숨기지 않는다.

여기서 `m_R`, `m_P95`, `m_fail`, `m_stratum`은 task tolerance와 GT uncertainty를 근거로 결과 확인 전에
고정한다. “악화되지 않음”을 point estimate 비교로 판정하지 않는다. Margin이 0보다 크면 허용 가능한
악화량을 뜻하므로 그 공학적 근거를 함께 기록한다.

A5는 calibration transform을 변경하지 않고 held-out target prediction에만 correction을 적용한다.
A4 대비 translation, rotation, tail, failure/coverage의 같은 계약을 통과할 때만 Ours-full로 승격한다.
그렇지 않으면 A4를 최종 방법으로 유지하고 correction은 supplementary 결과로 보고한다.

## 현재 실제 데이터가 보여주는 범위

아래는 2026-08-13에 다시 실행한 **session02 안정 구간 홀드아웃 software 결과**다. 모든 숫자는
동일 event split과 동일 raw-corner 평가 mask를 사용한다. `e_cross`와 `e_e2e`는 내부 경로 일관성이고
외부 GT 정확도가 아니다. 숫자는 3개 초기화 run의 평균이며 모든 실행이 수렴했다.

| 방법 | 수렴 횟수 | 홀드아웃 재투영 RMSE: 전체 / 보드 / 큐브(px) | `e_cross`(고정 카메라 간 불일치, mm/deg) | `e_e2e`(고정↔손목 경로 불일치, mm/deg) | 판정 범위 |
| --- | ---: | ---: | ---: | ---: | --- |
| A0 | 3/3 | 2.4829 / 2.4829 / — | — | — | 보드만 사용한 순차 보정 진단 완료 |
| A1 | 3/3 | 3.1267 / 2.5103 / 4.5317 | 13.7546 / 1.2193 | 4.2459 / 1.0239 | 순차 보정의 공통 영상 전처리 결과 |
| A2 | 3/3 | 3.1154 / 2.5038 / 4.5109 | 13.1276 / 1.1910 | 4.0693 / 1.0385 | 영상만 사용하는 통합 보정 기준선 |
| A3 | 3/3 | **2.4997** / 2.4952 / **2.5136** | **10.8320** / **1.0602** | 4.7904 / 1.1893 | 픽셀·카메라 간 불일치 최저; 외부 GT 우위는 미검증 |
| A4 예비실험 | 3/3 | 3.1030 / 2.5038 / 4.4758 | 13.0590 / 1.1901 | 4.0396 / 1.0316 | 임시 공분산 진단만 허용 |
| A5 | 미실행 | — | — | — | 6자유도 보정·독립 GT label 없음 |
| B1 예비실험 | 3/3 | 3.1128 / 2.5101 / 4.4927 | 13.6410 / 1.2158 | 4.2097 / 1.0186 | A4와 동일한 임시 공분산 사용 |
| B2 예비실험 | 3/3 | 4.4938 / — / 4.4938 | 11.9911 / 1.2833 | **3.8443** / **0.9315** | 큐브만 사용; A4와 동일한 임시 공분산 사용 |
| B3 | 3/3 | 2.4829 / 2.4829 / — | — | — | 보드만 사용하는 통합 보정 진단 완료 |

사용 데이터와 평가 계약은 다음과 같다.

- 입력: `data/session02/calib_train`, 안정적인 placement set 5–12의 48 events와 저장 RGB 72장
  (`cam0/1/3`: 각 8장, gripper `cam2`: 48장). Set 1–4는 cam0 이동 전 구간이라 제외했고,
  set13은 공통 event-stratified 관측 구조가 달라 제외했다.
- 역할: `cam0`, `cam1`, `cam3`는 고정 카메라, `cam2`는 eye-in-hand 카메라다. Intrinsics는
  `intrinsics/cam0.npz`–`cam3.npz`, 로봇 pose와 capture grouping은 `meta.json`, cube geometry는
  `config.py:CubeConfig`를 사용했다.
- Split: seed `20260731`, set별 5 train + 1 test event. Train event 40개, held-out event 8개
  (`29, 38, 46, 50, 53, 61, 65, 75`)이며 test-time refit은 없다.
- 관측량: train board 54 observations / 2,724 corners, train cube 55 / 768; held-out board
  17 / 706, held-out cube 17 / 228이다.
- 공통 path mask: SHA-256 `4339c33d283052a80ba07f715f1091e17b344179dbc9018c6f929a9948f9fc80`,
  9 cross-camera pairs와 3 eye-to-eye units. 출력값에 따른 pose gate나 outlier 제거는 없다.
- 공통 solver: raw-corner pixel backend, `soft_l1`, `f_scale=2 px`, 최대 300 evaluations,
  3 multi-start. A4/B1/B2는 실측 파일 대신 진단용 `sigma_t=3 mm`, `sigma_R=0.3 deg`,
  FK Huber scale 2.5를 썼다.

주요 짝 차이(`Δ`, 변화량)는 `후자 − 전자`이며 음수일수록 해당 오차가 개선된 것이다.

| 짝 비교 | 공정 비교 지표 | `Δ`(후자−전자) | 해석 |
| --- | --- | ---: | --- |
| A0→A1 | 보드 재투영 RMSE | +0.0274 px | 이 subset에서는 둘 다 고정 카메라 3대 등록; 큐브 관측성 이득 없음 |
| A1→A2 | 전체 재투영 RMSE | −0.0113 px | 통합 최적화 효과는 있으나 매우 작음 |
| A2→A3 | 전체 재투영 / `e_cross` 위치 / `e_e2e` 위치 | −0.6157 px / −2.2955 mm / +0.7211 mm | hard-FK는 픽셀·고정 카메라 간 일관성 개선, 고정↔손목 경로는 악화 |
| A2→A4 | 전체 재투영 / `e_cross` 위치 / `e_e2e` 위치 | −0.0124 px / −0.0685 mm / −0.0297 mm | 임시 공분산을 쓴 soft-FK 이득은 미미 |
| B1→A4 | 전체 재투영 / `e_cross` 위치 / `e_e2e` 위치 | −0.0098 px / −0.5819 mm / −0.1701 mm | 통합 최적화의 내부지표 이득은 작음 |
| B2→A4 | 큐브 재투영 / `e_cross` 위치 / `e_e2e` 위치 | −0.0180 px / +1.0680 mm / +0.1953 mm | 보드 추가 시 픽셀은 소폭 개선되나 경로 위치 일관성은 악화 |
| B3→A2 | 보드 재투영 RMSE | +0.0209 px | 큐브 추가가 보드 픽셀 적합도를 개선하지 않음 |

전체 원시 결과와 per-camera 값은
[`CP_result/session02/late_table1/TABLE1_RESULTS.md`](CP_result/session02/late_table1/TABLE1_RESULTS.md)에
정리했다. A4는 실측 `Sigma_FK`와 독립 외부 GT 없이 confirmatory 결과로 승격하지 않는다.

### 결과에서 허용되는 주장

| 주장 | 반드시 필요한 근거 | 현재 상태 |
| --- | --- | --- |
| 큐브가 관측성을 높인다 | `A0→A1`의 짝 관측 범위와 등록률 결과 | 현재 late subset에서는 등록률 차이 없음 |
| 통합 최적화가 순차 보정보다 낫다 | 공통 픽셀 backend의 `A1→A2` | 내부 지표상 `−0.0113 px`; 외부 GT 미평가 |
| 통합 최적화가 독립 보정보다 낫다 | 정보량을 맞춘 주 공정 비교 `B1→A4` | 내부 지표의 소폭 개선만 확인 |
| soft-FK가 hard-FK보다 낫다 | `A3→A4a`와 외부-GT 작업 결과 | 현재 내부 지표는 A3가 더 강함 |
| 공분산 가중과 강건 FK가 기여한다 | `A4a→A4b→A4c` 분해 실험과 FK 공분산 calibration | 실측 `Σ_FK`가 없어 미입증 |
| Ours-core가 외부 방법보다 정확하다 | 독립 외부 GT와 최종 CI·실패율·관측 범위 계약 전체 통과 | 미평가 |
| 후처리 보정이 최종 기여다 | 캘리브레이션을 고정한 `A4→A5`가 동일 계약 통과 | A5 미구현 |

외부 GT 완료 전에는 “Ours가 가장 정확하다” 또는 “절대 3D 정확도 2.034 mm”라고 표현하지 않는다.

### 이전 아티팩트

현재 집계의 상세 provenance는
[Session02-late Table 1 held-out 결과](CP_result/session02/late_table1/TABLE1_RESULTS.md)에 있다.
아래는 역사적 디버깅·scale 감사 자료이며 이번 집계에는 사용하지 않았다.

- [Canonical repeated ablation](CP_result/session01/validation/ablation_multisplit/multisplit_ablation.md)
- [Cross-target cube evaluation](CP_result/session01/validation/cross_target_cube/cross_target_cube.md)
- [Soft-anchor event split](CP_result/session01/diagnostics/D2_anchored_event_split/D2_anchored_event_split.md)
- [위치 hold-out 2×2 (D1)](CP_result/session01/diagnostics/D1_fk_correction_2x2/D1_fk_correction_2x2.md)
- [실험 설명과 제한](CP_EXPERIMENTS_README.md)

### 2026-08-06 robot scale 통일

D1·D2 아티팩트는 `RB_ROBOT_POS_SCALE=1.0229`로 만들어져 이 표의 나머지 행(k=1.0)과 기준이
달랐다. 두 실험을 pinned k=1.0으로 재실행해 표 전체를 하나의 scale 기준으로 통일했다.
`CP_common.ROBOT_POS_SCALE_PINNED`는 이제 유일한 k 정의점이며, 저장 아티팩트를 읽는 경로에도
scale 가드가 걸려 있다.

- **D2**: Table 1의 A2/A3를 **0.00σ**로 재현한다(k=1.0229 아티팩트는 1.51σ였다).
- **D1**: 최저 셀이 **A2@λ=3 + Ridge (1.586 mm) → A3 + Ridge (2.034 mm)**로 바뀌었다. 즉
  k=1.0에서는 채택 후보였던 soft-anchor arm이 최저 셀이 아니고, hard-FK(A3)가 최저다.
  보정 자체의 효과는 오히려 커졌다(Ridge: A2 −2.361 mm t=−3.86, A3 −2.227 mm t=−5.13, 각 12/13
  fold). FK 고정 여부는 13개 위치로 여전히 판정되지 않는다(A3−A2 @ ridge `t=−1.66`, 9/13).
- 보정 전 오차는 k=1.0에서 arm 전체가 1.2–2.6 mm 크다. FK-proxy 지표가 FK 자체를 기준으로
  삼으므로 이는 순환 논증이며, k의 물리적 확정 근거로 사용하지 않는다.

## 실행 계약

### 촬영 단계 강제사항

- `Step2_capture.py`에서 `--root_folder`를 생략하면 `data/sessionNN/calib_train`을 원자적으로 새로 만들고,
  `NN`은 기존 번호의 최댓값에 1을 더한다. 기존 번호는 빈 폴더라도 자동 재사용하거나 덮어쓰지 않는다.
- 과거 session을 의도적으로 이어 촬영할 때만 `--root_folder data/sessionNN/calib_train`을 명시한다.
- `Step2_capture.py`는 `A_placement`와 `B_eyetohand`의 관측 요구를 분리 적용한다.
- A/B 태그와 `cube_gripped` 상태가 모순되거나 알 수 없는 block이면 저장하지 않는다.
- 자동 촬영은 gate 실패 frame을 성공으로 집계하지 않으며 `force_save`는 기본 차단한다.
- Cross-camera frame 선택과 span은 공통 host-monotonic receipt clock을 사용하고, 장치별 timestamp/domain은
  진단용으로 별도 보존한다.
- `server/robot_calb.py`는 `safe_joints_empty`, `safe_joints_gripped`가 모두 없는 waypoint를 첫 모션 전에
  거부하고, 각 capture를 `safe→target→safe`로 실행한다.
- 위 코드 강제는 물리 경로의 충돌 안전성을 증명하지 않는다. 두 payload 상태의 전체 경로를 저속
  dry-run으로 검증한 뒤에만 최종 촬영한다.

### 데이터·분할

- 모든 방법은 동일 source session, camera intrinsics, robot poses, train/test session split과 blind external-GT
  poses를 사용한다.
- 방법이 요구하지 않는 target/depth 정보를 억지로 제공하지 않고, 실제 사용한 RGB/depth/robot pose,
  target 종류와 optimization level(pixel/pose)을 결과 표에 공개한다.
- Classical pose-level 방법에는 동일한 frozen PnP 결과와 motion pairs를 입력한다.
- Pixel-level 방법에는 동일한 raw corner detections와 visibility mask를 입력한다.
- Targetless 방법을 추가하면 동일 시점 raw RGB를 제공하되 texture/overlap 등 다른 operating condition을
  명시하고 공통 외부-GT endpoint로만 직접 순위를 비교한다.
- Session 전체를 train/test로 분리한다. 같은 물리 위치의 프레임이 양쪽에 섞이지 않게 한다.
- Position holdout은 center/edge/near/far/height를 stratify한다.
- 공통 카메라 교집합 정확도와 전체 카메라 coverage를 함께 보고한다.
- 추가로 등록한 카메라의 코너를 정확도 평균에서 빼거나, 실패 카메라를 조용히 제외하지 않는다.

### 공통 frontend·solver 예산

- 같은 optimization level 안에서는 동일 corner detector 결과 또는 완전히 동일한 재검출 설정을 사용한다.
- Pose-level 방법끼리는 동일 PnP-RANSAC과 marker rejection을, pixel-level 방법끼리는 동일 visual robust
  loss와 outlier threshold를 사용한다.
- A2/A4와 직접 인과 비교하는 iterative arm은 동일 multi-start 수, 최대 iteration, 종료조건을 사용한다.
  Closed-form 등 solver 구조가 다른 방법은 해당 조건이 다름을 공개하고 동일 외부-GT endpoint로 평가한다.
- 실패·예외를 누락하지 않고 failure reason과 함께 저장한다.
- Train reprojection은 진단용이고 최종 순위는 held-out/외부-GT 지표로 정한다.

### Simulation

Simulation은 실제 실험 전 sanity check와 GT transform 오차 분석에 사용한다. 최종 재실행 전 다음을
수정해야 한다.

- visual-only/independent solver에서 `fk_cube`와 GT board pose를 초기값·해에 사용하지 않는다.
- independent rigid alignment를 실제 prediction에 적용한다.
- noisy raw 2D corners와 visibility mask를 저장하고 방법별 held-out reprojection을 계산한다.
- 실물 59 mm cube, marker center/roll, 카메라별 K/D와 실제 camera pose를 사용한다.
- 예외를 숨기지 않고 seed 단위 실패율을 보고한다.
- 최소 30 independent seeds와 사전 정의한 random/stratified splits를 사용한다.
- `bTf`와 `gTc` GT 오차를 분리한다.

Simulation 결과는 실제 외부-GT 결과를 대신하지 않는다.

## 확정 전 사용자 판단 항목

아래 항목은 코드가 자동으로 결정할 수 없으며 실험 책임자가 사전에 확정해야 한다.

1. **최종 주장 범위**
   - 권장: 우선 A4를 Ours-core로 고정한다.
   - A5는 6-DoF correction이 A4 대비 외부 GT에서 통과한 후에만 최종 방법으로 채택한다.
   - 위치-only correction을 유지하면 논문 주장도 “3D translation localization”으로 제한해야 한다.

2. **독립 외부 GT 장비·방법**
   - 우선순위: laser tracker/측정암 → motion capture rigid body → 정밀 가공 3D jig.
   - Robot FK, A4 anchor에 사용한 값, correction label은 외부 GT로 재사용할 수 없다.
   - 어떤 장비를 사용할지와 장비 정확도 사양을 기록해야 한다.

3. **로봇 위치 스케일**
   - `CP_common.ROBOT_POS_SCALE_PINNED`(현재 `1.0`)과 `1.0229` 중 하나를 기존 결과에 맞춰
     선택하지 않는다. 값은 환경변수가 아니라 이 상수 하나이며, 바꾸면 저장된 결과를 전부 재생성한다.
   - 인증 길이 기준물·depth·외부 계측으로 물리적으로 확정한 뒤 모든 결과를 한 값으로 재생성한다.

4. **Non-inferiority margins**
   - `m_R`, `m_P95`, `m_fail`, `m_stratum`을 실제 task 허용오차와 GT uncertainty로 정한다.
   - 결과를 본 뒤 margin을 정하거나 pilot의 관측 효과크기를 margin으로 사용하면 안 된다.
   - Failure margin은 session 실패가 실제 운용에 미치는 비용을 반영하고, 등록 카메라 수 감소 허용 여부도
     별도로 고정한다.

5. **재촬영 규모**
   - 권장 pilot: 독립 재설치 5 sessions × session당 blind test pose 30개.
   - Pilot 분산으로 power analysis 후 최종 session 수를 확정한다.
   - Workspace center/edge/near/far/height와 cube orientation을 균형화한다.

6. **Classical baseline 범위**
   - 권장: C1–C4 모두 실행한다. 동일 입력으로 실행 비용이 작고 선택적 baseline이라는 비판을 줄인다.
   - 논문 Main Ablation에는 넣지 않고 별도 표로 보고한다.

7. **ADD와 ADD-S 선택**
   - Cube의 면 ID와 방향이 task에서 구분되면 ADD를 사용한다.
   - 대칭 방향을 동일 pose로 인정하는 task면 symmetry set을 사전 정의하고 ADD-S를 사용한다.

8. **Recent/targetless 외부 baseline 범위**
   - C1–C4를 SOTA로 부르지 않는다.
   - 추가 방법은 공개 코드, 입력 modality, GPU/의존성, target/scene overlap 적합성을 확인한 뒤 별도
     comparison contract에 사전 등록한다.

## 직접 실행해야 하는 물리 작업

- 큐브 변, marker 크기·중심·roll, ChArUco square 길이를 캘리퍼로 재측정한다.
- 카메라별 30–50장 intrinsic session을 중앙·모서리·거리·tilt ±30–45°로 다시 촬영한다.
- Exposure/gain/focus를 고정하고 target ROI sharpness 및 saturation gate를 활성화한다.
- 블록별 `capture_gate` threshold를 기존 데이터와 pilot에서 검증하고 실제 PASS frame만 저장한다.
- Cube placement는 각 면과 각 카메라의 관측 수를 균형화한다.
- Gripped cube는 위치와 roll/pitch/yaw가 다양한 30–50 자세로 촬영한다.
- 외부 GT 지그/장비로 calibration에 쓰지 않은 blind poses를 측정한다.
- 카메라 재설치 session을 반복하고 각 session의 환경·설정·config hash를 기록한다.

촬영부터 전체 결과 산출까지는
[단일 실행 체크리스트](Check_Calibration.md)를 위에서부터 순서대로 따른다.

## 남은 구현 순서

1. Block-aware gate·host-clock sync·safe-pose executor는 코드 반영 완료. 기존 데이터 replay와 저속
   physical dry-run으로 threshold/경로 검증
2. Target-ROI sharpness·clipping·marker-edge gate를 Step2 저장 조건에 구현하고 카메라별 threshold 고정
3. A2/A3/A4a/A4b/A4 공통 pixel backend와 fair B1/B2는 `CP_final_methods.py`에 구현 완료. 실측 covariance와
   첫 자동 번호 engineering-pilot session으로 production 검증
4. Blind prediction exporter와 external-GT hierarchical evaluator는 구현 완료. 실제 GT schema·failure
   logging으로 end-to-end 검증
5. B2 공정 arm production 검증과 C1–C4 same-session classical baseline 연결
6. 필요 시 A5 6-DoF SE(3) correction 구현
7. Simulation 누출·metric 문제 수정 후 30-seed 재실행
8. 기존 데이터 face-roll/PnP 재처리로 전체 파이프라인 검증
9. 외부 GT 포함 재촬영 및 session-paired 최종 평가
10. Paired hierarchical bootstrap CI·Holm 보정·P50/P95/failure table 생성

최종 논문 문장은 9–10단계의 외부-GT 평가와 통계 계약을 통과한 이후에만 확정한다.
