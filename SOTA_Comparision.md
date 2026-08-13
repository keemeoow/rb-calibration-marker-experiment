# 비교 대상 방법 정리 (SOTA baselines)

> Version: 2.0 · 기준일: 2026-08-13
> 대상 독자: 비교 실험을 맡은 학부연구생
> 우리 제안 방법(A/B 계열)과 우위 판정 기준은 `SOTA_Claim_Protocol.md` 참조

---

## 0. 이 문서를 읽는 법

우리가 만든 캘리브레이션 방법이 **기존 방법보다 정말 나은지** 확인하려면,
기존 방법을 **같은 데이터로 직접 돌려서** 비교해야 한다.
이 문서는 그 "기존 방법" 10가지가 각각 무엇이고 어떻게 돌리는지 정리한 것이다.

**논문을 다 읽을 필요는 없다.** 각 방법마다 아래 세 가지만 알면 실험은 돌아간다.

1. 이 방법이 **무엇을 입력받아 무엇을 내놓는가**
2. 이 방법이 **무엇을 가정하는가** (= 무엇을 못 하는가)
3. **어떻게 실행하는가** (OpenCV 한 줄인가, 남의 저장소를 돌려야 하는가)

자기가 맡은 방법의 논문만 초록 수준으로 보면 된다.
Tsai를 맡은 사람이 Calib3R 논문을 읽을 이유는 없다.

### 용어 주의

이 문서에서 **고전 방법(C 계열)을 "SOTA"라고 부르지 않는다.**
오래된 방법은 "기준선(baseline)"이지 최신 기술이 아니다.
반대로 **출판연도가 최신이라는 이유만으로 실험 셋업이 다른 방법을 경쟁자로 취급하지도 않는다.**
비교는 "같은 조건에서 같은 일을 하는 방법"끼리만 의미가 있다.

---

## 1. 먼저 알아야 할 개념 세 가지

### 1.1 hand-eye calibration이란

로봇은 **자기 손이 어디 있는지**를 알고(관절 센서 → FK),
카메라는 **물체가 어디 있는지**를 안다(사진 → PnP).
그런데 두 값의 기준이 달라서 그대로는 못 합친다.

**둘 사이의 고정된 변환을 찾는 것**이 hand-eye calibration이다.
이 변환만 알면 카메라가 본 위치를 로봇이 갈 수 있는 좌표로 바꿀 수 있다.

### 1.2 카메라를 어디에 두는가 — 두 가지 구성

| 구성 | 카메라 위치 | 찾는 것 | 다른 이름 |
| --- | --- | --- | --- |
| **eye-in-hand** | 로봇 손목에 부착 | 손 ↔ 카메라 | eye-on-hand |
| **eye-to-hand** | 작업대에 고정 | 로봇 베이스 ↔ 카메라 | eye-on-base |

**같은 것을 다른 이름으로 부르는 경우가 많으니 문서마다 확인해야 한다.**
OpenCV와 easy_handeye는 `eye-in-hand` / `eye-on-base`라고 쓴다.

우리 프로젝트는 **고정 카메라 여러 대 + 손목 카메라 하나**를 같이 쓴다(mixed 구성).
아래 방법 대부분은 **둘 중 하나만** 지원한다. 그래서 비교가 까다롭다.

### 1.3 `AX=XB` 와 `AX=ZB` — 방법을 가르는 축

이 두 식이 방법을 나누는 가장 큰 기준이다. 표기는 문헌마다 다르다
(`AX=YB`, `AX=ZB`는 같은 문제를 가리킨다).

**`AX=XB`** — 미지수 **하나**

로봇을 두 자세로 움직였을 때,
`A` = 로봇 손이 움직인 양, `B` = 카메라가 움직인 양, `X` = 손-카메라 변환.
"손이 이만큼 움직였으면 카메라도 이만큼 움직여야 한다"는 관계를 여러 쌍 모아 `X`를 푼다.

- 최소 3자세 필요. 단, **회전축이 서로 평행하면 안 된다.**
- 카메라가 세상 어디에 있는지는 **안 구한다.**

**`AX=ZB`** — 미지수 **둘**

`X`(손-카메라)와 `Z`(로봇 베이스-물체) 를 **동시에** 구한다.

- 우리 문제에 더 가깝다. 우리도 카메라 위치와 물체 위치를 같이 찾기 때문.
- **공정한 비교 상대는 `AX=XB`가 아니라 `AX=ZB` 계열이다.**

### 1.4 자세 단위 vs 픽셀 단위

| | 무엇을 최소화 | 특징 |
| --- | --- | --- |
| **pose-level** | 자세끼리의 차이 | PnP로 먼저 자세를 뽑고 그 자세들을 맞춤 |
| **pixel-level** | 코너 재투영오차 | 사진의 코너를 직접 씀. bundle adjustment 계열 |

C 계열과 D1은 pose-level, D2·D3는 pixel-level이다.
우리 방법은 pixel-level이라 **D3가 가장 가까운 비교 대상**이다.

---

## 2. 비교군 한눈에 보기

| ID | 방법 | 연도 | 계열 | 카메라 여러 대 | 픽셀 단위 | FK 불확실성 모델 | 실행 난이도 |
| --- | --- | --- | --- | --- | --- | --- | --- |
| C1 | Tsai–Lenz | 1989 | classical `AX=XB` | 아니오 | 아니오 | 없음 | ★ OpenCV 한 줄 |
| C2 | Park–Martin | 1994 | classical `AX=XB` | 아니오 | 아니오 | 없음 | ★ OpenCV 한 줄 |
| C3 | Horaud | 1995 | classical `AX=XB` | 아니오 | 아니오 | 없음 | ★ OpenCV 한 줄 |
| C4 | Daniilidis | 1999 | dual quaternion | 아니오 | 아니오 | 없음 | ★ OpenCV 한 줄 |
| D1 | Shah | 2013 | robot-world `AX=ZB` | 아니오 | 아니오 | 없음 | ★ OpenCV 한 줄 |
| D2 | Tabb–Ahmad Yousef | 2017 | iterative `AX=ZB` | 지원 | 설정에 따름 | 없음 | ★★★ 외부 코드 |
| D3 | Allegro et al. | 2024 | multi-camera 재투영 | **예** | **예** | robot pose를 고정 입력 | ★★★ 외부 코드 · **최우선** |
| D4 | Calib3R | 2025 | targetless 3D foundation | 예 | 2D/3D 항 | robot motion 항 | ★★★★ GPU · 조건부 |
| D5 | Ha | 2023 | probabilistic `AX=ZB` | 아니오 | 아니오 | **측정별 noise 모델** | ★★★ 외부 코드 |
| D6 | Ulrich–Hillemann | 2024 | uncertainty-aware | 아니오 | 설정에 따름 | **robot pose 보정** | ★★★ |

`FK 불확실성 모델 = 없음`은 **"로봇이 알려준 자세를 그냥 정답으로 받아들인다"**는 뜻이다.
어떤 형태의 noise 처리도 전혀 없다는 뜻은 아니다.

**어디부터 시작할까**
`★` 다섯 개(C1–C4, D1)는 OpenCV 함수 호출이라 하루면 다 돌린다. **여기부터 시작한다.**
`★★★` 은 남의 저장소를 우리 데이터 형식으로 바꿔 넣어야 해서 대부분의 시간이 여기 든다.

---

## 3. C 계열 — 고전 hand-eye (OpenCV 내장)

네 방법 모두 **같은 함수, 다른 옵션**이다. 입력도 출력도 같다.

```python
import cv2

R_cam2gripper, t_cam2gripper = cv2.calibrateHandEye(
    R_gripper2base, t_gripper2base,   # 로봇 FK에서 얻은 손 자세들
    R_target2cam,   t_target2cam,     # PnP에서 얻은 타깃 자세들
    method=cv2.CALIB_HAND_EYE_TSAI,   # ← 여기만 바꾸면 C1~C4
)
```

사용 가능한 `method` 상수:
`CALIB_HAND_EYE_TSAI` / `_PARK` / `_HORAUD` / `_ANDREFF` / `_DANIILIDIS`
(우리 비교군은 이 중 네 개를 쓴다. Andreff는 저장소 코드에서 초기값으로만 사용 중)

### 실행 시 가장 흔한 실수

이 네 가지에서 시간을 날리는 경우가 거의 전부다. **에러 없이 돌아가고 그럴듯한 숫자가 나오므로
틀린 줄도 모른다.**

| 실수 | 결과 |
| --- | --- |
| `R_gripper2base` 자리에 `base2gripper`를 넣음 (**방향 반대**) | 조용히 틀린 값 |
| mm와 m을 섞어 씀 | 1000배 틀리거나, 미묘하게만 틀림 |
| 회전축이 거의 평행한 자세만 수집 | 해가 불안정 |
| eye-in-hand용 입력을 eye-to-hand에 그대로 씀 | 구성마다 넣는 변환이 다름 |

**확인 방법:** 시뮬레이션(`CP_synthetic_7row.py`)은 정답을 알고 있다.
거기서 먼저 돌려 정답이 나오는지 보고 실데이터로 간다.

### C1. Tsai–Lenz (1989)

가장 전통적인 `AX=XB` 방법. 회전을 먼저 풀고 그 결과로 이동을 푸는 **순차 방식**이다.

- 역할: 가장 오래된 기준선
- 실행: `method=cv2.CALIB_HAND_EYE_TSAI`
- 한계: 카메라 한 대씩 따로 풀며, 픽셀 단위 목적함수가 아니다.
- 문헌: [Tsai and Lenz, 1989](https://doi.org/10.1109/70.34770)

### C2. Park–Martin (1994)

Lie group과 matrix logarithm으로 `AX=XB`를 푼다. Tsai와 **회전 표현 방식이 다르다.**

- 역할: 같은 문제를 다른 수학으로 푸는 고전 기준선
- 실행: `method=cv2.CALIB_HAND_EYE_PARK`
- 한계: C1과 동일 (pose-level, 카메라 개별 solve)

### C3. Horaud–Dornaika (1995)

회전 표현과 해법이 또 다른 고전 `AX=XB` 방법.

- 역할: **고전 solver를 무엇으로 고르느냐에 따른 성능 폭**을 확인하는 용도
- 실행: `method=cv2.CALIB_HAND_EYE_HORAUD`
- 한계: 우리 방식의 픽셀 단위 joint BA와 목적함수가 다르다.

### C4. Daniilidis (1999)

dual quaternion으로 **회전과 이동을 하나의 식에서 동시에** 다룬다.
C1–C3이 회전을 먼저 풀고 이동을 나중에 푸는 것과 대비된다.

- 역할: simultaneous closed-form 계열의 대표
- 실행: `method=cv2.CALIB_HAND_EYE_DANIILIDIS`
- 주의: **모든 노이즈 조건에서 다른 고전 방법보다 낫다고 가정하지 않는다.**
- 문헌: [Daniilidis, 1999](https://www.cis.upenn.edu/~kostas/mypub.dir/ijrr99.pdf)

### C 계열 보고 규칙

**C1–C4를 모두 실행한 뒤**, 동일한 외부 GT에서 가장 좋았던 것을 `best classical`로 표시한다.

> 결과를 본 다음에 한 방법만 골라 보고하지 않는다.

넷 다 돌려놓고 우리에게 유리한 하나만 싣는 것은 **cherry-picking**이며,
리뷰에서 가장 먼저 지적당하는 지점이다.

---

## 4. D1. Shah (2013) — robot-world까지 같이 푼다

`AX=ZB` 문제를 **닫힌 형태(closed-form)** 로 푼다.
Kronecker product와 SVD를 써서 두 미지수를 한 번에 구한다.

```python
R_base2world, t_base2world, R_gripper2cam, t_gripper2cam = cv2.calibrateRobotWorldHandEye(
    R_world2cam, t_world2cam,
    R_base2gripper, t_base2gripper,
    method=cv2.CALIB_ROBOT_WORLD_HAND_EYE_SHAH,
)
```

- 역할: **`AX=ZB` 계열의 기준선.** C 계열보다 우리 문제에 가깝다
- FK 취급: 입력된 robot pose를 최적화 중에 **고치지 않는다**(고정 입력)
- 문헌: [Shah, 2013](https://www.nist.gov/publications/solving-robot-worldhand-eye-calibration-problem-using-kronecker-product)

> 문헌마다 두 번째 미지수를 `Y` 또는 `Z`로 쓴다. 같은 것이다.

---

## 5. D2. Tabb & Ahmad Yousef (2017) — 반복 최적화, 카메라 여러 대

`AX=ZB`를 **닫힌 형태가 아니라 반복 최적화**로 푼다.
여러 cost 함수와 회전 표현을 비교하고, **카메라 여러 대(hand-multiple-eye)로 확장**한다.

- 역할: 반복 최적화 기준선이자 **multi-eye 지원 기준선**
- 실행: 저자 공개 [코드 저장소](https://github.com/amy-tabb/RWHEC-Tabb-AhmadYousef)
- 입력: 저자 형식으로 변환한 **동일한** board 관측과 robot pose
- 문헌: [Tabb and Ahmad Yousef, 2017](https://arxiv.org/abs/1907.12425)

### 공정성 규칙

- 공개된 cost variant 중 무엇을 쓸지는 **training 데이터 안에서만** 정하고, test에서는 고정한다.
- **논문에 없는 임의의 robustification이나 후처리를 추가하지 않는다.**
  추가 구현이 꼭 필요하면 원 방법과 `+refinement` 변형을 **별도 행으로** 보고한다.

남의 방법을 우리 입맛에 맞게 고쳐놓고 "그래도 우리가 이겼다"고 하면 비교가 무효다.

---

## 6. D3. Allegro et al. (2024) — 가장 가까운 비교 대상 · **최우선**

**우리 셋업과 가장 비슷한 공개 구현이다.** 여기에 시간을 가장 많이 써야 한다.

calibration board와 로봇 움직임을 써서 **여러 카메라의 자세를 함께 최적화**한다.
공통 board-to-end-effector 변환과 카메라 간 일관성을 이용하며,
**재투영오차를 최소화**한다 — 우리와 같은 방식이다.

- 역할: **가장 가까운 최신 multi-camera 기준선**
- 우리와의 핵심 차이: 둘 다 joint 재투영 최적화를 쓰지만,
  우리는 cube pose에 **covariance-weighted robust FK factor**를 둔다
- 공식 구현: [Multi-Camera-Hand-Eye-Calibration](https://github.com/davidea97/Multi-Camera-Hand-Eye-Calibration)
- 문헌: [Allegro et al., 2024](https://arxiv.org/abs/2406.11392)

### 적용 규칙

- Board-on-EE 세션에서 **모든 고정 카메라가 가능한 한 동시에** board를 관측한다.
- 원 구현의 intrinsics / image / robot-pose / config 형식을 그대로 쓴다.
- 원 방법이 직접 지원하지 않는 **mixed eye-in-hand + eye-to-hand 전체 구성을
  억지로 한 번의 실행에 밀어넣지 않는다.**
- 지원 가능한 카메라 subset의 정확도와 **전체 시스템 coverage를 분리해 보고**한다.
- 저자 기본값 변경, 버그 수정, adapter patch는 **commit hash와 함께 기록**한다.

### 표현 주의

Allegro를 **"hand-eye 분야 유일한 최신 SOTA"라고 쓰지 않는다.**
이 실험에서의 정의는 `가장 가까운 공개 multi-camera 기준선`이다.

---

## 7. D4. Calib3R (2025) — 마커 없이 (조건부)

체커보드나 마커 **없이** 일반 RGB 장면만으로 캘리브레이션한다.
3D foundation model로 pointmap을 만들고, 로봇 움직임과 합쳐
metric scale 복원과 camera-to-robot 캘리브레이션을 **동시에** 수행한다.

- 역할: **pattern-free 캘리브레이션의 최신 지점**
- 공식 구현: [Calib3R](https://github.com/davidea97/Calib3R)
- 문헌: [Allegro et al., 2025](https://arxiv.org/abs/2509.08813)
- 자원: 논문 실험은 고성능 GPU를 쓴다. **하드웨어와 실행 시간을 따로 공개**한다.

### 포함 조건 — 넷을 모두 만족할 때만 표에 넣는다

1. 공식 환경을 재현하고 **제공된 예제로 정상 실행**된다.
2. 우리 RGB sequence에 **충분한 texture, overlap, 시점 다양성**이 있다.
3. 카메라 장착 방식과 robot-pose convention을 원 방법에 맞게 **매핑할 수 있다.**
4. 캘리브레이션에 쓰지 않은 **동일한 외부 GT blind pose로 평가**할 수 있다.

조건을 못 채우면 **실패를 숨기지 않고** 원인과 시도한 버전·환경을 기록한다.

D4는 marker 기반 방법과 **입력 조건 자체가 다르므로**,
순위 경쟁 대상이 아니라 `targetless의 편의성 ↔ 정확도 trade-off`로 따로 해석한다.

---

## 8. D5 · D6 — FK 불확실성을 직접 다루는 방법

**우리 방법의 핵심 주장과 가장 직접 부딪히는 두 방법이다.**
"로봇 FK를 얼마나 믿을 것인가"를 이미 다루기 때문이다.

### D5. Ha (2023) — 확률 기반 `AX=ZB`

각 측정 `A_i`, `B_i`가 **서로 다른 noise 특성과 신뢰도**를 가진다는 점을 반영하는
maximum-likelihood 프레임워크. 추정 결과의 **불확실성까지 함께 산출**한다.

- 역할: pose 단위 covariance를 다루는 robot-world/hand-eye 기준선
- 공식 구현: [probabilisticAXYB](https://github.com/hjhdog1/probabilisticAXYB)
- 문헌: [Probabilistic Framework for Hand–Eye and Robot–World Calibration](https://doi.org/10.1109/TRO.2022.3214350)
- 비교 제한: 원 방법은 **raw-corner multi-camera joint BA가 아니므로**,
  중간 과정이 아니라 **공통 외부 GT 결과로만** 비교한다.

### D6. Ulrich & Hillemann (2024) — 불확실성 인식 hand-eye

산업용 로봇의 **absolute pose 불확실성을 명시적으로 모델링**하고,
hand-eye 자세와 **보정된 robot pose를 함께 추정**한다.
target 기반과 targetless 구성을 모두 지원하며 robot uncertainty 자체도 보고한다.

- 역할: **우리 방법과 가장 직접 겹치는 uncertainty-aware 기준선**
- 문헌: [Uncertainty-Aware Hand–Eye Calibration](https://doi.org/10.1109/TRO.2023.3330609)
- 비교 제한: mixed multi-camera 전체 구성을 직접 지원하지 않으면,
  지원 카메라 subset과 전체 coverage를 **분리해 보고**한다.

### 실행하지 않지만 반드시 인용 — Strobl & Hirzinger (2006)

SE(3) 확률 모델과 로봇의 이동/회전 정밀도 특성에 따른 weighting을 이용한
maximum-likelihood hand-eye calibration을 제안했다.

**covariance weighting의 역사적 출처이므로 반드시 인용한다.**
"우리가 처음"이라고 쓰면 안 되는 이유가 이 논문이다.

- 문헌: [Strobl & Hirzinger, *Optimal Hand-Eye Calibration*, 2006](https://doi.org/10.1109/IROS.2006.282250)

---

## 9. 2026년 이후 최신 문헌 처리

더 최신 논문(예: 2026년의 multi-camera robot-world/hand-eye dual-quaternion 계열)은
**related work에서 검토**한다. 다만 아래를 **모두** 만족할 때만 실제 실험 행으로 올린다.

- 논문과 알고리즘 세부가 충분히 공개됨
- 우리 데이터의 frame convention으로 **모호하지 않게** 매핑 가능
- reference implementation이 있거나 독립적으로 재현 가능한 절차가 있음
- 동일한 외부 GT 평가가 가능함

**출판연도만으로 D3를 대체하지 않는다.**

---

## 10. 데이터 수집 규칙

### 어떤 세션이 어떤 방법에 쓰이는가

| 준비물 | 쓰는 방법 |
| --- | --- |
| Board-on-EE 세션 | 고정 카메라용 classical, D1 Shah, D2 Tabb, D3 Allegro |
| eye-in-hand target 세션 | 손목 카메라용 classical 및 지원 방법 |
| Cube+board 통합 세션 | 우리 방법 계열 |
| texture 충분한 RGB sequence + robot pose | D4 Calib3R |
| **캘리브레이션에 쓰지 않은 외부 GT blind pose** | **모든 방법의 공통 최종 평가** |

### 촬영 최소 조건

- 같은 카메라 설치 세션 안에서 **방법별 raw 입력을 최대한 공유**한다.
- Board-on-EE는 **최소 20–30개**의 다양한 이동 및 roll/pitch/yaw 자세로 촬영한다.
- 노출·게인·초점을 고정하고 **동일한 intrinsics**를 쓴다.
- corner detection, raw RGB, timestamp, FK pose, visibility mask, config hash를 **모두 저장**한다.
- 카메라를 **다시 설치한 독립 세션**을 반복한다.
- train/test는 프레임이 아니라 **물리적 세션과 blind workspace pose 기준**으로 나눈다.

마지막 항목이 중요하다. 같은 세션의 프레임을 train/test로 나누면
**너무 쉬운 문제가 되어 모든 방법이 잘 나온다.**

---

## 11. 공정하게 돌리기 위한 규칙

### 모든 방법에 동일하게 고정할 것

- raw 영상과 외부 GT blind pose
- 카메라 intrinsics와 왜곡 모델
- robot pose timestamp 동기화
- classical 방법에 넣는 PnP pose
- pixel 단위 방법에 넣는 corner detection
- train/test 세션 분할
- 허용되는 초기화 / multi-start 횟수
- 실패 판정 기준과 시간 제한

### 방법마다 달라도 되는 것

- 원 논문이 요구하는 target 또는 targetless 입력
- 원 구현의 목적함수와 parameterization
- 저자가 권장한 hyperparameter
- 필요한 dependency와 계산 하드웨어

### 가장 중요한 한 줄

> hyperparameter는 **training 안에서만** 정하고, test와 external-GT blind set에서는 **동결한다.**
> **test 결과를 보고** method별 threshold, loss scale, 초기화를 **바꾸지 않는다.**

한 번이라도 test를 보고 조정하면 그 이후 모든 숫자는 신뢰할 수 없다.

---

## 12. 성능은 무엇으로 재는가

### Primary — 최종 순위는 이것으로만 정한다

| 지표 | 단위 | 정의 |
| --- | --- | --- |
| `TRE_t` | mm | 예측 자세와 **독립 외부 GT** 사이의 이동 오차 |
| `e_R` | deg | 두 회전 사이의 각도 |
| ADD 또는 ADD-S | mm | 작업 물체 모델 기준 자세 불일치 |

ADD-S는 큐브의 대칭을 **작업상 같은 자세로 인정할 때만** 쓴다.
대칭 집합은 **결과를 보기 전에** 확정한다.

### Secondary

P50/P95 `TRE_t`와 `e_R` · 카메라 등록 coverage `N_reg` ·
세션/카메라/자세별 실패율 · 캘리브레이션 및 추론 실행 시간과 메모리 · 필요한 영상 수

### Diagnostic only — **순위 판정에 쓰지 않는다**

held-out 재투영오차 · 카메라 간 불일치 · 복원된 큐브 치수 오차 ·
명령 대비 상대 이동 일관성 · FK-proxy 오차 · solver 수렴성과 condition number

### 왜 재투영오차가 diagnostic으로 밀려났는가 — **꼭 이해할 것**

일반적인 블로그와 튜토리얼은 **"재투영오차가 작으면 잘 된 것"** 이라고 말한다.
**이 프로젝트에서는 그 통념이 성립하지 않는다.**

카메라가 **일관되게** 치우쳐 있으면, 캘리브레이션 결과가 틀렸는데도
자기 예측과 자기 사진은 잘 맞아서 **재투영오차가 0에 가깝게 나온다.**
(발표자료 `presentation/calibration_1_11.pdf` 40쪽에 숫자 예시가 있다)

그래서 **재투영오차만으로 방법을 비교하면 결론이 나오지 않는다.**
순위는 **캘리브레이션에 쓰지 않은 외부 GT**로만 정한다.

targetless인 D4에는 marker 재투영오차가 아예 정의되지 않는다는 점도
공통 지표를 외부 GT로 두어야 하는 이유다.

### 실습으로 확인하기

말로 읽는 것보다 직접 보는 게 빠르다.
정답을 아는 시뮬레이션(`CP_synthetic_7row.py`)에서 **변환 방향을 일부러 뒤집어** 보라.
**재투영오차는 여전히 작은데 `e_t`는 커지는 것**을 직접 볼 수 있다.

---

## 13. 결과 표 템플릿 (baseline 부분)

| Method | Input | Cameras supported/evaluated | `TRE_t` mean [95% CI] ↓ | `TRE_t` P95 ↓ | `e_R` mean [95% CI] ↓ | ADD/ADD-S ↓ | Coverage ↑ | Failure ↓ | Time ↓ |
| --- | --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| C1 Tsai | target + FK/PnP | — | — | — | — | — | — | — | — |
| C2 Park | target + FK/PnP | — | — | — | — | — | — | — | — |
| C3 Horaud | target + FK/PnP | — | — | — | — | — | — | — | — |
| C4 Daniilidis | target + FK/PnP | — | — | — | — | — | — | — | — |
| D1 Shah | target + FK/PnP | — | — | — | — | — | — | — | — |
| D2 Tabb | target + FK/images | — | — | — | — | — | — | — | — |
| D3 Allegro | board + FK + images | — | — | — | — | — | — | — | — |
| D4 Calib3R | targetless RGB + FK | — | — | — | — | — | — | — | — |
| D5 Ha probabilistic AXYB | target poses + robot poses/covariance | — | — | — | — | — | — | — | — |
| D6 Uncertainty-Aware HEC | target/images + uncertain robot poses | — | — | — | — | — | — | — | — |

`Cameras supported/evaluated`는 **반드시 채운다.**
지원하지 못한 카메라를 빼고 얻은 낮은 오차와 전체 카메라 coverage를 **혼동하면 안 된다.**
카메라 3대 중 1대만 쓰고 얻은 좋은 숫자는 3대를 모두 쓴 방법과 나란히 놓을 수 없다.

---

## 14. 시작 순서 (권장)

1. **`CP_metric_board_only.py` 실행** — C 계열과 우리 방법의 비교표가 이미 나온다. 숫자 하나를 재현하는 것이 첫 목표
2. **`CP_synthetic_7row.py`로 시뮬레이션 실습** — 정답을 알고 있으므로 자기 이해를 검증할 수 있다. §12의 "일부러 틀리게 넣어보기"를 여기서 한다
3. **C1–C4, D1 실행** — OpenCV 한 줄. §3의 실수 목록을 옆에 두고 확인
4. **D3 Allegro** — 가장 중요하고 가장 오래 걸린다
5. **D2, D5, D6** — 외부 저장소
6. **D4 Calib3R** — §7의 포함 조건을 먼저 확인하고 판단

---

## 15. 참고 문헌 및 구현

- [Tsai & Lenz, *A New Technique for Fully Autonomous and Efficient 3D Robotics Hand/Eye Calibration*, 1989](https://doi.org/10.1109/70.34770)
- [Daniilidis, *Hand-Eye Calibration Using Dual Quaternions*, 1999](https://www.cis.upenn.edu/~kostas/mypub.dir/ijrr99.pdf)
- [Strobl & Hirzinger, *Optimal Hand-Eye Calibration*, 2006](https://doi.org/10.1109/IROS.2006.282250)
- [Shah, *Solving the Robot-World/Hand-Eye Calibration Problem Using the Kronecker Product*, 2013](https://www.nist.gov/publications/solving-robot-worldhand-eye-calibration-problem-using-kronecker-product)
- [Tabb & Ahmad Yousef, *Solving the Robot-World Hand-Eye(s) Calibration Problem with Iterative Methods*, 2017](https://arxiv.org/abs/1907.12425)
  · [official code](https://github.com/amy-tabb/RWHEC-Tabb-AhmadYousef)
- [Ha, *Probabilistic Framework for Hand–Eye and Robot–World Calibration*, 2023](https://doi.org/10.1109/TRO.2022.3214350)
  · [official code](https://github.com/hjhdog1/probabilisticAXYB)
- [Ulrich & Hillemann, *Uncertainty-Aware Hand–Eye Calibration*, 2024](https://doi.org/10.1109/TRO.2023.3330609)
- [Allegro et al., *Multi-Camera Hand-Eye Calibration for Human-Robot Collaboration in Industrial Robotic Workcells*, 2024](https://arxiv.org/abs/2406.11392)
  · [official code](https://github.com/davidea97/Multi-Camera-Hand-Eye-Calibration)
- [Allegro et al., *Calib3R*, 2025](https://arxiv.org/abs/2509.08813)
  · [official code](https://github.com/davidea97/Calib3R)

### 보조 자료 (개념이 안 잡힐 때)

- [OpenCV: Camera Calibration and 3D Reconstruction](https://docs.opencv.org/4.13.0/d9/d0c/group__calib3d.html) — `calibrateHandEye`, `calibrateRobotWorldHandEye` 파라미터 설명
- [easy_handeye README](https://github.com/IFL-CAMP/easy_handeye) — eye-in-hand / eye-on-base 구성 설명
- [다크프로그래머 — 카메라 캘리브레이션](https://darkpgmr.tistory.com/32) — 한국어. intrinsics, 왜곡, 핀홀 모델
- [Cyrill Stachniss — Basics about Bundle Adjustment](https://www.youtube.com/watch?v=sobyKHwgB0Y) — D3와 우리 방법의 기반
