# Baseline 비교 실험

## 개념

- 로봇과 카메라는 **서로 다른 "자기 기준"** 으로 위치를 잼
  - 로봇: "내 발바닥(base)에서 오른쪽 30cm, 앞 20cm"
  - 카메라: "내 렌즈에서 정면 50cm, 왼쪽 10cm"
- 캘리브레이션 = **"카메라 좌표 → 로봇 좌표" 번역표** 를 찾는 일
  - 이 표가 없으면: 카메라가 "컵 여기!"라고 해도 로봇은 못 집음
  - 이 표의 정확도가 논문 전체의 주제
- 모든 방법의 공통 재료 2가지
  - **FK(순기구학)**: 로봇은 모터 각도 센서로 자기 손 위치를 항상 앎
  - **마커 + PnP**: 마커의 실제 크기를 아니까, 사진 속 찌그러짐으로 카메라↔마커 거리·각도 역산
- 방법 간 차이 = "FK와 카메라 관측을 **어떤 수학으로, 무엇을 믿고** 연결하나"

---

## 1. Tsai–Lenz (1989) — 원조, "회전 먼저·이동 나중"

> 📄 Tsai & Lenz, *"A New Technique for Fully Autonomous and Efficient 3D Robotics Hand/Eye Calibration"*, IEEE Trans. Robotics and Automation, 1989 — [PDF](https://kmlee.gatech.edu/me6406/handeye.pdf) · 💻 OpenCV `calibrateHandEye(TSAI)`

- 문제 형태: **AX=XB** (A=손의 움직임(FK), B=카메라가 느낀 움직임, X=손→카메라 변환)
  - 셀카봉 비유: 손 궤적(내가 앎) + 폰이 본 장면 변화(폰이 앎) → 봉의 생김새(X) 역산
- 풀이: 2단계 분리 — ① 회전 먼저 → ② 그 답으로 이동 계산
- 장점: 빠르고 단순
- 약점: ①의 오차가 ②로 **전염** (첫 단추 구조)
- 표에서의 역할: 가장 기본 방식의 **출발선**

## 2. Daniilidis (1999) — "회전+이동 한 번에"

> 📄 Daniilidis, *"Hand-Eye Calibration Using Dual Quaternions"*, IJRR, 1999 — [PDF](https://www.cis.upenn.edu/~kostas/mypub.dir/ijrr99.pdf) · 💻 OpenCV `calibrateHandEye(DANIILIDIS)`

- 풀이: **듀얼 쿼터니언** — 회전·이동을 한 묶음으로 포장해 **연립 한 방**
- 효과: 오차 전염 없음 → 노이즈에 강함
- 위상: "공식 한 방(closed-form)" 계열의 **베스트**
- 표에서의 역할: 고전의 최강자 — 이보다 나아야 새 방법의 의미 있음

## 3. Shah (2013) — "미지수 2개를 공식 한 방에"

> 📄 Shah, *"Solving the Robot-World/Hand-Eye Calibration Problem Using the Kronecker Product"*, ASME J. Mechanisms and Robotics, 2013 — [PDF (ResearchGate)](https://www.researchgate.net/publication/275087810_Solving_the_Robot-WorldHand-Eye_Calibration_Problem_Using_the_Kronecker_Product) · 💻 OpenCV `calibrateRobotWorldHandEye(SHAH)`

- 문제 형태: **AX=ZB** (X=카메라 변환 + Z=보드 위치, **미지수 2개**)
- 풀이: Kronecker 곱 트릭 → 대입하면 답 나오는 **닫힌 해**, X·Z 동시 산출
- 핵심 특징: **FK를 100% 신뢰** — FK 오차가 답에 그대로 박힘
- 표에서의 역할: "FK 통째로 믿기" 진영의 표준 공식

## 4. Tabb & Ahmad Yousef (2017) — "공식 대신 사진에 대고 반복 수정"

> 📄 Tabb & Ahmad Yousef, *"Solving the Robot-World Hand-Eye(s) Calibration Problem with Iterative Methods"*, Machine Vision and Applications, 2017 — [PDF (arXiv)](https://arxiv.org/pdf/1907.12425) · 💻 [코드](https://github.com/amy-tabb/RWHEC-Tabb-AhmadYousef)

- 통찰: **오차의 근원 = 사진의 픽셀** (코너 검출이 1~2px씩 틀림) — 공식 한 방은 재는 자가 다름
- 풀이 절차
  1. 공식 한 방(Shah류)으로 대충의 답
  2. 그 답 기준 "코너가 찍혀야 할 픽셀"을 사진에 겹쳐 그림 (**재투영**)
  3. 실제 검출 위치와의 픽셀 어긋남을 재고, 줄어드는 방향으로 답 수정
  4. 수렴까지 2~3 반복
- 효과: 실데이터에서 공식 한 방보다 꾸준히 정확
- 우리와의 연결: "픽셀 기준 반복 수정" 철학은 우리와 동일
- 표에서의 역할: 재투영·반복 진영의 고전 대표 (단, FK는 100% 신뢰)

## 5. Allegro 외 (RA-L 2024) — "멀티카메라를 서로 검증시키며 한꺼번에"

> 📄 Allegro, Terreran & Ghidoni, *"Multi-Camera Hand-Eye Calibration for Human-Robot Collaboration in Industrial Robotic Workcells"*, IEEE RA-L, 2024 (ICRA 2025) — [PDF (arXiv)](https://arxiv.org/pdf/2406.11392) · 💻 [코드](https://github.com/davidea97/Multi-Camera-Hand-Eye-Calibration) (리포에 사본 있음)

- 배경: 1~4는 전부 1카메라용 → 따로 풀면 각자 오차를 안고 끝 (상호 검증 없음)
- 셋업: 보드를 **로봇 손에 부착** → 로봇이 움직여줌 → 전 카메라가 동시 관측
- 풀이: 전 카메라 **동시 최적화**, 제약 2가지
  - ① 각 카메라 답이 로봇 FK와 일치
  - ② **카메라끼리 서로 본 것끼리 무모순** (같은 순간 같은 보드 → 답 일치해야)
- 핵심: ②의 교차 검증 — 시험 답안 4명이 맞춰보기 → 개별 실수 검출·평균화
- 표에서의 역할: 마커 기반 멀티카메라 **현재 SOTA**, 우리의 최근접 경쟁자 (FK는 100% 신뢰)

## 6. Calib3R (2025) — "보드 없이 AI가 장면을 3D 복원"

> 📄 Allegro 외, *"Calib3R: A 3D Foundation Model for Multi-Camera to Robot Calibration and 3D Metric-Scaled Scene Reconstruction"*, arXiv:2509.08813, 2025 — [PDF (arXiv)](https://arxiv.org/pdf/2509.08813) · 💻 [코드](https://github.com/davidea97/Calib3R)

- 셋업: 마커 없음 — 카메라로 **일반 풍경** 촬영
- 풀이 절차
  - 3D 파운데이션 모델(AI)이 여러 사진에서 장면의 3D 구조 복원
  - 복원 3D는 **크기를 모름** → 로봇의 "정확히 10cm 이동"(FK)으로 크기 결정 + 로봇 기준 정렬
- 장단: 마커 준비 수고 없음 ↔ 마커만큼 정밀하기 어려움(통념)
- 표에서의 역할: 보드-프리 AI 진영 최신 대표 — 우리가 이기면 "왜 아직 마커냐"에 실험으로 답

---

## 7. 우리(ours)의 차별점

- 여섯 방법의 FK 취급: Shah·Tabb·Allegro = **100% 신뢰** / Tsai·Daniilidis = 움직임 비교만 / Calib3R = 크기 결정만
- 우리 = **"적당히 믿기" + "반복 오차 지우기"**
  - **soft anchor**: FK를 부드러운 닻으로 — 멀어지면 살짝 당기되, 관측이 강하게 반대하면 관측 우선
    - 당기는 세기 λ도 사람이 아니라 **데이터(CV)로 선택**
  - **잔차보정**: 캘리브 후 남는 "위치에 따라 반복되는 오차 패턴"(예: 작업대 왼쪽 = 항상 2mm 왼쪽 틀림)을
    직선 공식으로 학습해 미리 차감
- 요약: 이 2가지는 **여섯 방법 중 어디에도 없음** = 우리 기여

## 8. 평가: 실제 로봇에서만, "정답을 아는 값" 4가지로 채점 (확정)

| 채점 방식 | 비유 | 신뢰 근거 |
|---|---|---|
| 숨겨둔 문제 (held-out) | 안 보여준 문제로 시험 | 일반화 실력만 통함 (암기 불가) |
| 자로 잰 치수 | 복원한 큐브 변 길이 vs 캘리퍼 실측 | 물리 실측 = 진짜 정답 |
| 정확히 아는 이동량 | "100.0mm 직진" 명령 vs 카메라 측정 | 모터의 **상대 이동**은 매우 정확 |
| 사진과의 어긋남 (px) | 답을 사진에 겹쳐 그려 오차 측정 | 검출 코너 자체가 기준 |

- 조건: 전 방법을 **같은 날·같은 카메라·같은 데이터**로 실행 → 동일 지표 채점 = 메인 표(Table I)

---

## 9. 논문별 핵심 차이 (그들에게 "없는 것")

| 방법 | 없는 것 |
|---|---|
| **Tsai** | ① 카메라 결합 ② FK 닻 ③ 잔차보정 (+ 오차 전염되는 2단계 풀이) |
| **Daniilidis** | ① 결합 ② FK 닻 ③ 보정 (Tsai 대비 풀이만 개선) |
| **Shah** | ① 결합 ③ 보정 + FK **100% 신뢰** (우리는 soft) |
| **Tabb** | FK 100% 신뢰 + 보정 없음 (재투영·반복 철학은 우리와 동일 → 차이가 "FK 취급·보정"으로 깨끗이 분리) |
| **Allegro** | FK 100% 신뢰 + 보정 없음 + 평가가 자기일관성 (결합은 있음 — 최근접 경쟁자) |
| **Calib3R** | 마커 미사용(정밀도↔편의 트레이드) + FK는 크기용 + 보정 없음 |

- 한 줄 요약: **결합은 Allegro만 보유. soft anchor·잔차보정은 여섯 모두 없음 → 우리 기여**
- Tsai vs 내부 no-FK 구분
  - 공통: FK를 목적함수에 안 넣음
  - 차이: no-FK = 전 카메라가 공유 큐브로 묶여 **합의(통합)** / Tsai = 카메라별 **완전 독립**
  - 표의 역할 분담: Tsai↔no-FK 차이 = "결합의 가치", no-FK↔ours-B 차이 = "anchor+보정의 가치"

## 10. 멀티카메라 비교로 만드는 법 — 전부 가능 (그래서 선정)

### 부류 1: 원래 1카메라용 → "한 대씩 N번 + 조립" (Tsai · Daniilidis · Shah)

- 그리퍼캠: 기존 바닥 보드 데이터로 gTc 계산
- 고정캠 각각: 보드-온-EE 세션(로봇 손의 보드 움직임 관측)으로 **카메라별 독립** 계산
- 변환 N+1개를 모으면 시스템 전체 완성
- 의미: 이 "따로 풀고 조립" = **현업 표준 관행** → 결합 부재의 약점을 보여주는 행

### 부류 2: 원래부터 멀티카메라 (Tabb · Allegro) + 조건부 (Calib3R)

- **Tabb**: 제목의 "eye(**s**)" = 멀티 지원 내장
  - 할 일: 보드-온-EE 세션 전체 카메라 이미지 + FK를 그들 폴더 형식으로 변환
- **Allegro**: 애초에 멀티카메라 논문
  - 할 일: 같은 세션을 그들 형식으로 변환 (`cameraX/image/` + `pose/*.csv` + intrinsics yaml, `calibration_setup: 1`)
  - 그리퍼캠: eye-in-hand 모드(`calibration_setup: 0`) + 바닥 보드 데이터로 별도 실행
- **Calib3R**: 명시 지원은 "로봇 **탑재** 카메라" 기준 — 우리 고정캠은 비탑재·정지 상태
  - 가능성: 전 카메라 사진을 한 3D 장면으로 정합하는 구조 → 그리퍼캠이 크기·정렬 담당,
    고정캠은 장면 등록으로 편입되는 구성이 원리상 가능해 보임
  - 할 일: **코드 조사로 지원 여부 판정** (불가 시 그리퍼캠 비교로 축소 + 논문에 사유 명시)

### 공통 준비물

| 준비물 | 사용처 |
|---|---|
| **보드-온-EE 캡처 세션 1회** (보드 EE 강체 고정, 자세 20~30개, 전 고정캠 동시 촬영 + FK 기록) | Tsai·Daniilidis·Shah(고정캠), Tabb, Allegro |
| 기존 바닥 보드 데이터 | Tsai·Daniilidis·Shah(그리퍼캠), Allegro(그리퍼캠) |
| 기존 장면 사진 + FK (또는 텍스처 있는 신규 시퀀스) | Calib3R |
| 포맷 변환 스크립트 3개 (OpenCV 직호출 / Tabb 형식 / Allegro 형식) | 공통 |

---

