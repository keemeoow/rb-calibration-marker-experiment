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

## 4. Tabb & Ahmad Yousef (2017) — 재투영 반복: "공식 한 방 대신, 사진에 대고 수렴까지 반복 수정"

> 📄 Tabb & Ahmad Yousef, *"Solving the Robot-World Hand-Eye(s) Calibration Problem with Iterative Methods"*, Machine Vision and Applications, 2017 — [PDF](https://arxiv.org/pdf/1907.12425) · 💻 [코드](https://github.com/amy-tabb/RWHEC-Tabb-AhmadYousef)

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

> 📄 Allegro, Terreran & Ghidoni, *"Multi-Camera Hand-Eye Calibration for Human-Robot Collaboration in Industrial Robotic Workcells"*, IEEE RA-L, 2024 (ICRA 2025) — [PDF](https://arxiv.org/pdf/2406.11392) · 💻 [코드](https://github.com/davidea97/Multi-Camera-Hand-Eye-Calibration) (리포에 사본 있음)

- 배경: 1~4는 전부 1카메라용 → 따로 풀면 각자 오차를 안고 끝 (상호 검증 없음)
- 셋업: 보드를 **로봇 손에 부착** → 로봇이 움직여줌 → 전 카메라가 동시 관측
- 풀이: 전 카메라 **동시 최적화**, 제약 2가지
  - ① 각 카메라 답이 로봇 FK와 일치
  - ② **카메라끼리 서로 본 것끼리 무모순** (같은 순간 같은 보드 → 답 일치해야)
- 핵심: ②의 교차 검증 — 시험 답안 4명이 맞춰보기 → 개별 실수 검출·평균화
- 표에서의 역할: 마커 기반 멀티카메라 **현재 SOTA**, 우리의 최근접 경쟁자 (FK는 100% 신뢰)

## 6. Calib3R (2025) — "보드 없이 AI가 장면을 3D 복원" ⚠️ 실험군 제외 → 인용만 (코드 미공개)

> 📄 Allegro 외, *"Calib3R: A 3D Foundation Model for Multi-Camera to Robot Calibration and 3D Metric-Scaled Scene Reconstruction"*, arXiv:2509.08813, 2025 — [PDF](https://arxiv.org/pdf/2509.08813) · ⚠️ 코드: davidea97/Calib3R **현재 404** (비공개 전환 추정, 논문에도 공개 언급 없음) → 공개 재개 모니터링 + 대안 검토

- 셋업: 마커 없음 — 카메라로 **일반 풍경** 촬영
- 풀이 절차
  - 3D 파운데이션 모델(AI)이 여러 사진에서 장면의 3D 구조 복원
  - 복원 3D는 **크기를 모름** → 로봇의 "정확히 10cm 이동"(FK)으로 크기 결정 + 로봇 기준 정렬
- 장단: 마커 준비 수고 없음 ↔ 마커만큼 정밀하기 어려움(통념)
- 표에서의 역할: 보드-프리 AI 진영 최신 대표 — 우리가 이기면 "왜 아직 마커냐"에 실험으로 답

## 6B. Kalib (IROS 2025) — 학습기반 baseline ① 확정: "AI가 그리퍼 끝점을 추적"

> 📄 Tang, Liu, Xu & Lu, *"Kalib: Easy Hand-Eye Calibration with Reference Point Tracking"*, **IROS 2025** (DOI 10.1109/IROS60139.2025.11247188) — [PDF](https://arxiv.org/pdf/2408.10562) · 💻 [코드](https://github.com/robotflow-initiative/Kalib) (공개 ✅, 기성 파운데이션 트래커 가중치 공개)

- 셋업: 마커 없음 — 로봇 위 **기준점 1개**(그리퍼 끝)만 정의
- 풀이 절차
  - 로봇이 작업공간을 자연스럽게 움직임 → **비주얼 파운데이션 모델이 기준점을 영상에서 추적**
  - 같은 순간의 FK 3D 좌표와 짝지어 PnP → camera→base 변환
- **우리 3조건 전부 충족하는 유일한 AI 계열**
  - ✅ 코드 공개 ② ✅ **CAD/메시 불필요** (논문이 명시) + 로봇별 재학습 불필요 ③ ✅ ZEUS 적용 가능
  - 고정캠은 그리퍼가 드나드는 작업공간을 보므로 기준점 가시성 자연 충족 (팔 전체 가시성 불필요)
- 한계
  - 단일 카메라 방법 → 고정캠 per-cam 적용 + 조립 (부류 1 방식). 그리퍼캠은 자기 손 못 봄 → 제외
  - 픽셀 트래킹 정밀도 < 마커 코너 검출 → 우리에게 유리하되 공정한 비교군
- 데이터 요구: 스냅샷이 아닌 **연속 영상** (각 고정캠 30초~1분, 로봇 EE 스윕) + 동기화 FK
  → 보드-온-EE 캡처 세션 날 몇 분 추가로 해결
- 표에서의 역할: **학습기반 ① — AI 추적 계열 대표** (피어리뷰·코드·가중치·CAD-프리 전부 충족)

## 6C. MASt3R-SfM + FK 앵커 — 학습기반 baseline ② 확정: "AI 장면복원 + 로봇 앵커"

> 📄 MASt3R: Leroy, Cabon & Revaud, *"Grounding Image Matching in 3D with MASt3R"*, **ECCV 2024 (oral)** /
> MASt3R-SfM: *"a Fully-Integrated Solution for Unconstrained Structure-from-Motion"*, **3DV 2025**
> — 💻 [naver/mast3r](https://github.com/naver/mast3r) (코드 + 공식 체크포인트 공개 ✅)

- 셋업: 마커 없음 — 고정캠들이 찍은 **일반 장면 사진** + 로봇 FK 앵커 1개
- 풀이 절차
  1. MASt3R-SfM(3D 파운데이션 모델)이 고정캠 멀티뷰 사진에서 **카메라 상대 기하** 복원 (스케일 미지)
  2. FK 앵커 1개(FK-알려진 큐브/EE를 본 카메라 1대)로 **미터 스케일 + base 정렬**
  - (**Calib3R 논문의 자체 베이스라인 "MASt3R-SfM + Calib" 구성 그대로** — 자의적 조립 아님)
- 장단: 기존 캡처 사진 재활용 가능 ↔ 넓은 베이스라인 + 텍스처 빈약 워크셀 = 장면 기반 최악 조건
  → **기존 캡처로 30분 사전 정합 테스트 후 최종 확정** (실패 시 VGGT(CVPR 2025, 가중치 공개)로 교체)
  - 앵커 단계는 우리 구현 (전 방법 공통 적용으로 공정성 확보)
- 표에서의 역할: **학습기반 ② — AI 장면복원 계열 대표** (로봇-프리 멀티카메라 학습 계열 전체를 대표)
- 배제 재확인: EasyHeC++(IROS 2024, 피어리뷰·코드 있음)는 **CAD 필수**라 제외 — Calib3R 도 robot-body
  학습 계열을 동일 사유로 비교에서 제외한 선례 있음

---

## 7. 우리(ours)의 차별점

- 비교 방법들의 FK 취급: Shah·Tabb·Allegro = **100% 신뢰** / Tsai·Daniilidis = 움직임 비교만 /
  Calib3R·MASt3R-SfM = 크기·정렬 앵커만 / Kalib = 기준점 3D 좌표(정답 레이블)로 사용
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
| **Calib3R** | (인용만) 마커 미사용(정밀도↔편의 트레이드) + FK는 크기용 + 보정 없음 |
| **Kalib** | ① 카메라 결합 없음(per-cam) ② 마커 미사용(추적 정밀도 한계) ③ soft anchor·잔차보정 없음 |
| **MASt3R-SfM+앵커** | ① 마커 미사용(텍스처 의존) ② FK는 앵커만(관측-FK 융합 없음) ③ 잔차보정 없음 |

- 한 줄 요약: **결합은 Allegro·MASt3R-SfM만 보유. soft anchor·잔차보정은 전부 없음 → 우리 기여**
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

### 부류 1B: 학습기반 per-cam (Kalib)

- 고정캠 각각: EE 스윕 **연속 영상** + 동기화 FK로 카메라별 독립 계산 → 조립 (부류 1과 동일 방식)
- 그리퍼캠: 자기 손 못 봄 → 적용 제외 (고정캠 네트워크 비교 전용)
- 할 일: 영상+FK를 Kalib 입력 형식으로 변환, 기준점(그리퍼 끝) 정의

### 부류 2: 원래부터 멀티카메라 (Tabb · Allegro · MASt3R-SfM+앵커) + 제외 (Calib3R)

- **Tabb**: 제목의 "eye(**s**)" = 멀티 지원 내장
  - 할 일: 보드-온-EE 세션 전체 카메라 이미지 + FK를 그들 폴더 형식으로 변환
- **Allegro**: 애초에 멀티카메라 논문
  - 할 일: 같은 세션을 그들 형식으로 변환 (`cameraX/image/` + `pose/*.csv` + intrinsics yaml, `calibration_setup: 1`)
  - 그리퍼캠: eye-in-hand 모드(`calibration_setup: 0`) + 바닥 보드 데이터로 별도 실행
- **MASt3R-SfM + FK 앵커**: 고정캠 사진 전체를 한 번에 정합 (자체가 멀티뷰)
  - 할 일: ① 기존 캡처로 30분 사전 정합 테스트 ② 통과 시 FK 앵커 스크립트 작성 (전 방법 공통 적용)
- **Calib3R**: ⚠️ **코드 404 → 실험군 제외 확정 (2026-08-05)** — related work 인용만,
  "코드 미공개로 정량 비교 불가" 명시. 대체는 6C(MASt3R-SfM+앵커, Calib3R 자체 베이스라인 구성)가 담당.
  코드 재공개 또는 저자 회신 시 복귀 검토.

### 공통 준비물

| 준비물 | 사용처 |
|---|---|
| **보드-온-EE 캡처 세션 1회** (보드 EE 강체 고정, 자세 20~30개, 전 고정캠 동시 촬영 + FK 기록) | Tsai·Daniilidis·Shah(고정캠), Tabb, Allegro |
| 기존 바닥 보드 데이터 | Tsai·Daniilidis·Shah(그리퍼캠), Allegro(그리퍼캠) |
| **EE 스윕 연속 영상** (고정캠별 30초~1분 + 동기화 FK — 같은 날 몇 분 추가) | Kalib |
| 기존 캡처 장면 사진 (재활용, 사전 정합 테스트 선행) + FK 앵커 관측 1개 | MASt3R-SfM+앵커 |
| 포맷 변환 스크립트 5개 (OpenCV 직호출 / Tabb / Allegro / Kalib / MASt3R 앵커) | 공통 |

---

