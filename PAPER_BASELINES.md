# 논문 비교대상(Baseline) 분석표

ours-B(soft FK anchor + 잔차보정, 멀티카메라 unified)와 비교할 후보 전수 분석.
기준: **① 어떤 계열인가 ② 공개 코드가 실사용 가능한가 ③ 우리 세팅(ZEUS 로봇, 고정 3~4대 + 그리퍼 1대, 파지 큐브)에 적용 가능한가**.

## 1. 실험 표에 넣을 baseline (전부 공개 코드, 실행 가능)

| # | 방법 (논문) | 연도 / venue | 계열 | FK 사용 | 타깃 | 멀티캠 | 코드 | 적용 비용 | ours 대비 역할 |
|---|---|---|---|---|---|---|---|---|---|
| 1 | Tsai–Lenz / Park–Martin / Horaud / Andreff / Daniilidis | 1989–1999 | 고전 AX=XB, **카메라별 independent** | 모션쌍으로만 (상대 FK) | 임의 (우린 큐브/보드) | ✗ (per-cam 반복) | OpenCV `calibrateHandEye` 내장 | **0 — 이미 파이프라인 사용 중** | "independent 진영" 하한선 |
| 2 | Shah / Li | 2013 / 2010 | robot-world **AX=ZB closed-form** | **하드** (base–world 동시해) | 보드 | ✗ (per-cam) | OpenCV ≥4.5 `calibrateRobotWorldHandEye` 내장 | 반나절 (Python 직호출) | "fixed-FK 진영" 표준해 |
| 3 | Tabb & Ahmad Yousef | 2017 / MVA | robot-world **재투영(px) 반복해** | 하드 | 보드(체스보드) | **✅ eye(s) 지원** | [amy-tabb/RWHEC-Tabb-AhmadYousef](https://github.com/amy-tabb/RWHEC-Tabb-AhmadYousef) (C++/OpenCV4, ~2019 유지보수) | 중간 (빌드 + 입력포맷 변환) | 재투영 목적함수 고전 대표 — 우리 px 목적함수의 선행 |
| 4 | **Allegro·Terreran·Ghidoni** | **RA-L 2024 / ICRA 2025** | **멀티카메라 unified SOTA** (보드를 EE에 부착) | **하드** (bTg 상수 신뢰) | EE 부착 ChArUco | **✅ 핵심 기여** | [davidea97/Multi-Camera-Hand-Eye-Calibration](https://github.com/davidea97/Multi-Camera-Hand-Eye-Calibration) (C++/Ceres) — **리포에 소스 복사본 있음** | 중간 (Ceres 빌드; ChArUco 관측만 사용해야) | **최근접 경쟁자 + FK-하드 진영 최신 대표. 우선순위 1** |
| 5 | **Calib3R** | 2025 (arXiv) | **타깃리스, 3D foundation model(MASt3R) unified** | FK pose 로 스케일/정렬 | **없음 (타깃리스)** — URDF 도 불필요 | **✅** | [davidea97/Calib3R](https://github.com/davidea97/Calib3R) (Python) | 중간 (이미지+FK pose 만 필요 → 기존 데이터 호환) | "학습기반/타깃리스" 최신 대표 — '왜 아직 마커냐' 방어 |
| 6 | Koide & Menegatti *(조건부)* | RA-L 4(2):1021–1028, 2019 (ICRA'19) | 재투영 pose-graph (단일캠) | 모션쌍 | 보드 | ✗ (per-cam) | [koide3/st_handeye_graph](https://github.com/koide3/st_handeye_graph) (C++/g2o, 2019 이후 정체) | 높음 (g2o 버전 고정 → 빌드 리스크) | px 직최적화 선행 — 빌드 실패 시 인용으로 강등 |

**착수 순서 권장: 4 → 2 → 5 → 3 → (6 조건부)** — 비용 대비 서사 기여 순.

### 내부 ablation ↔ 외부 baseline 매핑 (논문 서사)

| 내부 4방법 | 대응하는 외부 진영 | 외부 대표 |
|---|---|---|
| fixed-FK | robot-world AX=ZB / FK 하드 unified | #2 Shah·Li, #3 Tabb, **#4 Allegro** |
| no-FK | 순수 카메라 합의 | #1 고전 (+ Evangelista 인용) |
| ours-A (λ=0 + 잔차보정) | — (우리 기여) | — |
| **ours-B (soft anchor + 잔차보정)** | — (우리 기여) | — |

## 2. 인용만 (코드 없음 / 로봇 종속 / 문제설정 상이)

| 논문 | 연도 | 계열 | 제외 사유 | 인용 위치 |
|---|---|---|---|---|
| Evangelista et al. (graph-based multi-cam HEC) | ICRA 2023 | 멀티캠 pose-graph | 공개 리포 특정 불가; 후속작 #4가 코드 공개+성능 상회 | A군 related work |
| EasyHeC / EasyHeC++ | RA-L 2023 / IROS 2024 | 학습 미분가능 렌더링 (로봇 몸체=타깃) | **CAD/URDF 필수인데 사용자 결정으로 CAD 미사용** (ZEUS CAD 입수 불가; UR3(CB3) 보유하나 세팅 교체 비용 큼) | E군 |
| Ali et al. (comparative study) | Sensors 2019 | 비교연구 | 방법이 아니라 프로토콜 | 평가지표 근거 |
| Zhuang 1994 / Horaud–Dornaika 1995 | 1994–95 | AX=ZB / AX=XB 기원 | 고전, OpenCV 구현으로 대체 | related work |
| CtRNet / CtRNet-X | CVPR'23 / ICRA'25 | 학습 키포인트+렌더링 | **학습된 로봇 전용** — ZEUS 재학습 비용 과다 | E군 |
| DREAM (NVlabs) | ICRA 2020 | 학습 키포인트+PnP | 동일 (Panda/KUKA/Baxter 전용) | E군 |
| RoboPose / RoboKeyGen / RoboTAG / MonoSE(3)-Diffusion | 2021–2025 | render-compare / diffusion | 로봇 종속 + 단일캠 pose 추정이 목적 | E군 |
| Hydra / PlaneHEC / LRBO2 | 2025 | RGB-D 마커프리 | 코드 미확인, 세팅 상이 | E군 |
| Furrer et al. | FSR 2017 | 시간동기+핸드아이 | 시간동기는 우리와 직교 (정적 캡처) | B군 각주 |
| ATOM (Pedrosa et al.) | 2021– | 멀티센서 통합 캘리브 프레임워크 | ROS 통합 무거움, 선택적 | A군 한 줄 |
| Peters et al. (actuated 3D sensor self-calib) | JFR 2024 | 운동학 포함 BA | 로봇 스케일 발견의 선행 사례 | D군 |
| GP/DNN FK 잔차보정 계열 (Active-GP 2023, GPR Measurement 2025, ICAR 2025, GA-DNN 2020) | 2020–25 | 기구학 오차 보상 | 레이저트래커 GT 전제 — 우리 Ridge 선형 선택의 대비군 | D군 |
| Sun & Hollerbach / Borm & Meng / Visual-Biased OI | 1991–2024 | 관측성 지표 | 방법 아님 — "13 set 충분성" 방어용 | F군 |
| Calib3R 외 동일저자 MEMROC | 2024 | 모바일로봇 멀티캠 | 플랫폼 상이 | 각주 |

## 3. 학습기반 관련 핵심 판단 근거

- 로봇 몸체를 타깃으로 쓰는 계열(EasyHeC·CtRNet·DREAM·RoboPose)은 **로봇 3D 모델(URDF+메시)이 원리적으로 필수** — 몸체 모델이 곧 마커 정의. 키포인트 계열은 추가로 로봇별 사전학습 필요.
- **URDF 없이 가능한 학습기반은 Calib3R가 유일** (foundation model 이 장면을 복원, 로봇 모델 불필요) → 학습기반 대표로 확정.
- **사용자 결정 (2026-08): CAD 모델은 사용하지 않는다.** ZEUS CAD 입수 불가. UR3(CB3)는 보유하나
  (UR3 는 URDF 공개 + DREAM/CtRNet 학습 로봇), 본 시스템이 ZEUS 기준이므로 로봇 교체 실험은 범위 밖.
  → 학습기반 비교 = Calib3R 단일. 논문에는 "robot-body 기반 학습 계열은 로봇 3D 모델·가시성을
  전제하므로 모델 미제공 로봇(본 세팅)에 부적용"으로 한 줄 방어.

## 4. 서지 확정 사항

- Koide & Menegatti, "General Hand-Eye Calibration Based on Reprojection Error Minimization," IEEE RA-L 4(2):1021–1028, 2019. DOI 10.1109/LRA.2019.2893612.
- Furrer et al., "Evaluation of Combined Time-Offset Estimation and Hand-Eye Calibration on Robotic Datasets," FSR 2017 (Springer PAR). 코드: ethz-asl/hand_eye_calibration.
- Tabb & Ahmad Yousef, "Solving the robot-world hand-eye(s) calibration problem with iterative methods," MVA 2017. 코드 2019.03까지 갱신, 공식 데이터셋 USDA 공개.
