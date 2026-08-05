# 논문 최종 표 설계 (baseline 확정 + 지표 정의)

T-RO 2편(Ulrich, Gauss–Helmert)은 제외 확정 (상용 구현/원리적 부적용 → related work 인용만).

---

## 0. 확정 비교군 (시스템 단위)

| Method | 유형 | 카메라 결합 | FK 사용 | 잔차보정 | 실행 경로 |
|---|---|---|---|---|---|
| Tsai–Lenz (per-cam) | closed-form solver | ✗ | 모션쌍만 | ✗ | OpenCV (파이프라인 내장) |
| Daniilidis (per-cam) | closed-form solver | ✗ | 모션쌍만 | ✗ | OpenCV |
| Shah (robot-world) | closed-form AX=ZB | ✗ | 하드 | ✗ | OpenCV `calibrateRobotWorldHandEye` |
| Tabb & Ahmad Yousef 2017 | iterative reproj. AX=ZB | 부분(공유 Z) | 하드 | ✗ | 공개 C++ (빌드) |
| Allegro RA-L 2024 | unified 멀티카메라 | ✅ | 하드 | ✗ | 공개 C++ (리포 내 소스) — **이미지 필요 → 실데이터 전용** |
| **Kalib (IROS 2025) — 학습기반 확정** | AI 기준점 추적 (마커리스, CAD 불필요) | ✗ (per-cam) | FK 3D 좌표 사용 | ✗ | 피어리뷰 ✅ 코드·가중치 ✅ — 고정캠 전용, EE 스윕 영상 필요 |
| fixed-FK (내부) | unified, 큐브=FK 하드 | ✅ | 하드 | ✗ | 우리 파이프라인 설정 |
| no-FK (내부) | unified, 큐브 자유 | ✅ | gauge만 | ✗ | 우리 파이프라인 설정 |
| ours-A | unified, λ=0 | ✅ | gauge만 | ✅ | 우리 파이프라인 |
| **ours-B** | unified, soft anchor | ✅ | **soft (λ CV선택)** | ✅ | 우리 파이프라인 |

부록행: Park, Horaud, Andreff, Li (OpenCV 플래그 스윕). 조건부: Koide 2019 (g2o 빌드 성공 시).

**시뮬 vs 실데이터 역할 분담 (사용자 확정 2026-08: 타 방법 비교는 실데이터 전용)**
- **시뮬 (GT 있음)**: **내부 4방법(fixed-FK/no-FK/ours-A/ours-B)만** — 메커니즘 분석·조건 스윕 전용.
  외부 방법은 시뮬에서 돌리지 않음 (이미지 기반 SOTA는 입력 자체 불가, pose 기반도 실데이터로 통일).
- **실데이터**: 외부 6방법 + ours 전부, 동일 캡처·동일 지표. SOTA 비교는 여기서만.

---

## 1. 실데이터 지표 정의 — "정답을 아는 것"만으로 구성

| 지표 | 정의 | 정답의 출처 | 성격 |
|---|---|---|---|
| **M1. e_task** (mm) ★주지표 | train set으로 캘리브 → **held-out set 큐브 위치 예측** vs FK | FK (proxy GT — 한계 명시) | 일반화 성능. self-consistency 아님 |
| **M2. 치수 복원 오차 δ_dim** (% 또는 mm) | 캘리브된 멀티뷰 삼각측량으로 큐브 코너 3D 복원 → 변 길이 vs **제작 치수(캘리퍼 실측)** | 물리 치수 — **진짜 GT** | 카메라 네트워크의 기하 충실도. FK 무관 (Calib3R의 δs와 동일 계열 → 비교 가능) |
| **M3. 상대변위 오차 e_disp** (mm) | 로봇이 큐브를 정확히 Δ(예: 100.0mm 직선) 이동 → 카메라 추정 변위 vs 명령 변위 | 엔코더 상대 FK — 절대 FK보다 훨씬 정확 | 스케일·방향 정확도 (로봇 스케일 이슈 캐치) |
| **M4. held-out 재투영** (px) | 캘리브에 안 쓴 이미지에서 체인으로 예측한 코너 vs 검출 코너 | 검출 픽셀 좌표 | 표준 지표 (관례상 병기; 약한 지표임을 앎) |
| **M5. 일관성 e_t / e_θ** (mm/deg) | Allegro 논문의 AX=ZB 체인 잔차 지표 그대로 | (자기일관성) | **Allegro와의 사과-사과 비교용**으로만 병기 |
| M6. (선택) 물리 접촉 | 카메라가 지시한 점을 EE로 터치 → 실측 오프셋 | 자/다이얼게이지 | end-to-end 데모. 표보다 사진+수치 1개 |

주지표는 M1, 물리 GT는 M2·M3. "기존 논문들은 M5(자기일관성)에 머물지만 우리는 held-out(M1)과
물리 GT(M2·M3)로 평가한다"가 평가 프로토콜 기여 문장.

---

## 2. Table I — 실데이터 메인 표 (골격)

*동일 캡처 데이터, 동일 관측(검출·PnP), 방법만 교체. mean ± std over K-fold holdout.*

| Method | 결합 | FK | 보정 | M1 e_task (mm) ↓ | M2 δ_dim (mm) ↓ | M3 e_disp (mm) ↓ | M4 reproj (px) ↓ | M5 e_t (mm) ↓ |
|---|---|---|---|---|---|---|---|---|
| Tsai (per-cam) | ✗ | motion | ✗ | TBD | TBD | TBD | TBD | TBD |
| Daniilidis (per-cam) | ✗ | motion | ✗ | TBD | TBD | TBD | TBD | TBD |
| Shah | ✗ | hard | ✗ | TBD | TBD | TBD | TBD | TBD |
| Tabb 2017 | △ | hard | ✗ | TBD | TBD | TBD | TBD | TBD |
| Allegro 2024 | ✅ | hard | ✗ | TBD | TBD | TBD | TBD | TBD |
| ours-A (λ=0) | ✅ | gauge | ✅ | TBD | TBD | TBD | TBD | TBD |
| **ours-B** | ✅ | **soft** | ✅ | **TBD** | **TBD** | **TBD** | **TBD** | **TBD** |

- **타깃리스(Calib3R) 행은 코드 미공개로 제외** — related work 에서 인용 + "코드 미공개로 정량 비교
  불가" 명시. 코드 재공개 시 행 복귀. (대안: MASt3R-SfM(공개)+핸드아이 정렬 자체 구성 — 선택)
- 모든 방법이 bTc(+gTc)를 출력 → 지표 계산은 방법 무관 동일 코드로.
- Allegro는 보드-온-EE 세션 입력, 나머지는 큐브 세션 — **관측 세션이 다른 방법은 각주로 명시**
  (동일 워크셀·동일 카메라·같은 날 캡처로 공정성 확보).

## 3. Table II — 시뮬레이션 GT 표 (골격, 내부 방법 전용)

*합성 장면(실측 배치·실측 K/D 반영), nominal 노이즈(px 0.3–0.5 + 계통 1%), 20+ seeds.
외부 방법 없음 — 시뮬은 우리 4방법의 메커니즘 분석 전용 (사용자 확정).*

| Method | e_cam vs GT (mm/deg) ↓ | e_gTc vs GT (mm/deg) ↓ | M1 e_task (mm) ↓ |
|---|---|---|---|
| fixed-FK | TBD | TBD | TBD |
| no-FK | TBD | TBD | TBD |
| ours-A | TBD | TBD | TBD |
| **ours-B** | **TBD** | **TBD** | **TBD** |

+ 조건 sweep은 표가 아니라 기존 figure로: 승자 히트맵(fig_ww_grid), 3축 sweep(fig_ww_sweeps).

## 4. Table III — ablation (이미 데이터 있음)

| 항목 | 내용 | 출처 |
|---|---|---|
| anchor λ sweep | 0 / 0.5 / (3) / 5 → e_task | 실데이터 CP_C1 + 시뮬 36조건 |
| 잔차보정 유무 | ±correction × ±anchor 2×2 | which-wins |
| set 수 | 데이터량 vs e_task | set sweep |
| 초기 솔버 교체 | Tsai/Park/Horaud/Daniilidis 초기값 → 최종 결과 불변 | 부록 |

---

## 5. 실행 전 필요한 것 (표 채우기 전제)

1. 보드-온-EE 캡처 세션 (Tabb·Allegro·Shah 고정캠용) — PAPER_BASELINES.md 참조
2. M3용 정밀 변위 시퀀스 캡처 (몇 분이면 됨 — 같은 날 세션에 포함 권장)
3. M2용 큐브 치수 캘리퍼 실측 (1회)
4. Kalib 리포 의존성 점검 (트래커 가중치 다운로드, 입력 포맷)
