# 시뮬레이션 실험 계획 + 표/그래프 감사 (Plan & Audit)

> 목표: 논문용 Table 2a / 2b / Fig A / Fig B 를 **시뮬레이션으로 빠짐없이** 산출한다.
> 이 문서는 (1) 폴더 구조·실행 계획, (2) **제안된 표/그래프의 셀 단위 감사** —
> 불가능(❌)·무의미(⚠️)·누락(➕)·수정필요(✎) 를 하나도 빠짐없이 짚고, (3) **감사 후 최종
> 표/그래프 명세** 를 정리한다. 코드 착수 전 이 감사에 먼저 합의한다.

---

## Part 0. 핵심 결론 먼저 (TL;DR)

감사 결과 **제안 표에 반드시 고쳐야 할 4가지 근본 문제**가 있다:

1. **"FK ✓/✗" 이분법이 잘못됨.** FK 는 3가지다 — **미사용 / 고정(fixed) / 후보정(correction)**.
   Table 2a 는 A3=후보정으로 쓰면서 Fig B 설명은 fixed 를 전제한다(모순). → **FK 축을
   3-값으로 분리**해야 표와 그래프가 일관됨. (지난 미팅 혼란의 근본 원인과 동일)
2. **B3 (board only + FK ✓) 는 성립 불가.** FK 후보정·FK prior 는 로봇이 아는 큐브 위치가
   있어야 하는데 board 는 FK 가 없다(테이블 고정, 로봇이 위치 모름). → **B3 삭제 또는
   재정의.**
3. **현재 시뮬은 pose-level** 이라 `e_reproj (px)` 와 **Fig A 코너 노이즈 σ(px)** 를 못 낸다.
   → **corner-level 시뮬(3D코너→2D투영→픽셀노이즈→PnP)을 새로 만들어야** 함. (➕ 필수 구현)
4. **Fig B "관절각 노이즈"** 는 로봇 DH 모델이 없어 그대로는 불가. → **FK pose 에 SE(3)
   노이즈 직접 주입**으로 대체(더 일반적이고 충분). (✎ 재정의)

나머지 대부분(e_X, e_cross, 절제 A0→A3, 노이즈 스윕)은 시뮬로 정직하게 산출 가능.

---

## Part 1. 폴더 구조 & 실행 계획

`/home/sstone/rb-calibration-marker-experiment/Simulation/` (GitHub 업로드용, 기존 코드 무시하고
독립적으로 깔끔하게 재구성).

```
Simulation/
├── README.md                  # 진입점: 개요 + 실행법 + 결과 요약
├── PLAN_AND_AUDIT.md          # 이 문서
├── core/                      # 공유 시뮬 엔진
│   ├── scene.py               # 씬 생성 (GT 변환 샘플)
│   ├── targets.py             # cube(5~6면)·board 마커 코너 3D 정의
│   ├── project.py             # ★ corner-level: 3D코너→2D투영→PnP (신규)
│   ├── noise.py               # 코너노이즈(px)·FK노이즈(SE3)·pose노이즈 주입
│   └── metrics.py             # e_X, e_task_pose, e_cross, e_reproj, N_reg
├── methods/                   # 캘리브 방식 (한 방식 = 한 파일)
│   ├── independent.py         # 독립 (고정·그리퍼 따로 + 조합)
│   ├── unified.py             # 통합 (bundle adjustment)
│   ├── fk_fixed.py            # FK 고정 (큐브=FK 상수)
│   └── fk_correction.py       # FK 잔차보정 (Ridge, 후처리)
├── experiments/               # 표/그래프별 러너
│   ├── table2a_ablation.py    # A0–A3, B1–B2 절제
│   ├── table2b_noise.py       # σ(px) 스윕 표
│   ├── figA_corner_noise.py   # 코너 노이즈 강건성
│   └── figB_fk_noise.py       # FK 노이즈 crossover
├── configs.py                 # 공통 설정(카메라 수, set 수, seed, σ 범위)
└── results/
    ├── tables/                # *.json + *.csv (표 원자료)
    └── figures/               # *.png
```

**실행 원칙 (현명하게):**
- 각 experiment 스크립트는 `--dump results/tables/*.json` 으로 원자료 저장 → 표·그림은
  저장된 json 에서 렌더(재계산 없이 다시 그림). Exp1~3 에서 검증된 패턴.
- 무거운 건 통합(bundle adjustment)뿐 → seed·조합 수를 config 로 조절, 빠른 스모크(seed 6)
  후 최종(seed 30) 분리 실행. nohup 분리 실행.
- k-fold(held-out)로 편차(음영 밴드) 산출.

---

## Part 2. Table 2a 감사 (셀 단위)

### 2a 원안
| # | Marker | Unified | FK | 의미 |
|---|---|---|---|---|
| A0 | board | ✗ | ✗ | baseline |
| A1 | cube+board | ✗ | ✗ | +cube |
| A2 | cube+board | ✓ | ✗ | +unified |
| A3★ | cube+board | ✓ | ✓ | Ours (full) |
| B1 | cube+board | ✗ | ✓ | −Unified |
| B2 | cube only | ✓ | ✓ | −board |
| B3 | board only | ✓ | ✓ | −cube |

### 행별 판정

| # | 판정 | 근거 |
|---|---|---|
| A0 | ✅ 가능 | board 만, 독립, 보정없음. 순수 baseline. |
| A1 | ✅ 가능 | cube 추가 효과. |
| A2 | ✅ 가능 | 통합 추가 효과. **FK✗ = 후보정 없음**(큐브 미지수 추정). |
| A3★ | ✅ 가능 | **단, "FK✓"의 정의를 후보정으로 고정**해야 함(fixed 아님). |
| B1 | ✅ 가능 | 독립 + FK 후보정. "통합 뺐을 때". |
| B2 | ✅ 가능 | cube only + 통합 + 후보정. cube 는 FK 있음. |
| **B3** | ❌ **불가/모순** | **board only 인데 FK✓**. board 는 로봇이 위치를 모름(FK 없음) → FK 후보정 대상이 없다. 또한 board only 는 예측 대상(cube)이 없어 downstream(큐브예측) 지표 자체가 정의 안 됨. |

**B3 처리안 3택 (택1 합의 필요):**
- (a) **삭제** — "−cube" 절제는 A3에서 cube 만 빼면 board only+독립+FK불가 → 사실상 A0 로 수렴. 별 정보 없음.
- (b) **재정의**: "board only, 통합, **FK✗**" 로 바꿈(FK 없이 board 만으로 통합). 이건 가능(그리퍼 hand-eye 로 base 확보). 단 downstream 은 test 에서 cube 관측 필요.
- (c) **downstream 을 board pose 로** 평가(cube 대신 board 예측). 그러면 board only 도 평가 가능하나 A3 와 평가 대상이 달라져 공정비교 깨짐.
→ **권장: (a) 삭제** 또는 (b) FK✗ 로 재정의. (c) 는 비권장.

### 열(지표)별 판정

| 지표 | 판정 | 비고 |
|---|---|---|
| **N_reg** (등록 카메라 수) | ✅ | board only 는 관측 부족으로 미등록 카메라 생길 수 있음(그 자체가 결과). |
| **e_X (mm/°)** 변환 GT 오차 | ✅ **시뮬 핵심** | GT 보유 → 카메라·gTc 절대오차. 실데이터엔 없는 시뮬만의 강점. |
| **e_task_pose (mm/°)** | ✎ **정의 필요** | "작업 pose 오차"가 held-out 큐브예측인지, 파지목표 pose 인지 명시해야. 후보: held-out cube pose(회전 포함). |
| **e_e2e (mm/°)** | ✎ **정의 필요** | "end-to-end"가 무엇인지 불명확. 후보: 카메라→물체→base 전체 체인 pose 오차 or eye-in↔eye-to 정합. **하나로 확정 필요.** |
| **e_cross (mm)** | ✅ | 카메라 간 큐브위치 예측 산포. |
| **e_reproj (px)** | ⚠️→➕ | **현재 pose-level 시뮬에선 무의미**(주입 노이즈 되비침). **corner-level 시뮬 구축 시에만 의미.** Fig A 도 이걸 요구하므로 **corner-level 구축 필수(➕)**. |

---

## Part 3. Table 2b 감사 (σ 스윕)

### 2b 원안: A2 / A3★ / B1 / B2 × σ∈{0, 0.5, 1.0, 2.0}px, 셀 = e_X

| 항목 | 판정 | 근거 |
|---|---|---|
| σ 단위 px | ➕ | **corner-level 필요**(현재 pose-level은 σ가 mm/°). 구축 후 가능. |
| A2 vs A3 격차↑ 주장 | ✎ **주의** | "FK로 미지수↓ → 노이즈 강건" 은 **FK-fixed 논리**. A3=후보정이면 이 주장이 직접 성립 안 함. → **A3 의 FK 가 fixed 인지 correction 인지에 따라 이 표의 의미가 갈림.** |
| B2 vs A3 (σ=0 동률, σ↑ 벌어짐) | ✅ | board 의 노이즈강건 기여. corner-level 에서 유의미. |
| B1 vs A3 | ✅ | 통합의 노이즈 유효성. |

**핵심 합의사항:** Table 2b·Fig A·Fig B 의 "FK" 가 **fixed 인지 correction 인지**를 먼저 정해야
한다. 우리 Exp3 결과상 **FK-fixed 가 노이즈에 취약(crossover 발생)** 이고, **FK-correction 은
강건**하다. Fig B 의 crossover 스토리는 **fixed 전용**이다(아래).

---

## Part 4. Fig A / Fig B 감사

### Fig A — 코너 노이즈 강건성 (σ px 스윕, 곡선 A2/A3/B1/B2)
| 항목 | 판정 |
|---|---|
| 코너노이즈 주입 `u'=u+N(0,σ²)` | ➕ **corner-level 구축 필요** (현재 불가) |
| 곡선 4개(A2/A3/B1/B2) | ✅ (구축 후) |
| A2 vs A3 격차로 "FK 오차전파 감소" 주장 | ✎ **fixed/correction 명시 필요** (Part 3 과 동일) |

### Fig B — FK 노이즈 강건성 (crossover)
| 항목 | 판정 |
|---|---|
| **관절각 노이즈 `θ'=θ+N`** | ✎ **재정의**: 로봇 DH 모델 없음 → **FK pose(bTg 또는 큐브 prior)에 SE(3) 노이즈 직접 주입**(σ_fk mm/°)으로 대체. 더 일반적·충분. |
| A2(수평) vs A3(우상향) crossover | ⚠️ **A3=fixed 여야 성립** | **Fig B 의 스토리("FK가 틀리면 잘못된 상수에 못박혀 교정 불가")는 정확히 FK-fixed 의 동작.** A3=후보정이면 이 crossover 가 그대로 안 나옴. → **Fig B 는 "Camera-based vs FK-fixed" 축(=우리 Exp3)으로 그려야 맞다.** |
| crossover=신뢰한계 + 실셋업 FK정확도 수직선 | ✅ (fixed 로 그리면 의미 있음) |

**결론:** Fig B 는 사실상 **Exp3(Camera-based / FK-fixed / FK-correction)의 FK-노이즈 버전**이다.
제안서가 A2 vs A3 로 적었지만, crossover 를 보려면 **"FK 미사용(A2) vs FK-fixed"** 를 그려야 하고,
**A3(후보정)는 세 번째 곡선**으로 함께 그리면 "후보정이 fixed 보다 FK노이즈에 강건"까지 보여줌.

---

## Part 5. 감사 후 최종 명세

### 5-1. FK 축을 3-값으로 (모든 표·그림 공통)
| 코드 | 뜻 | 캘리브 |
|---|---|---|
| `none` | FK 미사용 | 큐브 미지수 추정 (Camera-based) |
| `fixed` | FK 고정 | 큐브=FK 상수, 카메라·gTc만 최적화 |
| `corr` | FK 후보정 | Camera-based + train 잔차 Ridge 후보정 (**채택**) |

### 5-2. Table 2a (최종) — 절제, 시뮬 GT 지표
B3 삭제, FK 열을 3-값으로, 지표 정의 확정.

| # | Marker | Unified | FK | 의미 | N_reg | e_X(mm/°) | e_task(mm/°) | e_cross(mm) | e_reproj(px) |
|---|---|---|---|---|---|---|---|---|---|
| A0 | board | ✗ | none | baseline | | | | | |
| A1 | cube+board | ✗ | none | +cube | | | | | |
| A2 | cube+board | ✓ | none | +unified | | | | | |
| **A3★** | cube+board | ✓ | **corr** | **Ours** | | | | | |
| B1 | cube+board | ✗ | corr | −unified | | | | | |
| B2 | cube only | ✓ | corr | −board | | | | | |
| (B1′) | cube+board | ✓ | **fixed** | FK 고정 대조 | | | | | |

- **e_task_pose = held-out 큐브 pose 예측 오차(위치 mm + 회전°)**, **e_e2e 는 삭제**(정의
  모호, e_task 와 중복). → 지표 5개로 슬림화: N_reg / e_X / e_task / e_cross / e_reproj.
- (B1′) 추가 권장: **채택안(corr)이 fixed 보다 낫다**는 직접 근거(Exp3 핵심). 없으면 "왜 굳이
  후보정?"에 답 못 함.

### 5-3. Table 2b (최종) — 코너 노이즈 σ(px) 스윕
행: A2(none) / A3(corr) / B1(indep+corr) / B2(cube only) / **B1′(fixed)**. 셀 = e_X(mm/°).
corner-level 구축 후 산출. 핵심 대조: **corr vs fixed vs none** 의 σ 의존성.

### 5-4. Fig A (최종) — 코너 노이즈 강건성
x=σ(px) 0~2, y=e_X(mm) (+회전 서브플롯), 곡선 = none/fixed/corr(+board 유무). corr 굵게,
k-fold 음영. **corner-level 필수.**

### 5-5. Fig B (최종) — FK 노이즈 강건성 (crossover)
x=FK pose 노이즈 σ_fk(mm 또는 °), 곡선 3개: **none(수평) / fixed(우상향) / corr**. fixed 와
none 의 **crossover=FK 신뢰한계**, corr 이 그 사이에서 강건함을 함께 표시. 실셋업 추정
FK정확도 수직선.

---

## Part 6. 구현 우선순위 (합의 후 착수)

1. **[핵심·신규] corner-level 시뮬** (`core/project.py`, `core/targets.py`, `core/noise.py`):
   cube/board 마커 3D코너 정의 → 카메라 K,D 로 2D 투영 → 픽셀 노이즈 → solvePnP → pose.
   이게 있어야 e_reproj·Fig A·Table 2b(px) 가 나온다. **가장 큰 신규 작업.**
2. **[신규] FK pose 노이즈 주입** (`core/noise.py`): bTg/큐브 prior 에 SE(3) 섭동.
3. **[재사용] 4 방식** (`methods/`): 기존 Simul_test 의 independent/unified/fk_fixed/fk_correction
   로직 포팅(검증됨).
4. **[신규] 지표** (`core/metrics.py`): e_X·e_task·e_cross·e_reproj·N_reg.
5. **[러너] experiments/** 4개 + results json→figure 렌더.

---

## 부록 — 미해결/합의 필요 목록 (체크리스트)

- [ ] **B3 처리**: 삭제 / FK✗ 재정의 중 택1
- [ ] **FK 축 3-값** 채택 여부 (none/fixed/corr)
- [ ] **Fig B 를 fixed 축으로** 그리는 것 동의
- [ ] **e_task_pose 정의** = held-out 큐브 pose 로 확정, **e_e2e 삭제** 동의
- [ ] **B1′(fixed 대조)** 행 추가 여부
- [ ] **corner-level 시뮬 구축** 착수 승인 (가장 큰 작업)
- [ ] **FK 노이즈 = SE(3) 직접 주입**으로 대체 동의 (관절각 모델 안 씀)
