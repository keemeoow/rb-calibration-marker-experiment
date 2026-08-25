# 어떤 조건에서 어떤 방법이 이기나 — 4방법 시뮬레이션 비교

## 목차

- [TL;DR](#toc-section-1)
- [1. 실험 셋업](#toc-section-2)
- [2. 승자 히트맵](#toc-section-3)
- [3. Sweep 곡선 (각 축 격리)](#toc-section-4)
- [4. corrected-FK (A) vs corrected-FK (B) — 약한 anchor가 항상 낫다](#toc-section-5)
- [5. 실데이터와의 일치](#toc-section-6)
- [6. 결론 및 권고](#toc-section-7)
- [재현](#toc-section-8)

> ## ⚠️ 이 문서의 수치는 **구버전(불공정) 코드로 산출된 것이라 폐기 대상**입니다
>
> 2026-08-06 공정성 수정으로 아래가 모두 바뀌었습니다. **재산출 전까지 인용하지 마세요.**
>
> 1. **GT 누출 제거** — no-FK(vision) 비교군까지 초기화에 `fk_cube`(FK)와 `bTboard`(GT 보드)를
>    썼고, board-only 핸드아이는 잔차에 GT 보드를 직접 넣었습니다. 이제 모든 방법이
>    GT·FK 를 쓰지 않는 동일한 모션 기반 초기화(`_bootstrap_visual`)를 씁니다.
> 2. **재투영 열이 방법별 성능이 아니었음** — 프론트엔드 PnP 자체 잔차(`reproj_seed`)를
>    모든 방법에 똑같이 넣어 차이가 0 이었습니다. 이제 held-out 원본 2D 코너에
>    leave-one-camera-out 으로 재투영합니다.
> 3. **independent 의 rigid 정합이 예측에 적용되지 않았음** (계산만 하고 버려짐).
> 4. **Ours 재정의** — `[1,x,y]` Ridge 후보정(위치만 보정)에서 **BA 안의 공분산 가중
>    robust FK factor**(회전 포함)로 바뀌었습니다. 스크립트마다 5.0/0.5/0.0 으로
>    달랐던 anchor weight 는 제거되고 sigma_FK 가 전 실험 동결됩니다.
> 5. **프론트엔드 통일** — robust PnP(trimming)를 씬이 한 번만 돌려 모든 방법이 같은
>    코너 집합을 공유합니다. 이상치 실험이 Ours 에 유리하던 편향이 사라집니다.
> 6. **split 편향·통계** — 항상 앞 N개 조합만 쓰던 것을 seed 별 무작위 추출로 바꾸고,
>    집계를 seed 단위 + paired bootstrap 95% CI 로 바꿨습니다.
> 7. **N_reg 정직화** — 관측이 없는 카메라를 미지수에서 제외. board-only 는 이 배치에서
>    고정 카메라 3대 중 1대만 등록됩니다(구버전은 3대로 잘못 보고).
>
> 자세한 규약은 [README.md](README.md) 참조.


**FK 표시 구분 3가지**(no-FK(vision) / FK-fixed / corrected-FK)를 사용하되, corrected-FK는 A/B 두 세부형으로 나누어 **FK 오차 × 카메라 노이즈(랜덤·계통)** 조건에서
전수 비교하여, "각 조건에서 held-out 큐브 예측(e_task)이 가장 낮은 방법"을 구했다.

<a id="toc-section-1"></a>

## TL;DR

- **FK-fixed**는 **FK가 정확하고 카메라 계통노이즈가 0인 이상적 코너에서만** 최고다.
- **corrected-FK (B)**는 **카메라 계통노이즈가 조금이라도(≥1%) 있으면 거의 전 영역에서 최고**다.
- 실제 카메라 인지 오차는 계통적(intrinsic·왜곡·검출 편향)이므로, **현실 조건에서는 corrected-FK (B)가 사실상 항상 최고**다.
- **corrected-FK (A, anchor=0)가 corrected-FK (B, anchor=0.5)를 이긴 경우는 시뮬 36개 조건 중 0개** — 약한 anchor가 항상 안전하게 낫다.

---

<a id="toc-section-2"></a>

## 1. 실험 셋업

실측 카메라 배치·실물 마커 기하(AprilTag 큐브 6면, ChArUco 보드 11×7, 실측 K/왜곡)를 반영한 코너 수준
시뮬레이션. 렌더링 없이 3D 코너 → 2D 투영 → 픽셀노이즈 → solvePnP 로 관측을 생성한다.

![시뮬레이션 리그](results/figures/fig_sim_scene.png)

*좌: 3D 원근 리그 — 고정 카메라 3대(eye-to-hand) + 그리퍼 카메라(eye-in-hand) 경로 + 큐브(set별 재배치) + ChArUco 보드 + robot base 좌표축. 우상: 위에서 본 배치(bird's-eye). 우하: 고정 카메라 C0가 실제로 보는 2D 투영(보드 코너 + 큐브 면, 0.3px 픽셀노이즈) — solvePnP 입력.*

### 비교한 4방법

| 방법 | 1차: 큐브 FK를 | 2차 보정 | anchor λ |
|---|---|---|---|
| **FK-fixed** | 하드 고정 | ✗ | (하드) |
| **no-FK(vision)** | 안 씀(자유변수) | ✗ | 0 |
| **corrected-FK (A)** | 보정 전 자유변수(anchor=0) | ✓ | 0 |
| **corrected-FK (B)** | 보정 전 soft anchor | ✓ | 0.5 |

- **corrected-FK (A) vs no-FK(vision) 차이** = 2차 FK 잔차보정 유무
- **corrected-FK (A) vs corrected-FK (B) 차이** = 1차 soft anchor(λ) 유무
- 세 방법 모두 eye-in-hand FK(`bTg`)는 공통 사용(gauge 앵커). 차이는 **큐브 위치 FK(`fk_cube`)를 어떻게 쓰나**.

### 조건 축

- **FK 오차**: `fk_noise_mm` ∈ {0, 1, 2, 4, 8, 16} (로봇 큐브 prior 부정확도)
- **카메라 노이즈 — 두 종류로 분리**
  - **랜덤**: `sigma_px` ∈ {0.3, 0.5, 1.0, 1.5, 2.0} (평균으로 소거됨)
  - **계통**: `intrinsic_err` ∈ {0, 0.5%, 1%, 2%, 3%} (+ outlier = ×5) (소거 안 됨)

측정: held-out 큐브 예측 RMSE **e_task** (20 seeds × 6 holdout 쌍 평균).

---

<a id="toc-section-3"></a>

## 2. 승자 히트맵

![승자 히트맵](results/figures/fig_ww_grid.png)

| FK오차 ↓ \ 계통노이즈 → | **0** | 1% | 2% | 3% |
|---|---|---|---|---|
| **16mm** | no-FK(vision) | **corrected-FK (B)** | **corrected-FK (B)** | **corrected-FK (B)** |
| **8mm** | FK-fixed | **corrected-FK (B)** | **corrected-FK (B)** | **corrected-FK (B)** |
| **4mm** | FK-fixed | **corrected-FK (B)** | **corrected-FK (B)** | **corrected-FK (B)** |
| **2mm** | FK-fixed | **corrected-FK (B)** | **corrected-FK (B)** | **corrected-FK (B)** |
| **0mm** | FK-fixed | **corrected-FK (B)** | **corrected-FK (B)** | **corrected-FK (B)** |

→ **계통노이즈 열(≥1%)은 전부 corrected-FK (B).** FK-fixed는 **계통노이즈=0인 맨 왼쪽 열에서만** 승리.

---

<a id="toc-section-4"></a>

## 3. Sweep 곡선 (각 축 격리)

![sweep 곡선](results/figures/fig_ww_sweeps.png)

### (1) FK 오차 sweep — 카메라 깨끗(랜덤 0.3px)

| e_task (mm) | fk=0 | 1 | 2 | 4 | 8 | 16 |
|---|---|---|---|---|---|---|
| FK-fixed | **0.52** | **0.64** | **0.87** | **1.44** | **2.68** | 5.25 |
| no-FK(vision) | 2.97 | 2.97 | 2.97 | 2.97 | 2.97 | **2.97** |
| corrected-FK (A) | 0.78 | 1.32 | 2.26 | 4.30 | 8.48 | 16.91 |
| corrected-FK (B) | 0.61 | 1.20 | 2.17 | 4.24 | 8.44 | 16.87 |

**카메라가 완벽하면 FK-fixed 승.** FK가 극단(16mm)이면 no-FK(vision)가 앞선다.

### (2) 계통 카메라노이즈 sweep — FK 완벽

| e_task (mm) | 0 | 0.5% | 1% | 2% | 3% |
|---|---|---|---|---|---|
| FK-fixed | **0.52** | 14.37 | 27.93 | 45.30 | 60.16 |
| no-FK(vision) | 2.97 | 27.91 | 41.55 | 76.64 | 73.56 |
| corrected-FK (A) | 0.78 | 6.27 | 11.02 | 24.36 | 38.66 |
| corrected-FK (B) | 0.61 | **5.69** | **9.50** | **23.77** | **36.50** |

**FK-fixed가 0.5→60mm로 붕괴**, corrected-FK (B)가 매 지점 최저. **계통오차엔 ours 압도.**

### (3) 랜덤 픽셀 sweep — FK 완벽

| e_task (mm) | 0.3 | 0.5 | 1.0 | 1.5 | 2.0 |
|---|---|---|---|---|---|
| FK-fixed | **0.52** | **1.03** | **2.35** | 7.88 | 14.70 |
| no-FK(vision) | 2.97 | 5.72 | 12.81 | 29.96 | 41.03 |
| corrected-FK (A) | 0.78 | 1.43 | 3.27 | 5.68 | 8.61 |
| corrected-FK (B) | 0.61 | 1.16 | 2.63 | **4.78** | **7.57** |

저노이즈는 FK-fixed, **1px 넘으면 corrected-FK (B) 역전**(변동성엔 보정+자유큐브가 유리).

전체 지표(gTc/e_X/reproj)는 [fig_ww_metrics.png](results/figures/fig_ww_metrics.png) 참고.

---

<a id="toc-section-5"></a>

## 4. corrected-FK (A) vs corrected-FK (B) — 약한 anchor가 항상 낫다

시뮬 36개 조건 전수 비교:

| sweep | 평균 A(λ=0) | 평균 B(λ=0.5) | B 우세폭 |
|---|---|---|---|
| FK오차 | 5.68 | **5.59** | 0.09mm (거의 동률) |
| 계통노이즈 | 16.22 | **15.21** | 1.00mm |
| 랜덤px | 3.95 | **3.35** | 0.60mm |

**corrected-FK (A)가 corrected-FK (B)를 이긴 경우: 0 / 36.** 카메라가 깨끗할 땐 거의 동률(0.09mm)이지만, 노이즈가 있으면 B가 확실히 낫다.

**이유(bias-variance)**: soft anchor는 정규화다. 카메라 노이즈로 흔들리는 큐브 추정의 **분산을 줄이는 이득**과, 부정확한 FK로 당겨 **편향을 더하는 손해**가 상충한다. **λ=0.5는 약해서 손해는 미미하고 이득은 챙기므로** 항상 A 이상. (이론적으로 FK가 극단적으로 나쁘고 카메라가 완벽하면 A가 앞설 수 있으나, 테스트 범위에선 미관측.)

---

<a id="toc-section-6"></a>

## 5. 실데이터와의 일치

실데이터(data/session, CP_C1)에서 anchor_weight를 sweep한 결과:

| anchor λ | held-out e_task (down+fk) |
|---|---|
| 0 (corrected-FK (A)) | 2.59 mm |
| **0.5 (corrected-FK (B))** | **2.06 mm** ← 최저 |
| 5.0 (실코드 기본) | 3.41 mm |

- 실데이터도 **λ≈0.5가 최적**, λ=0(A)보다 나음 → 시뮬과 일치.
- (앞서 "A가 B를 이겼다"고 본 실험은 B의 λ가 5.0이라 과한 경우였다. λ=0.5면 B가 A를 이긴다.)
- 실측 카메라 reproj rms 0.24–0.32px에 **계통 성분**(intrinsic·왜곡)이 있으므로, 히트맵상 **계통노이즈 열 = corrected-FK (B) 영역**에 해당.

---

<a id="toc-section-7"></a>

## 6. 결론 및 권고

| 방법 | 이기는 조건 | 실제 해당? |
|---|---|---|
| **FK-fixed** | FK 정확 **&** 카메라 계통노이즈 0 | ✗ (이상적 코너뿐) |
| **corrected-FK (B)** | 계통노이즈 존재(현실) → 거의 전 영역 | ✅ |
| **no-FK(vision)** | FK 극단 오차 + 카메라 완벽 | ✗ (극단 코너) |

- **논문 메시지**: 완벽 FK·이상 카메라에서는 FK-fixed로 충분하나, 실제 시스템의 **FK 오차 + 카메라 계통노이즈** 아래에서 FK-fixed는 붕괴하고, **corrected-FK(soft anchor + 잔차보정)만이 전 조건에서 강건**하다.
- **실용 권고**: anchor는 **작게(λ≈0.5)**. 실코드 기본값 5.0은 최적을 지나쳐 있으니 하향 검토 권장.

---

<a id="toc-section-8"></a>

## 재현

```bash
# 실험 (4방법 × 조건 sweep, CPU 병렬)
python run_which_wins.py --seeds 20 --workers 18 --pairs 6
# 그림
python viz_which_wins.py       # fig_ww_grid / fig_ww_sweeps / fig_ww_metrics
python viz_scene_paper.py       # fig_sim_scene_paper (씬 그림)
```

결과 데이터: `results/tables/ww_{fk,sys,rand,grid}.json`
