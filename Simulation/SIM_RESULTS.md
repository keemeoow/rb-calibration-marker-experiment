# 시뮬레이션 결과 — 7방법 ablation (GT 기준, real 충실 재판정)

## 목차

- [방법 (7가지 ablation)](#toc-section-1)
- [지표 쉽게 이해하기](#toc-section-2)
- [📊 그림 A — 노이즈별 e_task (4패널)](#toc-section-3)
- [📊 그림 B — 승자맵 (FK 오차=0 고정; 계통·오검출을 변화)](#toc-section-4)
- [📋 표 1 — 현실 종합 조건 (헤드라인)](#toc-section-5)
- [📋 표 1b — 조건별 e_task (mm, GT) — "어디서 누가 이기나"](#toc-section-6)
- [📈 촬영 셋 수 → 성능 (대표 4방식)](#toc-section-7)
- [정직한 프레이밍 (논문용)](#toc-section-8)
- [재현](#toc-section-9)

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

> 리뷰어(real 코드 작성자)의 6개 지적을 모두 고치고, **시뮬을 실측(real) 구성에 맞춘 뒤**
> 다시 돌린 결과다. 특히 **FK 보정을 real 파이프라인 방식(de-bias)으로 재구현**했다.
> 대조·반영 내역은 [SIM_VS_REAL_CHECKLIST.md](SIM_VS_REAL_CHECKLIST.md), 리뷰 수정은
> [SIM_REVIEW_FIXES.md](SIM_REVIEW_FIXES.md).

---

## ⚡ 한 줄 결론

1. **진짜 기여 = 통합(unified) 공동 캘리브.** 모든 카메라를 하나로 묶어 푸는 통합이 따로 푸는
   독립(independent)보다 상대 정합(e_rel) 3~5배, 재투영 3배 정확. **멀티카메라 캘리브의 핵심.**
2. **큐브 필수.** 보드만 쓰면(EXP6) 81% 발산. 큐브의 다면 마커가 캘리브를 안정화한다.
3. **FK 보정(Ours)의 가치 = fixed-FK 대비 안전.** 실측 FK 는 systematic 오차가 있는데
   (리뷰어 확인: 180° flip + 오프셋 + 6~17mm), **raw FK 를 그대로 믿는 fixed-FK 는 이때 무너지고
   (FK 없을 때 0.96 → systematic FK 10mm 에서 2.63mm)**, **de-bias 하는 Ours 는 평평(~1.3mm)**하다.
4. **단, Ours 는 no-FK 를 이기지 못한다.** 고정 카메라 3대의 vision(~1.2mm)이 이미 FK prior(~2.6mm)보다
   정확해 FK 가 중복이다. Ours ≈ no-FK(둘 다 안전). → **"FK 를 쓸 거면 반드시 보정(Ours), 안 쓰면 no-FK."**

---

## 🔧 real 에 맞춘 것 (SIM_VS_REAL_CHECKLIST 요약)

| 항목 | 반영 |
|---|---|
| **FK 오차** | ≈0/random → **systematic**(상수 오정렬 + per-set 잔차, 실측 ~6.6mm). FK 있음/없음 둘 다 실험 |
| **FK 보정 방식** | "예측을 FK 로 당김"(틀림) → **real 방식 de-bias**: `T_delta=robust_avg(inv(FK)@vision)` 로 FK 를 vision 에 정렬 후 앵커 |
| **카메라 intrinsic** | 평균 K 1개 → **카메라별 개별 실측 K 4개** (fixed=cam0/1/3, grip=cam2) |
| **프로토콜** | 13/130 → 실측 **11 eih/set · 89 gripped** |
| 하향각 | 27° 유지 (물리 리그 재조정 확인) |
| 큐브·보드 기하 | 이미 실측 정본(config.py)과 일치 |

---

<a id="toc-section-1"></a>

## 방법 (7가지 ablation)

| # | FK | 캘리브 | 마커 | 이름 |
|---|---|---|---|---|
| **EXP1** | corrected-FK | 통합 | 큐브+보드 | **Ours** |
| EXP2 | corrected-FK | 따로 | 큐브+보드 | −통합 |
| EXP3 | corrected-FK | 통합 | 큐브만 | −보드 |
| EXP4 | no-FK(vision) | 통합 | 큐브+보드 | −FK |
| EXP5 | no-FK(vision) | 따로 | 큐브+보드 | −FK−통합 |
| EXP6 | no-FK(vision) | 통합 | 보드만 | −큐브 |
| EXP7 | FK-fixed | 통합(=따로) | 큐브+보드 | raw FK 고정 |

<a id="toc-section-2"></a>

- **FK 보정(corr)**: raw FK 를 vision 으로 **de-bias**(상수 오정렬 제거) 후 앵커 (= real `set_cube_center_prior`).
- **fixed-FK**: raw FK 를 그대로 하드 상수로 사용 (de-bias 안 함) — systematic FK 에 취약한 대조군.

## 지표

| 지표 | 뜻 | 실데이터서 측정 |
|---|---|---|
| **e_task** | 새 큐브 위치 예측 오차 (mm) — 실전 정확도 | ✗ (시뮬 GT) |
| **e_X** | 카메라+hand-eye 절대 오차 (mm) | ✗ |
| **e_rel** | 카메라 상대 정합 (mm, gauge 불변) — 3D 정합 | ✗ |
| **reproj_raw** | held-out 픽셀 재투영 (px) — FK 무관 | ✅ |
| **발산%** | e_task>100mm(수렴 실패) 비율 | — |

---

<a id="toc-section-3"></a>

## 📊 그림 A — 노이즈별 e_task (4패널)

![그림 A](results/figures/fig_paperA_sweeps.png)

| 노이즈 축 | 결과 | 해석 |
|---|---|---|
| **마커 σ** (코너 정밀도) | **Ours 최저** | 랜덤 코너노이즈에 가장 강건 |
| **계통노이즈** (intrinsic 편향) | corrected-FK ≈ FK-fixed 최저 | 순수 계통에는 FK-fixed도 경쟁력 있음 |
| **FK 오차** (랜덤) | **Ours 최악** | 보정이 틀린 FK를 쫓음 — 정직한 약점 (FK≈0이면 무관) |
| **오검출** (마커 misdetection) | **Ours 최저, no-FK(vision) 붕괴** | ⭐ 핵심 강점 |

<a id="toc-section-4"></a>

- **fixed-FK 만 우상향**(0.96→2.63). Ours·no-FK 는 평평 → **보정이 systematic FK 를 잡는다**.
- Ours 가 no-FK 를 못 이기는 건, 고정 카메라 vision 이 이미 FK 보다 정확해 FK 가 중복이기 때문.

## 📊 그림 A2 — 상대 정합(e_rel) vs 노이즈 ⭐ 핵심 기여

![그림 A2](results/figures/fig_paperA2_rel.png)

- **맨 아래 행(오검출 0%)** → FK-fixed 경쟁력
- **위로 갈수록(오검출 ≥5%, 현실)** → 전 계통 레벨에서 **Ours 압도** (격차 +3~11mm)
- → **"오검출이 있으면 Ours"** 를 한 장으로. (EXP6은 캘리브 붕괴라 승자 후보에서 제외)

---

<a id="toc-section-5"></a>

## 📋 표 1 — 현실 종합 조건 (헤드라인)

1. **통합 ≫ 독립** (견고). 정합·재투영에서 3~5배. 멀티카메라의 존재 이유.
2. **큐브 필수** (EXP6 81% 발산). 보드는 정합에 기여 안 함(EXP3 큐브만 ≥ EXP1).
3. **FK 보정(de-bias)의 가치 = fixed-FK 구제.** 실측 systematic FK 에서 fixed-FK 는 무너지고
   Ours 는 평평. "FK 를 쓸 거면 raw(fixed-FK) 말고 de-bias(Ours)."
4. **Ours 는 no-FK 를 이기지 못함** (이 셋업에선). 고정 카메라 vision(~1.2mm) > FK prior(~2.6mm)라
   FK 가 중복. set 을 줄여도 마찬가지. → **no-FK 도 똑같이 안전**하고 더 단순.

| 방법 | e_task mm | e_task ° | e_X mm | cam→base 병진 (bTf) | reproj px | cross mm |
|---|--:|--:|--:|--:|--:|--:|
| **EXP1 Ours** | **12.08** | **6.60** | 70.58 | 80.20 | **17.05** | 97.66 |
| EXP2 −통합 | 22.56 | 12.59 | **44.89** | **39.93** | 40.59 | **75.04** |
| EXP3 −보드 | 15.95 | 6.70 | 75.55 | 84.27 | 17.86 | 99.24 |
| EXP4 −FK | 25.90 | 7.26 | 73.88 | 85.46 | 19.56 | 97.65 |
| EXP5 −FK−통합 | 29.49 | 12.59 | **44.89** | **39.93** | 40.59 | **75.04** |
| EXP6 −FK−큐브 | ~~6.51~~ | 13.30 | 447.4 ⚠️ | 591.7 ⚠️ | 387.2 ⚠️ | 307.2 |
| EXP7 FK-fixed | 26.55 | 6.66 | 71.15 | 80.81 | **17.03** | 97.67 |

- **고정 카메라 3대가 vision 을 강하게** 만들어 FK 를 중복화한다. 고정 카메라가 적거나(1~2대)
  없으면(순수 eye-in-hand) FK 의 가치가 커질 수 있다 — **미검증(다음 실험 후보: 카메라 수 sweep)**.
- 실측 vision(~2mm) ≈ FK(~3mm) 이라 real 에선 FK 가 시뮬보다 조금 더 유용할 수 있다.
- 최종 판정은 실데이터 몫. reproj_raw(FK 무관)로 real 과 직접 대조 가능.

## 📌 논문 프레이밍 제안

**방법론 포인트**: reproj는 FK-fixed(17.0)도 Ours(17.1)급인데 e_task는 2배 차(26.6 vs 12.1).
→ **reproj만으론 방법을 못 가린다. 실전 지표(e_task/태스크)가 필요.**

<a id="toc-section-6"></a>

## 📋 표 1b — 조건별 e_task (mm, GT) — "어디서 누가 이기나"

| 방법 | 이상적 | 현실종합 | +FK오차 | +오검출 |
|---|--:|--:|--:|--:|
| **EXP1 Ours** | 0.00 | **12.08** | 3.82 | **21.77** |
| EXP2 −통합 | 0.00 | 22.56 | 4.20 | 62.28 |
| EXP3 −보드 | 0.00 | 15.95 | 3.84 | 27.91 |
| EXP4 −FK | 0.00 | 25.90 | **1.22** | 114.78 |
| EXP5 −FK−통합 | 0.00 | 29.49 | 2.69 | 96.60 |
| EXP6 −큐브 | 0.00 | ~~6.51~~ | ~~0.66~~ | 45.62 |
| EXP7 FK-fixed | 0.00 | 26.55 | 1.26 | 51.95 |

- **현실종합·오검출 열 → Ours 최고** (유효 방법 중)
- **FK오차 열 → Ours 약점** (FK-free가 유리) → **"FK 정확" 전제 명시 필요**

---

<a id="toc-section-7"></a>

## 📈 촬영 셋 수 → 성능 (대표 4방식)

*대표 지표 e_task, 현실 조건(FK≈0+계통2%+오검출5%), gripped 40, **20 seeds**.*

![셋 증가](results/figures/fig_sets_sweep.png)

| 방식 \ 셋 수 | 4 | 6 | 8 | 10 | 13 | 16 |
|---|--:|--:|--:|--:|--:|--:|
| **Ours** | 26.8 | 13.1 | 13.9 | **9.6** | 9.9 | 11.2 |
| FK-fixed | 28.0 | 26.7 | 26.1 | 25.3 | 27.5 | 28.6 |
| no-FK(vision) | 26.9 | 21.7 | 23.8 | 20.6 | 21.7 | 22.1 |
| −통합(indep) | 102.2 | 28.3 | 40.1 | 21.9 | 17.6 | 26.2 |

**핵심 발견**:
- **Ours만 셋↑에 크게 수렴** (26.8 → ~10mm, 6셋부터 최저) — FK 잔차보정이 셋이 많을수록 더 잘 학습됨.
- **FK-fixed는 평평(~27mm)** — 데이터를 늘려도 안 좋아짐. 오차가 **편향(계통+오검출)** 지배라, 데이터로 못 줄임.
- **no-FK(vision)도 평평(~22mm)**, −통합은 소량 셋에서 불안정(102mm).
- → **"더 많이 촬영할수록 이득 보는 건 Ours뿐"** — 논문의 실용적 강점 (데이터 효율성).

---

<a id="toc-section-8"></a>

## 정직한 프레이밍 (논문용)

1. **Ours가 최고인 조건**: FK≈0 + (계통노이즈 or **오검출**). 사용자 실제 계획과 일치.
2. **Ours의 약점**: 순수 FK 오차엔 취약(보정이 틀린 FK를 쫓음) → **"FK를 정확히 맞춘다"는 전제**가 필수. (표 1b +FK오차 열에 정직히 표기)
3. **핵심 셀링포인트 = 오검출 강건성**: 실제 마커 검출엔 오검출(grazing angle PnP 뒤집힘 등)이 있으므로 Ours가 정당화됨.
4. **큐브 필요성**: EXP6(−큐브) 붕괴(e_X 447)로 증명.
5. **평가 지표 주의**: reproj만으론 방법을 못 가린다 → e_task(GT)/물리 태스크로 판정. 실데이터는 GT가 없어 FK-프록시가 순환이므로, **진짜 정확도 판정은 시뮬 GT가 담당**.

<a id="toc-section-9"></a>

## 재현

```bash
cd Simulation
OMP_NUM_THREADS=1 python run_paper_sim.py --seeds 16 --splits 3 --workers 32  # 표·그림 데이터
python viz_paper_sim.py                                                        # 그림 A/A2/A3/B + 표
OMP_NUM_THREADS=1 python run_sets_sweep.py --seeds 24 --workers 16             # 학습 set 수 sweep
```
환경: conda `rb-calib` (numpy, scipy, opencv-python, matplotlib). 단일 스레드 권장(워커 경쟁 방지).
