# CP 기여도 실험 (C1 / C2 / C3) — 실데이터 정리

`Simul_test/` 의 순수 SE(3) 기하 **시뮬레이션** 으로 검증한 세 가지 설계 선택을, 실제 촬영
세션의 관측으로 **동일하게** 재현·측정하는 코드다. 시뮬은 GT(정답 카메라·핸드아이·큐브)를
알기에 정답 대비 오차를 직접 재지만, 실데이터에는 GT 가 없다. 그래서 GT 자리에 **로봇 FK
큐브중점을 정답 프록시**로 쓴다(held-out 예측 오차·consistency 로 대체).

각 CP 실험은 C1/C2/C3 서로 독립 실행되며, 공유 로더/기하/지표는 `CP_common.py` 에 있다.

---

## 시뮬 ↔ 실데이터 대응

| # | 기여 | 시뮬 (기준) | CP 실데이터 | 핵심 지표(FK 프록시) |
|---|---|---|---|---|
| **C1** | Unified vs Independent | `unified_vs_independent.py` | `CP_C1_unified_vs_independent.py` | held-out 큐브예측(mm) + `+fk` 보정, consistency |
| **C2** | Board vs Cube | `exp2_board_vs_cube.py` | `CP_C2_cube_vs_board.py` | 관측성(동시관측·시야각) + cross-camera/재투영 |
| **C3** | gTc estimation (Camera / FK / Camera+FK) | `exp3_gtc_estimation.py` | `CP_C3_prior_vs_noprior.py` | held-out FK 위치오차(mm) 3방식 비교 |

> 이번 정리에서 시뮬 대비 빠져 있던 **핵심 요소**를 실데이터 코드에 채웠다(아래 각 절 "정렬
> 추가"). GT 전용 지표(bTf·gTc·카메라위치 절대오차)는 실데이터에서 측정 불가라 제외하고,
> 측정 가능한 held-out 예측·consistency·관측성으로 대체했다.

---

## C1 — Unified vs Independent  (`CP_C1_unified_vs_independent.py`)

Eye-in-hand(그리퍼) + eye-to-hand(고정) 를 **하나로 통합(joint)** vs **독립(따로)** 으로 풀어
비교. 세 솔버:

- `independent`    : 고정=각자 FK 큐브 closed-form, 그리퍼=gTc 단독 least-sq. 정보교환 없음.
- `unified_joint`  : 모든 관측을 하나의 비선형 최소제곱으로 {T_base_Ci, gTc, cube[set]} 동시
                     최적화. cube 자유변수, gauge 는 FK soft anchor.
- `joint_fk_fixed` : cube 를 FK 로 고정하고 {T_base_Ci, gTc} 만 동시 최적화.

**정렬 추가 (시뮬 `eval_model` 짝):**
- `--test_sets` / `--holdout_frac` **held-out 분할** — 카메라는 train set 으로만 fit.
- **다운스트림 큐브예측**: held-out test set 큐브를 base 에서 예측 → FK 대비 RMSE(`down_mm`).
- **`+fk` (Ridge 잔차보정 = 시뮬 C방법)**: train 잔차를 `[1,x,y]` 에 Ridge 회귀해 예측을 후보정
  (`down+fk_mm`). 캘리브는 방식당 한 번, no-fk/`+fk` 는 예측단계에서만 다름.

**실행**
```bash
PYTHONPATH= python CP_C1_unified_vs_independent.py \
    --root_folder data/session --intrinsics_dir intrinsics \
    --holdout_frac 0.3 --split_seed 0            # 또는 --test_sets "0,4,6,12"
# split 을 안 주면 전체 fit(다운스트림 NA) — 기존 동작.
```

**FK prior 회전 보정 (기본 활성):** `meta.json` 의 `set_cube_center_6dof` 는 위치는 맞지만
회전이 관측과 179.8° 어긋나 있다. 그대로 쓰면 base 프레임이 뒤집혀 모든 지표가 수백 mm 로
부풀려지므로, train set 관측에서 복원한 상수 회전으로 대체한다(`correct_fk_cube_rotation`).
`--fk_prior_rotation use` 로 옛 동작을 재현할 수 있고, `--robust_average false` 로 이상치
제거를 끌 수 있다. 자세한 내용은 아래 "결과를 읽기 전" 절.

**산출물** `CP_result/C1/` : `joint_ablation_summary.{csv,json}`, `fk_prior_rotation_fix.json`,
`<mode>/T_base_C*.npy`·`T_gripper_cam.npy`·`T_base_O_set*.npy`. 요약표 컬럼: consistency_t/r,
grip_t, cube_vs_fk, cost, **down_mm / down+fk_mm**.

> 실세션(13 set, train=9/test=4): `+fk` 보정이 held-out 오차를 11~13mm → **2.5~2.8mm** 로
> 낮춘다 — 시뮬의 C 보정효과를 재현.

---

## C2 — Board vs Cube  (`CP_C2_cube_vs_board.py`)

평면 ChArUco board only vs board + graspable marker cube. 한 번 캘리브(Step3 `--target both`)
결과의 `transform_sets = {board_only, cube_only, hybrid}` 를 읽어 동일 검증(Step5)으로 비교.

**정렬 추가 (시뮬 `observability` 짝):**
- **관측성 지표** (`C2_observability.csv`): meta.json 검출값에서 직접 계산.
  - board = ChArUco 코너 검출 수(`charuco_detect_n ≥ --min_board_corners`) 로 관측 판정
    (평면이라 마주본 카메라만).
  - cube = `cube_visible`/`cube_pnp.ok` (6면 마커라 어느 각도든).
  - **동시관측**(촬영당 평균 관측 카메라 수), **≥2대 동시(%)**(cross-camera 연결성),
    **시야각 coverage**(캘리브된 카메라 base 위치↔타깃 중심 관측방향 사이각).

**선행: 캘리브 1회**
```bash
python Step3_calibration.py --root_folder <sess> --intrinsics_dir <intr> \
    --out_dir <sess>/calib_out --target both
```
**실행**
```bash
PYTHONPATH= python CP_C2_cube_vs_board.py \
    --root_folder data/session --intrinsics_dir intrinsics \
    --calib_dir data/session/calib_out
```

**산출물** `CP_result/C2/` : `C2_cube_vs_board.{md,csv,json}` (mode 비교) + **`C2_observability.csv`**.

> 실세션 결과가 시뮬 핵심 주장을 재현: **cube 동시관측 2.70대(≥2대 89.7%) vs board 1.03대
> (2.9%)** — cube 6면이 카메라 간 연결을 강하게 만든다.

---

## C3 — gTc estimation: Camera / FK / Camera+FK  (`CP_C3_prior_vs_noprior.py`)

로봇 큐브중점(FK)을 solve 에 쓰느냐(prior)로 나뉘는 방법들을, 시뮬 Exp3 의 **3방식**에 맞춰 비교.

| 사용자 명명 | CP 방법 | 의미 |
|---|---|---|
| no-fk-prior | `without-prior` (03/04) | 큐브를 미지수로 vision 만으로 추정 (Camera-based) |
| fk-prior | `with-prior` (03/04) | 로봇 FK 큐브중점을 solve 에 강제(soft) (FK-based) |
| **fk-prior+후보정** | **`05_fk_prior_correction`** | fk-prior 예측을 train 잔차 Ridge 로 후보정 |

**정렬 추가:** 빠져 있던 세 번째 방식 **`05_fk_prior_correction`** 을 추가. with-prior(=fk-prior)
카메라로 train 큐브위치를 추정 → FK 대비 잔차를 `[1,x,y]` Ridge 회귀(W)로 배우고, held-out
test 큐브예측에만 후보정 적용(gTc·카메라는 불변). 기존의 held-out `test_prior_trans_rmse_mm`
(FK 위치오차) 로 세 방식을 공정 비교한다.

> 후보정 C = 방법 A(FK) + 위치의존 잔차 Ridge 보정 (프로젝트 `simulatioin_test.md` 의 정의).
> 시뮬 `exp3_gtc_estimation.py` 는 후보정을 Camera-based 위에 얹지만, 여기서는 사용자 지정에
> 따라 **fk-prior 위에** 얹었다.

**실행** (held-out split 을 켜야 05·test 지표가 나온다)
```bash
PYTHONPATH= python CP_C3_prior_vs_noprior.py \
    --root_folder data/session --intrinsics_dir intrinsics \
    --holdout_frac 0.3 --split_seed 0            # 또는 --test_sets "0,4,6,12"
#   (선택) --prior_weight_sweep 0,1,10,30,100    # prior 세기 sweep 곡선
#   (선택) --ridge_lambda 1e-3                    # 05 후보정 Ridge 세기
```

**산출물** `CP_result/C3/` : `ablation_summary.{csv,json}`, `<method>__<prior_mode>/`,
sweep 시 `prior_weight_sweep.csv`. 핵심 컬럼 `test_prior_trans_rmse_mm`.

---

## ⚠️ 결과를 읽기 전 — FK 큐브중점 prior 의 회전은 못 쓴다

`meta.json` 의 `set_cube_center_6dof` 는 큐브 **중심 위치**는 맞지만 **자세(회전)** 가 실제
큐브와 **179.8° 어긋나 있다** — 사실상 뒤집힘. C3 는 이 문제를
`initialize_base_translation_anchored` 에서 "위치만 앵커, 회전은 기각" 으로 처리해 왔고,
C1 에도 같은 진단을 넣되 기각에서 그치지 않고 **train set 관측에서 복원한 상수 회전으로
대체**한다 (`correct_fk_cube_rotation`, 산출물 `fk_prior_rotation_fix.json`).

이 보정 전에는 base 프레임이 통째로 뒤집혀 **C1 의 모든 수치가 한 자릿수 mm → 수백 mm 로
부풀려졌다.** 동일 관측(고정 286 / 그리퍼 75)에서의 A/B:

| C1 지표 | 보정 전 (`--fk_prior_rotation use`) | **보정 후 (기본값)** |
|---|---:|---:|
| independent consistency_t | 362.3 mm | **16.0 mm** |
| unified_joint consistency_t | 137.2 mm | **15.3 mm** |
| unified_joint cube_vs_fk | 120.3 mm | **4.9 mm** |
| independent held-out down | 405.2 mm | **13.4 mm** |
| unified_joint held-out down+fk | 6.6 mm | **2.5 mm** |

> 옛 문서에 있던 "면 조합별 계통오차(+X,+Y 14mm)", "cam1 이 구조적으로 불리", "2면 2마커
> 최소 기하가 한계" 라는 해석은 **전부 이 뒤집힌 앵커가 만든 착시**였다. 정상 앵커에서
> 카메라간 큐브중점 일치도는 held-out median **2.45mm (p90 6.1mm, 5mm 이내 84.8%)** 이고
> 면 조합별 차이도 1.3~2.8mm 로 사라진다. 큐브 마커 배치 모델도 문제없다 —
> `CP_cube_selfcal.py` 로 재보정해도 2.45 → 2.46mm 로 변화가 없다.

남은 오차원: 전체 관측의 **5.3%(22/418)** 가 ~138mm 로 튄다. 전부 cam0 / (+X,+Y) 2마커 /
set 11·12 에 몰려 있는 PnP flip 이며, `--robust_average`(기본 True)의 MAD 이상치 제거로
카메라 등록·큐브예측에서 걸러낸다. RMS 와 median 의 격차는 이 꼬리 때문이다.

> FK prior 의 **위치** 오차(median 6.6mm, `prior_diagnostics.json`)는 그대로 남아 있으므로,
> "FK 대비" 지표의 절대값 하한은 대략 그 수준이다. held-out down 이 11~13mm 인 것은
> 캘리브 오차와 FK prior 자체 오차가 겹친 값으로 읽어야 한다.

---

## 실측 결과 (data/session, 13 set, holdout_frac 0.3 → train 9 / test 4)

### C1 (`CP_result/C1`)
| method | consistency_t (mm) | consistency_r (°) | grip_t (mm) | cube_vs_fk (mm) | held-out down (mm) | **down+fk (mm)** |
|---|---:|---:|---:|---:|---:|---:|
| independent | 15.98 | 8.98 | 15.06 | 0.00 | 13.39 | **2.81** |
| unified_joint | **15.25** | **7.47** | **13.67** | 4.86 | 11.67 | **2.51** |
| joint_fk_fixed | 16.77 | 8.47 | 15.06 | 0.00 | **11.53** | 2.58 |

→ unified_joint 가 내부 서브시스템 정합에서 가장 낮은 값을 보이지만(회전 8.98→7.47°,
grip 15.06→13.67mm), 차이는 크지 않다. raw held-out 은 unified_joint 11.67mm와
joint_fk_fixed 11.53mm로 0.14mm 차이이므로 held-out 일반화 우위를 주장할 수 없다.
`+fk` Ridge 보정은 이 split에서 **FK proxy 대비 held-out RMSE**를 11.5~13.4mm에서
**2.5~2.8mm**로 낮췄다. 이는 독립 물리 GT 대비 절대 위치 정확도가 아니다.

`downstream_se3_trans_rmse_mm`와 `downstream_fk_trans_rmse_mm`의 코드 경로는 순차가 아니라
병렬이다. 동일 raw train 예측으로부터 `learn_fk_rigid`와 `learn_fk_ridge`를 각각 학습하고,
held-out raw 예측에 서로 독립 적용한다. 따라서 SE(3)→Ridge 추가 개선으로 해석하지 않고 두
개선율 모두 Raw 대비로 계산한다.

### C2 (`CP_result/C2`)
| mode | 등록 카메라 | cross-camera (mm) | pose_repeat (mm) |
|---|---|---|---|
| board_only | **2대** | **26.93** | 26.93 |
| cube_only | 3대 | 4.76 | 4.75 |
| hybrid | 3대 | **4.40** | 4.41 |

관측성: board 동시관측 **1.03대**(≥2대 **2.9%**) vs cube **2.70대**(**89.7%**), 시야각 118.7° vs 103.8°.
→ board 만으로는 카메라 1대가 등록 불가(2대만), cross-camera **26.93→4.76mm (-82%)**.
시뮬 Exp2 의 "큐브가 카메라 간 연결을 강화" 주장을 그대로 재현.

### C3 (`CP_result/C3`) — held-out FK 위치 RMSE (핵심 지표)
| 방법 | test_prior_t (mm) | (예전 로더) | 비고 |
|---|---:|---:|---|
| **fk-prior+후보정** (05) | **20.98** | 79.34 | **최저** — fk-prior 대비 -12.7%, vision-only 대비 -7.5% |
| no-fk-prior (03) | 22.68 | 77.42 | vision-only |
| fk-prior (03) | 24.02 | 86.90 | train prior_t=0.01mm 인데 held-out 은 vision-only 보다 나쁨 |

> **결론이 뒤집혔다.** 예전에는 Ridge 후보정(05)이 fk-prior 의 손해를 일부 회복하는 데
> 그쳤지만(79.3 > 77.4), `exclude_gripped` + `fixed_min_markers=2` 로 관측 품질이 올라간
> 지금은 **05 가 vision-only 를 앞선다**(20.98 < 22.68). 즉 FK 큐브중점 prior 는
> *solve 에 강제로 넣을* 정보로는 여전히 손해(24.02)지만, *예측을 사후 보정하는* 정보로는
> 이득이다.
>
> **여전한 사실:** fk-prior 는 train FK 에 거의 완벽히 맞지만(0.01mm) held-out 은 vision-only
> 보다 나쁘다 — FK 과적합. 다만 격차가 9.5mm → 1.3mm 로 줄었다.
> **주의:** 04(재투영 최적화)에서는 with/without prior 가 23.02 / 23.01mm 로 사실상 동일하게
> 수렴해 prior 차이가 사라진다 — 재투영 항이 prior 영향을 씻어낸다.
>
> C1 의 held-out(11.5~13.4mm)보다 큰 이유는 지표 정의가 달라서다: C3 는 **촬영(event) 단위**
> 큐브 pose 를 prior 와 비교하고 고정 카메라만 쓰는 반면, C1 은 **set 단위**로 평균한 예측을
> 쓰고 그리퍼 카메라도 포함한다.

### solve 방법 — PnP 기반인가, 재투영오차 기반인가?

C3 는 네 가지 solve 를 모두 돌려 `CP_result/C3/<method>__<prior>/T_base_C*.npy` 로 각각 저장한다:

| # | 방법 | 성격 | reproj RMSE (px) | reproj median (px) | held-out FK (mm) |
|---|---|---|---:|---:|---:|
| 01 | `pnp_mean` | PnP 평균 | 16.79 | 12.60 | 26.25 |
| 02 | `pnp_robust_se3` | PnP + robust SE(3) 평균 | 9.31 | 1.48 | **22.68** |
| 03 | `pose_consistency_opt` | PnP pose 일관성 최적화 | 9.31 | 1.48 | **22.68** |
| 04 | **`direct_reprojection_opt`** | **재투영오차 최소화** | **5.51** | **0.79** | 23.01 |

> (전부 without-prior. 03 은 이 데이터에서 optimizer 가 기각되어 02 결과로 폴백됨.)

**요점 1 — 헤드라인 C3 표(22.68)는 재투영 기반이 아니다.** 위 핵심표는 방법 **03**(=PnP robust
SE(3) 폴백)을 쓴다. 재투영 기반은 **04** 뿐이고, 픽셀 재투영은 04 가 압도적으로 좋다
(RMSE **5.51px**, 9.31→ **−41%**; median 1.48→**0.79px**, **−47%**).

**요점 2 — 그런데 held-out FK 지표는 방법과 거의 무관하다** (22.68 / 23.01, 편차 1.5%).
재투영 최적화가 픽셀 정합을 41% 개선해도 held-out FK 오차는 안 바뀐다. held-out FK 지표는
**FK 큐브중점 prior 자체의 위치오차**(median 6.6mm)와 파지·배치 반복성이 지배하므로, 픽셀
수준 정밀도로는 못 넘는다. **픽셀 정확도가 필요하면 04, FK 대비 지표가 목적이면 어느 쪽이든
같다.**

**요점 3 — 메인 파이프라인(Step3 → `data/session/calib_out`)의 고정카메라 `T_base_Ci` 도 PnP
기반이다.** `calibration_summary.json` 의 방법 표기:
`T_base_C0 = cube_anchor_primary+board_refined`, `T_base_C1 = board_based+board_refined`,
`T_base_C3 = cube_anchor_primary` — 즉 **큐브 PnP 앵커 + 보드 PnP 리파인**이고, 전체 재투영오차
번들조정(BA)이 아니다. **C1·C2 결과도 이 Step3 산출물에서 파생**된다.

> **정리:** 현재 "캘리브레이션 결과 데이터"는 (헤드라인 C3·메인 calib_out·C1·C2 모두) **PnP 기반**
> 이다. 재투영오차 기반 결과는 방법 **04** 로 이미 계산·저장돼 있고
> (`CP_result/C3/04_direct_reprojection_opt__without_robot_cube_prior/T_base_C*.npy`),
> 픽셀 재투영은 최고지만 이 데이터셋의 held-out FK 지표는 개선하지 못한다(규약 오프셋 지배).
> 재투영 결과를 헤드라인으로 쓰려면 위 04 경로의 transforms 를 채택하면 된다.

---

## 해석 — 무엇이 입증됐고 무엇이 아직인가

### C1 (통합 vs 독립)
1. **통합의 이점은 유지되지만 폭은 작다** — FK prior 회전 보정 후 consistency_t 는
   16.0(independent) vs 15.3mm(unified) 로 거의 붙는다. 차이가 남는 곳은 회전(8.98→7.47°)과
   그리퍼 정합(15.06→13.67mm). 옛 문서의 "427→144mm(−66%)" 는 **뒤집힌 앵커가 independent 를
   더 심하게 망가뜨린 결과**였고, 통합의 진짜 이점을 과장한 수치다.
2. **`+fk` 보정은 이 split의 FK proxy 정합에서 유효** — 11.5~13.4mm → 2.5~2.8mm.
   Ridge는 FK proxy 대비 위치 의존 residual을 포착한다. 이것만으로 물리 camera error가
   선형임을 입증하지는 않는다. 관측된 의존성은 camera calibration, robot FK, grasp
   repeatability가 결합된 결과일 수 있다.
3. **절대 물리 정확도는 측정하지 않았다** — 외부 ground truth가 없으므로 consistency
   15~17mm, grip_align 13.7~15.1mm와 held-out 결과는 모두 내부 정합 또는 FK proxy 대비
   지표로만 해석한다. (`cube_pos_err_vs_fk=0.0` 은 FK 를 그대로 쓴 정의상 결과라 정보 없음.)

논문용 실데이터 Figure는 `CP_viz_c1_fk_correction.py`가 세 장으로 생성한다:
`fig_CP_C1_fk_correction.png`, `fig_CP_C1_internal_metrics.png`,
`fig_CP_C1_C3_interpretation.png`. set별 x/y/z residual 원자료가 summary CSV/JSON에 없으므로
residual 수치 그래프, error bar, p-value, 분포는 생성하지 않는다.

### Figure 디자인 규약

모든 논문·검증 Figure는 `figure_style.py`의 `apply_paper_style()`, `clean_axis()`,
`save_figure()`를 사용한다. 기준 디자인은 위 C1 실데이터 3장으로, 흰 배경·산세리프 글꼴·
절제된 semantic palette·수평 grid·열린 상단/우측 spine·220 dpi를 공통 적용한다.
새 Figure도 `AGENTS.md`의 저장소 규약에 따라 같은 스타일을 사용해야 한다.

### C2 (board vs cube)
1. **큐브 기여가 정량 입증** — board 만으로는 **cam3 을 아예 등록 못 함**(2대만). 큐브를 넣으면
   3대 전부 등록되고 cross-camera **26.93→4.76mm(−82%)**, 회전 반복도 15.6°→1.9°(**−88%**).
2. **인과가 관측성으로 설명됨** — board 는 평면이라 촬영당 평균 1.03대만 보고 **≥2대 동시 관측이
   2.9%뿐**. 카메라 간 상대자세를 풀려면 2대 이상 동시 관측이 필수인데 그게 없으니 그래프가
   끊긴다. 큐브는 6면이라 89.7% 에서 ≥2대 → 촘촘히 연결.
3. **board 검출 품질 자체는 나쁘지 않다**(board_reproj 4.02px). 문제는 정밀도가 아니라
   **가시성·연결성**이다 — 시뮬 Exp2 가 말한 바로 그 메커니즘.

> 큐브는 "정확도를 조금 높이는" 게 아니라 **캘리브 가능 여부 자체를 결정**한다.

### C3 (fk-prior / no-fk-prior / fk-prior+후보정)
1. **fk-prior 는 명백한 과적합** — train 에서 FK 에 0.02mm 로 완벽히 맞지만 대가로 카메라가
   망가진다(재투영 17.5→72.6px, pose 일관성 15.7→80.5mm). held-out 은 오히려 악화(77.4→86.9).
   64mm 틀린 목표에 억지로 맞춘 결과.
2. **후보정은 실제로 작동** — 86.9→79.34mm(**−8.7%**). 다만 이 데이터셋에선 vision-only(77.4)를
   넘지 못했다.
3. **시뮬과 결론이 갈리는 이유** — 시뮬 Exp3 는 "FK 가 완벽하다"고 가정한다. 실제 FK 는 위
   규약 오프셋 탓에 틀려 있어 "FK 를 solve 에 강제"하는 이점이 사라지고 손해만 남는다.
   **시뮬 결론이 틀린 게 아니라 전제가 실데이터에서 깨진 것.**
4. **04(재투영)에서는 차이가 소멸** — with/without 모두 77.53mm 로 수렴. 강한 픽셀 항이 prior
   영향을 씻어낸다 → 재투영 항이 충분하면 prior 선택은 무의미해진다.

---

## 종합 결론

| 기여 | 입증 여부 | 근거 |
|---|---|---|
| **C2 큐브** | ✅ **강하게 입증** | 카메라 3대 등록 가능, cross-camera −82% |
| **C1 통합** | ✅ 방향성 입증 | consistency −66% |
| **C1/C3 `+fk` 보정** | ✅ 효과 확인 | C1 −91%, C3 −8.7% |
| **C3 FK prior 강제** | ❌ **역효과** | held-out +12% 악화 |

### 권고 (우선순위)
1. 🔴 **`meta.json` 의 `set_cube_center_6dof` 회전 규약을 촬영 단계에서 고칠 것** — 지금은
   C1/C3 가 런타임에 보정하고 있다(179.8° 뒤집힘). 촬영 코드에서 바로잡으면 이 우회가
   불필요해지고, 회전 prior 를 실제로 쓸 수 있게 된다.
   `CP_result/C3/corrected_set_cube_center_6dof.json` 에 관측에서 복원한 값이 있다.
2. 🟠 **PnP flip 이상치 제거** — 전체 관측의 5.3%(22/418)가 ~138mm 로 튄다. 전부 cam0 /
   (+X,+Y) 2마커 / set 11·12. `--robust_average`(기본 True)가 평균 단계에서 걸러내지만,
   근본적으로는 그 두 set 에서 cam0 시야가 축퇴되지 않도록 촬영 배치를 조정하는 게 낫다.
3. 🟡 **FK 큐브중점 위치오차(median 6.6mm) 원인 규명** — "FK 대비" 지표의 하한을 결정한다
   (파지 반복성·FK 정확도).
4. 🟢 **큐브 마커 배치 모델은 손댈 필요 없음** — `CP_cube_selfcal.py` 로 재보정해도 held-out
   일치도가 2.45→2.46mm 로 변하지 않는다. 큐브를 재인쇄했을 때만 다시 돌리면 된다.
5. 🟢 **C2 는 그대로 사용 가능** — FK 미사용이라 prior 품질과 무관하게 성립한다.

---

## 시각화 산출물 (figures) — 시뮬 ↔ 실데이터 비교

`CP_viz_sim_vs_real.py` 가 **재실행·재계산 없이** 이미 저장된 산출물
(`Simul_test/figures/*.json`, `CP_result/*/*.csv`)만 읽어 생성한다. 의존성은 `numpy matplotlib`
뿐(`cv2` 불필요). 각 기여도마다 세 종류를 낸다: 시뮬만, 실데이터(CP)만, 그리고 **한 이미지에
2행으로 나란히** 비교(상단=시뮬 GT 기준, 하단=CP FK 프록시).

```bash
PYTHONPATH= python CP_viz_sim_vs_real.py                 # 9장 figure + 요약표 전부
PYTHONPATH= python CP_viz_sim_vs_real.py --only C3       # C3 만 (sim·cp·both)
PYTHONPATH= python CP_viz_sim_vs_real.py --which both    # 비교본만
#   cv2 없는 환경이면 conda python 등 matplotlib 있는 인터프리터 사용.
```

**산출물** `CP_result/figures/` (9장) + `CP_result/SIM_vs_CP_summary.{csv,md}` :

| 기여 | 시뮬만 | 실데이터만 | 비교(한 이미지) | 대응 Simul_test 원본 |
|---|---|---|---|---|
| C1 | `fig_SIM_C1_unified_vs_indep.png` | `fig_CP_C1_unified_vs_indep.png` | `fig_SIMvsCP_C1_unified_vs_indep.png` | `fig_unified_vs_indep.png` |
| C2 | `fig_SIM_C2_board_vs_cube.png` | `fig_CP_C2_board_vs_cube.png` | `fig_SIMvsCP_C2_board_vs_cube.png` | `fig_exp2_board_vs_cube.png` |
| C3 | `fig_SIM_C3_gtc_estimation.png` | `fig_CP_C3_gtc_estimation.png` | `fig_SIMvsCP_C3_gtc_estimation.png` | `fig_exp3_noise_sweep.png` |

- **스케일 주의:** 시뮬은 GT 대비 절대오차(~1mm), CP 는 FK 프록시 대비(~100mm)라 **행마다 자체
  y축**을 쓴다. 직접 비교는 C3 의 상대변화(%) 패널과 요약표의 `Δ%`·`재현` 판정으로 한다.
- **측정 불가 항목 은폐 없음:** C1 `bTf`(고정카메라 절대오차)는 실데이터에서 잴 수 없어 빈칸 대신
  명시적 **N/A 패널**로 렌더링. C1 gTc→`grip_align_trans_rmse_mm`, C2 카메라오차→`cross_camera`,
  타깃예측→`pose_repeat` 프록시로 대체(각 patch·주석에 표기).
- **색·텍스처:** Simul_test 팔레트(적=board/indep, 청=cube/joint, 녹=hybrid/CP 전용) 유지.
  `+fk`/후보정 변형은 색만이 아니라 **hatch(`//`)** 로도 구분(색각·흑백 인쇄 대비 이중 인코딩).

### 요약표 `SIM_vs_CP_summary.md` — 재현 판정 (11지표)

`O` 방향·크기 재현 / `△` 방향만(효과 <1/4) / `X` 반대 또는 효과없음. 발췌:

| 기여 | 지표 | 시뮬 Δ% | CP Δ% | 재현 |
|---|---|---:|---:|:---:|
| C1 | shared-base consistency | −93.3% | −66.4% | **O** |
| C1 | `+fk` 보정 효과 | −79.4% | −91.4% | **O** |
| C1 | held-out 예측(보정 전) | −63.0% | −0.3% | X (효과없음) |
| C2 | 동시관측 카메라 수 | +51.9% | +162.8% | **O** |
| C2 | cross-camera / cam pose | −43.2% | −83.7% | **O** |
| C2 | 시야각 coverage | +11.3% | −12.5% | X (반대) |
| C3 | FK prior vs vision-only | +13.4% | +12.2% | **O** |

> 요약표가 드러낸, 본문 표에 없던 3가지: ① C1 held-out 은 `+fk` **보정 전**엔 통합 이점이
> downstream 에 안 나타남(329.2 vs 328.4). ② C2 **시야각은 부호가 반대**(실데이터는 board 118.7°
> > cube 103.8°) — 시뮬 주장과 어긋나 figure 에 주석 표시. ③ C3 FK-prior 악화율이 시뮬(+13.4%)과
> 실데이터(+12.2%)에서 거의 정확히 일치 — 과적합 메커니즘이 잘 재현됨.

---

## 명명·규약

- 변환은 "목적지-from-출발지": `T_A_C = T_A_B @ T_B_C`. `bTg`=base←gripper(FK), `gTc`=
  gripper←camera(핸드아이), `T_base_Ci`=base←고정카메라, cube[set]=base←큐브.
- **Joint / bundle adjustment** = 미지수 동시 최적화(추정, 학습 아님).
- **`+fk` / FK-correction** = 캘리브 후 남은 위치의존 잔차를 **Ridge 회귀**로 배워 최종 예측을
  후보정 (supervision = 로봇 FK 큐브중점). 시뮬의 "C" 방법과 동일.
- **FK 프록시**: 실데이터에는 GT 가 없으므로 로봇 FK 큐브중점을 정답 대용으로 held-out 예측
  오차·consistency 를 측정. 시뮬은 이상적 상한, 실데이터는 현실값.

의존: `numpy scipy opencv-contrib-python`. 실행 시 `PYTHONPATH=` (시스템 ROS pytest 플러그인
충돌 회피), 프로젝트 루트에서.
