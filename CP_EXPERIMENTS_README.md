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

**산출물** `CP_result/C1/` : `joint_ablation_summary.{csv,json}`, `<mode>/T_base_C*.npy`·
`T_gripper_cam.npy`·`T_base_O_set*.npy`. 요약표 컬럼: consistency_t/r, grip_t, cube_vs_fk,
cost, **down_mm / down+fk_mm**.

> 실세션(13 set, train=9/test=4) 예: `+fk` 보정이 held-out 오차를 크게 낮춤
> (independent 332→60mm, unified 265→59mm) — 시뮬의 C 보정효과를 재현.

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

| 시뮬 Exp3 | CP 방법 | 의미 |
|---|---|---|
| ① Camera-based | `without-prior` (03/04) | 큐브를 미지수로 vision 만으로 추정 |
| ② FK-based | `with-prior` (03/04) | 로봇 FK 큐브중점을 solve 에 강제(soft) |
| ③ Camera+FK후보정 | **`05_camera_fk_correction`** | ①의 예측을 train 잔차 Ridge 로 후보정 |

**정렬 추가:** 시뮬 Exp3 에서 빠져 있던 세 번째 방식 **`05_camera_fk_correction`** 을 추가.
without-prior(=Camera-based) 카메라로 train 큐브위치를 추정 → FK 대비 잔차를 `[1,x,y]` Ridge
회귀(W)로 배우고, held-out test 큐브예측에만 후보정 적용(gTc·카메라는 불변). 기존의 held-out
`test_prior_trans_rmse_mm`(FK 위치오차) 로 세 방식을 공정 비교한다.

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
