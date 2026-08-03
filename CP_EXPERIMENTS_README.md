# CP 기여도 실험 (C1 / C2 / C3) — 실데이터 정리

`Simul_test/` 의 순수 SE(3) 기하 **시뮬레이션** 으로 검증한 세 가지 설계 선택을, 실제 촬영
세션의 관측으로 **동일하게** 재현·측정하는 코드다. 시뮬은 GT(정답 카메라·핸드아이·큐브)를
알기에 정답 대비 오차를 직접 재지만, 실데이터에는 GT 가 없다. 그래서 GT 자리에 **로봇 FK
큐브중점을 정답 프록시**로 쓴다(held-out 예측 오차·consistency 로 대체).

각 CP 실험은 C1/C2/C3 서로 독립 실행되며, 공유 로더/기하/지표는 `CP_common.py` 에 있다.

> **논문 핵심표의 source of truth는
> [`CP_result/Calibration_Experiment_table.md`](CP_result/Calibration_Experiment_table.md)다.** 아래의 기존
> C1/C2/C3는 서로 다른 historical/component 실험이므로, canonical A0–B3의 통제된 인과 비교나
> 외부-GT accuracy로 재해석하지 않는다. 특히 C1의 FK 후보정과 C2의 marker-source 결과는
> Main Ablation 행에 섞지 않는다.

---

## 시뮬 ↔ 실데이터 대응

| # | 기여 | 시뮬 (기준) | CP 실데이터 | 핵심 지표(FK 프록시) |
|---|---|---|---|---|
| **C1** | Unified vs Independent | `unified_vs_independent.py` | `CP_C1_unified_vs_independent.py` | held-out 큐브예측(mm) + `+fk` 보정, consistency |
| **C2** | Board vs Cube | `exp2_board_vs_cube.py` | `CP_C2_cube_vs_board.py` | 관측성(동시관측·시야각) + cross-camera/재투영 |
| **C3** | gTc estimation (Camera / FK / Camera+FK) | `exp3_gtc_estimation.py` | `CP_C3_prior_vs_noprior.py` | held-out FK 위치오차(mm) 3방식 비교 |
| **D1** | FK 고정 × 잔차 보정 2×2 | 시뮬 대응 없음 | `CP_D1_fk_correction_2x2.py` | held-out **위치**(set 단위 LOO) 큐브중점 예측(mm) |

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

기본값 `--joint_solve fk_fixed`는 단계식 해를 초기값으로 만든 뒤 STEP-E에서 기존
pose-SE(3) residual로 `{T_base_Ci, T_gripper_cam, T_base_board}`를 공동 최적화한다.
cube pose는 object frame에 정렬된 set별 FK pose로 고정된다. 검증 중인 canonical
per-corner pixel objective는 아래처럼 **명시적으로 opt-in**한다. 동치성·수렴 검증이
끝날 때까지 production 기본값은 바꾸지 않는다.

```bash
python3 Step3_calibration.py \
  --root_folder data/session \
  --intrinsics_dir intrinsics \
  --out_dir <isolated-output> \
  --target both \
  --joint_solve reprojection_fk_fixed
```

두 진입점은 [`calibration_reprojection_backend.py`](calibration_reprojection_backend.py)의
동일 residual, freeze mask, local SE(3) retraction, Jacobian, smooth robust
`soft_l1(f_scale=2 px)`, SciPy `x_scale='jac'`를
사용한다. [`calibration_corner_observations.py`](calibration_corner_observations.py)가
corner 관측 생성도 공유한다. 이전 단계식 동작은 `--joint_solve off`를 사용한다. 실제
채택 여부, 종료 status, optimality, 관측 수와 전후 direct consistency는
`calibration_summary.json > diagnostics > joint_optimization`에 기록된다.

**실행**
```bash
PYTHONPATH= python CP_C2_cube_vs_board.py \
    --root_folder data/session --intrinsics_dir intrinsics \
    --calib_dir data/session/calib_out
```

**산출물** `CP_result/C2/` : `C2_cube_vs_board.{md,csv,json}` (mode 비교) + **`C2_observability.csv`**.

> 실세션 결과가 시뮬 핵심 주장을 재현: **cube 동시관측 2.70대(≥2대 89.7%) vs board 1.03대
> (2.9%)** — cube 6면이 카메라 간 연결을 강하게 만든다.

### Main ablation pose-source schema

모든 A0–B3 조건은 손목 카메라 뷰를 정합하기 위한 운동학 백본
`T_base_gripper=FK(q)`를 공통으로 사용한다. 실험 축은 백본 FK 사용 여부가 아니라
target pose source이며, [`CP_ablation_schema.py`](CP_ablation_schema.py)에 7행 정의와
검증 규칙을 고정했다. 외부 고정 board에는 robot FK pose가 없으므로
`FK→board=FK-fixed`인 조건은 실행 전에 오류로 반려된다.

같은 스키마에는 seq 행의 stage별 자유변수/freeze 목록, B1·A3·B2가 공유해야 하는
단일 **board-free FK artifact**, event-stratified held-out reprojection 평가, noise-free
`A1=A2`/`B1=A3` sanity gate도 포함된다. Train reprojection은 최적화 진단값으로만
보고하고 method 순위의 1차 지표로 사용하지 않는다. 상세 계약은
[`CP_result/Calibration_Experiment_table.md`](CP_result/Calibration_Experiment_table.md)에 있다.

FK-fixed 행의 artifact는 board hand-eye보다 먼저, raw `set_cube_center_6dof`와 train eih
cube corners만으로 `T_gripper_cam`과 FK-cube→tag-object delta를 공동 추정한다.
B1/A3/B2는 동일 SHA와 fixed cube matrix를 사용하며 B2의 전처리에도 board가 들어가지 않는다.

```bash
python3 CP_build_board_free_fk_artifact.py \
  --root_folder data/session \
  --intrinsics_dir intrinsics \
  --out_dir CP_result/fk_cube_artifact
```

실데이터 canonical artifact는 108 observations/1148 corners, Jacobian 12/12 full rank,
condition 38.11이며 3개 초기값 모두 수렴했다. 기존 board-derived delta와는
9.148 mm/0.540° 차이다. canonical 파일과 board-derived 비교 파일은
[`CP_result/fk_cube_artifact`](CP_result/fk_cube_artifact/)에 분리되어 있고 비교 파일은
어떤 ablation 행도 소비하지 않는다.

```bash
python3 CP_ablation_7row.py \
  --root_folder data/session \
  --intrinsics_dir intrinsics \
  --calib_dir data/session/calib_out \
  --out_dir CP_result/ablation_7row_canonical \
  --num_inits 3 \
  --max_nfev 300 \
  --tol 1e-8
```

실행 전 noise-free synthetic gate를 강제로 통과한다. Canonical backend의 Table 1 숫자는
비차원화·종료조건을 train/synthetic 진단으로 먼저 고정했다. board-free artifact를 적용한
단일-seed 7행은 7/7, B1/A3/B2 3-seed는 9/9
수렴했다. 이후 모델 독립 path mask를 반영한 canonical 7행×3 seeds도 21/21 수렴했다.
산출물은 [`CP_result/ablation_7row_canonical`](CP_result/ablation_7row_canonical/)에 있다.

path mask는 held-out measurement만으로 fitting 전에 고정되며 SHA-256
`8122b1f886a40ab94fb5cb223c12e3926f43148841a3f28f62343eecf49b76a3`다. 모든 cube 행이
동일한 113 observations, 81 `e_cross` pairs, 29 `e_e2e` units를 평가하며 예측값에 따른
30 mm/10° 제거는 없다. 보고서는 marker별·카메라별 reprojection도 분리한다. A0/B3의 path
metric은 cube가 없으므로 N/A다. 현재 3-seed `±`는 initialization 분산이며 데이터
불확실성이 아니므로, 여러 event split/seed 반복 전에는 fixed-split 결과로 표기한다.

외부 GT가 없는 위치 hold-out 값은 `e_task_pose^{FK-proxy}`로만 부르고 absolute physical
accuracy로 표현하지 않는다.

반복 split 최종 runner는 다음과 같다.

```bash
python3 CP_ablation_multisplit.py \
  --out_dir CP_result/ablation_multisplit \
  --split_seeds 20260729,20260730,20260731,20260732,20260733 \
  --num_inits 5 --max_nfev 300 --tol 1e-8
```

5 splits×7 rows×5 initializations의 175/175 run이 수렴했다. 집계는
[`multisplit_ablation.md`](CP_result/ablation_multisplit/multisplit_ablation.md)에 있으며,
split mean의 표준편차와 split 내부 initialization 표준편차를 분리한다. paired contrast 결과는
A1→A2 overall `−0.1486±0.0216 px`(5/5 개선), B1→A3 `−0.00869±0.00220 px`
(5/5이나 효과 작음), A2→A3 `+0.1155±0.0253 px`(0/5 개선)다. B2→A3도 공통
cube reprojection/e_e2e/e_cross에서 모두 0/5 개선이다. 따라서 현재 실데이터는
estimated-pose Unified 효과만 지지하며 FK-fixed 및 board 추가의 정확도 이득은 지지하지 않는다.

### Canonical custom-GT synthetic sweep

```bash
python3 CP_synthetic_7row.py \
  --out_dir CP_result/synthetic_7row \
  --pixel_sigmas 0,0.5,1.0,2.0 --trials 10 \
  --fk_noise_specs 0:0,1:0.1,3:0.3,5:0.5 \
  --fk_sweep_pixel_sigma 1.0
```

이 결과는 **METRIC이 아니라** A0–B3 전체를 구현할 수 있는 pixel-level custom GT다.
포함된 METRIC `medium_workcell`은 eye-on-base checkerboard-only라 cube/eih/FK→cube 축을
만들 수 없다. headline은 모든 행에 공통인 `T_base_Ci`와 `T_gripper_cam`의 GT RMS다.

noise-free에서는 7행 모두 수치 오차 안에서 GT를 복원했다. A2 strict estimated-cube는
σ=0.5/1/2 px에서 6/10, 3/10, 0/10만 수렴하여 미수렴 조건의 GT 숫자를 표에서 제거했다.
A3는 모든 σ에서 10/10 수렴했다. B2→A3 board 이득은 pixel noise와 함께 커졌지만 실제
5-split 데이터에서는 0/5 개선이므로 실제 setup으로 일반화할 수 없다. pixel σ=1 px에서
FK cube-pose noise를 0→5 mm/0→0.5°로 늘리면 A3 translation GT error가
4.18→44.11 mm로 증가했다. 상세 표와 raw diagnostics는
[`CP_result/synthetic_7row`](CP_result/synthetic_7row/)에 있다.

### METRIC board-only external-GT baseline

```bash
python3 CP_metric_board_only.py \
  --dataset_dir '[]Multi-Camera-Hand-Eye-Calibration-main/data/medium_workcell' \
  --out_dir CP_result/metric_board_only
```

METRIC `medium_workcell`은 4-camera eye-on-base checkerboard 자료이므로 7행 factorial에
넣지 않는다. 별도 runner는 Tsai-Lenz/Park-Martin/Horaud/Daniilidis와 공유
`T_gripper_board`를 갖는 joint corner reprojection 호환 재구현을 번들 `T_base_cam` GT로
평가한다. Joint 결과는 4-camera RMS `0.9474 mm/0.0626°`, train coordinate RMSE
`0.2407 px`, Jacobian 30/30 full-rank다. 원본 Allegro C++/Ceres 바이너리는 현재 환경에
`cmake`가 없어 실행하지 않았으므로 이 값을 원본 바이너리 결과로 부르지 않는다. 상세
per-camera 값과 provenance는 [`CP_result/metric_board_only`](CP_result/metric_board_only/)에 있다.

### Table 4 real-data sensitivity

```bash
python3 CP_sensitivity_7row.py \
  --root_folder data/session \
  --intrinsics_dir intrinsics \
  --out_dir CP_result/sensitivity_7row
```

물리 task 실험과 분리된 추가수집 없는 subsampling runner다. 5개 고정 위치
`[0,2,6,9,12]`에 전체 train event `N=5/10/20/40`을 균등 배분하고, 같은 seed에서 작은
subset이 큰 subset에 포함되도록 했다. `cams=2/3/4`는 eih 한 대를 포함하며 고정 카메라는
`[0]⊂[0,1]⊂[0,1,3]`으로 추가한다. 해상도 조건은 저장 영상을 0.5×로 줄여 검출을 다시
수행한 것과 native 검출의 비교이며 센서 재촬영 결과가 아니다. 검출 corner는 native pixel
frame으로 되돌리고, 모든 조건을 같은 native held-out event와 `eih+cam0`에서 평가한다.

5개 subset seed에서 A2는 N=5의 1개 seed가 `max_nfev=300`에 도달해 headline 값을
제거했다. A0는 0.5×에서 고정 카메라 ChArUco 검출이 없어 0/5 모두 등록 불가였다. 핵심
B2→A3 공통-cube paired Δ(A3−B2)는 N=5에서 `−0.1558±0.1654 px`(4/5 개선)였지만,
N=10/20/40에서는 각각 `+0.0305/+0.0307/+0.0373 px`로 board 이득이 재현되지 않았다.
따라서 현재 실데이터로 강한 저N board 이득을 주장하지 않는다. 표와 raw provenance는
[`CP_result/sensitivity_7row`](CP_result/sensitivity_7row/)에 있다.

### Fixed-camera solver 01–04 controlled comparison

```bash
python3 CP_solver_01_04.py \
  --root_folder data/session \
  --intrinsics_dir intrinsics \
  --test_sets 0,4,6,12 \
  --out_dir CP_result/solver_01_04
```

01 mean, 02 robust SE(3), 03 pose consistency, 04 direct corner reprojection을 같은 train/test
cube 관측으로 강제 실행한다. 03과 04는 동일한 02 초기값 SHA를 공유하고, 04의 pose
regularizer는 0이며 production fallback은 비활성화된다. 1차 지표는 held-out 한 카메라의
PnP pose를 다른 카메라 corner로 전이하는 measurement-only cross-view RMSE다.

01/02/03/04의 held-out transfer는 각각 `37.9321/29.8026/29.8609/29.4867 px`였다.
04의 train reprojection은 02의 `9.3070`에서 `5.5139 px`로 크게 낮지만 held-out 개선은
`0.3159 px`에 그쳤고, 03은 02보다 소폭 나빴다. 따라서 train loss만으로 solver 우위를
주장하지 않는다. 이 실세션에는 외부 GT가 없으므로 수치는 absolute accuracy가 아니라
multi-view agreement다. 상세 결과는
[`CP_result/solver_01_04`](CP_result/solver_01_04/)에 있다.

### SOTA protocol audit

SOTA 표는 서로 다른 논문의 mm/°를 `e_task_pose` 하나로 합치지 않는다. strict
`learning-free`(pretrained model도 없음), `multi-cam`, 고정+손목 카메라가 한 objective에
들어가는 `joint eye-in+to`를 별도 검증하고, 각 숫자에 GT와 metric 정의를 붙인다.

- Tabb & Yousef는 학습 없는 multi-camera robot-world hand-eye지만 원 논문 확장은 같은 로봇에
  장착한 multiple-eye이며, mixed eye-in+eye-to joint 방법은 아니다.
- Allegro et al.은 여러 fixed camera와 robot-mounted board를 푸는 learning-free 방법이다.
  METRIC synthetic medium 보고값은 `0.75 mm/0.02°`, `13.78 s`다.
- EasyHeC++의 `1.35 mm/0.045°`는 5-view synthetic eye-to-hand camera-pose GT다. SAM 및
  pretrained matcher를 사용하므로 strict learning-free는 아니다.
- Li et al.의 `0.930 mm/0.265°`는 단일 structured-light camera에서 여섯 configuration과
  세 재장착 group 사이의 평균 표준편차다. external-GT accuracy나 multi-camera 결과가 아니다.
- 본 A3 real 결과는 외부 transform GT가 없으므로 `4.6230±0.4338 px` held-out corner RMSE만
  보고하며, 외부 논문의 mm/°와 순위를 만들지 않는다.

원문 링크와 전체 표·runtime/view 정의는
[`CP_result/Calibration_Experiment_table.md`](CP_result/Calibration_Experiment_table.md)의 Table 6에 있다.

### A2 strict-none vs A3 FK-fixed — historical 진단

```bash
python3 CP_A2_strict_none.py \
  --root_folder data/session \
  --intrinsics_dir intrinsics \
  --calib_dir data/session/calib_out \
  --test_sets 0,4,6,12 \
  --num_inits 5 \
  --max_nfev 300 \
  --out_dir CP_result/A2_strict_none
```

A2는 set별 cube pose를 visual-only 초기값에서 자유변수로 풀며 FK residual과
post-correction을 전혀 사용하지 않는다. A3는 동일 관측·optimizer에서 cube pose만
aligned FK에 고정한다. 다만 이 독립 script는 canonical backend 추출 전 historical
diagnostic이며, 핵심표의 A2/A3는 이제 7행 runner의 공통 backend 결과만 사용한다.
종료조건 미도달 시 단일 오차값을 핵심표 값으로 채택하지 않는다.

### D1 — FK 고정 × 잔차 보정 2×2 (`CP_D1_fk_correction_2x2.py`)

"cube pose를 FK로 고정하고 예측 단계에서 잔차를 보정하는 구성이 최선인가"를 **동일
backend·동일 solver 설정·동일 예측 mask**에서 직접 측정한다. 그 전까지 A3(7행 runner)와
Ridge 후보정(C1)은 backend·target set·split·지표가 모두 달라 조합의 우열을 말할 수 없었다.

```bash
RB_ROBOT_POS_SCALE=<k> PYTHONPATH= python3 CP_D1_fk_correction_2x2.py \
    --root_folder data/session --intrinsics_dir intrinsics \
    --calib_dir data/session/calib_out \
    --out_dir CP_result/D1_fk_correction_2x2
#   (선택) --folds 0,4          # 스모크용 부분 fold
#   (선택) --ridge_lambda 1e-3  # 보정 Ridge 세기
```

**Split은 위치(set) 단위 leave-one-out 13 fold다.** 잔차 보정은 공간적 일반화 주장이므로
모든 위치가 train 에 들어가는 이벤트 split 으로는 검증할 수 없다. 이 split 의 역할은
`CP_ablation_schema.POSITION_HOLDOUT_ROLE` 에 따라 **`FK-proxy` 전용**이다. Held-out 위치의
캡처 이벤트는 board 관측까지 양쪽 arm 에서 동일하게 제외하고, FK artifact 도 fold 마다 train
위치만으로 재추정한다(이벤트 누수는 assert 로 차단).

Arm 은 canonical 스키마의 A2/A3 그대로이며 차이는 `T_base_cube_by_set` 가 자유변수인지
(A2 vision-estimated) 학습셋 전용 FK artifact 에 고정되는지(A3 FK-fixed) 하나뿐이다. 보정은
`none / offset(3) / SE(3)(6) / Ridge[1,x,y](9)` 네 단계이며 **train 위치에서만 학습해 held-out
위치에만 적용**한다. 보정은 `T_base_Ci`·`T_gripper_cam` 을 바꾸지 않으므로 통제 지표(FK pose
재투영 px, `e_cross` mm)는 보정 유무와 무관하게 동일하다 — **어떤 보정 결과도 재투영 개선으로
보고할 수 없다.**

**산출물** `CP_result/D1_fk_correction_2x2/` : `D1_fk_correction_2x2.{md,json,csv}`.
결과·판정·한계는 핵심표 Table 1 안의
["A2/A3 위치 hold-out 재검증 — D1 2×2"](CP_result/Calibration_Experiment_table.md)
절에 있다. 요약하면 잔차 보정은 두 arm
모두에서 실재하는 효과(A2 −1.587 mm, t=−4.59 / A3 −0.824 mm, t=−2.46)이고, FK 고정 여부는
13개 위치로는 판정되지 않는다(모든 A3−A2 대비 `|t| ≤ 1.05`).

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

> **단일 split 에서는** `exclude_gripped` + `fixed_min_markers=2` 로 관측 품질이 올라가
> 05(20.98)가 vision-only(22.68)를 앞섰다. 하지만 이 순위는 **아래 LOSO 반복 검증에서
> 확인되지 않았다** — 아래 "⚠️ 이 순위는 반복 검증되지 않았다" 참고. 즉 이 표의 순위는
> **이 하나의 split 에 한정**해서 읽어야 한다.
>
> **반복 검증에서도 견고한 사실:** fk-prior(FK 를 solve 에 강제)는 train FK 에 거의 완벽히
> 맞지만(0.01mm) held-out 은 vision-only 보다 나쁘다 — FK 과적합. "FK 를 solve 에 강제하지
> 마라"는 방향은 유지된다. 논쟁적인 것은 **후보정(05)이 vision-only 를 이기느냐**이다.

#### ⚠️ 이 순위는 반복 검증되지 않았다 (LOSO, `CP_result/validate_loso`)

set 을 하나씩 빼는 leave-one-set-out(13 fold, closed-form) 로 다시 재면, **후보정이
vision-only 를 유의하게 이기지 못한다**:

| 방법 (closed-form LOSO) | mean ± std | median | range |
|---|---:|---:|---:|
| no-fk-prior | 12.8 ± 11.6 | 11.2 | 2.9 – 45.6 |
| + Ridge 후보정 | 12.8 ± 11.2 | **7.8** | 1.9 – 43.0 |

- median 으로는 개선(11.2→7.8)하지만 **mean 은 동일**(set 12 = 45mm 이상치가 지배).
- **fold 별로 절반은 개선, 절반은 악화** — train 12개 set 의 `[1,x,y]` 선형 fit 이 일부 held-out
  위치로 잘못 외삽한다. fold 편차가 매우 크다(2.9~45.6mm).
- 따라서 위 단일-split 의 "05 < no-fk-prior(20.98 < 22.68)" 는 **일반화 주장이 아니다.**

> 주의: LOSO 는 조인트 solve(fold 당 ~100s, 13× 반복 비현실적)를 뺀 **closed-form 프록시**라
> 절대값(12.8mm)이 위 단일-split 표(22.68mm)와 다르다 — 추정기가 다르다. LOSO 는 "후보정의
> 이점이 split 에 따라 뒤집힌다"는 **안정성** 질문에만 답한다. (반면 C1 의 held-out 는 같은
> LOSO 에서 보정 후 2.5±1mm 로 견고하게 재현된다 — C1 절 참고.)
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
2. **`+fk`/`+se3` 보정은 LOSO 반복 검증에서도 견고** — 단일 split 11.5~13.4mm → 2.5~2.8mm 를
   넘어, leave-one-set-out(13 fold, C1 실제 3솔버)에서 **+SE(3) 4.5±2mm, +Ridge 2.5±1mm 로
   전 fold 재현**된다(`CP_result/validate_loso/fig_CP_C1_loso.png`). raw 는 fold 간 ±5~7mm 로
   흔들리지만 보정 후에는 안정적이다 — 즉 이 값은 우연한 split 이 아니다. (반면 C3 의 후보정
   이점은 같은 LOSO 에서 재현되지 않는다 — C3 절 참고.)
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
1. **fk-prior 는 명백한 과적합 (반복 검증에서도 견고)** — train FK 에 거의 완벽히 맞지만
   (0.01mm) held-out 은 vision-only 보다 나쁘다(단일 split 24.02 vs 22.68mm). "FK 를 solve 에
   강제하지 마라"는 방향은 확실하다.
2. **후보정의 이점은 반복 검증되지 않았다** — 단일 split 에서는 05 가 vision-only 를 앞섰지만
   (20.98 < 22.68), LOSO(13 fold, closed-form)에서는 median 만 개선(11.2→7.8)하고 mean 은
   동일하며 **fold 별로 절반은 악화**한다. 즉 "후보정이 vision-only 를 이긴다"는 **일반화
   주장으로 쓸 수 없다.** (위 C3 결과 절의 LOSO 표 참고.)
3. **시뮬과 결론이 갈리는 이유** — 시뮬 Exp3 는 "FK 가 완벽하다"고 가정한다. 실제 FK 큐브중점은
   위치오차(median 6.6mm)가 있어 "FK 를 solve 에 강제"하는 이점이 사라지고 손해만 남는다.
   **시뮬 결론이 틀린 게 아니라 전제가 실데이터에서 깨진 것.**
4. **04(재투영)에서는 prior 차이가 소멸** — with/without 모두 23.0mm 로 수렴하고, 픽셀 재투영은
   04 가 압도적(5.51px)이지만 held-out FK 지표는 방법과 무관하다(22.68/23.01). FK 대비 지표는
   FK prior 자체 오차·파지 반복성이 지배하므로 픽셀 정밀도로는 못 넘는다.

---

## 종합 결론

| 기여 | 입증 여부 | 근거 |
|---|---|---|
| **C2 큐브** | ✅ **강하게 입증** | 카메라 3대 등록 가능, cross-camera −82% (FK 무관) |
| **C1 통합** | 🔸 방향성만 | 보정 후 구조 간 차이 작음(15~17mm), std 안에 묻힘 |
| **C1 `+fk`/`+se3` 보정** | ✅ **LOSO 재현** | held-out +SE(3) 4.5±2mm, +Ridge 2.5±1mm (13 fold) |
| **C3 후보정 이점** | ⚠️ **미확인** | 단일 split 만 우세, LOSO 에서 fold 절반 악화 |
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
