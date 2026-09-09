# Zeus 캘리브레이션 방식 비교 (2026-09-09 재촬영 데이터)

> late_table1(CP_result/session04, UR3)과 같은 취지로, Zeus 데이터에 대해
> "통합(unified) vs 독립(independent) x FK 처리 방식"을 비교한 결과다.
> table1.py의 정식 held-out split/공유 baseline 절차 그대로는 아니고(train-pooled
> + leave-one-out으로 근사). 코드: `fit_calibration_methods.py`,
> `eval_heldout_and_consistency.py`, `gt_pick_test.py`/`gt_compare_fits.py`(외부 GT).
>
> **2026-09-09 오후 session1/session3를 재촬영**하고 그 데이터로 전부 다시
> 계산했다 (`pass1_grasp_offset_replayed.json`도 재fit). 이 문서의 숫자는 전부
> 재촬영 데이터 기준이다.

**"독립"의 정의**: 고정캠 그룹(session1 + session2-고정캠 + session3-고정캠)과
그리퍼 그룹(session2-그리퍼캠 + session3-그리퍼캠)을 **서로 정보 교환 전혀
없이** 완전히 따로 캘리브레이션한다 — 각자 로봇 FK로 base 좌표계에 연결해서
캘리브레이션했을 방식 그대로. session2 큐브는 **양쪽 그룹 다 학습에 쓰되,
큐브 pose 변수를 서로 공유하지 않는다**(고정캠 그룹은 자기 큐브 pose 변수를,
그리퍼 그룹은 별개의 자기 큐브 pose 변수를 각자 추정). 다 끝난 뒤 "두 그룹이
같은 session2 큐브에 대해 각자 추정한 pose가 서로 얼마나 일치하는가"를 사후
합의(consensus)로 확인한다. (참고: table1.py의 공식
sequential_frozen_stage(A1, 그리퍼가 먼저 정하고 고정캠이 일방적으로 맞춤)는
이 정의와 안 맞아서 제거했다 — "서로 정보 교환 없음"이 아니라 한쪽이 다른
쪽에 일방적으로 맞추는 비대칭 구조이기 때문.)

## Table 1 전체 결과 (late_table1/Session04와 같은 컬럼 구조)

> ALL Cube RMSE는 late_table1엔 있지만 Zeus 쪽엔 아직 안 만든 지표라 N/A. External GT는 late_table1의 TRE와 같은 개념으로 xyz 3D 오차(mm)와 rz 오차(deg)를 씀(n=3, 아래 "결과 2" 참고).

| 방법 | Calibration train target | Optimization | FK/target-pose 처리 | Train RMSE px | ALL Cube RMSE px | Heldout Cube RMSE px | Cross-view Cube px | Cam-common Cube mm/deg | External GT xyz TRE / rz (mm/deg, n=3) |
|---|---|---|---|---:|---:|---:|---:|---:|---:|
| 통합_no-fk | board+cube | unified_joint_optimization | cube pose=estimated | 0.7121 | N/A | 2.9557 | 3.9234 | 4.2250 / 1.0534 | 2.96 / 0.69 |
| 통합_raw-fk | board+cube | unified_joint_optimization | cube pose=raw-FK-fixed | 0.8850 | N/A | **2.2087** | 4.2813 | 4.6416 / 1.1567 | **2.65** / 0.65 |
| 독립_no-fk | board+cube (그룹 분리) | independent_parallel (핸드오프 없음) | cube pose=estimated | 0.68 / 0.65 | N/A | 3.8053 | **3.6668** | **3.9182** / 1.1376 | 3.08 / **0.63** |

(독립의 Train은 고정캠/그리퍼캠 두 그룹을 완전히 따로 풀기 때문에 "고정캠값/그리퍼캠값"으로 표기. Heldout px/Cross-view px/Cam-common은 `eval_heldout_and_consistency.py`로 계산. Cross-view Cube px = late_table1과 같은 정의로, 한 카메라의 단일 이미지 PnP pose를 캘리브레이션된 extrinsics로 다른 카메라로 옮겨 재투영했을 때의 코너 px RMSE — 고정캠-고정캠 + 그리퍼캠-고정캠 쌍, 양방향, 세트당 4대 → 81쌍/162방향.)

Cross-view px / Cam-common을 **held-out 방식**(각 fold에서 빠진 세트에 대해서만 재고 15 fold pooled, late_table1이 held-out 세트에서 재는 것과 같은 구조)으로도 계산했다. 위 표의 train-pooled 값과 거의 같다:

| 방법 | Cross-view px (train-pooled) | Cross-view px (**held-out**) | Cam-common mm/deg (train-pooled) | Cam-common mm/deg (**held-out**) |
|---|---:|---:|---:|---:|
| 통합_no-fk | 3.9234 | 3.9473 | 4.2250 / 1.0534 | 4.2473 / 1.0570 |
| 통합_raw-fk | 4.2813 | 4.2703 | 4.6416 / 1.1567 | 4.6214 / 1.1619 |
| 독립_no-fk | 3.6668 | **3.6836** | 3.9182 / 1.1376 | **3.9319** / 1.1394 |

held-out으로 바꿔도 값이 0.03px / 0.03mm 안에서만 움직이고 순위는 그대로다. 이 두 지표는 "단일 이미지 PnP pose + 카메라 extrinsics"만 쓰는데, 15세트 중 1개를 빼고 다시 fit해도 extrinsics가 거의 안 바뀌기 때문 — 즉 이 지표들은 그 세트를 학습에 썼느냐와 거의 무관한 "카메라 배치 자체의 일치도"라는 뜻이고, held-out cube RMSE(그 세트가 빠지면 2.2→3.8px로 크게 움직임)와 성격이 다르다는 걸 다시 확인해준다.

**왜 독립이 카메라 간 일치도에서 제일 좋은가** — 쌍 종류별로 쪼개보면 (train-pooled 기준):

| 방법 | 쌍 | Cross-view px | Cam-common mm | Cam-common deg |
|---|---|---:|---:|---:|
| 통합_no-fk | 고정캠-고정캠 (39쌍) | 4.146 | 4.413 | 0.926 |
| 통합_raw-fk | 고정캠-고정캠 | 4.560 | 5.000 | 0.954 |
| 독립_no-fk | 고정캠-고정캠 | **3.826** | **3.928** | 0.930 |
| 통합_no-fk | 그리퍼캠-고정캠 (42쌍) | 3.724 | 4.050 | **1.172** |
| 통합_raw-fk | 그리퍼캠-고정캠 | 4.030 | 4.309 | 1.344 |
| 독립_no-fk | 그리퍼캠-고정캠 | **3.526** | **3.909** | 1.331 |

- 독립의 우위는 주로 **고정캠-고정캠 쌍**에서 온다(3.93 vs 4.41mm). 그리퍼캠-고정캠 쌍은 평행이동은 사실상 동률(3.91 vs 4.05mm)이고 회전은 오히려 통합이 낫다(1.17 vs 1.33°).
- 이유: 독립에서 고정캠 3대의 **상대 배치**는 고정캠들이 같이 본 것(session2 큐브, session3 보드, 둘 다 자유 변수)만으로 정해진다 — 순수 비전으로 맞춘 상대 기하. 통합은 그 큐브/보드 변수를 그리퍼캠과 **공유**하는데, 그리퍼캠의 base 좌표계 위치는 로봇 FK + hand-eye(`robot_T @ gtc`)를 거쳐 들어온다. FK 체인과 고정캠 비전 사이에 조금이라도 불일치가 있으면 그게 T_base_Ci에 타협으로 흡수되어 고정캠끼리의 상대 기하가 살짝 흐트러진다.
- 그 "FK 체인에 묶이는 추가 정보"가 바로 통합이 held-out에서 이기는 이유(base 좌표계 안에서의 **절대** 위치가 더 정확해짐)다. 즉 **FK 결합이 셀수록 카메라끼리의 일치도는 내려가고 절대 정확도는 올라가는** 트레이드오프: 독립(그룹 간 결합 0) → 통합_no-fk(큐브/보드 공유로 결합) → 통합_raw-fk(큐브를 FK 앵커에 못박음, 결합 최대 → 고정캠-고정캠 5.00mm로 최악).

## 데이터 풀 (모든 방식 공통, 162개 관측치)

| 소스 | 관측치 수 | 내용 |
|---|---:|---|
| session1 | 45 | 고정캠 3대 — 그리퍼로 쥔 큐브 (grasp+FK 모델, 재촬영) |
| session2-고정캠 | 42 | 고정캠 3대 — 바닥에 놓인 큐브 (세트별, 15곳) |
| session2-그리퍼캠 | 15 | 그리퍼캠 — 같은 바닥 큐브를 파킹 자세에서 봄 |
| session3-고정캠 | 45 | 고정캠 3대 — 바닥 마커보드 (재촬영) |
| session3-그리퍼캠 | 15 | 그리퍼캠 — 바닥 마커보드 (eye-in-hand, 재촬영) |

## 3가지 방식 정의

| 방식 | 큐브 위치 처리 | 고정캠·그리퍼캠 관계 |
|---|---|---|
| **통합_no-fk** | session2 세트별 큐브 pose = 자유 변수, 고정캠+그리퍼캠 관측치가 **같이** 그 변수를 결정 | 4개 소스 전부 하나의 최소제곱으로 동시에 품 |
| **통합_raw-fk** | session2 큐브 pose = "명령한 place pose @ T_gripper_cube(session1 원값)"로 고정(상수) | 4개 소스 전부 하나의 최소제곱(단, 큐브가 상수라 사실상 고정캠/그리퍼캠 블록이 서로 안 엮임) |
| **독립_no-fk** | 고정캠 그룹과 그리퍼 그룹을 **정보 교환 전혀 없이** 완전히 따로 풂. session2 큐브 pose는 각 그룹이 **별개의 자유 변수**로 따로 추정(공유 안 함), 끝난 뒤 두 추정치를 사후 합의로 비교 | 핸드오프 없음, 각자 로봇 FK로 base 좌표계에 독립적으로 연결 |

학습 목적함수는 전부 픽셀 재투영 오차(`solve_corner_reprojection`)다.

`raw-fk`는 no_fk에서만 통합/독립 구분이 의미가 있다 — 큐브 위치를 상수로 고정하면 고정캠/그리퍼캠 블록이 항상 수학적으로 분리되어(block-separable) 통합과 독립이 완전히 같은 답을 내므로, 독립_raw-fk는 안 만들었다.

**주의**: `통합_raw-fk`는 table1.py의 A3("raw-FK-fixed", 비전 개입 0인 순수 기계적 상수)와 이름만 같고 실제로는 다른 조건이다 — Zeus엔 그런 독립 측정 상수가 없어서 session1 비전 fit값(T_gripper_cube)을 앵커로 쓴다. 그래서 이 열은 "C3(FK 처리 방식) 검증"이 아니라 "우리만의 FK-고정 변형"으로만 해석해야 한다.

## 결과 1 — 내부 지표 (train / held-out / cross-camera)

Train RMSE는 학습에 쓴 데이터로 재는 것이고, held-out은 **정답을 그 세트를
본 카메라들 자신으로 삼각측량하지 않고, "그 세트에서 로봇이 실제로 명령받아
간 FK 위치 @ session1의 T_gripper_cube"로 계산한, 비전과 완전히 무관한 값**을
정답으로 써서 잰 것이다. Cross-cam은 같은 세트를 본 카메라들이 각자 독립적으로
계산한 큐브 pose끼리의 pairwise 차이 평균.

| 방식 | Train RMSE (px) | Held-out (px) | Held-out (mm / deg) | Cross-view (px) | Cross-cam (mm / deg) | 비고 |
|---|---:|---:|---:|---:|---:|---|
| 통합_raw-fk | 0.89 | **2.21** | **2.39 / 0.51** | 4.28 | 4.64 / 1.16 | A3 아님 (위 주의 참고) |
| **통합_no-fk** | **0.71** | 2.96 | 2.90 / 0.51 | 3.92 | 4.23 / 1.05 | |
| 독립_no-fk | 고정캠 0.68 / 그리퍼 0.65 | 3.81 | 3.18 / 0.49 | **3.67** | **3.92** / 1.14 | 큐브 사후합의: 평균 2.89mm/1.15°, 최대 6.78mm/2.35°. 보드 사후합의: 2.24mm/0.26° |

- held-out을 px로 보면 raw-fk가 제일 좋게 나오는데, 이건 "더 정확해서"가 아니라 "raw-fk 자체가 학습할 때부터 큐브 위치=FK로 못박아놓고 카메라를 맞췄기 때문"이다 — 검증 기준(FK 기반 GT)이 학습 목표(FK 앵커)와 같은 수식이라 유리한 게 당연. 절대적 정확도 우위로 해석하면 안 된다.
- 통합_no-fk가 독립_no-fk보다 held-out mm에서 낫다(2.90 vs 3.18mm). 통합이 고정캠·그리퍼캠 정보를 실제로 공유해서 쓰는 이점.
- Cross-view px와 Cross-cam mm/deg는 둘 다 "카메라들끼리 서로 동의하는 정도"라, 모든 카메라가 같은 방향으로 틀린 공통 편향은 원리적으로 못 잡는다(pairwise 뺄셈/전달에서 소거됨). 그래서 이 두 값은 held-out과 순위가 반대로 나온다(독립이 제일 좋음) — 모순이 아니라, 독립은 카메라들끼리 잘 맞춰졌지만 새 placement 예측(held-out)은 통합이 더 낫다는 뜻. Cross-view px가 Cross-cam mm/deg와 같은 순위인 건 같은 정보를 픽셀/3D 두 단위로 본 것이라 당연.

## 결과 2 — 외부 GT 실험 (3 트라이얼, `gt_compare_fits.py`)

GT 큐브를 바닥에 놓고, 그리퍼를 open/close 반복하며 **물리적으로**(카메라
화면이 아니라 손가락이 닫히면서 큐브가 자기 중심에 오게) 큐브 중심에 맞춘 뒤
읽은 flange TCP pose를 "정답"으로 쓴다 — 비전과 완전히 독립된 실측값. 3개
방식이 같은 사진에서 검출한 큐브 pose와 비교:
- **x, y**: 그리퍼가 이미 큐브 중심에 물리적으로 맞아 있으므로 offset 없이 직접 비교.
- **z**: offset 적용(`검출 z − (GT flange z − T_gripper_cube z)`; 그리퍼가
  거의 수직으로 잡는 자세라 이 뺄셈이 곧 SE(3) 변환과 동일).
- **rz**: offset 없이 직접 비교(T_gripper_cube 회전엔 ~180° 뒤집힘 컨벤션이
  섞여있어서 그대로 곱하면 오히려 왜곡됨).

방식별 평균(3 트라이얼, 절대값). xyz TRE = √(dx²+dy²+dz²)의 평균, late_table1의 TRE에 대응.

| 방식 | \|dx\| | \|dy\| | \|dz\| | **xyz TRE (mm)** | \|drz\| (deg) |
|---|---:|---:|---:|---:|---:|
| 통합_raw-fk | 0.65 | 2.01 | **0.94** | **2.65** | 0.65 |
| 통합_no-fk | 0.82 | 2.15 | 1.24 | 2.96 | 0.69 |
| 독립_no-fk | 0.68 | 2.10 | 1.65 | 3.08 | **0.63** |

트라이얼별 상세:

| 방식 | 실험 | dx | dy | dz | drz |
|---|---:|---:|---:|---:|---:|
| 통합_no-fk | 1 / 2 / 3 | +1.58 / -0.31 / +0.56 | -0.23 / +0.79 / +5.43 | -1.50 / -0.43 / -1.78 | +0.93 / +0.85 / +0.29 |
| 통합_raw-fk | 1 / 2 / 3 | +0.69 / -1.15 / -0.10 | -0.56 / +0.49 / +4.97 | -1.23 / -0.17 / -1.42 | +0.87 / +0.80 / +0.28 |
| 독립_no-fk | 1 / 2 / 3 | +1.57 / -0.26 / +0.20 | -0.54 / +0.54 / +5.24 | -1.89 / -0.81 / -2.24 | +0.86 / +0.79 / +0.24 |

- xyz TRE·z는 통합_raw-fk가 제일 좋고(2.65mm / 0.94mm), rz는 독립_no-fk가 제일 좋다(0.63°) — 지표마다 승자가 달라서, **n=3인 지금은 어느 방식이 확실히 최고라고 결론 내릴 수 없다**. 세 방식 차이(xyz TRE 0.4mm, rz 0.06°)가 트라이얼 간 편차보다 작다.
- 3개 방식 다 xyz TRE 2.7~3.1mm, rz<0.7°.
- **실험 3에서만 3개 방식 전부 dy가 +5.0~+5.4mm로 튄다** (실험 1·2는 1mm 안쪽). 방식과 무관하게 똑같이 튀는 걸로 봐서 그 트라이얼의 GT 자체(정렬 또는 큐브 놓인 상태)에 y방향 편차가 있었을 가능성이 높고, 이게 xyz TRE 평균을 지배하고 있다 — 실험 3을 빼면 xyz TRE는 1~2mm대로 내려온다. 트라이얼을 더 늘려서 확인 필요.
- 이 외부 GT는 내부 지표(held-out, cross-cam)가 원리적으로 못 잡는 "모든 카메라 공통 편향"까지 포함한 값이라, 최종 물리 순위는 이 지표로만 판정해야 한다.

## 데이터 사용률 실험 (30%/50%/70%) — ⚠️ 구버전 데이터(114개, 재촬영 전) 기준, 재실행 필요

session1+session3는 항상 전부 포함하고, session2의 15개 placement 중 학습에 쓰는 개수만 30%/50%/70%로 줄여서(나머지는 held-out), 매 비율마다 서로 다른 랜덤 조합으로 8번씩 반복 평균. 코드: `eval_data_efficiency.py`. **아래 표는 이번 재촬영(session1/3, 162개 관측치) 이전 데이터라 지금 파이프라인과 안 맞음 — 다시 돌려야 함.**

| 데이터 비율 (학습 세트 수) | 통합_no-fk (mm) | 통합_raw-fk (mm) | 독립_no-fk (mm) |
|---|---:|---:|---:|
| 30% (4개) | 2.14 | 2.23 | 2.18 |
| 50% (8개) | 2.13 | 2.32 | 2.26 |
| 70% (10개) | 2.18 | 2.28 | 2.31 |

## 아직 안 된 것 / 한계

- **C2(큐브 vs board-only)**: 검증 안 함. Zeus는 처음부터 큐브+보드를 같이 썼음.
- **C3(FK 처리)**: raw-fk/no-fk 2개만 있고, corrected-FK(A4)·vision-aligned-FK(A5, UR3에서 제일 좋았던 방법)는 없음. 있는 raw-fk도 진짜 A3가 아님(위 주의 참고).
- **C4(외부 GT)**: `gt_pick_test.py`/`gt_compare_fits.py`로 3트라이얼 진행(위 "결과 2"). 통계적으로 엄밀한 수준(P95/실패율 집계, 사전등록, n≥10 이상)은 아직 아님 — 특히 실험 3의 dy +5mm 이상치가 xyz TRE 평균을 지배하고 있어 트라이얼 추가가 필요. GT 큐브 자체의 물리 치수(config vs 실물)도 아직 완전히 재검증은 안 됨.
- **데이터 사용률 실험**: 재촬영 전 데이터 기준이라 재실행 필요(위 "데이터 사용률 실험" 섹션).
- **late_table1의 나머지 지표**: ALL Cube RMSE(train+heldout 합쳐서 한 번에 재투영)는 아직 안 만듦. Cross-view pixel transfer RMSE는 이번에 추가함(`cross_view_pixel_transfer`) — 단, late_table1은 held-out 세트에서 재는데 여기선 Cam-common과 같은 방식으로 전체 데이터 fit에서 잼(train-pooled).
- **T_gripper_cube 적용 방식**: `gt_pick_test.py`/`gt_compare_fits.py`는 T_gripper_cube 전체(x,y,z,rz,ry,rx)를 반영하도록 고쳤지만, `session2_pick_and_place.py`는 아직 예전 방식(z/ry/rx 고정값 + 큐브 원점 x,y)을 씀 — 필요하면 같이 고쳐야 함.
- held-out/cross-camera는 table1.py의 정식 split 절차가 아니라 leave-one-out으로 근사한 것 (train-pooled, held-out 완전 분리 아님).
- mm-공간 학습(`fit_calibration_methods_mm.py`)은 별도 실험으로 돌려봤으나(no-fk/독립에서 px 학습보다 held-out이 나빴음) 이 표에서는 제외 — 학습은 px 재투영으로 통일.
