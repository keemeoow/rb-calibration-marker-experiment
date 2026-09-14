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

## 2026-09-14 재촬영 결과 (GT 큐브, 카메라 리그 재배치, cam0 내부 파라미터 재촬영 반영)

> 09-14 밤에 session1/2/3를 전부 다시 찍고(`capture_replayed_0914`, `capture_placed_0914`(+held/released), `capture_replayed_0914`),
> 외부 GT 3장(`data/gt_frames_0914/20260914_232234/000..002`)을 찍었다. **세 세션 모두 GT 큐브**(`targets/gt_cube/cube_config.json`)를 썼고,
> 카메라 리그는 09-09와 130~335mm 다르게 놓였다(세션 간에는 ≤1mm/0.2° 고정 확인). 아래 숫자는 전부 0914 데이터 기준이며
> 0909 결과(다음 절 이하)와는 **같은 절차·다른 리그·다른 내부 파라미터**다. session3는 그리퍼캠만 쓴다(`--s3-gripper-only`; 고정캠 보드 41개 제외).

### 먼저 고친 것 세 가지

1. **코드 버그**: `fit_placement_fk_ablation.build_synthetic_meta_placed`가 session2 큐브 사진 경로를 `capture_placed/`로 **하드코딩**해서,
   `--session2-capture-subdir capture_placed_0914`를 줘도 **0909 옛 사진(메인 큐브)** 을 읽고 있었다. 처음 나온 train 15px / cam-common 234mm는 이 때문. 지금은 폴더명을 그대로 쓴다.
2. **cam0(serial 039422061216) 내부 파라미터 재촬영**: 지금까지 쓰던 `ur3_calibration/intrinsics/cam0.npz`는 fx 943.6으로 공장값(915.6)과 3% 달랐다.
   09-15 새로 찍은 값(ChArUco 11장, RMS 0.26px)은 fx 919.0 / cx 645(공장값과 0.4%). `intrinsics/overrides/039422061216_20260915.npz`에 넣었고,
   `fit_grasp_offset.load_intrinsics_by_label`이 `intrinsics/overrides/<serial>*.npz`가 있으면 그 카메라는 자동으로 그 값을 쓴다(모든 Zeus 스크립트 공통; `ZEUS_INTRINSICS_NO_OVERRIDE=1`로 끌 수 있음).
3. **GT 큐브 마커 모델이 실물과 달랐다** (`calibrate_gt_cube_geometry.py`로 자체 보정, 새 내부 파라미터로 다시 돌림):
   - 상단 마커(0,1)가 측면 마커 기준 **z −9.8mm** (도면 +74.5 → 실측 +64.5mm = 본체 윗면에서 +35mm). 다면 관측 PnP가 9~19px로 튀던 원인.
   - 측면 roll(90/180/270/0)은 메인 큐브 값이 맞았다. 마커 6개 6-DoF를 마커 5 게이지로 BA한 뒤 다면 코너 RMSE **2.29 → 0.49px** (사진 179장, `marker_pose_4x4`로 기록). 스티커 부착 오차는 위치 0.2~0.9mm, 기울기 0.2~0.8°, 면내 회전 ≤0.6°.
   - **마커가 공칭보다 약 1% 작게 인쇄됨**: 새 내부 파라미터로도 전체 스케일 0.985~1.01 스윕에서 0.99~0.995가 통합 train RMSE 최소(1.0: 0.549 → 0.99: 0.529px)이고 session2 큐브 사후합의는 0.99에서 최소(6.3 → 1.6mm). config에 0.99 적용(51→50.49mm, 25→24.75mm). **캘리퍼스로 측면 마커 한 변 ≈50.5mm인지 확인 바람.**

데이터 풀 168개: session1 41 / session2-고정캠 39 / session2-그리퍼캠 15 / session2-보드 58 / session3-그리퍼캠 15. `pass1_grasp_offset_replayed_0914.json`: T_gripper_cube t=[0.43, −0.07, 162.04]mm.

### Table 1 (0914, 새 내부 파라미터)

| 방법 | Train RMSE px | Heldout Cube RMSE px | Held-out mm/deg (PnP 평균) | Held-out mm/deg (**joint 삼각측량**) | Cross-view Cube px | Cam-common Cube mm/deg | External GT xyz TRE / rz (고정캠 3대, n=3) |
|---|---:|---:|---:|---:|---:|---:|---:|
| 통합_no-fk | 0.5292 | 2.1487 | 1.55 / 0.46 | 1.046 / 0.427 | 2.8485 | 3.6425 / 0.7267 | 3.04 / 0.87 |
| 통합_raw-fk | 0.7226 | **1.8909** | 1.62 / 0.45 | **0.820** / 0.427 | 3.2983 | 4.2243 / 0.7524 | 3.37 / 0.89 |
| 독립_no-fk | **0.5273 / 0.4937** | 2.4891 | 1.79 / 0.47 | 1.507 / 0.442 | **2.7262** | **3.5017** / 0.7388 | **2.86** / **0.81** |

held-out 방식 Cross-view / Cam-common(15 fold pooled): 통합_no-fk 2.8693px / 3.6623mm·0.7290°, 통합_raw-fk 3.2922 / 4.2039·0.7554, 독립 2.7431 / 3.5233·0.7403 — train-pooled와 같음.

평균 기준 일관성:

| 방법 | Cross-view **mean** \|e\| px | P95 px | 3D 코너 일관성 **mean** mm | RMS / P95 mm | 고정-고정 / 그리퍼-고정 mean mm |
|---|---:|---:|---:|---:|---:|
| 통합_no-fk | 3.380 | 7.49 | 3.657 | 4.006 / 6.70 | 4.32 / 3.10 |
| 통합_raw-fk | 3.979 | 8.05 | 4.237 | 4.591 / 7.27 | 5.10 / 3.51 |
| 독립_no-fk | **3.288** | 6.87 | **3.531** | 3.853 / 6.40 | 4.06 / 3.08 |

독립 사후합의: session2 큐브 평균 1.58mm/0.63°(최대 3.17mm/1.92°), session3 보드 0.56mm/0.26°.

0909와 같은 그림이다: **raw-fk는 FK 앵커 held-out(joint 0.82mm)에서 이기고, 카메라 간 일치(cross-view/cam-common/3D 코너)와 외부 GT는 독립·no-fk가 낫다.** 절대값은 0909보다 좋아졌다(cross-view 3.87→2.85px, cam-common 4.16→3.64mm, joint held-out 1.76→1.05mm) — 리그·큐브·내부 파라미터가 달라서 직접 비교는 못 하지만, 큐브 기하를 데이터로 맞춘 효과가 크다. cam0 내부 파라미터 교체 전후(같은 데이터)로는 cross-view 2.95→2.85px, 외부 GT TRE 3.39→3.04mm, 그리퍼캠 포함 외부 GT 2.48→2.04mm로 소폭 개선.

### 외부 GT (0914, `gt_eval_offline.py`, 절대값 평균 n=3)

GT flange pose: 000 (−242.01, 430.02, 178.71, −60.01), 001 (−302.00, 370.00, 178.71, −50.01), 002 (−301.99, 499.98, 178.71, −120.01). 사진은 로봇 미접속으로 찍어 robot.json에 pose가 없다 → 기본은 고정캠 3대 공동 추정. 그리퍼캠은 "촬영 자세 = GT + z300mm" 가정(`--assume-flange-dz 300`)으로만 넣을 수 있다.

| 방식 | \|dx\| | \|dy\| | \|dz\| | **xyz TRE** | \|drz\| | (그리퍼캠 포함 가정) xyz TRE / rz |
|---|---:|---:|---:|---:|---:|---:|
| 독립_no-fk | 1.42 | 1.07 | 2.11 | **2.86** | **0.81** | **1.82** / 0.87 |
| 통합_no-fk | 1.77 | 0.98 | 2.17 | 3.04 | 0.87 | 2.04 / 0.92 |
| 통합_raw-fk | 2.13 | 1.04 | 2.29 | 3.37 | 0.89 | 2.42 / 0.94 |

축별(부호 = 검출 − GT, 고정캠 3대 공동, n=3 평균): 통합_no-fk x −1.77, y −0.44, **z +2.17**, rz +0.87°; raw-fk x −2.13, y −0.61, z +2.29; 독립 x −1.42, y −0.82, z +2.11. 트라이얼별(통합_no-fk): dx −2.23/−1.81/−1.27, dy +0.80/−0.90/−1.23, dz +1.72/+1.82/+2.98, drz +0.63/+0.99/+0.98. 세 방식 모두 **dx ≈ −1.8mm, dz ≈ +2.2mm가 공통**이고 방식 간 차이(0.5mm)는 그보다 작다 → n=3으로 방식 순위를 못 가른다.

카메라별 단일 PnP(통합_no-fk, xyz TRE): cam0 3.03mm(축별 평균 x −1.86는 트라이얼 002만 +4.0으로 튐), fixed2 4.47, fixed3 5.33, 그리퍼(가정) 2.76 — 공동 추정(3.04)이 어느 고정캠 하나보다 좋다. z는 세 고정캠이 전부 +2.0~+2.4로 같고(카메라-FK 공통 편향), 그리퍼캠은 z −2.4로 부호가 반대(가정한 촬영 높이 오차 가능).

### 릴리즈 슬립 vs 카메라-FK 편향 (`analyze_release_slip.py`, held/released 15세트)

| 항목 | 중앙값 dx dy dz (mm) | MAD | \|·\| 평균 |
|---|---|---|---:|
| 릴리즈 슬립 (released − held, 3대 평균 pose) | −0.07 −0.16 +0.12 | 0.35 0.30 0.28 | 0.77 |
| 카메라-FK 편향 (held − FK@T_gripper_cube) | +0.71 +0.92 **+2.01** | 0.56 0.81 0.77 | 2.80 |
| 합계 (released − anchor) | +0.74 +1.01 +2.20 | 0.70 1.96 0.44 | 2.86 |

**카메라별(같은 카메라의 held/released 두 장만 빼서 외부 파라미터 오차를 완전히 상쇄)** 슬립: cam0 중앙값 [0.00, −0.08, +0.10] MAD [0.43, 0.27, 0.32], fixed2 [0.00, −0.03, +0.04] MAD [0.14, 0.32, 0.23], fixed3 [−0.09, −0.03, +0.07] MAD [0.34, 0.39, 0.39] mm; 카메라 간 편차(std) 평균 [0.42, 0.43, 0.33] mm가 슬립 측정의 노이즈 바닥. 15세트 중 13세트는 세 카메라 모두 |슬립| < 0.5mm(중앙값 기준 사실상 0), **세트 13·14만 세 카메라가 일치해서 1.1~2.0mm / 1.4° 슬립**을 본다. 세트 4·6(cam0)·7(fixed2)은 그 카메라 한 대만 튀고 나머지 둘은 0.1~0.5mm라 실제 슬립이 아니라 단일 카메라 PnP 튐이다.

그리퍼가 아직 큐브를 쥔 상태에서도 카메라가 보는 큐브가 FK 예측보다 **z +2.0mm** 위에 있다(MAD 0.8) — 외부 GT의 dz +2.2mm와 같은 값. 릴리즈 슬립은 0.8mm로 작다. 즉 held-out 0.5mm 벽의 정체는 슬립이 아니라 테이블 높이에서의 **카메라-FK 계통 편향**이고, 이건 session1(큐브가 카메라 가까이·높이 다양)로 맞춘 외부 파라미터가 테이블 높이까지 그대로 안 맞는 것이다. 이 편향은 cam0 내부 파라미터 교체 후에도 그대로라(2.23 → 2.01mm) cam0 문제가 아니었다. 마커 스케일을 1.0으로 두면 이 z 편향은 0에 가깝지만 카메라 쌍 불일치가 8mm로 커진다 — 스케일은 내부 일관성으로 정했고, 캘리퍼스 실측이 이 둘을 가른다.

재현: `fit_grasp_offset.py --capture-subdir capture_replayed_0914 --cube-config ../targets/gt_cube/cube_config.json --out pass1_grasp_offset_replayed_0914.json` → `fit_calibration_methods.py`/`eval_heldout_and_consistency.py` 에 `--session1-capture-subdir capture_replayed_0914 --session2-capture-subdir capture_placed_0914 --session3-capture-subdir capture_replayed_0914 --fit-json pass1_grasp_offset_replayed_0914.json --cube-config ../targets/gt_cube/cube_config.json --s3-gripper-only [--tag _0914]` → `gt_eval_offline.py --capture-root data/gt_frames_0914/20260914_232234 --trial ... --fits fit_*_0914.json` → `analyze_release_slip.py --fit fit_통합_no-fk_0914.json --capture-subdir capture_placed_0914 --cube-config ../targets/gt_cube/cube_config.json`. (GT 큐브 기하: `calibrate_gt_cube_geometry.py --gauge side5 --write` 후 스케일 0.99 적용.)

---

## Table 1 전체 결과 (late_table1/Session04와 같은 컬럼 구조) — 0909 데이터

> ALL Cube RMSE는 late_table1엔 있지만 Zeus 쪽엔 아직 안 만든 지표라 N/A. External GT는 late_table1의 TRE와 같은 개념으로 xyz 3D 오차(mm)와 rz 오차(deg)를 씀(n=3, 아래 "결과 2" 참고).

| 방법 | Calibration train target | Optimization | FK/target-pose 처리 | Train RMSE px | ALL Cube RMSE px | Heldout Cube RMSE px | Cross-view Cube px | Cam-common Cube mm/deg | External GT xyz TRE / rz (mm/deg, n=3) |
|---|---|---|---|---:|---:|---:|---:|---:|---:|
| 통합_no-fk | board+cube | unified_joint_optimization | cube pose=estimated | 0.6299 | N/A | 3.0863 | 3.8695 | 4.1569 / 0.9986 | 3.18 / 0.72 |
| 통합_raw-fk | board+cube | unified_joint_optimization | cube pose=raw-FK-fixed | 0.7707 | N/A | **2.2907** | 4.1042 | 4.4201 / 1.0675 | **2.77** / 0.69 |
| 독립_no-fk | board+cube (그룹 분리) | independent_parallel (핸드오프 없음) | cube pose=estimated | 0.61 / 0.55 | N/A | 3.8432 | **3.7037** | **3.9423** / 1.0798 | 3.20 / **0.66** |

| 통합_no-fk **cube-only** | cube only (board 전부 제외, session3 미사용) | unified_joint_optimization | cube pose=estimated | 0.9832 | N/A | 3.3627 | 3.8476 | 4.1516 / 0.9583 | 2.68 / 0.62 |
| 통합_raw-fk **cube-only** | cube only | unified_joint_optimization | cube pose=raw-FK-fixed | 1.2963 | N/A | **2.1210** | 5.4363 | 5.9870 / 1.0247 | **2.57** / 0.65 |
| 독립_no-fk cube-only | cube only | independent_parallel | — | N/A | N/A | N/A | N/A | N/A | N/A |

**cube-only(late_table1 B2 "−board" 대응)**: 보드 관측을 전부 빼면(관측치 220 → 102) 통합_no-fk는 train 0.63→0.98px, held-out px 3.09→3.36, joint 1.76→2.11mm로 전반적으로 나빠지고, 통합_raw-fk는 held-out px가 2.29→2.12로 좋아 보이지만 cross-view 4.10→5.44px / cam-common 4.42→5.99mm로 **카메라 간 일치도가 크게 무너진다**(큐브 FK 앵커에 카메라를 맞추는 걸 보드가 더 이상 견제해주지 않아서). **독립 cube-only는 정의 불가**: 그리퍼 그룹 관측이 파킹 자세 1개에서 찍은 큐브 15장뿐이고 큐브 pose가 자유 변수라 T_gripper_cam이 식별되지 않는다(보드가 그 역할). 결론: 보드는 통합에서 "있으면 좋은" 게 아니라 그리퍼캠 회전 식별과 raw-fk의 카메라 왜곡 억제에 필요하다. cube-only의 gtc 초기값은 session2 큐브만으로(`init_gtc_from_cubes`) 구했다.

(데이터 풀 220개 기준. 직전 162개(session2 보드 미포함) 값: 통합_no-fk 0.7121 / 2.9557 / 3.9234 / 4.2250·1.0534, 통합_raw-fk 0.8850 / 2.2087 / 4.2813 / 4.6416·1.1567, 독립 0.68·0.65 / 3.8053 / 3.6668 / 3.9182·1.1376 — 보드 추가로 train·cross-view·cam-common은 전부 개선, held-out px는 소폭 악화.)

(독립의 Train은 고정캠/그리퍼캠 두 그룹을 완전히 따로 풀기 때문에 "고정캠값/그리퍼캠값"으로 표기. Heldout px/Cross-view px/Cam-common은 `eval_heldout_and_consistency.py`로 계산. Cross-view Cube px = late_table1과 같은 정의로, 한 카메라의 단일 이미지 PnP pose를 캘리브레이션된 extrinsics로 다른 카메라로 옮겨 재투영했을 때의 코너 px RMSE — 고정캠-고정캠 + 그리퍼캠-고정캠 쌍, 양방향, 세트당 4대 → 81쌍/162방향.)

Cross-view px / Cam-common을 **held-out 방식**(각 fold에서 빠진 세트에 대해서만 재고 15 fold pooled, late_table1이 held-out 세트에서 재는 것과 같은 구조)으로도 계산했다. 위 표의 train-pooled 값과 거의 같다:

| 방법 | Cross-view px (train-pooled) | Cross-view px (**held-out**) | Cam-common mm/deg (train-pooled) | Cam-common mm/deg (**held-out**) | Held-out mm/deg (**joint 삼각측량**) |
|---|---:|---:|---:|---:|---:|
| 통합_no-fk | 3.8695 | 3.8852 | 4.1569 / 0.9986 | 4.1713 / 1.0012 | 1.764 / 0.511 |
| 통합_raw-fk | 4.1042 | 4.0937 | 4.4201 / 1.0675 | 4.4026 / 1.0719 | **0.945** / 0.538 |
| 독립_no-fk | 3.7037 | **3.7161** | 3.9423 / 1.0798 | **3.9523** / 1.0811 | 2.354 / **0.460** |

**공통 시야 카메라 쌍 일관성 — 평균 기준** (Table 1의 Cross-view px는 dx,dy 성분 RMS; 아래는 코너별 |e| 평균과 3D 코너 거리 평균. `corner_consistency_3d`는 카메라 A, B가 각자 추정한 큐브 pose로 코너 24개를 base에 놓고 대응 코너끼리 3D 거리를 잰 것 — Cam-common의 평행이동/회전을 코너 위치 하나로 합친 값):

| 방법 | Cross-view **mean** \|e\| px | Cross-view P95 px | 3D 코너 일관성 **mean** mm | 3D 코너 RMS / P95 mm | 고정-고정 / 그리퍼-고정 mean mm |
|---|---:|---:|---:|---:|---:|
| 통합_no-fk | 4.661 | 9.63 | 4.234 | 4.618 / 7.28 | 4.38 / 4.10 |
| 통합_raw-fk | 4.930 | 10.19 | 4.491 | 4.865 / 7.75 | 4.78 / 4.22 |
| 독립_no-fk | **4.411** | 9.54 | **3.988** | 4.412 / 7.09 | 4.04 / 3.94 |
| 통합_no-fk cube-only | 4.623 | 9.93 | 4.229 | 4.585 / 7.30 | 4.37 / 4.09 |
| 통합_raw-fk cube-only | 6.680 | 13.64 | 6.063 | 6.498 / 9.83 | 7.01 / 5.18 |

(mean |e|가 Table 1의 RMS보다 큰 건 Table 1이 성분 RMS(=|e| RMS/√2)이기 때문. 순위는 RMS·mean·3D 코너 어느 것으로 봐도 동일.)

**Train 재투영 오차의 여러 통계** (같은 fit, 코너별 유클리드 오차 |e| 기준; Table 1의 "Train RMSE px"는 dx,dy 성분을 각각 표본으로 본 RMS라 |e|의 RMS보다 √2배 작다):

| 방법 | mean \|e\| | RMS \|e\| | RMS(성분) = Table 1 | median | P95 |
|---|---:|---:|---:|---:|---:|
| 통합_no-fk | 0.697 | 0.891 | 0.630 | 0.563 | 1.81 |
| 통합_raw-fk | 0.816 | 1.090 | 0.771 | 0.644 | 2.10 |
| 독립_no-fk 고정캠 / 그리퍼 | 0.676 / 0.611 | 0.869 / 0.776 | 0.615 / 0.548 | 0.535 / 0.459 | 1.78 / 1.60 |
| 통합_no-fk cube-only | 1.163 | 1.390 | 0.983 | 0.987 | 2.59 |
| 통합_raw-fk cube-only | 1.550 | 1.833 | 1.296 | 1.339 | 3.30 |

`joint` 열은 held-out 큐브를 4대 코너로 한 번에 삼각측량(`joint_cube_estimate`)한 held-out — 단일 PnP 4개 평균(위 표 Held-out mm 2.4~3.1)보다 노이즈가 작아 방식 차이가 훨씬 선명하게 갈린다(결과 3 참고).

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

## 데이터 풀 (모든 방식 공통, 220개 관측치)

| 소스 | 관측치 수 | 내용 |
|---|---:|---|
| session1 | 45 | 고정캠 3대 — 그리퍼로 쥔 큐브 (grasp+FK 모델, 재촬영) |
| session2-고정캠 (큐브) | 42 | 고정캠 3대 — 바닥에 놓인 큐브 (세트별, 15곳) |
| session2-그리퍼캠 (큐브) | 15 | 그리퍼캠 — 같은 바닥 큐브를 파킹 자세에서 봄 |
| session2-보드 | 58 | 고정캠 3대(44) + 그리퍼캠(14) — session2 사진에 그대로 들어있던 바닥 마커보드 (신규: 예전엔 큐브만 뽑고 버렸음) |
| session3-고정캠 | 45 | 고정캠 3대 — 바닥 마커보드 (재촬영) |
| session3-그리퍼캠 | 15 | 그리퍼캠 — 바닥 마커보드 (eye-in-hand, 재촬영) |

session2와 session3의 보드는 같은 자리다(같은 고정캠으로 본 base 좌표 차이 0.25~0.42mm / 0.05°) → 두 세션의 보드 관측이 **하나의 `T_base_board` 변수**를 공유한다.

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
| 통합_raw-fk | 0.77 | **2.29** | **2.39 / 0.49** | 4.10 | 4.42 / 1.07 | A3 아님 (위 주의 참고) |
| **통합_no-fk** | **0.63** | 3.09 | 2.92 / 0.49 | 3.87 | 4.16 / 1.00 | |
| 독립_no-fk | 고정캠 0.61 / 그리퍼 0.55 | 3.84 | 3.12 / 0.46 | **3.70** | **3.94** / 1.08 | 큐브 사후합의: 평균 2.88mm/1.01°, 최대 7.33mm/2.33°. 보드 사후합의: 2.01mm/0.20° |

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

## 결과 3 — corrected-FK 변형 탐색 (`fit_corrected_fk.py`)

목표: 카메라 간 일치도와 못 본 큐브 held-out을 **둘 다** 통합_no-fk/통합_raw-fk보다
좋게 만드는 corrected-FK 찾기. 손으로 정한 상수 없이(FK 공분산, 보정 변환, 회귀
계수 전부 데이터에서 추정한 변수) 9가지 조합을 같은 데이터·같은 지표로 비교했다.

- 큐브 pose 처리: `free`(자유 변수) / `hard`(= 보정된 FK anchor로 고정) /
  `soft`(자유 변수 + 보정된 anchor 쪽으로 당기는 FK factor, sigma는 no-fk 잔차의
  MAD로 초기화 후 EM식 재추정 2회)
- 보정: `none` / `R`(그리퍼 프레임 고정 오프셋 `anchor@exp(cR)`) / `L`(base 프레임
  `exp(cL)@anchor`) / `LR` / `lin`(놓은 자세 [1,x,y,cos rz,sin rz]에 선형인 6xk 보정,
  no-fk 잔차 선형회귀로 초기화 후 공동 최적화)
- `free+none` = 통합_no-fk, `hard+none` = 통합_raw-fk, `soft+none` = A4(soft FK
  factor), `hard+R` = A5류(vision-aligned FK)

**먼저 FK 잔차의 구조** (no-fk 큐브 vs FK anchor, 그리퍼 프레임, 15세트):
계통 오프셋 중앙값 **(0.03, −1.05, +1.00) mm / (−0.32, 0.00, 0.12)°**, 랜덤 산포(MAD)
**(0.55, 0.53, 0.19) mm / (0.14, 0.16, 0.16)°**. 즉 놓을 때 그리퍼 기준으로 y −1, z +1
mm쯤 일정하게 밀리는 성분(보정 가능)과, 놓을 때마다 다른 0.2~0.55 mm/축의
슬립(어떤 알고리즘도 못 잡는 하한)이 있다.

**held-out 추정기 문제**: 기존 held-out mm은 "단일 이미지 PnP 4개 robust 평균"으로
큐브를 추정했는데, 같은 카메라(통합_no-fk)로 학습 세트 큐브를 다시 봐도 FK 대비
2.90 mm였다. 4대 카메라 코너를 **한 번에** 쓰는 공동 삼각측량(`joint_cube_estimate`,
frozen 카메라, 큐브 6-DoF만 변수)으로 바꾸면 같은 카메라에서 **1.70 mm** — 기존
held-out mm의 절반 가까이가 캘리브레이션이 아니라 평가 추정기 노이즈였다. 아래
표의 `ho_joint`가 그 추정기로 잰 held-out(모든 방식에 동일 적용).

| 변형 | Train px | Held-out px | Held-out mm (avg-PnP) | **Held-out mm (joint)** | Held-out deg (joint) | Cross-view px | Cam-common mm/deg | External z / TRE (mm) |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| free+none (통합_no-fk) | 0.712 | 2.956 | 2.904 | 1.700 | 0.492 | **3.923** | **4.225 / 1.053** | 1.24 / 2.96 |
| hard+none (통합_raw-fk) | 0.885 | **2.209** | **2.390** | **0.889** | 0.502 | 4.281 | 4.642 / 1.157 | **0.95 / 2.66** |
| soft+none (A4) | 0.748 | 2.540 | 2.554 | 1.235 | 0.504 | 3.972 | 4.279 / 1.108 | 1.10 / 2.92 |
| hard+R (A5류) | 0.853 | 2.601 | 2.864 | 1.440 | **0.478** | 4.197 | 4.572 / 1.094 | 1.11 / **2.66** |
| hard+L | 0.832 | 2.846 | 2.941 | 1.592 | 0.504 | 4.228 | 4.613 / 1.069 | 1.10 / 2.81 |
| hard+LR | 0.799 | 2.858 | 2.919 | 1.599 | 0.500 | 4.162 | 4.525 / 1.069 | 1.11 / 2.83 |
| soft+R | 0.723 | 2.908 | 2.893 | 1.662 | 0.482 | **3.921** | 4.228 / 1.056 | 1.23 / 2.94 |
| hard+lin | 0.763 | 2.921 | 2.913 | 1.667 | 0.505 | 4.011 | 4.349 / 1.057 | 1.17 / 2.89 |
| soft+lin | 0.718 | 2.953 | 2.906 | 1.699 | 0.493 | 3.926 | 4.229 / 1.053 | 1.23 / 2.96 |

(External = 저장된 외부 GT 사진 3장을 `gt_eval_offline.py`로 오프라인 재채점, |dz| 평균 / xyz TRE 평균. 추정된 보정값: hard+R cR = (−0.10, −0.66, +0.91) mm; soft+R cR = (0.09, −0.97, +1.05) mm — no-fk 잔차 중앙값과 일치.)

**결론**:
1. **held-out(FK 기준 GT)에서는 어떤 보정도 raw-fk(`hard+none`)를 못 이긴다** (joint 0.889 mm vs 나머지 1.24~1.70). 이건 알고리즘 문제가 아니라 **GT 정의의 구조** 때문이다: held-out GT = `FK@grasp_init`인데, 보정 변수는 학습 큐브를 정확히 그 값에서 계통 오프셋(~1.4 mm)만큼 떨어진 "비전이 실제로 본 자리"로 옮긴다. 그러면 카메라는 비전 기준으로 편향이 없어지지만(cross-view 개선), held-out GT와는 그 오프셋만큼 어긋난다. raw-fk는 큐브를 GT 정의와 똑같은 값에 못박으므로 이 지표에서 구조적으로 최선이다.
2. **카메라 간 일치도에서는 어떤 보정도 free+none을 못 이긴다** (3.92 px). 보정은 hard 계열의 일치도를 개선하지만(hard+none 4.28 → hard+LR 4.16) free 수준까지는 못 간다.
3. 그래서 두 지표는 **트레이드오프**로 남고, 둘 다에서 baseline을 이기는 변형은 없다. 그 중간을 잇는 최선은 `soft+none`(A4): held-out joint 1.70→1.24 mm로 개선하면서 일치도 손실은 3.92→3.97 px에 그친다. `hard+R`(A5류)도 중간 지점(1.44 / 4.20).
4. **외부 GT(n=3)**에서도 raw-fk가 z 0.95 mm로 제일 좋고 나머지는 1.10~1.24 mm로 차이가 트라이얼 간 편차보다 작다 — 순위를 확정할 수 없다.
5. **0.5 mm 목표는 이 데이터로는 도달 불가**: 최선(raw-fk joint) 0.889 mm이고, 그 아래엔 릴리즈 랜덤 슬립(축당 0.2~0.55 mm → 3D 0.6~0.8 mm)이 하한으로 깔려 있다. 더 내리려면 알고리즘이 아니라 (a) 놓는 동작 자체의 반복성 개선, (b) held-out 세트를 여러 프레임 찍어 평균, (c) 카메라 해상도/거리 개선이 필요하다.
6. 앞으로 판정에 쓸 지표: held-out은 **joint 추정기 값**을 기본으로 쓴다(avg-PnP 값은 추정기 노이즈가 지배). 그리고 "vision이 본 자리"와 "FK@grasp_init"의 1.4 mm 계통 차이가 실제 큐브 위치 기준으로 어느 쪽이 맞는지는 외부 GT 트라이얼을 늘려서만 가릴 수 있다 — 현재 n=3에서는 raw-fk 쪽(FK@grasp_init)이 약간 우세하다.

## 데이터 사용률 실험 (30%/50%/70%) — ⚠️ 구버전 데이터(114개, 재촬영 전) 기준, 재실행 필요

session1+session3는 항상 전부 포함하고, session2의 15개 placement 중 학습에 쓰는 개수만 30%/50%/70%로 줄여서(나머지는 held-out), 매 비율마다 서로 다른 랜덤 조합으로 8번씩 반복 평균. 코드: `eval_data_efficiency.py`. **아래 표는 이번 재촬영(session1/3, 162개 관측치) 이전 데이터라 지금 파이프라인과 안 맞음 — 다시 돌려야 함.**

| 데이터 비율 (학습 세트 수) | 통합_no-fk (mm) | 통합_raw-fk (mm) | 독립_no-fk (mm) |
|---|---:|---:|---:|
| 30% (4개) | 2.14 | 2.23 | 2.18 |
| 50% (8개) | 2.13 | 2.32 | 2.26 |
| 70% (10개) | 2.18 | 2.28 | 2.31 |

## 아직 안 된 것 / 한계

- **C2(큐브 vs board-only)**: 검증 안 함. Zeus는 처음부터 큐브+보드를 같이 썼음.
- **C3(FK 처리)**: corrected-FK(A4=`soft+none`)·vision-aligned-FK(A5류=`hard+R`) 포함 9개 변형을 `fit_corrected_fk.py`로 비교했으나(결과 3), held-out과 일치도를 동시에 baseline보다 개선하는 변형은 없었음. 있는 raw-fk는 진짜 A3가 아님(위 주의 참고).
- **C4(외부 GT)**: `gt_pick_test.py`/`gt_compare_fits.py`로 3트라이얼 진행(위 "결과 2"). 통계적으로 엄밀한 수준(P95/실패율 집계, 사전등록, n≥10 이상)은 아직 아님 — 특히 실험 3의 dy +5mm 이상치가 xyz TRE 평균을 지배하고 있어 트라이얼 추가가 필요. GT 큐브 자체의 물리 치수(config vs 실물)도 아직 완전히 재검증은 안 됨.
- **데이터 사용률 실험**: 재촬영 전 데이터 기준이라 재실행 필요(위 "데이터 사용률 실험" 섹션).
- **late_table1의 나머지 지표**: ALL Cube RMSE(train+heldout 합쳐서 한 번에 재투영)는 아직 안 만듦. Cross-view pixel transfer RMSE는 이번에 추가함(`cross_view_pixel_transfer`) — 단, late_table1은 held-out 세트에서 재는데 여기선 Cam-common과 같은 방식으로 전체 데이터 fit에서 잼(train-pooled).
- **T_gripper_cube 적용 방식**: `gt_pick_test.py`/`gt_compare_fits.py`는 T_gripper_cube 전체(x,y,z,rz,ry,rx)를 반영하도록 고쳤지만, `session2_pick_and_place.py`는 아직 예전 방식(z/ry/rx 고정값 + 큐브 원점 x,y)을 씀 — 필요하면 같이 고쳐야 함.
- held-out/cross-camera는 table1.py의 정식 split 절차가 아니라 leave-one-out으로 근사한 것 (train-pooled, held-out 완전 분리 아님).
- mm-공간 학습(`fit_calibration_methods_mm.py`)은 별도 실험으로 돌려봤으나(no-fk/독립에서 px 학습보다 held-out이 나빴음) 이 표에서는 제외 — 학습은 px 재투영으로 통일.
