# D1 — FK 고정 × 잔차 보정, soft-anchor λ sweep 포함

- split: **position (set-level) hold-out**, leave_one_cube_position_out, 13/13 folds usable
- `CP_common.ROBOT_POS_SCALE_PINNED` = **1.0000** (로봇 원본 값 그대로). 다른 값으로 만든 결과와는 FK 기준 자체가 달라 비교할 수 없다.
- 모든 위치 오차는 `FK-proxy`다. 외부 GT가 아니므로 절대 정확도로 읽지 않는다.
- position hold-out의 역할 규정: `external_GT_or_explicit_FK_proxy_only`
- 모든 arm이 동일 backend·동일 solver 설정·동일 예측 mask를 쓴다. 차이는 `T_base_cube_by_set`의 처리뿐이다: 하드 고정(A3) / 자유(λ=0) / soft anchor(λ>0).
- anchor는 큐브 probe 점의 변위(mm)로 표현하고 λ의 단위는 **px/mm**다 (lever 29.5 mm). λ=0은 canonical solve와 동일하며 실행 시 첫 fold에서 그 동치를 검증한다.

## 핵심표 — held-out 위치 오차 RMSE (mm, FK-proxy)

| arm | λ (px/mm) | none | offset(3) | SE(3)(6) | Ridge(9) |
| --- | ---: | ---: | ---: | ---: | ---: |
| A3 — FK-fixed (hard) | ∞ (hard) | 4.375 | 3.955 | 3.535 | 3.249 |
| A2 — vision-estimated (no anchor) | 0 | 5.079 | 4.731 | 3.804 | 3.399 |

최저 셀: **A3 @ ridge = 3.249 mm**.

상위 5개 셀:

| 순위 | arm | 보정 | RMSE (mm) |
| ---: | --- | --- | ---: |
| 1 | A3 | ridge | 3.249 |
| 2 | A2 | ridge | 3.399 |
| 3 | A3 | se3 | 3.535 |
| 4 | A2 | se3 | 3.804 |
| 5 | A3 | offset | 3.955 |

## paired 판정 — A3(하드 고정) 대비 (음수가 A3 우세)

| 비교 | mean±std (mm) | SE | t | A3 우세 fold |
| --- | ---: | ---: | ---: | ---: |
| A3 − A2 @ none | -0.487±0.874 | 0.252 | -1.93 | 10/13 |
| A3 − A2 @ offset | -0.696±0.729 | 0.210 | -3.31 | 11/13 |
| A3 − A2 @ se3 | -0.182±0.732 | 0.211 | -0.86 | 9/13 |
| A3 − A2 @ ridge | -0.130±0.570 | 0.164 | -0.79 | 8/13 |

## arm별 보정 이득 (음수가 개선)

| arm | 보정 | mean±std (mm) | SE | t | 개선 fold |
| --- | --- | ---: | ---: | ---: | ---: |
| A3 | offset − none | -0.319±1.211 | 0.350 | -0.91 | 8/13 |
| A3 | se3 − none | -0.674±1.430 | 0.413 | -1.63 | 8/13 |
| A3 | ridge − none | -0.929±1.752 | 0.506 | -1.84 | 9/13 |
| A2 | offset − none | -0.109±1.458 | 0.421 | -0.26 | 6/13 |
| A2 | se3 − none | -0.979±2.083 | 0.601 | -1.63 | 9/13 |
| A2 | ridge − none | -1.286±2.406 | 0.695 | -1.85 | 8/13 |

## 자유도·조건수와 통제 지표

보정은 예측 시점에만 적용되고 `T_base_Ci`/`T_gripper_cam`을 바꾸지 않는다. 따라서 오른쪽 두 통제 지표는 보정 유무와 무관하게 동일하며, 어떤 보정 결과도 재투영 개선으로 보고할 수 없다.

| arm | n_params | Jacobian cond | nfev | anchor 변위 RMS (mm) | 재투영 (px, FK pose) | e_cross (mm, FK 무관) |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| A3 | 30 | 139.9 | 20.8 | — | 7.274±6.329 | 19.307±6.457 |
| A2 | 102 | 523.1 | 150.4 | — | 7.692±6.522 | 17.793±6.301 |

## fold별 원본 (Ridge 보정 적용값, mm)

| held-out set | A3 | A2 |
| ---: | ---: | ---: |
| 0 | 3.361 | 2.980 |
| 1 | 1.752 | 2.097 |
| 2 | 3.015 | 3.105 |
| 3 | 2.239 | 2.003 |
| 4 | 3.771 | 4.684 |
| 5 | 3.577 | 4.348 |
| 6 | 3.029 | 3.500 |
| 7 | 1.607 | 1.328 |
| 8 | 4.176 | 4.748 |
| 9 | 5.167 | 3.867 |
| 10 | 3.647 | 4.215 |
| 11 | 2.944 | 2.905 |
| 12 | 1.975 | 2.165 |

## 해석 규칙

The residual correction is applied to the predicted cube centre after the fit is frozen.  It never modifies T_base_Ci or T_gripper_cam, so control_heldout_reprojection_fk_pose_px and control_heldout_e_cross_translation_mm are identical for every correction state of a given arm.

A correction can never be reported as a reprojection improvement. Any claim built on this experiment is a claim about held-out position agreement with the FK proxy only.
