# 시뮬(Simul_test) ↔ 실데이터(CP_result) 통합 비교표

`CP_viz_sim_vs_real.py` 가 두 산출물에서 직접 읽어 생성. 변화율(%)은 baseline 대비이며
**음수 = 개선(오차 감소)**, 관측성 지표(동시관측·시야각)만 **양수 = 개선**이다.
재현 판정: `O` 방향·크기 모두 재현, `△` 방향만 재현(효과 1/4 미만), `X` 반대 방향.

| 기여 | 지표 | baseline → proposed | 시뮬 base | 시뮬 prop | 시뮬 Δ% | CP base | CP prop | CP Δ% | 재현 |
|---|---|---|---:|---:|---:|---:|---:|---:|:---:|
| C1 | hand-eye gTc (sim: vs GT / CP: gripper-align RMSE) | independent → unified_joint | 23.095 | 0.169 | -99.3% | 15.06 | 13.67 | -9.2% | △  (방향만) |
| C1 | shared-base consistency | independent → unified_joint | 17.933 | 1.204 | -93.3% | 15.98 | 15.25 | -4.6% | X  (효과없음) |
| C1 | held-out cube prediction (no correction) | independent → unified_joint | 4.647 | 1.718 | -63.0% | 13.39 | 11.67 | -12.8% | △  (방향만) |
| C1 | held-out cube prediction: effect of +fk correction | unified_joint → unified_joint +fk | 1.718 | 0.354 | -79.4% | 11.67 | 2.51 | -78.5% | O |
| C2 | simultaneous observers | board → cube | 2.633 | 4.000 | 51.9% | 1.03 | 2.70 | 162.8% | O |
| C2 | viewpoint coverage | board → cube | 71.823 | 79.964 | 11.3% | 118.73 | 103.84 | -12.5% | X  (반대) |
| C2 | camera pose err (sim: vs GT / CP: cross-camera) | board_only → hybrid | 8.314 | 4.725 | -43.2% | 26.93 | 4.40 | -83.7% | O |
| C2 | target pred. err (sim: vs GT / CP: pose repeatability) | board_only → hybrid | 10.356 | 6.817 | -34.2% | 26.93 | 4.41 | -83.6% | O |
| C3 | held-out prediction: FK prior vs camera-only | Camera-based → FK-based | 1.073 | 1.217 | 13.4% | 22.68 | 24.02 | 5.9% | O |
| C3 | held-out prediction: effect of residual correction | sim: Camera-based / CP: fk-prior → + correction | 1.073 | 0.183 | -82.9% | 24.02 | 20.98 | -12.7% | △  (방향만) |
| C3 | best method vs camera-only baseline | Camera-based / no-fk-prior → best | 1.073 | 0.183 | -82.9% | 22.68 | 20.98 | -7.5% | △  (방향만) |

## 단위·프록시 주의

- 시뮬은 GT 대비 절대오차(mm), CP 는 GT 가 없어 **로봇 FK 큐브중점 프록시** 대비 오차다.
  절대 크기를 직접 비교하지 말고 **Δ% 와 방향**을 비교할 것.
- C1 `bTf`(고정카메라 절대오차)는 실데이터에서 측정 불가 → 표·figure 에서 제외(N/A).
- C1 gTc 는 CP 에서 `grip_align_trans_rmse_mm`(그리퍼 정합 RMSE) 프록시로 대체.
- C2 카메라오차/타깃예측은 CP 에서 각각 `cross_camera_mean_mm`/`pose_repeat_mm` 프록시.
- C3 후보정은 시뮬이 Camera-based 위에, CP 가 fk-prior 위에 얹는다(프로젝트 규약).
  따라서 '후보정 효과' 행의 baseline 이 서로 다르다.

생성: `PYTHONPATH= python CP_viz_sim_vs_real.py`
