# 사전 등록: Hard rejection threshold 민감도 실험 (Session04)

> 이 문서는 **결과를 보기 전에** 확정한 설계다. 실행 후 어떤 항목도 수정하지 않는다.
> 기존 [OUTLIER_LOSS_ABLATION.md](OUTLIER_LOSS_ABLATION.md)는 loss 함수(soft-L1 vs linear) 대조였고,
> 사전 품질 마스크를 고정했다. 이 실험은 그 고정했던 **마스크 자체**를 움직인다.

## 1. 연구 질문

프레임/관측 **hard rejection threshold**를 바꾸면 캘리브레이션 결과가 얼마나 달라지는가.
그리고 그 변화가 세 기준(Cube PnP RMSE, Cube inlier fraction, Board ChArUco corner 수) 중
어느 것에서 오는가.

## 2. 관측 품질 분포 (설계 근거)

Standard 정책을 통과한 관측의 실제 분포다. Grid는 이 분포를 보고 정했다.

| 지표 | min | p25 | 중앙 | p75 | p90 | max |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Cube PnP RMSE (px) | 0.170 | 0.424 | 0.515 | 0.673 | 0.784 | **2.100** |
| Cube inlier fraction | **0.875** | 1.000 | 1.000 | 1.000 | 1.000 | 1.000 |
| Board ChArUco corners | 11 | 28 | 48 | — | — | 60 |

여기서 확인된 사실: 현재 `standard`(RMSE ≤ 3.0)와 `strict`(≤ 2.0)는 **cube 관측을 각각 0개, 1개만 제거한다.**
즉 지금까지의 standard/strict 대조는 threshold 민감도 실험이 될 수 없었다.

## 3. 설계: One-Factor-At-A-Time (OFAT)

기준점 `P0` = 현재 standard = (cube RMSE ≤ 3.0, inlier ≥ 0.0, board corner ≥ 4).
각 지점은 **한 축만** P0에서 움직인다. 총 10개 지점.

| ID | Cube RMSE ≤ (px) | Cube inlier ≥ | Board corner ≥ | 움직인 축 |
| --- | ---: | ---: | ---: | --- |
| P0 | 3.0 | 0.00 | 4 | (기준점) |
| R1 | 1.5 | 0.00 | 4 | Cube RMSE |
| R2 | 1.0 | 0.00 | 4 | Cube RMSE |
| R3 | 0.7 | 0.00 | 4 | Cube RMSE |
| C1 | 3.0 | 0.00 | 12 | Board corner |
| C2 | 3.0 | 0.00 | 20 | Board corner |
| C3 | 3.0 | 0.00 | 28 | Board corner |
| C4 | 3.0 | 0.00 | 36 | Board corner |
| I1 | 3.0 | 0.90 | 4 | Cube inlier |
| I2 | 3.0 | 1.00 | 4 | Cube inlier |

## 4. 공정성 계약 (핵심)

threshold를 세게 걸면 **held-out 관측도 함께 지워진다.** 그러면 "어려운 시험 문제를 지워서"
held-out 오차가 좋아 보일 수 있다. 이 실험은 그 교란을 다음으로 차단한다.

1. **Hard rejection은 train event에만 적용한다.** Held-out event의 관측 집합은 모든 지점에서
   P0 population으로 **동결**한다. 따라서 모든 지점이 **같은 시험지**로 채점된다.
2. **Split 동결 검사.** 각 지점의 `split.train_events` / `split.test_events`가 P0의 것과
   완전히 같은지 확인한다. 다르면 그 지점은 비교 불가로 **표시하고 순위에서 제외한다**
   (조용히 제거하지 않는다).
3. **Held-out population 동결 검사.** 각 지점의 held-out 관측 수·corner 수가 P0와 같은지 확인한다.
4. Threshold 외의 모든 것(seed `20260731`, test_fraction `0.2`,
   `min_train_eih_cube_events=3`, include_sets `0-12`, loss `soft_l1`, num_inits, 물리 scale
   nominal 1.0)은 고정한다.

## 5. 적합 대상 row

`A0`, `A2`, `A3`. A4/B1/B2는 실측 covariance가 없어 Simulation prior preflight이므로
threshold 결론의 근거로 쓰지 않는다.

## 6. 사전 지정 endpoint

- **Primary**: row별 `heldout_overall_reprojection_rmse_px` (동결된 held-out 집합 기준).
- **Secondary**: `heldout_board_reprojection_rmse_px`, `heldout_cube_reprojection_rmse_px`,
  그리고 `heldout_path_metrics`의 `cross_view_pixel_transfer_rmse_px`,
  `e_cross_translation_rmse_mm`, `e_e2e_translation_rmse_mm`.
- **비용 지표**: 각 지점에서 남은 train 관측 수와 corner 수.

모든 값은 canonical Table 1 CSV와 동일하게 `num_inits` run의 평균으로 집계한다.

Secondary 중 cross-view/e2e 지표는 train anchor 집합이 함께 변하므로 population 수와
`evaluation_mask_sha256`를 항상 병기한다. 별도 `cross-target` 러너를 지점마다 다시 돌리는
대신 table1 run이 이미 계산한 held-out path metric을 쓴다 — 같은 적합 결과에서 나온 값이고,
지점마다 추가 실행을 하지 않으므로 비교가 더 좁게 통제된다.

## 7. 사전 지정 판정 규칙

- 어떤 지점이 `A0`, `A2`, `A3` **모두에서** primary endpoint를 P0 대비 **2% 이상** 낮추고,
  split·held-out 동결 검사를 통과하면 → 그 threshold를 새 canonical 후보로 보고한다.
- 그렇지 않으면 → **"현재 데이터에서 결과는 hard rejection threshold에 민감하지 않다"**로 보고하고
  canonical을 바꾸지 않는다.
- 어느 쪽이든 세 축의 곡선을 모두 싣는다. 좋아 보이는 지점만 고르지 않는다.

## 8. 재현

```bash
RB_ROBOT_POS_SCALE=1.0 PYTHONPATH= python3 tools/run_hard_threshold_sensitivity.py
```

Step2b 재검출이 결정론적임을 사전 확인했다(동일 인자 2회 실행 → manifest 해시 일치).
따라서 지점 간 차이는 threshold 효과이며 검출 재실행 잡음이 아니다.
