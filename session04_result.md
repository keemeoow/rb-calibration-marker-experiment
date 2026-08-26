# Session04 nominal 재캘리브레이션 결과

## 실행 계약

- 물리 config의 nominal metric scale `1.0`을 사용했습니다.
- 데이터에서 추정한 Board scale 정렬은 적용하지 않았습니다.
- `board_metric_scale.enabled=false`, `effective_square_length_mm=25.0`입니다.
- Step2b standard frozen manifest를 사용했으며 detector를 실험마다 다시 실행하지 않았습니다.
- set 최초 고정카메라 관측은 최적화 잔차에서 한 번만 사용했습니다.
- Gripper-to-Fixed 평가는 set 최초 fixed anchor를 같은 set의 모든 held-out gripper Event와 연결했습니다.
- 최종 Gripper-to-Fixed RMSE는 pair→Event→set→set 동일가중 순서로 집계했습니다.

## 평가 모집단

| 항목 | 결과 |
| --- | ---: |
| Eligible sets | 9개 (`4–12`) |
| Train | 119 observations / 3,702 corners |
| Held-out | 35 observations / 943 corners |
| Held-out gripper Events | 9개 |
| Board Gripper-to-Fixed | 23 pairs / 46 directions |
| Cube Gripper-to-Fixed | 27 pairs / 54 directions |

Held-out gripper Events는 `24, 33, 41, 45, 48, 56, 60, 70, 76`이며 모두
Gripper-to-Fixed 평가에 포함됐습니다. 각 set의 fixed anchor Event는
`24, 30, 36, 42, 48, 54, 60, 66, 72`입니다.

## Table 1 결과

| Method | Train px | Own held-out px | Board/Cube held-out px |
| --- | ---: | ---: | ---: |
| A0 | 3.8202 | 4.0530 | 4.0530 / N/A |
| A1 | 3.7844 | 4.0206 | 4.1007 / 3.7761 |
| A2 | 3.7513 | 3.9178 | 4.0415 / 3.5306 |
| A3 | 3.9474 | 3.8220 | 3.9644 / 3.3702 |
| A4 | 3.7532 | 3.9193 | 4.0481 / 3.5151 |
| B1 | 3.7851 | 4.0214 | 4.1047 / 3.7669 |
| B2 | 3.0882 | 4.5964 | N/A / 4.5964 |
| B3 | 3.8202 | 4.0530 | 4.0530 / N/A |

A3의 own held-out가 3.8220 px로 가장 낮지만, 외부 GT 순위가 아닙니다.
A4/B1/B2는 실제 FK covariance가 없는 simulation-prior 예비실험입니다.

## Set-anchor Gripper-to-Fixed 결과

| Method | Board px / mm / deg | Cube px / mm / deg |
| --- | ---: | ---: |
| A0 | 4.5444 / 5.4810 / 0.7966 | 10.2456 / 11.2086 / 1.5669 |
| A1 | 5.0193 / 6.4169 / 0.9736 | 9.6380 / 10.7764 / 1.6304 |
| A2 | 5.4411 / 7.2118 / 1.0295 | 8.9883 / 9.8188 / 1.6354 |
| A3 | 6.0437 / 7.3039 / 0.7528 | 7.9345 / 8.6983 / 1.4928 |
| A4 | 5.4698 / 7.2275 / 1.0129 | 8.9831 / 9.8212 / 1.6275 |
| B1 | 5.0473 / 6.4239 / 0.9741 | 9.6086 / 10.7567 / 1.6315 |
| B2 | 6.1255 / 8.6949 / 1.0881 | 8.5233 / 9.1899 / 1.6669 |
| B3 | 4.5446 / 5.4824 / 0.7967 | 10.2444 / 11.2069 / 1.5670 |

Board와 Cube 지표의 최저 방법이 다릅니다. Board 쪽은 A0/B3가 낮고 Cube 쪽은
A3가 낮으므로, 외부 GT 전에는 하나를 절대적인 최종 우승 방법으로 선택하지 않습니다.

## OpenCV hand-eye 7종 비교

동일 split과 set-anchor mask로 Tsai–Lenz, Park–Martin, Horaud, Andreff,
Daniilidis, Shah, Li를 추가 실행했습니다. OpenCV 계열에서는 Shah
robot-world/hand-eye가 Gripper-to-Fixed Board/Cube pixel 기준으로 각각
`4.2784 / 9.5130 px`였습니다. 현재 A3는 `6.0437 / 7.9345 px`로, Shah보다
Cube pixel은 16.6% 낮지만 Board pixel은 41.3% 높았습니다. 이는 단일 승자보다
Board–Cube 모델/측정 불일치와 절충이 남아 있음을 보여줍니다.

상세 표와 실행 계약은
[Session04 hand-eye 방법 비교](CP_result/session04/handeye_method_comparison/README.md)에
정리했습니다.

## 남아 있는 문제

Nominal geometry에서 Board와 Cube가 계산한 고정카메라 상대 위치는 Camera 1에서
19.423 mm, Camera 3에서 14.875 mm 차이가 납니다. 이는 여전히 조사 대상이지만,
실물 Board/Cube 치수를 측정하지 않았으므로 어느 geometry가 잘못됐다고 확정하거나
추정 scale을 공식 결과에 적용하지 않았습니다.

## 결과 파일

- [전체 Table 1 및 카메라 범위 평가](session04_result_table1.md)
- [Interactive 결과](_TABLE1_INTERACTIVE_session04.html)
- [Canonical CSV](CP_result/session04/late_table1/table1_results.csv)
- [방법·seed별 raw JSON](CP_result/session04/late_table1/table1_methods.json)
- [Cross-target raw JSON](CP_result/session04/cross_target_evaluation/cross_target_evaluation.json)
- [Marker-system 결과](CP_result/session04/marker_system_end_to_end/marker_system_end_to_end.csv)
- [OpenCV nominal 기준선](CP_result/session04/opencv_relative_baseline/OPENCV_RELATIVE_BASELINE.md)
- [OpenCV hand-eye/robot-world 7종 비교](CP_result/session04/handeye_method_comparison/README.md)
- [Soft-L1/linear 비교](CP_result/session04/outlier_ablation/OUTLIER_LOSS_ABLATION.md)
- [Step2b 관측 필터](data/session04/calib_out/capture_filter/CAPTURE_FILTER.md)

## 검증

- 전체 pytest: 16 passed
- Session04 set-anchor support 계약: passed
- Fixed-to-Fixed / Gripper-to-Fixed synthetic 계약: passed
- JSON·CSV·Markdown·HTML 동기화: passed
- 공식 보고서 nominal-scale 강제 검사: passed

이 결과는 외부 GT 전 내부 일관성 평가이며 절대 물리 정확도가 아닙니다.
