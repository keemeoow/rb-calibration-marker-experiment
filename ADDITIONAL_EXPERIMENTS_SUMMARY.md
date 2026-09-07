# 추가 실험 단일 요약표

이 문서는 Session04에서 최종 A0~A5/B1~B3 비교표 외에 추가로 수행한 진단,
민감도, 시각화 실험을 한 곳에 모은 인덱스다. 최종 방법 순위는
[CALIBRATION_EXPERIMENT_VALIDATION.md](CALIBRATION_EXPERIMENT_VALIDATION.md)의
단일 기준을 따르며, 사람이 읽는 최종 결과표는
[TABLE1_RESULTS.md](CP_result/session04/late_table1/TABLE1_RESULTS.md) 하나만 사용한다.

삭제 정리: 오래된 overall/board-heldout 중심 생성물인
`CP_result/session04/late_table1/CALIBRATION_RESULTS.md`는 제거했다. 행렬 근거는
`CP_result/session04/late_table1/calibration_matrices.json`에 남기고, 최종 비교표는
`TABLE1_RESULTS.md`, `table1_results.csv`, `TABLE1_INTERACTIVE.html`로만 유지한다.

| 추가 실험 | 확인 질문 | 핵심 결과 | 최종 반영 | 근거 파일 |
| --- | --- | --- | --- | --- |
| Observation filter / cube quality | 현재 cube 관측 자체에 큰 데이터 문제가 있는가 | Standard filter 기준 cube 99개, board 147개 관측 선택. Cube quality audit에서는 117 images read, 108 accepted PnP, 99 core multiface, 2 PnP-RMSE rejections. | 데이터 문제는 관리 가능한 수준으로 보고, heldout은 항상 cube-only로 고정. | [CAPTURE_FILTER.md](data/session04/calib_out/capture_filter/CAPTURE_FILTER.md), [cube quality README](data/session04/calib_out/verify/cube_observation_quality/README.md) |
| Frame-prune / refit / rollback | 피드백처럼 outlier를 image-frame 단위로 제거하면 결과가 좋아지는가 | 42 solver stages 중 prune/refit 15회 시도, accepted 0회, rollback 15회. 제거 전 train robust objective가 더 좋아 canonical은 첫 fit 유지. | frame 단위 outlier 제거 코드는 유지하되, Session04 최종값은 rollback 결과로 사용. | [table1_methods.json](CP_result/session04/late_table1/table1_methods.json), [calibration_summary.csv](CP_result/session04/late_table1/calibration_summary.csv) |
| Hard rejection standard vs strict | 경계 cube 관측 2개를 빼면 결론이 바뀌는가 | Train 119 obs / 3686 corners -> 117 obs / 3654 corners. 당시 보조 diagnostic residual 변화는 최대 0.49% 수준이고, A1->A2 개선 방향 유지. | strict를 canonical로 채택하지 않고 standard filter 유지. 최종 heldout 판정은 cube-only 지표만 사용. | [HARD_REJECTION_ABLATION.md](CP_result/session04/outlier_ablation/HARD_REJECTION_ABLATION.md) |
| Hard threshold OFAT | threshold를 넓게 sweep하면 더 좋은 고정 정책이 있는가 | 사전등록 규칙상 어떤 지점도 A0/A2/A3 모두에서 2% 이상 개선하지 못함. C3는 A0 primary endpoint를 +118.6% 악화. | 관측을 더 버리는 정책은 공식 채택하지 않음. | [HARD_THRESHOLD_SENSITIVITY.md](CP_result/session04/outlier_ablation/hard_threshold_sensitivity/HARD_THRESHOLD_SENSITIVITY.md), [PREREGISTRATION_HARD_THRESHOLD.md](CP_result/session04/outlier_ablation/PREREGISTRATION_HARD_THRESHOLD.md) |
| Soft-L1 vs linear loss | robust loss가 임의 선택인지, 실제로 도움이 되는가 | soft-L1은 A0/A1/A2/A4/A5/B1/B2/B3에서 linear보다 낮고, A3만 +0.18% 악화. B2 개선폭은 8.02%. | canonical solver loss는 `soft_l1`, 단 robust loss가 모든 조건을 자동 개선한다고 주장하지 않음. | [OUTLIER_LOSS_ABLATION.md](CP_result/session04/outlier_ablation/OUTLIER_LOSS_ABLATION.md) |
| Corner refinement / weighting | line intersection 또는 equal-observation weighting으로 board-cube 충돌을 해결할 수 있는가 | line intersection은 conflict 10.8077 -> 8.5679 mm로 줄였지만 cube transfer 2.8820 -> 2.9411 px, A2 overall/e2e 악화. equal-observation은 A2 cube 3.5958 -> 3.8922 px로 악화. | `CORNER_REFINE_APRILTAG + per_corner` 유지. 단일 pixel scale/weight 보정보다는 반복촬영 covariance가 필요. | [CORNER_WEIGHTING_ABLATION.md](CP_result/session04/corner_weighting_ablation/CORNER_WEIGHTING_ABLATION.md) |
| Board-cube relative pose validation | 큰 오차가 software/config bug인지, target-dependent 문제인지 | detector refinement와 metadata 수정 후 conflict 17.299 -> 10.808 mm, cube transfer 4.247 -> 2.882 px. event 54 제거 시 9.599 mm로 줄지만 conflict가 남아 단일 프레임 문제는 아님. | 공식 calibration은 joint reprojection solve로 유지. direct-PnP conflict는 single-target extrinsic 공개 금지 gate로만 사용. | [BOARD_CUBE_RELATIVE_POSE.md](data/session04/calib_out/verify/board_cube_relative_pose/BOARD_CUBE_RELATIVE_POSE.md), [BOARD_CUBE_CONFLICT_TRIAGE.md](data/session04/calib_out/verify/board_cube_relative_pose/BOARD_CUBE_CONFLICT_TRIAGE.md) |
| OpenCV relative-pose baseline | custom optimizer만의 문제가 아닌지 독립 PnP로 확인했는가 | Board-only는 board transfer 1.4141 px / 1.9469 mm에서 좋고 cube에서는 나쁨. Cube-only는 cube transfer 2.8820 px / 3.5148 mm에서 좋고 board에서는 나쁨. naive average도 양쪽을 해결하지 못함. | public/SOTA 우월성 증거가 아니라, target-dependent discrepancy가 optimizer 하나의 버그가 아님을 보이는 reference로 사용. | [OPENCV_RELATIVE_BASELINE.md](CP_result/session04/opencv_relative_baseline/OPENCV_RELATIVE_BASELINE.md) |
| External baseline frozen package | MATLAB/COLMAP/OpenCV 계열 외부 구현에 같은 입력을 줄 수 있는가 | frozen input package 생성: 154 observations, 4625 corners, train 119/test 35, board 77/cube 77, fixed 50/gripper 104. | 외부 adapter가 detector/refinement를 새로 돌리지 못하게 하는 입력 계약으로 유지. | [EXTERNAL_BASELINE_PACKAGE.md](CP_result/session04/external_baseline_package/EXTERNAL_BASELINE_PACKAGE.md) |
| FK factor sensitivity | FK factor가 관측 수에 묻히는지, covariance scale에 반응하는가 | A4 std scale 0.25x~4.0x에서 보조 heldout residual 3.8684~3.8902 px. FK cost fraction 0.6540%~0.0141%, 출력은 작지만 실제로 움직임. | A4/B1/B2는 FK covariance 측정 전에는 preflight. 최종 FK claim은 External GT로 확정. | [FK_FACTOR_SENSITIVITY.md](CP_result/session04/fk_factor_sensitivity/FK_FACTOR_SENSITIVITY.md) |
| Robot-base point-cloud diagnostic | 비교실험별 3D 공간 정합을 시각적으로 볼 수 있는가 | A0~A5/B1~B3 모두 robot-base point cloud로 렌더링. Cube plane RMSE는 A5 7.953 mm, B2 8.010 mm, A4 8.569 mm, A2 8.580 mm 순. | 발표 증거 시각화로 사용하되, depth plane RMSE는 external GT/robot-task accuracy가 아님. | [ROBOT_BASE_POINTCLOUD_DIAGNOSTIC.md](CP_result/session04/robot_base_pointcloud/ROBOT_BASE_POINTCLOUD_DIAGNOSTIC.md) |
| Presentation evidence update | 피드백별 해결과 실제 데이터 시각화를 한눈에 보여줄 수 있는가 | 8-3 피드백 해결 발표자료를 유형별 묶음과 실제 overlay/point-cloud evidence 중심으로 재생성. | 발표용 요약으로 사용. 최종 수치는 `TABLE1_RESULTS.md`와 동기화. | [캘리브레이션_8-3_피드백_해결_시각화_발표자료.pdf](캘리브레이션_8-3_피드백_해결_시각화_발표자료.pdf) |

최종 판단 규칙은 간단하다. 내부 cube 지표와 위 추가 실험은 방법을 설명하고
실험 구조가 무너지지 않았는지 확인하는 근거다. 최종 “우리 방법이 실제 3D 공간에서
가장 좋다”는 주장은 다음주 Independent External cube GT의 TRE, rotation, P95,
failure rate가 들어온 뒤에만 확정한다.
