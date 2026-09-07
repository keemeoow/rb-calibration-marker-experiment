# Robot-base Point-cloud Diagnostic

> **역할:** 8/3 피드백 #17에 대한 현재 데이터 기반 산출물이다. aligned depth를 각 Table 1 비교실험 row의 calibration transform으로 robot-base frame에 올려 정합을 시각화한다. 이 depth는 캘리브레이션 목적함수나 external GT가 아니므로 absolute robot-task accuracy로 해석하지 않는다.

## Comparison-row Summary

| Method | Experiment role | Targets | Obs | Depth samples | Board RMSE | Cube RMSE | Combined RMSE |
| --- | --- | --- | ---: | ---: | ---: | ---: | ---: |
| A0 | board-only sequential baseline | board | 12 | 8777 | 9.544 mm | N/A | 9.544 mm |
| A1 | board+cube sequential | board+cube | 24 | 11837 | 9.339 mm | 9.649 mm | 9.420 mm |
| A2 | board+cube unified internal main | board+cube | 24 | 11837 | 9.042 mm | 8.580 mm | 8.925 mm |
| A3 | raw-FK hard fixed diagnostic | board+cube | 24 | 11837 | 9.494 mm | 9.890 mm | 9.598 mm |
| A4 | soft-FK preflight | board+cube | 24 | 11837 | 9.027 mm | 8.569 mm | 8.911 mm |
| A5 | vision-aligned FK hard fixed | board+cube | 24 | 11837 | 8.996 mm | 7.953 mm | 8.738 mm |
| B1 | -Unified soft-FK baseline | board+cube | 24 | 11837 | 9.331 mm | 9.628 mm | 9.409 mm |
| B2 | -board cube-only soft-FK | cube | 12 | 3060 | N/A | 8.010 mm | 8.010 mm |
| B3 | -cube board-only unified | board | 12 | 8777 | 9.543 mm | N/A | 9.543 mm |

## Target-level Detail

| Method | Target | Obs | Depth samples | Median | RMSE | P95 |
| --- | --- | ---: | ---: | ---: | ---: | ---: |
| A0 | board | 12 | 8777 | 8.086 mm | 9.544 mm | 13.378 mm |
| A1 | board | 12 | 8777 | 7.861 mm | 9.339 mm | 13.296 mm |
| A1 | cube | 12 | 3060 | 9.119 mm | 9.649 mm | 11.870 mm |
| A2 | board | 12 | 8777 | 7.459 mm | 9.042 mm | 12.825 mm |
| A2 | cube | 12 | 3060 | 8.068 mm | 8.580 mm | 10.675 mm |
| A3 | board | 12 | 8777 | 8.054 mm | 9.494 mm | 13.713 mm |
| A3 | cube | 12 | 3060 | 8.926 mm | 9.890 mm | 12.976 mm |
| A4 | board | 12 | 8777 | 7.448 mm | 9.027 mm | 12.807 mm |
| A4 | cube | 12 | 3060 | 8.079 mm | 8.569 mm | 10.690 mm |
| A5 | board | 12 | 8777 | 7.309 mm | 8.996 mm | 12.651 mm |
| A5 | cube | 12 | 3060 | 7.367 mm | 7.953 mm | 10.133 mm |
| B1 | board | 12 | 8777 | 7.857 mm | 9.331 mm | 13.289 mm |
| B1 | cube | 12 | 3060 | 9.158 mm | 9.628 mm | 11.842 mm |
| B2 | cube | 12 | 3060 | 7.498 mm | 8.010 mm | 10.294 mm |
| B3 | board | 12 | 8777 | 8.085 mm | 9.543 mm | 13.378 mm |

## Visual Evidence

### A0 — board-only sequential baseline

#### Event 0024

![Robot-base point cloud A0 event 0024](robot_base_pointcloud_A0_event0024.png)

#### Event 0054

![Robot-base point cloud A0 event 0054](robot_base_pointcloud_A0_event0054.png)

#### Event 0072

![Robot-base point cloud A0 event 0072](robot_base_pointcloud_A0_event0072.png)

### A1 — board+cube sequential

#### Event 0024

![Robot-base point cloud A1 event 0024](robot_base_pointcloud_A1_event0024.png)

#### Event 0054

![Robot-base point cloud A1 event 0054](robot_base_pointcloud_A1_event0054.png)

#### Event 0072

![Robot-base point cloud A1 event 0072](robot_base_pointcloud_A1_event0072.png)

### A2 — board+cube unified internal main

#### Event 0024

![Robot-base point cloud A2 event 0024](robot_base_pointcloud_A2_event0024.png)

#### Event 0054

![Robot-base point cloud A2 event 0054](robot_base_pointcloud_A2_event0054.png)

#### Event 0072

![Robot-base point cloud A2 event 0072](robot_base_pointcloud_A2_event0072.png)

### A3 — raw-FK hard fixed diagnostic

#### Event 0024

![Robot-base point cloud A3 event 0024](robot_base_pointcloud_A3_event0024.png)

#### Event 0054

![Robot-base point cloud A3 event 0054](robot_base_pointcloud_A3_event0054.png)

#### Event 0072

![Robot-base point cloud A3 event 0072](robot_base_pointcloud_A3_event0072.png)

### A4 — soft-FK preflight

#### Event 0024

![Robot-base point cloud A4 event 0024](robot_base_pointcloud_A4_event0024.png)

#### Event 0054

![Robot-base point cloud A4 event 0054](robot_base_pointcloud_A4_event0054.png)

#### Event 0072

![Robot-base point cloud A4 event 0072](robot_base_pointcloud_A4_event0072.png)

### A5 — vision-aligned FK hard fixed

#### Event 0024

![Robot-base point cloud A5 event 0024](robot_base_pointcloud_A5_event0024.png)

#### Event 0054

![Robot-base point cloud A5 event 0054](robot_base_pointcloud_A5_event0054.png)

#### Event 0072

![Robot-base point cloud A5 event 0072](robot_base_pointcloud_A5_event0072.png)

### B1 — -Unified soft-FK baseline

#### Event 0024

![Robot-base point cloud B1 event 0024](robot_base_pointcloud_B1_event0024.png)

#### Event 0054

![Robot-base point cloud B1 event 0054](robot_base_pointcloud_B1_event0054.png)

#### Event 0072

![Robot-base point cloud B1 event 0072](robot_base_pointcloud_B1_event0072.png)

### B2 — -board cube-only soft-FK

#### Event 0024

![Robot-base point cloud B2 event 0024](robot_base_pointcloud_B2_event0024.png)

#### Event 0054

![Robot-base point cloud B2 event 0054](robot_base_pointcloud_B2_event0054.png)

#### Event 0072

![Robot-base point cloud B2 event 0072](robot_base_pointcloud_B2_event0072.png)

### B3 — -cube board-only unified

#### Event 0024

![Robot-base point cloud B3 event 0024](robot_base_pointcloud_B3_event0024.png)

#### Event 0054

![Robot-base point cloud B3 event 0054](robot_base_pointcloud_B3_event0054.png)

#### Event 0072

![Robot-base point cloud B3 event 0072](robot_base_pointcloud_B3_event0072.png)

## Reading Rules

- 각 row는 Table 1의 marker 구성만 평가한다. A0/B3는 board-only, B2는 cube-only다.
- 이 표의 RMSE는 selected target polygon 내부 aligned depth point와 해당 row가 예측한 target plane 사이의 robot-base frame distance다.
- `Combined RMSE`는 board/cube sample 수로 가중한 row 내부 진단값이며, external GT 순위가 아니다.

## Interpretation

- 현재 산출물은 point cloud를 **카메라 좌표가 아니라 robot-base 좌표계**에서 표현한다.
- 검은 outline은 각 row의 visual calibration이 예측한 board/cube plane이고, 색 점은 실제 aligned depth point다.
- 큰 깊이/plane 차이는 external GT 오차가 아니라 depth registration, target localization, intrinsic coverage, target surface sampling이 섞인 진단 신호다.
- 따라서 #17은 `비교실험 구성별 diagnostic 구현 완료 / physical GT 아님`으로 상태를 둔다.
