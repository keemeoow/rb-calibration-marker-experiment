# Calibration → Table 1 실행 순서

메인 실행 파일은 저장소 root의 `01_...py`부터 `06_...py`까지다. 01~05가
입력 준비와 calibration을 수행하고, 06은 05 결과만 읽어 상세 보고서를 만든다.
Cross-target·marker-system·OpenCV baseline은 calibration 완료에 필요하지 않아
`tools/`의 선택 평가로 분리했다.

## 전체 명령 순서

새 촬영에서는 `session04`를 자동 생성된 `sessionNN`으로, `include_sets`를 실제
배치 set 범위로 바꾼다.

```bash
# 01 — RealSense factory intrinsic과 depth calibration 저장
python3 01_export_intrinsics.py \
  --out_dir intrinsics \
  --color_w 1280 --color_h 720 --fps 15

# 02 — ChArUco로 color intrinsic 정밀 보정
python3 02_calibrate_intrinsics.py \
  --intr_dir intrinsics \
  --min_views 12 \
  --save_images

# 03 — RGB-D + robot FK calibration 데이터 촬영
python3 03_capture.py \
  --data_root data \
  --intrinsics_dir intrinsics \
  --use_robot \
  --robot_ip 192.168.0.23 \
  --robot_port 12348 \
  --show

# 04 — 저장 영상 전체 재검출 및 관측 manifest 고정
python3 04_filter_observations.py \
  --session-root data/session04/calib_train \
  --intrinsics-dir intrinsics

# 05 — A0~A5/B1~B3 calibration + frame-prune/refit/rollback
python3 05_calibrate.py \
  --root_folder data/session04/calib_train \
  --intrinsics_dir intrinsics \
  --include_sets 0-12 \
  --split_seed 20260731 \
  --num_inits 3 \
  --observation-manifest data/session04/calib_out/capture_filter/Step2b_observation_manifest.json \
  --out_dir CP_result/session04/late_table1

# 06 — 05 결과만으로 상세 결과·전체 calibration 행렬 출력
python3 06_make_report.py \
  --root_folder data/session04/calib_train \
  --table1 CP_result/session04/late_table1/table1_methods.json \
  --out_dir CP_result/session04/late_table1
```

## 단계별 입력 · 과정 · 결과

| 단계 | 입력 | 과정 | 결과 |
| --- | --- | --- | --- |
| 01 `export_intrinsics` | 연결된 RealSense, 스트림 설정 | serial 순으로 camera ID를 고정하고 factory intrinsic/extrinsic 및 depth scale을 읽는다 | `intrinsics/device_map.json`, `depth_scales.json`, `cam*.npz` |
| 02 `calibrate_intrinsics` | 01 결과, ChArUco board | 다양한 위치의 view로 OpenCV color intrinsic calibration을 수행한다 | 갱신된 `cam*.npz`, `factory_backup/`, `charuco_capture/` |
| 03 `capture` | 02 결과, board/cube, 카메라, robot FK | 프레임을 동기화하고 품질 gate를 통과한 event의 영상·로봇 pose를 저장한다 | `data/sessionNN/calib_train/meta.json`, RGB/depth 이미지 |
| 04 `filter_observations` | 03 세션, 고정 K/D | 모든 RGB를 다시 검출하고 관측 정책을 적용해 native-pixel corner와 원본 SHA-256을 고정한다 | `Step2b_observation_manifest.json`, QA CSV, overlay, `CAPTURE_FILTER.md` |
| 05 `calibrate` | 04 manifest, K/D, `meta.json`, robot FK | event 단위 train/held-out 분리, 공통 초기화, 9개 조건 fit, `frame-prune → refit → rollback`, held-out 평가 | `table1_methods.json`, 두 shared artifact |
| 06 `make_report` | 05의 `table1_methods.json` | 재최적화 없이 수렴·오차·prune 결정과 모든 행렬을 정리한다 | `CALIBRATION_RESULTS.md`, `calibration_summary.csv`, `calibration_matrices.json` |

## 각 calibration 행렬은 언제 나오는가

행렬 표기 `T_A_B`는 **B 좌표의 점을 A 좌표로 변환**한다. 모든 4×4 SE(3)
행렬의 translation 단위는 meter다.

| 시점 | 행렬 | 생성 방식 | 저장 위치 / 최종 여부 |
| --- | --- | --- | --- |
| 01 | `color_K`, `color_D`, `depth_K`, `depth_D`, `R_depth_to_color`, `t_depth_to_color` | RealSense factory calibration을 읽음 | `intrinsics/cam*.npz`; intrinsic 초기값 |
| 02 | refined `color_K`, `color_D` | ChArUco view로 재추정 | 같은 `cam*.npz` 갱신; **05에서 고정 사용** |
| 03 | event별 `T_base_gripper`, set별 raw cube-center pose | robot controller FK/기록값 | `calib_train/meta.json`; optimizer 입력이며 calibration 결과 아님 |
| 04 | board/cube PnP pose | 검출 품질과 positive-depth 확인을 위한 임시 solvePnP | 최종 calibration 행렬로 전달하지 않음 |
| 05 공통 초기화 | `shared_reference_state`, `row_reference_states` | train 관측만 사용한 PnP/robust pose 초기화 | `shared_train_only_baseline.json`; optimizer 시작점 |
| 05 FK 정렬 | `T_gripper_cam`, `T_fk_cube_center_to_tag_object`, `raw_fk_pose_by_set`, `aligned_fk_pose_by_set` | train-only board-free FK–cube alignment | `shared_board_free_fk_cube.json`; A4/A5/B1/B2 입력 |
| 05 각 행·seed 종료 | `T_base_Ci`, `T_gripper_cam`, `T_base_board`, `T_base_cube_by_set` | raw-corner reprojection fit 후 prune/refit 결과가 개선되면 채택, 아니면 첫 fit으로 rollback | `table1_methods.json → rows.<행>.runs[*].transforms`; **최종값** |
| 06 | 새 행렬 없음 | 05의 최종값을 사람이 읽는 문서와 독립 JSON으로 복사·요약 | `calibration_matrices.json`에 9행×3 seed 전체 보존 |

최종 배포 대상은 `T_base_Ci = T^B_Ci`와 `T_gripper_cam = T^G_C`다.
`T_base_board`와 `T_base_cube_by_set`은 카메라들을 같은 좌표계로 묶는 target pose라서
camera calibration 배포 파일과 구분한다. 대표 행렬은 held-out 점수로 고르지 않고,
사전에 고정된 unperturbed initialization인 seed 0을 표시한다.

## 05 결과를 읽는 정확한 위치

```text
table1_methods.json
└── rows
    └── A0 ... A5, B1 ... B3
        └── runs[seed 0, 1, 2]
            ├── converged
            ├── stages.*.frame_prune_refit
            ├── train_reprojection.overall.rmse_px
            ├── heldout_reprojection.overall.rmse_px
            └── transforms
                ├── T_base_Ci.{0,1,3}
                ├── T_gripper_cam
                ├── T_base_board
                └── T_base_cube_by_set.<set>
```

`frame_prune_refit.accepted=true`이면 제거 후 재적합 행렬이 최종값이다.
`rolled_back=true`이면 전체 train robust objective가 개선되지 않아 제거 전 행렬이 최종값이다.
둘 다 정상 종료이며, 06 보고서에서 행별 시도/채택/rollback 수를 확인할 수 있다.

## 선택 평가 — calibration 완료 후 필요할 때만

```bash
# 동일 frozen split의 fixed↔fixed / gripper↔fixed 내부 경로 평가
python3 tools/evaluate_cross_target.py --root_folder data/session04/calib_train

# board-only / cube-only / both marker-system end-to-end 비교
python3 tools/compare_markers.py --root_folder data/session04/calib_train

# FK-free OpenCV fixed-camera relative-pose 기준선
python3 tools/opencv_baseline.py --root_folder data/session04/calib_train
```

이 세 평가는 05의 calibration 행렬을 만드는 필수 단계가 아니다. 과거의 통합
Table 1 Markdown/HTML을 갱신할 때만 `tools/sync_table1_canonical_data.py`를 사용한다.

## 현재 Session04 결과 위치

- 상세 결과와 seed 0 행렬: `CP_result/session04/late_table1/CALIBRATION_RESULTS.md`
- 모든 행·seed의 정확한 행렬: `CP_result/session04/late_table1/calibration_matrices.json`
- 행별 수렴·오차·prune 요약: `CP_result/session04/late_table1/calibration_summary.csv`
- 계산 원본: `CP_result/session04/late_table1/table1_methods.json`
- 기존 확장 평가 표: `CP_result/session04/late_table1/TABLE1_RESULTS.md`

Session04 현재 결과는 9개 행×3개 seed 모두 수렴했다. Held-out reprojection은
행마다 marker 모집단이 달라 외부 절대 정확도나 전체 방법 순위로 해석하면 안 된다.
외부 GT와 robot task가 없으므로 실제 robot-base 절대 정확도는 후속 검증 대상이다.

COLMAP/MATLAB, robot task, point cloud, 신규 촬영/외부 GT는 현재 calibration 완료
범위에서 제외하고 후속 작업으로 유지한다.
