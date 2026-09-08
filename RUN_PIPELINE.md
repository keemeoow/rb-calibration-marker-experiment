# Calibration → Table 1 실행 순서

메인 실행 파일은 저장소 root의 `01_...py`부터 `06_...py`까지다. 01~05가
입력 준비와 calibration을 수행하고, 06은 05 결과만 읽어 CSV와 행렬 JSON을 만든다.
Cross-target·marker-system·OpenCV baseline은 calibration 완료에 필요하지 않아
`tools/`의 선택 평가로 분리했다.

## 촬영 프로토콜과 현재 구현 상태

최종 비교실험의 촬영 기준은 [CAPTURE_PROTOCOL.md](CAPTURE_PROTOCOL.md) 한 문서로
고정한다.

| 구분 | 상태 | 사용 범위 |
| --- | --- | --- |
| 기존 `03_capture.py` | legacy 구현 | 기존 `data/session04` 재현 및 진단 |
| 최종 composite-target protocol | capture 통신 구현 완료, pose 티칭·dry run 전 | 새 A0~A5/B1~B3 calibration session |

최종 프로토콜은 board와 cube를 하나의 강체 rig로 만들고, 모든 방법이 같은
`P1 15 + P2 20 + P3 10 = 45` planned event를 사용한다. 최종 protocol mode에서는
모든 camera와 robot/release state를 저장하고, marker gate는 진단으로만 사용한다.
실제 좌표를 채운 pose plan과 rig geometry 검증, 저속 dry run을 통과한 뒤 본 촬영한다.

## 현재 Session04 / legacy 명령 순서

아래 명령은 현재 코드와 기존 Session04 artifact를 재현하는 순서다. 새 최종 촬영
명령은 이 legacy 블록과 섞지 않고 아래 `최종 촬영 흐름` 절에서 관리한다.

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

# 03 — Legacy RGB-D + robot FK 촬영; 최종 45-event 촬영에는 사용 금지
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
  --out_dir CP_result/session04/calib_result_table1

# 06 — 05 결과만으로 요약 CSV·전체 calibration 행렬 출력
python3 06_make_report.py \
  --root_folder data/session04/calib_train \
  --table1 CP_result/session04/calib_result_table1/table1_methods.json \
  --out_dir CP_result/session04/calib_result_table1
```

## 단계별 입력 · 과정 · 결과

| 단계 | 입력 | 과정 | 결과 |
| --- | --- | --- | --- |
| 01 `export_intrinsics` | 연결된 RealSense, 스트림 설정 | serial 순으로 camera ID를 고정하고 factory intrinsic/extrinsic 및 depth scale을 읽는다 | `intrinsics/device_map.json`, `depth_scales.json`, `cam*.npz` |
| 02 `calibrate_intrinsics` | 01 결과, ChArUco board | 다양한 위치의 view로 OpenCV color intrinsic calibration을 수행한다 | 갱신된 `cam*.npz`, `factory_backup/`, `charuco_capture/` |
| 03 `capture` (현재 legacy) | 02 결과, board/cube, 카메라, robot FK | `A_placement/B_eyetohand` block에서 동기화 및 marker quality gate를 통과한 event를 저장한다 | `data/sessionNN/calib_train/meta.json`, RGB/depth 이미지 |
| 04 `filter_observations` | 03 세션, 고정 K/D | 모든 RGB를 다시 검출하고 관측 정책을 적용해 native-pixel corner와 원본 SHA-256을 고정한다 | `Step2b_observation_manifest.json`, QA CSV, overlay, `CAPTURE_FILTER.md` |
| 05 `calibrate` | 04 manifest, K/D, `meta.json`, robot FK | event 단위 train/held-out 분리, 공통 초기화, 9개 조건 fit, `frame-prune → refit → rollback`, held-out 평가 | `table1_methods.json`, 두 shared artifact |
| 06 `make_report` | 05의 `table1_methods.json` | 재최적화 없이 수렴·오차·prune 결정과 모든 행렬을 정리한다 | `calibration_summary.csv`, `calibration_matrices.json` |

## 최종 촬영 흐름

최종 protocol mode의 03 단계는 아래 상태 순서를 강제한다.

| 순서 | Phase | Event 수 | 실행 내용 | 핵심 산출물 |
| ---: | --- | ---: | --- | --- |
| 1 | Preflight | 0 | rig geometry, camera ID/intrinsic, 45 pose ID, 저장 공간 검증 | protocol/config/geometry hash |
| 2 | P1 Moving Rig | 15 | rig를 계속 grasp한 채 `x/y/z + roll/pitch/yaw` 다양성으로 fixed camera 동기 촬영 | still-gripped `T_flange_rig` 입력 |
| 3 | P2 Pick-and-Place | 20 | 10회 release 후 stationary target을 두 gripper viewpoint에서 fixed+gripper camera 동기 촬영 | 10 placement × 2 views |
| 4 | P3 Stationary Rig | 10 | rig를 고정하고 robot/gripper camera만 10개 pose로 이동 | eye-in-hand excitation |
| 5 | Complete | 0 | 45 planned ID, attempt, 실패 상태, hash를 검증하고 완료 상태 기록 | completion manifest |

Release 직전 FK 기록은 P2 metadata이며 별도 capture event로 세지 않는다. Marker/PnP
실패도 planned event를 교체하는 근거가 아니다. Camera transport·파일 저장·timestamp
sync 실패만 같은 planned ID로 재시도하고, 분석에는 marker 결과와 무관하게 첫
transport/sync-valid attempt를 사용한다.

최종 세션에서는 모든 row가 같은 45개 event를 사용한다. A0/B3는 cube를 calibration
전체에서 masking하고, B2는 board를 masking한다. Heldout, cross-view, External GT는
모두 cube-only이며, External GT 촬영은 45개 calibration-train event에 포함하지 않는다.

Pose JSON 생성·필드 입력·offline 검증 방법은
[CAPTURE_PROTOCOL.md](CAPTURE_PROTOCOL.md#8-자동-pose-json-준비와-실행)를 따른다. 검증된
plan을 사용한 PC 명령은 다음과 같다.

```bash
python3 03_capture.py \
  --data_root data \
  --intrinsics_dir intrinsics \
  --waypoints_file capture_plans/composite_rig_45.json \
  --use_robot --manual_robot \
  --robot_ip 192.168.0.23 --robot_port 12348 \
  --max_capture_span_ms 120 \
  --show
```

## 촬영 코드 전환 순서

1. 완료: `server/c1.py`와 `capture_pipeline/capture.py`에 P1/P2/P3 자동 실행,
   planned event/attempt 분리, release state, all-camera 저장을 구현했다.
2. 완료: 45-event pose template 생성기와 offline validator를 추가했다.
3. 다음: `04_filter_observations.py`가 marker 실패 event도 보존하고, detection 결과와 capture
   validity를 분리한 manifest를 만든다.
4. 다음: Calibration runtime/schema가 phase, placement, marker mask를 검증하고 A3/A4/A5의
   raw/corrected/aligned FK factor를 연결한다.
5. 다음: Regression test가 정확히 45 ID, `15/20/10` phase count, sync-only retry, A0/B3/B2
   mask, heldout leakage 금지를 자동 확인한다.

본 촬영 전에는 실제 pose plan offline 검증과 confirm mode 저속 dry run을 통과해야 한다.
04/05의 phase-aware 처리가 완료되기 전에는 새 세션으로 최종 비교표를 생성하지 않는다.

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
| 06 | 새 행렬 없음 | 05의 최종값을 CSV와 독립 JSON으로 복사·요약 | `calibration_matrices.json`에 9행×3 seed 전체 보존 |

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
            ├── cube_evaluation_reprojection.heldout.cube.rmse_px
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
# 동일 frozen split의 cross-view camera consistency 평가
python3 tools/evaluate_cross_target.py --root_folder data/session04/calib_train

# board-only / cube-only / both marker-system end-to-end 비교
python3 tools/compare_markers.py --root_folder data/session04/calib_train

# FK-free OpenCV fixed-camera relative-pose 기준선
python3 tools/opencv_baseline.py --root_folder data/session04/calib_train
```

## 다른 checkout에서 재실행할 때

`Step2b_observation_manifest.json`은 촬영한 머신의 절대 경로를 기록한다. 다른
checkout에서 그대로 재실행하면 session root 불일치로 05가 중단된다. 이때
`--allow-relocated-session-root`를 05·`evaluate_cross_target`·`compare_markers`에
함께 준다. 이 플래그는 기록된 경로 접두사만 현재 checkout으로 옮기며, meta.json,
intrinsics, 모든 영상의 SHA-256 검증은 그대로 수행한다. 즉 무결성 계약은
경로 문자열이 아니라 해시가 계속 담당한다.

보조 평가 두 개는 `--include_sets`와 `--split_seed`를 05와 동일하게 주어야 한다.
기본값(`5-12`)으로 실행하면 split이 달라져 `reconstructed split does not match
stored results`로 중단된다.

이 세 평가는 05의 calibration 행렬을 만드는 필수 단계가 아니다. 과거의 통합
최종 Table 1 Markdown/HTML을 갱신할 때만 `tools/sync_table1_canonical_data.py`를 사용한다.

## 현재 Session04 결과 위치

- 모든 행·seed의 정확한 행렬: `CP_result/session04/calib_result_table1/calibration_matrices.json`
- 행별 수렴·오차·prune 요약: `CP_result/session04/calib_result_table1/calibration_summary.csv`
- 계산 원본: `CP_result/session04/calib_result_table1/table1_methods.json`
- 최종 비교실험표와 평가지표: `CP_result/session04/calib_result_table1/TABLE1_RESULTS.md`

Session04 현재 결과는 9개 행×3개 seed 모두 수렴했다. 최종 표의 Train Cube RMSE와
heldout은 `cube_evaluation_reprojection`의 cube-only 값으로 통일했으며,
camera/Hand–Eye는 frozen하고 train cube로 set별 evaluation pose만 맞춘다. Row별
marker 모집단이 다른 solver Train RMSE는 계산 원본에만 남고 최종 비교표에는 쓰지
않는다. 현재 보고서 생성 시점의
robot-base 절대 정확도는 계산하지 않고, 다음주
Independent External GT 태스크에서 Translation Error, Rotation Error, P95,
Failure Rate를 산출한다.

COLMAP/MATLAB과 point cloud는 현재 calibration 완료 범위에서 제외하고 후속
작업으로 유지한다. Robot task/외부 GT는 다음주 예정 태스크로 분리한다.
