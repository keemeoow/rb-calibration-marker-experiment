# 04 Post-capture Observation Filter

- Session: `/home/sstone/rb-calibration-marker-experiment/data/session11_zeus_handheld_floor_wrist_meta_0909/calib_train`
- 생성 시각(UTC): `2026-09-14T07:53:59.427080+00:00`
- 원본 RGB/meta/intrinsics: **수정하지 않음**
- Calibration 입력: 재검출 결과의 native-pixel 2D corner를 manifest에 고정
- Cube corner refinement: `apriltag`

## 결과 요약

| 항목 | Standard | Strict |
|---|---:|---:|
| Cube 선택 관측 | 110 | 110 |
| Board 선택 관측 | 172 | 167 |
| Cube 재검출 복구 | 10 | 10 |

Cube standard disposition: `quarantine` 11, `recovered` 10, `rejected` 63, `selected` 100

## 정책

- `standard`: cube는 서로 다른 방향의 non-coplanar face 2개 이상, positive-depth PnP, RMSE ≤ 3.0 px. Board는 ChArUco corner ≥ 4.
- `strict`: 같은 기하 조건에 RMSE ≤ 2.0 px, inlier fraction ≥ 0.9, Board corner ≥ 12.
- `recovered`: 기본 검출은 core가 아니었지만 offline 재검출로 standard를 통과.
- `quarantine`: corner/PnP는 있으나 face 또는 임계값 부족. 자동 calibration에서 제외.
- `rejected`: marker 미검출, 영상 오류, PnP 실패/초과 등으로 frozen corner가 없음.

## 시각 검토

Standard의 recovered/quarantine/rejected cube 관측 48개를 한 장에 모았습니다.
초록은 recovered, 주황은 quarantine, 빨강은 rejected입니다. Rejected에 보라색 선이 있으면 촬영 당시 meta에 저장된 구형 검출 corner입니다.

![04 review overlay](Step2b_review_overlay.jpg)

## Standard 제외 cube 관측

| Event/camera | 결과 | Marker IDs | Faces | RMSE | 이유 |
|---|---|---|---|---:|---|
| E00/cam2 | quarantine | 2 | +X | 0.090 px | `noncore_single_marker` |
| E00/cam3 | rejected | — | — | — | `no_markers_detected` |
| E01/cam3 | rejected | — | — | — | `no_markers_detected` |
| E02/cam3 | rejected | — | — | — | `no_markers_detected` |
| E03/cam3 | rejected | — | — | — | `no_markers_detected` |
| E04/cam3 | rejected | — | — | — | `no_markers_detected` |
| E05/cam3 | rejected | — | — | — | `no_markers_detected` |
| E06/cam3 | rejected | — | — | — | `no_markers_detected` |
| E07/cam3 | rejected | — | — | — | `no_markers_detected` |
| E08/cam0 | rejected | — | — | — | `no_markers_detected` |
| E08/cam3 | rejected | — | — | — | `no_markers_detected` |
| E09/cam3 | rejected | — | — | — | `no_markers_detected` |
| E10/cam3 | rejected | — | — | — | `no_markers_detected` |
| E11/cam3 | rejected | — | — | — | `no_markers_detected` |
| E12/cam0 | rejected | — | — | — | `no_markers_detected` |
| E12/cam3 | rejected | — | — | — | `no_markers_detected` |
| E13/cam3 | rejected | — | — | — | `no_markers_detected` |
| E14/cam3 | rejected | — | — | — | `no_markers_detected` |
| E15/cam3 | rejected | — | — | — | `no_markers_detected` |
| E16/cam1 | quarantine | 3 | +Y | 0.043 px | `noncore_single_marker` |
| E17/cam3 | quarantine | 0, 1 | +Z | 0.204 px | `noncore_planar_multimarker` |
| E19/cam3 | quarantine | 0, 1 | +Z | 0.185 px | `noncore_planar_multimarker` |
| E20/cam1 | quarantine | 3 | +Y | 0.056 px | `noncore_single_marker` |
| E31/cam3 | quarantine | 0, 1 | +Z | 0.211 px | `noncore_planar_multimarker` |
| E36/cam0 | quarantine | 0, 1 | +Z | 0.322 px | `noncore_planar_multimarker` |
| E37/cam3 | quarantine | 0, 1 | +Z | 0.203 px | `noncore_planar_multimarker` |
| E40/cam1 | quarantine | 2 | +X | 0.017 px | `noncore_single_marker` |
| E43/cam3 | quarantine | 1, 0 | +Z | 0.184 px | `noncore_planar_multimarker` |
| E45/cam3 | quarantine | 1, 0 | +Z | 0.197 px | `noncore_planar_multimarker` |
| E46/cam1 | rejected | — | — | — | `no_markers_detected` |
| E46/cam2 | rejected | — | — | — | `no_markers_detected` |
| E46/cam3 | rejected | — | — | — | `no_markers_detected` |
| E47/cam1 | rejected | — | — | — | `no_markers_detected` |
| E47/cam2 | rejected | — | — | — | `no_markers_detected` |
| E47/cam3 | rejected | — | — | — | `no_markers_detected` |
| E48/cam1 | rejected | — | — | — | `no_markers_detected` |
| E48/cam2 | rejected | — | — | — | `no_markers_detected` |
| E48/cam3 | rejected | — | — | — | `no_markers_detected` |
| E49/cam1 | rejected | — | — | — | `no_markers_detected` |
| E49/cam2 | rejected | — | — | — | `no_markers_detected` |
| E49/cam3 | rejected | — | — | — | `no_markers_detected` |
| E50/cam1 | rejected | — | — | — | `no_markers_detected` |
| E50/cam2 | rejected | — | — | — | `no_markers_detected` |
| E50/cam3 | rejected | — | — | — | `no_markers_detected` |
| E51/cam1 | rejected | — | — | — | `no_markers_detected` |
| E51/cam2 | rejected | — | — | — | `no_markers_detected` |
| E51/cam3 | rejected | — | — | — | `no_markers_detected` |
| E52/cam1 | rejected | — | — | — | `no_markers_detected` |
| E52/cam2 | rejected | — | — | — | `no_markers_detected` |
| E52/cam3 | rejected | — | — | — | `no_markers_detected` |
| E53/cam1 | rejected | — | — | — | `no_markers_detected` |
| E53/cam2 | rejected | — | — | — | `no_markers_detected` |
| E53/cam3 | rejected | — | — | — | `no_markers_detected` |
| E54/cam1 | rejected | — | — | — | `no_markers_detected` |
| E54/cam2 | rejected | — | — | — | `no_markers_detected` |
| E54/cam3 | rejected | — | — | — | `no_markers_detected` |
| E55/cam1 | rejected | — | — | — | `no_markers_detected` |
| E55/cam2 | rejected | — | — | — | `no_markers_detected` |
| E55/cam3 | rejected | — | — | — | `no_markers_detected` |
| E56/cam1 | rejected | — | — | — | `no_markers_detected` |
| E56/cam2 | rejected | — | — | — | `no_markers_detected` |
| E56/cam3 | rejected | — | — | — | `no_markers_detected` |
| E57/cam1 | rejected | — | — | — | `no_markers_detected` |
| E57/cam2 | rejected | — | — | — | `no_markers_detected` |
| E57/cam3 | rejected | 2 | +X | 4370.142 px | `pnp_rmse_rejected` |
| E58/cam1 | rejected | — | — | — | `no_markers_detected` |
| E58/cam2 | rejected | — | — | — | `no_markers_detected` |
| E58/cam3 | rejected | — | — | — | `no_markers_detected` |
| E59/cam1 | rejected | — | — | — | `no_markers_detected` |
| E59/cam2 | rejected | — | — | — | `no_markers_detected` |
| E59/cam3 | rejected | — | — | — | `no_markers_detected` |
| E60/cam1 | rejected | — | — | — | `no_markers_detected` |
| E60/cam2 | rejected | — | — | — | `no_markers_detected` |
| E60/cam3 | rejected | — | — | — | `no_markers_detected` |

## Standard 제외 board 관측

| Event/camera | Corner 수 | 상태 | 이유 |
|---|---:|---|---|
| E00/cam1 | 0 | no_charuco_or_below_4_corners | `no_charuco_or_below_4_corners` |
| E00/cam3 | 0 | no_charuco_or_below_4_corners | `no_charuco_or_below_4_corners` |
| E02/cam1 | 0 | no_charuco_or_below_4_corners | `no_charuco_or_below_4_corners` |
| E02/cam3 | 0 | no_charuco_or_below_4_corners | `no_charuco_or_below_4_corners` |
| E03/cam3 | 0 | no_charuco_or_below_4_corners | `no_charuco_or_below_4_corners` |
| E04/cam3 | 0 | no_charuco_or_below_4_corners | `no_charuco_or_below_4_corners` |
| E08/cam1 | 0 | no_charuco_or_below_4_corners | `no_charuco_or_below_4_corners` |
| E08/cam3 | 0 | no_charuco_or_below_4_corners | `no_charuco_or_below_4_corners` |
| E11/cam3 | 0 | no_charuco_or_below_4_corners | `no_charuco_or_below_4_corners` |
| E12/cam3 | 0 | no_charuco_or_below_4_corners | `no_charuco_or_below_4_corners` |
| E17/cam3 | 0 | no_charuco_or_below_4_corners | `no_charuco_or_below_4_corners` |
| E18/cam1 | 0 | no_charuco_or_below_4_corners | `no_charuco_or_below_4_corners` |

## Strict에서 추가 제외되는 관측

Standard는 통과했지만 strict RMSE/inlier/board-corner 기준에서 추가 제외되는 관측입니다.

| Target | Event/camera | Corner 수 | RMSE | Inlier | 이유 |
|---|---|---:|---:|---:|---|
| board | E09/cam3 | 8 | — | — | `charuco_corners_below_12` |
| board | E15/cam3 | 7 | — | — | `charuco_corners_below_12` |
| board | E26/cam1 | 8 | — | — | `charuco_corners_below_12` |
| board | E34/cam2 | 10 | — | — | `charuco_corners_below_12` |
| board | E46/cam3 | 10 | — | — | `charuco_corners_below_12` |

## 재촬영 후보

현재 calibration 계약에서 cube를 사용하지 않는 gripped-cube event는 재촬영 후보에서 제외했습니다.

| Event | Set | 우선순위 | 남아 있는 board cameras | 이유 |
|---:|---:|---|---|---|
| 17 | 1 | high | — | neither standard core cube nor board observation is usable |
| 19 | 2 | medium | cam3 | missing_standard_core_cube; board observation remains usable |
| 31 | 8 | medium | cam3 | missing_standard_core_cube; board observation remains usable |
| 37 | 11 | medium | cam3 | missing_standard_core_cube; board observation remains usable |
| 43 | 14 | medium | cam3 | missing_standard_core_cube; board observation remains usable |
| 45 | 15 | medium | cam3 | missing_standard_core_cube; board observation remains usable |

## Event별 선택 결과

| Event | Set | Block | Standard | Cube cams | Board cams | Strict |
|---:|---:|---|---|---|---|---|
| 00 | None | B_eyetohand | selected_cube_and_board | cam0, cam1 | cam0, cam2 | selected_cube_and_board |
| 01 | None | B_eyetohand | selected_cube_and_board | cam0, cam1, cam2 | cam0, cam1, cam2, cam3 | selected_cube_and_board |
| 02 | None | B_eyetohand | selected_cube_and_board | cam0, cam1, cam2 | cam0, cam2 | selected_cube_and_board |
| 03 | None | B_eyetohand | selected_cube_and_board | cam0, cam1, cam2 | cam0, cam1, cam2 | selected_cube_and_board |
| 04 | None | B_eyetohand | selected_cube_and_board | cam0, cam1, cam2 | cam0, cam1, cam2 | selected_cube_and_board |
| 05 | None | B_eyetohand | selected_cube_and_board | cam0, cam1, cam2 | cam0, cam1, cam2, cam3 | selected_cube_and_board |
| 06 | None | B_eyetohand | selected_cube_and_board | cam0, cam1, cam2 | cam0, cam1, cam2, cam3 | selected_cube_and_board |
| 07 | None | B_eyetohand | selected_cube_and_board | cam0, cam1, cam2 | cam0, cam1, cam2, cam3 | selected_cube_and_board |
| 08 | None | B_eyetohand | selected_cube_and_board | cam1, cam2 | cam0, cam2 | selected_cube_and_board |
| 09 | None | B_eyetohand | selected_cube_and_board | cam0, cam1, cam2 | cam0, cam1, cam2, cam3 | selected_cube_and_board |
| 10 | None | B_eyetohand | selected_cube_and_board | cam0, cam1, cam2 | cam0, cam1, cam2, cam3 | selected_cube_and_board |
| 11 | None | B_eyetohand | selected_cube_and_board | cam0, cam1, cam2 | cam0, cam1, cam2 | selected_cube_and_board |
| 12 | None | B_eyetohand | selected_cube_and_board | cam1, cam2 | cam0, cam1, cam2 | selected_cube_and_board |
| 13 | None | B_eyetohand | selected_cube_and_board | cam0, cam1, cam2 | cam0, cam1, cam2, cam3 | selected_cube_and_board |
| 14 | None | B_eyetohand | selected_cube_and_board | cam0, cam1, cam2 | cam0, cam1, cam2, cam3 | selected_cube_and_board |
| 15 | None | B_eyetohand | selected_cube_and_board | cam0, cam1, cam2 | cam0, cam1, cam2, cam3 | selected_cube_and_board |
| 16 | 1 | A_placement | selected_cube_and_board | cam0, cam2 | cam0, cam1, cam2 | selected_cube_and_board |
| 17 | 1 | A_placement | rejected | — | — | rejected |
| 18 | 2 | A_placement | selected_cube_and_board | cam0, cam1, cam2 | cam0, cam2 | selected_cube_and_board |
| 19 | 2 | A_placement | board_only | — | cam3 | board_only |
| 20 | 3 | A_placement | selected_cube_and_board | cam0, cam2 | cam0, cam1, cam2 | selected_cube_and_board |
| 21 | 3 | A_placement | selected_cube_and_board | cam3 | cam3 | selected_cube_and_board |
| 22 | 4 | A_placement | selected_cube_and_board | cam0, cam1, cam2 | cam0, cam1, cam2 | selected_cube_and_board |
| 23 | 4 | A_placement | selected_cube_and_board | cam3 | cam3 | selected_cube_and_board |
| 24 | 5 | A_placement | selected_cube_and_board | cam0, cam1, cam2 | cam0, cam1, cam2 | selected_cube_and_board |
| 25 | 5 | A_placement | selected_cube_and_board | cam3 | cam3 | selected_cube_and_board |
| 26 | 6 | A_placement | selected_cube_and_board | cam0, cam1, cam2 | cam0, cam1, cam2 | selected_cube_and_board |
| 27 | 6 | A_placement | selected_cube_and_board | cam3 | cam3 | selected_cube_and_board |
| 28 | 7 | A_placement | selected_cube_and_board | cam0, cam1, cam2 | cam0, cam1, cam2 | selected_cube_and_board |
| 29 | 7 | A_placement | selected_cube_and_board | cam3 | cam3 | selected_cube_and_board |
| 30 | 8 | A_placement | selected_cube_and_board | cam0, cam1, cam2 | cam0, cam1, cam2 | selected_cube_and_board |
| 31 | 8 | A_placement | board_only | — | cam3 | board_only |
| 32 | 9 | A_placement | selected_cube_and_board | cam0, cam1, cam2 | cam0, cam1, cam2 | selected_cube_and_board |
| 33 | 9 | A_placement | selected_cube_and_board | cam3 | cam3 | selected_cube_and_board |
| 34 | 10 | A_placement | selected_cube_and_board | cam0, cam1, cam2 | cam0, cam1, cam2 | selected_cube_and_board |
| 35 | 10 | A_placement | selected_cube_and_board | cam3 | cam3 | selected_cube_and_board |
| 36 | 11 | A_placement | selected_cube_and_board | cam1, cam2 | cam0, cam1, cam2 | selected_cube_and_board |
| 37 | 11 | A_placement | board_only | — | cam3 | board_only |
| 38 | 12 | A_placement | selected_cube_and_board | cam0, cam1, cam2 | cam0, cam1, cam2 | selected_cube_and_board |
| 39 | 12 | A_placement | selected_cube_and_board | cam3 | cam3 | selected_cube_and_board |
| 40 | 13 | A_placement | selected_cube_and_board | cam0, cam2 | cam0, cam1, cam2 | selected_cube_and_board |
| 41 | 13 | A_placement | selected_cube_and_board | cam3 | cam3 | selected_cube_and_board |
| 42 | 14 | A_placement | selected_cube_and_board | cam0, cam1, cam2 | cam0, cam1, cam2 | selected_cube_and_board |
| 43 | 14 | A_placement | board_only | — | cam3 | board_only |
| 44 | 15 | A_placement | selected_cube_and_board | cam0, cam1, cam2 | cam0, cam1, cam2 | selected_cube_and_board |
| 45 | 15 | A_placement | board_only | — | cam3 | board_only |
| 46 | None | A_placement | selected_cube_and_board | cam0 | cam0, cam1, cam2, cam3 | selected_cube_and_board |
| 47 | None | A_placement | selected_cube_and_board | cam0 | cam0, cam1, cam2, cam3 | selected_cube_and_board |
| 48 | None | A_placement | selected_cube_and_board | cam0 | cam0, cam1, cam2, cam3 | selected_cube_and_board |
| 49 | None | A_placement | selected_cube_and_board | cam0 | cam0, cam1, cam2, cam3 | selected_cube_and_board |
| 50 | None | A_placement | selected_cube_and_board | cam0 | cam0, cam1, cam2, cam3 | selected_cube_and_board |
| 51 | None | A_placement | selected_cube_and_board | cam0 | cam0, cam1, cam2, cam3 | selected_cube_and_board |
| 52 | None | A_placement | selected_cube_and_board | cam0 | cam0, cam1, cam2, cam3 | selected_cube_and_board |
| 53 | None | A_placement | selected_cube_and_board | cam0 | cam0, cam1, cam2, cam3 | selected_cube_and_board |
| 54 | None | A_placement | selected_cube_and_board | cam0 | cam0, cam1, cam2, cam3 | selected_cube_and_board |
| 55 | None | A_placement | selected_cube_and_board | cam0 | cam0, cam1, cam2, cam3 | selected_cube_and_board |
| 56 | None | A_placement | selected_cube_and_board | cam0 | cam0, cam1, cam2, cam3 | selected_cube_and_board |
| 57 | None | A_placement | selected_cube_and_board | cam0 | cam0, cam1, cam2, cam3 | selected_cube_and_board |
| 58 | None | A_placement | selected_cube_and_board | cam0 | cam0, cam1, cam2, cam3 | selected_cube_and_board |
| 59 | None | A_placement | selected_cube_and_board | cam0 | cam0, cam1, cam2, cam3 | selected_cube_and_board |
| 60 | None | A_placement | selected_cube_and_board | cam0 | cam0, cam1, cam2, cam3 | selected_cube_and_board |

## Calibration에서 frozen manifest 사용

```bash
python3 05_calibrate.py \
  --root_folder /home/sstone/rb-calibration-marker-experiment/data/session11_zeus_handheld_floor_wrist_meta_0909/calib_train \
  --intrinsics_dir /home/sstone/rb-calibration-marker-experiment/data/session11_zeus_handheld_floor_wrist_meta_0909/intrinsics \
  --observation-manifest /home/sstone/rb-calibration-marker-experiment/data/session11_zeus_handheld_floor_wrist_meta_0909/calib_out/capture_filter/Step2b_observation_manifest.json \
  --observation-filter-policy standard
```

`strict` 비교 시 마지막 값만 `strict`로 바꾸면 됩니다. Manifest를 사용할 때 05 calibration은 detector를 다시 실행하지 않으며, meta/intrinsics/선택 RGB의 SHA-256이 달라지면 중단합니다.

## 산출물

- `Step2b_observation_manifest.json`: frozen 2D/3D corner, 정책, source SHA-256
- `Step2b_selected_observations.csv`: standard 선택 관측
- `Step2b_quarantine_observations.csv`: 복구되지 않은 저품질/planar 관측
- `Step2b_rejected_observations.csv`: 검출/PnP 실패 관측
- `Step2b_retake_candidates.csv`: event 단위 재촬영 후보
- `Step2b_review_overlay.jpg`: 육안 검토 contact sheet
