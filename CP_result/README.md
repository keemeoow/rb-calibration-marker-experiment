# 비교실험 결과 구조

## 바로가기

- [1. Canonical session04 결과](#1-canonical-session04-결과)
- [2. 데이터와 split 계약](#2-데이터와-split-계약)
- [3. Table 1 결과 요약](#3-table-1-결과-요약)
- [4. 단일 데이터 원천](#4-단일-데이터-원천)
- [5. 보조 평가](#5-보조-평가)
- [6. 재실행과 검증 원칙](#6-재실행과-검증-원칙)

## 1. Canonical session04 결과

현재 비교실험의 canonical 결과는 `session04`다. 

```text
CP_result/session04/
├── late_table1/
│   ├── TABLE1_RESULTS.md
│   ├── table1_methods.json
│   ├── table1_results.csv
│   ├── shared_train_only_baseline.json
│   └── shared_board_free_fk_cube.json
├── cross_target_evaluation/
│   ├── cross_target_evaluation.json
│   └── cross_target_evaluation.csv
├── marker_system_end_to_end/
│   ├── marker_system_end_to_end.json
│   └── marker_system_end_to_end.csv
├── opencv_relative_baseline/
│   ├── OPENCV_RELATIVE_BASELINE.md
│   ├── opencv_relative_baseline.json
│   └── opencv_relative_baseline.csv
└── outlier_ablation/
    ├── OUTLIER_LOSS_ABLATION.md
    ├── outlier_loss_ablation.csv
    ├── linear_table1/
    └── linear_cross_target/
```

상세 보고서는 [`../session04_result_table1.md`](../session04_result_table1.md), interactive 결과는 [`../_TABLE1_INTERACTIVE_session04.html`](../_TABLE1_INTERACTIVE_session04.html)에서 확인한다.

## 2. 데이터와 split 계약

- 입력: `data/session04/calib_train`
- 고정 배치 세트: `0-12` 전체, 13세트 × 6회 = 78 events
- split seed: `20260731`
- split: 65 train events / 13 held-out events
- 최소 train eye-in-hand cube 관측: `2`
- 초기값: 3개
- 탈락 세트: 없음

세트 2와 3은 품질 게이트를 통과한 eye-in-hand cube 관측이 각각 3개다. 기본 최소값 3을 적용하면 train/test를 동시에 확보할 수 없어 자동 제외되므로, 전체 고정 배치 세트를 사용하는 canonical 결과에서는 `--min_train_eih_cube_events 2`를 고정한다.

세트 13은 `B_eyetohand` 파지 상태 13회로, 움직이는 cube를 포함한다. 고정된 set별 cube pose를 전제로 하는 메인 Table 1에는 섞지 않고 gripped-target 전용 실험으로 분리한다.

## 3. Table 1 결과 요약

| Method | Train Overall px | Own Held-out Overall px | Board/Cube Held-out px | Convergence |
| --- | ---: | ---: | ---: | ---: |
| A0 | 6.1832 | 5.8773 | 5.8773 / N/A | 3/3 |
| A1 | 5.9224 | 6.3602 | 5.9153 / 7.7657 | 3/3 |
| A2 | 5.7846 | 6.2143 | 5.9229 / 7.1745 | 3/3 |
| A3 | 5.9805 | **5.4968** | 5.7250 / **4.5724** | 3/3 |
| A4 | 5.7846 | 6.1904 | 5.9128 / 7.1088 | 3/3 |
| B1 | 5.9154 | 6.3309 | 5.9092 / 7.6710 | 3/3 |
| B2 | 3.7132 | 6.6975 | N/A / 6.6975 | 3/3 |
| B3 | 6.1832 | 5.8772 | 5.8772 / N/A | 3/3 |

같은 cube+board marker population에서는 A3의 own held-out overall과 cube held-out 오차가 가장 낮다. 다만 Fixed-to-Fixed와 Gripper-to-Fixed의 board 지표는 A0/B3가 가장 낮고 cube 지표는 B2 또는 A3가 낮으므로, target별 trade-off를 분리해 해석해야 한다.

A4/B1/B2는 실측 FK covariance가 아닌 Simulation prior를 사용하는 preflight다. 이 결과는 외부 GT 절대 정확도 순위가 아니다.

## 4. 단일 데이터 원천

`session04/late_table1/table1_methods.json`은 A0~A4/B1~B3의 유일한 원시 실행 결과다. `table1_results.csv`, `TABLE1_RESULTS.md`, `session04_result_table1.md`, `_TABLE1_INTERACTIVE_session04.html`은 이 JSON과 보조 평가 JSON에서 생성한다.

파생 결과는 `tools/sync_table1_canonical_data.py`로 생성하고 `tools/verify_table1_visual_sync.py`로 JSON/CSV/Markdown/HTML 숫자와 provenance 일치를 검사한다.

## 5. 보조 평가

- `cross_target_evaluation`: 모든 방법의 동결 transform을 같은 held-out target/camera mask로 평가한다.
- `marker_system_end_to_end`: board-only, cube-only, board+cube를 modality별 초기화부터 분리해 평가한다.
- `opencv_relative_baseline`: OpenCV PnP 기반 FK-free fixed-camera reference baseline이다.
- `outlier_ablation`: 동일 관측에서 soft-L1과 linear loss를 비교한다.
- 모든 카메라 범위 평가는 내부 consistency/transfer 평가이며 독립 외부 GT 정확도가 아니다.

## 6. 재실행과 검증 원칙

모든 비교 명령에는 다음 공통 인자를 사용한다.

```text
--root_folder data/session04/calib_train
--intrinsics_dir intrinsics
--calib_dir data/session04/calib_out
--include_sets 0-12
--min_train_eih_cube_events 2
--split_seed 20260731
```

재실행 순서는 다음과 같다.

1. `Step3_calibration.py` 또는 `table1 --baseline_only`로 공통 baseline을 준비한다.
2. `Run_calibration_comparison.py table1 --num_inits 3`으로 A0~A4/B1~B3를 실행한다.
3. 같은 인자로 `cross-target`, `marker-system`, `opencv-relative`와 outlier ablation을 갱신한다.
4. `tools/sync_table1_canonical_data.py`로 CSV/Markdown/HTML을 생성한다.
5. `tools/verify_table1_visual_sync.py`, `tools/verify_e_cross_definition.py`, `tools/verify_camera_scope_evaluation.py`를 통과시킨다.

현재 결과는 JSON/CSV/Markdown/HTML 동기화, 24 runs·216 fixed-camera pairs의 독립 `e_cross` 재계산, camera-scope 계약 검증을 통과했다. A5는 독립 6-DoF correction label이 생기기 전까지 `not_run`이다.
