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
│   ├── TABLE1_INTERACTIVE.html
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
    ├── HARD_REJECTION_ABLATION.md
    ├── outlier_loss_ablation.csv
    ├── hard_rejection_ablation.csv
    ├── hard_rejection_ablation.json
    ├── linear_table1/
    ├── linear_cross_target/
    └── strict_table1/
```

상세 보고서는 [`session04/late_table1/TABLE1_RESULTS.md`](session04/late_table1/TABLE1_RESULTS.md), interactive 결과는 [`session04/late_table1/TABLE1_INTERACTIVE.html`](session04/late_table1/TABLE1_INTERACTIVE.html)에서 확인한다.

## 2. 데이터와 split 계약

- 입력: `data/session04/calib_train`
- 요청 세트: `0-12`; 표준 frozen-corner 품질 계약을 만족한 세트: `4-12`
- split seed: `20260731`
- split: 45 train events / 9 held-out events
- 최소 train eye-in-hand cube 관측: `3`
- 초기값: 3개
- 탈락 세트: `0-3` (train/test 분할 뒤 필요한 eye-in-hand cube 관측 부족)

세트 0-3은 standard observation manifest에서 train/test를 동시에 만들 수 있는 eye-in-hand cube 관측이 부족해 자동 제외된다. 품질 기준을 낮춰 억지로 포함하지 않는다.

세트 13은 `B_eyetohand` 파지 상태 13회로, 움직이는 cube를 포함한다. 고정된 set별 cube pose를 전제로 하는 메인 Table 1에는 섞지 않고 gripped-target 전용 실험으로 분리한다.

## 3. Table 1 결과 요약

숫자 표는 이 인덱스에 복제하지 않는다. 최신 값은 생성·검증되는
[`TABLE1_RESULTS.md`](session04/late_table1/TABLE1_RESULTS.md) 한 곳에서 확인한다.
이렇게 해야 재실행 뒤 README의 수치만 과거 값으로 남는 문제를 막을 수 있다.

- A1→A2는 같은 cube+board 관측에서 순차법과 통합법의 차이를 검증한다.
- A2와 A4의 근소한 차이는 A4의 우월성 근거가 아니다. A4/B1/B2는 Simulation prior를 쓰는 preflight다.
- A3의 raw FK hard constraint는 외부 GT가 아니며 tool/mechanical frame 오차를 그대로 포함한다.
- A5는 A4와 같은 train-only vision-aligned FK를 hard-fixed한 post-hoc 진단이다. Own held-out와 Fixed-to-Fixed cube는 낮지만 Fixed-to-Fixed board는 A4보다 높아 내부 지표에서도 일관된 승자가 아니다. 이전 A3 성능의 원인은 설명하지만 독립 보정이나 물리 정확도 우월성을 뜻하지 않는다.
- 현재 확증 대표 행은 A2, 방법론적 확장 후보는 A4, 원인 진단은 A5이며 A4/A5의 실제 순위는 blind external GT 이후 결정한다.
- 현재 순위는 내부 held-out reprojection 비교이며 절대 정확도 순위가 아니다.

## 4. 단일 데이터 원천

`session04/late_table1/table1_methods.json`은 A0~A5/B1~B3의 유일한 원시 실행 결과다. A6는 독립 실측 correction label을 위한 미실행 baseline 예약이다. `table1_results.csv`, `TABLE1_RESULTS.md`, `TABLE1_INTERACTIVE.html`은 이 JSON과 보조 평가 JSON에서 생성한다.

파생 결과는 `tools/sync_table1_canonical_data.py`로 생성하고 `tools/verify_table1_visual_sync.py`로 JSON/CSV/Markdown/HTML 숫자와 provenance 일치를 검사한다.

## 5. 보조 평가

- `cross_target_evaluation`: 모든 방법의 동결 transform을 같은 held-out target/camera mask로 평가한다.
- `marker_system_end_to_end`: board-only, cube-only, board+cube를 modality별 초기화부터 분리해 평가한다.
- `opencv_relative_baseline`: OpenCV PnP 기반 FK-free fixed-camera reference baseline이다.
- `outlier_ablation`: 동일 관측에서 soft-L1과 linear loss를 비교하고, standard/strict 사전 관측 제외 민감도를 동일 held-out에서 검증한다.
- [`BOARD_CUBE_RELATIVE_POSE.md`](../data/session04/calib_out/verify/board_cube_relative_pose/BOARD_CUBE_RELATIVE_POSE.md): Board/Cube geometry, corner ordering, detector refinement, intrinsic 및 target-dependent PnP 충돌을 진단한다. 여기서 direct-PnP 충돌은 최종 joint calibration 정확도나 외부 GT 오차가 아니다.
- 모든 카메라 범위 평가는 내부 consistency/transfer 평가이며 독립 외부 GT 정확도가 아니다.

## 6. 재실행과 검증 원칙

모든 비교 명령에는 다음 공통 인자를 사용한다.

```text
--root_folder data/session04/calib_train
--include_sets 0-12
--min_train_eih_cube_events 3
--split_seed 20260731
--observation-filter-policy standard
```

경로 인자는 더 이상 직접 넘기지 않는다. `--calib_dir`, `--out_dir`, `--observation-manifest`,
`--table1_result`, `--cross_target_result`는 모두 `--root_folder`의 `sessionNN`에서 유도된다
(`calibration_pipeline/runtime.py`의 `session_paths`). 명시적으로 넘기면 그 값이 우선한다.

이 유도 규칙이 없던 때는 일부 러너의 기본값 세션이 달라서 session04를 지정해도
session02 경로를 읽거나 쓰는 사고가 났다.
또한 `--observation-manifest`를 빠뜨리면 frozen manifest 대신 검출기를 다시 돌려
`cube_detection.source`가 비어 있는 결과가 조용히 생성됐다. 지금은 manifest 파일이 있으면
자동으로 사용하고, 없는 세션에서만 검출기로 폴백한다.

재실행 순서는 다음과 같다.

1. `05_calibrate.py --baseline_only`로 공통 baseline만 준비하거나, 바로 전체 fit을 실행한다.
2. `05_calibrate.py --num_inits 3`으로 A0~A5/B1~B3를 실행한다.
3. `06_make_report.py`로 calibration 요약 CSV, 전체 행렬 JSON, 상세 Markdown을 생성한다.
4. 필요할 때만 `tools/`의 cross-target, marker-system, OpenCV 평가와 outlier ablation을 갱신한다.
5. 선택 확장 평가를 갱신했다면 관련 verify 도구를 통과시킨다.

현재 결과는 9개 행×3개 seed의 27 runs를 포함한다. A5는 `posthoc_diagnostic`, A6는 독립 6-DoF correction label이 생기기 전까지 `not_run`이다. JSON/CSV/Markdown/HTML 동기화, 27 runs·243 fixed-camera pairs의 독립 `e_cross` 재계산과 camera-scope 계약을 통과했다.
