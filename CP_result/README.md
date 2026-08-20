# 비교실험 결과 구조

## 바로가기

- [1. Canonical session02 결과](#1-canonical-session02-결과)
- [2. 단일 데이터 원천](#2-단일-데이터-원천)
- [3. 보조 평가](#3-보조-평가)
- [4. 재실행 원칙](#4-재실행-원칙)

## 1. Canonical session02 결과

현재 논문용 비교실험 결과는 `session02`만 유지한다.

```text
CP_result/session02/
├── late_table1/
│   ├── TABLE1_RESULTS.md
│   ├── table1_methods.json
│   ├── table1_results.csv
│   ├── shared_train_only_baseline.json
│   └── shared_board_free_fk_cube.json
├── cross_target_evaluation/
│   ├── cross_target_evaluation.json
│   └── cross_target_evaluation.csv
└── marker_system_end_to_end/
    ├── marker_system_end_to_end.json
    └── marker_system_end_to_end.csv
```

## 2. 단일 데이터 원천

`table1_methods.json`은 A0~A4/B1~B3의 유일한 원시 실행 결과다. 과거의 `core_methods.json`과 `final_methods.json` 분리는 제거했다.

`table1_results.csv`, `TABLE1_RESULTS.md`, `_TABLE1_INTERACTIVE.html`은 이 실행 결과와 공통 평가 CSV에서 파생되며 `tools/verify_table1_visual_sync.py`로 숫자 일치를 검사한다.

## 3. 보조 평가

- `cross_target_evaluation`: 모든 방법의 동결 transform을 같은 held-out target/camera mask로 평가한다.
- `marker_system_end_to_end`: board-only, cube-only, board+cube를 modality별 초기화부터 분리해 평가한다.
- 둘 다 내부 consistency/transfer 평가이며 독립 외부 GT 정확도가 아니다.

## 4. 재실행 원칙

1. `Step3_calibration.py` 또는 `Run_calibration_comparison.py table1 --baseline_only`로 공통 baseline을 준비한다.
2. `Run_calibration_comparison.py table1` 한 진입점으로 A0~A4/B1~B3를 실행한다. A5 initial state도 같은 baseline에 예약된다.
3. 공통 평가를 갱신한다.
4. `tools/sync_table1_canonical_data.py`로 CSV/HTML을 동기화한다.
5. `tools/verify_table1_visual_sync.py`와 `tools/verify_e_cross_definition.py`를 통과시킨다.

실측 FK covariance가 없으면 A4/B1/B2는 preflight로만 표시한다. A5는 독립 6-DoF correction label이 생기기 전까지 `not_run`이다.
