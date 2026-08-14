# CP_result 디렉터리 구조

**세션(입력 데이터) → 역할** 2단 구조. 어떤 실험이 어느 촬영 세션에서 나왔는지가 최상위에서
바로 보이도록 정리했다. 각 스크립트의 기본 `--out_dir` 이 여기 경로를 그대로 가리키므로,
재실행해도 같은 자리에 덮어써진다.

```
CP_result/
├── session01/     data/session01 (구 data/session) 기반 — 2026-08-06 로봇 스케일 보정 후 전량 재생성
│   ├── main/          논문 본문 표
│   ├── components/    기여도별 구성요소 비교
│   ├── diagnostics/   원인 규명·진단 실험
│   ├── validation/    일반화·재현성 검증
│   ├── artifacts/     다른 실험이 입력으로 쓰는 산출물
│   └── figures/       session01 전용 figure
├── session02/     data/session02 기반
└── shared/        세션 무관 (시뮬레이터 / 외부 데이터셋 / SIM↔CP 비교)
```

## session01 — `data/session01`

로봇 kinematic 스케일 보정(`RB_ROBOT_POS_SCALE`, k=1.0) 적용 후 2026-08-06 에 일괄 재생성된 결과다.

| 경로 | 생성 스크립트 | 내용 |
|---|---|---|
| `main/ablation_7row_canonical/` | `CP_ablation_7row.py` | canonical 7-row ablation (본 표) |
| `main/final_methods_preflight/` | `CP_final_methods.py` | 최종 method 조합 preflight |
| `main/A2_strict_none/` | `CP_A2_strict_none.py` | A2(strict none) vs A3(FK-fixed) |
| `components/C1/` | `CP_C1_unified_vs_independent.py` | unified joint vs independent vs joint-FK-fixed |
| `components/C2/` | `CP_C2_cube_vs_board.py` | cube target vs board target + observability |
| `components/C3/` | `CP_C3_prior_vs_noprior.py` | robot-cube prior 유무 × solver 01–04 |
| `components/solver_01_04/` | `CP_solver_01_04.py` | solver 01–04 단독 비교 (로봇 FK 미사용) |
| `diagnostics/D1_fk_correction_2x2/` | `CP_D1_fk_correction_2x2.py` | FK 보정 2×2 (위치 hold-out) |
| `diagnostics/D2_anchored_event_split/` | `CP_D2_anchored_event_split.py` | soft-anchor event split |
| `validation/ablation_multisplit/` | `CP_ablation_multisplit.py` | 5개 split 반복 ablation |
| `validation/cross_target_cube/` | `CP_cross_target_cube_eval.py` | cross-target cube 평가 (multisplit + D2 를 입력으로 읽음) |
| `validation/sensitivity_7row/` | `CP_sensitivity_7row.py` | 하이퍼파라미터 민감도 |
| `validation/validate_holdout/` | `CP_validate_holdout.py` | hold-out 검증 |
| `validation/validate_loso/` | `CP_validate_loso.py` | leave-one-set-out + LOSO figure |
| `artifacts/fk_cube_artifact/` | `CP_build_board_free_fk_artifact.py` | board-free FK cube 산출물 (7-row/multisplit 입력) |
| `artifacts/cube_selfcal/` | `CP_cube_selfcal.py` | cube marker pose self-calibration |
| `figures/` | `CP_viz_c1_fk_correction.py` | `fig_CP_C1_fk_correction`, `..._internal_metrics`, `..._C3_interpretation` + provenance JSON |

## session02 — `data/session02`

| 경로 | 내용 |
|---|---|
| `late_table1/` | session02 후반 세트 기준 Table 1 held-out 결과 (`TABLE1_RESULTS.md`, `table1_results.csv`) + 근거가 된 `ablation_core/`, `final_methods_preflight/` |

## shared — 세션 무관

우리 로봇 FK 를 쓰지 않으므로 로봇 스케일 보정 대상이 **아니다**.

| 경로 | 생성 스크립트 | 내용 |
|---|---|---|
| `synthetic_7row/` | `CP_synthetic_7row.py` | 픽셀 레벨 GT 시뮬레이터 기반 7-row (실측 아님) |
| `metric_board_only/` | `CP_metric_board_only.py` | 외부 데이터셋 `Multi-Camera-Hand-Eye-Calibration/data/medium_workcell` |
| `figures/` | `CP_viz_sim_vs_real.py` | SIM / CP / SIMvsCP 9장 한 세트 |
| `SIM_vs_CP_summary.{csv,md}` | `CP_viz_sim_vs_real.py` | 시뮬↔실데이터 통합 요약표 (재실행 시 생성) |

`shared/figures/` 의 `fig_CP_*` 3장은 session01 데이터에서 나온 패널이지만,
`CP_viz_sim_vs_real.py` 가 SIM·CP·비교 9장을 한 디렉터리에 함께 쓰므로 세트를 쪼개지 않고 여기 둔다.

## 주의

- 결과 JSON 안의 `output_dir` / `calib_dir` / `path` 필드는 **실행 당시 기록(provenance)** 이라
  이번 정리에서 건드리지 않았다. 옛 평면 경로가 적혀 있어도 정상이며, 코드가 이 필드를 읽어
  파일을 로드하지는 않는다.
- `legacy/manifest.json` 의 해시 목록도 아카이브 시점 경로 그대로 둔다.
- 실행 방법과 해석은 [`../CP_EXPERIMENTS_README.md`](../CP_EXPERIMENTS_README.md) 참고.
