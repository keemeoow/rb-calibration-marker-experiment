# Historical experiment provenance

이 폴더는 과거 C1/C2/C3, 7행 ablation, fixed-weight soft-anchor 결과의 provenance 기록이다.
과거 실행 스크립트는 Table 1 중심 정리 과정에서 삭제됐으며, `manifest.json`의 SHA-256은
2026-08-06 당시 작업 트리를 식별하기 위한 역사적 값이다.

새 최종 실험의 진입점은 다음과 같다.

- `CP_table1_ablation.py`: A0/A1/A2/A3/B1/B2/B3 공통 backend
- `CP_table1_fk_ablation.py`: A2/A3/A4a/A4b/A4 및 fair B1/B2
- `CP_table1_blind_predict.py`: GT를 읽지 않는 blind pose prediction
- `CP_table1_external_gt_eval.py`: session-first paired hierarchical external-GT 평가

아래 과거 파일은 이미 삭제됐으며 이름과 hash만 `manifest.json`에 provenance로 남긴다.

- `CP_C1_unified_vs_independent.py`
- `CP_C2_cube_vs_board.py`
- `CP_C3_prior_vs_noprior.py`
- `CP_D1_fk_correction_2x2.py`
- `CP_D2_anchored_event_split.py`
- `CP_ablation_7row.py`
- `CP_ablation_multisplit.py`

`CP_result/D1_fk_correction_2x2/`와 과거 D2 경로의 일부 파일은 현재 작업 트리에서 이미 삭제 표시된
상태다. 이 manifest는 존재하지 않는 결과를 복구했다고 주장하지 않는다. 현재 논문 근거로 유지되는
canonical multisplit·cross-target·7-row 결과만 해시로 고정한다.
