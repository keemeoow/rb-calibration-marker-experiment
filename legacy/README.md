# Historical experiment provenance

이 폴더는 과거 C1/C2/C3, 7행 ablation, fixed-weight soft-anchor 결과를 삭제하지 않고 추적하기 위한
manifest 계층이다. 기존 스크립트와 결과는 현재 경로에 그대로 두어 README 링크와 import를 깨뜨리지
않는다. `manifest.json`의 SHA-256은 2026-08-06 작업 트리에서 확인한 내용이다.

새 최종 실험의 진입점은 다음과 같다.

- `CP_final_methods.py`: A2/A3/A4a/A4b/A4 및 fair B1
- `CP_blind_pose_predict.py`: GT를 읽지 않는 blind pose prediction
- `CP_final_external_gt_eval.py`: session-first paired hierarchical external-GT 평가

아래 과거 파일은 새 runner가 완전히 대체되고 논문 provenance archive가 생성될 때까지 삭제하지 않는다.

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
