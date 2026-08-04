# D2 rerun at `RB_ROBOT_POS_SCALE=1.0` (k=1.0)

This is **not** a replacement for [`../../D2_anchored_event_split/`](../../D2_anchored_event_split/).
Both are kept on purpose because they were measured at different robot scale constants.

| artifact | `RB_ROBOT_POS_SCALE` | reproduces Table 1 A2/A3 |
| --- | --- | --- |
| `CP_result/D2_anchored_event_split/` (committed) | **1.0229** | within 1.51σ (A3 `e_cross` 32.75 vs 38.05 mm) |
| this directory | **1.0** | to ~1e-5 (`worst_sigma` 0.0) |

Table 1's A0–B3 rows come from `CP_result/ablation_multisplit/`, which was run at k=1.0. The
committed D2 artifact was run at k=1.0229, so its cells sit on a different scale basis than the
rest of the table.

Verification that `ablation_multisplit` is k=1.0: the raw FK cube poses stored in
`ablation_multisplit/split_20260729/shared_board_free_fk_cube.json` reproduce to 0.0000 mm when
`Step3_calibration.load_nominal_set_cube_transforms` is called at k=1.0, and differ by 12.36 mm
mean / 16.50 mm max at k=1.0229.

This rerun exists so the adopted configuration `A2@λ=3` can be scored on the same basis as every
other row in [`../cross_target_cube.md`](../cross_target_cube.md). It was produced by
`CP_D2_anchored_event_split.py` with default options and no `RB_ROBOT_POS_SCALE` set, after that
script was changed to dump frozen transforms per run.

**Caution.** D2's built-in `reference_agreement` gate does not detect a scale mismatch. Its
tolerance is expressed in units of Table 1's split standard deviation, which is large enough
(3.5 mm for `e_cross`) that a 5.3 mm scale-induced shift still reports `within_tolerance: true`.
Check the FK poses directly instead.

k=1.0229 is itself provisional — the per-camera estimates span 1.4–2.9%. Neither directory should
be treated as the settled answer until k is confirmed by physical measurement.
