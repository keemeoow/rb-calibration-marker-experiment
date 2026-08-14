# Seven-row ablation

Primary metric: `heldout_event_reprojection_rmse_px`. Train reprojection is diagnostic only.

Noise-free sanity gate: **PASS**.

| Row | Target | U | FK→cube | FK→board | Status | N_reg | held-out reproj (px) | train reproj (px, diag.) |
| --- | --- | --- | --- | --- | --- | ---: | ---: | ---: |
| A0 | board | seq | — | estimated | converged (5/5) | 2 | 1.5269±0.0000 | 1.5728±0.0000 |
| A1 | cube+board | seq | estimated | estimated | converged (5/5) | 3 | 4.4601±0.0000 | 5.0848±0.0000 |
| A2 | cube+board | U | estimated | estimated | converged (5/5) | 3 | 4.3242±0.0000 | 4.9287±0.0000 |
| A3 | cube+board | U | FK-fixed | estimated | converged (5/5) | 3 | 4.3997±0.0001 | 5.0683±0.0000 |
| B1 | cube+board | seq | FK-fixed | estimated | converged (5/5) | 3 | 4.4060±0.0000 | 5.0770±0.0000 |
| B2 | cube | U | FK-fixed | — | converged (5/5) | 3 | 6.9938±0.0000 | 8.1155±0.0000 |
| B3 | board | U | — | estimated | converged (5/5) | 2 | 1.5269±0.0000 | 1.5728±0.0000 |

Held-out reprojection uses frozen parameters and an event-grouped, set-stratified split. It is not the whole-position FK-proxy metric. Non-converged rows must not be copied into Table 1 as final numbers.

## Model-independent path metrics

Mask SHA-256: `0e3563b2c70b4624fe12ce8922b9515703f8862481158480d0207bd8cb182173`. Every cube-bearing row evaluates all 85 e_cross pairs and 28 e_e2e units; there is no output-dependent 30 mm/10 degree gate.

| Row | cube reproj (px) | board reproj (px) | e_e2e (mm/deg) | e_cross (mm/deg) |
| --- | ---: | ---: | ---: | ---: |
| A0 | — | 1.5269±0.0000 | — / — | — / — |
| A1 | 7.1366±0.0000 | 1.5297±0.0000 | 16.1884±0.0001 / 7.3302±0.0001 | 37.8380±0.0003 / 33.7251±0.0000 |
| A2 | 6.9013±0.0000 | 1.5298±0.0000 | 16.0251±0.0001 / 7.3246±0.0000 | 37.0438±0.0001 / 33.7379±0.0000 |
| A3 | 7.0218±0.0000 | 1.5561±0.0003 | 15.9888±0.0004 / 7.3038±0.0001 | 36.2102±0.0003 / 33.7190±0.0000 |
| B1 | 7.0340±0.0000 | 1.5527±0.0000 | 15.9886±0.0001 / 7.3006±0.0000 | 36.2824±0.0001 / 33.7195±0.0000 |
| B2 | 6.9938±0.0000 | — | 15.5577±0.0004 / 7.3299±0.0001 | 36.0280±0.0002 / 33.7119±0.0000 |
| B3 | — | 1.5269±0.0000 | — / — | — / — |

## Held-out reprojection by camera

| Row | cam 0 (px) | cam 1 (px) | cam 2 (px) | cam 3 (px) |
| --- | ---: | ---: | ---: | ---: |
| A0 | 0.3082±0.0000 | 0.4511±0.0000 | 1.6480±0.0000 | — |
| A1 | 10.3739±0.0000 | 0.9776±0.0001 | 1.7388±0.0000 | 1.0236±0.0002 |
| A2 | 10.0286±0.0000 | 0.7822±0.0000 | 1.7534±0.0000 | 0.8437±0.0000 |
| A3 | 10.2499±0.0000 | 0.6811±0.0000 | 1.7149±0.0003 | 0.9835±0.0000 |
| B1 | 10.2714±0.0000 | 0.7165±0.0000 | 1.7046±0.0000 | 0.9835±0.0000 |
| B2 | 13.2769±0.0000 | 0.7193±0.0000 | 1.8156±0.0002 | 0.9835±0.0000 |
| B3 | 0.3082±0.0000 | 0.4512±0.0000 | 1.6480±0.0000 | — |

A separate whole-position result without external ground truth must be labelled `e_task_pose^{FK-proxy}` and must not be described as absolute physical accuracy.
