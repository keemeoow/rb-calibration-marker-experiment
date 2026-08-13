# Seven-row ablation

Primary metric: `heldout_event_reprojection_rmse_px`. Train reprojection is diagnostic only.

Noise-free sanity gate: **PASS**.

| Row | Target | U | FK→cube | FK→board | Status | N_reg | held-out reproj (px) | train reproj (px, diag.) |
| --- | --- | --- | --- | --- | --- | ---: | ---: | ---: |
| A0 | board | seq | — | estimated | converged (3/3) | 3 | 2.4829±0.0000 | 3.5745±0.0000 |
| A1 | cube+board | seq | estimated | estimated | converged (3/3) | 3 | 3.1267±0.0000 | 3.3980±0.0000 |
| A2 | cube+board | U | estimated | estimated | converged (3/3) | 3 | 3.1155±0.0001 | 3.3981±0.0000 |
| A3 | cube+board | U | FK-fixed | estimated | converged (3/3) | 3 | 2.4997±0.0000 | 3.6477±0.0000 |
| B3 | board | U | — | estimated | converged (3/3) | 3 | 2.4829±0.0000 | 3.5745±0.0000 |

Held-out reprojection uses frozen parameters and an event-grouped, set-stratified split. It is not the whole-position FK-proxy metric. Non-converged rows must not be copied into Table 1 as final numbers.

## Model-independent path metrics

Mask SHA-256: `4339c33d283052a80ba07f715f1091e17b344179dbc9018c6f929a9948f9fc80`. Every cube-bearing row evaluates all 9 e_cross pairs and 3 e_e2e units; there is no output-dependent 30 mm/10 degree gate.

| Row | cube reproj (px) | board reproj (px) | e_e2e (mm/deg) | e_cross (mm/deg) |
| --- | ---: | ---: | ---: | ---: |
| A0 | — | 2.4829±0.0000 | — / — | — / — |
| A1 | 4.5317±0.0001 | 2.5103±0.0000 | 4.2459±0.0005 / 1.0239±0.0001 | 13.7546±0.0016 / 1.2193±0.0000 |
| A2 | 4.5113±0.0002 | 2.5038±0.0000 | 4.0685±0.0004 / 1.0385±0.0001 | 13.1259±0.0005 / 1.1910±0.0000 |
| A3 | 2.5136±0.0000 | 2.4952±0.0000 | 4.7904±0.0005 / 1.1892±0.0002 | 10.8328±0.0007 / 1.0602±0.0000 |
| B3 | — | 2.4829±0.0000 | — / — | — / — |

## Held-out reprojection by camera

| Row | cam 0 (px) | cam 1 (px) | cam 2 (px) | cam 3 (px) |
| --- | ---: | ---: | ---: | ---: |
| A0 | 0.2799±0.0000 | 1.2804±0.0000 | 2.9157±0.0000 | 1.9078±0.0000 |
| A1 | 1.6270±0.0000 | 3.0082±0.0003 | 3.4071±0.0000 | 3.2672±0.0003 |
| A2 | 1.6197±0.0001 | 2.9410±0.0006 | 3.4038±0.0000 | 3.2458±0.0002 |
| A3 | 0.6008±0.0000 | 1.4072±0.0001 | 2.9816±0.0000 | 1.8076±0.0001 |
| B3 | 0.2799±0.0000 | 1.2804±0.0000 | 2.9157±0.0000 | 1.9078±0.0001 |

A separate whole-position result without external ground truth must be labelled `e_task_pose^{FK-proxy}` and must not be described as absolute physical accuracy.
