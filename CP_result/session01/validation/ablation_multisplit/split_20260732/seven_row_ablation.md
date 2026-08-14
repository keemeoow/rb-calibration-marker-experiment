# Seven-row ablation

Primary metric: `heldout_event_reprojection_rmse_px`. Train reprojection is diagnostic only.

Noise-free sanity gate: **PASS**.

| Row | Target | U | FK→cube | FK→board | Status | N_reg | held-out reproj (px) | train reproj (px, diag.) |
| --- | --- | --- | --- | --- | --- | ---: | ---: | ---: |
| A0 | board | seq | — | estimated | converged (5/5) | 2 | 1.3726±0.0000 | 1.6121±0.0000 |
| A1 | cube+board | seq | estimated | estimated | converged (5/5) | 3 | 4.6226±0.0000 | 5.0275±0.0000 |
| A2 | cube+board | U | estimated | estimated | converged (5/5) | 3 | 4.4902±0.0000 | 4.8863±0.0000 |
| A3 | cube+board | U | FK-fixed | estimated | converged (5/5) | 3 | 4.6008±0.0000 | 5.0245±0.0000 |
| B1 | cube+board | seq | FK-fixed | estimated | converged (5/5) | 3 | 4.6121±0.0000 | 5.0314±0.0000 |
| B2 | cube | U | FK-fixed | — | converged (5/5) | 3 | 7.4416±0.0000 | 8.0210±0.0000 |
| B3 | board | U | — | estimated | converged (5/5) | 2 | 1.3726±0.0000 | 1.6121±0.0000 |

Held-out reprojection uses frozen parameters and an event-grouped, set-stratified split. It is not the whole-position FK-proxy metric. Non-converged rows must not be copied into Table 1 as final numbers.

## Model-independent path metrics

Mask SHA-256: `5255292f66f3750b1d0fe8deadc2c32f7bd6abeab5e2bdab4579a0aa34706c5b`. Every cube-bearing row evaluates all 84 e_cross pairs and 28 e_e2e units; there is no output-dependent 30 mm/10 degree gate.

| Row | cube reproj (px) | board reproj (px) | e_e2e (mm/deg) | e_cross (mm/deg) |
| --- | ---: | ---: | ---: | ---: |
| A0 | — | 1.3726±0.0000 | — / — | — / — |
| A1 | 7.4989±0.0000 | 1.3742±0.0000 | 16.7800±0.0002 / 7.8488±0.0001 | 39.5031±0.0006 / 34.8298±0.0000 |
| A2 | 7.2731±0.0000 | 1.3676±0.0000 | 16.6249±0.0001 / 7.8510±0.0000 | 38.8737±0.0001 / 34.8374±0.0000 |
| A3 | 7.4722±0.0000 | 1.3410±0.0000 | 16.6032±0.0004 / 7.7997±0.0001 | 38.0044±0.0002 / 34.8225±0.0000 |
| B1 | 7.4846±0.0000 | 1.3628±0.0000 | 16.5966±0.0000 / 7.7918±0.0000 | 38.0702±0.0003 / 34.8231±0.0000 |
| B2 | 7.4416±0.0000 | — | 16.0604±0.0005 / 7.8241±0.0001 | 37.8177±0.0003 / 34.8163±0.0000 |
| B3 | — | 1.3726±0.0000 | — / — | — / — |

## Held-out reprojection by camera

| Row | cam 0 (px) | cam 1 (px) | cam 2 (px) | cam 3 (px) |
| --- | ---: | ---: | ---: | ---: |
| A0 | 0.3135±0.0000 | 0.4458±0.0000 | 1.4809±0.0000 | — |
| A1 | 10.9281±0.0000 | 1.0549±0.0001 | 1.5843±0.0000 | 1.0200±0.0002 |
| A2 | 10.5931±0.0000 | 0.8908±0.0000 | 1.6002±0.0000 | 0.8488±0.0000 |
| A3 | 10.9076±0.0000 | 0.7673±0.0000 | 1.5521±0.0000 | 0.9867±0.0000 |
| B1 | 10.9274±0.0000 | 0.7972±0.0000 | 1.5662±0.0000 | 0.9867±0.0000 |
| B2 | 14.4129±0.0000 | 0.8089±0.0000 | 1.9268±0.0000 | 0.9867±0.0000 |
| B3 | 0.3135±0.0000 | 0.4458±0.0000 | 1.4809±0.0000 | — |

A separate whole-position result without external ground truth must be labelled `e_task_pose^{FK-proxy}` and must not be described as absolute physical accuracy.
