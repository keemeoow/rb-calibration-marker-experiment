# Seven-row ablation

Primary metric: `heldout_event_reprojection_rmse_px`. Train reprojection is diagnostic only.

Noise-free sanity gate: **PASS**.

| Row | Target | U | FK→cube | FK→board | Status | N_reg | held-out reproj (px) | train reproj (px, diag.) |
| --- | --- | --- | --- | --- | --- | ---: | ---: | ---: |
| A0 | board | seq | — | estimated | converged (5/5) | 2 | 1.5448±0.0000 | 1.5741±0.0000 |
| A1 | cube+board | seq | estimated | estimated | converged (5/5) | 3 | 4.9906±0.0000 | 4.9836±0.0000 |
| A2 | cube+board | U | estimated | estimated | converged (5/5) | 3 | 4.8118±0.0000 | 4.8226±0.0000 |
| A3 | cube+board | U | FK-fixed | estimated | converged (5/5) | 3 | 4.9503±0.0000 | 4.9402±0.0000 |
| B1 | cube+board | seq | FK-fixed | estimated | converged (5/5) | 3 | 4.9610±0.0000 | 4.9472±0.0000 |
| B2 | cube | U | FK-fixed | — | converged (5/5) | 3 | 7.8938±0.0000 | 7.9081±0.0000 |
| B3 | board | U | — | estimated | converged (5/5) | 2 | 1.5447±0.0000 | 1.5741±0.0000 |

Held-out reprojection uses frozen parameters and an event-grouped, set-stratified split. It is not the whole-position FK-proxy metric. Non-converged rows must not be copied into Table 1 as final numbers.

## Model-independent path metrics

Mask SHA-256: `8122b1f886a40ab94fb5cb223c12e3926f43148841a3f28f62343eecf49b76a3`. Every cube-bearing row evaluates all 81 e_cross pairs and 29 e_e2e units; there is no output-dependent 30 mm/10 degree gate.

| Row | cube reproj (px) | board reproj (px) | e_e2e (mm/deg) | e_cross (mm/deg) |
| --- | ---: | ---: | ---: | ---: |
| A0 | — | 1.5448±0.0000 | — / — | — / — |
| A1 | 7.9939±0.0000 | 1.5479±0.0000 | 15.6973±0.0002 / 8.1669±0.0000 | 41.6081±0.0009 / 36.3697±0.0000 |
| A2 | 7.6867±0.0000 | 1.5530±0.0000 | 15.5440±0.0001 / 8.1607±0.0001 | 41.1118±0.0001 / 36.3874±0.0001 |
| A3 | 7.9233±0.0000 | 1.5529±0.0001 | 15.6203±0.0006 / 8.1220±0.0001 | 40.4206±0.0003 / 36.3760±0.0000 |
| B1 | 7.9382±0.0000 | 1.5629±0.0000 | 15.6093±0.0001 / 8.1135±0.0000 | 40.4852±0.0001 / 36.3764±0.0000 |
| B2 | 7.8938±0.0000 | — | 15.1998±0.0003 / 8.1591±0.0002 | 40.2324±0.0003 / 36.3709±0.0000 |
| B3 | — | 1.5447±0.0000 | — / — | — / — |

## Held-out reprojection by camera

| Row | cam 0 (px) | cam 1 (px) | cam 2 (px) | cam 3 (px) |
| --- | ---: | ---: | ---: | ---: |
| A0 | 0.3162±0.0000 | 0.4946±0.0000 | 1.6698±0.0000 | — |
| A1 | 11.7712±0.0000 | 1.1589±0.0001 | 1.7335±0.0000 | 1.0271±0.0002 |
| A2 | 11.3396±0.0000 | 0.8633±0.0000 | 1.7229±0.0000 | 0.8199±0.0000 |
| A3 | 11.6843±0.0000 | 0.7463±0.0001 | 1.7410±0.0001 | 1.0363±0.0000 |
| B1 | 11.7073±0.0000 | 0.7749±0.0000 | 1.7466±0.0000 | 1.0363±0.0000 |
| B2 | 15.4324±0.0000 | 0.7869±0.0000 | 1.9918±0.0001 | 1.0362±0.0001 |
| B3 | 0.3162±0.0000 | 0.4946±0.0000 | 1.6698±0.0000 | — |

A separate whole-position result without external ground truth must be labelled `e_task_pose^{FK-proxy}` and must not be described as absolute physical accuracy.
