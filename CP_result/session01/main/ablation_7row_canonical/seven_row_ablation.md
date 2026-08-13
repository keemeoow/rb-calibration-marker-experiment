# Seven-row ablation

Primary metric: `heldout_event_reprojection_rmse_px`. Train reprojection is diagnostic only.

Noise-free sanity gate: **PASS**.

| Row | Target | U | FK→cube | FK→board | Status | N_reg | held-out reproj (px) | train reproj (px, diag.) |
| --- | --- | --- | --- | --- | --- | ---: | ---: | ---: |
| A0 | board | seq | — | estimated | converged (3/3) | 2 | 1.6277±0.0000 | 1.8535±0.0000 |
| A1 | cube+board | seq | estimated | estimated | converged (3/3) | 3 | 4.9914±0.0000 | 5.0307±0.0000 |
| A2 | cube+board | U | estimated | estimated | converged (3/3) | 3 | 4.8207±0.0000 | 4.8882±0.0000 |
| A3 | cube+board | U | FK-fixed | estimated | converged (3/3) | 3 | 5.0262±0.0000 | 5.0352±0.0000 |
| B1 | cube+board | seq | FK-fixed | estimated | converged (3/3) | 3 | 5.0605±0.0000 | 5.0547±0.0000 |
| B2 | cube | U | FK-fixed | — | converged (3/3) | 3 | 7.8438±0.0000 | 7.8626±0.0000 |
| B3 | board | U | — | estimated | converged (3/3) | 2 | 1.6277±0.0000 | 1.8535±0.0000 |

Held-out reprojection uses frozen parameters and an event-grouped, set-stratified split. It is not the whole-position FK-proxy metric. Non-converged rows must not be copied into Table 1 as final numbers.

## Model-independent path metrics

Mask SHA-256: `342a6b38907d3e27567e012edfb513fd9812d144890997e1e25fab46cf6dd55c`. Every cube-bearing row evaluates all 81 e_cross pairs and 29 e_e2e units; there is no output-dependent 30 mm/10 degree gate.

| Row | cube reproj (px) | board reproj (px) | e_e2e (mm/deg) | e_cross (mm/deg) |
| --- | ---: | ---: | ---: | ---: |
| A0 | — | 1.6277±0.0000 | — / — | — / — |
| A1 | 7.9627±0.0000 | 1.6418±0.0000 | 14.3896±0.0000 / 8.1782±0.0001 | 38.2564±0.0007 / 36.3925±0.0001 |
| A2 | 7.6735±0.0000 | 1.6318±0.0000 | 14.4485±0.0002 / 8.1762±0.0001 | 38.7887±0.0001 / 36.4042±0.0001 |
| A3 | 7.9426±0.0000 | 1.8522±0.0000 | 14.5478±0.0002 / 8.1655±0.0002 | 34.5988±0.0001 / 36.3770±0.0000 |
| B1 | 7.9810±0.0000 | 1.9032±0.0000 | 14.5123±0.0000 / 8.1533±0.0000 | 34.6212±0.0001 / 36.3765±0.0000 |
| B2 | 7.8438±0.0000 | — | 15.6711±0.0003 / 8.1526±0.0001 | 34.5082±0.0001 / 36.3724±0.0000 |
| B3 | — | 1.6277±0.0000 | — / — | — / — |

## Held-out reprojection by camera

| Row | cam 0 (px) | cam 1 (px) | cam 2 (px) | cam 3 (px) |
| --- | ---: | ---: | ---: | ---: |
| A0 | 0.3162±0.0000 | 0.4946±0.0000 | 1.7602±0.0000 | — |
| A1 | 11.7149±0.0000 | 1.2548±0.0002 | 1.8213±0.0000 | 0.9955±0.0002 |
| A2 | 11.3278±0.0000 | 0.8133±0.0000 | 1.7867±0.0001 | 0.7500±0.0001 |
| A3 | 11.6744±0.0000 | 0.8669±0.0000 | 2.0570±0.0001 | 0.9757±0.0000 |
| B1 | 11.7320±0.0000 | 1.0663±0.0000 | 2.0876±0.0000 | 0.9758±0.0000 |
| B2 | 15.3357±0.0000 | 0.8553±0.0000 | 1.9688±0.0001 | 0.9757±0.0000 |
| B3 | 0.3162±0.0000 | 0.4946±0.0000 | 1.7603±0.0000 | — |

A separate whole-position result without external ground truth must be labelled `e_task_pose^{FK-proxy}` and must not be described as absolute physical accuracy.
