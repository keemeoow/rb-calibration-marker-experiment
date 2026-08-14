# Seven-row ablation

Primary metric: `heldout_event_reprojection_rmse_px`. Train reprojection is diagnostic only.

Noise-free sanity gate: **PASS**.

| Row | Target | U | FK→cube | FK→board | Status | N_reg | held-out reproj (px) | train reproj (px, diag.) |
| --- | --- | --- | --- | --- | --- | ---: | ---: | ---: |
| A0 | board | seq | — | estimated | converged (5/5) | 2 | 1.4691±0.0000 | 1.5807±0.0000 |
| A1 | cube+board | seq | estimated | estimated | converged (5/5) | 3 | 5.2703±0.0000 | 4.8895±0.0000 |
| A2 | cube+board | U | estimated | estimated | converged (5/5) | 3 | 5.1000±0.0000 | 4.7411±0.0000 |
| A3 | cube+board | U | FK-fixed | estimated | converged (5/5) | 3 | 5.2066±0.0000 | 4.8613±0.0000 |
| B1 | cube+board | seq | FK-fixed | estimated | converged (5/5) | 3 | 5.2158±0.0000 | 4.8683±0.0000 |
| B2 | cube | U | FK-fixed | — | converged (5/5) | 3 | 8.3911±0.0000 | 7.7664±0.0000 |
| B3 | board | U | — | estimated | converged (5/5) | 2 | 1.4691±0.0000 | 1.5807±0.0000 |

Held-out reprojection uses frozen parameters and an event-grouped, set-stratified split. It is not the whole-position FK-proxy metric. Non-converged rows must not be copied into Table 1 as final numbers.

## Model-independent path metrics

Mask SHA-256: `10556cc3c606a8294133f701040b3ea0c4ea84d4eafe58b95478aa278f0ec308`. Every cube-bearing row evaluates all 90 e_cross pairs and 30 e_e2e units; there is no output-dependent 30 mm/10 degree gate.

| Row | cube reproj (px) | board reproj (px) | e_e2e (mm/deg) | e_cross (mm/deg) |
| --- | ---: | ---: | ---: | ---: |
| A0 | — | 1.4691±0.0000 | — / — | — / — |
| A1 | 8.5291±0.0000 | 1.4688±0.0000 | 18.4170±0.0002 / 8.5348±0.0001 | 44.4501±0.0006 / 40.5058±0.0000 |
| A2 | 8.2373±0.0000 | 1.4742±0.0000 | 18.3545±0.0004 / 8.5228±0.0001 | 43.8052±0.0003 / 40.5118±0.0001 |
| A3 | 8.4170±0.0000 | 1.4811±0.0001 | 18.3221±0.0005 / 8.4852±0.0001 | 42.9276±0.0003 / 40.4908±0.0000 |
| B1 | 8.4330±0.0000 | 1.4797±0.0000 | 18.3252±0.0001 / 8.4794±0.0000 | 42.9917±0.0003 / 40.4915±0.0000 |
| B2 | 8.3911±0.0000 | — | 17.7224±0.0002 / 8.5193±0.0001 | 42.7473±0.0003 / 40.4829±0.0000 |
| B3 | — | 1.4691±0.0000 | — / — | — / — |

## Held-out reprojection by camera

| Row | cam 0 (px) | cam 1 (px) | cam 2 (px) | cam 3 (px) |
| --- | ---: | ---: | ---: | ---: |
| A0 | 0.3051±0.0000 | 0.5078±0.0000 | 1.5757±0.0000 | — |
| A1 | 12.7810±0.0000 | 1.1116±0.0001 | 1.6660±0.0000 | 1.0748±0.0001 |
| A2 | 12.3739±0.0000 | 0.8240±0.0001 | 1.6390±0.0000 | 0.8402±0.0002 |
| A3 | 12.6429±0.0001 | 0.7509±0.0001 | 1.6541±0.0001 | 0.9756±0.0000 |
| B1 | 12.6651±0.0000 | 0.7737±0.0000 | 1.6555±0.0000 | 0.9756±0.0000 |
| B2 | 16.3753±0.0000 | 0.7779±0.0000 | 1.9204±0.0001 | 0.9756±0.0000 |
| B3 | 0.3051±0.0000 | 0.5077±0.0001 | 1.5757±0.0000 | — |

A separate whole-position result without external ground truth must be labelled `e_task_pose^{FK-proxy}` and must not be described as absolute physical accuracy.
