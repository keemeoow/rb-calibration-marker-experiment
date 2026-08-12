# Seven-row ablation

Primary metric: `heldout_event_reprojection_rmse_px`. Train reprojection is diagnostic only.

Noise-free sanity gate: **PASS**.

| Row | Target | U | FK→cube | FK→board | Status | N_reg | held-out reproj (px) | train reproj (px, diag.) |
| --- | --- | --- | --- | --- | --- | ---: | ---: | ---: |
| A0 | board | seq | — | estimated | converged (5/5) | 2 | 1.2799±0.0000 | 1.6365±0.0000 |
| A1 | cube+board | seq | estimated | estimated | converged (5/5) | 3 | 3.9367±0.0000 | 5.1951±0.0000 |
| A2 | cube+board | U | estimated | estimated | converged (5/5) | 3 | 3.8112±0.0000 | 5.0469±0.0000 |
| A3 | cube+board | U | FK-fixed | estimated | converged (5/5) | 3 | 3.9577±0.0000 | 5.1708±0.0000 |
| B1 | cube+board | seq | FK-fixed | estimated | converged (5/5) | 3 | 3.9637±0.0000 | 5.1777±0.0000 |
| B2 | cube | U | FK-fixed | — | converged (5/5) | 3 | 6.2817±0.0000 | 8.2866±0.0000 |
| B3 | board | U | — | estimated | converged (5/5) | 2 | 1.2799±0.0000 | 1.6365±0.0000 |

Held-out reprojection uses frozen parameters and an event-grouped, set-stratified split. It is not the whole-position FK-proxy metric. Non-converged rows must not be copied into Table 1 as final numbers.

## Model-independent path metrics

Mask SHA-256: `d402575b380fac0e95ddeea162cf7449ebfd8c54ea970cf6ea3d417a3cd6b4dc`. Every cube-bearing row evaluates all 83 e_cross pairs and 29 e_e2e units; there is no output-dependent 30 mm/10 degree gate.

| Row | cube reproj (px) | board reproj (px) | e_e2e (mm/deg) | e_cross (mm/deg) |
| --- | ---: | ---: | ---: | ---: |
| A0 | — | 1.2799±0.0000 | — / — | — / — |
| A1 | 6.2874±0.0000 | 1.2884±0.0000 | 14.5789±0.0002 / 6.8520±0.0000 | 34.5896±0.0010 / 27.2219±0.0000 |
| A2 | 6.0729±0.0000 | 1.2859±0.0000 | 14.4293±0.0002 / 6.8524±0.0003 | 33.8342±0.0007 / 27.2339±0.0002 |
| A3 | 6.3157±0.0000 | 1.3099±0.0000 | 14.4060±0.0003 / 6.8329±0.0002 | 32.6775±0.0002 / 27.2190±0.0000 |
| B1 | 6.3264±0.0000 | 1.3088±0.0000 | 14.3849±0.0001 / 6.8248±0.0000 | 32.7502±0.0004 / 27.2198±0.0000 |
| B2 | 6.2817±0.0000 | — | 13.9031±0.0003 / 6.8667±0.0001 | 32.4718±0.0003 / 27.2113±0.0000 |
| B3 | — | 1.2799±0.0000 | — / — | — / — |

## Held-out reprojection by camera

| Row | cam 0 (px) | cam 1 (px) | cam 2 (px) | cam 3 (px) |
| --- | ---: | ---: | ---: | ---: |
| A0 | 0.3109±0.0000 | 0.4601±0.0000 | 1.3766±0.0000 | — |
| A1 | 9.2525±0.0000 | 1.0025±0.0001 | 1.4901±0.0000 | 1.1106±0.0003 |
| A2 | 8.9357±0.0000 | 0.8540±0.0000 | 1.5028±0.0000 | 0.8603±0.0002 |
| A3 | 9.3209±0.0000 | 0.7120±0.0001 | 1.5077±0.0001 | 0.9761±0.0000 |
| B1 | 9.3432±0.0000 | 0.7455±0.0000 | 1.4950±0.0000 | 0.9761±0.0000 |
| B2 | 12.0201±0.0000 | 0.7522±0.0000 | 1.7654±0.0002 | 0.9761±0.0000 |
| B3 | 0.3109±0.0000 | 0.4601±0.0000 | 1.3766±0.0000 | — |

A separate whole-position result without external ground truth must be labelled `e_task_pose^{FK-proxy}` and must not be described as absolute physical accuracy.
