# Cross-target held-out cube reprojection

Every row is scored on the **same** held-out cube corners with the **same** cube
poses (shared board-free FK artifact). No refit; all transforms frozen from the
stored multisplit runs. Scored fixed cameras: [0, 1] (intersection over all rows; eih path excluded).

| Row | Target | U | FK→cube | cube e2h reproj (px) | cam 0 (px) | cam 1 (px) | own-pose cube e2h (px) |
| --- | --- | :---: | --- | ---: | ---: | ---: | ---: |
| A0 | board | seq | — | 10.9130±1.0401 | 14.7273±1.5260 | 2.5594±0.0868 | — |
| A1 | cube+board | seq | estimated | 10.5768±1.0394 | 14.4086±1.5164 | 1.3027±0.1469 | 10.5799±1.1125 |
| A2 | cube+board | U | estimated | 10.5170±1.0399 | 14.3282±1.5168 | 1.2833±0.1396 | 10.2164±1.0753 |
| A3 | cube+board | U | FK-fixed | 10.5042±1.0581 | 14.3433±1.5436 | 0.7713±0.0330 | 10.5042±1.0581 |
| B1 | cube+board | seq | FK-fixed | 10.5238±1.0586 | 14.3695±1.5444 | 0.7827±0.0361 | 10.5238±1.0586 |
| B2 | cube | U | FK-fixed | 10.4751±1.0550 | 14.3035±1.5392 | 0.7691±0.0308 | 10.4751±1.0550 |
| B3 | board | U | — | 10.9129±1.0401 | 14.7273±1.5260 | 2.5584±0.0869 | — |
| A2@λ=3 | cube+board | U | vision-estimated + soft FK anchor (λ=3 px/mm) | 10.4968±1.0487 | 14.3246±1.5301 | 0.9316±0.0644 | 10.2675±1.0736 |

`±` is the standard deviation over split means; each split value is the mean of its 5 initializations.
`own-pose cube e2h` re-scores the same corners with the row's own fitted cube poses and is diagnostic only — it is not comparable across rows because the target itself differs.

## Paired per-split deltas (second row minus first, negative is better)

| Contrast | pooled (px) | cam 0 (px) | cam 1 (px) | second better |
| --- | ---: | ---: | ---: | ---: |
| A0→A1 | -0.3362±0.0153 | -0.3187±0.0251 | -1.2567±0.0991 | 5/5 |
| A1→A2 | -0.0598±0.0092 | -0.0804±0.0092 | -0.0194±0.0399 | 5/5 |
| A2→A3 | -0.0128±0.0212 | +0.0151±0.0332 | -0.5121±0.1131 | 3/5 |
| B1→A3 | -0.0196±0.0011 | -0.0262±0.0016 | -0.0114±0.0035 | 5/5 |
| B2→A3 | +0.0291±0.0042 | +0.0398±0.0059 | +0.0022±0.0037 | 0/5 |
| B3→A3 | -0.4087±0.0266 | -0.3840±0.0312 | -1.7871±0.0639 | 5/5 |
| B3→A0 | +0.0001±0.0001 | -0.0000±0.0001 | +0.0011±0.0009 | 1/5 |
| A2→A2@λ=3 | -0.0202±0.0096 | -0.0035±0.0155 | -0.3517±0.0754 | 5/5 |
| A3→A2@λ=3 | -0.0074±0.0124 | -0.0187±0.0180 | +0.1604±0.0392 | 3/5 |
| B3→A2@λ=3 | -0.4162±0.0199 | -0.4027±0.0233 | -1.6268±0.0489 | 5/5 |

`second better` counts splits on the pooled metric. Deltas are paired within a split, so the spread here is the meaningful error bar, not the per-row `±` above.

## Coverage dropped by the common-camera restriction

| split seed | scored cams | dropped cams | corners scored | corners dropped |
| --- | --- | --- | ---: | ---: |
| 20260729 | [0, 1] | [3] | 492 | 208 |
| 20260730 | [0, 1] | [3] | 512 | 228 |
| 20260731 | [0, 1] | [3] | 512 | 216 |
| 20260732 | [0, 1] | [3] | 508 | 228 |
| 20260733 | [0, 1] | [3] | 544 | 236 |

Dropped corners are cube corners seen only by cameras that board-only rows never registered. They are a coverage loss of those rows, not an accuracy result, and are excluded from the RMSE above rather than silently averaged in.

The shared FK cube pose is not external physical ground truth: it comes from train eih cube corners and raw FK, so a common error floor applies to every row. Report these values as `e_reproj^{cube|FK-fixed}` and do not read them as absolute accuracy.
