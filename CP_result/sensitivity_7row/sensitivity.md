# Table 4 — controlled real-data sensitivity

Primary values are native-frame held-out corner reprojection RMSE (px), mean±population-std across predeclared subset seeds. A numeric headline is withheld unless every subset and optimizer initialization converged.

- N is the total number of robot capture events, balanced equally over the fixed positions `[0, 2, 6, 9, 12]`.
- Camera count includes the eye-in-hand camera. Camera subsets are nested; evaluation always uses eye-in-hand plus fixed camera 0.
- `resolution_low` reruns marker detection at the declared raster scale, maps corners back to native pixels, and evaluates on the same native held-out pool.
- These real-data values use held-out measurement agreement, not external physical ground truth.

## Row-specific overall reprojection

Target sets differ across columns, so this table is a within-row sensitivity summary; it is not a cross-row accuracy ranking.

| condition | A0 | A2 | B2 | A3 |
| --- | ---: | ---: | ---: | ---: |
| views N=5 | 3.3656±2.1833 | unstable (2/5) | 13.9700±0.4614 | 7.4920±0.2142 |
| views N=10 | 1.5475±0.1930 | 7.1317±0.0789 | 13.5922±0.0562 | 7.2852±0.0279 |
| views N=20 | 1.6010±0.1078 | 7.1166±0.0269 | 13.5665±0.0255 | 7.2777±0.0053 |
| views N=40 | 1.5290±0.0201 | 7.0943±0.0036 | 13.5452±0.0032 | 7.2618±0.0031 |
| cams=2 | 1.5290±0.0201 | 6.5805±0.0046 | 13.5453±0.0032 | 7.2668±0.0032 |
| cams=3 | 1.5290±0.0201 | 7.0783±0.0044 | 13.5452±0.0032 | 7.2618±0.0031 |
| cams=4 | 1.5290±0.0201 | 7.0943±0.0036 | 13.5452±0.0032 | 7.2618±0.0031 |
| resolution=low | unstable (0/5) | 7.5783±0.0324 | 13.9681±0.0026 | 7.4964±0.0061 |
| resolution=native | 1.5290±0.0201 | 7.0943±0.0036 | 13.5452±0.0032 | 7.2618±0.0031 |

## Common-cube comparison — B2 versus A3

This is the valid board-addition comparison. Both cells use only held-out cube corners; Δ = A3−B2, so a negative value favors adding the board.

| condition | B2 cube (px) | A3 cube (px) | paired Δ (px) |
| --- | ---: | ---: | ---: |
| views N=5 | 13.9700±0.4614 | 13.9186±0.4041 | -0.0514±0.1058 (3/5) |
| views N=10 | 13.5922±0.0562 | 13.6849±0.0482 | +0.0927±0.0278 (0/5) |
| views N=20 | 13.5665±0.0255 | 13.6695±0.0138 | +0.1030±0.0164 (0/5) |
| views N=40 | 13.5452±0.0032 | 13.6540±0.0042 | +0.1088±0.0026 (0/5) |
| cams=2 | 13.5453±0.0032 | 13.6598±0.0042 | +0.1145±0.0026 (0/5) |
| cams=3 | 13.5452±0.0032 | 13.6540±0.0042 | +0.1088±0.0026 (0/5) |
| cams=4 | 13.5452±0.0032 | 13.6540±0.0042 | +0.1088±0.0026 (0/5) |
| resolution=low | 13.9681±0.0026 | 13.9798±0.0017 | +0.0118±0.0011 (0/5) |
| resolution=native | 13.5452±0.0032 | 13.6540±0.0042 | +0.1088±0.0026 (0/5) |

Raw per-subset convergence, corner support, solver diagnostics, and artifact hashes are retained in `sensitivity.json`.
