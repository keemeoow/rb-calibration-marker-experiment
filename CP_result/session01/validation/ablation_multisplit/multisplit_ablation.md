# Canonical repeated event-split ablation

Deltas are second row minus first row. Negative is better for all error metrics; positive is better only for N_reg. Split standard deviation is kept separate from within-split initialization standard deviation.

| Contrast | Component | splits | delta mean±split std | second better |
| --- | --- | ---: | ---: | ---: |
| A0_to_A1 | heldout_reprojection.board | 5 | 0.003148±0.002969 | 1/5 |
| A0_to_A1 | N_reg | 5 | 1.000000±0.000000 | 5/5 |
| A1_to_A2 | heldout_reprojection.overall | 5 | -0.148582±0.021607 | 5/5 |
| B1_to_A3 | heldout_reprojection.overall | 5 | -0.008689±0.002200 | 5/5 |
| A2_to_A3 | heldout_reprojection.overall | 5 | 0.115545±0.025289 | 0/5 |
| B2_to_A3 | heldout_reprojection.cube | 5 | 0.029603±0.002726 | 0/5 |
| B2_to_A3 | e_e2e | 5 | 0.499380±0.067601 | 0/5 |
| B2_to_A3 | e_cross | 5 | 0.188608±0.009011 | 0/5 |
| B3_to_A3 | heldout_reprojection.board | 5 | 0.009552±0.022378 | 1/5 |

## Held-out reprojection by camera across splits

Values are mean of split means ± standard deviation across split means.

| Row | cam 0 (px) | cam 1 (px) | cam 2 (px) | cam 3 (px) |
| --- | ---: | ---: | ---: | ---: |
| A0 | 0.3108±0.0039 | 0.4719±0.0247 | 1.5502±0.1091 | — |
| A1 | 11.0213±1.2013 | 1.0611±0.0672 | 1.6426±0.0946 | 1.0512±0.0358 |
| A2 | 10.6542±1.1641 | 0.8429±0.0371 | 1.6436±0.0895 | 0.8426±0.0132 |
| A3 | 10.9611±1.1446 | 0.7315±0.0310 | 1.6340±0.0906 | 0.9916±0.0227 |
| B1 | 10.9829±1.1448 | 0.7616±0.0279 | 1.6336±0.0917 | 0.9916±0.0227 |
| B2 | 14.3035±1.5392 | 0.7691±0.0308 | 1.8840±0.0819 | 0.9916±0.0227 |
| B3 | 0.3108±0.0039 | 0.4719±0.0247 | 1.5502±0.1090 | — |

B3_to_A3 remains a whole-system reference and has no marker-only causal interpretation.
No position-holdout value in this artifact is external physical ground truth.
