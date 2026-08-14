# Canonical fixed-split seven-row ablation

This directory contains the canonical real-data fixed-split run after the
corner-reprojection, convergence, board-free FK artifact, and evaluation-mask
contracts were frozen.

- 7 rows × 3 initialization seeds: 21/21 converged with SciPy status 2 (`ftol`).
- Path mask SHA-256: `8122b1f886a40ab94fb5cb223c12e3926f43148841a3f28f62343eecf49b76a3`.
- Common cube evaluation population: 113 observations, 81 `e_cross` pairs,
  29 `e_e2e` units, zero model-output rejection.
- `seven_row_ablation.md` reports overall, target-component, path, and
  per-camera metrics. `seven_row_ablation.json` retains every evaluated pair.

The reported standard deviation is across three initialization seeds on one
fixed event split. It is not a confidence interval over data collection. A
multi-split/seed run is required before treating the values as final uncertainty
estimates. No external physical ground truth is available here; this directory
does not report absolute task-pose accuracy.
