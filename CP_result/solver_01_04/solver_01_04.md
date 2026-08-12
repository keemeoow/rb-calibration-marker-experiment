# Fixed-camera solver comparison 01–04

All methods use identical train/test observations. Methods 03 and 04 start from the byte-identical method-02 state; 04 has no pose regularizer and no fallback. The primary metric transfers a held-out cube pose measured in one camera into another camera and reports coordinate RMSE.

These are held-out real-data agreement metrics without external physical GT.

| Method | status | held-out transfer (px) | e_cross (mm/°) | train reproj (px) | runtime (s) |
| --- | --- | ---: | ---: | ---: | ---: |
| 01_pnp_mean | closed_form | 37.9321 | 67.938/59.601 | 16.7912 | 0.008 |
| 02_pnp_robust_se3 | closed_form | 29.8026 | 57.162/61.640 | 9.3070 | 0.031 |
| 03_pose_consistency | converged | 29.8609 | 57.259/61.602 | 7.4401 | 167.016 |
| 04_direct_reprojection | converged | 29.4867 | 56.610/61.695 | 5.5139 | 265.376 |

Evaluation mask: `73dfce0954c941f449e4b847cb978c2500bd02ff7e1d5e0d468ef31a759d26b2`; 264 ordered transfers and 132 unordered cross-camera pairs.

Train reprojection is diagnostic only. It must not replace the held-out transfer metric when ranking the solvers.
