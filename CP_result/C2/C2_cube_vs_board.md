# Calibration Mode Comparison

Planar board seed, cube-only, and hybrid refinement were re-evaluated on the same dataset.

| mode | num_base_cameras | base_cameras | cross_camera_mean_mm | cube_reproj_mean_px | board_reproj_mean_px | mesh_rmse_mm | dimension_err_mm | pose_repeat_mm | pose_repeat_deg | handeye_pass |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| board_only | 2 | cam0, cam1 | 28.26 | 0.619 | 0.671 | 3.38 | 1.86 | 28.26 | 16.407 | PASS |
| cube_only | 3 | cam0, cam1, cam3 | 4.59 | 0.619 | 0.671 | 3.35 | 1.81 | 4.60 | 2.068 | PASS |
| hybrid | 3 | cam0, cam1, cam3 | 3.95 | 0.619 | 0.671 | 3.25 | 1.83 | 3.95 | 1.873 | PASS |
