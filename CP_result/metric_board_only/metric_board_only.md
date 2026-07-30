# METRIC medium_workcell — board-only external-GT baseline

This is a separate four-camera eye-on-base checkerboard experiment. It does not implement the cube/eih/FK→cube axes of the canonical seven-row ablation.

GT metrics are RMS over the four `T_base_cam` transform errors. Reprojection is a train diagnostic, not the external-GT metric.

| Method | status | e_t RMS (mm) | e_r RMS (°) | train reproj (px) | runtime (s) |
| --- | --- | ---: | ---: | ---: | ---: |
| Tsai-Lenz | converged | 1557.6214 | 63.3231 | 307.5653 | 0.176 |
| Park-Martin | converged | 173.2694 | 2.4656 | 37.7054 | 0.171 |
| Horaud | converged | 172.1610 | 2.1532 | 36.4326 | 0.178 |
| Daniilidis | converged | 367.1478 | 3.3385 | 73.9186 | 0.177 |
| Joint corner reprojection (compatible) | converged | 0.9474 | 0.0626 | 0.2407 | 13.695 |

Detected views per camera: `{'1': 52, '2': 99, '3': 90, '4': 72}`; unique robot poses: 251.

`Joint corner reprojection (compatible)` reproduces the documented eye-on-base transform chain and shared board variable in Python/SciPy. It is not labelled as an execution of the bundled Allegro C++/Ceres binary because `cmake` was unavailable in this environment.
