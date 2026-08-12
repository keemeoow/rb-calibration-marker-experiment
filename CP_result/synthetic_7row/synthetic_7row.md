# Custom-GT canonical seven-row synthetic sweep

This is not METRIC. The headline error is RMS over calibration transforms common to every row (`T_base_Ci` and `T_gripper_cam`), evaluated against exact GT.

| Row | sigma=0 px (mm/deg) | sigma=0.5 px (mm/deg) | sigma=1 px (mm/deg) | sigma=2 px (mm/deg) |
| --- | ---: | ---: | ---: | ---: |
| A0 | 0.0000±0.0000 / 0.0000±0.0000 | 79.4123±30.8424 / 3.6716±1.6905 | 130.1191±32.3859 / 5.9538±1.7338 | 202.0396±45.1105 / 9.6223±2.1562 |
| A1 | 0.0000±0.0000 / 0.0000±0.0000 | 21.9581±14.3231 / 0.8849±0.5592 | 44.8301±27.7913 / 1.7986±1.1009 | 93.2491±55.5227 / 3.7817±2.2779 |
| A2 | 0.0000±0.0000 / 0.0000±0.0000 | mixed (6/10) | mixed (3/10) | unstable (0/10) |
| A3 | 0.0000±0.0000 / 0.0000±0.0000 | 2.0814±0.5518 / 0.1099±0.0306 | 4.1766±1.1745 / 0.2200±0.0651 | 8.7136±2.4322 / 0.4587±0.1373 |
| B1 | 0.0000±0.0000 / 0.0000±0.0000 | 2.9814±1.4306 / 0.1577±0.0820 | 6.0799±2.7919 / 0.3215±0.1603 | 13.0314±5.4249 / 0.6894±0.3135 |
| B2 | 0.0000±0.0000 / 0.0000±0.0000 | 3.5592±0.6644 / 0.1919±0.0375 | 7.1300±1.4631 / 0.3841±0.0823 | 14.5894±3.4671 / 0.7852±0.1946 |
| B3 | 0.0000±0.0000 / 0.0000±0.0000 | 61.5659±32.9428 / 2.8209±1.6610 | mixed (9/10) | 189.7227±31.9070 / 8.9634±1.9877 |

Deltas in the effect CSV are second row minus first row; negative is better.
FK-noise sweep uses the stated pixel noise and perturbs the shared fixed cube poses byte-identically for B1/A3/B2 within each trial.
