# session04 Cube Observation Quality

- 판정 단위: `event × camera`
- 관측 선택 정책: `core_multiface`
- Gripped events: `제외`
- Cube config: `meta`

## 전체 요약

| 항목 | 결과 |
|---|---:|
| 전체 events | 78 |
| Core 관측 보유 events | 57 |
| 선택된 core 관측 | 96 |
| Legacy 기준 관측 | 106 |
| Single-marker 관측 | 10 |
| Planar multi-tag 관측 | 0 |
| PnP RMSE median | 1.066 px |
| PnP RMSE range | 0.134–2.847 px |

## Core 관측 조건

Calibration 핵심 관측은 아래 조건을 모두 만족합니다.

- 관측 face 수 ≥ 2
- Non-coplanar face 수 ≥ 2
- 3D corner 구성이 non-planar
- Positive-depth PnP 후보 수 ≥ 1

`Positive / all`은 모든 3D corner가 카메라 앞쪽(`z > 0`)에 놓이는 PnP 후보 수와 전체 후보 수입니다.

## Event 요약

| Event | Set | 결과 | PnP cameras | Core cameras | Marker IDs | Faces |
|---:|---:|---|---|---|---|---|
| 00 | 0 | CORE | 0, 1, 2, 3 | 0, 1, 2, 3 | 0, 1, 2, 3, 4, 5 | +X, +Y, +Z, -X, -Y |
| 01 | 0 | SINGLE MARKER | 2 | — | 1 | +Z |
| 02 | 0 | CORE | 2 | 2 | 1, 3 | +Y, +Z |
| 03 | 0 | NO OBSERVATION | — | — | — | — |
| 04 | 0 | SINGLE MARKER | 2 | — | 3 | +Y |
| 05 | 0 | SINGLE MARKER | 2 | — | 3 | +Y |
| 06 | 1 | CORE | 0, 1, 2, 3 | 0, 1, 2, 3 | 0, 1, 2, 3, 4, 5 | +X, +Y, +Z, -X, -Y |
| 07 | 1 | SINGLE MARKER | 2 | — | 3 | +Y |
| 08 | 1 | CORE | 2 | 2 | 0, 1, 3, 4 | +Y, +Z, -X |
| 09 | 1 | SINGLE MARKER | 2 | — | 3 | +Y |
| 10 | 1 | SINGLE MARKER | 2 | — | 3 | +Y |
| 11 | 1 | NO OBSERVATION | — | — | — | — |
| 12 | 2 | CORE | 0, 1, 2, 3 | 0, 1, 2, 3 | 0, 1, 2, 3, 4, 5 | +X, +Y, +Z, -X, -Y |
| 13 | 2 | CORE | 2 | 2 | 1, 4, 5 | +Z, -X, -Y |
| 14 | 2 | NO OBSERVATION | — | — | — | — |
| 15 | 2 | NO OBSERVATION | — | — | — | — |
| 16 | 2 | CORE | 2 | 2 | 0, 4, 5 | +Z, -X, -Y |
| 17 | 2 | NO OBSERVATION | — | — | — | — |
| 18 | 3 | CORE | 0, 1, 2, 3 | 0, 1, 2, 3 | 0, 1, 2, 3, 4, 5 | +X, +Y, +Z, -X, -Y |
| 19 | 3 | NO OBSERVATION | — | — | — | — |
| 20 | 3 | NO OBSERVATION | — | — | — | — |
| 21 | 3 | SINGLE MARKER | 2 | — | 4 | -X |
| 22 | 3 | CORE | 2 | 2 | 1, 3 | +Y, +Z |
| 23 | 3 | NO OBSERVATION | — | — | — | — |
| 24 | 4 | CORE | 0, 1, 2, 3 | 0, 1, 2, 3 | 0, 1, 2, 3, 4, 5 | +X, +Y, +Z, -X, -Y |
| 25 | 4 | SINGLE MARKER | 2 | — | 3 | +Y |
| 26 | 4 | CORE | 2 | 2 | 0, 1, 4 | +Z, -X |
| 27 | 4 | CORE | 2 | 2 | 0, 1, 2, 3 | +X, +Y, +Z |
| 28 | 4 | NO OBSERVATION | — | — | — | — |
| 29 | 4 | CORE | 2 | 2 | 3, 4 | +Y, -X |
| 30 | 5 | CORE | 0, 1, 2, 3 | 0, 1, 2, 3 | 0, 1, 2, 3, 4, 5 | +X, +Y, +Z, -X, -Y |
| 31 | 5 | CORE | 2 | 2 | 2, 5 | +X, -Y |
| 32 | 5 | CORE | 2 | 2 | 0, 1, 3 | +Y, +Z |
| 33 | 5 | CORE | 2 | 2 | 0, 1, 2, 5 | +X, +Z, -Y |
| 34 | 5 | NO OBSERVATION | — | — | — | — |
| 35 | 5 | CORE | 2 | 2 | 0, 1, 3 | +Y, +Z |
| 36 | 6 | CORE | 0, 1, 2, 3 | 0, 1, 2, 3 | 0, 1, 2, 3, 4, 5 | +X, +Y, +Z, -X, -Y |
| 37 | 6 | CORE | 2 | 2 | 0, 1, 3, 4 | +Y, +Z, -X |
| 38 | 6 | CORE | 2 | 2 | 0, 1, 4, 5 | +Z, -X, -Y |
| 39 | 6 | CORE | 2 | 2 | 1, 3, 4 | +Y, +Z, -X |
| 40 | 6 | NO OBSERVATION | — | — | — | — |
| 41 | 6 | CORE | 2 | 2 | 0, 1, 4, 5 | +Z, -X, -Y |
| 42 | 7 | CORE | 0, 1, 2, 3 | 0, 1, 2, 3 | 0, 1, 2, 3, 4, 5 | +X, +Y, +Z, -X, -Y |
| 43 | 7 | CORE | 2 | 2 | 0, 1, 2, 3 | +X, +Y, +Z |
| 44 | 7 | CORE | 2 | 2 | 0, 1, 3, 4 | +Y, +Z, -X |
| 45 | 7 | CORE | 2 | 2 | 0, 1, 2 | +X, +Z |
| 46 | 7 | CORE | 2 | 2 | 0, 1, 3, 4 | +Y, +Z, -X |
| 47 | 7 | SINGLE MARKER | 2 | — | 3 | +Y |
| 48 | 8 | CORE | 0, 1, 2, 3 | 0, 1, 2, 3 | 0, 1, 3, 4, 5 | +Y, +Z, -X, -Y |
| 49 | 8 | CORE | 2 | 2 | 0, 1, 5 | +Z, -Y |
| 50 | 8 | CORE | 2 | 2 | 0, 1, 4 | +Z, -X |
| 51 | 8 | CORE | 2 | 2 | 0, 1, 4, 5 | +Z, -X, -Y |
| 52 | 8 | CORE | 2 | 2 | 0, 1, 4, 5 | +Z, -X, -Y |
| 53 | 8 | CORE | 2 | 2 | 0, 1, 4, 5 | +Z, -X, -Y |
| 54 | 9 | CORE | 0, 1, 2, 3 | 0, 1, 2, 3 | 0, 1, 2, 3, 4, 5 | +X, +Y, +Z, -X, -Y |
| 55 | 9 | CORE | 2 | 2 | 0, 1, 4 | +Z, -X |
| 56 | 9 | CORE | 2 | 2 | 0, 3 | +Y, +Z |
| 57 | 9 | CORE | 2 | 2 | 0, 1, 3, 4 | +Y, +Z, -X |
| 58 | 9 | CORE | 2 | 2 | 0, 1, 3, 4 | +Y, +Z, -X |
| 59 | 9 | CORE | 2 | 2 | 0, 1, 3 | +Y, +Z |
| 60 | 10 | CORE | 0, 1, 2, 3 | 0, 1, 2, 3 | 0, 1, 2, 3, 4, 5 | +X, +Y, +Z, -X, -Y |
| 61 | 10 | CORE | 2 | 2 | 0, 1, 3, 4 | +Y, +Z, -X |
| 62 | 10 | CORE | 2 | 2 | 0, 1, 2, 3 | +X, +Y, +Z |
| 63 | 10 | CORE | 2 | 2 | 0, 1, 4 | +Z, -X |
| 64 | 10 | CORE | 2 | 2 | 1, 3 | +Y, +Z |
| 65 | 10 | CORE | 2 | 2 | 0, 1, 3 | +Y, +Z |
| 66 | 11 | CORE | 0, 1, 2, 3 | 0, 1, 2, 3 | 0, 1, 2, 3, 4, 5 | +X, +Y, +Z, -X, -Y |
| 67 | 11 | CORE | 2 | 2 | 0, 1, 4 | +Z, -X |
| 68 | 11 | CORE | 2 | 2 | 0, 1, 4, 5 | +Z, -X, -Y |
| 69 | 11 | SINGLE MARKER | 2 | — | 4 | -X |
| 70 | 11 | CORE | 2 | 2 | 0, 1, 3, 4 | +Y, +Z, -X |
| 71 | 11 | CORE | 2 | 2 | 1, 3 | +Y, +Z |
| 72 | 12 | CORE | 0, 1, 2, 3 | 0, 1, 2, 3 | 0, 1, 2, 3, 4, 5 | +X, +Y, +Z, -X, -Y |
| 73 | 12 | CORE | 2 | 2 | 0, 1, 5 | +Z, -Y |
| 74 | 12 | CORE | 2 | 2 | 0, 1, 2, 3 | +X, +Y, +Z |
| 75 | 12 | CORE | 2 | 2 | 0, 5 | +Z, -Y |
| 76 | 12 | CORE | 2 | 2 | 0, 1, 2, 5 | +X, +Z, -Y |
| 77 | 12 | CORE | 2 | 2 | 0, 1, 4, 5 | +Z, -X, -Y |

## Event별 camera 상세

각 Event를 펼치면 camera별 판정 근거를 확인할 수 있습니다.

<details><summary><strong>Event 00</strong> · set 0 · CORE</summary>

| Camera | Marker IDs | Faces | Face support | Planarity | PnP RMSE | Positive / all | Quality | Calibration |
|---|---|---|---|---|---:|---:|---|---|
| cam0 | 0, 1, 2, 5 | +X, +Z, -Y | 3 faces / 3 non-coplanar | non-planar | 2.031 px | 1 / 1 | nonplanar multiface | SELECTED |
| cam1 | 0, 3, 4 | +Y, +Z, -X | 3 faces / 3 non-coplanar | non-planar | 1.110 px | 1 / 1 | nonplanar multiface | SELECTED |
| cam2 (gripper) | 0, 1, 2 | +X, +Z | 2 faces / 2 non-coplanar | non-planar | 2.397 px | 1 / 1 | nonplanar multiface | SELECTED |
| cam3 | 2, 3 | +X, +Y | 2 faces / 2 non-coplanar | non-planar | 0.647 px | 1 / 1 | nonplanar multiface | SELECTED |

</details>

<details><summary><strong>Event 01</strong> · set 0 · SINGLE MARKER</summary>

| Camera | Marker IDs | Faces | Face support | Planarity | PnP RMSE | Positive / all | Quality | Calibration |
|---|---|---|---|---|---:|---:|---|---|
| cam2 (gripper) | 1 | +Z | 1 faces / 0 non-coplanar | planar | 0.291 px | 2 / 2 | single marker | noncore_single_marker |

</details>

<details><summary><strong>Event 02</strong> · set 0 · CORE</summary>

| Camera | Marker IDs | Faces | Face support | Planarity | PnP RMSE | Positive / all | Quality | Calibration |
|---|---|---|---|---|---:|---:|---|---|
| cam2 (gripper) | 1, 3 | +Y, +Z | 2 faces / 2 non-coplanar | non-planar | 1.126 px | 1 / 1 | nonplanar multiface | SELECTED |

</details>

<details><summary><strong>Event 03</strong> · set 0 · NO OBSERVATION</summary>

| Camera | Marker IDs | Faces | Face support | Planarity | PnP RMSE | Positive / all | Quality | Calibration |
|---|---|---|---|---|---:|---:|---|---|
| cam2 (gripper) | — | — | 0 faces / 0 non-coplanar | — | — | 0 / 0 | none | pnp_not_accepted |

</details>

<details><summary><strong>Event 04</strong> · set 0 · SINGLE MARKER</summary>

| Camera | Marker IDs | Faces | Face support | Planarity | PnP RMSE | Positive / all | Quality | Calibration |
|---|---|---|---|---|---:|---:|---|---|
| cam2 (gripper) | 3 | +Y | 1 faces / 0 non-coplanar | planar | 0.436 px | 2 / 2 | single marker | noncore_single_marker |

</details>

<details><summary><strong>Event 05</strong> · set 0 · SINGLE MARKER</summary>

| Camera | Marker IDs | Faces | Face support | Planarity | PnP RMSE | Positive / all | Quality | Calibration |
|---|---|---|---|---|---:|---:|---|---|
| cam2 (gripper) | 3 | +Y | 1 faces / 0 non-coplanar | planar | 0.689 px | 2 / 2 | single marker | noncore_single_marker |

</details>

<details><summary><strong>Event 06</strong> · set 1 · CORE</summary>

| Camera | Marker IDs | Faces | Face support | Planarity | PnP RMSE | Positive / all | Quality | Calibration |
|---|---|---|---|---|---:|---:|---|---|
| cam0 | 0, 1, 5 | +Z, -Y | 2 faces / 2 non-coplanar | non-planar | 0.892 px | 1 / 1 | nonplanar multiface | SELECTED |
| cam1 | 1, 3, 4 | +Y, +Z, -X | 3 faces / 3 non-coplanar | non-planar | 0.928 px | 1 / 1 | nonplanar multiface | SELECTED |
| cam2 (gripper) | 0, 1, 5 | +Z, -Y | 2 faces / 2 non-coplanar | non-planar | 0.884 px | 1 / 1 | nonplanar multiface | SELECTED |
| cam3 | 2, 3 | +X, +Y | 2 faces / 2 non-coplanar | non-planar | 1.646 px | 1 / 1 | nonplanar multiface | SELECTED |

</details>

<details><summary><strong>Event 07</strong> · set 1 · SINGLE MARKER</summary>

| Camera | Marker IDs | Faces | Face support | Planarity | PnP RMSE | Positive / all | Quality | Calibration |
|---|---|---|---|---|---:|---:|---|---|
| cam2 (gripper) | 3 | +Y | 1 faces / 0 non-coplanar | planar | 0.718 px | 2 / 2 | single marker | noncore_single_marker |

</details>

<details><summary><strong>Event 08</strong> · set 1 · CORE</summary>

| Camera | Marker IDs | Faces | Face support | Planarity | PnP RMSE | Positive / all | Quality | Calibration |
|---|---|---|---|---|---:|---:|---|---|
| cam2 (gripper) | 0, 1, 3, 4 | +Y, +Z, -X | 3 faces / 3 non-coplanar | non-planar | 2.366 px | 1 / 1 | nonplanar multiface | SELECTED |

</details>

<details><summary><strong>Event 09</strong> · set 1 · SINGLE MARKER</summary>

| Camera | Marker IDs | Faces | Face support | Planarity | PnP RMSE | Positive / all | Quality | Calibration |
|---|---|---|---|---|---:|---:|---|---|
| cam2 (gripper) | 3 | +Y | 1 faces / 0 non-coplanar | planar | 0.161 px | 2 / 2 | single marker | noncore_single_marker |

</details>

<details><summary><strong>Event 10</strong> · set 1 · SINGLE MARKER</summary>

| Camera | Marker IDs | Faces | Face support | Planarity | PnP RMSE | Positive / all | Quality | Calibration |
|---|---|---|---|---|---:|---:|---|---|
| cam2 (gripper) | 3 | +Y | 1 faces / 0 non-coplanar | planar | 0.231 px | 2 / 2 | single marker | noncore_single_marker |

</details>

<details><summary><strong>Event 11</strong> · set 1 · NO OBSERVATION</summary>

| Camera | Marker IDs | Faces | Face support | Planarity | PnP RMSE | Positive / all | Quality | Calibration |
|---|---|---|---|---|---:|---:|---|---|
| cam2 (gripper) | 0, 1, 3, 4 | +Y, +Z, -X | 3 faces / 3 non-coplanar | non-planar | 5.418 px | 1 / 1 | nonplanar multiface | pnp_not_accepted |

</details>

<details><summary><strong>Event 12</strong> · set 2 · CORE</summary>

| Camera | Marker IDs | Faces | Face support | Planarity | PnP RMSE | Positive / all | Quality | Calibration |
|---|---|---|---|---|---:|---:|---|---|
| cam0 | 0, 1, 2 | +X, +Z | 2 faces / 2 non-coplanar | non-planar | 0.817 px | 1 / 1 | nonplanar multiface | SELECTED |
| cam1 | 4, 5 | -X, -Y | 2 faces / 2 non-coplanar | non-planar | 0.800 px | 1 / 1 | nonplanar multiface | SELECTED |
| cam2 (gripper) | 0, 1, 2, 5 | +X, +Z, -Y | 3 faces / 3 non-coplanar | non-planar | 1.483 px | 1 / 1 | nonplanar multiface | SELECTED |
| cam3 | 3, 4 | +Y, -X | 2 faces / 2 non-coplanar | non-planar | 0.960 px | 1 / 1 | nonplanar multiface | SELECTED |

</details>

<details><summary><strong>Event 13</strong> · set 2 · CORE</summary>

| Camera | Marker IDs | Faces | Face support | Planarity | PnP RMSE | Positive / all | Quality | Calibration |
|---|---|---|---|---|---:|---:|---|---|
| cam2 (gripper) | 1, 4, 5 | +Z, -X, -Y | 3 faces / 3 non-coplanar | non-planar | 1.697 px | 1 / 1 | nonplanar multiface | SELECTED |

</details>

<details><summary><strong>Event 14</strong> · set 2 · NO OBSERVATION</summary>

| Camera | Marker IDs | Faces | Face support | Planarity | PnP RMSE | Positive / all | Quality | Calibration |
|---|---|---|---|---|---:|---:|---|---|
| cam2 (gripper) | — | — | 0 faces / 0 non-coplanar | — | — | 0 / 0 | none | pnp_not_accepted |

</details>

<details><summary><strong>Event 15</strong> · set 2 · NO OBSERVATION</summary>

| Camera | Marker IDs | Faces | Face support | Planarity | PnP RMSE | Positive / all | Quality | Calibration |
|---|---|---|---|---|---:|---:|---|---|
| cam2 (gripper) | 0, 1, 5 | +Z, -Y | 2 faces / 2 non-coplanar | non-planar | 17.185 px | 1 / 1 | nonplanar multiface | pnp_not_accepted |

</details>

<details><summary><strong>Event 16</strong> · set 2 · CORE</summary>

| Camera | Marker IDs | Faces | Face support | Planarity | PnP RMSE | Positive / all | Quality | Calibration |
|---|---|---|---|---|---:|---:|---|---|
| cam2 (gripper) | 0, 4, 5 | +Z, -X, -Y | 3 faces / 3 non-coplanar | non-planar | 1.300 px | 1 / 1 | nonplanar multiface | SELECTED |

</details>

<details><summary><strong>Event 17</strong> · set 2 · NO OBSERVATION</summary>

| Camera | Marker IDs | Faces | Face support | Planarity | PnP RMSE | Positive / all | Quality | Calibration |
|---|---|---|---|---|---:|---:|---|---|
| cam2 (gripper) | — | — | 0 faces / 0 non-coplanar | — | — | 0 / 0 | none | pnp_not_accepted |

</details>

<details><summary><strong>Event 18</strong> · set 3 · CORE</summary>

| Camera | Marker IDs | Faces | Face support | Planarity | PnP RMSE | Positive / all | Quality | Calibration |
|---|---|---|---|---|---:|---:|---|---|
| cam0 | 0, 1, 4, 5 | +Z, -X, -Y | 3 faces / 3 non-coplanar | non-planar | 1.346 px | 1 / 1 | nonplanar multiface | SELECTED |
| cam1 | 3, 4 | +Y, -X | 2 faces / 2 non-coplanar | non-planar | 0.496 px | 1 / 1 | nonplanar multiface | SELECTED |
| cam2 (gripper) | 0, 1, 4, 5 | +Z, -X, -Y | 3 faces / 3 non-coplanar | non-planar | 1.663 px | 1 / 1 | nonplanar multiface | SELECTED |
| cam3 | 2, 3 | +X, +Y | 2 faces / 2 non-coplanar | non-planar | 1.580 px | 1 / 1 | nonplanar multiface | SELECTED |

</details>

<details><summary><strong>Event 19</strong> · set 3 · NO OBSERVATION</summary>

| Camera | Marker IDs | Faces | Face support | Planarity | PnP RMSE | Positive / all | Quality | Calibration |
|---|---|---|---|---|---:|---:|---|---|
| cam2 (gripper) | — | — | 0 faces / 0 non-coplanar | — | — | 0 / 0 | none | pnp_not_accepted |

</details>

<details><summary><strong>Event 20</strong> · set 3 · NO OBSERVATION</summary>

| Camera | Marker IDs | Faces | Face support | Planarity | PnP RMSE | Positive / all | Quality | Calibration |
|---|---|---|---|---|---:|---:|---|---|
| cam2 (gripper) | — | — | 0 faces / 0 non-coplanar | — | — | 0 / 0 | none | pnp_not_accepted |

</details>

<details><summary><strong>Event 21</strong> · set 3 · SINGLE MARKER</summary>

| Camera | Marker IDs | Faces | Face support | Planarity | PnP RMSE | Positive / all | Quality | Calibration |
|---|---|---|---|---|---:|---:|---|---|
| cam2 (gripper) | 4 | -X | 1 faces / 0 non-coplanar | planar | 2.237 px | 2 / 2 | single marker | noncore_single_marker |

</details>

<details><summary><strong>Event 22</strong> · set 3 · CORE</summary>

| Camera | Marker IDs | Faces | Face support | Planarity | PnP RMSE | Positive / all | Quality | Calibration |
|---|---|---|---|---|---:|---:|---|---|
| cam2 (gripper) | 1, 3 | +Y, +Z | 2 faces / 2 non-coplanar | non-planar | 0.830 px | 1 / 1 | nonplanar multiface | SELECTED |

</details>

<details><summary><strong>Event 23</strong> · set 3 · NO OBSERVATION</summary>

| Camera | Marker IDs | Faces | Face support | Planarity | PnP RMSE | Positive / all | Quality | Calibration |
|---|---|---|---|---|---:|---:|---|---|
| cam2 (gripper) | — | — | 0 faces / 0 non-coplanar | — | — | 0 / 0 | none | pnp_not_accepted |

</details>

<details><summary><strong>Event 24</strong> · set 4 · CORE</summary>

| Camera | Marker IDs | Faces | Face support | Planarity | PnP RMSE | Positive / all | Quality | Calibration |
|---|---|---|---|---|---:|---:|---|---|
| cam0 | 0, 1, 3, 4 | +Y, +Z, -X | 3 faces / 3 non-coplanar | non-planar | 0.954 px | 1 / 1 | nonplanar multiface | SELECTED |
| cam1 | 2, 3 | +X, +Y | 2 faces / 2 non-coplanar | non-planar | 0.754 px | 1 / 1 | nonplanar multiface | SELECTED |
| cam2 (gripper) | 0, 1, 3, 4 | +Y, +Z, -X | 3 faces / 3 non-coplanar | non-planar | 0.935 px | 1 / 1 | nonplanar multiface | SELECTED |
| cam3 | 0, 2, 5 | +X, +Z, -Y | 3 faces / 3 non-coplanar | non-planar | 1.960 px | 1 / 1 | nonplanar multiface | SELECTED |

</details>

<details><summary><strong>Event 25</strong> · set 4 · SINGLE MARKER</summary>

| Camera | Marker IDs | Faces | Face support | Planarity | PnP RMSE | Positive / all | Quality | Calibration |
|---|---|---|---|---|---:|---:|---|---|
| cam2 (gripper) | 3 | +Y | 1 faces / 0 non-coplanar | planar | 0.323 px | 2 / 2 | single marker | noncore_single_marker |

</details>

<details><summary><strong>Event 26</strong> · set 4 · CORE</summary>

| Camera | Marker IDs | Faces | Face support | Planarity | PnP RMSE | Positive / all | Quality | Calibration |
|---|---|---|---|---|---:|---:|---|---|
| cam2 (gripper) | 0, 1, 4 | +Z, -X | 2 faces / 2 non-coplanar | non-planar | 1.371 px | 1 / 1 | nonplanar multiface | SELECTED |

</details>

<details><summary><strong>Event 27</strong> · set 4 · CORE</summary>

| Camera | Marker IDs | Faces | Face support | Planarity | PnP RMSE | Positive / all | Quality | Calibration |
|---|---|---|---|---|---:|---:|---|---|
| cam2 (gripper) | 0, 1, 2, 3 | +X, +Y, +Z | 3 faces / 3 non-coplanar | non-planar | 2.447 px | 1 / 1 | nonplanar multiface | SELECTED |

</details>

<details><summary><strong>Event 28</strong> · set 4 · NO OBSERVATION</summary>

| Camera | Marker IDs | Faces | Face support | Planarity | PnP RMSE | Positive / all | Quality | Calibration |
|---|---|---|---|---|---:|---:|---|---|
| cam2 (gripper) | — | — | 0 faces / 0 non-coplanar | — | — | 0 / 0 | none | pnp_not_accepted |

</details>

<details><summary><strong>Event 29</strong> · set 4 · CORE</summary>

| Camera | Marker IDs | Faces | Face support | Planarity | PnP RMSE | Positive / all | Quality | Calibration |
|---|---|---|---|---|---:|---:|---|---|
| cam2 (gripper) | 3, 4 | +Y, -X | 2 faces / 2 non-coplanar | non-planar | 0.900 px | 1 / 1 | nonplanar multiface | SELECTED |

</details>

<details><summary><strong>Event 30</strong> · set 5 · CORE</summary>

| Camera | Marker IDs | Faces | Face support | Planarity | PnP RMSE | Positive / all | Quality | Calibration |
|---|---|---|---|---|---:|---:|---|---|
| cam0 | 1, 2, 3 | +X, +Y, +Z | 3 faces / 3 non-coplanar | non-planar | 1.132 px | 1 / 1 | nonplanar multiface | SELECTED |
| cam1 | 2, 5 | +X, -Y | 2 faces / 2 non-coplanar | non-planar | 1.040 px | 1 / 1 | nonplanar multiface | SELECTED |
| cam2 (gripper) | 2, 3 | +X, +Y | 2 faces / 2 non-coplanar | non-planar | 0.880 px | 1 / 1 | nonplanar multiface | SELECTED |
| cam3 | 0, 1, 4, 5 | +Z, -X, -Y | 3 faces / 3 non-coplanar | non-planar | 1.414 px | 1 / 1 | nonplanar multiface | SELECTED |

</details>

<details><summary><strong>Event 31</strong> · set 5 · CORE</summary>

| Camera | Marker IDs | Faces | Face support | Planarity | PnP RMSE | Positive / all | Quality | Calibration |
|---|---|---|---|---|---:|---:|---|---|
| cam2 (gripper) | 2, 5 | +X, -Y | 2 faces / 2 non-coplanar | non-planar | 1.083 px | 1 / 1 | nonplanar multiface | SELECTED |

</details>

<details><summary><strong>Event 32</strong> · set 5 · CORE</summary>

| Camera | Marker IDs | Faces | Face support | Planarity | PnP RMSE | Positive / all | Quality | Calibration |
|---|---|---|---|---|---:|---:|---|---|
| cam2 (gripper) | 0, 1, 3 | +Y, +Z | 2 faces / 2 non-coplanar | non-planar | 1.664 px | 1 / 1 | nonplanar multiface | SELECTED |

</details>

<details><summary><strong>Event 33</strong> · set 5 · CORE</summary>

| Camera | Marker IDs | Faces | Face support | Planarity | PnP RMSE | Positive / all | Quality | Calibration |
|---|---|---|---|---|---:|---:|---|---|
| cam2 (gripper) | 0, 1, 2, 5 | +X, +Z, -Y | 3 faces / 3 non-coplanar | non-planar | 1.151 px | 1 / 1 | nonplanar multiface | SELECTED |

</details>

<details><summary><strong>Event 34</strong> · set 5 · NO OBSERVATION</summary>

| Camera | Marker IDs | Faces | Face support | Planarity | PnP RMSE | Positive / all | Quality | Calibration |
|---|---|---|---|---|---:|---:|---|---|
| cam2 (gripper) | — | — | 0 faces / 0 non-coplanar | — | — | 0 / 0 | none | pnp_not_accepted |

</details>

<details><summary><strong>Event 35</strong> · set 5 · CORE</summary>

| Camera | Marker IDs | Faces | Face support | Planarity | PnP RMSE | Positive / all | Quality | Calibration |
|---|---|---|---|---|---:|---:|---|---|
| cam2 (gripper) | 0, 1, 3 | +Y, +Z | 2 faces / 2 non-coplanar | non-planar | 0.973 px | 1 / 1 | nonplanar multiface | SELECTED |

</details>

<details><summary><strong>Event 36</strong> · set 6 · CORE</summary>

| Camera | Marker IDs | Faces | Face support | Planarity | PnP RMSE | Positive / all | Quality | Calibration |
|---|---|---|---|---|---:|---:|---|---|
| cam0 | 4, 5 | -X, -Y | 2 faces / 2 non-coplanar | non-planar | 1.425 px | 1 / 1 | nonplanar multiface | SELECTED |
| cam1 | 1, 3, 4 | +Y, +Z, -X | 3 faces / 3 non-coplanar | non-planar | 1.074 px | 1 / 1 | nonplanar multiface | SELECTED |
| cam2 (gripper) | 0, 1, 4 | +Z, -X | 2 faces / 2 non-coplanar | non-planar | 0.892 px | 1 / 1 | nonplanar multiface | SELECTED |
| cam3 | 0, 1, 2, 5 | +X, +Z, -Y | 3 faces / 3 non-coplanar | non-planar | 1.024 px | 1 / 1 | nonplanar multiface | SELECTED |

</details>

<details><summary><strong>Event 37</strong> · set 6 · CORE</summary>

| Camera | Marker IDs | Faces | Face support | Planarity | PnP RMSE | Positive / all | Quality | Calibration |
|---|---|---|---|---|---:|---:|---|---|
| cam2 (gripper) | 0, 1, 3, 4 | +Y, +Z, -X | 3 faces / 3 non-coplanar | non-planar | 0.874 px | 1 / 1 | nonplanar multiface | SELECTED |

</details>

<details><summary><strong>Event 38</strong> · set 6 · CORE</summary>

| Camera | Marker IDs | Faces | Face support | Planarity | PnP RMSE | Positive / all | Quality | Calibration |
|---|---|---|---|---|---:|---:|---|---|
| cam2 (gripper) | 0, 1, 4, 5 | +Z, -X, -Y | 3 faces / 3 non-coplanar | non-planar | 2.264 px | 1 / 1 | nonplanar multiface | SELECTED |

</details>

<details><summary><strong>Event 39</strong> · set 6 · CORE</summary>

| Camera | Marker IDs | Faces | Face support | Planarity | PnP RMSE | Positive / all | Quality | Calibration |
|---|---|---|---|---|---:|---:|---|---|
| cam2 (gripper) | 1, 3, 4 | +Y, +Z, -X | 3 faces / 3 non-coplanar | non-planar | 0.823 px | 1 / 1 | nonplanar multiface | SELECTED |

</details>

<details><summary><strong>Event 40</strong> · set 6 · NO OBSERVATION</summary>

| Camera | Marker IDs | Faces | Face support | Planarity | PnP RMSE | Positive / all | Quality | Calibration |
|---|---|---|---|---|---:|---:|---|---|
| cam2 (gripper) | — | — | 0 faces / 0 non-coplanar | — | — | 0 / 0 | none | pnp_not_accepted |

</details>

<details><summary><strong>Event 41</strong> · set 6 · CORE</summary>

| Camera | Marker IDs | Faces | Face support | Planarity | PnP RMSE | Positive / all | Quality | Calibration |
|---|---|---|---|---|---:|---:|---|---|
| cam2 (gripper) | 0, 1, 4, 5 | +Z, -X, -Y | 3 faces / 3 non-coplanar | non-planar | 1.167 px | 1 / 1 | nonplanar multiface | SELECTED |

</details>

<details><summary><strong>Event 42</strong> · set 7 · CORE</summary>

| Camera | Marker IDs | Faces | Face support | Planarity | PnP RMSE | Positive / all | Quality | Calibration |
|---|---|---|---|---|---:|---:|---|---|
| cam0 | 3, 4 | +Y, -X | 2 faces / 2 non-coplanar | non-planar | 0.678 px | 1 / 1 | nonplanar multiface | SELECTED |
| cam1 | 2, 3 | +X, +Y | 2 faces / 2 non-coplanar | non-planar | 0.632 px | 1 / 1 | nonplanar multiface | SELECTED |
| cam2 (gripper) | 1, 3 | +Y, +Z | 2 faces / 2 non-coplanar | non-planar | 0.654 px | 1 / 1 | nonplanar multiface | SELECTED |
| cam3 | 0, 4, 5 | +Z, -X, -Y | 3 faces / 3 non-coplanar | non-planar | 1.332 px | 1 / 1 | nonplanar multiface | SELECTED |

</details>

<details><summary><strong>Event 43</strong> · set 7 · CORE</summary>

| Camera | Marker IDs | Faces | Face support | Planarity | PnP RMSE | Positive / all | Quality | Calibration |
|---|---|---|---|---|---:|---:|---|---|
| cam2 (gripper) | 0, 1, 2, 3 | +X, +Y, +Z | 3 faces / 3 non-coplanar | non-planar | 1.486 px | 1 / 1 | nonplanar multiface | SELECTED |

</details>

<details><summary><strong>Event 44</strong> · set 7 · CORE</summary>

| Camera | Marker IDs | Faces | Face support | Planarity | PnP RMSE | Positive / all | Quality | Calibration |
|---|---|---|---|---|---:|---:|---|---|
| cam2 (gripper) | 0, 1, 3, 4 | +Y, +Z, -X | 3 faces / 3 non-coplanar | non-planar | 1.537 px | 1 / 1 | nonplanar multiface | SELECTED |

</details>

<details><summary><strong>Event 45</strong> · set 7 · CORE</summary>

| Camera | Marker IDs | Faces | Face support | Planarity | PnP RMSE | Positive / all | Quality | Calibration |
|---|---|---|---|---|---:|---:|---|---|
| cam2 (gripper) | 0, 1, 2 | +X, +Z | 2 faces / 2 non-coplanar | non-planar | 1.264 px | 1 / 1 | nonplanar multiface | SELECTED |

</details>

<details><summary><strong>Event 46</strong> · set 7 · CORE</summary>

| Camera | Marker IDs | Faces | Face support | Planarity | PnP RMSE | Positive / all | Quality | Calibration |
|---|---|---|---|---|---:|---:|---|---|
| cam2 (gripper) | 0, 1, 3, 4 | +Y, +Z, -X | 3 faces / 3 non-coplanar | non-planar | 0.989 px | 1 / 1 | nonplanar multiface | SELECTED |

</details>

<details><summary><strong>Event 47</strong> · set 7 · SINGLE MARKER</summary>

| Camera | Marker IDs | Faces | Face support | Planarity | PnP RMSE | Positive / all | Quality | Calibration |
|---|---|---|---|---|---:|---:|---|---|
| cam2 (gripper) | 3 | +Y | 1 faces / 0 non-coplanar | planar | 0.134 px | 2 / 2 | single marker | noncore_single_marker |

</details>

<details><summary><strong>Event 48</strong> · set 8 · CORE</summary>

| Camera | Marker IDs | Faces | Face support | Planarity | PnP RMSE | Positive / all | Quality | Calibration |
|---|---|---|---|---|---:|---:|---|---|
| cam0 | 1, 4, 5 | +Z, -X, -Y | 3 faces / 3 non-coplanar | non-planar | 0.903 px | 1 / 1 | nonplanar multiface | SELECTED |
| cam1 | 0, 1, 3, 4 | +Y, +Z, -X | 3 faces / 3 non-coplanar | non-planar | 1.014 px | 1 / 1 | nonplanar multiface | SELECTED |
| cam2 (gripper) | 0, 1, 4, 5 | +Z, -X, -Y | 3 faces / 3 non-coplanar | non-planar | 0.935 px | 1 / 1 | nonplanar multiface | SELECTED |
| cam3 | 0, 5 | +Z, -Y | 2 faces / 2 non-coplanar | non-planar | 0.906 px | 1 / 1 | nonplanar multiface | SELECTED |

</details>

<details><summary><strong>Event 49</strong> · set 8 · CORE</summary>

| Camera | Marker IDs | Faces | Face support | Planarity | PnP RMSE | Positive / all | Quality | Calibration |
|---|---|---|---|---|---:|---:|---|---|
| cam2 (gripper) | 0, 1, 5 | +Z, -Y | 2 faces / 2 non-coplanar | non-planar | 1.216 px | 1 / 1 | nonplanar multiface | SELECTED |

</details>

<details><summary><strong>Event 50</strong> · set 8 · CORE</summary>

| Camera | Marker IDs | Faces | Face support | Planarity | PnP RMSE | Positive / all | Quality | Calibration |
|---|---|---|---|---|---:|---:|---|---|
| cam2 (gripper) | 0, 1, 4 | +Z, -X | 2 faces / 2 non-coplanar | non-planar | 1.055 px | 1 / 1 | nonplanar multiface | SELECTED |

</details>

<details><summary><strong>Event 51</strong> · set 8 · CORE</summary>

| Camera | Marker IDs | Faces | Face support | Planarity | PnP RMSE | Positive / all | Quality | Calibration |
|---|---|---|---|---|---:|---:|---|---|
| cam2 (gripper) | 0, 1, 4, 5 | +Z, -X, -Y | 3 faces / 3 non-coplanar | non-planar | 1.096 px | 1 / 1 | nonplanar multiface | SELECTED |

</details>

<details><summary><strong>Event 52</strong> · set 8 · CORE</summary>

| Camera | Marker IDs | Faces | Face support | Planarity | PnP RMSE | Positive / all | Quality | Calibration |
|---|---|---|---|---|---:|---:|---|---|
| cam2 (gripper) | 0, 1, 4, 5 | +Z, -X, -Y | 3 faces / 3 non-coplanar | non-planar | 1.054 px | 1 / 1 | nonplanar multiface | SELECTED |

</details>

<details><summary><strong>Event 53</strong> · set 8 · CORE</summary>

| Camera | Marker IDs | Faces | Face support | Planarity | PnP RMSE | Positive / all | Quality | Calibration |
|---|---|---|---|---|---:|---:|---|---|
| cam2 (gripper) | 0, 1, 4, 5 | +Z, -X, -Y | 3 faces / 3 non-coplanar | non-planar | 1.095 px | 1 / 1 | nonplanar multiface | SELECTED |

</details>

<details><summary><strong>Event 54</strong> · set 9 · CORE</summary>

| Camera | Marker IDs | Faces | Face support | Planarity | PnP RMSE | Positive / all | Quality | Calibration |
|---|---|---|---|---|---:|---:|---|---|
| cam0 | 1, 3, 4 | +Y, +Z, -X | 3 faces / 3 non-coplanar | non-planar | 0.790 px | 1 / 1 | nonplanar multiface | SELECTED |
| cam1 | 0, 1, 2, 3 | +X, +Y, +Z | 3 faces / 3 non-coplanar | non-planar | 1.158 px | 1 / 1 | nonplanar multiface | SELECTED |
| cam2 (gripper) | 0, 1, 3, 4 | +Y, +Z, -X | 3 faces / 3 non-coplanar | non-planar | 0.891 px | 1 / 1 | nonplanar multiface | SELECTED |
| cam3 | 4, 5 | -X, -Y | 2 faces / 2 non-coplanar | non-planar | 1.279 px | 1 / 1 | nonplanar multiface | SELECTED |

</details>

<details><summary><strong>Event 55</strong> · set 9 · CORE</summary>

| Camera | Marker IDs | Faces | Face support | Planarity | PnP RMSE | Positive / all | Quality | Calibration |
|---|---|---|---|---|---:|---:|---|---|
| cam2 (gripper) | 0, 1, 4 | +Z, -X | 2 faces / 2 non-coplanar | non-planar | 1.537 px | 1 / 1 | nonplanar multiface | SELECTED |

</details>

<details><summary><strong>Event 56</strong> · set 9 · CORE</summary>

| Camera | Marker IDs | Faces | Face support | Planarity | PnP RMSE | Positive / all | Quality | Calibration |
|---|---|---|---|---|---:|---:|---|---|
| cam2 (gripper) | 0, 3 | +Y, +Z | 2 faces / 2 non-coplanar | non-planar | 1.094 px | 1 / 1 | nonplanar multiface | SELECTED |

</details>

<details><summary><strong>Event 57</strong> · set 9 · CORE</summary>

| Camera | Marker IDs | Faces | Face support | Planarity | PnP RMSE | Positive / all | Quality | Calibration |
|---|---|---|---|---|---:|---:|---|---|
| cam2 (gripper) | 0, 1, 3, 4 | +Y, +Z, -X | 3 faces / 3 non-coplanar | non-planar | 1.428 px | 1 / 1 | nonplanar multiface | SELECTED |

</details>

<details><summary><strong>Event 58</strong> · set 9 · CORE</summary>

| Camera | Marker IDs | Faces | Face support | Planarity | PnP RMSE | Positive / all | Quality | Calibration |
|---|---|---|---|---|---:|---:|---|---|
| cam2 (gripper) | 0, 1, 3, 4 | +Y, +Z, -X | 3 faces / 3 non-coplanar | non-planar | 1.303 px | 1 / 1 | nonplanar multiface | SELECTED |

</details>

<details><summary><strong>Event 59</strong> · set 9 · CORE</summary>

| Camera | Marker IDs | Faces | Face support | Planarity | PnP RMSE | Positive / all | Quality | Calibration |
|---|---|---|---|---|---:|---:|---|---|
| cam2 (gripper) | 0, 1, 3 | +Y, +Z | 2 faces / 2 non-coplanar | non-planar | 0.673 px | 1 / 1 | nonplanar multiface | SELECTED |

</details>

<details><summary><strong>Event 60</strong> · set 10 · CORE</summary>

| Camera | Marker IDs | Faces | Face support | Planarity | PnP RMSE | Positive / all | Quality | Calibration |
|---|---|---|---|---|---:|---:|---|---|
| cam0 | 2, 3 | +X, +Y | 2 faces / 2 non-coplanar | non-planar | 0.746 px | 1 / 1 | nonplanar multiface | SELECTED |
| cam1 | 0, 1, 2, 5 | +X, +Z, -Y | 3 faces / 3 non-coplanar | non-planar | 1.241 px | 1 / 1 | nonplanar multiface | SELECTED |
| cam2 (gripper) | 0, 1, 2, 3 | +X, +Y, +Z | 3 faces / 3 non-coplanar | non-planar | 1.026 px | 1 / 1 | nonplanar multiface | SELECTED |
| cam3 | 0, 3, 4 | +Y, +Z, -X | 3 faces / 3 non-coplanar | non-planar | 0.814 px | 1 / 1 | nonplanar multiface | SELECTED |

</details>

<details><summary><strong>Event 61</strong> · set 10 · CORE</summary>

| Camera | Marker IDs | Faces | Face support | Planarity | PnP RMSE | Positive / all | Quality | Calibration |
|---|---|---|---|---|---:|---:|---|---|
| cam2 (gripper) | 0, 1, 3, 4 | +Y, +Z, -X | 3 faces / 3 non-coplanar | non-planar | 1.246 px | 1 / 1 | nonplanar multiface | SELECTED |

</details>

<details><summary><strong>Event 62</strong> · set 10 · CORE</summary>

| Camera | Marker IDs | Faces | Face support | Planarity | PnP RMSE | Positive / all | Quality | Calibration |
|---|---|---|---|---|---:|---:|---|---|
| cam2 (gripper) | 0, 1, 2, 3 | +X, +Y, +Z | 3 faces / 3 non-coplanar | non-planar | 1.647 px | 1 / 1 | nonplanar multiface | SELECTED |

</details>

<details><summary><strong>Event 63</strong> · set 10 · CORE</summary>

| Camera | Marker IDs | Faces | Face support | Planarity | PnP RMSE | Positive / all | Quality | Calibration |
|---|---|---|---|---|---:|---:|---|---|
| cam2 (gripper) | 0, 1, 4 | +Z, -X | 2 faces / 2 non-coplanar | non-planar | 1.956 px | 1 / 1 | nonplanar multiface | SELECTED |

</details>

<details><summary><strong>Event 64</strong> · set 10 · CORE</summary>

| Camera | Marker IDs | Faces | Face support | Planarity | PnP RMSE | Positive / all | Quality | Calibration |
|---|---|---|---|---|---:|---:|---|---|
| cam2 (gripper) | 1, 3 | +Y, +Z | 2 faces / 2 non-coplanar | non-planar | 0.721 px | 1 / 1 | nonplanar multiface | SELECTED |

</details>

<details><summary><strong>Event 65</strong> · set 10 · CORE</summary>

| Camera | Marker IDs | Faces | Face support | Planarity | PnP RMSE | Positive / all | Quality | Calibration |
|---|---|---|---|---|---:|---:|---|---|
| cam2 (gripper) | 0, 1, 3 | +Y, +Z | 2 faces / 2 non-coplanar | non-planar | 0.744 px | 1 / 1 | nonplanar multiface | SELECTED |

</details>

<details><summary><strong>Event 66</strong> · set 11 · CORE</summary>

| Camera | Marker IDs | Faces | Face support | Planarity | PnP RMSE | Positive / all | Quality | Calibration |
|---|---|---|---|---|---:|---:|---|---|
| cam0 | 1, 3, 4 | +Y, +Z, -X | 3 faces / 3 non-coplanar | non-planar | 1.058 px | 1 / 1 | nonplanar multiface | SELECTED |
| cam1 | 0, 1, 2, 5 | +X, +Z, -Y | 3 faces / 3 non-coplanar | non-planar | 1.269 px | 1 / 1 | nonplanar multiface | SELECTED |
| cam2 (gripper) | 0, 1, 3 | +Y, +Z | 2 faces / 2 non-coplanar | non-planar | 0.615 px | 1 / 1 | nonplanar multiface | SELECTED |
| cam3 | 4, 5 | -X, -Y | 2 faces / 2 non-coplanar | non-planar | 2.128 px | 1 / 1 | nonplanar multiface | SELECTED |

</details>

<details><summary><strong>Event 67</strong> · set 11 · CORE</summary>

| Camera | Marker IDs | Faces | Face support | Planarity | PnP RMSE | Positive / all | Quality | Calibration |
|---|---|---|---|---|---:|---:|---|---|
| cam2 (gripper) | 0, 1, 4 | +Z, -X | 2 faces / 2 non-coplanar | non-planar | 0.890 px | 1 / 1 | nonplanar multiface | SELECTED |

</details>

<details><summary><strong>Event 68</strong> · set 11 · CORE</summary>

| Camera | Marker IDs | Faces | Face support | Planarity | PnP RMSE | Positive / all | Quality | Calibration |
|---|---|---|---|---|---:|---:|---|---|
| cam2 (gripper) | 0, 1, 4, 5 | +Z, -X, -Y | 3 faces / 3 non-coplanar | non-planar | 1.639 px | 1 / 1 | nonplanar multiface | SELECTED |

</details>

<details><summary><strong>Event 69</strong> · set 11 · SINGLE MARKER</summary>

| Camera | Marker IDs | Faces | Face support | Planarity | PnP RMSE | Positive / all | Quality | Calibration |
|---|---|---|---|---|---:|---:|---|---|
| cam2 (gripper) | 4 | -X | 1 faces / 0 non-coplanar | planar | 1.040 px | 2 / 2 | single marker | noncore_single_marker |

</details>

<details><summary><strong>Event 70</strong> · set 11 · CORE</summary>

| Camera | Marker IDs | Faces | Face support | Planarity | PnP RMSE | Positive / all | Quality | Calibration |
|---|---|---|---|---|---:|---:|---|---|
| cam2 (gripper) | 0, 1, 3, 4 | +Y, +Z, -X | 3 faces / 3 non-coplanar | non-planar | 1.412 px | 1 / 1 | nonplanar multiface | SELECTED |

</details>

<details><summary><strong>Event 71</strong> · set 11 · CORE</summary>

| Camera | Marker IDs | Faces | Face support | Planarity | PnP RMSE | Positive / all | Quality | Calibration |
|---|---|---|---|---|---:|---:|---|---|
| cam2 (gripper) | 1, 3 | +Y, +Z | 2 faces / 2 non-coplanar | non-planar | 1.102 px | 1 / 1 | nonplanar multiface | SELECTED |

</details>

<details><summary><strong>Event 72</strong> · set 12 · CORE</summary>

| Camera | Marker IDs | Faces | Face support | Planarity | PnP RMSE | Positive / all | Quality | Calibration |
|---|---|---|---|---|---:|---:|---|---|
| cam0 | 0, 1, 4, 5 | +Z, -X, -Y | 3 faces / 3 non-coplanar | non-planar | 1.053 px | 1 / 1 | nonplanar multiface | SELECTED |
| cam1 | 0, 1, 2, 3 | +X, +Y, +Z | 3 faces / 3 non-coplanar | non-planar | 0.986 px | 1 / 1 | nonplanar multiface | SELECTED |
| cam2 (gripper) | 0, 1, 4 | +Z, -X | 2 faces / 2 non-coplanar | non-planar | 1.493 px | 1 / 1 | nonplanar multiface | SELECTED |
| cam3 | 2, 5 | +X, -Y | 2 faces / 2 non-coplanar | non-planar | 0.943 px | 1 / 1 | nonplanar multiface | SELECTED |

</details>

<details><summary><strong>Event 73</strong> · set 12 · CORE</summary>

| Camera | Marker IDs | Faces | Face support | Planarity | PnP RMSE | Positive / all | Quality | Calibration |
|---|---|---|---|---|---:|---:|---|---|
| cam2 (gripper) | 0, 1, 5 | +Z, -Y | 2 faces / 2 non-coplanar | non-planar | 1.253 px | 1 / 1 | nonplanar multiface | SELECTED |

</details>

<details><summary><strong>Event 74</strong> · set 12 · CORE</summary>

| Camera | Marker IDs | Faces | Face support | Planarity | PnP RMSE | Positive / all | Quality | Calibration |
|---|---|---|---|---|---:|---:|---|---|
| cam2 (gripper) | 0, 1, 2, 3 | +X, +Y, +Z | 3 faces / 3 non-coplanar | non-planar | 1.525 px | 1 / 1 | nonplanar multiface | SELECTED |

</details>

<details><summary><strong>Event 75</strong> · set 12 · CORE</summary>

| Camera | Marker IDs | Faces | Face support | Planarity | PnP RMSE | Positive / all | Quality | Calibration |
|---|---|---|---|---|---:|---:|---|---|
| cam2 (gripper) | 0, 5 | +Z, -Y | 2 faces / 2 non-coplanar | non-planar | 1.201 px | 1 / 1 | nonplanar multiface | SELECTED |

</details>

<details><summary><strong>Event 76</strong> · set 12 · CORE</summary>

| Camera | Marker IDs | Faces | Face support | Planarity | PnP RMSE | Positive / all | Quality | Calibration |
|---|---|---|---|---|---:|---:|---|---|
| cam2 (gripper) | 0, 1, 2, 5 | +X, +Z, -Y | 3 faces / 3 non-coplanar | non-planar | 1.214 px | 1 / 1 | nonplanar multiface | SELECTED |

</details>

<details><summary><strong>Event 77</strong> · set 12 · CORE</summary>

| Camera | Marker IDs | Faces | Face support | Planarity | PnP RMSE | Positive / all | Quality | Calibration |
|---|---|---|---|---|---:|---:|---|---|
| cam2 (gripper) | 0, 1, 4, 5 | +Z, -X, -Y | 3 faces / 3 non-coplanar | non-planar | 2.847 px | 1 / 1 | nonplanar multiface | SELECTED |

</details>
