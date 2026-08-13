# Table 1 — Main Ablation: session02 late held-out 결과

생성일: 2026-08-13

이 보고서는 실제 셋업의 안정 구간에서 모든 실행 가능한 Table 1 방법을 같은 source events,
train/test split, intrinsics, raw-corner frontend와 평가 mask로 비교한 **software held-out 진단**이다.
독립 외부 GT가 없으므로 `TRE_t`, `e_R`, ADD/ADD-S와 절대 3D 정확도 순위는 제공하지 않는다.

## 사용 데이터

| 항목 | 사용값 |
| --- | --- |
| Session | `data/session02/calib_train` |
| Placement sets | 5–12 (8 sets) |
| Source events | 48 events |
| 저장 RGB | 72장: cam0=8, cam1=8, cam2=48, cam3=8 |
| Camera role | cam0/1/3=fixed eye-to-hand, cam2=gripper eye-in-hand |
| Intrinsics | `intrinsics/cam0.npz`–`cam3.npz` |
| Robot/capture metadata | `data/session02/calib_train/meta.json` |
| Cube geometry | `config.py:CubeConfig` |
| Excluded | sets 1–4: cam0 이동 전; set13: 공통 event-stratified 관측 구조 불일치 |

입력 파일 SHA-256은 다음과 같다.

| 파일 | SHA-256 |
| --- | --- |
| `meta.json` | `e88c6dcf27d12320fee436b23f765a98564dbd38724d418f3949a692228f7940` |
| `cam0.npz` | `e34d7176f271509e70566c02a3dd51b755588c88e172f75d6b0b3d9c464907d6` |
| `cam1.npz` | `8c1fbe08f3d4a37780d0bc7d085c153bfb4f04cc6751aedcc9d880714e60ca3` |
| `cam2.npz` | `3fbeeba8f3d4a37780d0bc7d085749015b6c7f595fb2bfda8bb76cc42b777138` |
| `cam3.npz` | `dc2cca9f67a992e5334711de0aea83e874d134cdc26a5842d7e9f6c88b96457e` |
| `config.py` | `e727f257973bc10cf59dd4354eb43016bfbba357d63d881f935dc5300e910ea5` |

## Held-out 계약

- Event-grouped, set-stratified 80/20 split, seed `20260731`.
- Train events: 40개. Held-out events: `29, 38, 46, 50, 53, 61, 65, 75`.
- Train 관측: board 54 observations / 2,724 corners, cube 55 / 768.
- Held-out 관측: board 17 / 706, cube 17 / 228.
- Path mask SHA-256:
  `4339c33d283052a80ba07f715f1091e17b344179dbc9018c6f929a9948f9fc80`.
- Path 평가량: 9 fixed-camera pairs, 3 fixed↔gripper units. Model-output-dependent gate 없음.
- 공통 solver: raw-corner pixel residual, `soft_l1`, `f_scale=2 px`, `max_nfev=300`,
  tolerance `1e-8`, 3 multi-start, test-time refit 없음.
- A4/B1/B2 preflight: 실측 covariance 대신 `sigma_t=3 mm`, `sigma_R=0.3 deg` 등방성
  placeholder, FK Huber scale `2.5`. 따라서 confirmatory-ready는 `false`다.

기본 seed는 held-out에 fixed-camera frame이 없어 path metric을 만들 수 없었다. 모델 결과를 보지 않고
fixed-camera held-out placement가 최소 3개가 되는 첫 seed를 순서대로 선택해 `20260731`로 고정했다.
이 seed에서 sets 5, 9, 11이 path metric을 지원한다.

## Main Table 전체 결과

숫자는 3개 초기화 run의 평균이다. 실행된 모든 행은 3/3 수렴했고 fixed camera 3대를 등록했다.
`e_cross`와 `e_e2e`는 내부 일관성 지표이며 외부 정확도가 아니다.

| ID | 수렴 | `N_reg` | Held-out reproj overall / board / cube (px) | `e_cross` (mm/deg) | `e_e2e` (mm/deg) | 상태 |
| --- | ---: | ---: | ---: | ---: | ---: | --- |
| A0 | 3/3 | 3 | 2.4829 / 2.4829 / — | — | — | 완료 |
| A1 | 3/3 | 3 | 3.1267 / 2.5103 / 4.5317 | 13.7546 / 1.2193 | 4.2459 / 1.0239 | 완료 |
| A2 | 3/3 | 3 | 3.1154 / 2.5038 / 4.5109 | 13.1276 / 1.1910 | 4.0693 / 1.0385 | 완료 |
| A3 | 3/3 | 3 | **2.4997** / 2.4952 / **2.5136** | **10.8320** / **1.0602** | 4.7904 / 1.1893 | 완료 |
| A4 | 3/3 | 3 | 3.1030 / 2.5038 / 4.4758 | 13.0590 / 1.1901 | 4.0396 / 1.0316 | preflight |
| A5 | — | — | — | — | — | 미구현; 6-DoF correction label·외부 GT 없음 |
| B1 | 3/3 | 3 | 3.1128 / 2.5101 / 4.4927 | 13.6410 / 1.2158 | 4.2097 / 1.0186 | preflight |
| B2 | 3/3 | 3 | 4.4938 / — / 4.4938 | 11.9911 / 1.2833 | **3.8443** / **0.9315** | preflight |
| B3 | 3/3 | 3 | 2.4829 / 2.4829 / — | — | — | 완료 |

A0/B3의 cube/path 칸은 실패가 아니라 해당 target을 목적함수에서 제외해 정의되지 않는 값이다.
A5는 correction이 calibration transform을 바꾸지 않는다는 명세상 calibration 진단은 A4와 같아야 하지만,
correction 구현과 독립 held-out label이 없어 별도 실행 결과로 간주하지 않았다.

## 카메라별 held-out reprojection

각 셀은 해당 방법이 사용하는 전체 target corner의 RMSE(px)다.

| ID | cam0 | cam1 | cam2 (gripper) | cam3 |
| --- | ---: | ---: | ---: | ---: |
| A0 | 0.2799 | 1.2804 | 2.9157 | 1.9078 |
| A1 | 1.6270 | 3.0082 | 3.4071 | 3.2672 |
| A2 | 1.6194 | 2.9404 | 3.4038 | 3.2457 |
| A3 | 0.6008 | 1.4071 | 2.9816 | 1.8076 |
| A4 | 1.6119 | 2.8801 | 3.4006 | 3.2057 |
| B1 | 1.6184 | 2.9379 | 3.4036 | 3.2251 |
| B2 | 3.2864 | 4.5068 | 4.5902 | 5.1526 |
| B3 | 0.2799 | 1.2804 | 2.9157 | 1.9078 |

## Paired contrast

Delta는 `후자 - 전자`다. 음수는 해당 오차 감소를 뜻한다. Target subset이 다른 비교는 공통 target
component만 사용했다.

| Contrast | 공통 비교 지표 | Delta |
| --- | --- | ---: |
| A0→A1 | board reproj | +0.0274 px |
| A1→A2 | overall reproj | -0.0113 px |
| A2→A3 | overall reproj / `e_cross_t` / `e_e2e_t` | -0.6157 px / -2.2955 mm / +0.7211 mm |
| A2→A4 | overall reproj / `e_cross_t` / `e_e2e_t` | -0.0124 px / -0.0685 mm / -0.0297 mm |
| B1→A4 | overall reproj / `e_cross_t` / `e_e2e_t` | -0.0098 px / -0.5819 mm / -0.1701 mm |
| B2→A4 | cube reproj / `e_cross_t` / `e_e2e_t` | -0.0180 px / +1.0680 mm / +0.1953 mm |
| B3→A2 | board reproj | +0.0209 px |

## Supplementary A4 decomposition

| ID | Held-out overall / board / cube (px) | `e_cross` (mm/deg) | `e_e2e` (mm/deg) |
| --- | ---: | ---: | ---: |
| A4a fixed-weight soft | 2.6052 / 2.4872 / 2.9406 | 11.9469 / 1.1231 | 4.2264 / 1.1012 |
| A4b covariance linear | 3.1023 / 2.5036 / 4.4741 | 13.0423 / 1.1904 | 4.0291 / 1.0308 |
| A4c=A4 covariance robust | 3.1030 / 2.5038 / 4.4758 | 13.0590 / 1.1901 | 4.0396 / 1.0316 |

Placeholder covariance에서 A4b→A4 robustification의 변화는 reprojection `+0.0007 px`,
`e_cross_t +0.0168 mm`, `e_e2e_t +0.0105 mm`로 사실상 없다. 이 결과로 covariance weighting이나
robust loss의 최종 기여를 확정할 수 없다.

## 결론

- 현재 내부 지표에서 A3 hard-FK가 held-out overall/cube reprojection과 cross-camera translation에서
  가장 낮다. B2는 e2e translation/rotation이 가장 낮지만 reprojection은 가장 나쁘다.
- A4-preflight는 A2 대비 overall reprojection `0.0124 px`, `e_cross_t 0.0685 mm`, `e_e2e_t 0.0297 mm`
  개선에 그쳤다. B1 대비 joint 이득도 작다.
- 지표별 순위가 일치하지 않으므로 내부 consistency만으로 최종 방법을 고르면 안 된다.
- 최종 A4/A5 판정에는 반복측정으로 만든 preregistered `Sigma_FK`, 독립 external-GT pose,
  A5 train-only 6-DoF correction 구현이 필요하다.

## 원시 산출물

- [Main Table 통합 CSV](table1_results.csv)
- [A0/A1/A2/A3/B3 JSON](ablation_core/seven_row_ablation.json)
- [A0/A1/A2/A3/B3 CSV](ablation_core/seven_row_ablation.csv)
- [A4 계열/B1/B2 JSON](final_methods_preflight/final_methods.json)
- [A4 계열/B1/B2 요약](final_methods_preflight/final_methods.md)

A2/A3는 두 실행기에 모두 존재한다. 위 Main Table에는 A4와 동일 runner에서 재실행한
`final_methods.json` 값을 사용했고, 두 산출물의 split과 path-mask hash가 동일한 것을 확인했다.
