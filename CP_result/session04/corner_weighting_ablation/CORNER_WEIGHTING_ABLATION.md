# Session04 Corner / Weighting Ablation

## 결론

두 후보 모두 canonical로 채택하지 않는다.

- `line_intersection`: Board–Cube camera transform 충돌은 줄였지만 Cube held-out transfer와 A2 overall/e2e가 악화했다.
- `equal_observation_total`: Board의 많은 corner가 목적함수를 독점하는 현상은 줄였지만 Cube와 전체 held-out가 크게 악화했다.
- canonical 입력은 `CORNER_REFINE_APRILTAG + per_corner`로 유지한다.

## 공정한 paired 비교

두 corner 방법을 현재 코드에서 각각 새 manifest로 생성했다. 두 manifest의 Board 관측은 동일하고 Cube corner mode만 다르다. split은 `seed=20260731`, eligible set은 `4–12`이다.

| 직접 fixed-camera 진단 | AprilTag | Line intersection | 판정 |
|---|---:|---:|---|
| Board–Cube translation conflict RMSE | 10.8077 mm | 8.5679 mm | 개선 |
| Board held-out transfer RMSE | 1.4141 px | 1.4141 px | 동일 |
| Cube held-out transfer RMSE | 2.8820 px | 2.9411 px | 악화 |
| Cube held-out pose translation RMSE | 3.5148 mm | 3.5566 mm | 악화 |
| Strict Cube observations | 97 | 98 | +1 |

Line refit은 비교 가능한 1,196개 Cube corner 중 1,128개를 변경했다. 평균 이동은 `0.652 px`, quad 중심 바깥 방향 성분은 `+0.217 px`, 평균 면적비는 `1.01627`이었다. 이 변화는 모든 camera와 marker ID에서 나타났으므로 한 camera, 한 marker, corner ordering 문제가 아니다.

## A2 3-seed 결과

모든 값은 paired fresh manifest에서 세 seed의 평균이다.

| A2 조건 | Overall px | Board px | Cube px | E_cross mm | Cross-view px | E_e2e mm | 수렴 |
|---|---:|---:|---:|---:|---:|---:|---:|
| AprilTag + per-corner | 3.8901 | 3.9840 | 3.5958 | 5.9594 | 3.9991 | 6.4469 | 3/3 |
| Line intersection + per-corner | 3.8953 | 4.0213 | 3.4930 | 4.9570 | 3.4621 | 6.6889 | 3/3 |
| AprilTag + equal-observation | 3.9631 | 3.9866 | 3.8922 | 6.2148 | 4.0143 | 6.7578 | 3/3 |

Line intersection은 Cube와 fixed-camera consistency를 개선하는 대신 공유 camera pose를 이동시켜 Board와 end-to-end를 악화했다. Equal-observation은 train의 Board/Cube observation 수가 `60/59`로 비슷해도 corner 수가 `2,959/724`이므로 Board 블록을 지나치게 약화시키는 결과가 되었다.

## 원인 판정

남은 오차는 intrinsic, 치수, corner ordering, 단일 PnP solver 문제가 아니다. 현재 데이터가 보여 주는 원인은 **영상 조건에 따라 달라지는 target-dependent corner localization bias**다. Black-border line intersection이 평균적으로 quad를 키워 Board–Cube scale/translation 충돌을 줄이지만, 그 이동량이 view마다 일정하지 않아 Cube 자체의 held-out multiview consistency를 악화한다.

따라서 한 개의 pixel scale이나 observation-count weight로 보정하면 안 된다. 다음 유효한 방법은 반복 정지 촬영으로 camera/marker/입사각별 corner covariance를 측정한 뒤, 그 사전 등록 covariance로 whitened reprojection을 수행하는 것이다. Session04만으로 그 covariance를 held-out 누수 없이 추정할 반복 관측이 없으므로 이번 결과에서는 canonical을 변경하지 않는다.

## 재현 명령

```bash
python3 04_filter_observations.py \
  --session-root data/session04/calib_train \
  --intrinsics-dir intrinsics \
  --output-dir /tmp/session04_capture_filter_line \
  --cube-corner-refinement-mode line_intersection

python3 05_calibrate.py \
  --root_folder data/session04/calib_train \
  --intrinsics_dir intrinsics \
  --calib_dir data/session04/calib_out \
  --include_sets 0-12 --split_seed 20260731 \
  --min_train_eih_cube_events 3 --num_inits 3 --rows A2 \
  --residual-weighting equal_observation_total \
  --observation-manifest data/session04/calib_out/capture_filter/Step2b_observation_manifest.json \
  --observation-filter-policy standard \
  --out_dir /tmp/table1_obsnorm
```
