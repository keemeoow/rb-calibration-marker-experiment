# External Baseline Frozen-observation Package

목적: 8/3 피드백 #7, 즉 같은 사진을 공개 구현에도 넣어 custom optimizer만의 문제인지 확인하라는 요구를 재현 가능한 입력 계약으로 분리한다.

이 package는 외부 baseline을 실행한 결과가 아니라 **외부 baseline이 반드시 사용해야 하는 frozen input**이다. Detector를 다시 돌리지 않고, Step 04에서 동결한 2D image points와 3D object points를 그대로 사용한다.

## 포함된 파일

- `external_baseline_observations.csv`: observation 단위 frozen 2D/3D correspondences
- `external_baseline_package.json`: split, source SHA-256, contract, counts
- `external_baseline_intrinsics.json`: fixed camera intrinsics

## 현재 support

- Observations: `154`
- Corners: `4625`
- By split: `{'test': 35, 'train': 119}`
- By target: `{'board': 77, 'cube': 77}`
- By camera role: `{'fixed': 50, 'gripper': 104}`

## 외부 adapter 계약

1. `external_baseline_observations.csv`의 `image_points_json`과 `object_points_json`을 그대로 사용한다.
2. 새 detector, 새 corner refinement, 모델별 outlier 제거를 실행하지 않는다.
3. `split=train`만으로 calibration/fit을 만들고, `split=test`는 평가에만 쓴다.
4. main-method camera poses, joint optimizer 결과, Robot FK, Hand-Eye, fitted shared target pose를 입력으로 쓰지 않는다.
5. 출력은 camera-pose 또는 prediction 파일과 adapter provenance SHA-256을 함께 남긴다.

## 해석 한계

이 package로 COLMAP/MATLAB/OpenCV 계열 baseline을 더 붙일 수 있지만, 결과가 좋거나 나쁘다고 해서 곧바로 external physical GT가 되는 것은 아니다. 역할은 custom optimizer dependency를 줄인 reference/sanity check다.
