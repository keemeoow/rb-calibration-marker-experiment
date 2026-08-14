# Board-only SOTA simulation scaffold

## 목적

- 실제 로봇 데이터 적용 전 calibration 구현 검증
- 공통 board geometry, camera model, robot trajectory 및 평가 방식 제공
- Single-camera method의 반복 적용과 joint multi-camera method를 같은 입력으로 비교
- 좌표계 방향, 단위, PnP/data adapter 및 결과 변환 오류 확인

공통 scaffold의 기본 target은 end-effector에 고정된 ChArUco board이다. 별도 OpenCV 예제에는
wrist camera도 포함하지만 cube와 soft-FK factor는 포함하지 않는다. 따라서 좌표계와 고전
baseline 구현의 첫 번째 검증 단계로 사용하며, `CP_synthetic_7row.py`의 cube/FK ablation과
목적을 섞지 않는다.

## 빠른 실행

### 기본 비교 실험: 인지 노이즈 sweep

실제 비교 실험의 기본 조건은 다음과 같이 고정한다.

- Wrist camera와 각 fixed camera를 각각 독립적으로 calibration
- Eye-in-hand와 eye-to-hand는 서로 다른 robot trajectory 사용 가능
- 각 실험군의 trajectory는 모든 방법과 noise level에서 동일하게 유지
- Robot pose, camera GT, board geometry는 변경하지 않음
- Camera frame에서 관측된 각 3D board corner에 `0, 1, 3, 5 mm` noise 적용
- 표기된 noise는 corner x/y/z 각 좌표에 더하는 Gaussian noise의 표준편차
- Noisy corner에 rigid board pose를 fitting하여 `T_camera_board`를 다시 계산
- 따라서 translation과 rotation calibration error가 함께 발생
- OpenCV의 Tsai, Park, Horaud, Andreff, Daniilidis를 동일 입력으로 모두 실행
- 각 조건을 기본 30회 반복하고 mean ± standard deviation 및 분포 보고

`presentation/` 폴더에서 실행:

```bash
bash ../SOTA_Simulation/launch_noise_sweep.sh
```

저장소 루트에서 실행:

```bash
bash SOTA_Simulation/launch_noise_sweep.sh
```

결과는 `SOTA_Simulation/outputs/opencv_noise_sweep/`에 저장된다.

- `figure1_fixed_setup.png`: wrist 및 fixed-camera 실험의 고정된 두 궤적
- `figure2_noise_results.png`: 방법·camera별 translation/rotation error의 mean ± std
- `figure3_trial_distributions.png`: 첫 번째 선택 방법의 noise별 translation error 분포
- `figure4_integrated_results.png`: 네 카메라를 동일 가중 평균한 system-level 결과
- `report.json`: 조건, trajectory hash, 모든 trial의 원시 결과
- `records.csv`: camera, noise level, trial별 원시 오차
- `integrated_records.csv`: trial별 4-camera 평균 및 worst-camera 오차

통합 결과는 각 trial에서 `wrist, cam0, cam1, cam3`의 extrinsic error를 동일 가중 평균한
macro-average이다. Translation과 rotation은 단위가 다르므로 서로 합쳐 하나의 score로 만들지
않는다. `worst_*` 열에는 각 trial에서 가장 큰 camera error도 함께 저장한다. 이 값은 독립적으로
calibration한 네 결과의 system-level 요약이며 joint multi-camera calibration 결과는 아니다.

일부 방법만 실행할 수도 있다.

```bash
python SOTA_Simulation/tsai_noise_sweep.py \
  --methods tsai park horaud andreff daniilidis \
  --noise-mm 0 1 3 5 --trials 30
```

방법별 파일을 복사할 필요는 없다. `--methods all`은 동일한 robot trajectory와 동일한 noisy
observation을 다섯 OpenCV solver에 전달한다.

### Multi-camera 관점의 held-out 평가

논문 결과용 지표는 전체 pose를 calibration에 쓰는 noise sweep과 분리한다. 기본 14개 pose 중
10개만 calibration에 사용하고 event `2, 5, 9, 12`는 solver에 전달하지 않은 채 평가에만 쓴다.

```bash
python SOTA_Simulation/opencv_multicam_evaluation.py \
  --methods all \
  --noise-mm 0 1 3 5 \
  --trials 30
```

출력 지표:

- Held-out chain error: held-out board 관측을 추정 extrinsic으로 base frame에 옮긴 pose의 GT error
- Camera pose accuracy: `wrist, cam0, cam1, cam3`의 extrinsic GT error를 동일 가중 평균
- Registration consistency: 네 카메라의 여섯 relative transform에 대한 GT error 평균
- Held-out reprojection RMSE: held-out pose의 모든 camera/board corner에 대한 pixel RMSE

Wrist reprojection에는 실제 `intrinsics/cam2.npz`를 사용한다. 결과는
`SOTA_Simulation/outputs/opencv_multicam_metrics/`의 `report.json`, `records.csv`, 두 figure에
저장한다. Translation, rotation, pixel error는 서로 다른 물리량이므로 하나의 임의 score로
합치지 않는다.

### 현실성 검토

- 현실과 같은 부분
  - 실제 board 크기와 실측 camera intrinsic/distortion 사용
  - Eye-in-hand와 eye-to-hand의 실제 transform chain 사용
  - 모든 알고리즘에 동일한 trajectory, observation, random seed 적용
  - Board pose 오차가 calibration의 translation과 rotation에 함께 전달됨
  - Camera별 독립 calibration과 base-frame 조립을 실제 순서대로 수행
- 현재 예제가 단순화한 부분
  - `0/1/3/5 mm` 실험은 camera frame의 **3D corner detector**를 가정한 stress test
  - RGB ChArUco 영상의 실제 경로인 `2D corner detection → PnP → board pose`는 아직 포함하지 않음
  - Intrinsic 오차, board 휨/인쇄 오차, motion blur, occlusion, outlier, timestamp 오차 없음
  - Robot FK의 absolute error, backlash, thermal drift 및 camera mounting drift 없음
  - Corner noise를 독립·등방 Gaussian으로 가정하며 depth와 view angle 의존성이 없음
- 결론
  - 좌표계, solver adapter 및 상대적인 noise sensitivity를 확인하는 reference example로는 타당
  - 실제 RGB 실험 성능을 예측하거나 논문 최종 수치를 생성하는 simulator로는 아직 불충분
  - 논문용 다음 단계는 noiseless pixel projection을 검증한 뒤 pixel noise, detection failure,
    PnP, FK 및 synchronization noise를 실제 측정 분포로 추가하는 것

### 먼저 볼 예제: Tsai–Lenz 전체 흐름

처음에는 아래 교육용 예제를 실행한다. 노이즈가 없는 상태에서 Tsai–Lenz를 이용해
eye-in-hand와 eye-to-hand를 각각 계산하고, 결과를 robot base 좌표계에 모으는 과정을
네 단계의 그림으로 보여준다.

`presentation/` 폴더에서:

```bash
bash ../SOTA_Simulation/launch_tsai_demo.sh
```

저장소 루트에서:

```bash
bash SOTA_Simulation/launch_tsai_demo.sh
```

- Step 1 — eye-in-hand: 정지 보드 + 움직이는 wrist camera로 `T_gripper_wrist` 계산
- Step 2 — eye-to-hand: gripper에 부착된 보드 + fixed camera로 `T_base_fixed_i` 계산
- Step 3 — base-frame view: 추정된 fixed camera와 pose 0의 wrist camera만 base frame에 표시
- Step 4 — GT error: 무잡음 결과가 수치 정밀도 수준의 0인지 확인

이 예제에서는 변환식 자체를 먼저 검증하기 위해 image corner에 PnP 오차를 넣지 않고
정확한 `T_camera_board`를 Tsai–Lenz에 전달한다. 다음 단계에서 pixel 관측과 PnP를 연결한다.

중요: Tsai–Lenz를 fixed camera마다 한 번씩 독립 실행한 것이므로, 이것만으로 joint
multi-camera calibration이 되는 것은 아니다. 학생들은 이 결과를 좌표계와 입출력 변환의
정답 예제로 사용한 뒤, SOTA 방법의 shared variable 또는 joint objective를 연결한다.

생성 결과:

```text
SOTA_Simulation/outputs/tsai_combined_noiseless/
├── report.json
├── step1_eye_in_hand.png
├── step2_eye_to_hand.png
├── step3_merged.png
└── step4_errors.png
```

### 전용 Conda 환경 생성

최초 한 번만 실행한다.

```bash
conda env create -f SOTA_Simulation/environment.yml
```

환경이 이미 있고 패키지 구성을 갱신할 때:

```bash
conda env update -n sota-calibration-sim \
  -f SOTA_Simulation/environment.yml --prune
```

### GUI로 실행

저장소 안의 어느 폴더에서든 launcher의 경로만 맞추면 실행할 수 있다. 예를 들어
`presentation/`에서는 다음과 같이 실행한다.

```bash
bash ../SOTA_Simulation/launch_gui.sh
```

저장소 루트에서는 다음과 같다.

```bash
bash SOTA_Simulation/launch_gui.sh
```

- 3D scene 창: mouse drag로 회전, scroll로 확대/축소
- Reprojection 창: camera별 observed/estimated corner 확인
- Error 창: camera별 translation/rotation GT error 확인
- 모든 창을 닫으면 program 종료
- PNG와 JSON 결과도 `SOTA_Simulation/outputs/joint_noiseless/`에 저장

### 저장소 루트에서 실행

```bash
python -m SOTA_Simulation.run \
  --method joint_reference \
  --output SOTA_Simulation/outputs/joint_noiseless \
  --show
```

현재 위치가 `presentation/`처럼 저장소의 하위 폴더라면 module 방식 대신 script 경로를 사용한다.

```bash
python ../SOTA_Simulation/run.py \
  --method joint_reference \
  --output ../SOTA_Simulation/outputs/joint_noiseless \
  --show
```

현재 위치와 관계없이 실행하려면 저장소의 절대 경로를 사용할 수 있다.

```bash
python /home/sstone/rb-calibration-marker-experiment/SOTA_Simulation/run.py \
  --method joint_reference \
  --output /home/sstone/rb-calibration-marker-experiment/SOTA_Simulation/outputs/joint_noiseless \
  --show
```

카메라별 독립 calibration 예제:

```bash
python SOTA_Simulation/run.py \
  --method independent_reference \
  --output SOTA_Simulation/outputs/independent_noiseless
```

`joint_reference`와 `independent_reference`는 scaffold 검증용 예제이며 논문 baseline이 아니다.

## 생성되는 파일

```text
outputs/<run>/
├── calibration_input_no_gt.npz  # 외부 method에 전달할 공통 입력
├── report.json                  # GT pose error, reprojection RMSE, pass/fail
├── scene_3d.png                 # 카메라, robot trajectory, board pose
├── reprojection.png             # 관측 corner와 추정 결과 overlay
└── calibration_errors.png       # 카메라별 translation/rotation error
```

## 실제 board 및 camera 값

[`config.example.json`](config.example.json)의 기본값:

- ChArUco: 11×7 squares
- Square length: 25 mm
- Marker length: 18 mm
- Fixed cameras: cam0, cam1, cam3
- Camera intrinsics: `intrinsics/cam0.npz`, `cam1.npz`, `cam3.npz`
- Image size와 distortion: 각 NPZ의 실측값 사용

Board를 교체하면 다음 값을 함께 수정한다.

- `squares_x`, `squares_y`
- `square_length_m`, `marker_length_m`
- Dictionary
- Board frame 원점 및 corner ordering을 사용하는 외부 adapter

현재 simulator의 board point는 물리 board 중심을 원점으로 하는 내부 chessboard corner이다.
외부 저장소가 왼쪽 위 corner를 원점으로 사용하면 adapter에서 변환을 명시해야 한다.

## 공통 method interface

학생이 작성할 부분은 [`adapter_template.py`](adapter_template.py)의 `calibrate()`이다.

입력 `CalibrationInput`:

- `board_points`: board frame의 3D corner [m]
- `cameras`: camera별 `K`, distortion, image size
- `T_base_gripper[event]`: exact 또는 noise가 추가된 robot pose
- `observations`: camera/event별 corner ID와 pixel 좌표
  - `T_camera_board_exact`: pose-level solver의 noiseless convention test용 관측 pose
- `initial_T_base_camera`: 공통 초기값
- `initial_T_gripper_board`: 공통 초기값
- `metadata`: 좌표계와 단위 설명

반환 `CalibrationResult`:

- `T_base_camera[name]`: camera frame에서 robot base frame으로 가는 변환
- `T_gripper_board`: board frame에서 gripper frame으로 가는 변환
- Translation 단위: metre
- `success` 및 method-specific diagnostics

외부 adapter 실행 예:

```bash
python -m SOTA_Simulation.run \
  --method SOTA_Simulation.my_tabb_adapter:create_method \
  --output SOTA_Simulation/outputs/tabb_noiseless
```

## NPZ 입력 형식

외부 저장소를 별도 process로 실행할 때 `calibration_input_no_gt.npz`를 사용한다.

- `board_points`: `(P, 3)`
- `camera_names`: `(C,)`
- `camera_K`: `(C, 3, 3)`
- `camera_distortion`: `(C, 5)`
- `image_size`: `(C, 2)`, width/height
- `event_ids`: `(E,)`
- `T_base_gripper`: `(E, 4, 4)`
- `observation_camera_index`: `(O,)`
- `observation_event`: `(O,)`
- `observation_T_camera_board_exact`: `(O, 4, 4)`, pose-level noiseless 입력
- `observation_offsets`: `(O+1,)`
- `observation_point_indices`: flattened corner indices
- `observation_image_points`: flattened `(N, 2)` pixels

Observation `j`의 corner는
`observation_offsets[j]:observation_offsets[j+1]` 범위에서 읽는다.

## 권장 검증 순서

1. 기본 config의 noiseless recovery 통과
2. Seed, camera pose와 initialization을 바꾼 반복 recovery
3. `pixel_noise_sigma`를 0.1, 0.3, 0.5 px로 증가
4. `corner_dropout_probability` 및 `camera_event_dropout_probability` 증가
5. 필요 시 outlier와 FK noise 모델 추가

무잡음 합격은 literal `0.0`이 아니라 config의 numerical tolerance로 판단한다. 이 tolerance는
method 결과를 보기 전에 확정한다.

## 변환식

모든 변환은 `T_destination_source` 표기법을 사용한다.

```text
T_base_board(k)   = T_base_gripper(k) @ T_gripper_board
T_camera_board(k) = inverse(T_base_camera) @ T_base_board(k)
p_camera          = T_camera_board(k) @ p_board
```

외부 논문 또는 코드의 표기 방향이 다르면 adapter에서만 변환한다. 공통 simulator 내부 convention은
변경하지 않는다.
