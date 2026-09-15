# Zeus 저장 pose 기반 자동 촬영 파이프라인

상태: **P1/P2/P3 통합 dry-run 통과, 실제 robot 저속 검증 전**

현재 촬영은 `zeus_gello_calibration/data`의 기존 pose를 모두 사용한다. 개수는
하드코딩하지 않고 원본 `capture/*/robot.json` 수에서 결정하며, P1/P2/P3를 한 번에
실행해 하나의 새 session에 저장한다. PC의 유일한 진입점은 root `03_capture.py`다.

## 1. 현재 촬영 계약

| Phase | 원본 | 현재 수 | Robot/target 동작 | 촬영 |
| --- | --- | ---: | --- | --- |
| P1 Moving Rig | `session1.../capture` | 16 | 저장 joint로 이동, cube 계속 파지 | pose마다 1회 |
| P2 Pick-and-Place | `session2.../capture` | 15 | 저장 x/y/yaw 위치에 place/pick | placement마다 고정 CAM pose에서 1회 |
| P3 Stationary Rig | `session3.../capture` | 15 | 마지막 P2 위치를 유지하고 저장 joint로 이동 | pose마다 1회 |
| 합계 | 원본 전체 | **46** | 순서 P1 -> P2 -> P3 | 단일 session |

원본 pose가 추가되거나 제거되면 수와 `expected_event_count`도 자동으로 바뀐다.
현재 실행에서는 `16 + 15 + 15 = 46`이다.

## 2. P2 pose 해석

session2의 15개 `robot.json`은 모두 gripper CLOSED 상태다. 따라서 이 값은 release
이후 카메라 pose가 아니라, cube를 들고 기록한 placement source다.

평면에 cube를 안전하게 놓기 위해 15개 source record를 모두 사용하되 다음처럼
placement pose를 만든다.

```text
x, y, yaw  <- 각 session2 저장 pose
z          <- GRASP_REF_POSE의 178.75 mm
roll/pitch <- GRASP_REF_POSE의 180/0 deg
```

저장 raw 6D pose는 z가 `92.568~155.873 mm`이고 roll/pitch도 달라서 그대로 gripper를
열면 공중 낙하 또는 기울어진 접촉이 될 수 있다. raw 6D 전부를 release pose로 쓰지
않는다. 실제 place 높이가 바뀌었다면 코드의 `P2_GRASP_REF_POSE`를 먼저 재측정한다.

각 placement의 자동 순서는 다음과 같다.

```text
P1 종료 -> 검증된 P2_START_JOINTS 이동
-> 이전 위치 pick -> +Z 50 mm retreat
-> 다음 저장 x/y/yaw의 approach -> place -> gripper open
-> +Z 50 mm retreat -> P2_CAMERA_JOINTS 이동 -> 모든 카메라 촬영
```

마지막 placement는 다시 집지 않고 그대로 유지한 뒤 P3를 시작한다.

## 3. 해상도별 intrinsics 준비

캡처 해상도마다 intrinsics를 따로 만든다. 현재 `intrinsics/`는 `1280x720@15`이고,
자동 재촬영 기본값도 `1280x720@15`다. 더 높은 color 해상도로 촬영하려면
`color 1920x1080 + depth 1280x720 @15` 조합을 먼저 시도한다. 해상도를 바꾸면
반드시 새 intrinsic 폴더를 만들고, 03/04에서 같은 폴더를 사용한다.

```bash
cd '/home/jysim/*jiwoo/rb-calibration-marker-experiment'

rs-enumerate-devices -s

mkdir -p intrinsics_1920x1080_rgbd720
cp intrinsics/device_map.json intrinsics_1920x1080_rgbd720/device_map.json

python3 01_export_intrinsics.py \
  --out_dir intrinsics_1920x1080_rgbd720 \
  --color_w 1920 --color_h 1080 \
  --depth_w 1280 --depth_h 720 \
  --fps 15 \
  --gripper_serial 752112070297

python3 02_calibrate_intrinsics.py \
  --intr_dir intrinsics_1920x1080_rgbd720 \
  --min_views 20 \
  --use_factory_guess
```

다른 해상도는 폴더명과 `--color_w/--color_h/--depth_w/--depth_h/--fps`를 바꿔
같은 절차를 반복한다.
02에서는 카메라별로 ChArUco 보드를 화면 중앙/모서리/가까운 거리/먼 거리/기울어진
각도로 골고루 보여주고, `SPACE`로 20~30장 이상 잡은 뒤 `c` 또는 Enter로 보정한다.
고해상도 모드에서는 02가 `1920x1080` color intrinsic을 보정하고, depth는
`1280x720` raw stream을 color frame에 align해 저장한다. 4대 동시 USB bandwidth가
부족하면 `1280x720@15` 기본 경로로 되돌린다.

## 4. Robot 서버

ZEUS PC에서 GELLO teleoperation과 다른 motion client를 모두 종료한다. 그다음 Python 2
i611 환경에서 다음 서버를 실행한다.

```bash
python ~/zeus_gello.py
```

카메라 PC는 기본 `192.168.0.23:12350`으로 연결한다. 비상정지에 손이 닿아야 하며,
P1 시작 전에 cube가 원본과 같은 `T_flange_cube` 상태로 파지되어 있어야 한다.

## 5. Dry-Run

저장 pose만 읽고 robot과 카메라는 연결하지 않는다.

```bash
cd '/home/jysim/*jiwoo/rb-calibration-marker-experiment'

python3 03_capture.py \
  --saved-pose-replay --all-phases \
  --data-root '/home/jysim/*jiwoo/rb-calibration-marker-experiment/zeus_gello_calibration/data'
```

정상 출력은 P1 16, P2 15, P3 15, 총 46 events다.

## 6. 저속 Robot 검증

카메라와 파일 저장 없이 각 단계에서 Enter를 받아 robot 동작을 검증한다. 이 명령도
P2에서 실제 gripper open/close와 pick/place를 수행하므로 cube를 파지한 상태로 시작한다.

```bash
python3 03_capture.py \
  --saved-pose-replay --all-phases \
  --data-root '/home/jysim/*jiwoo/rb-calibration-marker-experiment/zeus_gello_calibration/data' \
  --execute --motion-only \
  --jnt-speed 3 \
  --p2-move-speed 10 \
  --p2-descend-speed 5
```

P1/P2/P3 모든 pose에서 joint limit, cable, camera, table, target 충돌과 실제 cube
접촉 높이를 확인한다. 하나라도 맞지 않으면 `--no-step`을 실행하지 않는다.

## 7. 스텝별 시험 촬영

저속 robot 검증을 통과한 뒤 카메라 4대와 저장까지 한 단계씩 확인한다.

```bash
python3 03_capture.py \
  --saved-pose-replay --all-phases \
  --data-root '/home/jysim/*jiwoo/rb-calibration-marker-experiment/zeus_gello_calibration/data' \
  --device-map intrinsics_1920x1080_rgbd720/device_map.json \
  --session-label zeus_saved_pose_step_check_1920 \
  --width 1920 --height 1080 \
  --depth-width 1280 --depth-height 720 \
  --fps 15 \
  --execute --no-cam-reset \
  --jnt-speed 3 \
  --p2-move-speed 10 \
  --p2-descend-speed 5
```

중간에 `q`를 입력하면 이미 촬영한 데이터는 `incomplete` session으로 보존된다.

## 8. 연속 자동 촬영

6~7절을 실제 hardware에서 모두 통과한 뒤 실행한다. 시작 직전에 `go`를 한 번
입력하면 event별 입력 없이 P1 -> P2 -> P3를 연속 실행한다.

```bash
python3 03_capture.py \
  --saved-pose-replay --all-phases \
  --data-root '/home/jysim/*jiwoo/rb-calibration-marker-experiment/zeus_gello_calibration/data' \
  --device-map intrinsics_1920x1080_rgbd720/device_map.json \
  --session-label zeus_saved_pose_replay_1920 \
  --width 1920 --height 1080 \
  --depth-width 1280 --depth-height 720 \
  --fps 15 \
  --execute --no-step --no-cam-reset \
  --jnt-speed 3 \
  --p2-move-speed 10 \
  --p2-descend-speed 5
```

속도는 검증된 뒤에만 올린다. `overlap=0`이 기본이므로 각 pose에서 완전히 정지한 뒤
촬영한다. 통합 촬영은 한 USB controller의 여러 카메라를 동시에 reset하지 않도록
hardware reset을 기본 생략한다. 장애 복구가 꼭 필요할 때만 `--cam-reset`으로 한 대씩
순차 reset한다.

## 9. 단일 Session 결과

실제 촬영을 시작할 때 기존 session을 덮어쓰지 않고 다음 번호를 자동 할당한다.

```text
zeus_gello_calibration/data/
└── session<NN>_zeus_saved_pose_replay_1920_<MMDD>/
    ├── session_manifest.json
    ├── calib_train/
    │   ├── events/000 ... 045/
    │   ├── meta.json
    │   └── capture_protocol_manifest.json
    ├── blind_test/
    ├── calib_out/
    └── audit/
```

`meta.json`에는 phase, source pose 경로, 실제 flange pose/joints, gripper state,
placement ID, set index, 카메라별 RGB-D 경로, `capture_config.camera_stream`의
실제 캡처 해상도가 저장된다. 완료 판정은 다음과 같다.

```text
capture_protocol = saved_pose_replay_v1
expected_event_count = 원본 P1 + P2 + P3 개수
selected_event_count = 저장에 성공한 event 개수
missing_planned_event_ids = []
status = complete
```

## 10. 촬영 후 처리

실제 생성된 session의 `calib_train`을 지정한다.

```bash
python3 04_filter_observations.py \
  --session-root zeus_gello_calibration/data/session<NN>_zeus_saved_pose_replay_1920_<MMDD>/calib_train \
  --intrinsics-dir intrinsics_1920x1080_rgbd720
```

04는 가변 event 수를 `capture_config.expected_event_count`에서 읽고 P1의 gripped cube
관측도 포함한다. P2 train/held-out은 event가 아니라 `placement_id` 단위로 분리한다.

현재 05의 phase-aware common-rig target solver는 아직 구현 전이므로 촬영과 04 결과
확인까지만 진행한다. External GT는 calibration session과 분리된 blind session에
추가한다.
