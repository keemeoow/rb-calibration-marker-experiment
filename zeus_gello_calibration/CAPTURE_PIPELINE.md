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

### 3.1. 먼저 원본 RGB 촬영

현재는 **원본 촬영과 intrinsic 보정을 분리**한다. `--capture_only`에서는 보드
검출기를 실행하지 않고 `SPACE`를 누를 때마다 PNG를 저장한다. 코너 수, 선명도,
최소 장수로 촬영을 막지 않으며, `cam*.npz`의 intrinsic은 갱신하지 않는다.

Intrinsic 촬영에는 아래 명령처럼 `--capture_only`를 반드시 사용한다. 마커 ID,
코너 수, 보드 위치/커버리지, 선명도, 촬영 장수에 따른 성공/실패 판정을 하지 않는다.
카메라 프레임이 들어오면 `SPACE`로 저장하고, 장수와 관계없이 `c`/Enter로 넘긴다.
카메라 시리얼/해상도 일치와 파일 저장 오류만 확인한다. 이 옵션을 생략하면 기존
실시간 검출/보정 모드가 실행되므로 원본 촬영 명령에서 빼지 않는다.

1280용 `01` 결과가 준비되어 있으므로, 열려 있는 이전 `02` 창은 `q`로 종료하고
다음 명령으로 촬영한다. `realsense-viewer`도 닫는다.

```bash
cd '/home/jysim/*jiwoo/rb-calibration-marker-experiment'

python3 02_calibrate_intrinsics.py \
  --intr_dir intrinsics_1280x720 \
  --capture_only --no-reset_devices
```

카메라 4대를 순서대로 촬영한다. 한 대당 20장 정도를 화면 중앙/가장자리,
서로 다른 거리와 기울기에서 모은 뒤 `c` 또는 Enter로 다음 카메라로 넘어간다.
20장에 자동으로 종료되지는 않는다. `q`는 종료, `u`는 마지막 샘플을 후처리
대상에서 제외하는 키다. 이미 저장한 원본 파일은 두 경우 모두 유지한다.

원본과 카메라 정보는 다음에 저장한다.

```text
<intr_dir>/raw_capture/
  capture_manifest.json        # 시리얼, 해상도, 이미지 목록, 후처리 포함 여부
  capture_report.json          # 카메라별 마지막 촬영 실행 요약
  cam0/view_<timestamp>.png
  cam1/view_<timestamp>.png
  cam2/view_<timestamp>.png
  cam3/view_<timestamp>.png
```

같은 명령을 다시 실행하면 기존 장수를 표시하고 추가 저장한다. 새 촬영 묶음이나
다른 실물 보드는 `--images_dir 새_폴더`로 별도 수집하고, 후처리에도 같은 옵션을
사용한다. 보드가 흐리거나 가려진 사진도 저장되므로 촬영할 때 직접 확인한다.

1920은 별도 폴더에 `01`을 한 번 실행한 다음 같은 원본 촬영 모드를 쓴다.
이미 해당 해상도의 `01` 결과가 준비되어 있으면 `02`만 실행한다.

```bash
rs-enumerate-devices -s
python3 01_export_intrinsics.py \
  --out_dir intrinsics_1920x1080_rgbd720 \
  --color_w 1920 --color_h 1080 \
  --depth_w 1280 --depth_h 720 \
  --fps 15 \
  --gripper_serial 136622073980

python3 02_calibrate_intrinsics.py \
  --intr_dir intrinsics_1920x1080_rgbd720 \
  --capture_only --no-reset_devices
```

### 3.2. 보드 정의 확인 후 저장 이미지로 보정

`--from_images`는 RealSense 연결이나 미리보기 창 없이 저장한 이미지로 보정한다.
보드 정의를 바꿔 재실행할 수 있다. 아래는 현재 보드 후보의 예시이므로 실제
원본 패턴의 칸 수, dictionary, 시작 ID를 확인한 값으로 지정한다.

보드의 공통 기본 정의는 `calibration_pipeline/config.py`의
`CharucoBoardConfig`에 있다 (11x7, 25/18 mm, `DICT_4X4_250`, 시작 ID 5).
아래 6x9 / 시작 ID 90 정의는 공통 기본값을 바꾸지 않고, 이 intrinsic 후처리
명령의 옵션으로만 덮어쓴다. `--capture_only`에서는 어느 정의도 검출에 사용하지
않으며, 촬영 전에 보드 정의를 확정하거나 수정할 필요가 없다.

| 항목 | 후처리 보드 옵션 |
| --- | --- |
| 체커 칸 수 | 가로 6 x 세로 9 (내부 코너 수가 아닌 칸 수) |
| 체커 한 칸 / 마커 한 변 | 25 mm / 18 mm |
| Dictionary | `DICT_4X4_250` |
| 시작 마커 ID | 90 (27개 마커: ID 90~116) |
| 패턴 크기 / 내부 코너 수 | 여백 제외 150 x 225 mm / 40개 |

```bash
python3 02_calibrate_intrinsics.py \
  --intr_dir intrinsics_1280x720 \
  --from_images --min_views 12 --use_factory_guess \
  --squares_x 6 --squares_y 9 \
  --square_len_m 0.025 --marker_len_m 0.018 \
  --dictionary DICT_4X4_250 --marker_id_start 90
```

1920도 `--intr_dir`를 `intrinsics_1920x1080_rgbd720`으로 바꿔 같은 보드 옵션으로
보정한다. `--min_views 12`는 후처리에서 보드 검출에 성공한 이미지의 최소 장수다.
촬영 장수 제한은 아니며, 부족하면 해당 카메라의 기존 intrinsic을 유지한다.

보정 결과는 기존처럼 `cam*.npz`의 `color_K/color_D`에 저장하며 최초 값은
`factory_backup/`에 보관한다. `charuco_intrinsics_report.json`에는 지정한 보드,
이미지별 검출 결과/제외 이유, 사용 장수, RMS를 기록한다. 후처리에서 시리얼과
해상도가 다른 이미지를 혼합하거나 이미지 크기를 자동 변환하지 않는다.

구형 짝수 행 보드는 `--legacy_pattern`을 추가한다. 실시간 검출/보정이 필요한
경우 두 모드 옵션을 모두 생략하면 기존 방식으로 실행되며, 그 모드에서 거부된
프레임은 `charuco_capture/camN/diagnostics/`에 원본과 `[DIAG]` 정보를 저장한다.

### 3.3. 카메라 확인

`02` 시작 로그의 `Intrinsics directory`가 `01`의 출력 폴더와 같고 대상이 4대인지
확인한다. 연결 시리얼이 맵에 빠져 있거나 `cam*.npz`의 시리얼과 맵이 서로 다르면
촬영 시작 전에 오류로 중단한다. `gripper_cam_idx`가 비어 있으면 모두 FIXED로
표시되며 원본 촬영/intrinsic 보정은 가능하지만 `03` 전에 역할을 지정해야 한다.

`rs-enumerate-devices -s`에서 실제 카메라 4대와 그리퍼 시리얼을 먼저 확인한다.
예시의 그리퍼 시리얼이 현재 장치와 다르면 `--gripper_serial`을 실제 시리얼로
바꾼다. 장치 시리얼이 바뀌었으면 빈 새 폴더에 `01`을 실행해 새 맵을 만들고,
실물 카메라와 `cam_idx`의 관계를 확인한다. 이전 `device_map.json`을 복사하면
`01`이 새 시리얼을 자동 추가해 `cam4` 이상이 생성될 수 있다.

스크립트 실행 뒤 viewer에서도 카메라가 전부 사라지고 `lsusb`에 Intel `8086` 장치가
없다면 USB 호스트가 장치를 잃은 상태다. 이때는 PC/USB 허브를 재시작한 뒤
`rs-enumerate-devices -s`로 4대를 다시 확인하고 촬영을 재개한다.

고해상도 `02` 촬영은 `1920x1080` RGB 원본을 저장한다. 이후 `03` RGB-D 촬영에서는
`1280x720` raw depth stream을 color frame에 align해 저장한다. 4대 동시 USB
bandwidth가 부족하면 `1280x720@15` 기본 경로로 되돌린다.

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
