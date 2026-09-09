# Final Calibration Capture Protocol

상태: **프로토콜 및 capture 통신 구현 완료, 실제 pose 티칭·dry run 전**  
적용 대상: 새 composite-target calibration session  
비적용 대상: 기존 `data/session04` legacy 촬영

이 문서는 최종 비교실험 `A0~A5`, `B1~B3`에 공통으로 사용할 촬영 계약이다.
연구 주장과 최종 판정 원칙은 [RESEARCH_STORYLINE.md](RESEARCH_STORYLINE.md)를 따른다.

## 1. 핵심 계약

모든 비교 행은 **동일한 45개 planned synchronized capture event와 동일한 pose ID**를
사용한다. 촬영 시간이나 검출된 marker 수를 예산으로 맞추지 않는다. 비교 행에 따라
다른 영상을 추가하거나, marker 검출 결과가 나쁜 event를 새 event로 교체하지 않는다.

| 항목 | 고정 규칙 |
| --- | --- |
| 물리 target | planar board와 multi-face cube를 하나의 강체 composite rig로 결합 |
| 촬영 예산 | P1 15 + P2 20 + P3 10 = 총 45 planned event |
| event 정의 | 동일 시점의 모든 연결 카메라 RGB-D와 robot state를 한 묶음으로 저장 |
| 비교 공정성 | 모든 row가 같은 raw event를 사용하고 marker observation만 사전 정의대로 masking |
| 최종 평가 target | heldout, cross-view, External GT 모두 cube-only |
| External GT | 45개 calibration-train event와 분리된 별도 촬영 |

`45 events`는 검출 성공 event 수가 아니라 **사전에 정한 촬영 시도 위치 수**다.

## 2. 장치와 좌표계

### 2.1 Composite target

- Board와 cube는 촬영 중 서로 움직이지 않는 하나의 rig여야 한다.
- `T_rig_board`, `T_rig_cube`를 촬영 전에 측정하고 geometry file과 hash를 기록한다.
- 두 target이 동시에 보이는 시야를 확보하되, board가 cube를 가리거나 gripper와
  충돌하지 않는지 전 카메라에서 확인한다.
- A0/B3도 별도 board 촬영을 사용하지 않는다. 같은 이미지에서 cube observation만
  calibration 전체에서 가린다.

행렬 표기 `T_A_B`는 B 좌표의 점을 A 좌표로 변환한다.

| 기호 | 의미 |
| --- | --- |
| `T_base_flange(e)` | event `e`에서 robot controller가 기록한 flange FK |
| `T_flange_rig` | P1의 still-gripped 관측으로 추정한 flange-to-rig |
| `T_rig_board`, `T_rig_cube` | rig에 고정된 board/cube 기하 |
| `T_base_rig(e)` | event 또는 placement의 base 좌표계 target pose |
| `T_gripper_cam` | eye-in-hand camera extrinsic |
| `T_base_Ci` | fixed camera `i`의 base 좌표계 extrinsic |

rig를 잡고 있을 때 target pose는 다음 체인을 따른다.

```text
T_base_rig(e)   = T_base_flange(e) * T_flange_rig
T_base_board(e) = T_base_rig(e) * T_rig_board
T_base_cube(e)  = T_base_rig(e) * T_rig_cube
```

P2에서 release한 뒤에는 같은 `placement_id`의 두 event가 하나의 stationary
`T_base_rig(placement_id)`를 공유한다. P3 전체도 하나의 stationary rig pose를 공유한다.

## 3. 45-Event 촬영 순서

| Phase | Planned event ID | 수 | Target 상태 | 필수 관측 | 목적 |
| --- | --- | ---: | --- | --- | --- |
| P1 Moving Rig | `P1_00`~`P1_14` | 15 | gripped | 모든 fixed camera + robot FK | `T_flange_rig`, 공간/회전 다양성 |
| P2 Pick-and-Place | `P2_S00_V0`~`P2_S09_V1` | 20 | released, placement별 2 views | fixed + gripper camera + robot FK | 통합 calibration, placement discrepancy |
| P3 Stationary Rig | `P3_00`~`P3_09` | 10 | stationary | fixed + gripper camera + robot FK | eye-in-hand excitation |
| 합계 |  | **45** |  |  | 공통 calibration-train budget |

### 3.1 P1 Moving Rig: 15 events

1. Robot이 composite rig를 잡은 상태를 15개 pose 내내 유지한다.
2. 작업 공간의 `x/y/z` 위치와 `roll/pitch/yaw`를 모두 변화시킨다. yaw-only 회전은
   허용하지 않는다.
3. 각 pose에서 robot settle 후 모든 fixed camera를 동기 촬영한다.
4. 연결된 gripper camera 영상도 원본으로 저장한다. 다만 camera와 rig가 함께
   움직여 상대 pose가 거의 고정되므로 P1 gripper 영상은 hand-eye excitation으로
   사용하지 않는다.
5. `T_flange_rig` 또는 순수 FK correction은 이 still-gripped 구간만 사용해 추정한다.

### 3.2 P2 Pick-and-Place: 10 placements x 2 views

각 `placement_id = S00~S09`에 대해 아래 순서를 반복한다.

1. Rig를 grasp하고 사전 정의한 placement로 이동한다.
2. Release 직전 `T_base_flange`와 grasp/release metadata를 저장한다. 이 기록은
   capture event 수에 포함하지 않는다.
3. Rig를 release하고 완전히 정지시킨다.
4. Rig를 움직이지 않은 채 gripper camera를 첫 viewpoint로 이동해 `V0`를 동기 촬영한다.
5. Rig를 그대로 둔 채 두 번째 viewpoint로 이동해 `V1`을 동기 촬영한다.

각 viewpoint에서 fixed camera와 gripper camera를 함께 저장한다. 10개 placement 중
2~3개만 높은 top view로 편중하지 말고, 나머지도 거리·방향·고도 차이를 갖게 한다.
두 view 사이에 target이 움직이면 해당 placement를 성공 event로 꾸미지 않고
`target_motion_suspected=true`로 기록한다.

P2의 release 예측은 다음과 같다.

```text
T_base_rig_pred = T_base_flange(release) * T_flange_rig
```

이 예측과 post-release VISION pose의 차이는 robot FK뿐 아니라 grasp 반복성, release
slip, 접촉 및 settling 오차를 함께 포함한다. 따라서 이를 **effective placement
correction**으로 부르며, 순수 FK correction이라고 해석하지 않는다.

### 3.3 P3 Stationary Rig: 10 events

1. Composite rig를 한 위치에 놓고 10개 event 동안 움직이지 않는다.
2. Robot과 gripper camera만 10개 pose로 이동한다.
3. translation baseline과 `roll/pitch/yaw` excitation을 모두 포함한다.
4. 각 pose에서 fixed camera와 gripper camera를 동기 촬영한다.
5. P3 도중 rig가 움직였다고 의심되면 motion flag를 남긴다.

## 4. 저장 및 실패 처리 계약

### 4.1 Event state machine

```text
INIT
  -> P1_GRIPPED (15 planned poses)
  -> P2_PLACED  (10 placements, 2 views each)
  -> P3_STATIONARY (10 robot/camera poses)
  -> LOCKED
```

Phase 전환은 operator confirmation과 상태 검사를 요구한다. `target_state`가 예상과
다르면 저장을 강행하지 않고 해당 attempt를 실패로 기록한다.

### 4.2 최소 metadata

각 attempt는 `meta.json` 또는 별도 manifest에 아래 정보를 보존해야 한다.

| 분류 | 필수 필드 |
| --- | --- |
| 식별 | `protocol_version`, `session_id`, `planned_event_id`, `attempt_id`, `phase` |
| 상태 | `target_rig_id`, `target_state`, `placement_id`, `view_index`, `grasp_id` |
| Robot | capture 시각의 `T_base_flange`, joints, release 시각의 `T_base_flange` |
| 동기화 | camera별 timestamp, 전체 span, sync pass/fail와 실패 이유 |
| 파일 | camera별 RGB/depth path, transport/save success |
| 검출 | board/cube 검출 수와 품질, 단 **저장 채택 조건과 분리** |
| 추적 | `attempt_status`, `retry_of_attempt_id`, motion/safety flag |
| 재현성 | intrinsic hash, rig geometry hash, capture config hash |

#### 로봇 서버에서 PC로 보내 저장할 실제 값

최종 프로토콜에서 `server/c1.py`는 촬영 직전 아래 `robot_state_v2`를 만들어 PC로
보낸다. PC는 이를 해당 attempt의 `meta.json`에 원본 그대로 저장한다.

| JSON 필드 | 단위/기준 | 사용 목적 |
| --- | --- | --- |
| `flange_pose_6dof_mm_deg` | `[x,y,z,rz,ry,rx]`, mm/deg, `RzRyRx` | **FK와 hand-eye 계산의 기준 pose** |
| `joints_deg` | 6축 degree | 실제 도달 자세, 반복 실행 및 motion 검증 |
| `motion_tcp_pose_6dof_mm_deg` | tool3 TCP, mm/deg | 이동 재현과 안전 진단 전용 |
| `gripper_io` | controller digital input 4개 | grasp/release 상태 증거 |
| `server_epoch_s` | robot-server wall clock | 명령 시각 추적용, camera hardware sync 기준은 아님 |
| `motion_tool_id`, `motion_tool_offset_6dof_mm_deg` | tool3 정의 | TCP pose 해석과 tool 설정 변경 검출 |

각 event에는 이 실제 상태와 함께 다음 계획/실행 provenance도 저장한다.

| 구분 | 저장 필드 |
| --- | --- |
| 계획 식별 | `protocol_version`, `planned_event_id`, `capture_index`, `phase` |
| target 상태 | `target_state`, `placement_id`, `view_index`, `grasp_id` |
| 명령값 | `planned_waypoint.capture_joints`, `motion_safety.target_joints_actual` |
| 시도 결과 | `attempt_index`, `attempt_status`, `selected_for_analysis`, `retry_allowed` |
| P2 release | `release_state.pre_release`, `post_release`, commanded place/approach pose |

P2의 `pre_release`와 `post_release`도 각각 완전한 `robot_state_v2`다. Release 직전
flange FK가 effective placement prediction의 입력이고, release 이후 상태와 gripper I/O는
실제 release가 수행되었는지 확인하는 증거다.

최종 composite rig에서는 기존 `capture_cube_center_6dof`를 저장하거나 FK 정답으로
사용하지 않는다. 현재 robot tool4는 과거 cube-only offset이므로 새 rig의 중심을
나타내지 않는다. `T_flange_rig`는 P1의 실제 flange FK와 VISION observation으로 추정한다.

#### PC에서 함께 저장되는 값

| 분류 | 저장 내용 |
| --- | --- |
| 원본 frame | 모든 연결 camera의 RGB, aligned depth, write 성공 여부 |
| 시간 | host monotonic receipt, device timestamp/domain, camera 간 span |
| transport | 누락 camera, 누락 timestamp, span 초과, 파일 저장 실패 |
| marker 진단 | board/cube corner, PnP, reprojection, depth/ROI 품질 |
| 고정 artifact | pose plan 원본·SHA-256, rig geometry 원본·SHA-256, intrinsic/config |

최종 세션의 핵심 JSON은 다음 네 개다.

| 파일 | 역할 |
| --- | --- |
| `sessionNN/capture_waypoints.json` | 로봇에 전달한 frozen 45-event pose plan |
| `sessionNN/calib_train/meta.json` | 모든 camera attempt, robot/release state, marker 진단 |
| `sessionNN/calib_train/capture_waypoints_recorded.json` | 서버가 실제 보고한 pose와 status의 실행 기록 |
| `sessionNN/calib_train/capture_protocol_manifest.json` | 45 planned ID의 완료·누락·attempt 수 잠금 결과 |

### 4.3 공정한 실패 처리

1. Camera transport, 파일 저장 또는 timestamp sync 실패만 같은 `planned_event_id`로
   재시도할 수 있다. 모든 attempt는 삭제하지 않는다.
2. 분석 입력은 marker 검출 여부를 보지 않고 **첫 transport/sync-valid attempt**로
   결정한다.
3. Board/cube 검출 실패, corner 수 부족, PnP 실패를 이유로 새 pose를 추가하거나
   해당 event를 교체하지 않는다.
4. valid attempt가 없으면 그 event는 실패로 남기고 모든 비교 row에 동일하게 적용한다.
5. 물리적으로 rig가 움직였거나 잘못 grasp된 event도 숨기지 않고 상태 flag와 함께
   보존한다. 전체 campaign 재촬영 여부는 row 결과를 보기 전에 결정한다.

최종 protocol mode에서는 quality gate가 진단값으로만 남는다. Marker 검출 실패는
저장되고, transport/sync 실패 attempt도 별도 `event_id`로 보존된 뒤 같은
`planned_event_id`만 재시도한다. Legacy mode의 기존 gate 동작은 유지된다.

## 5. 비교 행별 관측 사용 규칙

촬영 후 split, marker mask, solver 설정은 row 결과를 보기 전에 고정한다.

| Row | Calibration에 사용하는 관측 | FK 사용 |
| --- | --- | --- |
| A0 | board-only | sequential baseline |
| A1 | board + cube | sequential, VISION |
| A2 | board + cube | unified, VISION |
| A3 | board + cube | unified, FK hard fixed (보정 전 controller pose) |
| A4 | board + cube | unified, train-only effective placement correction soft factor |
| A5 | board + cube | unified, corrected-FK hard fixed (train-only VISION alignment) |
| B1 | board + cube | sequential corrected-FK soft factor |
| B2 | cube-only | unified corrected-FK soft factor |
| B3 | board-only | unified, VISION |

- A0/B3는 cube corner를 초기화, filtering, objective, prune, hyperparameter 선택에서 모두
  가린다.
- B2는 같은 단계에서 board corner를 모두 가린다.
- A1~A5/B1만 두 target residual을 함께 사용한다.
- 모든 row는 같은 event split을 사용하며 heldout event의 cube corner로 calibration이나
  nuisance pose를 다시 맞추지 않는다.
- cube-only heldout과 External GT 평가는 calibration과 설정을 frozen한 뒤 한 번 계산한다.

A4의 soft covariance는 10개 placement만으로 자유로운 full `6x6` covariance를 추정하지
않는다. External GT 공개 전에 block-diagonal 또는 shrinkage 규칙을 사전 등록하거나,
추가 반복 데이터로 covariance를 추정한다. A5의 alignment 절차와 모든 hyperparameter도
External GT 확인 전에 hash와 함께 고정한다.

## 6. 촬영 전 통과 조건

- [ ] Composite rig 고정성과 `T_rig_board`, `T_rig_cube` geometry 검증
- [ ] P1/P2/P3 pose list와 45개 `planned_event_id` 사전 생성
- [ ] 전 카메라 intrinsic, serial-to-camera ID, exposure 설정 고정
- [ ] capture state machine, attempt manifest, sync-only retry 정책 구현 및 dry run
- [ ] analysis에서 A0/B3/B2 marker mask와 공통 event split 자동 검증
- [ ] P2 release metadata와 P3 stationary-target 검사 저장
- [ ] calibration train과 External GT session/path 완전 분리

External GT의 세부 촬영과 blind evaluation은
[protocol_templates/CAPTURE_CAMPAIGN_PROTOCOL.md](protocol_templates/CAPTURE_CAMPAIGN_PROTOCOL.md)에
별도로 기록한다. External GT cube를 사용하기 전에는 geometry validator, face orientation,
marker ID 독립성을 반드시 통과시킨다.

## 7. 구현 대상

| 상태 | 대상 | 내용 |
| --- | --- | --- |
| 완료 | `server/c1.py` | P1/P2/P3 자동 실행, release 전후 state, sync-only retry |
| 완료 | `capture_pipeline/capture.py` | plan 검증, planned event/attempt 분리, all-camera 저장, 완료 manifest |
| 완료 | pose-plan 도구 | 45-event template 생성 및 robot 없이 offline 검증 |
| 다음 | `04_filter_observations.py` | 실패 event를 숨기지 않는 manifest, row-independent detection 기록 |
| 다음 | calibration runtime/schema | phase·placement·marker mask 검증과 A3/A4/A5 factor 연결 |

## 8. 자동 pose JSON 준비와 실행

### 8.1 Template 생성

```bash
python3 tools/create_capture_pose_plan.py \
  --out capture_plans/composite_rig_45.json
```

생성된 JSON은 의도적으로 실행 불가능하다. 다음 값을 실제 robot teaching 결과로
채운 뒤 `template_only=false`로 바꾼다.

1. `safe_joints_empty`, `safe_joints_gripped`
2. P1/P2/P3의 45개 `capture_joints`
3. S00~S09의 `place_approach_joints`, `place_approach_tcp`, `place_tcp`
4. `target_rig_id`, `rig_geometry_file`, 실제 geometry file의 SHA-256

`place_approach_tcp`는 `place_tcp`와 x/y·회전이 각각 2mm/2deg 이내로 같고 z가 최소
10mm 높아야 한다. 실행 중에는 approach joint 도달 후 실제 TCP가 저장값과 2mm/2deg
이내인지 다시 검사한 뒤에만 contact pose로 하강한다.

Rig geometry는
`protocol_templates/composite_rig_geometry_TEMPLATE.json`을 기준으로 작성한다.
`T_rig_board`와 `T_rig_cube`는 meter 단위의 유효한 SE(3)여야 한다.

### 8.2 Offline 검증

```bash
python3 tools/validate_capture_pose_plan.py \
  capture_plans/composite_rig_45.json
```

45개 ID 순서, `15/20/10` 수, target 상태, 10개 placement 접근 pose, safe pose,
rig ID·geometry·SHA-256 중 하나라도 맞지 않으면 robot 연결 전에 중단한다.

### 8.3 한 번의 자동 촬영

Robot server에서 먼저 실행한다.

```bash
# 첫 dry run은 --noconfirm을 빼고 event마다 확인한다.
python c1.py --auto pc --speed 30

# 물리 검증이 끝난 본 촬영만 완전 자동 실행한다.
python c1.py --auto pc --speed 30 --noconfirm
```

PC에서는 frozen plan을 넘겨 새 session을 자동 생성한다.

```bash
python3 03_capture.py \
  --data_root data \
  --intrinsics_dir intrinsics \
  --waypoints_file capture_plans/composite_rig_45.json \
  --use_robot --manual_robot \
  --robot_ip 192.168.0.23 --robot_port 12348 \
  --max_capture_span_ms 120 \
  --show
```

Robot server에는 최신 `server/c1.py`와 같은 revision의
`capture_pipeline/waypoint_safety.py`를 `waypoint_safety.py` 이름으로 함께 배포해야 한다.
자동화는 **한 session에서 P1 15 -> P2 20 -> P3 10**을 연속 실행한다. 중간 marker
실패 때문에 pose 수가 늘어나지 않으며, 종료 시 completion manifest가 정확히 45개
selected event인지 확인한다.
