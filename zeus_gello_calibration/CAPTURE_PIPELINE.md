# Zeus 최종 캘리브레이션 촬영 파이프라인

상태: **03 촬영 계약 구현 완료, 실제 pose teaching·robot dry run 전, 05 공통-rig 모델 구현 전**

이 문서가 새 캘리브레이션 촬영의 유일한 실행 가이드다. PC 촬영 진입점은
root의 `03_capture.py` 하나이며, 데이터는 명령의 `--data_root` 아래에만 저장한다.
이 문서의 최종 명령은 `zeus_gello_calibration/data`를 명시한다. `capture_pipeline/`은
`03_capture.py`가 호출하는 내부 구현이고 직접 실행하지 않는다.

## 1. 최종 계약

| 항목 | 고정 규칙 |
| --- | --- |
| 물리 target | planar board와 multi-face cube를 하나의 강체 composite rig로 결합 |
| 촬영 예산 | P1 15 + P2 20 + P3 10 = 총 45 planned event |
| event | 같은 시점의 모든 카메라 RGB-D와 robot state를 한 묶음으로 저장 |
| 공정성 | A0~A5/B1~B3 모두 같은 45개 raw event 사용; P2는 placement 단위 split |
| marker ablation | A0/B3는 cube를, B2는 board를 calibration 전 과정에서 masking |
| 평가 target | heldout, cross-view, External GT 모두 cube-only |
| External GT | calibration 45 events와 분리된 별도 blind session |

Marker가 잘 검출된 pose만 추가 촬영하면 표본이 편향된다. 따라서 marker/PnP 실패는
진단으로 저장하되 event를 교체하지 않는다. 카메라 전송, 파일 저장 또는 timestamp
동기 실패만 같은 `planned_event_id`의 새 attempt로 재시도한다.

## 2. 실행 파일

| 위치 | 역할 | 직접 실행 |
| --- | --- | --- |
| `03_capture.py` | 최종 `composite_rig_45_v2` PC 촬영 진입점 | 예 |
| `capture_pipeline/` | 카메라, 저장, 검출 진단, sync, protocol 내부 구현 | 아니오 |
| `server/c1.py` | ZEUS PC에서 P1/P2/P3를 실행하고 상태를 PC로 전송 | 예 |
| `tools/create_capture_pose_plan.py` | 45-event pose-plan 빈 양식 생성 | 준비 시 |
| `tools/validate_capture_pose_plan.py` | robot 연결 전 plan/geometry 검증 | 매 촬영 전 |

`03_capture.py`는 `--waypoints_file`, `--use_robot`, `--manual_robot`이 없거나 plan의
`capture_protocol`이 `composite_rig_45_v2`가 아니면 카메라 연결 전에 종료한다.

## 3. 촬영 구성

| Phase | Planned event | 수 | Target 상태 | 목적 |
| --- | --- | ---: | --- | --- |
| P1 Moving Rig | `P1_00`~`P1_14` | 15 | gripped | `T_flange_rig`, x/y/z와 roll/pitch/yaw 다양성 |
| P2 Pick-and-Place | placement 10개 x view 2개 | 20 | released | 통합 calibration, placement discrepancy |
| P3 Stationary Rig | `P3_00`~`P3_09` | 10 | stationary | eye-in-hand translation/rotation excitation |
| 합계 |  | **45** |  | 공통 calibration-train budget |

P1에서는 한 번 파지한 rig를 놓지 않고 15개 pose로 이동한다. P2에서는 10개 위치마다
release한 뒤 rig를 움직이지 않은 상태에서 gripper camera view 두 개를 촬영한다.
P3에서는 rig를 한 위치에 고정하고 robot과 gripper camera만 10개 pose로 이동한다.
P2 release 직전 FK 기록은 metadata이며 별도 event로 세지 않는다.

분석 그룹은 P2 `PLACEMENT_00~09 -> set_index 0~9`, P3 stationary rig
`-> set_index 10`으로 고정한다. P3가 물리적으로 마지막 P2 위치에 놓여 있어도
`set_index=9`를 재사용하지 않는다.

행렬 `T_A_B`는 B 좌표의 점을 A 좌표로 변환한다.

```text
T_base_rig(e)   = T_base_flange(e) * T_flange_rig
T_base_board(e) = T_base_rig(e) * T_rig_board
T_base_cube(e)  = T_base_rig(e) * T_rig_cube
```

P1 VISION과 robot flange FK로 구한 `T_flange_rig`에는 실제 장착 오차가 포함된다.
P2의 release 예측과 post-release VISION 차이는 robot FK뿐 아니라 재파지, slip, 접촉,
settling 오차도 포함하므로 `effective placement correction`으로 해석한다.

## 4. 촬영 전 필수 점검

1. `server/c1.py`의 새 큐브 관련 `CUBE_GRIP_DEPTH_MM`, `PLACE_TCP_Z_MM`,
   `GRIP_TCP_Z_MM`을 실제 robot에서 재측정해 갱신한다. 현재 파일의 구 큐브 값으로
   새 92 mm 큐브를 자동 이동하면 충돌할 수 있으므로 **그대로 실행하면 안 된다**.
2. Composite rig가 촬영 중 변형되지 않게 고정하고 `T_rig_board`, `T_rig_cube`를
   meter 단위의 유효한 SE(3)로 측정한다.
3. 모든 카메라 intrinsic을 실제 촬영 해상도 `1280x720`에서 준비하고
   `device_map.json`의 serial과 gripper camera를 확인한다.
4. P1/P2/P3 pose가 workspace의 위치, 거리, 고도와 여러 회전축을 포함하는지 확인한다.
5. 저속 confirm dry run에서 접근 방향, singularity, joint limit, cable, table과
   camera 충돌을 확인한다. 비상정지에 손이 닿는 상태에서 진행한다.

## 5. Intrinsic 준비

```bash
python3 01_export_intrinsics.py \
  --out_dir intrinsics \
  --color_w 1280 --color_h 720 --fps 15

python3 02_calibrate_intrinsics.py \
  --intr_dir intrinsics \
  --min_views 12 --save_images
```

촬영 해상도와 `camN.npz`의 `color_w/color_h`가 다르면 `03_capture.py`가 중단한다.

## 6. Pose JSON 준비

### 6.1 Template 생성

```bash
python3 tools/create_capture_pose_plan.py \
  --out capture_plans/composite_rig_45.json
```

생성된 파일은 의도적으로 실행 불가능하다. Robot teaching 결과를 사용해 다음 값을
모두 채우고 `template_only=false`로 바꾼다.

1. `safe_joints_empty`, `safe_joints_gripped`
2. P1/P2/P3의 45개 `capture_joints`
3. 각 capture joint를 teaching할 때 robot server가 읽은 45개
   `capture_flange_pose_6dof_mm_deg`
4. 10개 placement의 `place_approach_joints`, `place_approach_tcp`, `place_tcp`
5. `target_rig_id`, `rig_geometry_file`, geometry 파일의 실제 SHA-256

Geometry는 `protocol_templates/composite_rig_geometry_TEMPLATE.json`을 복제해 실제
측정값을 넣는다. Template이나 `FILL_ME`, `null`이 하나라도 남은 plan은 실행하지 않는다.

### 6.2 Offline 검증

```bash
python3 tools/validate_capture_pose_plan.py \
  capture_plans/composite_rig_45.json
```

검증기는 45개 ID 순서, `15/20/10` 개수, target 상태, placement 접근 pose, safe pose,
rig ID, geometry와 SHA-256을 확인한다. 또한 같은 joint pose 반복과 다음 최소 다양성을
수치로 검사한다.

| 구간 | 촬영 전 통과해야 하는 최소 범위 |
| --- | --- |
| P1 | x/y/z span `100/100/80 mm`, roll/pitch span `15/15 deg` |
| P2 placement | x/y span `100/100 mm`, yaw span `20 deg` |
| P2 각 2-view pair | flange 이동 `30 mm`, roll 또는 pitch 차이 `10 deg` |
| P3 | x/y/z span `80/80/50 mm`, roll/pitch span `15/15 deg` |

`capture_joints`만으로는 Cartesian x/y/z와 roll/pitch 범위를 알 수 없다. 그래서
pose-plan v4는 teaching 순간의 flange pose를 함께 요구한다. 이 검사는 범위 부족과
중복 pose를 잡는 preflight이며, 카메라 시야·마커 검출·Jacobian condition까지 보장하는
검사는 아니므로 dry run과 preview 확인도 그대로 필요하다. `PASS`가 아니면 robot을
연결하지 않는다.

## 7. 한 번의 자동 촬영

ZEUS PC에는 저장소의 최신 `server/c1.py`와
`capture_pipeline/waypoint_safety.py`를 `waypoint_safety.py` 이름으로 함께 배포한다.

```bash
# ZEUS PC: 첫 물리 검증은 event마다 확인
python c1.py --auto pc --speed 30

# confirm dry run을 통과한 뒤에만 완전 자동 실행
python c1.py --auto pc --speed 30 --noconfirm
```

그다음 카메라 PC에서 유일한 촬영 명령을 실행한다.

```bash
python3 03_capture.py \
  --data_root zeus_gello_calibration/data \
  --session_label zeus_composite_rig \
  --intrinsics_dir intrinsics \
  --waypoints_file capture_plans/composite_rig_45.json \
  --use_robot --manual_robot \
  --robot_ip 192.168.0.23 --robot_port 12348 \
  --max_capture_span_ms 120 \
  --show
```

한 실행에서 지정한 `--data_root` 아래에
`session<NN>_zeus_composite_rig_<MMDD>/`를 새로 할당하고
P1 15 -> P2 20 -> P3 10을 연속 수행한다. 새 촬영은 `--session_label`이 필수다.
의도적인 resume에서만 `--root_folder`로 이미 존재하는 `calib_train`을 명시하며,
이때는 `--session_label`을 함께 쓰지 않는다.

### 7.1 기존 joint pose 자동 재촬영

`replay_and_recapture.py`는 현재 `zeus_gello_calibration/data`에 저장된 legacy
session의 `capture/<index>/robot.json`에서 6축 joint를 읽고, 각 자세에 `movej`로
도착해 완전히 정지한 뒤 다시 촬영한다. 데이터 루트는 다음처럼 명시한다.

현재 원본은 session1 16 pose와 session3 15 pose다. 이는 최종 프로토콜의 P1 15,
P2 20, P3 10 event와 개수 및 metadata 계약이 다르므로 **최종 45-event 데이터셋을
생성하는 명령은 아니다**. 기존 자세의 동기화 재촬영 또는 경로 확인에만 사용한다.

1. Robot을 움직이지 않는 계획 확인:

```bash
python3 zeus_gello_calibration/replay_and_recapture.py \
  --session 1 \
  --data-root zeus_gello_calibration/data \
  --output-subdir capture_replayed_auto
```

2. 카메라 없이 저속으로 한 pose씩 이동 경로 확인:

```bash
python3 zeus_gello_calibration/replay_and_recapture.py \
  --session 1 \
  --data-root zeus_gello_calibration/data \
  --execute --motion-only --jnt-speed 3
```

3. 같은 저속으로 한 pose씩 확인하며 실제 촬영:

```bash
python3 zeus_gello_calibration/replay_and_recapture.py \
  --session 1 \
  --data-root zeus_gello_calibration/data \
  --output-subdir capture_replayed_step_check \
  --execute --jnt-speed 3
```

4. 2~3단계를 실제 robot에서 통과한 뒤 연속 자동 촬영:

```bash
python3 zeus_gello_calibration/replay_and_recapture.py \
  --session 1 \
  --data-root zeus_gello_calibration/data \
  --output-subdir capture_replayed_auto \
  --execute --no-step --jnt-speed 3
```

`--no-step`도 시작 직전에 `go`를 한 번 요구한다. session1은 촬영 전부터 cube를
동일한 강체 파지 상태로 유지해야 하며, 필요하면 `--regrasp-joints`를 사용한다.
session3도 위 명령의 `--session 1`을 `--session 3`으로 바꿔 실행할 수 있으며,
stationary target을 움직이면 안 된다.

기존 `capture_replayed`에는 이미 결과가 있으므로 시험 촬영은
`capture_replayed_step_check`, 최종 자동 촬영은 `capture_replayed_auto`로 분리한다.
지정한 출력 폴더가 비어 있지 않으면 코드는 촬영 전에 중단한다. 원본 `capture`를
덮어쓰는 `--overwrite-source`는 사용하지 않는다.

session2는 각 placement에서 cube release, 촬영, pick 동작을 수행해야 하므로 이
replay 코드로 실행할 수 없다. 기존 legacy session2를 재현할 때만
`zeus_gello_calibration/session2_pick_and_place.py`를 쓰고, 최종 P1/P2/P3 45-event
촬영은 위 7절의 `server/c1.py`와 root `03_capture.py`만 사용한다.

## 8. 저장 결과와 완료 판정

Robot server가 보내고 PC가 원본으로 보존하는 핵심 값은 다음과 같다.

| 필드 | 단위/기준 | 용도 |
| --- | --- | --- |
| `flange_pose_6dof_mm_deg` | `[x,y,z,rz,ry,rx]`, mm/deg | `T_base_flange`, FK와 hand-eye 기준 |
| `joints_deg` | 6축 degree | 도달 자세와 반복 실행 검증 |
| `motion_tcp_pose_6dof_mm_deg` | tool3 TCP, mm/deg | 이동/안전 진단 |
| `gripper_io` | digital input 4개 | grasp/release 상태 증거 |
| `server_epoch_s` | robot-server wall clock | 명령 시각 추적 |
| `release_state.pre_release/post_release` | 완전한 robot state | P2 release 검증 |

PC는 모든 카메라 RGB, aligned depth, device/host timestamp, 파일 hash, marker 진단,
intrinsic/config/geometry hash도 함께 저장한다.

| 파일 | 역할 |
| --- | --- |
| `sessionNN/capture_waypoints.json` | robot에 전달한 frozen pose plan |
| `sessionNN/composite_rig_geometry.json` | 촬영에 사용한 frozen rig geometry |
| `sessionNN/calib_train/meta.json` | 모든 attempt, camera, robot/release state |
| `sessionNN/calib_train/capture_waypoints_recorded.json` | 실제 실행 pose와 상태 |
| `sessionNN/calib_train/capture_protocol_manifest.json` | 45 planned ID 완료·누락·attempt 수 |

완료 조건은 manifest의 `expected_event_count=45`, `selected_event_count=45`, 누락 ID 0개다.
분석에는 marker 성공 여부와 무관하게 각 planned ID의 첫 transport/sync-valid attempt만
사용한다. P2는 `split_unit_id=PLACEMENT_XX`, P3는
`analysis_group_id=P3:STATIONARY_RIG`로 저장된다.

## 9. 촬영 후 처리 상태

`sessionNN`은 실제 생성된 번호로 바꾼다.

```bash
python3 04_filter_observations.py \
  --session-root zeus_gello_calibration/data/session<NN>_<설명>_<MMDD>/calib_train \
  --intrinsics-dir intrinsics
```

04는 final protocol에서 `selected_for_analysis=true`인 45개 event만 사용하고 P1의
gripped cube 관측도 자동으로 보존한다. transport/sync 실패 attempt는 manifest에
들어가지 않는다.

현재 아래 05/06 명령은 **아직 실행하지 않는다**.

```bash
python3 05_calibrate.py \
  --root_folder zeus_gello_calibration/data/session<NN>_<설명>_<MMDD>/calib_train \
  --intrinsics_dir intrinsics \
  --split_seed 20260731 \
  --num_inits 3 \
  --observation-manifest zeus_gello_calibration/data/session<NN>_<설명>_<MMDD>/calib_out/capture_filter/Step2b_observation_manifest.json

python3 06_make_report.py \
  --root_folder zeus_gello_calibration/data/session<NN>_<설명>_<MMDD>/calib_train
```

05에는 P2 placement 단위 split이 구현되어 있다. 같은 placement의 view 0/1은 모두
train 또는 모두 held-out이며 P1/P3는 항상 train이다. 그러나 기존 05 solver는 board를
하나의 정지 target, cube를 set별 target으로 가정한다. P1에서 함께 움직이는 board와
cube를 `T_rig_board`, `T_rig_cube`로 하나의 공통 rig pose에 연결하는 모델은 아직
구현되지 않았다. 현재 코드는 이 상태에서 잘못된 결과를 내는 대신 명시적으로 중단한다.

표준 명칭은 `VISION`, `FK`, `corrected-FK`다. ALL/Train/Held-out Cube RMSE와
ALL/Train/Held-out Cross-view Cube RMSE는 내부 지표이고, 최종 방법 순위는 별도
Independent External GT의 TRE mm/rotation deg로 결정한다. External GT가 없으면 해당
열은 `Pending/null`로 둔다.

## 10. 남은 실행 전 상태

- 완료: `03_capture.py` 최종-only 진입점과 45-event PC/server 통신
- 완료: pose-plan v4 다양성 validator, sync-only retry, completion manifest
- 완료: P2 placement-grouped split, P3 독립 `set_index=10`, selected-attempt filtering
- 미완료: 실제 composite rig geometry 측정과 45 pose teaching
- 미완료: 새 큐브 기준 robot grip/place 높이 재실측 및 저속 dry run
- 미완료: 05의 phase-aware 공통-rig reprojection 모델과 그 회귀 검증
- 미완료: calibration 45 events와 분리된 Independent External GT 수집
