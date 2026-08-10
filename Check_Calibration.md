# 재촬영부터 Calibration Table 전체 결과까지 — 단일 실행 체크리스트

이 파일 하나만 위에서 아래로 실행한다. 비교 방법과 주장 기준의 원문은
[Calibration_Experiment_table.md](Calibration_Experiment_table.md)이며, 현장 촬영·분석·반복 session·최종
통계의 실행 상태는 모두 이 파일에서만 관리한다.

## 사용법

- 완료한 항목은 IDE에서 `- [ ]`를 `- [x]`로 바꾼다.
- `<date>`, `<MEASURED_M>`, pose JSON, safe joint 값은 실제 값으로 교체한 뒤 명령을 실행한다.
- 방법별로 다시 촬영하지 않는다. **한 session의 동일 raw frames, frozen intrinsics, train/test mask와
  blind GT를 모든 방법이 공유**한다.
- 상위 단계의 진행 조건을 통과하지 못하면 다음 단계로 넘어가지 않고 첫 번호 session을 `engineering pilot`으로
  표시한다.
- 한 session만으로 “Ours가 가장 우수하다”고 결론 내리지 않는다. 최종 통계 단위는 독립 카메라 재설치
  session이다.

## 0. 현재 준비 상태와 오늘의 목표

### 구현 상태

- [x] A2/A3/A4a/A4b/A4 covariance-weighted robust solver 구현
- [x] 동일 정보량의 fair B1 independent solver 구현
- [x] GT를 읽지 않는 blind prediction exporter 구현
- [x] External-GT hierarchical bootstrap·Holm·margin evaluator 구현
- [x] OpenCV 4.12 환경에서 관련 테스트 32개 통과
- [x] 기존 데이터에서 A2/A3/A4a/A4b/A4/B1 software preflight 수렴
- [ ] Target-ROI sharpness·clipping·marker-edge를 실제 저장 gate에 연결하고 검증
- [ ] 실제 반복측정 FK covariance artifact 준비
- [ ] A0/A1을 A2와 동일한 factorized visual backend에 연결
- [ ] B2 soft-FK cube-only와 B3 board-only 공정 arm 연결
- [ ] C1–C4 same-session classical hand-eye runner 연결
- [ ] A5를 포함한다면 train-only 6-DoF correction 구현

현재 미완료 항목이 있으므로 첫 고해상도 촬영은 **첫 자동 번호 session의 engineering pilot**이다. 그 raw data로
모든 미완료 runner와 gate까지 end-to-end 검증한 뒤 confirmatory session을 시작한다.

### 3시간 촬영의 현실적인 완료 범위

3시간 안의 목표는 한 설치 상태에서 다음을 확보하는 것이다.

- 고해상도 intrinsic: 카메라당 train 30장 이상 + validation 10장
- A_placement: 최소 13 placements × placement당 8 accepted views
- B_eyetohand: 최소 30 accepted poses
- Blind external-GT: 서로 다른 최소 30 poses
- 현장 audit와 원본 백업

3시간 안에 5개 독립 session, 모든 method 실행과 최종 통계까지 끝내는 것은 목표가 아니다.

## 1. 촬영 전 사전 결정

결과를 보기 전에 결정하고 session manifest에 기록한다.

- [ ] Ours-core를 A4로 고정한다.
- [ ] 기본 결과 범위를 `A0,A1,A2,A3,A4a,A4b,A4,B1,B2,B3,C1,C2,C3,C4`로 고정한다.
- [ ] A5를 제외할지 포함할지 결정한다.
- [ ] 외부 GT 장비·jig·좌표계·측정 순서와 uncertainty 측정법을 확정한다.
- [ ] ADD 또는 ADD-S와 cube symmetry set을 확정한다.
- [ ] 회전 non-inferiority margin `m_R`을 확정한다.
- [ ] Tail margin `m_P95`, failure margin `m_fail`, worst-workspace margin `m_stratum`을 확정한다.
- [ ] Multi-start 선택 규칙을 “blind GT를 보지 않고 converged run 중 training objective 최저”로 고정한다.
- [ ] Pilot session을 최종 confirmatory 표본에 포함할지 사전에 정한다.
- [ ] Train pose pool과 blind pose pool을 물리적으로 분리한다.

## 2. 촬영 시작 전 GO/NO-GO

아래 중 하나라도 미완료면 최종 후보가 아닌 engineering pilot으로만 촬영한다.

- [x] 외부 GT 장비를 바로 측정할 수 있다.
- [x] 네 카메라가 연결되고 serial→cam index와 gripper cam2가 고정돼 있다.
- [x] Cube/marker/ChArUco 실측 config가 준비돼 있다.
- [x] Empty/gripped safe pose가 준비돼 있다.
- [ ] 보완된 teaching pose 파일이 준비되고 다양성 검사를 통과했다.
- [ ] 카메라별 ROI sharpness threshold와 clipping·marker-edge 기준이 저장 gate에 연결됐다.
- [x] OpenCV 4.x 환경에서 필수 테스트가 통과한다.
- [x] 새 session 저장 공간이 충분하다.
- [ ] 로봇 작업자가 승인된 safeguarding 상태와 E-stop 접근성을 확인했다.

필수 테스트:

```bash
python -m pytest -q \
  test_final_experiment_contract.py \
  test_capture_gate.py \
  test_camera_timestamp_buffer.py \
  test_waypoint_safety.py \
  test_CP_ablation_7row.py
```

진행 조건:

- [ ] 테스트가 전부 통과한다.
- [ ] 촬영 commit 또는 working-tree patch hash를 기록했다.
- [ ] 기존 `data/session`과 `intrinsics`를 덮어쓰지 않는다.

## 3. 타이머 시작 전에 끝낼 물리 준비

### 3.1 Target·robot·GT 실측

- [ ] Cube 실제 변 길이와 marker ID별 검은 사각 변을 측정한다.
- [ ] Marker center·부착 face·인쇄 roll과 `face_roll_deg`를 확인한다.
- [ ] ChArUco square/marker 길이를 측정한다.
- [ ] Grasp 중심과 `T_gripper_cube`, GT jig의 `T_cube_GTmarker`를 확인한다.
- [ ] 인증 길이 기준물로 PnP·depth·robot FK scale을 교차검증한다.
- [ ] `CP_common.ROBOT_POS_SCALE_PINNED`의 물리적 근거를 기록한다.
- [ ] 실측 config와 intrinsic 파일의 SHA-256을 기록한다.

### 3.2 품질 gate 고정

카메라별 선명한 target ROI pilot 20장을 사람이 확인해 threshold를 정한다.

- [ ] Target ROI Laplacian sharpness threshold를 카메라별로 고정한다.
- [ ] Target ROI dark/white clipping을 각각 5% 미만으로 고정한다.
- [ ] Marker-image edge 거리 최소 10 px를 적용한다.
- [ ] Marker 투영 한 변 최소 약 30 px를 적용한다.
- [ ] Multi-marker PnP 목표 1 px 이하, 허용 상한 2 px를 적용한다.
- [ ] Depth samples 최소 20, plane residual 목표 10 mm 이하·상한 20 mm를 적용한다.
- [ ] Gate 결과와 camera별 실패 이유가 `meta.json`에 저장된다.
- [ ] Gate 실패 frame이 `--allow_force_save` 없이 저장되지 않는다.
- [ ] 기존 데이터 replay에서 blur·clipping·edge 실패 사례를 사람이 확인한다.

### 3.3 Teaching pose와 회전 다양성

```bash
python test_robot_teaching_waypoint.py \
  --root <TEACHING_POOL_DIR> \
  --n_set <EXPECTED_SET_COUNT> \
  --n_grip <EXPECTED_GRIP_COUNT> \
  --n_pose <EXPECTED_EIH_POSE_COUNT>
```

- [ ] A pose에 결합 roll/pitch가 다른 자세 4–6개 이상을 보완한다.
- [ ] Placement yaw의 기존 빈 구간 약 205°를 채운다.
- [ ] A/eih와 B/gripped 모두 회전축 3방향 excitation을 통과한다.
- [ ] 중복 pose, joint limit·singularity·self-collision 근접 pose를 제거한다.
- [ ] 모든 pose의 camera·marker visibility를 preview한다.
- [ ] Joint angles, `T_base_gripper`, quaternion, rotation matrix와 규약을 저장한다.

### 3.4 FK covariance

Blind GT를 사용하지 않고 같은 grasp/placement를 반복 측정한다.

- [ ] `Log(T_prior^-1 T_measured)` 6-DoF residual을 최소 3회 이상 수집한다.
- [ ] Twist 순서 `[rx,ry,rz,tx,ty,tz]`, 단위 `[rad,m]`를 사용한다.
- [ ] Covariance가 symmetric positive-definite인지 확인한다.
- [ ] [FK covariance 템플릿](protocol_templates/fk_covariance_TEMPLATE.json)을 실제 값으로 채운다.
- [ ] `measurement_source`, `n_repeats≥3`, `blind_external_gt_used=false`를 기록한다.
- [ ] Blind 결과를 보기 전에 파일 hash와 robust-loss scale을 고정한다.

## 4. 3시간 현장 촬영

### 00:00–00:10 — 새 폴더와 설정 동결

```text
intrinsics/recapture_<date>/
data/
└── sessionNN/
    ├── session_manifest.json
    ├── calib_train/
    ├── blind_test/
    ├── calib_out/
    ├── calibration_methods/
    ├── predictions/
    └── audit/
```

- [ ] 날짜·작업자·session ID를 기록한다.
- [ ] Code/config/intrinsic hash와 camera serial→index를 기록한다.
- [ ] 해상도·FPS·exposure·gain·white balance·focus를 기록하고 동결한다.
- [ ] Robot controller/model·TCP·scale·rotation convention을 기록한다.
- [ ] 외부 GT 장비·좌표계·uncertainty를 기록한다.
- [ ] `--allow_force_save`, `--allow_intrinsics_res_mismatch`, `--no-save-depth`를 사용하지 않는다.

### 00:10–00:25 — 네 카메라 고해상도 동시 확인

- [ ] `1280×720 @ 15 FPS`로 네 카메라가 동시에 5분 이상 안정적으로 동작한다.
- [ ] RGB/depth alignment가 정상이고 USB timeout·frame drop이 반복되지 않는다.
- [ ] Host timestamp span 목표 50 ms 이하, 허용 상한 120 ms 이하를 확인한다.
- [ ] Target ROI가 선명하고 marker 크기·edge 조건을 통과한다.

실패하면 해상도를 강행하지 말고 해당 session을 engineering pilot으로 표시한다.

### 00:25–00:55 — 고해상도 intrinsic 재촬영

```bash
python Step1_dump_all_intrinsics.py \
  --out_dir intrinsics/recapture_<date> \
  --color_w 1280 --color_h 720 --fps 15

python Step1b_charuco_intrinsics.py \
  --intr_dir intrinsics/recapture_<date> \
  --min_views 30 \
  --square_len_m <MEASURED_M> \
  --marker_len_m <MEASURED_M> \
  --save_images
```

카메라별:

- [ ] 중앙·네 모서리, 가까움·중간·멀리를 포함한다.
- [ ] Pitch/yaw 약 ±30–45°를 포함하고 fronto-parallel 반복을 피한다.
- [ ] Blur·clipping·잘린 board view를 제외한다.
- [ ] Train accepted view 최소 30장, 가능하면 40장을 확보한다.
- [ ] Fitting에 사용하지 않는 validation view 10장을 확보한다.

### 00:55–01:05 — Intrinsic 즉시 판정

- [ ] 네 카메라 intrinsic 해상도가 모두 `1280×720`이다.
- [ ] Calibration RMS 목표 약 0.3 px 이하와 held-out validation을 함께 통과한다.
- [ ] Focal length와 principal point가 물리적으로 타당하다.
- [ ] 실패한 카메라만 즉시 재촬영한다.
- [ ] 기존 640×480 intrinsic scaling으로 대체하지 않는다.
- [ ] 이후 모든 비교 방법에서 intrinsic을 동결한다.

### 01:05–01:15 — Safe path dry-run

```bash
python tools/build_waypoints_from_pool.py \
  --grip <GRIP_POSES_JSON> \
  --poses <A_POSES_JSON> \
  --sets <SETS_JSON> \
  --output data/next_capture_waypoints.json \
  --safe_joints_empty <D1,D2,D3,D4,D5,D6> \
  --safe_joints_gripped <D1,D2,D3,D4,D5,D6> \
  --n_per_set 6 --b_station_set <마지막 set 인덱스>
```

`--b_station_set`은 `A_sets_plus_B_station` 레이아웃을 만든다. Set마다 A placement만
찍고, 마지막에 큐브를 스테이션 한 곳으로 옮겨 거기서만 B 스윕을 한 번 한다. 기존
`per_set_AB`(set마다 `--n_grip_per_set`개씩 B 반복)도 그대로 쓸 수 있으며, 어느
레이아웃인지는 출력 파일의 `capture_protocol`로 선언되어 서버가 첫 모션 전에 그 모양
그대로 검증한다.

- [ ] Empty payload `current→safe→첫 A pose→safe`를 저속 확인한다.
- [ ] Gripped payload `current→safe→첫 B pose→safe`를 저속 확인한다.
- [ ] Safe pose뿐 아니라 모든 `current→safe→target→safe` 경로를 확인한다.
- [ ] 케이블·카메라 mount·board·cube jig 간섭이 없다.
- [ ] 작업자는 승인된 safeguarding 상태이며 E-stop에 접근 가능하다.

### 01:15–02:20 — Calibration train 촬영

```bash
python Step2_capture.py \
  --data_root data \
  --waypoints_file data/next_capture_waypoints.json \
  --intrinsics_dir intrinsics/recapture_<date> \
  --use_robot --manual_robot \
  --robot_ip 192.168.0.23 --robot_port 12348 \
  --width 1280 --height 720 --fps 15 \
  --settle_time 2.0 \
  --max_capture_span_ms 120 \
  --min_cams_with_cube 2 \
  --min_fixed_cams_with_cube 1 \
  --min_cube_pnp_ok_cams 2 \
  --min_fixed_cube_pnp_ok_cams 1 \
  --fixed_multimarker_min_markers 2 \
  --max_cube_pnp_reproj_mean_px 2 \
  --min_depth_samples 20 \
  --a_min_fixed_multimarker_cams 1 \
  --gripper_cube_min_markers 1 \
  --min_gripper_charuco_corners 8 \
  --max_gripper_depth_plane_mean_mm 20 \
  --b_min_fixed_cams_with_cube 2 \
  --b_min_fixed_cube_pnp_ok_cams 2 \
  --b_min_fixed_multimarker_cams 2 \
  --b_min_fixed_depth_quality_cams 1 \
  --b_max_fixed_depth_plane_mean_mm 20 \
  --start_gate --show
```

`--root_folder`를 생략했으므로 실행할 때마다 기존 번호의 최댓값에 1을 더한
`data/sessionNN/calib_train`이 자동 생성된다. 터미널에 출력된 `SESSION` 경로를 아래의
`<SESSION_DIR>`에 기록한다. 기존 session을 의도적으로 이어 찍을 때만
`--root_folder data/sessionNN/calib_train`을 직접 지정한다.

- [ ] 이번 실행에서 출력된 `<SESSION_DIR>`를 기록했다.

#### 01:15–02:00 — A_placement

- [ ] 최소 13 placements, 권장 15 placements를 확보한다.
- [ ] Placement당 최소 8 accepted eih views를 확보한다.
- [ ] Center/left/right/near/far/low/high와 가능한 roll/pitch/yaw를 포함한다.
- [ ] Cube를 고정 카메라 최소 2대가 관측한다.
- [ ] 전체 cube PnP 성공 카메라 2대 이상, gripper cube PnP 성공을 확인한다.
- [ ] Gripper ChArUco corners 8개 이상을 확인한다.
- [ ] 가능한 frame에서 cube marker 2개 이상, 목표 3개 이상을 확보한다.
- [ ] 모든 저장 frame의 gate가 PASS다.

시간이 부족하면 placement를 13개 아래로 줄이지 말고 placement당 view를 8개까지 줄인다.

#### 02:00–02:20 — B_eyetohand

- [ ] 최소 30 accepted gripped poses를 확보한다.
- [ ] Workspace 위치와 roll/pitch/yaw 세 축을 모두 변화시킨다.
- [ ] 거의 같은 joint/TCP pose를 반복하지 않는다.
- [ ] 고정 카메라 2대 이상에서 cube visible·PnP·multi-marker 조건을 통과한다.
- [ ] B에서는 gripper cube/ChArUco 관측을 저장 필수조건으로 강제하지 않는다.
- [ ] 모든 이동이 `target→safe→next target` 순서를 지킨다.

### 02:20–02:50 — Blind external-GT test

Calibration train 종료 후 카메라·focus·exposure·GT registration을 움직이지 않는다.

- [ ] Calibration과 다른 6-DoF poses를 최소 30개 사용한다.
- [ ] Center 5, edge 6, near/far 6, low/high 6, hull 경계/밖 4, strong rotation 3개 이상을 포함한다.
- [ ] 각 pose에 고유 `external_gt_pose_id`를 저장한다.
- [ ] 같은 ID로 `T_base_cube_GT` translation·rotation과 uncertainty를 저장한다.
- [ ] RGB/depth/robot state/GT pose ID가 일대일 대응한다.
- [ ] Blind GT는 prediction 완료 전 calibration·threshold·run 선택 코드에서 읽지 않는다.

30분 안에 30개를 못 채우면 최종 데이터처럼 포장하지 않고 engineering pilot으로 표시한다.

### 02:50–03:00 — 카메라 이동 전 현장 감사와 백업

- [ ] Gate PASS 100%, force-save 0개
- [ ] A accepted captures 104개 이상
- [ ] B accepted poses 30개 이상
- [ ] Blind GT poses 30개 이상
- [ ] RGB/depth/meta/robot/GT 파일 수 일치
- [ ] Intrinsic과 capture 해상도 일치
- [ ] Camera·marker ID·placement·rotation coverage 충족
- [ ] Timestamp span P50/P95/max 확인
- [ ] Sharpness·clipping·marker-edge 실패 frame 0개
- [ ] PnP reprojection RMSE/median/P95 확인
- [ ] Depth valid ratio와 plane residual 확인
- [ ] Config·intrinsic·code hash 저장
- [ ] 원본을 별도 저장장치에 백업

하나라도 실패하면 카메라를 움직이지 말고 해당 항목만 추가 촬영한다.

## 5. 촬영 직후 현재 session 분석

### 5.1 기본 calibration과 데이터 진단

```bash
python Step3_calibration.py \
  --root_folder <SESSION_DIR>/calib_train \
  --intrinsics_dir intrinsics/recapture_<date> \
  --out_dir <SESSION_DIR>/calib_out \
  --target both \
  --joint_solve reprojection_fk_fixed

python Step4_verify.py \
  --root_folder <SESSION_DIR>/calib_train \
  --intrinsics_dir intrinsics/recapture_<date>

python Step5_export_reports.py \
  --root_folder <SESSION_DIR>/calib_train \
  --intrinsics_dir intrinsics/recapture_<date>
```

- [ ] Camera 등록 수와 failure reason을 확인한다.
- [ ] Face-roll/PnP/depth/rotation convention 오류가 없다.
- [ ] 이 단계 숫자를 final method ranking으로 사용하지 않는다.

### 5.2 Historical 7행 regression

```bash
python CP_ablation_7row.py \
  --root_folder <SESSION_DIR>/calib_train \
  --intrinsics_dir intrinsics/recapture_<date> \
  --calib_dir <SESSION_DIR>/calib_out \
  --out_dir <SESSION_DIR>/historical_7row \
  --num_inits 3 --max_nfev 300 --tol 1e-8
```

- [ ] 7행의 observation 지원 여부와 failure reason을 확인한다.
- [ ] Historical B1/B2 정의가 현재 최종 명세와 다르므로 숫자를 최종표에 복사하지 않는다.

### 5.3 A2/A3/A4 분해와 fair B1

```bash
python CP_final_methods.py \
  --root_folder <SESSION_DIR>/calib_train \
  --intrinsics_dir intrinsics/recapture_<date> \
  --calib_dir <SESSION_DIR>/calib_out \
  --fk_covariance_json <SESSION_DIR>/fk_covariance.json \
  --out_dir <SESSION_DIR>/calibration_methods \
  --methods A2,A3,A4a,A4b,A4,B1 \
  --num_inits 3 --max_nfev 300 --tol 1e-8
```

- [ ] FK covariance의 `confirmatory_ready=true`를 확인한다.
- [ ] A2/A4의 visual objective와 solver budget이 동일하다.
- [ ] A4a→A4b→A4 기여와 fair B1→A4를 분리한다.
- [ ] 모든 run의 convergence·failure를 기록한다.
- [ ] Method별 run index를 사전 고정한 training-only 규칙으로 선택한다.

### 5.4 최종표에 아직 연결할 행

아래 항목이 첫 engineering-pilot session에서 실행되기 전에는 confirmatory 촬영을 시작하지 않는다.

- [ ] A0/A1: A2와 동일 factorized visual backend로 실행
- [ ] B2: A4와 동일 covariance-weighted robust soft-FK의 cube-only arm 실행
- [ ] B3: A2와 동일 factorized visual backend의 board-only arm 실행
- [ ] C1 Tsai–Lenz same-session baseline 실행
- [ ] C2 Park–Martin same-session baseline 실행
- [ ] C3 Horaud same-session baseline 실행
- [ ] C4 Daniilidis same-session baseline 실행
- [ ] A5 포함 시 train-only 6-DoF SE(3) correction 실행
- [ ] 모든 방법에 대해 deterministic best-run exporter 또는 고정 run-index manifest를 생성

### 5.5 Blind prediction

GT 파일을 열지 않고 method마다 반복한다.

```bash
python CP_blind_pose_predict.py \
  --final_methods_json <SESSION_DIR>/calibration_methods/final_methods.json \
  --method A4 \
  --run_index <TRAIN_ONLY_SELECTED_INDEX> \
  --blind_root <SESSION_DIR>/blind_test \
  --intrinsics_dir intrinsics/recapture_<date> \
  --output <SESSION_DIR>/predictions/A4.json
```

- [ ] 모든 method가 동일 blind pose ID를 평가한다.
- [ ] Missing prediction을 failure로 남긴다.
- [ ] 공통 카메라 교집합 정확도와 전체 camera coverage를 모두 저장한다.
- [ ] Prediction이 모두 저장된 후에만 blind GT를 evaluator에 연결한다.

### 5.6 현재 session 등급

- [ ] **Final-candidate**: 모든 수량·gate·GT·runner·audit 통과
- [ ] **Engineering pilot**: 파이프라인은 통과했지만 gate·수량·runner 중 일부 미달
- [ ] **Discard/re-shoot**: intrinsic/config 불일치, 안전 위반, blur/timestamp 또는 GT 대응 실패

첫 engineering-pilot session은 전체 파이프라인 검증용이며 기본적으로 최종 통계에서 제외한다. 이 단계 뒤에 method 정의,
threshold, split, hyperparameter와 분석 코드를 동결한다.

## 6. 독립 재설치 session 반복

첫 engineering-pilot session의 end-to-end 통과 후 동일 절차를 반복한다.

- [ ] 다음 번호 session 전에 카메라를 독립적으로 재설치하고 새 manifest를 작성한다.
- [ ] Session당 A 13–15 placements × 6 views를 촬영한다.
- [ ] Session당 B는 스테이션 1곳에서 15 poses를 한 번만 촬영한다.
- [ ] Session당 서로 다른 blind external-GT 30 poses를 촬영한다.
- [ ] 카메라를 움직이기 전에 session audit와 백업을 완료한다.
- [ ] 독립적인 pilot session 5개를 완료한다.
- [ ] Pilot의 session-level paired TRE variance로 power analysis를 수행한다.
- [ ] 필요한 최종 독립 session 수를 확정하고 추가 촬영한다.

한 session의 pose 30개는 독립 session 30개가 아니다.

### 촬영량 근거와 감수한 리스크 (2026-08-10 확정)

B를 set마다 반복하지 않는 이유: gripped 캡처의 cube 관측은 solve에 들어가지 않는다
(`CP_ablation_7row.detect_observations`의 `exclude_gripped_cube=True`, `CP_C1`/`CP_C3`/
`CP_cube_selfcal`/`Step3` 모두 동일). B 게이트는 그리퍼 카메라도 요구하지 않는다
(`B_eyetohand` 프로파일의 `min_cams_with_cube: 0`). 남는 소비처는
`CP_common.estimate_robot_pos_scale` 하나이고, 그 추정기는 상대회전 ≤5°·변위 ≥40 mm인
pure-translation 쌍만 쓴다. 따라서 스테이션 15 poses 중 일부는 **자세를 고정한 채 위치만
벌려** 티칭해야 하며, 자세가 전부 다른 15 poses는 유효 쌍을 0개 만든다.

A를 set당 6 views로 정한 것은 하한을 깎은 결정이다. Canonical 아티팩트
(`CP_result/ablation_7row_canonical/seven_row_ablation.json`, 13 sets × 11 views) 실측
전환율은 A view → `train_eih_cube_event` 기준 평균 0.75, 최악 set 0.55다.

| n_per_set | 기대 | 최악 set | split 계약 최소 3 |
| ---: | ---: | ---: | --- |
| 6 (채택) | 4.5 | **3.3** | 턱걸이 |
| 8 | 6.0 | 4.4 | 여유 |

- [ ] 6 views에서는 전환율이 나쁜 set이 계약 최소치 아래로 떨어져 solve에서 탈락할 수 있다.
      탈락한 set은 position holdout에서 위치 하나가 통째로 사라진다. **촬영 직후 set별
      `train_eih_cube_events`를 확인하고, 3 미만인 set은 카메라를 옮기기 전에 재촬영한다.**
- [ ] Builder가 생성 시 같은 투영값을 출력하고 `_meta.projected_train_eih_cube_per_set`에
      기록한다. 그 경고를 무시하고 진행하지 않는다.

### 카메라별 저장 범위

`Step2_capture.py`는 solve가 쓸 수 없는 프레임을 저장하지 않는다. 두 규칙 모두
`capture_config`에 기록되므로 한 session이 두 저장 정책을 섞을 수 없고, 저장하지 않은
카메라도 `cams[ci].saved=false`와 `skip_reason`으로 남는다(조용히 빠지지 않는다).

| 옵션 | 기본값 | 규칙 | 근거 (session01 실측) |
| --- | --- | --- | --- |
| `--a_fixed_cam_views_per_set` | 1 | A: set당 앞 N개 캡처만 고정 카메라를 저장. 나머지 뷰는 그리퍼 카메라만 | 큐브가 바닥에 정지해 있고 팔만 움직인다. set×고정카메라 39조합 전부가 뷰마다 동일 마커 수, 팔에 가려 첫 뷰만 놓친 경우 **0/39** |
| `--b_save_gripper_cam` | off | B: 그리퍼 카메라를 저장하지 않음 | 큐브를 쥐고 있어 손목 카메라가 큐브를 못 본다. `cube_visible` **0/89**, ChArUco ≥8 코너 **3/89**. B 게이트도 손목 카메라에 아무 요구 없음 |

- [ ] 잃는 것은 관측이 아니라 반복에 의한 √N 노이즈 평균화다. A 6뷰 → 1뷰면 set별
      고정-카메라 큐브 코너 노이즈가 √6 ≈ 2.4배 커진다. 이를 줄이려면
      `--a_fixed_cam_views_per_set 2`로 올린다(저장량은 여전히 크게 준다).
- [ ] 13 sets × A 6 + B 15 기준 카메라-프레임 **372 → 162 (56% 감소)**,
      이미지 84 MB → 37 MB. `marker_quads/` 진단 이미지(93장 39 MB)는 이 규칙의
      대상이 아니므로 필요 없으면 별도로 정리한다.

## 7. External-GT 최종 평가

[External-GT manifest 템플릿](protocol_templates/external_gt_eval_manifest_TEMPLATE.json)을 복사해 모든
session, GT와 method prediction 경로를 채운다.

```bash
python CP_final_external_gt_eval.py \
  --manifest data/final_external_gt_manifest.json \
  --output_dir CP_result/final_external_gt
```

### 필수 산출물

- [ ] `TRE_t` mm: mean/P50/P95/max/95% CI
- [ ] Rotation geodesic error deg: mean/P50/P95/max/95% CI
- [ ] 사전 선택한 ADD 또는 ADD-S: mean/P95/95% CI
- [ ] Workspace center/edge/near/far/height별 P50/P95와 worst stratum
- [ ] Calibration/inference failure rate와 등록 카메라 수
- [ ] Held-out raw-corner reprojection RMSE/median/P95
- [ ] Cross-camera 3D disagreement
- [ ] Session 간 `bTf`와 `gTc` repeatability를 각각 mm/deg로 보고
- [ ] Runtime·iterations·convergence rate·필요 views

### “Ours가 가장 우수하다”는 주장 합격 조건

- [ ] A4−confirmatory baseline 각각의 mean TRE hierarchical CI 상한이 0보다 작다.
- [ ] TRE baseline family가 Holm 보정을 통과한다.
- [ ] Rotation superiority 또는 사전 `m_R` non-inferiority를 통과한다.
- [ ] ADD/ADD-S의 사전 정의 계약을 통과한다.
- [ ] P95가 `m_P95`를 통과한다.
- [ ] Failure rate가 `m_fail`을 통과하고 등록 카메라 수가 줄지 않는다.
- [ ] Worst-stratum P95가 `m_stratum`을 통과한다.
- [ ] TRE 개선량이 외부 GT uncertainty floor보다 크다.
- [ ] 실패 session을 정확도 평균에서 삭제하지 않는다.

모든 항목을 통과한 뒤에만 “Ours가 3D 공간 정합에서 가장 우수하다”고 결론 낸다.

## 8. 최종 결과 확정 후 코드 정리

- [ ] 최종 runner·prediction·evaluator를 clean environment에서 재실행한다.
- [ ] 논문 표와 그림이 새 final artifact에서만 생성된다.
- [ ] `legacy/manifest.json` hash와 실제 파일이 일치한다.
- [ ] 과거 결과를 release/tag/archive로 보존한다.
- [ ] Import와 test 의존성을 확인한다.
- [ ] 그 다음에만 과거 CP 코드의 물리 삭제 여부를 결정한다.

현재는 이 단계의 조건을 충족하지 않았으므로 기존 코드 파일을 삭제하지 않는다.
