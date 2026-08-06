# 실제 셋업 재촬영 프로토콜 — 최종 체크리스트

이 문서는 [Main Ablation 명세](Calibration_Experiment_table.md)에 따라 실제 데이터를 다시 수집하는
현장 절차다. 목표는 동일 데이터로 모든 방법을 paired 비교하고, FK와 독립된 외부 GT로 3D 공간
정합 정확도를 검증하는 것이다.

설정:

- 고정 카메라: cam0, cam1, cam3
- eye-in-hand 카메라: cam2
- Target: 59 mm AprilTag cube + ChArUco board
- 권장 pilot: 독립 camera-installation 5 sessions
- Session당 blind external-GT test poses: 최소 30개

## 촬영 시작 전 중단 조건

아래 항목 중 하나라도 충족되지 않으면 최종 촬영을 시작하지 않는다.

- [ ] 외부 GT 장비/지그와 측정 절차가 확정됐다.
- [ ] Cube·marker·ChArUco 실측값이 설정에 반영됐다.
- [ ] `face_roll_deg`와 marker center가 실제 cube와 일치함을 dry-run으로 확인했다.
- [ ] 카메라 serial→cam index와 gripper camera index가 고정됐다.
- [ ] RGB/depth 해상도와 intrinsic 해상도가 일치한다.
- [ ] `A_placement`와 `B_eyetohand`가 서로 다른 capture gate를 사용한다.
- [ ] Gate 실패 frame이 강제 저장되지 않는 설정으로 pilot을 통과했다.
- [ ] 모든 camera timestamp가 같은 clock domain인지 확인했다.
- [ ] Session manifest에 code/config hash를 기록할 수 있다.

현재 `Step2_capture.py`는 블록과 무관하게 동일 gate를 적용한다. 기존 데이터의 232/232 FAIL은
`A_placement`와 `B_eyetohand`에 동시에 성립할 수 없는 조건을 요구한 영향이 크다. **블록별 gate가
구현되기 전의 촬영은 최종 데이터로 사용하지 않는다.**

## 1. 촬영 폴더와 데이터 분리

기존 `data/session`을 덮어쓰지 않는다. 한 session은 카메라 설치 상태를 유지한 채 intrinsic 확인,
calibration train, blind test를 수집한 단위다.

```text
data/recapture_<date>/
├── session_01/
│   ├── session_manifest.json
│   ├── calib_train/
│   └── blind_test/
├── session_02/
│   ├── session_manifest.json
│   ├── calib_train/
│   └── blind_test/
└── ... session_05/

intrinsics/recapture_<date>/
├── cam0.npz
├── cam1.npz
├── cam2.npz
├── cam3.npz
├── factory_backup/
└── capture_images/
```

`calib_train`과 `blind_test` 사이에는 다음을 유지한다.

- 카메라 위치·초점·해상도·exposure·gain 변경 금지
- Robot base와 외부 GT 좌표계 변경 금지
- Calibration train 위치를 blind test에서 재사용 금지
- Blind GT 값은 calibration·hyperparameter 선택·correction 학습에서 열람 금지

Session이 끝난 뒤에만 카메라를 재장착하거나 전원을 재시작하고 다음 session으로 넘어간다.

## 2. Session manifest에 기록할 정보

각 session 시작 전에 다음을 기록한다.

- [ ] 날짜·작업자·session ID
- [ ] Git commit 또는 working-tree patch hash
- [ ] `config.py`, cube config JSON, intrinsic 파일 SHA-256
- [ ] 카메라별 serial, cam index, 역할(fixed/eih)
- [ ] RGB/depth 해상도, FPS
- [ ] Exposure, gain, white balance, focus/laser power 등 센서 설정
- [ ] Robot IP, controller/kinematic model version, tool/TCP 설정
- [ ] `RB_ROBOT_POS_SCALE` 값과 그 물리적 결정 근거
- [ ] 외부 GT 장비명, 정격 정확도, 좌표계 정의
- [ ] Cube grasp ID와 실측 `T_G_O` 초기값
- [ ] Train/blind-test pose 목록과 생성 seed

## 3. 촬영 전 물리 실측

추측값을 사용하지 않고 캘리퍼 또는 더 정확한 측정기로 기록한다.

- [ ] Cube 실제 변 길이 → `cube_side_m`
- [ ] 옆면 ID 2–5의 검은 사각 변을 각각 측정 → `marker_size_by_id`
- [ ] 윗면 ID 0–1의 검은 사각 변을 각각 측정
- [ ] 각 marker 중심의 cube frame 좌표와 부착 face 확인
- [ ] 각 marker 인쇄 roll과 `face_roll_deg` 확인
- [ ] ChArUco square length와 marker length 측정
- [ ] Gripper 탭 위치·폭·높이와 grasp 중심 측정
- [ ] 인증 길이 기준물 또는 3D jig의 실제 치수와 불확도 기록

기존 확인사항:

- 윗면 marker는 탭 상단 `z=+29.5 mm`에 부착돼 있다.
- 기존 `meta.json`에는 `face_roll_deg=0`이 저장됐지만 현재 모델은 ID 2/3/4에 90/180/270°를
  사용한다. 신규 session에는 **실제로 사용한 config snapshot**을 저장한다.

## 4. 기존 데이터 재처리 — 신규 촬영 전 필수 dry-run

신규 촬영 전에 기존 raw corners로 전체 파이프라인을 검증한다.

- [ ] `markers[].corners_2d`를 현재 `face_roll_deg`로 재-PnP한다.
- [ ] 기존 옆면 다중관측 565건이 재처리되는지 확인한다.
- [ ] Multi-marker cube pose가 기존 약 45건에서 약 674건 수준으로 회복되는지 확인한다.
- [ ] 재처리 multi-marker reprojection 평균이 약 0.66 px 수준인지 확인한다.
- [ ] `depth_valid`, `depth_num_samples`, `depth_plane_median_mm`가 채워지는지 확인한다.
- [ ] Block별 capture gate를 기존 데이터에 적용해 PASS/FAIL 이유가 물리적으로 타당한지 확인한다.
- [ ] `set_cube_center_6dof` 회전 규약의 180° 뒤집힘이 촬영 단계에서 해결됐는지 확인한다.

이 dry-run이 실패하면 새 데이터를 촬영해도 같은 설정 오류가 반복된다.

## 5. Intrinsic 재촬영

Intrinsic은 본 실험 전에 확정하고 Main Ablation 전체에서 동결한다.

### 권장 수량

- 카메라당 accepted train views: 40장 권장, 최소 30장
- 카메라당 held-out validation views: 10장
- Validation view는 intrinsic fitting에 넣지 않는다.

### 촬영 분포

- [ ] 보드가 이미지 중앙과 네 모서리를 모두 덮는다.
- [ ] 가까움/중간/멀리의 최소 3개 거리를 사용한다.
- [ ] Pitch와 yaw를 각각 약 ±30–45° 범위로 변화시킨다.
- [ ] Fronto-parallel view만 반복하지 않는다.
- [ ] 보드가 잘리지 않고 충분한 ChArUco corner가 보인다.
- [ ] 각 view 사이에서 위치·거리·tilt가 실제로 달라진다.

### 품질 조건

- [ ] Focus/exposure/gain을 고정한다.
- [ ] `square_len_m`과 `marker_len_m`에 실측값을 사용한다.
- [ ] Target ROI sharpness가 카메라별 pilot threshold 이상이다.
- [ ] Target ROI의 검정·흰색 clipping 비율이 각각 5% 미만이다.
- [ ] Calibration RMS 목표는 0.3 px 이하로 하되 held-out validation도 함께 본다.
- [ ] Factory와 새 intrinsic의 focal length·principal point 차이를 기록한다.

예시 명령은 다음과 같다. 실측 치수로 값을 교체한다.

```bash
python Step1_dump_all_intrinsics.py \
  --out_dir intrinsics/recapture_<date>

python Step1b_charuco_intrinsics.py \
  --intr_dir intrinsics/recapture_<date> \
  --min_views 40 \
  --square_len_m <MEASURED_M> \
  --marker_len_m <MEASURED_M> \
  --save_images
```

`Step1b`의 전역 sharpness 표시만 믿지 않고, 최종 capture에는 cube/board **target ROI 기반** gate를
사용한다. 기존 cam2는 전역 sharpness 하위 5%가 고정 카메라보다 현저히 낮았으므로 카메라별
threshold가 필요하다.

## 6. 독립 scale·depth 사전 검증

`RB_ROBOT_POS_SCALE`을 기존 결과가 좋아지는 값으로 선택하지 않는다.

- [ ] 인증 길이 기준물 또는 정밀 3D jig를 고정 카메라 3대가 보도록 촬영한다.
- [ ] 매트한 cube 본체/기준면에서 depth를 측정한다. 인쇄 marker의 IR dropout 영역은 피한다.
- [ ] RGB에 정렬된 depth PNG와 `depth_scale_m_per_unit`을 저장한다.
- [ ] Marker 영역 유효 depth sample이 최소 20개인지 확인한다.
- [ ] Depth plane median residual 목표 ≤10 mm, 허용 상한 ≤20 mm로 pilot을 확인한다.
- [ ] PnP 거리, depth 거리, 외부 기준 길이를 비교한다.

판별:

- PnP만 scale이 다름 → intrinsic/marker geometry 문제
- PnP와 depth가 일치하고 FK만 다름 → robot/FK scale 문제
- 세 값이 모두 다름 → 좌표계·depth scale·시간동기 문제부터 해결

Scale을 확정한 뒤 모든 session과 모든 비교 방법에서 같은 값을 사용한다.

## 7. 현장 공통 품질 게이트

Gate는 preview에서 계속 표시하고 **PASS frame만 저장**한다. `force-save`는 디버깅 폴더에서만
허용하며 최종 session에서는 사용하지 않는다.

### 전 카메라 공통

- [ ] Robot 정지 후 최소 2.0초 settle
- [ ] RGB/depth frame 유효, 해상도 일치
- [ ] Camera timestamp span 목표 ≤50 ms, 허용 상한 ≤120 ms
- [ ] Target ROI sharpness ≥ 카메라별 사전 threshold
- [ ] Target ROI dark/white clipping 각각 <5%
- [ ] Marker가 이미지 경계에서 최소 10 px 이상 떨어짐
- [ ] Marker 투영 한 변이 약 30 px 이상
- [ ] Multi-marker PnP reprojection 목표 ≤1 px, 허용 상한 ≤2 px
- [ ] Depth sample ≥20, depth plane median 목표 ≤10 mm, 상한 ≤20 mm
- [ ] Gate 결과와 실패 이유를 `meta.json`에 저장

Sharpness threshold는 전체 이미지 하나의 상수로 정하지 않는다. 각 카메라의 선명한 pilot target ROI
20장을 수동 확인한 뒤 그 분포로 threshold를 사전 고정한다.

## 8. Calibration train 촬영

한 installation session에서 `A_placement`와 `B_eyetohand`를 모두 수집한다. 두 블록은 용도와
gate가 다르다.

### 8.1 A_placement — 배치 cube + 고정/eih 공동관측

목적: A0–A5, B1–B3가 공유할 calibration observations를 수집한다.

권장 구성:

- Workspace placement 15개 권장, 허용 범위 13–20개
- 각 placement에서 eye-in-hand 관측 자세 8–12개
- center/left/right/near/far/low/high 위치를 균형화
- Cube yaw뿐 아니라 식별 가능한 범위에서 roll/pitch도 변화
- 동일 placement 안에서도 eih camera의 위치·roll/pitch/yaw를 넓게 변화

Placement별 coverage 합격 조건:

- [ ] Cube를 고정 카메라 최소 2대가 동시에 관측
- [ ] 그리퍼 카메라 cube PnP 성공
- [ ] 그리퍼 카메라 ChArUco corner ≥8
- [ ] 전체 camera 중 cube PnP 성공 ≥2대
- [ ] 이미지당 cube marker ≥2, 목표 ≥3
- [ ] ID 0과 ID 1의 고정카메라 관측 각각 누적 ≥10회/placement
- [ ] 각 옆면 ID 2–5의 고정카메라 관측 각각 누적 ≥10회/placement
- [ ] 모든 고정카메라가 session 전체에서 충분한 cube/board 공통관측을 가짐

`A_placement` gate 권장 논리:

```text
fixed cube-visible cams >= 1
all cube-visible cams >= 2
fixed cube-PnP cams >= 1
all cube-PnP cams >= 2
gripper cube-PnP == PASS
gripper ChArUco corners >= 8
gripper depth support == PASS
common quality gate == PASS
```

한 placement가 coverage 조건을 채우지 못하면 카메라를 움직이지 말고 cube orientation 또는 eih view만
추가 촬영한다.

### 8.2 B_eyetohand — gripped cube robot-motion anchor

목적: 고정 카메라를 robot motion에 연결하고 `T_G_O`와 eye-to-hand 관측성을 확보한다.

권장 구성:

- Gripped cube poses 40개 권장, 최소 30개
- Robot workspace 전역에서 position 변화
- Roll/pitch/yaw를 각 축에서 넓게 변화
- 거의 같은 관절·TCP pose 반복 금지
- Cube가 고정 카메라 2대 이상에 보이는 자세 우선

`B_eyetohand` gate 권장 논리:

```text
fixed cube-visible cams >= 2
fixed cube-PnP cams >= 2
fixed-camera multi-marker/depth quality == PASS
timestamp/common quality gate == PASS
gripper cube-PnP requirement == OFF
gripper ChArUco requirement == OFF
```

Cube와 gripper camera가 함께 움직이는 경로는 고정 카메라 calibration에 퇴화할 수 있으므로,
`B_eyetohand`에서 gripper cube PnP나 ChArUco를 강제하지 않는다. 기존 232/232 FAIL을 반복하지 않도록
이 블록 구분을 반드시 코드에 반영한다.

### Step2 실행 형태

블록별 gate 구현과 dry-run 통과 후 다음 형태로 실행한다.

```bash
python Step2_capture.py \
  --root_folder data/recapture_<date>/session_01/calib_train \
  --intrinsics_dir intrinsics/recapture_<date> \
  --use_robot --manual_robot \
  --robot_ip 192.168.0.23 --robot_port 12348 \
  --settle_time 2.0 \
  --max_capture_span_ms 120 \
  --start_gate --show
```

현재 CLI만으로는 두 블록에 서로 다른 gate를 완전히 지정할 수 없다. Block-aware gate 코드가 반영되지
않았다면 위 명령을 최종 촬영에 사용하지 않는다.

## 9. Blind external-GT test 촬영

Calibration train을 끝낸 뒤 카메라를 움직이지 않고 촬영한다.

- Session당 blind target poses 최소 30개
- Train placement와 물리적으로 다른 위치·자세 사용
- Workspace center/edge/near/far/low/high를 균형화
- 각 pose의 translation과 rotation GT를 외부 장비로 측정
- GT 측정 불확도와 반복측정 표준편차 기록
- 동일 pose에서 모든 카메라 RGB/depth와 robot state 동시 저장
- Calibration 또는 correction 학습 과정에서 blind GT를 열람하지 않음
- 모든 방법을 동일 raw frame과 동일 GT pose로 평가

권장 분포 예시:

| 구간 | 최소 pose 수 |
| --- | ---: |
| Workspace center | 5 |
| 좌·우 edge | 6 |
| near·far | 6 |
| low·high | 6 |
| train convex hull 밖 또는 경계 | 4 |
| 강한 rotation | 3 |

한 pose가 여러 구간 조건을 만족할 수 있지만, 최종적으로 30개 이상의 서로 다른 6-DoF pose를 확보한다.

## 10. Session 종료 전 현장 감사

카메라를 움직이기 전에 자동 summary와 사람이 보는 preview를 모두 확인한다.

- [ ] 저장 frame의 capture gate PASS 비율 100%
- [ ] 강제 저장 frame 0개
- [ ] Camera별/placement별/marker ID별 관측 수가 목표 충족
- [ ] A_placement에서 ≥2 camera 동시관측 비율 확인
- [ ] B_eyetohand 30–40 pose의 rotation/translation span 확인
- [ ] Blind test 30 poses와 외부 GT가 일대일 대응
- [ ] RGB/depth/meta/robot pose 파일 수 일치
- [ ] Timestamp span 분포와 outlier 확인
- [ ] Target ROI blur·clipping outlier 확인
- [ ] Multi-marker PnP reprojection RMSE/median/P95 확인
- [ ] Depth valid ratio와 plane residual 확인
- [ ] Config·intrinsic·manifest hash 저장

부족한 항목이 있으면 카메라 설치를 유지한 채 해당 placement/marker/pose만 추가 촬영한다. Session을
종료하고 카메라를 재장착한 뒤에는 누락 데이터를 같은 session에 추가하지 않는다.

## 11. Session 반복

- [ ] Session 01 완료 후 카메라를 의도적으로 재장착하거나 전원·mount 상태를 초기화한다.
- [ ] 동일 절차로 최소 5 sessions 반복한다.
- [ ] 각 session의 calibration train과 blind test는 독립 폴더에 저장한다.
- [ ] 방법별 hyperparameter를 blind test 결과에 맞춰 session마다 바꾸지 않는다.
- [ ] Pilot 5 sessions의 paired TRE 분산으로 최종 표본 수 power analysis를 수행한다.

## 12. 촬영 후 평가

### 1차 외부-GT 지표

- [ ] 3D target registration error `TRE_t` mean/P50/P95/max/95% CI
- [ ] Rotation geodesic error `e_R` mean/P50/P95/max/95% CI
- [ ] ADD 또는 사전 정의한 symmetry 기반 ADD-S
- [ ] Workspace 구간별 TRE
- [ ] Calibration failure rate와 `N_reg`

### 2차 진단 지표

- [ ] 공통 held-out raw-corner reprojection RMSE/median/P95
- [ ] Cross-camera 3D disagreement
- [ ] Session 간 `bTf`, `gTc` repeatability를 각각 mm/deg로 보고
- [ ] Depth point-to-plane 또는 point-to-CAD distance
- [ ] Runtime, iteration, condition number, 필요한 views

### 통계

- [ ] Session 단위 paired comparison
- [ ] Paired bootstrap 95% CI
- [ ] 여러 baseline 비교에 Holm 보정
- [ ] A4의 paired TRE 차이 CI 상한이 baseline 각각에 대해 0보다 작은지 확인
- [ ] Rotation superiority 또는 사전 non-inferiority margin 통과
- [ ] P95와 failure rate가 악화되지 않는지 확인

## 최종 촬영 순서 요약

1. 외부 GT와 판단 기준 확정
2. Cube/board/grasp 물리 실측
3. 기존 데이터 face-roll/depth/block-gate dry-run
4. Intrinsic 40 train + 10 validation views/camera
5. 인증 길이·depth·외부 계측으로 robot scale 확정
6. Session 01 설치 및 camera별 품질 threshold 고정
7. A_placement 15 placements × 8–12 eih views
8. B_eyetohand gripped cube 30–40 poses
9. Blind external-GT test 30 poses
10. 현장 audit 통과 후에만 카메라 재장착
11. Session 02–05 반복
12. 모든 방법을 동일 데이터로 실행하고 session-paired 통계 산출

## 촬영 책임자가 사전에 결정할 것

- [ ] 외부 GT 장비/지그와 정격 정확도
- [ ] A4를 Ours-core로 고정할지 여부
- [ ] A5 6-DoF correction을 이번 실험 범위에 포함할지 여부
- [ ] Rotation non-inferiority margin
- [ ] ADD 또는 ADD-S 및 cube symmetry 정의
- [ ] Camera별 ROI sharpness threshold
- [ ] Robot scale의 물리적 확정값
- [ ] Pilot 이후 필요한 최종 session 수

외부 GT 없이 촬영하면 내부 정합과 FK-proxy 비교까지만 가능하다. “Ours가 3D 공간 정합에서 가장
우수하다”는 결론이 목표라면 blind external-GT test는 선택사항이 아니라 필수다.
