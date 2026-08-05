# 시뮬 vs 실측(real) 구성 대조 체크리스트

real 데이터(`data/session`, 13 sets·232 captures)와 실측 config(`config.py` 정본,
`intrinsics/charuco_intrinsics_report.json`, `device_map.json`, `calib_out/*`)를 시뮬과
하나씩 대조한 결과. **정본 = `config.py`** (인접 `rb-ArucoCube_Robot_multi_calibration`는
옛 30mm ArUco 큐브라 무관).

## ✅ 일치 (조치 불필요)

| 항목 | real | 시뮬 |
|---|---|---|
| 큐브 기하 (side 59mm, top 25/side 51mm, id→면, 마커중심, roll {2:90,3:180,4:270}) | config.py 정본 | targets.py **정확히 일치** |
| 보드 (11×7, square 25mm, 내부코너 10×6=60, 평면 1장) | config.py | targets.py 일치 |
| 카메라 위상 (고정 3 + 그리퍼 1 = 4대) | meta.json n_fixed=3, gripper_idx=2 | scene.py 3+1 일치 |
| 해상도 640×480 | 모든 cam*.npz | project.py 일치 |
| 왜곡 모델 k3=0 (brown, k1k2+p1p2) | dist_model | DEFAULT_DIST 일치 |
| 큐브 다면 (카메라당 ~2마커/2면) | 검출 402회 2마커 | 입사각 가시성 일치 |
| gripped 모드 존재 (로봇이 큐브 들고 고정캠 관측) | cube_gripped=true ×89 | n_gripped 모델 일치 |
| 그리퍼→큐브 X를 데이터로 추정 | calibrateRobotWorldHandEye | solve_unified 의 X 미지수 일치 |
| 코너 노이즈 σ | 실측 0.15~0.19px | 0.2 (약간 보수적) 사실상 일치 |
| 계통 intrinsic <1% | 실측 | 0.5% 일치 |

## ❌ 불일치 (조치 필요) — 우선순위순

### 🔴 1. FK 오차 모델 — 가장 큰 불일치 (결론에 직접 영향)
- **real**: FK 큐브 prior 가 **systematic**. `T_set_cube_center_to_object` = **~180° flip + 37mm Z 오프셋**,
  prior 위치오차 **median 6.6mm**, 그리퍼 pose 잔차 ~11.5mm, 일관성 15~17mm, **5.3% gross flip(~138mm)**.
  (`set_cube_center_prior.json`, `gripper_base_pose_model.json`, `verification_metrics.json`,
  `CP_EXPERIMENTS_README.md:155,159,170`)
- **시뮬**: realistic 프리셋 FK≈0, FK sweep 은 **random 제로평균** 노이즈(`scene.py:127`).
- **영향**: **FK 보정(Ours의 핵심)은 systematic 오차를 제거하는 게 목적인데, 시뮬은 그걸 안 넣음.**
  → 앞선 "FK 보정 무의미" 재판정은 **불공정한 FK 모델** 탓. real 처럼 systematic FK 를 넣어야 공정.
- **조치**: FK 오차를 **systematic**(위치의존 smooth + 상수 오프셋, 실측 ~6.6mm)으로 모델링.
  180° flip 은 프레임 규약(gauge)이라 별도. realistic 프리셋의 FK 도 ≈0 이 아니라 실측값으로.

### 🟠 2. 카메라 intrinsic — 개별 vs 평균
- **real**: 카메라 4대 각자 다른 K (cy 최대 ~22px, cx ~15px, fx ~10px 차이).
  fixed=cam0/1/3, gripper=cam2. (`charuco_intrinsics_report.json`)
- **시뮬**: 4대에 **평균 K 하나**(DEFAULT_K)만 사용. per-camera 개별성 없음.
- **조치**: **개별 실측 K 4개**를 카메라별로 사용 (아래 값). intrinsic_err 랜덤섭동 대신 실측 개별값.

### ✅ 3. 카메라 하향각(downtilt) — 27° 유지 (해결: 물리 셋업 재조정됨)
- **real 캘리브 파일**: 하향각 ~10~15° (T_base_C0 11.5°, C1 15.5°, C3 9.9°; npz 9~12°).
- **사용자 확인**: **물리 리그가 캘리브 이후 실제로 25~30°로 재조정됨**(보드를 더 잘 보려고).
  → 저장된 캘리브가 stale 이고 **시뮬 27° 가 현재 물리 셋업에 맞음**.
- **판정**: **조치 불필요. 시뮬 27° 유지.** (저장 npz 하향각은 무시하고 27° 강제하는 현 로직이 옳음.)

### 🟡 4. 프로토콜 카운트 — eih/set, gripped 수
- **real**: eye-in-hand **11 shots/set**(143 total), gripped **89**(set당 2~9, 불균일).
- **시뮬**: N_EVENTS=**13**/set, N_GRIPPED=**130**.
- **조치**: N_EVENTS=11, N_GRIPPED=89 로. (랭킹엔 영향 적고 절대값 현실화.)

### 🟡 5. 고정카메라 set당 관측 수 — 1 vs 11
- **real**: 각 set 에서 11 event 마다 4대 동시 촬영 → 고정캠이 정지 큐브를 **11번**(중복, 노이즈 평균).
- **시뮬**: 고정캠은 set당 큐브 **1번**만 관측(`obs_fix_cube[(ci,s)]`).
- **영향**: 시뮬 고정캠이 real 보다 노이즈 큼(중복평균 없음). 랭킹 무관, 절대값만.
- **조치**: (선택) event 마다 고정캠도 관측하게 확장. 랭킹 무관이라 후순위.

### 🟡 6. 오검출 — 코너 2% vs pose-flip 5.3% 꼬리
- **real**: 코너 ~2%@1~2px + **pose-level 180° flip 5.3%@~138mm**(평면 PnP 뒤집힘, cam0·2마커면·set11~12 집중).
- **시뮬**: 코너 2%@2px 만. pose-flip 꼬리 없음(단 robust Ransac 이 코너 이상치는 처리).
- **조치**: (선택) pose-flip 이상치(면 뒤집힘) 주입해 robust 처리의 한계 시험.

### 🟢 7. (선택) 마커 중심 nominal vs self-cal
- **real**: 자가보정 marker center 가 nominal 과 최대 ~1.5mm 차 (`marker_poses.json`).
- **시뮬**: nominal 사용. 조치: (선택) self-cal 값으로 교체 — 영향 미미.

## 실측 개별 intrinsic (조치 2용)

| cam | 역할 | fx | fy | cx | cy | k1 | k2 | p1 | p2 | rms |
|---|---|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| 0 | fixed | 592.785 | 595.882 | 326.205 | 258.082 | 0.07230 | -0.18836 | 0.00746 | 0.00613 | 0.321 |
| 1 | fixed | 595.188 | 597.633 | 322.448 | 249.406 | 0.09137 | -0.22003 | -0.00185 | 0.00808 | 0.235 |
| 2 | **gripper** | 602.506 | 603.523 | 337.486 | 236.303 | 0.08259 | -0.19207 | -0.00278 | 0.00881 | 0.259 |
| 3 | fixed | 599.711 | 601.883 | 323.207 | 239.089 | 0.12098 | -0.29069 | -0.00310 | 0.00036 | 0.244 |

(k3=0 전부. 시뮬 DEFAULT_K 는 이 4대의 평균이 맞음 — 개별성만 소실.)
