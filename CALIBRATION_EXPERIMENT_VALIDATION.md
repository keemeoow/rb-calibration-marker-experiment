# Calibration 최종 비교실험표·평가지표 단일 기준

상태: External GT 추가 전 최종 프로토콜. 공통 45-event composite-target 촬영 계약 확정,
phase-aware solver 구현 전
범위: `A0~A5`, `B1~B3` 한 세트만 사용
핵심 변경: **heldout 평가는 항상 cube만 본다.** Board heldout은 최종 판정
지표에서 제거하고, board는 calibration/training 또는 ablation용 관측으로만 쓴다.

논문 기여도와 스토리라인의 최상위 기준은 [RESEARCH_STORYLINE.md](RESEARCH_STORYLINE.md)를 따른다.
Session04 자동 생성 결과는 [TABLE1_RESULTS.md](CP_result/session04/late_table1/TABLE1_RESULTS.md)에
이 최종 기준으로 재생성한다. External GT가 들어간 최종 비교표와 평가지표도 이 문서를
단일 기준으로 사용한다.

## 1. 최종 실험 원칙

1. 최종 비교행은 `A0~A5`, `B1~B3`만 사용한다. A6나 추가 board-only FK row는
   최종 표에 넣지 않는다.
2. planar board와 multi-face cube를 서로 움직이지 않는 하나의 **composite target rig**로
   고정한다. 로봇은 이 rig 전체를 grasp/place하며, 모든 행은 같은 45개 raw capture event와
   같은 pose ID를 사용한다.
3. A0/B3는 공통 영상에서 cube residual만 제외하고, B2는 board residual만 제외한다.
   A1~A5/B1은 두 residual을 모두 사용한다. 따라서 target 추가 효과에 별도 촬영 수 증가가
   섞이지 않는다.
4. 모든 train/heldout 재투영, cross-view, external-GT 최종 평가는 **cube target만**
   사용한다. Row별 marker 모집단이 달랐던 solver Train RMSE는 최종 표에서 제거한다.
5. 내부 pixel 지표와 External GT가 충돌하면 External GT를 최종 판정 기준으로 둔다.

## 2. 공통 45-event 촬영 프로토콜

`1 capture event`는 하나의 사전등록 robot pose에서 모든 연결 카메라가 동기 촬영한
한 시점을 뜻한다. 촬영 시간은 예산이나 평가지표로 사용하지 않는다. 검출에 성공한
event만 골라 수를 맞추지 않고, 계획한 45개 event 전체를 공통 모집단으로 유지한다.

| Phase | 물리 상태 | 횟수 | 필수 관측 | 목적 |
| --- | --- | ---: | --- | --- |
| P1 Moving Rig | robot이 composite target을 계속 강체 파지한 채 이동 | 15 events | fixed cameras 동기 촬영 | `T_flange_rig`와 fixed-camera 관계 추정, xyz/회전 다양성 확보 |
| P2 Pick-and-Place | composite target을 10번 배치하고 배치마다 release 후 정확히 2개 viewpoint 촬영 | 10 placements, 20 events | fixed + gripper cameras 동기 촬영, release FK는 metadata로 기록 | 통합 calibration과 placement prediction 오차 측정 |
| P3 Stationary Rig | target을 놓은 채 robot/gripper camera만 이동 | 10 events | fixed + gripper cameras 동기 촬영 | eye-in-hand calibration과 cross-view 연결 |
| Total | calibration train capture | **45 events** | 동일 event/pose manifest | 모든 row의 동일 촬영 예산 |

### 2.1 Pose와 성공/실패 규칙

- P1의 15 pose는 x/y/z 위치와 roll/pitch/yaw를 함께 바꾸며, yaw 한 축 회전만 반복하지 않는다.
- P2는 `2~3장`처럼 가변적으로 두지 않고 placement마다 release 후 정확히 2 event로
  고정한다. Release pose 기록 자체는 영상 capture event에 포함하지 않는다.
- P3의 10 pose는 target rig를 움직이지 않고 camera translation, 높이, 두 축 이상의 회전을 포함한다.
- marker 미검출이나 PnP 실패 event는 다른 event로 교체하지 않는다. 해당 방법의 유효 관측 수와
  failure rate에 그대로 반영한다.
- 평가용 cube/External-GT capture는 이 45개 train event와 분리하며 모든 row가 동일하게 사용한다.

### 2.2 Phase-aware target pose 모델

rig에 대한 board와 cube의 강체 변환 `T_rig_board`, `T_rig_cube`는 전체 촬영에서 변하지
않아야 한다. P1과 P2의 grasp 구간은 다음 kinematic model을 사용한다.

```text
T_base_rig(event)   = T_base_flange(event) * T_flange_rig
T_base_board(event) = T_base_rig(event) * T_rig_board
T_base_cube(event)  = T_base_rig(event) * T_rig_cube
```

P2의 release 이후와 P3에서는 placement별 `T_base_rig(set)`을 공유하는 stationary-target
모델을 사용한다. 기존처럼 하나의 전역 `T_base_board`를 모든 event에 적용하면 움직이는
P1/P2 board 관측을 설명할 수 없으므로 최종 촬영 전에 solver를 수정해야 한다.

P2의 release FK와 P1에서 구한 `T_flange_rig`로 예측한 pose를 놓인 뒤 vision pose와
비교하면 robot FK뿐 아니라 grasp 반복성, release slip, 접촉 및 settling 오차까지 포함된다.
따라서 이 값을 학습하면 `pure FK correction`이 아니라 **effective placement correction**으로
표기한다. 순수 FK correction은 target이 아직 강체 파지된 train 관측만 사용한다.

## 3. 최종 비교실험표 구성

아래 표가 최종 Table 1 구조다. 모든 row는 같은 camera intrinsics `K/D`, 같은 raw
corner detector, 같은 split, 같은 solver 설정, 같은 External GT cube pose list를 사용한다.

| Row | Calibration train residual | Capture population | Optimization | FK / target-pose 처리 | Cube heldout / External GT 평가 | 검증 질문 |
| --- | --- | --- | --- | --- | --- | --- |
| A0 | board only | 공통 45 events | sequential frozen-stage | phase-aware rig pose visual-estimated, raw FK는 robot motion chain에만 사용 | cube only | 동일 촬영 예산의 board-only sequential baseline |
| A1 | board + cube | 공통 45 events | sequential frozen-stage | phase-aware rig pose visual-estimated | cube only | 추가 촬영 없이 cube residual을 더한 효과 |
| A2 | board + cube | 공통 45 events | unified joint optimization | phase-aware rig pose visual-estimated | cube only | visual-only unified feedback 효과 |
| A3 | board + cube | 공통 45 events | unified joint optimization | grasp 구간 target pose를 raw FK로 hard fixed | cube only | raw FK hard constraint가 cube 정확도에 주는 영향 |
| A4 | board + cube | 공통 45 events | unified joint optimization | target pose는 free, train-only corrected-FK를 soft factor로 사용 | cube only | corrected-FK soft factor 효과 |
| A5 | board + cube | 공통 45 events | unified joint optimization | preregistered vision-aligned/corrected FK pose를 hard fixed | cube only | aligned FK hard fixed 효과. 사전등록 없으면 사후 진단 |
| B1 | board + cube | 공통 45 events | sequential frozen-stage | A4와 같은 corrected-FK soft factor | cube only | soft-FK 조건에서 sequential vs unified |
| B2 | cube only | 공통 45 events | unified joint optimization | cube/rig pose는 free, corrected-FK soft factor 사용 | cube only | 동일 영상에서 board residual 제거 효과 |
| B3 | board only | 공통 45 events | unified joint optimization | phase-aware rig pose visual-estimated, raw FK는 robot motion chain에만 사용 | cube only | 동일 영상에서 cube residual 추가 효과의 기준선 |

### A0/B3에서 board-only를 어떻게 해석할지

모든 event에 board와 cube가 함께 촬영되지만 A0/B3의 objective와 초기화에는 cube corner를
넣지 않는다. Cube 영상은 공통 cube 평가 모집단에서 frozen calibration을 채점할 때만 쓴다.
따라서 A0/B3는 촬영 수와 pose가 같은 순수 board-residual baseline이다.

하지만 최종 표를 `A0~A5/B1~B3` 하나로 유지하려면 다음처럼 고정한다.

- A0/B3는 phase-aware rig pose를 푸는 board-only **visual-estimated baseline**으로 둔다.
- grasp/release FK와 rig mount metadata는 모든 행에서 동일하게 기록한다.
- board-only FK-fixed 또는 board-only corrected-FK를 별도 방법으로 주장하려면 새로운 row가
  필요하므로, 이번 최종 단일 Table 1에는 넣지 않는다.
- A3/A4/A5/B1/B2의 FK 처리 설명은 더 이상 cube 전용이 아니라
  **gripper-mounted target pose에 대한 FK 처리 방식**으로 일반화한다.

## 4. 최종 직접 비교 구조

| 비교 | 고정되는 조건 | 달라지는 조건 | 최종 주 평가 | 해석 |
| --- | --- | --- | --- | --- |
| A0 -> B3 | 공통 45 events, board residual only | sequential vs unified | External cube GT, heldout cube RMSE | 단일 target에서 두 구조가 사실상 같아야 하는 negative control |
| A0 -> A1 | 공통 45 events, sequential, 같은 board residual | 같은 영상의 cube residual 추가 | External cube GT, heldout cube RMSE | 촬영 수 증가 없는 cube 관측 추가 효과 |
| A1 -> A2 | 공통 45 events, board+cube, visual-estimated pose | sequential vs unified | External cube GT, heldout cube RMSE | unified visual feedback 효과 |
| B3 -> A2 | 공통 45 events, unified, 같은 board residual | 같은 영상의 cube residual 추가 | External cube GT, heldout cube RMSE | cube residual이 최종 cube 평가에 주는 영향 |
| A2 -> A3 | board+cube, unified | visual-estimated pose vs raw-FK hard fixed | External cube GT, heldout cube RMSE | raw FK를 hard GT처럼 쓰는 것이 좋은지 확인 |
| A2 -> A4 | board+cube, unified | corrected-FK soft factor 추가 | External cube GT, heldout cube RMSE | soft FK prior가 실제 cube 정확도에 주는 영향 |
| B1 -> A4 | board+cube, corrected-FK soft factor | sequential vs unified | External cube GT, heldout cube RMSE | FK 조건에서도 unified가 필요한지 확인 |
| B2 -> A4 | 공통 45 events, cube residual, corrected-FK soft factor, unified | 같은 영상의 board residual 추가 | External cube GT, heldout cube RMSE | board residual이 cube calibration에 도움 되는지 확인 |
| A3/A4 -> A5 | board+cube, unified, FK 정보 사용 | raw/soft/hard aligned FK 처리 | External cube GT, heldout cube RMSE | A5가 사전등록된 방법인지, 아니면 진단인지 분리 |

물리적으로 함께 촬영된 cube가 A0/B3에 정보 누출을 만들지 않도록 cube detection은 해당
행의 초기화, outlier 선택, residual, hyperparameter 선택에서 모두 차단하고 평가 단계에서만
연다. 동일 event에서 cube가 더 많은 corner와 camera co-visibility를 제공하는 효과는 제안한
target system의 장점으로 포함하며, 동일 corner 수 subsampling은 보조 민감도 분석으로 둔다.

## 5. 최종 평가지표 구성

최종 보고서는 아래 지표 묶음 하나만 사용한다. `Board heldout`은 제거하고,
모든 heldout 성능 표기는 cube 기준으로 통일한다.

| 지표 | 계산 | 공정성 | 해석 한계 | 최종 사용 |
| --- | --- | --- | --- | --- |
| External cube TRE / rotation / P95 / failure | GT 공개 전 blind prediction을 저장한 뒤, 독립 External GT cube pose와 비교 | 모든 row가 같은 cube pose list, 같은 GT, 같은 tolerance를 사용 | GT 측정계 uncertainty floor보다 작은 차이는 주장 금지 | **최종 주 지표** |
| ALL Cube RMSE px | train cube로 set별 evaluation cube pose를 맞춘 뒤 train 724 + heldout 236 cube corner 재투영 | 모든 row에서 camera/hand-eye frozen, 같은 960 corners | train이 75.4%이고 heldout과 섞이므로 일반화 지표가 아님 | 전체 fit sanity check |
| Train Cube RMSE px | frozen calibration에서 train cube로 set별 evaluation pose를 맞춘 뒤 같은 train cube 재투영 | 모든 row가 동일한 724 train cube corners 사용 | pose를 맞춘 관측을 다시 채점하는 in-sample fit이므로 순위 지표가 아님 | train-split fit 진단 |
| Heldout Cube RMSE px | train-only cube pose source와 frozen calibration으로 미사용 cube event corner 재투영 | test-time calibration refit 없음, 모든 row가 동일한 236 heldout cube corners 사용 | 같은 set의 다른 event에 대한 corner-pooled image-space 평가이며 새 위치·물리 정확도가 아님 | 내부 보조 지표 |
| Cross-view pixel transfer RMSE px | 한 카메라의 cube PnP pose를 다른 카메라 영상으로 전달해 observed cube corner와 비교 | 동일한 36 pairs, 72 directions, 904 destination-corners를 직접 pooling; 결과 기반 pair 제거 없음 | 27 fixed-gripper pair 중 18개는 train fixed-anchor를 쓰고 Hand-Eye/FK가 섞이는 내부 closure | 카메라 간 pixel 일관성 |
| Cam-common Obj-Cam consistency mm/deg | 같은 frozen cube pair의 두 경로가 계산한 `T_base_cube` 차이를 translation mm / rotation deg로 계산 | Cross-view와 같은 36 pair를 직접 pooling | Cross-view px와 같은 discrepancy의 단위 변환적 표현이라 독립 증거가 아니며 공통 계통오차를 검출하지 못함 | 카메라 간 3D 일관성 |

### 제거하는 지표 / 표기

- `Board heldout RMSE`: 최종 평가는 항상 cube이므로 제거한다.
- `Board/Cube heldout overall`: Board와 Cube를 섞은 pooled ranking은 제거한다.
- 그리퍼-고정카메라 closure 별도 지표: 제거한다. 대신 `Cross-view pixel transfer`와
  `Cam-common Obj-Cam consistency`에 gripper camera pair를 포함한다.
- 고정카메라-쌍 별도 순위 지표명: 사용하지 않는다. 필요한 경우 support 설명에서만
  `fixed-camera pair`, `fixed-gripper pair`로 표기한다.

### A0/B3의 cube RMSE 계산 방식

A0/B3는 calibration 학습 단계에서는 cube corner를 쓰지 않는 board-only 방법이다.
하지만 원본 capture에 cube 이미지가 있으므로, 최종 camera/hand-eye transform을
frozen한 뒤 **train cube 관측으로 set별 evaluation cube pose만** 맞추면
`Train Cube RMSE px`, `ALL Cube RMSE px`, `Heldout Cube RMSE px`를 계산할 수 있다.

이때 train cube는 평가용 nuisance pose를 만들기 위한 입력일 뿐이며,
A0/B3의 camera extrinsic이나 hand-eye transform을 다시 최적화하지 않는다.
Heldout cube event는 pose fit에도 calibration에도 쓰지 않고, 점수 계산에만 사용한다.

## 6. Cross-view 지표에 gripper camera를 넣는 방식

카메라 pair `a,b`는 fixed-camera pair와 fixed-gripper pair를 모두 포함한다.
최종 보고서에서는 두 pair type의 **원시 pair/direction 오차를 직접 pooling**하고,
support 설명에 pair와 destination-corner 수를 표시한다. 이미 서로 다른 방식으로 집계된
scope별 RMSE를 다시 평균하지 않는다.

```text
T_base_cam(k, event) =
    T_base_Ck                            if camera k is fixed
    T_base_gripper(event) * T_gripper_Ck if camera k is gripper-mounted

T_base_cube_from_a = T_base_cam(a, event) * T_cam_a_cube(PnP)
T_base_cube_from_b = T_base_cam(b, event) * T_cam_b_cube(PnP)

Obj-Cam translation error mm = || t_a - t_b || * 1000
Obj-Cam rotation error deg   = angle(R_a^-1 R_b)
```

Pixel transfer도 gripper camera에 대해 계산 가능하다.

```text
T_cam_b_cube_from_a = inv(T_base_cam(b, event)) * T_base_cube_from_a
project cube corners into image b
pixel residual = projected corner - observed corner in image b
RMSE_px = component-wise pooled RMSE over all frozen pair directions
```

현재 Session04 support는 9 fixed-fixed + 27 fixed-gripper pair다. Fixed-gripper 중
18 pair는 같은 set의 train fixed-anchor와 heldout gripper event를 연결하고, 9 pair만
양쪽이 heldout이다. 모든 방법에 같은 mask를 적용하므로 방법 간 계산은 공정하지만,
이를 순수 heldout이나 FK-free metric으로 부르면 안 된다.

px 기준 산출 가능 조건:

- fixed-fixed는 같은 event, fixed-gripper는 cube가 고정된 같은 set에서 source와
  destination 관측이 연결되어야 한다.
- destination camera의 cube corner observation이 있어야 한다.
- destination camera intrinsics `K/D`가 고정되어 있어야 한다.

다음 촬영에서는 heldout cube event마다 fixed cameras와 gripper camera를 같은 event id로
동기화하면 train fixed-anchor 의존성을 없앨 수 있다. 현재 Session04 값은
**mixed train-anchor/heldout internal closure**로 유지한다.

## 7. External GT가 들어오면 최종 순위를 정하는 규칙

1. 최종 순위는 External cube TRE/RMSE, rotation error, P95, failure rate로 정한다.
2. Heldout Cube RMSE px와 cross-view consistency가 좋아도 External GT가 나쁘면
   최종 방법으로 주장하지 않는다.
3. A5는 GT 공개 전에 절차가 사전등록되어 있으면 후보 method로 비교할 수 있다.
   그렇지 않으면 사후 진단으로만 둔다.
4. A4의 corrected-FK covariance는 GT 결과를 보기 전에 고정해야 한다.
5. 모든 비교는 같은 cube heldout pose list에서 paired comparison으로 계산한다.
6. 예측 누락을 방법별 complete-case 삭제하지 않고 failure로 계산한다.

## 8. 구현 전 체크리스트

- `capture manifest`에 `phase`, `target_rig_id`, `planned_pose_id`, `event_id`,
  `placement_id`, `grasp_id`, `release_event_id`, 카메라별 capture/detection 성공을 명시한다.
- `T_rig_board`, `T_rig_cube`, grasp 구간의 `T_flange_rig`와 placement별
  `T_base_rig(set)`을 구분해 저장한다.
- 모든 row의 calibration input `event_id`가 같은 45개인지 검사하고, A0/B3/B2의 marker
  masking이 초기화와 outlier 선택 전부터 적용되는지 자동 검증한다.
- 하나의 정적 `T_base_board` 전제를 phase-aware moving/stationary rig model로 교체한다.
  단, 기존 Session04 loader는 legacy mode로 보존한다.
- heldout evaluator는 `target == cube`만 선택한다.
- cross-view evaluator는 fixed-camera pair와 fixed-gripper pair를 같은 metric family에서
  계산하고 최종 보고서는 combined 값만 노출한다.
- `ALL Cube RMSE`를 train+heldout cube evaluation population으로 추가한다.
- `Train Cube RMSE`는 모든 row에서 동일한 train cube evaluation population으로 계산한다.
- External GT evaluator는 GT 공개 전 blind prediction hash를 저장하고, GT 공개 후
  동일 prediction 파일만 채점한다.
