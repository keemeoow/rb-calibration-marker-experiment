# Calibration 최종 비교실험표·평가지표 단일 기준

상태: External GT 추가 전 최종 프로토콜
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
2. A0/B3의 board-only calibration은 고정된 workspace board가 아니라,
   **gripper에 부착한 board**를 cube 촬영 pose 다양성과 같은 수준으로 움직여 촬영한다.
3. 따라서 board-only 방법에서도 `T_base_gripper`와 `T_gripper_board`를 통해
   board pose에 FK 정보를 연결할 수 있다. 다만 9개 행을 유지하려면 board-only FK-fixed,
   board-only corrected-FK를 별도 행으로 추가하지 않고, FK 처리 축은 A3/A4/A5/B1/B2의
   target-pose 처리 방식으로만 비교한다.
4. 모든 heldout, cross-view, external-GT 최종 평가는 **cube target만** 사용한다.
   Board heldout RMSE는 최종 표에서 제거한다.
5. 내부 pixel 지표와 External GT가 충돌하면 External GT를 최종 판정 기준으로 둔다.

## 2. 최종 비교실험표 구성

아래 표가 최종 Table 1 구조다. 모든 row는 같은 camera intrinsics `K/D`, 같은 raw
corner detector, 같은 split, 같은 solver 설정, 같은 External GT cube pose list를 사용한다.

| Row | Calibration train target | Pose diversity | Optimization | FK / target-pose 처리 | Cube heldout / External GT 평가 | 검증 질문 |
| --- | --- | --- | --- | --- | --- | --- |
| A0 | board-on-gripper only | cube 촬영 pose 다양성과 동일하게 board를 gripper에 부착해 촬영 | sequential frozen-stage | board pose visual-estimated. FK metadata는 기록하지만 constraint로 쓰지 않음 | cube only | board-only sequential baseline |
| A1 | board-on-gripper + cube-on-gripper | board/cube 모두 동일 pose strata | sequential frozen-stage | board/cube pose visual-estimated | cube only | sequential 구조에서 cube 관측 추가 효과 |
| A2 | board-on-gripper + cube-on-gripper | A1과 동일 | unified joint optimization | board/cube pose visual-estimated | cube only | visual-only unified feedback 효과 |
| A3 | board-on-gripper + cube-on-gripper | A2와 동일 | unified joint optimization | gripper-mounted target pose를 raw FK로 hard fixed | cube only | raw FK hard constraint가 cube 정확도에 주는 영향 |
| A4 | board-on-gripper + cube-on-gripper | A2와 동일 | unified joint optimization | gripper-mounted target pose는 free, corrected-FK를 soft factor로 사용 | cube only | corrected-FK soft factor 효과 |
| A5 | board-on-gripper + cube-on-gripper | A2와 동일 | unified joint optimization | preregistered vision-aligned/corrected FK pose를 hard fixed | cube only | aligned FK hard fixed 효과. 사전등록 없으면 사후 진단 |
| B1 | board-on-gripper + cube-on-gripper | A4와 동일 | sequential frozen-stage | A4와 같은 corrected-FK soft factor | cube only | soft-FK 조건에서 sequential vs unified |
| B2 | cube-on-gripper only | cube pose strata 동일 | unified joint optimization | cube pose는 free, corrected-FK soft factor 사용 | cube only | soft-FK 조건에서 board residual 제거 효과 |
| B3 | board-on-gripper only | A0와 동일 | unified joint optimization | board pose visual-estimated. FK metadata는 기록하지만 constraint로 쓰지 않음 | cube only | board-only 조건에서 sequential vs unified, 그리고 A2 대비 cube residual 효과 |

### A0/B3에서 board FK를 어떻게 해석할지

board를 gripper에 붙이면 `T_base_gripper(event)`가 board 위치에 대한 정보를 갖는다.
따라서 board-only에서도 raw-FK-fixed나 corrected-FK가 물리적으로 가능해진다.

하지만 최종 표를 `A0~A5/B1~B3` 하나로 유지하려면 다음처럼 고정한다.

- A0/B3는 board-only **visual-estimated baseline**으로 둔다.
- board FK metadata는 기록한다.
- board-only FK-fixed 또는 board-only corrected-FK를 별도 방법으로 주장하려면 새로운 row가
  필요하므로, 이번 최종 단일 Table 1에는 넣지 않는다.
- A3/A4/A5/B1/B2의 FK 처리 설명은 더 이상 cube 전용이 아니라
  **gripper-mounted target pose에 대한 FK 처리 방식**으로 일반화한다.

## 3. 최종 직접 비교 구조

| 비교 | 고정되는 조건 | 달라지는 조건 | 최종 주 평가 | 해석 |
| --- | --- | --- | --- | --- |
| A0 -> B3 | board-only, board-on-gripper pose diversity, cube heldout | sequential vs unified | External cube GT, heldout cube RMSE | board-only에서 통합 구조 효과 |
| A0 -> A1 | sequential, board-on-gripper baseline | cube train 관측 추가 | External cube GT, heldout cube RMSE | cube 관측 추가가 최종 cube 평가에 주는 영향 |
| A1 -> A2 | board+cube, visual-estimated pose | sequential vs unified | External cube GT, heldout cube RMSE | unified visual feedback 효과 |
| B3 -> A2 | unified, board baseline 포함 | cube train residual 유무 | External cube GT, heldout cube RMSE | cube residual이 최종 cube 평가에 주는 영향 |
| A2 -> A3 | board+cube, unified | visual-estimated pose vs raw-FK hard fixed | External cube GT, heldout cube RMSE | raw FK를 hard GT처럼 쓰는 것이 좋은지 확인 |
| A2 -> A4 | board+cube, unified | corrected-FK soft factor 추가 | External cube GT, heldout cube RMSE | soft FK prior가 실제 cube 정확도에 주는 영향 |
| B1 -> A4 | board+cube, corrected-FK soft factor | sequential vs unified | External cube GT, heldout cube RMSE | FK 조건에서도 unified가 필요한지 확인 |
| B2 -> A4 | cube, corrected-FK soft factor, unified | board residual 유무 | External cube GT, heldout cube RMSE | board residual이 cube calibration에 도움 되는지 확인 |
| A3/A4 -> A5 | board+cube, unified, FK 정보 사용 | raw/soft/hard aligned FK 처리 | External cube GT, heldout cube RMSE | A5가 사전등록된 방법인지, 아니면 진단인지 분리 |

## 4. 최종 평가지표 구성

최종 보고서는 아래 지표 묶음 하나만 사용한다. `Board heldout`은 제거하고,
모든 heldout 성능 표기는 cube 기준으로 통일한다.

| 지표 | 계산 | 공정성 | 해석 한계 | 최종 사용 |
| --- | --- | --- | --- | --- |
| External cube TRE / rotation / P95 / failure | GT 공개 전 blind prediction을 저장한 뒤, 독립 External GT cube pose와 비교 | 모든 row가 같은 cube pose list, 같은 GT, 같은 tolerance를 사용 | GT 측정계 uncertainty floor보다 작은 차이는 주장 금지 | **최종 주 지표** |
| ALL Cube RMSE px | train+heldout 전체 cube evaluation data에 frozen calibration을 적용해 cube corner 재투영 | 모든 row에 같은 cube image/corner/GT pose list 적용 | train과 heldout을 섞으므로 일반화 지표가 아님 | 전체 fit sanity check |
| Train RMSE px | 각 row의 train corner 재투영 | 동일 solver/loss/split 사용 | 학습 적합도일 뿐 방법 순위 지표가 아님 | 수렴 진단 |
| Heldout Cube RMSE px | 미사용 cube event corner에 frozen calibration과 사전 정의된 cube pose source를 적용해 재투영 | test-time calibration refit 없음, heldout target은 항상 cube | 같은 pose list의 image-space 일반화이며 새 task 성공을 직접 보장하지 않음 | 내부 보조 지표 |
| Cross-view pixel transfer RMSE px | 한 카메라의 cube PnP pose를 다른 카메라 영상으로 전달해 observed cube corner와 비교 | 양방향, 동일 pair mask, 동일 event, 결과 기반 pair 제거 없음 | fitted camera pose에 의존하는 내부 일관성 | 카메라 간 pixel 일관성 |
| Cam-common Obj-Cam consistency mm/deg | 두 카메라가 같은 cube event에서 계산한 `T_base_cube` 차이를 translation mm / rotation deg로 계산 | measurement-only PnP, 같은 observation pair, fixed-camera pair와 fixed-gripper pair를 함께 집계 | 공통 systematic error는 검출하지 못함. gripper pair는 Hand-Eye와 Robot FK 오차가 섞임 | 카메라 간 3D 일관성 |

### 제거하는 지표 / 표기

- `Board heldout RMSE`: 최종 평가는 항상 cube이므로 제거한다.
- `Board/Cube heldout overall`: Board와 Cube를 섞은 pooled ranking은 제거한다.
- 그리퍼-고정카메라 closure 별도 지표: 제거한다. 대신 `Cross-view pixel transfer`와
  `Cam-common Obj-Cam consistency`에 gripper camera pair를 포함한다.
- 고정카메라-쌍 별도 순위 지표명: 사용하지 않는다. 필요한 경우 support 설명에서만
  `fixed-camera pair`, `fixed-gripper pair`로 표기한다.

## 5. Cross-view 지표에 gripper camera를 넣는 방식

카메라 pair `a,b`는 fixed-camera pair와 fixed-gripper pair를 모두 포함한다.
최종 보고서에서는 두 pair type을 하나의 cross-view/cam-common 지표로 함께 집계하고,
support 설명에만 pair 구성을 표시한다.

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
RMSE_px = bidirectional pixel-transfer RMSE over all frozen pairs
```

px 기준 산출 가능 조건:

- 같은 event에서 source camera와 destination camera가 같은 cube를 동시에 관측해야 한다.
- destination camera의 cube corner observation이 있어야 한다.
- destination camera intrinsics `K/D`가 고정되어 있어야 한다.

따라서 최종 촬영에서는 heldout cube event마다 fixed cameras와 gripper camera를 같은
event id로 동기화해 저장해야 한다. 이 조건을 만족하면 fixed-gripper pair도 px 기준
cross-view pixel transfer를 낼 수 있다.

## 6. External GT가 들어오면 최종 순위를 정하는 규칙

1. 최종 순위는 External cube TRE/RMSE, rotation error, P95, failure rate로 정한다.
2. Heldout Cube RMSE px와 cross-view consistency가 좋아도 External GT가 나쁘면
   최종 방법으로 주장하지 않는다.
3. A5는 GT 공개 전에 절차가 사전등록되어 있으면 후보 method로 비교할 수 있다.
   그렇지 않으면 사후 진단으로만 둔다.
4. A4의 corrected-FK covariance는 GT 결과를 보기 전에 고정해야 한다.
5. 모든 비교는 같은 cube heldout pose list에서 paired comparison으로 계산한다.

## 7. 구현 전 체크리스트

- `capture manifest`에 `board_on_gripper`, `cube_on_gripper`, `T_gripper_board`,
  `T_gripper_cube`, `pose_id`, `event_id`를 명시한다.
- `fk_to_board`가 더 이상 물리적으로 불가능하다는 전제를 제거한다. 단 최종 9행 표에서는
  board-only FK variant를 추가 row로 만들지 않는다.
- heldout evaluator는 `target == cube`만 선택한다.
- cross-view evaluator는 fixed-camera pair와 fixed-gripper pair를 같은 metric family에서
  계산하고 최종 보고서는 combined 값만 노출한다.
- `ALL Cube RMSE`를 train+heldout cube evaluation population으로 추가한다.
- External GT evaluator는 GT 공개 전 blind prediction hash를 저장하고, GT 공개 후
  동일 prediction 파일만 채점한다.
