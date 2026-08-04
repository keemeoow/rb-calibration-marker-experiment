# 비교실험 표 설계 및 사실 검증 기록

표기:

- `—`: 현재 데이터/코드로 측정되지 않음
- `†`: 요청 행과 완전히 같은 조건이 아닌 부분 실험값
- `‡`: 외부 GT 오차가 아닌 내부 일관성 지표
- `*`: 본 데이터가 아닌 인용 논문 보고값
- `§`: 수정한 production 코드를 기존 세션에 적용한 격리 dry-run 값. 기존 `data/session/calib_out`은 덮어쓰지 않음
- `¶`: **다른 split에서 측정한 값**. 위치(set) 단위 leave-one-out 13 fold(D1)이며, 같은 행의 다른 열(이벤트 단위 split)과 한 줄로 이어 읽지 않는다
- `≡`: 위 행과 **수치가 동일**함. 잔차 보정은 `T_base_Ci`·`T_gripper_cam`을 바꾸지 않으므로 pose에서 파생되는 지표는 정의상 변하지 않는다
- `‖`: **`RB_ROBOT_POS_SCALE=1.0229`로 측정한 값.** 표의 나머지 셀은 전부 **k=1.0**(보정 없음)이다. 즉 이 셀들은 같은 표의 다른 셀과 **로봇 스케일 상수 자체가 다르므로 직접 빼면 안 된다.** 아래 "로봇 스케일 기준 불일치" 참조

## 남아 있는 사실 검증·보류 항목

| 검증 항목 | 상태 | 필요한 작업 |
| --- | --- | --- |
| 실제 `e_task_pose` 외부 GT | 대기 | 현재 FK-proxy translation만 있으므로 독립 외부 측정계로 translation/rotation GT를 수집해야 함 |
| **로봇 스케일 기준 불일치** | **미해결** | Table 1이 두 개의 `RB_ROBOT_POS_SCALE` 값을 섞고 있다. 아래 절 참조 |

### 로봇 스케일 기준 불일치 (2026-08-04 확인)

Table 1의 A0–B3와 `e_reproj` 열은 [`ablation_multisplit/`](ablation_multisplit/)에서 나오며
**k=1.0**(보정 없음)으로 측정됐다. 반면 `‖`로 표시한 채택 구성 셀(D1/D2 계열)은 **k=1.0229**로
측정됐다. 한 표 안에 두 기준이 섞여 있다.

확인 방법과 근거:

- `ablation_multisplit/split_20260729/shared_board_free_fk_cube.json`에 저장된 raw FK 큐브 포즈는
  `load_nominal_set_cube_transforms`를 **k=1.0**으로 호출했을 때 **0.0000 mm** 일치하고,
  k=1.0229에서는 평균 12.36 mm / 최대 16.50 mm 어긋난다.
- `CP_D2_anchored_event_split.py`를 k=1.0으로 재실행하면 Table 1의 A2/A3를 **~1e-5**로 재현한다
  (`worst_sigma` 0.0). 커밋된 D2 artifact는 **1.51σ**로만 맞으며, A3 `e_cross`가
  32.75 vs 38.05 mm(**5.29 mm 차이**)다.

**D2의 `reference_agreement` 게이트는 이 불일치를 잡지 못한다.** 허용오차가 Table 1의 split
표준편차 단위(예: `e_cross` 3.5 mm)라서 5.3 mm의 스케일 이동이 `within_tolerance: true`로
통과한다. 두 artifact가 같은 스케일인지 확인할 때 이 게이트를 근거로 삼으면 안 되고, 위처럼
FK 포즈를 직접 비교해야 한다.

이 표의 `e_reproj` 열은 채택 구성 행까지 **전부 k=1.0으로 통일**했다. 그 행의 값은
[`cross_target_cube/D2_rerun_k1.0/`](cross_target_cube/D2_rerun_k1.0/)의 k=1.0 재실행에서
나왔으므로 `‖`를 붙이지 않았다. 같은 행의 `e_task_pose`(`¶`)·`e_e2e`·`e_cross`(`‖`)는 아직
k=1.0229 값이다. **한 행을 가로로 이어 읽지 말 것.**

k=1.0229 자체가 잠정값이다(카메라별 추정이 1.4~2.9%로 벌어진다). 어느 쪽으로 통일할지는
k를 물리 측정으로 확정한 뒤 전체를 한 번에 재생성하는 것이 맞다. `ablation_multisplit`은
현재 재생성 목록에 들어 있지 않으므로 함께 추가해야 한다.
| Task-level 물리 실행 | **보류 TO_DO** | 하나의 실물 task, 독립 외부 측정법, 성공·오차 정의와 실행 프로토콜을 확정한 뒤 재개 |

## 기본 정보 제공 Table

`뷰당 코너`는 타깃이 한 번 이상 검출된 저장 뷰만 집계했다. Cube 코너 수는 `검출 tag 수 × 4`다.

| Target | 확정 사양 | 뷰당 코너 e2h (mean±std) | 뷰당 코너 eih (mean±std) | 동시관측 고정 카메라 (mean) |
| --- | --- | ---: | ---: | ---: |
| ChArUco board | 11×7 squares, checker 25 mm, marker 18 mm | 5.25±0.90 (n=281) | 51.43±8.97 (n=146) | 1.03 (n=105 events) |
| AprilTag cube | cube side 59 mm; top tags 25 mm, side tags 51 mm | 7.85±1.91 (n=627) | 10.74±3.64 (n=137) | 2.70 (n=232 events) |

두 대 이상 고정 카메라 동시관측 비율은 board 2.86%, cube 89.66%다. 사양은 [`config.py`](../config.py), 검출은 [`data/session/meta.json`](../data/session/meta.json), 관측성은 [`C2_observability.csv`](C2/C2_observability.csv)에서 확인했다.

## Table 1 — Main Ablation (실제 셋업)

모든 조건은 운동학 백본 `T_base_gripper=FK(q)`를 공통으로 사용해 손목 카메라 뷰를 하나의 hand-eye 변환으로 정합한다. 이는 실험 변수가 아니므로 표의 `✓/✗` 축으로 두지 않는다. 아래 pose-source 열은 오직 **cube pose를 FK 값으로 고정했는지**만 나타낸다.

축(기여도) 정의:

| 기여도 | 축 | 값 | 의미 |
| --- | --- | --- | --- |
| C2 | **Marker** | board / cube / cube+board | 어떤 타겟을 쓰는가 |
| C1 | **U** | seq / U | eye-to-hand와 eye-in-hand를 순차로 푸는가, 한 목적함수로 푸는가 |
| C3 | FK | ✗ / ✓ | 타겟 포즈를 FK로 확정하는가, 미지수로 추정하는가 |


| # | Marker | Unified | FK→cube | 의미 | N_reg (base에 등록된 카메라 수) | e_task_pose^{FK-proxy} (mm) (held-out **위치** 큐브중점의 base 좌표 예측 오차) | e_e2e (mm/°) (eye-to-hand 경로 vs eye-in-hand 경로 큐브 위치 일치) | e_cross (mm) e_cross (mm)(고정 카메라들끼리의 일치도) | e_reproj^{cube\|FK-fixed} (px) (공통 큐브 코너, cam0·cam1 pooled) | ↳ cam 1 (px) (판별 카메라) | 상태 |
| --- | --- | :---: | --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| A0 | board | seq | — | baseline | 2 | — | — | — | 10.9130±1.0401 | 2.5594±0.0868 | 5 splits×5 init, 25/25 수렴 |
| A1 | cube+board | seq | vision-estimated | +cube | 3 | — | 16.3323±1.2684 / 7.7466±0.5970 | 39.5978±3.3399 | 10.5768±1.0394 | 1.3027±0.1469 | 5 splits×5 init, 25/25 수렴 |
| A2 | cube+board | U | vision-estimated | +unified | 3 | 3.451¶ | 16.1955±1.2976 / 7.7423±0.5937 | 38.9338±3.4075 | 10.5170±1.0399 | 1.2833±0.1396 | 5 splits×5 init, 25/25 수렴 |
| ↳ A2 + 잔차보정 | cube+board | U | vision-estimated | +예측단계 Ridge`[1,x,y]` | ≡ | 1.847¶ | ≡ | ≡ | ≡ | ≡ | 위치 LOO 13/13 fold 수렴 |
| **↳ A2@λ=3 + 잔차보정 ★** | **cube+board** | **U** | **vision-estimated + soft FK anchor (λ=3 px/mm)** | **Ours (최종 채택)** | 3 | **1.586¶** | **15.1168±1.3115 / 7.7342±0.5882‖** | **35.0872±3.5329‖** | **10.4968±1.0487** | **0.9316±0.0644** | 위치 LOO 13/13 + 이벤트 5 splits×3 init, 15/15 수렴 |
| A3 | cube+board | U | FK-fixed | 7행 full row | 3 | 2.988¶ | 16.1881±1.2856 / 7.7087±0.5853 | 38.0480±3.5117 | 10.5042±1.0581 | **0.7713±0.0330** | 5 splits×5 init, 25/25 수렴 |
| ↳ A3 + 잔차보정 | cube+board | U | FK-fixed | +예측단계 Ridge`[1,x,y]` | ≡ | 2.355¶ | ≡ | ≡ | ≡ | ≡ | 위치 LOO 13/13 fold 수렴 |
| B1 | cube+board | seq | FK-fixed | −Unified | 3 | — | 16.1809±1.2931 / 7.7020±0.5852 | 38.1160±3.5083 | 10.5238±1.0586 | 0.7827±0.0361 | 5 splits×5 init, 25/25 수렴 |
| B2 | cube only | U | FK-fixed | −board | 3 | — | 15.6887±1.2425 / 7.7399±0.5867 | 37.8594±3.5187 | 10.4751±1.0550 | **0.7691±0.0308** | 5 splits×5 init, 25/25 수렴 |
| B3 | board only | U | — | −cube | 2 | — | — | — | 10.9129±1.0401 | 2.5584±0.0869 | 5 splits×5 init, 25/25 수렴 |

**★ 최종 채택 구성(Ours) = `A2@λ=3 + 잔차보정`**: Unified 목적함수 + cube pose를 **미지수로
추정하되 FK 쪽으로 soft anchor(λ=3 px/mm)를 걸고** + 예측 단계에서 FK로 지도학습한 `[1,x,y]`
Ridge 잔차보정. 즉 **cube pose를 FK로 하드 고정하지 않는다.** FK는 (1) hand-eye 운동학 백본과
gauge 초기화, (2) solve의 soft anchor, (3) 잔차보정의 지도 신호로 쓴다.

채택 근거:

1. **측정된 최저 셀**이다 — 1.586 mm. 고정 λ=0(1.847)과 A3(2.355)보다 낮다.
2. **λ 선택이 사후 체리피킹이 아니다.** 중첩 교차검증(바깥 fold마다 나머지 12개로 λ를 고르고
   그 fold에서 평가)에서 **13/13 fold 전부가 λ=3을 선택**했고, 절차 자체의 성능이 1.586 mm다.
   28개 셀에서 우연히 좋은 값을 집은 것이라면 fold마다 다른 λ가 뽑혀야 하는데 그렇지 않다.
3. λ=0 대비 **−0.238 mm, t=−2.96, 11/13 fold** 우세이며, λ=0.3→1→3에서 단조로 개선되는
   추세 안에 있다.
4. 시뮬레이션 Exp3의 최적 구성 `Camera + FK-correction`과 **같은 계열**이다(cube pose를 하드
   고정하지 않고 FK를 보정 신호로 씀). 아래 "시뮬레이션과의 대응" 참조.

**λ는 하이퍼파라미터이므로 "교차검증으로 정한다"고 명시해야 한다.** 임의로 고른 값이 아니다.

**과대주장 금지 — 반드시 함께 말할 것.**

- `A3 + 보정`(2.355) 대비 우위는 paired 검정에서 **유의하지 않다**(t=+1.05). FK 하드 고정이
  나쁘다는 주장은 이 데이터로 할 수 없다.
- 13개 위치를 계속 재사용한다. 중첩 CV도 같은 위치 풀 안에서 돌며, 바깥 fold의 위치가 λ
  선택에 쓰인 다른 fold들의 **학습 데이터**에는 들어 있다(약한 누수).
- 차이의 절대 크기가 0.26 mm로 작고, 정답이 FK 프록시다.
- `‖` 열은 D2 재실행 값이므로 같은 표의 A2/A3 셀과 직접 빼지 않는다. paired 비교는 아래
  "채택 구성의 이벤트 split 평가"에 D2 자체 기준값으로 정리했다.

A3의 `의미` 열을 `7행 full row`로 적은 것은 [`CP_ablation_schema.py`](../CP_ablation_schema.py)가
A3에 붙인 코드 라벨 `"Ours (full)"`이 **7행 ablation 안에서 모든 구성요소를 켠 행**을 뜻하기
때문이다. 그 라벨은 최종 채택 구성을 가리키지 않는다. A3의 방어 가능한 강점은 정확도가 아니라
**수렴 안정성과 조건수**다(Table 2a σ=1 px에서 A2 3/10 vs A3 10/10; 조건수 316→51).

위 `±`는 5개 event split에서 얻은 split mean의 표준편차다. 각 split 안의 5개 initialization
표준편차는 별도로 저장하며 overall reprojection 기준 평균 약 `10⁻⁵ px`로 split 분산보다
훨씬 작다.
`e_task_pose` 열은 외부 물리 GT가 없으므로 **`FK-proxy`로만** 채운다. A2/A3와 그 잔차보정
행의 값은 [`CP_D1_fk_correction_2x2.py`](../CP_D1_fk_correction_2x2.py)의 **위치(set) 단위
leave-one-out 13 fold**에서 측정한 held-out 큐브중점 예측 오차 RMSE(mm)이며, 정답 자리에
로봇 FK를 넣은 값이라 **absolute accuracy가 아니다.** 이 열만 다른 split이므로 `¶`로 표시했고,
같은 행의 다른 열(이벤트 단위 split)과 한 줄로 이어 읽으면 안 된다. 큐브중점의 위치만
예측하므로 회전 성분은 없다. 나머지 행이 `—`인 것은 D1이 A2/A3 대비만 측정했기 때문이다.

`↳` 잔차보정 행에서 `N_reg`·`e_e2e`·`e_cross`·`e_reproj`가 `≡`인 것은 근사가 아니라 **정의상
동일**하다. 보정은 캘리브레이션이 끝난 뒤 예측된 큐브중점에만 적용되고 `T_base_Ci`·
`T_gripper_cam`을 전혀 바꾸지 않으므로, pose에서 파생되는 모든 지표가 비트 단위로 같다.
따라서 **어떤 보정 결과도 재투영 개선으로 보고할 수 없다.** 근거와 paired 검정은 아래
"A2/A3 위치 hold-out 재검증 — D1 2×2"에 있다.

#### `e_reproj`는 항상 큐브로 잰다

`e_reproj` 열은 **모든 행에 같은 질문**을 던진 값이다: 그 행의 `T_base_Ci`를 그대로 얼려서
**같은 held-out 큐브 코너**를, **같은 큐브 포즈**(전 행 공통 board-free FK artifact)로,
**같은 카메라**에서 재투영했을 때의 RMSE다. 재적합은 없다.

이전 판의 `e_reproj overall`은 행마다 자기 target set으로만 채점했기 때문에 board-only 행
(A0/B3)이 board 코너만으로 1.4387±0.0995 px를 받아 표에서 가장 좋아 보였다. 그 값은 성능이
아니라 **큐브를 한 번도 설명하도록 요구받지 않았다는 사실**을 뜻했다. 같은 큐브로 물으면
A0/B3는 cam1에서 2.56 px로 A3(0.77 px)의 **3.3배**다. 옛 target-set 한정 값
(A0/B3 1.4387, A1 4.6561, A2 4.5075, A3 4.6230, B1 4.6317, B2 7.4004 px)은
[`ablation_multisplit/multisplit_ablation.md`](ablation_multisplit/multisplit_ablation.md)에
카메라별·marker별로 남아 있으며, **행 간 비교에는 쓰지 않는다.**

통제 조건 세 가지를 반드시 함께 읽어야 한다.

- **채점 카메라는 cam0·cam1뿐이다.** 모든 행이 base에 등록한 카메라의 교집합이다. cam3은
  cube를 쓰는 행에만 존재하므로 포함하면 정확도가 아니라 커버리지를 비교하게 된다. 대신
  A0/B3는 cam3을 등록하지 못해 5 split 합계 큐브 코너 3684개 중 **1116개(30.3%)를 평가조차
  할 수 없다.** 이 손실은 RMSE에 섞지 않고 개수로만 보고한다. 즉 위 표는 **board-only 행에
  유리하게 기울어진 조건**에서 얻은 값이다.
- **pooled 열은 순위 근거로 쓰지 않는다.** cam0가 전 행에서 14.3~14.7 px로 똑같이 나빠서
  행 간 차이(최대 0.41 px)를 덮는다. 이는 축의 효과가 아니라 cam0 자체의 문제다. 판별력은
  `cam 1` 열에 있다.
- **eih 경로는 제외했다.** 공통 FK 큐브 포즈가 artifact의 `T_gripper_cam`으로 만들어졌으므로
  eih 재투영을 채점하면 그 gtc를 채택한 행이 유리해진다. e2h만 남기면 측정 대상이
  `T_base_Ci` 하나로 좁혀진다.

공통 FK 큐브 포즈는 **외부 물리 GT가 아니다.** train eih 큐브 코너와 raw FK에서 나왔으므로
전 행에 같은 오차 바닥이 깔린다(cam0의 14 px 상당 부분이 여기일 가능성이 크다). 행 간
비교로는 공정하지만 absolute accuracy로 읽으면 안 되며, 그래서 열 이름에
`^{cube|FK-fixed}`를 붙였다.

split 단위 paired delta는 [`cross_target_cube/cross_target_cube.md`](cross_target_cube/cross_target_cube.md)에
있다. 핵심만 옮기면 `B3→A3`가 cam1에서 **−1.7871±0.0639 px, 5/5 split**,
`B3→A0`는 **+0.0011±0.0009 px, 1/5**다. 즉 큐브를 기준으로 물어도 **board-only에서 Unified는
효과가 없다**(C1 축이 죽는다). 산출 코드는
[`CP_cross_target_cube_eval.py`](../CP_cross_target_cube_eval.py)다.

행별 인과 검증은 최신 설계를 유지한다.

| 델타 | 통제 | 검증 |
| --- | --- | --- |
| A0→A1 | seq 고정; 공통 target인 board pose는 영상으로 추정 | cube target 추가의 관측성 이득. 단, target set과 cube pose 변수도 함께 추가됨을 명시 |
| A1→A2 | target set과 FK→cube=vision-estimated 고정 | cube+board에서 U 효과 |
| B1→A3 | target set과 FK→cube=FK-fixed 고정 | FK-fixed cube 조건에서 U 효과 |
| A2→A3 | target set·U 고정 | cube pose만 vision-estimated→FK-fixed로 바꾼 효과 |
| B2→A3 | U·FK-fixed cube 고정 | board 추가 효과 |
| B3→A3 | U 고정 | board-only 대 full system의 **전체 구성 비교**; marker-only 인과 비교가 아님 |

확정된 4행 외의 미정 행은 요청문에 적힌 2개가 아니라 A0·A1·B1의 **3개**다. 위 표가 세 행까지 포함한 전체 7행 runner 스펙이다. 보고 표에는 board pose-source 열을 두지 않지만, Runner는 board pose를 FK-fixed/soft-anchor로 지정하는 입력을 물리적으로 불가능한 조건으로 즉시 반려한다.

### FK→cube 축이 실제로 바꾸는 것 — 자유도와 조건수

FK 고정이 제거하는 것은 미지수 1개가 아니라 **위치당 6 DOF × 학습 위치 수**다. D1 fold 0
(고정 카메라 3대, 학습 위치 12개)의 실측 `jacobian.n_params` (두 arm 모두 nullity 0):

| arm | 자유 파라미터 | 구성 | Jacobian condition (13 fold mean) | nfev (mean) |
| --- | ---: | --- | ---: | ---: |
| A2 vision-estimated | 102 | 카메라 6×3 + gTc 6 + board 6 + **큐브 6×12 = 72** | 316.1 | 106.5 |
| A3 FK-fixed | 30 | 카메라 6×3 + gTc 6 + board 6 | **51.2** | **19.4** |

즉 A3는 72 DOF를 제거해 **조건수 약 6배·수렴 속도 약 5배**를 얻는다. 이것이 FK 고정이 사는
것이며, 위 결과가 보여주듯 **정확도를 사는 것이 아니다.** 반대로 잔차 보정은 예측 단계에서
전역 9 DOF(`Ridge [1,x,y]`)만 더한다. 따라서 두 설계의 차이는 "FK 정보를 쓰느냐"가 아니라
**같은 FK 정보를 위치별 hard constraint(6S DOF)로 주입하느냐, 전역 smooth 회귀(9 DOF)로
주입하느냐**다. 후자는 위치들에 걸쳐 FK 잡음이 평균화되고, 전자는 위치별 FK 오차가 그대로
공유 변환(`bTCi`, `gTc`)에 실린다.

큐브 pose 자체는 부실하게 결정되는 nuisance 파라미터가 아니다 — 기본 정보 표대로 고정 카메라
평균 2.70대가 동시 관측하고(board 1.03대) 손목 카메라도 본다. 제거해서 얻는 식별성 이득은
작고 FK 오차 주입 손실은 크다는 것이 위 수치의 해석이다.

중간 지점인 soft anchor(`r_anchor = λ · r_SE3(bTO^(s), bTO,FK^(s))`)는 자유도를 남긴 채 gauge만
잡는다. C1 LOSO 13-fold에서 `unified_joint(soft-anchor λ=5) + SE(3)` = 2.374 mm가 전체 셀
최소값이었다. 다만 D1은 λ=0 끝점만, C1은 λ=5만, 그것도 서로 다른 backend·target set에서
측정했다. **λ sweep은 미실행이며 Supplementary의 FK-weight sensitivity 항목이 그 자리다.**

통제 지표 — 보정은 예측 시점에만 적용되고 `T_base_Ci`/`T_gripper_cam`을 바꾸지 않으므로 아래
두 값은 보정 유무와 무관하게 동일하다. **어떤 보정 결과도 재투영 개선으로 보고할 수 없다.**

| 통제 지표 | A2 | A3 | paired delta (A3−A2) |
| --- | ---: | ---: | ---: |
| held-out 재투영 (px, train 전용 FK pose) | 5.697±6.511 | 4.353±6.651 | −1.343, t=−6.66, 13/13 |
| held-out `e_cross` (mm, FK 무관) | 22.127±31.294 | 20.525±27.165 | −1.602, t=−0.91, 7/13 |

재투영 지표는 A3가 13/13으로 이기지만 **순환이다**: A3의 카메라는 FK 큐브 pose에 맞춰 적합된
뒤 그 FK pose에서 픽셀로 평가된다. FK를 전혀 참조하지 않는 `e_cross`는 유의하지 않으므로
(t=−0.91) **비순환 근거로 A3의 우위를 주장할 수 없다.** 두 지표는 항상 함께 인용한다.
`e_cross`의 큰 표준편차는 Table 1에 기록한 cam 0 / PnP flip 꼬리와 같은 원인이다.

이 결과는 Table 1의 A2→A3(+0.116 px, 5/5 악화)와 모순되지 않는다. Table 1은 held-out
**이벤트의 재투영**이고 D1은 held-out **위치의 3D 예측**이다. 서로 다른 질문이므로 한
줄에 놓고 비교하지 않는다.

남은 한계: (1) 13개 위치로는 `|Δ| ≈ 0.4 mm`를 가릴 검정력이 없다. (2) 1차 지표가 FK를 정답으로
쓰므로 최종 판정에는 외부 GT(Table 3, 보류)가 필요하다. (3) 보정 특징이 `[1,x,y]`뿐이며 큐브
위치의 z 분산이 작아 z 항을 검정하지 못했다.

산출물은 [`D1_fk_correction_2x2/D1_fk_correction_2x2.md`](D1_fk_correction_2x2/D1_fk_correction_2x2.md),
[JSON](D1_fk_correction_2x2/D1_fk_correction_2x2.json), [CSV](D1_fk_correction_2x2/D1_fk_correction_2x2.csv)에 있다.

### 전용 runner 핵심 계약

#### 데이터와 평가

- Capture event 단위로 train/test를 나누고 cube 위치별로 층화한다. 같은 event의 관측을 양쪽에 나누지 않는다.
- 1차 지표는 모든 pose를 고정한 held-out corner reprojection RMSE다. Test 관측으로 초기화·재정렬·refit·outlier 선택을 하지 않는다.
- 모든 행은 fitting 전에 만든 동일한 measurement-only mask를 사용한다. Cube가 없는 A0/B3의 path metric은 0이 아니라 N/A다.
- Target set이 다른 행은 공통 marker component만 비교한다. Train loss는 수렴 진단값이며 성능 순위에 사용하지 않는다.
- 외부 GT가 없는 pose 오차는 반드시 `FK-proxy`로 표시하고 physical accuracy로 해석하지 않는다.

#### 행별 변수 처리

- `K`, distortion, target geometry, backbone `T_base_gripper[event]`, split, mask와 optimizer 설정은 전 행 공통이다.
- Seq(A0/A1/B1)는 eih 단계 출력을 완전히 고정한 뒤 e2h 단계에서 `T_base_Ci`만 푼다. Alternating pass는 금지한다.
- Unified(A2/A3/B2/B3)는 해당 행의 자유변수를 하나의 objective에서 공동 최적화한다.
- B1/A3/B2의 FK-fixed cube pose와 hand-eye 초기값은 train eih cube corner와 raw FK만으로 만든 동일한 board-free artifact를 공유한다. 행별 재정렬은 금지한다.
- Board가 있는 행의 `T_base_board`는 항상 영상으로 추정하며 FK pose source를 사용하지 않는다.

#### 공통 backend와 수렴 판정

- 모든 행은 동일한 per-corner reprojection backend, local SE(3) retraction과 명시적 freeze mask를 사용한다.
- Solver 설정은 `soft_l1(f_scale=2 px)`, `x_scale='jac'`, `max_nfev=300`, tolerance `1e-8`로 고정한다.
- 각 run은 종료 status, nfev, optimality, Jacobian rank/nullity/condition과 train support를 저장한다.
- Noise-free synthetic에서 A1=A2와 B1=A3가 같은 해와 zero reprojection에 도달하지 않으면 실데이터 실행을 중단한다.
- Train support가 없는 vision-estimated cube 위치는 test 영상으로 맞추지 않고 `unsupported`로 기록한다.

구현은 [`CP_ablation_7row.py`](../CP_ablation_7row.py),
[`calibration_reprojection_backend.py`](../calibration_reprojection_backend.py),
[`calibration_fk_cube_artifact.py`](../calibration_fk_cube_artifact.py),
[`calibration_path_evaluation.py`](../calibration_path_evaluation.py)에 있다.
Canonical 5-split × 5-init의 175/175 run이 이 계약으로 수렴했으므로 완료된 구현 이력과
후보 optimizer 비교는 이 문서에서 제외한다.

### Canonical 5-split × 5-init 반복 결과

미리 선언한 split seed `20260729, 20260730, 20260731, 20260732, 20260733`를 하나도
제외하지 않고 5 splits×7 rows×5 initializations, 총 175 row runs를 실행했다. 모든 run이
수렴했고 각 split은 45 held-out events를 가진다. 실행 계획과 split별 원본, paired effect
집계는 [`ablation_multisplit/`](ablation_multisplit/)에 있다.

오차 delta는 항상 `두 번째 행−첫 번째 행`이므로 음수가 개선이다.

| 비교 | 공통 component | delta mean±split std | 두 번째 행 개선 split |
| --- | --- | ---: | ---: |
| A0→A1 | board reproj | +0.00315±0.00297 px | 1/5 |
| A0→A1 | N_reg | +1.000±0.000 | 5/5 |
| A1→A2 | overall reproj | **−0.14858±0.02161 px** | **5/5** |
| B1→A3 | overall reproj | −0.00869±0.00220 px | 5/5 |
| A2→A3 | overall reproj | **+0.11555±0.02529 px** | **0/5** |
| B2→A3 | cube reproj | +0.02960±0.00273 px | 0/5 |
| B2→A3 | e_e2e translation | +0.49938±0.06760 mm | 0/5 |
| B2→A3 | e_cross translation | +0.18861±0.00901 mm | 0/5 |
| B3→A3 | board reproj (reference) | +0.00955±0.02238 px | 1/5 |

따라서 실데이터에서 방어되는 주장은 제한적이다. vision-estimated cube 조건의 Unified는 일관된
held-out 개선을 보였지만, FK-fixed 조건의 Unified 효과는 약 0.009 px로 매우 작다.
FK-fixed(A2→A3)는 5/5 split에서 overall reprojection을 악화했고, board 추가(B2→A3)도
공통 cube/path 지표에서 0/5 개선이다. 이 결과로 “FK 고정이 오차 전파를 줄인다”거나
“board가 정확도를 높인다”고 주장하면 안 된다. 해당 인과 주장은 다음 synthetic noise sweep과
low-N sensitivity에서 별도로 검증해야 한다.

A2→A3(FK 고정)에 한정한 후속 검증은 바로 아래 "A2/A3 위치 hold-out 재검증 — D1 2×2"에 있다.
그쪽은 split 단위와 지표가 달라(이벤트 재투영 px → 위치 3D 예측 mm) 위 delta 표와 같은
줄에서 비교하지 않는다.

카메라별 반복 split 평균에서 cam 0 reprojection은 A1/A2/A3가 각각
11.021±1.201/10.654±1.164/10.961±1.145 px로 다른 카메라(대체로 0.73–1.64 px)보다
현저히 크다. ungated cross 지표의 큰 값은 이 카메라/특정 event 문제와 연결되므로 별도
detector/pose-ambiguity 진단 대상이다. 현재 결과에서 사후 제거하지 않는다.

### A2/A3 위치 hold-out 재검증 — D1 2×2

[`CP_D1_fk_correction_2x2.py`](../CP_D1_fk_correction_2x2.py)로 실행했다. "cube pose를 FK로
고정하고 예측 단계에서 잔차를 보정하는 구성이 최선인가"는 지금까지 **동일 조건에서 측정된
적이 없었다.** A3는 canonical corner backend·이벤트 split·px 지표이고, C1의 Ridge 후보정은
historical SE(3) pose-residual 솔버·cube-only·set split·mm 지표였다. 이 표는 그 조합을 한
backend·한 solver 설정·한 예측 mask에서 직접 측정한 결과다.

Split은 **위치(set) 단위 leave-one-out 13 fold**다. 잔차 보정은 공간적 일반화 주장이므로
모든 위치가 train에 들어가는 이벤트 split으로는 검증할 수 없다. 이 split의 역할은
`POSITION_HOLDOUT_ROLE`에 따라 **`FK-proxy` 전용**이며 absolute accuracy로 해석하지 않는다.
두 arm은 `T_base_cube_by_set`가 자유변수인지(A2) 학습셋 전용 FK artifact에 고정되는지(A3)만
다르고, held-out 위치의 캡처 이벤트는 board 관측까지 양쪽 모두에서 제외했다. 13/13 수렴.

Cube pose를 다루는 방식은 이산 선택이 아니라 **하나의 연속 축**이다. 목적함수에 soft anchor를
더하고 그 세기 `λ`를 쓸면 A2와 A3가 그 축의 양 끝점이 된다.

```
min_Θ  Σ ρ(‖r_reproj‖²)  +  Σ_s ‖ λ · ( T_est(s)·P − T_FK(s)·P ) ‖²
                              └── 큐브 probe 점의 변위 (mm) ──┘
```

`P`는 큐브 중심에서 ±29.5 mm 떨어진 6개 점이다. anchor 항이 재투영 항과 **같은 기하량**(코너가
얼마나 움직였나)을 픽셀 대신 mm로 재므로 `λ`의 단위는 **px/mm**이고, `λ=1`은 "큐브 1 mm 변위 =
재투영 1 px"를 뜻한다. `λ=0`이면 A2, `λ→∞`면 A3다.

held-out 위치의 큐브중점 예측 오차 RMSE (mm, `FK-proxy`):

| arm | λ (px/mm) | none | offset (3) | SE(3) (6) | Ridge `[1,x,y]` (9) | Jacobian cond | nfev |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| A3 (하드 고정) | ∞ | 2.988 | 2.341 | 2.401 | 2.355 | **51.2** | **19.4** |
| A2 | 0 | 3.451 | 2.685 | 2.479 | 1.847 | 316.1 | 106.5 |
| A2@λ=0.3 | 0.3 | 3.414 | 2.677 | 2.476 | 1.839 | 315.0 | 98.2 |
| A2@λ=1 | 1 | 3.229 | 2.657 | 2.461 | 1.771 | 309.6 | 92.9 |
| **A2@λ=3 ★** | 3 | 2.729 | 2.560 | 2.405 | **1.586** | 297.3 | 86.4 |
| A2@λ=10 | 10 | **1.907** | **1.845** | 1.914 | 1.704 | 267.7 | 86.0 |
| A2@λ=30 | 30 | 2.875 | 2.254 | 2.312 | 2.260 | 155.6 | 105.0 |
| A2@λ=100 | 100 | 2.983 | 2.337 | 2.397 | 2.351 | 160.4 | 85.7 |

**구현 검증**: λ=100의 값이 A3와 fold 단위 최대 **0.011 mm** 차이로 일치한다. anchor가 λ→∞에서
하드 고정으로 정확히 수렴하므로 이 축은 실제로 연속이며, A2·A3가 같은 축의 두 끝점임이 확인된다.
λ=0은 실행 시 canonical solver와 `max|ΔT| = 0` 동치를 assert로 검증한다.

fold 단위 paired 검정 (음수가 개선):

| 항목 | mean (mm) | SE | t | 개선 fold |
| --- | ---: | ---: | ---: | ---: |
| **λ=3 − λ=0 (Ridge)** | **−0.238** | 0.080 | **−2.96** | **11/13** |
| λ=1 − λ=0 (Ridge) | −0.069 | 0.034 | −2.01 | 9/13 |
| λ=10 − λ=0 (Ridge) | −0.162 | 0.204 | −0.79 | 7/13 |
| A3 − λ=0 (Ridge) | +0.361 | 0.345 | +1.05 | 5/13 |
| A3 − A2 @ none | −0.401 | 0.552 | −0.73 | 7/13 |
| λ=0: Ridge − none | **−1.587** | 0.345 | **−4.59** | 11/13 |
| λ=3: Ridge − none | **−1.138** | — | **−4.34** | 11/13 |
| A3: Ridge − none | **−0.824** | 0.335 | **−2.46** | 10/13 |

**중첩 교차검증** — 바깥 fold마다 나머지 12개 fold에서 λ를 고르고 그 fold에서만 평가했다.

| | RMSE (mm) |
| --- | ---: |
| 중첩 CV (λ를 절차로 선택) | **1.586** |
| 고정 λ=0 | 1.847 |
| A3 | 2.355 |

**13개 바깥 fold 전부가 λ=3을 선택했다.** 선택이 흔들리지 않으므로 λ=3의 우위는 28개 셀에서
우연히 집힌 값이 아니며, 다중비교 보정을 근거로 기각할 사안이 아니다. 다중비교 방어는 선택이
불안정할 때 필요한 것이다.

판정:

- **잔차 보정은 모든 arm에서 실재하는 효과다.** λ=0 −1.587 mm(t=−4.59), λ=3 −1.138 mm(t=−4.34),
  A3 −0.824 mm(t=−2.46). 계수는 train 위치에서만 학습해 held-out 위치에 적용했으므로 이는
  **공간적 일반화**다.
- **보정 없는 열이 λ에 대해 U자를 그린다** — 3.451 → 1.907(λ=10) → 2.983(λ=100). 양 끝점이
  둘 다 최적이 아니고 중간이 가장 좋다. cube pose를 완전히 자유롭게 두는 것도, FK에 하드
  고정하는 것도 최선이 아니라는 직접 증거다.
- **조건수는 λ와 함께 단조 개선된다**(316 → 51). 즉 λ는 "정확도 대 수치 안정성"을 연속적으로
  조절하는 손잡이이며, 두 목표가 서로 다른 λ에서 최적이 된다.
- **FK 하드 고정이 나쁘다는 주장은 할 수 없다.** A3 − λ=0은 t=+1.05로 유의하지 않다.
- **"FK 고정만으로는 악화되고 후보정이 있어야 vision-estimated를 이긴다"는 가설은 지지되지
  않는다.** 보정 전 A3는 A2보다 나쁘지 않았고(−0.401, t=−0.73), 보정 후에는 A2가 근소하게
  앞선다(+0.361, t=+1.05). 어느 쪽도 유의하지 않다.

### 채택 구성의 이벤트 split 평가 — D2

D1은 위치 hold-out·mm 지표라 Table 1의 나머지 열을 채우지 못한다. soft anchor는 solve 자체를
바꾸므로 잔차보정 행처럼 `≡`(정의상 동일)로 넘길 수도 없다. [`CP_D2_anchored_event_split.py`](../CP_D2_anchored_event_split.py)가
그 칸을 채운다. **`CP_ablation_7row.py`는 수정하지 않았다** — 7행 계약 검증과 noise-free sanity
gate가 걸려 있어 여덟 번째 arm을 넣으면 계약 재검증과 artifact desync가 발생한다. 대신 그
runner의 `prepare_ablation_data()`(검출·이벤트 split·train 전용 FK artifact·초기화·solver
설정·path mask)를 그대로 재사용하고 목적함수의 anchor 항만 교체했다.

Canonical split seed 5개 × 3 initialization, 45 run, 45/45 수렴. split mean의 mean±std:

| arm | λ | N_reg | e_reproj cube (px) | e_e2e (mm/°) | e_cross (mm) | Jacobian cond |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| A2 (기준) | 0 | 3 | 7.2407±0.7233 | 15.2490±1.3521 / 7.7516±0.5914 | 36.8593±3.5208 | 327.7 |
| **A2@λ=3 ★** | 3 | 3 | **7.2512±0.7241** | **15.1168±1.3115 / 7.7342±0.5882** | **35.0872±3.5329** | 302.1 |
| A3 (기준) | ∞ | 3 | 7.4635±0.7114 | 15.0259±1.2153 / 7.7342±0.5855 | 32.7547±3.2790 | 51.0 |

세 arm 모두 `N_reg=3`으로 같은 카메라를 등록하므로 이 표 안에서는 큐브 재투영을 직접 비교해도
된다. (Table 1의 `e_reproj` 열은 board-only 행까지 포함해야 해서 카메라 교집합·공통 큐브 포즈로
따로 채점한 값이므로, 이 표의 숫자와 자리를 맞바꿔 읽지 않는다.)

split 단위 paired delta — **A2(λ=0) 대비**, 음수가 개선:

| 비교 | e_reproj cube (px) | e_e2e (mm) | e_cross (mm) |
| --- | ---: | ---: | ---: |
| **A2@λ=3 − A2** | +0.0105±0.0100, t=+2.10, 0/5 | **−0.1321±0.0581, t=−4.55, 5/5** | **−1.7721±0.0995, t=−35.62, 5/5** |
| A3 − A2 | +0.2228±0.0423, t=+10.53, 0/5 | −0.2230±0.2366, t=−1.89, 3/5 | **−4.1046±0.3433, t=−23.91, 5/5** |

#### ⚠ Table 1 재현 확인 — 기존 "통과" 판정은 무효

이전 판은 A2/A3가 Table 1과 최대 1.51σ로 맞으므로 "검출 RANSAC 변동" 범위이고 따라서 anchored
행이 Table 1과 같은 척도 위에 있다고 적었다. **틀렸다.** 편차의 원인은 RANSAC이 아니라
`RB_ROBOT_POS_SCALE`이다. 이 표는 **k=1.0229**, Table 1의 A0–B3는 **k=1.0**이다(위
"로봇 스케일 기준 불일치" 참조). σ 판정은 허용오차가 Table 1의 split 표준편차 단위라서
5.29 mm의 스케일 이동을 걸러내지 못한다.

같은 코드를 **k=1.0으로 재실행**하면 Table 1의 A2/A3가 `~1e-5`로 재현된다
([`cross_target_cube/D2_rerun_k1.0/`](cross_target_cube/D2_rerun_k1.0/)). 그런데 그 기준에서는
**채택 구성의 판정이 뒤집힌다.**

| A2@λ=3 − A2 | k=1.0229 (위 표) | k=1.0 (재실행) |
| --- | ---: | ---: |
| e_reproj cube (px) | +0.0105±0.0100, t=+2.10, 0/5 | **+0.0305±0.0105, t=+5.81, 0/5** |
| e_e2e (mm) | **−0.1321±0.0581, t=−4.55, 5/5** | **+0.0397±0.0185, t=+4.29, 0/5** |
| e_cross (mm) | **−1.7721±0.0995, t=−35.62, 5/5** | **−0.1704±0.0674, t=−5.06, 5/5** |

읽어야 할 것:

- **`e_e2e`는 부호가 뒤집힌다.** k=1.0229에서 −0.132 mm(5/5 개선)이던 것이 k=1.0에서
  +0.040 mm(0/5, t=+4.29)로 **유의하게 악화**된다. "경로 일치도가 일관되게 개선된다"는 주장은
  k에 의존한다.
- **`e_cross` 이득은 10배 줄어든다**(−1.772 → −0.170 mm). 방향은 유지되지만 크기가 무너진다.
- **"재투영을 희생하지 않는다"도 성립하지 않는다.** k=1.0에서는 +0.0305 px(t=+5.81, 0/5)로
  모든 split에서 악화한다.
- A3 − A2의 `e_cross`도 −4.105 → −0.886 mm로 같은 방향으로 줄어든다.

**k가 확정되지 않은 상태이므로 두 판정 중 어느 쪽도 최종이 아니다.** k=1.0229는 잠정값이고
카메라별 추정이 1.4~2.9%로 벌어져 있다([`robot-kinematic-scale-error`] 근거). 채택 구성의
이벤트-split 우위는 **k에 대한 민감도가 결론을 바꿀 만큼 크다**는 사실을 함께 보고해야 하며,
k를 물리 측정으로 확정하기 전에는 이 절의 어느 수치도 단독 근거로 쓰지 않는다. D1의 위치
hold-out 결과(λ 중첩 CV 13/13)는 별개의 split·지표이므로 이 문제와 분리해 읽는다.

판정 — **k=1.0229에서만 성립한다.** 각 항목에 k=1.0 재실행 결과를 붙여 둔다.

- **soft anchor의 재투영 손실.** k=1.0229에서는 A2@λ=3 − A2가 큐브 재투영 +0.0105 px
  (t=+2.10)로 A3의 +0.2228 px(t=+10.53) 대비 21배 작다 — 하드 고정이 치르는 대가를 거의
  치르지 않는다는 뜻이다. 다만 0/5로 방향 자체는 악화이며, **k=1.0에서는 +0.0305 px
  (t=+5.81, 0/5)로 손실이 3배 커진다.** "재투영을 희생하지 않는다"고는 말할 수 없다.
- **경로 일치도 개선은 k에 의존한다.** k=1.0229에서 `e_cross` −1.772 mm(5/5, t=−35.62),
  `e_e2e` −0.132 mm(5/5, t=−4.55)다. **k=1.0에서는 `e_cross`가 −0.170 mm로 10배 줄고
  `e_e2e`는 +0.040 mm(0/5)로 부호가 뒤집힌다.** 이 두 지표는 잔차보정이 바꿀 수 없으므로
  캘리브레이션 자체의 변화를 가리키는 것은 맞지만, 그 변화의 크기와 부호가 미확정 상수 하나에
  좌우된다.
- **A3는 `e_cross`를 더 크게 줄이지만(−4.105) 재투영을 확실히 악화시킨다.** anchor 세기는
  이 둘의 교환비를 정하는 손잡이라는 해석은 두 k에서 모두 유지된다(k=1.0에서도 A3는
  `e_cross` −0.886 mm, 재투영 +0.196 px로 같은 교환 방향이다).
- 따라서 **"세 축에서 동시에 방어된다"고 말할 수 없다.** 현재 방어되는 것은 위치 예측
  (D1 1.586 mm, 별개 split·지표)이고, 경로 일치도와 재투영은 **k 확정 이후에 재판정**해야
  한다.

`e_e2e`·`e_cross`는 외부 GT가 아닌 내부 일관성 지표이므로 절대 정확도로 읽지 않는다. 산출물은
[`D2_anchored_event_split/`](D2_anchored_event_split/)(k=1.0229)과
[`cross_target_cube/D2_rerun_k1.0/`](cross_target_cube/D2_rerun_k1.0/)(k=1.0)에 있다.

### 시뮬레이션과의 대응 — 같은 최적 구성을 가리킨다

`Simul_test/exp3_gtc_estimation.py`는 같은 세 방식을 GT가 있는 합성 장면에서 비교한다
(8 sets, held-out 큐브 예측 mm; 원본 [`exp3_noise_sweep_data.json`](../Simul_test/figures/exp3_noise_sweep_data.json)).

| 시뮬 방식 | σ=2 mm | σ=6 mm | σ=15 mm | D1 대응 |
| --- | ---: | ---: | ---: | --- |
| ① Camera-based (큐브=미지수) | 0.390 | 1.073 | 2.624 | A2 none |
| ② FK-based (큐브=FK 고정) | 0.405 | 1.217 | **3.039** | A3 none |
| ③ **Camera + FK-correction** | **0.069** | **0.183** | **0.470** | **A2 + Ridge** |

③은 ①번 모델 **위에** 잔차 보정을 얹은 것이며 큐브는 끝까지 자유변수다
(`W = learn_fk_residual(scene, m_cam, train_sets)`). 미지수를 실제로 제거한 ②는 held-out에서
전 구간 꼴찌다. 시뮬의 gTc 복원 오차가 그 이유를 보여준다: σ=2→15 mm에서 camera-based는
0.116→0.249 mm인데 FK-based는 0.130→**0.978 mm**로 4배 악화한다. **큐브를 상수로 만들면
관측 노이즈를 흡수할 자유도가 사라져 `gTc`로 몰린다.**

C1 시뮬(`unified_vs_independent.py`)에는 FK-fixed arm이 아예 없다. FK는 gauge 초기화에만
쓰고 `+fk` 잔차 보정은 모든 방식에 대칭으로 적용하며, 최고는 `Joint+fk` 0.354 mm다.

따라서 **시뮬과 실데이터의 최고 셀이 일치한다: `vision-estimated cube pose + FK-supervised
잔차 보정`.** C3 LOSO도 같은 방향이다(`no_fk_prior` 12.199 < `fk_prior_correction` 13.054 mm).

주의 — CP의 C3 구현(`05_fk_prior_correction`)은 시뮬과 달리 보정을 **fk-prior 위에** 얹는다.
이는 [`CP_EXPERIMENTS_README.md`](../CP_EXPERIMENTS_README.md)에 명시된 의도적 divergence이며,
시뮬 ③과 같은 방식이 아니다. 두 결과를 같은 방식으로 인용하지 않는다.

또한 `CP_ablation_schema.py`가 A3에 붙인 `"Ours (full)"` 라벨은 **7행 ablation 안에서 모든
구성요소를 켠 행**이라는 뜻이지, 위 대응이 가리키는 권장 구성이라는 뜻이 아니다. A3의 근거는
정확도가 아니라 **수렴 안정성**(Table 2a: σ=1 px에서 A2 3/10 vs A3 10/10)이다.

### Canonical fixed-split 3-seed 해석

정식 산출물은 [`ablation_7row_canonical/`](ablation_7row_canonical/)에 있다. 7행의 모든
stage가 3/3회 SciPy status 2(`ftol`, cost plateau)로 종료했고 초기값 간 분산은 작았다.
이는 `gtol` 도달을 뜻하지 않으므로 "gradient가 0인 해"가 아니라 **안정적인 cost-plateau
수렴**으로 보고한다.

공통 component에 한정한 fixed-split 관찰은 다음과 같다.

- A1→A2: overall held-out reprojection은 4.9907→4.8118 px, cube component는
  7.9939→7.6867 px로 감소했다. vision-estimated 조건의 Unified 이득 방향이다.
- B1→A3: overall 4.9610→4.9503 px로 차이가 작고, `e_e2e`는
  15.6093→15.6200 mm로 소폭 악화, `e_cross`는 40.4852→40.4208 mm로 소폭 개선됐다.
  FK-fixed 조건의 Unified 효과는 현재 split에서 사실상 작고 지표별 방향도 일치하지 않는다.
- A2→A3: overall 4.8118→4.9503 px, cube 7.6867→7.9233 px,
  `e_e2e` 15.5439→15.6200 mm로 악화했고 `e_cross`만 41.1119→40.4208 mm로 개선됐다.
  따라서 현재 실데이터만으로 FK-fixed가 전반적 오차 전파를 줄였다고 주장할 수 없다.
- B2→A3는 공통 cube component만 보면 7.8938→7.9233 px, `e_e2e`
  15.1997→15.6200 mm, `e_cross` 40.2326→40.4208 mm다. 이 split에서는 board 추가 이득이
  관찰되지 않았다. 저N sensitivity와 synthetic noise sweep이 board 유지 여부의 다음 판정이다.
- A0→A1의 공통 board component는 1.5448→1.5479 px로 개선되지 않았지만 N_reg는
  2→3으로 증가했다. 이는 pixel accuracy 이득이 아니라 camera registration/observability
  이득으로만 해석해야 한다.

ungated `e_cross`가 약 40 mm이고 rotation이 약 36°인 이유는 cam 0을 포함한 일부 pair가
크기 때문이다. 대표적으로 event 229의 cam 0–1/3 차이는 약 148–152 mm,
150–152°다. cam 1–3만 보면 A3 translation/rotation RMSE가 약 13.7 mm/1.18°다.
이것은 output threshold로 숨기지 않고 per-pair JSON에 보존했다. 향후 detector-quality 또는
pose-ambiguity 기준을 추가하려면 **모델 fitting 전에 정의하고 모든 행에 같은 새 mask로 전체
재실행**해야 한다.

### 별도 strict-none 진단 — 핵심표 미사용

Strict-none 공정 비교 결과는 [`A2_strict_none_vs_A3_fk_fixed.md`](A2_strict_none/A2_strict_none_vs_A3_fk_fixed.md)에 있다. A2 Jacobian은 84/84 full rank, nullity 0이어서 명백한 scale/base-frame gauge는 검출되지 않았다. 그러나 5개 초기값 모두 300 evaluations에서 종료조건을 만족하지 못했고, Jacobian condition은 279.8–352.2로 A3의 56.1–131.8보다 나빴다. FK/object-frame 정렬은 train 9개 set에서만 추정했다. 참고 held-out FK-proxy는 A2 4.71±0.50 mm/0.79±0.14°, A3 3.35±0.42 mm/0.80±0.11°지만, 둘 다 미수렴이므로 핵심표 확정값으로 사용하지 않는다.

### 현재 존재하는 부분 결과

| C1 method | Target set | FK→cube | held-out FK-proxy raw (mm) | SE(3) correction 후 (mm) | pooled proxy (mm/°) |
| --- | --- | --- | ---: | ---: | ---: |
| independent | cube only | FK-fixed | 13.39 | 5.06 | 15.98 / 8.98 |
| unified_joint | cube only | vision-estimated + FK soft-anchor (λ=5) | 11.67 | 4.50 | 15.25 / 7.47 |
| joint_fk_fixed | cube only | FK-fixed | 11.53 | 4.59 | 16.77 / 8.47 |

출처: [`joint_ablation_summary.csv`](C1/joint_ablation_summary.csv), [`CP_C1_unified_vs_independent.py`](../CP_C1_unified_vs_independent.py). `pooled proxy`는 직접 e2h–eih pair metric이 아니다.

### Supplementary — FK-weight sensitivity

기존 `unified_joint(anchor_weight=5)`의 11.67 mm는 A2에서 제거하고 soft-anchor 참고 결과로만 유지한다. 후속 sensitivity는 동일 runner에서 weight `0, 0.1, 1, 5, 10`처럼 실행해야 하며, 현재 확정된 것은 weight 5의 기존 C1 부분 결과뿐이다.

| FK anchor weight | Target set | FK→cube | held-out FK-proxy raw (mm) | 상태 |
| ---: | --- | --- | ---: | --- |
| 0 | cube+board | vision-estimated | 단일값 미채택 | 새 cube+board reprojection BA에서 수렴 불안정 |
| 5 (요청 C1) | cube+board | vision-estimated + FK soft-anchor (λ=5) | — | 동일 cube+board runner로 미실행 |
| 5 (기존 부분값) | cube only | vision-estimated + FK soft-anchor (λ=5) | 11.67† | 기존 cube-only C1 pose-residual 결과 |

### Post-correction ablation — 핵심 A2/A3와 분리

| C1 method | raw (mm) | Ridge 후 (mm) | SE(3) 후 (mm) |
| --- | ---: | ---: | ---: |
| independent | 13.39 | 2.81 | 5.06 |
| unified soft-anchor | 11.67 | 2.51 | 4.50 |
| joint FK-fixed | 11.53 | 2.58 | 4.59 |

이 값들은 fitting 후 FK residual을 학습한 결과이므로 Table 1의 raw A2/A3 셀에 넣지 않는다.
위 세 행은 backend·target set·split·지표가 서로 다른 historical 결과이므로 이것만으로
"FK-fixed + 후보정"의 우열을 말할 수 없다. 그 조합의 통제 비교는 위 "A2/A3 위치 hold-out 재검증 — D1 2×2"다.

| C2 source | N_reg | e_cross mean (mm) | repeatability (mm/°) | cube/board reproj (px) |
| --- | ---: | ---: | ---: | ---: |
| board_only | 2 | 26.94 | 26.94 / 15.60 | 0.619 / 0.671 |
| cube_only | 3 | 4.69 | 4.69 / 1.90 | 0.619 / 0.671 |
| hybrid | 3 | 4.86 | 4.86 / 1.83 | 0.619 / 0.671 |

출처: [`C2_cube_vs_board.csv`](C2/C2_cube_vs_board.csv). C2는 marker-source 실험이지 U/FK 통제 실험이 아니다. Reprojection 값은 mode와 무관한 marker PnP 집계라 현재 ablation 판별력이 없다.

## Table 2a — Table 1의 시뮬레이션 (custom pixel-level GT)

**이 표는 Table 1과 같은 7행(A0–B3)을 GT가 있는 합성 장면에서 돌린 것이다.** 같은
`CP_ablation_schema.MAIN_ABLATION_CONDITIONS`, 같은 canonical corner backend, 같은 variable
partition과 freeze 규칙을 쓰므로 Table 1의 행과 1:1로 대응한다. 실데이터에는 외부 GT가 없어
`FK-proxy`로만 말할 수 있는 것을, 여기서는 **참값 대비 오차**로 말할 수 있다는 점이 존재 이유다.

> **시뮬레이션 계열이 두 개라는 점에 주의.** 이 표의 출처는 [`CP_synthetic_7row.py`](../CP_synthetic_7row.py)이고,
> [`Simul_test/`](../Simul_test/)는 **다른 계열**이다. `Simul_test/`는 C1(Unified vs
> Independent) · C2(Board vs Cube) · C3(gTc estimation)라는 **세 축을 각각 따로** 검증하는
> 초기 순수 SE(3) 기하 시뮬이며, A0–B3 factorial도 canonical corner backend도 쓰지 않는다.
> 따라서 `Simul_test/`로는 이 표를 만들 수 없고, 그쪽 결과는 Table 1의 행과 직접 대응시키지
> 않는다. `Simul_test/`의 결과는 채택 구성의 근거로서 Table 1 안의
> "시뮬레이션과의 대응" 절에서 인용한다.

셀은 모든 행에 공통인
calibration transforms `T_base_Ci`와 `T_gripper_cam`의 GT 오차 RMS `e_X (mm/°)`, 10개
사전 선언 corner-noise seed의 mean±std다. Target-pose error는 headline에서 제외해 target set이
다른 행도 같은 물리량으로 비교한다.

| # | Marker | Unified | FK→cube | 의미 | σ=0 px | σ=0.5 | σ=1.0 | σ=2.0 |
| --- | --- | :---: | --- | --- | --- | --- | --- | --- |
| A0 | board | seq | — | baseline | 0.0000/0.0000 | 79.4123±30.8424 / 3.6716±1.6905 | 130.1191±32.3859 / 5.9538±1.7338 | 202.0396±45.1105 / 9.6223±2.1562 |
| A1 | cube+board | seq | vision-estimated | +cube | 0.0000/0.0000 | 21.9581±14.3231 / 0.8849±0.5592 | 44.8301±27.7913 / 1.7986±1.1009 | 93.2491±55.5227 / 3.7817±2.2779 |
| A2 | cube+board | U | vision-estimated | +Unified | 0.0000/0.0000 | **mixed (6/10)** | **mixed (3/10)** | **unstable (0/10)** |
| **A3★** | cube+board | **U** | **FK-fixed** | Ours | 0.0000/0.0000 | **2.0814±0.5518 / 0.1099±0.0306** | **4.1766±1.1745 / 0.2200±0.0651** | **8.7136±2.4322 / 0.4587±0.1373** |
| B1 | cube+board | seq | FK-fixed | −Unified | 0.0000/0.0000 | 2.9814±1.4306 / 0.1577±0.0820 | 6.0799±2.7919 / 0.3215±0.1603 | 13.0314±5.4249 / 0.6894±0.3135 |
| B2 | cube only | U | FK-fixed | −board | 0.0000/0.0000 | 3.5592±0.6644 / 0.1919±0.0375 | 7.1300±1.4631 / 0.3841±0.0823 | 14.5894±3.4671 / 0.7852±0.1946 |
| B3 | board only | U | — | −cube | 0.0000/0.0000 | 61.5659±32.9428 / 2.8209±1.6610 | **mixed (9/10)** | 189.7227±31.9070 / 8.9634±1.9877 |

판정:

- A2는 σ=0.5/1/2에서 각각 6/10, 3/10, 0/10만 `ftol` 수렴했고 나머지는 모두
  `max_nfev=300`에 도달했다. 미수렴 run의 GT 숫자는 diagnostic JSON에만 두고 headline
  평균에서 제거했다. A3는 모든 noise에서 10/10 수렴하므로 strict vision-estimated pose의 noise
  안정성이 낮고 FK-fixed가 이를 제거한다는 **수렴 안정성 증거**는 있다. 그러나 A2/A3 numeric
  gap은 A2가 완전 수렴하지 않아 headline으로 계산하지 않는다.
- B2→A3 translation delta는 σ=0.5/1/2에서 −1.478/−2.953/−5.876 mm이고 rotation도
  −0.082/−0.164/−0.326°다. custom simulator에서는 dense board의 noise 강건성 이득이
  noise와 함께 증가한다. 반면 실데이터 5 splits에서는 B2→A3가 0/5 개선이므로 이 synthetic
  주장이 실제 setup에서 재현됐다고 말할 수 없다.
- B1→A3 translation delta는 σ=0.5/1/2에서 −0.900/−1.903/−4.318 mm로 Unified 효과가
  noise와 함께 증가한다.

### Supplementary — FK-fixed cube pose noise

pixel σ=1 px에서 B1/A3/B2가 공유하는 fixed cube pose에 FK noise를 주입했다. A2는 같은
pixel 조건에서 3/10만 수렴해 numeric A2/A3 비교를 하지 않는다.

| FK cube pose noise σ | A3 (mm/°) | B1 (mm/°) | B2 (mm/°) |
| --- | ---: | ---: | ---: |
| 0 mm / 0° | 4.1766±1.1745 / 0.2200±0.0651 | 6.0799±2.7919 / 0.3215±0.1603 | 7.1300±1.4631 / 0.3841±0.0823 |
| 1 mm / 0.1° | 9.5144±4.7204 / 0.5516±0.2361 | 14.1866±5.7129 / 0.7741±0.3266 | 11.2176±3.7589 / 0.6271±0.2008 |
| 3 mm / 0.3° | 26.5421±14.8343 / 1.5471±0.7423 | 37.9927±18.3182 / 2.1047±1.0171 | 25.5868±14.0235 / 1.4958±0.6879 |
| 5 mm / 0.5° | 44.1076±24.9309 / 2.5822±1.2636 | 63.4551±31.4096 / 3.5277±1.7660 | 41.1787±24.3411 / 2.4496±1.1987 |

FK pose noise가 증가하면 A3 오차도 빠르게 증가한다. 특히 B2→A3 board 이득은 3 mm부터
translation/rotation 모두 반전되어, board가 잘못된 FK pose를 보정하지 못하고 오히려 결합
bias를 만들 수 있음을 보인다.

### Fig. A — 코너 노이즈 강건성

- 검출된 2D corner에 독립 Gaussian noise `N(0, σ²)`를 x/y 각각 주입
- x축 σ(px), y축 `e_X`; translation/rotation은 별도 subplot 권장
- A2/A3/B1/B2 네 곡선, A3 강조, 반복 실험 편차 band
- 실제 검출 오차 범위와 2 px stress condition을 본문에서 구분

### Fig. B — FK 노이즈 강건성

- joint angle에 `N(0, σ_fk²)`를 주입한 뒤 FK를 다시 계산
- A2와 A3 모두 hand-eye 정합용 백본 FK를 사용하므로 둘 다 joint/FK noise의 영향을 받는다.
- A3에는 cube pose를 고정하는 `FK→cube` 경로가 추가되므로 A2 대비 추가 민감도를 측정한다.
- A2를 수평선 또는 “FK noise 독립”으로 미리 가정하지 않고 실제 곡선과 반복실험 CI로 판단한다. The you go. The power sippy there's the body open there. They saw liberation independent unified no it's contesting control
- 두 곡선의 교차점이 관측되는 경우에만 “cube pose를 FK-fixed로 둘 수 있는 범위” 후보로 해석한다.
- 현재는 joint noise→Cartesian pose error 전파와 실물 FK 정확도 추정법이 구현되지 않음

## Table 2b — METRIC GT 데이터의 적용 가능성

이 절은 Table 1의 시뮬레이션이 아니라 **외부 데이터셋을 쓸 수 있는지에 대한 판정 기록**이다.
Table 1의 합성 대응은 위 Table 2a다.

저장소의 `medium_workcell` METRIC 자료는 4대 eye-on-base 카메라와 checkerboard만 있고
`calibration_setup: 1`이다. cube, eye-in-hand camera, `FK→cube` pose가 없어 A0–B3의
Marker/U/FK factorial을 물리적으로 만들 수 없다. 따라서 위 Table 2a custom simulator
결과를 “METRIC 결과”로 부르지 않는다.

호환 가능한 별도 실험은 [`CP_metric_board_only.py`](../CP_metric_board_only.py)로 실행했다.
checkerboard 검출 수는 cam1–4 각각 52/99/90/72 views이고 robot pose는 251개다. 아래 값은
번들 `GT/gt_cam*.csv`를 `T_base_cam`으로 해석한 뒤 4개 카메라 SE(3) 오차의 RMS다.
Classical 방법은 카메라별 OpenCV hand-eye이고, joint 방법은 문서화된 eye-on-base 변환 체인과
공유 `T_gripper_board`를 corner reprojection으로 푼 Python/SciPy **호환 재구현**이다.

| Method | 상태 | e_t RMS (mm) | e_r RMS (°) | train reproj (px) | runtime (s) |
| --- | --- | ---: | ---: | ---: | ---: |
| Tsai-Lenz | 수렴 | 1557.6214 | 63.3231 | 307.5653 | 0.176 |
| Park-Martin | 수렴 | 173.2694 | 2.4656 | 37.7054 | 0.171 |
| Horaud | 수렴 | 172.1610 | 2.1532 | 36.4326 | 0.178 |
| Daniilidis | 수렴 | 367.1478 | 3.3385 | 73.9186 | 0.177 |
| Joint corner reprojection (compatible) | 수렴, rank 30/30 | **0.9474** | **0.0626** | **0.2407** | 13.695 |

Classical aggregate 오차는 cam1의 실패성 해에 크게 지배되며, per-camera raw 값은 JSON에
보존했다. Joint 결과는 외부 GT이므로 real-session FK-proxy와 달리 physical transform accuracy로
해석할 수 있지만, 이 데이터는 board-only라 A0–B3나 Ours(A3)의 결과가 아니다. 번들 Allegro
C++/Ceres 실행기는 현재 환경에 `cmake`가 없어 빌드하지 못했으므로 joint 결과를 원본 바이너리
실행값으로 표기하지 않는다. 산출물은 [`metric_board_only/metric_board_only.md`](metric_board_only/metric_board_only.md),
[JSON](metric_board_only/metric_board_only.json), [CSV](metric_board_only/metric_board_only.csv)에 있다.

## [*DEMO] Table 3 — Task-level (Pick and Place or peg in hole or 다트?) 물리 실행 : Soon ..........

> **상태: 보류 TO_DO (나중에 해야할 일).** 이 표는 실물 장비와 독립적인 외부 측정법을 준비한 뒤 실행한다. 그전에는 값을 채우지 않으며, FK-proxy agreement를 물리적 정확도로 대체하지 않는다.

task 종류는 pick-and-place/peg-in-hole/dart 중 아직 확정되지 않았다. 서로 다른 task를 섞지 말고 하나의 성공 판정과 오차 측정법을 결과 확인 전에 고정해야 한다.

| 구성 | 성공률 (n=20) | 안착 오차 mean±std (mm) | max (mm) |
| --- | ---: | ---: | ---: |
| A0 baseline | 미측정 | 미측정 | 미측정 |
| A2 cube pose vision-estimated | 미측정 | 미측정 | 미측정 |
| B2 cube only | 미측정 | 미측정 | 미측정 |
| **A3 Ours** | 미측정 | 미측정 | 미측정 |

### DEMO 하기 전 체크사항

- [ ] 하나의 물리 task를 선택한다: pick-and-place, peg-in-hole, dart 중 하나.
- [ ] 로봇 FK 및 평가 대상 카메라와 독립적인 안착 오차 측정법과 기준 좌표계를 정한다.
- [ ] 성공/실패 판정 임계값, 측정 정밀도, `max`의 정의를 결과 확인 전에 고정한다.
- [ ] `n=20`의 단위를 확정한다. 권장안은 **구성별 20회(A0/A2/B2/A3, 총 80회)**이며 실패 trial도 제외하지 않는다.
- [ ] 구성별 실행 순서의 무작위화 또는 교차 배치 방식과 재보정 주기를 정한다.
- [ ] 시작 pose, target 위치, 속도, 경로, gripper 설정 등 구성 외 조건을 동일하게 고정한다.
- [ ] trial별 구성, 순서, 목표 pose, 측정 pose, 성공 여부, 실패 사유, calibration artifact ID를 저장하는 로그 형식을 만든다.
- [ ] 위 프로토콜을 먼저 동결한 뒤 물리 실행을 수행하고 Table 3을 채운다.

완료 기준은 구성별 유효한 20회 로그가 모두 존재하고, 실패를 포함한 성공률과 독립 외부 측정 기반의 mean±std/max를 재계산할 수 있는 것이다. 외부 GT가 준비되지 않으면 Table 1의 `e_task_pose`도 비워 두거나 명시적으로 `FK-proxy`라고 표시하며, physical accuracy로 해석하지 않는다.

## Table 4 — 민감도

[`CP_sensitivity_7row.py`](../CP_sensitivity_7row.py)로 추가 수집 없이 실행했다. `N`은 고정된 5개 위치 `[0,2,6,9,12]`에 균등 배분한 **전체 train capture-event 수**이고, 같은 seed에서는 `N=5⊂10⊂20⊂40`이다. 카메라 수는 손목 카메라를 포함하며 고정 카메라를 `[0]⊂[0,1]⊂[0,1,3]` 순서로 추가한다. 모든 조건은 동일한 native-resolution held-out event와 공통 카메라 `eih+cam0`에서 평가했다.

해상도 `낮음`은 저장 영상의 0.5× downsampling에서 검출을 다시 수행한 조건이고 `높음`은 native 영상이다. 0.5× 검출 좌표는 native pixel frame으로 되돌린 뒤 같은 K·loss를 사용하므로 표의 px 단위가 같다. 이는 센서 재촬영 해상도 실험이 아니다. 셀은 5개 subset seed의 held-out overall corner-reprojection RMSE mean±population-std (px)이며, 모든 반복이 수렴하지 않으면 숫자를 headline에서 제거했다.

| 조건 | A0 | A2 | B2 | **A3** |
| --- | ---: | ---: | ---: | ---: |
| views N=5 | 2.8353±1.3884 | **수렴 불안정 (4/5)** | 14.1248±0.5165 | 7.4812±0.2169 |
| N=10 | 1.5127±0.1749 | 7.1463±0.0754 | 13.6974±0.0611 | 7.2616±0.0324 |
| N=20 | 1.5826±0.0868 | 7.1287±0.0209 | 13.6668±0.0279 | 7.2386±0.0073 |
| N=40 | 1.5038±0.0206 | 7.1122±0.0034 | 13.6436±0.0037 | 7.2220±0.0026 |
| cams=2 | 1.5038±0.0206 | 6.5995±0.0038 | 13.6436±0.0037 | 7.2232±0.0028 |
| cams=3 | 1.5038±0.0206 | 7.0982±0.0040 | 13.6436±0.0037 | 7.2220±0.0026 |
| cams=4 | 1.5038±0.0206 | 7.1122±0.0034 | 13.6436±0.0037 | 7.2220±0.0026 |
| 해상도=낮음 (0.5×) | **고정카메라 board 초기화 불가 (0/5)** | 7.5949±0.0231 | 13.8852±0.0049 | 7.3823±0.0078 |
| 해상도=높음 (native) | 1.5038±0.0206 | 7.1122±0.0034 | 13.6436±0.0037 | 7.2220±0.0026 |

위 표의 overall 값은 행마다 target set이 달라 **행 내부 민감도**만 비교할 수 있고, 행 간 정확도 순위로 읽으면 안 된다. 최우선 판정인 B2 대 A3는 공통 cube corner만 사용하면 다음과 같다. Δ는 `A3−B2`이므로 음수가 board 추가에 유리하다.

| 조건 | B2 cube (px) | A3 cube (px) | paired Δ (px), A3 개선 seed |
| --- | ---: | ---: | ---: |
| N=5 | 14.1248±0.5165 | 13.9689±0.3604 | −0.1558±0.1654, 4/5 |
| N=10 | 13.6974±0.0611 | 13.7279±0.0444 | +0.0305±0.0356, 1/5 |
| N=20 | 13.6668±0.0279 | 13.6975±0.0250 | +0.0307±0.0083, 0/5 |
| N=40 | 13.6436±0.0037 | 13.6809±0.0034 | +0.0373±0.0013, 0/5 |

따라서 N=5에서는 board 추가가 작은 평균 이득을 보이지만 효과 크기보다 subset 변동이 크고 5/5 일관적이지 않다. N≥10에서는 이득이 재현되지 않는다. 현재 데이터는 “board가 적은 뷰에서 항상 수렴 또는 정확도를 향상한다”는 강한 주장을 지지하지 않는다. A2의 N=5 실패 seed는 Jacobian full-rank(60/60)였지만 `max_nfev=300`에 도달했고, 이는 gauge failure가 아니라 저N 최적화 불안정으로 보고한다. 0.5×에서 A0는 모든 seed에서 고정 카메라 board 검출이 없어 등록할 수 없었으며, 이 역시 숫자로 대체하지 않는다.

전체 산출물은 [`sensitivity_7row/sensitivity.md`](sensitivity_7row/sensitivity.md), [JSON](sensitivity_7row/sensitivity.json), [CSV](sensitivity_7row/sensitivity.csv)에 있다.

## Table 5 — Calibration solver 원리 4가지

[`CP_solver_01_04.py`](../CP_solver_01_04.py)로 동일 train sets `1,2,3,5,7,8,9,10,11`과 held-out sets `0,4,6,12`에서 강제 실행했다. 03과 04는 byte-identical 02 초기값(SHA-256 `4706ca48…`)을 사용하며, 04의 pose regularizer는 0이고 production fallback/adoption guard를 끈다. 1차 지표는 held-out 한 카메라에서 측정한 cube PnP pose를 다른 카메라 corner로 전이한 coordinate RMSE다. 평가 unit은 fitting 전에 measurement availability만으로 고정했다.

| 방식 | 계산 방법 | 상태 | held-out transfer (px) | e_cross (mm/°) | train reproj (px) | runtime (s) |
| --- | --- | --- | ---: | ---: | ---: | ---: |
| 01 `pnp_mean` | 프레임별 PnP pose 단순 평균 | closed-form | 37.9321 | 67.938 / 59.601 | 16.7912 | 0.008 |
| 02 `pnp_robust_se3` | robust SE(3) 평균 | closed-form | 29.8026 | 57.162 / 61.640 | 9.3070 | 0.031 |
| 03 `pose_consistency` | PnP pose의 SE(3) 일치 최소화 | 수렴, 강제 후보 사용 | 29.8609 | 57.259 / 61.602 | 7.4401 | 167.016 |
| 04 `reprojection` | 3D corner의 순수 pixel reprojection 최소화 | 수렴, 강제 후보 사용 | **29.4867** | **56.610** / 61.695 | **5.5139** | 265.376 |

공통 mask는 ordered transfer 264개, unordered cross pair 132개다. 외부 GT가 없으므로 이는 multi-view agreement이지 absolute camera-pose accuracy가 아니다. 04는 held-out transfer가 가장 낮지만 02 대비 개선은 0.3159 px로 작다. 반면 train reprojection은 9.3070→5.5139 px로 크게 감소한다. 따라서 production에서 `04 기본/03 fallback`이라는 정책이나 train loss 감소만으로 04의 일반화 우위를 크게 주장하면 안 된다. 03은 optimizer가 수렴했지만 squared diagnostic cost가 증가했고, fallback을 막은 강제 비교에서는 02보다 held-out이 0.0583 px 나빴다.

산출물은 [`solver_01_04/solver_01_04.md`](solver_01_04/solver_01_04.md), [JSON](solver_01_04/solver_01_04.json), [CSV](solver_01_04/solver_01_04.csv)에 있다.

## Table 6 — SOTA 비교

원문의 서로 다른 수치를 하나의 `e_task_pose` 열에 넣는 설계는 폐기했다. 이 표에서
`learning-free`는 **calibration inference에 학습·pretrained model을 전혀 쓰지 않는 strict 정의**,
`multi-cam`은 한 calibration system이 2대 이상 카메라를 다룰 수 있다는 뜻이다.
`joint eye-in+to`만 고정 카메라와 손목 카메라가 **동일 목적함수**에 함께 들어가는지를 뜻한다.
따라서 두 구성을 각각 지원하거나 순차로 푸는 방법은 `✗`다.

| Method | learning-free (strict) | multi-cam | joint eye-in+to | 보고값 | 평가 정의 / GT | views | runtime (s) |
| --- | :---: | :---: | :---: | --- | --- | --- | ---: |
| Independent + rigid align (A0) | ✓ | ✓ | ✗ | 1.4387±0.0995 px‡ | 본 real data held-out corner RMSE; 외부 GT 없음 | canonical runtime split: 172 train capture events‡ | 1.864‡ |
| Tabb & Yousef 2017 | ✓ | ✓ | ✗ | 5.79 mm / 0.36°* | Allegro 재현의 METRIC synthetic medium, camera-pose external GT 평균 | full METRIC protocol* | 87.92* |
| Allegro et al. 2024 | ✓ | ✓ | ✗ | 0.75 mm / 0.02°* | METRIC synthetic medium, camera-pose external GT 평균 | full METRIC protocol* | 13.78* |
| EasyHeC++ | ✗ | ✗ | ✗ | 1.35 mm / 0.045°* | xArm synthetic eye-to-hand, camera-pose GT, 5-view 값 | 5* | — |
| Li et al. 2024 | ✗ | ✗ | ✗ | 0.930 mm / 0.265°* | real eye-in-hand **구성 간 repeatability**(평균 표준편차); external-GT accuracy가 아님 | 25 point clouds/config* | <1 inference + 5 move* |
| **Ours (A3)** | ✓ | ✓ | **✓** | 4.6230±0.4338 px‡ | 본 real data held-out corner RMSE; 외부 GT 없음 | canonical runtime split: 172 train capture events‡ | 8.993‡ |

`‡` runtime은 canonical fixed split의 optimizer-only 시간(3 initialization 평균)이라 검출·artifact
생성 시간을 포함하지 않는다. A0/A3의 px와 외부 논문의 mm/°는 같은 물리량이 아니므로 숫자로
순위를 만들 수 없다. Ours의 mm/° SOTA 셀은 독립 외부 GT를 수집할 때까지 비운다.

원문 사실 검증:

- [Tabb & Yousef](https://arxiv.org/abs/1907.12425)는 학습 없이 robot-world hand-eye를 풀고
  2·3대 카메라를 같은 로봇에 장착한 hand-multiple-eye로 확장한다. 고정+손목 카메라 혼합
  objective는 아니다. 표의 METRIC 숫자는 원 논문 숫자가 아니라 아래 Allegro 논문의 재현값이다.
- [Allegro et al.](https://arxiv.org/abs/2406.11392)는 로봇에 부착한 board를 여러 **고정** 카메라가
  동시에 보고, 공통 board-to-end-effector 및 camera-to-camera 제약을 최적화한다. METRIC
  synthetic medium의 원문값은 0.75 mm/0.02° 및 13.78 s다. `<10 images`는 ABB real
  workcell의 별도 내부일관성 실험 주장이고 이 GT 숫자의 view 수가 아니다.
- [EasyHeC++](https://arxiv.org/abs/2410.09293)는 per-arm 재학습은 하지 않지만 GroundedSAM/SAM과
  pretrained dense feature matcher를 사용하므로 strict 정의에서는 learning-free가 아니다.
  1.35 mm/0.045°는 real task 오차가 아니라 synthetic xArm eye-to-hand의 5-view camera-pose GT다.
- [Li et al.](https://arxiv.org/abs/2311.01335)은 학습 기반 robot-base detection·PREDATOR registration과
  단일 low-cost structured-light camera를 사용한다. 0.930 mm/0.265°는 여섯 joint configuration,
  세 재장착 group에서 얻은 calibration 결과의 평균 표준편차다. 이를 absolute calibration error나
  multi-camera 결과로 인용하면 안 된다.
- 저장소의 METRIC compatible joint 재구현 0.9474 mm/0.0626°는 원본 Allegro C++ 실행값이 아니며,
  위 논문 보고값을 대체하지 않는다.

## 남아 있는 작업

- **Task-level 물리 실행 — 보류 TO_DO:** Table 3의 체크리스트에 따라 하나의 task와 독립 외부
  측정법, 성공·오차 정의, 실행 프로토콜을 먼저 확정한 뒤 구성별 20회를 수집한다. 준비 전에는
  Table 3의 결과와 physical-accuracy 값을 비워 둔다.
