#!/usr/bin/env python3
"""05. Table 1의 모든 실행 가능 조건(A0~A5, B1~B3)을 한 러너로 적합한다.

핵심 아이디어
-------------
모든 행이 **같은 목적함수 하나**를 풀고, 행마다 달라지는 것은 오직 두 가지다.

  (1) 어떤 관측을 넣는가            (board / cube / 둘 다)
  (2) 어떤 변수를 자유롭게 두는가   (freeze mask)

그래서 "구조가 달라서 좋아졌다"와 "코드가 달라서 좋아졌다"를 구분할 수 있다.

목적함수 / 수식
---------------
잔차는 오직 한 종류, **한 corner의 재투영 오차**다. 카메라-카메라 잔차도, 표적
자세에 대한 사전 잔차도 없다.

관측 하나(카메라 c, 이벤트 e, 표적 O, corner k)에 대해

    r_k = π( K_c, D_c, (T^B_{C_c}(e))^{-1} · T^B_O ,  X_k )  -  x_k       [px]

여기서 카메라의 base 자세 T^B_{C_c}(e)는 카메라 종류에 따라 다르게 조립된다.

    고정카메라 i      :  T^B_{C_i}                       (이벤트와 무관한 상수)
    그리퍼카메라 g    :  T^B_{C_g}(e) = T^B_G(e) · T^G_{C_g}
                         T^B_G(e)는 로봇 FK로 주어지는 **고정 입력**이며 추정하지 않는다

표적의 base 자세 T^B_O는 board면 T_base_board, cube면 set별 T_base_cube_by_set이다.

전체 문제는 robust 최소자승이다. SciPy TRF, soft_l1, f_scale = 2 px.

    minimize_θ  Σ_k  ρ( r_k )

    z = (r / f)^2 ,  soft_l1:  ρ(z) = 2( sqrt(1+z) - 1 )
    cost = 0.5 · f^2 · Σ ρ(z)          (SciPy와 동일한 1/2 계수 포함)

loss='linear'이면 같은 잔차가 통상적인 최소자승이 된다. soft_l1은 큰 잔차의
영향력을 sqrt로 눌러, 코너 오검출 하나가 전체 해를 끌고 가는 것을 막는다.

SE(3) 변수는 접평면 파라미터화로 갱신한다. 회전은 3-벡터 rotvec, 이동은 3-벡터이며,
스케일이 다른 두 양을 같은 신뢰영역에서 다루기 위해 x_scale='jac'을 쓴다.

행(row)별 자유변수 — freeze mask
--------------------------------
    A2 / A4 : T_base_Ci, T_gripper_cam, T_base_board, T_base_cube_by_set
    A3 / A5 : T_base_Ci, T_gripper_cam, T_base_board      (cube 자세는 상수로 고정)
    B2      : T_base_Ci, T_gripper_cam, T_base_cube_by_set (board 없음)
    B3      : T_base_Ci, T_gripper_cam, T_base_board       (cube 없음)
    A0 / A1 / B1 : 순차(sequential) 2단계. stage1에서 eye-in-hand를,
                   stage2에서 eye-to-hand를 풀며 앞 단계 결과를 고정한다.

A3는 큐브 자세를 컨트롤러 raw FK로, A5는 train 영상으로 정렬한 FK로 **하드 고정**한다.
고정이란 상태에는 존재하되 자유변수 목록에서 빠진다는 뜻이고, 잔차 항이 따로 생기지
않는다. A4/B1/B2는 반대로 FK를 soft factor로 넣어 잔차 블록을 추가한다.

절차
----
  1. 이벤트 단위 train/held-out 분할 (split seed 고정)
  2. 모든 행이 공유하는 초기 상태 하나를 만든다 (shared_reference_state)
  3. 행별 선언된 처리만 적용해 초기 상태를 특수화한다
  4. 적합 -> frame-prune -> 재적합 -> 개선 없으면 롤백
  5. held-out 평가: 카메라/hand-eye를 동결한 채 train cube로 set별 평가 자세만
     맞춘 뒤, 사용하지 않은 held-out corner를 재투영한다

  4의 prune은 잔차가 큰 (event, camera) 프레임을 떼고 다시 맞춘 뒤, 전체 robust
  목적함수가 나아지지 않으면 되돌린다. 좋아 보이는 프레임만 남기는 선택이 되지
  않도록 판단 기준을 최종 목적함수 값에 묶어 둔 것이다.

  seed 3개로 반복하며, 보고값은 3회의 평균이다(최솟값이 아니다).

평가 지표
---------
    RMSE_px = sqrt( (1/2N) Σ_k ( (u_k - û_k)^2 + (v_k - v̂_k)^2 ) )

    분모가 2N인 것은 corner 하나가 u, v 두 개의 스칼라 잔차를 만들기 때문이다.

입력 / 처리 / 출력
------------------
입력: 04단계의 동결된 관측 manifest, intrinsics, 촬영 meta, 로봇 FK.
처리: 위 1~5.
출력: table1_methods.json, shared_train_only_baseline.json,
      shared_board_free_fk_cube.json (모두 해시로 출처 고정).

구현 위치
---------
    calibration_pipeline/table1.py            - 이 단계의 본체
      main()                          - 인자 해석, 준비, 행×seed 루프
      prepare_ablation_data()         - 관측 로드, event 단위 split, FK 정렬 산출물 생성
      build_shared_reference_state()  - 모든 행이 공유하는 초기 상태 하나를 구성
      make_initial_state()            - 행별 freeze/고정 자세 특수화
      run_condition_once()            - 한 행 한 seed 적합 (unified / sequential)
      run_factor_condition_once()     - FK soft factor 행(A4/B1/B2) 전용 경로
      fit_train_only_cube_evaluation()- held-out cube 평가용 자세 적합
      canonical_solver_options()      - 전 행 공통 solver 설정
      write_outputs()                 - 계약 검증 후 JSON 기록

    calibration_pipeline/reprojection.py      - 목적함수와 최적화 엔진
      solve_corner_reprojection()     - 위 minimize 식을 SciPy로 푸는 지점
      project_points()                - π(·). cv2.projectPoints 래퍼
      robust_least_squares_cost()     - soft_l1 / huber / linear 비용 계산
      PixelObs                        - 관측 하나(표적, 카메라, 이벤트, 3D/2D 점)

    calibration_pipeline/schema.py            - 행 정의와 계약
      UNIFIED_FREE_VARIABLES          - 행별 자유변수 목록(freeze mask)
      SEQUENTIAL_STAGE_SPECS          - 순차 행의 stage별 자유변수
      RAW_FK_CUBE_CENTER_TO_OBJECT    - A3용 사전등록 기계 좌표 변환(추정 아님)

    calibration_pipeline/fk_alignment.py      - A5/A4/B1/B2가 공유하는 FK 정렬 산출물
      estimate_board_free_fk_cube_artifact()
                                      - board 없이 train eye-in-hand cube corner만으로
                                        T^G_Cg와 FK-큐브 델타를 함께 추정.
                                        held-out 이벤트 사용 시 예외를 던진다.

    calibration_pipeline/fk_factor.py         - FK를 soft factor로 넣는 잔차 블록
    calibration_pipeline/observations.py      - manifest -> PixelObs 변환
    calibration_pipeline/evaluation.py        - 재투영 지표 집계
    calibration_pipeline/path_evaluation.py   - cross-view / cam-common 일관성 지표

    이 파일은 calibration_pipeline/table1.py 의 main() 을 노출하는 진입점이다.

주의
----
최종 물리 정확도 판정은 이 단계의 픽셀 지표가 아니라 독립 External cube GT로 한다.
여기서 나오는 held-out RMSE는 GT 이전의 내부 보조 지표다.
"""

# calibration_pipeline/table1.py 의 main() 을 그대로 사용한다.
# (인자 파서, 준비 단계, 행×seed 루프가 모두 그쪽에 있다)
from calibration_pipeline.table1 import main


if __name__ == "__main__":
    main()
