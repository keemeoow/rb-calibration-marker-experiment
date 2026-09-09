#!/usr/bin/env python3
"""06. 05단계 결과에서 최종 변환행렬과 요약 CSV를 뽑아낸다.

성격
----
이 단계는 **아무것도 새로 추정하지 않는다.** 최적화도, 재적합도, held-out 점수에
따른 선택도 없다. 05단계가 남긴 ABLATION_TEST_table1_methods.json을 읽어 그대로 옮겨 적고
집계만 한다. 보고 단계에서 값이 바뀔 여지를 없애려고 일부러 이렇게 분리했다.

대표 seed는 `--representative_seed`(기본 0)로 **고정**되며, held-out 성적이 가장
좋은 seed를 고르지 않는다. 성적으로 seed를 고르면 그 순간 held-out이 선택에
쓰인 것이 되어 더 이상 held-out이 아니게 되기 때문이다. 수렴/prune 통계는
3 seed 전체에 대해 평균과 표준편차로 보고한다.

내보내는 변환행렬의 의미
------------------------
모두 4x4 SE(3)이고 이동 성분 단위는 미터다.

    T^B_Ci      고정카메라 i 좌표계 -> 로봇 base 좌표계.
                실제 배포에 쓰이는 산출물. 점 변환은  X_base = T^B_Ci · X_cam.

    T^G_C       손목(그리퍼) 카메라 좌표계 -> 그리퍼 좌표계. hand-eye 변환이며
                역시 배포 산출물이다. 이벤트별 base 자세는 FK와 합성해 얻는다.

                    T^B_C(e) = T^B_G(e) · T^G_C

    T^B_board   보드 좌표계 -> base. 최적화된 **표적 자세**이며 카메라 캘리브레이션
                산출물이 아니다. 그 세션의 보드가 어디 있었는지를 뜻할 뿐이다.

    T^B_cube(s) set s의 큐브 자세 -> base. 마찬가지로 표적 자세이고, 행에 따라
                추정된 값일 수도 있고 고정된 값일 수도 있다(A3/A5는 고정).

앞의 두 개만 다른 세션으로 가져다 쓸 수 있고, 뒤의 두 개는 그 세션 안에서만
의미가 있다 — 이 구분이 MATRIX_SEMANTICS에 문자열로 함께 기록된다.

무결성 검사
-----------
기록 전에 다음을 확인하고, 어긋나면 예외를 던져 산출물을 만들지 않는다.
  - 행 집합이 정확히 A0~A5, B1~B3 인지
  - 각 행에 run이 존재하고 대표 seed가 그 안에 있는지
  - 각 run이 T_base_Ci 와 T_gripper_cam 을 실제로 갖고 있는지

입력 / 처리 / 출력
------------------
입력: 05단계가 만든 ABLATION_TEST_table1_methods.json.
처리: 수렴/frame-prune 결정 요약, 대표 seed의 최종 변환행렬 복사.
출력: 같은 디렉터리에 calibration_summary.csv, calibration_matrices.json.

구현 위치
---------
    calibration_pipeline/report.py
      main() / parse_args()   - 경로 기본값 해석 (ABLATION_TEST_result/<session>/ABLATION_TEST_table1)
      write_report()          - 검증 -> 행 요약 -> CSV/JSON 기록
      _validate()             - 위 "무결성 검사"
      _row_summary()          - 행 하나의 수렴·prune·지표 요약
      _matrix_artifact()      - 대표 seed의 변환행렬 + 의미 문자열 묶음 생성
      _frame_prune_records()  - prune 적용/롤백 이력 추출
      _mean_std() / _numbers()- seed 간 평균과 표준편차
      METHOD_ORDER            - 행 순서 고정
      MATRIX_SEMANTICS        - 위에 설명한 각 행렬의 의미 문자열
      CANONICAL_LABEL_OVERRIDES - A3/A5/A4의 표시 이름 고정

    calibration_pipeline/runtime.py
      DEFAULT_SESSION_ROOT, session_paths() - 세션 경로 규칙

    이 파일은 calibration_pipeline/report.py 의 main() 을 노출하는 진입점이다.

사람이 읽는 비교표는 여기서 만들지 않는다
------------------------------------------
ABLATION_TEST_TABLE1_RESULTS.md 는 tools/sync_table1_canonical_data.py 가 같은 JSON에서
생성한다. 결과 원천을 하나로 두려고 러너·리포트·문서 생성을 분리해 놓은 것이다.
"""

# calibration_pipeline/report.py 의 main() 을 그대로 사용한다.
# (인자 파서와 write_report() 호출이 모두 그쪽에 있다)
from calibration_pipeline.report import main


if __name__ == "__main__":
    main()
