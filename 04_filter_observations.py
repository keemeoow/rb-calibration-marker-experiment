#!/usr/bin/env python3
"""04. 저장된 영상을 다시 검출해 관측 품질을 판정하고 corner를 동결(freeze)한다.

역할
----
03단계는 촬영만 한다. 여기서 영상을 **다시** 검출해 05단계가 쓸 2D corner를 확정하고,
그 목록을 해시로 고정한다. 촬영 중 실시간 검출 결과는 쓰지 않는다 — 검출기 설정이
바뀌면 결과가 달라지므로, 캘리브레이션 입력은 항상 이 단계의 manifest 하나로만
결정되게 만든 것이다. 원본 촬영 데이터는 절대 수정하지 않는다.

원리 / 수식
-----------
표적별로 PnP를 풀어 카메라-표적 자세를 얻는다. 3D 표적점 X_k와 검출된 2D corner
x_k에 대해

    T^C_O = argmin_T  Σ_k || π(K, D, T, X_k) - x_k ||^2

    잔차 RMSE = sqrt( (1/N) Σ_k || π(K, D, T, X_k) - x_k ||^2 )   [px]

이 RMSE는 "이 관측이 강체 표적 모델과 얼마나 맞는가"를 재는 값이다. 값이 크면
검출이 틀렸거나(코너 오검출), 표적 모델이 틀렸거나(치수/부착 오류), 영상이 흐린
것이므로 그 관측을 05단계에 넣으면 안 된다.

큐브 관측의 core 조건 (이 중 하나라도 어기면 비핵심으로 분류)
    - PnP 자체가 수렴할 것
    - 서로 다른 면이 2개 이상 보일 것          (observed_face_count >= 2)
    - 그 면들이 동일 평면이 아닐 것            (noncoplanar_face_count >= 2)
    - 평면 축퇴가 아닐 것                      (is_planar == False)
    - 양의 깊이 해가 하나 이상 있을 것

한 면만 보이면 표적점이 모두 한 평면에 놓여 PnP가 평면 축퇴에 빠진다. 이때 자세는
겉보기 재투영오차가 낮아도 깊이 방향과 기울기가 사실상 정해지지 않는다(평면
호모그래피의 이중해). 그래서 낮은 RMSE만으로 통과시키지 않고 **기하 조건을 먼저**
본다.

두 가지 선별 정책 (기본값)
    standard : cube RMSE <= 3.0 px, inlier >= 0.0, board corner >= 4
    strict   : cube RMSE <= 2.0 px, inlier >= 0.9, board corner >= 12

판정 결과는 selected / recovered / quarantine / rejected 로 나뉜다. quarantine은
"검출은 됐지만 기준 미달"이라 재촬영 후보로 남고, rejected는 검출 자체 실패다.

입력 / 처리 / 출력
------------------
입력: 촬영된 calib_train 디렉터리와 02단계의 고정카메라 intrinsics.
처리: 큐브/보드 corner 재검출 -> PnP -> 품질 등급 분류 -> 원본 영상 해시 기록.
출력: capture_filter manifest(관측 동결본), 검토용 CSV, 재촬영 후보 목록, 오버레이.

구현 위치
---------
    calibration_pipeline/filter_observations.py
      main() / run_filter()      - 전체 파이프라인 지휘
      _cube_records()            - 큐브 재검출과 PnP, 면 개수/평면성 진단
      _board_records()           - ChArUco 재검출과 corner 수집
      _core_support()            - 위 "core 조건" 판정 (평면 축퇴 방어)
      _cube_policy_decision()    - RMSE/inlier 임계 적용, 탈락 사유 문자열 생성
      _disposition()             - selected / recovered / quarantine / rejected 분류
      _event_summaries() / _summary() - 이벤트·전체 집계
      _retake_rows()             - 재촬영 후보 산출
      _image_provenance() / _sha256() / _canonical_sha256()
                                 - 원본 영상과 manifest의 출처 해시 고정
      _draw_review_overlay()     - 사람이 눈으로 검토할 오버레이 생성
      _write_csv() / _write_readme() - 검토용 산출물 기록

    이 파일은 위 모듈의 main() 을 그대로 노출하는 얇은 진입점이다.

실행 예
-------
    python3 04_filter_observations.py \\
      --session-root data/session04/calib_train \\
      --intrinsics-dir intrinsics
"""

# calibration_pipeline/filter_observations.py 의 main() 을 그대로 사용한다.
# (인자 파서와 run_filter() 호출이 모두 그쪽에 있다)
from calibration_pipeline.filter_observations import main


if __name__ == "__main__":
    main()
