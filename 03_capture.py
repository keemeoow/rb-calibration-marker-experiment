#!/usr/bin/env python3
"""03. 여러 카메라의 RGB-D와 로봇 상태를 이벤트 단위로 동기 촬영한다.

이 단계가 만드는 것
-------------------
이후 모든 계산의 원재료는 "한 이벤트 e에서 동시에 성립하는 다음 세 가지"다.

  1. 각 카메라 c의 color/depth 영상
  2. 그 순간의 로봇 순기구학 자세      T^B_G(e)   (base -> gripper)
  3. 그 이벤트가 속한 set 번호와 큐브 배치 정보 (set_cube_center_6dof)

05단계의 미지수가 풀리는 이유가 여기에 있다. 고정카메라 경로와 그리퍼카메라 경로가
같은 표적을 동시에 보면 두 경로가 하나의 닫힌 루프를 이룬다.

    고정카메라 경로:    T^B_O = T^B_Ci  ·  T^Ci_O(e)
    그리퍼카메라 경로:  T^B_O = T^B_G(e) · T^G_Cg · T^Cg_O(e)

    두 식을 같다고 놓으면

        T^B_Ci · T^Ci_O(e)  =  T^B_G(e) · T^G_Cg · T^Cg_O(e)

    이는 hand-eye 문제의 표준형 A X = X B 와 같은 구조이며, T^G_Cg(hand-eye)와
    T^B_Ci(고정카메라 외부 파라미터)를 함께 결정한다. 자세 T^B_G(e)가 서로 다른
    회전축을 갖는 이벤트가 여럿 있어야 X가 유일하게 정해진다 — 그래서 촬영은
    "여러 자세"가 핵심이지 "많은 장수"가 핵심이 아니다.

측정값은 여기서 픽셀로만 저장되고, 자세 추정은 04/05단계에서 한다. 이 단계에서
검출한 마커 pose는 **진단용**이며 캘리브레이션에 쓰이지 않는다.

두 가지 촬영 모드
-----------------
composite_rig_45_v1 (최종 프로토콜)
    사전 검증된 pose plan을 로봇 서버로 보내고 P1 15 / P2 20 / P3 10, 합 45개
    planned event를 순서대로 자동 촬영한다. 마커 품질은 기록만 하고, 전송/동기
    실패만 같은 event ID로 재시도한다. 품질을 이유로 재촬영하면 표적이 잘 보이는
    자세만 남아 표본이 편향되므로 일부러 분리해 둔 것이다.

legacy
    로봇이 큐브를 놓고 `set`을 실행해 set 기준 자세를 저장한 뒤, 같은 set에서
    그리퍼 카메라를 여러 자세로 옮기며 촬영한다.

입력 / 처리 / 출력
------------------
입력: 02단계의 intrinsics/, 카메라들, 큐브/보드, 로봇 서버, (선택) waypoint 파일.
처리: 이벤트마다 모든 카메라를 동시에 그랩하고 로봇/릴리스 상태를 함께 기록하며,
      영상 해시로 전송 무결성을 검사한다.
출력: RGB-D 영상, meta.json(이벤트별 자세·set·검출 진단), 기록된 waypoint,
      프로토콜 완료 manifest.

구현 위치
---------
    capture_pipeline/capture.py
      main()                        - 인자 해석, 카메라/로봇 연결, 촬영 루프
      wait_for_start_command_capture() - 서버 start 신호 대기 및 이벤트 진행
      load_device_map()             - serial -> cam_idx, gripper_cam_idx 로드
      load_intrinsics()             - cam{idx}.npz 에서 (K, D, depth_scale) 로드
      estimate_per_marker_poses()   - 마커별 PnP (진단 표시용, 캘리브레이션 미사용)
      evaluate_transport_integrity()- 프레임 누락/불일치 검사
      load_and_validate_rig_geometry() - rig 형상 파일과 rig_id 일치 확인
      canonical_json_sha256() / file_sha256() - meta와 영상의 출처 해시 고정
      make_quad_image() / annotate_image() / append_status_footer() - 화면 피드백
      make_capture_gate_config() / build_capture_gate_lines() - 촬영 게이트 표시

import를 main() 안에 두는 이유: pyrealsense2와 로봇 서버 의존성을 실행 시점까지
미루기 위해서다.

서버 쪽 조작 (참고)
-------------------
    python c1.py --auto pc --speed 30
    [set 0 z+100]  gotoj 37.96, -9.45, -136.81, 0.25, -33.05, -117.92
    p z,-100 / gc
    [자동 촬영] start        [티칭] rs(set) / rp(pose) -A / rg(grip) -B
"""

def main() -> None:
    # capture_pipeline/capture.py 의 main() 을 그대로 실행한다.
    # (지연 import: pyrealsense2 / 로봇 서버 의존성을 실행 시점까지 미룬다)
    from capture_pipeline.capture import main as run
    run()


if __name__ == "__main__":
    main()
