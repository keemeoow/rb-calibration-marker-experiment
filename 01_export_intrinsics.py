#!/usr/bin/env python3
"""01. 연결된 모든 RealSense의 공장(factory) intrinsics와 depth scale을 덤프한다.

원리
----
핀홀 카메라 모델에서 카메라 좌표계의 3D 점 X_c = (X, Y, Z)는 다음으로 정규화된다.

    x_n = X / Z ,  y_n = Y / Z

여기에 렌즈 왜곡을 먹인 뒤 intrinsic matrix K를 곱해 픽셀 좌표가 나온다.

    r^2 = x_n^2 + y_n^2
    x_d = x_n (1 + k1 r^2 + k2 r^4 + k3 r^6) + 2 p1 x_n y_n + p2 (r^2 + 2 x_n^2)
    y_d = y_n (1 + k1 r^2 + k2 r^4 + k3 r^6) + p1 (r^2 + 2 y_n^2) + 2 p2 x_n y_n

    [u, v, 1]^T = K [x_d, y_d, 1]^T ,   K = [[fx, 0, cx], [0, fy, cy], [0, 0, 1]]

이 단계가 저장하는 것은 위 식의 상수 (fx, fy, cx, cy)와 D = (k1, k2, p1, p2, k3)이며,
RealSense SDK가 보고하는 값을 **그대로** 옮겨 적을 뿐 추정하지 않는다.

깊이 값은 정수 raw 값이므로 미터로 바꾸려면 depth scale s를 곱한다.

    Z[m] = d_raw * s          (s = depth_scale, D4xx 계열은 보통 1e-3 m/unit)

주의: D415/D435의 color 스트림은 공장 왜곡계수 D를 전부 0으로 보고한다. 즉 이
단계의 D는 "왜곡 없음"이라는 뜻이 아니라 "공장에서 제공하지 않음"이라는 뜻이며,
실제 렌즈 왜곡은 02단계에서 ChArUco로 직접 추정해 덮어쓴다.

입력 / 처리 / 출력
------------------
입력: 연결된 RealSense 장치들, 요청한 color/depth 해상도와 FPS.
처리: 시리얼을 열거해 안정적인 serial -> cam_idx 매핑을 만들고, 각 스트림의
      intrinsics와 depth scale, depth-to-color extrinsic을 읽는다.
출력: intrinsics/device_map.json, depth_scales.json, cam{idx}.npz.

구현 위치
---------
이 파일은 얇은 진입점(entry point)이고 실제 동작은 아래에 있다.

    capture_pipeline/export_intrinsics.py
      main()          - 장치 열거, 스트림 개시, npz/json 기록까지 전체 흐름
      _intr_to_KD()   - pyrealsense2 intrinsics -> (K 3x3, D Nx1) 변환.
                        위 수식의 fx/fy/ppx/ppy와 coeffs를 그대로 옮긴다.

import를 main() 안에 두는 이유: 이 모듈은 pyrealsense2를 필요로 하므로,
하드웨어가 없는 환경에서 파일을 import만 해도 죽는 일이 없게 하기 위해서다.

다음 단계
---------
02_calibrate_intrinsics.py 가 여기서 만든 cam{idx}.npz의 color_K/color_D만
교체한다. depth 관련 필드는 이 단계의 값이 끝까지 쓰인다.
"""

def main() -> None:
    # capture_pipeline/export_intrinsics.py 의 main() 을 그대로 실행한다.
    # (지연 import: pyrealsense2 의존성을 실행 시점까지 미룬다)
    from capture_pipeline.export_intrinsics import main as run
    run()


if __name__ == "__main__":
    main()
