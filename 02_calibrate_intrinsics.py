#!/usr/bin/env python3
"""02. ChArUco 보드 관측으로 각 카메라의 color intrinsics (K, D)를 직접 보정한다.

왜 필요한가
-----------
01단계가 덤프하는 D415/D435의 color 왜곡계수 D는 전부 0으로 보고된다. 그 상태로
05단계의 재투영 최적화를 돌리면 렌즈 왜곡이 통째로 잔차에 남아 외부 파라미터
(T^B_Ci, T^G_Cg)로 흡수되어 버린다. 그래서 여기서 K, D를 직접 추정한다.

원리 / 수식
-----------
ChArUco 보드는 체스판 격자 위에 ArUco 마커를 얹은 표적이다. 마커 ID를 디코딩하면
어느 교차점이 몇 번 corner인지 알 수 있으므로, 보드가 잘리거나 일부만 보여도
3D 보드 좌표 X_k (Z=0 평면 위, 단위 m)와 2D 픽셀 x_k의 대응이 확정된다.

V개 뷰에 대해 K, D와 뷰별 자세 (R_v, t_v)를 동시에 추정한다. 최소화 대상은
전체 재투영 제곱오차다.

    (K*, D*, {R_v, t_v}*) = argmin  Σ_v Σ_k || π(K, D, R_v, t_v, X_k) - x_vk ||^2

    π(·)는 01번 파일에 적은 핀홀 + Brown-Conrady 왜곡 투영이다.

OpenCV가 돌려주는 RMS는 위 합을 corner 수로 나눈 뒤 제곱근을 취한 값이다.

    RMS = sqrt( (1/N) Σ_v Σ_k || π(...) - x_vk ||^2 )      [px]

뷰별 RMS도 따로 계산해 어떤 뷰가 해를 끌고 가는지 본다.

이 문제가 잘 풀리려면 뷰가 **화각을 고르게 덮고 기울기가 다양해야** 한다. 정면에서만
찍으면 fx/fy와 t_z가 서로를 상쇄해(scale-depth ambiguity) K가 제대로 분리되지 않고,
화면 가장자리를 안 덮으면 반경 왜곡 k1, k2가 관측되지 않는다. 그래서 수집 루프가
커버리지와 선명도(라플라시안 분산)를 화면에 띄우고 프레임을 골라 받는다.

입력 / 처리 / 출력
------------------
입력: 01단계의 intrinsics/, 연결된 카메라, config.py의 CharucoBoardConfig
      (기본 11x7, square 25mm, marker 18mm, DICT_4X4_250).
처리: 카메라를 하나씩 열어 대화형으로 뷰를 모으고 cv2.calibrateCamera를 돌린다.
출력: cam{idx}.npz의 color_K/color_D만 교체. 원본은 factory_backup/에 보관.
      depth_K, depth_scale, R_depth_to_color, 해상도, 시리얼은 그대로 둔다.

구현 위치
---------
    capture_pipeline/calibrate_intrinsics.py
      main()                        - device_map 로드 후 카메라별 루프 지휘
      collect_for_camera()          - 라이브 프리뷰, SPACE 수동 그랩, 뷰 수집
      _sharpness()                  - 라플라시안 분산 기반 흐림 판정
      _obj_img_from_charuco()       - ChArUco 검출 결과 -> (3D X_k, 2D x_k) 대응쌍
      _run_calib()                  - cv2.calibrateCamera 호출 + 뷰별 RMS 계산
                                      (위 argmin 식이 실제로 풀리는 지점)
      calibrate_intrinsics()        - 이상 뷰 제거 후 재보정하는 상위 래퍼
      _draw_overlay()               - 커버리지/선명도/수락 개수 화면 피드백
      overwrite_color_intrinsics()  - npz의 color_K/color_D만 교체하고 나머지 보존

import를 main() 안에 두는 이유: pyrealsense2 의존성을 실행 시점까지 미루기 위해서다.

선행 조건
---------
01_export_intrinsics.py 를 먼저 돌려 device_map.json과 cam{idx}.npz(depth 필드
포함)가 있어야 한다. 보드 설정이 실제 인쇄물과 다르면 K가 통째로 틀어진다.
"""

def main() -> None:
    # capture_pipeline/calibrate_intrinsics.py 의 main() 을 그대로 실행한다.
    # (지연 import: pyrealsense2 의존성을 실행 시점까지 미룬다)
    from capture_pipeline.calibrate_intrinsics import main as run
    run()


if __name__ == "__main__":
    main()
