# Session04 캘리브레이션 방법 비교

> 상태: 외부 GT 전 내부 비교. 아래 수치는 절대 위치 정확도나 논문 간 SOTA 순위가 아니라, 동일한 Session04 관측에서 측정한 재투영 및 경로 일관성이다.

## 핵심 결과

- OpenCV 계열 중 Gripper-to-Fixed Board pixel 최저는 **Shah robot-world/hand-eye (4.278 px)** 이다.
- OpenCV 계열 중 Gripper-to-Fixed Cube pixel 최저는 **Shah robot-world/hand-eye (9.513 px)** 이다.
- 현재 A3는 Board/Cube 각각 **6.044 / 7.935 px** 이다. 두 표적을 임의의 단일 점수로 합치지 않았다.
- Shah 대비 A3는 Cube pixel/translation이 각각 **16.6% / 19.3% 낮지만**, Board pixel은 **41.3% 높다**. A3가 전체적으로 우월한 것이 아니라 Board–Cube 절충점이 다르다.
- 이 비교로 hand-eye와 Robot FK 오차를 분리할 수 없으므로 mm/deg도 두 경로의 일관성으로 해석한다.

## 동일 조건

- 데이터: `/Users/woo/Documents/GitHub/Robot-Lab/rb-calibration-marker-experiment/data/session04/calib_train`
- 학습 Event: [25, 26, 27, 28, 29, 30, 31, 32, 34, 35, 36, 37, 38, 39, 40, 42, 43, 44, 46, 47, 49, 50, 51, 52, 53, 54, 55, 57, 58, 59, 61, 62, 63, 64, 65, 66, 67, 68, 69, 71, 72, 73, 74, 75, 77]
- Held-out Event: [24, 33, 41, 45, 48, 56, 60, 70, 76]
- 평가 set: [4, 5, 6, 7, 8, 9, 10, 11, 12]
- 물리 scale: nominal `1.0`; 데이터 추정 scale 미사용
- OpenCV 7종: train-only Board PnP와 Robot FK로 fit; train-only Board로 고정카메라 등록
- 평가: 같은 set의 첫 fixed anchor × 해당 set의 모든 held-out gripper Event; Event→set→set 동일가중
- held-out refit 및 모델 출력 기반 관측 제거 없음

## 전체 경로 비교

| 방법 | Fit 정보 | Held-out Board reproj (px) | G→F Board (px / mm / deg) | G→F Cube (px / mm / deg) |
| --- | --- | ---: | ---: | ---: |
| A0 current board-only | Board; sequential nonlinear reprojection | 4.053 | 4.544 / 5.481 / 0.797 | 10.246 / 11.209 / 1.567 |
| A2 current Board+Cube | Board+Cube; unified nonlinear reprojection | 4.041 | 5.441 / 7.212 / 1.030 | 8.988 / 9.819 / 1.635 |
| A3 current full | Board+Cube; unified reprojection; FK-fixed Cube poses | 3.964 | 6.044 / 7.304 / 0.753 | 7.935 / 8.698 / 1.493 |
| Tsai–Lenz | Board-only closed-form + robust fixed-camera registration | 5.415 | 6.144 / 6.591 / 1.156 | 10.251 / 11.160 / 1.719 |
| Park–Martin | Board-only closed-form + robust fixed-camera registration | 4.425 | 4.982 / 5.149 / 0.568 | 9.598 / 10.780 / 1.469 |
| Horaud | Board-only closed-form + robust fixed-camera registration | 4.319 | 4.844 / 4.994 / 0.552 | 9.565 / 10.765 / 1.461 |
| Andreff | Board-only closed-form + robust fixed-camera registration | 4.163 | 4.680 / 4.835 / 0.556 | 9.660 / 10.848 / 1.464 |
| Daniilidis | Board-only closed-form + robust fixed-camera registration | 4.339 | 4.838 / 4.976 / 0.567 | 9.562 / 10.762 / 1.471 |
| Shah robot-world/hand-eye | Board-only closed-form + robust fixed-camera registration | 3.892 | 4.278 / 4.288 / 0.552 | 9.513 / 10.780 / 1.461 |
| Li robot-world/hand-eye | Board-only closed-form + robust fixed-camera registration | 7.512 | 9.000 / 10.246 / 0.550 | 10.861 / 12.960 / 1.461 |

`G→F`는 Gripper-to-Fixed이다. px/mm/deg는 합산하지 않는다. A0와 OpenCV 7종은 Board-only 정보 예산이 같고, A2/A3에는 Cube 정보의 효과가 포함된다.

## 고정카메라 부분 비교

| 방법 | Fixed Board (px / mm / deg) | Fixed Cube (px / mm / deg) |
| --- | ---: | ---: |
| A0 current board-only | 1.185 / 1.719 / 0.193 | 12.653 / 16.997 / 1.755 |
| A2 current Board+Cube | 3.546 / 4.973 / 0.513 | 9.546 / 12.607 / 1.725 |
| A3 current full | 5.470 / 7.968 / 0.423 | 7.513 / 9.753 / 1.789 |
| Tsai–Lenz | 1.408 / 1.948 / 0.176 | 12.698 / 17.050 / 1.766 |
| Park–Martin | 1.408 / 1.948 / 0.176 | 12.698 / 17.050 / 1.766 |
| Horaud | 1.408 / 1.948 / 0.176 | 12.698 / 17.050 / 1.766 |
| Andreff | 1.408 / 1.948 / 0.176 | 12.698 / 17.050 / 1.766 |
| Daniilidis | 1.408 / 1.948 / 0.176 | 12.698 / 17.050 / 1.766 |
| Shah robot-world/hand-eye | 1.408 / 1.948 / 0.176 | 12.698 / 17.050 / 1.766 |
| Li robot-world/hand-eye | 1.408 / 1.948 / 0.176 | 12.698 / 17.050 / 1.766 |

OpenCV 7종의 Fixed-to-Fixed 값이 같은 것은 오류가 아니다. 이 기준선들은 동일한 train-only Board 관측으로 고정카메라를 등록하고 hand-eye 해법만 바꾸므로, 공통 Board 좌표계의 강체변환은 고정카메라 상대 pose에서 상쇄된다.

## 결과가 말해 주는 현재 문제

- Board-only 계열은 Board에서 좋지만 Cube로 일반화할 때 오차가 커진다.
- A2/A3는 Cube를 공동최적화하면서 Cube 오차를 줄이는 대신 Board 쪽 오차가 증가한다.
- 따라서 현재 잔차는 단순히 고전 hand-eye 해법이 약해서 생긴 것으로 보기 어렵다. 한 외부파라미터가 Board와 Cube를 동시에 만족하지 못하는 **표적 간 모델/측정 불일치**가 남아 있다는 증거다.
- 가능한 원인은 Cube 3D geometry·corner ordering·실측 치수, Board 치수, intrinsic/distortion, 또는 표적별 검출 systematic bias다. 이 표만으로 원인 하나를 확정하지는 않는다.
- 이 불일치를 해결하기 전에는 더 복잡한 SOTA 최적화가 한 표적의 오차를 다른 표적으로 이동시키는 결과가 될 수 있다.

## 최근 방법과의 적용성 비교

| 방법 | 핵심 | Session04 수치 | 현재 판단 |
| --- | --- | --- | --- |
| OpenCV hand-eye 5종 | AX=XB closed-form/separable/simultaneous 해법 | 실행 완료 | Board-only 고전 기준선 |
| OpenCV Shah/Li | robot-world와 hand-eye 동시 추정 | 실행 완료 | Board-only 동시추정 기준선 |
| 현재 A3 | Board+Cube raw-corner 공동최적화 + FK-fixed Cube pose | 실행 완료 | 현재 데이터 구조에 직접 맞음 |
| Allegro et al. Multi-Camera Hand-Eye (RA-L 2024, ICRA 2025) | camera-base와 camera-camera 상대 pose 공동최적화 | 미실행 | 공개 C++ 구현의 Session04 adapter와 동일 mask 연결 필요 |
| Tabb & Yousef iterative robot-world/hand-eye (2019) | 재투영오차 직접 최소화; multiple-eye 확장 | 미실행 | 동일 관측 모델 이식 필요 |

Allegro 공개 구현은 카메라별 동일 프레임 수를 요구한다. Session04는 fixed camera를 set당 한 번만 저장하므로 fixed 영상을 복제하지 않고 누락 프레임으로 표현하는 adapter가 필요하다.

## 검증 및 출처

- 합성 좌표계 계약: **PASS** (30 poses, 7 methods)
- OpenCV 버전: `4.12.0`
- fixed-to-fixed mask SHA-256: `5543693435a83f0256182658a8251c7c20a6f46501b77ebd414fc0e315dc47cd`
- gripper-to-fixed mask SHA-256: `0287336178679d3f6f7c7d4408a0c282f5cf99a5c1ffd391378c9c6ec7090f1e`
- [OpenCV 공식 문서](https://docs.opencv.org/doc/doxygen/html/d4/d93/group__calib.html)
- [Allegro et al. 논문](https://arxiv.org/abs/2406.11392), [공개 구현](https://github.com/davidea97/Multi-Camera-Hand-Eye-Calibration)
- [Tabb & Yousef](https://arxiv.org/abs/1907.12425)

## 해석 한계

외부 tracker/정밀 치구/독립 3D GT가 없어 절대 정확도는 평가하지 않았다. Fixed-to-Fixed는 공통 계통오차를 놓칠 수 있고, Gripper-to-Fixed는 hand-eye와 FK 오차를 함께 포함한다. 따라서 논문 원문 수치와의 직접 순위표가 아니다.
