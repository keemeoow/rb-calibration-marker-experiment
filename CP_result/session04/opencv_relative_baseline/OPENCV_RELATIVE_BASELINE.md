# OpenCV Relative-pose Reference Baseline

이 기준선은 OpenCV PnP로 학습 영상의 고정카메라 상대 자세를 직접 계산한 FK-free 진단이다. Joint optimizer, Robot FK, Hand–Eye, shared target pose를 사용하지 않는다. SOTA 비교나 절대 정확도 주장이 아니다.

| Train target | Board transfer px | Board translation mm | Cube transfer px | Cube translation mm | Train candidates/inliers |
| --- | ---: | ---: | ---: | ---: | ---: |
| Board only | 1.4141 | 1.9469 | 12.7009 | 17.0416 | 9 / 8 |
| Cube only | 11.3082 | 16.6594 | 4.2470 | 4.4798 | 12 / 10 |
| Board + Cube naive average | 3.4211 | 4.4409 | 11.6733 | 15.2586 | 21 / 10 |

## Board-vs-Cube Relative-transform Conflict

같은 물리적 카메라 관계를 board train 관측과 cube train 관측에서 각각 계산한 뒤 비교한 값이다. FK는 들어가지 않는다.

| Camera | Translation disagreement mm | Rotation disagreement deg |
| --- | ---: | ---: |
| 1 | 19.4232 | 0.2723 |
| 3 | 14.8746 | 0.2170 |

해석: Board-only는 held-out board에서 좋지만 cube에서는 나쁘고, Cube-only는 그 반대다. 즉 현재 큰 수치는 custom optimizer 하나만의 문제라기보다 target geometry/detection/pose convention 사이의 불일치 가능성을 함께 보여준다. Board와 cube 후보를 단순 평균한 결과도 양쪽 모두를 해결하지 못했다.

다음 진단: 동일 event의 board-PnP 상대 자세와 cube-PnP 상대 자세의 차이를 camera pair별로 분해하고, cube 3D geometry·corner ordering·intrinsic/distortion을 우선 점검한다.
