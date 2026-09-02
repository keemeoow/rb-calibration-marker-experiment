# B — Independent OpenCV Relative-pose Reference Baseline

이 기준선은 OpenCV PnP로 학습 영상의 고정카메라 상대 자세를 직접 계산하는 독립 FK-free 기준선이다. Main-method transform, Joint optimizer, Robot FK, Hand–Eye, shared target pose를 사용하지 않는다. SOTA 비교나 절대 정확도 주장이 아니다.

A의 방법별 Fixed-to-Fixed/e_cross는 held-out 자기 일관성을 보는 보조 지표이고, 이 B가 그 값과 독립적으로 계산되는 relative-pose 기준선이다.

> **굵은 값**은 Board 또는 Cube 평가 열의 최솟값이다. 외부 GT 정확도 순위를 뜻하지 않는다.

| Train target | Board transfer px | Board translation mm | Cube transfer px | Cube translation mm | Train candidates/inliers |
| --- | ---: | ---: | ---: | ---: | ---: |
| Board only | **1.4141** | **1.9469** | 6.7698 | 10.1800 | 9 / 8 |
| Cube only | 6.4928 | 9.1998 | **2.8820** | **3.5148** | 12 / 12 |
| Board + Cube naive average | 2.2084 | 2.8479 | 6.5830 | 9.4558 | 21 / 11 |

## Board-vs-Cube Relative-transform Conflict

같은 물리적 카메라 관계를 board train 관측과 cube train 관측에서 각각 계산한 뒤 비교한 값이다. FK는 들어가지 않는다.

| Camera | Translation disagreement mm | Rotation disagreement deg |
| --- | ---: | ---: |
| 1 | 12.9524 | 0.5270 |
| 3 | 8.1146 | 0.3825 |

해석: Board-only는 held-out board에서 좋지만 cube에서는 나쁘고, Cube-only는 그 반대다. 즉 현재 큰 수치는 custom optimizer 하나만의 문제라기보다 target geometry/detection/pose convention 사이의 불일치 가능성을 함께 보여준다. Board와 cube 후보를 단순 평균한 결과도 양쪽 모두를 해결하지 못했다.

다음 진단: 동일 event의 board-PnP 상대 자세와 cube-PnP 상대 자세의 차이를 camera pair별로 분해하고, cube 3D geometry·corner ordering·intrinsic/distortion을 우선 점검한다.
