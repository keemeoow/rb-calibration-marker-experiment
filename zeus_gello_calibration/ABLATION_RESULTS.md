# Zeus Ablation Test Table 1

> 생성 코드: `zeus_gello_calibration/table1_zeus.py`
>
> 촬영 데이터 루트: `/home/jysim/*jiwoo/rb-calibration-marker-experiment/zeus_gello_calibration/data`
>
> External GT: `Pending`

## 1. 비교실험 구성

| Row | Calibration target | Optimization | FK 사용 방식 | 실행 상태 |
| --- | --- | --- | --- | --- |
| A0 | Board | Sequential | VISION | 완료 |
| A1 | Board + Cube | Sequential | VISION | 완료 |
| A2 | Board + Cube | Unified | VISION | 완료 |
| A3 | Board + Cube | Unified | FK hard fixed (mechanical) | Pending |
| A4 | Board + Cube | Unified | corrected-FK soft factor | 완료 |
| A5 | Board + Cube | Unified | corrected-FK hard fixed | 완료 |
| B1 | Board + Cube | Sequential | corrected-FK soft factor | 완료 |
| B2 | Cube | Unified | corrected-FK soft factor | 완료 |
| B3 | Board | Unified | VISION | 완료 |

## 2. 최종 내부 결과

모든 pixel 값은 작을수록 좋다. 굵은 순위 결론은 External GT가 아니라 내부 camera-consistency에만 해당한다.

| Row | ALL Cube px | Train Cube px | Held-out Test Cube px | ALL Cross-view px | Train Cross-view px | Held-out Test Cross-view px | External GT | Convergence |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | --- | ---: |
| A0 | 2.0790 | 2.0790 | 2.0790 | 4.0555 | 4.0555 | 4.0555 | Pending | 15/15 |
| A1 | 2.1510 | 2.1446 | 2.1434 | 4.0384 | 4.0420 | 4.0785 | Pending | 15/15 |
| A2 | 2.0327 | 2.0332 | 2.0432 | 3.9357 | 3.9396 | 3.9591 | Pending | 15/15 |
| A3 | Pending | Pending | Pending | Pending | Pending | Pending | Pending | Pending |
| A4 | 2.0226 | 2.0236 | 2.0342 | 3.9345 | 3.9384 | 3.9581 | Pending | 15/15 |
| A5 | 1.5087 | 1.5215 | 1.5938 | 4.6730 | 4.6517 | 4.6515 | Pending | 15/15 |
| B1 | 2.0834 | 2.0803 | 2.0862 | 4.0097 | 4.0140 | 4.0408 | Pending | 15/15 |
| B2 | 2.1736 | 2.1647 | 2.1954 | 3.9543 | 3.9770 | 4.0693 | Pending | 15/15 |
| B3 | 2.0790 | 2.0790 | 2.0790 | 4.0555 | 4.0555 | 4.0555 | Pending | 15/15 |

## 3. 평가지표 계산 방법

| 지표 | 계산 | 판정 역할 |
| --- | --- | --- |
| ALL Cube RMSE px | 전체 placement로 별도 fit 후, P1 VISION corrected-FK Cube pose를 전체 관측에 재투영 | full-data 적합 진단 |
| Train Cube RMSE px | leave-one-placement-out 각 fold의 train placement 재투영 | 학습 적합 진단 |
| Held-out Test Cube RMSE px | 해당 fold에서 제외한 placement를 frozen calibration으로 재투영 | 내부 보조 지표 |
| ALL Cross-view Cube RMSE px | 전체 데이터 fit에서 source-camera PnP를 destination으로 양방향 전달 | full-data camera 일관성 |
| Train Cross-view Cube RMSE px | 각 fold의 train placement에서 같은 양방향 전달 | 학습 camera 일관성 |
| Held-out Test Cross-view Cube RMSE px | calibration에서 제외한 placement에서 destination 관측을 scoring에만 사용 | 내부 주 비교 지표 |
| External GT | 독립 `T_base_cube_GT`와 frozen prediction의 TRE/rotation/P95/failure | 최종 물리 순위, 현재 Pending |

Pixel RMSE는 `sqrt(mean(dx^2, dy^2))`이며 placement를 동일 가중한다. `ALL`은 Train과 Held-out Test의 산술평균이 아니라 전체 데이터로 다시 fit한 결과다.

## 4. 현재 해석

- 내부 주 지표의 최저값은 **A4 (3.9581 px)**이지만, A2와의 차이는 0.0010 px라 현재 데이터에서는 사실상 동률로 해석한다.
- 내부 Cross-view 최저값은 카메라 간 일관성을 뜻하며 실제 3D 절대 정확도 최고를 뜻하지 않는다.
- Cube RMSE는 모든 row에 같은 P1 VISION corrected-FK reference를 사용하므로 corrected-FK 계열에 구조적으로 유리할 수 있다.
- A3는 영상과 독립적인 mechanical `T_flange_cube`가 없어 Pending이다. 현재 P1 fit을 A3로 부르면 A5와 정의가 중복된다.
- A4/B1/B2의 2.0 mm, 0.30 deg covariance는 실측값이 아니므로 preflight 결과다.
- A0와 B3는 stationary board에서 최적화 블록이 사실상 분리되어 수치가 거의 같다. 현재 데이터로는 board-only Unified 이점을 검증할 수 없다.
- 현재 데이터에는 계획한 gripper-mounted board 촬영이 없으므로 A0/B3 결과는 새 45-event 프로토콜의 최종 결과가 아니다.
- 최종 방법 채택은 External GT 열이 채워진 뒤 결정한다.

### 주요 paired contrast

`Δ`는 왼쪽 방법에서 오른쪽 방법을 뺀 Held-out Test Cross-view RMSE다. 음수이면 왼쪽 방법이 낮다.

| Contrast | Mean Δ px | 왼쪽 방법이 낮은 placement | 해석 |
| --- | ---: | ---: | --- |
| A2 - A1 | -0.1318 | 13/15 | Unified VISION이 Sequential VISION보다 낮음 |
| A4 - A2 | -0.0003 | 6/15 | corrected-FK soft factor와 VISION이 사실상 동률 |
| A5 - A4 | +0.6711 | 3/15 | corrected-FK hard fixed가 soft factor보다 높음 |

## 5. 사용 데이터

| 구분 | Observation 수 |
| --- | ---: |
| P1 gripped Cube | 45 |
| P2 fixed-camera Cube | 42 |
| P2 gripper-camera Cube | 15 |
| P3 Board | 60 |
| 전체 | 162 |
