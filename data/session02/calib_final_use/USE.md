# 지금 바로 쓰는 캘리브 (session02, 2026-08-13)

## 한 줄

**cam1 과 cam3 만 쓴다. cam0 는 쓰지 않는다.** 두 카메라 결과를 축별 median 으로 합치면
로봇 FK 기준 오차 **RMSE 3.7 mm / median 3.4 mm / 최대 5.9 mm**, ridge 보정까지 얹으면
**RMSE 2.7 mm / median 2.6 mm / 최대 4.4 mm** 다.

## 쓰는 법

```python
from calibration_use import Calibration

calib = Calibration()                       # 이 폴더를 읽는다
p_base = calib.fuse({1: T_cam1_object,
                     3: T_cam3_object})     # 미터, base 프레임 3-vector
```

한 대만 보일 때는 `calib.to_base(3, T_cam3_object)`.
그리퍼 카메라(cam2)를 쓰려면 로봇의 현재 자세가 필요하다:
`calib.to_base(2, T_cam2_object, robot_T_base_gripper=T)`.

보정을 끄려면 `Calibration(use_ridge=False)` 또는 호출마다 `apply_ridge=False`.

## 왜 이 구성인가 — 카메라별 실측

로봇 FK 큐브중점 기준, 13개 배치 위치에서 잰 위치오차:

| 구성 | RMSE | median | 최대 |
|---|---:|---:|---:|
| **cam1 + cam3 + ridge** | **2.71 mm** | **2.55 mm** | 4.39 mm |
| cam1 + cam3 + 그리퍼 + ridge | 2.70 mm | 2.59 mm | 4.28 mm |
| cam1 + cam3 | 3.71 mm | 3.35 mm | 5.91 mm |
| cam3 단독 | 4.44 mm | 3.89 mm | 8.45 mm |
| cam2(그리퍼) 단독 | 4.61 mm | 3.62 mm | 7.39 mm |
| cam1 단독 | 4.69 mm | 3.65 mm | 9.69 mm |
| cam0+cam1+cam3 (기존 기본값) | 4.34 mm | 4.09 mm | 6.55 mm |
| **cam0 단독** | **16.76 mm** | 13.85 mm | 25.19 mm |

cam0 하나 빼는 것만으로 4.34 → 3.71 mm 로 좋아진다. cam1 과 cam3 은 서로 median 2.2 mm
로 일치하는데 둘 다 cam0 와는 10 mm 씩 어긋난다 — 즉 어긋나는 쪽은 cam0 다.

한 대만 쓴다면 **cam3** 이 낫다(median 은 cam1 이 근소하게 좋지만 최대오차가 9.7 vs 8.5 mm).

## 행렬 출처

`data/session02/calib_out` 의 Step3 결과(`fk_mode=fixed`, session02 전체 13위치 + eye-to-hand
블록)를 그대로 가져왔다. 재캘리브하지 않았다 — cam0 를 빼고 다시 돌려도 cam1/cam3 정확도는
같았다(3.71 vs 3.71 mm). cam0 는 fit 을 오염시킨 게 아니라 자기만 틀린 것이다.

파일: `T_base_C1.npy`, `T_base_C3.npy`, `T_gripper_cam.npy`, `T_C1_C3.npy`,
`calibration_for_use.json`(전부 한 파일에).

## ridge 보정

`p_corrected = p + [1, x, y] · W`. 예측된 base 위치의 x·y 에 따라 잔차를 빼주는 선형 보정으로,
13개 위치 전부로 학습했다. 위 표의 2.71 mm 는 **보정을 leave-one-out 으로 검증한 값**이다
(한 위치를 빼고 12곳으로 계수를 학습해 뺀 위치에 적용). in-sample 은 2.08 mm.

**학습 범위**: x ∈ [−424, −78] mm, y ∈ [278, 639] mm, z ≈ 10 mm (테이블 위).
이 범위를 벗어나면 선형 외삽이라 `calibration_use.py` 가 보정을 자동으로 건너뛰고 경고한다.
높이가 크게 다른 물체를 다룰 때는 보정을 끄는 편이 안전하다.

## 로봇 툴오프셋(TCP)을 바꿔서 쓸 때

캘리브는 **툴오프셋 z = 150 mm** 상태에서 했다. 지금 실제 그리퍼(115.5 mm)나 flange(0)로
바꿔 쓴다면:

- **고정 카메라 cam1·cam3 는 아무것도 안 바꿔도 된다.** `T_base_Ci` 는 base 프레임과
  카메라 프레임만 잇는다. TCP 가 어디로 정의돼 있든 base 프레임은 그대로다.
- **그리퍼 카메라(cam2)만 영향을 받는다.** `T_gripper_cam` 은 캘리브 당시 TCP(150 mm)
  기준이다. 툴오프셋을 t 로 바꾸면 사이에 `Trans(0, 0, 150 − t)` 를 끼워야 한다.

| 로봇 툴오프셋 | 끼울 값 | `T_tcp_cam` 위치 (mm) |
|---|---:|---|
| 150 mm (캘리브 당시) | 0 | (34.3, 27.9, −109.5) |
| 115.5 mm (실제 그리퍼) | **+34.5 mm** | (34.3, 27.9, **−75.0**) |
| 0 mm (flange) | +150 mm | (34.3, 27.9, +40.5) |

부호에 주의 — 툴이 짧아지면 TCP 가 flange 쪽으로 당겨지고, 카메라는 그대로 있으므로
새 TCP 기준으로는 카메라가 +z 로 **더 멀어진다**. 그래서 −34.5 가 아니라 **+34.5** 다.

로더는 인자로 처리한다:

```python
calib = Calibration(robot_tool_offset_z_m=0.1155)   # 실제 그리퍼로 바꿔 쓸 때
```

### FK 큐브 높이는 이미 보정돼 있다

로봇이 기록한 배치 큐브중점의 z 는 전 세트 −32.70 mm 인데, 캘리브가 영상으로 정렬한
값은 +11 mm 안팎이다. 그 43.7 mm 차이(툴오프셋 불일치 + 큐브 기하)는 이미
`T_fk_cube_center_to_tag_object` 라는 상수(179.42°, 43.5 mm)로 **데이터에서 추정돼 흡수**됐다.
가정한 값이 아니라 eye-in-hand 영상으로 푼 값이다. 그래서 z=150 설정이 틀렸더라도
`T_base_C1`·`T_base_C3` 로 새는 일은 없다.

## 읽을 때 주의

- 위 숫자는 **로봇 FK 를 기준으로 잰 대용치**다. 외부 GT 가 아니라서 FK 자체의 오차가
  섞여 있다. 절대 정확도의 하한이 아니라 "이 정도 안쪽" 으로 읽어야 한다.
- 회전(자세) 정확도는 측정하지 않았다. 위 수치는 전부 위치에 대한 것이다.
- 카메라를 건드렸거나 옮겼으면 이 행렬은 그 순간 무효다.
