# 세 FK 방식 비교와 최종 캘리브 행렬 (session02, 2026-08-13)

데이터: `data/session02/calib_train` — 위치 0~12 (배치 13곳, 77 이벤트) + 위치 13
(그리퍼로 큐브를 쥔 eye-to-hand 블록, 13 이벤트). 고정 카메라 0·1·3, 그리퍼 카메라 2.

---

## 0. 세 방식이 실데이터에서 뜻하는 것

| 사용자 명명 | 실데이터 arm | 큐브 자세를 어떻게 두는가 | 최종 행렬이 달라지나 |
|---|---|---|---|
| no-FK | A2 | 자유변수. 영상만으로 추정 | **예** |
| fixed-FK | A3 | (train 전용) 정렬된 FK 값에 고정 | **예** |
| corrected-FK | A3 또는 A2 + 잔차 보정 | base arm 과 동일 | **아니오** |

**corrected-FK 는 행렬을 바꾸지 않는다.** 보정은 캘리브가 끝난 뒤 *예측된 큐브 중심*에만
적용되는 후처리이고, `T_base_Ci`·`T_gripper_cam` 은 건드리지 않는다. 이건 해석이 아니라
`CP_D1_fk_correction_2x2.py` 가 assert 로 강제하는 불변식이다. 그래서 "최종 행렬을 무엇으로
뽑을까" 는 결국 **no-FK vs fixed-FK 두 갈래**이고, corrected-FK 는 그 위에 얹는 계수다.

한 가지 더 — "fixed-FK 의 FK 값"은 로봇이 보고하는 큐브중점 원시값이 아니라 거기에 상수
강체변환을 한 번 먹인 값이다. 그 상수는 전 위치에서 동일하게 **179.42°, 43.5 mm** 로,
로봇 서버의 큐브중점 좌표계와 AprilTag 큐브 물체 좌표계 사이의 규약 차이다. 위치마다
달라지는 FK 오차 보정이 아니므로, "원시 FK 에 고정" 은 별도의 방식이 아니라 그냥 틀린
좌표계다. 세 방식을 원시-FK 까지 넣어 네 갈래로 벌리지 않은 이유다.

---

## 1. 방식 판정 — 위치 단위 leave-one-out 13 fold

`ABLATION_TEST_result/D1_session02_fkmodes/` (`CP_D1_fk_correction_2x2.py`, 같은 backend·같은 solver
설정·같은 예측 mask). 한 위치를 통째로 빼고(그 위치의 board 관측까지) 나머지 12곳으로만
맞춘 뒤, 빼둔 위치의 큐브 중심을 예측한다. 보정 계수도 train 위치에서만 학습한다.

### 주지표 — held-out 위치 오차 RMSE (mm), 로봇 FK 기준

| arm | 보정 없음 | offset(3) | se3(6) | ridge(9) |
|---|---:|---:|---:|---:|
| **A3 fixed-FK** | **4.375** | 3.955 | 3.535 | **3.249** |
| A2 no-FK | 5.079 | 4.731 | 3.804 | 3.399 |

fixed-FK 가 네 칸 모두에서 앞선다. 다만 차이는 작고 통계적으로 강하지 않다:
보정 없음 −0.487 mm (t=−1.93, 13중 10 fold), offset −0.696 mm (t=−3.31, 11 fold),
se3 −0.182 mm (t=−0.86), ridge −0.130 mm (t=−0.79).

잔차 보정 자체의 이득: A3 에서 ridge −0.929 mm (t=−1.84, 9 fold), A2 에서 −1.286 mm
(t=−1.85, 8 fold). 방향은 일관되지만 13개 위치로 단정할 만큼은 아니다.

### 통제지표 — FK 를 전혀 안 쓰는 지표에서는 반대로 나온다

| 지표 | A3 fixed-FK | A2 no-FK | 차이 |
|---|---:|---:|---:|
| 고정카메라간 큐브중점 일치 RMSE (mm) | 19.31 | **17.79** | A3 −1.51 불리 (t=4.90, 12/13 fold) |
| FK 자세에서의 재투영 (px) | **7.27** | 7.69 | A3 −0.42 유리 (t=−2.22, 9/13 fold) |

**두 지표를 같이 읽어야 한다.** A3 는 FK 에 맞춰 fit 했으니 FK 기준 지표에서 유리하고,
A2 는 카메라끼리 맞추도록 fit 했으니 카메라 일치도에서 유리하다. 각자 자기 홈그라운드를
이긴 셈이라 어느 쪽도 압도하지 못한다. 이건 session01 에서 나온 "FK 고정 여부는 13개
위치로는 판정되지 않는다" 와 같은 결론이다.

---

## 2. 전체 데이터 full fit — 두 방식의 행렬

`CP_final_fk_mode_fit.py` 로 hold-out 없이 13개 위치를 전부 써서 각 arm 을 한 번씩 맞췄다.
판정용 backend 와 동일하다. 산출물은 `no_fk/`, `fixed_fk/` 하위에 있다.

두 행렬의 차이는 작다: `T_base_C0` 2.0 mm/0.43°, `T_base_C1` 8.5 mm/0.89°,
`T_base_C3` 3.5 mm/0.46°, `T_gripper_cam` 0.9 mm/0.26°.

---

## 3. 기존 production(Step3) 결과와의 대조 — 이게 결정적이었다

`data/session02/calib_out` 에 이미 있던 Step3 결과(`fk_mode=fixed`)를 같은 두 지표로
재보니, 실험 backend 로 뽑은 두 행렬보다 **양쪽 다 낫다**.

| 행렬 | FK 기준 위치오차 RMSE (mm) | 카메라간 일치 RMSE (mm) |
|---|---:|---:|
| **Step3 production (fixed-FK)** | **3.872** | **12.17** |
| CP full fit — fixed-FK | 4.225 | 17.53 |
| CP full fit — no-FK | 4.802 | 16.72 |

세 값 모두 같은 13개 위치 전부를 쓴 in-sample 값이라 조건은 같다. 카메라 일치도에서 5 mm
가까이 벌어지는 건 Step3 파이프라인이 갖고 있는 robust 평균(MAD 이상치 제거)과 단계별
refinement 덕이다. 실험 backend 는 방식 비교를 공정하게 하려고 그런 장치를 일부러 빼놓은
버전이므로, 절대 성능은 production 이 앞서는 게 자연스럽다.

Step3 재실행 재현성도 확인했다(같은 인자로 다시 돌림): `T_base_C*` 0.8~2.8 mm, 0.11~0.17°
차이. AprilTag 검출·PnP 의 무작위성 수준이고, 커밋된 쪽이 근소하게 낫다(FK 3.872 vs 3.903,
일치도 12.17 vs 12.98). 그래서 커밋된 값을 그대로 쓴다.

---

## 4. 결론 — 최종 채택

> **fixed-FK 방식으로 뽑은 `data/session02/calib_out` 의 행렬을 최종으로 쓴다.**
> 큐브 중심 예측에는 `cube_centre_ridge_correction.json` 을 선택적으로 얹는다.

근거 세 줄:

1. held-out 판정에서 fixed-FK 가 no-FK 를 네 개 보정 조건 모두에서 앞섰다(작지만 일관).
2. corrected-FK 는 행렬을 바꾸지 않으므로 별도 행렬이 아니라 보정 계수로 붙는다.
3. 그 fixed-FK 를 실제로 구현한 production 파이프라인(Step3)의 출력이 두 지표 모두에서
   실험 backend 산출물보다 좋다.

### 최종 행렬 (미터, base 프레임)

| 변환 | 위치 (mm) |
|---|---|
| T_base_C0 | (279.8, 439.8, 356.3) |
| T_base_C1 | (−561.1, −55.6, 227.7) |
| T_base_C3 | (−545.3, 1025.2, 254.1) |
| T_gripper_cam | (34.3, 27.8, −109.5) |

파일: `data/session02/calib_out/{T_base_C0,T_base_C1,T_base_C3,T_gripper_cam,T_C0_C1,T_C0_C3}.npy`
및 `final_transforms_base_frame.json`.

### 잔차 보정 (선택)

`data/session02/calib_out/cube_centre_ridge_correction.json` — 위 행렬로 예측한 큐브 중심
p 에 `p + [1, x, y]·W` 를 더한다. 13개 위치 전부로 학습한 in-sample RMSE:
보정 없음 3.872 → offset 3.488 → se3 3.062 → **ridge 2.051 mm**.
held-out 이득은 −0.93 mm (t=−1.84) 수준이니 "확실한 이득" 이 아니라 "쓰면 손해는 아닌
정도" 로 읽어야 한다. 행렬은 건드리지 않으므로 언제든 껐다 켤 수 있다.

---

## 5. 남는 한계

- **외부 기준이 없다.** 두 지표 모두 대용치다. FK 기준 지표는 로봇 FK 자체 오차(session01
  기준 median 6.6 mm)를 포함하고, 카메라 일치도 지표는 절대 위치를 말하지 않는다.
- **13개 위치는 방식 판정에 빠듯하다.** 주지표의 t 값이 −0.79~−3.31 사이를 오간다.
- 카메라간 일치 RMSE 가 12 mm 대로 큰 건 관측의 일부에서 나는 PnP flip 꼬리 때문이다.
  median 은 8.89 mm 다.
- cam0 는 여전히 다른 두 대보다 나쁘다. Step3 의 재투영 refinement 가 가드(73~74 mm/7.4~7.8°)
  에 걸려 기각됐다. 이번 판정과 별개로 남아 있는 문제다.
- Step4 검증(`Step4_verify.py`)은 아직 session02 에 돌리지 않았다.

## 재현 명령

```bash
# 방식 판정 (13 fold, 약 30분)
PYTHONPATH= python CP_D1_fk_correction_2x2.py \
    --root_folder data/session02/calib_train --intrinsics_dir intrinsics \
    --calib_dir data/session02/calib_out \
    --out_dir ABLATION_TEST_result/D1_session02_fkmodes --lambdas 0 --folds all

# 두 방식의 전체 데이터 fit (약 1.5분)
PYTHONPATH= python CP_final_fk_mode_fit.py \
    --root_folder data/session02/calib_train --intrinsics_dir intrinsics \
    --calib_dir data/session02/calib_out --out_dir ABLATION_TEST_result/final_fk_mode_fit

# 최종 채택 행렬 (약 20초)
PYTHONPATH= python Step3_calibration.py \
    --root_folder data/session02/calib_train --intrinsics_dir intrinsics
```
