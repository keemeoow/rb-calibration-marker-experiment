# Session04 Calibration Evaluation (캘리브레이션 평가)

> Status: Final protocol before External GT. 비교 행은 A0~A5, B1~B3 한 벌만 사용하고, heldout 평가는 항상 cube만 본다.

## Current Data Warnings (현재 데이터 경고)

- Evaluation support: fixed cameras `0, 1, 3`, overall 17 obs / 340 corners; board 8 / 232, cube 9 / 108.
- Split support: 9 eligible sets; dropped sets `0, 1, 2, 3`.
- Cube detection: 117 images read, 108 accepted PnP observations, 99 core multiface selected, 2 PnP-RMSE rejections.
- Board-Cube conflict: direct PnP disagreement is 10.8077 mm translation RMSE and 0.5270 deg max rotation; joint solve mitigates it but does not remove the cause.

## Final Protocol Lock (최종 단일 기준)

| 항목 | 최종 기준 | 제외한 것 |
| --- | --- | --- |
| 비교 행 | A0~A5, B1~B3만 사용 | A6, 별도 board-only FK 변형, marker-system 별도 순위 |
| Heldout target | 항상 cube만 평가 | Board heldout, board와 cube를 섞은 pooled overall ranking |
| 최종 주 지표 | External cube TRE / rotation / P95 / failure | 내부 px만으로 물리 순위 확정 |
| 보조 내부 지표 | Train Cube, ALL Cube, Heldout Cube, Cross-view camera consistency | row별 marker가 다른 solver Train RMSE, pair type별 별도 순위 |
| A5 해석 | External GT 공개 전에 frozen이면 최종 후보 | GT를 본 뒤 정의한 사후 선택 |

A0/B3는 calibration 단계에서는 board-only로 유지한다. 다만 최종 cube 평가지표는 모든 row에서 train cube 관측으로 set별 evaluation pose만 맞춘 뒤, calibration 결과를 frozen한 상태로 계산한다.

## Final Comparison Table (최종 비교실험표)

> 굵은 값은 현재 artifact에서 관측된 Heldout Cube RMSE 최솟값이다. External GT가 들어오기 전에는 최종 물리 순위로 해석하지 않는다.

| Method (방법) | Calibration train target | Optimization | FK / target-pose 처리 | Train Cube RMSE px | ALL Cube RMSE px | Heldout Cube RMSE px | Cross-view Cube px | Cam-common Cube mm/deg | External GT TRE/Rot/P95/Fail | Convergence | Data status |
| --- | --- | --- | --- | ---: | ---: | ---: | ---: | ---: | --- | ---: | --- |
| A0 (baseline) | board-on-gripper only | sequential_frozen_stage | board pose=estimated; cube=eval only | 3.4489 | 3.5063 | 3.6768 | 7.1528 | 8.6775 / 0.9037 | Pending | 3/3 | Current data available (현재 데이터 있음) |
| A1 (+cube) | board+cube | sequential_frozen_stage | cube pose=estimated | 3.3414 | 3.4314 | 3.6938 | 7.1995 | 8.8294 / 1.0503 | Pending | 3/3 | Current data available (현재 데이터 있음) |
| A2 (+unified) | board+cube | unified_joint_optimization | cube pose=estimated | 3.3525 | 3.4140 | 3.5960 | 6.1948 | 7.2868 / 1.0280 | Pending | 3/3 | Current data available (현재 데이터 있음) |
| A3 (raw-FK hard fixed) | board+cube | unified_joint_optimization | cube pose=raw-FK-fixed | 4.2878 | 4.9967 | 6.7199 | 6.8268 | 8.3382 / 2.0892 | Pending | 3/3 | Current data available (현재 데이터 있음) |
| A4 (corrected-FK soft factor) | board+cube | unified_joint_optimization | cube pose=corrected-FK-factor | 3.3547 | 3.4111 | 3.5786 | 6.2061 | 7.3077 / 1.0151 | Pending | 3/3 | Current data available; measured FK covariance pending (현재 데이터 있음; FK covariance 측정 대기) |
| A5 (vision-aligned FK hard fixed) | board+cube | unified_joint_optimization | cube pose=vision-aligned-FK-fixed | 3.5312 | 3.5037 | **3.4180** | 5.6072 | 6.6745 / 0.8862 | Pending | 3/3 | Current data available; freeze before External GT scoring (현재 데이터 있음; External GT 채점 전 고정 필요) |
| B1 (−Unified) | board+cube | sequential_frozen_stage | cube pose=corrected-FK-factor | 3.3450 | 3.4322 | 3.6870 | 7.1779 | 8.8059 / 1.0540 | Pending | 3/3 | Current data available; measured FK covariance pending (현재 데이터 있음; FK covariance 측정 대기) |
| B2 (−board) | cube only | unified_joint_optimization | cube pose=corrected-FK-factor | 3.0201 | 3.4308 | 4.4608 | 6.4557 | 7.4292 / 1.0493 | Pending | 3/3 | Current data available; measured FK covariance pending (현재 데이터 있음; FK covariance 측정 대기) |
| B3 (−cube) | board-on-gripper only | unified_joint_optimization | board pose=estimated; cube=eval only | 3.4489 | 3.5061 | 3.6763 | 7.1502 | 8.6736 / 0.9035 | Pending | 3/3 | Current data available (현재 데이터 있음) |

### 평가지표

| 평가지표 | 설명 | 평가 지표 원리 |
| --- | --- | --- |
| External cube TRE / rotation / P95 / failure | 최종 물리 정확도 지표 | GT 공개 전에 각 방법의 cube pose prediction을 저장하고, 다음주 독립 External cube GT와 같은 pose list에서 translation, rotation, P95, failure를 계산한다. |
| ALL Cube RMSE px | 전체 cube 영상에 대한 fit sanity check | 카메라/hand-eye는 각 방법의 최종 calibration 결과로 고정한다. 그 뒤 train cube만으로 set별 `T_base_cube`를 nuisance pose로 맞추고, train 724 + heldout 236 cube corner를 합친 corner-pooled RMSE를 낸다. Train이 75.4%이므로 일반화 순위에는 쓰지 않는다. |
| Train Cube RMSE px | 동일 train cube 모집단의 fit 진단 | 모든 A0~A5/B1~B3에서 calibration을 frozen하고, train cube로 set별 evaluation pose를 맞춘 뒤 동일한 724개 train cube corner를 재투영한다. 같은 관측으로 pose를 맞추고 채점하므로 순위용 지표가 아니다. |
| Heldout Cube RMSE px | External GT 전 내부 보조 지표 | ALL Cube와 같은 train-only cube pose source를 사용하되, calibration과 pose fit에 쓰지 않은 동일한 236개 heldout cube corner를 corner-pooled RMSE로 계산한다. |
| Cross-view pixel transfer RMSE px | 카메라 간 pixel 일관성 | 한 카메라의 cube PnP pose를 다른 카메라로 전달한다. 동일한 36개 pair(9 fixed-fixed + 27 fixed-gripper)의 72개 방향, 904개 destination-corner를 직접 pooling한다. fixed-gripper 중 18개 pair는 train fixed-anchor를 사용하므로 mixed-anchor 내부 closure다. |
| Cam-common Obj-Cam consistency mm/deg | 카메라 간 3D pose 일관성 | 같은 frozen pair에서 두 카메라 경로가 만든 `T_base_cube` 차이를 36개 pair에 직접 pooling한다. fixed-gripper 경로에는 Hand-Eye와 Robot FK가 포함되며, Cross-view px와 독립된 증거는 아니다. |

> A0/B3는 calibration 단계에서는 cube를 쓰지 않는다. Cube train 관측은 카메라/hand-eye를 다시 맞추지 않고, cube RMSE 계산을 위한 set별 evaluation pose만 맞추는 데 사용한다.

## Matched Contrast Decision Table (비교실험 구성 확정표)

최종 비교는 아래 contrast만 사용한다. 모든 heldout 평가는 cube만 보며, External GT가 들어오면 같은 cube pose list에서 paired comparison으로 판정한다.

| Tier (구분) | Direct Contrast (직접 비교) | Question (검증 질문) | Primary Metric (주 지표) | Session04 Result | Decision (판정) |
| --- | --- | --- | --- | --- | --- |
| Final protocol | A0 -> B3 | 단일 target에서 sequential과 unified가 사실상 같아지는가 | External cube GT + heldout cube RMSE | Cube 3.6768 -> 3.6763 (-0.0004) | 구조 구현의 negative control이다. 현재 0.0004 px 차이로 기대한 동등성을 지지한다. |
| Final protocol | A0 -> A1 | board-on-gripper baseline에 cube train 관측을 추가하면 cube 평가가 개선되는가 | External cube GT + heldout cube RMSE | Cube 3.6768 -> 3.6938 (+0.0171) | External cube GT와 heldout cube RMSE로 판정한다. |
| Final protocol | A1 -> A2 | Vision-only 조건에서 unified feedback이 도움이 되는가 | External cube GT + heldout cube RMSE | Cube 3.6938 -> 3.5960 (-0.0978) | External cube GT와 heldout cube RMSE로 판정한다. |
| Final protocol | B3 -> A2 | unified 구조에서 cube residual이 최종 cube 평가에 필요한가 | External cube GT + heldout cube RMSE | Cube 3.6763 -> 3.5960 (-0.0803) | External cube GT와 heldout cube RMSE로 판정한다. |
| Final protocol | A2 -> A3 | Vision-estimated cube pose를 raw-FK hard fixed로 바꾸면 어떤가 | External cube GT + heldout cube RMSE | Cube 3.5960 -> 6.7199 (+3.1239) | raw FK hard fixed가 실제 cube 정합을 높이는지 External GT로 확인한다. |
| Final protocol | B1 -> A4 | 같은 soft FK factor에서 sequential과 unified 중 무엇이 나은가 | External cube GT + heldout cube RMSE | Cube 3.6870 -> 3.5786 (-0.1084) | External cube GT와 heldout cube RMSE로 판정한다. |
| Final protocol | A2 -> A4 | Unified vision-only에 soft FK factor를 추가하면 이득이 있는가 | External cube GT + heldout cube RMSE | Cube 3.5960 -> 3.5786 (-0.0174) | soft FK factor의 최종 이득은 External cube GT로 판정한다. |
| Final protocol | B2 -> A4 | Soft FK 조건에서 board residual이 cube 보정에 도움 되는가 | External cube GT + heldout cube RMSE | Cube 4.4608 -> 3.5786 (-0.8821) | External cube GT와 heldout cube RMSE로 판정한다. |
| Final protocol | A3 -> A5 | Raw FK hard fixed와 vision-aligned FK hard fixed의 차이는 무엇인가 | External cube GT + heldout cube RMSE | Cube 6.7199 -> 3.4180 (-3.3019) | A5가 GT 공개 전에 frozen method이면 최종 후보로 판정 가능하다. |
| Final protocol | A4 -> A5 | 같은 aligned FK를 soft factor와 hard fixed로 쓰면 무엇이 달라지는가 | External cube GT + heldout cube RMSE | Cube 3.5786 -> 3.4180 (-0.1606) | A5가 GT 공개 전에 frozen method이면 최종 후보로 판정 가능하다. |

> A5는 External GT 공개 전에 방법·파라미터·alignment artifact가 frozen이면 최종 후보로 비교할 수 있다. GT를 본 뒤 A5를 정의하면 사후 진단으로만 남긴다.

## Metric Decision Matrix (평가지표 판정표)

| Metric (지표) | Tier (등급) | Use (사용법) | Limit (제한) | Current Support (현재 근거) |
| --- | --- | --- | --- | --- |
| External cube TRE / rotation / P95 / failure | Final primary metric | 독립 External GT cube pose와 blind prediction을 비교해 최종 순위를 정함 | GT 측정계 uncertainty floor보다 작은 차이는 주장하지 않는다. | pending; External GT 추가 후 산출 |
| ALL Cube RMSE px | Fit sanity check | train-only cube pose fit 후 train+heldout cube corner에 frozen calibration을 적용 | train과 heldout을 섞고 train이 약 75%를 차지하므로 일반화 지표가 아니다. | train+heldout cube 960 corners (724 + 236) |
| Train Cube RMSE px | Train-split fit diagnostic | frozen calibration에서 train cube로 set별 pose를 맞춘 뒤 같은 train cube를 재투영 | 평가 pose를 맞춘 동일 관측의 in-sample fit이므로 방법 순위 지표가 아니다. | 모든 row에서 동일한 train cube 724 corners |
| Heldout Cube RMSE px | Internal support metric | train-only cube pose source와 frozen calibration으로 미사용 cube event를 재투영 | 같은 set의 다른 event이며 corner-pooled 값이라 새 위치 일반화나 물리 GT가 아니다. | heldout cube 236 corners |
| Cross-view pixel transfer RMSE | Supplementary camera consistency | 동일 frozen pair의 양방향 destination-corner 오차를 두 camera scope에서 직접 pooling | fixed-gripper pair에는 Hand-Eye/FK와 train fixed-anchor가 섞이며 공통 systematic error를 검출하지 못한다. | 36 pairs (9 fixed-fixed + 27 fixed-gripper; 18 train-anchor), 72 directions / 904 destination-corners |
| Cam-common Obj-Cam consistency mm/deg | Supplementary camera consistency | 같은 frozen cube pair에서 두 경로가 계산한 object pose 차이를 mm/deg로 pooling | Cross-view px와 같은 pair discrepancy의 다른 단위 표현이며 독립 증거가 아니다. | 36 pairs (9 fixed-fixed + 27 fixed-gripper; 18 train-anchor), 72 directions / 904 destination-corners |

> `Convergence 3/3`은 서로 다른 초기화 seed 3회 모두에서 SciPy solver가 `success=True`로 종료됐다는 뜻이다. Sequential 행은 두 stage가 모두 성공해야 하며, B1은 stage 1과 모든 fixed-camera stage 2가 성공해야 1회 수렴으로 센다. 이는 solver 종료 조건 충족을 뜻할 뿐, 절대 정확도나 전역 최적해를 보장하지 않는다.

## Cross-view Camera Consistency (cube-only)

고정카메라 pair와 고정카메라↔그리퍼카메라 pair의 원시 오차를 같은 frozen mask에서 직접 pooling한다. px는 destination-corner 수로, mm/deg는 pair 수로 가중한다. 별도 pair-type 순위는 만들지 않는다.

| Method (방법) | Cross-view Cube px | Cam-common Cube mm | Cam-common Cube deg | Support |
| --- | ---: | ---: | ---: | --- |
| A0 | 7.1528 | 8.6775 | 0.9037 | 36 pairs (9 fixed-fixed + 27 fixed-gripper; 18 train-anchor), 72 directions / 904 destination-corners |
| A1 | 7.1995 | 8.8294 | 1.0503 | 36 pairs (9 fixed-fixed + 27 fixed-gripper; 18 train-anchor), 72 directions / 904 destination-corners |
| A2 | 6.1948 | 7.2868 | 1.0280 | 36 pairs (9 fixed-fixed + 27 fixed-gripper; 18 train-anchor), 72 directions / 904 destination-corners |
| A3 | 6.8268 | 8.3382 | 2.0892 | 36 pairs (9 fixed-fixed + 27 fixed-gripper; 18 train-anchor), 72 directions / 904 destination-corners |
| A4 | 6.2061 | 7.3077 | 1.0151 | 36 pairs (9 fixed-fixed + 27 fixed-gripper; 18 train-anchor), 72 directions / 904 destination-corners |
| A5 | **5.6072** | **6.6745** | **0.8862** | 36 pairs (9 fixed-fixed + 27 fixed-gripper; 18 train-anchor), 72 directions / 904 destination-corners |
| B1 | 7.1779 | 8.8059 | 1.0540 | 36 pairs (9 fixed-fixed + 27 fixed-gripper; 18 train-anchor), 72 directions / 904 destination-corners |
| B2 | 6.4557 | 7.4292 | 1.0493 | 36 pairs (9 fixed-fixed + 27 fixed-gripper; 18 train-anchor), 72 directions / 904 destination-corners |
| B3 | 7.1502 | 8.6736 | 0.9035 | 36 pairs (9 fixed-fixed + 27 fixed-gripper; 18 train-anchor), 72 directions / 904 destination-corners |

## Objective Block Diagnostics (목적함수 블록 진단)

| Method (방법) | FK 처리 | Visual residual components (시각 잔차 수) | FK blocks / components (FK 블록/잔차 수) | Visual robust cost (시각 비용) | FK robust cost (FK 비용) | FK cost fraction (FK 비용 비율) |
| --- | --- | ---: | ---: | ---: | ---: | ---: |
| A2 | estimated | 7372 | 0 / 0 | 23950.81 | 0.00 | 0.000% |
| A3 | raw-FK-fixed (hard constant; residual 없음) | 7372 | 0 / 0 | 36683.56 | 0.00 | 0.000% |
| A4 | corrected-FK-factor | 7372 | 9 / 54 | 23960.94 | 29.28 | 0.122% |
| A5 | vision-aligned-FK-fixed (hard constant; residual 없음) | 7372 | 0 / 0 | 25788.40 | 0.00 | 0.000% |
| B1 | corrected-FK-factor | 6068 | 9 / 54 | 23343.41 | 33.96 | 0.145% |
| B2 | corrected-FK-factor | 1448 | 9 / 54 | 3348.98 | 38.57 | 1.139% |

> 이 비율은 최종 목적함수 값의 분해다. 각 항의 Jacobian과 변수 연결 구조가 다르므로, FK cost 비율을 파라미터 영향력 비율로 해석하면 안 된다.

## Calculation (계산 방식)

최종 평가는 Target $O=cube$만 사용한다.

$$T^{B,(i)}_O=T^B_{C_i}T^{C_i}_{O,\mathrm{PnP}}$$

$$T^B_{C_g}(e)=T^B_G(e)T^G_{C_g}$$

$$T^{B,(g)}_O(e)=T^B_G(e)T^G_{C_g}T^{C_g}_{O,\mathrm{PnP}}$$

Cross-view pixel transfer는 한 카메라의 측정 PnP pose를 다른 카메라로 전달해 cube corner pixel error를 계산한다. 9개 fixed-fixed pair와 27개 fixed-gripper pair의 양방향 destination-corner squared error를 직접 pooling한다. fixed-gripper 중 18개 pair는 train fixed-anchor와 heldout gripper event를 연결한다.

Heldout Cube RMSE는 train cube로 맞춘 set별 evaluation pose와 frozen calibration transform을 미사용 cube event corner에 적용해 계산한다. Heldout cube corner는 pose fit에 쓰지 않는다.

$$RMSE_{px}=\sqrt{\frac{1}{2N}\sum_k((u_k-\hat u_k)^2+(v_k-\hat v_k)^2)}$$

Train Cube RMSE는 모든 row에서 같은 724개 train cube corner를 사용한다. ALL Cube RMSE는 같은 train-only cube pose source로 train 724개와 heldout 236개 cube corner를 재투영한 뒤 합친 fit sanity check이며 train이 75.4%를 차지한다.

## Interpretation Limit (해석 한계)

Cross-view pixel transfer와 Cam-common Obj-Cam consistency는 같은 pair discrepancy를 px와 mm/deg로 표현한 상관된 내부 지표다. 방법별 추정값에 의존하고 fixed-gripper pair에는 Hand-Eye/FK가 섞이므로 공통 systematic error를 검출하지 못한다. 따라서 최종 주장은 External cube GT로만 결정한다.

## Terminology (용어 설명)

- **$T^B_{C_i}$, Base-to-Fixed-Camera Transform (베이스–고정카메라 변환)**: 고정카메라 외부 파라미터.
- **$T^G_{C_g}$, Hand–Eye Transform (핸드–아이 변환)**: 그리퍼에서 그리퍼카메라로의 변환.
- **$T^B_G(e)$, Robot FK Pose (이벤트별 로봇 순기구학 자세)**: 이벤트 $e$의 베이스–그리퍼 변환이며 평가 중 고정 입력이다.
- **PnP, Perspective-n-Point (3D–2D 자세 추정)**: 3D 표적점과 2D 영상점으로 카메라–표적 자세를 계산한다.
- **RMSE, Root Mean Squared Error (평균제곱근오차)**: 잔차 제곱 평균의 제곱근. px, mm, deg는 서로 합치지 않는다.
- **External cube GT**: GT 공개 전 저장한 blind prediction과 독립 cube GT pose를 비교하는 최종 주 지표.

## External GT Task (다음주 예정 태스크)

Independent External GT가 들어오면 모든 row의 cube pose prediction을 같은 GT cube pose list와 비교한다. 최종 결과는 Translation Error, Rotation Error, P95, Failure Rate로 산출한다.
