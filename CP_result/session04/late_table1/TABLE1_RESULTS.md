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
| 보조 내부 지표 | ALL Cube, Train, Heldout Cube, Cross-view camera consistency | pair type별 값을 별도 순위 지표로 분리 |
| A5 해석 | External GT 공개 전에 frozen이면 최종 후보 | GT를 본 뒤 정의한 사후 선택 |

현재 Session04 artifact는 이전 촬영 구성에서 생성된 값이므로, 최종 board-on-gripper A0/B3 cube 평가가 없으면 해당 칸은 N/A로 둔다.

## Final Comparison Table (최종 비교실험표)

> 굵은 값은 현재 artifact에서 관측된 Heldout Cube RMSE 최솟값이다. External GT가 들어오기 전에는 최종 물리 순위로 해석하지 않는다.

| Method (방법) | Calibration train target | Optimization | FK / target-pose 처리 | Train RMSE px | ALL Cube RMSE px | Heldout Cube RMSE px | Cross-view Cube px | Cam-common Cube mm/deg | External cube GT | Convergence | Data status |
| --- | --- | --- | --- | ---: | ---: | ---: | ---: | ---: | --- | ---: | --- |
| A0 (baseline) | board-on-gripper only | sequential_frozen_stage | board pose=estimated; cube heldout only | 3.8202 | N/A | N/A | 7.2577 | 8.6775 / 0.9037 | Pending | 3/3 | Current data available (현재 데이터 있음) |
| A1 (+cube) | board+cube | sequential_frozen_stage | cube pose=estimated | 3.7923 | 3.8075 | 4.1402 | 7.3085 | 8.8294 / 1.0503 | Pending | 3/3 | Current data available (현재 데이터 있음) |
| A2 (+unified) | board+cube | unified_joint_optimization | cube pose=estimated | 3.7421 | 3.4139 | 3.5958 | 6.3003 | 7.2868 / 1.0280 | Pending | 3/3 | Current data available (현재 데이터 있음) |
| A3 (raw-FK hard fixed) | board+cube | unified_joint_optimization | cube pose=raw-FK-fixed | 5.1587 | 7.8551 | 6.3959 | 6.8626 | 8.3382 / 2.0892 | Pending | 3/3 | Current data available (현재 데이터 있음) |
| A4 (corrected-FK soft factor) | board+cube | unified_joint_optimization | cube pose=corrected-FK-factor | 3.7441 | 3.4149 | 3.5805 | 6.3126 | 7.3077 / 1.0151 | Pending | 3/3 | Current data available; measured FK covariance pending (현재 데이터 있음; FK covariance 측정 대기) |
| A5 (vision-aligned FK hard fixed) | board+cube | unified_joint_optimization | cube pose=vision-aligned-FK-fixed | 3.9648 | 3.8102 | **3.2274** | 5.7166 | 6.6745 / 0.8862 | Pending | 3/3 | Current data available; freeze before External GT scoring (현재 데이터 있음; External GT 채점 전 고정 필요) |
| B1 (−Unified) | board+cube | sequential_frozen_stage | cube pose=corrected-FK-factor | 3.7887 | 3.7832 | 4.1182 | 7.2867 | 8.8059 / 1.0540 | Pending | 3/3 | Current data available; measured FK covariance pending (현재 데이터 있음; FK covariance 측정 대기) |
| B2 (−board) | cube only | unified_joint_optimization | cube pose=corrected-FK-factor | 3.0269 | 3.4423 | 4.4827 | 6.5284 | 7.4292 / 1.0493 | Pending | 3/3 | Current data available; measured FK covariance pending (현재 데이터 있음; FK covariance 측정 대기) |
| B3 (−cube) | board-on-gripper only | unified_joint_optimization | board pose=estimated; cube heldout only | 3.8202 | N/A | N/A | 7.2550 | 8.6736 / 0.9035 | Pending | 3/3 | Current data available (현재 데이터 있음) |

## Matched Contrast Decision Table (비교실험 구성 확정표)

최종 비교는 아래 contrast만 사용한다. 모든 heldout 평가는 cube만 보며, External GT가 들어오면 같은 cube pose list에서 paired comparison으로 판정한다.

| Tier (구분) | Direct Contrast (직접 비교) | Question (검증 질문) | Primary Metric (주 지표) | Session04 Result | Decision (판정) |
| --- | --- | --- | --- | --- | --- |
| Final protocol | A0 -> B3 | board-on-gripper only에서 sequential과 unified의 차이는 무엇인가 | External cube GT + heldout cube RMSE | Cube N/A | External cube GT에서 최종 판정한다. 현재 데이터에 cube heldout이 없으면 N/A로 둔다. |
| Final protocol | A0 -> A1 | board-on-gripper baseline에 cube train 관측을 추가하면 cube 평가가 개선되는가 | External cube GT + heldout cube RMSE | Cube N/A | External cube GT와 heldout cube RMSE로 판정한다. |
| Final protocol | A1 -> A2 | Vision-only 조건에서 unified feedback이 도움이 되는가 | External cube GT + heldout cube RMSE | Cube 4.1402 -> 3.5958 (-0.5443) | External cube GT와 heldout cube RMSE로 판정한다. |
| Final protocol | B3 -> A2 | unified 구조에서 cube residual이 최종 cube 평가에 필요한가 | External cube GT + heldout cube RMSE | Cube N/A | External cube GT와 heldout cube RMSE로 판정한다. |
| Final protocol | A2 -> A3 | Vision-estimated cube pose를 raw-FK hard fixed로 바꾸면 어떤가 | External cube GT + heldout cube RMSE | Cube 3.5958 -> 6.3959 (+2.8000) | raw FK hard fixed가 실제 cube 정합을 높이는지 External GT로 확인한다. |
| Final protocol | B1 -> A4 | 같은 soft FK factor에서 sequential과 unified 중 무엇이 나은가 | External cube GT + heldout cube RMSE | Cube 4.1182 -> 3.5805 (-0.5378) | External cube GT와 heldout cube RMSE로 판정한다. |
| Final protocol | A2 -> A4 | Unified vision-only에 soft FK factor를 추가하면 이득이 있는가 | External cube GT + heldout cube RMSE | Cube 3.5958 -> 3.5805 (-0.0153) | soft FK factor의 최종 이득은 External cube GT로 판정한다. |
| Final protocol | B2 -> A4 | Soft FK 조건에서 board residual이 cube 보정에 도움 되는가 | External cube GT + heldout cube RMSE | Cube 4.4827 -> 3.5805 (-0.9023) | External cube GT와 heldout cube RMSE로 판정한다. |
| Final protocol | A3 -> A5 | Raw FK hard fixed와 vision-aligned FK hard fixed의 차이는 무엇인가 | External cube GT + heldout cube RMSE | Cube 6.3959 -> 3.2274 (-3.1685) | A5가 GT 공개 전에 frozen method이면 최종 후보로 판정 가능하다. |
| Final protocol | A4 -> A5 | 같은 aligned FK를 soft factor와 hard fixed로 쓰면 무엇이 달라지는가 | External cube GT + heldout cube RMSE | Cube 3.5805 -> 3.2274 (-0.3531) | A5가 GT 공개 전에 frozen method이면 최종 후보로 판정 가능하다. |

> A5는 External GT 공개 전에 방법·파라미터·alignment artifact가 frozen이면 최종 후보로 비교할 수 있다. GT를 본 뒤 A5를 정의하면 사후 진단으로만 남긴다.

## Metric Decision Matrix (평가지표 판정표)

| Metric (지표) | Tier (등급) | Use (사용법) | Limit (제한) | Current Support (현재 근거) |
| --- | --- | --- | --- | --- |
| External cube TRE / rotation / P95 / failure | Final primary metric | 독립 External GT cube pose와 blind prediction을 비교해 최종 순위를 정함 | GT 측정계 uncertainty floor보다 작은 차이는 주장하지 않는다. | pending; External GT 추가 후 산출 |
| ALL Cube RMSE px | Fit sanity check | train+heldout 전체 cube evaluation data에 frozen calibration을 적용 | train과 heldout을 섞으므로 일반화 지표가 아니다. | cube 9 obs / 108 corners |
| Train reprojection RMSE | Solver diagnostic | 수렴/적합 상태 확인 | 학습 관측에 대한 fit이므로 방법 우월성 지표가 아니다. | row별 train residual |
| Heldout Cube RMSE px | Internal support metric | 미사용 cube event corner에 frozen transform을 적용해 재투영 | 같은 set의 다른 event라 새 위치 일반화나 물리 GT가 아니다. | heldout cube 236 corners |
| Cross-view pixel transfer RMSE | Supplementary camera consistency | 한 카메라 PnP pose를 다른 카메라로 전달해 cube corner px 오차 계산 | 모든 고정카메라에 함께 존재하는 systematic error와 절대 물리 오차를 검출하지 못한다. | fixed cameras 0, 1, 3; gripper camera pair 포함 |
| Cam-common Obj-Cam consistency mm/deg | Supplementary camera consistency | 두 카메라가 계산한 cube object pose 차이를 mm/deg로 집계 | 공통 계통오차는 검출하지 못하며 외부 GT 순위용이 아니다. | fixed-camera pair와 fixed-gripper pair를 cube-only로 함께 집계 |

> `Convergence 3/3`은 서로 다른 초기화 seed 3회 모두에서 SciPy solver가 `success=True`로 종료됐다는 뜻이다. Sequential 행은 두 stage가 모두 성공해야 하며, B1은 stage 1과 모든 fixed-camera stage 2가 성공해야 1회 수렴으로 센다. 이는 solver 종료 조건 충족을 뜻할 뿐, 절대 정확도나 전역 최적해를 보장하지 않는다.

## Cross-view Camera Consistency (cube-only)

고정카메라 pair와 고정카메라↔그리퍼카메라 pair를 같은 cross-view cube consistency 지표 안에서 함께 집계한다. 별도 pair-type 순위 지표는 만들지 않는다.

| Method (방법) | Cross-view Cube px | Cam-common Cube mm | Cam-common Cube deg | Support |
| --- | ---: | ---: | ---: | --- |
| A0 | 7.2577 | 8.6775 | 0.9037 | 9 fixed-camera pairs + 27 fixed-gripper pairs |
| A1 | 7.3085 | 8.8294 | 1.0503 | 9 fixed-camera pairs + 27 fixed-gripper pairs |
| A2 | 6.3003 | 7.2868 | 1.0280 | 9 fixed-camera pairs + 27 fixed-gripper pairs |
| A3 | 6.8626 | 8.3382 | 2.0892 | 9 fixed-camera pairs + 27 fixed-gripper pairs |
| A4 | 6.3126 | 7.3077 | 1.0151 | 9 fixed-camera pairs + 27 fixed-gripper pairs |
| A5 | **5.7166** | **6.6745** | **0.8862** | 9 fixed-camera pairs + 27 fixed-gripper pairs |
| B1 | 7.2867 | 8.8059 | 1.0540 | 9 fixed-camera pairs + 27 fixed-gripper pairs |
| B2 | 6.5284 | 7.4292 | 1.0493 | 9 fixed-camera pairs + 27 fixed-gripper pairs |
| B3 | 7.2550 | 8.6736 | 0.9035 | 9 fixed-camera pairs + 27 fixed-gripper pairs |

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

Cross-view pixel transfer는 한 카메라의 측정 PnP pose를 다른 카메라로 전달해 cube corner pixel error를 계산한다. 고정카메라 pair와 고정카메라↔그리퍼카메라 pair를 같은 보조 지표로 함께 집계하고, 별도 pair-type 순위 지표는 만들지 않는다.

Heldout Cube RMSE는 미사용 cube event corner에 frozen transform을 적용해 계산한다.

$$RMSE_{px}=\sqrt{\frac{1}{2N}\sum_k((u_k-\hat u_k)^2+(v_k-\hat v_k)^2)}$$

ALL Cube RMSE는 train cube와 heldout cube를 같은 방식으로 계산한 뒤 corner 수로 가중해 합친 fit sanity check다.

## Interpretation Limit (해석 한계)

Cross-view pixel transfer와 Cam-common Obj-Cam consistency는 방법별 추정값에 의존하므로 공통 systematic error를 검출하지 못한다. 따라서 최종 주장은 External cube GT로만 결정한다.

## Terminology (용어 설명)

- **$T^B_{C_i}$, Base-to-Fixed-Camera Transform (베이스–고정카메라 변환)**: 고정카메라 외부 파라미터.
- **$T^G_{C_g}$, Hand–Eye Transform (핸드–아이 변환)**: 그리퍼에서 그리퍼카메라로의 변환.
- **$T^B_G(e)$, Robot FK Pose (이벤트별 로봇 순기구학 자세)**: 이벤트 $e$의 베이스–그리퍼 변환이며 평가 중 고정 입력이다.
- **PnP, Perspective-n-Point (3D–2D 자세 추정)**: 3D 표적점과 2D 영상점으로 카메라–표적 자세를 계산한다.
- **RMSE, Root Mean Squared Error (평균제곱근오차)**: 잔차 제곱 평균의 제곱근. px, mm, deg는 서로 합치지 않는다.
- **External cube GT**: GT 공개 전 저장한 blind prediction과 독립 cube GT pose를 비교하는 최종 주 지표.

## External GT Task (다음주 예정 태스크)

Independent External GT가 들어오면 모든 row의 cube pose prediction을 같은 GT cube pose list와 비교한다. 최종 결과는 Translation Error, Rotation Error, P95, Failure Rate로 산출한다.
