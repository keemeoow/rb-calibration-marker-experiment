# Session04 Calibration Evaluation (캘리브레이션 평가)

> Status: Final protocol before External GT. 비교 행은 A0~A5, B1~B3 한 벌만 사용하고, heldout 평가는 항상 cube만 본다.

## Result at a Glance (결과 한눈에 보기)

| 핵심 질문 | 현재 Session04 내부 결과 | 허용되는 해석 |
| --- | --- | --- |
| 현재 내부 후보는 무엇인가 | A5: Heldout 3.4180 px, Cross-view 5.6072 px, Cam-common 6.6745 mm / 0.8862 deg | heldout과 두 camera-consistency 지표에서 모두 최소인 최종 후보. 물리 정확도 1위 확정은 External GT 이후 |
| FK hard fixed는 유효한가 | A2 3.5960 -> A3 6.7199 px | 현재 데이터에서는 +3.1239 px 악화되어 채택 근거가 없음 |
| corrected-FK soft factor 이득은 큰가 | A2 3.5960 -> A4 3.5786 px | 개선은 0.0174 px로 작아 External GT 없이 우수성을 주장하기 어려움 |
| Train 최소가 최종 우수성을 뜻하는가 | B2 Train 3.0201 px, Heldout 4.4608 px | 아니오. Train RMSE는 동일 관측에 대한 in-sample fit 진단 |
| 최종 결론이 확정됐는가 | External cube GT: pending | 현재는 A5를 사전 고정할 근거까지이며 최종 물리 순위는 미확정 |

## Current Data Warnings (현재 데이터 경고)

> 아래 수치는 기존 `data/session04`를 재평가한 **내부 preflight 결과**다. 새 `CAPTURE_PROTOCOL.md`의 composite rig 45-event 촬영 결과가 아니므로, 최종 논문 수치로 확정하지 않는다.

| 점검 항목 | 현재 데이터 | 결과 해석에 미치는 영향 |
| --- | --- | --- |
| 평가 support | fixed cameras `0, 1, 3`; overall 17 obs / 340 corners; board 8 / 232, cube 9 / 108 | 현재 촬영에서 공통으로 관측된 범위만 평가 |
| Split support | 9 eligible sets; dropped sets `0, 1, 2, 3` | 제외 set을 숨기지 않고 모든 방법에 동일 적용 |

## Final Protocol Lock (최종 단일 기준)

| 항목 | 최종 고정 기준 | 공정성 이유 |
| --- | --- | --- |
| 비교 행 | A0~A5, B1~B3 한 벌만 사용 | 같은 row 정의를 모든 데이터와 문서에서 유지 |
| 촬영 예산 | 새 촬영은 composite rig 45 planned events와 동일 pose ID 사용 | 검출 marker 수가 아니라 raw capture opportunity를 동일하게 통제 |
| Marker ablation | 같은 raw image에서 row별 board/cube observation만 사전 정의대로 masking | A0/B3에 유리한 별도 board 촬영을 추가하지 않음 |
| 평가 target | heldout, cross-view, External GT 모두 cube-only | 모든 row를 동일한 실제 3D target으로 평가 |
| 최종 주 지표 | External cube TRE / rotation / P95 / failure | 내부 재투영 오차로 물리 정확도를 확정하지 않음 |
| 내부 보조 지표 | Heldout Cube, Cross-view, Cam-common; Train/ALL은 fit 진단 | 같은 support와 frozen transform으로 계산 |
| A5 해석 | External GT 공개 전에 방법, 파라미터, alignment artifact 고정 | GT를 본 뒤 방법을 선택하는 사후 편향 방지 |

> 이 표는 **새 최종 촬영의 계약**이다. 아래 결과 수치는 이 계약 적용 전 legacy Session04 내부 데이터이므로 설계 검증용 preflight로만 사용한다.

## Final Comparison Table (최종 비교실험표)

### 비교실험 구성

| Method (방법) | Calibration 입력 target | 최적화 구조 | FK / target pose 처리 | 직접 검증하는 질문 |
| --- | --- | --- | --- | --- |
| A0 (baseline) | Board only (동일 composite-rig events) | Sequential (stage별 frozen) | VISION (board pose free); Cube: evaluation only | Board-only sequential baseline |
| A1 (+cube) | Board + Cube | Sequential (stage별 frozen) | VISION (cube pose free) | A0 대비 cube 관측 추가 효과 |
| A2 (+unified) | Board + Cube | Unified joint optimization | VISION (cube pose free) | A1 대비 unified feedback 효과 |
| A3 (FK hard fixed) | Board + Cube | Unified joint optimization | FK hard fixed | A2 대비 FK hard fixed 효과 |
| A4 (corrected-FK soft factor) | Board + Cube | Unified joint optimization | corrected-FK soft factor | A2 대비 corrected-FK soft factor 효과 |
| A5 (corrected-FK hard fixed (VISION-aligned)) | Board + Cube | Unified joint optimization | corrected-FK hard fixed (VISION-aligned) | A3/A4 대비 corrected-FK hard fixed 효과 |
| B1 (−Unified) | Board + Cube | Sequential (stage별 frozen) | corrected-FK soft factor | A4와 같은 corrected-FK soft factor에서 sequential 효과 |
| B2 (−board) | Cube only | Unified joint optimization | corrected-FK soft factor | A4 대비 board residual 제거 효과 |
| B3 (−cube) | Board only (동일 composite-rig events) | Unified joint optimization | VISION (board pose free); Cube: evaluation only | A2 대비 cube residual 제거; A0/B3 구조 대조 |

> A0/B3는 calibration objective에서 cube를 완전히 가린다. 다만 평가 때는 다른 행과 동일하게 calibration을 frozen하고 train cube 관측으로 set별 evaluation pose만 맞춘 뒤 cube-only 지표를 계산한다.

### Cube 재투영 결과

모든 값은 초기화 seed 3회의 평균이다. 굵은 값은 현재 내부 Heldout Cube RMSE 최솟값이며, External GT 기반 최종 순위가 아니다.

| Method (방법) | Train Cube RMSE px | ALL Cube RMSE px | Heldout Cube RMSE px | Heldout-Train px | Convergence |
| --- | ---: | ---: | ---: | ---: | ---: |
| A0 (baseline) | 3.4489 | 3.5063 | 3.6768 | 0.2278 | 3/3 |
| A1 (+cube) | 3.3414 | 3.4314 | 3.6938 | 0.3525 | 3/3 |
| A2 (+unified) | 3.3525 | 3.4140 | 3.5960 | 0.2435 | 3/3 |
| A3 (FK hard fixed) | 4.2878 | 4.9967 | 6.7199 | 2.4321 | 3/3 |
| A4 (corrected-FK soft factor) | 3.3547 | 3.4111 | 3.5786 | 0.2239 | 3/3 |
| A5 (corrected-FK hard fixed (VISION-aligned)) | 3.5312 | 3.5037 | **3.4180** | -0.1132 | 3/3 |
| B1 (−Unified) | 3.3450 | 3.4322 | 3.6870 | 0.3420 | 3/3 |
| B2 (−board) | 3.0201 | 3.4308 | 4.4608 | 1.4407 | 3/3 |
| B3 (−cube) | 3.4489 | 3.5061 | 3.6763 | 0.2275 | 3/3 |

> `Heldout-Train`은 heldout RMSE에서 train RMSE를 뺀 진단값이다. 양수면 heldout 오차가 더 크다는 뜻이지만, 음수라고 새 위치 일반화가 증명되는 것은 아니다.

> `Convergence 3/3`은 세 seed에서 solver 종료 조건을 충족했다는 뜻일 뿐, 절대 정확도나 전역 최적해를 보장하지 않는다. A4의 measured FK covariance는 아직 대기 중이며, A5는 External GT 채점 전에 방법과 alignment artifact를 고정해야 한다.

### Cross-view Camera Consistency (카메라 간 일관성 결과, cube-only)

고정카메라 pair와 고정카메라-그리퍼카메라 pair의 원시 오차를 같은 frozen mask에서 직접 pooling한다. px는 destination-corner 수로, mm/deg는 pair 수로 가중한다. 별도 pair-type 순위는 만들지 않는다.

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

## Metric Decision Matrix (평가지표 판정표)

### 평가지표 (한글로): 설명, 평가 지표 낸 방법

| Metric (평가지표) | 역할 | 계산 방법 | 공정성 통제와 한계 | Current Support (현재 근거) |
| --- | --- | --- | --- | --- |
| External cube TRE / rotation / P95 / failure | 최종 물리 정확도 주 지표 | GT 공개 전에 저장한 blind cube pose와 동일 pose ID의 독립 GT를 paired 비교한다. translation norm, SO(3) geodesic rotation, P95, 사전 정의 failure rate를 계산한다. | 모든 방법에 같은 GT pose list와 failure threshold를 적용한다. GT 측정 uncertainty floor보다 작은 차이는 주장하지 않는다. | Pending; External GT 추가 후 산출 |
| Heldout Cube RMSE px | External GT 전 내부 보조 지표 | train cube만으로 set별 evaluation pose를 맞춘 뒤 calibration과 pose를 frozen하고, 미사용 heldout cube corner를 재투영한다. | 모든 방법에 같은 cube corner와 split을 쓰고 test-time refit을 금지한다. 같은 set의 다른 event이므로 새 위치 일반화나 물리 GT는 아니다. | 동일 heldout cube 236 corners |
| Cross-view pixel transfer RMSE | 카메라 간 pixel 일관성 보조 지표 | 한 카메라의 cube PnP pose를 상대 카메라로 전달하고 양방향 destination-corner squared error를 직접 pooling해 px RMSE를 계산한다. | 모든 방법에 동일한 양방향 pair mask를 쓴다. fixed-gripper pair에는 Hand-Eye/FK와 train fixed-anchor가 섞이며 공통 systematic error를 검출하지 못한다. | 36 pairs (9 fixed-fixed + 27 fixed-gripper; 18 train-anchor), 72 directions / 904 destination-corners |
| Cam-common Obj-Cam consistency mm/deg | 카메라 간 3D pose 일관성 보조 지표 | 같은 frozen cube pair에서 두 camera 경로가 계산한 `T_base_cube`의 translation norm과 SO(3) rotation 차이를 pair-pooled RMSE로 계산한다. | Cross-view px와 동일 pair discrepancy의 다른 단위 표현이므로 독립 증거가 아니며 공통 systematic error를 검출하지 못한다. | 36 pairs (9 fixed-fixed + 27 fixed-gripper; 18 train-anchor), 72 directions / 904 destination-corners |
| ALL Cube RMSE px | 전체 fit sanity check | train cube로 맞춘 set별 evaluation pose와 frozen calibration을 train+heldout cube corner 전체에 적용해 corner-pooled RMSE를 계산한다. | 모든 방법에 같은 cube 모집단을 쓰지만 train과 heldout을 섞고 train이 약 75%를 차지하므로 일반화 순위 지표가 아니다. | train+heldout cube 960 corners (724 + 236) |
| Train Cube RMSE px | Train-split fit 진단 | frozen calibration에서 train cube로 set별 evaluation pose를 맞춘 뒤 같은 train cube corner를 재투영한다. | 모든 방법에 같은 cube 모집단을 쓰지만 pose를 맞춘 관측을 다시 채점하는 in-sample 값이므로 방법 순위 지표가 아니다. | 동일 train cube 724 corners |

### 공통 계산 규칙

$$RMSE_{px}=\sqrt{\frac{1}{2N}\sum_k((u_k-\hat u_k)^2+(v_k-\hat v_k)^2)}$$

$$T^{B,(i)}_{cube}=T^B_{C_i}T^{C_i}_{cube,\mathrm{PnP}},\qquad T^{B,(g)}_{cube}(e)=T^B_G(e)T^G_{C_g}T^{C_g}_{cube,\mathrm{PnP}}$$

$$e_t=\lVert t_{pred}-t_{GT}\rVert_2,\qquad e_R=\cos^{-1}((\operatorname{tr}(R_{GT}^{T}R_{pred})-1)/2)$$

- 카메라와 hand-eye transform은 방법별 calibration 종료 후 frozen한다.
- Heldout cube corner는 calibration, evaluation-pose fit, threshold 선택에 쓰지 않는다.
- 내부 지표는 같은 support를 직접 pooling하고, seed 3개 평균은 반복 실험 표본으로 해석하지 않는다.
- px, mm, deg는 서로 합치지 않고 각각 별도 열로 보고한다.

## Matched Contrast Decision Table (비교실험 구성 확정표)

최종 비교는 아래 contrast만 사용한다. 모든 heldout 평가는 cube만 보며, External GT가 들어오면 같은 cube pose list에서 paired comparison으로 판정한다.

> 검증 질문은 새 45-event 최종 설계 기준이다. 아래 Session04 delta는 대응되는 legacy preflight이며, 새 촬영 후 같은 contrast를 다시 계산해야 한다.

음수 delta는 두 번째 방법의 Heldout Cube RMSE가 개선됐다는 뜻이다.

| Direct Contrast (직접 비교) | Question (검증 질문) | Session04 Heldout Cube 결과 | Decision (판정) |
| --- | --- | --- | --- |
| A0 -> B3 | 단일 target에서 sequential과 unified가 사실상 같아지는가 | Cube 3.6768 -> 3.6763 (-0.0004) | 구조 구현의 negative control이다. 현재 0.0004 px 차이로 기대한 동등성을 지지한다. |
| A0 -> A1 | 같은-event board-only baseline에 cube train 관측을 추가하면 cube 평가가 개선되는가 | Cube 3.6768 -> 3.6938 (+0.0171) | 현재는 0.0171 px 악화로 내부 개선 근거가 없다. External GT로 재판정한다. |
| A1 -> A2 | VISION 조건에서 unified feedback이 도움이 되는가 | Cube 3.6938 -> 3.5960 (-0.0978) | 현재 0.0978 px 개선으로 unified feedback을 약하게 지지한다. |
| B3 -> A2 | unified 구조에서 cube residual이 최종 cube 평가에 필요한가 | Cube 3.6763 -> 3.5960 (-0.0803) | 현재 0.0803 px 개선으로 cube residual의 내부 이득을 약하게 지지한다. |
| A2 -> A3 | VISION cube pose를 FK hard fixed로 바꾸면 어떤가 | Cube 3.5960 -> 6.7199 (+3.1239) | 현재 3.1239 px 악화되어 FK hard fixed를 반박한다. |
| B1 -> A4 | 같은 corrected-FK soft factor에서 sequential과 unified 중 무엇이 나은가 | Cube 3.6870 -> 3.5786 (-0.1084) | 현재 0.1084 px 개선으로 unified 구조를 약하게 지지한다. |
| A2 -> A4 | Unified VISION에 corrected-FK soft factor를 추가하면 이득이 있는가 | Cube 3.5960 -> 3.5786 (-0.0174) | 현재 개선은 0.0174 px로 작아 corrected-FK 우수성 근거로 부족하다. |
| B2 -> A4 | corrected-FK soft factor 조건에서 board residual이 cube 보정에 도움 되는가 | Cube 4.4608 -> 3.5786 (-0.8821) | 현재 0.8821 px 개선으로 board residual의 내부 이득을 지지한다. |
| A3 -> A5 | FK hard fixed와 corrected-FK hard fixed의 차이는 무엇인가 | Cube 6.7199 -> 3.4180 (-3.3019) | 현재 3.3019 px 개선으로 raw frame mismatch 보정 필요성을 지지한다. |
| A4 -> A5 | 같은 corrected-FK를 soft factor와 hard fixed로 쓰면 무엇이 달라지는가 | Cube 3.5786 -> 3.4180 (-0.1606) | 현재 0.1606 px 개선으로 A5를 External GT 전 고정할 후보로 둔다. |

> A5는 External GT 공개 전에 방법·파라미터·alignment artifact가 frozen이면 최종 후보로 비교할 수 있다. GT를 본 뒤 A5를 정의하면 사후 진단으로만 남긴다.

## Objective Block Diagnostics (목적함수 블록 진단)

| Method (방법) | FK 처리 | Visual residual components (시각 잔차 수) | FK blocks / components (FK 블록/잔차 수) | Visual robust cost (시각 비용) | FK robust cost (FK 비용) | FK cost fraction (FK 비용 비율) |
| --- | --- | ---: | ---: | ---: | ---: | ---: |
| A2 | VISION | 7372 | 0 / 0 | 23950.81 | 0.00 | 0.000% |
| A3 | FK hard fixed (hard constant; residual 없음) | 7372 | 0 / 0 | 36683.56 | 0.00 | 0.000% |
| A4 | corrected-FK soft factor | 7372 | 9 / 54 | 23960.94 | 29.28 | 0.122% |
| A5 | corrected-FK hard fixed (VISION-aligned) (hard constant; residual 없음) | 7372 | 0 / 0 | 25788.40 | 0.00 | 0.000% |
| B1 | corrected-FK soft factor | 6068 | 9 / 54 | 23343.41 | 33.96 | 0.145% |
| B2 | corrected-FK soft factor | 1448 | 9 / 54 | 3348.98 | 38.57 | 1.139% |

> 이 비율은 최종 목적함수 값의 분해다. 각 항의 Jacobian과 변수 연결 구조가 다르므로, FK cost 비율을 파라미터 영향력 비율로 해석하면 안 된다.

## Terminology (용어 설명)

- **$T^B_{C_i}$, Base-to-Fixed-Camera Transform (베이스–고정카메라 변환)**: 고정카메라 외부 파라미터.
- **$T^G_{C_g}$, Hand–Eye Transform (핸드–아이 변환)**: 그리퍼에서 그리퍼카메라로의 변환.
- **$T^B_G(e)$, Robot FK Pose (이벤트별 로봇 순기구학 자세)**: 이벤트 $e$의 베이스–그리퍼 변환이며 평가 중 고정 입력이다.
- **PnP, Perspective-n-Point (3D–2D 자세 추정)**: 3D 표적점과 2D 영상점으로 카메라–표적 자세를 계산한다.
- **RMSE, Root Mean Squared Error (평균제곱근오차)**: 잔차 제곱 평균의 제곱근. px, mm, deg는 서로 합치지 않는다.
- **External cube GT**: GT 공개 전 저장한 blind prediction과 독립 cube GT pose를 비교하는 최종 주 지표.

## External GT Task (다음주 예정 태스크)

Independent External GT가 들어오면 모든 row의 cube pose prediction을 같은 GT cube pose list와 비교한다. 최종 결과는 Translation Error, Rotation Error, P95, Failure Rate로 산출한다.
