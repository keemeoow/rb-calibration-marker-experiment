# Calibration Experiment — 최종 비교실험 명세

이 문서는 실제 셋업에서 제안 방법의 기여를 분리하고, “Ours가 3D 공간 정합에서 우수하다”는
주장을 어떤 조건에서 허용할지 정한 최종 실험 계약이다. 과거 디버깅 수치와 내부 감사 과정은
반복하지 않고 원본 산출물에 남긴다.

현재 결론은 다음과 같다.

- Unified visual calibration의 held-out pixel consistency 개선은 기존 데이터에서 확인됐다.
- Soft-FK는 hard-FK보다 합리적인 제안 방향이지만, 공분산 기반 robust factor는 아직 구현·평가되지 않았다.
- 현재 1.586 mm는 외부 GT가 아닌 FK-proxy translation 결과다. 절대 3D 정확도 근거가 아니다.
- 따라서 외부 GT 실험 전에는 “Ours가 가장 정확하다”고 결론 내리지 않는다.

## 방법 정의

모든 방법은 운동학 백본 `T_base_gripper = FK(q)`를 공통으로 사용한다. 실험 축의 `FK`는 이
백본의 사용 여부가 아니라, **cube pose에 FK 정보를 어떤 방식으로 주입하는가**를 뜻한다.

- `seq`: eye-in-hand와 eye-to-hand를 순차적으로 풀고 앞 단계 결과를 동결한다.
- `U-BA`: 고정·손목 카메라, hand-eye, target pose를 하나의 pixel-level bundle adjustment에서 푼다.
- `hard-FK`: cube pose를 FK pose에 고정한다.
- `soft-FK`: cube pose를 자유변수로 두고 covariance-weighted robust FK factor를 추가한다.
- `correction`: calibration 이후 held-out target pose 예측에만 적용한다. Calibration transform은 바꾸지 않는다.

제안 soft-FK residual은 다음을 기본형으로 한다.

```text
r_FK = Log(T_FK^-1 T_cube)
E_FK = rho(r_FK^T Sigma_FK^-1 r_FK)
```

`Sigma_FK`와 robust-loss scale은 별도 반복측정 또는 training-only inner validation에서 정하고 test에서
동결한다. 기존 상수 가중치 `lambda=3 px/mm`는 A4의 근접 선행 실험일 뿐 최종 공분산 모델이 아니다.

## Table 1 — Main Ablation (실제 셋업)

Table 1에는 제안 기여를 직접 검증하는 A/B 계열만 둔다. 모든 행은 동일한 raw frames, corner
detections, train/test mask, robust frontend, initialization budget과 공통 평가 mask를 사용한다.

| ID | 방법 / 검증 기여 | Target | Solve | FK→cube | Correction | 반드시 비교할 paired contrast | 현재 상태 |
| --- | --- | --- | --- | --- | --- | --- | --- |
| A0 | Board-only sequential — 평면 타깃 기준선 | board | seq | 해당 없음 | 없음 | A0→A1: cube 추가에 따른 관측성·등록률 변화 | 기존 결과 있음 |
| A1 | Cube+board sequential — cube 관측성 | cube+board | seq | vision-estimated | 없음 | A0→A1; A1→A2 | 기존 결과 있음 |
| A2 | Unified visual-only — 통합 최적화 | cube+board | U-BA | 사용 안 함; cube 자유변수 | 없음 | **A1→A2:** solve만 seq→U | 기존 pose-level 결과 있음; 최종 pixel-level 공통 backend 재실행 필요 |
| A3 | Unified hard-FK — FK 강제·안정성 | cube+board | U-BA | hard constraint | 없음 | **A2→A3:** FK factor none→hard | 기존 결과 있음; 정확도보다 조건수·수렴 baseline |
| **A4★** | **Unified soft-FK (Ours-core)** — 제안 캘리브레이션 원리 | **cube+board** | **joint pixel-level U-BA + robust loss** | **covariance-weighted robust factor** | 없음 | **A2→A4:** soft factor 기여; **A3→A4:** hard→soft 효과; **B1→A4:** independent→joint 효과 | **최종 구현·외부 GT 평가 필요** |
| A5 | Unified soft-FK + 6-DoF correction — Ours-full 후보 | cube+board | A4와 동일 | A4와 동일 | train-only SE(3) residual correction | **A4→A5:** correction만 추가 | 미구현. 통과 전에는 Ours-full로 부르지 않음 |
| B1 | Robust independent — 공정 독립 baseline | cube+board | 카메라별 독립 robust solve + rigid composition | A4와 동일한 `Sigma_FK` 정보량 | 없음 | **B1→A4:** joint optimization의 순수 효과 | 공정 arm 미실행. 기존 sequential/FK-fixed B1으로 대체 금지 |
| B2 | Cube-only unified — board 기여 | cube | A4와 동일 | A4와 동일 soft-FK | 없음 | **B2→A4:** board만 추가 | soft-FK 공정 arm 미실행 |
| B3 | Board-only unified — cube 기여 | board | A2와 동일 | 해당 없음 | 없음 | **B3→A2:** cube만 추가 | 기존 행 있음; paired delta 재산출 필요 |

### 행별 인과 해석

- `A1→A2`만 Unified의 순수 효과로 사용한다.
- `A2→A4`는 visual-only 대비 soft-FK factor의 효과다.
- `A3→A4`는 같은 FK 정보를 hard constraint 대신 robust soft factor로 넣는 효과다.
- `A4→A5`만 correction의 추가 효과다. `A2→A5`를 correction 단순효과로 해석하지 않는다.
- Board의 효과는 `B2→A4`, cube의 효과는 `B3→A2`로 판단한다.
- `B3→A3`는 marker와 FK 조건을 동시에 바꾸므로 cube 기여의 근거로 사용하지 않는다.
- A0/A1은 관측성·전통 baseline이다. 제안 방법의 joint 효과는 정보량을 맞춘 `B1→A4`로 판단한다.

## Table 2 — Classical Hand-eye Baselines (별도 비교)

Classical 방법은 mixed eye-in+eye-to joint calibration이 아니므로 Main Ablation에 포함하지 않는다.
동일 pose frontend와 동일 raw observations로 hand-eye를 계산한 후, 같은 규칙으로 fixed-camera 경로를
합성해 최종 task에서 평가한다.

| ID | 방법 | 기본 형태 | 공정 비교 조건 | 상태 |
| --- | --- | --- | --- | --- |
| C1 | Tsai–Lenz | AX=XB, rotation/translation 분리 | 동일 robust pose frontend·motion pairs·평가 mask | 미실행 |
| C2 | Park–Martin | Lie algebra 기반 AX=XB | C1과 동일 | 미실행 |
| C3 | Horaud | quaternion 기반 AX=XB | C1과 동일 | 미실행 |
| C4 | Daniilidis | dual-quaternion AX=XB | C1과 동일 | 미실행 |

Classical 방법은 적용 범위가 A4보다 좁다는 점을 명시한다. 그러나 동일한 외부-GT 최종 task 결과는
직접 비교할 수 있다. 네 방법은 동일 입력으로 실행 비용이 낮으므로 최종 실험에는 모두 포함하는 것을
기본안으로 한다.

## 평가 지표

### 1차 지표 — 우수성 판정

| 지표 | 정의 | 보고값 | 주의 |
| --- | --- | --- | --- |
| `TRE_t` (mm) | 독립 외부 GT와 예측 target 중심의 3D 거리 | mean, P50, P95, max, 95% CI | **최우선 지표**. FK를 GT로 사용 금지 |
| `e_R` (deg) | `Log(R_GT^T R_pred)`의 geodesic angle | mean, P50, P95, max, 95% CI | 위치-only Ridge는 이 지표를 개선할 수 없음 |
| ADD / ADD-S (mm) | GT·예측 pose로 변환한 object surface point의 평균 거리 | mean, P95, 95% CI | cube 대칭을 인정하면 ADD-S 사용 |
| Workspace error | center/edge/near/far/height 구간별 `TRE_t` | 구간별 P50/P95 | train 영역 안·밖을 분리 |
| Reliability | calibration failure rate, `N_reg` | session별 성공률과 등록 카메라 수 | 실패 run을 오차 평균에서 조용히 제외 금지 |

외부 GT는 A4/A5의 FK factor, correction label, intrinsic calibration과 독립이어야 한다. 독립성이
확보되지 않으면 지표 이름에 `FK-proxy` 또는 `internal`을 붙이고 1차 지표로 사용하지 않는다.

### 2차 지표 — 원인 분석·운용성

| 지표 | 정의 | 용도 |
| --- | --- | --- |
| Held-out raw-corner reprojection | 공통 held-out 실제 2D corner에 frozen transform을 재투영한 RMSE/median/P95 | pixel fit과 일반화 진단 |
| Cross-camera disagreement | 같은 target의 카메라별 3D 예측 산포(mm/deg) | 내부 정합 진단. 절대 정확도 아님 |
| Transform repeatability | 독립 재설치 session 간 `bTf`, `gTc` 변화(mm/deg) | 재현성 |
| Depth alignment | point-to-plane 또는 point-to-CAD distance(mm) | scale·3D geometry 교차검증 |
| Efficiency | runtime, iterations, condition number, 필요한 views | 실용성·수렴 안정성 |

`e_X`처럼 `T_base_Ci`와 `T_gripper_cam`을 하나로 평균한 숫자는 사용하지 않는다. Simulation 또는
외부 transform GT가 있는 경우 다음을 분리 보고한다.

- 고정 카메라 `bTf`: translation mm / rotation deg
- hand-eye `gTc`: translation mm / rotation deg
- 카메라 간 상대변환: translation mm / rotation deg

## 통계 계약과 최종 합격 조건

- 통계 단위는 frame이나 split이 아니라 **독립 camera-installation session**이다.
- 같은 session의 동일 held-out poses에서 모든 방법을 paired 평가한다.
- 방법별 mean과 함께 P50/P95/max 및 paired bootstrap 95% CI를 보고한다.
- Ours와 여러 baseline을 비교할 때 p-value는 Holm 보정한다.
- 실패 run도 failure rate에 포함하며 성공 run만 골라 정확도를 비교하지 않는다.
- Hyperparameter, `Sigma_FK`, robust threshold, correction model은 training-only로 선택한다.

Ours-core A4가 우수하다고 판정하려면 다음을 모두 만족해야 한다.

1. 주 baseline 각각에 대해 `delta TRE = TRE_A4 - TRE_baseline`의 paired bootstrap 95% CI 상한이 0보다 작다.
2. 회전오차가 유의하게 낮거나, 사전 정의한 task 기반 non-inferiority margin 안에 든다.
3. P95 TRE와 calibration failure rate가 baseline보다 악화되지 않는다.
4. Workspace 특정 구간에서만 이기고 전체 평균으로 숨기는 현상이 없다.

A5는 A4 대비 translation과 rotation 모두 같은 조건을 통과할 때만 Ours-full로 승격한다. 그렇지
않으면 A4를 최종 방법으로 유지하고 correction은 supplementary 결과로 보고한다.

## 현재 실제 데이터가 보여주는 범위

아래 값은 기존 canonical event split과 cross-target artifact의 요약이다. 외부 GT가 아니며 서로
다른 split·robot scale 값이 섞인 수치를 한 행의 절대 성능처럼 읽지 않는다.

| 행 | `N_reg` | 기존 `e_e2e` (mm/deg) | 기존 `e_cross` (mm) | 공통 cube reproj (px) | 해석 |
| --- | ---: | ---: | ---: | ---: | --- |
| A0 | 2 | — | — | 10.9130±1.0401 | Board-only는 cam3 미등록. 관측성 한계 |
| A1 | 3 | 16.3323 / 7.7466 | 39.5978 | 10.5768±1.0394 | Cube 추가 후 3대 등록 |
| A2 | 3 | 16.1955 / 7.7423 | 38.9338 | 10.5170±1.0399 | A1 대비 held-out overall reproj 5/5 split 개선 |
| A3 | 3 | 16.1881 / 7.7087 | 38.0480 | 10.5042±1.0581 | 조건수·수렴 안정성은 좋지만 외부 GT 정확도 미검증 |
| A4 근접 arm (`lambda=3`) | 3 | 15.1168 / 7.7342 | 35.0872 | 10.4968±1.0487 | 공분산 기반 A4가 아닌 상수 soft anchor 선행 결과 |
| A5 위치-only interim | 3 | calibration 지표는 A4와 동일 | calibration 지표는 A4와 동일 | calibration 지표는 A4와 동일 | FK-proxy translation 1.586 mm. 6-DoF·절대 GT 근거 아님 |
| B3 | 2 | — | — | 10.9129±1.0401 | Cube 단순효과 paired delta 미산출 |

확인된 paired 사실만 사용한다.

- `A1→A2`: held-out overall reprojection `-0.1486±0.0216 px`, 5/5 split 개선.
- `A2→A3`: 기존 own-target overall reprojection `+0.1155±0.0253 px`, 0/5 split 개선.
- 기존 soft anchor는 cross-camera disagreement를 줄였지만 외부 3D GT 우위를 뜻하지 않는다.
- 위치-only correction의 1.586 mm는 FK-proxy이므로 논문의 절대 정확도 헤드라인으로 사용하지 않는다.

세부 provenance:

- [Canonical repeated ablation](CP_result/ablation_multisplit/multisplit_ablation.md)
- [Cross-target cube evaluation](CP_result/cross_target_cube/cross_target_cube.md)
- [Soft-anchor event split](CP_result/D2_anchored_event_split/D2_anchored_event_split.md)
- [실험 설명과 제한](CP_EXPERIMENTS_README.md)

## 실행 계약

### 데이터·분할

- 모든 방법에 동일 raw RGB/depth, robot poses와 corner detections를 제공한다.
- Session 전체를 train/test로 분리한다. 같은 물리 위치의 프레임이 양쪽에 섞이지 않게 한다.
- Position holdout은 center/edge/near/far/height를 stratify한다.
- 공통 카메라 교집합 정확도와 전체 카메라 coverage를 함께 보고한다.
- 추가로 등록한 카메라의 코너를 정확도 평균에서 빼거나, 실패 카메라를 조용히 제외하지 않는다.

### 공통 frontend·solver 예산

- 동일 corner detector 결과 또는 완전히 동일한 재검출 설정을 사용한다.
- 동일 PnP-RANSAC, marker rejection, robust loss와 outlier threshold를 사용한다.
- 동일 multi-start 수, 최대 iteration, 종료조건을 사용한다.
- 실패·예외를 누락하지 않고 failure reason과 함께 저장한다.
- Train reprojection은 진단용이고 최종 순위는 held-out/외부-GT 지표로 정한다.

### Simulation

Simulation은 실제 실험 전 sanity check와 GT transform 오차 분석에 사용한다. 최종 재실행 전 다음을
수정해야 한다.

- visual-only/independent solver에서 `fk_cube`와 GT board pose를 초기값·해에 사용하지 않는다.
- independent rigid alignment를 실제 prediction에 적용한다.
- noisy raw 2D corners와 visibility mask를 저장하고 방법별 held-out reprojection을 계산한다.
- 실물 59 mm cube, marker center/roll, 카메라별 K/D와 실제 camera pose를 사용한다.
- 예외를 숨기지 않고 seed 단위 실패율을 보고한다.
- 최소 30 independent seeds와 사전 정의한 random/stratified splits를 사용한다.
- `bTf`와 `gTc` GT 오차를 분리한다.

Simulation 결과는 실제 외부-GT 결과를 대신하지 않는다.

## 확정 전 사용자 판단 항목

아래 항목은 코드가 자동으로 결정할 수 없으며 실험 책임자가 사전에 확정해야 한다.

1. **최종 주장 범위**
   - 권장: 우선 A4를 Ours-core로 고정한다.
   - A5는 6-DoF correction이 A4 대비 외부 GT에서 통과한 후에만 최종 방법으로 채택한다.
   - 위치-only correction을 유지하면 논문 주장도 “3D translation localization”으로 제한해야 한다.

2. **독립 외부 GT 장비·방법**
   - 우선순위: laser tracker/측정암 → motion capture rigid body → 정밀 가공 3D jig.
   - Robot FK, A4 anchor에 사용한 값, correction label은 외부 GT로 재사용할 수 없다.
   - 어떤 장비를 사용할지와 장비 정확도 사양을 기록해야 한다.

3. **로봇 위치 스케일**
   - `RB_ROBOT_POS_SCALE=1.0`과 `1.0229` 중 하나를 기존 결과에 맞춰 선택하지 않는다.
   - 인증 길이 기준물·depth·외부 계측으로 물리적으로 확정한 뒤 모든 결과를 한 값으로 재생성한다.

4. **회전 non-inferiority margin**
   - 로봇 task가 허용하는 자세 오차로 정한다. 결과를 본 뒤 margin을 정하면 안 된다.

5. **재촬영 규모**
   - 권장 pilot: 독립 재설치 5 sessions × session당 blind test pose 30개.
   - Pilot 분산으로 power analysis 후 최종 session 수를 확정한다.
   - Workspace center/edge/near/far/height와 cube orientation을 균형화한다.

6. **Classical baseline 범위**
   - 권장: C1–C4 모두 실행한다. 동일 입력으로 실행 비용이 작고 선택적 baseline이라는 비판을 줄인다.
   - 논문 Main Ablation에는 넣지 않고 별도 표로 보고한다.

7. **ADD와 ADD-S 선택**
   - Cube의 면 ID와 방향이 task에서 구분되면 ADD를 사용한다.
   - 대칭 방향을 동일 pose로 인정하는 task면 symmetry set을 사전 정의하고 ADD-S를 사용한다.

## 직접 실행해야 하는 물리 작업

- 큐브 변, marker 크기·중심·roll, ChArUco square 길이를 캘리퍼로 재측정한다.
- 카메라별 30–50장 intrinsic session을 중앙·모서리·거리·tilt ±30–45°로 다시 촬영한다.
- Exposure/gain/focus를 고정하고 target ROI sharpness 및 saturation gate를 활성화한다.
- 기존 `capture_gate`가 블록별 조건을 사용하도록 수정한 뒤 실제 PASS frame만 저장한다.
- Cube placement는 각 면과 각 카메라의 관측 수를 균형화한다.
- Gripped cube는 위치와 roll/pitch/yaw가 다양한 30–50 자세로 촬영한다.
- 외부 GT 지그/장비로 calibration에 쓰지 않은 blind poses를 측정한다.
- 카메라 재설치 session을 반복하고 각 session의 환경·설정·config hash를 기록한다.

촬영 세부 체크리스트는 [RECAPTURE_PROTOCOL.md](RECAPTURE_PROTOCOL.md)를 따른다.

## 남은 구현 순서

1. 공통 raw-corner 평가 backend와 failure logging 고정
2. A4 covariance-weighted robust FK factor 구현
3. B1/B2 공정 arm 구현
4. C1–C4 classical baseline 연결
5. 필요 시 A5 6-DoF SE(3) correction 구현
6. Simulation 누출·metric 문제 수정 후 30-seed 재실행
7. 기존 데이터 재처리로 파이프라인 검증
8. 외부 GT 포함 재촬영 및 session-paired 최종 평가
9. Bootstrap CI·Holm 보정·P50/P95/failure table 생성

최종 논문 문장은 8–9단계의 합격 조건을 통과한 이후에만 확정한다.
