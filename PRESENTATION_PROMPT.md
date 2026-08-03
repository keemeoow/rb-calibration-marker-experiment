# 발표자료 생성 프롬프트 (rb-calibration-marker-experiment)

## 역할

너는 로보틱스 캘리브레이션 연구실 세미나 발표자료를 만드는 조력자다.
아래 리포지토리(`rb-calibration-marker-experiment`)의 코드와 산출물을 근거로,
**연구실 내부 세미나용 발표 슬라이드 초안 + 발표 스크립트**를 만든다.
청중은 로보틱스/컴퓨터비전 배경은 있지만 이 프로젝트 내부 사정은 모르는 대학원생·교수다.

## 시스템 개요 (배경)

- 로봇: Rainbow Robotics 6-DOF 팔. 손목에 eye-in-hand RealSense 1대, 작업공간 주변에 eye-to-hand 고정 RealSense N대.
- 타깃: (a) ChArUco board (11×7, checker 25 mm, marker 18 mm), (b) 로봇이 잡을 수 있는 AprilTag cube (한 변 59 mm, top tag 25 mm, side tag 51 mm).
- 좌표계: `base → gripper`(FK), `gripper → cam`(hand-eye `gTc`), `base → 고정카메라 i`(`bTCi`), `base → 타깃`(`bTO`, `bTB`).
- 목표: **고정 카메라 + 손목 카메라 + 로봇 FK를 하나의 목적함수로 동시 캘리브레이션**하고, 그 설계 선택이 오차 전파에 어떤 영향을 주는지 실험으로 분리 검증.

## 반드시 읽어야 할 파일

| 목적 | 경로 |
| --- | --- |
| 핵심표 / 모든 결과의 source of truth | `CP_result/Calibration_Experiment_table.md` |
| 7행 조건 스키마·통제 계약(코드로 강제됨) | `CP_ablation_schema.py` |
| 코너 재투영 목적함수 backend | `calibration_reprojection_backend.py` |
| 7행 러너 | `CP_ablation_7row.py` |
| FK 후보정(Ridge/SE(3)/offset) 정의 | `CP_C1_unified_vs_independent.py` (`learn_fk_ridge`, `learn_fk_rigid`, `learn_fk_offset`, `_resid_feature`, `downstream_rmse`) |
| 후보정 교차검증(LOSO 13-fold) 결과 | `CP_result/validate_loso/loso_summary.json`, `CP_validate_loso.py` |
| 실험 배경/시뮬 대응 | `CP_EXPERIMENTS_README.md`, `simulatioin_test.md`, `Simul_test/` |
| 노이즈 스윕(synthetic) | `CP_synthetic_7row.py`, `CP_result/synthetic_7row/` |
| 저N/저해상도 민감도 | `CP_sensitivity_7row.py`, `CP_result/sensitivity_7row/` |
| 반복 split 집계 | `CP_result/ablation_multisplit/multisplit_ablation.md` |
| **2×2 (FK 고정 × 잔차 보정), 위치 단위 13-fold** | `CP_D1_fk_correction_2x2.py`, `CP_result/D1_fk_correction_2x2/` |

## 절대 규칙 (어기면 발표가 틀림)

1. **모든 숫자는 위 파일에서 직접 확인한 값만 쓴다.** 기억이나 추정으로 채우지 않는다. 확인 못 한 칸은 `미측정`으로 남긴다.
2. **외부 GT가 없다.** 실세션의 pose 오차는 전부 로봇 FK를 정답 프록시로 쓴 값이므로 반드시 `FK-proxy`로 표기하고 절대정확도로 말하지 않는다.
3. **px와 mm는 다른 물리량이다.** 우리 held-out reprojection RMSE(px)와 외부 논문의 mm/°를 한 줄에 놓고 순위 매기지 않는다.
4. **target set이 다른 행은 overall 값으로 비교하지 않는다.** (`CP_ablation_schema.EVALUATION_COMPARISON_CONTRACT`가 정의한 공통 component로만 비교)
5. **Train loss는 수렴 진단값**이지 성능 순위 근거가 아니다.
6. **결과가 가설을 지지하지 않으면 그대로 발표한다.** 실데이터에서 A2→A3(FK-fixed)와 B2→A3(board 추가)는 개선되지 않았다. 이걸 숨기거나 유리하게 돌려 말하지 말고 "synthetic에서만 지지되는 주장"으로 명확히 구분한다.
7. 로봇 kinematic scale 보정(`RB_ROBOT_POS_SCALE`, k≈1.0229)은 **잠정값**이다. 언급할 때 잠정임을 명시한다.
8. **"FK-fixed + 잔차 보정"을 Ours로 발표하지 않는다.** 시뮬(Exp3 ②)과 실데이터(D1) 모두 그 구성을 최선으로 지지하지 않는다. 확정된 Ours는 **`A2@λ=3 + 잔차보정`**(cube pose를 미지수로 두되 FK soft anchor λ=3, 그 위에 FK-supervised 잔차보정)이며 정의와 근거는 아래 "Part 1.5"에 있다.
9. **Ours의 이벤트 split 값은 D2에서 측정됐다** (`CP_result/D2_anchored_event_split/`). 핵심표에서 `‖`로 표시된 칸이며, 검출 RANSAC 변동 때문에 **같은 표의 A2/A3 셀과 직접 빼지 말고** D2 자체 기준값과 비교하라.

---

# Part 1 — 오차 전파와 목적함수

다음 4개 논점을 이 순서로, 수식과 함께 설명하는 슬라이드를 만들어라.

## 1-1. 오차 전파 사슬을 먼저 세운다

고정 카메라가 관측한 타깃을 base 좌표로 옮기는 두 경로를 수식으로 대비시켜라.

- eye-to-hand 경로: `bTO = bTCi · CiTO`
- eye-in-hand 경로: `bTO = FK(q_e) · gTc · cTO`

두 경로가 base에서 만나기 때문에, **FK 오차 / hand-eye 오차 / PnP pose 오차가 곱셈으로 누적**된다는 것을 보여라.
독립(sequential) 풀이에서는 1단계 오차가 2단계에 **고정된 편향으로 상속**되고 되돌릴 수 없다는 점이 이후 Unified 설계의 동기다.
근거 지표: `e_e2e`(두 경로 간 큐브 위치 불일치), `e_cross`(고정 카메라들끼리의 불일치).

## 1-2. 왜 FK 정보를 쓰고, 왜 "후보정"이 필요한가

**용어부터 정리하라.** 발표에서 "후보정"이라는 말 대신 아래 중 하나를 골라 쓰고 슬라이드에 정의를 명시:

- 추천: **"예측 단계 잔차 보정(prediction-time residual compensation)"**
- 대안: "계통오차 사후 보상(systematic-error post-compensation)", "잔차 모델 기반 사후 보정"
- **금지**: 캘리브레이션을 다시 푸는 것으로 오해되는 표현. 이 보정은 **캘리브레이션 결과(`bTCi`, `gTc`)를 전혀 바꾸지 않고**, 예측 시점에만 적용되는 별도 단계다. 이 구분을 슬라이드에 반드시 박아라.

논리 전개:

1. 실데이터에는 외부 GT가 없다. → 정답 자리에 로봇 FK 큐브중점을 프록시로 넣는다.
2. FK는 base gauge를 고정해 주고(스케일·기준 프레임 확정) target pose의 자유도를 줄여 준다. → 그래서 solve에 쓴다.
3. 그런데 FK 자체가 완벽하지 않다. 이 프로젝트에서 실제로 확인된 것:
   - `meta.json`의 `set_cube_center_6dof`는 **위치는 맞지만 회전이 179.8° 뒤집혀** 있었다. 보정 전 C1 지표가 수백 mm로 부풀었다(예: independent held-out down 405.2 mm → 13.4 mm). → `correct_fk_cube_rotation`으로 대체.
   - 로봇이 Cartesian 거리를 **약 2.3% 적게 보고**한다(순수 병진 pair에서 `|dVis|/|dFlange| ≈ 1.023`, 회전 무관). → `RB_ROBOT_POS_SCALE`(k≈1.0229, 잠정)로 병진만 보정.
   - FK prior 위치 자체 오차 median ≈ 6.6 mm가 남는다 → "FK 대비" 지표의 절대값 하한.
4. 따라서 **FK를 solve에 넣는 것만으로는 계통오차가 남고**, 남은 잔차가 무작위가 아니라 구조를 가지므로 예측 단계에서 별도로 보상할 여지가 있다. 이것이 잔차 보정 단계의 존재 이유다.

## 1-3. 왜 (하필) 선형 회귀인가 — 근거로 설득할 것

이건 "간단해서"가 아니라 **잔차의 구조를 실측으로 확인한 결과**라는 흐름으로 만들어라.

**(a) 잔차 모델 후보를 자유도 계단으로 놓고 비교했다** (모두 train에서만 학습, held-out에만 적용):

| 모델 | 자유도 | 형태 |
| --- | ---: | --- |
| offset only | 3 | `p̂ + b` |
| SE(3) rigid (Kabsch) | 6 | `R p̂ + t` |
| Ridge affine `[1, x, y]` | 9 | `p̂ + φ(p̂)ᵀ W`, `φ = [1, x, y]` |

**(b) LOSO 13-fold 결과** (`CP_result/validate_loso/loso_summary.json`, 단위 mm, mean±std) — 값은 파일에서 재확인할 것:

| method | raw | +offset(3) | +SE(3)(6) | +Ridge(9) |
| --- | ---: | ---: | ---: | ---: |
| independent | 19.55±6.94 | 19.43±6.97 | 2.57±1.82 | 2.63±1.94 |
| unified_joint | 16.59±4.98 | 16.77±5.44 | 2.37±1.42 | 2.78±1.11 |

**(c) 여기서 읽어야 할 결론 — 이게 슬라이드의 핵심 메시지다:**

- 상수 offset(3-DOF)은 **전혀 효과가 없다** → 남은 오차는 위치 무관 bias가 **아니다**.
- 6-DOF 강체 정렬만으로 19.5 → 2.6 mm로 떨어진다 → 남은 오차의 대부분은 **base 프레임 정렬 잔차(회전+평행이동)**, 즉 좌표계 수준의 **계통 기하 오차**이지 관측 노이즈가 아니다.
- 축별 RMS(`axis_rms`)를 보면 raw에서 rx 14.21 / ry 14.97 / **rz 0.61** mm → 오차가 거의 전부 **x–y 평면 안**에 있다. 즉 평면 내 회전/스케일로 설명되는 형태다.
- 학습된 `ridge_W_mean`을 뜯어 보면 `∂d_y/∂x ≈ −0.093`, `∂d_x/∂y ≈ +0.091`로 **거의 반대칭**이고 대각 성분은 0에 가깝다 → **z축 둘레 약 5°의 평면 내 회전**이 지배적이고 스케일 항은 거의 남아 있지 않다(로봇 스케일 보정 이후이므로 일관적).
- **따라서 선형(affine) 모델이 정당하다**: 잔차의 생성 원인이 좌표계 정렬 오차와 등방 스케일 오차이고, 이 둘은 위치에 대해 **정확히 1차**로 작용한다. 비선형 모델을 쓸 물리적 근거가 없고, 13 set 규모에서 과적합만 늘린다.
- **모델 선택의 진단적 가치**: Ridge `[1,x,y]`는 대각 성분으로 등방 스케일을 흡수할 수 있지만 SE(3)는 못 한다. 실제로 로봇 스케일 보정을 켰을 때 `+SE(3)`는 약 5.0 → 2.8 mm로 개선되고 `+Ridge`는 평평했다 — 이 비대칭이 스케일 오차를 특정한 결정적 증거였다. 즉 두 모델을 **나란히 두는 것 자체가 진단 도구**다.
- 정규화: `λ‖W‖²` (기본 `λ=1e-3`), **절편은 정규화하지 않는다**(`reg[0,0]=0`). 13개 set이라 표본이 적어 안정화가 필요하다.
- 일반화 검증: `ridge_W_std`가 fold 간 작다는 점, 그리고 보정 계수를 **train fold에서만 학습해 held-out set에 적용**했다는 프로토콜을 명시하라.

**(d) 반드시 같이 말할 한계:**
- 정답이 FK 프록시이므로 "FK와의 불일치"를 줄인 것이지 물리적 정확도를 증명한 게 아니다.
- 단일 split(train 9 / test 4) 결과는 Ridge 2.81 / SE(3) 5.06 mm로 순위가 LOSO와 다르다. **1차 근거는 13-fold LOSO**이고 단일 split은 보조라고 명시하라.
- `x, y`만 특징으로 쓰고 `z`를 뺀 이유(큐브 위치가 대체로 한 평면 위에서 움직여 z 방향 여기가 부족)와 그 위험을 언급하라.

## 1-3-2. (중요) 보정이 무엇을 바꾸고 무엇을 못 바꾸는지 반드시 명시하라

잔차 보정은 예측 시점에만 적용되고 `bTCi`, `gTc`를 **바꾸지 않는다.** 여기서 따라 나오는 세 가지 구조적 사실을 슬라이드에 못 박아라.

1. **재투영 오차(px)는 보정 전후가 완전히 동일하다.** 재투영은 pose에만 의존하기 때문이다. → 보정을 Table 1의 `e_reproj` 열에 반영할 수 없고, "보정으로 px가 좋아졌다"는 말은 성립 자체가 안 된다.
2. **`e_cross`도 사실상 안 변한다.** 모든 카메라 예측에 항등에 가까운 동일 affine을 곱하는 것이라 카메라 간 차이는 거의 그대로다.
3. **따라서 보정이 개선하는 지표는 "FK 대비" 지표 하나뿐이고, 그건 보정이 학습한 대상이다.**

이 마지막 항목이 순환성 문제의 핵심이다. 얼버무리지 말고 다음 형태로 정면 돌파하라:

> 우리가 보인 것은 **"FK와 vision의 불일치가 위치에 대해 1차 구조를 갖고 held-out 위치로 일반화된다"**(19.5 → 2.6 mm, 13-fold)는 사실이다.
> 그 불일치의 **어느 쪽이 틀렸는지는 외부 GT 없이 판정할 수 없다.** FK가 맞으면 보정은 절대정확도를 올리고, vision이 맞으면 보정은 절대정확도를 떨어뜨린다.
> 그래서 현재 주장은 "일반화되는 계통장의 발견"까지이고, "정확도 향상"이 아니다.

## 1.5 — 최종 제안(Ours)의 정의 ★★ (발표에서 가장 중요한 절)

> **확정 사항 (2026-07-31 채택).** 최종 Ours는 **`A2@λ=3 + 잔차보정`**이다. 핵심표 Table 1의
> `↳ A2@λ=3 + 잔차보정 ★` 행이 그것이며, held-out 위치 오차 **1.586 mm**(`FK-proxy`, 위치 LOO 13 fold).
> A3(FK-fixed)를 Ours로 제시하지 마라. λ는 임의값이 아니라 **교차검증으로 정한 값**이라고 반드시 밝혀라.

**Ours = Unified + cube pose를 미지수로 두되 FK soft anchor(λ=3 px/mm) + FK-supervised 잔차 보정.**

즉 **FK로 cube pose를 하드 고정하지 않는다.** FK의 역할은 세 가지다:

1. **gauge 확정과 초기화** — hand-eye 백본 `bTg = FK(q)`, base 프레임 기준 설정
2. **solve의 soft anchor** — cube pose를 자유변수로 두되 FK 쪽으로 λ만큼 당김
3. **잔차 보정의 지도 신호(supervision)** — train 위치에서 (카메라 예측 vs FK)의 잔차를 회귀

**왜 cube pose를 FK로 고정하면 안 되는가 — 메커니즘부터 말하라.**
큐브 pose를 상수로 만들면 관측 노이즈를 흡수할 자유도가 사라지고, **노이즈가 갈 곳이 `gTc`밖에 없어진다.** 시뮬에서 이게 직접 측정된다: 관측 노이즈 2→15 mm에서 gTc 복원 오차가 camera-based는 0.116→0.249 mm로 완만한데, FK-fixed는 0.130→**0.978 mm**로 4배 악화한다.

**시뮬과 실데이터가 같은 최적 구성을 가리킨다. 이게 이 발표의 헤드라인이다.**

시뮬 Exp3 (`Simul_test/exp3_gtc_estimation.py`, `figures/exp3_noise_sweep_data.json`, 8 sets, held-out 큐브 예측 mm):

| 방식 | σ=2 mm | σ=6 mm | σ=15 mm |
| --- | ---: | ---: | ---: |
| ① Camera-based (큐브=미지수) | 0.390 | 1.073 | 2.624 |
| ② FK-based (큐브=FK 고정) | 0.405 | 1.217 | **3.039 (꼴찌)** |
| ③ **Camera + FK-correction** | **0.069** | **0.183** | **0.470 (최고)** |

③은 ①번 모델 위에 보정을 얹은 것이며 **큐브는 끝까지 자유변수**다 (`W = learn_fk_residual(scene, m_cam, ...)`). 미지수를 실제로 제거한 ②가 전 구간 꼴찌다. C1 시뮬(`unified_vs_independent.py`)에는 FK-fixed arm이 **아예 없고**, FK는 gauge 초기화 전용이며 `+fk` 보정은 양쪽에 대칭으로 적용된다 — 최고는 `Joint+fk` 0.354 mm다.

실데이터 D1 (아래 a-0)의 최고 셀은 **A2(vision-estimated) + Ridge = 1.847 mm**로 시뮬 ③과 같은 구성이다. C3 LOSO도 같은 방향이다 (`no_fk_prior` 12.199 < `fk_prior_correction` 13.054 mm).

### 1.5-1. "그럼 FK로 미지수를 제거하지 않는 것인가?" — 이 슬라이드를 반드시 만들어라

청중이 가장 먼저 던질 질문이다. 답은 **"제거하지 않는다. 그러나 FK 정보를 버리는 것도 아니다"**이고, 자유도 숫자로 설명해야 설득된다.

**(1) 먼저 규모를 바로잡아라 — 제거되는 건 "미지수 1개"가 아니다.**
큐브 pose는 위치마다 SE(3), 즉 **위치당 6 DOF**다. D1 fold 0 (고정카메라 3대, 학습 위치 12개)의 실측 파라미터 수:

| arm | 자유 파라미터 | 구성 |
| --- | ---: | --- |
| A2 vision-estimated | **102** | 카메라 6×3 + gTc 6 + board 6 + **큐브 6×12 = 72** |
| A3 FK-fixed | **30** | 카메라 6×3 + gTc 6 + board 6 |

FK 고정이 제거하는 것은 **6 × S = 72 DOF**다 (`jacobian.n_params`, 두 arm 모두 rank 충족, nullity 0).

**(2) FK 정보는 버려지지 않는다 — 주입하는 위치와 형태가 바뀔 뿐이다.**

| | FK를 쓰는 방식 | 자유도 | FK 오차가 가는 곳 |
| --- | --- | ---: | --- |
| A3 | 위치마다 **hard constraint** | 6×S 제거 | 위치 s의 FK 오차를 그 위치에서 흡수 못 함 → **공유 파라미터 `bTCi`, `gTc`로 밀려남** |
| A2 + 보정 | 전역 **smooth 회귀의 지도 신호** | 예측 단계에 9 추가 | 부드러운 계통 성분만 흡수, 위치별 FK 잡음은 **회귀 잔차로 남아 공유 파라미터를 오염시키지 않음** |

같은 FK 정보를 `per-position hard (6S DOF)`로 쓰느냐 `global smooth (9 DOF)`로 쓰느냐의 차이다. 후자는 위치들에 걸쳐 **잡음이 평균화**되고, 전자는 위치마다의 FK 오차가 그대로 공유 변환에 실린다. 시뮬의 gTc 0.130→0.978 mm가 그 귀결이다.

덧붙일 것: 큐브 pose는 애초에 부실하게 결정되는 nuisance 파라미터가 **아니다.** 큐브는 고정 카메라 평균 2.70대가 동시 관측하고(board는 1.03대) 손목 카메라도 본다. 관측만으로 이미 잘 결정되므로 **제거해서 얻는 식별성 이득은 작고, FK 오차 주입 손실은 크다.**

**(3) 그래도 FK 고정이 사는 것이 있다 — 정확도가 아니라 수치 안정성이다.** D1 실측:

| | Jacobian condition | nfev (반복 횟수) |
| --- | ---: | ---: |
| A2 | 316.1 | 106.5 |
| A3 | **51.2** | **19.4** |

72 DOF를 빼면 조건수 약 6배, 수렴 속도 약 5배가 좋아진다. synthetic σ=1 px의 A2 3/10 vs A3 10/10 수렴도 같은 현상이다. **"FK 고정은 정확도를 사는 게 아니라 조건수와 속도를 사는 것"**이라고 정확히 말하라.

**(4) 스펙트럼으로 제시하고, 중간 지점을 권하라.**

```
hard fix (A3, λ=∞)  ↔  soft anchor (λ)  ↔  no anchor (A2, λ=0)
     조건수 최선                              노이즈 흡수 최선
     FK 오차 주입 최악                        조건수 최악
```

soft anchor는 큐브 pose를 자유변수로 두되 FK 쪽으로 λ만큼 당긴다: `r_anchor = λ · r_SE3(bTO^(s), bTO,FK^(s))`. 자유도는 살아 있어 노이즈를 흡수하고 gauge는 잡혀 조건수가 개선된다. **C1 LOSO 13-fold에서 `unified_joint(soft-anchor λ=5) + SE(3)` = 2.374 mm가 전체 셀 최소값이었다.**

**(5) λ sweep 결과 — 이 슬라이드가 Ours를 정당화한다.**
D1에 anchor 축을 넣어 `λ ∈ {0, 0.3, 1, 3, 10, 30, 100}`을 같은 13-fold에서 돌렸다
(`CP_result/D1_fk_correction_2x2/`). held-out 위치 오차 RMSE (mm):

| arm | λ | none | Ridge | Jacobian cond |
| --- | ---: | ---: | ---: | ---: |
| A3 | ∞ | 2.988 | 2.355 | **51.2** |
| A2 | 0 | 3.451 | 1.847 | 316.1 |
| A2@λ=1 | 1 | 3.229 | 1.771 | 309.6 |
| **A2@λ=3 ★** | 3 | 2.729 | **1.586** | 297.3 |
| A2@λ=10 | 10 | **1.907** | 1.704 | 267.7 |
| A2@λ=100 | 100 | 2.983 | 2.351 | 160.4 |

슬라이드에 반드시 넣을 네 가지:

- **구현 검증**: λ=100이 A3와 fold 단위 최대 0.011 mm 차이로 일치한다 → anchor가 λ→∞에서 하드 고정으로 정확히 수렴하며, **A2와 A3는 같은 축의 두 끝점**이다. λ=0은 canonical solver와 `max|ΔT|=0` 동치를 코드가 assert한다.
- **보정 없는 열이 U자**: 3.451 → 1.907(λ=10) → 2.983(λ=100). **양 끝점이 둘 다 최적이 아니다.**
- **λ=3의 우위는 사후 체리피킹이 아니다**: 중첩 교차검증(바깥 fold마다 나머지 12개로 λ 선택)에서 **13/13 fold가 전부 λ=3을 골랐고**, 절차 자체의 성능이 1.586 mm다. λ=0 대비 −0.238 mm, t=−2.96, 11/13. 28개 셀에서 우연히 집힌 값이라면 fold마다 다른 λ가 뽑혔어야 한다. 다중비교 방어는 선택이 불안정할 때 필요한 것이므로 여기엔 적용되지 않는다.
- **조건수는 λ와 함께 단조 개선**(316 → 51)되지만 정확도 최적은 중간이다 → **λ는 "정확도 대 수치 안정성"을 조절하는 손잡이**이고 두 목표의 최적점이 다르다.

**λ는 하이퍼파라미터이므로 "교차검증으로 정했다"고 명시하라.** 임의로 고른 값이 아니다.

**(6) 이벤트 split에서도 검증됐다 — Ours 슬라이드의 결정타.** (`CP_result/D2_anchored_event_split/`)
D1은 위치 hold-out·mm였으므로 Table 1의 나머지 열을 채우기 위해 canonical 이벤트 split(5 seeds × 3 init, 45/45 수렴)에서 다시 쟀다. A2(λ=0) 대비 split 단위 paired delta:

| 비교 | e_reproj (px) | e_e2e (mm) | e_cross (mm) |
| --- | ---: | ---: | ---: |
| **A2@λ=3 − A2** | +0.0055, t=+0.78, 2/5 | **−0.132, t=−4.55, 5/5** | **−1.772, t=−35.62, 5/5** |
| A3 − A2 | +0.159, **t=+7.42, 0/5** | −0.223, t=−1.89, 3/5 | −4.105, t=−23.91, 5/5 |

한 줄 메시지: **"soft anchor는 재투영을 희생하지 않으면서 경로 일치도를 일관되게 개선한다. 하드 고정은 재투영을 확실히 악화시킨다."** A3의 재투영 악화는 5/5 split에서 t=+7.42로 분명한 반면, λ=3의 재투영 변화는 t=+0.78로 유의하지 않다.

이게 중요한 이유: `e_e2e`·`e_cross`는 **잔차보정이 바꿀 수 없는 값**이다(규칙 1-3-2). 여기서 개선됐다는 것은 보정 덕이 아니라 **캘리브레이션 자체가 좋아졌다**는 뜻이다. 따라서 채택 구성은 **위치 예측(D1) · 경로 일치도(D2) · 재투영(손실 없음) 세 축에서 동시에** 방어된다.

**Table 1 재현 확인을 반드시 함께 말하라.** D2는 A2/A3를 같이 돌려 Table 1 값을 재현하는지 검사하며, 최대 편차 1.51σ(허용 2σ)로 통과했다. 이 확인 없이는 D2 숫자를 Table 1 옆에 놓을 수 없다.

**⚠ 코드 라벨과의 불일치를 반드시 짚어라.**
`CP_ablation_schema.py`는 A3(FK-fixed)에 `"Ours (full)"` 라벨을 달고 있다. 이는 **7행 ablation 안에서 "모든 구성요소를 켠 행"**이라는 뜻이며, 위에서 정의한 발표용 권장 구성과 다르다. 슬라이드에서 이 차이를 한 줄로 명시하고, 라벨을 근거로 A3를 권장 구성이라 말하지 마라.

**(a-0) 실데이터 2×2 — `CP_D1_fk_correction_2x2.py`.**

동일 backend(canonical corner reprojection) · 동일 solver 설정 · 동일 예측 mask · **위치(set) 단위 leave-one-out 13 fold**, 13/13 수렴. 지표는 held-out 위치의 큐브 중심 예측 오차(mm, `FK-proxy`). 산출물: `CP_result/D1_fk_correction_2x2/`.

held-out 위치 오차 RMSE (mm):

| arm | none | offset(3) | SE(3)(6) | Ridge(9) |
| --- | ---: | ---: | ---: | ---: |
| A2 vision-estimated | 3.451 | 2.685 | 2.479 | **1.847** |
| A3 FK-fixed | 2.988 | 2.341 | 2.401 | 2.355 |

paired 13-fold 검정 (delta = 두 번째 − 첫 번째, 음수가 개선):

| 항목 | mean (mm) | SE | t | fold |
| --- | ---: | ---: | ---: | ---: |
| A3−A2 @ none | −0.401 | 0.552 | −0.73 | 7/13 |
| A3−A2 @ ridge | +0.361 | 0.345 | +1.05 | 5/13 |
| A2: ridge − none | **−1.587** | 0.345 | **−4.59** | 11/13 |
| A3: ridge − none | **−0.824** | 0.335 | **−2.46** | 10/13 |
| 상호작용 ridge | +0.763 | 0.488 | +1.56 | 4/13 |

**읽는 법 — 이 세 줄을 슬라이드에 그대로 써라:**

1. **최고 셀은 `A2 + Ridge` = 1.847 mm이며, 이는 시뮬 ③ `Camera + FK-correction`과 같은 구성이다.** 시뮬과 실데이터가 같은 최적 구성을 가리킨다 — 이게 D1의 핵심 결과다.
2. **잔차 보정이 효과의 주체다.** 양쪽 arm 모두에서 통계적으로 분명한 개선(A2 −1.587 mm, t=−4.59; A3 −0.824 mm, t=−2.46)이고, **보정 이득은 A2 쪽이 약 두 배 크다** — 시뮬에서 FK-fixed가 노이즈를 gTc로 몰아 보정 여지를 잃는 것과 같은 방향이다.
3. **FK 고정의 효과는 이 표본으로는 판정되지 않는다.** 모든 A3−A2 대비가 `|t| ≤ 1.05`다. "차이가 없다"가 아니라 **"13개 위치로는 가릴 수 없다"**로 표현하라. 상호작용은 가설과 반대 부호(+0.763)이지만 역시 유의하지 않다.

따라서 실데이터만 놓고 보면 "FK 고정이 해롭다"고 단정할 수는 없다. 그러나 **최고 셀·보정 이득의 크기·시뮬의 메커니즘이 모두 같은 방향**이므로, 권장 구성은 `vision-estimated + 보정`이다.

**(a-1) A3가 유일하게 명확히 이기는 지표와, 그것을 인용하면 안 되는 이유.**
held-out 위치에서 train 전용 FK pose로 재투영한 오차는 A3가 13/13 fold에서 낮다(−1.343 px, t=−6.66). 그러나 이것은 **A3의 카메라가 애초에 FK 큐브 pose에 맞춰 적합된 뒤 FK pose에서 픽셀을 재는 것**이라 순환이다. 반면 FK를 전혀 참조하지 않는 `e_cross`는 −1.602 mm이지만 t=−0.91, 7/13으로 **유의하지 않다**. 즉 "FK를 안 쓰는 지표에서도 A3가 낫다"고 말할 근거는 없다. 두 통제 지표를 반드시 함께 제시하라.

**(a-2) 이 결과와 canonical 7행 결과는 모순이 아니다.**
7행의 A2→A3 악화(+0.116 px, 5/5)는 **held-out 이벤트의 재투영**이고, D1은 **held-out 위치의 3D 예측**이다. 서로 다른 질문에 대한 서로 다른 답이므로 한 줄에 놓고 비교하지 마라.

**(a) 과거 자료를 이어 붙이는 것은 여전히 금지다.**
A3는 canonical corner-reprojection backend / cube+board / event 단위 split.
Ridge 보정은 C1의 historical SE(3) pose-residual 솔버 / cube-only / set 단위 split / mm 지표.
backend·target set·split·지표가 전부 다르다. `Calibration_Experiment_table.md`가 이미 "이 값들은 Table 1의 raw A2/A3 셀에 넣지 않는다"고 규정한다. → **두 결과를 이어 붙여 하나의 파이프라인 성능처럼 제시하지 마라.**

**(b) 가장 가까운 근거는 오히려 불리하다.** LOSO 13-fold (`CP_result/validate_loso/loso_summary.json`, held-out 큐브위치 mm, mean):

| method | raw | +offset(3) | +SE(3)(6) | +Ridge(9) |
| --- | ---: | ---: | ---: | ---: |
| independent | 19.545 | 19.431 | 2.574 | 2.628 |
| unified_joint (soft-anchor λ=5) | 16.592 | 16.766 | **2.374** | 2.780 |
| **joint_fk_fixed** | 16.718 | 16.658 | 2.676 | **3.057 (꼴찌)** |

C3도 같은 방향이다: `no_fk_prior` 12.199 mm vs `fk_prior_correction` 13.054 mm (13-fold). `fk_prior` 단독은 13-fold 전부 실패해 `n=0`이다.

주의: 단일 split(train 9 / test 4)에서는 joint_fk_fixed가 raw 11.53 mm로 가장 좋았다. **13-fold로 넓히면 순위가 뒤집힌다.** 발표에서는 반드시 LOSO를 1차 근거로 쓰고, 단일 split 숫자만으로 조합의 우위를 말하지 마라.

**(c) 그러므로 슬라이드에서는 다음 네 가지를 분리해 제시하라.**

- ✅ **가장 강한 주장 (시뮬+실데이터 일치)**: 최적 구성은 **cube pose를 미지수로 두고 푼 뒤 FK로 지도학습한 잔차 보정을 얹는 것**이다. 시뮬 Exp3 ③(0.069/0.183/0.470 mm)이 ①②를 전 구간 압도하고, 실데이터 D1의 최고 셀(A2+Ridge 1.847 mm)이 같은 구성이다.
- ✅ **메커니즘까지 설명된다**: cube pose를 FK로 고정하면 관측 노이즈가 흡수될 자유도가 사라져 `gTc`로 몰린다. 시뮬에서 gTc 오차 0.130→0.978 mm(4배), camera-based는 0.116→0.249 mm.
- ✅ **잔차 보정의 일반화**: train 위치에서 학습한 계수가 held-out 위치로 전이된다 (D1 A2 −1.587 mm, t=−4.59, 11/13).
- ⚠️ **실데이터만으로는 판정 불가로 남는 것**: FK 고정 여부의 직접 효과. D1의 모든 A3−A2 대비가 |t|≤1.05다. 시뮬은 FK-fixed에 불리하지만 실데이터가 그 크기를 재현할 검정력이 없다고 정직하게 말하라.
- ❌ **절대 하지 말 것**: (i) A3의 px 값과 C1의 보정 후 mm 값을 이어 붙여 하나의 파이프라인 성능처럼 제시하는 것. (ii) D1에서 A3가 이긴 유일한 지표(FK pose 재투영)를 순환성 설명 없이 인용하는 것. (iii) `CP_ablation_schema.py`의 `"Ours (full)"` 라벨을 근거로 A3를 권장 구성이라 말하는 것 — 그 라벨은 7행 ablation 내부의 "전부 켠 행"이라는 뜻이다.

**(d) 남은 과제를 "다음 단계" 슬라이드로 구체적으로 적어라.** (2×2는 D1으로 완료됐다.)

1. **검정력**: 13개 위치로는 |delta| ~0.4 mm를 가릴 수 없다. A3−A2를 판정하려면 위치 수를 늘리거나 위치당 반복 촬영으로 fold 분산을 줄여야 한다. 필요 표본을 D1의 관측 std로 역산해 슬라이드에 숫자로 적어라.
2. **순환성 차단**: 현재 1차 지표가 FK를 정답으로 쓴다. FK를 참조하지 않는 `e_cross`는 유의하지 않았으므로(t=−0.91), 판정에는 **외부 GT가 필요하다**(Table 3, 현재 보류).
3. **자유도 회계**: FK-fixed는 solve에서 `6 × |sets|` DOF를 제거하고, 보정은 예측 단계에서 9 DOF를 되돌려 준다. 순 자유도를 표로 밝히고, D1이 보여준 대로 **"vision-estimated + 보정"이 최소한 대등한 경쟁자**임을 명시하라.
4. **보정 모델의 z축**: 특징이 `[1,x,y]`뿐이다. 큐브 위치가 z로 충분히 퍼진 촬영을 추가해 `z` 항이 필요한지 검정한다.

## 1-4. 목적함수의 수학적 구성

`calibration_reprojection_backend.py`를 근거로 **한 장에 목적함수 전체**를 적어라.

**변수(모두 SE(3)):** `{bTCi}`, `gTc`, `bTB`(board), `{bTO^(s)}`(set별 cube).
**상수(전 조건 공통, 절대 자유변수가 아님):** `K`, distortion `D`, target geometry, backbone `bTg[e] = FK(q_e)`, train/test split, 관측 품질 mask, optimizer 설정, 초기화 seed (`GLOBAL_FIXED_INPUTS`).

**코너 단위 잔차:**

```
관측 (e: event, c: camera, m: marker, j: corner)
bT_cam(c, e) = bTg[e] · gTc        (c == gripper cam)
             = bTCi                 (c == fixed cam i)

r_{e,c,j} = π( K_c, D_c, (bT_cam)⁻¹ · bT_target · X_j ) − u_{e,c,j}      ∈ R²
```

**목적함수:**

```
min_Θ  Σ_{e,c,j}  ρ( ‖ r_{e,c,j} ‖² ),    ρ = soft_l1,  f_scale = 2 px
```

여기에 반드시 같이 설명할 것:

- **국소 SE(3) retraction**: 파라미터는 절대 pose가 아니라 기준 pose 주변의 6차원 증분 `δ`, `T = T_ref · Exp(δ)`. 회전/병진 스케일을 나눠 `x_scale='jac'`와 함께 조건수를 관리.
- **freeze mask**: 행마다 어떤 변수가 자유이고 어떤 게 고정인지를 **명시적 마스크**로 강제 (`UNIFIED_FREE_VARIABLES`, `SEQUENTIAL_STAGE_SPECS`). 순차(seq) 행은 1단계 출력을 완전히 얼린 뒤 2단계에서 `bTCi`만 풀고, alternating pass를 금지한다 — 이게 "정보가 되돌아오지 않는다"를 코드로 못 박은 부분이다.
- **Jacobian 희소성**: 각 관측은 (그 카메라, 그 타깃) 블록에만 기여 → 희소 구조를 명시적으로 넘겨 준다.
- **gauge 고정**: cube pose를 자유변수로 두면 base gauge가 뜬다. 두 처리 방식을 대비하라 —
  - hard: cube pose를 FK로 고정 (A3/B1/B2)
  - soft anchor(보충 실험): `r_anchor = λ · r_SE3(bTO^(s), bTO,FK^(s))`, `λ=5`
  그리고 A2(strict none)에서 Jacobian condition이 279.8–352.2로 A3의 56.1–131.8보다 나빴다는 진단을 붙여라.
- **solver 계약**: `soft_l1(f_scale=2px)`, `x_scale='jac'`, `max_nfev=300`, tol `1e-8`. 수렴은 대부분 SciPy status 2(cost plateau)이므로 **"gradient가 0인 해"가 아니라 안정적 cost-plateau 수렴**으로 정확히 표현하라.
- **잔차 보정과의 관계도**: 목적함수 최적화 → (캘리브 결과 고정) → 예측 → 잔차 보정. 두 단계가 분리돼 있다는 파이프라인 그림 한 장.

---

# Part 2 — 캘리브레이션 실험 재설계

## 2-1. 왜 재설계했나

원래는 C1(Unified vs Independent) / C2(Board vs Cube) / C3(gTc estimation)라는 **서로 다른 historical 실험 3개**였고, 조건이 겹치고 통제가 달라 인과 해석이 불가능했다.
그래서 세 기여를 **직교하는 3개 축**으로 재정의하고 한 러너·한 backend·한 평가 프로토콜로 다시 짰다.

| 기여 | 축 | 값 | 의미 |
| --- | --- | --- | --- |
| C2 | **Marker** | board / cube / cube+board | 어떤 타깃을 쓰는가 |
| C1 | **Unified** | seq / U | eye-to-hand와 eye-in-hand를 순차로 푸는가, 한 목적함수로 푸는가 |
| C3 | **FK→cube** | vision-estimated / FK-fixed | 타깃 pose를 FK로 확정하는가, 미지수로 추정하는가 |

**핵심 주의(슬라이드에 반드시):** FK backbone `bTg = FK(q)`는 **모든 조건이 공통으로** 쓴다. FK 열은 "FK를 쓰냐 마냐"가 아니라 **"cube pose를 FK 값으로 고정했냐"**만 뜻한다. 또 고정된 작업공간 board는 FK pose source가 물리적으로 존재하지 않으므로 러너가 그런 입력을 즉시 반려한다.

## 2-2. 7행 구성 (A0/A1/A2/A3/B1/B2/B3)

`CP_ablation_schema.MAIN_ABLATION_CONDITIONS`가 정의:

| # | Marker | Unified | FK→cube | 의미 |
| --- | --- | :---: | --- | --- |
| A0 | board | seq | — | baseline |
| A1 | cube+board | seq | vision-estimated | +cube |
| A2 | cube+board | U | vision-estimated | +unified |
| **A3★** | **cube+board** | **U** | **FK-fixed** | **Ours (full)** |
| B1 | cube+board | seq | FK-fixed | −Unified |
| B2 | cube only | U | FK-fixed | −board |
| B3 | board only | U | — | −cube |

A는 baseline에서 하나씩 쌓아 올리는 축, B는 full에서 하나씩 빼는 축이라는 구조를 그림으로 보여라.

**통제 계약**(슬라이드 한 장 값어치가 있다):
- capture event 단위 train/test 분할 + cube 위치별 층화. 같은 event를 양쪽에 나누지 않는다.
- 1차 지표 = 모든 pose를 고정한 **held-out corner reprojection RMSE**. test 관측으로 초기화·재정렬·refit·outlier 선택을 하지 않는다.
- 모든 행이 fitting **전에** 만든 동일한 measurement-only mask를 쓴다.
- B1/A3/B2가 쓰는 FK-fixed cube pose는 train eih cube corner와 raw FK만으로 만든 **동일한 board-free artifact를 byte-identical하게 공유**(SHA-256 검증). 행별 재정렬 금지.
- 비교 쌍마다 **어떤 공통 component로만 비교할지**를 사전에 선언(`EVALUATION_COMPARISON_CONTRACT`).
- noise-free synthetic에서 A1=A2, B1=A3가 같은 해와 zero reprojection에 도달하지 못하면 실데이터 실행을 중단하는 sanity gate.

## 2-3. 결과와 해석

**(a) 실데이터 — canonical 5 split × 5 init, 175/175 수렴** (delta는 `두 번째 행 − 첫 번째 행`, 음수가 개선):

| 비교 | 공통 component | delta | 개선 split |
| --- | --- | ---: | ---: |
| A0→A1 | board reproj | +0.00315±0.00297 px | 1/5 |
| A0→A1 | N_reg | +1.000±0.000 | 5/5 |
| A1→A2 | overall reproj | **−0.14858±0.02161 px** | **5/5** |
| B1→A3 | overall reproj | −0.00869±0.00220 px | 5/5 |
| A2→A3 | overall reproj | **+0.11555±0.02529 px** | **0/5** |
| B2→A3 | cube reproj | +0.02960±0.00273 px | 0/5 |
| B3→A3 | board reproj (참고) | +0.00955±0.02238 px | 1/5 |

**(b) 실데이터에서 방어되는 주장 / 안 되는 주장을 슬라이드에서 명확히 갈라라:**

- ✅ **cube 추가는 카메라 등록 가능 수를 늘린다** — N_reg 2 → 3, 5/5 split. 단 이건 **pixel accuracy 이득이 아니라 관측성(registration) 이득**이다. board는 평면이라 2대 이상 고정 카메라 동시관측 비율이 2.86%뿐이지만, cube는 89.66%다.
- ✅ **vision-estimated cube 조건에서 Unified는 일관되게 개선한다** — A1→A2, 5/5 split, −0.149 px.
- ⚠️ **FK-fixed 조건에서 Unified 효과는 사실상 없다** — B1→A3 −0.009 px. 방향은 맞지만 크기가 무의미하고 지표별로 방향이 엇갈린다.
- ❌ **"FK 고정이 오차 전파를 줄인다"는 실데이터로 주장할 수 없다** — A2→A3는 5/5 split에서 악화.
- ❌ **"board 추가가 정확도를 높인다"도 실데이터로 주장할 수 없다** — B2→A3는 공통 cube/path 지표에서 0/5 개선.

**(c) 그럼 A3를 7행 ablation의 full row로 두는 근거는 무엇인가 — synthetic에서만 지지되는 부분을 분리해 발표하라** (`CP_result/synthetic_7row/`, pixel corner noise σ 스윕, 10 seed).

**주의: 여기서 A3의 근거는 "수렴 안정성"이며, Part 1.5의 권장 구성(vision-estimated + 보정)과 충돌하지 않는다.** A3는 "cube pose를 고정하면 최적화가 안정된다"를 보이는 행이고, Part 1.5는 "정확도를 위해서는 고정하지 말고 보정하라"는 결론이다. 두 문장을 같은 슬라이드에 나란히 놓고, 안정성과 정확도가 다른 축임을 명시하라.

- **수렴 안정성**: A2(strict vision-estimated)는 σ=0.5/1/2에서 각각 **6/10, 3/10, 0/10만 수렴**. A3는 모든 노이즈에서 **10/10 수렴**. → FK-fixed의 실질 이득은 정확도 숫자가 아니라 **최적화 안정성**이다. 이게 A3의 가장 정직한 세일즈 포인트다.
- A2가 완전 수렴하지 않으므로 **A2/A3의 numeric gap은 headline로 계산하지 않았다**는 점을 명시.
- B1→A3(Unified 효과) translation delta는 σ=0.5/1/2에서 −0.900/−1.903/−4.318 mm로 **노이즈와 함께 커진다**.
- B2→A3(board 효과)도 synthetic에서는 −1.478/−2.953/−5.876 mm로 이득이 커지지만, **실데이터 5 split에서는 0/5 개선** → 재현되지 않았다고 분명히 말하라.
- **FK pose noise 주입**: FK cube pose에 3 mm/0.3° 노이즈를 주면 board 추가 이득이 **부호가 뒤집힌다**. → FK-fixed는 FK가 충분히 정확할 때만 유효한 설계이고, 그 범위를 벗어나면 board가 잘못된 FK를 보정하지 못하고 결합 bias를 만든다. **A3의 적용 조건**으로 발표하라.

**(d) 민감도** (`CP_result/sensitivity_7row/`): B2 대 A3를 공통 cube corner로 paired 비교하면 N=5에서만 −0.156±0.165 px(4/5)로 작은 이득, N≥10에서는 재현되지 않음(0~1/5). → "board가 적은 뷰에서 항상 도움된다"는 강한 주장은 현재 데이터가 지지하지 않는다.

**(e) 미해결로 남겨 두어야 할 것** (숨기지 말고 "다음 과제" 슬라이드로):
- 외부 GT 부재 → `e_task_pose` 열은 비어 있고 Table 3(task-level 물리 실행)은 보류.
- cam 0의 reprojection이 다른 카메라(0.73–1.64 px) 대비 11 px 수준으로 튄다. 사후 제거하지 않고 보존했으며 detector/pose-ambiguity 진단 대상이다. 새 mask를 도입하려면 **fitting 전에 정의하고 전 행을 같은 mask로 재실행**해야 한다.
- 로봇 스케일 k는 잠정값(1.4–2.3% 범위). 다이얼 게이지 물리 측정으로 확정 필요.

---

# 출력 형식

1. **슬라이드 아웃라인**을 먼저 제시 (총 18–22장 목표, 각 장에 한 줄 메시지). Part 1.5는 최소 3장을 배정한다 (Ours 정의 / 자유도 설명 1.5-1 / D1 결과) — 이 발표에서 가장 중요한 부분이다.
2. 그다음 **각 슬라이드를 다음 형식으로** 작성:

```
### Slide N — <제목>
**한 줄 메시지:** ...
**본문 불릿:** (3–5개, 각 1행)
**수식/표/그림:** (LaTeX 수식 또는 표 마크다운, 또는 "그림: <무엇을 그릴지 구체적으로>")
**발표 스크립트:** (구어체 3–6문장)
**근거 파일:** (경로 + 어떤 값인지)
```

3. 마지막에 **예상 질문 & 답변** 9개를 붙여라. 최소한 다음은 포함하고, **각 답변은 "모른다/아직 아니다"를 인정하는 형태여도 좋다**:
   - "FK를 정답으로 쓰면서 FK 오차를 보정한다는 건 순환논법 아닌가?" → 1-3-2의 정면 돌파 논리를 그대로 쓸 것
   - **"그럼 FK로 미지수를 제거하지 않는 것인가?"** → 1.5-1의 자유도 표를 그대로 쓸 것. 제거되는 건 1개가 아니라 6×S=72 DOF이고, 권장 구성은 그걸 제거하지 않되 FK를 전역 9 DOF 회귀의 지도 신호로 쓴다고 답하라
   - "보정을 얹으면 FK-fixed가 이기나?" → LOSO 13-fold에서는 **아니다**(joint_fk_fixed+Ridge 3.057로 꼴찌). D1 2×2에서도 최고 셀은 A2+Ridge(1.847)다
   - "FK 고정을 하면 조건수가 좋아진다면서 왜 안 쓰나?" → 조건수·속도 이득(316→51, 106→19 nfev)은 인정하되 정확도 이득이 아니며, soft anchor가 그 중간이라고 답할 것
   - "선형 회귀 대신 GP나 신경망을 쓰면 더 좋지 않나?" → 잔차의 물리적 원인이 1차라는 근거 + 13 set 표본 크기로 답할 것
   - "A2→A3가 나빠졌는데 왜 A3가 Ours인가?" → 수렴 안정성으로만 답하고 정확도로 답하지 말 것
   - "보정으로 재투영 오차가 얼마나 줄었나?" → **줄지 않는다**(보정은 extrinsic을 바꾸지 않음)고 정확히 답할 것
   - "held-out reprojection px가 실제 로봇 작업 정확도와 무슨 상관인가?"
4. 언어는 **한국어**, 기술 용어는 영문 병기(예: 재투영 오차 reprojection error).
5. 수식은 LaTeX으로 쓰되 슬라이드에 들어갈 정도로만 간결하게. 유도 과정은 생략하고 **각 항이 무엇을 뜻하는지**를 한 줄씩 설명하는 방식으로.
6. 확인하지 못한 값은 지어내지 말고 `[확인 필요: <파일>]`로 표기하라.
