# Simulation 코드 리뷰 — 검증 & 수정 계획

> ✅ **2026-08 상태: 6개 전부 수정·재실행 완료.** 재판정 결과는 [SIM_RESULTS.md](SIM_RESULTS.md).
> 요지: robust 통일(⑥) 후 "Ours 오검출 압승"이 사라지고, **통합≫독립**(정합)이 진짜 기여로 남음.
> FK 보정은 무의미(중립~해로움), 큐브 필수·보드 불필요. 아래는 수정 당시의 검증·계획 기록.

real 데이터 코드 작성자(리뷰어)가 지적한 6개 이슈를 **실제 코드로 하나씩 검증**한 결과와 수정 계획.
결론부터: **리뷰가 대부분 정확**하다. 특히 ⑥(이상치 unfair)은 우리가 관측한 "이상치에서 Ours 압승"이
**인위적일 수 있음**을 정확히 짚는다.

## 요약

| # | 이슈 | 판정 | 영향 | 우선순위 |
|---|---|---|---|---|
| **6** | 이상치 실험이 Ours에 유리 (비강건 PnP + 완벽 FK label) | ✅ Valid | **핵심 결과 왜곡** | **1** |
| **1** | no-FK 비교군이 init에서 FK/GT 사용 | ✅ Valid | no-FK/독립 공정성 | 2 |
| **5** | 통계 약함 + anchor weight 스크립트마다 다름 | ✅ Valid | 신뢰도 | 3 |
| **4** | 큐브 기하(반변)·intrinsic 부정확 | ✅ Valid | 현실성 | 4 |
| **3** | reproj가 방법별 성능이 아님 / raw corner 미보존 | ✅ Valid | reproj 지표 | 5 |
| **2** | independent align이 평가에 미적용 | ✅ **이미 수정됨** | — | 완료 |

---

## ① no-FK가 초기화에서 FK/GT를 사용한다 — ✅ Valid

**리뷰**: `_bootstrap` 초기화가 `fk_cube`와 GT 보드 `bTboard`를 사용. independent는 초기값 영향을
그대로 받아 "FK 미사용"이 아니다. 보드-only hand-eye도 GT 보드 pose를 직접 사용.

**검증** (코드 확인):
- [core/methods.py:52,55](core/methods.py#L52) — `_bootstrap`이 `sc.fk_cube[s]`(FK), `sc.bTboard`(GT)로 카메라·gTc 초기화.
- [core/methods.py:247-252](core/methods.py#L247) — `_handeye_to_board`가 **GT 보드 pose를 잔차에 직접** 사용. (보드는 실제론 미지수)
- [core/methods.py:111](core/methods.py#L111) — 보드 초기값 fallback도 `sc.bTboard`(GT).

**판정**: 맞다. no-FK/independent가 순수하게 FK/GT-free가 아니다. 특히 **보드 위치를 GT로 쓰는 건 명백한 정보 누수**.

**수정**:
- GT는 **장면 생성·평가에만** 사용.
- visual-only 초기화(관측만).
- gauge freedom → **기준 카메라 1개 고정** 또는 명시적 gauge constraint.
- 보드도 **자유변수**로 추정.
- 모든 방법에 동일한 초기화 품질 + multi-start 횟수.

## ② independent align이 평가에 미적용 — ✅ 이미 수정됨

**리뷰**: `_rigid_align()`이 align을 계산하나 `predict_cube_pos/pose()`가 안 씀 → independent 불완전.

**검증**: [core/metrics.py:85,158](core/metrics.py#L85) — 현재 코드는 `_align_T`로 predict/e_cross에서
**align을 적용**한다 (커밋 `47550ba`, "e_task 카메라당 1표" 수정 때 함께 반영). 리뷰어가 받은 버전엔 없었음.

**판정**: 이미 고쳐짐. ✅

## ③ reproj가 방법별 성능이 아니다 — ✅ Valid

**리뷰**: `run_config`/`run_table2a`가 관측 생성 시 PnP 오차 `reproj_seed`를 모든 방법에 넣음.
또한 시뮬이 원본 noisy 2D corner를 보존 안 하고 PnP pose만 보존.

**검증**:
- [core/experiment.py:85-87](core/experiment.py#L85) — `run_config`가 `reproj_seed`(관측 PnP 오차, **모든 방법 동일값**)를 e_reproj로 사용. → 방법 비교 불가. **맞다.**
- 단 `run_paper_sim`은 `eval_model`의 `unified_reproj`(방법별)를 씀 → 그건 OK.
- **하지만 `unified_reproj`도 raw corner가 아니라 "관측 pose"에 재투영** → 진짜 재투영 아님. 시뮬이 raw corner를 저장 안 함.

**수정** (논문용 재투영):
- 생성된 **2D corner + visibility mask + marker ID 저장**.
- train 추정 transform 고정 → **held-out 원본 2D corner에 직접 재투영**.
- 방법별 **공통 camera·corner mask** 사용.
- **train reprojection과 held-out reprojection 분리**.

## ④ 큐브 기하가 실물과 다르다 — ✅ Valid

**리뷰**: `CUBE_HALF_M = 51/2 = 25.5mm`인데 실물 반변 ~29.5mm. 윗면 마커 중심 오프셋·실제
`marker_center_m` 미반영. 카메라도 평균 intrinsic + 회전 27° 강제.

**검증**: [core/targets.py:24](core/targets.py#L24) — `CUBE_HALF_M = CUBE_SIDE_SIZE_M/2 = 25.5mm`.
**마커 크기(51mm)를 큐브 크기로 사용** → 마커가 면 전체를 안 덮으면 물리 큐브가 더 큼(29.5mm).

**판정**: 맞다. 현재는 **"digital twin"이 아니라 "실측값 일부 쓴 합성 장면"**.

**수정**: 큐브 반변 **29.5mm** + 윗면 마커 오프셋 + 실제 `marker_center_m` 반영. (카메라는 실측 개별 intrinsic·자세 반영 검토)

## ⑤ 통계 설계가 약하다 — ✅ Valid (심각)

**리뷰**: 4 seeds, 앞 2 split만, `except: pass`로 예외 숨김, split을 독립 표본 취급, "실제 규모 동일 순위"
미검증, anchor weight가 스크립트마다 5.0/0.5/0.0.

**검증**:
- **anchor 불일치** ⚠️: `configs.EXP1`(=Ours)은 anchor **5.0**(`ExpConfig` 기본값 — 미설정 시), 그러나
  [run_which_wins.py:30](run_which_wins.py#L30)·`run_realistic`은 **0.5**. → **paper_sim의 "Ours"는 실제로 anchor 5.0이었다** (0.5라 말한 것과 다름). 리뷰어 지적 정확.
- 3~4 seeds, `n>=2`로 앞 2 split만, 여러 스크립트에 `except Exception: pass`. 다 사실.

**수정**:
- anchor를 **train-only inner validation으로 한 번 선택 후 전 평가에서 동결**.
- seeds↑ (예: 20+), **모든 holdout split** 사용, split 통계 처리 주의.
- `except: pass` **제거**(실패 표면화).

## ⑥ 이상치 실험이 Ours에 유리하게 설계됨 — ✅ Valid (핵심)

**리뷰**: corner에 큰 이상치 + 비강건 `solvePnP` + 비강건 LM. fixed/no-FK가 붕괴하는 동안 Ours만
완벽한 FK label로 사후 회귀 학습 → Ours 유리. 모든 방법에 동일 robust 전처리 필요.

**검증**:
- [core/project.py:93](core/project.py#L93) — `cv2.solvePnP` (**비강건**, Ransac 아님).
- `solve_unified` — `least_squares method="lm"` (**Huber/Cauchy 없음**).
- 이상치를 코너에 주입(기존 15px) 후, Ours의 2차 보정은 **완벽한 FK label** 대비 학습.

**판정**: 정확하다. **우리가 관측한 "이상치에서 Ours 압승"이 비강건 PnP + 완벽 FK label 덕분인
인위적 우위였을 가능성**이 크다. (실측 이상치는 ~2px로 작다는 것도 별도로 확인됨.)

**수정** (모든 방법 동일):
- `solvePnPRansac` 또는 동일 robust PnP.
- 동일 corner/marker rejection.
- joint solver에 **Huber/Cauchy loss**.
- 동일 실패 판정, fallback 금지.
- **robust frontend 유무를 별도 실험으로 분리**.

---

## 수정 우선순위 (제안)

1. **⑥ robust 통일** — 모든 방법 Ransac + Huber. → "이상치 Ours 우위"가 진짜인지 재판정 (논문 핵심).
2. **① init 정리** — 관측-only init + gauge 고정 + 보드 자유변수.
3. **⑤ 통계·anchor 동결** — anchor 하나로 통일, seeds↑, 전체 split, except 제거.
4. **④ 큐브 반변 29.5mm** + marker 반영.
5. **③ raw corner 보존** — 진짜 재투영 지표.
6. ~~② align~~ — 완료.

## 핵심 메시지

리뷰어의 지적은 대부분 옳고, 특히 **⑥과 ①은 지금까지 "Ours 유리" 결과를 인위적으로 만들었을 수
있다.** 이걸 고친 뒤 다시 돌려야 **Ours의 진짜 가치**(있다면)가 드러난다. 안 고치면 "Ours가 좋다"는
결론이 셋업 편향의 산물일 위험.
