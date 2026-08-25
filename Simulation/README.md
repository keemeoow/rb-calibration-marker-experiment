# Calibration Simulation — 7-Experiment Ablation

## 목차

- [7개 실험 (확정 리스트)](#toc-section-1)
- [실행](#toc-section-2)
- [지표 (Table 2a 열)](#toc-section-3)
- [폴더 구조](#toc-section-4)
- [노이즈 모델](#toc-section-5)
- [알려진 한계 (논문에 명시할 것)](#toc-section-6)

멀티카메라(eye-to-hand) + 그리퍼카메라(eye-in-hand) + 큐브/보드 캘리브레이션을 **순수 SE(3)
기하 시뮬레이션**(렌더링 없음, ground-truth 보유)으로 검증한다. **자체 완결 패키지** — 부모
저장소에 의존하지 않는다(numpy + scipy 만 필요).

> 설계: **통합 코드(core) 하나 + 7개 얇은 실험 스크립트.** 각 실험은 core 의 `ExpConfig`
> 설정만 다르다.

---

<a id="toc-section-1"></a>

## 7개 실험 (확정 리스트)

기본 = **EXP1 (Ours)** = Step3형 FK prior 보정 + 통합 캘리브 + 큐브+보드. 여기서 하나씩 제거:

| # | FK | 캘리브 | 마커 | 의미 |
|---|---|---|---|---|
| **EXP1★** | corrected-FK (`factor`) | 통합 | 큐브+보드 | **Ours (기본)** |
| EXP2 | corrected-FK (`factor`) | 따로 | 큐브+보드 | −통합 |
| EXP3 | corrected-FK (`factor`) | 통합 | 큐브만 | −보드 |
| EXP4 | no-FK(vision) (`none`) | 통합 | 큐브+보드 | −FK |
| EXP5 | no-FK(vision) (`none`) | 따로 | 큐브+보드 | −FK −통합 |
| EXP6 | no-FK(vision) (`none`) | 통합 | 보드만 | −큐브 |
| EXP7 | FK-fixed (`fixed`) | 통합 | 큐브+보드 | raw FK 고정 대조 |
| EXP8 | corrected-FK (`corr`) | 통합 | 큐브+보드 | 구 방식(Ridge 후보정) 비교군 |

제약: **보드만 + FK 는 불가**(보드는 로봇이 위치를 모름 = FK 없음).

### FK 표시 구분 3개와 내부 코드 모드

- **no-FK(vision)** (`none`): 큐브를 미지수로 추정하고 cube-pose FK를 사용하지 않는다.
- **FK-fixed** (`fixed`): 큐브를 raw FK 상수로 고정하고 카메라·gTc만 최적화한다.
- **corrected-FK** (`factor`, `corr`): FK를 보정해 사용한다. 현재 주 방법인 `factor`는 큐브를 자유변수로 두고 FK를 **BA 안의 공분산 가중 robust 잔차 블록**으로 추가한다.
  sigma_FK(2.0mm / 0.30°)·Huber f_scale 은 `core/methods.py` 모듈 상수로 **전 실험 동결**.
  회전까지 함께 구속되며, FK 가 크게 틀린 set 은 Huber 가 자동으로 감쇠한다.
- `corr`는 no-FK(vision)으로 캘리브레이션한 뒤 예측 **위치만** [1,x,y] Ridge로 후보정하는 corrected-FK의 구 비교군이다.
  회전을 보정하지 않아 "3D pose calibration" 으로 설명하기 어렵다.

`[1,x,y]` Ridge는 corr의 일부가 아니다. C1 실데이터의 **출력 post-correction**을 재현할 때만
`ExpConfig(post_correction="ridge")`로 별도 활성화한다. 실데이터에서 이 지표는 외부 물리 GT가
아닌 FK proxy 일치도이며, 절대 물리 정확도를 뜻하지 않는다.

### 전체 조건 실행

유효한 `통합/독립 × 보드/큐브/둘 다 × no-FK/fixed-FK/corr` 14개 조합은 다음으로 실행한다.

```bash
python run_factorial.py --seeds 20 --splits 3 \
  --dump results/tables/factorial.json

# 실제형 systematic FK 오차와 관측 이상치를 함께 주는 예
python run_factorial.py --seeds 20 --splits 3 \
  --fk_sys_mm 10 --fk_sys_deg 5 --outlier_rate 0.02
```

보드만 조건에는 큐브 FK prior가 존재하지 않으므로 `fixed/corr` 조합을 만들지 않는다.

---

<a id="toc-section-2"></a>

## 실행

```bash
cd Simulation

# 개별 실험
python experiments/exp1.py --seeds 20

# 7개 전체 → Table 2a
python run_all.py --seeds 20 --dump results/tables/table2a.json
```

의존: `numpy`, `scipy`. (conda 환경 `rb-calib` 사용:
`/home/sstone/anaconda3/envs/rb-calib/bin/python run_all.py`)

> BA(통합 최적화)가 무거워 seed×조합이 많으면 느리다. `run_config(..., n_splits=)` 로
> seed 당 holdout 조합 수를 제한한다(기본 3). 통계는 seed 수로 확보.

---

<a id="toc-section-3"></a>

## 지표 (Table 2a 열)

| 지표 | 뜻 | 비고 |
|---|---|---|
| **N_reg** | 등록된 고정 카메라 수 | |
| **e_X** (mm/°) | 변환행렬(bTf + gTc) GT 대비 오차 | **시뮬 핵심** |
| **e_task** (mm/°) | held-out 큐브 pose 예측 오차 | 실전 성능 |
| **e_cross** (mm) | 카메라 간 큐브위치 일관성 | |
| **e_reproj** (px) | 재투영 오차 | **corner-level 필요(미구현)** |

`e_X`(bTf 와 gTc 의 평균)는 해석이 모호하므로 **bTf 와 gTc 를 분리해 보고**한다.

#### 재투영 규약 (중요)
- 씬이 보관한 **원본 노이즈 2D 코너**에 직접 재투영한다.
  (구버전은 프론트엔드 PnP 자체 잔차 `reproj_seed` 를 모든 방법에 똑같이 넣어
  **방법별 차이가 아예 없었다** — 방법 비교 지표가 아니었음.)
- 타깃 pose 는 GT 가 아니라 **모델이 추정한 값**을 쓴다.
- **leave-one-camera-out**: 평가 대상 카메라를 뺀 나머지 카메라+그리퍼로 타깃 pose 를
  추정한 뒤 대상 카메라에 재투영 → 자기 관측을 자기가 맞추는 자명한 해를 배제.
- 재투영에는 그 관측이 실제로 쓴 K/dist(부정확 intrinsic)를 사용한다. 참 K 는 GT 누출.

### 절제로 검증되는 것
- **FK factor → bTf/gTc/e_task 개선**: EXP1 vs EXP4
- **보드 → bTf 개선**: EXP1 vs EXP3
- **통합 → gTc/e_cross 개선**: EXP1 vs EXP2
- **corrected-FK vs FK-fixed**: FK가 정확하면 비슷하고, FK가 부정확해질수록 corrected-FK가 우세하다 (EXP1 vs EXP7).
- **큐브 → 커버리지**: 이 카메라 배치에서 보드만으로는 고정 카메라 일부가 등록되지 않는다 (EXP6 의 N_reg)

### 통계 규약
집계 단위는 **seed**. seed 안의 split 은 같은 씬을 공유해 독립 표본이 아니므로
split 을 먼저 평균한 뒤 seed 간 통계를 낸다. Ours 대비 **paired bootstrap 95% CI** 를
보고하며, **CI 전체가 0 보다 커야**(다른방법 − Ours > 0) "Ours 가 유의하게 우수"라고 말한다.

---

<a id="toc-section-4"></a>

## 폴더 구조

```
Simulation/
├── core/                   # 통합 엔진 (한 곳)
│   ├── se3.py              # SE(3) 유틸 (자체 완결)
│   ├── scene.py            # 씬 생성 + 노이즈 주입
│   ├── methods.py          # 통합/독립 × FK 3-값 × 마커
│   ├── metrics.py          # e_X, e_task, e_cross, N_reg
│   └── experiment.py       # run_config(cfg): 통합 러너
├── configs.py              # 7개 ExpConfig
├── experiments/exp1..7.py  # 7개 얇은 스크립트
├── run_all.py              # 7개 전체 → Table 2a
└── results/tables|figures/ # 산출물
```

---

<a id="toc-section-5"></a>

## 노이즈 모델

- **systematic** : 타깃 base 위치 (x,y) 에 선형 의존하는 편향 (움직이는 큐브에서 나타남;
  corrected-FK가 학습). 실제 검출오차의 지배성분(렌즈왜곡·intrinsic 잔차 등).
- **jitter** : 매 관측 독립 가우시안 검출 지터 (보드·큐브 공통, 항상). 고정 보드도 현실적
  노이즈를 갖게 함.
- **fk_noise** : 로봇 FK 큐브 prior 에 SE(3) 섭동 (Fig B: FK 부정확 모델). `SimScene(fk_noise_mm=)`.

---

<a id="toc-section-6"></a>

## 알려진 한계 (논문에 명시할 것)

- **corner-level 시뮬** → `e_reproj(px)`, Fig A(코너 노이즈 σ px). 현재는 pose-level.
- **Fig A / Fig B / Table 2b** 러너 (scene 은 fk_noise 지원하므로 Fig B 는 바로 확장 가능).

자세한 감사·계획: [PLAN_AND_AUDIT.md](PLAN_AND_AUDIT.md).
