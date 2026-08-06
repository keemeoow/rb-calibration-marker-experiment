# Calibration Simulation — 7-Experiment Ablation

멀티카메라(eye-to-hand) + 그리퍼카메라(eye-in-hand) + 큐브/보드 캘리브레이션을 **순수 SE(3)
기하 시뮬레이션**(렌더링 없음, ground-truth 보유)으로 검증한다. **자체 완결 패키지** — 부모
저장소에 의존하지 않는다(numpy + scipy 만 필요).

> 설계: **통합 코드(core) 하나 + 7개 얇은 실험 스크립트.** 각 실험은 core 의 `ExpConfig`
> 설정만 다르다.

---

## 7개 실험 (확정 리스트)

기본 = **EXP1 (Ours)** = FK 잔차보정 + 통합 캘리브 + 큐브+보드. 여기서 하나씩 제거:

| # | FK | 캘리브 | 마커 | 의미 |
|---|---|---|---|---|
| **EXP1★** | 잔차보정(corr) | 통합 | 큐브+보드 | **Ours (기본)** |
| EXP2 | 잔차보정 | 따로 | 큐브+보드 | −통합 |
| EXP3 | 잔차보정 | 통합 | 큐브만 | −보드 |
| EXP4 | 안씀(none) | 통합 | 큐브+보드 | −FK |
| EXP5 | 안씀 | 따로 | 큐브+보드 | −FK −통합 |
| EXP6 | 안씀 | 통합 | 보드만 | −큐브 |
| EXP7 | 고정(fixed) | 통합(=독립) | 큐브+보드 | FK 고정 대조 |

제약: **보드만 + FK 는 불가**(보드는 로봇이 위치를 모름 = FK 없음). **fixed 는 통합=독립**
(큐브 상수라 카메라 분리).

### FK 3-값
- **none** : 큐브를 미지수로 추정 (FK 미사용)
- **fixed**: 큐브 = FK 상수로 고정 → 카메라·gTc 만 최적화
- **corr** : none 으로 캘리브 후 train 잔차를 [1,x,y] Ridge 로 후보정 (**채택**)

---

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

## 지표 (Table 2a 열)

| 지표 | 뜻 | 비고 |
|---|---|---|
| **N_reg** | 등록된 고정 카메라 수 | |
| **e_X** (mm/°) | 변환행렬(bTf + gTc) GT 대비 오차 | **시뮬 핵심** |
| **e_task** (mm/°) | held-out 큐브 pose 예측 오차 | 실전 성능 |
| **e_cross** (mm) | 카메라 간 큐브위치 일관성 | |
| **e_reproj** (px) | 재투영 오차 | **corner-level 필요(미구현)** |

### 절제로 검증되는 것 (스모크 확인)
- **FK 보정 → e_task 개선**: EXP1 vs EXP4
- **보드 → bTf/e_X 개선**: EXP1 vs EXP3
- **통합 → gTc 개선**: EXP1 vs EXP2
- **fixed**: FK 완벽하면 bTf 최고, 하지만 e_task 최악 (EXP7)

---

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

## 노이즈 모델

- **systematic** : 타깃 base 위치 (x,y) 에 선형 의존하는 편향 (움직이는 큐브에서 나타남;
  FK 후보정이 학습). 실제 검출오차의 지배성분(렌즈왜곡·intrinsic 잔차 등).
- **jitter** : 매 관측 독립 가우시안 검출 지터 (보드·큐브 공통, 항상). 고정 보드도 현실적
  노이즈를 갖게 함.
- **fk_noise** : 로봇 FK 큐브 prior 에 SE(3) 섭동 (Fig B: FK 부정확 모델). `SimScene(fk_noise_mm=)`.

---

## 아직 미구현 (추후)

- **corner-level 시뮬** → `e_reproj(px)`, Fig A(코너 노이즈 σ px). 현재는 pose-level.
- **Fig A / Fig B / Table 2b** 러너 (scene 은 fk_noise 지원하므로 Fig B 는 바로 확장 가능).

자세한 감사·계획: [PLAN_AND_AUDIT.md](PLAN_AND_AUDIT.md).
