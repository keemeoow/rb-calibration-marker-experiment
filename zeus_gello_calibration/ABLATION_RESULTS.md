# Zeus 캘리브레이션 방식 비교 (2026-09-09 촬영 데이터)

> late_table1(CP_result/session04, UR3)과 같은 취지로, Zeus 데이터에 대해
> "통합(unified) vs 독립(independent) x FK 처리 방식" 4가지를 비교한 결과다.
> table1.py의 정식 held-out split/공유 baseline 절차 그대로는 아니고(train-pooled
> + leave-one-out으로 근사), C1(통합 vs sequential)만 table1.py의 실제 알고리즘을
> 그대로 재현했다. 코드: `fit_calibration_methods.py`, `eval_heldout_and_consistency.py`.

## 데이터 풀 (모든 방식 공통, 114개 관측치)

| 소스 | 관측치 수 | 내용 |
|---|---:|---|
| session1 | 42 | 고정캠 3대 — 그리퍼로 쥔 큐브 (grasp+FK 모델) |
| session2-고정캠 | 42 | 고정캠 3대 — 바닥에 놓인 큐브 (세트별, 15곳) |
| session2-그리퍼캠 | 15 | 그리퍼캠 — 같은 바닥 큐브를 파킹 자세에서 봄 (신규 발견) |
| session3-그리퍼캠 | 15 | 그리퍼캠 — 바닥 마커보드 (eye-in-hand) |

## 4가지 방식 정의

| 방식 | 큐브 위치 처리 | 고정캠·그리퍼캠 관계 |
|---|---|---|
| **통합_no-fk** | session2 세트별 큐브 pose = 자유 변수, 고정캠+그리퍼캠 관측치가 **같이** 그 변수를 결정 | 4개 소스 전부 하나의 최소제곱으로 동시에 품 |
| **통합_raw-fk** | session2 큐브 pose = "명령한 place pose @ T_gripper_cube(session1 원값)"로 고정(상수) | 4개 소스 전부 하나의 최소제곱(단, 큐브가 상수라 사실상 고정캠/그리퍼캠 블록이 서로 안 엮임) |
| **sequential_no-fk** (table1.py A1 공식 알고리즘) | stage1(그리퍼만)이 큐브/보드/gtc를 먼저 자유롭게 풀고 **얼림** → stage2(고정캠만)가 그 위에서 T_base_Ci만 품 | 그리퍼 → 고정캠, 한 방향 핸드오프 (역방향 피드백 없음) |
| **독립_no-fk** (진짜 독립) | 고정캠 그룹(session1+session2-고정캠)과 그리퍼 그룹(session2-그리퍼캠+session3)을 **정보 교환 전혀 없이** 완전히 따로 풂. session2는 사후 합의 검증에만 사용 | 핸드오프 없음, 각자 로봇 FK로 base 좌표계에 독립적으로 연결(고정캠은 session1 grasp+FK, 그리퍼는 session3 board eye-in-hand+FK) |

`raw-fk` 계열은 no_fk에서만 sequential/진짜독립이 의미가 있다 — 큐브 위치를 상수로 고정하면 고정캠/그리퍼캠 블록이 항상 수학적으로 분리되어 통합·독립 구분 자체가 무의미해지기 때문에(block-separable) 별도로 안 만들었다.

**주의**: `통합_raw-fk`는 table1.py의 A3("raw-FK-fixed", 비전 개입 0인 순수 기계적 상수)와 이름만 같고 실제로는 다른 조건이다 — Zeus엔 그런 독립 측정 상수가 없어서 session1 비전 fit값(T_gripper_cube)을 앵커로 쓴다. 그래서 이 열은 "C3(FK 처리 방식) 검증"이 아니라 "우리만의 FK-고정 변형"으로만 해석해야 한다.

## 결과

| 방식 | Train RMSE (px) | Held-out RMSE (px) | Cross-cam (mm/deg) | 비고 |
|---|---:|---:|---:|---|
| **통합_no-fk** | **0.93** | **3.00** | **3.71 / 0.97** | 4개 중 train/held-out/cross-cam 전부 1등 |
| 통합_raw-fk | 1.37 | 3.39 | 4.59 / 1.07 | A3 아님 (위 주의 참고) |
| sequential_no-fk | 1.46 | 3.28 | 4.25 / 1.17 | table1.py A1 공식 알고리즘 그대로 |
| 독립_no-fk | 고정캠 1.12 / 그리퍼 0.66 | 3.17 | 4.05 / 0.96 | 사후 합의: 평균 3.18mm/0.95°, 최대 5.57mm/2.45° |

## 핵심 결론 (C1: 통합 vs sequential/독립)

1. **통합_no-fk가 모든 지표에서 최고다.** train(0.93px)에서 가장 크게 앞서고, held-out(3.00px)/cross-cam(3.71mm)에서도 계속 1등이지만 격차는 train만큼 크지 않다 — train RMSE 차이의 상당 부분은 "파라미터를 공유해서 학습 데이터에 더 잘 맞춘 것"이지 순수 일반화 이득만은 아니라는 신호다.
2. **sequential(table1.py 공식 baseline)은 4개 중 가장 나쁘다** (train 1.46px) — UR3의 A1→A2 개선(4.14→3.60px)과 같은 방향. 그리퍼가 먼저 큐브를 정하고 고정캠이 거기 무조건 맞추는 구조라, 더 정밀한 고정캠(reproj err 0.1~1px)이 상대적으로 노이즈 큰 그리퍼(2px대)의 오차를 그대로 물려받기 때문으로 보인다.
3. **독립_no-fk(진짜 독립)은 sequential보다 낫다** (held-out 3.17 vs 3.28px) — 한쪽이 다른 쪽에 일방적으로 맞추는 구조가 없어서 그런 걸로 보인다. 다만 고정캠 그룹(1.12px)과 그리퍼 그룹(0.66px)을 각자 보면 둘 다 그럴듯한데, **같은 큐브에 대해 서로 3.18mm/0.95° 어긋난다** — "각자는 괜찮아 보여도 서로 안 맞을 수 있다"는 걸 직접 보여주는 지표이자, 통합이 왜 필요한지에 대한 가장 직접적인 근거.

## 아직 안 된 것 / 한계

- **C2(큐브 vs board-only)**: 검증 안 함. Zeus는 처음부터 큐브+보드를 같이 썼음.
- **C3(FK 처리)**: raw-fk/no-fk 2개만 있고, corrected-FK(A4)·vision-aligned-FK(A5, UR3에서 제일 좋았던 방법)는 없음. 그리고 있는 raw-fk도 진짜 A3가 아님(위 주의 참고).
- **C4(외부 GT)**: `gt_pick_test.py`/`gt_compare_fits.py`로 진행 중. 통계적으로 엄밀한 수준(TRE/회전/P95/실패율 집계, 사전등록)은 아직 아님. 또한 GT 큐브 자체의 물리 치수(config vs 실물)가 아직 미해결 — 실측 후 재검증 필요.
- **T_gripper_cube 적용 방식**: `gt_pick_test.py`는 이제 T_gripper_cube 전체(x,y,z,rz,ry,rx)를 반영하도록 고쳤지만, `session2_pick_and_place.py`는 아직 예전 방식(z/ry/rx 고정값 + 큐브 원점 x,y)을 씀 — 필요하면 같이 고쳐야 함.
- held-out/cross-camera는 table1.py의 정식 split 절차가 아니라 leave-one-out으로 근사한 것 (train-pooled, held-out 완전 분리 아님).
