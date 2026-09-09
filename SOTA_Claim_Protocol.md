# 제안 방법 판정 계약 (A/B 계열 전용)

> Version: 1.0
> 기준일: 2026-08-06 (원문) / 2026-08-13 분리
> 원문: `SOTA_Comparision.md` v1.0 의 제안 방법 관련 절

`SOTA_Comparision.md`를 학부연구생용 baseline 설명서로 정리하면서,
**제안 방법(A2/A4/A5/B1)과 그 우위 판정에 관한 내용**만 이 문서로 옮겼다.
비교 대상 논문 설명은 `SOTA_Comparision.md`를 본다.

---

## 1. 비교 목적

이 비교는 단순히 많은 방법을 나열하기 위한 것이 아니다. 다음 세 질문을 분리해 검증한다.

1. 카메라별 independent calibration보다 multi-camera joint calibration이 우수한가?
2. Robot FK를 고정 관측으로 사용하는 기존 방법보다 covariance-weighted robust soft-FK factor가 우수한가?
3. Calibration 이후의 residual correction이 독립 외부 GT에서도 추가 이득을 주는가?

세 번째 질문은 A5가 A4 대비 사전 정의한 translation·rotation 합격 조건을 모두 통과한 경우에만
최종 기여로 채택한다. 그 전까지 제안 방법은 A4(Ours-core)이다.

## 2. 제안 방법 행

| ID | 방법 | 계열 | Joint multi-camera | Pixel-level objective | FK uncertainty model | 실행 상태 |
| --- | --- | --- | --- | --- | --- | --- |
| B1 | Independent fair | 카메라별 독립 | 아니오 | 예 | 없음 | 필수 |
| A2 | Unified visual-only | cube+board U-BA | 예 | 예 | 사용 안 함 | 필수 |
| Ours-core | A4 | unified cube+board U-BA | 예 | 예 | covariance-weighted robust soft-FK | 필수 |
| Ours-full 후보 | A5 | A4 + held-out 6-DoF correction | 예 | A4와 동일 | A4와 동일 | 합격 시에만 |

Soft-FK 또는 robot uncertainty 자체를 최초 기여로 주장하지 않는다.
Strobl–Hirzinger(2006), Ha(2023), Ulrich–Hillemann(2024)이 stochastic 또는
uncertainty-aware hand–eye calibration을 이미 다룬다.
제안 방법의 차별점은 이러한 불확도 처리를 mixed eye-in-hand/eye-to-hand,
raw pixel-level, cube+board multi-camera U-BA의 shared latent cube poses에 결합하는 데 둔다.

## 3. 통계 계약

추론 단위는 camera-installation session이다. 동일 session의 blind poses를 독립 session처럼 세지 않는다.

1. Session을 paired 방식으로 복원추출한다.
2. 각 session 내부에서 동일 blind pose pair를 복원추출한다.
3. Method 간 paired difference를 계산한다.
4. Hierarchical bootstrap 95% CI를 산출한다.

Primary comparison family에는 Holm correction을 적용한다.
5 sessions × 30 blind poses는 pilot으로 사용하고,
최종 session 수는 pilot의 session-level variance를 이용한 power analysis로 정한다.

## 4. 사전 합격 조건

수치 margin은 robot task tolerance와 외부 GT uncertainty를 기준으로 실험 전에 확정한다.

```text
UpperCI(mean(TRE_A4 - TRE_baseline)) < 0
UpperCI(P95_A4 - P95_baseline) < m_P95
UpperCI(e_R,A4 - e_R,baseline) < m_R
FailureRate_A4 - FailureRate_baseline <= m_fail
```

Ours-core 우위 주장은 다음을 모두 만족할 때만 허용한다.

1. 사전 지정한 primary baseline(B1, best classical, D3) 대비 mean `TRE_t` superiority를 통과한다.
2. Rotation error가 superiority 또는 `m_R` 이내 non-inferiority를 통과한다.
3. P95와 failure rate가 각 margin 안에 있다.
4. Worst workspace stratum에서 큰 열화가 없다.
5. 차이가 외부 GT measurement uncertainty floor보다 해석 가능한 크기다.

A5는 A4 대비 같은 계약을 translation과 rotation 모두에서 통과할 때만 Ours-full로 채택한다.

## 5. 보고 문장 계약

외부 GT 전에는 다음 수준만 허용한다.

> Unified visual calibration은 기존 데이터의 held-out pixel consistency를 개선했다.
> Fixed-weight soft anchor는 유망한 선행 결과를 보였지만,
> covariance-weighted robust A4와 독립 외부 GT 평가는 아직 완료되지 않았다.

외부 GT와 통계 계약을 통과한 뒤에만 다음 형식의 문장을 사용한다.

> 동일한 independent sessions와 blind external-GT poses에서 A4는 사전 지정한 primary baselines 대비
> translation error를 낮추면서 rotation, P95 및 failure-rate non-inferiority 조건을 충족했다.

"가장 정확하다", "SOTA를 달성했다" 같은 포괄적 표현은
평가한 방법·셋업·workspace·GT 범위를 문장 안에서 제한할 수 있을 때만 사용한다.

## 6. 전체 결과 표 템플릿

baseline 행은 `SOTA_Comparision.md` 참조. 아래는 제안 방법 행을 포함한 전체 표이다.

| Method | Input | Cameras supported/evaluated | `TRE_t` mean [95% CI] ↓ | `TRE_t` P95 ↓ | `e_R` mean [95% CI] ↓ | ADD/ADD-S ↓ | Coverage ↑ | Failure ↓ | Time ↓ |
| --- | --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| C1 Tsai | target + FK/PnP | — | — | — | — | — | — | — | — |
| C2 Park | target + FK/PnP | — | — | — | — | — | — | — | — |
| C3 Horaud | target + FK/PnP | — | — | — | — | — | — | — | — |
| C4 Daniilidis | target + FK/PnP | — | — | — | — | — | — | — | — |
| D1 Shah | target + FK/PnP | — | — | — | — | — | — | — | — |
| D2 Tabb | target + FK/images | — | — | — | — | — | — | — | — |
| D3 Allegro | board + FK + images | — | — | — | — | — | — | — | — |
| D4 Calib3R | targetless RGB + FK | — | — | — | — | — | — | — | — |
| D5 Ha probabilistic AXYB | target poses + robot poses/covariance | — | — | — | — | — | — | — | — |
| D6 Uncertainty-Aware HEC | target/images + uncertain robot poses | — | — | — | — | — | — | — | — |
| B1 Independent fair | cube+board + FK + images | — | — | — | — | — | — | — | — |
| A2 Unified visual-only | cube+board + images | — | — | — | — | — | — | — | — |
| A4 Ours-core | cube+board + soft-FK + images | — | — | — | — | — | — | — | — |
| A5 Ours-full candidate | A4 + correction | — | — | — | — | — | — | — | — |

`Cameras supported/evaluated`는 반드시 채운다.
지원하지 못한 camera를 제외하고 얻은 낮은 오차와 전체 camera coverage를 혼동하지 않는다.
