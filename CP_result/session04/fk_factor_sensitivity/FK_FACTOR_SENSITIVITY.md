# FK Factor Sensitivity (Preflight)

목적: 8/3 피드백 #4, 즉 "카메라 관측 수가 많아 FK 항이 묻히는가"에 답하기 위한 preflight 분석이다.

이 실험은 canonical Table 1을 덮어쓰지 않는다. Session04 manifest, split, solver, A4 row를 고정하고 Simulation prior covariance의 표준편차 scale만 바꾼다. `preflight_std_scale < 1`은 FK factor를 더 강하게, `> 1`은 더 약하게 넣는다는 뜻이다.

> External GT를 사용하지 않았으므로 이 결과는 물리 정확도 우월성 근거가 아니라 FK factor 영향도 점검이다.

- A2 held-out baseline: overall `3.8901` px, board `3.9840` px, cube `3.5958` px
- Scales: 0.25x, 0.5x, 1.0x, 2.0x, 4.0x
- Runs per scale: 3

| Std scale | FK weight approx | trans std mm | rot std deg | A4 held-out overall | board | cube | Δoverall vs A2 | FK cost fraction | Converged |
| ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 0.25x | 16.00x | 0.500 | 0.075 | 3.8684 | 3.9870 | 3.4914 | -0.0217 | 0.6540% | 3/3 |
| 0.50x | 4.00x | 1.000 | 0.150 | 3.8841 | 3.9900 | 3.5499 | -0.0060 | 0.2722% | 3/3 |
| 1.00x | 1.00x | 2.000 | 0.300 | 3.8899 | 3.9884 | 3.5805 | -0.0002 | 0.1220% | 3/3 |
| 2.00x | 0.25x | 4.000 | 0.600 | 3.8902 | 3.9858 | 3.5904 | +0.0001 | 0.0469% | 3/3 |
| 4.00x | 0.06x | 8.000 | 1.200 | 3.8902 | 3.9846 | 3.5943 | +0.0001 | 0.0141% | 3/3 |

## 해석

- 이 scale sweep 안에서 가장 낮은 A4 held-out overall은 `0.25x`의 `3.8684` px다.
- 가장 강한 FK 설정 `0.25x`와 가장 약한 설정 `4.00x` 사이의 held-out overall 차이는 `-0.0218` px다.
- 따라서 현재 데이터에서는 FK factor가 완전히 무시된다고 보기는 어렵지만, A2 대비 A4의 내부 held-out 차이는 매우 작아 최종 우월성 claim으로 쓰기에는 부족하다.
- 이 결과는 #4에 대한 개선된 답변이다. 단순 residual 개수나 FK cost fraction만 보지 않고, covariance scale을 바꿨을 때 출력 지표가 실제로 움직이는지도 같이 본다.

## 남은 것

- measured FK covariance가 없으므로 여전히 preflight다.
- 다음주 Independent External GT 이후 A2/A4 물리 정확도 비교로 최종 FK claim을 확정한다.
