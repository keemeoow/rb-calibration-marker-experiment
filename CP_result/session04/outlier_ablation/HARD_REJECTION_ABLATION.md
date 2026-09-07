# Hard-Rejection Sensitivity (관측 사전 제외 민감도)

## 결론

strict 정책은 Table 1 학습에서 cube 관측 2개(32 corners)를 추가 제외했지만, 모든 방법의 held-out 전체 RMSE 변화는 최대 **0.49%**였다. A1→A2 통합 최적화 개선 방향도 유지됐다. 따라서 현재 결론은 이 두 경계 관측의 포함 여부에 민감하지 않다.

단, 이는 동일 내부 held-out reprojection에 대한 민감도 결과이며 절대 정확도 또는 외부 GT 검증을 대신하지 않는다.

## 무엇을 바꿨나

- Standard: cube PnP RMSE ≤ 3.0 px, inlier fraction ≥ 0.0; board corners ≥ 4
- Strict: cube PnP RMSE ≤ 2.0 px, inlier fraction ≥ 0.9; board corners ≥ 12
- Train: 119 obs / 3686 corners → 117 obs / 3654 corners

실제 Table 1 학습에서 추가 제외된 관측:

- `cube:E0063:cam2`: PnP RMSE 2.0999 px; 16 corners, reason `pnp_rmse_above_2px`
- `cube:E0077:cam2`: PnP inlier fraction 0.875; 16 corners, reason `inlier_fraction_below_0.9`

## 왜 직접 비교가 공정한가

- 같은 event-grouped split, 동일 seed와 3개 초기값을 사용했다.
- intrinsic, distortion, target geometry, FK, solver, soft-L1 loss를 동일하게 고정했다.
- held-out는 양쪽 모두 35 obs / 939 corners이며 SHA-256 `28a8cd6fe28618eb4af05e9fadf19448fd115d5e721842625f1432a2aaa8a942`로 완전히 같다.
- strict 여부는 최적화 전에 고정되며 fitted model 출력으로 평가 관측을 제거하지 않는다.

## 결과

| Method | Standard px | Strict px | Strict 변화 | Held-out obs/corners |
| --- | ---: | ---: | ---: | ---: |
| A0 | 4.0530 | 4.0530 | +0.00% | 17/703 |
| A1 | 4.0837 | 4.0635 | -0.49% | 35/939 |
| A2 | 3.8901 | 3.8901 | -0.00% | 35/939 |
| A3 | 4.7835 | 4.7862 | +0.06% | 35/939 |
| A4 | 3.8899 | 3.8899 | -0.00% | 35/939 |
| A5 | 3.7270 | 3.7097 | -0.47% | 35/939 |
| B1 | 4.0783 | 4.0656 | -0.31% | 35/939 |
| B2 | 4.4827 | 4.4791 | -0.08% | 18/236 |
| B3 | 4.0530 | 4.0530 | +0.00% | 17/703 |

## 주장별 확인

- A1→A2 held-out 개선: standard 4.74% → strict 4.27%로 방향이 유지됐다.
- A2와 A4 차이: standard +0.0001 px, strict +0.0002 px로 둘 다 사실상 동률이다. 따라서 이 실험도 A4 우월성 주장의 근거가 아니다.
- A3 raw-FK-fixed는 두 정책 모두 A2/A4보다 높은 오차를 유지했다. 즉 A3의 차이는 해당 두 경계 관측만으로 설명되지 않는다.
- A5도 두 정책에서 낮은 내부 px를 유지한다. External GT 공개 전에 방법과 alignment artifact가 frozen이면 최종 후보지만, strict 민감도 통과 자체가 외부 물리 정확도 우월성을 뜻하지 않는다.

## 재현 명령

```bash
python3 tools/summarize_hard_rejection_ablation.py
```
