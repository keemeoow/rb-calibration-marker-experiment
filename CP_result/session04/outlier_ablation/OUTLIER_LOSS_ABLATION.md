# Outlier Loss Ablation (이상치 손실함수 대조)

이 실험은 사전 품질 마스크·train/test split·관측 코너를 그대로 고정하고 최종 최적화 loss만 `soft_l1`과 `linear`로 바꿨다. 따라서 차이는 최적화 중 soft weighting 효과이며, hard rejection threshold 변화 효과가 아니다.

- Train: 119 observations, 3686 corners, SHA-256 `c116ad02c4e4625a756aeb75e9b1b327c47ee59c2c54fe97556cf08182e65990`
- Held-out: 35 observations, 939 corners, SHA-256 `28a8cd6fe28618eb4af05e9fadf19448fd115d5e721842625f1432a2aaa8a942`

| Method | Held-out soft-L1 px | Held-out linear px | soft-L1 change | Fixed→Fixed cube soft/linear px | Gripper→Fixed cube soft/linear px |
| --- | ---: | ---: | ---: | ---: | ---: |
| A0 | 4.0530 | 4.0785 | -0.63% | 6.6803 / 6.6803 | 7.4402 / 7.5668 |
| A1 | 4.0837 | 4.0888 | -0.12% | 6.3167 / 5.6121 | 7.6100 / 7.5738 |
| A2 | 3.8901 | 3.9799 | -2.26% | 3.9992 / 4.8293 | 6.8987 / 7.1625 |
| A3 | 4.7835 | 4.7750 | +0.18% | 5.8562 / 5.8161 | 7.1666 / 7.5306 |
| A4 | 3.8899 | 3.9788 | -2.23% | 4.0375 / 4.8223 | 6.9064 / 7.1652 |
| A5 | 3.7270 | 3.8979 | -4.38% | 3.4706 / 3.2235 | 6.2895 / 6.3779 |
| B1 | 4.0783 | 4.0857 | -0.18% | 6.2871 / 5.6205 | 7.5907 / 7.5840 |
| B2 | 4.4827 | 4.8736 | -8.02% | 3.5610 / 4.1416 | 7.2526 / 8.1543 |
| B3 | 4.0530 | 4.0785 | -0.63% | 6.6767 / 6.6808 | 7.4385 / 7.5665 |

해석: held-out 전체 오차 기준으로 soft-L1이 linear보다 낮은 방법은 A0 0.63%, A1 0.12%, A2 2.26%, A4 2.23%, A5 4.38%, B1 0.18%, B2 8.02%, B3 0.63%이고, 높은 방법은 A3 0.18%이다. 따라서 robust loss가 모든 조건을 일괄 개선한다고 주장할 수 없으며, 방법별·표적별 결과를 함께 보고해야 한다.

A5는 train-vision-aligned FK를 hard-fixed한 최종 후보가 될 수 있다. 단, 이 sensitivity 결과만으로 A5를 외부 물리 정확도 winner로 확정하지는 않으며, GT 공개 전에 방법과 alignment artifact를 frozen해야 한다.

한계: 이 결과는 이미 적용된 사전 PnP 품질 마스크를 고정한 실험이다. 프레임/관측 hard rejection 자체의 민감도는 threshold를 사전 등록한 별도 실험으로 확인해야 한다.
