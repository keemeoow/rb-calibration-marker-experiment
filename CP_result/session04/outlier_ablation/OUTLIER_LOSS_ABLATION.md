# Outlier Loss Ablation (이상치 손실함수 대조)

이 실험은 사전 품질 마스크·train/test split·관측 코너를 그대로 고정하고 최종 최적화 loss만 `soft_l1`과 `linear`로 바꿨다. 따라서 차이는 최적화 중 soft weighting 효과이며, hard rejection threshold 변화 효과가 아니다.

- Train: 176 observations, 5351 corners, SHA-256 `e7cda60126f7c3be5330a4d8633a722bbf56dd942063dc3b671e190d98a1e6f1`
- Held-out: 40 observations, 1131 corners, SHA-256 `a5c58b78ff6f083d28875116f89e585d43d02ac7d845494febfa91cac091f37c`

| Method | Held-out soft-L1 px | Held-out linear px | soft-L1 change | Fixed→Fixed cube soft/linear px | Gripper→Fixed cube soft/linear px |
| --- | ---: | ---: | ---: | ---: | ---: |
| A0 | 5.8773 | 5.7282 | +2.60% | 11.4709 / 11.5087 | 9.4778 / 9.7547 |
| A1 | 6.3602 | 6.1152 | +4.01% | 10.7893 / 9.5986 | 9.7098 / 9.2676 |
| A2 | 6.2143 | 6.0791 | +2.22% | 9.6100 / 8.9306 | 9.2498 / 8.8773 |
| A3 | 5.4968 | 5.4487 | +0.88% | 7.1163 / 6.5803 | 8.6882 / 8.4244 |
| A4 | 6.1904 | 6.0320 | +2.63% | 9.5791 / 8.9832 | 9.2592 / 8.8871 |
| B1 | 6.3309 | 6.0668 | +4.35% | 10.7375 / 9.6838 | 9.7038 / 9.2613 |
| B2 | 6.6975 | 6.7745 | -1.14% | 6.9340 / 9.0353 | 8.0188 / 9.3986 |
| B3 | 5.8772 | 5.7282 | +2.60% | 11.4711 / 11.5083 | 9.4786 / 9.7546 |

해석: soft-L1은 A0/A3의 held-out 전체 오차를 약 10% 줄였고, A4에서는 약 2% 줄였다. 그러나 B2 held-out 전체는 오히려 커졌다. 따라서 robust loss가 모든 조건을 일괄 개선한다고 주장할 수 없으며, 방법별·표적별 결과를 함께 보고해야 한다.

한계: 이 결과는 이미 적용된 사전 PnP 품질 마스크를 고정한 실험이다. 프레임/관측 hard rejection 자체의 민감도는 threshold를 사전 등록한 별도 실험으로 확인해야 한다.
