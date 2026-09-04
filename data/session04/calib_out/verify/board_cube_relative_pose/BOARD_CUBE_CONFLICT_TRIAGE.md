# Board-Cube Conflict Immediate Triage

“현재 데이터로는 단일 프레임/solver/config 오류는 배제했고, 남은 큰 오차는 cube sparse observation과 target-dependent localization/intrinsic bias로 좁혔습니다. 그래서 공식 결과는 joint reprojection 기반 A2로 유지하고, 다음 촬영에서 intrinsic coverage와 board/cube 동시 관측 조건을 강화해 원인을 확정하겠습니다.”

> **판정:** 지금 데이터만으로 원인을 더 좁힐 수는 있다. 새로 발견된 단순 software/config bug는 없고, 남은 문제는 `cube sparse observation + target-dependent effective scale/localization bias + 제한된 intrinsic coverage`가 섞인 체계 오차로 보는 것이 가장 안전하다.

## 지금 바로 확인한 결론

| 질문 | 결과 | 해석 |
| --- | --- | --- |
| 재실행하면 같은가? | `10.8077 mm` / max `0.5270°` | stale 산출물이 아니라 현재 checkout에서도 재현됨 |
| 한 프레임 문제인가? | event 54 제거 시 `9.599 mm` | 가장 영향이 크지만 conflict가 남아서 단일 프레임 문제는 아님 |
| solver 문제인가? | PnP scan 최선도 `10.692 mm`, stereoCalibrate도 `12.928 mm` | solver 교체로 해결되지 않음 |
| 관측 하한을 올리면? | `8.973 mm`까지 감소하지만 pair당 후보 1개 수준 | 공식 필터로 쓰기에는 support가 너무 약함 |
| 지금 바꿀 수 있는가? | Board scale/factory K/D로 숫자는 낮아지지만 다른 증거와 충돌 | 공식 결과는 변경하지 않는 것이 맞음 |

## Relative Candidate Dispersion

Board 후보는 매우 안정적이고, cube 후보는 특히 `cam0-cam3`에서 크게 흔들린다.

| Target | Pair | Candidates | Translation std | Worst event | Worst deviation | Corner support |
| --- | --- | ---: | ---: | ---: | ---: | --- |
| board | cam0-cam1 | 6 / 5 | 0.199 mm | E0066 | 2.577 mm | 46/13 corners |
| board | cam0-cam3 | 3 / 3 | 0.127 mm | E0066 | 1.104 mm | 46/19 corners |
| cube | cam0-cam1 | 6 / 6 | 0.745 mm | E0042 | 4.651 mm | 12/12 corners |
| cube | cam0-cam3 | 6 / 6 | 4.399 mm | E0054 | 17.131 mm | 8/8 corners |

## Leave-One Event Sensitivity

| Drop event | Conflict RMSE | Improvement | cam1 | cam3 |
| ---: | ---: | ---: | ---: | ---: |
| E0054 | 9.599 mm | 1.209 mm | 12.146 mm | 6.062 mm |
| E0066 | 10.704 mm | 0.104 mm | 13.194 mm | 7.422 mm |
| E0042 | 10.900 mm | -0.092 mm | 12.673 mm | 8.775 mm |
| E0072 | 10.916 mm | -0.109 mm | 13.330 mm | 7.788 mm |
| E0036 | 11.116 mm | -0.308 mm | 13.012 mm | 8.823 mm |
| E0030 | 11.484 mm | -0.676 mm | 13.911 mm | 8.381 mm |

## Cube Quality Filter Sensitivity

| Filter | Cube train obs | Conflict RMSE | Candidate support | 판정 |
| --- | ---: | ---: | --- | --- |
| base | 59 | 10.808 mm | cam1 6/6, cam3 6/6 | 충분히 사라지지 않음 |
| cube >=12 corners | 45 | 10.207 mm | cam1 2/2, cam3 2/2 | support 부족 |
| cube >=3 faces | 25 | 10.207 mm | cam1 2/2, cam3 2/2 | support 부족 |
| cube has +Z face | 50 | 10.002 mm | cam1 3/3, cam3 2/2 | support 부족 |
| cube >=12 corners & >=3 faces | 25 | 10.207 mm | cam1 2/2, cam3 2/2 | support 부족 |
| cube >=12 corners & +Z face | 45 | 10.207 mm | cam1 2/2, cam3 2/2 | support 부족 |

## Same-Support Check

Board/cube가 다른 event support를 쓰는 것이 1차 원인인지 확인했다.

| Pair 기준 | Common events | Conflict RMSE | 해석 |
| --- | --- | ---: | --- |
| cam0-cam1 | E0030, E0036, E0042, E0054, E0066, E0072 | 10.808 mm | 같은 event로 제한해도 해결되지 않음 |
| cam0-cam3 | E0054, E0066, E0072 | 13.026 mm | 같은 event로 제한해도 해결되지 않음 |

## 현재 처리 방침

1. 공식 Table 1 / 발표 결론은 그대로 `A2 = internal main`으로 둔다.
2. `10.8077 mm`는 최종 정확도가 아니라 Board-only PnP와 Cube-only PnP의 target-dependent diagnostic conflict로만 말한다.
3. cube 관측을 더 세게 자르거나 factory K/D로 바꾸면 일부 숫자는 줄지만, support 또는 held-out cube transfer가 나빠져 공식 해결책으로 쓰지 않는다.
4. 다음 실험은 새 intrinsic coverage와 Track A 반복촬영에서 같은 board/cube를 다양한 거리·화면 위치로 다시 찍어 target-dependent scale/localization bias를 분리한다.
