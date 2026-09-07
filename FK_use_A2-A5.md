# FK 사용 방식 정리: A2-A5 최종 기준

이 문서는 Table 1의 A2, A3, A4, A5가 FK와 gripper-mounted target pose를
어떻게 다르게 쓰는지 정리한다. 최종 비교표는
[CALIBRATION_EXPERIMENT_VALIDATION.md](CALIBRATION_EXPERIMENT_VALIDATION.md)를
단일 기준으로 따른다.

핵심 결론:

> **A5는 더 이상 무조건 사후 진단으로 배제하지 않는다.**
> External GT 공개 전에 방법, 파라미터, train-only alignment artifact를 frozen하면
> A5는 최종 후보 method로 비교할 수 있다. 최종 채택은 External cube GT 결과로 정한다.

## 한 줄 요약

| Row | 한 줄 정의 | 최종 역할 |
| --- | --- | --- |
| A2 | board+cube pose를 모두 영상 reprojection으로 free하게 추정하는 unified visual-only 방법 | vision-only unified 후보 |
| A3 | cube pose를 raw FK로 hard fixed하고 나머지를 visual reprojection으로 푸는 방법 | raw FK hard constraint 후보 |
| A4 | cube pose는 free로 두고, vision-aligned FK를 soft factor/prior로 추가하는 방법 | corrected-FK soft 후보 |
| A5 | vision-aligned FK cube pose를 hard fixed하고 visual reprojection만 푸는 방법 | GT 전 frozen 시 최종 후보 |

## 공통 구조

- 네 방법 모두 `board+cube` marker set을 사용한다.
- 네 방법 모두 `unified_joint_optimization` 구조다.
- camera-to-camera residual을 직접 objective에 넣지 않는다.
- 카메라는 shared target pose variable을 통해 결합된다.
- intrinsics `K/D`는 고정값이다.
- 최종 heldout 평가는 항상 `cube`만 사용한다.

## 방법별 정확한 정의

| Row | Cube pose source | Cube pose in optimizer | Objective term | FK residual |
| --- | --- | --- | --- | --- |
| A2 | visual initialization 후 영상 residual로 추정 | free | robust visual reprojection 1항 | 없음 |
| A3 | controller raw FK + mechanical frame map | hard fixed | robust visual reprojection 1항 | 없음 |
| A4 | train-only vision-aligned FK artifact | free | robust visual reprojection + whitened robust FK factor 2항 | 있음 |
| A5 | train-only vision-aligned FK artifact | hard fixed | robust visual reprojection 1항 | 없음 |

## 현재 Session04 내부 cube 결과

아래 값은 External GT 전 내부 참고값이다. 최종 물리 순위는 아니다.

| Row | Train overall px | Heldout Cube px | Cross-view Cube px | Cam-common Cube mm/deg | 해석 |
| --- | ---: | ---: | ---: | ---: | --- |
| A2 | 3.7421 | 3.5958 | 6.3003 | 7.2868 / 1.0280 | vision-only unified 후보 |
| A3 | 5.1587 | 6.3959 | 6.8626 | 8.3382 / 2.0892 | raw FK hard fixed는 현재 내부 cube에서 악화 |
| A4 | 3.7441 | 3.5805 | 6.3126 | 7.3077 / 1.0151 | A2와 거의 동률인 soft-FK 후보 |
| A5 | 3.9648 | 3.2274 | 5.7166 | 6.6745 / 0.8862 | 현재 내부 cube 지표 최저, GT 전 frozen 필요 |

## A5를 채택할 수 있는가?

가능하다. 단 조건이 있다.

1. `Delta_train` 추정 절차가 External GT 공개 전에 고정되어야 한다.
2. A5의 hyperparameter, artifact hash, 사용 train observation list가 고정되어야 한다.
3. External GT cube pose list, failure 기준, tolerance를 GT 확인 전에 고정해야 한다.
4. GT를 본 뒤 A5 정의를 바꾸면 그때는 최종 method가 아니라 사후 진단이다.

따라서 현재 입장은 다음과 같다.

- 내부 cube 지표 기준으로는 A5가 가장 좋아 보인다.
- 그래서 A5를 버리지 않는다.
- 그러나 A5가 최종 제안 방법인지 여부는 External cube GT로 확정한다.
- External GT에서도 A5가 TRE/rotation/P95/failure에서 가장 좋으면, A5를 최종 방법으로 채택할 수 있다.

## 발표용 결론 문장

> 현재 내부 cube 지표에서는 A5가 가장 낮습니다. 그래서 A5를 배제하지 않고,
> External GT 공개 전에 train-only alignment artifact와 평가 코드를 frozen한 최종 후보로 둡니다.
> 최종 제안 방법은 Independent External cube GT의 Translation Error, Rotation Error,
> P95, Failure Rate 결과로 결정하겠습니다.
