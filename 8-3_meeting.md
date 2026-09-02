# 8/3 미팅 피드백 반영 현황 (2026-09-02 확인)

원본: [`8-3_meeting.txt`](8-3_meeting.txt) (1,584줄). 발언 시각 기준으로 피드백 19건을
추출하고 현재 코드와 Session04 산출물에서 직접 확인했다. 상태는 **코드와 실행 계약 기준**이며,
2026-09-02에 새 frame-prune 코드로 Session04 canonical 결과까지 다시 생성했다.

**요약: ✅ 반영 11건 · ⚠️ 부분 반영 7건 · ❌ 미반영 1건**

> **현재 실행 제약:** 실제 로봇 사용과 신규 촬영은 불가하다. 따라서 #6 Track A와 #14–#16은
> 코드·스키마·분석기 준비까지만 진행하고 실험 완료로 표시하지 않는다. 기존 Session04 데이터로
> 가능한 재실행/진단만 수행한다. Session04에는 aligned depth가 있지만, 이는 독립 물리 GT가
> 아니므로 robot task 정확도의 대체 증거로 사용하지 않는다.
>
> **현재 마일스톤:** 캘리브레이션 파이프라인 완성까지를 현재 범위로 한다. COLMAP/MATLAB
> external baseline, point cloud, robot task, 신규 촬영 및 외부 GT는 모두 후속 단계로 미룬다.

| # | 시각 | 피드백 | 상태 | 근거 / 남은 것 |
| ---: | --- | --- | :---: | --- |
| 1 | 00:27 | 이상치는 **이미지 레벨로 프레임을 지워라** | ✅ | [`select_frame_prune_subset`](calibration_pipeline/reprojection.py#L592)이 같은 `(event_id, camera_id)`의 board/cube 관측 전체를 한 frame으로 묶어 MAD 기준으로 최대 30% 제거한다. 이 코드로 Session04 canonical 결과를 재생성했다. |
| 2 | 02:16–04:21 | Robust loss로 영향만 줄이는 것과 제거 후 **extrinsic을 다시 푸는 것**은 다르다 | ✅ | [`run_frame_prune_refit`](calibration_pipeline/table1.py#L575)이 `fit → frame-prune → refit → rollback`을 수행한다. 제거 전 train 전체의 동일 robust objective가 개선될 때만 refit을 채택하며 held-out은 선택·재적합·판정에 쓰지 않는다. Canonical 42개 solver stage 중 15개가 prune/refit을 실행했고, 모두 full-train cost가 증가해 1차 결과로 정상 rollback됐다. |
| 3 | 06:18–10:17 | 3-2에서 reprojection error를 쓰는가? 카메라 간 관계도 목적함수에 넣는가? | ✅ | 메인 optimizer의 camera-to-camera residual은 0개이고, 카메라는 공유 target pose 변수로 결합된다. visual-only 행은 1항, FK-factor 행은 visual+FK 2항이다. [`TABLE1_RESULTS.md`](CP_result/session04/late_table1/TABLE1_RESULTS.md#code-consistency-audit-코드-일치성-검증) |
| 4 | 26:44 | 카메라가 많으면 관측 수가 많아 **FK 항이 묻힌다** | ✅ | Objective Block Diagnostics에 visual/FK residual 수와 cost를 분리했다. 기존 결과의 FK cost fraction은 A4 `0.122%`, B1 `0.145%`, B2 `1.139%`이며 이를 parameter 영향력 비율로 해석하지 말라는 제한도 기록했다. |
| 5 | 33:20 | FK 경로에 유리한 지표만 보고하는 것처럼 보이면 안 된다 | ✅ | FK-free Fixed-to-Fixed, FK-dependent Gripper-to-Fixed, 독립 OpenCV relative-pose baseline을 분리해 보고한다. 결과는 혼합적이다. 예를 들어 A2 Fixed-to-Fixed cube는 `3.999 px`지만 OpenCV Cube-only baseline은 `2.882 px`라서 “우리 방법이 41% 우수”라고 결론 내릴 수 없다. |
| 6 | 36:37–38:18 | **전체 수치가 너무 크다**(약 10 mm / 10 px). 원인을 찾아라 | ⚠️ | Cube geometry/corner ordering/refinement 오류를 수정해 direct-PnP target conflict를 `17.299 → 10.808 mm`로 낮췄다. PnP solver만 바꿔도 해결되지 않았고, 현재 진단은 target-dependent corner localization bias를 지목한다. 확정에는 Track A 반복 촬영이 필요하다. [`BOARD_CUBE_RELATIVE_POSE.md`](data/session04/calib_out/verify/board_cube_relative_pose/BOARD_CUBE_RELATIVE_POSE.md) |
| 7 | 38:18, 1:47:22–1:55:53 | 같은 사진을 **공개 구현**에도 넣어 데이터 문제인지 코드 문제인지 확인하라 | ⚠️ | OpenCV PnP 기반 독립 fixed-camera 기준선까지 구현·실행했다. Full MATLAB multiview/COLMAP 대조는 현재 캘리브레이션 완성 범위에서 제외하고 **후속 단계**로 연기한다. [`OPENCV_RELATIVE_BASELINE.md`](CP_result/session04/opencv_relative_baseline/OPENCV_RELATIVE_BASELINE.md) |
| 8 | 43:32–47:33 | **mm 지표를 왜 뺐나.** 절대값은 아니어도 이 세팅에서는 의미 있다 | ✅ | Fixed-to-Fixed와 Gripper-to-Fixed 모두 pixel/translation mm/rotation deg를 함께 보고하며 외부 GT 정확도와 구분한다. |
| 9 | 44:26 | RGB-D depth를 캘리브레이션 계산에 쓰는가? | ✅ | 최종 Table 1 solver는 RGB의 3D-object/2D-pixel 대응과 고정 intrinsic만 사용한다. 센서 depth는 목적함수에 들어가지 않는다. |
| 10 | 55:32–56:01 | Train/test가 같은 이미지의 코너를 나눠 가지면 안 된다 | ✅ | 전사에서 event가 이미 분리됐음을 확인한 항목이다. 현재도 event-grouped, set-stratified split이며 같은 event의 모든 카메라·코너는 한쪽에만 속한다. [`schema.py`](calibration_pipeline/schema.py#L416) |
| 11 | 1:01:06–1:03:31 | 마커 구성이 다른 행끼리 직접 비교하지 말고 공통 지표를 써라 | ✅ | Own-heldout은 동일 marker population끼리만 비교하고, 공통 frozen evaluation mask와 두 camera scope를 별도로 보고한다. |
| 12 | 1:11:11–1:15:46 | 고정카메라만 보는 공통 지표 외에 **손목 카메라 포함 지표**도 필요하다 | ✅ | Fixed-to-Fixed와 Gripper-to-Fixed를 모두 구현했고 camera-scope 검증을 통과한다. |
| 13 | 1:33:26–1:34:18 | FK 없는 “독립” 방식을 굳이 기여로 둘 필요가 있는가 | ⚠️ | A1은 제안 방법이 아니라 A2와 동일 cube+board 관측에서 Sequential→Unified 효과만 분리하는 ablation으로 유지 중이다. 교수님도 “해도 되지만 굳이 기여는 아니다”라는 취지였으므로 삭제보다 **기여 방법으로 부르지 않는 것**이 현재 판단이다. 논문 지면에서 행을 뺄지는 최종 표 편집 결정이다. |
| 14 | 1:36:51–1:37:24 | 핵심은 카메라 간 오차보다 **로봇 그리퍼의 작업 정확도**다 | ⚠️ | Blind prediction·외부 GT 채점 코드와 Track C 계약은 준비됐지만, 독립 GT/재파지/작업 데이터가 없어 핵심 실험은 아직 실행하지 못했다. [`CAPTURE_CAMPAIGN_PROTOCOL.md`](protocol_templates/CAPTURE_CAMPAIGN_PROTOCOL.md#4-track-c--외부-gt) |
| 15 | 1:37:24–1:39:01 | **눈금 큐브** 재파지로 x/y/z 오차를 실측하라 | ⚠️ | Track C에 독립 pose/GT 측정 절차는 정의돼 있다. 눈금 큐브 전용 입력 포맷과 실측 데이터는 없으며, 눈금자 translation만으로는 6-DoF 최종 GT가 되지 않는다는 제한을 기록했다. |
| 16 | 1:41:41–1:42:47 | **peg-in-hole 또는 grasp success rate/정밀도**를 평가하라 | ⚠️ | Paired task-trial schema와 evaluator를 구현했다. 모든 방법의 동일 pair 기록을 강제하고 success rate/Wilson 95% CI, XYZ contact error, P95, paired 차이를 출력한다. 실측 robot trial은 아직 없다. [`task_trial.py`](calibration_pipeline/task_trial.py) |
| 17 | 1:42:47–1:44:28 | **point cloud 정합을 로봇 관점**에서 표현하라 | ❌ | 전용 point-cloud/robot-contact 평가 코드와 데이터가 없다. |
| 18 | 1:40:07–1:40:33 | 오버레이로 마커 검출 정확도를 정성 확인하라 | ✅ | Cube 재검출 overlay 도구가 있다. 전사 취지대로 이는 **검출 QA**이며 로봇 작업 정확도 지표로 승격하지 않는다. [`render_cube_redetection_overlays.py`](tools/render_cube_redetection_overlays.py) |
| 19 | 1:47:12 | FK를 어떻게 쓸지, 무엇을 제안 방법으로 둘지 정해야 한다 | ⚠️ | 현재 결정은 **A2=확증 대표행, A4=불확실성-aware 확장 preflight, A5=post-hoc 원인 진단**이다. A4는 Simulation covariance라 A2 대비 우월성을 주장하지 않는다. 최종 물리 순위는 measured FK covariance와 blind external GT 뒤에 확정한다. |

## 남은 작업의 성격과 순서

1. **완료:** 새 frame-prune 코드로 Table 1 전체, cross-target, marker-system, OpenCV reference를 재실행하고 CSV/Markdown/HTML을 동기화했다.
2. **현재 범위:** 캘리브레이션 입력 검증, 최적화, frame-prune/refit/rollback, 평가 산출물과 재현성 검증만 마무리한다.
3. **후속 — external baseline:** #7의 MATLAB multiview/COLMAP adapter와 실행은 현재 급한 작업에서 제외한다. 기존 OpenCV reference까지만 유지한다.
4. **후속 — robot task/GT:** #6 Track A, #14–#16 Track C는 코드 scaffold만 유지하고 신규 촬영이 가능해질 때 재개한다.
5. **후속 — point cloud:** #17의 robot-base point-cloud 진단도 현재 캘리브레이션 범위가 완성된 뒤 진행한다.

## 판정

기존 문서는 frame-prune 구현 전 상태와 현재 저장소를 섞어 기록했고, OpenCV baseline과 external-GT
scaffold를 누락했으며, 공개 baseline 대비 우위도 잘못 해석했다. 위 표가 현재 코드 기준의 수정된
tracking이다. 가장 큰 미완료 항목은 여전히 **독립 물리 GT를 사용한 robot task 정확도 실험**이다.
