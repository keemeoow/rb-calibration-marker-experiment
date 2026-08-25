# 멀티카메라 Calibration Pipeline 발표 대본 — 3~5분

## 도입

저희의 목표는 robot base 기준 fixed camera의 pose $\mathbf T_{B,C_i}$와, flange 기준 gripper camera의 장착 pose
$\mathbf T_{F,C_g}$를 구하는 것입니다. 전체 과정은 먼저 각 이미지에서 target pose를 측정하고, 여러 카메라를
단계적으로 연결한 다음, 마지막에 전체 관계를 동시에 최적화하는 순서로 진행됩니다.

## Step 1 — Camera 파라미터 준비

먼저 각 카메라의 RGB 내부계수와 렌즈 왜곡계수를 준비합니다. 이 값은 3D 점을 2D pixel로 투영하는 PnP와
reprojection에 사용됩니다. RealSense 내부의 depth-to-color 외부계수도 저장하는데, 이것은 robot과 camera 사이
관계가 아니라 aligned depth를 만들기 위한 장치 내부 값입니다.

## Step 2 — 동기 촬영과 online gate

여러 카메라의 RGB와 aligned depth, 촬영 시각, 그리고 당시 base-to-flange pose를 하나의 event로 저장합니다.
저장 전 marker 검출, 임시 PnP, reprojection error, depth 품질과 촬영 시차를 검사하고, 명백히 불량한 event는
제외합니다.

## Step 3-A — 개별 PnP와 reprojection

각 camera와 event를 먼저 독립적으로 처리합니다. Target model에 미리 정의된 3D corner와 RGB에서 검출한 2D
pixel, camera 내부계수를 PnP에 넣어 camera 기준 cube 또는 board pose를 구합니다.

$\mathbf T_{C_i,O}^{e}$는 event $e$에서 camera $i$가 본 cube의 3D pose입니다. PnP 이후에는 이 pose로 corner를
영상에 다시 투영하고 실제 pixel과 비교합니다. 이 reprojection error는 새로운 pose를 만드는 값이 아니라, 개별
PnP pose가 자기 이미지에 얼마나 잘 맞는지를 나타내는 품질값입니다. 실패한 관측은 제외하고, 후보가 여러 개면 더
일관적인 pose를 선택합니다.

## Step 3-B — Fixed camera 연결

같은 event에서 같은 cube를 본 fixed camera들의 PnP 결과를 연결하면 camera 사이 상대 pose를 구할 수 있습니다.
이를 여러 event에서 반복하고, 크게 튀는 outlier는 제외한 뒤 대표 상대 pose 하나로 통합합니다. 이 단계가 끝나면
fixed camera끼리의 배치는 알지만, 아직 robot base에서 어디에 있는지는 모릅니다.

## Step 3-C — Gripper camera hand–eye

여러 event의 base-to-flange pose와 gripper camera의 target PnP pose를 이용해 hand–eye를 풉니다. 그 결과 flange와
gripper camera 사이의 고정된 장착 pose $\mathbf T_{F,C_g}$ 하나를 얻습니다. 다른 event들과 크게 어긋나는 pose는
제외하고, 전체 robot motion에서 가장 일관적인 결과를 선택합니다.

## Step 3-D — Robot base 등록과 단계적 보정

이제 gripper camera 경로와 fixed camera의 공통 cube 또는 board 관측을 연결해 fixed camera를 robot base 좌표계에
등록합니다. 먼저 같은 set의 cube가 카메라마다 같은 base pose로 계산되는지 비교하여 3D pose 수준에서 camera
pose를 보정합니다.

그다음 이 결과를 초기값으로 사용해 원본 2D marker corner의 reprojection error가 작아지도록 camera pose를 다시
정밀화합니다. 큰 corner error는 robust loss로 영향력을 낮추고, 최적화 후 오차가 개선되지 않거나 camera pose가
과도하게 변하면 보정 결과를 버리고 이전 값을 유지합니다.

## Step 3-E — Fixed-FK 전체 동시 최적화

마지막으로 fixed camera, gripper camera, 전체 event와 set의 관계를 하나의 문제에서 동시에 최적화합니다. 발표에서
사용하는 Fixed-FK 방식은 set별 raw FK cube pose $\mathbf T_{B,O}^{s,\mathrm{FK}}$가 정확하다고 가정하고, 이 값을
움직이지 않는 기준으로 고정합니다.

Fixed camera와 gripper camera가 계산한 cube pose가 모두 이 FK cube pose에 맞도록 모든 $\mathbf T_{B,C_i}$와
$\mathbf T_{F,C_g}$를 함께 조정합니다. 여기서 joint optimization은 robot joint가 아니라 여러 calibration 변수를
공동으로 푼다는 뜻입니다. 전체 오차가 개선되고 pose 변화가 타당할 때만 새 결과를 채택합니다.

## Step 4·5 — 검증과 출력

최종적으로 여러 camera가 같은 cube pose를 만드는지, hand–eye가 robot motion과 일관적인지, reprojection과 depth
검증을 통과하는지 확인합니다. 이 단계에서는 transform을 다시 수정하지 않고 정확도를 평가합니다. 검증된
base-to-fixed-camera pose와 flange-to-gripper-camera pose를 최종 결과로 출력합니다.

## 마무리

정리하면, 개별 이미지에서는 PnP로 target pose를 측정하고 reprojection으로 품질을 판단합니다. 이후 fixed camera와
gripper camera를 robot base 좌표계로 연결하고, 3D pose와 2D pixel 오차를 순서대로 줄입니다. 마지막에는 raw FK
cube pose를 고정 기준으로 전체 calibration 행렬을 동시에 최적화하고, 실제로 개선된 결과만 최종값으로 사용합니다.
