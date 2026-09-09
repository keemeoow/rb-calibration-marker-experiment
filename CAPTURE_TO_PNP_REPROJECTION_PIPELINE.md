# 촬영부터 최종 Calibration까지의 전체 데이터 흐름

이 문서는 현재 저장소의 실제 파이프라인을 발표에서 설명하기 위한 수준으로 정리한 문서다. 세부 옵션이나
threshold보다 다음 네 가지에 집중한다.

발표의 FK 방식은 **Fixed-FK**를 기준으로 한다. 즉, set별 raw FK cube pose를 정답으로 가정해 그대로 고정하며,
vision으로 FK를 보정하는 Corrected-FK 방식은 이 문서의 범위에 포함하지 않는다.

1. 어떤 범위의 데이터를 사용하는가: 개별 camera·개별 event인가, 여러 camera·여러 event인가
2. 어떤 알고리즘을 사용하는가
3. 입력으로 무엇을 받아 무엇을 출력하는가
4. 오차가 있는 데이터를 버리는가, 선택·가중하는가, 최적화로 보정하는가, 검증만 하는가

---

## 0. 데이터 단위와 전체 실행 순서

| 단위 | 기호 | 의미 |
|---|---|---|
| Camera | $i$ | 물리적인 camera 한 대 |
| Event | $e$ | 같은 촬영 명령에서 얻은 여러 camera frame과 robot 상태의 묶음 |
| Set | $s$ | 같은 cube placement 또는 grasp 조건에 속하는 event 묶음 |
| Session | 없음 | calibration에 사용하는 전체 camera×set×event 데이터 |

### 기호를 읽는 방법

#### 좌표계 기호

| 기호 | 의미 |
|---|---|
| $B$ | robot base 좌표계 |
| $F$ | robot flange 좌표계 |
| $C_i$ | fixed camera $i$의 좌표계 |
| $C_g$ | gripper camera 좌표계 |
| $O$ | cube 좌표계 |
| $W$ | 고정 ChArUco board 좌표계 |

#### 첨자와 윗첨자

| 기호 | 의미 |
|---|---|
| $i$ | camera 번호 |
| $e$ | 촬영 event 번호 |
| $s$ | set 번호 |
| $k$ | target에 정의된 corner 번호 |
| $N$ | 한 번의 PnP에 사용한 대응 corner의 총개수 |

#### 자주 사용하는 값

| 기호 | 의미 |
|---|---|
| $\mathbf T_{A,B}$ | 좌표계 $B$의 점을 좌표계 $A$로 옮기는 $4\times4$ pose 행렬 |
| $\mathbf K_i$ | camera $i$의 RGB 내부계수 행렬 |
| $\mathbf D_i$ | camera $i$의 RGB 렌즈 왜곡계수 |
| $\mathbf X_O^k$ | cube 모델에서 미리 정의한 $k$번째 corner의 3D 위치 |
| $\mathbf X_W^k$ | board 모델에서 미리 정의한 $k$번째 corner의 3D 위치 |
| $\mathbf u_{i,e}^k$ | camera $i$의 event $e$ RGB 영상에서 검출한 $k$번째 corner의 2D pixel 위치 |
| $\widehat{\mathbf u}_{i,e}^k$ | 3D corner와 pose로 계산한 예측 2D pixel 위치 |
| $\pi(\cdot)$ | camera 좌표계의 3D 점을 RGB 영상의 2D pixel로 투영하는 함수 |
| $\mathbf r_{i,e}^k$ | 예측 pixel과 실제 검출 pixel의 차이인 reprojection residual |
| $E_{i,e}$ | camera $i$의 event $e$ PnP 관측을 대표하는 평균 pixel error |

$\mathbf X_O^k$와 $\mathbf X_W^k$는 camera나 depth가 측정해 주는 값이 아니다. Cube 크기, marker 부착 위치,
board 규격을 이용해 calibration 전에 target 모델에 미리 저장해 둔 3D 좌표다.

```text
[Step 1] Camera 내부 파라미터 준비
    ↓
[Step 2] RGB + aligned depth + flange pose 동기 촬영 및 온라인 품질 검사
    ↓
[Step 3-A] 각 camera×event에서 PnP로 cube/board pose 측정
    ↓
[Step 3-B] 여러 fixed camera를 서로 연결
    ↓
[Step 3-C] 여러 event로 flange와 gripper camera 사이 hand-eye 계산
    ↓
[Step 3-D] fixed camera들을 robot base 좌표계에 등록하고 단계적으로 보정
    ↓
[Step 3-E] 전체 camera·event·set의 관계를 동시에 최적화
    ↓
[Step 4] 최종 결과 검증
    ↓
[Step 5] 보고서와 transform 출력
```

### 이 문서에서 말하는 오류 처리

| 처리 | 의미 |
|---|---|
| 제외 | 신뢰할 수 없는 관측을 이후 계산에 사용하지 않음 |
| 선택·가중 | 여러 측정 중 더 일관된 값을 선택하거나 불확실한 값의 영향력을 낮춤 |
| 최적화 | 여러 관계가 동시에 잘 맞도록 transform을 수치적으로 수정함 |
| 검증 | 결과를 바꾸지 않고 최종 오차를 측정하여 보고함 |

---

## 1. Step 1 — 카메라 파라미터 준비

### 범위와 출력

Camera별 `intrinsics/cam{i}.npz`에 다음 값이 저장된다.

| 값 | 의미 | 개수 |
|---|---|---:|
| $\mathbf K_i$ | color camera 내부계수 | camera당 하나 |
| $\mathbf D_i$ | color lens distortion | camera당 하나 |
| $\mathbf K_{D,i},\mathbf D_{D,i}$ | depth stream 내부계수·왜곡 | camera당 하나 |
| $s_{\mathrm{depth},i}$ | raw depth unit을 metre로 바꾸는 scale | camera당 하나 |
| $\mathbf R_{C,D},\mathbf t_{C,D}$ | 장치 내부 depth→color 외부계수 | 장치당 하나 |

`Step1b_charuco_intrinsics.py`를 실행하면 color $\mathbf K_i,\mathbf D_i$를 ChArUco 관측으로 보정한다.
Depth 관련 필드와 장치 내부 depth→color 정보는 유지한다.

### 중요한 구분

- $\mathbf T_{C,D}$: RealSense 한 대 내부의 depth sensor와 RGB sensor 관계
- $\mathbf T_{B,C_i}$: Step 3에서 구하는 robot base와 camera 관계

둘은 서로 다른 외부계수다.

---

## 2. Step 2 — 동기 촬영과 온라인 품질 검사

각 event에서 여러 camera의 **RGB·aligned depth**, 촬영 시각, 그리고 robot이 제공하는 **base-to-flange pose**
$\mathbf T_{B,F}^{e}$를 함께 저장한다. Aligned depth는 depth 값을 RGB pixel 좌표에 맞춰 놓은 영상이다.

저장 전에는 marker 검출, 임시 PnP의 reprojection error, depth 품질, camera 간 촬영 시차 등을 검사한다.
기준을 통과하지 못한 event는 저장하지 않는다.

촬영 block은 두 종류다.

- `A_placement`: 놓여 있는 cube와 고정 board를 gripper camera 및 fixed camera가 관측
- `B_eyetohand`: robot flange 쪽에서 cube를 잡아 움직이고 fixed camera들이 관측

---

## 3. Step 3-A — 개별 이미지에서 PnP pose 측정

### 적용 범위

이 단계는 **camera 한 대의 event 한 장**, 즉 각 $(i,e)$ 관측을 독립적으로 처리한다. (여러 camera를 하나의
calibration으로 통합하는게 아님)


### 입력과 알고리즘

| 입력 | 의미 |
|---|---|
| $N\times3$ object points | cube 또는 board 좌표계에서 이미 알고 있는 marker corner의 3D 위치 |
| $N\times2$ image points | RGB 영상에서 검출된 같은 corner의 2D pixel 위치 |
| $\mathbf K_i,\mathbf D_i$ | 해당 RGB camera의 내부계수와 왜곡계수 |

Object points는 target 모델에서 가져오고 image points는 RGB marker 검출에서 얻는다. 두 배열의 같은 행은 같은
실제 corner를 가리킨다. Aligned depth는 이 두 배열을 만드는 기본 입력이 아니며, 뒤에서 PnP 결과의 깊이 일관성을
확인하는 보조 정보로 사용한다.

PnP는 알려진 3D 점들이 관측된 2D pixel에 가장 잘 겹치도록 target의 camera 기준 pose를 계산한다.

$$
\mathbf T_{C_i,O}^{e}
=\operatorname{PnP}
\left(\mathbf X_O,\mathbf u_{i,e},\mathbf K_i,\mathbf D_i\right)
$$

- $O$: cube 좌표계
- $\mathbf T_{C_i,O}^{e}$: event $e$에서 camera $i$가 본 cube의 3차원 pose

위 식은 cube를 예로 든 것이다. 실제로 cube와 board는 다음처럼 기호를 구분한다.

| 검출한 target | PnP 출력 | camera×event당 개수 |
|---|---|---:|
| Cube | $\mathbf T_{C_i,O}^{e}$ | 검출에 성공하면 하나 |
| Board | $\mathbf T_{C_i,W}^{e}$ | 검출에 성공하면 하나 |

한 이미지에서 cube와 board가 모두 검출되면 두 pose가 각각 하나씩 나온다. Cube의 여러 면이나 marker를 함께
사용하더라도 모두 같은 cube 모델에 속하므로 최종 출력은 cube 전체를 나타내는 pose 하나다.

### Reprojection error와 오류 처리

PnP가 계산한 pose로 3D corner를 영상에 다시 투영하고, 실제 검출 pixel과의 거리를 확인한다.

$$
\widehat{\mathbf u}_{i,e}^{k}
=\pi\!\left(\mathbf K_i,\mathbf D_i,
\mathbf T_{C_i,O}^{e}\mathbf X_O^k\right)
$$

$$
\mathbf r_{i,e}^{k}
=\widehat{\mathbf u}_{i,e}^{k}-\mathbf u_{i,e}^{k}
$$

이 오차의 단위는 pixel이며 depth는 이 식에 직접 들어가지 않는다.

Reprojection 계산이 새로운 pose 행렬을 출력하는 것은 아니다. 이 단계의 출력은 corner마다 계산된 pixel residual
$\mathbf r_{i,e}^{k}$와 이를 하나의 품질값으로 요약한 카메라 i 의 한 이벤트 내의 평균 pixel error $E_{i,e}$다.

$$
E_{i,e}
=\frac{1}{N}\sum_{k=1}^{N}
\left\|\mathbf r_{i,e}^{k}\right\|
$$

따라서 PnP 관측 하나에는 **PnP가 구한 pose**와 **그 pose가 자기 이미지에 얼마나 잘 맞는지 나타내는 pixel
error**가 함께 연결된다.

투영식은 다음 순서로 읽는다.

1. Target 모델에서 미리 알고 있는 3D corner $\mathbf X_O^k$를 가져온다.
2. $\mathbf T_{C_i,O}^{e}\mathbf X_O^k$를 계산하여 target 기준 3D corner를 camera 기준 3D corner로 변환한다.
3. 투영 함수 $\pi(\cdot)$가 렌즈 왜곡계수 $\mathbf D_i$와 내부계수 $\mathbf K_i$를 적용한다.
4. 계산으로 예측한 pixel $\widehat{\mathbf u}_{i,e}^k$를 얻는다.
5. 실제 검출 pixel $\mathbf u_{i,e}^k$와 비교하여 reprojection residual $\mathbf r_{i,e}^k$를 얻는다.

$\mathbf T_{C_i,O}^{e}\mathbf X_O^k$에서 얻는 깊이는 PnP pose와 target 모델로 계산된 camera 기준 $Z$값이다.
촬영된 aligned depth 값은 사용하지 않음.

- marker를 찾지 못하거나 PnP가 성립하지 않는 관측: **제외**
- 가능한 pose가 여러 개이면 reprojection·depth·관측 정보가 더 일관된 pose: **선택**
- 신뢰도가 낮지만 사용할 수 있는 관측: 이후 통합 계산에서 **영향력을 낮춤**

여기서 얻은 reprojection error는 **개별 PnP가 자기 이미지에 얼마나 잘 맞는지**를 보는 값이다. 아직 최종
multi-camera calibration의 정확도를 뜻하지 않는다.

### 다음 단계로 전달되는 값

| 값 | 의미 |
|---|---|
| $\mathbf T_{C_i,O}^{e}$ | camera $i$가 event $e$에서 본 cube의 PnP pose |
| $\mathbf r_{i,e}^{k}$ | 각 corner의 2D pixel residual |
| $E_{i,e}$ | 해당 PnP 관측의 대표 pixel error |

다음 단계가 camera 관계를 계산할 때 직접 사용하는 기본 측정값은 $\mathbf T_{C_i,O}^{e}$다. Pixel error
$E_{i,e}$는 그 pose를 제외할지, 여러 후보 중 선택할지, 영향력을 낮출지를 판단하는 품질 정보로 사용한다.

---

## 4. Step 3-B — 여러 fixed camera를 서로 연결

### 적용 범위

같은 event에서 같은 위치의 cube를 본 **여러 fixed camera의 PnP 결과를 통합**한다.

Camera $i$와 $j$가 같은 cube를 보았다면 cube pose를 중간 연결점으로 사용해 camera 사이 상대변환을 계산할 수 있다.

$$
\mathbf T_{C_i,C_j}^{e}
=\mathbf T_{C_i,O}^{e}
\left(\mathbf T_{C_j,O}^{e}\right)^{-1}
$$

### 입력과 출력

| 구분 | 데이터 범위 |
|---|---|
| 입력 | 여러 fixed camera×여러 공통 event의 PnP pose |
| 출력 | fixed camera 사이의 상대변환, camera pair당 하나 |

### 여러 event의 측정을 하나로 합치는 방법

같은 camera pair의 상대변환을 여러 event에서 반복해서 구하면 측정 노이즈 때문에 값이 조금씩 다르게 나온다.
따라서 다음 순서로 camera pair를 대표하는 상대변환 하나를 만든다.

1. 공통 cube를 본 event마다 $\mathbf T_{C_i,C_j}^{e}$를 계산한다.
2. 다른 event들과 크게 다른 측정을 제외한다.
3. 남은 측정들을 튀는 값의 영향을 적게 받는 방식으로 평균내어 camera pair의 상대변환 하나를 얻는다.
4. 여러 camera를 순서대로 연결했을 때 결과가 서로 모순되지 않는지 확인한다.

여기서 3번은 여러 측정을 하나로 만드는 **통합 계산**이고, 2번과 4번의 불일치 검사는 **오류 처리**다.

### 오류 처리만 따로 정리

- 다른 event들과 크게 어긋나는 상대변환: **제외**
- 정보가 약한 단일 면 관측: 사용할 수 있으면 **가중치를 낮춤**
- 여러 camera의 연결 결과와 모순되는 측정: **제외하거나 영향력을 낮춤**

이 단계는 개별 PnP 측정을 그대로 믿는 대신, 여러 camera가 같은 물체를 봤다는 조건을 이용해 서로 모순되는
측정의 영향을 줄인다.

---

## 5. Step 3-C — Flange–gripper camera hand-eye calibration

### 적용 범위

**Gripper camera 한 대의 여러 event**와 각 event의 robot flange pose를 함께 사용한다.

### 입력과 출력

| 구분 | 데이터 |
|---|---|
| 입력 | event별 base-to-flange pose $\mathbf T_{B,F}^{e}$ |
| 입력 | event별 gripper-camera-to-target PnP pose $\mathbf T_{C_g,O}^{e}$ |
| 출력 | flange-to-gripper-camera transform $\mathbf T_{F,C_g}$, session당 하나 |

Robot이 움직이면 flange와 gripper camera가 함께 움직인다. 여러 event의 상대운동이 서로 맞도록 고전적인
hand-eye 관계를 푼다.

$$
\mathbf A_{ab}\mathbf X=\mathbf X\mathbf B_{ab},
\qquad \mathbf X=\mathbf T_{F,C_g}
$$

### 오류 처리

- 다른 event들과 크게 어긋나는 pose: MAD 기반으로 **제외**
- 여러 hand-eye 해법: 전체 event에서 가장 일관된 결과를 **선택**
- 선택한 초기 hand-eye: board와 cube 관계가 더 잘 맞도록 robust loss로 **미세 최적화**
- 최적화 결과가 실제 오차를 줄이지 않으면: 수정값을 채택하지 않고 **초기 결과 유지**

---

## 6. Step 3-D1 — Fixed camera를 robot base에 등록

### 왜 필요한가

Step 3-B까지는 fixed camera들이 서로 어떻게 배치됐는지만 안다. 최종 calibration에는 각 fixed camera가 robot
base 기준으로 어디에 있는지 필요하다.

### 적용 범위와 알고리즘

**Gripper camera의 hand-eye 결과, robot flange pose, target 관측, fixed camera의 상대관계**를 연결한다.

먼저 움직이는 gripper camera의 event별 base pose를 계산한다.

$$
\mathbf T_{B,C_g}^{e}
=\mathbf T_{B,F}^{e}\mathbf T_{F,C_g}
$$

그 camera가 본 target을 통해 target의 base pose를 얻는다.

$$
\mathbf T_{B,O}^{e}
=\mathbf T_{B,C_g}^{e}\mathbf T_{C_g,O}^{e}
$$

같은 target을 본 fixed camera의 pose는 다음 연결로 계산할 수 있다.

$$
\mathbf T_{B,C_i}
=\mathbf T_{B,O}^{e}
\left(\mathbf T_{C_i,O}^{e}\right)^{-1}
$$

### Board 경로와 cube 경로

두 경로는 서로 다른 target을 중간 연결점으로 사용해 같은 $\mathbf T_{B,C_i}$를 계산하는 방법이다.

Board 경로는 gripper camera와 fixed camera가 관측한 board $W$를 이용한다.

$$
\mathbf T_{B,W}^{e}
=\mathbf T_{B,C_g}^{e}\mathbf T_{C_g,W}^{e}
$$

$$
\mathbf T_{B,C_i}^{e,W}
=\mathbf T_{B,W}^{e}
\left(\mathbf T_{C_i,W}^{e}\right)^{-1}
$$

Cube 경로는 두 camera가 관측한 cube $O$를 이용한다.

$$
\mathbf T_{B,C_i}^{e,O}
=\mathbf T_{B,O}^{e}
\left(\mathbf T_{C_i,O}^{e}\right)^{-1}
$$

두 경로의 최종 목적은 모두 같은 fixed-camera pose $\mathbf T_{B,C_i}$다. 하지만 board corner 검출 오차와 cube
corner 검출 오차가 서로 다르므로 실제 계산 결과는 완전히 같지 않을 수 있다.

### 입력과 출력

| 구분 | 데이터 범위 |
|---|---|
| 입력 | gripper camera×여러 event, fixed camera×여러 event, flange pose, hand-eye 결과 |
| 출력 | base-to-fixed-camera transform $\mathbf T_{B,C_i}$, fixed camera당 하나 |

### 오류 처리

#### Event 관측 처리

- PnP가 실패했거나 다른 event의 결과와 크게 어긋나는 관측: 계산에서 **제외**
- 기하적으로 사용할 수 있지만 단일 cube 면만 보였거나 품질이 상대적으로 낮은 관측: 버리지 않고 **영향력을 낮춤**

즉, 명백히 잘못된 값은 제외하고, 불확실하지만 정보가 남아 있는 값은 작은 가중치로 사용한다.


이 단계의 결과는 아직 최종값이 아니라, 뒤의 refinement가 시작할 수 있도록 만드는 물리적으로 해석 가능한 초기
calibration이다.

---

## 7. Step 3-D2 — 같은 set의 pose 일관성 보정

### 적용 범위

이 단계는 **cube가 놓여 있어 같은 set 안에서 pose가 고정됐다고 볼 수 있는 관측**에 적용한다. 따라서 해당 set의
cube는 어느 fixed camera로 계산하더라도 robot base 좌표계에서 같은 위치와 방향으로 나와야 한다. Robot이 잡고
움직이는 cube 관측은 현재 기본 fixed-set residual에서는 제외한다.

Camera $i$의 event $e$ 관측으로 계산한 cube pose는 다음과 같다.

$$
\mathbf T_{B,O}^{i,e}
=\mathbf T_{B,C_i}\mathbf T_{C_i,O}^{e}
$$

- $\mathbf T_{B,C_i}$: Step 3-D1까지 구한 fixed-camera calibration
- $\mathbf T_{C_i,O}^{e}$: Step 3-A에서 PnP로 측정한 camera 기준 cube pose
- $\mathbf T_{B,O}^{i,e}$: 두 값을 연결해 계산한 robot-base 기준 cube pose

### Fixed-FK cube pose와 set anchor

Step 2에는 robot FK와 cube-center tool offset으로 계산된 set별 base-to-cube pose가 저장된다.

$$
\mathbf T_{B,O}^{s,\mathrm{FK}}
$$

발표에서 사용하는 **Fixed-FK** 방식은 이 raw FK cube pose를 정답으로 가정하고 set anchor로 그대로 고정한다.
Vision으로 FK를 정렬하거나 보정하지 않으며, 별도의 보정변환 $\boldsymbol\Delta$도 사용하지 않는다.

### 알고리즘과 오류 처리

각 camera가 계산한 $\mathbf T_{B,O}^{i,e}$를 Fixed-FK 기준값 $\mathbf T_{B,O}^{s,\mathrm{FK}}$와 비교한다.

예를 들어 같은 cube가 camera 1에서는 기준보다 오른쪽으로 계산되고 camera 2에서는 기준 위치에 계산된다면,
camera 1의 $\mathbf T_{B,C_1}$에 위치 편향이 남아 있다고 볼 수 있다. 이 반복적인 차이가 줄도록 fixed-camera
transform의 보정량을 계산해 적용한다.

---

## 8. Step 3-D3 — Raw-corner reprojection 보정

### 적용 범위

Fixed camera별 여러 event의 **원본 2D marker corner 전체**를 다시 사용한다.

### 알고리즘

Step 3-D2에서 얻은 fixed-camera pose $\mathbf T_{B,C_i}$를 최적화의 초기값으로 사용한다. D2가 여러 camera의
3D pose 일관성을 이용한 보정이라면, D3는 그 결과를 이어받아 원본 2D corner의 reprojection residual을 직접
최소화하는 camera별 비선형 최적화다.

Camera pose와 object pose는 예측 pixel 자체가 아니라, 예측 pixel을 계산하는 데 사용되는 3D pose다.

| 값 | 이 단계에서의 역할 |
|---|---|
| $\mathbf T_{B,C_i}$ | 최적화 변수. Step 3-D2 결과를 초기값으로 사용한다. |
| $\mathbf T_{B,O}^{e}$ | event별 object pose. 이 단계에서는 고정한다. |
| $\mathbf X_O^k$ | object 모델에 미리 정의된 3D corner |
| $\widehat{\mathbf u}_{i,e}^k$ | 현재 camera pose로 계산되는 예측 pixel |
| $\mathbf u_{i,e}^k$ | RGB 영상에서 실제로 검출된 pixel |

먼저 object의 3D corner를 현재 camera pose를 이용해 camera 좌표계로 변환한다.

$$
\mathbf X_{C_i}^{e,k}
=\left(\mathbf T_{B,C_i}\right)^{-1}
\mathbf T_{B,O}^{e}\mathbf X_O^k
$$

변환된 3D corner에 camera 내부계수와 왜곡 모델을 적용하면 예측 pixel이 계산된다.

$$
\widehat{\mathbf u}_{i,e}^{k}
=\pi\!\left(\mathbf K_i,\mathbf D_i,
\mathbf X_{C_i}^{e,k}\right)
$$

최적화 과정에서는 $\mathbf T_{B,O}^{e}$와 실제 pixel $\mathbf u_{i,e}^k$는 고정하고,
$\mathbf T_{B,C_i}$만 갱신한다. Camera pose가 갱신될 때마다 예측 pixel $\widehat{\mathbf u}_{i,e}^k$를 다시
계산하며, 모든 event와 corner의 reprojection residual 합이 가장 작아지는 camera pose를 찾는다.

$$
\min_{\mathbf T_{B,C_i}}
\sum_{e,k}
\rho\!\left(
\left\|
\widehat{\mathbf u}_{i,e}^{k}-\mathbf u_{i,e}^{k}
\right\|^2
\right)
$$

$\rho$는 큰 pixel 오차 한두 개가 전체 결과를 끌고 가지 못하게 하는 robust loss다.

### 입력과 출력

| 구분 | 데이터 범위 |
|---|---|
| 시작값 | Step 3-D2까지 얻은 fixed-camera pose $\mathbf T_{B,C_i}$ |
| 고정 입력 | event별 object pose $\mathbf T_{B,O}^{e}$, 3D 모델점, 실제 검출 2D corner, $\mathbf K_i,\mathbf D_i$ |
| 출력 | pixel 관측에 맞게 보정된 $\mathbf T_{B,C_i}$ |

### 오류 처리

- 작은·중간 오차: camera transform을 움직여 **최적화로 보정**
- 큰 corner 오차: robust loss로 **영향력을 낮춤**
- pixel RMSE가 좋아지지 않거나 camera pose가 비정상적으로 크게 변함: **최적화 결과 거부**, 이전 결과 유지

Step 3-A의 reprojection은 PnP pose의 자기 적합도를 검사했다. 이 단계의 reprojection은 이미 연결된 base·camera·object
관계를 사용해 **최종 fixed-camera calibration 자체를 보정**한다는 차이가 있다.

---

## 9. Step 3-E — 전체 system joint optimization

### 적용 범위

이 단계에서 처음으로 **전체 fixed camera, gripper camera, 전체 event와 set의 관계를 하나의 문제로 동시에** 본다.
발표에서 설명하는 방식은 raw FK cube pose를 고정하는 **Fixed-FK**다.

### 입력과 최적화 변수

| 구분 | 내용 |
|---|---|
| 초기값 | 앞 단계에서 얻은 fixed-camera pose와 hand-eye transform |
| 고정 입력 | event별 flange pose, 선택된 PnP pose, raw FK set별 cube pose $\mathbf T_{B,O}^{s,\mathrm{FK}}$ |
| 주요 최적화 변수 | 모든 $\mathbf T_{B,C_i}$와 $\mathbf T_{F,C_g}$, 고정 board pose |

Fixed-FK에서는 $\mathbf T_{B,O}^{s,\mathrm{FK}}$를 vision 관측으로 수정하지 않고 최적화 안에서도 움직이지 않는
기준으로 둔다. 즉, FK가 제공한 cube pose가 정확하다고 가정한다.

Fixed camera가 계산한 cube pose와 gripper camera가 계산한 cube pose가 모두 이 고정 FK pose에 맞도록 한다.

$$
\mathbf T_{B,C_i}\mathbf T_{C_i,O}^{e}
\approx \mathbf T_{B,O}^{s,\mathrm{FK}}
$$

$$
\mathbf T_{B,F}^{e}\mathbf T_{F,C_g}\mathbf T_{C_g,O}^{e}
\approx \mathbf T_{B,O}^{s,\mathrm{FK}}
$$

이를 위해 fixed-camera pose $\mathbf T_{B,C_i}$와 hand-eye $\mathbf T_{F,C_g}$를 함께 움직인다. Board 관측과
camera 간 교차관계도 같은 최적화에 포함한다. Raw FK cube pose가 없는 set은 Fixed-FK의 고정 기준을 만들 수
없으므로 해당 Fixed-FK 계산에 사용할 수 없다.

### 오류 처리

- 일반적인 측정 오차: 전체 transform을 함께 움직여 **joint(공동) optimization으로 보정**
- 일부 큰 residual: robust loss와 사전 gate로 **영향력 억제 또는 제외**
- 최적화가 목적함수를 개선하지 못함: **최종값으로 채택하지 않음**
- 기존 결과에서 비현실적으로 큰 transform 변화가 발생함: **채택하지 않음**

즉, 모든 관측을 무조건 평균내는 것이 아니라 앞 단계에서 만든 신뢰 가능한 초기값 주변에서 전체 시스템의 모순을
줄이고, 결과가 실제로 좋아질 때만 채택한다.

### 최종 calibration 출력

| 행렬 | 개수 | 의미 |
|---|---:|---|
| $\mathbf T_{B,C_i}$ | fixed camera당 하나 | robot base 기준 fixed camera pose |
| $\mathbf T_{F,C_g}$ | 하나 | flange 기준 gripper camera의 고정된 장착 pose |


---

## 10. Step 4 — 최종 결과 검증

### 적용 범위

**전체 camera와 선택된 전체 검증 event**를 사용해 Step 3의 결과를 평가한다.

주요 검증 항목은 다음과 같다.

- 같은 cube를 본 camera들이 base 좌표계에서 같은 pose를 만드는지
- hand-eye 결과가 robot motion과 일관적인지
- marker corner reprojection error가 충분히 작은지
- depth로 복원한 표면과 cube 크기가 물리적으로 타당한지

### 오류 처리

Step 4는 원칙적으로 calibration transform을 다시 학습하거나 보정하는 단계가 아니다. 오차를 **측정·시각화·보고**하여
결과를 사용할 수 있는지 판단한다.

현재 cube reprojection 지표 일부는 각 검증 이미지에서 새로 푼 PnP의 self-fit 값이다. 따라서 이 값 하나만으로
최종 calibration 전체가 정확하다고 결론내리지 않고 cross-camera·hand-eye·depth 지표를 함께 본다.

---

## 11. Step 5 — 결과 출력

Step 4의 검증 결과와 최종 transform을 보고서로 내보낸다. 이 단계에서는 새로운 calibration 계산이나 보정을 하지
않는다.

---

## 12. 전체 단계 요약

| 단계 | 사용 범위 | 핵심 알고리즘 | 출력 | 오류 처리 |
|---|---|---|---|---|
| Step 1 | camera별 | factory parameter 로드, 선택적 intrinsic calibration | camera 내부 파라미터 | 본문에서 생략 |
| Step 2 | event별 여러 camera | 동기 촬영과 온라인 gate | RGB·aligned depth·flange pose | 불량 event 제외 |
| Step 3-A | 개별 camera×event | PnP와 self-fit reprojection | target pose | 실패 제외, pose 선택·가중 |
| Step 3-B | 여러 fixed camera×공통 event | 상대변환 consensus | camera 간 상대 pose | outlier 제외, 강건 통합 |
| Step 3-C | gripper camera×여러 event | hand-eye | $\mathbf T_{F,C_g}$ | outlier 제외 후 robust refinement |
| Step 3-D1 | 여러 camera×여러 event | board/cube를 통한 base 등록 | 초기 $\mathbf T_{B,C_i}$ | 안정적 경로 선택·결합 |
| Step 3-D2 | 여러 camera×set | pose consistency refinement | pose-level 보정값 | 부분/전체 채택 또는 거부 |
| Step 3-D3 | fixed camera별 전체 corner | raw-corner reprojection optimization | pixel-level 보정값 | robust 최적화 후 개선 시 채택 |
| Step 3-E | 전체 camera×event×set | joint optimization | 최종 calibration | 개선·변화량 검사 후 채택 |
| Step 4 | 전체 검증 데이터 | 일관성·reprojection·depth 평가 | 검증 지표 | 결과 수정 없이 보고 |
| Step 5 | 최종 결과 | report/export | 사용 가능한 transform | 계산 없음 |

---

## 13. Depth가 사용되는 위치

Depth는 PnP reprojection 수식이나 기본 global solver의 주 residual로 직접 들어가지 않는다.

```text
aligned depth
  → 촬영 단계의 품질 검사
  → PnP pose와 실제 표면의 일관성 확인
  → PnP 측정 선택과 신뢰도 판단 지원
  → 최종 cube 표면·크기 검증
```

따라서 현재 방식은 RGB의 2D corner가 calibration 계산의 중심이고, depth는 잘못된 RGB/PnP 관측을 판별하고 최종
결과의 물리적 타당성을 확인하는 보조 정보로 사용된다.
