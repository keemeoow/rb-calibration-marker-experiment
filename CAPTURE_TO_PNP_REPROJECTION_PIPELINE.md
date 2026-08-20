# 촬영부터 PnP·Reprojection·Calibration까지의 전체 데이터 흐름

이 문서는 우리 캘리브레이션 파이프라인에서 촬영된 데이터가 어떤 순서로 처리되는지 설명한다.

각 단계에서 다음을 명시한다.

- 한 camera·한 event만 처리하는지, 여러 camera·event·set을 함께 처리하는지
- 실제 입력 데이터가 무엇인지
- 출력 데이터와 행렬이 몇 개 만들어지는지
- 오차가 발생했을 때 데이터를 버리는지, 가중치만 낮추는지, 행렬 보정에 사용하는지
- 출력이 다음 단계에서 어떻게 사용되는지

---

## 0. 먼저 구분해야 하는 데이터 범위

### Camera

물리적인 카메라 한 대를 뜻한다.

- fixed cam 1
- fixed cam 2
- fixed cam 3
- gripper cam

camera index는 $i$로 표시한다.

### Event

한 시점에 여러 카메라와 로봇 상태를 함께 기록한 촬영 묶음이다.

```text
event e
 ├─ cam0 RGB/depth
 ├─ cam1 RGB/depth
 ├─ cam2 RGB/depth
 ├─ cam3 RGB/depth
 ├─ robot joints at event e
 └─ timestamps
```

event index는 $e$로 표시한다.

### Set

같은 cube placement 또는 같은 grasp 조건에 속하는 여러 event의 묶음이다.

- set 0: cube를 위치 A에 둔 상태에서 여러 event 촬영
- set 1: cube를 위치 B에 둔 상태에서 여러 event 촬영
- set 2: cube를 위치 C에 둔 상태에서 여러 event 촬영

set index는 $s$로 표시하고, event $e$가 속한 set은 $s(e)$로 쓴다.

### Session

전체 calibration 데이터다.

- 전체 camera × 전체 set × 전체 event

모든 event에서 모든 camera가 유효한 관측을 제공하는 것은 아니다. 가림, marker 검출 실패, 품질 gate로
일부 `(camera,event)` 관측이 빠질 수 있다.

---

## 1. 전체 흐름 요약

```text
[0] RGB·Depth·Robot joint 동기 촬영
                 │
                 ▼
[1] Camera intrinsics·cube geometry·FK 준비
                 │
                 ▼
[2] Camera별 RGB에서 marker 2D corner 검출
                 │
                 ▼
[3] Cube 3D corner와 RGB 2D corner 대응 구성
                 │
                 ▼
[4] Camera×event별 PnP pose 후보 계산
                 │
                 ▼
[5] PnP self-fit reprojection error 계산
                 │
                 ▼
[6] Depth-plane consistency 검사
                 │
                 ▼
[7] Camera×event별 대표 T(camera <- cube) 선택
                 │
                 ▼
[8] 전체 event의 관측으로 hand-eye·camera 초기 calibration
                 │
                 ▼
[9] Set별 cube pose와 FK frame 정렬
                 │
                 ▼
[10] Camera별 전체 corner reprojection refinement
                 │
                 ▼
[11] 전체 camera·event·set joint optimization
                 │
                 ▼
[12] Holdout pixel·camera consistency·외부 GT 평가
```

---

## 2. 단계 0 — RGB·Depth·Robot 상태 촬영

### 처리 범위

- event 하나 × 가능한 전체 camera

이 단계는 camera 한 대만 독립적으로 저장하는 단계가 아니라, 같은 시점의 여러 camera와 robot 상태를 하나의
event로 묶는 단계다.

### 입력

| 입력 | 의미 |
|---|---|
| RealSense camera들 | RGB와 depth 측정 |
| Robot joint encoder | 현재 관절값 측정 |
| Capture plan | 현재 set, placement/gripped 상태 |
| Camera/host clock | 촬영 timestamp |

### 출력

camera $i$, event $e$에 대해:

- $\mathbf I_i^e$ : RGB image
- $\mathbf Z_i^e$ : aligned depth image
- $t_i^e$ : timestamp

event 전체에 대해:

- $\mathbf q_e$        : robot joint vector
- `set_index` : $s(e)$
- cube state : placed 또는 gripped

실제 저장 예시는 다음과 같다.

- cam{i}/rgb_*.jpg
- cam{i}/depth_*.png
- meta.json

### RGB와 depth 데이터의 의미

RGB와 depth는 모두 가로×세로 배열이지만 값의 의미가 다르다.

- $\mathbf I_i^e[v,u]$ = pixel $(u,v)$의 색상 $(R,G,B)$
- $\mathbf Z_i^e[v,u]$ = pixel $(u,v)$의 camera $z$축 방향 깊이

Depth는 color image에 정렬된 aligned depth를 사용한다.

### 이 단계에서 만들어지는 행렬

아직 PnP나 camera calibration 행렬을 만들지 않는다.

Robot joint로부터 다음 단계에서 event별 FK 행렬을 만들 수 있다.

$$
T_{B,G}^{e}=FK(q_e)
$$

$\mathbf T_{B,G}^{e}$는 event당 하나다.

### 오차 처리

| 문제 | 처리 |
|---|---|
| Camera frame 누락 | 해당 `(camera,event)` 관측 없음 |
| Camera timestamp 차이가 너무 큼 | event 전체 reject 가능 |
| RGB blur 또는 clipping | event 전체 reject 가능 |
| 필수 camera 수 부족 | event 전체 reject 가능 |
| Depth 저장 비활성화 | depth 기반 gate를 비활성화하고 RGB 경로만 진행 가능 |

이 단계에서는 불량 이미지를 숫자적으로 보정하지 않는다. 저장을 거절하거나 해당 camera 관측을 누락시킨다.

### 다음 단계로 전달

- RGB images
- aligned depth images
- robot $\mathbf q_e$
- timestamps
- set index

### 촬영 직후 저장 전 품질 검사

“촬영 단계에서 오차가 큰 이미지를 버린다”는 말은 정확히는 다음 과정이다.

```text
여러 camera의 RGB/depth를 임시 획득
        ↓
각 camera 영상에서 marker·ChArUco 검출
        ↓
각 camera에서 임시 cube PnP 계산
        ↓
PnP reprojection·depth-plane·RGB ROI 품질 계산
        ↓
같은 event의 전체 camera 결과를 capture gate에 입력
        ↓
PASS이면 정식 calibration capture로 저장
FAIL이면 해당 event를 저장하지 않음
```

따라서 촬영 과정에서도 뒤에서 설명하는 marker 검출, PnP, reprojection, depth 검사를 **저장 여부 판단용으로
먼저 실행**한다. 이후 Step3는 저장된 관측을 재사용하거나 RGB/depth를 다시 읽어 후보를 재계산할 수 있다.

### Camera 한 장만 빠지는 경우와 event 전체가 버려지는 경우

두 상황을 구분해야 한다.

#### Camera 한 장의 관측만 유효하지 않은 경우

- cam2에서 marker 검출 실패
- → cam2의 해당 event PnP 없음
- → 다른 camera들이 최소 요구 조건을 만족하면 event는 통과할 수도 있음

즉 camera 한 대가 실패했다고 항상 event 전체를 버리는 것은 아니다.

#### Event 전체가 버려지는 경우

- 유효 camera 수가 profile의 최소값보다 적음
- 또는
- 필수 gripper/fixed-camera 조건 실패
- 또는
- ROI·depth·timestamp 같은 event gate 실패
- → event 전체 FAIL

Event가 FAIL이면 그 시점의 여러 camera frame과 robot pose 묶음을 calibration observation으로 사용하지 않는다.

### 촬영 gate가 검사하는 항목

| 검사 항목 | 계산 범위 | 문제가 있을 때의 기본 처리 |
|---|---|---|
| Cube marker visible camera 수 | event의 전체 camera | 최소 수 미달 시 event reject |
| Fixed-camera visible 수 | event의 fixed camera 전체 | 최소 수 미달 시 event reject |
| Multi-marker fixed-camera 수 | event의 fixed camera 전체 | 최소 수 미달 시 event reject |
| Cube PnP 성공 camera 수 | event의 전체 camera | 최소 수 미달 시 event reject |
| PnP mean reprojection | camera×event | 해당 PnP를 gate-quality로 세지 않음 |
| Gripper-camera PnP | gripper camera×event | 해당 block에서 필수면 event reject |
| ChArUco corner 수 | gripper camera×event | 최소 수 미달 시 event reject |
| Depth valid sample 수 | camera×event | depth-valid로 세지 않음 |
| Depth-plane mean error | camera×event | 해당 block 기준 초과 시 event reject 가능 |
| Marker ROI clipping | marker가 보인 camera×event | 기준 초과 camera가 있으면 event reject |
| Marker ROI sharpness | marker가 보인 camera×event | 활성화된 기준 미달 시 event reject |
| Timestamp 누락 | event의 전체 camera | 기본적으로 event reject |
| Camera timestamp span | event의 전체 camera | 기준 초과 시 event reject |
| Cube placed/gripped 상태 | event | capture block과 모순되면 event reject |

### RGB 화질 검사

Marker가 실제로 있는 ROI만 대상으로 화질을 계산한다.

#### Clipping

너무 어둡거나 너무 밝아 흑색/백색으로 포화된 pixel 비율을 본다.

- 기본 `max_roi_clip_frac` = $0.05$

Marker ROI의 clipping 비율이 5%를 넘으면 해당 camera가 reject 목록에 들어가고 event gate가 실패한다.

#### Sharpness

Laplacian variance로 blur 정도를 측정한다.

- 값이 작음 → 흐린 영상
- 값이 큼   → edge가 선명한 영상

Camera마다 scale이 달라 기본 threshold는 `0`, 즉 비활성이다. Pilot data로 camera별 기준을 정한 경우에만
실제 reject gate로 동작한다.

### PnP reprojection 촬영 gate

각 camera에서 임시 PnP를 푼 뒤 mean reprojection error를 계산한다.

- 기본 `max_cube_pnp_reproj_mean_px` = $2.0\,\mathrm{px}$
- PnP solve 성공 AND mean reprojection ≤ 2 px
- → gate-quality PnP
- PnP solve 실패 또는 mean reprojection > 2 px
- → 그 camera의 PnP를 gate-quality로 세지 않음

그 결과 전체 event에서 요구하는 PnP 성공 camera 수가 부족하면 event를 버린다.

### Depth 촬영 gate

- 기본 최소 valid depth sample 수 = 20

PnP가 성공했고 marker polygon 안에서 최소 sample 수를 만족해야 depth-valid 관측으로 센다.

Depth는 block에 따라 요구 조건이 다르다.

### A-placement 기본 gate

A-placement는 cube가 놓여 있고 gripper camera도 관측에 사용하는 block이다.

| 조건 | 기본값 |
|---|---:|
| Cube visible camera | 2대 이상 |
| Fixed camera visible | 1대 이상 |
| Multi-marker fixed camera | 1대 이상, camera당 marker 2개 이상 |
| PnP-ok camera | 2대 이상 |
| Fixed PnP-ok camera | 1대 이상 |
| Gripper marker | 1개 이상 |
| Gripper ChArUco corner | 8개 이상 |
| Gripper PnP | 필수 |
| Gripper depth-valid | depth 촬영 시 기본 필수 |
| Gripper depth-plane mean | 40 mm 이하 |
| PnP mean reprojection | 2 px 이하 |
| Valid depth sample | 20개 이상 |
| Timestamp span | 120 ms 이하 |
| Marker ROI clipping | 5% 이하 |

이 중 하나가 실패하면 기본적으로 A-placement event 전체를 저장하지 않는다.

### B-eye-to-hand 기본 gate

B-eye-to-hand는 cube를 gripper가 잡고 있고 fixed camera 관측이 중심인 block이다.

| 조건 | 기본값 |
|---|---:|
| Fixed camera cube visible | 2대 이상 |
| Multi-marker fixed camera | 2대 이상, camera당 marker 2개 이상 |
| Fixed PnP-ok camera | 2대 이상 |
| Depth-quality fixed camera | 1대 이상 |
| Fixed depth-plane mean | 20 mm 이하인 camera가 요구 수 이상 |
| Gripper-camera PnP | 요구하지 않음 |
| Gripper-camera depth | 요구하지 않음 |
| PnP mean reprojection | 2 px 이하 |
| Valid depth sample | 20개 이상 |
| Timestamp span | 120 ms 이하 |
| Marker ROI clipping | 5% 이하 |

B block에서는 wrist camera가 자신이 잡고 있는 cube를 보기 어렵기 때문에 gripper-camera 관측을 기본 필수로
두지 않는다.

### 촬영 gate에서 “보정”하는가?

촬영 gate는 불량 RGB/depth 값을 고치거나 camera pose를 최종 보정하지 않는다.

- 좋은 event → 저장하고 다음 단계에 사용
- 나쁜 event → 저장하지 않거나 calibration observation에서 제외

즉 주 역할은 **reject**다. PnP는 gate 계산을 위한 임시 pose를 만들지만 이 시점의 목적은 최종
multi-camera calibration이 아니라 저장할 가치가 있는 event인지 판단하는 것이다.

`force_save`와 같은 예외 경로는 진단용이며 논문용 calibration 데이터에는 사용하지 않는 것이 정책이다.

---

## 3. 단계 1 — 고정 데이터와 사전 파라미터 준비

### 처리 범위

- Camera intrinsics는 camera별 하나를 로드한다.
- Cube geometry는 session 전체에서 공통으로 사용한다.
- Robot FK는 event별 하나를 계산한다.

### 입력

| 입력 데이터 | 실제 위치/필드 |
|---|---|
| Camera intrinsics | `intrinsics/cam{i}.npz` |
| Robot joint | `meta.json`의 joint 정보 |
| Cube geometry | `config.py` 또는 resolved cube config |

### 출력

camera마다:

- $\mathbf K_i$             : $3\times3$ camera intrinsic matrix
- $\mathbf D_i$             : lens distortion coefficients
- $s_{\mathrm{depth},i}$ : raw depth unit을 metre로 바꾸는 scale

event마다:

- $\mathbf T_{B,G}^{e}=\operatorname{FK}(\mathbf q_e)$

session 공통:

- $\mathbf X_O^{m,k}$ : cube object frame에서 marker $m$의 corner $k$가 놓인 3D 위치

### 데이터 차원

- $\mathbf K_i$             : $3\times3$
- $\mathbf D_i$             : distortion coefficient vector
- $\mathbf T_{B,G}^{e}$         : $4\times4$
- $\mathbf X_O^{m,k}$ : 3D point $(X,Y,Z)$, metre

### 행렬 개수

| 행렬 | 개수 |
|---|---|
| $\mathbf K_i,\mathbf D_i$ | camera당 하나 |
| $\mathbf T_{B,G}^{e}$ | event당 하나 |
| Cube geometry | session 공통 한 모델 |

### 오차 처리

| 문제 | 처리 |
|---|---|
| Intrinsics 파일 없음/불량 | 해당 camera calibration 진행 불가 |
| Robot joint/FK 없음 | FK가 필요한 해당 event 경로 제외 |
| Cube config와 실제 marker 배치 불일치 | cube-model self-check warn 또는 calibration 중단 |

$\mathbf K_i,\mathbf D_i$는 이 단계 이후 production calibration에서 고정된 값으로 취급한다.

### 다음 단계로 전달

- camera별 $\mathbf K_i,\mathbf D_i$, depth scale
- event별 $\mathbf T_{B,G}^{e}$
- cube의 3D marker corner model

---

## 4. 단계 2 — RGB에서 AprilTag corner 검출

### 처리 범위

- 개별 camera $i$ × 개별 event $e$

다른 camera나 다른 event의 데이터는 이 검출 과정에 들어가지 않는다.

### 입력

- RGB image $\mathbf I_i^e$
- AprilTag dictionary
- cube marker ID/face config

Depth, robot FK, 다른 camera pose는 입력하지 않는다.

### 처리

- RGB → grayscale
-      → marker 검출
-      → marker ID 확인
-      → 네 corner의 pixel 좌표 추출
-      → cube config에 맞게 corner 순서 재정렬

### 출력

marker $m$, corner $k$에 대해:

$$
u_{i,e}^{m,k}=
\begin{bmatrix}u\\v\end{bmatrix}
\quad[px]
$$

예:

- marker ID 2
- `corners_2d`의 예:

$$
\begin{bmatrix}
320 & 240\\
356 & 242\\
354 & 278\\
318 & 276
\end{bmatrix}
$$


### 출력 개수

각 `(camera,event)`에서 보인 marker 수만큼 marker record가 나온다.

- marker 1개 검출 → 2D corner 4개
- marker 3개 검출 → 보통 2D corner 12개

### 이 단계에서 행렬이 만들어지는가?

아니다. 아직 $\mathbf T_{C,O}$를 계산하지 않는다. 2D pixel 관측만 만든다.

### 오차 처리

| 문제 | 처리 종류 | 동작 |
|---|---|---|
| 검출 실패 | Reject | 해당 marker 관측 없음 |
| Cube config에 없는 ID | Reject | 해당 marker 제외 |
| Marker aspect가 기준 미달 | Reject | 너무 비스듬한 marker 제외 |
| Corner ordering 불일치 | Reorder | 값을 새로 만들지 않고 순서만 정렬 |
| 남은 marker 부족 | Reject | 해당 `(camera,event)` PnP 생성 불가 |

검출되지 않은 corner를 임의로 보간해 만들지는 않는다.

### 다음 단계로 전달

- marker IDs
- marker별 RGB 2D corners

---

## 5. 단계 3 — 3D object points와 2D image points 대응 구성

### 처리 범위

- 개별 camera $i$ × 개별 event $e$

### 입력

- 검출된 marker ID
- 검출된 2D corner $\mathbf u_{i,e}^{m,k}$
- cube geometry $\mathbf X_O^{m,k}$

### 출력 1: Object points

- `object_points`: $N\times3$

각 행은 cube object frame에서 corner의 알려진 3D 위치다.

$$
\begin{bmatrix}
X_0 & Y_0 & Z_0\\
X_1 & Y_1 & Z_1\\
\vdots & \vdots & \vdots
\end{bmatrix}
$$


단위는 metre다. 이 3D 값은 depth camera가 측정한 점이 아니라, cube 크기와 marker 부착 위치로 미리
정의한 모델이다.

### 출력 2: Image points

- `image_points`: $N\times2$

각 행은 동일한 corner가 RGB 이미지에서 검출된 2D pixel이다.

$$
\begin{bmatrix}
u_0 & v_0\\
u_1 & v_1\\
\vdots & \vdots
\end{bmatrix}
$$


### 두 배열의 관계

같은 row index는 동일한 실제 corner를 뜻한다.

- `object_points[0]` ↔ `image_points[0]`
- `object_points[1]` ↔ `image_points[1]`
- ...

$M$개 marker가 각각 네 corner를 제공하면 일반적으로:

$$
N=4M
$$

이다.

### 이 단계에서 행렬이 만들어지는가?

아니다. PnP가 사용할 3D–2D 대응 배열을 만든다.

### 오차 처리

- 유효하지 않은 marker는 correspondence에 넣지 않는다.
- 3D와 2D corner 수 또는 순서가 맞지 않으면 PnP에 사용할 수 없다.
- 남은 marker 수가 `min_markers`보다 적으면 해당 PnP를 만들지 않는다.

### 다음 단계로 전달

- $N\times3$ `object_points`
- $N\times2$ `image_points`
- used marker IDs

---

## 6. 단계 4 — Camera×event별 PnP pose 후보 계산

### 처리 범위

- 개별 camera $i$ × 개별 event $e$

PnP 자체는 다른 camera나 다른 event를 함께 보지 않는다.

### 입력

| 입력 | 차원 | 출처 |
|---|---:|---|
| `object_points` | $N\times3$ | Cube 3D model |
| `image_points` | $N\times2$ | 해당 camera/event RGB 검출 |
| $\mathbf K_i$ | $3\times3$ | 해당 camera intrinsics |
| $\mathbf D_i$ | vector | 해당 camera distortion |

### 직접 입력하지 않는 데이터

- Depth image
- Robot FK
- 다른 camera의 PnP pose
- $\mathbf T_{B,C_i}$
- Set 정보

### PnP 내부 계산

PnP는 다음 pixel residual이 작아지는 pose를 찾는다.

$$
r_k(T)=
\pi(K_i,D_i,T X_O^k)-u_{i,e}^k
$$

$$
T_{C_i,O}^{e*}
=\arg\min_T\sum_k\|r_k(T)\|_2^2
$$

즉 PnP는 내부에서 이미 다음을 반복한다.

```text
T(camera <- cube) 후보
      ↓
3D corner를 pixel로 projection
      ↓
실제 RGB corner와 pixel error 계산
      ↓
error가 줄어드는 pose 탐색
```

### 출력

- rvec                 : cube rotation in camera
- tvec                 : cube origin position in camera
- $\mathbf T_{C_i,O}^{e,c}$         : $4\times4$ camera-to-cube pose candidate
- used marker IDs

$\mathbf T_{C_i,O}$의 정확한 정의는:

$$
X_{C_i}=T_{C_i,O}X_O
$$

즉 cube frame의 점을 camera frame으로 옮긴다. Translation은 camera에서 본 cube object-frame 원점의 위치다.

### 후보가 여러 개인 이유

한 `(camera,event)`에서도 다음 후보가 생길 수 있다.

- 모든 marker를 함께 쓴 multi-marker PnP
- marker 0만 쓴 single-marker PnP
- marker 2만 쓴 single-marker PnP
- 평면 marker의 IPPE solution 0
- 평면 marker의 IPPE solution 1
- capture 때 저장한 pose
- Step3에서 다시 계산한 pose

따라서 출력은 처음부터 행렬 하나로 한정되지 않는다.

```text
cam2, event7
 ├─ PnP pose candidate 0
 ├─ PnP pose candidate 1
 └─ PnP pose candidate 2
```

여기서 $c$는 candidate index다.

### PnP 방식

| 상황 | 방법 |
|---|---|
| Single planar marker | IPPE 복수 pose 후보 |
| 충분한 multi-corner | `solvePnPRansac`/iterative PnP |

### 오차 처리

| 문제 | 처리 종류 | 동작 |
|---|---|---|
| RANSAC 기준을 크게 벗어난 corner | Reject/robust | Pose 계산 영향 제거 |
| PnP solver 실패 | Reject | 해당 후보 없음 |
| Camera 뒤쪽에 놓이는 pose | Candidate penalty/reject | 물리적으로 부적절한 후보 |
| 보이는 face 방향과 모순 | Candidate penalty | 우선순위 낮춤 |
| Planar ambiguity | 후보 유지 | 다음 단계에서 비교 가능 |

### 다음 단계로 전달

- camera×event별 한 개 이상의 $\mathbf T_{C_i,O}^{e,c}$ 후보

---

## 7. 단계 5 — PnP self-fit reprojection error 계산

### 처리 범위

- 개별 camera $i$ × 개별 event $e$ × 개별 PnP candidate $c$

### 중요한 설명

단계 4와 단계 5는 서로 다른 종류의 오차를 쓰는 것이 아니다. 같은 pixel residual을 다른 목적으로 사용한다.

- 단계 4:
- reprojection residual을 줄여 $\mathbf T_{C,O}$ 후보를 계산
- 단계 5:
- 계산이 끝난 $\mathbf T_{C,O}$ 후보의 reprojection error를 다시 측정·기록

단계 5의 결과를 다시 PnP에 넣어 새로운 별도 PnP를 수행하는 것은 아니다.

### 입력

- $\mathbf T_{C_i,O}^{e,c}$
- 같은 candidate에 사용한 cube 3D corners $\mathbf X_O^k$
- $\mathbf K_i,\mathbf D_i$
- 실제 RGB corners $\mathbf u_{i,e}^{k}$

### 처리

3D corner를 다시 이미지에 투영한다.

$$
\hat u_{i,e}^{k,c}
=\pi(K_i,D_i,T_{C_i,O}^{e,c}X_O^k)
$$

실제 검출 pixel과 차이를 계산한다.

$$
r_{i,e}^{k,c}
=\hat u_{i,e}^{k,c}-u_{i,e}^k
=
\begin{bmatrix}
\hat u-u\\
\hat v-v
\end{bmatrix}
$$

$$
e_{i,e}^{k,c}=\|r_{i,e}^{k,c}\|_2
$$

### 출력

- corner별 projected pixel
- corner별 $(\Delta u,\Delta v)$
- corner별 scalar error [px]
- mean error [px]
- median error [px]
- P90 error [px]

### RMSE

$$
RMSE=
\sqrt{\frac{1}{N}\sum_{k=1}^{N}e_k^2}
$$

큰 오차를 제곱하기 때문에 mean보다 큰 outlier에 더 민감하다. 단위는 pixel이다.

### 오차 처리

| 적용 위치 | 기본 기준/처리 |
|---|---|
| Capture gate | 평균 reprojection이 기본 2 px를 넘으면 gate-quality PnP로 인정하지 않음 |
| Step3 fixed candidate | 평균 3 px 초과 후보 제외 |
| Step3 gripper candidate | 평균 5 px 초과 후보 제외 |
| Step3 fixed role filter | 평균 2.75 px 초과 후보 제외 |
| Candidate weight | error가 클수록 downstream 영향 감소 |

PnP reprojection error는 같은 이미지로 pose를 구하고 같은 이미지에서 측정한 self-fit이다. 따라서 작다고 해서
multi-camera calibration이 정확하다는 뜻은 아니다.

### 다음 단계로 전달

```text
각 camera-event의 PnP pose 후보
+
후보별 reprojection quality
```

---

## 8. 단계 6 — Depth-plane consistency 검사

### 처리 범위

- 개별 camera $i$ × 개별 event $e$ × PnP candidate $c$

### 입력

- aligned depth image $\mathbf Z_i^e$
- $s_{\mathrm{depth},i}$
- $\mathbf K_i,\mathbf D_i$
- PnP candidate $\mathbf T_{C_i,O}^{e,c}$
- marker polygon pixel 영역

### 처리

PnP pose로 camera 기준 marker plane을 계산한다.

- PnP pose → marker plane origin $\mathbf p_0$와 normal $\mathbf n$

Marker polygon 안의 depth pixel로 camera ray를 만든다.

$$
\mathbf q(u,v)=
\begin{bmatrix}x_n\\y_n\\1\end{bmatrix}
$$

Ray와 PnP marker plane의 교점으로 예상 깊이를 계산한다.

$$
z_{\mathrm{pred}}=\frac{\mathbf n^\mathsf{T}\mathbf p_0}
{\mathbf n^\mathsf{T}\mathbf q}
$$

Depth camera 측정값은:

$$
z_{\mathrm{meas}}=\mathbf Z_i^e[v,u]\,s_{\mathrm{depth},i}
$$

오차는:

$$
e_{\mathrm{depth}}=\left|z_{\mathrm{meas}}-z_{\mathrm{pred}}\right|\quad[\mathrm{mm}]
$$

### 출력

- depth valid 여부
- valid depth sample 수
- depth-plane mean/median/max [mm]
- depth inlier ratio
- predicted/measured depth scale와 bias

### Depth 자체의 오류 문제

RealSense depth에도 다음 오류가 있다.

- 물체 경계의 튐
- 검은색/반사 표면
- RGB-depth alignment 오차
- 먼 거리의 큰 noise
- 0 또는 invalid depth
- 비스듬한 표면의 불안정성

그래서 depth 한 점을 GT처럼 사용하지 않는다.

현재는:

- marker polygon 내부의 많은 depth sample 사용
- 0/invalid sample 제외
- 최소 sample 수 요구
- mean/median/inlier ratio 사용
- 비교적 느슨한 threshold 사용

으로 큰 PnP depth 오류를 잡는 보조 검사로 사용한다.

### 오차 처리

| 상황 | 처리 |
|---|---|
| Depth sample 부족 | depth-invalid 처리 |
| Capture 필수 depth 조건 실패 | event 전체 reject 가능 |
| Depth-plane error가 큼 | candidate 우선순위·가중치 감소 |
| Depth 없음 | 일부 경로에서 weight 감소 또는 depth gate 비활성화 |

Depth error로 $\mathbf T_{C,O}$나 $\mathbf T_{B,C_i}$를 직접 최적화하지 않는다.

### 다음 단계로 전달

- 각 candidate의 depth quality 정보

---

## 9. 단계 7 — Camera×event별 대표 PnP 후보 선택

### 처리 범위

- 개별 camera $i$ × 개별 event $e$ 안의 모든 candidate

아직 다른 event 전체를 한꺼번에 calibration하지 않는다.

### 입력

- $\mathbf T_{C_i,O}^{e,0}$, $\mathbf T_{C_i,O}^{e,1}$, ...
- 각 후보의 marker/face 수
- 각 후보의 reprojection error
- 각 후보의 depth quality
- visibility/cheirality
- 후보 source

### 처리

대체로 다음 정보를 이용해 대표 후보를 정한다.

- 1. 더 많은 marker/서로 다른 cube face를 사용했는가
- 2. Depth가 유효하고 PnP plane과 잘 맞는가
- 3. Reprojection error가 작은가
- 4. 물리적으로 가능한 pose인가
- 5. Candidate source 우선순위

### 출력

유효한 `(camera,event)`당 대표 pose 최대 하나:

- $\mathbf T_{C_i,O}^{e}$
- `err_mean_px`
- used marker IDs
- depth-plane metrics
- source

진단과 일부 후속 ambiguity 처리를 위해 전체 `_candidates` 목록도 보존할 수 있다.

### 행렬 개수 예시

camera 4대, event 30개라면 대표 PnP pose의 이론적 최대 개수는:

$$
4\times30=120
$$

이다. 실제로는 가림과 검출 실패 때문에 더 적다.

### 오차 처리

| 문제 | 처리 |
|---|---|
| Threshold를 넘은 후보 | Reject |
| 순위가 낮은 후보 | 대표 관측으로 사용하지 않음 |
| Depth가 불량하지만 완전 reject 조건은 아님 | Down-weight 또는 낮은 rank |
| 후보가 하나도 없음 | 해당 `(camera,event)` 관측 없음 |

### 다음 단계로 전달

- `pnp_obs[i][e]` = 대표 $\mathbf T_{C_i,O}^{e}$와 품질 정보

---

## 10. 단계 8 — Event 전체에 대한 capture gate

### 처리 범위

- 동일 event $e$의 전체 camera 결과

PnP는 camera×event별로 독립적으로 풀었지만, capture를 calibration 데이터로 인정할지는 같은 event의
전체 camera를 함께 보고 결정한다.

### 입력

- camera별 marker 검출 수
- camera별 PnP 성공 여부
- camera별 reprojection error
- camera별 depth valid/plane error
- camera timestamps
- RGB ROI blur/clipping
- gripper/placement block 조건

### 출력

- event accepted 또는 rejected
- reject reasons

### 오차 처리

기본 설정의 예:

- PnP reprojection mean ≤ 2 px
- 최소 PnP 성공 camera 수 충족
- 최소 fixed-camera 수 충족
- 최소 depth sample 수 충족
- 필요한 block에서 depth-quality camera 수 충족
- timestamp span ≤ 기준

하나의 camera PnP가 실패했다고 항상 event 전체를 버리는 것은 아니다. Profile이 요구하는 최소 camera 수를
충족하지 못할 때 event 전체를 버린다.

### 다음 단계로 전달

- accepted event들의 camera별 대표 PnP 관측
- robot FK
- set 정보
- board 관측

---

## 11. 여기까지의 데이터 범위 정리

단계 0~8까지의 핵심 계산 단위는 다음과 같다.

| 단계 | 처리 범위 | 출력 행렬 |
|---|---|---|
| Capture | event × 전체 camera | 아직 calibration 행렬 없음 |
| Marker detection | camera×event | 행렬 없음 |
| Correspondence | camera×event | 행렬 없음 |
| PnP candidates | camera×event | 복수 $\mathbf T_{C_i,O}^{e,c}$ |
| Reprojection/depth 검사 | camera×event×candidate | 행렬 수정 없음, 품질 지표 추가 |
| Candidate selection | camera×event | 대표 $\mathbf T_{C_i,O}^{e}$ 하나 |
| Capture gate | event×전체 camera | event accept/reject |

즉 여기까지는:

- 한 camera에서 한 event에 보인 cube pose

를 반복해서 만든 것이다.

아직 최종 $\mathbf T_{B,C_i}$는 없다.

---

## 12. 단계 9 — 여러 camera·event를 이용한 초기 calibration

이 단계부터 서로 다른 camera와 여러 event를 연결한다.

### 12.1 Fixed-camera 상대변환

#### 처리 범위

- reference fixed camera와 camera $i$의 공통 event 전체

#### 입력

event $e$에서:

- $\mathbf T_{C_{ref},O}^{e}$
- $\mathbf T_{C_i,O}^{e}$
- 두 PnP의 품질 정보

#### Event별 출력 후보

$$
T_{C_{ref},C_i}^{e}
=T_{C_{ref},O}^{e}(T_{C_i,O}^{e})^{-1}
$$

#### 전체 출력

모든 공통 event 후보를 robust average한다.

- fixed camera $i$당 $\mathbf T_{C_{\mathrm{ref}},C_i}$ 하나

#### 오차 처리

- Reprojection이 작은 관측에 높은 weight
- Depth가 나쁜 관측의 weight 감소
- Robust SE(3) average로 큰 pose outlier 영향 감소/제거

### 12.2 Gripper-camera hand-eye

#### 처리 범위

- gripper camera 하나 × 전체 eligible event

#### 입력

- event별 $\mathbf T_{B,G}^{e}$
- event별 $\mathbf T_{C_g,W}^{e}$ (ChArUco 우선)
- 또는 fallback $\mathbf T_{C_g,O}^{e}$

#### 출력

- $\mathbf T_{G,C_g}$ 하나

$\mathbf T_{G,C_g}$는 event마다 만드는 행렬이 아니다. Gripper와 카메라가 단단히 고정되어 있으므로 전체 session에서
하나다.

Event별 gripper-camera base pose는:

$$
T_{B,C_g}^{e}=T_{B,G}^{e}T_{G,C_g}
$$

이므로 event마다 달라진다.

#### 오차 처리: MAD

MAD는 Median Absolute Deviation이다.

$$
MAD=median(|e_i-median(e)|)
$$

Hand-eye 초기해에서 target base pose가 다른 event들과 크게 어긋난 event를 찾는다.

$$
threshold=median(e)+2(1.4826)MAD(e)
$$

Threshold 초과 event는 hand-eye 재계산 subset에서 제외한다. 너무 많이 버리지 않도록 최소 데이터 수를
보장한다.

#### 오차 처리: Huber loss

Huber loss는 작은 residual은 제곱오차로 맞추고 큰 residual은 영향력을 낮춘다.

$$
L_\delta(r)=
\begin{cases}
\frac12r^2,&|r|\le\delta\\
\delta(|r|-\frac12\delta),&|r|>\delta
\end{cases}
$$

MAD가 hard reject라면 Huber는 soft down-weight다.

### 12.3 Fixed camera를 robot base에 등록

#### 처리 범위

- fixed camera 하나 × 그 camera가 target을 본 전체 event/set

#### Cube 경로 입력

- event별 $\mathbf T_{B,O}^{e}$
- event별 $\mathbf T_{C_i,O}^{e}$

#### Board 경로 입력

- event별 $\mathbf T_{B,W}^{e}$
- event별 $\mathbf T_{C_i,W}^{e}$

#### Event별 camera pose 후보

$$
T_{B,C_i}^{e}
=T_{B,target}^{e}(T_{C_i,target}^{e})^{-1}
$$

#### 최종 출력

- fixed camera $i$당 $\mathbf T_{B,C_i}$ 하나

camera가 세 대면:

- $\mathbf T_{B,C_1}$
- $\mathbf T_{B,C_2}$
- $\mathbf T_{B,C_3}$

세 개가 나온다. Set마다 별도의 fixed-camera 행렬을 만드는 것이 아니다.

#### 오차 처리

- Event별 후보를 robust average
- Reprojection/depth 품질에 따라 weight 조정
- Cube와 board source를 정책에 따라 merge/refine
- 관측 부족 camera는 이전 단계 값 또는 fallback 유지

### 다음 단계로 전달

- fixed camera당 초기 $\mathbf T_{B,C_i}$ 하나
- global $\mathbf T_{G,C_g}$ 하나
- global/평균 $\mathbf T_{B,W}$
- event/set별 cube anchor

---

## 13. 단계 10 — Set별 cube pose와 FK frame 정렬

### 처리 범위

- 전체 유효 set을 함께 사용

### 필요한 이유

Robot에 저장한 `set_cube_center` frame과 AprilTag cube object frame이 정확히 같지 않을 수 있다.

```text
Robot/TCP가 생각하는 cube-center frame
≠
AprilTag geometry가 정의하는 cube object frame
```

### 입력

set $s$마다:

- $\mathbf T_{B,S}^{s}$ : robot에 저장된 nominal set cube-center pose
- $\mathbf T_{B,O}^{s}$ : visual observation에서 만든 AprilTag cube pose

### Set별 offset 후보

$$
\Delta_s=(T_{B,S}^{s})^{-1}T_{B,O}^{s}
$$

### Global offset 출력

전체 set의 offset을 robust average한다.

$$
\bar\Delta=RobustAvg_s(\Delta_s)
$$

이 offset은 session에 하나다.

### Set별 corrected cube prior 출력

$$
\widetilde T_{B,O}^{s}=T_{B,S}^{s}\bar\Delta
$$

$\mathbf T_{B,O}^{s}$는 set마다 하나다.

### 행렬 개수

- Global frame offset: 하나
- Corrected cube pose: set 수만큼
- Fixed-camera $\mathbf T_{B,C_i}$: 여전히 camera당 하나

### 오차 처리

- Visual set anchor와 nominal prior가 gate 안이면 corrected prior 채택
- Gate 밖이면 해당 prior를 강제하지 않고 visual anchor 유지
- Prior가 나쁘다고 fixed-camera 행렬을 set별로 따로 만들지는 않음

### 다음 단계로 전달

- set별 fixed/aligned $\mathbf T_{B,O}^{s}$
- camera별 초기 $\mathbf T_{B,C_i}$

---

## 14. 단계 11 — Fixed-camera raw-corner reprojection refinement

이 단계는 앞의 PnP self-fit reprojection과 목적과 변수 범위가 다르다.

### 처리 범위

- fixed camera 하나 × 그 camera의 전체 유효 placement event/set

다른 fixed camera와 joint로 풀지 않고 camera별로 독립 최적화한다. 그러나 해당 camera 내부에서는 여러
event의 모든 raw corner를 한꺼번에 사용한다.

### 입력

| 입력 | 개수/범위 |
|---|---|
| 초기 $\mathbf T_{B,C_i}$ | fixed camera당 하나 |
| $\mathbf T_{B,O}^{e}$ | 유효 event마다 하나, 이 단계에서는 고정 |
| RGB corner $\mathbf u_{i,e}^{k}$ | 해당 camera의 모든 event corner |
| Cube 3D corner $\mathbf X_O^k$ | session 공통 |
| $\mathbf K_i,\mathbf D_i$ | 해당 camera에 하나 |

### 미지수

- 해당 fixed camera의 $\mathbf T_{B,C_i}$ 하나

Event마다 별도 $\mathbf T_{B,C_i}$를 만들지 않는다.

예:

- cam2의 event 50개
- → corner 관측은 수백 개
- → 수정하는 행렬은 $\mathbf T_{B,C_2}$ 하나

### Camera 기준 cube pose 예측

$$
T_{C_i,O}^{e}(T_{B,C_i})
=(T_{B,C_i})^{-1}T_{B,O}^{e}
$$

### Pixel reprojection residual

$$
r_{i,e}^{k}
=\pi(K_i,D_i,
(T_{B,C_i})^{-1}T_{B,O}^{e}X_O^k)
-u_{i,e}^k
$$

### PnP self-fit과의 차이

- 앞 단계 PnP self-fit:
- camera×event마다 $\mathbf T_{C_i,O}^{e}$를 따로 움직여 그 이미지 하나를 맞춤
- 이 단계 calibration reprojection:
- $\mathbf T_{B,C_i}$ 하나를 움직여 그 camera의 전체 event를 동시에 맞춤

### 출력

- refined $\mathbf T_{B,C_i}$ 후보 하나
- pixel RMSE before/after
- 초기값 대비 translation/rotation 변화

### RMSE

$$
RMSE=
\sqrt{\frac{1}{N}\sum_{k=1}^{N}\|r_k\|_2^2}
$$

단위는 pixel이다.

### 오차 처리

1. Huber loss로 큰 corner residual을 soft down-weight한다.
2. Optimizer가 $\mathbf T_{B,C_i}$를 자동으로 조금씩 바꾸며 pixel error를 줄인다.
3. 다음 조건을 모두 만족할 때만 결과를 채택한다.

- Pixel RMSE 감소
- AND translation 변화 ≤ 20 mm
- AND rotation 변화 ≤ 3°

그렇지 않으면 최적화한 후보 행렬을 버리고 이전 $\mathbf T_{B,C_i}$를 유지한다.

### 다음 단계로 전달

- fixed camera별 refined $\mathbf T_{B,C_i}$ 하나

---

## 15. 단계 12 — 전체 system의 production joint optimization

### 처리 범위

- 전체 fixed camera
- + gripper camera
- + 전체 eligible event
- + 전체 set

여기서 처음으로 주요 global calibration 변수들을 하나의 목적함수에서 함께 조정한다.

### 기본 모드: `fk_fixed`

#### 입력 visual data

- 모든 유효 camera×event의 PnP pose
- $\mathbf T_{C_i,O}^{e}$
- $\mathbf T_{C_i,W}^{e}$

Raw RGB corner가 기본 visual residual은 아니다.

#### 입력 FK와 cube prior

- event별 $\mathbf T_{B,G}^{e}$
- set별 fixed $\mathbf T_{B,O}^{s}$

#### 최적화 변수

fixed camera가 세 대라면:

- $\mathbf T_{B,C_1}$
- $\mathbf T_{B,C_2}$
- $\mathbf T_{B,C_3}$
- $\mathbf T_{G,C_g}$
- $\mathbf T_{B,W}$

각 fixed camera pose는 하나이고, $\mathbf T_{G,C_g}$와 $\mathbf T_{B,W}$도 각각 하나다.

Set별 $\mathbf T_{B,O}^{s}$는 `fk_fixed`에서는 움직이지 않는다.

#### Fixed-camera cube residual

$$
T_{B,C_i}T_{C_i,O}^{e}
\approx T_{B,O}^{s(e)}
$$

#### Gripper-camera cube residual

$$
T_{B,G}^{e}T_{G,C_g}T_{C_g,O}^{e}
\approx T_{B,O}^{s(e)}
$$

#### Board residual

$$
T_{B,C_i}T_{C_i,W}^{e}\approx T_{B,W}
$$

$$
T_{B,G}^{e}T_{G,C_g}T_{C_g,W}^{e}\approx T_{B,W}
$$

#### PnP pose residual의 의미

PnP pose를 base로 연결한 결과와 target pose 사이의:

- rotation 차이 3개
- translation 차이 3개

를 사용한다.

Raw pixel $(\Delta u,\Delta v)$를 직접 비교하는 것이 아니다.

### 오차 처리

| 단계 | 처리 |
|---|---|
| Fixed-board가 초기 anchor에서 25 mm/5° 초과 | 해당 observation hard reject |
| Cross-path가 초기해에서 30 mm/10° 초과 | 해당 pair hard reject |
| 남은 큰 pose residual | Huber soft down-weight |
| Solver 실패 | Joint 후보 전체 reject |
| Objective 미개선 | Joint 후보 전체 reject |
| Global transform 변화 50 mm/15° 초과 | Joint 후보 전체 reject |
| 모든 조건 통과 | 전체 joint 결과 채택 |

### 출력

- fixed camera당 최종 $\mathbf T_{B,C_i}$ 하나
- global $\mathbf T_{G,C_g}$ 하나
- global $\mathbf T_{B,W}$ 하나

### Optional 모드: `reprojection_fk_fixed`

처리 범위와 최적화 변수는 기본 모드와 같다. 차이는 visual residual이다.

- 기본 `fk_fixed`:
- PnP pose의 3D rotation/translation residual
- `reprojection_fk_fixed`:
- Raw RGB corner의 2D $(\Delta u,\Delta v)$ pixel residual

두 모드 모두 set별 cube pose는 FK-aligned pose에 고정한다.

Optional raw-corner 모드에서도 PnP는 초기값과 관측 후보 선별에 도움을 줄 수 있지만, 최종 visual residual은
PnP pose가 아니라 raw corner다.

### `off` 모드

STEP-D까지 계산한 staged 결과를 그대로 사용하고 최종 joint solve를 수행하지 않는다.

---

## 16. 단계 13 — 최종 평가

평가에서는 calibration 행렬을 고정해야 한다.

### 16.1 PnP self-fit reprojection

#### 처리 범위

- 개별 camera×event

#### 사용하는 pose

- 그 이미지로 PnP를 풀어 얻은 $\mathbf T_{C_i,O}^{e}$

#### 의미

- PnP frontend와 cube model이 해당 이미지를 얼마나 잘 맞추는가

최종 multi-camera calibration 정확도를 단독으로 증명하지 않는다.

### 16.2 Held-out calibration reprojection

#### 처리 범위

- 전체 test camera×test event

#### 사용하는 pose

$$
T_{C_i,O}^{e}
=(T_{B,C_i}^{final})^{-1}T_{B,O}^{e,test}
$$

해당 test 이미지로 $\mathbf T_{C_i,O}^{e}$를 다시 PnP fitting하지 않아야 독립적인 calibration-chain 평가가 된다.

#### 출력

- Test corner pixel RMSE/P50/P95

### 16.3 Camera 간 일관성

같은 event를 본 camera pair의 base cube pose를 비교한다.

$$
T_{B,O}^{i,e}=T_{B,C_i}T_{C_i,O}^{e}
$$

$$
T_{B,O}^{j,e}=T_{B,C_j}T_{C_j,O}^{e}
$$

출력은 mm/degree다. GT 없이 계산할 수 있지만 모든 camera의 공통 bias는 찾지 못한다.

### 16.4 외부 GT 정확도

최종 calibration으로 얻은 3D 위치·방향을 calibration에 사용하지 않은 독립 지그/측정값과 비교한다.

- translation error [mm]
- rotation error [degree]

FK-fixed 방법을 같은 FK로만 평가하면 독립 검증이 아니므로 별도 external GT가 필요하다.

### 평가 오차 처리

- 평가 error가 큼
- → FAIL 또는 낮은 성능으로 보고
- → test 데이터를 다시 calibration에 넣어 행렬을 보정하지 않음

Test 결과를 보고 방법을 수정한 뒤 같은 test를 반복 선택하면 test leakage가 된다.

---

## 17. 행렬의 개수와 공유 범위 최종 요약

| 행렬 | 언제 만들어지나 | 몇 개 존재하나 | 무엇을 설명하나 |
|---|---|---|---|
| $\mathbf K_i,\mathbf D_i$ | 사전 intrinsics | camera당 하나 | 해당 camera의 모든 이미지 |
| $\mathbf T_{C_i,O}^{e,c}$ | PnP 후보 | camera×event×candidate | 한 이미지의 cube pose 후보 |
| $\mathbf T_{C_i,O}^{e}$ | 후보 선택 후 | 유효 camera×event당 하나 | 한 이미지의 대표 cube pose |
| $\mathbf T_{B,G}^{e}$ | FK | event당 하나 | event의 gripper pose |
| $\mathbf T_{C_{ref},C_i}$ | 상대 camera 초기화 | fixed camera당 하나 | reference와 camera의 관계 |
| $\mathbf T_{G,C_g}$ | hand-eye | session에 하나 | gripper-camera 설치 변환 |
| $\mathbf T_{B,C_g}^{e}$ | FK+hand-eye | event당 하나 | 움직이는 gripper camera pose |
| $\mathbf T_{B,O}^{e}$ | cube anchor | event당 하나 | event의 cube base pose |
| $\mathbf T_{B,O}^{s}$ | set alignment | set당 하나 | placement set의 cube pose/prior |
| $\mathbf T_{B,C_i}$ | fixed-camera calibration | fixed camera당 하나 | 모든 set/event에서 공유하는 최종 extrinsic |
| $\mathbf T_{B,W}$ | board calibration | 고정 board에 하나 | 모든 event에서 공유하는 board pose |

가장 중요한 예시는 다음이다.

- Fixed cam 2가 3개 set의 100개 event를 관측
- PnP 관측:
- $\mathbf T_{C_2,O}^{0}$, $\mathbf T_{C_2,O}^{1}$, ..., $\mathbf T_{C_2,O}^{99}$
- → 최대 100개
- 최종 fixed-camera calibration:
- $\mathbf T_{B,C_2}$
- → 단 하나

100개의 PnP 관측과 수백 개의 raw corner는 $\mathbf T_{B,C_2}$ 하나를 안정적으로 구하기 위한 반복 측정이다.

---

## 18. 오차 처리 방식 최종 요약

| 처리 | 의미 | 대표 사용 위치 |
|---|---|---|
| Reject | 데이터 또는 solution을 제외 | 검출 실패, PnP threshold, MAD, pose gate |
| Down-weight | 데이터는 유지하고 영향 감소 | Depth penalty, robust average, Huber |
| Optimize | 오차가 줄도록 행렬 수정 | PnP 내부, STEP-D3, STEP-E |
| Diagnostic | 수치만 보고하고 행렬 수정 안 함 | Holdout, 외부 GT, verification |

같은 reprojection error도 위치에 따라 역할이 다르다.

- PnP 내부 reprojection
- → camera×event의 $\mathbf T_{C_i,O}$ 후보 계산
- PnP 이후 self-fit reprojection
- → 후보 reject/rank/weight
- STEP-D3 calibration reprojection
- → camera의 global $\mathbf T_{B,C_i}$ 하나를 직접 보정
- Holdout reprojection
- → 최종 결과 평가만 수행

---

## 19. 현재 depth 사용 여부의 정확한 결론

현재 방식은 depth를 실제로 사용한다.

- Depth 촬영·저장
- → PnP가 예측한 marker plane과 비교
- → capture gate
- → pose candidate ranking
- → observation weighting

하지만 다음 계산의 직접 residual에는 depth가 들어가지 않는다.

- solvePnP의 3D–2D 입력
- STEP-D3 pixel reprojection residual
- production STEP-E pose residual
- optional STEP-E raw-corner pixel residual

따라서 정확한 표현은:

- 우리 방식은 depth를 후보 검증·선택·가중에 사용하는 depth-aware calibration이지만, depth residual을
- 최종 목적함수에 직접 넣어 extrinsic을 푸는 RGB-D joint optimization은 아니다.

---

## 20. 전체 방식을 한 문장으로 설명

- 각 event에서 camera별 RGB marker corner와 알려진 cube 3D corner로 여러 $\mathbf T_{C,O}$ PnP 후보를
- 만들고, PnP reprojection·가시성·depth-plane 품질로 대표 pose를 선택한다. 그 뒤 전체 camera와 여러
- event/set의 pose를 robot FK 및 ChArUco 관측과 연결해 camera별 하나의 $\mathbf T_{B,C_i}$와 global
- $\mathbf T_{G,C_g}$를 계산한다. Fixed camera는 전체 event의 raw corner reprojection으로 미세 조정하고,
- 마지막 production joint solve에서는 set별 FK-aligned cube pose를 고정한 채 PnP pose residual로 전체
- camera와 hand-eye를 공동 보정한다.
