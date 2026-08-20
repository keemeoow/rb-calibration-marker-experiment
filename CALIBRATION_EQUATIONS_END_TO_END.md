# 우리 캘리브레이션 방식의 End-to-End 수식 정리

이 문서는 현재 저장소의 실제 파이프라인을 기준으로, RGB·depth·로봇 FK가 어떤 순서로 사용되어
고정 카메라 외부 파라미터와 hand-eye 변환을 만드는지 정리한다.

핵심 구분은 다음과 같다.

- **RGB marker corner**: 2차원 픽셀 관측 `(u,v)`이다.
- **Depth**: 각 픽셀에 대응하는 카메라 z축 방향 거리 `z`이다.
- **Target model**: 큐브나 보드 좌표계에서 미리 알고 있는 3차원 코너 `(X,Y,Z)`이다.
- **PnP**: 알려진 3D 코너와 관측된 2D 픽셀로 `T_camera_target`을 구한다.
- **Reprojection**: 3D 코너를 추정 변환과 카메라 모델로 다시 2D에 투영한다.
- **Production joint solve**: 기본적으로 PnP가 만든 3D pose residual을 사용한다.
- **Optional corner joint solve**: PnP pose 대신 raw 2D corner residual을 직접 사용한다.

---

## 0. 기호와 좌표계

### 0.1 좌표계

| 기호 | 의미 |
|---|---|
| `B` | 로봇 base 좌표계 |
| `G` | 로봇 gripper 좌표계 |
| `C_g` | gripper camera 좌표계 |
| `C_i` | `i`번째 fixed camera 좌표계 |
| `O` | AprilTag cube object 좌표계 |
| `W` | ChArUco board 좌표계 |

### 0.2 변환 표기

`T_A_B`는 B 좌표의 점을 A 좌표로 옮기는 4×4 rigid transform이다.

```math
\mathbf X_A = \mathbf T_{A,B}\mathbf X_B
```

```math
\mathbf T_{A,B} =
\begin{bmatrix}
\mathbf R_{A,B} & \mathbf t_{A,B}\\
\mathbf 0^T & 1
\end{bmatrix}
```

- `R`: 3×3 회전행렬
- `t`: 3×1 translation, 코드 내부에서는 기본적으로 metre
- 역변환: `T_B_A = T_A_B^{-1}`
- 연쇄변환: `T_A_C = T_A_B T_B_C`

예를 들어 카메라 기준 큐브 점을 base로 옮기면:

```math
\mathbf X_B
= \mathbf T_{B,C_i}\mathbf T_{C_i,O}\mathbf X_O
```

### 0.3 데이터 차원과 단위

| 값 | 차원 | 단위 |
|---|---:|---|
| RGB corner `u=(u,v)` | 2D | pixel |
| Depth `z(u,v)` | scalar | m 또는 depth unit을 m로 변환 |
| Target corner `X=(X,Y,Z)` | 3D | m |
| Translation | 3D | m, 보고 시 mm |
| Rotation residual | 3D rotvec | rad, 보고 시 degree |
| Reprojection residual | 2D | pixel |

### 0.4 Camera, event, set, session의 차이

이 문서에서 데이터의 범위를 구분하는 index는 다음과 같다.

| 단위 | 기호 | 뜻 | 예시 |
|---|---|---|---|
| Camera | `i` | 물리적인 카메라 한 대 | fixed cam 1, gripper cam |
| Event | `e` | 한 시점의 동기 촬영 | 모든 카메라 이미지와 robot joint 한 묶음 |
| Set | `s` | 같은 cube placement/grasp 조건에 속한 여러 event 묶음 | cube를 한 위치에 놓고 여러 robot pose에서 촬영 |
| Session | 없음 | 전체 calibration 데이터 | 모든 camera × 모든 set × 모든 event |
| Marker | `m` | cube 면에 붙은 marker 하나 | AprilTag ID 2 |
| Corner | `k` | marker의 코너 하나 | 한 marker당 4개 |

관계는 다음과 같다.

```text
Calibration session
 ├─ set 0
 │   ├─ event 0: cam0 RGB/depth, cam1 RGB/depth, ..., robot q0
 │   ├─ event 1: cam0 RGB/depth, cam1 RGB/depth, ..., robot q1
 │   └─ ...
 ├─ set 1
 │   ├─ event 10: 모든 camera 관측 + robot q10
 │   └─ ...
 └─ ...
```

모든 event에서 모든 camera 관측이 반드시 유효한 것은 아니다. 가림, 검출 실패, gate 때문에 일부
`(camera,event)` 조합은 빠질 수 있다.

### 0.5 행렬마다 몇 개 존재하는가

| 행렬 | 개수/공유 범위 | 의미 |
|---|---|---|
| `K_i,D_i` | camera당 하나 | camera 내부 파라미터; 모든 set/event에서 공유 |
| `T_Ci_O^e` | camera×event당 하나 | event `e`에서 camera `i`가 본 cube pose |
| `T_Ci_W^e` | camera×event당 하나 | event `e`에서 camera `i`가 본 board pose |
| `T_B_G^e` | event당 하나 | event `e`의 robot FK gripper pose |
| `T_B_Ci` | fixed camera당 하나 | 고정 카메라의 최종 extrinsic; 전체 session에서 공유 |
| `T_G_Cg` | gripper camera 설치 전체에 하나 | gripper와 wrist camera 사이 hand-eye |
| `T_B_Cg^e` | event당 하나 | 움직이는 gripper camera의 base pose |
| `T_B_W` | 고정 board 하나당 하나 | board가 움직이지 않는다고 가정한 global pose |
| `T_B_O^e` | event당 하나 | event별 cube base pose/anchor |
| `T_B_O^s` | set당 하나 | 같은 placement set에서 공유하는 cube pose/prior |
| `T_Cref_Ci` | fixed camera당 하나 | reference camera와 camera `i` 사이 상대변환 |

그리퍼 카메라의 base pose는 event마다 달라진다.

```math
T_{B,C_g}^{e}=T_{B,G}^{e}T_{G,C_g}
```

그러나 `T_G_Cg` 자체는 카메라가 gripper에 단단히 고정되어 있으므로 session 전체에서 하나이다.

### 0.6 가장 자주 혼동하는 두 행렬

```text
T_Ci_O^e
= camera i, event e마다 달라지는 PnP 관측
```

```text
T_B_Ci
= fixed camera i에 하나만 존재하는 최종 calibration 결과
```

예를 들어 fixed cam 2가 100개 event에서 cube를 봤다면:

```text
T_C2_O^0, T_C2_O^1, ..., T_C2_O^99  → 최대 100개
T_B_C2                                  → 단 하나
```

`T_B_C2` 하나가 모든 set과 event를 설명해야 한다.

### 0.7 실제 저장 데이터와 수식 기호의 대응

| 실제 데이터 | 대표 저장 위치/필드 | 수식 기호 | 범위 |
|---|---|---|---|
| RGB image | `cam{i}/rgb_*.jpg` | `I_i^e` | camera×event당 한 장 |
| Aligned depth | `cam{i}/depth_*.png` | `Z_i^e` | camera×event당 한 장 |
| RGB/depth 경로 | `meta.json`의 camera record | 해당 event 파일 연결 | camera×event |
| Robot joints | `capture_robot_joints_6dof` | `q_e` | event당 하나 |
| Robot pose matrix | `robot_pose_matrix_4x4` 등 | `T_B_G^e` | event당 하나 |
| Set index | `set_index` | `s(e)` | event당 하나 |
| Cube 상태 | `cube_gripped`, `capture_block` | observation mode | event당 하나 |
| Camera intrinsics | `intrinsics/cam{i}.npz`의 `color_K,color_D` | `K_i,D_i` | camera당 하나 |
| Depth scale | 같은 intrinsics 파일 | `s_depth,i` | camera당 하나 |
| Cube geometry | `config.py`/resolved cube config | `X_O^{m,k}`, face mapping | session 공통 |
| Stored PnP | `meta.json`의 `cube_pnp` | `T_Ci_O^e` 후보 | camera×event |
| ChArUco observation | `meta.json`의 `charuco` | `T_Ci_W^e`, 2D corners | camera×event |

Step3는 저장된 PnP를 재사용할 수도 있고 RGB/depth 파일을 다시 읽어 PnP 후보와 depth 지표를 재계산할 수도
있다.

---

## 1. 카메라 내부 파라미터 캘리브레이션

카메라별 내부 파라미터는 다음과 같다.

```math
\mathbf K_i =
\begin{bmatrix}
f_{x,i} & 0 & c_{x,i}\\
0 & f_{y,i} & c_{y,i}\\
0 & 0 & 1
\end{bmatrix}
```

왜곡계수는 OpenCV 표기로 다음 일부 또는 전부를 사용한다.

```math
\mathbf D_i=(k_1,k_2,p_1,p_2,k_3,\ldots)
```

ChArUco 캡처 `j`의 알려진 보드 코너 `X_W^k`와 검출 픽셀 `u_{ij}^k`를 사용해:

```math
(\mathbf K_i^*,\mathbf D_i^*,\{\mathbf T_{C_i,W}^{j*}\})
= \arg\min
\sum_{j,k}
\left\|
\pi(\mathbf K_i,\mathbf D_i,
\mathbf T_{C_i,W}^{j}\mathbf X_W^k)
-\mathbf u_{ij}^k
\right\|_2^2
```

를 푼다. 이후 본 캘리브레이션에서는 `K_i,D_i`를 **고정된 상수**로 사용한다.

---

## 2. 3D 점을 2D 픽셀로 투영하는 식

Reprojection의 중간에 내부계수만 들어가는 것이 아니다. 다음 네 가지가 모두 필요하다.

1. target의 3D 점 `X_O`
2. target을 camera 좌표로 옮기는 외부변환 `T_C_O`
3. 내부 파라미터 `K`
4. 렌즈 왜곡 `D`

### 2.1 Object 3D 점을 camera 3D 점으로 변환

```math
\begin{bmatrix}X_C\\Y_C\\Z_C\\1\end{bmatrix}
=
\mathbf T_{C,O}
\begin{bmatrix}X_O\\Y_O\\Z_O\\1\end{bmatrix}
```

여기까지는 3D→3D 외부변환이다.

### 2.2 Perspective division

```math
x=\frac{X_C}{Z_C},\qquad
y=\frac{Y_C}{Z_C}
```

`(x,y)`는 왜곡 전 normalized image coordinate이다.

### 2.3 렌즈 왜곡

```math
r^2=x^2+y^2
```

대표적인 OpenCV radial+tangential 모델은:

```math
x_d=x(1+k_1r^2+k_2r^4+k_3r^6)
+2p_1xy+p_2(r^2+2x^2)
```

```math
y_d=y(1+k_1r^2+k_2r^4+k_3r^6)
+p_1(r^2+2y^2)+2p_2xy
```

### 2.4 내부 파라미터로 픽셀 생성

```math
\hat u=f_xx_d+c_x,\qquad
\hat v=f_yy_d+c_y
```

전체 projection 함수를 줄여 쓰면:

```math
\hat{\mathbf u}
=\pi(\mathbf K,\mathbf D,\mathbf T_{C,O}\mathbf X_O)
```

따라서 reprojection에는 `K,D`뿐 아니라 **3D 점과 외부변환 `T_C_O`가 반드시 들어간다.**

---

## 3. RGB AprilTag 검출과 cube PnP

AprilTag 검출에서 얻는 것은 각 코너의 2D 픽셀이다.

```math
\mathbf u_{i,e}^{m,k}=(u,v)\quad[\mathrm{px}]
```

- `i`: 카메라
- `e`: capture event
- `m`: marker ID
- `k`: marker corner 번호

큐브 형상 설정에서는 같은 코너의 cube-frame 3D 위치를 알고 있다.

```math
\mathbf X_O^{m,k}=(X,Y,Z)\quad[\mathrm m]
```

PnP는 다음 pose를 구한다.

```math
\widehat{\mathbf T}_{C_i,O}^{e}
=\arg\min_{\mathbf T_{C_i,O}}
\sum_{m,k}
\rho\!\left(
\left\|
\pi(\mathbf K_i,\mathbf D_i,
\mathbf T_{C_i,O}\mathbf X_O^{m,k})
-\mathbf u_{i,e}^{m,k}
\right\|_2^2
\right)
```

PnP의 입력과 출력은 다음과 같다.

| 구분 | 값 |
|---|---|
| 입력 1 | cube model의 3D corner `[m]` |
| 입력 2 | RGB에서 검출한 2D corner `[px]` |
| 입력 3 | `K_i,D_i` |
| 직접적인 depth 입력 | 없음 |
| 출력 | `T_Ci_O`: camera 기준 cube 6DoF pose |

코드에서는:

- 단일 marker: IPPE의 복수 planar pose 후보를 생성
- 충분한 다중 corner: `solvePnPRansac`
- IPPE 내부 선택은 visibility, cheirality, reprojection을 사용하고, 저장된 복수 후보를 Step3에서
  다시 비교할 때는 depth-plane 품질도 후보 rank에 반영

한다.

---

## 4. PnP reprojection error

PnP가 구한 pose로 각 3D 코너를 다시 이미지에 투영한다.

```math
\hat{\mathbf u}_{i,e}^{m,k}
=\pi(\mathbf K_i,\mathbf D_i,
\widehat{\mathbf T}_{C_i,O}^{e}\mathbf X_O^{m,k})
```

코너별 2D residual은:

```math
\mathbf r_{i,e}^{m,k}
=\hat{\mathbf u}_{i,e}^{m,k}-\mathbf u_{i,e}^{m,k}
=
\begin{bmatrix}
\hat u-u\\
\hat v-v
\end{bmatrix}
\quad[\mathrm{px}]
```

코너별 scalar error는:

```math
e_{i,e}^{m,k}=\|\mathbf r_{i,e}^{m,k}\|_2\quad[\mathrm{px}]
```

평균과 RMSE는 서로 다르다.

```math
e_{\mathrm{mean}}
=\frac{1}{N}\sum_k e_k
```

```math
e_{\mathrm{RMSE}}
=\sqrt{\frac{1}{N}\sum_k e_k^2}
```

주의할 점은 이 값이 **해당 이미지의 코너로 PnP를 맞춘 뒤 같은 코너에 측정한 self-fit**이라는 것이다.
작은 PnP reprojection error가 곧바로 정확한 multi-camera extrinsic이나 정확한 mm 위치를 보장하지는 않는다.

---

## 5. Depth 측정과 3D unprojection

Depth image도 배열 형태는 2D이지만, 각 픽셀에 z-depth가 저장된다.

```math
z_{\mathrm{meas}}(u,v)
=d_{\mathrm{raw}}(u,v)s_{\mathrm{depth}}
```

왜곡을 제거한 normalized pixel을 `(x_n,y_n)`이라 하면 카메라 ray는:

```math
\mathbf q(u,v)=
\begin{bmatrix}x_n\\y_n\\1\end{bmatrix}
```

RealSense의 z-depth를 사용하는 3D unprojection은:

```math
\mathbf X_C(u,v)
=z_{\mathrm{meas}}(u,v)
\begin{bmatrix}x_n\\y_n\\1\end{bmatrix}
```

왜곡이 없다고 단순화하면:

```math
X_C=\frac{u-c_x}{f_x}z,\qquad
Y_C=\frac{v-c_y}{f_y}z,\qquad
Z_C=z
```

이 경로는 `pixel + depth + K,D → camera-frame 3D point`이다.

---

## 6. 현재 depth-plane 품질 검사

PnP pose로 marker plane의 camera-frame 원점 `p_0`와 법선 `n`을 계산한다.
픽셀 ray를 `q=[x_n,y_n,1]^T`라 하면 ray와 plane의 교점은:

```math
\mathbf X_{\mathrm{pred}}=t\mathbf q
```

```math
t=\frac{\mathbf n^T\mathbf p_0}{\mathbf n^T\mathbf q}
```

`q_z=1`이므로 예측 z-depth는:

```math
z_{\mathrm{pred}}=t
```

Depth plane residual은:

```math
e_{\mathrm{depth}}(u,v)
=|z_{\mathrm{meas}}(u,v)-z_{\mathrm{pred}}(u,v)|
\quad[\mathrm{mm}]
```

중요한 구분:

- RGB reprojection residual: `(u_pred-u_obs,v_pred-v_obs)`, 단위 pixel
- Depth-plane residual: `z_meas-z_pred`, 단위 mm
- 현재 production 기본 경로에서 depth는 주로 후보 선택과 품질 검사에 사용
- depth point residual은 기본 joint objective의 주 residual이 아님

---

## 7. 같은 큐브로 fixed camera 상대변환 계산

동일 event `e`에서 reference camera와 camera `i`가 같은 큐브를 본다.

```math
\widehat{\mathbf T}_{C_r,O}^{e},\qquad
\widehat{\mathbf T}_{C_i,O}^{e}
```

그러면 camera `i` 좌표를 reference camera 좌표로 바꾸는 후보는:

```math
\widehat{\mathbf T}_{C_r,C_i}^{e}
=\widehat{\mathbf T}_{C_r,O}^{e}
(\widehat{\mathbf T}_{C_i,O}^{e})^{-1}
```

event별 후보를 reprojection 품질로 가중하고 robust SE(3) average한다.

```math
\widehat{\mathbf T}_{C_r,C_i}
=\operatorname{RobustAvg}_{e}
\left(\widehat{\mathbf T}_{C_r,C_i}^{e};w_e\right)
```

현재 상대변환 초기화 가중치의 핵심 형태는:

```math
w_e\propto
\frac{1}{e_{\mathrm{reproj},r,e}\,
e_{\mathrm{reproj},i,e}}
```

---

## 8. 로봇 FK

event `e`의 robot joint vector를 `q_e`라 하면:

```math
\mathbf T_{B,G}^{e}=FK(\mathbf q_e)
```

FK의 입력은 관절값이고 출력은 base 기준 gripper 6DoF pose이다. RGB나 depth는 FK 식에 들어가지 않는다.

---

## 9. Gripper camera hand-eye 초기해

구하려는 고정 변환은:

```math
\mathbf T_{G,C_g}
```

고정된 ChArUco board를 event마다 관측하면:

```math
\mathbf T_{B,W}^{e}
=\mathbf T_{B,G}^{e}
\mathbf T_{G,C_g}
\widehat{\mathbf T}_{C_g,W}^{e}
```

보드는 움직이지 않으므로 모든 event에서 `T_B_W^e`가 동일해야 한다.

```math
\mathbf T_{B,W}^{1}
\approx\mathbf T_{B,W}^{2}
\approx\cdots
```

OpenCV hand-eye는 event 사이 상대 motion을 만들어 고전적인 형태로 푼다.

```math
\mathbf A_{ab}\mathbf X
=\mathbf X\mathbf B_{ab}
```

여기서 `X=T_G_Cg`이고, `A`는 gripper motion, `B`는 camera에서 본 target motion에서 만들어진다.
현재 구현은 여러 hand-eye method 후보를 평가하고 board stability가 좋은 해를 선택한다.

ChArUco가 충분하지 않으면 `T_Cg_W` 대신 cube PnP의 `T_Cg_O`를 fallback으로 사용한다.

---

## 10. Hand-eye의 board+cube nonlinear refinement

후보 `T_G_Cg`가 주어졌을 때 event별 board base pose는:

```math
\mathbf T_{B,W}^{e}(\mathbf T_{G,C_g})
=\mathbf T_{B,G}^{e}
\mathbf T_{G,C_g}
\widehat{\mathbf T}_{C_g,W}^{e}
```

이들이 평균 board pose와 가까워지도록 한다.

```math
E_{\mathrm{board}}
=\sum_e
d_{SE(3)}(mathbf T_{B,W}^{e},\bar{\mathbf T}_{B,W})
```

같은 placement set `s`의 gripper-camera cube 관측도:

```math
\mathbf T_{B,O}^{e}
=\mathbf T_{B,G}^{e}
\mathbf T_{G,C_g}
\widehat{\mathbf T}_{C_g,O}^{e}
```

```math
E_{\mathrm{cube}}
=\sum_s\sum_{e\in s}
\left\|
\mathbf t(\mathbf T_{B,O}^{e})-
\bar{\mathbf t}_{B,O}^{s}
\right\|_2^2
```

현재 staged hand-eye refinement는 board 항과 낮은 가중치의 cube 항을 Huber loss로 함께 줄인다.

---

## 11. Base 기준 board pose

hand-eye가 정해지면 event별 board pose는:

```math
\widehat{\mathbf T}_{B,W}^{e}
=\mathbf T_{B,G}^{e}
\widehat{\mathbf T}_{G,C_g}
\widehat{\mathbf T}_{C_g,W}^{e}
```

이를 reprojection 품질로 가중 평균한다.

```math
\widehat{\mathbf T}_{B,W}
=\operatorname{Avg}_e
(\widehat{\mathbf T}_{B,W}^{e};w_e)
```

```math
w_e\propto\frac{1}{e_{\mathrm{charuco\ reproj},e}}
```

---

## 12. Board를 이용한 fixed camera의 base 등록

fixed camera `i`가 같은 board를 보면:

```math
\widehat{\mathbf T}_{C_i,W}^{e}
```

를 얻는다. 따라서 event별 fixed camera pose 후보는:

```math
\widehat{\mathbf T}_{B,C_i}^{e}
=\widehat{\mathbf T}_{B,W}^{e}
(\widehat{\mathbf T}_{C_i,W}^{e})^{-1}
```

여러 event의 후보를 robust average하여 board 기반 `T_B_Ci`를 얻는다.

---

## 13. Cube를 이용한 fixed camera의 base 등록

base 기준 cube anchor `T_B_O^e`와 fixed camera PnP `T_Ci_O^e`가 있으면:

```math
\widehat{\mathbf T}_{B,C_i}^{e}
=\widehat{\mathbf T}_{B,O}^{e}
(\widehat{\mathbf T}_{C_i,O}^{e})^{-1}
```

여러 event를 robust average한다.

```math
\widehat{\mathbf T}_{B,C_i}
=\operatorname{RobustAvg}_e
(\widehat{\mathbf T}_{B,C_i}^{e};w_e)
```

cube-primary 경로는 cube 기반 값을 중심으로 사용하고 board 기반 값을 refinement/보조 source로 사용한다.

---

## 14. Set cube-center frame과 AprilTag object frame 정렬

로봇에 저장된 nominal set cube-center pose를:

```math
\mathbf T_{B,S}^{s}
```

비전 관측으로 얻은 같은 set의 AprilTag object pose를:

```math
\widehat{\mathbf T}_{B,O}^{s}
```

라고 한다. 두 frame 사이의 set별 offset 후보는:

```math
\boldsymbol{\Delta}_s
=(\mathbf T_{B,S}^{s})^{-1}
\widehat{\mathbf T}_{B,O}^{s}
```

공통 offset은:

```math
\bar{\boldsymbol{\Delta}}
=\operatorname{RobustAvg}_{s}(\boldsymbol{\Delta}_s)
```

정렬된 FK/set prior는:

```math
\widetilde{\mathbf T}_{B,O}^{s}
=\mathbf T_{B,S}^{s}\bar{\boldsymbol{\Delta}}
```

이 값이 production `fk_fixed` joint solve의 고정 cube pose가 된다.

---

## 15. Fixed camera set-consistency refinement

set `s(e)`의 cube pose를 고정하면 각 event의 camera 후보는:

```math
\mathbf T_{B,C_i}^{e}
=\widetilde{\mathbf T}_{B,O}^{s(e)}
(\widehat{\mathbf T}_{C_i,O}^{e})^{-1}
```

이 후보들을 robust average하여 기존 camera pose를 보수적으로 업데이트한다. translation/rotation 변화량 guard를
넘으면 기존 값을 유지한다.

---

## 16. STEP-D3: fixed camera raw-corner reprojection refinement

이 단계에서는 event별 cube pose `T_B_O^e`를 고정하고, 각 fixed camera pose `T_B_Ci`만 최적화한다.

camera 기준 cube pose의 예측은:

```math
\mathbf T_{C_i,O}^{e}
=(\mathbf T_{B,C_i})^{-1}\mathbf T_{B,O}^{e}
```

예측 픽셀은:

```math
\hat{\mathbf u}_{i,e}^{k}
=\pi(\mathbf K_i,\mathbf D_i,
(\mathbf T_{B,C_i})^{-1}
\mathbf T_{B,O}^{e}\mathbf X_O^k)
```

pixel residual은:

```math
\mathbf r_{i,e}^{k}
=\hat{\mathbf u}_{i,e}^{k}-\mathbf u_{i,e}^{k}
```

camera별 목적함수는:

```math
\mathbf T_{B,C_i}^*
=\arg\min_{\mathbf T_{B,C_i}}
\sum_{e,k}\rho_{\mathrm{Huber}}
(\|\mathbf r_{i,e}^{k}\|_2^2)
```

고정값과 미지수는 다음과 같다.

| 구분 | 값 |
|---|---|
| 고정 | `T_B_O^e`, `X_O^k`, `K_i,D_i`, 관측 pixel |
| 미지수 | 해당 fixed camera의 `T_B_Ci` |
| residual | 2D pixel residual |
| 채택 조건 | RMSE 감소 + pose 변화량 guard 통과 |

---

## 17. Production STEP-E: PnP pose 기반 joint `fk_fixed`

### 17.1 최적화 변수와 고정값

최적화 변수:

```math
\Theta=
\{\mathbf T_{B,C_i}\}_{i=1}^{N}
\cup\{\mathbf T_{G,C_g}\}
\cup\{\mathbf T_{B,W}\}
```

고정값:

```math
\{\widetilde{\mathbf T}_{B,O}^{s}\},
\{\mathbf T_{B,G}^{e}=FK(q_e)\},
\{\widehat{\mathbf T}_{C,O}^{e}\},
\{\widehat{\mathbf T}_{C,W}^{e}\}
```

여기서 영상 관측은 raw pixel이 아니라 PnP가 압축한 `T_C_target` pose이다.

### 17.2 SE(3) pose residual

두 pose `A,B`의 차이는:

```math
\mathbf E=\mathbf A^{-1}\mathbf B
```

```math
\mathbf r_{pose}(\mathbf A,\mathbf B)
=
\begin{bmatrix}
\operatorname{Log}_{SO(3)}(\mathbf R_E)/\sigma_r\\
\mathbf t_E/\sigma_t
\end{bmatrix}
```

현재 기본 sigma는 개념적으로 rotation과 translation의 크기를 비교 가능한 normalized residual로 만드는 역할을 한다.

### 17.3 Fixed camera cube residual

```math
\mathbf A_{i,e}^{cube}
=\mathbf T_{B,C_i}\widehat{\mathbf T}_{C_i,O}^{e}
```

```math
\mathbf r_{i,e}^{cube,fixed}
=\mathbf r_{pose}
(\mathbf A_{i,e}^{cube},
\widetilde{\mathbf T}_{B,O}^{s(e)})
```

### 17.4 Gripper camera cube residual

```math
\mathbf A_{e}^{cube,gripper}
=\mathbf T_{B,G}^{e}
\mathbf T_{G,C_g}
\widehat{\mathbf T}_{C_g,O}^{e}
```

```math
\mathbf r_{e}^{cube,gripper}
=\mathbf r_{pose}
(\mathbf A_{e}^{cube,gripper},
\widetilde{\mathbf T}_{B,O}^{s(e)})
```

### 17.5 Fixed camera board residual

```math
\mathbf A_{i,e}^{board}
=\mathbf T_{B,C_i}\widehat{\mathbf T}_{C_i,W}^{e}
```

```math
\mathbf r_{i,e}^{board,fixed}
=\mathbf r_{pose}
(\mathbf A_{i,e}^{board},\mathbf T_{B,W})
```

### 17.6 Gripper camera board residual

```math
\mathbf A_{e}^{board,gripper}
=\mathbf T_{B,G}^{e}
\mathbf T_{G,C_g}
\widehat{\mathbf T}_{C_g,W}^{e}
```

```math
\mathbf r_{e}^{board,gripper}
=\mathbf r_{pose}
(\mathbf A_{e}^{board,gripper},\mathbf T_{B,W})
```

### 17.7 같은 event의 camera cross-path residual

같은 cube를 본 두 경로의 base pose 예측을 `A_e,B_e`라 하면:

```math
\mathbf r_e^{cross}=\mathbf r_{pose}(\mathbf A_e,\mathbf B_e)
```

fixed-fixed 예:

```math
\mathbf A_e=\mathbf T_{B,C_i}\widehat{\mathbf T}_{C_i,O}^{e}
```

```math
\mathbf B_e=\mathbf T_{B,C_j}\widehat{\mathbf T}_{C_j,O}^{e}
```

fixed-gripper 예:

```math
\mathbf B_e=\mathbf T_{B,G}^{e}
\mathbf T_{G,C_g}\widehat{\mathbf T}_{C_g,O}^{e}
```

### 17.8 전체 production 목적함수

관측 그룹 크기 때문에 한 그룹이 자동으로 지배하지 않도록 그룹별 `1/sqrt(N_g)` 정규화를 적용한다.

```math
E_{joint}(\Theta)
=\sum_g\sum_{n\in g}
\rho_{Huber}\left(
\left\|
\frac{w_g}{\sqrt{N_g}}\mathbf r_{g,n}(\Theta)
\right\|_2^2
\right)
```

```math
\Theta^*=\arg\min_{\Theta}E_{joint}(\Theta)
```

solver가 수렴하고 목적함수가 감소하며 각 transform의 변화량 guard를 통과할 때만 결과를 채택한다.

---

## 18. Optional STEP-E: raw-corner `reprojection_fk_fixed`

이 모드의 최적화 변수와 FK-fixed cube pose는 production 방식과 같다. 차이는 영상 residual이다.

### 18.1 Fixed camera에서 target pose 예측

```math
\mathbf T_{C_i,O}^{e}(\Theta)
=(\mathbf T_{B,C_i})^{-1}
\widetilde{\mathbf T}_{B,O}^{s(e)}
```

### 18.2 Gripper camera에서 target pose 예측

```math
\mathbf T_{C_g,O}^{e}(\Theta)
=(\mathbf T_{B,G}^{e}\mathbf T_{G,C_g})^{-1}
\widetilde{\mathbf T}_{B,O}^{s(e)}
```

### 18.3 Board pose 예측

```math
\mathbf T_{C_i,W}^{e}(\Theta)
=(\mathbf T_{B,C_i})^{-1}\mathbf T_{B,W}
```

```math
\mathbf T_{C_g,W}^{e}(\Theta)
=(\mathbf T_{B,G}^{e}\mathbf T_{G,C_g})^{-1}
\mathbf T_{B,W}
```

### 18.4 Raw-corner residual

target이 cube 또는 board일 때 공통으로:

```math
\mathbf r_{i,e}^{k}(\Theta)
=\pi(\mathbf K_i,\mathbf D_i,
\mathbf T_{C_i,target}^{e}(\Theta)
\mathbf X_{target}^{k})
-\mathbf u_{i,e}^{k}
```

목적함수는:

```math
\Theta^*
=\arg\min_{\Theta}
\sum_{i,e,k}
\rho_{soft\_L1}
(\|\mathbf r_{i,e}^{k}(\Theta)\|_2^2)
```

현재 canonical 설정의 핵심은 `soft_l1`, `f_scale=2 px`, SciPy TRF이다.

이 모드는 모든 raw corner가 직접 residual을 만들지만 production 기본값은 아니며 opt-in이다.

---

## 19. FK soft-factor / corrected-FK 실험식

실험 방법에서는 cube pose를 완전히 FK에 고정하는 대신 cube pose를 변수로 두고 FK에 가까워지도록 soft constraint를 줄 수 있다.

estimated cube pose와 FK target pose 사이 residual은:

```math
\mathbf r_{FK}^{s}
=
\operatorname{Log}_{SE(3)}
\left((\mathbf T_{B,O}^{s})^{-1}
\mathbf T_{B,O,FK}^{s}\right)
```

rotation과 translation 순서로 쓰면:

```math
\mathbf r_{FK}^{s}
=
\begin{bmatrix}
\mathbf r_{rot}^{s}\ [rad]\\
\mathbf r_{trans}^{s}\ [m]
\end{bmatrix}
```

FK covariance `Sigma_s`가 있으면 whitening한다.

```math
\widetilde{\mathbf r}_{FK}^{s}
=\mathbf L_s^{-1}\mathbf r_{FK}^{s},
\qquad
\mathbf \Sigma_s=\mathbf L_s\mathbf L_s^T
```

전체 soft-FK 목적함수의 개념형은:

```math
E(\Theta)
=E_{visual}(\Theta)
+\lambda_{FK}\sum_s
\rho(\|\widetilde{\mathbf r}_{FK}^{s}\|_2^2)
```

- `FK-fixed`: `T_B_O^s`를 변수에서 제거하고 FK-aligned pose로 고정
- `soft-FK`: `T_B_O^s`도 움직일 수 있지만 FK residual로 제한
- `visual-only`: FK factor를 제거
- `corrected-FK`: calibration 이후 별도 residual model로 예측을 보정하며 calibration transform 자체와 구분

---

## 20. 평가 지표 수식

### 20.1 PnP self-fit reprojection

```math
e_{pnp,px}
=\operatorname{RMSE}_{k}
\left(
\pi(K,D,\widehat T_{C,O}^{PnP}X_O^k)-u^k
\right)
```

같은 이미지의 코너로 pose를 맞추고 같은 코너에서 재므로 frontend fit이다.

### 20.2 Calibration-chain reprojection

최종 calibration과 독립적으로 주어진 target pose를 사용한다.

```math
e_{calib,px}
=\operatorname{RMSE}_{e,k}
\left(
\pi(K_i,D_i,
(T_{B,C_i}^{calib})^{-1}T_{B,O}^{e}X_O^k)
-u_{i,e}^k
\right)
```

holdout에서는 calibration transform을 고정하고 학습에 사용하지 않은 event corner에 계산해야 한다.

### 20.3 FK 기준 cube 오차

카메라 `i`가 예측하는 base cube pose:

```math
\widehat T_{B,O}^{i,e}
=T_{B,C_i}\widehat T_{C_i,O}^{e}
```

FK 기준 translation error:

```math
e_{FK,trans}^{i,e}
=1000\left\|
\mathbf t(\widehat T_{B,O}^{i,e})
-\mathbf t(T_{B,O,FK}^{e})
\right\|_2\quad[mm]
```

이 지표는 FK를 학습에 쓴 방법에는 독립 GT가 아니다.

### 20.4 Camera 간 cube 일관성

```math
e_{cross,trans}^{i,j,e}
=1000\left\|
\mathbf t(\widehat T_{B,O}^{i,e})
-\mathbf t(\widehat T_{B,O}^{j,e})
\right\|_2\quad[mm]
```

GT 없이 계산 가능하지만 모든 카메라가 같이 틀리는 common bias는 발견하지 못한다.

### 20.5 Rotation error

```math
\mathbf R_{err}=\mathbf R_A^T\mathbf R_B
```

```math
e_{rot}
=\cos^{-1}\left(
\frac{\operatorname{tr}(\mathbf R_{err})-1}{2}
\right)\quad[rad]
```

보고할 때 degree로 변환한다.

### 20.6 Depth-plane error

```math
e_{depth,mm}
=1000\,|z_{meas}-z_{pred}|
```

pixel reprojection과 다른 지표이다.

### 20.7 Cross-camera pixel transfer

Depth 기반으로 camera A 픽셀을 3D로 복원하면:

```math
\mathbf X_{C_A}
=\operatorname{unproject}(\mathbf u_A,z_A,K_A,D_A)
```

camera B로 옮겨 투영한다.

```math
\mathbf X_{C_B}
=\mathbf T_{C_B,C_A}\mathbf X_{C_A}
```

```math
\hat{\mathbf u}_B
=\pi(K_B,D_B,\mathbf X_{C_B})
```

```math
e_{A\rightarrow B,px}
=\|\hat{\mathbf u}_B-\mathbf u_B\|_2
```

동일한 실제 3D point correspondence, visibility, depth validity를 보장해야 의미가 있다.

---

## 21. Reprojection 식에 무엇이 들어가는가: 최종 답

RGB corner reprojection의 가장 축약된 식은:

```math
\boxed{
\hat{\mathbf u}
=\pi(\mathbf K,\mathbf D,
\mathbf T_{C,target}\mathbf X_{target})
}
```

```math
\boxed{
\mathbf r_{px}
=\hat{\mathbf u}-\mathbf u_{observed}
}
```

각 항은 다음과 같다.

| 항 | 의미 | 차원 |
|---|---|---|
| `X_target` | 미리 알고 있는 cube/board corner | 3D |
| `T_C,target` | target을 camera 좌표로 옮기는 외부변환 | 3D pose |
| `K` | 초점거리와 principal point | 내부계수 |
| `D` | 렌즈 왜곡 | 내부계수/왜곡 |
| `u_observed` | RGB에서 검출한 실제 corner | 2D pixel |
| `r_px` | 예측 pixel과 실제 pixel의 차이 | 2D pixel |

**Depth는 이 RGB corner reprojection 식의 직접 입력이 아니다.** Depth를 사용할 때는 다음 두 별도 경로 중 하나이다.

```math
\text{pixel}+\text{depth}+K,D
\rightarrow\text{3D unprojection}
```

또는:

```math
z_{pred}(PnP\ plane)-z_{meas}(depth)
\rightarrow\text{depth-plane residual}
```

---

## 22. 현재 파이프라인을 한 줄씩 요약

```text
1. ChArUco 3D–2D 대응으로 카메라별 K,D 계산 후 고정
2. RGB에서 AprilTag/ChArUco 2D corner 검출
3. 알려진 target 3D corner + RGB 2D corner + K,D로 PnP
4. PnP reprojection과 depth-plane 검사로 관측 품질 평가
5. 같은 cube를 이용해 fixed camera 상대변환 초기화
6. FK와 움직이는 gripper-camera의 board 관측으로 hand-eye 초기화
7. board/cube 공통 anchor로 fixed camera를 robot base에 등록
8. FK set-center frame과 AprilTag cube object frame의 상수 offset 정렬
9. cube pose를 고정하고 raw pixel reprojection으로 fixed camera를 미세 조정
10. 기본 STEP-E에서는 PnP pose residual로 cameras+hand-eye+board를 공동 최적화
11. opt-in STEP-E에서는 raw corner pixel residual로 같은 변수들을 공동 최적화
12. held-out pixel, camera 간 일관성, 독립 외부-GT mm/deg를 서로 분리해 평가
```

---

## 23. 해석 시 반드시 피해야 할 혼동

1. `Depth image가 있다`와 `PnP가 depth를 직접 사용한다`는 같은 말이 아니다.
2. PnP reprojection error가 작다고 multi-camera extrinsic이 정확한 것은 아니다.
3. pixel reprojection error와 3D position error(mm)는 같은 지표가 아니다.
4. camera 간 일관성과 외부 GT 기준 정확도는 같은 지표가 아니다.
5. production `fk_fixed` joint residual은 pose residual이며 raw pixel residual이 아니다.
6. STEP-D3에는 raw pixel reprojection refinement가 이미 있지만, 이후 STEP-E 기본 joint solve는 pose residual을 사용한다.
7. FK-fixed 방법을 같은 FK로 평가하면 독립적인 정확도 증명이 아니므로 외부 지그/GT가 필요하다.

---

## 24. 관련 구현 위치

- Camera intrinsics: `Step1b_charuco_intrinsics.py`
- RGB cube PnP와 PnP reprojection: `apriltag_cube.py`
- Capture-time PnP/depth 기록: `Step2_capture.py`
- Staged calibration, hand-eye, fixed-camera 등록, production joint solve: `Step3_calibration.py`
- Canonical raw-corner joint backend: `calibration_reprojection_backend.py`
- Soft-FK factor: `calibration_fk_factor.py`
- Verification/reporting: `Step4_verify.py`, `Step5_export_reports.py`

---

## 25. 촬영부터 PnP와 reprojection까지 단계별 입력·출력

### 25.1 PnP가 reprojection의 필수 선행 단계인가?

수학적으로는 **아니다**. Reprojection에 반드시 필요한 것은 `T_C,target`이며, 그 pose를 얻는 방법이
꼭 PnP일 필요는 없다.

```math
\hat u=\pi(K,D,T_{C,target}X_{target})
```

`T_C,target`은 다음 중 어느 경로로도 주어질 수 있다.

- 같은 이미지의 3D–2D 대응으로 PnP를 풀어 얻음
- 최종 calibration chain에서 `T_C,target=T_{B,C}^{-1}T_{B,target}`로 계산
- robot FK와 hand-eye chain으로 계산
- 외부 GT 장비에서 제공
- reprojection optimizer 안에서 미지수로 직접 변화

따라서 상황별 답은 다음과 같다.

| Reprojection 종류 | 직전 PnP가 필요한가? |
|---|---|
| PnP self-fit reprojection | 필요. 방금 PnP로 구한 pose를 검사하기 때문 |
| 최종 calibration-chain reprojection | 필수 아님. 최종 extrinsic과 target pose로 계산 가능 |
| Raw-corner joint optimization | 필수 아님. optimizer의 현재 pose 후보를 매 반복마다 투영 |
| Depth-plane check | 현재 구현에서는 PnP pose가 예측 plane을 만들기 때문에 필요 |

### 25.2 단계 0 — 센서 동기 촬영

| 입력 | 처리 | 출력 |
|---|---|---|
| RealSense 장치 | RGB와 color-aligned depth 촬영 | RGB `I_i^e`, depth `Z_i^e`, timestamp |
| Robot joint encoder | 관절값 읽기 | `q_e` |

RGB는 `(H,W,3)`, depth는 `(H,W)` 배열이다. Depth는 기본적으로 저장된다.

### 25.3 단계 1 — 사전 calibration 값 로드

| 입력 | 처리 | 출력 |
|---|---|---|
| intrinsics 파일 | camera별 값 읽기 | `K_i`, `D_i`, `s_depth,i` |
| joint `q_e` | robot FK | `T_B_G^e=FK(q_e)` |
| cube 설정 | marker face/크기/부착 자세 구성 | marker corner `X_O^{m,k}` |

### 25.4 단계 2 — RGB marker 검출

| 입력 | 처리 | 출력 |
|---|---|---|
| RGB `I_i^e` | grayscale 변환과 AprilTag/Aruco 검출 | marker IDs, raw 2D corners |
| marker ID와 cube config | corner 순서 정렬, face mapping, aspect 검사 | `u_i,e^{m,k}`와 대응 `X_O^{m,k}` |

이 단계의 핵심 출력은:

```text
object_points: N×3, metre
image_points:  N×2, pixel
```

이다. Depth는 marker의 2D corner 검출에는 들어가지 않는다.

### 25.5 단계 3 — PnP

| 구분 | 값 |
|---|---|
| 입력 | `object_points N×3` |
| 입력 | `image_points N×2` |
| 입력 | `K_i,D_i` |
| PnP 직접 입력이 아닌 것 | depth image, robot FK, 다른 카메라 pose |
| 출력 | `rvec,tvec` |
| 변환 출력 | `T_Ci_O^e` |

다중 corner에서는 RANSAC PnP, 단일 평면 marker에서는 IPPE 후보를 사용한다.

### 25.6 단계 4 — PnP pose로 3D corner 재투영

| 입력 | 처리 | 출력 |
|---|---|---|
| `X_O^{m,k}` | `T_Ci_O^e`로 camera 3D 좌표 변환 | `X_Ci^{m,k}` |
| `X_Ci^{m,k}` | perspective division + `D_i` + `K_i` | 예측 pixel `u_hat_i,e^{m,k}` |

```math
\hat u_{i,e}^{m,k}
=\pi(K_i,D_i,T_{C_i,O}^{e}X_O^{m,k})
```

### 25.7 단계 5 — RGB reprojection error 계산

| 입력 | 처리 | 출력 |
|---|---|---|
| 예측 pixel `u_hat` | 실제 검출 pixel `u`와 뺄셈 | 2D residual `[du,dv]` px |
| 모든 corner residual | norm·통계 | mean, median, P90, RMSE px |

```math
r_{px}^{m,k}=\hat u^{m,k}-u^{m,k}
```

```math
e^{m,k}=\|r_{px}^{m,k}\|_2
```

### 25.8 단계 6 — Depth-plane 검사

| 입력 | 처리 | 출력 |
|---|---|---|
| PnP `T_Ci_O^e` | marker plane을 camera 좌표로 변환 | 예측 marker plane |
| marker polygon 내부 depth | raw depth×depth scale | `z_meas` |
| pixel ray + 예측 plane | ray-plane intersection | `z_pred` |
| `z_meas,z_pred` | 차이 계산 | depth-plane mean/median/max mm |

이 depth 검사는 RGB reprojection error 계산과 별도이다.

### 25.9 단계 7 — Capture gate와 PnP 후보 선택

| 사용 정보 | 현재 역할 |
|---|---|
| marker/face 수 | 많은 face의 pose를 우선 |
| RGB reprojection error | 큰 오차 후보 제거·가중치 감소 |
| depth valid sample 수 | capture 및 후보 품질 검사 |
| depth-plane error | 작은 후보 우선, 큰 후보의 영향 감소 |
| visibility/IPPE 조건 | planar flip 후보 구분 |

선택 결과는 event·camera별 PnP observation이다.

```text
T_Ci_O^e
err_mean_px
used_marker_ids
depth_plane_mean_mm
기타 품질 정보
```

### 25.10 단계 8 — PnP pose들을 calibration에 사용

PnP observation을 이용해:

```text
fixed camera 상대변환
hand-eye 초기값
base 기준 fixed camera pose
event/set cube anchor
```

를 만든다. 이 단계부터 robot FK와 다른 camera 관측들이 결합된다.

### 25.11 단계 9 — Calibration-level reprojection

STEP-D3에서는 PnP 결과로 만든 초기 calibration과 cube anchor를 사용하지만, residual 자체는 raw RGB corner이다.

```math
T_{C_i,O}^{e}(T_{B,C_i})
=T_{B,C_i}^{-1}T_{B,O}^{e}
```

```math
r_{px}
=\pi(K_i,D_i,
T_{B,C_i}^{-1}T_{B,O}^{e}X_O)-u_{observed}
```

여기서는 매 반복마다 새로운 PnP를 푸는 것이 아니다. Optimizer가 `T_B_Ci`를 움직이고 그 pose를 바로
projection하여 pixel error를 계산한다.

### 25.12 현재 depth 사용 여부의 최종 판정

현재 기본 실행에서 depth는 실제로 다음에 사용된다.

1. RGB와 정렬해 촬영하고 PNG로 저장 — 기본 ON
2. PnP pose가 예측한 marker plane과 실제 depth 비교
3. capture gate: 유효 depth sample과 plane error 조건
4. IPPE/PnP 후보 순위 결정
5. multi-camera cube consensus와 camera pose 평균의 observation weight

현재 기본 실행에서 depth가 직접 사용되지 않는 곳은 다음과 같다.

1. `solvePnP`의 3D–2D 입력
2. STEP-D3 pixel reprojection residual
3. production STEP-E `fk_fixed`의 SE(3) pose residual
4. optional STEP-E raw-corner pixel residual

따라서 정확한 표현은 다음과 같다.

> 현재 캘리브레이션은 depth-aware이다. Depth가 관측 선택과 가중치를 바꾸므로 최종 결과에 간접적으로
> 영향을 준다. 그러나 depth residual을 최종 calibration objective에 직접 넣어 extrinsic을 푸는
> RGB-D joint optimization은 아니다.

---

## 26. 단계별 오차 처리: 폐기, 가중치 감소, 보정, 진단

오차가 발견됐을 때의 처리는 네 종류로 구분해야 한다.

| 처리 | 의미 |
|---|---|
| **Reject** | 해당 corner, marker, pose, event를 계산에서 제외 |
| **Down-weight** | 데이터는 남기되 최종 결과에 주는 영향 감소 |
| **Optimize** | 오차를 residual로 사용해 `K,D` 또는 pose를 수정 |
| **Diagnostic** | 수치만 기록하고 현재 결과는 수정하지 않음 |

여기서 “보정”은 일반적으로 원본 RGB/depth 값을 고치는 것이 아니다. 관측은 그대로 두고, 오차가 작아지도록
`K,D,T_B_Ci,T_G_Cg` 같은 **모델 파라미터를 수정**한다.

### 26.1 전체 요약표

| 단계 | 감지하는 문제/오차 | 현재 처리 | 데이터를 버리는가? | 보정에 쓰는가? |
|---|---|---|---|---|
| Intrinsics 수집 | corner 부족, view 부족 | calibration 생략, factory 값 유지 | 해당 camera 보정 생략 | 조건 충족 view만 `K,D` 보정에 사용 |
| Intrinsics 1차 결과 | view별 reprojection이 `mean+std` 초과 | 해당 view 제거 후 2차 calibration | 예, view 단위 | 남은 view로 `K,D` 재보정 |
| Marker 검출 | 알 수 없는 ID | marker 제외 | 예, marker 단위 | 아니오 |
| Marker 기하검사 | aspect가 기준 미달 | marker/candidate 제외 | 예 | 아니오 |
| Multi-corner PnP | 일부 corner가 모델과 불일치 | RANSAC이 inlier subset으로 pose 계산 | 내부적으로 outlier 영향 제거 | 남은 correspondence로 pose 계산 |
| Single-plane PnP | IPPE 복수해/flip | 먼저 물리 가능성·reprojection으로 선택; 저장된 복수 후보의 Step3 비교에는 depth도 반영 | 선택되지 않은 후보 폐기 | 아니오, 후보 선택에 사용 |
| PnP reprojection | mean error가 threshold 초과 | PnP observation 후보 제외 | 예, pose 단위 | PnP 내부에서는 pose 계산 목적함수로 이미 사용 |
| Capture gate | camera 수·reprojection·depth·timestamp 불량 | capture 전체 저장 거부 | 예, event 단위 | 아니오 |
| Depth-plane 검사 | 측정 depth와 PnP plane 불일치 | capture gate, 후보 rank, weight에 반영 | 조건에 따라 reject | 직접 pose 보정은 아니며 간접 영향 |
| Candidate selection | 후보가 여러 개 | marker 수→depth→reprojection 순으로 우선순위 | 선택되지 않은 후보 제외 | 아니오 |
| Camera pose 평균 | 불안정한 PnP pose | robust average 및 낮은 weight | 일부는 outlier 취급 | 남은 pose로 초기 extrinsic 계산 |
| Hand-eye 초기해 | event별 board pose 불일치 | MAD outlier event를 빼고 다시 계산 | 재계산에서는 예 | 남은 event로 `T_G_Cg` 보정 |
| Hand-eye nonlinear refine | board/cube pose consistency residual | Huber로 큰 residual 영향 감소 | 원칙적으로 유지 | `T_G_Cg` 최적화에 사용 |
| Set-prior gate | FK prior와 visual anchor 불일치 | prior를 채택하지 않고 visual anchor 유지 | prior만 거절 | 조건 통과 prior는 camera refinement에 사용 |
| Set consistency refine | 새 camera pose가 초기값과 차이 | full/partial/reject adoption | 관측 자체보다 결과 후보를 제한 | camera pose 보정 |
| STEP-D3 reprojection | raw corner pixel residual | Huber 최적화 | 큰 residual은 soft down-weight | `T_B_Ci` 보정에 직접 사용 |
| STEP-D3 adoption | RMSE 미개선 또는 20 mm/3° 초과 | 최적화 결과 전체 거절, 이전 pose 유지 | 결과 후보를 버림 | 조건 통과 때만 채택 |
| STEP-E pose gate | board/cross-path가 초기해와 과도하게 불일치 | 해당 pose/pair 제외 | 예, observation/pair 단위 | 제외되지 않은 데이터만 사용 |
| STEP-E robust solve | pose residual이 큼 | Huber로 영향 감소 | 바로 버리지는 않음 | joint transforms 보정 |
| STEP-E adoption | objective 미개선, 실패, 50 mm/15° 초과 | joint 결과 전체 거절 | 결과 후보를 버림 | 조건 통과 때만 최종 채택 |
| Optional corner joint | raw pixel residual이 큼 | soft-L1로 영향 감소 | 바로 버리지는 않음 | joint transforms 보정 |
| Verification | test metric이 큼 | PASS/FAIL 및 보고 | calibration 데이터 변경 없음 | 아니오 |

### 26.2 RGB marker 검출 실패

marker가 검출되지 않거나 cube config에 없는 ID이면 해당 marker는 correspondence를 만들지 못한다.

```text
검출 실패 marker → 해당 marker 제외
남은 marker가 최소 개수 미달 → 해당 camera의 PnP 없음
필수 camera 수 미달 → capture event 전체 거절
```

검출되지 않은 corner를 추정해 채워 넣지는 않는다.

### 26.3 PnP와 reprojection threshold

다중 corner PnP의 RANSAC 내부 기준은 현재 5 px이다. PnP 이후에는 모든 사용 corner의 평균
reprojection error를 다시 계산한다.

```math
e_{mean}>\tau_{pnp}
\quad\Rightarrow\quad
\text{pose candidate reject}
```

주요 기본 기준은 서로 적용 시점이 다르다.

| 위치 | 기본 기준 | 동작 |
|---|---:|---|
| Capture gate | mean reprojection ≤ 2 px | gate-quality PnP로 인정 |
| Step3 후보 생성, fixed | mean reprojection ≤ 3 px | 초과 후보 제외 |
| Step3 후보 생성, gripper | mean reprojection ≤ 5 px | 초과 후보 제외 |
| Step3 fixed role filter | mean reprojection ≤ 2.75 px | 초과 fixed 후보 제외 |

따라서 “reprojection error를 계산만 한다”가 아니라, PnP pose의 통과 여부와 weight에도 사용한다.

### 26.4 Depth 오차 처리

Depth는 상황에 따라 reject와 down-weight를 모두 수행한다.

#### Capture 단계

- Depth 저장 기본값: ON
- 최소 유효 depth sample 기본값: 20
- A-placement: gripper depth-valid를 기본 요구
- A-placement gripper depth-plane mean 기본 상한: 40 mm
- B-eye-to-hand: depth-quality fixed camera를 기본 1대 이상 요구
- B-eye-to-hand fixed depth-plane mean 기본 상한: 20 mm

이 조건을 만족하지 않으면 해당 capture를 저장하지 않을 수 있다.

#### Step3 후보 선택과 평균 단계

Depth가 유효하면 plane error가 작은 pose를 우선한다. Pose observation weight의 개념형은:

```math
w\propto
\frac{N_{marker}^2}{e_{reproj}}
\cdot
\frac{\text{depth support}}{1+e_{depth}/3}
```

Depth가 없으면 일부 경로에서 weight를 낮춘다. 즉 Step3에서는 나쁜 depth가 항상 pose를 즉시 삭제하는
것은 아니며, 후보 선택 또는 영향 감소로 처리될 수도 있다.

Depth error로 `T_B_Ci`를 직접 미분하여 수정하지는 않는다.

### 26.5 Hand-eye outlier 처리

초기 hand-eye로 event별 stationary-target base pose를 계산한다.

```math
T_{B,target}^{e}=T_{B,G}^{e}T_{G,C_g}T_{C_g,target}^{e}
```

평균 target pose에서의 translation residual에 대해:

```math
\tau=\operatorname{median}(e)
+2(1.4826)\operatorname{MAD}(e)
```

```math
e_e>\tau\Rightarrow\text{해당 event를 재계산 subset에서 제외}
```

단, 너무 많은 event를 버리지 않도록 최소 8개 및 전체의 60% 이상을 유지한다. 이후 board+cube nonlinear
refinement에서는 Huber loss를 사용해 큰 residual의 영향만 줄인다.

### 26.6 STEP-D3 pixel reprojection 처리

이 단계에서는 raw corner를 먼저 hard reject하기보다 Huber loss로 큰 residual의 영향력을 줄인다.

```math
\min_{T_{B,C_i}}\sum_{e,k}\rho_{Huber}(r_{px,i,e,k})
```

최적화가 끝난 뒤:

```text
pixel RMSE 감소 AND camera 변화 ≤ 20 mm, 3°
    → 새 camera pose 채택
그 외
    → 최적화 후보를 버리고 기존 camera pose 유지
```

즉 pixel 오차를 **실제 camera pose 보정에 사용**하지만, 위험한 결과는 최종 guard에서 거절한다.

### 26.7 Production STEP-E 처리

최적화 전에 명백한 pose outlier를 hard gate한다.

```text
fixed-camera board: 초기 board anchor와 25 mm 또는 5° 초과 → 제외
cube cross-path: 초기 두 경로가 30 mm 또는 10° 초과 → pair 제외
```

남은 residual은 Huber loss로 joint optimization에 사용한다. 최종 결과는:

```text
solver success
AND objective 감소
AND 각 주요 transform 변화 ≤ 50 mm, 15°
```

일 때만 채택한다. 아니면 STEP-D까지의 기존 결과를 유지한다.

### 26.8 평가 단계의 오차

Holdout reprojection, camera 간 일관성, 외부-GT error는 원칙적으로 **결과를 평가하는 값**이다.

```text
평가 error가 큼 → FAIL 또는 성능 저하로 보고
평가 데이터를 다시 학습에 투입해 pose 수정 → 하면 안 됨
```

평가 오차를 보고 threshold나 모델을 바꾼 뒤 같은 test set을 다시 평가하면 test leakage가 생긴다.

---

## 27. 범위와 개수를 포함한 전체 파이프라인 기준표

이 절은 앞 절의 수식을 실제 데이터 처리 순서와 연결한 최종 기준표이다.

### 27.1 Stage 0 — Camera intrinsics calibration

**처리 범위:** camera 한 대씩 독립적으로 처리하되, 해당 camera의 여러 ChArUco view를 한꺼번에 사용한다.

| 항목 | 내용 |
|---|---|
| 입력 범위 | 개별 camera × 여러 intrinsics view |
| 입력 데이터 | ChArUco board 3D corners, 각 view의 RGB 2D corners |
| 고정값 | board geometry |
| 1차 미지수 | `K_i,D_i`, view별 `T_Ci_W^j` |
| 출력 | camera `i`에 하나의 `K_i,D_i` |
| 출력 공유 범위 | 그 camera의 모든 calibration set/event |
| 오차 | view별 pixel reprojection error |
| 오차 처리 | `mean+std` 초과 view 제거 후 2차 calibration |
| 보정되는 값 | `K_i,D_i`; 원본 RGB corner는 수정하지 않음 |

최종적으로 camera가 4대면 `K,D`도 네 벌이다.

### 27.2 Stage 1 — Synchronized capture

**처리 범위:** event 하나를 만들 때 가능한 모든 camera와 robot state를 같은 묶음으로 저장한다.

| 입력 | 출력 |
|---|---|
| 전체 연결 camera | `I_i^e`: camera별 RGB 한 장 |
| 전체 연결 camera | `Z_i^e`: camera별 aligned depth 한 장 |
| robot joint encoder | `q_e` |
| camera/host clock | camera별 timestamp |
| capture plan | `set_index=s(e)`, placement/gripped 상태 |

출력 cardinality:

```text
event e 하나
 ├─ camera 수만큼 RGB/depth 후보
 ├─ robot q_e 하나
 └─ set index 하나
```

오차 처리:

- timestamp span, blur, clipping, 필수 관측 수, PnP/depth 조건 실패 시 event 전체 저장 거절 가능
- 특정 camera frame이 없는 경우 downstream에서 그 `(i,e)` 관측만 존재하지 않을 수 있음

### 27.3 Stage 2 — Marker detection and correspondence

**처리 범위:** `(camera i, event e)` 한 쌍씩 독립적으로 실행한다.

| 항목 | 내용 |
|---|---|
| 입력 범위 | 개별 camera × 개별 event |
| 입력 데이터 | RGB `I_i^e`, cube config |
| 입력하지 않는 데이터 | 다른 camera RGB, robot FK, depth 값 |
| 출력 1 | marker IDs |
| 출력 2 | `u_i,e^{m,k}`: 검출 2D corner `[px]` |
| 출력 3 | `X_O^{m,k}`: 대응하는 model 3D corner `[m]` |

`M`개 marker가 모두 네 corner를 제공하면 보통:

```math
N=4M
```

```text
object_points.shape = (N,3)
image_points.shape  = (N,2)
```

두 배열의 같은 행은 동일한 실제 corner이다.

오차 처리:

- unknown marker, aspect 불량 marker는 제외
- 검출 corner 값을 기하학적으로 “보정해 생성”하지는 않음
- 남은 correspondence가 부족하면 해당 `(i,e)` PnP를 만들지 않음

### 27.4 Stage 3 — PnP candidate generation

**처리 범위:** `(camera i, event e)`별로 독립 실행한다.

| 항목 | 내용 |
|---|---|
| 입력 범위 | 개별 camera × 개별 event |
| 입력 | `N×3 object_points`, `N×2 image_points`, 그 camera의 `K_i,D_i` |
| 직접 입력하지 않음 | robot FK, 다른 camera pose, depth image |
| 내부 목적함수 | 3D corner를 투영한 pixel과 검출 pixel의 차이 최소화 |
| 출력 후보 | `rvec,tvec,T_Ci_O^{e,c}` |
| 후보 index `c` | multi-marker, single-marker, IPPE solution, 저장/재검출 source 등을 구분 |

여기서 행렬은 아직 camera당 하나가 아니다.

```text
cam2, event7
 ├─ candidate 0: T_C2_O^(7,0)
 ├─ candidate 1: T_C2_O^(7,1)
 └─ candidate 2: T_C2_O^(7,2)
```

PnP 내부에서는 reprojection residual로 pose를 계산하고, RANSAC은 큰 residual correspondence의 영향을
제거한다.

### 27.5 Stage 4 — PnP self-fit reprojection measurement

**처리 범위:** `(camera,event,candidate)` 하나씩 계산한다.

| 항목 | 내용 |
|---|---|
| 입력 | `T_Ci_O^{e,c}`, 같은 PnP의 `X_O^k`, `K_i,D_i`, 실제 `u_i,e^k` |
| 출력 | corner별 `[du,dv]`, scalar pixel error, mean/median/P90 |
| 행렬 개수 변화 | 없음; 후보를 측정할 뿐 |
| 오차 사용 | threshold reject, candidate ranking, downstream weight |

Stage 3과 Stage 4는 같은 residual을 사용하지만 역할이 다르다.

```text
Stage 3: 그 residual을 줄여 pose 후보를 계산
Stage 4: 계산이 끝난 후보의 residual을 명시적으로 다시 측정·기록
```

Stage 4의 측정값을 다시 Stage 3에 넣어 별도 PnP를 반복하는 것은 아니다.

### 27.6 Stage 5 — Depth-plane check

**처리 범위:** `(camera,event,PnP candidate)`별로 marker polygon 내부의 여러 depth pixel을 사용한다.

| 항목 | 내용 |
|---|---|
| 입력 범위 | 개별 camera × 개별 event × pose candidate |
| 입력 | aligned depth `Z_i^e`, depth scale, `K_i,D_i`, candidate `T_Ci_O^{e,c}` |
| 비교 | PnP가 예측한 marker-plane z vs 측정 depth z |
| 출력 | valid sample 수, plane mean/median/max mm, inlier ratio |
| pose를 직접 수정하는가 | 아니오 |
| 결과 사용 | capture gate, candidate rank, observation weight |

Depth는 노이즈가 있으므로 독립 GT로 쓰지 않고 다수 pixel의 보조 consistency check로 사용한다.

### 27.7 Stage 6 — Candidate selection

**처리 범위:** `(camera i,event e)` 안에 있는 모든 후보를 비교한다.

| 항목 | 내용 |
|---|---|
| 입력 범위 | 개별 camera × 개별 event의 후보 전체 |
| 입력 | `T_Ci_O^{e,c}` 후보들, marker/face 수, reprojection, depth, source |
| 출력 | 대표 `T_Ci_O^e` 하나 + 진단용 `_candidates` 목록 |
| 출력 개수 | 유효한 `(camera,event)`당 대표 pose 최대 하나 |
| 제외 | 순위가 낮거나 threshold를 넘은 후보 |

예를 들어 camera 4대, 유효 event 30개라면 이론적 최대 대표 PnP 관측은:

```math
4\times30=120
```

개다. 실제로는 가림과 gate 때문에 더 적다.

### 27.8 Stage 7 — Capture/event gate

**처리 범위:** event 하나 안의 전체 camera 결과를 함께 검사한다.

| 항목 | 내용 |
|---|---|
| 입력 범위 | 동일 event의 전체 camera |
| 입력 | camera별 marker 수, PnP 성공, reprojection, depth, timestamp, ROI 품질 |
| 출력 | event accept/reject와 이유 |
| reject 영향 | event 전체가 calibration session에서 빠질 수 있음 |

PnP 자체는 `(camera,event)`별 독립 계산이지만, capture 저장 판단은 multi-camera event 단위이다.

### 27.9 Stage 8 — Fixed-camera relative initialization

**처리 범위:** reference fixed camera와 각 fixed camera의 모든 공통 event를 사용한다.

| 항목 | 내용 |
|---|---|
| 입력 범위 | camera pair `(C_ref,C_i)` × 공통 event 전체 |
| 입력 | event별 대표 `T_Cref_O^e,T_Ci_O^e` |
| event별 후보 출력 | `T_Cref_Ci^e=T_Cref_O^e(T_Ci_O^e)^-1` |
| 최종 출력 | fixed camera `i`당 `T_Cref_Ci` 하나 |
| 오차 처리 | reprojection/depth 기반 weight + robust SE(3) average |

여기서 처음으로 서로 다른 camera의 PnP 관측이 기하학적으로 연결된다.

### 27.10 Stage 9 — Hand-eye initialization and refinement

**처리 범위:** gripper camera의 모든 유효 event를 함께 사용한다. ChArUco를 우선하고 부족하면 cube PnP를
사용한다.

| 항목 | 내용 |
|---|---|
| 입력 범위 | gripper camera 하나 × eligible event 전체 |
| 입력 | event별 `T_B_G^e`, `T_Cg_W^e` 또는 fallback `T_Cg_O^e` |
| 출력 | session 전체에 하나의 `T_G_Cg` |
| 보조 출력 | event별 `T_B_target^e`, 평균 `T_B_W` |
| 오차 처리 | MAD event reject, Huber down-weight, 개선된 solution만 채택 |

`T_G_Cg`는 event별로 따로 만들지 않는다. 움직이는 것은 `T_B_G^e`이며 설치 변환 `T_G_Cg`는 하나다.

### 27.11 Stage 10 — Fixed cameras를 robot base에 등록

**처리 범위:** fixed camera 하나씩, 그 camera가 board/cube와 겹치는 모든 event를 사용한다.

| 항목 | 내용 |
|---|---|
| 입력 범위 | 개별 fixed camera × 전체 overlapping event/set |
| board 경로 입력 | `T_B_W^e,T_Ci_W^e` |
| cube 경로 입력 | `T_B_O^e,T_Ci_O^e` |
| event별 camera 후보 | `T_B_Ci^e=T_B_target^e(T_Ci_target^e)^-1` |
| 최종 출력 | fixed camera `i`당 `T_B_Ci` 하나 |
| 오차 처리 | 후보 filter, depth/reprojection weight, robust average, source merge |

camera가 세 대면 `T_B_C1,T_B_C2,T_B_C3` 세 개가 나온다. Set마다 별도의 fixed-camera 행렬을 만들지 않는다.

### 27.12 Stage 11 — Set prior alignment

**처리 범위:** 전체 set을 함께 사용해 frame offset 하나를 학습하고, set별 pose를 만든다.

| 항목 | 내용 |
|---|---|
| 입력 범위 | session의 전체 유효 set |
| 입력 | set별 nominal `T_B_S^s`, visual estimate `T_B_O^s` |
| 중간 출력 | set별 offset 후보 `Delta_s` |
| global 출력 | session에 하나의 평균 frame offset `Delta_bar` |
| set별 출력 | `T_B_O_prior^s=T_B_S^s Delta_bar` |
| 출력 개수 | global offset 하나 + set 수만큼 cube prior |
| 오차 처리 | prior gate 실패 시 해당 prior를 강제하지 않고 visual anchor 유지 |

`T_B_O^s`는 set마다 하나지만 `T_B_Ci`는 fixed camera마다 하나라는 차이가 중요하다.

### 27.13 Stage 12 — Set-consistency camera refinement

**처리 범위:** fixed camera 하나와 그 camera가 관측한 전체 event/set을 사용한다.

| 항목 | 내용 |
|---|---|
| 입력 범위 | 개별 fixed camera × 전체 유효 event/set |
| 입력 | event별 `T_Ci_O^e`, set별 `T_B_O^s` |
| event별 camera 후보 | `T_B_O^{s(e)}(T_Ci_O^e)^-1` |
| 출력 | fixed camera당 수정된 `T_B_Ci` 하나 |
| 오차 처리 | robust average; 변화량에 따라 full/partial/reject adoption |

여기에서도 event별 fixed-camera 행렬을 최종 출력으로 유지하지 않는다. 모든 event는 한 camera 행렬을 추정하기
위한 반복 측정이다.

### 27.14 Stage 13 — STEP-D3 raw-corner reprojection refinement

**처리 범위:** fixed camera 하나씩 독립적으로 최적화하되, 해당 camera의 모든 유효 placement event corner를
한꺼번에 사용한다.

| 항목 | 내용 |
|---|---|
| 입력 범위 | 개별 fixed camera × 전체 유효 placement event/set |
| 입력 관측 | 모든 event의 raw RGB corner `u_i,e^k` |
| 고정값 | event별 `T_B_O^e`, `K_i,D_i`, target 3D corner |
| 초기값 | 앞 단계의 `T_B_Ci` 하나 |
| 미지수 | 그 fixed camera의 `T_B_Ci` 하나 |
| 출력 | refined `T_B_Ci` 후보 하나 |
| residual | 전체 event corner의 pixel residual |
| 오차 처리 | Huber down-weight; RMSE 개선 및 20 mm/3° guard 통과 시 채택 |

이 단계에서 절대로 다음처럼 만들지 않는다.

```text
event 1용 T_B_Ci
event 2용 T_B_Ci
event 3용 T_B_Ci
```

대신:

```text
모든 event → fixed camera i의 T_B_Ci 하나
```

를 보정한다.

### 27.15 Stage 14 — Production STEP-E `fk_fixed` joint solve

**처리 범위:** 전체 fixed camera, gripper camera, 모든 eligible event/set을 하나의 목적함수에서 처리한다.

| 구분 | 범위와 개수 |
|---|---|
| 입력 visual observation | 유효한 모든 `(camera,event)`의 PnP target pose |
| 입력 FK | event별 `T_B_G^e` |
| 고정 cube prior | set별 `T_B_O^s` |
| 미지수 1 | fixed camera마다 `T_B_Ci` 하나 |
| 미지수 2 | session 전체에 `T_G_Cg` 하나 |
| 미지수 3 | 고정 board에 `T_B_W` 하나 |
| 출력 | 위 global transforms의 공동 보정 결과 |
| residual | PnP pose를 base로 연결한 3D rotation/translation residual |

예를 들어 fixed camera가 세 대이면 joint 변수 family는:

```text
T_B_C1, T_B_C2, T_B_C3,
T_G_Cg,
T_B_W
```

이다. `T_B_O^s`는 set마다 존재하지만 `fk_fixed`에서는 움직이지 않는다.

오차 처리:

- 명백한 board/cross-path pose는 hard gate
- 남은 큰 residual은 Huber down-weight
- objective가 개선되지 않거나 global transform 변화 guard를 넘으면 joint solution 전체를 거절

### 27.16 Stage 15 — Optional raw-corner joint solve

**처리 범위와 global 변수는 Stage 14와 동일**하고 visual residual만 다르다.

| 기본 `fk_fixed` | `reprojection_fk_fixed` |
|---|---|
| 입력 visual data: PnP pose | 입력 visual data: raw 2D corner |
| residual: 6DoF pose difference | residual: `[du,dv]` pixel difference |
| cube prior: set별 fixed | cube prior: set별 fixed |
| 전체 camera를 joint 처리 | 전체 camera를 joint 처리 |

PnP는 optional corner solve의 좋은 초기값과 관측 선별에 여전히 도움을 줄 수 있지만, 최종 visual residual 자체는
PnP pose가 아니라 raw corner이다.

### 27.17 Stage 16 — Evaluation

평가 단위를 명확히 구분해야 한다.

| 평가 | 입력 범위 | 사용하는 행렬 | 출력 |
|---|---|---|---|
| PnP self-fit | 개별 `(camera,event)` | 그 이미지로 구한 `T_Ci_O^e` | px |
| Held-out calibration reprojection | 전체 test camera×event | frozen `T_B_Ci`, 독립 `T_B_O^e` | px |
| Camera cross consistency | 같은 event를 본 camera pair 전체 | camera별 global `T_B_Ci` + PnP pose | mm/deg |
| FK 기준 오차 | camera×event 전체 | visual base pose vs FK pose | mm/deg |
| 외부 GT 정확도 | test pose 전체 | final calibration vs independent GT | mm/deg |

평가 데이터는 calibration 행렬을 다시 수정하는 데 사용하지 않는다.

### 27.18 단계별 범위만 압축한 표

| Stage | 계산 단위 | 여러 camera를 함께 보나? | 여러 set/event를 함께 보나? | 대표 출력 |
|---|---|---:|---:|---|
| Intrinsics | camera 하나 | 아니오 | 여러 view | camera당 `K,D` 하나 |
| Capture | event 하나 | 예 | 아니오 | event data bundle |
| Detection | camera×event | 아니오 | 아니오 | 2D corners |
| PnP candidates | camera×event | 아니오 | 아니오 | 복수 `T_Ci_O^{e,c}` |
| Candidate selection | camera×event | 아니오 | 아니오 | 대표 `T_Ci_O^e` 하나 |
| Capture gate | event | 예 | 아니오 | accept/reject |
| Relative fixed cams | camera pair | 두 대 | 공통 event 전체 | pair당 `T_Cref_Ci` 하나 |
| Hand-eye | gripper camera | 주로 한 대 | event 전체 | global `T_G_Cg` 하나 |
| Base registration | fixed camera 하나 | anchor 생성 시 결합 | event/set 전체 | camera당 `T_B_Ci` 하나 |
| Set alignment | set 전체 | 간접적으로 예 | 전체 set | set당 `T_B_O^s` + offset 하나 |
| D3 reprojection | fixed camera 하나 | 아니오 | 해당 camera의 event 전체 | camera당 `T_B_Ci` 하나 |
| STEP-E joint | 전체 system | 예 | eligible event/set 전체 | 모든 global transforms |
| Evaluation | metric에 따라 다름 | 일부는 예 | test 전체 | px 또는 mm/deg |
