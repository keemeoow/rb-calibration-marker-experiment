# 현재 캘리브레이션 알고리즘의 End-to-End 수식

이 문서는 현재 `Step2_capture.py`–`Step5_export_reports.py` 실행 경로의 수식을 정리한다. 실험용 Table-1
soft-FK ablation 전체를 설명하는 문서가 아니라 README의 production/engineering pipeline 문서다.

수식은 Markdown Preview 호환성을 위해 inline은 `$...$`, 여러 줄은 `$$...$$`만 사용한다. 수식을 인용문이나
코드 블록 안에 넣지 않는다.

---

## 0. 좌표계와 표기

| 기호 | 의미 |
|---|---|
| $B$ | robot base frame |
| $G$ | robot gripper/end-effector frame |
| $C_i$ | fixed camera $i$의 RGB optical frame |
| $C_g$ | gripper camera의 RGB optical frame |
| $D_i$ | camera $i$의 depth optical frame |
| $O$ | AprilTag cube object frame |
| $W$ | ChArUco board frame |
| $S_s$ | set $s$에 저장된 nominal cube-center frame |
| $e$ | event index |
| $s(e)$ | event $e$가 속한 set |
| $m,k,c$ | marker, corner, PnP-candidate index |

$\mathbf T_{A,B}$는 $B$ 좌표의 homogeneous point를 $A$ 좌표로 옮긴다.

$$
\overline{\mathbf X}_A
=\mathbf T_{A,B}\overline{\mathbf X}_B
$$

$$
\mathbf T_{A,B}
=
\begin{bmatrix}
\mathbf R_{A,B} & \mathbf t_{A,B}\\
\mathbf 0^{\mathsf T} & 1
\end{bmatrix}
\in SE(3)
$$

$$
\mathbf T_{B,A}=\mathbf T_{A,B}^{-1},
\qquad
\mathbf T_{A,C}=\mathbf T_{A,B}\mathbf T_{B,C}
$$

주요 데이터 기호는 다음과 같다.

| 기호 | 차원 | 의미 |
|---|---:|---|
| $\mathbf I_i^e$ | $H\times W\times3$ | RGB image |
| $\mathbf Z_i^e$ | $H\times W$ | RGB에 정렬된 depth image |
| $\mathbf u_{i,e}^{m,k}$ | $2$ | 검출된 RGB pixel |
| $\mathbf X_O^{m,k}$ | $3$ | cube model의 알려진 3D corner |
| $\mathbf K_i$ | $3\times3$ | RGB camera intrinsic matrix |
| $\mathbf D_i$ | vector | RGB distortion coefficients |
| $\mathbf T_{C_i,O}^{e,c}$ | $4\times4$ | PnP cube-pose candidate |
| $\mathbf T_{B,F}^{e}$ | $4\times4$ | robot이 제공하는 event별 base-to-flange pose |
| $\mathbf T_{B,C_i}$ | $4\times4$ | fixed camera당 하나의 global extrinsic |
| $\mathbf T_{F,C_g}$ | $4\times4$ | session에 하나인 flange-to-camera hand-eye transform |

---

## 1. RGB camera model

### 1.1 Intrinsic matrix

$$
\mathbf K_i=
\begin{bmatrix}
f_{x,i} & 0 & c_{x,i}\\
0 & f_{y,i} & c_{y,i}\\
0 & 0 & 1
\end{bmatrix}
$$

Camera-frame point는:

$$
\mathbf X_C=
\begin{bmatrix}X_C&Y_C&Z_C\end{bmatrix}^{\mathsf T}
$$

Normalized coordinate는:

$$
x=\frac{X_C}{Z_C},
\qquad
y=\frac{Y_C}{Z_C}
$$

### 1.2 OpenCV radial/tangential distortion

$$
r^2=x^2+y^2
$$

$$
x_d=x(1+k_1r^2+k_2r^4+k_3r^6)
+2p_1xy+p_2(r^2+2x^2)
$$

$$
y_d=y(1+k_1r^2+k_2r^4+k_3r^6)
+p_1(r^2+2y^2)+2p_2xy
$$

Pixel projection은:

$$
u=f_xx_d+c_x,
\qquad
v=f_yy_d+c_y
$$

이를 하나의 함수로 쓴다.

$$
\mathbf u=\pi(\mathbf K,\mathbf D,\mathbf X_C)
$$

---

## 2. 원본 depth를 RGB에 alignment

원본 depth pixel $\mathbf u_D=(u_D,v_D)$와 raw 값 $d$에서 metre 단위 depth를 만든다.

$$
z_D=d\,s_{\mathrm{depth}}
$$

왜곡 보정된 normalized ray를 $\widetilde{\mathbf q}_D$라 하면 depth-camera 3D 점은:

$$
\mathbf X_D=z_D\widetilde{\mathbf q}_D
$$

Factory-calibrated device extrinsic으로 RGB frame에 옮긴다.

$$
\overline{\mathbf X}_C
=\mathbf T_{C,D}\overline{\mathbf X}_D
$$

RGB pixel과 RGB-camera $z$를 구한다.

$$
\mathbf u_C=\pi(\mathbf K_C,\mathbf D_C,\mathbf X_C),
\qquad
Z_{\mathrm{aligned}}[v_C,u_C]=(\mathbf X_C)_z
$$

따라서 alignment에는 depth와 RGB 내부계수뿐 아니라 $\mathbf T_{C,D}$가 필요하다. 이 변환은 우리가 Step3에서
구하는 $\mathbf T_{B,C_i}$와 다르다.

---

## 3. Cube 3D–RGB 2D correspondence

Cube geometry가 marker $m$, corner $k$의 3D 위치를 제공한다.

$$
\mathbf X_O^{m,k}
=\begin{bmatrix}X&Y&Z\end{bmatrix}^{\mathsf T}
$$

RGB detector는 같은 corner의 pixel을 제공한다.

$$
\mathbf u_{i,e}^{m,k}
=\begin{bmatrix}u&v\end{bmatrix}^{\mathsf T}
$$

$M$개 marker가 각각 네 corner를 제공하면:

$$
N=4M,
\qquad
\texttt{object\_points}\in\mathbb R^{N\times3},
\qquad
\texttt{image\_points}\in\mathbb R^{N\times2}
$$

여기서 3D object point는 depth로 측정한 점이 아니라 cube 크기와 marker 부착 위치로 정의된 모델 점이다.

---

## 4. Camera×event별 PnP

PnP 미지수는 camera에서 본 cube pose다.

$$
\mathbf T_{C_i,O}^{e,c}
=
\begin{bmatrix}
\mathbf R_{i,e,c}&\mathbf t_{i,e,c}\\
\mathbf 0^{\mathsf T}&1
\end{bmatrix}
$$

Corner projection은:

$$
\widehat{\mathbf u}_{i,e}^{k,c}
=\pi\!\left(
\mathbf K_i,\mathbf D_i,
\mathbf R_{i,e,c}\mathbf X_O^k+\mathbf t_{i,e,c}
\right)
$$

Pixel residual은:

$$
\mathbf r_{i,e}^{k,c}
=\widehat{\mathbf u}_{i,e}^{k,c}-\mathbf u_{i,e}^{k}
$$

개념적 PnP 목적함수는:

$$
\mathbf T_{C_i,O}^{e,c}
=\underset{\mathbf T\in SE(3)}{\operatorname{argmin}}
\sum_{k\in\mathcal K_{i,e,c}}
\left\|\mathbf r_{i,e}^{k,c}(\mathbf T)\right\|_2^2
$$

현재 구현은 조건에 따라 다음을 사용한다.

- corner가 8개 이상: iterative PnP, RANSAC 사용 가능
- single planar marker: IPPE의 복수 pose 후보
- RANSAC pixel threshold: $5\,\mathrm{px}$, 최대 200 iteration, confidence 0.999

### PnP reprojection 통계

$$
e_k=\|\mathbf r_k\|_2
$$

$$
e_{\mathrm{mean}}=\frac1N\sum_{k=1}^{N}e_k,
\qquad
e_{\mathrm{median}}=\operatorname{median}(e_k)
$$

$$
e_{\mathrm{P90}}=\operatorname{percentile}_{90}(e_k)
$$

이 오차는 같은 이미지로 pose를 풀고 같은 이미지에서 재투영한 self-fit이다.

---

## 5. PnP pose를 이용한 depth-plane 검사

Aligned-depth pixel $(u,v)$를 왜곡 보정한 ray는 $z$성분을 1로 둔다.

$$
\mathbf q(u,v)=
\begin{bmatrix}x_n&y_n&1\end{bmatrix}^{\mathsf T}
$$

PnP가 예측한 marker plane의 camera-frame origin과 normal을 $\mathbf p_0,\mathbf n$이라 하면 ray-plane 교점의
예상 depth는:

$$
z_{\mathrm{pred}}(u,v)
=\frac{\mathbf n^{\mathsf T}\mathbf p_0}
{\mathbf n^{\mathsf T}\mathbf q(u,v)}
$$

Measured depth는:

$$
z_{\mathrm{meas}}(u,v)
=\mathbf Z_i^e[v,u]\,s_{\mathrm{depth},i}
$$

단, 저장 PNG가 이미 metric 값으로 변환된 표현이라면 중복 scale을 적용하면 안 된다. 현재 코드 경로에서는 Z16 raw
unit을 읽고 저장된 `depth_scale_m_per_unit`을 적용한다.

Plane residual은:

$$
e_{\mathrm{depth}}(u,v)
=1000\left|z_{\mathrm{meas}}(u,v)-z_{\mathrm{pred}}(u,v)\right|
\quad[\mathrm{mm}]
$$

Depth inlier는 기본 plane tolerance $8\,\mathrm{mm}$로 계산한다.

$$
\rho_{\mathrm{inlier}}
=\frac1{N_d}\sum_{j=1}^{N_d}
\mathbf 1\!\left(e_{\mathrm{depth},j}<8\,\mathrm{mm}\right)
$$

기록값에는 mean/median/max, sample 수, marker 수, inlier ratio, predicted/measured scale, bias가 포함된다.

---

## 6. 후보 rank와 weight

1차 후보 rank는 다음 lexicographic 정보에 해당한다.

$$
\operatorname{rank}(c)=
\left(
-n_{\mathrm{marker}},
\operatorname{depthRank}(c),
e_{\mathrm{mean}},
\operatorname{sourcePriority}
\right)
$$

더 많은 marker ID가 먼저 우선하며, 그다음 depth-valid 여부, 작은 plane error, 큰 depth sample/marker support를
비교한다.

Camera/set consensus에서 사용하는 대표적인 observation weight는:

$$
w_c
=\frac{n_{\mathrm{face}}^2\,w_{\mathrm{single}}}
{\max(e_{\mathrm{mean}},\epsilon)
\left(1+0.15\,p_{\mathrm{depth}}\right)}
$$

Single-face fixed-camera 관측은 specialized 경로에서 $w_{\mathrm{single}}=0.35$를 쓸 수 있다. 별도의
`candidate_weight` 경로도 같은 핵심 원칙을 사용한다.

$$
w_c^{\prime}
\propto
\frac{n_{\mathrm{face}}^2}{e_{\mathrm{mean}}}
\times\text{single-face penalty}
\times\text{depth sample/marker support}
\times\frac1{1+e_{\mathrm{depth}}/3}
$$

따라서 depth가 pose를 직접 새로 푸는 것은 아니지만 candidate의 순위와 downstream 영향력을 바꾼다.

### Multi-camera event candidate score

Camera $i$의 후보를 base로 옮긴 cube pose는:

$$
\mathbf T_{B,O}^{i,e,c}
=\mathbf T_{B,C_i}^{e}\mathbf T_{C_i,O}^{e,c}
$$

다른 camera 후보의 consensus $\mathbf T_{B,O}^{\mathrm{ref}}$와 비교하는 점수는 개념적으로:

$$
S(c)=
\frac{d_t+5d_R}{w_{\mathrm{face}}}
+10e_{\mathrm{mean}}
+\lambda_d p_{\mathrm{depth}}
+\lambda_p p_{\mathrm{prior}}
+p_{\mathrm{single}}
$$

기본 profile은 consensus를 $4\,\mathrm{mm}/0.7^\circ$, cube-only specialized profile은
$7\,\mathrm{mm}/1^\circ$로 prune한다.

---

## 7. Fixed-camera 상대변환

같은 event에서 reference camera와 camera $i$가 cube를 관측하면:

$$
\mathbf T_{C_{\mathrm{ref}},C_i}^{e}
=\mathbf T_{C_{\mathrm{ref}},O}^{e}
\left(\mathbf T_{C_i,O}^{e}\right)^{-1}
$$

전체 공통 event 후보를 robust weighted SE(3) average한다.

$$
\mathbf T_{C_{\mathrm{ref}},C_i}
=\operatorname{RobustAvg}_{e}
\left(\mathbf T_{C_{\mathrm{ref}},C_i}^{e};w_e\right)
$$

이 값은 camera pair당 하나다.

---

## 8. Robot flange pose와 hand-eye

Robot이 제공하는 base 기준 flange pose는 event마다 하나다.

$$
\mathbf T_{B,F}^{e}
$$

Hand-eye 미지수는 session 전체에 하나다.

$$
\mathbf X=\mathbf T_{F,C_g}
$$

OpenCV hand-eye는 event pair의 상대 motion으로 고전식:

$$
\mathbf A_{ab}\mathbf X=\mathbf X\mathbf B_{ab}
$$

를 푼다. 추정한 $\mathbf T_{F,C_g}$로 event별 target base pose를 만들면:

$$
\mathbf T_{B,Q}^{e}
=\mathbf T_{B,F}^{e}
\mathbf T_{F,C_g}
\mathbf T_{C_g,Q}^{e},
\qquad Q\in\{W,O\}
$$

고정 target이면 이 값들이 event 사이에서 같아야 한다.

### MAD outlier cutoff

Translation residual $d_e$에 대해:

$$
\operatorname{MAD}
=\operatorname{median}_e
\left|d_e-\operatorname{median}(d)\right|
$$

$$
\tau
=\operatorname{median}(d)
+2(1.4826)\operatorname{MAD}
$$

$d_e>\tau$인 event를 refinement subset에서 제외하되 최소 데이터 수를 보장한다. 이후 board+cube local refine은
Huber loss를 사용한다.

---

## 9. Board와 cube를 이용한 fixed-camera base pose

### Board 경로

$$
\mathbf T_{B,W}^{e}
=\mathbf T_{B,F}^{e}
\mathbf T_{F,C_g}
\mathbf T_{C_g,W}^{e}
$$

$$
\mathbf T_{B,C_i}^{e,\mathrm{board}}
=\mathbf T_{B,W}^{e}
\left(\mathbf T_{C_i,W}^{e}\right)^{-1}
$$

### Cube 경로

$$
\mathbf T_{B,C_i}^{e,\mathrm{cube}}
=\mathbf T_{B,O}^{e}
\left(\mathbf T_{C_i,O}^{e}\right)^{-1}
$$

각 source를 robust average한 뒤 `auto` mode가 translation scatter를 비교한다. 한 source의 scatter가 다른 source의
0.7배보다 작으면 그 source가 primary다. Secondary와의 차이가 $25\,\mathrm{mm}/5^\circ$ 이내이면 SE(3)
interpolation으로 blend한다.

---

## 10. Event/set cube anchor와 frame alignment

Fixed camera $i$가 예측한 event cube pose는:

$$
\mathbf T_{B,O}^{i,e}
=\mathbf T_{B,C_i}\mathbf T_{C_i,O}^{e}
$$

Event consensus는:

$$
\mathbf T_{B,O}^{e}
=\operatorname{RobustAvg}_{i}
\left(\mathbf T_{B,O}^{i,e};w_{i,e}\right)
$$

현재 hybrid set anchor는 개념적으로 fixed consensus의 translation과 gripper/FK 경로의 rotation을 결합한 뒤
set별 robust average한다.

$$
\mathbf T_{B,O}^{s}
=\operatorname{RobustAvg}_{e:s(e)=s}
\left(\mathbf T_{B,O}^{e}\right)
$$

Robot nominal set-center와 AprilTag object frame의 set별 offset은:

$$
\boldsymbol\Delta_s
=\left(\mathbf T_{B,S}^{s}\right)^{-1}\mathbf T_{B,O}^{s}
$$

$$
\overline{\boldsymbol\Delta}
=\operatorname{RobustWeightedAvg}_{s}(\boldsymbol\Delta_s)
$$

Corrected FK-aligned cube prior는:

$$
\widetilde{\mathbf T}_{B,O}^{s}
=\mathbf T_{B,S}^{s}\overline{\boldsymbol\Delta}
$$

Raw $\mathbf T_{B,S}^{s}$를 바로 object pose로 쓰지 않는 것이 중요하다.

---

## 11. D2 set-consistency camera refinement

Set anchor를 고정하고 각 event에서 camera pose 후보를 역산한다.

$$
\mathbf T_{B,C_i}^{e}
=\mathbf T_{B,O}^{s(e)}
\left(\mathbf T_{C_i,O}^{e}\right)^{-1}
$$

$$
\mathbf T_{B,C_i}^{\mathrm{cand}}
=\operatorname{RobustAvg}_{e}
\left(\mathbf T_{B,C_i}^{e};w_{i,e}\right)
$$

초기값 $\mathbf T^{0}_{B,C_i}$과 후보의 변화량은:

$$
\Delta t_i
=1000\left\|\mathbf t_i^{\mathrm{cand}}-\mathbf t_i^0\right\|_2
\quad[\mathrm{mm}]
$$

$$
\Delta R_i
=\operatorname{angle}
\left((\mathbf R_i^0)^{\mathsf T}\mathbf R_i^{\mathrm{cand}}\right)
\quad[\mathrm{degree}]
$$

- $15\,\mathrm{mm}/3^\circ$ 이내: full adoption
- 각 threshold의 3배 이내: 제한된 step으로 interpolation
- 그보다 큼: reject

---

## 12. D3 fixed-camera raw-corner reprojection refinement

Event cube pose를 고정하고 camera $i$의 $\mathbf T_{B,C_i}$ 하나만 최적화한다.

$$
\mathbf T_{C_i,O}^{e}(\mathbf T_{B,C_i})
=\left(\mathbf T_{B,C_i}\right)^{-1}\mathbf T_{B,O}^{e}
$$

$$
\mathbf r_{i,e}^{k}(\mathbf T_{B,C_i})
=\pi\!\left(
\mathbf K_i,\mathbf D_i,
\left(\mathbf T_{B,C_i}\right)^{-1}
\mathbf T_{B,O}^{e}\mathbf X_O^k
\right)-\mathbf u_{i,e}^{k}
$$

$$
\mathbf T_{B,C_i}^{*}
=\underset{\mathbf T_{B,C_i}\in SE(3)}{\operatorname{argmin}}
\sum_{e,k}\rho_{\mathrm{Huber},\,2\mathrm{px}}
\left(\left\|\mathbf r_{i,e}^{k}\right\|_2\right)
$$

Pixel RMSE는:

$$
\operatorname{RMSE}_{\mathrm{px}}
=\sqrt{\frac1N\sum_{j=1}^{N}\|\mathbf r_j\|_2^2}
$$

채택 조건은 RMSE 감소와 $20\,\mathrm{mm}/3^\circ$ transform guard다.

---

## 13. Production Step-E `fk_fixed` pose joint solve

### 13.1 변수와 고정값

최적화 변수:

$$
\boldsymbol\Theta
=\left\{
\mathbf T_{B,C_1},\ldots,\mathbf T_{B,C_F},
\mathbf T_{F,C_g},
\mathbf T_{B,W}
\right\}
$$

고정값:

$$
\left\{
\widetilde{\mathbf T}_{B,O}^{s},
\mathbf T_{B,F}^{e},
\mathbf T_{C_i,O}^{e},
\mathbf T_{C_i,W}^{e}
\right\}
$$

Corrected set prior가 없으면 이 solve는 실행하지 않는다.

### 13.2 SE(3) pose error

예측 $\mathbf A$와 target $\mathbf B$ 사이 error transform은:

$$
\mathbf E=\mathbf A^{-1}\mathbf B
$$

Rotation과 translation residual은:

$$
\mathbf r_R=\operatorname{Log}_{SO(3)}(\mathbf R_E),
\qquad
\mathbf r_t=\mathbf t_E
$$

기본 scale로 정규화한다.

$$
\overline{\mathbf r}
=
\begin{bmatrix}
\mathbf r_R/\sigma_R\\
\mathbf r_t/\sigma_t
\end{bmatrix},
\qquad
\sigma_R=1^\circ,
\quad
\sigma_t=5\,\mathrm{mm}
$$

### 13.3 여섯 관측 그룹

Fixed-camera cube:

$$
\mathbf T_{B,C_i}\mathbf T_{C_i,O}^{e}
\approx\widetilde{\mathbf T}_{B,O}^{s(e)}
$$

Gripper-camera cube:

$$
\mathbf T_{B,F}^{e}\mathbf T_{F,C_g}\mathbf T_{C_g,O}^{e}
\approx\widetilde{\mathbf T}_{B,O}^{s(e)}
$$

Fixed-camera board:

$$
\mathbf T_{B,C_i}\mathbf T_{C_i,W}^{e}
\approx\mathbf T_{B,W}
$$

Gripper-camera board:

$$
\mathbf T_{B,F}^{e}\mathbf T_{F,C_g}\mathbf T_{C_g,W}^{e}
\approx\mathbf T_{B,W}
$$

Fixed↔fixed cube cross-path:

$$
\mathbf T_{B,C_i}\mathbf T_{C_i,O}^{e}
\approx
\mathbf T_{B,C_j}\mathbf T_{C_j,O}^{e}
$$

Fixed↔gripper cube cross-path:

$$
\mathbf T_{B,C_i}\mathbf T_{C_i,O}^{e}
\approx
\mathbf T_{B,F}^{e}\mathbf T_{F,C_g}\mathbf T_{C_g,O}^{e}
$$

### 13.4 Group normalization과 목적함수

관측 그룹 $g$에 $N_g$개의 pose pair가 있으면 group scale은:

$$
\alpha_g=\frac{w_g}{\sqrt{N_g}}
$$

전체 목적함수는:

$$
\boldsymbol\Theta^*
=\underset{\boldsymbol\Theta}{\operatorname{argmin}}
\sum_g\sum_{j=1}^{N_g}
\rho_{\mathrm{Huber}}
\left(\left\|\alpha_g\overline{\mathbf r}_{g,j}\right\|_2\right)
$$

현재 기본 group weight는 cube, board, cross-path 모두 1이다.

### 13.5 관측 gate

- Set-FK cube residual: `cube_gripped=False`만 사용
- Fixed cube set residual: 서로 다른 face가 2개 이상인 관측만 사용
- Same-event cross-path: gripped cube도 사용 가능
- Fixed-board 초기 pose 차이: $25\,\mathrm{mm}/5^\circ$ 초과 제외
- Cross-path 초기 차이: $30\,\mathrm{mm}/10^\circ$ 초과 제외

### 13.6 결과 채택

$$
J_{\mathrm{final}}<J_{\mathrm{initial}}
$$

이어야 하고, $\mathbf T_{B,C_i}$와 $\mathbf T_{F,C_g}$의 모든 변화가:

$$
\Delta t\le50\,\mathrm{mm},
\qquad
\Delta R\le15^\circ
$$

여야 한다. Solver 성공까지 모두 만족할 때만 joint 결과 전체를 채택한다.

---

## 14. Optional Step-E `reprojection_fk_fixed`

이 경로는 PnP pose residual 대신 raw corner residual을 사용한다.

Target $Q\in\{O,W\}$의 camera pose 예측은:

$$
\mathbf T_{C_i,Q}^{e}
=\left(\mathbf T_{B,C_i}\right)^{-1}\mathbf T_{B,Q}^{e}
$$

Gripper camera에서는:

$$
\mathbf T_{C_g,Q}^{e}
=\left(\mathbf T_{B,F}^{e}\mathbf T_{F,C_g}\right)^{-1}
\mathbf T_{B,Q}^{e}
$$

Corner residual은:

$$
\mathbf r_{i,e}^{k}
=\pi\!\left(\mathbf K_i,\mathbf D_i,
\mathbf T_{C_i,Q}^{e}\mathbf X_Q^k\right)
-\mathbf u_{i,e}^{k}
$$

목적함수는:

$$
\boldsymbol\Theta^*
=\underset{\boldsymbol\Theta}{\operatorname{argmin}}
\sum_{i,e,k}
\rho_{\mathrm{softL1},\,2\mathrm{px}}
\left(\|\mathbf r_{i,e}^{k}\|_2\right)
$$

현재 Step3 adapter의 자유변수는 $\{\mathbf T_{B,C_i},\mathbf T_{F,C_g},\mathbf T_{B,W}\}$이고 set cube poses는
고정한다. 현재 loader는 gripped-cube corner를 제외한다. Solver 성공, pixel RMSE 감소, 모든 camera/hand-eye/board
변화 $50\,\mathrm{mm}/15^\circ$ guard를 통과해야 채택한다.

이 경로는 opt-in이며 현재 기본은 pose 기반 `fk_fixed`다.

---

## 15. 검증 수식과 의미

### 15.1 Step4 cube reprojection

Fresh PnP pose $\widehat{\mathbf T}_{C_i,O}^{e}$를 같은 이미지에서 다시 구한 뒤:

$$
e_{i,e}^{\mathrm{self}}
=\frac1N\sum_k
\left\|
\pi(\mathbf K_i,\mathbf D_i,
\widehat{\mathbf T}_{C_i,O}^{e}\mathbf X_O^k)
-\mathbf u_{i,e}^k
\right\|_2
$$

를 계산한다. 이는 self-fit이며 최종 $\mathbf T_{B,C_i}$의 독립 held-out pixel error가 아니다.

### 15.2 Cross-camera position consistency

$$
\mathbf p_{i,e}
=\operatorname{trans}
\left(\mathbf T_{B,C_i}^{e}\mathbf T_{C_i,O}^{e}\right)
$$

$$
\overline{\mathbf p}_e
=\frac1{N_e}\sum_i\mathbf p_{i,e}
$$

$$
e_{i,e}^{\mathrm{cross}}
=1000\left\|\mathbf p_{i,e}-\overline{\mathbf p}_e\right\|_2
\quad[\mathrm{mm}]
$$

공통 bias는 보지 못하므로 외부 GT 정확도와 같지 않다.

### 15.3 Hand-eye board consistency

$$
\mathbf T_{B,W}^{e}
=\mathbf T_{B,F}^{e}\mathbf T_{F,C_g}\mathbf T_{C_g,W}^{e}
$$

Event별 translation의 표준편차와 최대 편차를 본다.

### 15.4 Depth mesh RMSE

Depth point를 PnP cube frame으로 옮긴 점을 $\mathbf x_{O,j}$, 가장 가까운 cube surface 거리를
$d_{\mathrm{surf}}(\mathbf x_{O,j})$라 하면:

$$
\operatorname{RMSE}_{\mathrm{mesh}}
=\sqrt{\frac1N\sum_jd_{\mathrm{surf}}(\mathbf x_{O,j})^2}
$$

이 검증은 depth 품질과 cube model 정합을 함께 반영하지만 독립적인 transform GT는 아니다.

### 15.5 진짜 held-out calibration-chain reprojection

독립 test라면 test 이미지로 pose를 다시 fitting하지 않고 frozen calibration과 독립 target pose를 써야 한다.

$$
\mathbf T_{C_i,O}^{e,\mathrm{test}}
=\left(\mathbf T_{B,C_i}^{\mathrm{train}}\right)^{-1}
\mathbf T_{B,O}^{e,\mathrm{test}}
$$

$$
e_{\mathrm{holdout}}
=\sqrt{\frac1N\sum_{i,e,k}
\left\|
\pi(\mathbf K_i,\mathbf D_i,
\mathbf T_{C_i,O}^{e,\mathrm{test}}\mathbf X_O^k)
-\mathbf u_{i,e}^{k}
\right\|_2^2}
$$

README의 기본 Step4는 이 값을 계산하는 전용 held-out runner가 아니다.

---

## 16. 행렬 개수와 공유 범위

| 행렬 | 개수 |
|---|---:|
| $\mathbf K_i,\mathbf D_i$ | camera당 하나 |
| $\mathbf T_{C_i,O}^{e,c}$ | camera×event×candidate |
| $\mathbf T_{C_i,O}^{e}$ | 유효 camera×event당 하나 |
| $\mathbf T_{B,F}^{e}$ | flange pose가 있는 event당 하나 |
| $\mathbf T_{C_{\mathrm{ref}},C_i}$ | fixed camera당 하나 |
| $\mathbf T_{F,C_g}$ | session당 하나 |
| $\mathbf T_{B,C_i}$ | fixed camera당 하나 |
| $\mathbf T_{B,O}^{e}$ | 유효 anchor event당 하나 |
| $\mathbf T_{B,O}^{s}$ | set당 하나 |
| $\overline{\boldsymbol\Delta}$ | session당 하나 |
| $\mathbf T_{B,W}$ | 고정 board당 하나 |

가장 중요한 구분은 다음과 같다.

```text
T(C_i <- O; event e) = 이미지마다 생기는 PnP 관측
T(B <- C_i)          = 모든 set/event가 함께 추정하는 camera당 최종 행렬 하나
```

---

## 17. Reprojection에 실제로 들어가는 값

| Reprojection 종류 | Pose를 어디서 얻나 | 식에 들어가는 값 | depth 직접 입력 |
|---|---|---|---|
| PnP self-fit | 같은 이미지의 PnP | $\mathbf K_i,\mathbf D_i,\mathbf T_{C_i,O}^{e},\mathbf X_O^k$ | 아니오 |
| D3 calibration reprojection | global camera + fixed event cube | 위 값 + $\mathbf T_{B,C_i},\mathbf T_{B,O}^{e}$ | 아니오 |
| Optional joint corner | global cameras/hand-eye/board + fixed set cube | 전체 transform chain과 raw corner | 아니오 |
| Step4 self-fit | 검증 이미지의 fresh PnP | PnP와 같은 값 | 아니오 |
| Depth-plane 검사 | PnP marker plane | PnP pose, aligned depth, depth scale | 예 |

Reprojection에는 내부계수만 들어가는 것이 아니다. 3D point를 camera frame으로 옮기는 pose가 반드시 필요하다.
PnP는 그 pose를 얻는 한 방법이고, D3나 joint solve에서는 global calibration chain이 pose를 제공한다.

---

## 18. 오차를 버리는지 보정에 쓰는지

| 오차 | 처리 |
|---|---|
| 촬영 gate 실패 | 기본 event 미저장 |
| PnP RANSAC outlier corner | 해당 PnP 해 계산에서 제외 |
| 높은 PnP reprojection | 후보 제외 또는 gate 실패 |
| Single-face fixed PnP | 허용하되 낮은 weight |
| Single-face gripper PnP | 제거 |
| Depth 불량 | gate, rank, weight에서 불리; pose를 depth로 직접 수정하지 않음 |
| Multi-camera 불일치 | candidate 재선택·hard prune·MAD trim |
| Hand-eye event outlier | MAD subset 제외 |
| D2 큰 camera 변화 | full/partial/reject |
| D3 큰 corner residual | Huber soft down-weight |
| D3 RMSE 미개선/guard 초과 | 새 camera pose reject |
| Joint 명백 outlier | optimization 전에 hard gate |
| Joint 잔여 큰 residual | Huber 또는 optional soft-L1 |
| Joint 실패/미개선/guard 초과 | joint 결과 전체 reject, staged 유지 |
| Step4 검증 실패 | 보고·진단·재촬영 판단; 자동 재학습 안 함 |

---

## 19. Depth 사용 여부의 최종 판정

Depth는 다음에 사용한다.

- Step2 capture gate
- PnP plane consistency
- 후보 rank와 observation weight
- event candidate consensus의 penalty
- Step4 depth mesh/dimension verification

Depth는 다음 기본 residual에는 직접 들어가지 않는다.

- PnP RGB reprojection residual
- D3 RGB corner residual
- Production `fk_fixed` SE(3) pose residual
- Optional `reprojection_fk_fixed` RGB corner residual

따라서 현재 방식은 depth-aware이지만 최종 solver가 RGB와 depth residual을 동시에 최소화하는 RGB-D bundle
adjustment는 아니다.

---

## 20. 구현 위치

| 내용 | 구현 파일/함수 |
|---|---|
| RealSense intrinsics/extrinsics dump | `Step1_dump_all_intrinsics.py` |
| Aligned depth | `camera.py`, `rs.align(rs.stream.color)` |
| A/B capture gate와 저장 정책 | `Step2_capture.py`, `capture_gate.py` |
| Cube PnP·IPPE·depth plane | `apriltag_cube.py` |
| Candidate rank/weight/consensus | `calibration_runtime_utils.py` |
| Hand-eye, D1–D3, pose joint | `Step3_calibration.py` |
| Optional canonical corner solver | `calibration_reprojection_backend.py` |
| Step4 검증 | `Step4_verify.py`, `downstream_metrics.py` |
| Report/export | `Step5_export_reports.py` |
