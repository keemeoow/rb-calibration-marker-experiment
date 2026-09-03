# 멀티카메라 캘리브레이션 완전 설명서

## 목차

- [0. 발표를 한 문장으로 시작한다면](#toc-section-1)
- [1. 왜 카메라마다 큐브의 좌표가 다를까?](#toc-section-2)
- [2. 먼저 좌표계 기호를 정리한다](#toc-section-3)
- [3. 변환행렬은 무엇인가?](#toc-section-4)
- [4. 카메라가 실제로 측정하는 것은 무엇인가?](#toc-section-5)
- [5. 우리가 찾아야 하는 미지수](#toc-section-6)
- [6. 두 종류 카메라의 관측 경로](#toc-section-7)
- [7. 두 자세가 얼마나 다른지 계산하는 방법](#toc-section-8)
- [8. 모든 방법이 공유하는 기본 목적함수](#toc-section-9)
- [9. 방법 1: no-FK(vision)](#toc-section-10)
- [10. 방법 2: raw-FK-fixed](#toc-section-11)
- [11. 방법 3: corrected-FK](#toc-section-12)
- [12. 네 FK 방식 비교](#toc-section-13)
- [13. Unified 방식](#toc-section-14)
- [14. Sequential(`seq`, `-Unified`) 방식](#toc-section-15)
- [15. Board-only, Cube-only, Both](#toc-section-16)
- [16. 왜 전체 조합이 $24$개가 아니라 $18$개인가?](#toc-section-17)
- [17. corrected-FK와 Ridge 출력 후보정은 다르다](#toc-section-18)
- [18. 평가 지표를 해석하는 방법](#toc-section-19)
- [19. 코드 흐름으로 다시 보기](#toc-section-20)
- [20. 교수님께 설명할 때의 추천 순서](#toc-section-21)
- [21. 최종 한 장 요약용 수식](#toc-section-22)
- [22. 예상 질문과 짧은 답변](#toc-section-23)

> 대상: 캘리브레이션을 처음 배우는 사람  
> 목표: 이 문서만 읽고 `no-FK(vision)`, `raw-FK-fixed`, `corrected-FK factor`, `vision-aligned-FK-fixed`, `Unified`, `Sequential`, `Board-only`, `Cube-only`, `Both`를 실제 코드와 같은 수식으로 설명하기
> 표기 원칙: 모든 수학 변수와 기호는 Markdown LaTeX인 `$...$` 또는 `$$...$$` 안에 작성했다. 따라서 수식 부분을 그대로 복사할 수 있다.

---

<a id="toc-section-1"></a>

## 0. 발표를 한 문장으로 시작한다면

멀티카메라 캘리브레이션은 다음 문제다.

> 서로 다른 위치에서 같은 물체를 본 여러 카메라가, 그 물체의 위치와 방향을 로봇 베이스라는 하나의 공통 좌표계에서 똑같이 말하도록 카메라 사이의 관계를 찾는 과정이다.

이 프로젝트에서는 여기에 두 가지 선택 축이 더해진다.

1. 로봇이 계산한 큐브 자세인 FK를 사용하지 않을지, raw 값에 고정할지, 영상 정렬 후 soft factor 또는 hard constraint로 사용할지 결정한다.
2. 고정 카메라와 그리퍼 카메라를 하나의 문제로 동시에 풀지, 서로 분리해서 풀지 결정한다.

---

<a id="toc-section-2"></a>

## 1. 왜 카메라마다 큐브의 좌표가 다를까?

카메라마다 자기 자신을 원점으로 사용하기 때문이다.

- 카메라 $0$은 큐브가 자기 앞에 있다고 말한다.
- 카메라 $1$은 같은 큐브가 자기 오른쪽에 있다고 말한다.
- 그리퍼 카메라는 같은 큐브가 자기 아래에 있다고 말할 수 있다.

세 답은 서로 모순이 아니다. 기준 좌표계가 다를 뿐이다.

따라서 모든 관측을 공통 기준인 로봇 베이스 좌표계 $B$로 옮겨야 한다.

---

<a id="toc-section-3"></a>

## 2. 먼저 좌표계 기호를 정리한다

| 기호 | 의미 |
|---|---|
| $B$ | 로봇 베이스 좌표계 |
| $C_i$ | $i$번째 고정 카메라 좌표계 |
| $G$ | 로봇 그리퍼 좌표계 |
| $C_g$ | 그리퍼에 부착된 카메라 좌표계 |
| $O$ | 큐브 또는 캘리브레이션 물체 좌표계 |
| $s$ | 큐브를 놓은 세트 번호 |
| $e$ | 그리퍼 카메라 촬영 이벤트 번호 |
| $s(e)$ | 이벤트 $e$가 속한 세트 번호 |

여기서 $G$는 코드 전체에서 하나의 물리 frame이어야 한다. 서로 다른 날의 기록을 합칠 때 일부 event가 TCP, 일부 event가 flange 기준이면 하나의 $T^G_C$로 두 묶음을 동시에 설명할 수 없다. 기록 frame $G_r$를 canonical frame $G$로 바꾸는 알려진 강체변환을 $T^{G_r}_{G}$라 하면 먼저

$$
{}^B T_G(e)
=
{}^B T_{G_r}(e)\,{}^{G_r}T_G
$$

를 모든 event에 적용한다. cube-center tool frame도 같은 방식으로 정규화한다. 이 단계는 영상 또는 fitted calibration을 보지 않는 좌표계 변환이며, 최적화나 outlier 제거가 아니다. 현재 session02에서는 legacy tool3 150 mm $\rightarrow$ flange에 $-150$ mm, legacy cube-center tool4 177.5 mm $\rightarrow$ physical 143 mm에 $-34.5$ mm의 local-$z$ 우측변환을 적용한다. 변환 뒤 같은 set 11의 두 기록 묶음은 최대 $0.68\,\mathrm{mm}$, $0.00024^\circ$ 차이로 일치한다.

이 문서에서는 다음과 같은 변환 표기를 사용한다.

$$
{}^{A}\mathbf{T}_{B}
$$

읽는 방법은 다음과 같다.

> 좌표계 $B$에서 표현된 값을 좌표계 $A$에서 표현하도록 바꾸는 변환이다.

위 첨자 $A$는 도착 좌표계이고, 아래 첨자 $B$는 출발 좌표계다.

---

<a id="toc-section-4"></a>

## 3. 변환행렬은 무엇인가?

하나의 강체 자세는 회전과 이동으로 이루어진 $4 \times 4$ 동차변환행렬로 표현한다.

$$
\mathbf{T}
=
\begin{bmatrix}
\mathbf{R} & \mathbf{t} \\
\mathbf{0}^{\mathsf T} & 1
\end{bmatrix}
$$

각 항의 의미는 다음과 같다.

$$
\mathbf{R}\in\mathrm{SO}(3),
\qquad
\mathbf{t}\in\mathbb{R}^{3}
$$

- $\mathbf{R}$은 물체가 어느 방향을 보는지 나타내는 $3 \times 3$ 회전행렬이다.
- $\mathbf{t}$는 물체가 어디에 있는지 나타내는 $3 \times 1$ 이동벡터다.

좌표점 $\mathbf{p}_B$를 좌표계 $A$로 옮기는 식은 다음과 같다.

$$
\begin{bmatrix}
\mathbf{p}_A \\
1
\end{bmatrix}
=
{}^{A}\mathbf{T}_{B}
\begin{bmatrix}
\mathbf{p}_B \\
1
\end{bmatrix}
$$

변환행렬이라는 형식을 쓰는 이유는 세 가지다.

1. 위치와 방향을 하나의 값으로 묶어서 다룬다. 따로 관리하면 계산할 때마다 둘을 맞춰줘야 하지만, 하나로 묶으면 자세 전체를 변수 하나로 취급할 수 있다.
2. 좌표계를 갈아타는 일을 곱셈 한 번으로 처리한다. 회전만이면 $3 \times 3$ 행렬로 충분하지만, 이동은 곱셈이 아니라 덧셈이라서 그대로는 섞이지 않는다. 마지막 행에 $1$을 덧붙여 $4 \times 4$로 만들면 이동까지 곱셈 안으로 들어온다. 그 덕분에 여러 좌표계를 거치는 경로를 행렬 곱만으로 이어붙일 수 있다. 자세한 내용은 3.1절에서 다룬다.
3. 되돌리기가 쉽다. 어떤 변환이든 역행렬 하나로 방향을 반대로 뒤집을 수 있다. 자세한 내용은 3.2절에서 다룬다.

### 3.1 변환행렬을 곱한다는 뜻

경로를 차례대로 연결한다는 뜻이다.

$$
{}^{A}\mathbf{T}_{C}
=
{}^{A}\mathbf{T}_{B}
{}^{B}\mathbf{T}_{C}
$$

이를 행렬 내부까지 펼치면 다음과 같다.

$$
\begin{bmatrix}
\mathbf{R}_1 & \mathbf{t}_1 \\
\mathbf{0}^{\mathsf T} & 1
\end{bmatrix}
\begin{bmatrix}
\mathbf{R}_2 & \mathbf{t}_2 \\
\mathbf{0}^{\mathsf T} & 1
\end{bmatrix}
=
\begin{bmatrix}
\mathbf{R}_1\mathbf{R}_2 & \mathbf{R}_1\mathbf{t}_2+\mathbf{t}_1 \\
\mathbf{0}^{\mathsf T} & 1
\end{bmatrix}
$$

### 3.2 역행렬의 뜻

변환의 방향을 반대로 돌린다.

$$
\mathbf{T}^{-1}
=
\begin{bmatrix}
\mathbf{R}^{\mathsf T} & -\mathbf{R}^{\mathsf T}\mathbf{t} \\
\mathbf{0}^{\mathsf T} & 1
\end{bmatrix}
$$

즉, 회전은 전치행렬 $\mathbf{R}^{\mathsf T}$로 되돌리고, 이동도 그 회전 좌표계에 맞추어 반대로 적용한다.

이 문서에서 역행렬을 쓰는 이유는 세 가지다.

1. 알고 있는 변환이 필요한 방향과 반대일 때 뒤집어서 쓴다.
2. 두 자세가 얼마나 다른지 잰다. 자세끼리는 뺄셈이 성립하지 않으므로, 한쪽에서 다른 쪽까지 얼마나 더 움직여야 하는지를 대신 구한다. 자세한 내용은 7장에서 다룬다.
3. 여러 변환이 곱해진 식에서 이미 아는 항을 걷어내고 미지수만 남긴다. 방정식에서 이항하는 것과 같다. 실제 사용 예는 14장에 나온다.

---

<a id="toc-section-5"></a>

## 4. 카메라가 실제로 측정하는 것은 무엇인가?

카메라가 직접 측정하는 값은 보드나 큐브의 **2차원 코너 픽셀**이다. 현재 실제 캘리브레이션 solver는 PnP 자세를 최종 잔차로 쓰지 않고, 알려진 3차원 코너를 영상에 투영한 raw distorted-pixel 재투영잔차를 최소화한다.

PnP는 공통 입력 품질검사, 초기화, 평가용 관측 자세를 만드는 데만 사용한다. 이 구분은 중요하다. 최종 bundle adjustment가 PnP로 한번 압축된 6차원 자세가 아니라 모든 코너의 픽셀 정보를 직접 사용하기 때문이다.

여기서 PnP는 Perspective-n-Point의 줄임말이다. 한 장의 사진만 가지고 물체가 카메라 기준으로 어디에 어떤 방향으로 놓여 있는지를 알아내는 방법이다.

PnP를 풀려면 두 가지가 필요하다.

1. **물체 위 점들의 3차원 좌표.** 물체 자신의 좌표계에서 잰 값이다. 예를 들어 한 변이 $50\,\mathrm{mm}$인 체커보드 격자라면 코너들이 $(0,0,0)$, $(50,0,0)$, $(0,50,0)$처럼 놓여 있다는 사실을 설계 도면에서 이미 알고 있다.
2. **같은 점들이 사진에서 찍힌 2차원 픽셀 좌표.** 코너 검출 알고리즘이 찾아준다.

같은 점이 3차원에서는 어디이고 사진에서는 몇 번째 픽셀인지, 그 짝을 여러 개 모으면 물체의 자세를 역으로 풀 수 있다. 이름의 $n$은 그 짝의 개수를 뜻하며, 원리상 최소 $3$개, 안정적으로 풀려면 보통 그보다 많이 쓴다.

단, 카메라 내부파라미터를 미리 알고 있어야 한다. 내부파라미터란 초점거리와 이미지 중심처럼 3차원 점이 이미지의 어느 픽셀에 맺히는지를 정하는 카메라 자체의 값이다. 이 값은 캘리브레이션 이전에 따로 구해 둔다.

아래의 $\mathbf{Z}$는 좌표 경로를 설명하기 위한 PnP 자세 표기다. 실제 최적화 목적함수는 8장에서 정의하는 코너 픽셀 $\mathbf{u}_{o,j}$를 사용한다.

cube 관측의 PnP 품질 마스크는 train/test split과 모든 method fit보다 먼저 한 번만 계산한다. planar 관측은 IPPE의 positive-depth 후보 중 all-corner RMSE가 가장 작은 해를 쓰고, non-planar 관측은 RANSAC-EPNP로 초기화한다. 어느 경우든 **검출된 모든 코너**의 Euclidean pixel RMSE로 판정하며, 기본 상한은 고정 카메라 $3\,\mathrm{px}$, 그리퍼 카메라 $5\,\mathrm{px}$다. 따라서 모든 비교 행은 같은 pre-fit 입력 마스크에서 시작한다. 그 뒤 각 학습 단계는 1차 적합 결과로 `(event,camera)` 이미지 전체의 RMSE를 계산하고, MAD threshold를 넘는 frame을 최대 30\%까지 coverage-safe하게 제외하여 한 번 재적합한다. 재적합 결과는 제거 전 학습 관측 전체의 동일 robust objective가 개선될 때만 채택하고, 아니면 1차 결과로 rollback한다. Held-out 관측은 이 선택과 재적합에 사용하거나 제거하지 않는다.

### 4.1 고정 카메라 관측

$$
\mathbf{Z}_{i,s}
\equiv
{}^{C_i}\mathbf{T}_{O,s}
$$

이는 세트 $s$의 큐브가 고정 카메라 $i$에서 어떻게 보였는지를 뜻한다.

### 4.2 그리퍼 카메라 관측

$$
\mathbf{Z}_{g,e}
\equiv
{}^{C_g}\mathbf{T}_{O,e}
$$

이는 이벤트 $e$에서 큐브가 그리퍼 카메라에 어떻게 보였는지를 뜻한다.

### 4.3 로봇 FK로 알고 있는 그리퍼 자세

$$
\mathbf{G}_e
\equiv
{}^{B}\mathbf{T}_{G,e}
$$

$\mathbf{G}_e$는 촬영 순간 그리퍼의 위치와 방향이다. 이것은 로봇 관절각으로부터 FK를 계산하여 얻는다.

여기서 FK는 Forward Kinematics, 우리말로 순기구학의 줄임말이다. 로봇의 각 관절이 몇 도 꺾여 있는지를 알 때, 링크 길이 같은 로봇의 설계값을 이용해 손끝이 베이스 기준으로 어디에 어떤 방향으로 있는지를 계산하는 것이다. 카메라를 쓰지 않고 로봇 자신의 관절 센서만으로 얻는 값이라는 점이 중요하다.

---

<a id="toc-section-6"></a>

## 5. 우리가 찾아야 하는 미지수

### 5.1 고정 카메라 외부파라미터

$$
\mathbf{C}_i
\equiv
{}^{B}\mathbf{T}_{C_i}
$$

### 5.2 그리퍼와 그리퍼 카메라 사이의 hand-eye 변환

$$
\mathbf{X}
\equiv
{}^{G}\mathbf{T}_{C_g}
$$

### 5.3 세트별 큐브 자세

$$
\mathbf{O}_s
\equiv
{}^{B}\mathbf{T}_{O,s}
$$

우리 multi-cam calibration에서 비교하는 네 FK 방식의 가장 큰 차이는 $\mathbf{O}_s$를 vision으로만 자유롭게 찾는지, raw FK와 mechanical frame map에 고정하는지, 영상 정렬 FK에 고정하는지, 또는 자유변수로 두되 영상 정렬 FK의 공분산 인자를 추가하는지다.

---

<a id="toc-section-7"></a>

## 6. 두 종류 카메라의 관측 경로

### 6.1 고정 카메라 경로

베이스에서 고정 카메라로 가고, 고정 카메라에서 큐브로 간다.

$$
{}^{B}\mathbf{T}_{C_i}
{}^{C_i}\mathbf{T}_{O,s}
=
{}^{B}\mathbf{T}_{O,s}
$$

앞에서 정의한 짧은 기호를 사용하면 다음과 같다.

$$
\mathbf{C}_i\mathbf{Z}_{i,s}=\mathbf{O}_s
$$

따라서 고정 카메라가 예측한 큐브 자세는 다음과 같다.

$$
\widehat{\mathbf{O}}^{\mathrm{fix}}_{i,s}
=
\mathbf{C}_i\mathbf{Z}_{i,s}
$$

### 6.2 그리퍼 카메라 경로

베이스에서 그리퍼로 가고, 그리퍼에서 카메라로 가고, 카메라에서 큐브로 간다.

$$
{}^{B}\mathbf{T}_{G,e}
{}^{G}\mathbf{T}_{C_g}
{}^{C_g}\mathbf{T}_{O,e}
=
{}^{B}\mathbf{T}_{O,s(e)}
$$

짧은 기호로 쓰면 다음과 같다.

$$
\mathbf{G}_e\mathbf{X}\mathbf{Z}_{g,e}
=
\mathbf{O}_{s(e)}
$$

따라서 그리퍼 카메라가 예측한 큐브 자세는 다음과 같다.

$$
\widehat{\mathbf{O}}^{\mathrm{grip}}_e
=
\mathbf{G}_e\mathbf{X}\mathbf{Z}_{g,e}
$$

캘리브레이션이 잘되었다면 같은 세트의 모든 예측은 같은 큐브 자세로 모여야 한다.

---

<a id="toc-section-8"></a>

## 7. 두 자세가 얼마나 다른지 계산하는 방법

두 자세 $\mathbf{A}$와 $\mathbf{B}$의 상대변환은 다음과 같다.

$$
\mathbf{E}
=
\mathbf{A}^{-1}\mathbf{B}
$$

$\mathbf{A}=\mathbf{B}$라면 $\mathbf{E}=\mathbf{I}$가 된다.

행렬 내부까지 펼치면 다음과 같다.

$$
\mathbf{A}^{-1}\mathbf{B}
=
\begin{bmatrix}
\mathbf{R}_A^{\mathsf T}\mathbf{R}_B
&
\mathbf{R}_A^{\mathsf T}(\mathbf{t}_B-\mathbf{t}_A)
\\
\mathbf{0}^{\mathsf T} & 1
\end{bmatrix}
$$

여기서 회전 부분은 상대 회전이고, 이동 부분은 $\mathbf{A}$ 좌표계에서 본 상대 이동이다.

### 7.1 이해하기 쉬운 이론식

$$
\boldsymbol{\varepsilon}(\mathbf{A},\mathbf{B})
=
\log\!\left(\mathbf{A}^{-1}\mathbf{B}\right)
$$

여기서 $\log(\cdot)$는 일반 숫자의 자연로그가 아니다. $\mathrm{SE}(3)$ 행렬을 작은 회전 $3$개와 작은 이동 $3$개, 총 $6$개의 오차 숫자로 바꾸는 Lie logarithm이다.

개념적으로는 다음과 같이 생각할 수 있다.

$$
\log:\mathrm{SE}(3)\rightarrow\mathfrak{se}(3)
$$

$$
\boldsymbol{\varepsilon}
=
\begin{bmatrix}
\boldsymbol{\omega} \\
\mathbf{v}
\end{bmatrix}
\in\mathbb{R}^{6}
$$

$\boldsymbol{\omega}\in\mathbb{R}^{3}$은 회전오차이고, $\mathbf{v}\in\mathbb{R}^{3}$은 이동오차다.

---

<a id="toc-section-9"></a>

## 8. 모든 방법이 공유하는 기본 목적함수

물체 좌표계의 $j$번째 3차원 코너를 $\mathbf{p}_j$, 측정 픽셀을 $\mathbf{u}_{o,j}$, 내부파라미터와 왜곡을 포함한 투영함수를 $\pi(\mathbf{K},\mathbf{D},\cdot)$라 하자. 관측 $o$가 고정 카메라에서 왔다면 예측 픽셀은 다음과 같다.

$$
\widehat{\mathbf{u}}_{o,j}
=
\pi\!\left(
\mathbf{K}_i,\mathbf{D}_i,
\mathbf{C}_i^{-1}\mathbf{Q}_{s(o)}\mathbf{p}_j
\right).
$$

그리퍼 카메라 관측이면 다음과 같다.

$$
\widehat{\mathbf{u}}_{o,j}
=
\pi\!\left(
\mathbf{K}_g,\mathbf{D}_g,
(\mathbf{G}_{e(o)}\mathbf{X})^{-1}
\mathbf{Q}_{s(o)}\mathbf{p}_j
\right).
$$

현재 기본 visual 목적함수는 native-pixel $u,v$ 성분에 `soft_l1`, $f_v=2\,\mathrm{px}$를 적용한다.

$$
\mathcal{E}_{\mathrm{vis}}
=
\sum_{o,j}\sum_{q\in\{u,v\}}
f_v^2\rho_{\mathrm{softL1}}
\left(
\frac{(\widehat{u}_{o,j,q}-u_{o,j,q})^2}{f_v^2}
\right).
$$

### 8.1 식에 나오는 기호

$\mathcal{E}_{\mathrm{vis}}$의 아래첨자 vis는 vision, 즉 카메라로 본 정보에서 나온 오차라는 뜻이다. 로봇 관절값에서 나온 FK 정보와 구분하려고 붙인 이름이다.

| 기호 | 뜻 | 값을 아는가 |
|---|---|---|
| $\mathbf{C}_i$ | 고정 카메라 $i$의 자세. 베이스 기준 | 모른다. 찾아야 한다 |
| $\mathbf{X}$ | 그리퍼와 그리퍼 카메라 사이의 hand-eye 변환 | 모른다. 찾아야 한다 |
| $\mathbf{Q}_s$ | 세트 $s$의 공통 큐브 자세. 베이스 기준 | 방식마다 다르다. 9장부터 설명 |
| $\mathbf{p}_j$ | target 좌표계의 3차원 코너 | 안다. target geometry |
| $\mathbf{u}_{o,j}$ | 이미지에서 검출한 native-pixel 코너 | 안다. detector 측정값 |
| $\mathbf{K}_i,\mathbf{D}_i$ | 내부파라미터와 왜곡계수 | 안다. 모든 단계에서 고정 |
| $\mathbf{G}_e$ | 이벤트 $e$의 그리퍼 자세. 베이스 기준 | 안다. 로봇 FK 값 |
| $\pi(\cdot)$ | 3차원 코너를 왜곡된 이미지 픽셀로 투영 | OpenCV camera model |

읽는 순서는 다음과 같다. 안쪽부터 밖으로 나간다.

1. 현재 카메라와 target pose로 각 3차원 코너의 픽셀 위치를 예측한다.
2. 예측 픽셀에서 detector가 측정한 픽셀을 뺀다.
3. $u,v$ 각 성분에 같은 robust loss를 적용한다.
4. 고정 카메라와 그리퍼 카메라의 모든 선택 관측을 합한다.

### 8.2 이 식이 뜻하는 것

$\mathcal{E}_{\mathrm{vis}}$를 최소로 만드는 $\mathbf{C}_i$와 $\mathbf{X}$를 찾는 것이 캘리브레이션이다. 측정 픽셀, $\mathbf{K}$, $\mathbf{D}$, target geometry, $\mathbf{G}$는 움직일 수 없고, 행에서 선언한 pose 변수만 움직인다.

우리 캘리브레이션에서 비교하는 세가지 FK 방식은 이 관측식 자체가 다르지 않다. $\mathbf{Q}_s$를 무엇으로 정의하느냐가 다르다.

---

<a id="toc-section-10"></a>

## 9. 방법 1: no-FK(vision)

### 9.1 쉬운 설명

로봇이 알려주는 큐브 자세를 사용하지 않는다. 카메라 위치, hand-eye, 세트별 큐브 자세를 vision 관측만으로 함께 찾는다.

### 9.2 정확한 목적함수

$$
\left\{
\{\widehat{\mathbf{C}}_i\},
\widehat{\mathbf{X}},
\{\widehat{\mathbf{O}}_s\}
\right\}
=
\underset{
\{\mathbf{C}_i\},\mathbf{X},\{\mathbf{O}_s\}
}{\operatorname{argmin}}
\mathcal{E}_{\mathrm{vis}}
\left(
\{\mathbf{C}_i\},
\mathbf{X},
\{\mathbf{O}_s\}
\right)
$$

이때 공통 자세는 자유변수다.

$$
\mathbf{Q}_s=\mathbf{O}_s
$$

여기서 비슷해 보이는 세 기호를 구분하고 넘어가자.

| 기호 | 무엇인가 |
|---|---|
| $\mathbf{Q}_s$ | 카메라 예측을 맞출 기준 자리 |
| $\mathbf{O}_s$ | 그 자리를 채우는 자유변수 |
| $\widehat{\mathbf{O}}_s$ | 최적화가 끝난 뒤 나온 추정 결과 |

따라서 $\mathbf{Q}_s=\mathbf{O}_s$는 두 값이 같다는 뜻이 아니라, 기준 자리를 고정값이 아니라 자유변수로 채운다는 선언이다. `raw-FK-fixed`는 같은 자리를 raw FK와 mechanical frame map으로 채운다.

셋 중 어느 것도 큐브의 진짜 자세는 아니다. 진짜 자세는 10.3절에서 $\mathbf{O}^{\mathrm{true}}_s$로 따로 표기한다.

### 9.3 장점과 주의점

- raw FK의 계통오차가 캘리브레이션에 직접 들어오지 않는다.
- 대신 세트마다 $6$자유도인 $\mathbf{O}_s$가 추가되어 미지수가 많아진다.
- 카메라 연결이나 로봇 움직임이 충분하지 않으면 전체 좌표계의 gauge가 흔들릴 수 있다.
- `no-FK(vision)`도 $\mathbf{G}_e$는 사용한다. 큐브 FK prior $\mathbf{F}_s$만 사용하지 않는다.

---

<a id="toc-section-11"></a>

## 10. 방법 2: raw-FK-fixed

### 10.1 쉬운 설명

로봇 controller가 기록한 raw cube-center FK pose를 고정 상수로 두고 움직이지 못하게 한다. controller tool4 frame과 AprilTag cube object frame은 축 정의만 다르므로, 영상에서 적합한 $\boldsymbol{\Delta}_{train}$ 대신 사전 등록된 mechanical 좌표변환 $\mathbf{M}=R_y(180^\circ)$를 적용한다. 두 frame의 원점은 모두 cube center이므로 $\mathbf{M}$의 translation은 0이다. 즉 cube 표적의 set별 6자유도를 최적화에서 제거하지만, 이 raw FK를 외부 정답이라고 가정하지는 않는다.

raw FK cube-center 자세와 tag-object 좌표계 정렬을 다음과 같이 정의한다.

$$
\mathbf{F}^{\mathrm{raw}}_s
\equiv
{}^{B}\mathbf{T}^{\mathrm{FK}}_{\mathrm{cube\ center},s},
\qquad
\widetilde{\mathbf{F}}^{raw}_s
=
\mathbf{F}^{\mathrm{raw}}_s\mathbf{M},
\qquad
\mathbf{M}=
\begin{bmatrix}
-1&0&0&0\\
0&1&0&0\\
0&0&-1&0\\
0&0&0&1
\end{bmatrix}.
$$

큐브 자세는 다음 제약을 만족해야 한다.

$$
\mathbf{O}_s=\widetilde{\mathbf{F}}^{raw}_s
$$

### 10.2 정확한 목적함수

$$
\left\{
\{\widehat{\mathbf{C}}_i\},
\widehat{\mathbf{X}}
\right\}
=
\underset{
\{\mathbf{C}_i\},\mathbf{X}
}{\operatorname{argmin}}
\mathcal{E}_{\mathrm{vis}}
\left(
\{\mathbf{C}_i\},
\mathbf{X},
\{\widetilde{\mathbf{F}}^{raw}_s\}
\right)
$$

실제 A3는 board pose도 자유변수로 포함하며, $\mathcal{E}_{\mathrm{vis}}$는 A3 행에 포함된 모든 native-pixel corner residual의 목적함수다. cube pose만 변수 목록에서 제거된다.

### 10.3 왜 FK 오차가 카메라로 전파되는가?

실제 큐브 자세를 $\mathbf{O}^{\mathrm{true}}_s$라고 하자. raw FK와 mechanical frame map에 남는 계통오차 $\mathbf{D}$가 있으면 다음처럼 쓸 수 있다.

$$
\widetilde{\mathbf{F}}^{raw}_s
=
\mathbf{O}^{\mathrm{true}}_s\mathbf{D}
$$

그런데 최적화는 $\widetilde{\mathbf{F}}^{raw}_s$를 움직일 수 없다. 따라서 남은 오차를 줄이기 위해 $\mathbf{C}_i$나 $\mathbf{X}$가 잘못 움직일 수 있다.

raw-FK-fixed는 FK가 정확할 때 미지수가 적고 안정적이지만, FK에 공통적인 오정렬이 있으면 그 오차를 캘리브레이션 결과에 흡수시킬 위험이 있다.

### 10.4 카메라에도 계통오차가 있으면

카메라도 내부파라미터나 왜곡 보정이 조금 틀리면 항상 같은 방향으로 치우친 $\mathbf{Z}$를 내놓는다. 예를 들어 큐브를 늘 $3\,\mathrm{mm}$ 멀리 있다고 보고하는 식이다.

이 경우 raw-FK-fixed에서는 두 계통오차를 구분할 수 없다. $\widetilde{\mathbf{F}}^{raw}_s$가 고정되어 있으므로 어긋남은 전부 $\mathbf{C}_i$와 $\mathbf{X}$로 밀려 들어가는데, 그 어긋남 안에는 FK가 틀린 몫과 카메라가 틀린 몫이 섞여 있다. 최적화 입장에서 두 몫은 똑같이 생겼고, 어느 쪽 책임인지 판단할 정보가 목적함수 안에 없다. 결국 둘 다 캘리브레이션 결과에 흡수된다.

더 곤란한 점은 이 상황에서 잔차가 오히려 작게 나온다는 것이다. $\mathbf{C}_i$가 카메라 편향만큼 반대로 움직이면 관측은 깔끔하게 설명되기 때문이다. 수렴은 잘 된 것처럼 보이지만 $\mathbf{C}_i$는 물리적 진짜 위치에서 벗어나 있다.

---

<a id="toc-section-12"></a>

## 11. 방법 3: corrected-FK

### 11.1 가장 쉬운 설명

raw FK를 정답처럼 고정하지 않는다. 먼저 **학습 EIH cube 코너만** 사용해 raw FK와 tag-object 좌표계 사이의 공통 정렬 $\boldsymbol{\Delta}$를 구한다. 최종 캘리브레이션에서는 세트별 cube pose $\mathbf{O}_s$를 계속 자유변수로 두고, 정렬 FK $\widetilde{\mathbf{F}}_s$와의 차이를 6차원 공분산으로 whitening한 robust soft factor를 추가한다.

따라서 현재 코드의 `corrected-FK`는 hard gate나 pose 교체가 아니다. vision과 FK가 가까우면 FK factor가 자세를 안정시키고, 멀면 Huber loss가 그 영향력을 연속적으로 낮춘다.

### 11.2 단계 1: board-free train-only FK 정렬

raw FK의 cube-center 좌표계와 영상에서 사용하는 tag-object 좌표계는 그대로 같다고 가정할 수 없다. 현재 구현은 학습 세트의 EIH cube 코너와 robot gripper pose를 사용해 $\mathbf{X}$와 공통 우측 정렬 $\boldsymbol{\Delta}$를 함께 추정한다.

$$
\widetilde{\mathbf{F}}_s
=
\mathbf{F}_s\boldsymbol{\Delta}
$$

정렬 단계가 만족하는 핵심 관계는 다음과 같다.

$$
\mathbf{G}_e\mathbf{X}\,{}^{C_g}\mathbf{T}_{O,e}
\approx
\mathbf{F}_{s(e)}\boldsymbol{\Delta}
$$

여기서 board, 고정 카메라 관측, held-out event는 사용하지 않는다. 실제 최적화는 PnP pose 차이가 아니라 왜곡을 포함한 native-pixel cube 코너 재투영잔차를 사용한다.

### 11.3 단계 2: 6차원 FK 오차와 공분산 whitening

세트별 자유 cube pose $\mathbf{O}_s$와 정렬 FK 사이의 상대변환을 6차원으로 쓴다.

$$
\mathbf{e}^{\mathrm{FK}}_s
=
\begin{bmatrix}
\operatorname{Log}_{\mathrm{SO}(3)}
(\mathbf{R}(\mathbf{O}_s)^{\mathsf T}
 \mathbf{R}(\widetilde{\mathbf{F}}_s))
\\
\mathbf{t}(\mathbf{O}_s^{-1}\widetilde{\mathbf{F}}_s)
\end{bmatrix}
\in\mathbb{R}^{6}.
$$

잔차 순서는 $[r_x,r_y,r_z,t_x,t_y,t_z]$이고 단위는 radian과 metre다. 세트별 FK 공분산을 $\boldsymbol{\Sigma}_s$라 하고 Cholesky 분해를 적용하면 다음 whitened 잔차를 얻는다.

$$
\boldsymbol{\Sigma}_s=\mathbf{L}_s\mathbf{L}_s^{\mathsf T},
\qquad
\mathbf{w}_s=\mathbf{L}_s^{-1}\mathbf{e}^{\mathrm{FK}}_s.
$$

이렇게 하면 회전과 이동을 임의의 가중치로 더하지 않고, 측정된 불확실성의 표준편차 단위로 비교할 수 있다. 공분산은 대칭 positive-definite $6\times6$이어야 한다.

### 11.4 단계 3: raw-corner vision 항과 robust FK factor의 결합

최종 A4 목적함수는 다음 구조다.

$$
\underset{\{\mathbf{C}_i\},\mathbf{X},\mathbf{B},\{\mathbf{O}_s\}}
{\operatorname{argmin}}
\left[
\sum_{o,j,q}
f_v^2\rho_{\mathrm{softL1}}
\left(
\frac{(\widehat{u}_{o,j,q}-u_{o,j,q})^2}{f_v^2}
\right)
+
\sum_s\sum_{k=1}^{6}
f_{\mathrm{FK}}^2\rho_{\mathrm{Huber}}
\left(
\frac{w_{s,k}^2}{f_{\mathrm{FK}}^2}
\right)
\right].
$$

$o$는 camera-event-target 관측, $j$는 코너, $q\in\{u,v\}$는 픽셀 성분이다. 현재 기본값은 $f_v=2\,\mathrm{px}$, $f_{\mathrm{FK}}=3$이다. 즉 FK 오차가 대략 $3\sigma$ 안에서는 quadratic하게 작용하고, 그 밖에서는 Huber가 영향 증가를 선형으로 제한한다. SciPy에는 전체 loss를 `linear`로 두고 두 종류의 M-estimator를 각각 residual에 명시적으로 인코딩하므로, 픽셀과 FK에 같은 단위의 loss가 잘못 적용되지 않는다.

### 11.5 hard gate가 아닌 이유

$\mathbf{O}_s$는 최적화 변수에서 제거되지 않는다. 또한 어떤 임계값으로 FK pose와 vision pose 중 하나를 선택하지도 않는다. 각 세트는 모든 반복에서 vision 코너와 FK factor의 영향을 동시에 받으며, 영향의 상대 크기는 $\boldsymbol{\Sigma}_s$와 Huber 함수로 결정된다. 따라서 갑작스러운 all-or-nothing 전환이 없고, 공분산이 큰 방향은 약하게, 작은 방향은 강하게 구속된다.

### 11.6 실측 covariance와 현재 결과의 경계

`--fk_covariance_json`이 없으면 코드는 Simulation과 맞춘 등방성 prior인 이동 $2.0\,\mathrm{mm}$, 회전 $0.30^\circ$를 사용한다. 이 결과는 **preflight**이며 confirmatory 결과가 아니다. 다음주 Independent External GT 평가 전에는 GT를 보지 않고 미리 등록한 물리 반복측정 covariance가 필요하다.

따라서 현재 A4 수치는 ``Simulation covariance를 사용한 동일 marker population의 held-out reprojection preflight''로만 기술한다. A2보다 절대 정확도가 높다거나 corrected-FK factor의 우월성이 입증됐다고 기술하지 않는다.

6차원 표본 공분산의 rank는 반복 수 $N$에 대해 최대 $N-1$이므로, full-rank $6\times6$ 공분산에는 최소 $N=7$개의 독립 반복이 필요하다. 코드도 7회 미만, 비대칭, 비양정치 covariance를 거부한다. 실제로는 안정적인 추정을 위해 7회보다 충분히 많은 반복을 사용하는 편이 바람직하다.

### 11.7 A5: vision-aligned-FK-fixed post-hoc 진단

A5는 11.2절에서 만든 동일한 train-only 정렬 FK를 사용하지만, A4와 달리 cube pose를 자유변수로 두지 않는다.

$$
\mathbf{O}_s=\widetilde{\mathbf{F}}_s
=\mathbf{F}^{\mathrm{raw}}_s\boldsymbol{\Delta}_{\mathrm{train}}
$$

$$
\left\{\{\widehat{\mathbf{C}}_i\},\widehat{\mathbf{X}},\widehat{\mathbf{B}}\right\}
=
\underset{\{\mathbf{C}_i\},\mathbf{X},\mathbf{B}}
{\operatorname{argmin}}
\mathcal{E}_{\mathrm{vis}}
\left(\{\mathbf{C}_i\},\mathbf{X},\mathbf{B},
\{\widetilde{\mathbf{F}}_s\}\right).
$$

A3와 A5는 모두 set당 cube 6자유도를 제거하며 목적함수에는 visual residual 한 항만 남는다. 차이는 A3가 영상과 무관한 mechanical $R_y(180^\circ)$를 쓰고, A5가 train EIH cube 영상으로 적합한 $\boldsymbol{\Delta}_{\mathrm{train}}$을 쓴다는 점이다. A4와 A5는 동일 aligned-FK artifact를 공유하므로 soft factor와 hard constraint의 차이를 진단할 수 있다.

그러나 A5는 결과를 확인한 뒤 원인을 분리하기 위해 추가되었고 aligned pose도 train 영상에서 왔다. 따라서 `posthoc_diagnostic`으로만 보고하며, 독립 실측 correction, 외부 GT 또는 최종 우월 방법으로 부르지 않는다. 독립 실측 6-DoF correction을 사용하는 미래 행은 A6로 예약한다.

---

<a id="toc-section-13"></a>

## 12. 네 FK 방식 비교

| 방법 | 공통 큐브 자세 $\mathbf{Q}_s$ | 큐브 자세의 상태 | 핵심 의미 |
|---|---:|---|---|
| `no-FK(vision)` | 없음 | $\mathbf{O}_s$ 자유변수 | cube FK prior 없이 raw-corner vision으로 추정 |
| `raw-FK-fixed` | $\mathbf{O}_s=\widetilde{\mathbf{F}}^{raw}_s$ | cube 변수 제거 | raw FK와 mechanical frame map을 hard constraint로 사용 |
| `corrected-FK factor` | $\mathbf{w}_s=\mathbf{L}_s^{-1}\mathbf{e}^{\mathrm{FK}}_s$ | $\mathbf{O}_s$ 자유변수 | vision 목적함수에 covariance-whitened robust FK factor 추가 |
| `vision-aligned-FK-fixed` | $\mathbf{O}_s=\mathbf{F}^{raw}_s\boldsymbol{\Delta}_{train}$ | cube 변수 제거 | train-vision-aligned FK를 hard constraint로 사용하는 post-hoc 진단 |

한 문장으로 요약하면 다음과 같다.

- `no-FK(vision)`: 큐브 자세도 직접 찾는다.
- `raw-FK-fixed`: 큐브 자세를 raw FK와 mechanical frame map에 못 박는다.
- `corrected-FK factor`: 큐브 자세를 자유롭게 두고 정렬 FK를 불확실성이 있는 soft measurement로 사용한다.
- `vision-aligned-FK-fixed`: 같은 정렬 FK에 큐브 자세를 못 박아 이전 A3 성능의 원인을 진단한다.

---

<a id="toc-section-14"></a>

## 13. Unified 방식

### 13.1 쉬운 설명

고정 카메라와 그리퍼 카메라의 raw-corner residual을 하나의 벡터로 쌓고, 두 경로가 공유하는 target pose와 hand-eye를 포함한 모든 행-선언 자유변수를 동시에 갱신한다.

이 문서에서 Eye-in-Hand(EIH)는 그리퍼 장착 카메라 관측, Eye-to-Hand(E2H)는 고정 카메라 관측을 뜻한다. 최적화 구조의 정식 표기는 코드의 `unified_joint_optimization`에 대응하는 Unified Joint Optimization과 `sequential_frozen_stage`에 대응하는 Sequential Frozen-Stage Optimization이다.

### 13.2 수식

$$
\boldsymbol{\theta}_{\mathrm{uni}}
=
\left(
\{\mathbf{C}_i\},
\mathbf{X},
\mathbf{B},
\{\mathbf{O}_s\}
\right)
$$

$$
\widehat{\boldsymbol{\theta}}_{\mathrm{uni}}
=
\underset{\boldsymbol{\theta}_{\mathrm{uni}}}
{\operatorname{argmin}}
\left[
\mathcal{E}^{\mathrm{px}}_{\mathrm{fix}}
+
\mathcal{E}^{\mathrm{px}}_{\mathrm{grip}}
+
\mathbb{1}_{\mathrm{FKfactor}}\mathcal{E}_{\mathrm{FK}}
\right]
$$

여기서 pixel 항은 각 행에 포함된 모든 native-pixel corner residual이다. A4/B1/B2에는 eligible set 9개에 대응하는 FK factor 9개, 즉 6D residual 54개가 추가된다. A3에서는 $\mathbf{O}_s=\widetilde{\mathbf{F}}^{raw}_s$, A5에서는 $\mathbf{O}_s=\mathbf{F}^{raw}_s\boldsymbol{\Delta}_{train}$을 고정하므로 cube pose가 자유변수 목록에서 제거된다.

Unified의 핵심은 한쪽 관측이 $\mathbf{O}_s$를 움직이면, 같은 $\mathbf{O}_s$를 공유하는 다른 쪽의 $\mathbf{C}_i$와 $\mathbf{X}$도 같은 반복 안에서 영향을 받는다는 것이다. $e_{cross}$ 같은 평가 지표는 이 목적함수에 들어가지 않는다.

---

<a id="toc-section-15"></a>

## 14. Sequential(`seq`, `-Unified`) 방식

### 14.1 용어와 실제 코드 계약

현재 real-data Table 1의 `seq` 또는 `-Unified`는 두 독립 좌표계를 따로 푼 뒤 Kabsch로 맞추는 legacy simulation 방식이 아니다. 하나의 베이스 좌표계 안에서 다음 두 단계를 순서대로 실행하며, stage 1 결과를 stage 2에서 고정한다.

### 14.2 Stage 1: Eye-in-Hand(EIH)만 최적화

그리퍼 카메라 관측만 사용해 $\mathbf{X}$와 행에 포함된 target pose를 푼다.

$$
(\widehat{\mathbf{X}},\widehat{\mathbf{B}},
 \{\widehat{\mathbf{O}}_s\})
=
\underset{\mathbf{X},\mathbf{B},\{\mathbf{O}_s\}}
{\operatorname{argmin}}
\left[
\mathcal{E}^{\mathrm{px}}_{\mathrm{grip}}
+
\mathbb{1}_{\mathrm{B1}}\mathcal{E}_{\mathrm{FK}}
\right].
$$

A0는 board만, A1은 board와 cube, B1은 board와 cube에 corrected-FK factor까지 사용한다. 행에 없는 target 변수와 잔차는 식에서 제거된다.

### 14.3 Freeze boundary

Stage 1에서 얻은 $\widehat{\mathbf{X}}$, $\widehat{\mathbf{B}}$, $\widehat{\mathbf{O}}_s$는 stage 2의 상수다. 이후 fixed-camera 관측이 이 값을 바꾸지 못하고, stage 2 뒤 stage 1로 돌아가는 alternating pass도 없다.

### 14.4 Stage 2: fixed camera별 최적화

각 고정 카메라 $i$는 자신의 raw-corner residual만 사용해 $\mathbf{C}_i$를 푼다.

$$
\widehat{\mathbf{C}}_i
=
\underset{\mathbf{C}_i}{\operatorname{argmin}}
\mathcal{E}^{\mathrm{px}}_{\mathrm{fix},i}
(\mathbf{C}_i\mid
 \widehat{\mathbf{B}},\{\widehat{\mathbf{O}}_s\}).
$$

target과 hand-eye가 고정되어 있으므로 camera block들은 수학적으로 분리 가능하다.

### 14.5 Unified와 Sequential의 핵심 차이

Unified는 EIH/E2H residual을 동시에 보고 $\mathbf{C}_i$, $\mathbf{X}$, target pose 사이에 양방향 feedback이 있다. Sequential은 EIH가 target과 $\mathbf{X}$를 먼저 정하고, E2H는 그 결과를 받아 $\mathbf{C}_i$만 갱신한다. 두 방식 모두 같은 raw detections, $\mathbf{K},\mathbf{D}$, split, solver 설정을 사용한다.

---

<a id="toc-section-16"></a>

## 15. Board-only, Cube-only, Both

FK 방식과 별개로 어떤 타깃 관측을 사용할지도 선택한다.

### 15.1 Cube-only

큐브 관측 잔차만 사용한다.

$$
\mathcal{E}_{\mathrm{cube}}
=
\sum_{o\in\mathcal{O}_{\mathrm{cube}}}
\sum_j\sum_{q\in\{u,v\}}
f_v^2\rho_{\mathrm{softL1}}
\left((\widehat u_{o,j,q}-u_{o,j,q})^2/f_v^2\right)
$$

### 15.2 Board-only

베이스 좌표계에서 고정된 보드 자세를 $\mathbf{Q}^{\mathrm{board}}$라고 두면 보드 관측만 사용한다.

$$
\mathcal{E}_{\mathrm{board}}
=
\sum_{o\in\mathcal{O}_{\mathrm{board}}}
\sum_j\sum_{q\in\{u,v\}}
f_v^2\rho_{\mathrm{softL1}}
\left((\widehat u_{o,j,q}-u_{o,j,q})^2/f_v^2\right)
$$

보드는 로봇이 잡고 이동시키는 큐브 FK prior를 갖지 않는다. 따라서 이 프로젝트에서는 `Board-only + raw-FK-fixed`, `Board-only + corrected-FK factor`, `Board-only + vision-aligned-FK-fixed`를 정의하지 않는다.

### 15.3 Both

큐브와 보드의 잔차를 함께 사용한다.

$$
\mathcal{E}_{\mathrm{both}}
=
\mathcal{E}_{\mathrm{cube}}
+
\mathcal{E}_{\mathrm{board}}
$$

큐브는 여러 방향에서 보이기 쉽고, 보드는 많은 평면 코너를 제공한다. 두 타깃은 서로 다른 관측 정보를 보완한다.

---

<a id="toc-section-17"></a>

## 16. 왜 전체 조합이 $24$개가 아니라 $18$개인가?

형식적으로는 다음 세 축이 있다.

$$
2\;\text{solver modes}
\times
3\;\text{target modes}
\times
4\;\text{FK modes}
=24
$$

하지만 `Board-only`에는 큐브 FK prior가 없으므로 `raw-FK-fixed`, `corrected-FK factor`, `vision-aligned-FK-fixed`를 사용할 수 없다.

제외되는 조합 수는 다음과 같다.

$$
2\;\text{solver modes}
\times
1\;\text{board-only mode}
\times
3\;\text{invalid FK modes}
=6
$$

따라서 유효한 전체 조합은 다음과 같다.

$$
24-6=18
$$

이는 물리적으로 정의 가능한 개념 조합의 수다. 현재 real-data Table은 이 18개를 전부 실행하는 full factorial이 아니라, A0–A4/B1–B3의 8개 사전 정의 비교행과 결과 원인을 분리하기 위해 추가한 post-hoc A5 한 행을 실행한다. A6는 독립 실측 correction label을 위한 미실행 예약이다.

---

<a id="toc-section-18"></a>

## 17. corrected-FK와 Ridge 출력 후보정은 다르다

이 둘은 자주 혼동되지만 서로 다른 단계다.

### 17.1 corrected-FK

캘리브레이션 목적함수 안에서 raw FK 자세를 train-only vision으로 정렬하고, 정렬 pose와의 6차원 오차를 공분산으로 whitening한 robust factor를 사용한다.

$$
\mathbf{F}_s
\rightarrow
\widetilde{\mathbf{F}}_s=\mathbf{F}_s\boldsymbol{\Delta}
\rightarrow
\mathbf{w}_s=\mathbf{L}_s^{-1}
\mathbf{e}^{\mathrm{FK}}_s
\rightarrow
\mathcal{E}_{\mathrm{vis}}+\mathcal{E}_{\mathrm{FK}}
$$

### 17.2 선택적 Ridge 출력 후보정

legacy simulation의 `corr` 출력 후보정은 캘리브레이션이 끝난 뒤 예측 위치의 잔차를 회귀로 학습한다. 이것은 현재 real-data Table 1의 A4 corrected-FK factor가 아니다.

예측 위치를 다음과 같이 둔다.

$$
\mathbf{p}_s
=
\begin{bmatrix}
x_s & y_s & z_s
\end{bmatrix}^{\mathsf T}
$$

특징벡터는 다음과 같다.

$$
\boldsymbol{\phi}(\mathbf{p}_s)
=
\begin{bmatrix}
1 & x_s & y_s
\end{bmatrix}^{\mathsf T}
$$

train set의 FK proxy 위치를 $\mathbf{f}_s$라고 하면 학습 잔차는 다음과 같다.

$$
\mathbf{y}_s
=
\mathbf{f}_s-\mathbf{p}_s
$$

행렬을 쌓아 Ridge 계수 $\mathbf{W}$를 구한다.

$$
\widehat{\mathbf{W}}
=
\left(
\boldsymbol{\Phi}^{\mathsf T}\boldsymbol{\Phi}
+
\lambda\mathbf{I}
\right)^{-1}
\boldsymbol{\Phi}^{\mathsf T}\mathbf{Y}
$$

보정 예측은 다음과 같다.

$$
\mathbf{p}^{\mathrm{post}}_s
=
\mathbf{p}_s
+
\boldsymbol{\phi}(\mathbf{p}_s)^{\mathsf T}
\widehat{\mathbf{W}}
$$

이 Ridge 결과는 FK proxy와의 일치도를 높이는 출력 보정이다. 실제 외부 측정 장비의 정답과 비교한 절대 물리 정확도라고 말하면 안 된다.

---

<a id="toc-section-19"></a>

## 18. 평가 지표를 해석하는 방법

### 18.1 시뮬레이션

시뮬레이션은 생성할 때 실제 변환을 알고 있으므로 외부 ground truth와 직접 비교할 수 있다.

추정 변환을 $\widehat{\mathbf{T}}$라고 하고 정답을 $\mathbf{T}^{\star}$라고 하면 다음 오차를 사용할 수 있다.

$$
e_T
=
1000
\left\|
\mathbf{t}(\widehat{\mathbf{T}})
-
\mathbf{t}(\mathbf{T}^{\star})
\right\|_2
$$

$$
e_R
=
\frac{180}{\pi}
\cos^{-1}
\left(
\frac{
\operatorname{tr}
(\widehat{\mathbf{R}}^{\mathsf T}\mathbf{R}^{\star})-1
}{2}
\right)
$$

### 18.2 실데이터

실데이터에 외부 모션캡처나 정밀 측정장비가 없다면 진짜 $\mathbf{T}^{\star}$를 모른다. 따라서 다음 지표는 서로 구분해야 한다.

- 재투영오차: 이미지 관측을 얼마나 잘 다시 설명하는가
- 카메라 간 consistency: 여러 카메라의 예측이 서로 얼마나 일치하는가
- held-out FK-proxy 오차: 학습에 쓰지 않은 세트의 예측이 로봇 FK와 얼마나 일치하는가

FK proxy 오차는 다음처럼 쓸 수 있다.

$$
e_{\mathrm{proxy}}
=
\frac{1}{|\mathcal{S}_{\mathrm{test}}|}
\sum_{s\in\mathcal{S}_{\mathrm{test}}}
1000
\left\|
\widehat{\mathbf{p}}_s
-
\mathbf{p}^{\mathrm{FK}}_s
\right\|_2
$$

하지만 반드시 다음을 기억해야 한다.

$$
\text{FK-proxy agreement}
\neq
\text{absolute physical accuracy}
$$

### 18.3 Blind external GT 설계

독립 tracker 좌표계를 $W$라 하고 로봇 base와 cube를 같은 측정계에서 측정하면 다음 GT를 얻는다.

$$
\mathbf{T}^{B}_{cube,GT}
=
(\mathbf{T}^{W}_{B})^{-1}\mathbf{T}^{W}_{cube}.
$$

이때 calibration RGB camera, controller FK, A4 FK factor, A5의 $\boldsymbol{\Delta}_{train}$은 GT 생성에 사용하면 안 된다. 각 방법의 calibration artifact와 blind prediction을 GT 공개 전에 고정한 뒤 같은 pose ID끼리 다음 오차를 paired 비교한다.

$$
e_t=1000\left\|\mathbf{t}(\widehat{\mathbf{T}}^B_{cube})-
\mathbf{t}(\mathbf{T}^B_{cube,GT})\right\|_2,
\qquad
e_R=\operatorname{angle}\!\left(
\mathbf{R}_{GT}^{\mathsf T}\widehat{\mathbf{R}}\right).
$$

코드 최소값은 독립 camera-installation session 2개지만, 최종 권장 설계는 5 sessions $\times$ 30 blind poses다. workspace 위치·거리·입사각 strata를 GT 공개 전에 고정하고 session을 먼저, pose를 두 번째로 재표집하는 paired hierarchical bootstrap을 사용한다. Primary contrast는 A4 대 A2, secondary contrast는 A4 대 A5이며, translation/rotation/P95/failure/ADD-S와 비열등성 margin도 GT 공개 전에 등록한다. 이 Independent External GT 평가는 다음주 예정 태스크로 분리하고, 실행 계약은 `protocol_templates/CAPTURE_CAMPAIGN_PROTOCOL.md`와 `external_gt_eval_manifest_TEMPLATE.json`에 있다.

### 18.4 현재 Session04에서 허용되는 비교 결론

Primary pixel 비교는 같은 marker population을 공유하는 행끼리만 수행한다. 동일한 cube+board 관측을 사용하는 A1과 A2에서 own held-out overall RMSE는 각각 $4.0837\,\mathrm{px}$와 $3.8901\,\mathrm{px}$다. 따라서 이 비교가 지원하는 결론은 ``동일 관측·초기값·solver에서 Unified Joint Optimization이 Sequential Frozen-Stage Optimization보다 held-out reprojection을 낮췄다''이다. 새로운 작업 위치의 절대 자세 정확도가 더 높다는 결론은 아니다.

A2와 A4의 own held-out overall은 각각 $3.8901\,\mathrm{px}$와 $3.8899\,\mathrm{px}$이고, cube held-out은 $3.5958\,\mathrm{px}$와 $3.5805\,\mathrm{px}$다. 현재 A4는 실측 covariance가 아니라 Simulation prior를 사용하므로 이 작은 차이로 corrected-FK factor의 우월성을 주장하지 않는다. 허용되는 표현은 ``A4는 Simulation covariance 기반 preflight에서 A2와 유사한 동일-population held-out reprojection을 보였다''이다.

A5의 own held-out overall은 $3.7270\,\mathrm{px}$, board/cube held-out은 각각 $3.8804/3.2274\,\mathrm{px}$로 현재 내부 수치상 A4보다 낮다. Fixed-to-Fixed cube transfer도 A5 $3.4706\,\mathrm{px}$가 A4 $4.0375\,\mathrm{px}$보다 낮지만, Fixed-to-Fixed board transfer는 A5 $4.8563\,\mathrm{px}$가 A4 $3.1156\,\mathrm{px}$보다 높다. 즉 내부 지표에서도 A5는 모든 표적·범위의 일관된 승자가 아니다. 또한 A5는 train EIH cube 영상으로 적합한 aligned FK를 hard-fixed하고 결과 확인 후 추가한 post-hoc 진단이다. 따라서 이 결과는 ``이전 A3의 낮은 오차가 vision-aligned hard constraint에서 왔다''는 원인 설명에는 사용할 수 있지만, A5가 A4보다 실제 공간에서 정확하다는 주장에는 사용할 수 없다. 현재 확증 대표 행은 A2, 불확실성을 다루는 방법론적 확장 후보는 A4, 원인 진단은 A5이며 A4/A5 물리 순위는 다음주 Independent External GT 이후 결정한다.

### 18.5 논문 실험 결과 본문용 서술

모든 비교행에 동일한 frozen corner 관측, event-grouped/set-stratified train–held-out split, camera intrinsic, target geometry, solver 설정 및 train-only shared initialization을 적용하였다. 동일한 board+cube marker population을 사용하는 vision 조건에서 Sequential Frozen-Stage Optimization인 A1의 held-out reprojection RMSE는 $4.0837\,\mathrm{px}$였고, Unified Joint Optimization인 A2는 $3.8901\,\mathrm{px}$로 $4.74\%$ 감소하였다. 표적별로는 board가 $4.0645\rightarrow3.9840\,\mathrm{px}$, cube가 $4.1402\rightarrow3.5958\,\mathrm{px}$로 감소했다. 두 행은 최적화 구조 외의 입력과 목적함수를 공유하므로, 이 결과는 Eye-in-Hand(EIH)와 Eye-to-Hand(E2H) 관측 사이의 양방향 Unified feedback이 동일 marker population의 held-out reprojection을 개선했음을 지원한다. 반면 raw FK cube pose를 set별 hard constraint로 고정한 A3는 held-out overall $4.7835\,\mathrm{px}$와 cube $6.3959\,\mathrm{px}$를 보여, raw tool4/mechanical pose를 외부 정답으로 간주할 수 없음을 확인했다. Corrected-FK factor를 추가한 A4의 held-out overall은 $3.8899\,\mathrm{px}$로 A2와 사실상 동일했으며, 현재 A4/B1/B2는 실측 FK covariance가 아닌 Simulation prior를 사용한 preflight다. Train-vision-aligned FK를 hard-fixed한 post-hoc A5는 held-out overall $3.7270\,\mathrm{px}$를 보였지만, 이는 이전 A3 성능의 원인을 설명하는 진단이지 독립 물리 정확도 증거가 아니다. 따라서 현재 확증 대표 행은 A2이고 A4는 방법론적 확장 preflight, A5는 원인 진단이다. 본 실험은 Unified feedback과 raw-FK hard-fix의 효과를 동일-population 내부 held-out reprojection에서 입증하지만, A4/A5 중 실제 공간에서 더 정확한 방법이나 새로운 작업 위치의 절대 물리 정확도는 다음주 Independent External GT 전에는 결정하지 않는다.

### 18.6 Paper-ready English Results paragraph

All methods were evaluated using the same frozen corner observations, event-grouped and set-stratified train–held-out split, camera intrinsics, target geometry, solver settings, and train-only shared initialization. Under the vision-only condition with an identical board-and-cube marker population, A1 (Sequential Frozen-Stage Optimization) achieved a held-out reprojection RMSE of $4.0837\,\mathrm{px}$, whereas A2 (Unified Joint Optimization) achieved $3.8901\,\mathrm{px}$, corresponding to a $4.74\%$ reduction. By target, the board RMSE decreased from $4.0645$ to $3.9840\,\mathrm{px}$, and the cube RMSE decreased from $4.1402$ to $3.5958\,\mathrm{px}$. Because A1 and A2 share the same observations, initialization, and objective terms and differ only in optimization structure, these results support the conclusion that bidirectional feedback between Eye-in-Hand (EIH) and Eye-to-Hand (E2H) observations improves held-out reprojection within the same marker population. In contrast, A3, which hard-fixed each set-specific cube pose to raw FK transformed only by the prescribed mechanical mapping, yielded an overall held-out RMSE of $4.7835\,\mathrm{px}$ and a cube RMSE of $6.3959\,\mathrm{px}$, showing that raw tool4/mechanical poses cannot be treated as external ground truth. A4, which added a corrected-FK factor, achieved $3.8899\,\mathrm{px}$ overall and remains a preflight because its covariance prior is simulation-derived. Post-hoc A5 hard-fixed the same train-vision-aligned FK targets and achieved $3.7270\,\mathrm{px}$ overall; this diagnoses the source of the former A3 result but is neither an independent correction nor evidence of superior physical accuracy. A2 is therefore the current confirmatory representative, A4 is the uncertainty-aware extension candidate, and A5 is a causal diagnostic. Blind external ground truth is required to determine the physical ranking of A4 and A5.

### 18.7 Paper-ready English Table 1 caption

**Table 1. Quantitative comparison of the calibration variants on Session04.** All reprojection values are root-mean-square errors (RMSEs) in native distorted-pixel coordinates; lower is better. The variants use the same frozen detection pool, event-grouped and set-stratified train–held-out split, camera intrinsics, target geometry, solver settings, and train-only shared initialization, while the marker population, optimization structure, and cube-pose treatment vary as specified. ``Own Held-out Overall'' is evaluated on each row's own marker population and is therefore compared only between rows with identical populations; ``Board/Cube Held-out'' reports target-specific RMSE. A3 applies the prescribed mechanical frame mapping to raw FK without vision alignment. A4, B1, and B2 use a simulation-derived covariance prior and are preflight evaluations. A5 hard-fixes the train-vision-aligned FK targets and is a post-hoc diagnostic. Boldface denotes the lowest target-specific held-out RMSE among Complete confirmatory rows only. ``Convergence $3/3$'' indicates successful solver termination for all three initialization seeds, not global optimality or absolute physical accuracy. External ground truth is not used in this table.

### 18.8 Paper-ready English column labels and table notes

논문 표에서는 결합된 ``Board/Cube Held-out'' 열을 두 열로 분리해 각 숫자의 의미를 직접 드러낸다. 권장 최종 열 순서는 다음과 같다.

| Current report label | Paper-ready label |
|---|---|
| Method | Method |
| Marker Set | Marker Population |
| Optimization | Optimization Structure |
| Cube Pose | Cube-Pose Treatment |
| Train Overall | Train RMSE (px) |
| Own Held-out Overall | Own Held-Out RMSE (px) |
| Board Held-out | Board Held-Out RMSE (px) |
| Cube Held-out | Cube Held-Out RMSE (px) |
| Convergence | Conv. |
| Status | Status |

긴 셀 문자열은 `sequential_frozen_stage`를 ``Sequential Frozen-Stage'', `unified_joint_optimization`을 ``Unified Joint'', `estimated`를 ``Vision-estimated'', `raw-FK-fixed`를 ``Raw-FK fixed$^{\ddagger}$'', `corrected-FK-factor`를 ``Corrected-FK factor$^{\dagger}$'', `vision-aligned-FK-fixed`를 ``Aligned-FK fixed$^{\S}$''로 줄여 쓴다. A4, B1, B2에는 $\dagger$, A5에는 $\S$를 표시한다.

**Paper-ready table note.** Values are RMSEs in native distorted-pixel coordinates; lower is better. Own held-out RMSE is computed on each method's declared marker population, so only rows with identical populations are directly comparable. Boldface indicates the lowest target-specific held-out RMSE among Complete confirmatory rows, and N/A indicates that the corresponding target was excluded. $^{\dagger}$ denotes a preflight result using a simulation-derived covariance prior rather than a physically measured FK covariance. $^{\ddagger}$ denotes cube poses hard-fixed to controller raw FK after applying only the prescribed $R_y(180^\circ)$ mechanical frame mapping. $^{\S}$ denotes the post-hoc A5 diagnostic that hard-fixes train-vision-aligned FK targets. Neither hard-fixed pose source is external ground truth. Conv. reports successful solver termination across three initialization seeds and does not imply global optimality or absolute physical accuracy.

### 18.9 Copy-ready LaTeX Table 1

아래 코드는 preamble의 `\usepackage{booktabs}`와 `\usepackage{graphicx}`를 사용한다. 두 단 논문을 기준으로 `table*`와 `\textwidth`를 사용했으며, 한 단 문서에서는 각각 `table`과 `\columnwidth`로 바꾸면 된다.

```latex
\begin{table*}[t]
\centering
\caption{Quantitative comparison of the predefined calibration variants on Session04. All reprojection values are RMSEs in native distorted-pixel coordinates; lower is better.}
\label{tab:session04_calibration}
\setlength{\tabcolsep}{3pt}
\renewcommand{\arraystretch}{1.12}
\resizebox{\textwidth}{!}{%
\begin{tabular}{@{}llllrrrrcl@{}}
\toprule
Method &
\shortstack{Marker\\Population} &
\shortstack{Optimization\\Structure} &
\shortstack{Cube-Pose\\Treatment} &
\shortstack{Train\\RMSE (px)} &
\shortstack{Own Held-Out\\RMSE (px)} &
\shortstack{Board Held-Out\\RMSE (px)} &
\shortstack{Cube Held-Out\\RMSE (px)} &
Conv. & Status \\
\midrule
A0 (Baseline)             & Board        & Sequential Frozen-Stage & N/A                 & 3.8202 & 4.0530 & 4.0530          & N/A             & 3/3 & Complete  \\
A1 (+Cube)                & Board + Cube & Sequential Frozen-Stage & Vision-estimated    & 3.7923 & 4.0837 & 4.0645          & 4.1402          & 3/3 & Complete  \\
A2 (+Unified)             & Board + Cube & Unified Joint            & Vision-estimated    & 3.7421 & 3.8901 & \textbf{3.9840} & \textbf{3.5958} & 3/3 & Complete  \\
A3$^{\ddagger}$ (Raw-FK fixed) & Board + Cube & Unified Joint       & Raw-FK fixed        & 5.1587 & 4.7835 & 4.1026          & 6.3959          & 3/3 & Complete  \\
A4$^{\dagger}$ (+FK factor)    & Board + Cube & Unified Joint       & Corrected-FK factor & 3.7441 & 3.8899 & 3.9884          & 3.5805          & 3/3 & Preflight \\
A5$^{\S}$ (Aligned-FK fixed)   & Board + Cube & Unified Joint       & Aligned-FK fixed    & 3.9648 & 3.7270 & 3.8804          & 3.2274          & 3/3 & Post-hoc \\
B1$^{\dagger}$ ($-$Unified)    & Board + Cube & Sequential Frozen-Stage & Corrected-FK factor & 3.7887 & 4.0783 & 4.0648 & 4.1182 & 3/3 & Preflight \\
B2$^{\dagger}$ ($-$Board)      & Cube         & Unified Joint        & Corrected-FK factor & 3.0269 & 4.4827 & N/A             & 4.4827          & 3/3 & Preflight \\
B3 ($-$Cube)               & Board        & Unified Joint            & N/A                 & 3.8202 & 4.0531 & 4.0531          & N/A             & 3/3 & Complete  \\
\bottomrule
\end{tabular}%
}
\vspace{2pt}

\parbox{\textwidth}{\footnotesize
Own held-out RMSE is computed on each method's declared marker population; only rows with identical populations are directly comparable. Boldface indicates the lowest target-specific held-out RMSE among Complete confirmatory rows. $^{\dagger}$ denotes a simulation-covariance preflight, $^{\ddagger}$ denotes raw-FK mechanical hard fixing, and $^{\S}$ denotes the post-hoc train-vision-aligned hard-fixed diagnostic. Neither fixed pose is external ground truth. Conv. reports successful solver termination across three initialization seeds and does not imply global optimality or absolute physical accuracy.}
\end{table*}
```

---

<a id="toc-section-20"></a>

## 19. 코드 흐름으로 다시 보기

아래 코드는 실제 구현을 이해하기 위한 축약 의사코드다.

```python
# 1. 모든 방법이 공유할 raw corner와 pre-fit 품질 마스크를 만든다.
corners = detect_native_pixel_corners(images)
corners = common_pnp_quality_mask(corners, max_rmse_fixed=3, max_rmse_gripper=5)
train, heldout = event_grouped_set_stratified_split(corners)

# 2. factor 행을 위해 train EIH cube corner만으로 FK 좌표계를 정렬한다.
delta = estimate_board_free_fk_alignment(train)
aligned_fk = {s: raw_fk[s] @ delta for s in sets}
raw_fixed_fk = {s: raw_fk[s] @ mechanical_Ry_180 for s in sets}

# 3. FK 사용 방식에 따라 cube 변수와 목적함수를 정한다.
if fk_mode == "none":
    cube_pose = "free"
    fk_factor = None

elif fk_mode == "fixed":
    cube_pose = raw_fixed_fk
    fk_factor = None

elif fk_mode == "aligned_fixed":
    cube_pose = aligned_fk
    fk_factor = None

elif fk_mode == "factor":
    cube_pose = "free"
    fk_factor = covariance_whitened_huber_factor(aligned_fk, Sigma)

# 4. 각 solver stage에서 fit → frame-prune → refit → rollback을 수행한다.
def guarded_stage_fit(stage_solver, stage_train):
    first = stage_solver(stage_train)
    kept = prune_event_camera_frames_by_mad(
        stage_train, first, max_fraction=0.30)
    candidate = stage_solver(kept, initialize_from=first)
    if full_robust_cost(candidate, stage_train) < full_robust_cost(first, stage_train):
        return candidate
    return first

if solver_mode == "unified":
    model = guarded_stage_fit(solve_joint_raw_corner_BA, train)
else:
    stage1 = guarded_stage_fit(solve_eih_raw_corner_BA, train_eih)
    model = guarded_stage_fit(
        solve_fixed_cameras_with_stage1_frozen, train_e2h)

# 5. heldout에서는 모든 parameter를 고정하고 같은 관측을 평가한다.
metrics = evaluate_without_refit(model, heldout)
```

---

<a id="toc-section-21"></a>

## 20. 교수님께 설명할 때의 추천 순서

1. 카메라마다 자기 좌표계를 사용하므로 같은 큐브도 다른 좌표로 보인다고 설명한다.
2. 모든 것을 로봇 베이스 좌표계 $B$로 옮기는 것이 캘리브레이션이라고 설명한다.
3. 변환행렬 $\mathbf{T}$가 회전 $\mathbf{R}$과 이동 $\mathbf{t}$를 포함한다고 설명한다.
4. 고정 카메라 경로 $\mathbf{C}_i\mathbf{Z}_{i,s}$와 그리퍼 카메라 경로 $\mathbf{G}_e\mathbf{X}\mathbf{Z}_{g,e}$를 설명한다.
5. 실제 solver는 pose 차이가 아니라 raw distorted-pixel corner 재투영오차를 최소화한다고 설명한다.
6. cube pose를 자유롭게 둘지, raw/aligned FK에 hard-fixed할지, covariance factor로 연결할지가 네 FK 방식의 차이라고 설명한다.
7. 두 카메라 계열을 한 residual vector로 함께 풀면 Unified이고, EIH를 먼저 푼 뒤 동결하여 fixed camera만 풀면 Sequential이라고 설명한다.
8. 실데이터의 FK 기반 평가는 절대 정답이 아니라 proxy라는 점을 마지막에 분명히 말한다.

---

<a id="toc-section-22"></a>

## 21. 최종 한 장 요약용 수식

### 공통 vision 목적함수

```latex
\mathcal{E}_{\mathrm{vis}}
=
\sum_{o,j}\sum_{q\in\{u,v\}}
f_v^2\rho_{\mathrm{softL1}}
\left(
\frac{(\widehat{u}_{o,j,q}-u_{o,j,q})^2}{f_v^2}
\right)
```

### 네 FK 방식

```latex
\begin{array}{ll}
\text{no-FK:} & \mathbf{O}_s\text{ is free},\\
\text{raw-FK-fixed:} & \mathbf{O}_s=\mathbf{F}^{raw}_s\mathbf{M},\\
\text{vision-aligned-FK-fixed:} & \mathbf{O}_s=\mathbf{F}^{raw}_s\boldsymbol{\Delta}_{train},\\
\text{corrected-FK factor:} & \mathbf{O}_s\text{ is free and }\mathcal{E}_{\mathrm{FK}}\text{ is added}.
\end{array}
```

### corrected-FK 핵심

```latex
\widetilde{\mathbf{F}}_s=\mathbf{F}_s\boldsymbol{\Delta},
\qquad
\mathbf{w}_s=\operatorname{chol}(\boldsymbol{\Sigma}_s)^{-1}
\mathbf{e}^{\mathrm{FK}}_s
```

```latex
\mathcal{E}_{\mathrm{FK}}
=
\sum_s\sum_{k=1}^{6}
f_{\mathrm{FK}}^2\rho_{\mathrm{Huber}}
\left(w_{s,k}^2/f_{\mathrm{FK}}^2\right)
```

### Unified와 Sequential

```latex
\text{Unified:}
\qquad
\min_{\{\mathbf{C}_i\},\mathbf{X},\mathbf{B},\{\mathbf{O}_s\}}
\left(
\mathcal{E}^{\mathrm{px}}_{\mathrm{fix}}
+
\mathcal{E}^{\mathrm{px}}_{\mathrm{grip}}
+\mathcal{E}_{\mathrm{FK}}
\right)
```

```latex
\text{Sequential:}
\qquad
\min_{\mathbf{X},\mathbf{B},\{\mathbf{O}_s\}}
(\mathcal{E}^{\mathrm{px}}_{\mathrm{grip}}+\mathcal{E}_{\mathrm{FK}}),
\quad\text{freeze},\quad
\min_{\{\mathbf{C}_i\}}
\mathcal{E}^{\mathrm{px}}_{\mathrm{fix}}.
```

---

<a id="toc-section-23"></a>

## 22. 예상 질문과 짧은 답변

### 질문: 목적함수 안의 항들은 모두 행렬인가?

pose 변수는 $4\times4$ 변환행렬이지만 visual residual은 각 코너의 $u,v$ 픽셀 차이다. corrected-FK factor만 상대 pose를 회전 3개와 이동 3개의 6차원 벡터로 바꾼 뒤 covariance whitening한다.

### 질문: no-FK(vision)는 로봇 FK를 전혀 쓰지 않는가?

큐브 FK prior $\mathbf{F}_s$는 쓰지 않는다. 하지만 움직이는 그리퍼 카메라를 베이스에 연결하기 위한 촬영 순간 그리퍼 자세 $\mathbf{G}_e$는 사용한다.

### 질문: corrected-FK는 soft anchor인가?

그렇다. 정확히는 train-only 정렬 FK를 중심으로 한 covariance-whitened robust pose factor다. cube pose는 자유변수로 남으며 hard gate나 pose 교체는 없다.

### 질문: Sequential과 legacy Independent는 같은 방법인가?

아니다. 현재 real-data `seq`는 EIH stage를 먼저 풀고 그 결과를 동결한 뒤 fixed camera만 푼다. 두 좌표계를 독립적으로 푼 뒤 rigid alignment하는 legacy simulation 절차와 구분해야 한다.

### 질문: 실데이터 FK 오차가 작으면 실제로도 정확한가?

반드시 그렇지는 않다. 외부 정답 장비가 없다면 FK와의 일치도일 뿐이며, 절대 물리 정확도는 아니다.
