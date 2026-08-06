# 멀티카메라 캘리브레이션 완전 설명서

> 대상: 캘리브레이션을 처음 배우는 사람  
> 목표: 이 문서만 읽고 `No-FK`, `Fixed-FK`, `Corrected-FK`, `Unified`, `Independent/Separated`, `Board-only`, `Cube-only`, `Both`를 수식으로 설명하기  
> 표기 원칙: 모든 수학 변수와 기호는 Markdown LaTeX인 `$...$` 또는 `$$...$$` 안에 작성했다. 따라서 수식 부분을 그대로 복사할 수 있다.

---

## 0. 발표를 한 문장으로 시작한다면

멀티카메라 캘리브레이션은 다음 문제다.

> 서로 다른 위치에서 같은 물체를 본 여러 카메라가, 그 물체의 위치와 방향을 로봇 베이스라는 하나의 공통 좌표계에서 똑같이 말하도록 카메라 사이의 관계를 찾는 과정이다.

이 프로젝트에서는 여기에 두 가지 선택 축이 더해진다.

1. 로봇이 계산한 큐브 자세인 FK를 사용하지 않을지, 그대로 믿을지, 보정해서 사용할지 결정한다.
2. 고정 카메라와 그리퍼 카메라를 하나의 문제로 동시에 풀지, 서로 분리해서 풀지 결정한다.

---

## 1. 왜 카메라마다 큐브의 좌표가 다를까?

카메라마다 자기 자신을 원점으로 사용하기 때문이다.

- 카메라 $0$은 큐브가 자기 앞에 있다고 말한다.
- 카메라 $1$은 같은 큐브가 자기 오른쪽에 있다고 말한다.
- 그리퍼 카메라는 같은 큐브가 자기 아래에 있다고 말할 수 있다.

세 답은 서로 모순이 아니다. 기준 좌표계가 다를 뿐이다.

따라서 모든 관측을 공통 기준인 로봇 베이스 좌표계 $B$로 옮겨야 한다.

---

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

이 문서에서는 다음과 같은 변환 표기를 사용한다.

$$
{}^{A}\mathbf{T}_{B}
$$

읽는 방법은 다음과 같다.

> 좌표계 $B$에서 표현된 값을 좌표계 $A$에서 표현하도록 바꾸는 변환이다.

위 첨자 $A$는 도착 좌표계이고, 아래 첨자 $B$는 출발 좌표계다.

---

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

## 4. 카메라가 실제로 측정하는 것은 무엇인가?

카메라는 이미지에서 보드나 큐브의 코너를 검출하고 PnP를 풀어서 다음 자세를 얻는다.

여기서 PnP는 Perspective-n-Point의 줄임말이다. 한 장의 사진만 가지고 물체가 카메라 기준으로 어디에 어떤 방향으로 놓여 있는지를 알아내는 방법이다.

PnP를 풀려면 두 가지가 필요하다.

1. **물체 위 점들의 3차원 좌표.** 물체 자신의 좌표계에서 잰 값이다. 예를 들어 한 변이 $50\,\mathrm{mm}$인 체커보드 격자라면 코너들이 $(0,0,0)$, $(50,0,0)$, $(0,50,0)$처럼 놓여 있다는 사실을 설계 도면에서 이미 알고 있다.
2. **같은 점들이 사진에서 찍힌 2차원 픽셀 좌표.** 코너 검출 알고리즘이 찾아준다.

같은 점이 3차원에서는 어디이고 사진에서는 몇 번째 픽셀인지, 그 짝을 여러 개 모으면 물체의 자세를 역으로 풀 수 있다. 이름의 $n$은 그 짝의 개수를 뜻하며, 원리상 최소 $3$개, 안정적으로 풀려면 보통 그보다 많이 쓴다.

단, 카메라 내부파라미터를 미리 알고 있어야 한다. 내부파라미터란 초점거리와 이미지 중심처럼 3차원 점이 이미지의 어느 픽셀에 맺히는지를 정하는 카메라 자체의 값이다. 이 값은 캘리브레이션 이전에 따로 구해 둔다.

정리하면 PnP의 출력이 곧 아래에서 정의하는 관측값 $\mathbf{Z}$이다. 즉 카메라 좌표계에서 본 물체의 자세다.

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

우리 multi-cam calibration 에서 비교하는 세 FK 방식의 가장 큰 차이는 $\mathbf{O}_s$를 자유롭게 찾는지, FK 값에 고정하는지, 보정된 anchor에 고정하는지다.

---

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

## 8. 모든 방법이 공유하는 기본 목적함수

세트 $s$의 공통 큐브 자세를 $\mathbf{Q}_s$라고 하자. 그러면 vision 관측의 전체 오차는 다음과 같다.

$$
\begin{aligned}
\mathcal{E}_{\mathrm{vis}}
\left(\{\mathbf{C}_i\},\mathbf{X},\{\mathbf{Q}_s\}\right)
={}&
\sum_{i,s}
\left\|
\mathbf{r}
\left(
\mathbf{C}_i\mathbf{Z}_{i,s},
\mathbf{Q}_s
\right)
\right\|_2^2
\\
&+
\sum_e
\left\|
\mathbf{r}
\left(
\mathbf{G}_e\mathbf{X}\mathbf{Z}_{g,e},
\mathbf{Q}_{s(e)}
\right)
\right\|_2^2.
\end{aligned}
$$

### 8.1 식에 나오는 기호

$\mathcal{E}_{\mathrm{vis}}$의 아래첨자 vis는 vision, 즉 카메라로 본 정보에서 나온 오차라는 뜻이다. 로봇 관절값에서 나온 FK 정보와 구분하려고 붙인 이름이다.

| 기호 | 뜻 | 값을 아는가 |
|---|---|---|
| $\mathbf{C}_i$ | 고정 카메라 $i$의 자세. 베이스 기준 | 모른다. 찾아야 한다 |
| $\mathbf{X}$ | 그리퍼와 그리퍼 카메라 사이의 hand-eye 변환 | 모른다. 찾아야 한다 |
| $\mathbf{Q}_s$ | 세트 $s$의 공통 큐브 자세. 베이스 기준 | 방식마다 다르다. 9장부터 설명 |
| $\mathbf{Z}_{i,s}$ | 고정 카메라 $i$가 세트 $s$에서 본 큐브 자세 | 안다. PnP 측정값 |
| $\mathbf{Z}_{g,e}$ | 그리퍼 카메라가 이벤트 $e$에서 본 큐브 자세 | 안다. PnP 측정값 |
| $\mathbf{G}_e$ | 이벤트 $e$의 그리퍼 자세. 베이스 기준 | 안다. 로봇 FK 값 |
| $\mathbf{r}(\cdot,\cdot)$ | 두 자세의 차이를 $6$개 숫자로 만드는 잔차 함수 | 7장에서 정의 |
| $\|\cdot\|_2^2$ | 그 $6$개 숫자를 제곱해서 더한 값 | 오차 하나의 크기 |
| $\sum_{i,s}$ | 모든 고정 카메라와 모든 세트에 대해 더한다 | |
| $\sum_e$ | 모든 그리퍼 카메라 촬영 이벤트에 대해 더한다 | |
| $s(e)$ | 이벤트 $e$가 속한 세트 번호 | 안다. 촬영 기록 |

읽는 순서는 다음과 같다. 안쪽부터 밖으로 나간다.

1. $\mathbf{C}_i\mathbf{Z}_{i,s}$로 고정 카메라들이 예측한 큐브 자세를 만든다. 
2. 그 예측을 공통 큐브 자세 $\mathbf{Q}_s$와 비교하여 잔차 $6$개 숫자를 얻는다.
3. 제곱해서 더해 오차 하나의 숫자로 만든다.
4. 모든 카메라와 모든 세트에 대해 이 값을 전부 합친다.
5. 그리퍼 카메라 쪽도 $\mathbf{G}_e\mathbf{X}\mathbf{Z}_{g,e}$로 같은 과정을 거쳐 합친다.

### 8.2 이 식이 뜻하는 것

첫 번째 합은 고정 카메라 예측을 공통 큐브 자세에 맞춘다. 두 번째 합은 그리퍼 카메라 예측을 같은 공통 큐브 자세에 맞춘다.

$\mathcal{E}_{\mathrm{vis}}$를 최소로 만드는 $\mathbf{C}_i$와 $\mathbf{X}$를 찾는 것이 캘리브레이션이다. 측정값 $\mathbf{Z}$와 $\mathbf{G}$는 이미 정해져 있으므로 움직일 수 없고, 오직 미지수만 움직여서 모든 예측이 한 점에 모이게 만든다.

우리 캘리브레이션에서 비교하는 세가지 FK 방식은 이 관측식 자체가 다르지 않다. $\mathbf{Q}_s$를 무엇으로 정의하느냐가 다르다.

---

## 9. 방법 1: No-FK

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

따라서 $\mathbf{Q}_s=\mathbf{O}_s$는 두 값이 같다는 뜻이 아니라, 기준 자리를 고정값이 아니라 자유변수로 채운다는 선언이다. `Fixed-FK`는 같은 자리를 FK 값으로 채운다.

셋 중 어느 것도 큐브의 진짜 자세는 아니다. 진짜 자세는 10.3절에서 $\mathbf{O}^{\mathrm{true}}_s$로 따로 표기한다.

### 9.3 장점과 주의점

- raw FK의 계통오차가 캘리브레이션에 직접 들어오지 않는다.
- 대신 세트마다 $6$자유도인 $\mathbf{O}_s$가 추가되어 미지수가 많아진다.
- 카메라 연결이나 로봇 움직임이 충분하지 않으면 전체 좌표계의 gauge가 흔들릴 수 있다.
- `No-FK`도 $\mathbf{G}_e$는 사용한다. 큐브 FK prior $\mathbf{F}_s$만 사용하지 않는다.

---

## 10. 방법 2: Fixed-FK

### 10.1 쉬운 설명

로봇 FK가 알려준 큐브 자세를 정답으로 간주하고 움직이지 못하게 고정한다.

raw FK 큐브 prior를 다음과 같이 정의한다.

$$
\mathbf{F}_s
\equiv
{}^{B}\mathbf{T}^{\mathrm{FK}}_{O,s}
$$

큐브 자세는 다음 제약을 만족해야 한다.

$$
\mathbf{O}_s=\mathbf{F}_s
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
\{\mathbf{F}_s\}
\right)
$$

이를 완전히 펼치면 다음과 같다.

$$
\begin{aligned}
\min_{\{\mathbf{C}_i\},\mathbf{X}}
{}&
\sum_{i,s}
\left\|
\mathbf{r}(\mathbf{C}_i\mathbf{Z}_{i,s},\mathbf{F}_s)
\right\|_2^2
\\
&+
\sum_e
\left\|
\mathbf{r}
(\mathbf{G}_e\mathbf{X}\mathbf{Z}_{g,e},\mathbf{F}_{s(e)})
\right\|_2^2.
\end{aligned}
$$

### 10.3 왜 FK 오차가 카메라로 전파되는가?

실제 큐브 자세를 $\mathbf{O}^{\mathrm{true}}_s$라고 하자. raw FK에 계통오차 $\mathbf{D}$가 있으면 다음처럼 쓸 수 있다.

$$
\mathbf{F}_s
=
\mathbf{O}^{\mathrm{true}}_s\mathbf{D}
$$

그런데 최적화는 $\mathbf{F}_s$를 움직일 수 없다. 따라서 남은 오차를 줄이기 위해 $\mathbf{C}_i$나 $\mathbf{X}$가 잘못 움직일 수 있다.

Fixed-FK는 FK가 정확할 때 미지수가 적고 안정적이지만, FK에 공통적인 오정렬이 있으면 그 오차를 캘리브레이션 결과에 흡수시킬 위험이 있다.

### 10.4 카메라에도 계통오차가 있으면

카메라도 내부파라미터나 왜곡 보정이 조금 틀리면 항상 같은 방향으로 치우친 $\mathbf{Z}$를 내놓는다. 예를 들어 큐브를 늘 $3\,\mathrm{mm}$ 멀리 있다고 보고하는 식이다.

이 경우 Fixed-FK에서는 두 계통오차를 구분할 수 없다. $\mathbf{F}_s$가 고정되어 있으므로 어긋남은 전부 $\mathbf{C}_i$와 $\mathbf{X}$로 밀려 들어가는데, 그 어긋남 안에는 FK가 틀린 몫과 카메라가 틀린 몫이 섞여 있다. 최적화 입장에서 두 몫은 똑같이 생겼고, 어느 쪽 책임인지 판단할 정보가 목적함수 안에 없다. 결국 둘 다 캘리브레이션 결과에 흡수된다.

더 곤란한 점은 이 상황에서 잔차가 오히려 작게 나온다는 것이다. $\mathbf{C}_i$가 카메라 편향만큼 반대로 움직이면 관측은 깔끔하게 설명되기 때문이다. 수렴은 잘 된 것처럼 보이지만 $\mathbf{C}_i$는 물리적 진짜 위치에서 벗어나 있다.

---

## 11. 방법 3: Corrected-FK

### 11.1 가장 쉬운 설명

raw FK를 무조건 믿지 않는다. 먼저 vision으로 큐브 자세를 계산하고, raw FK가 vision과 어떤 공통 차이를 갖는지 학습한다. 그 차이로 FK를 보정한 뒤, vision과 충분히 가까운 세트에서만 그 보정된 FK를 사용한다.

### 11.2 단계 1: vision-only 초기 해

먼저 No-FK 문제를 풀어 초기 카메라와 hand-eye를 구한다.

$$
\{\mathbf{C}^{(0)}_i\},\mathbf{X}^{(0)},\{\mathbf{O}^{(0)}_s\}
=
\operatorname{SolveNoFK}(\text{vision observations})
$$

### 11.3 단계 2: 세트별 vision 합의 자세

각 세트에서 고정 카메라와 그리퍼 카메라가 예측한 큐브 자세를 모은다.

$$
\mathcal{P}_s
=
\left\{
\mathbf{C}^{(0)}_i\mathbf{Z}_{i,s}
\right\}_{i}
\cup
\left\{
\mathbf{G}_e\mathbf{X}^{(0)}\mathbf{Z}_{g,e}
\right\}_{e:s(e)=s}
$$

기호를 읽는 법은 다음과 같다.

- $\mathcal{P}_s$는 세트 $s$의 큐브 자세 예측을 출처 구분 없이 전부 담은 주머니다. 카메라가 $3$대이고 그 세트에서 그리퍼 촬영이 $4$번 있었다면 자세 $7$개가 들어 있다.
- $\cup$는 합집합이다. 고정 카메라 예측과 그리퍼 카메라 예측을 한 통에 담아 동등하게 취급하겠다는 뜻이다.
- 위첨자 $(0)$은 11.2절에서 얻은 초기 추정값이라는 표시다. 최종 결과가 아니라 중간 단계 값이므로 햇 기호와 구분한다.

이 예측들을 강건 평균하여 vision 합의 자세 $\mathbf{V}_s$를 만든다.

$$
\mathbf{V}_s
=
\operatorname{RobustAverage}
\left(\mathcal{P}_s\right)
$$

이 단계에서는 예측들을 모두 동등하게 다루고, MAD 기준으로 이상치만 제거한다.

- **이상치**: 나머지와 동떨어진 값. 코너 오검출이나 큐브 면 착각으로 생긴다. $7$개 중 $6$개가 $2\,\mathrm{mm}$ 안에 모여 있는데 하나가 $100\,\mathrm{mm}$ 튀면 단순 평균은 약 $14\,\mathrm{mm}$ 밀려난다.
- **MAD**: Median Absolute Deviation, 중앙값 절대편차. 값들이 흩어진 정도를 중앙값으로 재기 때문에 표준편차와 달리 이상치에 오염되지 않는다. 중앙값에서 MAD의 몇 배 이상 떨어진 값을 버린다.
- **강건 평균**: MAD로 이상치를 거른 뒤 남은 값을 평균한다.


### 11.4 단계 3: raw FK와 vision 사이의 공통 차이

각 세트의 차이는 다음과 같다.

$$
\boldsymbol{\Delta}_s
=
\mathbf{F}_s^{-1}\mathbf{V}_s
$$

즉, raw FK에서 vision 결과로 가려면 얼마나 더 움직여야 하는지를 뜻한다.

세트별 차이를 강건 가중 평균하여 공통 보정량을 구한다.

$$
\overline{\boldsymbol{\Delta}}
=
\operatorname{RobustWeightedAverage}_s
\left(
\boldsymbol{\Delta}_s;,w_s
\right)
$$

여기서 $w_s$는 세트 $s$를 얼마나 믿을지 나타내는 가중치이며, 그 세트를 뒷받침한 관측의 개수를 쓴다. 고정 카메라 $3$대가 보고 그리퍼 촬영이 $4$번 있었던 세트라면 $w_s=7$이다. 카메라 종류는 구분하지 않고 개수만 센다.

관측이 $2$개뿐인 세트의 $\boldsymbol{\Delta}_s$는 우연에 크게 흔들리므로, 관측이 많은 세트의 값을 더 크게 반영하여 공통 보정량이 빈약한 세트에 끌려가지 않게 한다.

### 11.5 단계 4: FK에 공통 보정량 적용

현재 구현은 보정량을 오른쪽에 곱한다.

$$
\mathbf{F}^{\mathrm{corr}}_s
=
\mathbf{F}_s\overline{\boldsymbol{\Delta}}
$$

### 11.6 단계 5: gate로 보정 FK를 믿어도 되는지 검사

앞 단계에서 구한 $\overline{\boldsymbol{\Delta}}$는 모든 세트에 공통으로 적용되는 하나의 보정량이다. 따라서 세트마다 사정이 다르면 잘 맞지 않을 수 있다. 큐브가 그리퍼 안에서 미끄러진 세트, 그 세트에서만 관절 오차가 유난히 컸던 경우가 그렇다. 반대로 그 세트의 관측이 부족해서 vision 쪽이 틀렸을 수도 있다.

어느 쪽이 틀렸는지는 알 수 없지만, 둘이 크게 어긋났다는 사실은 확인할 수 있다. 그래서 세트마다 보정된 FK와 vision 합의를 나란히 놓고 차이를 재고, 그 차이가 기준을 넘으면 해당 세트에서는 FK를 사용하지 않는다. 이 검사를 gate라고 부른다.

이동 차이는 다음과 같다.

$$
d_t(s)
=
1000
\left\|
\mathbf{t}(\mathbf{V}_s)
-
\mathbf{t}(\mathbf{F}^{\mathrm{corr}}_s)
\right\|_2
$$

회전 차이는 다음과 같다.

$$
d_R(s)
=
\frac{180}{\pi}
\cos^{-1}
\left(
\frac{
\operatorname{tr}
\left(
\mathbf{R}(\mathbf{V}_s)^{\mathsf T}
\mathbf{R}(\mathbf{F}^{\mathrm{corr}}_s)
\right)-1
}{2}
\right)
$$

세트 $s$는 두 조건을 모두 만족할 때만 통과한다.

$$
d_t(s)\leq \tau_t,
\qquad
d_R(s)\leq \tau_R
$$

기준선 $\tau_t$와 $\tau_R$은 상수가 아니라, $d_t(s)$와 $d_R(s)$ 값들이 실제로 어떻게 분포하는지를 보고 정한다.

$$
\tau
=
\max\left(
\operatorname{median}_s(d)
+
k \cdot 1.4826 \cdot \operatorname{MAD}_s(d),
\;
\tau^{\min}
\right),
\qquad
k=2.5
$$

상수 $1.4826$은 MAD를 표준편차와 같은 척도로 환산하는 값이다. 정규분포에서 MAD에 이 값을 곱하면 표준편차가 된다. 이 환산 덕분에 $k$를 표준편차의 배수로 읽을 수 있다. $k=2.5$는 중앙값에서 표준편차 $2.5$배까지 정상으로 인정한다는 뜻이며, $\overline{\boldsymbol{\Delta}}$를 구하는 강건 평균이 이미 쓰고 있는 값과 같다.

기준선을 이렇게 잡는 이유는 $d_t(s)$가 무엇을 재는 값인지에 있다. 11.5절에서 본 것처럼 어떤 세트의 $\boldsymbol{\Delta}_s$가 공통 보정량 $\overline{\boldsymbol{\Delta}}$와 정확히 같다면 $\mathbf{F}^{\mathrm{corr}}_s=\mathbf{V}_s$가 되어 $d_t(s)=0$이다. 즉 $d_t(s)$는 FK가 절대적으로 얼마나 틀렸는지가 아니라, 그 세트가 다른 세트들의 공통 경향에서 얼마나 벗어났는지를 재는 값이다. 그렇다면 얼마부터 유별난 값인지는 나머지 세트들이 얼마나 모여 있느냐에 따라 달라진다. 세트들이 모두 $2\,\mathrm{mm}$ 안에 모여 있다면 $10\,\mathrm{mm}$짜리 세트는 명백히 이상하지만, 원래부터 $30\,\mathrm{mm}$씩 흩어져 있었다면 같은 $10\,\mathrm{mm}$도 평범하다. 고정 상수는 이 두 경우를 구분하지 못한다. 

여기에는 보호 장치가 필요하다.

**하한 $\tau^{\min}$이 필요한 이유.** 기준선을 중앙값 근처로만 잡으면 문제가 생긴다. 중앙값이란 절반은 그보다 작고 절반은 그보다 크다는 뜻이다. 따라서 세트들이 모두 잘 맞아서 흩어짐이 거의 없으면, 기준선이 중앙값에 딱 붙어버리고 **멀쩡한 세트의 절반이 탈락한다.** 걸러낼 이상치가 없는데도 절반이 FK를 못 쓰게 되는 것이다.

**하한을 무엇으로 정하는가.** 여기에 다시 상수를 쓰면 처음 문제로 돌아간다. 대신 vision 합의 자신의 흔들림을 쓴다.

각 세트에서 카메라들이 예측한 큐브 자세는 서로 조금씩 어긋나 있다. 그 어긋난 정도가 곧 $\mathbf{V}_s$의 불확실성이다. 11.3절에서 강건 평균을 낼 때 이미 계산되는 값이기도 하다.

$$
\tau^{\min}_t
=
\operatorname{median}_s
\left(
\operatorname{scatter}_t(\mathcal{P}_s)
\right),
\qquad
\tau^{\min}_R
=
\operatorname{median}_s
\left(
\operatorname{scatter}_R(\mathcal{P}_s)
\right)
$$

이렇게 두는 이유는 분해능 때문이다. 어떤 세트에서 카메라들끼리 $1.5\,\mathrm{mm}$씩 어긋나 있다면, $\mathbf{V}_s$ 자체가 그만큼 불확실하다. 이때 FK가 $\mathbf{V}_s$에서 $1\,\mathrm{mm}$ 벗어나 있다고 해도 그것이 FK의 잘못인지 vision의 잘못인지 구별할 방법이 없다. 자의 눈금보다 작은 차이를 재려는 것과 같다. 구별할 수 없는 차이로 세트를 탈락시키는 것은 근거가 없으므로, 그 아래는 문제 삼지 않는다.

정리하면 하한의 역할은 무언가를 더 걸러내는 것이 아니라, **구별할 수 없는 차이로 걸러내지 않도록 보장하는 것**이다.

### 11.7 단계 6: gate 결과에 따라 anchor 확정

세트 $s$의 anchor는 gate 통과 여부로 결정된다.

$$
\mathbf{A}_s
=
\begin{cases}
\mathbf{F}^{\mathrm{corr}}_s,
&
d_t(s)\leq\tau_t
\ \land\ 
d_R(s)\leq\tau_R,
\\[4pt]
\mathbf{V}_s,
&
\text{otherwise}.
\end{cases}
$$

통과한 세트는 보정된 FK를 **그대로** anchor로 삼는다. 몇 퍼센트만 반영하는 식의 부분 신뢰는 두지 않는다. gate가 이미 그 세트를 믿을지 말지 판정했으므로, 통과한 뒤에 다시 비중을 깎을 근거가 없기 때문이다.

탈락한 세트는 FK를 쓰지 않고 vision 합의 $\mathbf{V}_s$를 anchor로 삼는다.

정리하면 판단은 세트마다 전부 아니면 전무다.

| gate | anchor | FK 사용 |
|---|---|---|
| 통과 | $\mathbf{F}^{\mathrm{corr}}_s$ | 그대로 사용 |
| 탈락 | $\mathbf{V}_s$ | 사용하지 않음 |

### 11.8 단계 7: anchor를 고정하고 최종 refinement

`Corrected-FK`는 만들어진 $\mathbf{A}_s$를 최종 단계에서 고정 anchor로 사용한다.

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
\{\mathbf{A}_s\}
\right)
$$

즉, Corrected-FK는 단순히 목적함수에 약한 벌점 하나를 더하는 것과 다르다. vision 초기 해, FK 공통 정렬, gate 판정, anchor 확정, 고정 anchor refinement의 순서로 동작한다.

### 11.9 아직 검증이 남은 부분

이 장의 절차는 확정된 것이 아니라 검증과 보완이 진행 중이다. 특히 다음 항목들은 실험으로 근거를 마련해야 한다.

- **$k=2.5$의 타당성.** 현재는 강건 평균이 쓰는 값과 통일했을 뿐이다. $k$를 바꿔가며 결과가 얼마나 달라지는지 확인해야 한다.
- **gate가 실제로 도움이 되는가.** 세트를 탈락시키는 것이 결과를 개선하는지, 아니면 정보를 버리는 손해가 더 큰지는 gate를 끈 조건과 비교해야 알 수 있다.
- **하한을 vision 산포로 두는 방식의 적절성.** 산포의 중앙값을 쓰는 것이 맞는지, 세트별 산포를 각각 쓰는 편이 나은지는 아직 비교하지 않았다.
- **시뮬레이션과 실제 파이프라인의 일치.** 적응형 기준선은 현재 시뮬레이션에만 반영되어 있으며, 실제 캘리브레이션 코드에는 아직 옮기지 않았다.

따라서 이 장의 수치와 정책은 이후 실험 결과에 따라 바뀔 수 있다.

---

## 12. 세 FK 방식 비교

| 방법 | 공통 큐브 자세 $\mathbf{Q}_s$ | 큐브 자세의 상태 | 핵심 의미 |
|---|---:|---|---|
| `No-FK` | $\mathbf{Q}_s=\mathbf{O}_s$ | 자유변수 | FK를 사용하지 않고 vision 합의로 직접 찾음 |
| `Fixed-FK` | $\mathbf{Q}_s=\mathbf{F}_s$ | raw FK에 고정 | FK를 정답으로 간주 |
| `Corrected-FK` | $\mathbf{Q}_s=\mathbf{A}_s$ | 보정된 anchor에 고정 | vision으로 FK를 검증하고 통과한 세트만 사용 |

한 문장으로 요약하면 다음과 같다.

- `No-FK`: 큐브 자세도 직접 찾는다.
- `Fixed-FK`: 큐브 자세를 raw FK에 못 박는다.
- `Corrected-FK`: raw FK를 vision으로 보정하고 gate를 통과한 세트에서만 사용한다.

---

## 13. Unified 방식

### 13.1 쉬운 설명

고정 카메라와 그리퍼 카메라가 같은 $\mathbf{Q}_s$를 공유하도록 모든 미지수를 하나의 최적화에서 동시에 푼다.

### 13.2 수식

$$
\boldsymbol{\theta}_{\mathrm{uni}}
=
\left(
\{\mathbf{C}_i\},
\mathbf{X},
\{\mathbf{Q}_s\}
\right)
$$

$$
\widehat{\boldsymbol{\theta}}_{\mathrm{uni}}
=
\underset{\boldsymbol{\theta}_{\mathrm{uni}}}
{\operatorname{argmin}}
\left[
\mathcal{E}_{\mathrm{fix}}
+
\mathcal{E}_{\mathrm{grip}}
+
\mathcal{E}_{\mathrm{anchor}}
\right]
$$

각 관측항은 다음과 같다.

$$
\mathcal{E}_{\mathrm{fix}}
=
\sum_{i,s}
\left\|
\mathbf{r}
(\mathbf{C}_i\mathbf{Z}_{i,s},\mathbf{Q}_s)
\right\|_2^2
$$

$$
\mathcal{E}_{\mathrm{grip}}
=
\sum_e
\left\|
\mathbf{r}
(\mathbf{G}_e\mathbf{X}\mathbf{Z}_{g,e},\mathbf{Q}_{s(e)})
\right\|_2^2
$$

$\mathcal{E}_{\mathrm{anchor}}$는 FK 방식에 따라 없거나, hard constraint로 대체되거나, 별도의 soft anchor로 사용될 수 있다. `Corrected-FK`에서는 $\mathbf{A}_s$를 고정한 refinement로 구현된다.

Unified의 핵심은 한쪽 관측이 $\mathbf{Q}_s$를 움직이면, 같은 $\mathbf{Q}_s$를 공유하는 다른 쪽의 $\mathbf{C}_i$와 $\mathbf{X}$도 영향을 받는다는 것이다. 즉 두 서브시스템이 정보를 교환한다.

---

## 14. Independent와 Separated 방식

### 14.1 먼저 용어를 정확히 구분한다

이 프로젝트의 시뮬레이션 설정에서 `Independent`, `Separated`, `따로 풀기`, `-unified`는 같은 축을 가리킨다.

즉, `Independent`와 `Separated`라는 서로 다른 두 솔버가 따로 있는 것이 아니다.

- `Independent`: 결과표와 함수 이름에서 사용하는 이름
- `Separated`: 고정 카메라 서브시스템과 그리퍼 카메라 서브시스템을 분리해서 푼다는 설명

### 14.2 단계 1: 고정 카메라 서브시스템만 푼다

고정 카메라용 공통 큐브 자세를 $\mathbf{Q}^{\mathrm{fix}}_s$라고 두면 다음 문제를 푼다.

$$
\left\{
\{\widehat{\mathbf{C}}_i\},
\{\widehat{\mathbf{Q}}^{\mathrm{fix}}_s\}
\right\}
=
\underset{
\{\mathbf{C}_i\},
\{\mathbf{Q}^{\mathrm{fix}}_s\}
}{\operatorname{argmin}}
\sum_{i,s}
\left\|
\mathbf{r}
\left(
\mathbf{C}_i\mathbf{Z}_{i,s},
\mathbf{Q}^{\mathrm{fix}}_s
\right)
\right\|_2^2
$$

Fixed-FK 또는 보정 anchor가 주어진 독립 방식에서는 카메라별로 다음 후보를 직접 만들 수 있다.

$$
\mathbf{C}_{i,s}^{\mathrm{cand}}
=
\mathbf{Q}_s
\mathbf{Z}_{i,s}^{-1}
$$

그리고 여러 세트의 후보를 강건 평균한다.

$$
\widehat{\mathbf{C}}_i
=
\operatorname{RobustAverage}_s
\left(
\mathbf{Q}_s\mathbf{Z}_{i,s}^{-1}
\right)
$$

여기서 Fixed-FK이면 $\mathbf{Q}_s=\mathbf{F}_s$이고, Corrected-FK이면 $\mathbf{Q}_s=\mathbf{A}_s$다.

### 14.3 단계 2: 그리퍼 카메라 서브시스템만 푼다

그리퍼 쪽 전용 큐브 자세를 $\mathbf{Q}^{\mathrm{grip}}_s$라고 두면 다음 문제를 푼다.

$$
\left\{
\widehat{\mathbf{X}},
\{\widehat{\mathbf{Q}}^{\mathrm{grip}}_s\}
\right\}
=
\underset{
\mathbf{X},
\{\mathbf{Q}^{\mathrm{grip}}_s\}
}{\operatorname{argmin}}
\sum_e
\left\|
\mathbf{r}
\left(
\mathbf{G}_e\mathbf{X}\mathbf{Z}_{g,e},
\mathbf{Q}^{\mathrm{grip}}_{s(e)}
\right)
\right\|_2^2
$$

FK 또는 corrected anchor가 주어진 경우에는 각 관측에서 hand-eye 후보를 직접 만들 수 있다.

$$
\mathbf{X}_e^{\mathrm{cand}}
=
\mathbf{G}_e^{-1}
\mathbf{Q}_{s(e)}
\mathbf{Z}_{g,e}^{-1}
$$

따라서 다음처럼 평균할 수 있다.

$$
\widehat{\mathbf{X}}
=
\operatorname{RobustAverage}_e
\left(
\mathbf{G}_e^{-1}
\mathbf{Q}_{s(e)}
\mathbf{Z}_{g,e}^{-1}
\right)
$$

### 14.4 단계 3: No-FK 독립 방식의 두 결과를 정렬한다

No-FK에서는 두 서브시스템이 서로 다른 큐브 합의를 만들 수 있다. 현재 시뮬레이션은 두 쪽이 예측한 큐브 중심을 이용하여 강체 정렬을 구한다.

고정 카메라 쪽 세트 중심을 다음과 같이 둔다.

$$
\mathbf{p}^{\mathrm{fix}}_s
=
\mathbf{t}
\left(
\operatorname{Average}_i
(\widehat{\mathbf{C}}_i\mathbf{Z}_{i,s})
\right)
$$

그리퍼 카메라 쪽 세트 중심은 다음과 같다.

$$
\mathbf{p}^{\mathrm{grip}}_s
=
\mathbf{t}
\left(
\operatorname{Average}_{e:s(e)=s}
(\mathbf{G}_e\widehat{\mathbf{X}}\mathbf{Z}_{g,e})
\right)
$$

그리퍼 예측점을 고정 카메라 결과에 맞추는 강체 정렬 $\mathbf{H}=(\mathbf{R}_H,\mathbf{t}_H)$를 구한다.

$$
\left(
\widehat{\mathbf{R}}_H,
\widehat{\mathbf{t}}_H
\right)
=
\underset{
\mathbf{R}_H\in\mathrm{SO}(3),
\mathbf{t}_H\in\mathbb{R}^{3}
}{\operatorname{argmin}}
\sum_s
\left\|
\mathbf{p}^{\mathrm{fix}}_s
-
\left(
\mathbf{R}_H\mathbf{p}^{\mathrm{grip}}_s
+
\mathbf{t}_H
\right)
\right\|_2^2
$$

정렬 후 그리퍼 예측은 다음처럼 사용한다.

$$
\widehat{\mathbf{O}}^{\mathrm{grip\rightarrow fix}}_e
=
\mathbf{H}
\left(
\mathbf{G}_e\widehat{\mathbf{X}}\mathbf{Z}_{g,e}
\right)
$$

현재 시뮬레이션 코드의 `_rigid_align`은 공통 세트가 최소 $3$개일 때 큐브 중심점들에 Kabsch 강체 정렬을 적용한다.

### 14.5 Unified와 Independent의 핵심 차이

Unified는 처음부터 하나의 $\mathbf{Q}_s$를 공유한다.

$$
\mathbf{Q}^{\mathrm{fix}}_s
=
\mathbf{Q}^{\mathrm{grip}}_s
=
\mathbf{Q}_s
$$

Independent는 각각 푼 다음 마지막에 정렬한다.

$$
\left(
\{\mathbf{C}_i\},\{\mathbf{Q}^{\mathrm{fix}}_s\}
\right)
\quad\text{and}\quad
\left(
\mathbf{X},\{\mathbf{Q}^{\mathrm{grip}}_s\}
\right)
\quad\text{are solved separately}
$$

따라서 Unified에서는 양쪽 관측이 최적화 중에 정보를 교환하지만, Independent에서는 마지막 정렬 전까지 정보교환이 없다.

---

## 15. Board-only, Cube-only, Both

FK 방식과 별개로 어떤 타깃 관측을 사용할지도 선택한다.

### 15.1 Cube-only

큐브 관측 잔차만 사용한다.

$$
\mathcal{E}_{\mathrm{cube}}
=
\sum_{i,s}
\left\|
\mathbf{r}
(\mathbf{C}_i\mathbf{Z}^{\mathrm{cube}}_{i,s},\mathbf{Q}^{\mathrm{cube}}_s)
\right\|_2^2
+
\sum_e
\left\|
\mathbf{r}
(\mathbf{G}_e\mathbf{X}\mathbf{Z}^{\mathrm{cube}}_{g,e},
\mathbf{Q}^{\mathrm{cube}}_{s(e)})
\right\|_2^2
$$

### 15.2 Board-only

베이스 좌표계에서 고정된 보드 자세를 $\mathbf{Q}^{\mathrm{board}}$라고 두면 보드 관측만 사용한다.

$$
\mathcal{E}_{\mathrm{board}}
=
\sum_{i,s}
\left\|
\mathbf{r}
(\mathbf{C}_i\mathbf{Z}^{\mathrm{board}}_{i,s},\mathbf{Q}^{\mathrm{board}})
\right\|_2^2
+
\sum_e
\left\|
\mathbf{r}
(\mathbf{G}_e\mathbf{X}\mathbf{Z}^{\mathrm{board}}_{g,e},\mathbf{Q}^{\mathrm{board}})
\right\|_2^2
$$

보드는 로봇이 잡고 이동시키는 큐브 FK prior를 갖지 않는다. 따라서 이 프로젝트에서는 `Board-only + Fixed-FK`와 `Board-only + Corrected-FK`를 정의하지 않는다.

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

## 16. 왜 전체 조합이 $18$개가 아니라 $14$개인가?

형식적으로는 다음 세 축이 있다.

$$
2\;\text{solver modes}
\times
3\;\text{target modes}
\times
3\;\text{FK modes}
=18
$$

하지만 `Board-only`에는 큐브 FK prior가 없으므로 `Fixed-FK`와 `Corrected-FK`를 사용할 수 없다.

제외되는 조합 수는 다음과 같다.

$$
2\;\text{solver modes}
\times
1\;\text{board-only mode}
\times
2\;\text{invalid FK modes}
=4
$$

따라서 유효한 전체 조합은 다음과 같다.

$$
18-4=14
$$

---

## 17. Corrected-FK와 Ridge 출력 후보정은 다르다

이 둘은 자주 혼동되지만 서로 다른 단계다.

### 17.1 Corrected-FK

캘리브레이션 입력 쪽에서 raw FK 자세를 vision으로 정렬하고, gate와 blend를 거쳐 anchor $\mathbf{A}_s$를 만든다.

$$
\mathbf{F}_s
\rightarrow
\mathbf{F}^{\mathrm{corr}}_s
\rightarrow
\mathbf{A}_s
\rightarrow
\text{calibration refinement}
$$

### 17.2 선택적 Ridge 출력 후보정

C1의 선택적 출력 후보정은 캘리브레이션이 끝난 뒤 예측 위치의 잔차를 회귀로 학습한다.

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

---

## 19. 코드 흐름으로 다시 보기

아래 코드는 실제 구현을 이해하기 위한 축약 의사코드다.

```python
# 1. 이미지 관측에서 pose를 만든다.
Z_fixed = solve_pnp(fixed_camera_images)
Z_gripper = solve_pnp(gripper_camera_images)

# 2. FK 사용 방식에 따라 set target을 정한다.
if fk_mode == "none":
    target_mode = "free"

elif fk_mode == "fixed":
    set_anchors = raw_fk
    target_mode = "fixed"

elif fk_mode == "corr":
    visual_model = solve_unified(fk_mode="none")
    visual_by_set = build_visual_consensus(visual_model)
    delta = robust_average(inv(raw_fk[s]) @ visual_by_set[s])
    corrected_fk = {s: raw_fk[s] @ delta for s in sets}
    set_anchors = gate_select(
        visual_by_set,          # gate 탈락 시 사용
        corrected_fk,           # gate 통과 시 그대로 사용
        gate_k=2.5,             # median + k*1.4826*MAD
        gate_floor_mm=5.0,
        gate_floor_deg=1.0,
    )
    target_mode = "fixed"

# 3. 통합 또는 독립 구조로 캘리브레이션한다.
if solver_mode == "unified":
    model = solve_all_observations_together(set_anchors, target_mode)
else:
    fixed_model = solve_fixed_subsystem(set_anchors, target_mode)
    gripper_model = solve_gripper_subsystem(set_anchors, target_mode)
    model = align_and_combine(fixed_model, gripper_model)
```

---

## 20. 교수님께 설명할 때의 추천 순서

1. 카메라마다 자기 좌표계를 사용하므로 같은 큐브도 다른 좌표로 보인다고 설명한다.
2. 모든 것을 로봇 베이스 좌표계 $B$로 옮기는 것이 캘리브레이션이라고 설명한다.
3. 변환행렬 $\mathbf{T}$가 회전 $\mathbf{R}$과 이동 $\mathbf{t}$를 포함한다고 설명한다.
4. 고정 카메라 경로 $\mathbf{C}_i\mathbf{Z}_{i,s}$와 그리퍼 카메라 경로 $\mathbf{G}_e\mathbf{X}\mathbf{Z}_{g,e}$를 설명한다.
5. 두 경로가 같은 $\mathbf{Q}_s$에 도착하도록 오차를 최소화한다고 설명한다.
6. $\mathbf{Q}_s$를 어떻게 정하는지가 `No-FK`, `Fixed-FK`, `Corrected-FK`의 차이라고 설명한다.
7. 두 카메라 계열을 동시에 풀면 Unified이고, 따로 풀고 마지막에 정렬하면 Independent 또는 Separated라고 설명한다.
8. 실데이터의 FK 기반 평가는 절대 정답이 아니라 proxy라는 점을 마지막에 분명히 말한다.

---

## 21. 최종 한 장 요약용 수식

### 공통 vision 목적함수

```latex
\begin{aligned}
\mathcal{E}_{\mathrm{vis}}
={}&
\sum_{i,s}
\left\|
\mathbf{r}(\mathbf{C}_i\mathbf{Z}_{i,s},\mathbf{Q}_s)
\right\|_2^2
\\
&+
\sum_e
\left\|
\mathbf{r}(\mathbf{G}_e\mathbf{X}\mathbf{Z}_{g,e},\mathbf{Q}_{s(e)})
\right\|_2^2.
\end{aligned}
```

### 세 FK 방식

```latex
\mathbf{Q}_s=
\begin{cases}
\mathbf{O}_s, & \text{No-FK},\\
\mathbf{F}_s, & \text{Fixed-FK},\\
\mathbf{A}_s, & \text{Corrected-FK}.
\end{cases}
```

### Corrected-FK 핵심

```latex
\boldsymbol{\Delta}_s=\mathbf{F}_s^{-1}\mathbf{V}_s,
\qquad
\overline{\boldsymbol{\Delta}}
=
\operatorname{RobustWeightedAverage}_s(\boldsymbol{\Delta}_s),
\qquad
\mathbf{F}^{\mathrm{corr}}_s
=
\mathbf{F}_s\overline{\boldsymbol{\Delta}}.
```

```latex
\mathbf{A}_s
=
\begin{cases}
\mathbf{F}^{\mathrm{corr}}_s,
& d_t(s)\leq\tau_t\ \land\ d_R(s)\leq\tau_R,\\
\mathbf{V}_s, & \text{otherwise}.
\end{cases}
```

### Unified와 Independent

```latex
\text{Unified:}
\qquad
\min_{\{\mathbf{C}_i\},\mathbf{X},\{\mathbf{Q}_s\}}
\left(
\mathcal{E}_{\mathrm{fix}}
+
\mathcal{E}_{\mathrm{grip}}
\right)
```

```latex
\text{Independent/Separated:}
\qquad
\min_{\{\mathbf{C}_i\},\{\mathbf{Q}^{\mathrm{fix}}_s\}}
\mathcal{E}_{\mathrm{fix}}
\quad\text{and}\quad
\min_{\mathbf{X},\{\mathbf{Q}^{\mathrm{grip}}_s\}}
\mathcal{E}_{\mathrm{grip}},
\quad
\text{followed by rigid alignment }\mathbf{H}.
```

---

## 22. 예상 질문과 짧은 답변

### 질문: 목적함수 안의 항들은 모두 행렬인가?

$\mathbf{C}_i$, $\mathbf{Z}_{i,s}$, $\mathbf{G}_e$, $\mathbf{X}$, $\mathbf{Z}_{g,e}$, $\mathbf{Q}_s$는 모두 $4 \times 4$ 변환행렬이다. $\mathbf{r}(\cdot,\cdot)$ 또는 $\log(\cdot)$를 적용한 뒤에는 회전 $3$개와 이동 $3$개로 이루어진 $6$차원 벡터가 된다. 마지막의 $\|\cdot\|_2^2$가 그 벡터를 하나의 오차 숫자로 바꾼다.

### 질문: No-FK는 로봇 FK를 전혀 쓰지 않는가?

큐브 FK prior $\mathbf{F}_s$는 쓰지 않는다. 하지만 움직이는 그리퍼 카메라를 베이스에 연결하기 위한 촬영 순간 그리퍼 자세 $\mathbf{G}_e$는 사용한다.

### 질문: Corrected-FK는 soft anchor인가?

아니다. gate를 통과한 세트는 corrected FK를 그대로 anchor로 삼고, 탈락한 세트는 vision 합의를 쓴다. 세트마다 전부 아니면 전무로 결정하며, 그렇게 만든 $\mathbf{A}_s$를 최종 refinement에서 고정한다. 목적함수에 벌점 항을 더하는 legacy soft-anchor 방식과는 다르다.

### 질문: Independent와 Separated는 다른 방법인가?

이 프로젝트에서는 같은 의미다. 두 서브시스템을 따로 풀기 때문에 separated라고 설명하고, 코드와 결과표에서는 independent라고 부른다.

### 질문: 실데이터 FK 오차가 작으면 실제로도 정확한가?

반드시 그렇지는 않다. 외부 정답 장비가 없다면 FK와의 일치도일 뿐이며, 절대 물리 정확도는 아니다.
