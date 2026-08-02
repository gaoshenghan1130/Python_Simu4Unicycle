# Closed-Loop Eigenvalue and Eigenvector Analysis

Define the state vector as

$$
x
=
\begin{bmatrix}
x_1\\
x_2\\
x_3\\
x_4
\end{bmatrix}
=
\begin{bmatrix}
\theta\\
r\\
u_1\\
u_2
\end{bmatrix}.
$$

The transformed coordinate used previously is

$$
q_2=r-R\theta.
$$

The generalized speeds are defined as

$$
u_1=\dot\theta,
\qquad
u_2=\dot q_2=\dot r-R\dot\theta.
$$

Therefore, the kinematic equations are

$$
\boxed{
\dot\theta=u_1,
\qquad
\dot r=Ru_1+u_2.
}
$$

The nonlinear dynamics are

$$
\begin{bmatrix} 
J & 0 \\ 
0 & m
\end{bmatrix}
\begin{bmatrix} 
\dot{u}_1 \\ 
\dot{u}_2 
\end{bmatrix}
=
\begin{bmatrix}
FR-mgr\cos\theta+G\sin\theta
-mr\left(2u_2u_1+Ru_1^2\right)
\\[1mm]
F-mg\sin\theta+mru_1^2
\end{bmatrix}.
$$

Linearizing around

$$
\theta=0,
\qquad
r=0,
\qquad
u_1=0,
\qquad
u_2=0,
$$

gives

$$
J\dot u_1=G\theta-mgr+RF,
$$

and

$$
m\dot u_2=-mg\theta+F.
$$

Therefore, the linearized system is

$$
\dot x=Ax+BF,
$$

where

$$
\boxed{
A
=
\begin{bmatrix}
0&0&1&0\\
0&0&R&1\\[1mm]
\dfrac{G}{J}
&
-\dfrac{mg}{J}
&
0
&
0\\[3mm]
-g&0&0&0
\end{bmatrix},
\qquad
B
=
\begin{bmatrix}
0\\
0\\
\dfrac{R}{J}\\[2mm]
\dfrac{1}{m}
\end{bmatrix}.
}
$$

Consider the state-feedback controller

$$
F=-Kx,
$$

with

$$
K
=
\begin{bmatrix}
k_1&k_2&k_3&k_4
\end{bmatrix}.
$$

Thus,

$$
F=-k_1\theta-k_2r-k_3u_1-k_4u_2.
$$

The closed-loop system is

$$
\dot x=(A-BK)x.
$$

The matrix product is

$$
BK
=
\begin{bmatrix}
0&0&0&0\\
0&0&0&0\\[1mm]
\dfrac{Rk_1}{J}
&
\dfrac{Rk_2}{J}
&
\dfrac{Rk_3}{J}
&
\dfrac{Rk_4}{J}\\[3mm]
\dfrac{k_1}{m}
&
\dfrac{k_2}{m}
&
\dfrac{k_3}{m}
&
\dfrac{k_4}{m}
\end{bmatrix}.
$$

Hence, the closed-loop system matrix is

$$
\boxed{
A_{\mathrm{cl}}
=A-BK
=
\begin{bmatrix}
0&0&1&0\\
0&0&R&1\\[1mm]
\dfrac{G-Rk_1}{J}
&
-\dfrac{mg+Rk_2}{J}
&
-\dfrac{Rk_3}{J}
&
-\dfrac{Rk_4}{J}\\[3mm]
-g-\dfrac{k_1}{m}
&
-\dfrac{k_2}{m}
&
-\dfrac{k_3}{m}
&
-\dfrac{k_4}{m}
\end{bmatrix}.
}
$$

## Closed-loop eigenvalue equation

Let lambda be an eigenvalue of the closed-loop matrix, with corresponding eigenvector

$$
v
=
\begin{bmatrix}
v_1\\
v_2\\
v_3\\
v_4
\end{bmatrix}.
$$

The eigenvector equation is

$$
(A_{\mathrm{cl}}-\lambda I)v=0.
$$

Explicitly,

$$
\begin{bmatrix}
-\lambda&0&1&0\\
0&-\lambda&R&1\\[1mm]
\dfrac{G-Rk_1}{J}
&
-\dfrac{mg+Rk_2}{J}
&
-\dfrac{Rk_3}{J}-\lambda
&
-\dfrac{Rk_4}{J}\\[3mm]
-g-\dfrac{k_1}{m}
&
-\dfrac{k_2}{m}
&
-\dfrac{k_3}{m}
&
-\dfrac{k_4}{m}-\lambda
\end{bmatrix}
\begin{bmatrix}
v_1\\
v_2\\
v_3\\
v_4
\end{bmatrix}
=0.
$$

The first equation gives

$$
v_3=\lambda v_1.
$$

The second equation gives

$$
Rv_3+v_4=\lambda v_2.
$$

Therefore,

$$
\boxed{
v_3=\lambda v_1,
\qquad
v_4=\lambda(v_2-Rv_1).
}
$$

Substituting these relations into the third equation gives

$$
\left[
\lambda^2
+\frac{R(k_3-Rk_4)}{J}\lambda
+\frac{Rk_1-G}{J}
\right]v_1
+
\left[
\frac{Rk_4}{J}\lambda
+\frac{mg+Rk_2}{J}
\right]v_2
=0.
$$

Define

$$
\boxed{
P_{11}(\lambda)
=
\lambda^2
+\frac{R(k_3-Rk_4)}{J}\lambda
+\frac{Rk_1-G}{J}
}
$$

and

$$
\boxed{
P_{12}(\lambda)
=
\frac{Rk_4}{J}\lambda
+\frac{mg+Rk_2}{J}.
}
$$

Then the third equation becomes

$$
P_{11}(\lambda)v_1+P_{12}(\lambda)v_2=0.
$$

Similarly, substituting the eigenvector relations into the fourth equation gives

$$
\left[
-R\lambda^2
+\frac{k_3-Rk_4}{m}\lambda
+g+\frac{k_1}{m}
\right]v_1
+
\left[
\lambda^2
+\frac{k_4}{m}\lambda
+\frac{k_2}{m}
\right]v_2
=0.
$$

Define

$$
\boxed{
P_{21}(\lambda)
=
-R\lambda^2
+\frac{k_3-Rk_4}{m}\lambda
+g+\frac{k_1}{m}
}
$$

and

$$
\boxed{
P_{22}(\lambda)
=
\lambda^2
+\frac{k_4}{m}\lambda
+\frac{k_2}{m}.
}
$$

Therefore,

$$
\begin{bmatrix}
P_{11}(\lambda)&P_{12}(\lambda)\\
P_{21}(\lambda)&P_{22}(\lambda)
\end{bmatrix}
\begin{bmatrix}
v_1\\
v_2
\end{bmatrix}
=0.
$$

A nonzero eigenvector exists only if

$$
\boxed{
P_{11}(\lambda)P_{22}(\lambda)
-P_{12}(\lambda)P_{21}(\lambda)
=0.
}
$$

Thus, the closed-loop characteristic polynomial is

$$
\boxed{
\begin{aligned}
p_{\mathrm{cl}}(\lambda)
={}&
\lambda^4
+
\left(
\frac{Rk_3}{J}
+\frac{k_4}{m}
\right)\lambda^3
\\[1mm]
&+
\left[
\frac{Rk_1-G+mgR+R^2k_2}{J}
+\frac{k_2}{m}
\right]\lambda^2
\\[1mm]
&-
\left(
\frac{Gk_4}{Jm}
+\frac{gk_3}{J}
\right)\lambda
\\[1mm]
&-
\left[
\frac{Gk_2}{Jm}
+\frac{g(k_1+Rk_2)}{J}
+\frac{mg^2}{J}
\right].
\end{aligned}
}
$$

The closed-loop eigenvalues satisfy

$$
\boxed{
p_{\mathrm{cl}}(\lambda_i)=0.
}
$$

## Closed-loop eigenvector

For each eigenvalue,

$$
P_{11}(\lambda_i)v_1+P_{12}(\lambda_i)v_2=0.
$$

One possible choice is

$$
v_1=P_{12}(\lambda_i),
\qquad
v_2=-P_{11}(\lambda_i).
$$

Using the kinematic relations, an analytical eigenvector is

$$
\boxed{
v(\lambda_i)
=
\begin{bmatrix}
P_{12}(\lambda_i)\\[1mm]
-P_{11}(\lambda_i)\\[1mm]
\lambda_iP_{12}(\lambda_i)\\[1mm]
-\lambda_i\left[P_{11}(\lambda_i)+RP_{12}(\lambda_i)\right]
\end{bmatrix}.
}
$$

If

$$
P_{12}(\lambda_i)\neq0,
$$

the eigenvector can be normalized by setting

$$
v_1=1.
$$

Define

$$
\boxed{
\eta_i
=
\frac{r}{\theta}
=
-\frac{P_{11}(\lambda_i)}{P_{12}(\lambda_i)}.
}
$$

The normalized eigenvector becomes

$$
\boxed{
v_i
=
\begin{bmatrix}
1\\
\eta_i\\
\lambda_i\\
\lambda_i(\eta_i-R)
\end{bmatrix}.
}
$$


In expension form, the normalized eigenvector is

$$
\begin{bmatrix}
1\\[1mm]
-\dfrac{\lambda_i^2+\frac{Rk_3}{J}\lambda_i+\frac{Rk_1-G}{J}}{\frac{Rk_4}{J}\lambda_i+\frac{mg+Rk_2}{J}}\\[1mm]
\lambda_i\\[1mm]
-\lambda_i\left[\dfrac{\lambda_i^2+\frac{Rk_3}{J}\lambda_i+\frac{Rk_1-G}{J}}{\frac{Rk_4}{J}\lambda_i+\frac{mg+Rk_2}{J}}-R\right]
\end{bmatrix}.
$$