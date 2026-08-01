 ## Closed-Loop Eigenvalue and Eigenvector Analysis

Define the state vector as

\[
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
u_2\\
\dot{\theta}\\
\dot{u}_2
\end{bmatrix},
\qquad
u_2=r-R\theta.
\]

The linearized system is

\[
\dot{x}=Ax+BF,
\]

$$
\begin{bmatrix} 
J & 0 \\ 
0 & m
\end{bmatrix}
\begin{bmatrix} 
\dot{u}_1 \\ 
\dot{u}_2 
\end{bmatrix}
= \begin{bmatrix}
F R - m g r \cos\theta + G \sin\theta - m r (2 u_2 u_1 + R u_1^2) \\ 
F - m g \sin\theta + m r u_1^2 
\end{bmatrix}
$$

where

\[
A
=
\begin{bmatrix}
0&0&1&0\\
0&0&0&1\\[1mm]
\dfrac{G-mgR}{J}
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
\]

Consider the state-feedback controller

\[
F=-Kx,
\]

with

\[
K=
\begin{bmatrix}
k_1&k_2&k_3&k_4
\end{bmatrix}.
\]

The closed-loop system is therefore

\[
\dot{x}=(A-BK)x.
\]

The matrix \(BK\) is

\[
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
\]

Hence, the closed-loop system matrix is

\[
\boxed{
A_{\mathrm{cl}}
=
A-BK
=
\begin{bmatrix}
0&0&1&0\\
0&0&0&1\\[1mm]
\dfrac{G-mgR-Rk_1}{J}
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
\]

### Closed-loop eigenvalue equation

Let \(\lambda\) be an eigenvalue of \(A_{\mathrm{cl}}\), with corresponding
eigenvector

\[
v
=
\begin{bmatrix}
v_1\\
v_2\\
v_3\\
v_4
\end{bmatrix}.
\]

The eigenvector equation is

\[
(A_{\mathrm{cl}}-\lambda I)v=0.
\]

Explicitly,

\[
\begin{bmatrix}
-\lambda&0&1&0\\
0&-\lambda&0&1\\[1mm]
\dfrac{G-mgR-Rk_1}{J}
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
\]

The first two equations give

\[
v_3=\lambda v_1,
\qquad
v_4=\lambda v_2.
\]

Substituting these relations into the third equation gives

\[
\left[
\lambda^2
+\frac{Rk_3}{J}\lambda
+\frac{Rk_1-G+mgR}{J}
\right]v_1
+
\left[
\frac{Rk_4}{J}\lambda
+\frac{mg+Rk_2}{J}
\right]v_2
=0.
\]

Define

\[
\boxed{
P_{11}(\lambda)
=
\lambda^2
+\frac{Rk_3}{J}\lambda
+\frac{Rk_1-G+mgR}{J}
}
\]

and

\[
\boxed{
P_{12}(\lambda)
=
\frac{Rk_4}{J}\lambda
+\frac{mg+Rk_2}{J}.
}
\]

Then the third equation becomes

\[
P_{11}(\lambda)v_1
+
P_{12}(\lambda)v_2
=0.
\]

Similarly, substituting \(v_3=\lambda v_1\) and
\(v_4=\lambda v_2\) into the fourth equation gives

\[
\left[
g+\frac{k_1}{m}+\frac{k_3}{m}\lambda
\right]v_1
+
\left[
\lambda^2+\frac{k_4}{m}\lambda+\frac{k_2}{m}
\right]v_2
=0.
\]

Define

\[
\boxed{
P_{21}(\lambda)
=
g+\frac{k_1}{m}+\frac{k_3}{m}\lambda
}
\]

and

\[
\boxed{
P_{22}(\lambda)
=
\lambda^2+\frac{k_4}{m}\lambda+\frac{k_2}{m}.
}
\]

Therefore,

\[
\begin{bmatrix}
P_{11}(\lambda)&P_{12}(\lambda)\\
P_{21}(\lambda)&P_{22}(\lambda)
\end{bmatrix}
\begin{bmatrix}
v_1\\
v_2
\end{bmatrix}
=0.
\]

A nonzero eigenvector exists only if

\[
\boxed{
P_{11}(\lambda)P_{22}(\lambda)
-
P_{12}(\lambda)P_{21}(\lambda)
=0.
}
\]

Thus, the closed-loop characteristic polynomial is

\[
\boxed{
\begin{aligned}
p_{\mathrm{cl}}(\lambda)
={}&
\lambda^4
+
\left(
\frac{Rk_3}{J}
+
\frac{k_4}{m}
\right)\lambda^3\\
&+
\left(
\frac{Rk_1-G+mgR}{J}
+
\frac{k_2}{m}
\right)\lambda^2\\
&-
\left(
\frac{Gk_4}{Jm}
+
\frac{gk_3}{J}
\right)\lambda\\
&-
\left(
\frac{Gk_2}{Jm}
+
\frac{gk_1}{J}
+
\frac{mg^2}{J}
\right).
\end{aligned}
}
\]

The closed-loop eigenvalues \(\lambda_i\) satisfy

\[
\boxed{
p_{\mathrm{cl}}(\lambda_i)=0.
}
\]

### Closed-loop eigenvector

For each eigenvalue \(\lambda_i\), the position components satisfy

\[
P_{11}(\lambda_i)v_1
+
P_{12}(\lambda_i)v_2
=0.
\]

One possible choice is

\[
v_1=P_{12}(\lambda_i),
\qquad
v_2=-P_{11}(\lambda_i).
\]

Using

\[
v_3=\lambda_i v_1,
\qquad
v_4=\lambda_i v_2,
\]

an analytical eigenvector is

\[
\boxed{
v(\lambda_i)
=
\begin{bmatrix}
P_{12}(\lambda_i)\\[1mm]
-P_{11}(\lambda_i)\\[1mm]
\lambda_iP_{12}(\lambda_i)\\[1mm]
-\lambda_iP_{11}(\lambda_i)
\end{bmatrix}.
}
\]

Expanding the terms gives

\[
\boxed{
v(\lambda_i)
=
\begin{bmatrix}
\dfrac{Rk_4}{J}\lambda_i
+
\dfrac{mg+Rk_2}{J}\\[4mm]

-\lambda_i^2
-\dfrac{Rk_3}{J}\lambda_i
-\dfrac{Rk_1-G+mgR}{J}\\[4mm]

\lambda_i
\left(
\dfrac{Rk_4}{J}\lambda_i
+
\dfrac{mg+Rk_2}{J}
\right)\\[4mm]

-\lambda_i
\left(
\lambda_i^2
+
\dfrac{Rk_3}{J}\lambda_i
+
\dfrac{Rk_1-G+mgR}{J}
\right)
\end{bmatrix}.
}
\]

If

\[
P_{12}(\lambda_i)\neq0,
\]

the eigenvector can instead be normalized by setting

\[
v_1=1.
\]

Then

\[
v_2
=
-\frac{P_{11}(\lambda_i)}{P_{12}(\lambda_i)},
\]

and the normalized eigenvector becomes

\[
\boxed{
v(\lambda_i)
=
\begin{bmatrix}
1\\[3mm]
-\dfrac{P_{11}(\lambda_i)}
{P_{12}(\lambda_i)}\\[5mm]
\lambda_i\\[3mm]
-\lambda_i
\dfrac{P_{11}(\lambda_i)}
{P_{12}(\lambda_i)}
\end{bmatrix}.
}
\]

Therefore, within a single closed-loop eigenmode,

\[
\boxed{
\frac{u_2}{\theta}
=
-\frac{
\lambda_i^2
+\dfrac{Rk_3}{J}\lambda_i
+\dfrac{Rk_1-G+mgR}{J}
}{
\dfrac{Rk_4}{J}\lambda_i
+\dfrac{mg+Rk_2}{J}
}.
}
\]

Since

\[
\dot{\theta}=\lambda_i\theta,
\qquad
\dot{u}_2=\lambda_i u_2,
\]

we also have

\[
\boxed{
\frac{\dot{u}_2}{\dot{\theta}}
=
\frac{u_2}{\theta}.
}
\]

If we assume $\frac{k_3}{k_4} = \frac{k_1}{k_2} = \rho$, then we have

$$
\frac{u_2}{\theta} = -\frac{\lambda_i^2 + \frac{Rk_3}{J}\lambda_i + \frac{Rk_1-G+mgR}{J}}{\frac{Rk_4}{J}\lambda_i + \frac{mg+Rk_2}{J}} = \\
 -\frac{\lambda_i^2 +\frac{R\rho k_4}{J}\lambda_i + \frac{R\rho k_2}{J} +\frac{\rho mg}{J} + \frac{-G+mgR - \rho mg}{J}}{\frac{Rk_4}{J}\lambda_i + \frac{mg+Rk_2}{J}} \\
 = - \rho - \frac{-G+mgR - \rho mg + \lambda_i^2J}{Rk_4\lambda_i + mg + Rk_2}
$$