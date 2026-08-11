# Open-Loop Eigenvalue and Eigenvector Analysis

Define the generalized speeds as

$$
u_1=\dot\theta,
\qquad
u_2=\dot r-R\dot\theta.
$$

The state vector is

$$
x=
\begin{bmatrix}
x_1\\x_2\\x_3\\x_4
\end{bmatrix}
=
\begin{bmatrix}
\theta\\r\\u_1\\u_2
\end{bmatrix}.
$$

Therefore, the kinematic equations are

$$
\dot\theta=u_1,
\qquad
\dot r=Ru_1+u_2.
$$

Also define

$$
J_0=m_wR^2+m_p(R+h)^2+I_w+I_b+I_{rod},
\qquad
J(r)=J_0+mr^2. \\
\qquad
G = m_w g R + m_p g (R + h) \\
$$

## Equilibria

The equilibrium curve is

$$
F^*=mg\sin\theta^*,
\qquad
r^*=\frac{G+mgR}{mg}\tan\theta^*,
$$

with

$$
u_1^*=u_2^*=0,
\qquad
J^*=J_0+m(r^*)^2.
$$

For a nontrivial equilibrium, $F^*\neq0$. Therefore, open loop means that no state feedback is applied while the equilibrium force is held constant:

$$
K=0,
\qquad
F=F^*,
\qquad
\delta F=0.
$$

## Linearization Along the Equilibrium Curve

The open-loop linearization around

$$
x^*=\begin{bmatrix}\theta^*&r^*&0&0\end{bmatrix}^{T}
$$

is

$$
\delta\dot x=A_{ol}(\theta^*)\delta x.
$$

At the equilibrium point, the numerator of the $\dot u_1$ equation is zero. Therefore, differentiating $1/J(r)$ does not contribute to the first-order model. The velocity-dependent terms are quadratic and also vanish to first order.

Define

$$
a_\theta^*
=\frac{mgr^*\sin\theta^*+G\cos\theta^*}{J^*},
$$

$$
a_r^*=-\frac{mg\cos\theta^*}{J^*},
\qquad
c^*=-g\cos\theta^*.
$$

Then

$$
\boxed{
A_{ol}(\theta^*)=
\begin{bmatrix}
0&0&1&0\\
0&0&R&1\\
a_\theta^*&a_r^*&0&0\\
c^*&0&0&0
\end{bmatrix}.
}
$$

If an input perturbation is retained, then

$$
\delta\dot x=A_{ol}(\theta^*)\delta x+B^*\delta F,
$$

where

$$
B^*=
\begin{bmatrix}
0\\
0\\
\dfrac{R}{J^*}\\[1mm]
\dfrac{1}{m}
\end{bmatrix}.
$$

Using the equilibrium curve, $a_\theta^*$ becomes

$$
\boxed{
a_\theta^*
=\frac{G+mgR\sin^2\theta^*}{J^*\cos\theta^*}.
}
$$

It is also useful to define

$$
\eta^*=a_\theta^*+Ra_r^*.
$$

Along the equilibrium curve,

$$
\boxed{
\eta^*
=\frac{G-mgR\cos(2\theta^*)}{J^*\cos\theta^*}.
}
$$

## Open-Loop Eigenvalues

Let $\lambda$ be an eigenvalue with eigenvector

$$
v=\begin{bmatrix}v_1&v_2&v_3&v_4\end{bmatrix}^{T}.
$$

The first two eigenvector equations give

$$
v_3=\lambda v_1,
$$

$$
Rv_3+v_4=\lambda v_2,
$$

and hence

$$
v_4=\lambda(v_2-Rv_1).
$$

The third and fourth equations reduce to

$$
\begin{bmatrix}
\lambda^2-a_\theta^*&-a_r^*\\
-(c^*+R\lambda^2)&\lambda^2
\end{bmatrix}
\begin{bmatrix}
v_1\\v_2
\end{bmatrix}
=0.
$$

A nonzero eigenvector exists only if

$$
\lambda^4
-(a_\theta^*+Ra_r^*)\lambda^2
-a_r^*c^*=0.
$$

Therefore, the characteristic equation is

$$
\boxed{
\lambda^4-\eta^*\lambda^2
-\frac{mg^2\cos^2\theta^*}{J^*}=0.
}
$$

Define

$$
D^*
=\sqrt{(\eta^*)^2
+\frac{4mg^2\cos^2\theta^*}{J^*}}.
$$

The four eigenvalues are

$$
\boxed{
\lambda_{1,2}
=\pm\sqrt{\frac{\eta^*+D^*}{2}}
}
$$

and

$$
\boxed{
\lambda_{3,4}
=\pm i\sqrt{\frac{D^*-\eta^*}{2}}.
}
$$

Thus, the uncontrolled system generally has one unstable real eigenvalue, one stable real eigenvalue, and one imaginary conjugate pair.

## Open-Loop Eigenvectors

From

$$
c^*v_1=\lambda_i v_4
$$

and

$$
v_4=\lambda_i(v_2-Rv_1),
$$

we obtain

$$
v_2
=\left(R+\frac{c^*}{\lambda_i^2}\right)v_1
=\left(R-\frac{g\cos\theta^*}{\lambda_i^2}\right)v_1.
$$

Setting $v_1=1$ gives

$$
\boxed{
v_i=
\begin{bmatrix}
1\\[2mm]
R-\dfrac{g\cos\theta^*}{\lambda_i^2}\\[3mm]
\lambda_i\\[2mm]
-\dfrac{g\cos\theta^*}{\lambda_i}
\end{bmatrix}.
}
$$

An equivalent form without division is

$$
\boxed{
v_i\propto
\begin{bmatrix}
\lambda_i^2\\
R\lambda_i^2-g\cos\theta^*\\
\lambda_i^3\\
-g\lambda_i\cos\theta^*
\end{bmatrix}.
}
$$

Consequently, within a single open-loop eigenmode,

$$
\boxed{
\frac{r}{\theta}
=R-\frac{g\cos\theta^*}{\lambda_i^2}.
}
$$

For the generalized displacement $r-R\theta$,

$$
\boxed{
\frac{r-R\theta}{\theta}
=-\frac{g\cos\theta^*}{\lambda_i^2}.
}
$$

## Trivial Equilibrium

At the trivial equilibrium,

$$
\theta^*=r^*=0,
\qquad
F^*=0,
\qquad
J^*=J_0.
$$

The coefficients become

$$
a_{\theta,0}=\frac{G}{J_0},
\qquad
a_{r,0}=-\frac{mg}{J_0},
\qquad
c_0=-g,
$$

and

$$
\eta_0
=a_{\theta,0}+Ra_{r,0}
=\frac{G-mgR}{J_0}.
$$

Thus,

$$
A_{ol}(0)=
\begin{bmatrix}
0&0&1&0\\
0&0&R&1\\
\dfrac{G}{J_0}&-\dfrac{mg}{J_0}&0&0\\[2mm]
-g&0&0&0
\end{bmatrix}.
$$

The characteristic equation is

$$
\boxed{
\lambda^4
-\frac{G-mgR}{J_0}\lambda^2
-\frac{mg^2}{J_0}=0.
}
$$

Define

$$
D_0
=\sqrt{
\left(\frac{G-mgR}{J_0}\right)^2
+\frac{4mg^2}{J_0}
}.
$$

The eigenvalues are

$$
\lambda_{1,2}
=\pm\sqrt{
\frac{\dfrac{G-mgR}{J_0}+D_0}{2}
},
$$

$$
\lambda_{3,4}
=\pm i\sqrt{
\frac{D_0-\dfrac{G-mgR}{J_0}}{2}
}.
$$

For each eigenvalue $\lambda_i$, the eigenvector is

$$
\boxed{
v_i=
\begin{bmatrix}
1\\[2mm]
R-\dfrac{g}{\lambda_i^2}\\[3mm]
\lambda_i\\[2mm]
-\dfrac{g}{\lambda_i}
\end{bmatrix}.
}
$$
