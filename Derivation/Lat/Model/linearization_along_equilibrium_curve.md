# Linearization Along the Equilibrium Curve

Given:

$$
\begin{bmatrix} 
m_w R^2 + m_{rod} r^2 + m_p (R + h)^2 + (I_w + I_b  + I_{rod}) & 0 \\ 
0 & m_{rod} 
\end{bmatrix}
\begin{bmatrix} 
\dot{u}_1 \\ 
\dot{u}_2 
\end{bmatrix}
= \begin{bmatrix} 
F R - m_{rod} g r \cos\theta + m_w g R \sin\theta + m_p g (R + h) \sin\theta - m_{rod} r (2 u_2 u_1 + R u_1^2) \\ 
F - m_{rod} g \sin\theta + m_{rod} r u_1^2 
\end{bmatrix}
$$

Define

$$
J_0=m_wR^2+m_p(R+h)^2+I_w+I_b+I_{rod},
\qquad J(r)=J_0+mr^2,
$$

where $m=m_{rod}$ and

$$
G=m_wgR+m_pg(R+h).
$$

Using $u_1=\dot\theta$ and $u_2=\dot r-R\dot\theta$, the equilibrium curve is

$$
F_e=mg\sin\theta_e,
\qquad
r_e=\frac{G+mgR}{mg}\tan\theta_e,
$$

with $u_{1,e}=u_{2,e}=0$. Define

$$
J_e=J_0+mr_e^2,
$$

$$
a_\theta
=\frac{mgr_e\sin\theta_e+G\cos\theta_e}{J_e}
=\frac{G+mgR\sin^2\theta_e}{J_e\cos\theta_e},
$$

$$
a_r=-\frac{mg\cos\theta_e}{J_e}.
$$

## Linearization in the Original Generalized-Speed Coordinates

Use the state

$$
x_u=\begin{bmatrix}\theta&r&u_1&u_2\end{bmatrix}^{T},
$$

with the kinematic equations

$$
\dot\theta=u_1,
\qquad
\dot r=Ru_1+u_2.
$$

Let

$$
\theta=\theta_e+\delta\theta,
\quad r=r_e+\delta r,
\quad F=F_e+\delta F,
\quad u_1=\delta u_1,
\quad u_2=\delta u_2.
$$

At the equilibrium point, the numerator of the $\dot u_1$ equation is zero. Therefore, differentiating $1/J(r)$ does not contribute to the first-order model. The velocity-dependent terms are quadratic and also vanish to first order. Hence,

$$
\delta\dot u_1
=a_\theta\delta\theta+a_r\delta r
+\frac{R}{J_e}\delta F,
$$

$$
\delta\dot u_2
=-g\cos\theta_e\,\delta\theta
+\frac{1}{m}\delta F.
$$

Thus,

$$
\delta\dot x_u=A_u\delta x_u+B_u\delta F,
$$

where

$$
A_u=
\begin{bmatrix}
0&0&1&0\\
0&0&R&1\\
a_\theta&a_r&0&0\\
-g\cos\theta_e&0&0&0
\end{bmatrix},
\qquad
B_u=
\begin{bmatrix}
0\\
0\\
\dfrac{R}{J_e}\\
\dfrac{1}{m}
\end{bmatrix}.
$$

## Linearization in the Physical State Coordinates

For the state

$$
x=\begin{bmatrix}\theta&\dot\theta&r&\dot r\end{bmatrix}^{T},
$$

the linearized system around $(\theta_e,r_e,F_e)$ is

$$
\delta\dot x=A(\theta_e)\delta x+B(\theta_e)\delta F,
$$

where

$$
A(\theta_e)=
\begin{bmatrix}
0&1&0&0\\
a_\theta&0&a_r&0\\
0&0&0&1\\
Ra_\theta-g\cos\theta_e&0&Ra_r&0
\end{bmatrix},
$$

$$
B(\theta_e)=
\begin{bmatrix}
0\\[1mm]
\dfrac{R}{J_e}\\[1mm]
0\\[1mm]
\dfrac{R^2}{J_e}+\dfrac{1}{m}
\end{bmatrix}.
$$

If the equilibrium force is included as feedforward,

$$
F=mg\sin\theta+v,
$$

then

$$
\delta\dot x=A_{curve}(\theta_e)\delta x+B(\theta_e)\delta v,
$$

with

$$
A_{curve}(\theta_e)=
\begin{bmatrix}
0&1&0&0\\
a_c&0&a_r&0\\
0&0&0&1\\
Ra_c&0&Ra_r&0
\end{bmatrix},
\qquad
a_c=\frac{G+mgR}{J_e\cos\theta_e}.
$$

The tangent direction of the equilibrium curve satisfies

$$
A_{curve}
\begin{bmatrix}
1&0&r_e'(\theta_e)&0
\end{bmatrix}^{T}=0,
\qquad
r_e'(\theta_e)=\frac{G+mgR}{mg}\sec^2\theta_e.
$$
