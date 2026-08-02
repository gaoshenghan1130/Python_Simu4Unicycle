# Equilibrium Curve

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

After simplifying the above equation, we have:

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


Where:

$$
u_1 = \dot\theta,
\qquad
u_2 = \dot r-R\dot\theta.
$$




At equilibrium, we have $\dot{u}_1 = 0, u_1 = 0$ and $\dot{u}_2 = 0, u_2 = 0$, which gives us the following equations:

$$
FR-mgr\cos\theta+G\sin\theta = 0
$$

$$
F-mg\sin\theta= 0
$$

So:

$$
F = mg\sin\theta \\
r = \frac{G\sin\theta + mgR\sin\theta}{mg\cos\theta} \\
= \frac{G + mgR}{mg} tan\theta 
$$


Plotting the above equation gives us the equilibrium curve of the system:

