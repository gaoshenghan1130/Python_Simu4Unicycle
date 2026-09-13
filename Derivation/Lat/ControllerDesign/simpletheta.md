# Controller with only-theta concerned movement

Given our system:

$$
u_1 = \theta \\
u_2 = r - R\theta
$$

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

The state-space graph of must be able to cross the equilibrium curve: $r = \frac{G}{mg}\tan\theta$ in order to stabilize the system.

However, due to the limitation of the model itself, the movement of the states will stay close to the curve of (With a given initial condition of $\theta_0$ and $r_0$):

$$
\boxed{
\theta_{\mathrm{lim}}(r)
=
\theta_0+
\frac{R}{\ell}
\left[
\tan^{-1}\left(\frac{r}{\ell}\right)
-
\tan^{-1}\left(\frac{r_0}{\ell}\right)
\right]
}.
$$

Or to simplify, we assume $r_0 = 0$ and express the curve back into the form of $r$ (with $a = \frac{G + mgR}{mg}$):

$$
r = \ell\tan\left(
\frac{\ell}{R}(\theta-\theta_0)+\tan^{-1}\left(\frac{r_0}{\ell}\right)
\right)
=
\ell\tan\left(
\frac{\ell}{R}(\theta-\theta_0)
\right) 
$$

Still keep the same simplification of $$
m=m_{\mathrm{rod}},
$$

$$
J(r)=J_0+mr^2,
\qquad
J_0=m_wR^2+m_p(R+h)^2+I_w+I_b+I_{\mathrm{rod}},
$$

$$
G=m_wgR+m_pg(R+h).
$$

The equation of motion can then be written as

$$
\begin{bmatrix}
J(r) & 0\\
0 & m
\end{bmatrix}
\begin{bmatrix}
\dot u_1\\
\dot u_2
\end{bmatrix}
=
\begin{bmatrix}
FR-mgr\cos\theta+G\sin\theta
-mr\left(2u_2u_1+Ru_1^2\right)
\\[1mm]
F-mg\sin\theta+mru_1^2
\end{bmatrix},
$$


## Time Optimization for limited state space region:

We want