# Control strategy possibility proof

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

We want to know whether the control strategy of first moving theta is possible. That is, we can calculate the eiginvector of the system and see if we can find a direction that results in a positive $\dot{\theta}$ and a negative $\dot{r}$, which means we can move theta first and then move r.

## Simplification

Define:

$$
J = m_w R^2 + m_p (R + h)^2 + (I_w + I_b  + I_{rod}) \\
m = m_{rod} \\
G = m_w g R + m_p g (R + h) \\
$$

And approximate $r$ to be small, then we can simplify the system to (neglect $m_{rod}r^2$):

Then:

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

## Jacobian

Accordingly, we can get the Jacobian of the system (Linearize at the equilibrium point $\theta = 0, r = 0, u_1 = 0, u_2 = 0$):

$$
A = \begin{bmatrix}
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1 \\
\frac{G}{J} & -\frac{m g}{J} & 0 & 0 \\
-g & 0 & 0 & 0
\end{bmatrix}
$$

## Eigenvector & Eigenvalue

To get the eigenvalues and eigenvectors, we can use the characteristic polynomial:

$$
\det(A - \lambda I) = 0
$$

$$
\det \big(\begin{bmatrix}
-\lambda&0&1&0\\
0&-\lambda&0&1\\
\dfrac{G}{J}&-\dfrac{mg}{J}&-\lambda&0\\
-g&0&0&-\lambda
\end{bmatrix} \big) = 0
$$

\[
\begin{aligned}
\det(A-\lambda I)
={}&
(-g)(-1)^{4+1}
\begin{vmatrix}
0&1&0\\
-\lambda&0&1\\
-\dfrac{mg}{J}&-\lambda&0
\end{vmatrix}\\
&+
(-\lambda)(-1)^{4+4}
\begin{vmatrix}
-\lambda&0&1\\
0&-\lambda&0\\
\dfrac{G}{J}&-\dfrac{mg}{J}&-\lambda
\end{vmatrix}.
\end{aligned}
\]


For the first minor,

\[
\begin{aligned}
\begin{vmatrix}
0&1&0\\
-\lambda&0&1\\
-\dfrac{mg}{J}&-\lambda&0
\end{vmatrix}
={}&
0\begin{vmatrix}
0&1\\
-\lambda&0
\end{vmatrix}
-1\begin{vmatrix}
-\lambda&1\\
-\dfrac{mg}{J}&0
\end{vmatrix}
+0\begin{vmatrix}
-\lambda&0\\
-\dfrac{mg}{J}&-\lambda
\end{vmatrix}\\
={}&
-\dfrac{mg}{J}
\end{aligned}
\]

For the second minor,

\[
\begin{aligned}
\begin{vmatrix}
-\lambda&0&1\\
0&-\lambda&0\\
\dfrac{G}{J}&-\dfrac{mg}{J}&-\lambda
\end{vmatrix}
={}&
-\lambda\begin{vmatrix}
-\lambda&0\\
-\dfrac{mg}{J}&-\lambda
\end{vmatrix}
-0\begin{vmatrix}
0&1\\
\dfrac{G}{J}&-\lambda
\end{vmatrix}
+1\begin{vmatrix}
0&-\lambda\\
\dfrac{G}{J}&-\dfrac{mg}{J}
\end{vmatrix}\\
={}&
-\lambda(\lambda^2) + \dfrac{G}{J}\lambda\\
={}&
-\lambda^3 + \dfrac{G}{J}\lambda
\end{aligned}
\]

Thus the characteristic polynomial is:

\[
\begin{aligned}
\det(A-\lambda I)
={}&
-g\dfrac{mg}{J} - \lambda(-\lambda^3 + \dfrac{G}{J}\lambda)\\
={}&
-\dfrac{mg^2}{J} + \lambda^4 - \dfrac{G}{J}\lambda^2\\
={}&
\lambda^4 - \dfrac{G}{J}\lambda^2 - \dfrac{mg^2}{J}
\end{aligned} 
\]

Define $\mu = \lambda^2$, then we have:

\[
\mu^2 - \dfrac{G}{J}\mu - \dfrac{mg^2}{J} = 0
\]

And 

$$\mu_{1,2} = \dfrac{\dfrac{G}{J} \pm \sqrt{(\dfrac{G}{J})^2 + 4\dfrac{mg^2}{J}}}{2}$$

$$
\lambda_{1,2} = \pm\sqrt{\mu_1}, \lambda_{3,4} = \pm\sqrt{\mu_2}
$$

Now consider the corresponding eigenvectors. For $v = \begin{bmatrix} v_1 \\ v_2 \\ v_3 \\ v_4 \end{bmatrix}$, we have:

$$
\begin{bmatrix}
-\lambda&0&1&0\\
0&-\lambda&0&1\\
\dfrac{G}{J}&-\dfrac{mg}{J}&-\lambda&0\\
-g&0&0&-\lambda
\end{bmatrix}
\begin{bmatrix} v_1 \\ v_2 \\ v_3 \\ v_4 \end{bmatrix} = 0
$$

Expanding this, we get the following system of equations:

$$
\begin{aligned}
-\lambda v_1 + v_3 &= 0 \\
-\lambda v_2 + v_4 &= 0 \\
\dfrac{G}{J} v_1 - \dfrac{mg}{J} v_2 - \lambda v_3 &= 0 \\
-g v_1 - \lambda v_4 &= 0
\end{aligned}
$$

Thus (set $v_1 = 1$), we have:

$$
v_3 = \lambda, \\
v_2 = \dfrac{\frac{G}{J} - \lambda^2}{\frac{mg}{J}} = \dfrac{G - J\lambda^2}{mg}, \\
v_4 = \lambda v_2 = \lambda \dfrac{G - J\lambda^2}{mg}
$$

In fact, notice that:

$$
\dfrac{G - J\mu}{mg} = - \frac{g}{\mu}
$$

The expression of the eigenvector is:

$$
v = \begin{bmatrix} 1 \\ -\frac{g}{\mu} \\ \lambda \\ -\frac{g\lambda}{\mu} \end{bmatrix}
$$

where $\lambda$ is the eigenvalue and $\mu = \lambda^2$.

$$
\mu = \dfrac{\dfrac{G}{J} \pm \sqrt{(\dfrac{G}{J})^2 + 4\dfrac{mg^2}{J}}}{2}
$$