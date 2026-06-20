# Latitudinal Dynamics

Given theta with a right hand rule direction:

## Kinetic Energy

$$
T = \frac{1}{2} m_L \left( (\dot{r} - R \dot{\theta})^2 + (r\dot{\theta})^2 \right) \times 2+ \frac{1}{2} m_w (R \dot{\theta})^2 + \frac{1}{2} m_p ((R + h) \dot{\theta})^2
$$

$$
T = m_L (\dot{r}^2 - 2R\dot{r}\dot{\theta} + R^2\dot{\theta}^2 + r^2\dot{\theta}^2) + \frac{1}{2} m_w R^2 \dot{\theta}^2 + \frac{1}{2} m_p (R + h)^2 \dot{\theta}^2
$$

## Potential Energy

$$
V = m_L g (R\cos\theta + r\sin\theta) \times 2 + m_w g R \cos\theta + m_p g (R + h) \cos\theta
$$

## Lagrangian ($L = T - V$)

$$
L = m_L \dot{r}^2 - 2m_L R\dot{r}\dot{\theta} + m_L R^2\dot{\theta}^2 + m_Lr^2\dot{\theta}^2  + \frac{1}{2} m_w R^2 \dot{\theta}^2 - 2m_L g R\cos\theta - 2m_L g r\sin\theta - m_w g R\cos\theta - m_p g (R + h) \cos\theta
$$

## Equations of Motion: $\theta$ Dynamics

$$
\frac{d}{dt} \left( \frac{\partial L}{\partial \dot{\theta}} \right) - \frac{\partial L}{\partial \theta} = Q_{\theta}
$$

$$
\frac{\partial L}{\partial \dot{\theta}} = - 2 m_L R \dot{r} + 2 m_L R^2 \dot{\theta} + 2 m_L r^2 \dot{\theta} + m_w R^2\dot{\theta} + m_p (R + h)^2 \dot{\theta}
$$

$$
\frac{d}{dt} \left( \frac{\partial L}{\partial \dot{\theta}} \right) = - 2 m_L R \ddot{r} + 2 m_L R^2 \ddot{\theta} + 2 m_L r^2 \ddot{\theta} + 4 m_L r \dot{r} \dot{\theta} + m_w R^2 \ddot{\theta} + m_p (R + h)^2 \ddot{\theta}
$$

$$
\frac{\partial L}{\partial \theta} = 2 m_L g R \sin\theta - 2 m_L g r \cos\theta + m_w g R \sin\theta + m_p g (R + h) \sin\theta
$$

$$
Q_{\theta} = 0
$$

## Equations of Motion: $r$ Dynamics

$$
\frac{d}{dt} \left( \frac{\partial L}{\partial \dot{r}} \right) - \frac{\partial L}{\partial r} = Q_r
$$

$$
\frac{\partial L}{\partial \dot{r}} = 2 m_L \dot{r} - 2 m_L R \dot{\theta}
$$

$$
\frac{d}{dt} \left( \frac{\partial L}{\partial \dot{r}} \right) = 2 m_L \ddot{r} - 2 m_L R \ddot{\theta}
$$

$$
\frac{\partial L}{\partial r} = 2 m_L r \dot{\theta}^2 - 2 m_L g \sin\theta
$$

$$
Q_{r} = F
$$

## Matrix Form

$$
\begin{bmatrix} 
2m_L(R^2 + r^2) + m_w R^2 + m_p (R + h)^2 & -2m_LR \\ 
-2m_LR & 2m_L 
\end{bmatrix} 
\begin{bmatrix} 
\ddot{\theta} \\ 
\ddot{r} 
\end{bmatrix} 
= 
\begin{bmatrix} 
-4m_L r \dot{r} \dot{\theta} + \left( 2m_L R + m_w R + m_p (R + h) \right) g \sin\theta - 2m_L g r \cos\theta \\ 
F + 2m_L r \dot{\theta}^2 - 2m_L g \sin\theta 
\end{bmatrix}
$$

Compare to lagrange form with reverted $\theta$ direction (Without interia terms and friction):

$$
\begin{bmatrix}
2m_L R^2 + m_b(R+h)^2 + m_w R^2 + 2m_L r^2 & 2m_L R \\
2m_L R & 2m_L
\end{bmatrix}
\begin{bmatrix}
\ddot{\theta} \\
\ddot r
\end{bmatrix}
= \begin{bmatrix}
2m_L gR\sin\theta + 2m_L gr\cos\theta + m_b g(R+h)\sin\theta + m_w gR\sin\theta - 4m_L r\dot r\dot\theta \\
2m_L g\sin\theta + F + 2m_L r\dot\theta^2
\end{bmatrix}
$$

The two models will equivalent when $\theta$ direction is reverted.

And the model derived in this file is also consistent with the Appells derived in the previous file.




