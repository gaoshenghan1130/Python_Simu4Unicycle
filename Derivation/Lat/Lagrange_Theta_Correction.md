# Latitudinal Dynamics

Given theta with a right hand rule direction:

## Kinetic Energy

$$
T = \frac{1}{2} m_L \left( (\dot{r} - R \dot{\theta})^2 + (r\dot{\theta})^2 \right) \times 2+ \frac{1}{2} m_w (R \dot{\theta})^2 
$$

$$
T = m_L (\dot{r}^2 - 2R\dot{r}\dot{\theta} + R^2\dot{\theta}^2 + r^2\dot{\theta}^2) + \frac{1}{2} m_w R^2 \dot{\theta}^2 
$$

## Potential Energy

$$
V = m_L g (R\cos\theta + r\sin\theta) \times 2 + m_w g R \cos\theta 
$$

## Lagrangian ($L = T - V$)

$$
L = m_L \dot{r}^2 - 2m_L R\dot{r}\dot{\theta} + m_L R^2\dot{\theta}^2 + m_Lr^2\dot{\theta}^2  + \frac{1}{2} m_w R^2 \dot{\theta}^2 - 2m_L g R\cos\theta - 2m_L g r\sin\theta - m_w g R\cos\theta
$$

## Equations of Motion: $\theta$ Dynamics

$$
\frac{d}{dt} \left( \frac{\partial L}{\partial \dot{\theta}} \right) - \frac{\partial L}{\partial \theta} = Q_{\theta}
$$

$$
\frac{\partial L}{\partial \dot{\theta}} = - 2 m_L R \dot{r} + 2 m_L R^2 \dot{\theta} + 2 m_L r^2 \dot{\theta} + m_w R^2\dot{\theta}
$$

$$
\frac{d}{dt} \left( \frac{\partial L}{\partial \dot{\theta}} \right) = - 2 m_L R \ddot{r} + 2 m_L R^2 \ddot{\theta} + 2 m_L r^2 \ddot{\theta} + 4 m_L r \dot{r} \dot{\theta} + m_w R^2 \ddot{\theta} 
$$

$$
\frac{\partial L}{\partial \theta} = 2 m_L g R \sin\theta - 2 m_L g r \cos\theta + m_w g R \sin\theta
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
\begin{bmatrix} 2m_L(R^2 + r^2) + m_w R^2 & -2m_LR \\ -2m_LR & 2m_L \end{bmatrix} \begin{bmatrix} \ddot{\theta} \\ \ddot{r} \end{bmatrix} = \begin{bmatrix} -2m_L r \dot{r} \dot{\theta} - 2m_L r \dot{\theta}^2 + \left( 2m_L R + m_w R \right) g \sin\theta - 2m_L g r \cos\theta \\ F + 2m_L r \dot{\theta}^2 - 2m_L g \sin\theta \end{bmatrix}
$$

$$
\begin{bmatrix} 2m_L r^2 + m_w R^2 & 0 \\ 0 & 2m_L \end{bmatrix} \begin{bmatrix} \ddot{\theta} \\ \ddot{x} \end{bmatrix} = \begin{bmatrix} RF - 2m_L r \dot{\theta}(\dot{r} + \dot{x}) + m_w R g \sin\theta - 2m_L g r \cos\theta \\ F + 2m_L r \dot{\theta}^2 - 2m_L g \sin\theta \end{bmatrix}
$$


Compare to appell form

$$
\begin{bmatrix} 
m_w R^2 + m_{rod} r^2 & 0 \\ 
0 & m_{rod} 
\end{bmatrix}
\begin{bmatrix} 
\dot{u}_1 \\ 
\dot{u}_2 
\end{bmatrix}
=
\begin{bmatrix} 
F R - m_{rod} g r \cos\theta + m_w g R \sin\theta - m_{rod} r (2 u_2 u_1 + R u_1^2) \\ 
F - m_{rod} g \sin\theta + m_{rod} r u_1^2 
\end{bmatrix}
$$



