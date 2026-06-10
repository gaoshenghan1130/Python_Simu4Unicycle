# Latitudinal Dynamics

## Kenetic Energy

$$
T = \frac{1}{2} m_L (\dot{r} + R \dot{\theta})^2 \times 2+ \frac{1}{2} m_b ((R+h) \dot{\theta})^2 + \frac{1}{2} m_w (R \dot{\theta})^2 + \frac{1}{2} I_b \dot{\theta}^2 + \frac{1}{2} I_w \dot{\theta}^2 + \frac{1}{2} I_{rod} \dot{\theta}^2
$$

$$
T = m_L (\dot{r}^2 + 2R\dot{r}\dot{\theta} + R^2\dot{\theta}^2) + \frac{1}{2} m_b (R+h)^2 \dot{\theta}^2 + \frac{1}{2} m_w R^2 \dot{\theta}^2 + \frac{1}{2} I_b \dot{\theta}^2 + \frac{1}{2} I_w \dot{\theta}^2 + \frac{1}{2} I_{rod} \dot{\theta}^2
$$

## Potential Energy

$$
V = m_L g (Rcos\theta - r\sin\theta) \times 2 + m_b g (R+h)cos\theta + m_w g R cos\theta 
$$

## Lagrangian

$$L = m_L \dot{r}^2 + 2m_L R\dot{r}\dot{\theta} + m_L R^2\dot{\theta}^2 + \frac{1}{2} m_b (R+h)^2 \dot{\theta}^2 + \frac{1}{2} m_w R^2 \dot{\theta}^2 + \frac{1}{2} I_b \dot{\theta}^2 + \frac{1}{2} I_w \dot{\theta}^2 + \frac{1}{2} I_{rod} \dot{\theta}^2 \\
- 2m_L g R\cos\theta + 2m_L g r\sin\theta - m_b g (R+h)\cos\theta - m_w g R\cos\theta$$

$$
\frac{d}{dt} \left( \frac{\partial L}{\partial \dot{\theta}} \right) - \frac{\partial L}{\partial \theta} = Q_{\theta}
$$

$$
\frac{d}{dt} \left( \frac{\partial L}{\partial \dot{r}} \right) - \frac{\partial L}{\partial r} = Q_r
$$

## Calculate the equations of motion


$$
\begin{align*}
\frac{\partial L}{\partial \dot{\theta}} = & 2 m_L R \dot{r} + 2 m_L R^2 \dot{\theta} + m_b (R+h)^2 \dot{\theta} + m_w R^2\dot{\theta} + I_b\dot{\theta} + I_w\dot{\theta} + I_{rod} \dot{\theta}\\
\end{align*}
$$

$$
\begin{align*}
\frac{\partial_1L}{\partial_1\theta} =\frac{d}{dt} \left( \frac{\partial L}{\partial \dot{\theta}} \right) = & 2 m_L R \ddot{r} + 2 m_L R^2 \ddot{\theta} + m_b (R+h)^2 \ddot{\theta} + m_w R^2 \ddot{\theta} + I_b\ddot{\theta} + I_w\ddot{\theta} + I_{rod} \ddot{\theta}\\
\end{align*}
$$

$$
\begin{align*}
\frac{\partial_2 L}{\partial_2 \theta} = & 2 m_L g R \sin\theta + 2 m_L g r \cos\theta + m_b g (R+h) \sin\theta + m_w g R \sin\theta \\
\end{align*}
$$

$$
\begin{align*}
\frac{\partial L}{\partial \dot{r}} = & 2 m_L \dot{r} + 2 m_L R \dot{\theta} \\
\end{align*}
$$

$$
\begin{align*}
\frac{\partial_1L}{\partial_1 r}=\frac{d}{dt} \left( \frac{\partial L}{\partial \    
\dot{r}} \right) = & 2 m_L \ddot{r} + 2 m_L R \ddot{\theta} \\
\end{align*}
$$

$$
\begin{align*}
\frac{\partial_2 L}{\partial_2 r} =& 2 m_L g \sin\theta \\
\end{align*}
$$

$$
Q_{r} = F - F_r
$$

$$
Q_{\theta} = (-F+F_r)*R
$$

In matrix form, the equations of motion can be expressed as:    

$$
\begin{bmatrix}2 m_L R^2 + m_b (R+h)^2 + m_w R^2 + I_b + I_w + I_{rod} & 2 m_L R \\ 2 m_L R & 2 m_L \end{bmatrix} \begin{bmatrix}\ddot{\theta} \\ \ddot{r} \end{bmatrix} = \begin{bmatrix} 2 m_L g R \sin\theta + 2 m_L g r \cos\theta + m_b g (R+h) \sin\theta + m_w g R \sin\theta - FR + F_rR \\ 2 m_L g \sin\theta + F -F_r\end{bmatrix}
$$
