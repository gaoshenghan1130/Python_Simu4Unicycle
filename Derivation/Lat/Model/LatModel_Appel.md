# Latitude Model Analysis with new pseudo definition

Define the $E_2$ as the axis with the same $j2$ direction with the wheel. 


Center of the wheel:

$$
r_w = R \hat{j_2}
$$

Center of the rod:

$$
r_{rod} = R \hat{j_2} + r \hat{i_2}
$$

Velocity of the wheel:
$$
v_w = \dot{r}_w = - R \dot{\theta} \hat{i_2}
$$

Velocity of the pendulum:
$$
v_p = - (R + h) \dot{\theta} \hat{i_2}
$$


Velocity of the rod:
$$
v_{rod} = \dot{r}_{rod} = - R \dot{\theta} \hat{i_2} + \dot{r} \hat{i_2} + r \dot{\theta} \hat{j_2} = (- R \dot{\theta} + \dot{r}) \hat{i_2} + r \dot{\theta} \hat{j_2}
$$

Define the following pseudo velocity:

$$
u_1 = \dot{\theta} \\
u_2 = \dot{r} - R \dot{\theta} = \sigma
$$

Then we can express the velocity of the wheel and the rod in terms of the pseudo velocity:

$$
v_w = - R u_1 \hat{i_2} \\
v_{rod} = u_2 \hat{i_2} + r u_1 \hat{j_2}
$$

By Appell's method:

$$
S = \frac{1}{2} \sum_{i} m_i \mathbf{a}_i^2
$$

With equation of motion:

$$
\frac{\partial S}{\partial \ddot{q}_r} = Q_r
$$

**Accelerations:**

$$
\mathbf{a}_w = \dot{v}_w = -R \dot{u}_1 \hat{i_2} - R u_1 (u_1 \hat{j_2}) = -R \dot{u}_1 \hat{i_2} - R u_1^2 \hat{j_2}
$$

$$
\mathbf{a}_p = \dot{v}_p = - (R + h) \dot{u}_1 \hat{i_2} - (R + h) u_1 (u_1 \hat{j_2}) = - (R + h) \dot{u}_1 \hat{i_2} - (R + h) u_1^2 \hat{j_2}
$$

$$
\mathbf{a}_{rod} = \dot{v}_{rod} = \dot{u}_2 \hat{i_2} + u_2 (u_1 \hat{j_2}) + \dot{r} u_1 \hat{j_2} + r \dot{u}_1 \hat{j_2} + r u_1 (-u_1 \hat{i_2}) \\
= \dot{u}_2 \hat{i_2} + (u_2 u_1 + \dot{r} u_1 + r \dot{u}_1) \hat{j_2} - r u_1^2 \hat{i_2} \\
= (\dot{u}_2 - r u_1^2 ) \hat{i_2} + (u_2 u_1 + \dot{r} u_1 + r \dot{u}_1) \hat{j_2}
$$

**Get S:**

$$
S = \frac{1}{2} m_w |\mathbf{a}_w|^2 + \frac{1}{2} m_{rod} |\mathbf{a}_{rod}|^2 + \frac{1}{2} m_p |\mathbf{a}_p|^2 + \frac{1}{2}(I_w + I_b  + I_{rod}) \ddot{\theta}^2 \\
= \frac{1}{2} m_w (R^2 \dot{u}_1^2 + R^2 u_1^4) + \frac{1}{2} m_{rod} [(\dot{u}_2 - r u_1^2)^2 + (u_2 u_1 + \dot{r} u_1 + r \dot{u}_1)^2] \\ + \frac{1}{2} m_p ((R + h)^2 \dot{u}_1^2 + (R + h)^2 u_1^4) + \frac{1}{2}(I_w + I_b  + I_{rod}) \dot{u_1}^2
$$

**Equations of motion:**

$$
\frac{\partial S}{\partial \dot{u}_1} = Q_1 \\ 
\frac{\partial S}{\partial \dot{u}_2} = Q_2
$$

$$
\frac{\partial S}{\partial \dot{u}_1} = m_w R^2 \dot{u}_1 + m_{rod}(u_2 u_1 + \dot{r} u_1 + r \dot{u}_1) r + m_p (R + h)^2 \dot{u}_1 + (I_w + I_b  + I_{rod}) \dot{u}_1
$$

As  $\dot{r} = u_2 + R u_1$:

$$
\frac{\partial S}{\partial \dot{u}_1} = (m_w R^2 + m_p (R + h)^2 + m_{rod} r^2 + I_w + I_b  + I_{rod}) \dot{u}_1 + m_{rod} r (2 u_2 u_1 + R u_1^2)
$$


$$
\frac{\partial S}{\partial \dot{u}_2} = m_{rod} (\dot{u}_2 - r u_1^2)
$$

**Virtual work:**

$$
\delta P_F = \Sigma F_i \cdot \delta \dot{r}_i\\
= F \cdot \delta v_{rod} - F \cdot \delta v_w \\
= F \hat{i_2} \cdot (\delta u_2 \hat{i_2} + r \delta u_1 \hat{j_2}) - F \hat{i_2} \cdot (- R \delta u_1 \hat{i_2}) \\
= F \delta u_2 + F R \delta u_1
$$

$$
\delta P_G = G_{rod} \cdot \delta \dot{r}_{rod} + G_w \cdot \delta \dot{r}_w \\
= - m_{rod} g \hat{j_1} \cdot (\delta u_2 \hat{i_2} + r \delta u_1 \hat{j_2}) + (- m_w g) \hat{j_1} \cdot ( -R \delta u_1 \hat{i_2}) + (- m_p g) \hat{j_1} \cdot ( -(R + h) \delta u_1 \hat{i_2}) \\
= - m_{rod} g \sin\theta \delta u_2  - m_{rod} g r \cos\theta \delta u_1 + m_w g R \sin\theta \delta u_1 + m_p g (R + h) \sin\theta \delta u_1 \\
= - m_{rod} g \sin\theta \delta u_2 - (m_{rod} g r \cos\theta - m_w g R \sin\theta - m_p g (R + h) \sin\theta) \delta u_1
$$

**Final equations of motion:**

$$
\frac{\partial S}{\partial \dot{u}_1} = Q_1 = F R - m_{rod} g r \cos\theta + m_w g R \sin\theta + m_p g (R + h) \sin\theta \\
\frac{\partial S}{\partial \dot{u}_2} = Q_2 = F - m_{rod} g \sin\theta
$$

In matrix form:

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