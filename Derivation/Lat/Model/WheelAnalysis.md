# Wheel analysis

Given a wheel with:

$$
\vec{\omega} = \begin{bmatrix} \dot{\phi} \\ \dot{\theta} \\ \dot{\psi} \end{bmatrix}, \quad \vec{L} = \begin{bmatrix} I_d \dot{\phi} \\ I_a \dot{\theta} \\ I_d \dot{\psi} \end{bmatrix}
$$

With Euler's equations:

$$
\vec{\tau} = \frac{d\vec{L}}{dt} + \vec{\Omega} \times \vec{L}
$$

We set $\vec{\Omega} = \dot{\phi}\hat{i} + \dot{\psi}\hat{k}$ so as not to rotate the wheel about its axis of symmetry. Then we have:

$$
\vec{\Omega} \times \vec{L} = \begin{vmatrix} 
\hat{i} & \hat{j} & \hat{k} \\ 
\dot{\phi} & 0 & \dot{\psi} \\ 
I_d \dot{\phi} & I_a \dot{\theta} & I_d \dot{\psi} 
\end{vmatrix} 
= \hat{i}(0 - I_a \dot{\theta} \dot{\psi}) - \hat{j}(I_d \dot{\phi} \dot{\psi} - I_d \dot{\phi} \dot{\psi}) + \hat{k}(\dot{\phi} I_a \dot{\theta} - 0)
$$

And get (on the $x$ axis):

$$
\tau_x = I_d \ddot{\phi} - I_a \dot{\theta} \dot{\psi}
$$

When the wheel is leans to $\phi$, $\tau_x = m g R \sin\phi$:

$$
I_d \ddot{\phi} - I_a \dot{\theta} \dot{\psi} = m g R \sin\phi
$$

Here $I_d\dot{\theta}\dot{\psi}$ represents the virtual force that is preventing the wheel from leaning over.

So the updated model for the system will be:

$$
\begin{bmatrix} 
2m_L(R^2 + r^2) + m_w R^2 + m_p (R + h)^2 + (I_w + I_b  + I_{rod})  & -2m_LR \\ 
-2m_LR & 2m_L 
\end{bmatrix} 
\begin{bmatrix} 
\ddot{\theta} \\ 
\ddot{r} 
\end{bmatrix} 
= \begin{bmatrix} 
-4m_L r \dot{r} \dot{\theta} + \left( 2m_L R + m_w R + m_p (R + h) \right) g \sin\theta - 2m_L g r \cos\theta + I_w\dot{\theta}\dot{\psi} \\ 
F + 2m_L r \dot{\theta}^2 - 2m_L g \sin\theta 
\end{bmatrix}
$$