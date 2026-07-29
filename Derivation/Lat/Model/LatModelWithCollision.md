# Latitudinal Model with Collision

Now that we have the latitudinal dynamics:

$$
\begin{bmatrix} 
2m_L(R^2 + r^2) + m_w R^2 + m_p (R + h)^2 + (I_w + I_b  + I_{rod})  & -2m_LR  \\ 
-2m_LR & 2m_L 
\end{bmatrix} 
\begin{bmatrix} 
\ddot{\theta} \\ 
\ddot{r} 
\end{bmatrix} 
= \begin{bmatrix} 
-4m_L r \dot{r} \dot{\theta} + \left( 2m_L R + m_w R + m_p (R + h) \right) g \sin\theta - 2m_L g r \cos\theta \\ 
F + 2m_L r \dot{\theta}^2 - 2m_L g \sin\theta 
\end{bmatrix}
$$

If we want to consider the collision of the pendulum with the wheel we can calulate the behavior at the moment when $r$ exceeds the limit $r_{max}$

From real world experiment we can see that the pendulum would stop. By conservation of angular momentum we can calculate the new angular velocity of the pendulum after the collision.

Say at a moment before the collision, we have state vector $x = [\theta, \dot{\theta}, r, \dot{r}]^T$ and after the collision we have state vector $x' = [\theta', \dot{\theta}', r', \dot{r}']^T$ predicted by the model, then if $r' > r_{max}$, we need to update the state vector to $x'' = [\theta'', \dot{\theta}'', r_{max}, \dot{r}'']^T$:

By continuity of $\theta$ and $r$, we have $\theta'' = \theta'$.

To simplify, define:

$$
A_{max} = 2m_L(R^2 + r_{max}^2) + m_w R^2 + m_p (R + h)^2 + (I_w + I_b  + I_{rod})
$$

Then by conservation of angular momentum, we have:

$$
A_{max} \dot{\theta}'' - 2 m_L R \dot{r}'' = A \dot{\theta}' - 2 m_L R \dot{r}'
$$

Since the rod sticks to the wheel instantly, $\dot{r}'' =0$, we can solve for $\dot{\theta}''$:

$$
\dot{\theta}'' = \frac{A \dot{\theta}' - 2 m_L R \dot{r}'}{A_{max}} = \dot{\theta}' - \frac{2 m_L R \dot{r}'}{A_{max}}
$$

Thus the final state vector after the collision is:

$$
x'' = \begin{bmatrix} \theta' \\ \dot{\theta}' - \frac{2 m_L R \dot{r}'}{A_{max}}  \\ r_{max} \\ 0 \end{bmatrix}
$$

## After collision dynamics

After collision, $r$ sticks to the wheel, for a short period of time, we can assume that $r = r_{max}$ (of course depend on direction, etc.) and $\dot{r} = 0$. Then the dynamics of the system can be simplified to:

$$
\begin{bmatrix} 
2m_L(R^2 + r_{max}^2) + m_w R^2 + m_p (R + h)^2 + (I_w + I_b  + I_{rod})  & 0  \\ 
0 & 2m_L 
\end{bmatrix}
\begin{bmatrix} 
\ddot{\theta} \\ 
\ddot{r} 
\end{bmatrix}
= \begin{bmatrix} 
\left( 2m_L R + m_w R + m_p (R + h) \right) g \sin\theta - 2m_L g r_{max} \cos\theta \\ 
F + 2m_L r_{max} \dot{\theta}^2 - 2m_L g \sin\theta 
\end{bmatrix}
$$

The condition of exiting collision is when $\ddot{r}$ is the opposite direction of $r_{max}$.
