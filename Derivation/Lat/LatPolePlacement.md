# Latitude Model Pole Placement analysis

The system moment of inertia $J_0$ is defined as follows:
$$J_0 = 2m_L R^2 + m_b (R+h)^2 + m_w R^2 + I_b + I_w + I_{rod}$$

Gravity coupling coefficients:
* $G_{\theta 1} = 2m_g R + m_g(R+h) + m_g R$
* $G_{r1} = 2m_g r$
* $G_{\theta 2} = 2m_g$

The system dynamics can be expressed in the form $N \ddot{q} = G q + F$, where $q = [\theta, r]^T$:

$$
N = \begin{bmatrix} 
J_0 & 2m_L R^2 \\ 
2m_L R^2 & 2m_L 
\end{bmatrix}
$$

$$
\begin{bmatrix} J_0 & 2m_L R^2 \\ 2m_L R^2 & 2m_L \end{bmatrix} \begin{bmatrix} \ddot{\theta} \\ \ddot{r} \end{bmatrix} = \begin{bmatrix} G_{\theta 1} & G_{r 1} \\ G_{\theta 2} & 0 \end{bmatrix} \begin{bmatrix} \theta \\ r \end{bmatrix} + \begin{bmatrix} 0 \\ F \end{bmatrix}
$$

Define the state vector $z = [\theta, r, \dot{\theta}, \dot{r}]^T$ and input $u = F$.

$$
\dot{z} = A z + B u
$$

Where:
$$
A = N^{-1} \begin{bmatrix} 0 & 0 & 1 & 0 \\ 0 & 0 & 0 & 1 \\ G_{\theta 1} & G_{r 1} & 0 & 0 \\ G_{\theta 2} & 0 & 0 & 0 \end{bmatrix}, \quad B = N^{-1} \begin{bmatrix} 0 \\ 0 \\ 0 \\ 1 \end{bmatrix}
$$

Through closed-loop feedback control $u = -Kz$, we place the closed-loop poles to satisfy the desired characteristic equation:
$$\det(\lambda I - (A - BK)) = 0$$

The continuing MATLAB code is in `MATLAB/analysis/PolePlacement.m`.

The result for the eigenvalues are:

$$
\lambda^4 + 1.114 k_4 \lambda^3 - 0.04071 k_3 \lambda^3 + 1.114 k_2 \lambda^2 - 0.04071 k_1 \lambda^2 - 15.66 \lambda^2 + 6.254 k_3 \lambda - 18.2 k_4 \lambda + 6.254 k_1 - 18.2 k_2 - 55.21 = 0
$$

To make it look better, we can rearrange the terms:

$$
\lambda^4 + (1.114 k_4 - 0.04071 k_3) \lambda^3 + (1.114 k_2 - 0.04071 k_1 - 15.66) \lambda^2 + (6.254 k_3 - 18.2 k_4) \lambda + (6.254 k_1 - 18.2 k_2 - 55.21) = 0
$$