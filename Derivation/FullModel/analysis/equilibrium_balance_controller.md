# Equilibrium-Manifold Feedback Controller

## Control Law

$$
V=\frac{v}{R},\qquad
\Omega=\frac{\sigma_3}{\cos\theta},\qquad
\kappa=\frac{J_{W1}V}{J_{Pz}+J_R+J_{W2}}.
$$

$$
e_r=r-r_{\rm des}(\theta,V,\Omega),\qquad
 e_\theta=\theta-\kappa\sigma_3,\qquad
z=\begin{bmatrix}e_r&\dot e_r&e_\theta&\sigma_1\end{bmatrix}^{T}.
$$

$$
\boxed{F=
F_{\rm ff}-k_1e_r-k_2\dot e_r-k_3e_\theta-k_4\sigma_1,
\qquad K=\begin{bmatrix}k_1&k_2&k_3&k_4\end{bmatrix}.}
$$


## Longitudinal Feedback

$$
\dot\phi=\sigma_2-\sigma_3\tan\theta,\qquad
\dot\gamma=\sigma_g-\sigma_3\tan\theta.
$$

$$
M_2=B_PV-
K_{\rm lon}\begin{bmatrix}\dot\phi-V\\\dot\gamma\\\gamma\end{bmatrix}.
$$

