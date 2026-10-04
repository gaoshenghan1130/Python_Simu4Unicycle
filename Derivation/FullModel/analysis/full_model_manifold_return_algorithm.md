# Full-model equilibrium states

$$
x=
\begin{bmatrix}
\sigma_1&\sigma_2&\sigma_3&\sigma_r&\sigma_g&
\psi&\theta&\phi&r&\gamma&x_G&y_G
\end{bmatrix}^{T},
\qquad
u=\begin{bmatrix}F&M_2\end{bmatrix}^{T}.
$$

$$
\Omega=\dot\psi,\qquad V=\dot\phi .
$$

$$
\boxed{
\sigma_1=0,\quad
\sigma_r=0,\quad
\sigma_3=\Omega\cos\theta,\quad
\sigma_g=\Omega\sin\theta,\quad
\sigma_2=V+\Omega\sin\theta .
}
$$

## Stationary equilibrium

$$
\boxed{
\sigma=0,\qquad
\theta=0,\qquad
r=0,\qquad
\gamma=0,\qquad
F=0,\qquad
M_2=0 .
}
$$

$$
\boxed{
\begin{aligned}
F_e&=m_r g\sin\theta,\\
M_{2e}&=0,\\
r_e&=
\frac{R(m_r+m_p+m_w)+h m_p\cos\gamma}{m_r}
\tan\theta .
\end{aligned}}
$$

## Straight rolling

$$
\boxed{
\begin{aligned}
\Omega&=0,\\
\sigma&=\begin{bmatrix}0&V&0&0&0\end{bmatrix}^{T},\\
\theta&=0,\qquad r=0,\qquad \gamma=0,\\
\dot\psi&=0,\qquad \dot\phi=V,\\
\dot x_G&=RV,\qquad \dot y_G=0,\\
F_e&=0,\qquad M_{2e}=B_PV .
\end{aligned}}
$$

$$
B_P=0\quad\Longrightarrow\quad M_{2e}=0 .
$$

## General turning-rolling

$$
\boxed{
\begin{aligned}
\Omega&\neq0,\qquad V\neq0,\\
\sigma&=
\begin{bmatrix}
0\\
V+\Omega\sin\theta\\
\Omega\cos\theta\\
0\\
\Omega\sin\theta
\end{bmatrix},\\
\dot\psi&=\Omega,\qquad \dot\phi=V,\\
r&=r_e(\theta,\Omega,V,\gamma),\\
\gamma&=\gamma_e(\theta,\Omega,V).
\end{aligned}}
$$

$$
\boxed{
\begin{aligned}
F_e={}&m_r\left[
R(V+\Omega\sin\theta)\Omega\cos\theta+g\sin\theta
-r\Omega^2\cos^2\theta
\right],\\
M_{2e}={}&-b\sin\gamma\,\Omega^2+B_PV .
\end{aligned}}
$$

$$
\boxed{
\mathcal C_i(\theta,\Omega,V,r,\gamma)=0,
\qquad i=1,2,3
}
$$

$$
\mathcal E_{\mathrm{turn}}
=
\left\{
\left(x_e(\theta,\Omega,V),u_e(\theta,\Omega,V)\right):
\mathcal C_i=0
\right\}.
$$

## Nontilted turning

$$
\boxed{
\begin{aligned}
\theta&=0,\qquad \gamma=0,\qquad \Omega\neq0,\qquad V\neq0,\\
\sigma&=\begin{bmatrix}0&V&\Omega&0&0\end{bmatrix}^{T},\\
\mathcal D&=J_{W1}+R^2(m_p+m_w)+R h m_p,\\
r_e&=\frac{V\Omega(R^2m_r+\mathcal D)}{m_r(g+R\Omega^2)},\\
F_e&=m_r(RV\Omega-r_e\Omega^2),\\
M_{2e}&=B_PV.
\end{aligned}
}
$$

$$
F_e=0
\quad\Longrightarrow\quad
\Omega^2=\frac{R m_r g}{\mathcal D},
\qquad r_e=\frac{RV}{\Omega}.
$$

$$
\begin{aligned}
V&=\frac{2.375}{R},\qquad m_r=2.3\ {\rm kg},\\
\Omega&=0.0273641\ {\rm rad/s}
\quad\Longrightarrow\quad
r_e=0.00672845\ {\rm m},\qquad
F_e=0.149465\ {\rm N},\qquad M_{2e}=0.
\end{aligned}
$$

## Spinning

$$
\boxed{
\begin{aligned}
V&=0,\qquad \Omega\neq0,\\
\sigma&=
\begin{bmatrix}
0\\
\Omega\sin\theta\\
\Omega\cos\theta\\
0\\
\Omega\sin\theta
\end{bmatrix},\\
\dot\phi&=0,\qquad \dot\psi=\Omega .
\end{aligned}}
$$

$$
\boxed{
\theta=0,\qquad r=0,\qquad \gamma=0,\qquad V=0
}
$$

$$
\boxed{
\theta\neq0,\qquad r\neq0,\qquad V=0
}
\qquad
\text{(tilted spinning)}.
$$
