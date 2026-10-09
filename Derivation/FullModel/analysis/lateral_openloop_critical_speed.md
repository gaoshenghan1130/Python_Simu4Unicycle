# Lateral-open-loop critical speed: analytical condition and validation

## Result

For the `mate_current_mass` parameters and the implemented gamma-PD loop,
the upper stability boundary is

\[
\boxed{v_{\mathrm{crit,high}}\approx 1.723\ \mathrm{m/s}.}
\]

This is the high-speed boundary of the current MATLAB model, not one of the
paper's tabulated critical speeds. The plotted run at \(1.758\,\mathrm{m/s}\)
is approximately 2% above it.

## Analytical linearization

Use \(\Omega=v/R\) and linearize about upright straight rolling:

\[
\theta=\gamma=r=\sigma_1=\sigma_3=\sigma_r=\sigma_\gamma=0,
\qquad \sigma_2=\Omega,
\qquad F=M_2=0.
\]

The internal dynamic state is

\[
z=\begin{bmatrix}\sigma_1&\sigma_2&\sigma_3&\sigma_r&\sigma_\gamma&
\theta&r&\gamma\end{bmatrix}^{T}.
\]

Let

\[
\begin{aligned}
I_\ell&=J_{Px}+J_R+J_{W2}+R^2(m_p+m_w)+2Rh m_p+h^2m_p,\\
I_w&=J_{W1}+R^2(m_p+m_r+m_w),\\
c&=J_{Pz}+J_R+J_{W2},\\
I_\gamma&=J_{Py}+h^2m_p,\\
b&=Rh m_p,\\
\kappa&=J_{W1}+R^2(m_p+m_w)+Rh m_p,\\
G_\ell&=g\big[R(m_p+m_w)+hm_p\big],\qquad G_\gamma=ghm_p.
\end{aligned}
\]

The linearized pseudovelocity mass matrix is

\[
M_0=\begin{bmatrix}
I_\ell&0&0&0&0\\
0&I_w&0&0&b\\
0&0&c&0&0\\
0&0&0&m_r&0\\
0&b&0&0&I_\gamma
\end{bmatrix}.
\]

The first-order residual matrices, with columns ordered as \(\sigma\) and
\((\theta,r,\gamma)\), are

\[
L_\sigma(v)=\begin{bmatrix}
B_RR^2&0&-\kappa\Omega&B_RR&0\\
0&B_P&0&0&-B_P\\
J_{W1}\Omega&0&0&0&0\\
B_RR&0&Rm_r\Omega&B_R&0\\
0&-B_P&0&0&B_P
\end{bmatrix},\qquad
L_q=\begin{bmatrix}
-G_\ell&gm_r&0\\
0&0&0\\
0&0&0\\
gm_r&0&0\\
0&0&-G_\gamma
\end{bmatrix}.
\]

The equation convention is
\(M_0\dot\sigma+L_\sigma\sigma+L_q q=Q_{M_2}M_2\), where
\(Q_{M_2}=[0,1,0,0,-1]^T\). The implemented actuator law is

\[
F=0,\qquad M_2=K_p\gamma+K_d\dot\gamma,
\qquad K_p=3,\quad K_d=0.8.
\]

Since \(\dot\gamma=\sigma_\gamma-\sigma_3\tan\theta\), its upright
linearization is \(\delta\dot\gamma=\delta\sigma_\gamma\). Define

\[
H=\begin{bmatrix}
1&0&0&0&0\\
R&0&0&1&0\\
0&0&0&0&1
\end{bmatrix},\quad
e_5=\begin{bmatrix}0&0&0&0&1\end{bmatrix}^{T},\quad
e_3=\begin{bmatrix}0&0&1\end{bmatrix}^{T}.
\]

Then the eight-dimensional internal closed-loop matrix is

\[
A_d(v)=\begin{bmatrix}
-M_0^{-1}L_\sigma(v)+M_0^{-1}Q_{M_2}K_d e_5^T &
-M_0^{-1}L_q+M_0^{-1}Q_{M_2}K_p e_3^T\\
H&0_{3\times3}
\end{bmatrix}.
\]

The remaining four full-model states are phase/position coordinates and
contribute zero eigenvalues. Thus the dynamic characteristic equation is

\[
P_8(\lambda;v)=\det\!\left(\lambda I_8-A_d(v)\right)=0.
\]

The linear stability margin is

\[
\alpha(v)=\max_{\lambda\in\operatorname{eig}(A_d(v))}
                 \Re\lambda.
\]

The upper critical speed is the upper boundary where \(\alpha(v)\) changes
sign. Equivalently, it is the upper-speed imaginary-axis crossing of
\(P_8(\lambda;v)=\det(\lambda I_8-A_d(v))\):

\[
\Re P_8(i\omega;v)=0,\qquad
\Im P_8(i\omega;v)=0,\qquad \omega>0.
\]

The determinant equation is the analytical criterion; its root and the
eigenvalues are evaluated numerically for the stated parameter set.

## Numerical result and validation

The parameters substituted in \(M_0,L_\sigma,L_q\) are those returned by
`simulation_preset('mate_current_mass')`: \(R=0.253\,\mathrm m\),
\(m_w=2.436\,\mathrm{kg}\), \(m_r=1\,\mathrm{kg}\),
\(m_p=2.799\,\mathrm{kg}\), \(h=0.025\,\mathrm m\),
\(B_R=B_P=0\), and the inertias in `model_parameters.m`. Solving the upper
imaginary-axis crossing gives \(v\approx1.723\,\mathrm{m/s}\). The value
returned by the MATLAB check is `result.critical_speed_mps`; the refined
crossing interval is `result.critical_bracket_mps`.

Validation uses both sides of the crossing and the nonlinear simulation:

- Immediately below the refined crossing, the dynamic spectrum has a mode
  with positive real part; above it, there is no exponentially growing
  dynamic mode (zero/imaginary-axis phase modes are not counted as decay).
- At \(1.758\,\mathrm{m/s}\), with \(\theta_0=0.1^\circ\), \(F=0\), and
  gamma-PD active, the 30 s nonlinear trajectory remains bounded and
  oscillatory. It does **not** converge to \(\theta=0\), which is consistent
  with neutral rather than asymptotic stability.

The result is specific to this parameter set and gamma-PD gains. It should
not be identified with the paper's critical speeds, which come from its own
linearized model and parameter values.
