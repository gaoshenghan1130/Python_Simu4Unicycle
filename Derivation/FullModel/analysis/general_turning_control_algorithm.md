# General turning manifold control algorithm

This note records the manifold and feedback law used by
`verify_turning_manifold_approach.m`. The MATLAB state and input conventions
are

$$
x=\begin{bmatrix}
\sigma_1&\sigma_2&\sigma_3&\sigma_r&\sigma_g&
\psi&\theta&\phi&r&\gamma&x_G&y_G
\end{bmatrix}^{T},\qquad
u=\begin{bmatrix}F&M_2\end{bmatrix}^{T}.
$$

Here \(\sigma_1=\dot\theta\),
\(\sigma_2=\dot\phi+\dot\psi\sin\theta\),
\(\sigma_3=\dot\psi\cos\theta\),
\(\sigma_r=\dot r-R\dot\theta\), and
\(\sigma_g=\dot\gamma+\dot\psi\sin\theta\).

## 1. General turning relative-equilibrium manifold

Let \(\theta\) be the constant lean angle, \(\Omega=\dot\psi\ne0\) the
constant yaw rate, and \(\nu=\dot\phi\) the constant wheel-phase rate. The
simulation sets the rolling-speed parameter to \(v=R\nu\). A steady-turning
relative equilibrium satisfies

$$
\dot\theta=\dot r=\dot\gamma=\dot\sigma=0,
$$

and therefore

$$
\boxed{
\sigma_e(\theta,\Omega,\nu)=
\begin{bmatrix}
0\\
\nu+\Omega\sin\theta\\
\Omega\cos\theta\\
0\\
\Omega\sin\theta
\end{bmatrix}.}
$$

For the \(\gamma=0\) branch used by the controller, parameterize the full
state by

$$
x_e=
\begin{bmatrix}
0&\nu+\Omega\sin\theta&\Omega\cos\theta&0&\Omega\sin\theta&
\psi_e&\theta&\phi_e&r_e&0&x_{Ge}&y_{Ge}
\end{bmatrix}^{T}.
$$

The phase coordinates \(\psi_e,\phi_e,x_{Ge},y_{Ge}\) locate the orbit; they
do not change its shape. The feedforward input is

$$
\boxed{
F_e=m_r\!\left[R(\nu+\Omega\sin\theta)\Omega\cos\theta
+g\sin\theta-r_e\Omega^2\cos^2\theta\right],\qquad
M_{2e}=B_P\nu.}
$$

The value \(r_e\) is determined from the full model compatibility equation.
In the implementation it is the root of the first residual component,
\(C_1(x_e,u_e)=0\), with \((F_e,M_{2e})\) as above. The resulting point is
accepted only if

$$
\boxed{C(x_e,u_e)=0,\qquad
f(x_e,u_e)=\begin{bmatrix}0_{5\times1}\\k(x_e)\end{bmatrix},}
$$

where \(C\) is `model_residual`, \(f\) is the full state derivative, and
\(k\) contains the nonzero phase and position rates. Thus the manifold is a
family of relative equilibria: its shape is constant while the phase
coordinates move.

## 2. Selecting the reachable member of the manifold

The local controller uses the 9 relative coordinates

$$
y=x_I,\qquad I=(1,2,3,4,5,6,7,9,10),
$$

or, explicitly,

$$
y=\begin{bmatrix}
\sigma_1&\sigma_2&\sigma_3&\sigma_r&\sigma_g&\psi&\theta&r&\gamma
\end{bmatrix}^{T}.
$$

For each candidate \(\Omega\), form the relative linearization

$$
A=\left.\frac{\partial f_I}{\partial y}\right|_{(x_e,u_e)},\qquad
B=\left.\frac{\partial f_I}{\partial u}\right|_{(x_e,u_e)},\qquad
\mathscr C=[B\;AB\;\cdots\;A^8B].
$$

Its rank is 8 (out of 9). Since \(\mathscr C\in\mathbb R^{9\times18}\),
rank-nullity gives a one-dimensional left nullspace:

$$
\dim\ker(\mathscr C^T)=9-\operatorname{rank}(\mathscr C)=1.
$$

Thus there is one uncontrollable direction, unique up to sign. Let
\(w(\Omega)\) be its unit vector, chosen so that

$$
\boxed{w(\Omega)^T\mathscr C(\Omega)=0,\qquad
\|w(\Omega)\|=1.}
$$

The sign is fixed consistently because \(w\) and \(-w\) describe the same
direction, but a sign flip would make the scalar root function jump. The
initial condition selects the yaw rate by solving

$$
\boxed{g(\Omega)=w(\Omega)^T
\left[y_0-y_e(\theta,\Omega,\nu)\right]=0.}
$$

The scalar \(g(\Omega)\) is the signed component of the initial error along
the unit vector \(w(\Omega)\). Therefore \(g(\Omega)=0\) means that this
error has no component in the uncontrollable direction. This zero-mode
condition chooses the member of the manifold compatible with the initial
state; the remaining error lies in the controllable subspace.

## 3. Transverse LQR and nonlinear control law

Take an SVD of \(\mathscr C\), and let \(T\in\mathbb R^{9\times8}\) contain
an orthonormal basis for its controllable subspace. Project the linear model
and cost into this subspace:

$$
A_c=T^TAT,\qquad B_c=T^TB,\qquad Q_c=T^TQT,
$$

$$
Q=\operatorname{diag}(2,1,1,2,1,0.5,20,20,5),\qquad
R_u=\operatorname{diag}(0.1,0.1).
$$

The continuous-time LQR gain is

$$
K_c=\operatorname{LQR}(A_c,B_c,Q_c,R_u),\qquad K=K_cT^T.
$$

The moving reference follows the selected orbit. Its heading and wheel phase
are \(\psi_r(t)=\psi_e+\Omega t\) and \(\phi_r(t)=\phi_e+\nu t\); the wheel
center follows

$$
x_{G,r}(t)=x_{Ge}+\frac{v}{\Omega}
\left[\sin(\psi_e+\Omega t)-\sin\psi_e\right],
$$

$$
y_{G,r}(t)=y_{Ge}-\frac{v}{\Omega}
\left[\cos(\psi_e+\Omega t)-\cos\psi_e\right].
$$

With \(e(t)=y(t)-y_r(t)\), wrap the heading component into \((-\pi,\pi]\):

$$
e_6\leftarrow\operatorname{atan2}(\sin e_6,\cos e_6).
$$

The implemented saturated nonlinear feedback law is

$$
\boxed{
u(t)=\operatorname{sat}_{[-u_{\max},u_{\max}]}
\left(u_e-Ke(t)\right),\qquad
u_{\max}=\begin{bmatrix}30\;\mathrm{N}\\15\;\mathrm{N\,m}\end{bmatrix}.}
$$

The feedforward term maintains the target turning motion; feedback reduces
the controllable displacement from its moving orbit.

## 4. Second stage: transfer from general turning to upright turning

The two-stage entry point first settles on the general-turning branch at
\(\theta_1=0.1^\circ\). It then passes that terminal state directly into a
new controller-design and simulation call:

$$
x_0^{(2)}=x^{(1)}(T_1),\qquad \theta_2^*=0.
$$

The second call repeats the target-orbit selection for \(\theta_2^*=0\). It
searches for a new yaw rate \(\Omega_2\) satisfying

$$
w(\Omega_2)^T\left[y_0^{(2)}-y_e(0,\Omega_2,\nu)\right]=0,
$$

then recomputes the upright relative equilibrium \((x_e^{(2)},u_e^{(2)})\),
its local matrices \((A_2,B_2)\), the controllable subspace, and the LQR gain
\(K_2\). In particular, the new reference has

$$
\theta_r^{(2)}(t)=0,\qquad
e_{\theta}(t)=\theta(t)-\theta_r^{(2)}(t)=\theta(t).
$$

The second-stage input is therefore

$$
u_2(t)=\operatorname{sat}\!\left(
u_e^{(2)}-K_2 e_2(t)
\right),
$$

where \(e_2\) contains the lean error together with the other relative-state
errors. The lean is brought back by this coupled feedback around the upright
turning orbit; the code does not overwrite \(\theta\) or command an
instantaneous reset. The feedforward input supports the new turning motion,
while LQR feedback drives the initial lean and associated transverse errors
toward that orbit. The yaw rate is reselected because the neutral direction
and target equilibrium change with the target lean.

## 5. Manifold convergence criteria

The simulation declares convergence only if the lean limit is not hit and
the final relative rate, tail-window ranges of \(\theta\), \(r\), and yaw
rate, and lean-target error all fall below their specified tolerances. The
absolute heading and position continue to evolve along the orbit and are not
required to converge to constants.
