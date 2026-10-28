# General turning manifold control

## 1. Equations of motion

Write the five pseudovelocities as

$$
\sigma=[\sigma_1,\sigma_2,\sigma_3,\sigma_r,\sigma_g]^T.
$$

The full nonlinear model has the form

$$
\boxed{M(x)\dot\sigma+C(x,u)=0,\qquad
\dot q=k(x),\qquad
f(x,u)=\begin{bmatrix}-M(x)^{-1}C(x,u)\\k(x)\end{bmatrix},}
$$

where \(q=[\psi,\theta,\phi,r,\gamma,x_G,y_G]^T\),
\(u=[F,M_2]^T\), and the public force convention is \(F=-F_1\).
The kinematic equations are

$$
k(x)=\begin{bmatrix}
\frac{\sigma_3}{\cos\theta}\\
\sigma_1\\
\sigma_2-\sigma_3\tan\theta\\
R\sigma_1+\sigma_r\\
\sigma_g-\sigma_3\tan\theta\\
R\sigma_1\sin\psi\cos\theta+R\sigma_2\cos\psi\\
-R\sigma_1\cos\psi\cos\theta+R\sigma_2\sin\psi
\end{bmatrix}.
$$

For a compact expression of the mass matrix, define
\(J_a=J_{Px}-J_{Pz}+m_ph^2\). Then

$$
M=\begin{bmatrix}
a&0&b&0&0\\
0&c&d&0&e\\
b&d&j&0&0\\
0&0&0&m_r&0\\
0&e&0&0&\ell
\end{bmatrix},
$$

$$
\begin{aligned}
a&=J_{Pz}+J_R+J_{W2}+R^2(m_p+m_w)
 +2Rhm_p\cos\gamma+m_rr^2+J_a\cos^2\gamma,\\
b&=-Rhm_p\sin\gamma-J_a\sin\gamma\cos\gamma,\\
c&=J_{W1}+R^2(m_p+m_r+m_w),\qquad d=-Rm_rr,\\
e&=Rhm_p\cos\gamma,\\
j&=J_{Pz}+J_R+J_{W2}+m_rr^2+J_a\sin^2\gamma,\\
\ell&=J_{Py}+m_ph^2.
\end{aligned}
$$

The residual separates into the unforced dynamics and the input:

$$
C(x,u)=C_0(x)-Du,\qquad
D=\begin{bmatrix}R&0\\0&1\\0&0\\1&0\\0&-1\end{bmatrix}.
$$

Here \(C_0(x)=C(x,0)\) contains the velocity, gravity, and damping
terms; its component expressions are implemented in
`MATLAB/Full/model/model_residual.m`. The mass matrix and kinematics above
follow `mass_matrix.m` and `model_kinematics.m` in the same directory.

## 2. Steady-turning manifold

The state and input conventions are

$$
x=[\sigma_1,\sigma_2,\sigma_3,\sigma_r,\sigma_g,\psi,\theta,\phi,r,\gamma,x_G,y_G]^T,
\qquad u=[F,M_2]^T,
$$

$$
\sigma_1=\dot\theta,\quad
\sigma_2=\dot\phi+\dot\psi\sin\theta,\quad
\sigma_3=\dot\psi\cos\theta,\quad
\sigma_r=\dot r-R\dot\theta,\quad
\sigma_g=\dot\gamma+\dot\psi\sin\theta.
$$

For constant lean \(\theta\), yaw rate \(\Omega=\dot\psi\ne0\), and wheel-phase
rate \(\nu=\dot\phi\), the controller uses the \(\gamma=0\) branch:

$$
\boxed{
x_e=[0,\nu+\Omega\sin\theta,\Omega\cos\theta,0,\Omega\sin\theta,
\psi_e,\theta,\phi_e,r_e,0,x_{Ge},y_{Ge}]^T.
}
$$

The rolling-speed parameter is \(v=R\nu\). The steady inputs are

$$
\boxed{
F_e=m_r\left[R(\nu+\Omega\sin\theta)\Omega\cos\theta
+g\sin\theta-r_e\Omega^2\cos^2\theta\right],
\qquad M_{2e}=B_P\nu.
}
$$

Solve the first full-model residual \(C_1(x_e,u_e)=0\) for \(r_e\), then verify

$$
C(x_e,u_e)=0,\qquad
f(x_e,u_e)=\begin{bmatrix}0_{5\times1}\\k(x_e)\end{bmatrix}.
$$

On this branch the five equilibrium residual equations reduce to

$$
\begin{aligned}
0=C_{1e}={}&-RF_e-Rg(m_p+m_w)\sin\theta
-ghm_p\sin\theta+gm_rr_e\cos\theta\\
&+\sigma_{2e}\sigma_{3e}
 (-J_{W1}-R^2(m_p+m_w)-Rhm_p-Rm_rr_e\tan\theta)\\
&+\sigma_{3e}^2
 (Rhm_p+m_rr_e^2+J_a+J_{Pz}+J_R+J_{W2})\tan\theta\\
&+\sigma_{3e}\sigma_{ge}
 (J_{Px}-J_{Py}-J_{Pz}-2Rhm_p-2J_a),\\
0=C_{2e}={}&B_P(\sigma_{2e}-\sigma_{ge})-M_{2e},\\
0=C_{3e}={}&0,\\
0=C_{4e}={}&-F_e+Rm_r\sigma_{2e}\sigma_{3e}
 +gm_r\sin\theta-m_rr_e\sigma_{3e}^2,\\
0=C_{5e}={}&-C_{2e},
\end{aligned}
$$

where \(\sigma_{2e}=\nu+\Omega\sin\theta\),
\(\sigma_{3e}=\Omega\cos\theta\), and
\(\sigma_{ge}=\Omega\sin\theta\).
Thus \(C_{4e}=0\) determines \(F_e\), \(C_{2e}=0\) gives
\(M_{2e}=B_P\nu\), and \(C_{1e}=0\) determines \(r_e\).


### 2.1 Inverting the rod travel limit to obtain the initial lean

Use \(v=1.2\ \mathrm{m/s}\), \(r(0)=0\), \(\dot\psi(0)=0\), zero other
initial rates apart from forward rolling, and \(\theta_{\mathrm{eq}}=0\).
The calculation proceeds directly from
\(r_{\mathrm{eq}}\to\Omega=\dot\psi_{\mathrm{eq}}\to\theta_0\), without
fitting a proportional relation between initial lean and yaw rate.

**1. Rod limit to equilibrium yaw rate.** Assuming the peak-to-equilibrium
rod displacement ratio remains 4.72, the positive limiting equilibrium
position is

$$
r_* = \frac{0.075}{4.72}=0.01588983051\ \mathrm m.
$$

The exact upright-turning equilibrium equation at this speed is

$$
r_* = \frac{2.80321284612\,\Omega_*}{22.563+0.5819\Omega_*^2},
$$

or \(0.5819r_*\Omega_*^2-2.80321284612\Omega_*+22.563r_*=0\).
Selecting the small-rate root connected to straight rolling gives

$$
\boxed{\Omega_*=0.12795090509\ \mathrm{rad/s}.}
$$

**2. Equilibrium yaw rate to initial lean using the rolling invariant.**
At the upright straight-rolling linearization point, the model gives

$$
\dot\sigma_3=a\dot\theta,\qquad
C=\sigma_3-a\theta=\mathrm{constant},\qquad
 a=-\frac{J_{W1}(v/R)}{J_{Pz}+J_R+J_{W2}}.
$$

For the current parameters and \(v=1.2\ \mathrm{m/s}\),

$$
a=-3.96472370555\ \mathrm{s^{-1}}.
$$

Initially \(\dot\psi(0)=0\), so \(\sigma_3(0)=0\) and
\(C=-a\theta_0\). At the upright target, \(\theta_{\mathrm{eq}}=0\)
and \(\sigma_{3,\mathrm{eq}}=\dot\psi_{\mathrm{eq}}=\Omega\). Consequently,

$$
\boxed{\Omega=-a\theta_0,\qquad
\theta_0=\frac{\Omega}{3.96472370555}\quad[\mathrm{rad}].}
$$

Substituting the yaw-rate limit obtained from \(r_*\) in step 1,

$$
\theta_{0,*}=\frac{0.12795090509}{3.96472370555}
=0.03227233840\ \mathrm{rad}=1.84906879^\circ.
$$

Thus the rod-limit inversion using the straight-rolling invariant is

$$
\boxed{|r_{\mathrm{eq}}|<0.01588983\ \mathrm m
\ \Longrightarrow\ |\Omega|<0.12795091\ \mathrm{rad/s}
\ \Longrightarrow\ |\theta_0|<1.84907^\circ.}
$$

This is an invariant of the fixed-speed straight-rolling **linearized**
model, not an exact nonlinear conservation law. Together with the assumed
4.72 peak-to-equilibrium ratio, it gives an estimated initial-angle limit.
The earlier 1.87190-degree value used a different local approximation:
relinearization at the nonzero turning target and its neutral-vector
matching condition. It is not the result of this straight-rolling invariant.
See also [the original invariant derivation](../FullModelPolePlacement_withDamping.md#lean-offset-proof).

## 3. Linearized state and input matrices

Use the reduced state

$$
y=[\sigma_1,\sigma_2,\sigma_3,\sigma_r,\sigma_g,\psi,\theta,r,\gamma]^T,
\qquad I=(1,2,3,4,5,6,7,9,10).
$$

Its dynamics \(\dot y=f_I(y,u)\) are independent of the omitted coordinates
\(\phi,x_G,y_G\). Let \(y_r(t)\) be the steady-turning reference, with
\(\psi_r(t)=\psi_e+\Omega t\), and define
\(e=y-y_r(t)\) and \(\delta u=u-u_e\). Locally,

$$
\boxed{\dot e=Ae+B\delta u+
O\!\left((\|e\|+\|\delta u\|)^2\right),}
$$

$$
A=\left.\frac{\partial f_I}{\partial y}\right|_e\in\mathbb R^{9\times9},
\qquad
B=\left.\frac{\partial f_I}{\partial u}\right|_e\in\mathbb R^{9\times2}.
$$

The matrices are constant along this reference because the reduced vector
field is independent of heading. The upper five rows describe acceleration;
the lower four rows describe kinematics. Since \(C_e=0\), differentiating
\(-M^{-1}C\) eliminates the term containing the derivative of \(M^{-1}\).
Consequently,

$$
\boxed{
A=\begin{bmatrix}-M_e^{-1}C_y|_e\\H_e\end{bmatrix},
\qquad
B=\begin{bmatrix}M_e^{-1}D\\0_{4\times2}\end{bmatrix},
}
$$

where \(M_e=M(x_e)\), \(C_y|_e\in\mathbb R^{5\times9}\) is evaluated
at fixed input \(u_e\), and

$$
C_y|_e=\left[
 C_{\sigma_1}\;C_{\sigma_2}\;C_{\sigma_3}\;C_{\sigma_r}\;
 C_{\sigma_g}\;0_{5\times1}\;C_\theta\;C_r\;C_\gamma
\right]_e.
$$

In the stated state order, the explicit kinematic block is

$$
H_e=\begin{bmatrix}
0&0&\frac{1}{\cos\theta_e}&0&0&0&
\frac{\sigma_{3e}\tan\theta_e}{\cos\theta_e}&0&0\\
1&0&0&0&0&0&0&0&0\\
R&0&0&1&0&0&0&0&0\\
0&0&-\tan\theta_e&0&1&0&
-\frac{\sigma_{3e}}{\cos^2\theta_e}&0&0
\end{bmatrix}.
$$

Thus \(A\) maps state errors to changes in their rates, and \(B\) maps
force and torque perturbations to changes in those rates. These are the
nine-state open-loop matrices, before projection or feedback. For fixed
\(T\), the projected matrices used by LQR are \(A_c=T^TAT\) and
\(B_c=T^TB\); the unsaturated full error feedback gives \(A-BK\).

## 4. Local target selection

Use the nine relative coordinates

$$
y=x_I=[\sigma_1,\sigma_2,\sigma_3,\sigma_r,\sigma_g,\psi,\theta,r,\gamma]^T,
\qquad I=(1,2,3,4,5,6,7,9,10).
$$

At each candidate equilibrium, form

$$
A=\left.\frac{\partial f_I}{\partial y}\right|_e,\qquad
B=\left.\frac{\partial f_I}{\partial u}\right|_e,\qquad
\mathscr C=[B\;AB\;\cdots\;A^8B]\in\mathbb R^{9\times18}.
$$

On the accepted branch, \(\mathscr C\) has numerical rank eight. Let \(w\)
span its one-dimensional left nullspace:

$$
w^T\mathscr C=0,\qquad \|w\|=1.
$$

For prescribed \(\theta\) and \(\nu\), choose the yaw rate from the initial state:

$$
\boxed{g(\Omega)=w(\Omega)^T[y_0-y_e(\theta,\Omega,\nu)]=0.}
$$

Keep the sign of \(w\) consistent during root finding. This places the initial
error in the local controllable subspace; it does not establish an exact
nonlinear invariant or guarantee nonlinear reachability.

## 5. SVD basis and transverse LQR

The singular value decomposition is

$$
\mathscr C=U\Sigma V^T,\qquad
\mathscr C\mathscr C^Tu_i=s_i^2u_i,\qquad
s_1\ge\cdots\ge s_9\ge0.
$$

Thus the columns \(u_i\) of \(U\) are orthonormal eigenvectors of
\(\mathscr C\mathscr C^T\), and the singular values \(s_i\) are square roots
of its eigenvalues. They identify the output directions and stretching
factors of \(\mathscr C\). Signs are arbitrary; repeated singular values
allow different bases within the same subspace.

For rank eight, take

$$
\boxed{T=[u_1,\ldots,u_8]\in\mathbb R^{9\times8},\qquad w=u_9,}
$$

$$
T^TT=I_8,\qquad T^Tw=0,\qquad e=TT^Te+ww^Te.
$$

MATLAB uses `[U,S,~]=svd(ctrb(A,B))`, counts \(s_i>10^{-9}s_1\), and
sets `T=U(:,1:8)`. Numerical rank depends on scaling and tolerance;
the basis is recomputed at each new target.

For \(z=T^Te\), project the model and cost:

$$
A_c=T^TAT,\qquad B_c=T^TB,\qquad Q_c=T^TQT,
$$

$$
Q=\operatorname{diag}(2,1,1,2,1,0.5,20,20,5),\qquad
R_u=\operatorname{diag}(0.1,0.1),
$$

$$
K_c=\operatorname{LQR}(A_c,B_c,Q_c,R_u),\qquad K=K_cT^T.
$$

The relative reference keeps the equilibrium shape and rates, with
\(\psi_r(t)=\psi_e+\Omega t\). Set \(e=y-y_r\) and wrap its heading component:

$$
e_6\leftarrow\operatorname{atan2}(\sin e_6,\cos e_6).
$$

Apply the saturated feedback

$$
\boxed{
u(t)=\operatorname{sat}_{[-u_{\max},u_{\max}]}(u_e-Ke(t)),
\qquad
u_{\max}=\begin{bmatrix}30\,\mathrm N\\15\,\mathrm{N\,m}\end{bmatrix}.
}
$$

The target and gain remain fixed during each stage. Wheel phase and global
position are excluded from feedback. Local decay rates follow the eigenvalues
of \(A_c-B_cK_c\), not the singular values of \(\mathscr C\).


