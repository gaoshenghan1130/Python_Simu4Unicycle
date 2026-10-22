# General turning manifold control

## 1. Steady-turning manifold

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

## 2. Local target selection

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

## 3. SVD basis and transverse LQR

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


