# Mechanically Locked Centered Rod: Constrained Full Model and Analytical Eigenvalues

## 1. Problem definition

A fixed rod means that its translation relative to the body along the wheel axle is prevented by an ideal mechanical lock:

$$r(t)=0,\qquad \dot r(t)=0,\qquad \ddot r(t)=0.$$

The rod retains its full mass $m_r$ and rotational inertia $J_R$, and it leans together with the wheel and body.
The lean angle $\theta$, heading $\psi$, and longitudinal body angle $\gamma$ remain free.
This constraint differs from the finite-gain `center_rod` PD controller in the full model and from removing the rod.
Default parameters come from `MATLAB/Full/config/model_parameters.m`, with $m_r=2.3$ kg.
This derivation constrains the repository's current full model while retaining its rigid-body and contact assumptions.

The original state and input vectors are

$$x=[\sigma_1,\sigma_2,\sigma_3,\sigma_r,\sigma_g,\psi,\theta,\phi,r,\gamma,x_G,y_G]^T,
\qquad u=[F,M_2]^T.$$

Here \(\sigma_1=\dot\theta\),
\(\sigma_2=\dot\phi+\dot\psi\sin\theta\),
\(\sigma_3=\dot\psi\cos\theta\),
\(\sigma_r=\dot r-R\dot\theta\), and
\(\sigma_g=\dot\gamma+\dot\psi\sin\theta\).

The original kinematics give

$$\dot r=R\sigma_1+\sigma_r.$$

Therefore, the mechanical lock requires $\boxed{\sigma_r=-R\sigma_1}$, rather than $\sigma_r=0$.
The force $F$ becomes the constraint reaction exerted by the lock and can no longer be prescribed independently as a control input.

## 2. Exact nonlinear reduction

Write the original dynamics as

$$M(x)\dot\sigma+c(x,[0,M_2]^T)=b_F F,\qquad
b_F=[R,0,0,1,0]^T.$$

Define four independent pseudovelocities and a constant transformation matrix:

$$v=[\sigma_1,\sigma_2,\sigma_3,\sigma_g]^T,\qquad
\sigma=Tv,\qquad
T=\begin{bmatrix}1&0&0&0\\0&1&0&0\\0&0&1&0\\-R&0&0&0\\0&0&0&1\end{bmatrix}.$$

Since $T^Tb_F=0$, premultiplication by $T^T$ eliminates the unknown reaction force:

$$\boxed{M_f\dot v+c_f=0,\qquad M_f=T^TMT,\quad c_f=T^Tc(x,[0,M_2]^T).}$$

All full-model expressions are evaluated at $r=0,\sigma_r=-R\sigma_1$.
Thus, $c_f=[c_1-Rc_4,c_2,c_3,c_5]^T$. This subtracts $R$ times the fourth original equation from the first and retains the second, third, and fifth equations. All gyroscopic terms are retained.

To make the inertia changes explicit, define

$$D_\gamma=J_{Px}-J_{Pz}+m_ph^2,\quad b=Rm_ph,\quad C=J_{Pz}+J_R+J_{W2},$$
$$W=J_{W1}+R^2(m_p+m_w+m_r),\quad I=J_{Py}+m_ph^2.$$

The complete constrained mass matrix is

$$M_f=\begin{bmatrix}
C+R^2(m_p+m_w+m_r)+2b\cos\gamma+D_\gamma\cos^2\gamma&0&-b\sin\gamma-D_\gamma\sin\gamma\cos\gamma&0\\
0&W&0&b\cos\gamma\\
-b\sin\gamma-D_\gamma\sin\gamma\cos\gamma&0&C+D_\gamma\sin^2\gamma&0\\
0&b\cos\gamma&0&I
\end{bmatrix}.$$

Locking the rod adds $m_rR^2$ to the lean inertia. The rod damping term $B_R\dot r$ vanishes identically.

The reduced ten-state vector is

$$z=[\sigma_1,\sigma_2,\sigma_3,\sigma_g,\psi,\theta,\phi,\gamma,x_G,y_G]^T.$$

The remaining kinematic equations are

$$\begin{aligned}
\dot\psi&=\sigma_3/\cos\theta,&\dot\theta&=\sigma_1,\\
\dot\phi&=\sigma_2-\sigma_3\tan\theta,&\dot\gamma&=\sigma_g-\sigma_3\tan\theta,\\
\dot x_G&=R\sigma_1\sin\psi\cos\theta+R\sigma_2\cos\psi,\\
\dot y_G&=-R\sigma_1\cos\psi\cos\theta+R\sigma_2\sin\psi.
\end{aligned}$$

Recover the reaction force from the fourth original equation:

$$\boxed{F=m_r[-R\dot\sigma_1+R\sigma_2\sigma_3+g\sin\theta].}$$

This uses the original model's public input convention $F=-F_1$. The ideal lock does no work through relative translation: $F\dot r=0$.

## 3. Upright straight rolling and linearization

Let $\Omega=V/R$ and choose a straight reference along $+x$. The nominal trajectory is

$$\sigma_2=\Omega,\quad \phi=\Omega t,\quad x_G=Vt,$$

with all other states zero.
Variables below denote small perturbations from this trajectory.

Define

$$\begin{aligned}
J&=J_{Px}+J_R+J_{W2}+R^2(m_p+m_w+m_r)+2Rm_ph+m_ph^2,\\
G&=g[R(m_p+m_w+m_r)+m_ph],\\
H&=J_{W1}+R^2(m_p+m_w+m_r)+Rm_ph=W+b,\\
b&=Rm_ph,\\
P&=gm_ph,\qquad \Delta=WI-b^2.
\end{aligned}$$

For reference, the original full-model mass matrix acts on the five pseudovelocities
$\sigma=[\sigma_1,\sigma_2,\sigma_3,\sigma_r,\sigma_g]^T$. Before imposing the rod constraint, it is

$$\boxed{M(x)=\begin{bmatrix}
M_{11}&0&M_{13}&0&0\\
0&W&-Rm_rr&0&b\cos\gamma\\
M_{13}&-Rm_rr&M_{33}&0&0\\
0&0&0&m_r&0\\
0&b\cos\gamma&0&0&I
\end{bmatrix},}$$

where, using $D_\gamma,b,C,W,I$ defined in Section 2,

$$\begin{aligned}
M_{11}&=C+R^2(m_p+m_w)+2b\cos\gamma+m_rr^2+D_\gamma\cos^2\gamma,\\
M_{13}&=-b\sin\gamma-D_\gamma\sin\gamma\cos\gamma,\\
M_{33}&=C+m_rr^2+D_\gamma\sin^2\gamma.
\end{aligned}$$

At the centered upright configuration $r=0,\theta=0,\gamma=0$, define the original lean inertia and gravity coefficient as

$$\begin{aligned}
I_\ell&=J_{Px}+J_R+J_{W2}+R^2(m_p+m_w)+2Rm_ph+m_ph^2,\\
G_\ell&=g[R(m_p+m_w)+m_ph].
\end{aligned}$$

The original mass matrix at this configuration is

$$\boxed{M_0=\begin{bmatrix}
I_\ell&0&0&0&0\\
0&W&0&0&b\\
0&0&C&0&0\\
0&0&0&m_r&0\\
0&b&0&0&I
\end{bmatrix}.}$$

Applying the constraint transformation $\sigma=Tv$ from Section 2 gives the four-dimensional matrix, in the order $v=[\sigma_1,\sigma_2,\sigma_3,\sigma_g]^T$:

$$\boxed{M_{f,0}=T^TM_0T=\begin{bmatrix}
I_\ell+m_rR^2&0&0&0\\
0&W&0&b\\
0&0&C&0\\
0&b&0&I
\end{bmatrix}=\begin{bmatrix}
J&0&0&0\\
0&W&0&b\\
0&0&C&0\\
0&b&0&I
\end{bmatrix}.}$$

Thus, at the upright configuration, the constrained lean and yaw inertias are $J$ and $C$, respectively, with

$$\boxed{J=I_\ell+m_rR^2,\qquad G=G_\ell+m_rgR.}$$

The added lean inertia comes from enforcing $\sigma_r=-R\sigma_1$ in the original rod kinetic-energy term $\tfrac12m_r\sigma_r^2$. Simply deleting the fourth row and column of $M_0$ would miss this contribution. The longitudinal inertia block $\begin{bmatrix}W&b\\b&I\end{bmatrix}$ is unchanged.
The first-order terms of the first and fourth original equations are

$$I_\ell\dot\sigma_1=G_\ell\theta+(J_{W1}+R^2(m_p+m_w)+b)\Omega\sigma_3+RF,$$
$$m_r\dot\sigma_r=-m_rg\theta-Rm_r\Omega\sigma_3+F.$$

Substituting $\dot\sigma_r=-R\dot\sigma_1$ and eliminating $F$ gives

$$\boxed{J\dot\sigma_1=G\theta+H\Omega\sigma_3,\qquad
C\dot\sigma_3=-J_{W1}\Omega\sigma_1,\qquad \dot\theta=\sigma_1.}$$

The first-order longitudinal dynamics are

$$\boxed{\begin{bmatrix}W&b\\b&I\end{bmatrix}
\begin{bmatrix}\dot{\delta\sigma_2}\\\dot\sigma_g\end{bmatrix}
=\begin{bmatrix}M_2\\P\gamma-M_2\end{bmatrix},\qquad \dot\gamma=\sigma_g.}$$

The lateral and longitudinal dynamics decouple at first order. The remaining states satisfy

$$\dot\psi=\sigma_3,\quad \dot{\delta\phi}=\delta\sigma_2,\quad
\dot{\delta x_G}=R\delta\sigma_2,\quad \dot y_G=-R\sigma_1+V\psi.$$


## 4. Analytical lateral eigenvalues and critical speed

Let $k=G/J,\ d=H\Omega/J,\ a=J_{W1}\Omega/C$. Then

$$\frac{d}{dt}\begin{bmatrix}\theta\\\sigma_1\\\sigma_3\end{bmatrix}
=\begin{bmatrix}0&1&0\\k&0&d\\0&-a&0\end{bmatrix}
\begin{bmatrix}\theta\\\sigma_1\\\sigma_3\end{bmatrix}.$$

Expanding the matrix equation row by row gives

$$\boxed{\begin{aligned}
\dot\theta&=\sigma_1,\\
\dot\sigma_1&=k\theta+d\sigma_3
=\frac{G}{J}\theta+\frac{H\Omega}{J}\sigma_3,\\
\dot\sigma_3&=-a\sigma_1
=-\frac{J_{W1}\Omega}{C}\sigma_1.
\end{aligned}}$$

Equivalently, substituting $\Omega=V/R$ makes the dependence on forward speed explicit:

$$\frac{d}{dt}\begin{bmatrix}\theta\\\sigma_1\\\sigma_3\end{bmatrix}
=\begin{bmatrix}
0&1&0\\
\dfrac{G}{J}&0&\dfrac{HV}{JR}\\
0&-\dfrac{J_{W1}V}{CR}&0
\end{bmatrix}
\begin{bmatrix}\theta\\\sigma_1\\\sigma_3\end{bmatrix}.$$

The characteristic polynomial and eigenvalues are

$$\det(\lambda I-A_\ell)=\lambda(\lambda^2-k+da),$$
$$\boxed{\lambda_{\ell,0}=0,\qquad
\lambda_{\ell,\pm}=\pm\sqrt{\frac{G}{J}-\frac{HJ_{W1}}{JC}\frac{V^2}{R^2}}.}$$

The corresponding critical speed is

$$\boxed{V_c=R\sqrt{\frac{GC}{HJ_{W1}}}.}$$

Substituting $G=g[R(m_p+m_w+m_r)+m_ph]$, $C=J_{Pz}+J_R+J_{W2}$, and $H=J_{W1}+R^2(m_p+m_w+m_r)+Rm_ph$ gives the expression in the original physical parameters:

$$\boxed{
V_c=R\sqrt{
\frac{
g\big[R(m_p+m_w+m_r)+m_ph\big]\big(J_{Pz}+J_R+J_{W2}\big)
}{
J_{W1}\big[J_{W1}+R^2(m_p+m_w+m_r)+Rm_ph\big]
}
}.
}$$

