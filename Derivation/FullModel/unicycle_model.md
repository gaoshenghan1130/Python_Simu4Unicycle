# Unicycle: Notation and Nonlinear Model

$\theta$ corresponds to $\vartheta$ in the notebook. All parameters use SI units. The MATLAB model input is $u=[F,M_2]^T$.

## States

$$
q=\begin{bmatrix}\psi&\theta&\phi&r&\gamma&x_G&y_G\end{bmatrix}^{T},
\qquad
\sigma=\begin{bmatrix}\sigma_1&\sigma_2&\sigma_3&\sigma_r&\sigma_g\end{bmatrix}^{T}.
$$

$$
\boxed{x=\begin{bmatrix}\sigma_1&\sigma_2&\sigma_3&\sigma_r&\sigma_g&\psi&\theta&\phi&r&\gamma&x_G&y_G\end{bmatrix}^{T}}
$$

| MATLAB index | Notebook name | Meaning | Unit |
|---|---|---|---|
| 1 | `σ1` | $\dot\theta$, lean angular velocity | rad/s |
| 2 | `σ2` | Wheel angular velocity component along the wheel axle | rad/s |
| 3 | `σ3` | Wheel angular velocity component along the third axis of the tilted frame | rad/s |
| 4 | `σr` | $\dot r-R\dot\theta$ | m/s |
| 5 | `σg` | $\dot\gamma+\dot\psi\sin\theta$ | rad/s |
| 6 | `ψ` | Heading angle / yaw | rad |
| 7 | `ϑ` | Lean angle | rad |
| 8 | `φ` | Wheel rotation angle | rad |
| 9 | `r` | Signed displacement of the rod center of mass along the wheel axle | m |
| 10 | `γ` | Body/inverted-pendulum tilt angle about the wheel axle | rad |
| 11 | `xG` | Global $x$ coordinate of the wheel center | m |
| 12 | `yG` | Global $y$ coordinate of the wheel center | m |

Based on their kinematics, `xG,yG` correspond to the wheel-center coordinates $x_C,y_C$ in the paper. They are neither the overall system center-of-mass coordinates nor the ground-contact trajectory coordinates.

## Pseudovelocities

$$
\boxed{
\begin{aligned}
\sigma_1&=\dot\theta,\\
\sigma_2&=\dot\phi+\dot\psi\sin\theta,\\
\sigma_3&=\dot\psi\cos\theta,\\
\sigma_r&=\dot r-R\dot\theta,\\
\sigma_g&=\dot\gamma+\dot\psi\sin\theta.
\end{aligned}}
$$

Thus, $\sigma_r$ must not be treated as $\dot r$, and $\sigma_g$ is not generally equal to $\dot\gamma$.

## Inputs

The original `Cfcn` input is

$$u_{\rm nb}=\begin{bmatrix}F_1&M_2\end{bmatrix}^{T}.$$

The simulation controller first computes $F$ and then passes it to the model through `return np.array([-F, M2])`. Therefore, the input convention used here is

$$\boxed{u=\begin{bmatrix}F&M_2\end{bmatrix}^{T},\qquad F=-F_1.}$$

$F_1$ follows the convention for the lateral internal force acting on the wheel; $F$ follows the opposite convention for the force acting on the moving rod. $M_2$ is the driving torque on the wheel, with the reaction torque acting on the body. Force and torque are measured in N and N·m, respectively.

## Parameters

The parameter array must follow this exact order:

$$p=[g,R,m_w,J_{W1},J_{W2},m_r,J_R,B_R,h,m_p,J_{Px},J_{Py},J_{Pz},B_P].$$

| Parameter | Meaning | `ps_sim` value | Unit |
|---|---|---:|---|
| $g$ | Gravitational acceleration | 9.81 | m/s² |
| $R$ | Wheel radius | 0.253 | m |
| $m_w$ | Wheel mass | 2.436 | kg |
| $J_{W1}$ | Wheel moment of inertia about the axle ($y$ axis) | 0.09099921839 | kg·m² |
| $J_{W2}$ | Wheel moment of inertia about the $x,z$ axes | 0.04591427768 | kg·m² |
| $m_r$ | Total mass of the moving rod assembly | 2.3 | kg |
| $J_R$ | Rod moment of inertia about its center-of-mass $x,z$ axes | 0.0517629 | kg·m² |
| $B_R$ | Viscous damping coefficient for relative rod motion | 0 | N·s/m |
| $h$ | Distance from the wheel center to the body center of mass | 0.025 | m |
| $m_p$ | Body/inverted-pendulum mass | 2.799 | kg |
| $J_{Px}$ | Body moment of inertia about its center-of-mass $x$ axis | 0.01290418213 | kg·m² |
| $J_{Py}$ | Body moment of inertia about its center-of-mass $y$ axis | 0.02090219895 | kg·m² |
| $J_{Pz}$ | Body moment of inertia about its center-of-mass $z$ axis | 0.01118711607 | kg·m² |
| $B_P$ | Rotational viscous damping between the wheel and body | 0 | N·m·s/rad |

The inertias are defined about the respective centers of mass. The code separately includes parallel-axis terms such as $m_ph^2$ and $m_rr^2$. When substituting CAD data, verify the reference points to avoid counting these terms twice.

## Full Nonlinear Model

### 1. General Form

To distinguish the notebook variable `C` from the inertial-force vector in the paper, the gravity and inertial terms are denoted by $n(q,\sigma)$, with the viscous terms written separately:

$$
\boxed{M(q)\dot\sigma+n(q,\sigma)+D\sigma=Q_u u,}
$$

$$
Q_u=\begin{bmatrix}
R&0\\0&1\\0&0\\1&0\\0&-1
\end{bmatrix},\qquad
D=\begin{bmatrix}
B_RR^2&0&0&B_RR&0\\
0&B_P&0&0&-B_P\\
0&0&0&0&0\\
B_RR&0&0&B_R&0\\
0&-B_P&0&0&B_P
\end{bmatrix}.
$$

The relative velocities associated with damping are

$$\dot r=R\sigma_1+\sigma_r,\qquad \dot\phi-\dot\gamma=\sigma_2-\sigma_g.$$

The relationship to the original code is

$$C_{\rm nb}[0:5]=n+D\sigma-Q_u\begin{bmatrix}-F_1\\M_2\end{bmatrix}.$$

Defining $k(q,\sigma)=\dot q$ gives

$$\boxed{\dot x=f(x,u)=\begin{bmatrix}M(q)^{-1}(Q_u u-n-D\sigma)\\k(q,\sigma)\end{bmatrix}.}$$

The implementation uses MATLAB `M \ (...)` without explicitly computing the inverse. In a 12×12 implicit formulation, the mass matrix is $\mathcal M=\operatorname{blkdiag}(M,I_7)$.

### 2. Mass Matrix

For compact notation, define

$$a=J_{Px}-J_{Pz}+m_ph^2,\qquad b=Rm_ph,\qquad c=J_{Pz}+J_R+J_{W2},$$

$$s_\gamma=\sin\gamma,\quad c_\gamma=\cos\gamma,\quad t_\theta=\tan\theta.$$

These abbreviations are local to this document and are distinct from $c_1$ in the paper.

$$
\boxed{M=\begin{bmatrix}
M_{11}&0&M_{13}&0&0\\
0&M_{22}&-Rm_rr&0&bc_\gamma\\
M_{13}&-Rm_rr&M_{33}&0&0\\
0&0&0&m_r&0\\
0&bc_\gamma&0&0&J_{Py}+m_ph^2
\end{bmatrix}}
$$

where

$$
\begin{aligned}
M_{11}&=c+R^2(m_p+m_w)+2bc_\gamma+m_rr^2+ac_\gamma^2,\\
M_{13}&=-(b+ac_\gamma)s_\gamma,\\
M_{22}&=J_{W1}+R^2(m_p+m_r+m_w),\\
M_{33}&=c+m_rr^2+as_\gamma^2.
\end{aligned}
$$

### 3. Full Gravity and Inertial Terms

The following five terms correspond entry by entry to the first five entries of the original `Cfcn`, with the input and viscous terms removed.

$$
\begin{aligned}
n_1={}&-g\left[R(m_p+m_w)+hm_pc_\gamma\right]\sin\theta
       +gm_rr\cos\theta\\
&+Rm_rr\sigma_1^2+2m_rr\sigma_1\sigma_r\\
&+(b+ac_\gamma)s_\gamma t_\theta\sigma_1\sigma_3
 -2(b+ac_\gamma)s_\gamma\sigma_1\sigma_g\\
&-\left[J_{W1}+R^2(m_p+m_w)+bc_\gamma+Rm_rrt_\theta\right]\sigma_2\sigma_3\\
&+\left[bc_\gamma+m_rr^2+ac_\gamma^2+c\right]t_\theta\sigma_3^2\\
&+\left[J_{Px}-J_{Py}-J_{Pz}-2bc_\gamma-2ac_\gamma^2\right]\sigma_3\sigma_g.
\end{aligned}
$$

$$
\begin{aligned}
n_2={}&-bs_\gamma(\sigma_3^2+\sigma_g^2)-2Rm_r\sigma_3\sigma_r\\
&+\left[R^2(m_p-m_r+m_w)+bc_\gamma+Rm_rrt_\theta\right]\sigma_1\sigma_3.
\end{aligned}
$$

$$
\begin{aligned}
n_3={}&J_{W1}\sigma_1\sigma_2+bs_\gamma\sigma_2\sigma_3
       +ghm_ps_\gamma\sin\theta+2m_rr\sigma_3\sigma_r\\
&+\left[Rm_rr-(m_rr^2+as_\gamma^2+c)t_\theta\right]\sigma_1\sigma_3\\
&+\left[-J_{Px}+J_{Py}+J_{Pz}+2as_\gamma^2\right]\sigma_1\sigma_g\\
&-as_\gamma c_\gamma t_\theta\sigma_3^2
 +2as_\gamma c_\gamma\sigma_3\sigma_g.
\end{aligned}
$$

$$n_4=Rm_r\sigma_2\sigma_3+gm_r\sin\theta-m_rr(\sigma_1^2+\sigma_3^2).$$

$$
\begin{aligned}
n_5={}&bs_\gamma t_\theta\sigma_2\sigma_3-ghm_ps_\gamma\cos\theta\\
&+(b+ac_\gamma)s_\gamma\sigma_1^2
 +(a+bc_\gamma-2as_\gamma^2)\sigma_1\sigma_3
 -as_\gamma c_\gamma\sigma_3^2.
\end{aligned}
$$

Here, $m_rr$ means $m_r\,r$, and $m_rr^2$ means $m_r\,r^2$.

### 4. Seven Kinematic Equations

$$
\boxed{
\begin{aligned}
\dot\psi&=\frac{\sigma_3}{\cos\theta},\\
\dot\theta&=\sigma_1,\\
\dot\phi&=\sigma_2-\sigma_3\tan\theta,\\
\dot r&=R\sigma_1+\sigma_r,\\
\dot\gamma&=\sigma_g-\sigma_3\tan\theta,\\
\dot x_G&=R\sigma_1\sin\psi\cos\theta+R\sigma_2\cos\psi,\\
\dot y_G&=-R\sigma_1\cos\psi\cos\theta+R\sigma_2\sin\psi.
\end{aligned}}
$$

These coordinates require $\cos\theta\ne0$.
