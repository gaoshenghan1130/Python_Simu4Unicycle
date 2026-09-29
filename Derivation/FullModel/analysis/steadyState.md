# State-space representation

Let

$$
x=\begin{bmatrix}q^T&\sigma^T\end{bmatrix}^T\in\mathbb{R}^{12},
\qquad
u=\begin{bmatrix}F&M\end{bmatrix}^T\in\mathbb{R}^{2},
$$

where

$$
q=\begin{bmatrix}\psi&\theta&\phi&r&\gamma&x_G&y_G\end{bmatrix}^{T},
\qquad
\sigma=\begin{bmatrix}\sigma_1&\sigma_2&\sigma_3&\sigma_r&\sigma_g\end{bmatrix}^{T}.
$$

The pseudo-velocities are defined by

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

The dynamic equations are written as

$$
M(q)\dot{\sigma}+n(q,\sigma)+D\sigma=Q_u u,
$$

where

$$
Q_u=\begin{bmatrix}R&0\\0&1\\0&0\\1&0\\0&-1\end{bmatrix},
\qquad
D=\begin{bmatrix}
B_RR^2&0&0&B_RR&0\\
0&B_P&0&0&-B_P\\
0&0&0&0&0\\
B_RR&0&0&B_R&0\\
0&-B_P&0&0&B_P
\end{bmatrix},
$$

$$
M=\begin{bmatrix}
M_{11}&0&M_{13}&0&0\\
0&M_{22}&-Rm_rr&0&bc_\gamma\\
M_{13}&-Rm_rr&M_{33}&0&0\\
0&0&0&m_r&0\\
0&bc_\gamma&0&0&J_{Py}+m_ph^2
\end{bmatrix},
$$

$$
\begin{aligned}
a&=J_{Px}-J_{Pz}+m_ph^2, & b&=Rm_ph, & c&=J_{Pz}+J_R+J_{W2},\\
s_\gamma&=\sin\gamma, & c_\gamma&=\cos\gamma, & t_\theta&=\tan\theta,\\
M_{11}&=c+R^2(m_p+m_w)+2bc_\gamma+m_rr^2+ac_\gamma^2,\\
M_{13}&=-(b+ac_\gamma)s_\gamma,\\
M_{22}&=J_{W1}+R^2(m_p+m_r+m_w),\\
M_{33}&=c+m_rr^2+as_\gamma^2.
\end{aligned}
$$

The components of $n=[n_1,n_2,n_3,n_4,n_5]^T$ are

$$
\begin{aligned}
n_1={}&-g[R(m_p+m_w)+hm_pc_\gamma]\sin\theta+gm_rr\cos\theta\\
&+Rm_rr\sigma_1^2+2m_rr\sigma_1\sigma_r+(b+ac_\gamma)s_\gamma t_\theta\sigma_1\sigma_3\\
&-2(b+ac_\gamma)s_\gamma\sigma_1\sigma_g-[J_{W1}+R^2(m_p+m_w)+bc_\gamma+Rm_rrt_\theta]\sigma_2\sigma_3\\
&+[bc_\gamma+m_rr^2+ac_\gamma^2+c]t_\theta\sigma_3^2\\
&+[J_{Px}-J_{Py}-J_{Pz}-2bc_\gamma-2ac_\gamma^2]\sigma_3\sigma_g,\\
n_2={}&-bs_\gamma(\sigma_3^2+\sigma_g^2)-2Rm_r\sigma_3\sigma_r\\
&+[R^2(m_p-m_r+m_w)+bc_\gamma+Rm_rrt_\theta]\sigma_1\sigma_3,\\
n_3={}&J_{W1}\sigma_1\sigma_2+bs_\gamma\sigma_2\sigma_3+ghm_ps_\gamma\sin\theta+2m_rr\sigma_3\sigma_r\\
&+[Rm_rr-(m_rr^2+as_\gamma^2+c)t_\theta]\sigma_1\sigma_3\\
&+[-J_{Px}+J_{Py}+J_{Pz}+2as_\gamma^2]\sigma_1\sigma_g-as_\gamma c_\gamma t_\theta\sigma_3^2\\
&+2as_\gamma c_\gamma\sigma_3\sigma_g,\\
n_4={}&Rm_r\sigma_2\sigma_3+gm_r\sin\theta-m_rr(\sigma_1^2+\sigma_3^2),\\
n_5={}&bs_\gamma t_\theta\sigma_2\sigma_3-ghm_ps_\gamma\cos\theta+(b+ac_\gamma)s_\gamma\sigma_1^2\\
&+(a+bc_\gamma-2as_\gamma^2)\sigma_1\sigma_3-as_\gamma c_\gamma\sigma_3^2.
\end{aligned}
$$

The five dynamic equations in component form are

$$
\begin{aligned}
\sum_{j=1}^{5}M_{1j}\dot\sigma_j
&+n_1+B_RR^2\sigma_1+B_RR\sigma_r=RF,\\
\sum_{j=1}^{5}M_{2j}\dot\sigma_j
&+n_2+B_P(\sigma_2-\sigma_g)=M_2,\\
\sum_{j=1}^{5}M_{3j}\dot\sigma_j
&+n_3=0,\\
\sum_{j=1}^{5}M_{4j}\dot\sigma_j
&+n_4+B_RR\sigma_1+B_R\sigma_r=F,\\
\sum_{j=1}^{5}M_{5j}\dot\sigma_j
&+n_5+B_P(\sigma_g-\sigma_2)=-M_2.
\end{aligned}
$$

Equivalently,

$$
\boxed{
M(q)\dot\sigma+n(q,\sigma)+D\sigma
=
\begin{bmatrix}RF&M_2&0&F&-M_2\end{bmatrix}^{T}.}
$$

The kinematic equations are

$$
\dot q=K(q,\sigma)=
\begin{bmatrix}
\sigma_3/\cos\theta\\\sigma_1\\\sigma_2-\sigma_3\tan\theta\\
R\sigma_1+\sigma_r\\\sigma_g-\sigma_3\tan\theta\\
R\sigma_1\sin\psi\cos\theta+R\sigma_2\cos\psi\\
-R\sigma_1\cos\psi\cos\theta+R\sigma_2\sin\psi
\end{bmatrix}.
$$

Therefore, the complete nonlinear model is

$$
\boxed{
\dot{x}=f(x,u)=
\begin{bmatrix}
K(q,\sigma)\\[1mm]
M(q)^{-1}\left[Q_u u-n(q,\sigma)-D\sigma\right]
\end{bmatrix}.}
$$

## Steady-state equations

For $\dot{x}=0$, one has $\dot{\sigma}=0$ and therefore
$M(q)\dot{\sigma}=0$. The dynamic equations reduce to

$$
\begin{aligned}
0={}&-g[R(m_p+m_w)+hm_pc_\gamma]\sin\theta+gm_rr\cos\theta\\
&+Rm_rr\sigma_1^2+2m_rr\sigma_1\sigma_r
+(b+ac_\gamma)s_\gamma t_\theta\sigma_1\sigma_3\\
&-2(b+ac_\gamma)s_\gamma\sigma_1\sigma_g
-[J_{W1}+R^2(m_p+m_w)+bc_\gamma+Rm_rrt_\theta]\sigma_2\sigma_3\\
&+[bc_\gamma+m_rr^2+ac_\gamma^2+c]t_\theta\sigma_3^2\\
&+[J_{Px}-J_{Py}-J_{Pz}-2bc_\gamma-2ac_\gamma^2]\sigma_3\sigma_g\\
&+B_RR^2\sigma_1+B_RR\sigma_r-RF,
\end{aligned}
$$

$$
\begin{aligned}
0={}&-bs_\gamma(\sigma_3^2+\sigma_g^2)-2Rm_r\sigma_3\sigma_r\\
&+[R^2(m_p-m_r+m_w)+bc_\gamma+Rm_rrt_\theta]\sigma_1\sigma_3\\
&+B_P(\sigma_2-\sigma_g)-M_2,
\end{aligned}
$$

$$
\begin{aligned}
0={}&J_{W1}\sigma_1\sigma_2+bs_\gamma\sigma_2\sigma_3
+ghm_ps_\gamma\sin\theta+2m_rr\sigma_3\sigma_r\\
&+[Rm_rr-(m_rr^2+as_\gamma^2+c)t_\theta]\sigma_1\sigma_3\\
&+[-J_{Px}+J_{Py}+J_{Pz}+2as_\gamma^2]\sigma_1\sigma_g\\
&-as_\gamma c_\gamma t_\theta\sigma_3^2
+2as_\gamma c_\gamma\sigma_3\sigma_g,
\end{aligned}
$$

$$
\begin{aligned}
0={}&Rm_r\sigma_2\sigma_3+gm_r\sin\theta
-m_rr(\sigma_1^2+\sigma_3^2)\\
&+B_RR\sigma_1+B_R\sigma_r-F,
\end{aligned}
$$

$$
\begin{aligned}
0={}&bs_\gamma t_\theta\sigma_2\sigma_3-ghm_ps_\gamma\cos\theta\\
&+(b+ac_\gamma)s_\gamma\sigma_1^2
+(a+bc_\gamma-2as_\gamma^2)\sigma_1\sigma_3\\
&-as_\gamma c_\gamma\sigma_3^2+B_P(\sigma_g-\sigma_2)+M_2.
\end{aligned}
$$

And 

$$
K(q,\sigma)=
\begin{bmatrix}
\sigma_3/\cos\theta\\\sigma_1\\\sigma_2-\sigma_3\tan\theta\\
R\sigma_1+\sigma_r\\\sigma_g-\sigma_3\tan\theta\\
R\sigma_1\sin\psi\cos\theta+R\sigma_2\cos\psi\\
-R\sigma_1\cos\psi\cos\theta+R\sigma_2\sin\psi
\end{bmatrix} = \vec{0}
$$
