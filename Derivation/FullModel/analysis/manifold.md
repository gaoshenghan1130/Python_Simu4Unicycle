# Equilibrium Manifold

With 

$$
x=\begin{bmatrix}q^T&\sigma^T\end{bmatrix}^T\in\mathbb{R}^{12},
\qquad
u=\begin{bmatrix}F&M\end{bmatrix}^T\in\mathbb{R}^{2},
$$


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

The same constraint can be factorized to keep the repeated $r\tan\theta$
terms visible:

$$
\boxed{
\begin{aligned}
0={}&22.563r-19.388\tan\theta\\
&-\dot\phi\dot\psi
\left(0.591+0.582r\tan\theta\right)\\
&+\dot\psi^2\Big\{
-0.523\sin\theta\\
&\qquad+r\cos\theta
\left[0.582-0.582\tan^2\theta+2.3r\tan\theta\right]
\Big\}.
\end{aligned}}
$$

The two repeated structures are therefore

$$
0.591+0.582r\tan\theta,
$$

and

$$
0.582-0.582\tan^2\theta+2.3r\tan\theta.
$$

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

For a stationary equilibrium, one additionally imposes $K(q,\sigma)=0$.
For a steady-turning relative equilibrium, $K(q,\sigma)$ is not zero because
$\dot\psi=\Omega/\cos\theta$ and $\dot\phi=V-\Omega\tan\theta$.

## Steady-turning relative-equilibrium manifold

The steady-turning conditions are

$$
\dot\theta=0,
\qquad
\dot r=0,
\qquad
\dot\gamma=0,
\qquad
\dot\sigma=0.
$$

Hence,

$$
\boxed{
\sigma_1=0,
\qquad
\sigma_r=0,
\qquad
\sigma_g=\dot\psi\sin\theta,
\qquad
\sigma_2=\dot\phi+\dot\psi\sin\theta,
\qquad
\sigma_3=\dot\psi\cos\theta.}
$$

The remaining coordinates satisfy

$$
\dot\psi=\mathrm{constant}\neq0,
\qquad
\dot\phi=\mathrm{constant}.
$$

After substitution, the reduced dynamic equations are

$$
\begin{aligned}
RF={}&-g[R(m_p+m_w)+hm_pc_\gamma]\sin\theta+gm_rr\cos\theta\\
&-[J_{W1}+R^2(m_p+m_w)+bc_\gamma+Rm_rr\tan\theta]
(\dot\phi+\dot\psi\sin\theta)\dot\psi\cos\theta\\
&+[bc_\gamma+m_rr^2+ac_\gamma^2+c]\tan\theta\,\dot\psi^2\cos^2\theta\\
&+[J_{Px}-J_{Py}-J_{Pz}-2bc_\gamma-2ac_\gamma^2]
\dot\psi^2\sin\theta\cos\theta,
\end{aligned}
$$

$$
M_2=-bs_\gamma\dot\psi^2
+B_P\dot\phi,
$$

$$
0=bs_\gamma(\dot\phi+\dot\psi\sin\theta)\dot\psi\cos\theta
+ghm_ps_\gamma\sin\theta
+2as_\gamma c_\gamma\dot\psi^2\sin\theta\cos\theta,
$$

$$
F=Rm_r(\dot\phi+\dot\psi\sin\theta)\dot\psi\cos\theta
+gm_r\sin\theta-m_rr\dot\psi^2\cos^2\theta,
$$

$$
-M_2=bs_\gamma\tan\theta(\dot\phi+\dot\psi\sin\theta)\dot\psi\cos\theta
-ghm_ps_\gamma\cos\theta-as_\gamma c_\gamma\dot\psi^2\cos^2\theta-B_P\dot\phi.
$$

Therefore,

$$
\mathcal E_{\mathrm{turn}}
=\left\{x_e(\theta,r,\gamma,\dot\phi,\dot\psi):
\sigma_1=0,\ \sigma_r=0,\
\sigma_2=\dot\phi+\dot\psi\sin\theta,\
\sigma_3=\dot\psi\cos\theta,\
\sigma_g=\dot\psi\sin\theta,\ \dot\sigma=0\right\}.
$$

## Input-eliminated constraints

The third dynamic equation gives the first constraint

$$
\boxed{
\begin{aligned}
0={}&bs_\gamma(\dot\phi+\dot\psi\sin\theta)\dot\psi\cos\theta
+ghm_ps_\gamma\sin\theta\\
&+2as_\gamma c_\gamma\dot\psi^2\sin\theta\cos\theta.
\end{aligned}}
$$

The first and fourth dynamic equations both contain $F$. Eliminating $F$ by
subtracting $R$ times the fourth equation from the first gives

$$
\boxed{
\begin{aligned}
0={}&-g[R(m_p+m_w)+hm_pc_\gamma]\sin\theta+gm_rr\cos\theta\\
&-[J_{W1}+R^2(m_p+m_w)+bc_\gamma+Rm_rr\tan\theta]
(\dot\phi+\dot\psi\sin\theta)\dot\psi\cos\theta\\
&+[bc_\gamma+m_rr^2+ac_\gamma^2+c]\tan\theta\,\dot\psi^2\cos^2\theta\\
&+[J_{Px}-J_{Py}-J_{Pz}-2bc_\gamma-2ac_\gamma^2]
\dot\psi^2\sin\theta\cos\theta\\
&-R\left[Rm_r(\dot\phi+\dot\psi\sin\theta)\dot\psi\cos\theta
+gm_r\sin\theta-m_rr\dot\psi^2\cos^2\theta\right].
\end{aligned}}
$$

If $M_2$ is also eliminated using the second and fifth dynamic equations, the
full model has the additional torque-compatibility constraint

$$
\boxed{
\begin{aligned}
0={}&-bs_\gamma\dot\psi^2
+bs_\gamma\tan\theta(\dot\phi+\dot\psi\sin\theta)\dot\psi\cos\theta\\
&-ghm_ps_\gamma\cos\theta
-as_\gamma c_\gamma\dot\psi^2\cos^2\theta.
\end{aligned}}
$$

## Simplified steady-turning manifold

Using

$$
\sigma_2=\dot\phi+\dot\psi\sin\theta,
\qquad
\sigma_3=\dot\psi\cos\theta,
\qquad
\sigma_g=\dot\psi\sin\theta,
$$

the first and third input-eliminated constraints factor as

$$
\boxed{
\sin\gamma\left[
b(\dot\phi+\dot\psi\sin\theta)\dot\psi\cos\theta
+ghm_p\sin\theta
+2ac_\gamma\dot\psi^2\sin\theta\cos\theta
\right]=0,}
$$

and

$$
\boxed{
\sin\gamma\left[
-b\dot\psi^2
+b\tan\theta(\dot\phi+\dot\psi\sin\theta)\dot\psi\cos\theta
-ghm_p\cos\theta
-ac_\gamma\dot\psi^2\cos^2\theta
\right]=0.}
$$

Consequently, the steady-turning set contains the branch

$$
\boxed{\gamma=0,}
$$




## $\gamma=0$ branch

On the branch $\gamma=0$,

$$
s_\gamma=0,
\qquad c_\gamma=1,
\qquad
\sigma_2=\dot\phi+\dot\psi\sin\theta,
\qquad
\sigma_3=\dot\psi\cos\theta,
\qquad
\sigma_g=\dot\psi\sin\theta.
$$

The two constraints containing the factor $s_\gamma$ are then identically
satisfied. The torque-compatibility constraint is also identically satisfied.
The remaining force-eliminated constraint is

$$
\boxed{
\begin{aligned}
0={}&-g[R(m_p+m_w+m_r)+hm_p]\sin\theta\\
&+g m_r r\cos\theta\\
&-\dot\phi\dot\psi\cos\theta
\left[J_{W1}+R^2(m_p+m_w+m_r)+R m_p h+R m_r r\tan\theta\right]\\
&+\dot\psi^2\Big\{
-\left[J_{W1}+R^2(m_p+m_w+m_r)+R m_p h\right]\sin\theta\cos\theta\\
&\qquad+[J_{Px}+J_R+J_{W2}+m_p h^2+R m_p h]
\tan\theta\cos^2\theta\\
&\qquad+[-J_{Px}-J_{Py}+J_{Pz}-2R m_p h-2m_p h^2]
\sin\theta\cos\theta\\
&\qquad+R m_r r\cos^2\theta
-R m_r r\tan\theta\sin\theta\cos\theta\\
&\qquad+m_r r^2\tan\theta\cos^2\theta
\Big\}.
\end{aligned}}
$$

Thus, after imposing $\gamma=0$, the steady-turning relative-equilibrium
manifold is described by the above single remaining constraint together with

$$
\sigma_1=0,
\qquad
\sigma_r=0,
\qquad
\sigma_2=\dot\phi+\dot\psi\sin\theta,
\qquad
\sigma_3=\dot\psi\cos\theta,
\qquad
\sigma_g=\dot\psi\sin\theta.
$$

Using the parameter values

$$
\begin{aligned}
g[R(m_p+m_w+m_r)+hm_p]&=19.388,\\
gm_r&=22.563,\\
J_{W1}+R^2(m_p+m_w+m_r)+Rm_ph&=0.591,\\
J_{Px}+J_R+J_{W2}+m_ph^2+Rm_ph&=0.130,\\
-J_{Px}-J_{Py}+J_{Pz}-2Rm_ph-2m_ph^2&=-0.0615,\\
Rm_r&=0.582,
\end{aligned}
$$

the remaining constraint becomes

$$
\boxed{
\begin{aligned}
0={}&-19.388\sin\theta+22.563 r\cos\theta\\
&-\dot\phi\dot\psi\cos\theta
\left(0.591+0.582 r\tan\theta\right)\\
&+\dot\psi^2\Big\{
-0.653\sin\theta\cos\theta\\
&\qquad+0.130\tan\theta\cos^2\theta\\
&\qquad+0.582 r\cos^2\theta\\
&\qquad-0.582 r\tan\theta\sin\theta\cos\theta\\
&\qquad+2.3 r^2\tan\theta\cos^2\theta
\Big\}.
\end{aligned}}
$$

Dividing the constraint by $\cos\theta$, assuming $\cos\theta\neq0$, and
using $\tan\theta\cos\theta=\sin\theta$ gives

$$
\boxed{
\begin{aligned}
0={}&-19.388\tan\theta+22.563 r\\
&-\dot\phi\dot\psi
\left(0.591+0.582 r\tan\theta\right)\\
&+\dot\psi^2\Big[
-0.523\sin\theta
+0.582 r\cos\theta\\
&\qquad-0.582 r\tan\theta\sin\theta
+2.3 r^2\sin\theta
\Big].
\end{aligned}}
$$

Or extract the repeated $r\tan\theta$ terms to give:

\[ 
\boxed{ \begin{aligned} 0={}&22.563\left(r-0.859274\tan\theta\right)\\ &-0.5819\dot\phi\dot\psi \left(1.015657+r\tan\theta\right)\\ &+\dot\psi^2\Big[ 2.3r\cos\theta\left(0.253+r\tan\theta\right)\\ &\qquad-0.5819\sin\theta \left(0.897923+r\tan\theta\right) \Big]. \end{aligned}} 
\]