# Linearization of the Full Unicycle Model and Comparison with the Partial Model

## 1. Scope and conventions

This document derives the stationary-upright linearization of the supplied MATLAB functions `model_rhs.m`, `mass_matrix.m`, `model_residual.m`, and `model_kinematics.m`. It then compares the lateral block with the linear model constructed in the supplied `PolePlacement.m`.

The main result is conditional but exact: **with matching physical parameters, matching input signs, and zero rod damping, the supplied partial lateral linear model is identical to the lateral block of the full model after a change of velocity coordinates.** This equivalence holds before pole placement.

The comparison concerns the linear matrices in `PolePlacement.m`. Its dependencies `LatParam.m` and `LatModel_SignCorrection.m` were not supplied in this turn, so their parameter values and nonlinear implementation have not been independently checked. No separate longitudinal partial model was supplied.

The full state and input orders are

$$
x=\begin{bmatrix}\sigma_1&\sigma_2&\sigma_3&\sigma_r&\sigma_g&\psi&\theta&\phi&r&\gamma&x_G&y_G\end{bmatrix}^{T},
\qquad u=\begin{bmatrix}F&M_2\end{bmatrix}^{T}.
$$

Here, $F$ is the public MATLAB input, with the notebook convention $F=-F_1$. All feedback gains in this document use $\delta u=-K\delta x$.

The operating point is

$$
\boxed{x_*=0_{12},\qquad u_*=0_2.}
$$

This means zero forward speed, zero yaw rate, upright lean and body angles, and a centered rod. The heading and position origins are chosen to be zero. All variables in the linear equations below are perturbations from this operating point; $\delta$ is suppressed for readability.

## 2. Full 12-state linearization in the original state order

Partition the state as

$$
x=\begin{bmatrix}\sigma\\q\end{bmatrix},\qquad
\sigma=\begin{bmatrix}\sigma_1&\sigma_2&\sigma_3&\sigma_r&\sigma_g\end{bmatrix}^{T},\qquad
q=\begin{bmatrix}\psi&\theta&\phi&r&\gamma&x_G&y_G\end{bmatrix}^{T}.
$$

Define the constants

$$
\begin{aligned}
I_\ell&=J_{Px}+J_R+J_{W2}+R^2(m_p+m_w)+2Rm_ph+m_ph^2,\\
G_\ell&=g\big[R(m_p+m_w)+m_ph\big],\\
I_w^*&=J_{W1}+R^2(m_p+m_r+m_w),\\
I_g&=J_{Py}+m_ph^2,\\
b&=Rm_ph,\\
c&=J_{Pz}+J_R+J_{W2},\\
G_g&=gm_ph.
\end{aligned}
$$

In particular, $I_\ell$ does not contain $m_rR^2$: this is a consequence of using $\sigma_r=\dot r-R\dot\theta$. That term reappears in ordinary velocity coordinates in Section 6.

At the operating point, the pseudovelocity mass matrix is

$$
M_0=\begin{bmatrix}
I_\ell&0&0&0&0\\
0&I_w^*&0&0&b\\
0&0&c&0&0\\
0&0&0&m_r&0\\
0&b&0&0&I_g
\end{bmatrix}.
$$

The input and damping matrices are

$$
Q_u=\begin{bmatrix}R&0\\0&1\\0&0\\1&0\\0&-1\end{bmatrix},
\qquad
D=\begin{bmatrix}
B_RR^2&0&0&B_RR&0\\
0&B_P&0&0&-B_P\\
0&0&0&0&0\\
B_RR&0&0&B_R&0\\
0&-B_P&0&0&B_P
\end{bmatrix}.
$$

The linear gravity matrix, in the stated $q$ order, is

$$
N_q=\begin{bmatrix}
0&-G_\ell&0&gm_r&0&0&0\\
0&0&0&0&0&0&0\\
0&0&0&0&0&0&0\\
0&gm_r&0&0&0&0&0\\
0&0&0&0&-G_g&0&0
\end{bmatrix}.
$$

The kinematic matrix is

$$
H_0=\begin{bmatrix}
0&0&1&0&0\\
1&0&0&0&0\\
0&1&0&0&0\\
R&0&0&1&0\\
0&0&0&0&1\\
0&R&0&0&0\\
-R&0&0&0&0
\end{bmatrix}.
$$

Thus, the complete linear model is

$$
M_0\dot\sigma+D\sigma+N_qq=Q_uu,\qquad \dot q=H_0\sigma,
$$

or explicitly in state-space form,

$$
\boxed{\dot x=Ax+Bu,\qquad
A=\begin{bmatrix}
-M_0^{-1}D&-M_0^{-1}N_q\\
H_0&0_{7\times7}
\end{bmatrix},\qquad
B=\begin{bmatrix}M_0^{-1}Q_u\\0_{7\times2}\end{bmatrix}.}
$$

These formulas specify every entry of the original-order $12\times12$ matrix $A$ and $12\times2$ matrix $B$. Matrix inverses denote the mathematical expressions; MATLAB should use left division.

To obtain this form, expand $M(q)\dot\sigma+n(q,\sigma)+D\sigma=Q_uu$ to first order. Since $\dot\sigma_*=0$, derivatives of $M$ multiply zero at the operating point. All velocity-product terms are second order at rest. The remaining first-order terms are precisely $N_qq$ and $D\sigma$.

## 3. Lateral 4-state subsystem

Choose

$$
x_\ell=\begin{bmatrix}\theta&r&\sigma_1&\sigma_r\end{bmatrix}^{T},
\qquad \sigma_1=\dot\theta,\qquad \sigma_r=\dot r-R\dot\theta.
$$

The subsystem equations are

$$
\begin{aligned}
\dot\theta&=\sigma_1,\\
\dot r&=R\sigma_1+\sigma_r,\\
I_\ell\dot\sigma_1&=G_\ell\theta-gm_rr-B_RR(R\sigma_1+\sigma_r)+RF,\\
\dot\sigma_r&=-g\theta-\frac{B_R}{m_r}(R\sigma_1+\sigma_r)+\frac{F}{m_r}.
\end{aligned}
$$

Therefore,

$$
\boxed{\dot x_\ell=A_\ell x_\ell+B_\ell F,}
$$

$$
A_\ell=\begin{bmatrix}
0&0&1&0\\
0&0&R&1\\
G_\ell/I_\ell&-gm_r/I_\ell&-B_RR^2/I_\ell&-B_RR/I_\ell\\
-g&0&-B_RR/m_r&-B_R/m_r
\end{bmatrix},\qquad
B_\ell=\begin{bmatrix}0\\0\\R/I_\ell\\1/m_r\end{bmatrix}.
$$

At this operating point, no longitudinal or yaw state appears in these equations.

## 4. Longitudinal 4-state subsystem

Choose

$$
x_p=\begin{bmatrix}\phi&\gamma&\sigma_2&\sigma_g\end{bmatrix}^{T}.
$$

At first order about rest, $\dot\phi=\sigma_2$ and $\dot\gamma=\sigma_g$. Define

$$
H_p=\begin{bmatrix}I_w^*&b\\b&I_g\end{bmatrix},\quad
G_p=\begin{bmatrix}0&0\\0&G_g\end{bmatrix},\quad
D_p=B_P\begin{bmatrix}1&-1\\-1&1\end{bmatrix},\quad
b_p=\begin{bmatrix}1\\-1\end{bmatrix}.
$$

Then

$$
H_p\begin{bmatrix}\dot\sigma_2\\\dot\sigma_g\end{bmatrix}
=G_p\begin{bmatrix}\phi\\\gamma\end{bmatrix}
-D_p\begin{bmatrix}\sigma_2\\\sigma_g\end{bmatrix}+b_pM_2,
$$

and

$$
\boxed{\dot x_p=A_px_p+B_pM_2,\qquad
A_p=\begin{bmatrix}0_{2\times2}&I_2\\H_p^{-1}G_p&-H_p^{-1}D_p\end{bmatrix},\qquad
B_p=\begin{bmatrix}0_{2\times1}\\H_p^{-1}b_p\end{bmatrix}.}
$$

## 5. Remaining four states and controllability at rest

Define the linear coordinate combinations

$$
\xi=x_G-R\phi,\qquad \eta=y_G+R\theta,\qquad
x_e=\begin{bmatrix}\sigma_3&\psi&\xi&\eta\end{bmatrix}^{T}.
$$

The full linear equations give

$$
\dot\sigma_3=0,\qquad \dot\psi=\sigma_3,\qquad \dot\xi=0,\qquad \dot\eta=0.
$$

Hence, with $z=[x_\ell^T\;x_p^T\;x_e^T]^T=Tx$,

$$
\boxed{
\dot z=
\begin{bmatrix}A_\ell&0&0\\0&A_p&0\\0&0&A_e\end{bmatrix}z+
\begin{bmatrix}B_\ell&0\\0&B_p\\0_{4\times1}&0_{4\times1}\end{bmatrix}u,
\qquad
A_e=\begin{bmatrix}0&0&0&0\\1&0&0&0\\0&0&0&0\\0&0&0&0\end{bmatrix}.
}
$$

The transformation is invertible: $x_G=\xi+R\phi$ and $y_G=\eta-R\theta$, with all other states retained explicitly.

For the supplied full-model parameters,

$$
\operatorname{rank}\mathcal C(A_\ell,B_\ell)=4,\quad
\operatorname{rank}\mathcal C(A_p,B_p)=4,\quad
\boxed{\operatorname{rank}\mathcal C(A,B)=8.}
$$

The remaining four modes are uncontrollable and have zero eigenvalues. In particular, a nonzero yaw-rate perturbation gives $\psi(t)=\psi(0)+\sigma_3(0)t$: these zero modes do not imply asymptotic stability of the full state.

For separate feedback $F=-K_\ell x_\ell$ and $M_2=-K_px_p$,

$$
\operatorname{spec}(A-BK)=
\operatorname{spec}(A_\ell-B_\ell K_\ell)
\cup\operatorname{spec}(A_p-B_pK_p)
\cup\{0,0,0,0\}.
$$

Thus, full-state feedback at this stationary operating point cannot arbitrarily assign all 12 poles either. This is a statement about the linearization at rest, not a general statement about nonlinear maneuverability.

## 6. Exact comparison with the supplied partial lateral model

### 6.1 Parameter correspondence

The necessary parameter mapping is

| Partial-model symbol or field | Full-model symbol |
|---|---|
| $2m_L$ | $m_r$ |
| $m_B$ | $m_p$ |
| $m_W$ | $m_w$ |
| `par.I_b` | $J_{Px}$ |
| `par.I_rod` | $J_R$ |
| `par.I_w` | $J_{W2}$ |
| $R,h,g$ | The same $R,h,g$ |

These inertia correspondences require matching physical axes and reference points. The supplied partial matrix has no damping, so comparison with it uses $B_R=0$.

The partial script defines

$$
J_\theta=2m_LR^2+m_B(R+h)^2+m_WR^2+I_b+I_{\rm rod}+I_w.
$$

Under the mapping above,

$$
\boxed{J_\theta=I_\ell+m_rR^2.}
$$

Similarly, its gravity coefficients become

$$
G_{\theta1}=G_\ell+m_rgR,\qquad
G_{\theta2}=G_{r1}=-m_rg.
$$

### 6.2 Ordinary-velocity form

The state in the partial script is

$$
x_r=\begin{bmatrix}\theta&r&\dot\theta&\dot r\end{bmatrix}^{T}.
$$

With the above parameter mapping, its equations are exactly

$$
N\dot x_r=\widetilde A x_r+\widetilde B F,
$$

$$
N=\begin{bmatrix}
1&0&0&0\\0&1&0&0\\
0&0&I_\ell+m_rR^2&-m_rR\\
0&0&-m_rR&m_r
\end{bmatrix},\quad
\widetilde A=\begin{bmatrix}
0&0&1&0\\0&0&0&1\\
G_\ell+m_rgR&-m_rg&0&0\\
-m_rg&0&0&0
\end{bmatrix},\quad
\widetilde B=\begin{bmatrix}0\\0\\0\\1\end{bmatrix}.
$$

Thus $A_r=N^{-1}\widetilde A$ and $B_r=N^{-1}\widetilde B$.

In second-order notation this reads

$$
\begin{aligned}
(I_\ell+m_rR^2)\ddot\theta-m_rR\ddot r
&=(G_\ell+m_rgR)\theta-m_rgr,\\
-m_rR\ddot\theta+m_r\ddot r&=-m_rg\theta+F.
\end{aligned}
$$

The second equation gives

$$
\ddot r-R\ddot\theta=-g\theta+F/m_r=\dot\sigma_r.
$$

Adding $R$ times the second equation to the first gives

$$
I_\ell\ddot\theta=G_\ell\theta-m_rgr+RF,
$$

which is exactly the full-model lateral equation in Section 3 with $B_R=0$.

### 6.3 Similarity transformation and gain conversion

Define

$$
\boxed{x_\ell=Sx_r,\qquad
S=\begin{bmatrix}1&0&0&0\\0&1&0&0\\0&0&1&0\\0&0&-R&1\end{bmatrix}.}
$$

Then

$$
\boxed{A_\ell=SA_rS^{-1},\qquad B_\ell=SB_r.}
$$

This is an exact coordinate equivalence, stronger than merely matching eigenvalues.

For feedback gains, the corresponding relations are

$$
\boxed{K_r=K_\ell S,\qquad K_\ell=K_rS^{-1}.}
$$

For example, if

$$
K_r=\begin{bmatrix}k_\theta&k_r&k_{\dot\theta}&k_{\dot r}\end{bmatrix},
$$

then

$$
K_\ell=\begin{bmatrix}k_\theta&k_r&k_{\dot\theta}+Rk_{\dot r}&k_{\dot r}\end{bmatrix}.
$$

The closed-loop matrices therefore satisfy

$$
\boxed{A_\ell-B_\ell K_\ell=S(A_r-B_rK_r)S^{-1}.}
$$

With corresponding initial states and feedback, the lateral linear trajectories describe the same physical motion. Directly comparing the third gain without converting $\dot r$ to $\sigma_r$ would be incorrect.

## 7. Numerical verification with the full-model parameters

The following values are taken from the previously supplied full-model Markdown, not from the unavailable `LatParam.m`:

| Parameter | Value | Parameter | Value |
|---|---:|---|---:|
| $g$ | 9.81 | $R$ | 0.253 |
| $m_w$ | 2.436 | $m_r$ | 2.3 |
| $m_p$ | 2.799 | $h$ | 0.025 |
| $J_{W1}$ | 0.09099921839 | $J_{W2}$ | 0.04591427768 |
| $J_R$ | 0.0517629 | $J_{Px}$ | 0.01290418213 |
| $J_{Py}$ | 0.02090219895 | $J_{Pz}$ | 0.01118711607 |
| $B_R$ | 0 | $B_P$ | 0 |

All values are in SI units. They give

$$
I_\ell=0.48282519981,\qquad G_\ell=13.6793583,\qquad J_\theta=0.63004589981.
$$

The lateral matrices in pseudovelocities are

$$
A_\ell\approx\begin{bmatrix}
0&0&1&0\\
0&0&0.253&1\\
28.33190626&-46.73119798&0&0\\
-9.81&0&0&0
\end{bmatrix},\qquad
B_\ell\approx\begin{bmatrix}0\\0\\0.52399916\\0.43478261\end{bmatrix}.
$$

In the partial script's ordinary-velocity coordinates,

$$
A_r\approx\begin{bmatrix}
0&0&1&0\\
0&0&0&1\\
28.33190626&-46.73119798&0&0\\
-2.64202772&-11.82299309&0&0
\end{bmatrix},\qquad
B_r\approx\begin{bmatrix}0\\0\\0.52399916\\0.56735440\end{bmatrix}.
$$

The longitudinal matrices are

$$
A_p\approx\begin{bmatrix}
0&0&1&0\\
0&0&0&1\\
0&-0.95895846&0&0\\
0&31.05443536&0&0
\end{bmatrix},\qquad
B_p\approx\begin{bmatrix}0\\0\\3.18437704\\-46.63583990\end{bmatrix}.
$$

Numerically differentiating the actual scalar expressions in the supplied MATLAB model, evaluated in Python, agreed with the analytic full-model decomposition to a maximum entrywise error below $10^{-12}$. The partial-model similarity identities also agreed to below $10^{-12}$. Central differences independently agreed with complex-step differentiation to below $10^{-8}$. MATLAB itself was not available for execution.

## 8. Pole choices: paper, supplied script, and earlier example

| Source | Pole choice | Interpretation |
|---|---|---|
| Paper, Section VI, p. 9 | Repeated $\lambda_i=-12\;\mathrm{s}^{-1}$ | Reported control-design target, with linearization using instantaneous wheel pitch rate |
| Supplied `PolePlacement.m` | `P_desired = [-2.25, -1.25, -2.00, -1.50]` | Declared lateral targets; not actually used by the active gain assignment |
| Earlier generated script, lateral | `[-2, -2.5, -3, -3.5]` | Illustrative values, not paper values |
| Earlier generated script, longitudinal | `[-3, -3.5, -4, -4.5]` | Illustrative values, not paper values |

The supplied script currently executes

```matlab
K_numeric = [-805.909522, 900, -46.963696, 50]; % place(A, B, P_desired);
```

Therefore, changing `P_desired` alone does not change the gain. To use the declared distinct poles, the active assignment would need to be

```matlab
K_numeric = place(A, B, P_desired);
```

Using the full-model parameter values in Section 7, those declared poles give approximately

$$
K_r=\begin{bmatrix}-365.02774817&398.06803047&-56.21897016&64.26088078\end{bmatrix},
$$

or equivalently,

$$
K_\ell=\begin{bmatrix}-365.02774817&398.06803047&-39.96096732&64.26088078\end{bmatrix}.
$$

With those same parameter values, the hardcoded gain instead gives lateral poles approximately

$$
-1.32382672\pm8.00194550\,i,\qquad
-0.55556453\pm1.66731648\,i.
$$

These are conditional calculations: the actual poles in the user's local project depend on the values returned by `LatParam()`.

The paper's repeated target corresponds to $1/12\approx0.0833$ s as the exponential time scale; it is not a guarantee of that settling time, particularly with repeated modes. It also does not mean all 12 full-state roots are assignable: Section V of the paper explicitly discusses lack of full state controllability and selects control outputs.

Do not directly replace a four-state SISO `place` pole vector with `[-12,-12,-12,-12]`: MATLAB `place` rejects multiplicity greater than `rank(B)`, which is one for either four-state block here. This is an algorithm restriction, not an impossibility of repeated-pole assignment for a controllable SISO system. Exact repeated-pole design requires a suitable method, such as characteristic-polynomial matching; selecting nearby distinct roots is a different target. See the [MathWorks `place` documentation](https://www.mathworks.com/help/control/ref/place.html).

## 9. Why the paper's rolling linearization is different from the rest case

The paper linearizes around straight rolling at a constant wheel pitch rate and decomposes its 12-state model into a seven-state lateral subsystem and a five-state longitudinal subsystem; see Eqs. (60)-(64) and (71)-(72). Its state coordinates, actuator idealizations, and physical parameters are not identical to the supplied full-model implementation.

Even for the supplied implementation, the four-state lateral subsystem is not generally autonomous at nonzero forward speed. Let the nominal wheel rate be $\Omega=\sigma_{2*}\ne0$, with zero nominal lean, body tilt, rod displacement and yaw. Linearize about the straight-rolling trajectory, using deviations from $\phi_*(t)=\phi_0+\Omega t$ and $x_{G*}(t)=x_{G0}+R\Omega t$.

For example, direct differentiation of the supplied model gives

$$
\delta\dot\sigma_3=-\frac{J_{W1}}{c}\Omega\,\delta\sigma_1,
\qquad
\delta\dot\psi=\delta\sigma_3,
\qquad
\delta\dot\eta=R\Omega\,\delta\psi,
$$

and the lateral acceleration equations acquire the terms

$$
\delta\dot\sigma_1\supset
\frac{J_{W1}+R^2(m_p+m_w)+b}{I_\ell}\Omega\,\delta\sigma_3,
\qquad
\delta\dot\sigma_r\supset-R\Omega\,\delta\sigma_3.
$$

These couplings vanish at $\Omega=0$ but remain at nonzero speed. Thus the rest-point four-state equivalence does not establish equivalence of moving lateral/path-following dynamics. Lateral-longitudinal separation and reduction to only four lateral states are different claims.

The precise conclusion is: **the supplied partial lateral linearization matches the stationary full model's lateral block, under the stated parameter and coordinate mapping. It does not replace the entire 12-state system or establish equivalence at other operating conditions.**

## 10. Existing simulation-comparison inconsistencies in `PolePlacement.m`

These do not affect the matrix identity proved above, but they matter if that script is used to compare linear and nonlinear trajectories:

- `theta_eq = 0.1` and a corresponding `r_eq` are computed, but `z_eq` is subsequently set to the zero vector.
- The nonlinear controller and initial condition use that zero `z_eq`, whereas `delta_x0` subtracts the previously computed nonzero `theta_eq` and `r_eq`. The two simulations therefore start from different physical perturbations.
- `F_eq` is computed and plotted but is not included in the active controller.

For a comparison at the origin, use the same zero equilibrium and the same perturbation in both simulations. A comparison about a genuinely nonzero equilibrium requires an equilibrium-consistent feedforward input and a new Jacobian there; the origin linearization is not exact at that other point.

## Sources

1. Supplied full-model MATLAB functions and the parameter values in `unicycle_model.md`.
2. Supplied `PolePlacement.m`, especially Sections 1-4.
3. M. B. Vizi, D. Takacs, G. Stepan, and G. Orosz, *Integrating path-planning and control for robotic unicycles*, arXiv:2507.02700v1, supplied PDF. Section V, pp. 7-9, and Section VI, p. 9. Findings here refer specifically to this uploaded version.
4. [MathWorks: `place` — Pole placement design](https://www.mathworks.com/help/control/ref/place.html), especially the pole multiplicity restriction under input argument `p`.
