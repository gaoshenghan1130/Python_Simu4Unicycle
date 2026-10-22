# Lean and Rod Overshoot versus Speed

Updated: 2026-10-20. Use the states, input signs and equations in
[Full Model and Conventions](../full_model_conventions.md).
Equations use radians; reported angles use degrees.

**Data version:** the results below are from the original 66-case study
with target lean **0.1°**. The scripts now default to **0°**.
These historical results have not been recomputed for the new target.

## 1. Setup and overshoot definitions

For a chosen initial lean and speed,

$$
x_0=[0,v/R,0,0,0,0,\theta_0,0,0,0,0,0]^T.
$$

Each speed uses a newly matched turning reference and a newly calculated
LQR gain. The nonlinear simulation allows the speed to change.
Settings: $m_r=2.3$ kg, $B_R=6.5$ N·s/m, $B_P=0$,
$|F|\leq30$ N, $|M_2|\leq15$ N·m, and no rod-position limit.
For the nine-state relative model in the convention document,

$$
Q=\operatorname{diag}(2,1,1,2,1,0.5,20,20,5),\qquad
R_u=\operatorname{diag}(0.1,0.1).
$$

Let $s_\theta=\operatorname{sign}(\theta_0-\theta_e)$ and
$s_r=\operatorname{sign}(r_e-r_0)$. For nonzero signs, define

$$
O_\theta=\max\{0,\max_{0\leq t\leq T}s_\theta[\theta(t)-\theta_0]\},
\qquad
O_r=\max\{0,\max_{0\leq t\leq T}s_r[r(t)-r_e]\}.
$$

$O_\theta$ measures motion beyond the initial lean, not crossing the target.
The script separately records the **first outward peak**, maximum $|r|$,
and terminal errors. Rod target excess can include a terminal offset,
not just an oscillatory overshoot.

## 2. Exact initial acceleration

Define

$$
I_\theta=J_{Px}+J_R+J_{W2}+R^2(m_p+m_w)+2Rhm_p+h^2m_p,
\qquad
G_\theta=g[R(m_p+m_w)+hm_p].
$$

For the initial condition above, the full model gives exactly

$$
\boxed{\ddot\theta(0)=\frac{RF_0+G_\theta\sin\theta_0}{I_\theta}},
\qquad
\boxed{\ddot r(0)=R\ddot\theta(0)+F_0/m_r-g\sin\theta_0}.
$$

$F_0$ is the actual initial force after saturation. Speed enters through
the reference and feedback gain used to calculate $F_0$; the initial
gyroscopic terms vanish because $\sigma_3(0)=0$.

Since $\dot\theta(0)=0$,

$$\theta(t)=\theta_0+\tfrac12\ddot\theta(0)t^2+O(t^3)$$

If $s_\theta\ddot\theta(0)>0$, initial outward motion must occur.
A candidate speed for reversing this motion satisfies

$$
RF_0(v,\theta_0)+G_\theta\sin\theta_0=0.
$$

Reversing the initial motion does not rule out a later overshoot.

## 3. Linear response and speed expansion

Let $e=z-z_e$ and $J=A-BK$. The exact linear response is

$$
e_L(t)=\exp(Jt)e_0,\qquad y_L(t)=y_e+C_y\exp(Jt)e_0,
$$

where $C_y$ selects lean or rod position. Interior peak times satisfy

$$
C_yJ\exp(Jt_p)e_0=0.
$$

The response is analytic, but a general multimode peak time still needs a
scalar root solve. Check interval endpoints as well.

For nearby speeds, let $\delta=v-v_c$ and expand

$$
J(v)=J_0+\delta J_1+\delta^2J_2+O(\delta^3),\qquad
e_0(v)=b_0+\delta b_1+\delta^2b_2+O(\delta^3),
$$

where $J_1=J'(v_c)$ and $J_2=J''(v_c)/2$.
The response coefficients follow from

$$
\begin{bmatrix}\eta_0\\\eta_1\\\eta_2\end{bmatrix}
=\exp\left[t\begin{pmatrix}
J_0&0&0\\J_1&J_0&0\\J_2&J_1&J_0
\end{pmatrix}\right]
\begin{bmatrix}b_0\\b_1\\b_2\end{bmatrix},
\qquad e_L\approx\eta_0+\delta\eta_1+\delta^2\eta_2.
$$

Expand $y_e(v)$ too. If
$y(t,\delta)=y_0(t)+\delta y_1(t)+\delta^2y_2(t)$ has an isolated,
smooth maximum at $t_0$, with $y_0''(t_0)<0$, then

$$
y_{\rm peak}(\delta)\approx
y_0(t_0)+\delta y_1(t_0)+\delta^2
\left[y_2(t_0)-\frac{[y_1'(t_0)]^2}{2y_0''(t_0)}\right].
$$

This includes the change in peak time. It is local and can fail when a
different peak becomes dominant or the maximum occurs at an endpoint.

## 4. Second-order nonlinear correction

For a fixed design, write the smooth closed-loop field as

$$
\dot e=Je+\tfrac12H[e,e]+O(\|e\|^3).
$$

The first two perturbation terms satisfy

$$
\dot e_1=Je_1,\quad e_1(0)=e_0,\qquad
\dot e_2=Je_2+\tfrac12H[e_1,e_1],\quad e_2(0)=0.
$$

Lifting the 45 distinct quadratic monomials of nine states gives a
54-dimensional linear system. Its matrix exponential evaluates $e_1+e_2$.
This is a second-order approximation, not an exact nonlinear solution.
It improves some early peaks but can produce spurious later peaks for
large disturbances. Saturation and heading wrapping also limit this method.

If $w^TJ=0$, then

$$
\frac{d}{dt}(w^Te)=\tfrac12w^TH[e,e]+O(\|e\|^3).
$$

Thus, matching the linear neutral mode does not guarantee zero terminal
error in the nonlinear simulation.

## 5. Original numerical results

All 66 cases completed 25 s without actuator saturation.
For $\theta_0=5^\circ$ and **target $\theta_e=0.1^\circ$**:

| Initial speed (m/s) | First lean peak (deg) | Peak above initial lean (deg) | Maximum rod displacement (m) |
|---:|---:|---:|---:|
| 1.5 | 18.826 | 13.826 | 0.15146 |
| 2.0 | 12.496 | 7.496 | 0.10071 |
| 2.5 | 9.892 | 4.892 | 0.12916 |
| 4.0 | 7.180 | 2.180 | 0.20774 |
| 6.0 | 6.190 | 1.190 | 0.28946 |
| 10.0 | 5.584 | 0.584 | 0.41767 |

Higher speed reduced the first lean peak on the tested grid, but did not
eliminate it. Rod displacement decreased initially, then increased.
At higher speeds, rod target excess was mainly a terminal offset.

For initial lean 0.25°, the maximum relative first-peak-angle error was
about 1.49% for the linear model. For 5°, it was 16.82% for the linear
model and 10.20% for the second-order model. The denominator is peak
angle, not overshoot.

Use matrix exponentials and the speed expansion to explain local trends.
Use full nonlinear simulations for large-disturbance performance.
No general closed-form nonlinear overshoot formula has been established.

## 6. Run the study

From `MATLAB/Full`:

```matlab
% Current default: target lean = 0 degrees
study = run_manifold_overshoot_study(5,[1.5 2 2.5 4 6]);

% Reproduce the historical 0.1-degree target study
options = struct('target_deg',0.1);
study = run_manifold_overshoot_study( ...
    [0.25 1 5],[1.5:0.25:6 7 8 10],options);
```

Implementation: `analysis/manifold/study_manifold_overshoot_vs_speed.m`
and `manifold_local_design.m`.
Outputs: `results/manifold_overshoot_vs_speed/`, including peak metrics,
speed-series checks, local peak polynomials, MAT data and PNG figures.
Each run overwrites the corresponding output files.
Stopped runs describe only the trajectory before termination.
