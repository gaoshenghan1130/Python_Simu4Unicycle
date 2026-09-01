# CBF Regions for Rod Motion

Assume the simplified flat-rod model

$$
m_r\ddot r=F,
\qquad
|F|\le F_{\max},
\qquad
|r|\le r_{\max}.
$$

Let

$$
v=\dot r,
\qquad
a_{\max}=\frac{F_{\max}}{m_r}.
$$

## Physical Position Limits — Red Dashed Lines

The mechanical travel constraint is

$$
-r_{\max}\le r\le r_{\max}.
$$

The two red vertical dashed lines in the phase-plane plot represent these limits.

## Force-Limited Braking Region — Orange Region

When $v>0$, the minimum distance required to stop the rod is

$$
d_{\mathrm{stop}}
=
\frac{v^2}{2a_{\max}}.
$$

Avoiding the positive limit therefore requires

$$
v^2
\le
2a_{\max}(r_{\max}-r).
$$

Similarly, when $v<0$, avoiding the negative limit requires

$$
v^2
\le
2a_{\max}(r_{\max}+r).
$$

Thus, the force-limited braking region is

$$
\mathcal C_{\mathrm{brake}}
=
\left\{
(r,v):
-\sqrt{2a_{\max}(r_{\max}+r)}
\le v\le
\sqrt{2a_{\max}(r_{\max}-r)}
\right\}.
$$

This is the orange region in the phase-plane plot. It is the largest region from which the rod can still be stopped before reaching a mechanical limit using $|F|\le F_{\max}$.

## First-Level HOCBF Region — Green Region

The first-level HOCBF conditions are

$$
-v+\alpha_0(r_{\max}-r)\ge0,
$$

$$
v+\alpha_0(r_{\max}+r)\ge0.
$$

Equivalently,

$$
|v+\alpha_0r|
\le
\alpha_0r_{\max}.
$$

Therefore,

$$
\mathcal C_1
=
\left\{
(r,v):
|r|\le r_{\max},
\quad
|v+\alpha_0r|
\le\alpha_0r_{\max}
\right\}.
$$

This is the green parallelogram in the phase-plane plot. The two magenta dashed lines are its upper and lower velocity boundaries.

## Second-Order HOCBF Force Bounds

Let

$$
s=\alpha_0+\alpha_1,
\qquad
p=\alpha_0\alpha_1.
$$

The two HOCBF constraints give

$$
F_{\mathrm{CBF},-}
=
m_r
\left[
-sv-p(r_{\max}+r)
\right],
$$

$$
F_{\mathrm{CBF},+}
=
m_r
\left[
-sv+p(r_{\max}-r)
\right].
$$

Therefore, the admissible force must satisfy

$$
F_{\mathrm{CBF},-}
\le F\le
F_{\mathrm{CBF},+}.
$$

Including the actuator limit gives

$$
F_{\mathrm{lower}}
=
\max
\left(
F_{\mathrm{CBF},-},
-F_{\max}
\right),
$$

$$
F_{\mathrm{upper}}
=
\min
\left(
F_{\mathrm{CBF},+},
F_{\max}
\right).
$$

The applied force is

$$
F^*
=
\operatorname{clip}
\left(
F_{\mathrm{nom}},
F_{\mathrm{lower}},
F_{\mathrm{upper}}
\right).
$$

In the force plot, the magenta dotted curve is $F_{\mathrm{CBF},-}$ and the cyan dotted curve is $F_{\mathrm{CBF},+}$.
## HOCBF-Feasible Region Under the Force Limit

Let

$$
s=\alpha_0+\alpha_1,
\qquad
p=\alpha_0\alpha_1,
\qquad
a_{\max}=\frac{F_{\max}}{m_r}.
$$

A force satisfying both the HOCBF constraints and actuator limit exists only if

$$
\max(F_{\mathrm{CBF},-},-F_{\max})
\le
\min(F_{\mathrm{CBF},+},F_{\max}).
$$

Since

$$
F_{\mathrm{CBF},-}
\le
F_{\mathrm{CBF},+}
$$

automatically, feasibility reduces to

$$
F_{\mathrm{CBF},-}\le F_{\max},
$$

$$
F_{\mathrm{CBF},+}\ge-F_{\max}.
$$

Substituting the HOCBF force bounds and simplifying gives

$$
-a_{\max}-pr_{\max}
\le
sv+pr
\le
a_{\max}+pr_{\max},
$$

or equivalently,

$$
|sv+pr|
\le
a_{\max}+pr_{\max}.
$$

Together with the first-level HOCBF conditions, the controller-independent feasible region is

$$
\boxed{
\mathcal C_{\mathrm{feas}}
=
\left\{
(r,v):
\begin{aligned}
|r|&\le r_{\max},\\
|v+\alpha_0r|&\le\alpha_0r_{\max},\\
|sv+pr|&\le a_{\max}+pr_{\max}
\end{aligned}
\right\}.
}
$$

This region means that at least one force satisfying

$$
|F|\le F_{\max}
$$

and both HOCBF constraints exists. It does not depend on the nominal controller.

## Region Where a PD Controller Requires No CBF Intervention

Assume that the nominal controller is

$$
F_{\mathrm{PD}}
=
K_p(r_{\mathrm{ref}}-r)-K_dv.
$$

The CBF does not modify the PD force whenever

$$
F_{\mathrm{CBF},-}
\le
F_{\mathrm{PD}}
\le
F_{\mathrm{CBF},+}
$$

and

$$
|F_{\mathrm{PD}}|\le F_{\max}.
$$

The lower HOCBF condition

$$
F_{\mathrm{PD}}\ge F_{\mathrm{CBF},-}
$$

becomes

$$
(m_rp-K_p)r
+
(m_rs-K_d)v
+
m_rpr_{\max}
+
K_pr_{\mathrm{ref}}
\ge0.
$$

The upper HOCBF condition

$$
F_{\mathrm{PD}}\le F_{\mathrm{CBF},+}
$$

becomes

$$
(K_p-m_rp)r
+
(K_d-m_rs)v
+
m_rpr_{\max}
-
K_pr_{\mathrm{ref}}
\ge0.
$$

The actuator-force constraint gives

$$
\left|
K_p(r_{\mathrm{ref}}-r)-K_dv
\right|
\le F_{\max}.
$$

Therefore, the complete PD no-intervention region is

$$
\boxed{
\mathcal C_{\mathrm{PD}}
=
\left\{
(r,v):
\begin{aligned}
|r|&\le r_{\max},\\
|v+\alpha_0r|&\le\alpha_0r_{\max},\\
|F_{\mathrm{PD}}|&\le F_{\max},\\
F_{\mathrm{CBF},-}
&\le F_{\mathrm{PD}}
\le F_{\mathrm{CBF},+}
\end{aligned}
\right\}.
}
$$

For the special case

$$
r_{\mathrm{ref}}=0,
$$

the PD controller becomes

$$
F_{\mathrm{PD}}=-K_pr-K_dv.
$$

The two HOCBF inequalities can then be combined as

$$
\left|
(m_rp-K_p)r
+
(m_rs-K_d)v
\right|
\le
m_rpr_{\max}.
$$

The force limit becomes

$$
|K_pr+K_dv|
\le F_{\max}.
$$

Thus, for $r_{\mathrm{ref}}=0$,

$$
\boxed{
\mathcal C_{\mathrm{PD}}
=
\left\{
(r,v):
\begin{aligned}
|r|&\le r_{\max},\\
|v+\alpha_0r|&\le\alpha_0r_{\max},\\
|sv+pr|&\le a_{\max}+pr_{\max},\\
|K_pr+K_dv|&\le F_{\max},\\
|(m_rp-K_p)r+(m_rs-K_d)v|
&\le m_rpr_{\max}
\end{aligned}
\right\}.
}
$$

The state space can therefore be separated into three regions:

1. Inside $\mathcal C_{\mathrm{PD}}$, the PD force already satisfies all constraints, so

   $$
   F^*=F_{\mathrm{PD}}.
   $$

2. Inside $\mathcal C_{\mathrm{feas}}$ but outside $\mathcal C_{\mathrm{PD}}$, a safe force exists, but the PD force is unsafe. The CBF modifies the PD force.

3. Outside $\mathcal C_{\mathrm{feas}}$, no force satisfying $|F|\le F_{\max}$ can satisfy the selected HOCBF conditions.

In the current phase-plane figure, the green region is the first-level HOCBF set, while the orange region is the force-limited braking region. The exact set $\mathcal C_{\mathrm{PD}}$ is an intersection of linear inequalities and is therefore generally a polygon.

The black dash-dot ellipse is a conservative invariant subset of $\mathcal C_{\mathrm{PD}}$. Inside this ellipse, the nominal PD controller is guaranteed to remain admissible without CBF intervention. It does not represent the complete PD no-intervention region.

## PD Lyapunov Invariant Ellipse — Black Dash-Dot Curve

Define the PD tracking-error state as

$$
e=
\begin{bmatrix}
r-r_{\mathrm{ref}}\\
v
\end{bmatrix}.
$$

Before actuator saturation or CBF intervention, the PD force is

$$
F_{\mathrm{PD}}
=
-K_pe_1-K_de_2.
$$

Therefore, the nominal closed-loop dynamics are

$$
\dot e=A_{\mathrm{cl}}e,
$$

where

$$
A_{\mathrm{cl}}
=
\begin{bmatrix}
0 & 1\\
-\dfrac{K_p}{m_r} & -\dfrac{K_d}{m_r}
\end{bmatrix}.
$$

When $K_p>0$ and $K_d>0$. Choose any matrix
$Q=Q^\mathsf T>0$ and solve the Lyapunov equation

$$
A_{\mathrm{cl}}^\mathsf TP
+
PA_{\mathrm{cl}}
=
-Q.
$$

This gives $P=P^\mathsf T>0$. Define

$$
V(e)=e^\mathsf TPe.
$$

Along the unsaturated PD closed-loop dynamics,

$$
\dot V
=
e^\mathsf T
\left(
A_{\mathrm{cl}}^\mathsf TP
+
PA_{\mathrm{cl}}
\right)e
=
-e^\mathsf TQe
\le0.
$$

Consequently, every Lyapunov sublevel set

$$
\mathcal E_\rho
=
\left\{
(r,v):
\begin{bmatrix}
r-r_{\mathrm{ref}}\\
v
\end{bmatrix}^{\mathsf T}
P
\begin{bmatrix}
r-r_{\mathrm{ref}}\\
v
\end{bmatrix}
\le\rho
\right\}
$$

is invariant under the nominal PD dynamics.

To guarantee that neither the actuator saturation nor the CBF modifies the
PD force, the ellipse must satisfy

$$
\mathcal E_\rho
\subseteq
\mathcal C_{\mathrm{PD}}.
$$

For an affine constraint written in the error coordinates as

$$
c_i+d_i^\mathsf Te\ge0,
$$

its minimum over $\mathcal E_\rho$ is

$$
\min_{e^\mathsf TPe\le\rho}
\left(c_i+d_i^\mathsf Te\right)
=
c_i-
\sqrt{\rho\,d_i^\mathsf TP^{-1}d_i}.
$$

Therefore, the entire ellipse satisfies this constraint if

$$
\rho
\le
\rho_i
=
\frac{c_i^2}
{d_i^\mathsf TP^{-1}d_i}.
$$

The relevant affine constraints are the position limits,

$$
r_{\max}-r_{\mathrm{ref}}-e_1\ge0,
$$

$$
r_{\max}+r_{\mathrm{ref}}+e_1\ge0,
$$

the actuator-force limits,

$$
\left|K_pe_1+K_de_2\right|
\le F_{\max},
$$

the first-level HOCBF constraints,

$$
\alpha_0(r_{\max}-r_{\mathrm{ref}})
-
\begin{bmatrix}
\alpha_0 & 1
\end{bmatrix}e
\ge0,
$$

$$
\alpha_0(r_{\max}+r_{\mathrm{ref}})
+
\begin{bmatrix}
\alpha_0 & 1
\end{bmatrix}e
\ge0,
$$

and the two second-order HOCBF conditions. Let

$$
s=\alpha_0+\alpha_1,
\qquad
p=\alpha_0\alpha_1,
$$

and define

$$
d_{\mathrm{CBF}}
=
\begin{bmatrix}
m_rp-K_p\\
m_rs-K_d
\end{bmatrix}.
$$

The second-order conditions under the raw PD force become

$$
m_rp(r_{\max}+r_{\mathrm{ref}})
+
d_{\mathrm{CBF}}^\mathsf Te
\ge0,
$$

$$
m_rp(r_{\max}-r_{\mathrm{ref}})
-
d_{\mathrm{CBF}}^\mathsf Te
\ge0.
$$

The ellipse used in the figure is obtained from

$$
\boxed{
\rho^\star
=
\min_i \rho_i
}
$$

over all the constraints above. Hence,

$$
\boxed{
\mathcal E_{\rho^\star}
\subseteq
\mathcal C_{\mathrm{PD}}.
}
$$

Inside this ellipse, the nominal PD controller remains inside the ellipse and
satisfies the position, actuator-force, first-level HOCBF, and second-order
HOCBF constraints. Therefore,

$$
F^\star=F_{\mathrm{PD}}
$$

without CBF intervention.

The black dash-dot curve in the phase-plane plot is the boundary of
$\mathcal E_{\rho^\star}$. It is centered at $(r_{\mathrm{ref}},0)$, so it is
centered at the origin only when $r_{\mathrm{ref}}=0$.

This is the largest admissible Lyapunov sublevel set for the selected matrix
$P$. It is generally a conservative subset of the complete polygonal region
$\mathcal C_{\mathrm{PD}}$ and is not necessarily the globally
maximum-volume invariant ellipse.

## Maximum-Force CBF Recovery Region — Dark-Teal Boundary

To test the CBF against an aggressive nominal command, assume that the
nominal controller continuously requests the maximum positive force

$$
F_{\mathrm{nom}}(t)=F_{\max}.
$$

This does **not** mean that the force physically applied to the rod is fixed
at $F_{\max}$. The CBF safety filter is still allowed to modify the command.
The applied force is

$$
F_{+}^{*}(r,v)
=
\operatorname{clip}
\left(
F_{\max},
F_{\mathrm{lower}}(r,v),
F_{\mathrm{upper}}(r,v)
\right).
$$

Whenever the HOCBF force interval is feasible, this simplifies to

$$
F_{+}^{*}(r,v)
=
\min
\left(
F_{\max},
F_{\mathrm{CBF},+}(r,v)
\right),
$$

because the nominal command is already at the positive actuator limit. The
resulting closed-loop dynamics are

$$
\dot r=v,
\qquad
\dot v=\frac{F_{+}^{*}(r,v)}{m_r}.
$$

Let

$$
x_{+}(t;x_0)
=
\begin{bmatrix}
r_{+}(t;x_0)\\
v_{+}(t;x_0)
\end{bmatrix}
$$

denote the trajectory starting from $x_0=(r_0,v_0)$. For a selected test
horizon $T$, define the positive maximum-force CBF recovery region as

$$
\boxed{
\mathcal R_{+}^{T}
=
\left\{
x_0\in\mathcal C_{\mathrm{brake}}:
\begin{aligned}
|r_{+}(t;x_0)|&\le r_{\max},
&&\forall t\in[0,T],\\
F_{\mathrm{lower}}(x_{+}(t;x_0))
&\le F_{\mathrm{upper}}(x_{+}(t;x_0)),
&&\forall t\in[0,T],\\
x_{+}(T;x_0)&\in\mathcal C_{\mathrm{feas}}
\end{aligned}
\right\}.
}
$$

Thus, an initial condition belongs to $\mathcal R_{+}^{T}$ when the CBF can
successfully override a continuous $+F_{\max}$ nominal request, prevent the
rod from crossing either mechanical limit, keep the HOCBF-QP feasible, and
bring the state into the HOCBF-feasible region by time $T$.

The set $\mathcal R_{+}^{T}$ is computed numerically in the MATLAB script:

1. A grid of initial states is generated over the force-limited braking
   region $\mathcal C_{\mathrm{brake}}$.
2. Each grid state is propagated under the CBF-filtered command
   $F_{\mathrm{nom}}=+F_{\max}$.
3. A state is rejected if the rod crosses $|r|=r_{\max}$ or if the admissible
   force interval becomes empty at any integration step.
4. The final state must satisfy the first-level and second-order HOCBF
   feasibility conditions.

The dark-teal contour in the phase-plane plot is the numerically calculated
boundary of $\mathcal R_{+}^{T}$. Because this set is obtained using a finite
grid, a finite integration step, and a finite horizon, it is a numerical
approximation rather than an analytic forward-invariance certificate.

For the symmetric flat-rod model, the corresponding region for a constant
negative maximum-force request is the reflection

$$
\mathcal R_{-}^{T}
=
\left\{
(-r,-v):(r,v)\in\mathcal R_{+}^{T}
\right\}.
$$

If the nominal force may remain at either extreme with an unknown fixed sign,
a more conservative initial set is

$$
\mathcal R_{\pm}^{T}
=
\mathcal R_{+}^{T}
\cap
\mathcal R_{-}^{T}.
$$

If the **applied** force were instead physically fixed at $+F_{\max}$ and the
CBF were not allowed to modify it, then

$$
r(t)
=
r_0+v_0t+\frac{1}{2}a_{\max}t^2.
$$

Consequently, $r(t)\rightarrow+\infty$ as $t\rightarrow\infty$, and no
nonempty infinite-horizon safe set can exist for finite rod travel limits.
