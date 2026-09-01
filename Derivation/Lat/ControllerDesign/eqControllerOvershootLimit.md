# Equilibrium Controller Design: Overshoot Limit

## Reduced Lateral Model

The reduced lateral dynamics are

$$
\begin{bmatrix}
m_wR^2+m_{\mathrm{rod}}r^2+m_p(R+h)^2
+I_w+I_b+I_{\mathrm{rod}} & 0\\
0 & m_{\mathrm{rod}}
\end{bmatrix}
\begin{bmatrix}
\dot u_1\\
\dot u_2
\end{bmatrix}
=
\begin{bmatrix}
FR-m_{\mathrm{rod}}gr\cos\theta
+m_wgR\sin\theta
+m_pg(R+h)\sin\theta
-m_{\mathrm{rod}}r(2u_2u_1+Ru_1^2)\\
F-m_{\mathrm{rod}}g\sin\theta
+m_{\mathrm{rod}}ru_1^2
\end{bmatrix}.
$$

Define

$$
m=m_{\mathrm{rod}},
$$

$$
J(r)=J_0+mr^2,
\qquad
J_0=m_wR^2+m_p(R+h)^2+I_w+I_b+I_{\mathrm{rod}},
$$

and

$$
G=m_wgR+m_pg(R+h).
$$

The equations of motion can then be written as

$$
J(r)\dot u_1
=
FR-mgr\cos\theta+G\sin\theta
-mr(2u_2u_1+Ru_1^2),
$$

$$
m\dot u_2
=
F-mg\sin\theta+mru_1^2,
$$

where

$$
u_1=\dot\theta,
\qquad
u_2=\dot r-R\dot\theta.
$$

## Equilibrium Curve

At equilibrium,

$$
u_1=u_2=\dot u_1=\dot u_2=0.
$$

The second equation gives

$$
F_{\mathrm{eq}}=mg\sin\theta.
$$

Substituting this result into the first equation gives

$$
mgR\sin\theta-mgr\cos\theta+G\sin\theta=0.
$$

Therefore, the equilibrium curve is

$$
r_{\mathrm{eq}}(\theta)
=
\frac{G+mgR}{mg}\tan\theta.
$$

Define

$$
a=\frac{G+mgR}{mg},
$$

so that

$$
r_{\mathrm{eq}}(\theta)=a\tan\theta.
$$

## Force-Dominated Local Direction

Define the force-independent terms

$$
\Phi_\theta
=
-mgr\cos\theta+G\sin\theta
-mr(2u_2u_1+Ru_1^2),
$$

and

$$
\Phi_2=-mg\sin\theta+mru_1^2.
$$

The angular acceleration is

$$
\ddot\theta
=
\frac{RF+\Phi_\theta}{J(r)}.
$$

Because

$$
\dot r=u_2+Ru_1,
$$

the rod acceleration is

$$
\ddot r
=
\frac{F+\Phi_2}{m}
+
R\frac{RF+\Phi_\theta}{J(r)}.
$$

For an initially stationary state,

$$
\dot\theta(0)=\dot r(0)=0.
$$

The local displacement ratio is therefore equal to the corresponding acceleration ratio:

$$
\lim_{t\rightarrow0^+}
\frac{r(t)-r_0}{\theta(t)-\theta_0}
=
\frac{\ddot r(0)}{\ddot\theta(0)}.
$$

This follows directly from the second-order local expansion of the motion, or equivalently by applying L'Hôpital's rule twice.

Now let the force magnitude become large while $\Phi_\theta$ and $\Phi_2$ remain bounded relative to $F$. Then

$$
\begin{aligned}
\lim_{|F|\rightarrow\infty}
\frac{\ddot r}{\ddot\theta}
&=
\frac{
\dfrac{1}{m}+\dfrac{R^2}{J(r)}
}{
\dfrac{R}{J(r)}
}\\
&=
\frac{J(r)+mR^2}{mR}.
\end{aligned}
$$

Therefore, the force-dominated local displacement direction is

$$
\boxed{
\lim_{|F|\rightarrow\infty}
\frac{\mathrm dr}{\mathrm d\theta}
=
\frac{J(r)+mR^2}{mR}
}.
$$

Equivalently,

$$
\boxed{
\lim_{|F|\rightarrow\infty}
\frac{\mathrm d\theta}{\mathrm dr}
=
\frac{mR}{J(r)+mR^2}
}.
$$

This is a local asymptotic result. It determines the direction approached by an initially stationary system under a sufficiently large force, but it does not by itself prove that an arbitrary finite-force trajectory follows this direction over a finite displacement.

## Force-Dominated Limiting Curve

To extend the local force-dominated direction to a finite trajectory, assume that the trajectory remains close to the local high-force direction throughout the considered monotone motion. Under this approximation,

$$
\frac{\mathrm d\theta}{\mathrm dr}
\approx
\frac{mR}{J(r)+mR^2}.
$$

Define

$$
\ell^2=R^2+\frac{J_0}{m}.
$$

Since

$$
J(r)+mR^2=m(r^2+\ell^2),
$$

the approximate trajectory equation becomes

$$
\frac{\mathrm d\theta}{\mathrm dr}
\approx
\frac{R}{r^2+\ell^2}.
$$

Integrating from $(\theta_0,r_0)$ gives

$$
\boxed{
\theta_{\mathrm{lim}}(r)
=
\theta_0+
\frac{R}{\ell}
\left[
\tan^{-1}\left(\frac{r}{\ell}\right)
-
\tan^{-1}\left(\frac{r_0}{\ell}\right)
\right]
}.
$$

This is an asymptotic force-dominated curve, not the exact trajectory of an arbitrary finite-force controller.

Near $r=0$,

$$
\frac{R}{r^2+\ell^2}
=
\frac{R}{\ell^2}+O(r^2).
$$

Therefore, the limiting trajectory is approximately linear:

$$
\theta_{\mathrm{lim}}(r)
\approx
\theta_0+k_0(r-r_0),
$$

where

$$
\boxed{
k_0
=
\frac{R}{\ell^2}
=
\frac{mR}{J_0+mR^2}
}.
$$

The absence of a term proportional to $r$ in the slope expansion explains why the simulated trajectory can appear nearly straight near $r=0$.

## Estimated Intersection with the Equilibrium Curve

The force-dominated limiting curve reaches the equilibrium curve when

$$
r_*=a\tan\theta_*.
$$

Substitution into the limiting curve gives

$$
\boxed{
\theta_*
=
\theta_0+
\frac{R}{\ell}
\left[
\tan^{-1}\left(
\frac{a\tan\theta_*}{\ell}
\right)
-
\tan^{-1}\left(\frac{r_0}{\ell}\right)
\right]
}.
$$

This scalar nonlinear equation can be solved numerically for $\theta_*$. It estimates the first intersection in the high-force limit.

Under the small-angle and locally linear approximations,

$$
\tan\theta\approx\theta,
$$

$$
\theta_{\mathrm{lim}}(r)
\approx
\theta_0+k_0(r-r_0).
$$

Using $r_*\approx a\theta_*$ gives

$$
\theta_*
\approx
\theta_0+k_0(a\theta_*-r_0).
$$

Therefore,

$$
\boxed{
\theta_*
\approx
\frac{\theta_0-k_0r_0}{1-ak_0}
}.
$$

For $r_0=0$,

$$
\boxed{
\frac{\theta_*}{\theta_0}
\approx
\frac{1}{1-ak_0}
}.
$$

## Numerical Evaluation

Using the current model parameters gives

$$
J_0=0.4828252\ \mathrm{kg\,m^2},
\qquad
a=0.8592739\ \mathrm{m},
\qquad
\ell=0.5233861\ \mathrm{m}.
$$

For

$$
\theta_0=0.1\ \mathrm{rad},
\qquad
r_0=0,
$$

define the angular excursion ratio

$$
\rho=\frac{\theta_*}{\theta_0}.
$$

The nonlinear intersection equation becomes

$$
\rho
=
1+
\frac{R}{\ell\theta_0}
\tan^{-1}
\left[
\frac{a}{\ell}
\tan(\theta_0\rho)
\right].
$$

Substituting the numerical values gives

$$
\rho
=
1+
4.8339076
\tan^{-1}
\left[
1.6417592\tan(0.1\rho)
\right].
$$

Solving this scalar equation numerically gives

$$
\boxed{
\frac{\theta_*}{\theta_0}
=
3.815680
}.
$$