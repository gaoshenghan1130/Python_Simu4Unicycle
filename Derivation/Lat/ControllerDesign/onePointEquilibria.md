## Nonzero Closed-Loop Equilibria Under Linear State Feedback

The open-loop static equilibrium curve is

$$
F^*
=
m_{\mathrm{rod}}g\sin\theta^*,
$$

and

$$
r^*
=
\frac{
G+m_{\mathrm{rod}}gR
}{
m_{\mathrm{rod}}g
}
\tan\theta^*.
$$

Define

$$
a_{\mathrm{eq}}
=
\frac{
G+m_{\mathrm{rod}}gR
}{
m_{\mathrm{rod}}g
}.
$$

The equilibrium curve can then be written as

$$
r^*
=
a_{\mathrm{eq}}\tan\theta^*.
$$

Consider the linear state-feedback controller

$$
F
=
-K_\theta\theta
-K_rr
-K_{\sigma_1}\sigma_1
-K_{\sigma_2}\sigma_2.
$$

At a static equilibrium,

$$
\sigma_1^*=0,
\qquad
\sigma_2^*=0,
$$

so the derivative gains do not affect the equilibrium location. The
controller force at equilibrium is therefore

$$
F^*
=
-K_\theta\theta^*
-K_rr^*.
$$

Equating the controller force with the force required by the mechanical
equilibrium gives

$$
-K_\theta\theta^*
-K_rr^*
=
m_{\mathrm{rod}}g\sin\theta^*.
$$

Substituting

$$
r^*
=
a_{\mathrm{eq}}\tan\theta^*
$$

gives the closed-loop equilibrium equation

$$
\boxed{
K_\theta\theta^*
+
K_ra_{\mathrm{eq}}\tan\theta^*
+
m_{\mathrm{rod}}g\sin\theta^*
=
0
}.
$$

Define

$$
f(\theta^*) = K_\theta\theta^* + K_ra_{\mathrm{eq}}\tan\theta^* + m_{\mathrm{rod}}g\sin\theta^*.
$$

We want it to have another root on $\theta^* \in (0, \pi/2)$. And we know that $f(0) = 0$ and:

$$
f(\theta^*)' = K_\theta + K_ra_{\mathrm{eq}}\sec^2\theta^* + m_{\mathrm{rod}}g\cos\theta^*
$$

There should be a pole $\theta_p$ such that $f(\theta_p)' = 0$, and the sufficient condition for existence of a nonzero route will be:

$$
f(\theta_p) < 0 \\
$$

Thus:

$$
K_\theta\theta_p + K_ra_{\mathrm{eq}}\tan\theta_p + m_{\mathrm{rod}}g\sin\theta_p < 0 \\
K_\theta + K_ra_{\mathrm{eq}}\sec^2\theta_p + m_{\mathrm{rod}}g\cos\theta_p = 0
$$

We consider the boundary condition where the first equation is satisfied with equality. Then we can solve for $K_r$ and $K_\theta$:

$$
K_\theta\theta_p + K_ra_{\mathrm{eq}}\tan\theta_p + m_{\mathrm{rod}}g\sin\theta_p = 0 \\
K_\theta + K_ra_{\mathrm{eq}}\sec^2\theta_p + m_{\mathrm{rod}}g\cos\theta_p = 0
$$