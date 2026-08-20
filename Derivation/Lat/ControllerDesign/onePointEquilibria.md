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

To make more sense of this equation:

$$
K_\theta\theta^* = -K_ra_{\mathrm{eq}}\tan\theta^* - m_{\mathrm{rod}}g\sin\theta^*.
$$