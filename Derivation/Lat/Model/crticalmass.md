
## Critical Moving Mass for the Eigenvector Sign Change

At the upright equilibrium, the position components of the two real
eigenvectors satisfy

$$
\frac{\delta r}{\delta\theta}
=
R-\frac{g}{\lambda_{\mathrm{r}}^2},
$$

where $\lambda_{\mathrm{r}}$ denotes either real eigenvalue. The signs of
$\delta r$ and $\delta\theta$ change from opposite to identical when

$$
\frac{\delta r}{\delta\theta}=0.
$$

Therefore, the critical condition is

$$
\lambda_{\mathrm{r}}^2=\frac{g}{R}.
$$

At the upright equilibrium, the characteristic equation is

$$
\lambda^4
-\frac{G-m_{\mathrm{rod}}gR}{J}
\lambda^2
-\frac{m_{\mathrm{rod}}g^2}{J}
=0.
$$

Substituting

$$
\lambda^2=\frac{g}{R}
$$

into the characteristic equation gives

$$
\frac{g^2}{R^2}
-\frac{G-m_{\mathrm{rod}}gR}{J}
\frac{g}{R}
-\frac{m_{\mathrm{rod}}g^2}{J}
=0.
$$

Expanding the second term yields

$$
\frac{g^2}{R^2}
-\frac{gG}{JR}
+\frac{m_{\mathrm{rod}}g^2}{J}
-\frac{m_{\mathrm{rod}}g^2}{J}
=0.
$$

The two terms containing $m_{\mathrm{rod}}$ cancel, leaving

$$
\frac{g^2}{R^2}
-\frac{gG}{JR}
=0.
$$

Thus, the eigenvector position ratio changes sign when

$$
\boxed{
J_{\mathrm{crit}}=\frac{GR}{g}
}.
$$

Consequently,

$$
\frac{\delta r}{\delta\theta}>0
\quad\Longleftrightarrow\quad
GR>gJ,
$$

whereas

$$
\frac{\delta r}{\delta\theta}<0
\quad\Longleftrightarrow\quad
GR<gJ.
$$

The moving mass does not appear explicitly in this condition. It can affect
the sign only by changing the rod inertia and hence the total lean inertia
$J$.

For the present design, let

$$
m\equiv m_{\mathrm{rod}}.
$$

The bare rod has mass $m_0$, while the remaining mass is divided equally
between the two ends. Therefore, the mass attached to each end is

$$
m_{\mathrm{end}}
=
\frac{m-m_0}{2}.
$$

The rod-assembly inertia is

$$
I_{\mathrm{rod}}(m)
=
\frac{1}{12}m_0L_{\mathrm{rod}}^2
+
2m_{\mathrm{end}}\ell_{\mathrm{end}}^2.
$$

Substituting the expression for $m_{\mathrm{end}}$ gives

$$
I_{\mathrm{rod}}(m)
=
\frac{1}{12}m_0L_{\mathrm{rod}}^2
+
(m-m_0)\ell_{\mathrm{end}}^2.
$$

Hence, the constant part of the lean inertia becomes

$$
J(m)
=
m_wR^2
+
m_b(R+h)^2
+
I_w
+
I_b
+
\frac{1}{12}m_0L_{\mathrm{rod}}^2
+
(m-m_0)\ell_{\mathrm{end}}^2.
$$

Using

$$
\begin{aligned}
m_w &= 2.436~\mathrm{kg},&
m_b &= 2.799~\mathrm{kg},\\
R &= 0.2527~\mathrm{m},&
h &= 0.025~\mathrm{m},\\
I_w &= 0.0459~\mathrm{kg\,m^2},&
I_b &= 0.0129~\mathrm{kg\,m^2},\\
m_0 &= 0.30~\mathrm{kg},&
L_{\mathrm{rod}} &= 0.314~\mathrm{m},\\
\ell_{\mathrm{end}} &= 0.157~\mathrm{m},
\end{aligned}
$$

gives

$$
J(m)
=
0.4252779
+
0.024649m
\quad
\mathrm{kg\,m^2}.
$$

The gravitational coefficient is

$$
G
=
m_wgR+m_bg(R+h)
=
13.6639517~\mathrm{N\,m}.
$$

Therefore,

$$
J_{\mathrm{crit}}
=
\frac{GR}{g}
=
\frac{13.6639517(0.2527)}{9.81}
=
0.3519756~\mathrm{kg\,m^2}.
$$

The corresponding critical moving mass satisfies

$$
0.4252779
+
0.024649m_{\mathrm{crit}}
=
0.3519756.
$$

Solving for $m_{\mathrm{crit}}$ gives

$$
\boxed{
m_{\mathrm{crit}}
=
\frac{0.3519756-0.4252779}{0.024649}
=
-2.9738~\mathrm{kg}
}.
$$

The calculated critical mass is negative and therefore physically
unrealizable. Hence, no positive moving mass can reverse the sign of the real
eigenvector position ratio for the present geometry and inertia parameters.
For every physically admissible value of $m_{\mathrm{rod}}$,

$$
\boxed{
\frac{\delta r}{\delta\theta}<0
}.
$$

For the three available mass configurations,

| $m_{\mathrm{rod}}$ | $\delta r/\delta\theta$ |
|---:|---:|
| $1.10~\mathrm{kg}$ | $-0.05789~\mathrm{m/rad}$ |
| $1.70~\mathrm{kg}$ | $-0.05987~\mathrm{m/rad}$ |
| $2.30~\mathrm{kg}$ | $-0.06145~\mathrm{m/rad}$ |

Thus, increasing the moving mass makes the ratio slightly more negative but
does not cause a sign change.

## Nonlinear region

![alt text](mc1.jpg) ![alt text](mc2.jpg)