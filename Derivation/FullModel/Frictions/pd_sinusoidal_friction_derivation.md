# Piecewise Solution for Sinusoidal PD Tracking

## 1. Model

The desired trajectory and PD control force are

$$
x_d=A\sin(\omega t),\qquad
F=P(x_d-x)+D(\dot x_d-\dot x).
$$

With viscous and Coulomb friction, the sliding dynamics are

$$
m\ddot x+(D+b)\dot x+Px
=PA\sin(\omega t)+DA\omega\cos(\omega t)
-C\operatorname{sgn}(\dot x).
$$

Equivalently,

$$
\boxed{
m\ddot x+(D+b)\dot x+Px=
\begin{cases}
PA\sin(\omega t)+DA\omega\cos(\omega t)-C,
&\dot x>0,\\
PA\sin(\omega t)+DA\omega\cos(\omega t)+C,
&\dot x<0.
\end{cases}
} 
$$


Assume $m,P,D+b>0$, $A,\omega>0$, and $C\geq0$.
The following analysis concerns sliding motion without sticking.

## 2. Solution on Each Sliding Interval

Define the sinusoidal particular solution

$$
x_p(t)=B_0\sin(\omega t-\phi_0),
$$

where

$$
B_0=
\frac{A\sqrt{P^2+D^2\omega^2}}
{\sqrt{(P-m\omega^2)^2+(D+b)^2\omega^2}},
$$

$$
\phi_0=
\operatorname{atan2}((D+b)\omega,P-m\omega^2)
-\operatorname{atan2}(D\omega,P).
$$

For $t_j<t<t_{j+1}$, let
$s_j=\operatorname{sgn}(\dot x)=\pm1$ and $\tau=t-t_j$.
Then

$$
\boxed{
x(t)=x_p(t)-\frac{s_jC}{P}
+k_{1j}e^{r_1\tau}+k_{2j}e^{r_2\tau},
}
$$

where

$$
r_{1,2}=
\frac{-(D+b)\pm\sqrt{(D+b)^2-4mP}}{2m}.
$$

The roots $r_1,r_2$ are determined by the system parameters.
The constants $k_{1j},k_{2j}$ are determined by the state
at the start of each interval.

For $r_1\ne r_2$, define

$$
y_j=x_j-x_p(t_j)+\frac{s_jC}{P},\qquad
w_j=v_j-\dot x_p(t_j).
$$

Then

$$
k_{1j}=\frac{w_j-r_2y_j}{r_1-r_2},\qquad
k_{2j}=\frac{r_1y_j-w_j}{r_1-r_2}.
$$

For a repeated root $r$, replace the exponential sum by

$$
\left[y_j+(w_j-r y_j)\tau\right]e^{r\tau}.
$$

Complex-conjugate roots give a real solution.
The homogeneous terms must be retained on every interval,
including steady periodic motion.

## 3. Switching Conditions

For distinct roots, the next switching candidate is the first
subsequent zero of

$$
\dot x(t)=
\dot x_p(t)
+r_1k_{1j}e^{r_1(t-t_j)}
+r_2k_{2j}e^{r_2(t-t_j)}.
$$

The assumed direction must satisfy

$$
s_j\dot x(t)>0,\qquad t_j<t<t_{j+1}.
$$

At a reversal, position and velocity remain continuous:

$$
x(t_j^-)=x(t_j^+),\qquad
\dot x(t_j^-)=\dot x(t_j^+)=0.
$$

A zero of velocity must be checked to determine whether
a reversal without sticking is consistent with the dynamics.

## 4. Symmetric Steady-State Assumptions

Let $T=2\pi/\omega$ denote the input period and $T'$ denote
the response period.

Experimental observations suggest that the steady response
has the same period as the input. We therefore assume

$$
\boxed{T'=T=\frac{2\pi}{\omega}.}
$$

Additionally, assume half-wave symmetry, exactly two reversals
per period, and no sticking:

$$
x(t+T'/2)=-x(t),\qquad
\dot x(t+T'/2)=-\dot x(t).
$$

Define

$$
H=\frac{T'}2=\frac{\pi}{\omega}.
$$

Consecutive reversal times then satisfy

$$
t_{j+1}=t_j+H.
$$

Let $t_a$ denote a positive displacement maximum and let $X>0$
denote the response amplitude, distinct from the input amplitude $A$.

The endpoint conditions for the following negative-motion interval are

$$
x(t_a)=X,\qquad \dot x(t_a)=0,
$$

$$
x(t_a+H)=-X,\qquad \dot x(t_a+H)=0.
$$

The time $t_a$ is unknown and need not coincide with a zero
of $\dot x_p(t)$.

## 5. Determining the Interval Constants

Let

$$
\tau=t-t_a,\qquad c=\frac CP.
$$

Since the motion is negative over $0<\tau<H$, the solution is

$$
x(t_a+\tau)=x_p(t_a+\tau)+c
+k_1e^{r_1\tau}+k_2e^{r_2\tau},
$$

$$
\dot x(t_a+\tau)=\dot x_p(t_a+\tau)
+r_1k_1e^{r_1\tau}+r_2k_2e^{r_2\tau}.
$$

Because $H=\pi/\omega$,

$$
x_p(t_a+H)=-x_p(t_a),\qquad
\dot x_p(t_a+H)=-\dot x_p(t_a).
$$

The four endpoint conditions become

$$
X=x_p(t_a)+c+k_1+k_2,
$$

$$
0=\dot x_p(t_a)+r_1k_1+r_2k_2,
$$

$$
-X=-x_p(t_a)+c+k_1e^{r_1H}+k_2e^{r_2H},
$$

$$
0=-\dot x_p(t_a)
+r_1k_1e^{r_1H}+r_2k_2e^{r_2H}.
$$

Adding the two displacement equations and the two velocity equations
eliminates $X$, $x_p(t_a)$, and $\dot x_p(t_a)$:

$$
(1+e^{r_1H})k_1+(1+e^{r_2H})k_2=-2c,
$$

$$
r_1(1+e^{r_1H})k_1+r_2(1+e^{r_2H})k_2=0.
$$

For $r_1\ne r_2$,

$$
\boxed{
k_1=
\frac{2cr_2}{(r_1-r_2)(1+e^{r_1H})},
\qquad
k_2=
-\frac{2cr_1}{(r_1-r_2)(1+e^{r_2H})}.
}
$$

Thus, under the stated steady-state assumptions, $k_1,k_2$
can be calculated without first determining $t_a$ or $X$.

## 6. Amplitude Without Explicitly Solving for the Reversal Time

The zero-velocity condition at $t_a$ gives

$$
\dot x_p(t_a)=-(r_1k_1+r_2k_2).
$$

Since

$$
x_p(t_a)=B_0\sin(\omega t_a-\phi_0),
$$

$$
\dot x_p(t_a)=B_0\omega\cos(\omega t_a-\phi_0),
$$

the sine-cosine identity gives

$$
x_p(t_a)^2+
\left(\frac{\dot x_p(t_a)}{\omega}\right)^2=B_0^2.
$$

Therefore,

$$
x_p(t_a)=
\pm\sqrt{
B_0^2-
\left(\frac{r_1k_1+r_2k_2}{\omega}\right)^2
}.
$$

Substituting into $X=x_p(t_a)+c+k_1+k_2$ yields

$$
\boxed{
X=c+k_1+k_2
\pm\sqrt{
B_0^2-
\left(\frac{r_1k_1+r_2k_2}{\omega}\right)^2
}.
}
$$

The branch continuously connected to the frictionless response
$X=B_0$ uses the positive square root:

$$
\boxed{
X=\frac CP+k_1+k_2+
\sqrt{
B_0^2-
\left(\frac{r_1k_1+r_2k_2}{\omega}\right)^2
}.
}
$$

This expression gives a candidate steady-state peak amplitude.
It does not require explicitly solving for $t_a$.

For repeated roots, use the repeated-root solution or take
the corresponding limit of the combined expressions.

### Parameter Identification

Given the measured amplitude $X$ and sticking duration $T_s$,
the sliding duration is

$$
L=\frac{\pi}{\omega}-T_s.
$$

For each candidate pair $(b,C)$, the release condition determines
$t_m$, and the initial state $x(t_m)=X$, $\dot x(t_m)=0$
determines the sliding solution.

The parameters can then be obtained numerically in MATLAB
by solving the two endpoint equations:

$$
\boxed{
\begin{cases}
x(t_m+L;b,C)=-X,\\
\dot x(t_m+L;b,C)=0.
\end{cases}
}
$$

## 7. Modified Identification Method

Since the sticking intervals are short and difficult to measure
accurately, we instead combine the no-sticking amplitude formula
from Section 6 with the sliding endpoint position condition
from Section 7:

$$
\boxed{
\begin{cases}
X_0(b,C)=X_{\mathrm{meas}},\\
x(t_m+L;b,C)=-X_{\mathrm{meas}},
\end{cases}
\qquad
L=\frac{\pi}{\omega}-T_{s,\mathrm{meas}}.
}
$$

Here, $X_0(b,C)$ is the no-sticking amplitude prediction.
The second equation uses the sliding solution initialized at
release from $x=X_{\mathrm{meas}}$, $\dot x=0$.

MATLAB's `fminsearch` is used to adjust $b$ and $C$ until both
equations are approximately satisfied. Multiple initial guesses
are used, with the parameters constrained to be positive and
the release condition required to have a real solution.

The endpoint velocity is not used to identify the parameters.
Instead, it is checked afterward:

$$
\dot x(t_m+L;\hat b,\hat C)\approx0.
$$

Near a true stopping point, position changes slowly because
the velocity is close to zero. The position condition may
therefore be less sensitive to timing errors than the velocity
condition, although this does not guarantee more reliable
parameter estimates.

This method is approximate because the amplitude formula
neglects sticking. The identified parameters are subsequently
tested in the full stick-slip simulation and compared with
the measured position and velocity responses.

The output time shift is applied only for comparison and does
not participate in parameter identification.

## 8. Preliminary Experimental Results

The hybrid identification method was applied to two sinusoidal
tracking experiments with desired amplitudes of 25 mm and 50 mm,
both at 0.5 Hz.

The model parameters used were

$$
m=1.0\ \mathrm{kg},\qquad
P=100\ \mathrm{N/m},\qquad
D=8\ \mathrm{N\,s/m}.
$$

Static and kinetic Coulomb friction were assumed equal:

$$
C_s=C_k=C.
$$

### Data Preparation

The measured position was centered using the extrema within
the selected analysis window:

$$
x_{\mathrm{offset}}
=
\frac{x_{\max}+x_{\min}}{2},
\qquad
x_{\mathrm{centered}}(t)
=
x_{\mathrm{measured}}(t)-x_{\mathrm{offset}}.
$$

The removed offsets were $-0.550$ mm and $-0.835$ mm for the
25 mm and 50 mm datasets, respectively.

For visual comparison, the simulated signals were advanced
by 70 ms. This time alignment was applied after simulation
and did not participate in parameter identification.

### Identified Parameters

| Quantity | 25 mm input | 50 mm input |
|---|---:|---:|
| Measured response amplitude $X$ [mm] | 23.120 | 47.325 |
| Identified $b$ [N s/m] | 6.66817 | 6.39715 |
| Identified $C$ [N] | 0.234922 | 0.388243 |
| Scaled identification residual norm | $4.10\times10^{-13}$ | $3.36\times10^{-13}$ |
| Sliding endpoint velocity [m/s] | $3.49\times10^{-4}$ | $-5.53\times10^{-4}$ |

The viscous coefficients differ by approximately 4% relative
to their mean. The Coulomb estimates show a larger variation
between the two experiments.

The small residual norms indicate that the two selected
identification equations were solved accurately. They do not
by themselves establish the accuracy of the friction model
or the uniqueness of the parameter estimates.

### Manual Tuning and Interpretation

A shared manually tuned parameter set,

$$
b=6.5\ \mathrm{N\,s/m},\qquad C=0.30\ \mathrm{N},
$$

also gave close visual agreement with the measured position
and velocity responses after time alignment.

Together with the two identification results, this suggests
that the viscous coefficient is relatively consistent across
the tested amplitudes:

$$
\boxed{b\approx6.5\pm0.2\ \mathrm{N\,s/m}.}
$$

This range summarizes the present estimates and manual tuning;
it is not a statistical confidence interval.

The Coulomb friction magnitude $C$ is more difficult to identify
reliably. First, the hybrid method uses a no-sticking amplitude
relation, so the effect of sticking is only partially represented.
This approximation, together with uncertainty in the short
sticking intervals, may contribute to the variation in the
identified values.

Second, the actual friction may depend on rod position.
If the mechanism is viewed as a sliding rod supported by a sleeve,
clearance between the rod and sleeve may allow slight tilting
and uneven contact. As the rod moves farther from the center,
changes in loading and moment may redistribute the contact
pressure and potentially increase the normal contact forces,
changing the friction and breakaway force.

This provides a possible mechanical explanation for differences
between the 25 mm and 50 mm experiments. However, the current
results do not establish position-dependent friction: the two
tests also involve different velocities, and the identified
parameter $C$ combines static and kinetic friction under the
assumption $C_s=C_k$.