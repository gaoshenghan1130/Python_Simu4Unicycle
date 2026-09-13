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

## 7. Sticking Duration from the Sliding Interval

Assume equal static and kinetic Coulomb friction magnitudes:

$$
C_s=C_d=C.
$$

Retain the same-period and half-wave symmetry assumptions,
but now allow one sticking interval at each displacement extremum.

Define

$$
T=\frac{2\pi}{\omega},\qquad H=\frac{T}{2}.
$$

Let $T_s$ denote the duration of one sticking interval.
The duration of the sliding interval is therefore

$$
\boxed{L=H-T_s.}
$$

If the measured total sticking duration per full period is
$T_{\mathrm{stop}}$, then

$$
T_s=\frac{T_{\mathrm{stop}}}{2},\qquad
L=\frac{T-T_{\mathrm{stop}}}{2}.
$$

### Initial State at Release

Define $t_m$ as the time when the rod leaves the positive
displacement maximum and starts moving in the negative direction:

$$
x(t_m)=X,\qquad \dot x(t_m)=0.
$$

At release, the PD force crosses the negative friction threshold:

$$
\boxed{
PA\sin(\omega t_m)+DA\omega\cos(\omega t_m)-PX=-C.
}
$$

Define

$$
G=A\sqrt{P^2+D^2\omega^2},\qquad
\delta=\operatorname{atan2}(D\omega,P).
$$

The release condition becomes

$$
G\sin(\omega t_m+\delta)=PX-C.
$$

For a descending threshold crossing,

$$
\boxed{
t_m=
\frac{
\pi-\arcsin\!\left(\frac{PX-C}{G}\right)-\delta+2\pi n
}{\omega}.
}
$$

Choose the integer $n$ to select the desired cycle.
A nondegenerate descending crossing requires

$$
\left|\frac{PX-C}{G}\right|<1.
$$

### Sliding Solution

For $\tau=t-t_m$ with $0<\tau<L$, the velocity is negative:

$$
x(t_m+\tau)=x_p(t_m+\tau)+\frac CP
+k_1e^{r_1\tau}+k_2e^{r_2\tau}.
$$

Define

$$
y=X-x_p(t_m)-\frac CP,\qquad
w_0=-\dot x_p(t_m).
$$

For distinct roots,

$$
\boxed{
k_1=\frac{w_0-r_2y}{r_1-r_2},\qquad
k_2=\frac{r_1y-w_0}{r_1-r_2}.
}
$$

These constants are determined from the release state.
They replace the no-sticking half-cycle constants when
sticking is included.

### End of Sliding and Subsequent Sticking

After sliding for $L=H-T_s$, the rod reaches the negative extremum:

$$
\boxed{
x(t_m+L)=-X,\qquad
\dot x(t_m+L)=0.
}
$$

Thus,

$$
\boxed{
x_p(t_m+L)+\frac CP
+k_1e^{r_1L}+k_2e^{r_2L}+X=0,
}
$$

$$
\boxed{
\dot x_p(t_m+L)
+r_1k_1e^{r_1L}+r_2k_2e^{r_2L}=0.
}
$$

The rod then remains at $x=-X$ for $T_s$:

$$
x(t)=-X,\qquad \dot x(t)=0,
\qquad t_m+L\le t\le t_m+H.
$$

At $t_m+H$, it starts moving in the positive direction.
By half-wave symmetry, its release force is $+C$.

### Parameter Identification

Given the measured amplitude $X$ and sticking duration $T_s$, define

$$
L=\frac{\pi}{\omega}-T_s.
$$

The release time is determined by

$$
t_m(C)=
\frac{
\pi-\arcsin\!\left(\frac{PX-C}{G}\right)-\delta
}{\omega}
\pmod{T}.
$$

Define the endpoint residuals

$$
R_x(b,C;X,T_s)
=
x_p(t_m+L)+\frac CP
+k_1e^{r_1L}+k_2e^{r_2L}+X,
$$

$$
R_v(b,C;X,T_s)
=
\dot x_p(t_m+L)
+r_1k_1e^{r_1L}+r_2k_2e^{r_2L},
$$

where $x_p,r_1,r_2,k_1,k_2$ are evaluated using the candidate
parameters and the release state.

The parameter estimates are obtained by solving

$$
\boxed{
\begin{pmatrix}
\hat b\\
\hat C
\end{pmatrix}
\in
\left\{
\begin{pmatrix}
b\\
C
\end{pmatrix}
\in[0,\infty)^2:
\begin{aligned}
R_x(b,C;X,T_s)&=0,\\
R_v(b,C;X,T_s)&=0
\end{aligned}
\right\}.
}
$$

Only solutions satisfying the release, sliding, and sticking
conditions are admissible. Local identifiability requires

$$
\det
\left[
\frac{\partial(R_x,R_v)}{\partial(b,C)}
\right]_{(\hat b,\hat C)}
\ne0.
$$

Conversely, for prescribed $b,C$, the predicted amplitude and
sticking duration satisfy

$$
\boxed{
\begin{pmatrix}
X\\
T_s
\end{pmatrix}
\in
\left\{
\begin{pmatrix}
\xi\\
\eta
\end{pmatrix}
:
\begin{aligned}
&\xi>0,\qquad 0\le\eta<\frac{\pi}{\omega},\\
&R_x(b,C;\xi,\eta)=0,\\
&R_v(b,C;\xi,\eta)=0
\end{aligned}
\right\}.
}
$$

## 8. Hybrid Amplitude–Position Parameter Identification

Short sticking intervals can be difficult to measure accurately.
We therefore replace the endpoint velocity equation in the
identification objective with the no-sticking amplitude relation.

The resulting method combines:

1. The no-sticking analytical amplitude.
2. The position reached after the measured sliding duration.

The endpoint velocity is retained as a validation quantity.

### No-Sticking Amplitude Residual

Let $X_0(b,C)$ denote the no-sticking amplitude derived in Section 6:

$$
X_0(b,C)
=
\frac CP+S_0+
\sqrt{B_0^2-\left(\frac{Q_0}{\omega}\right)^2}.
$$

Here, $S_0$ and $Q_0$ are computed from the no-sticking
half-cycle conditions. They must not be confused with the
initial conditions of the sliding segment that includes sticking.

An equivalent matrix representation is

$$
M=
\begin{pmatrix}
0 & 1\\
-P/m & -(D+b)/m
\end{pmatrix},
\qquad
H=\frac{\pi}{\omega},
$$

$$
\boxed{
\begin{pmatrix}
S_0\\
Q_0
\end{pmatrix}
=
\left(I+e^{MH}\right)^{-1}
\begin{pmatrix}
-2C/P\\
0
\end{pmatrix}.
}
$$

This representation also applies when the characteristic roots
are repeated.

Define the amplitude residual

$$
\boxed{
R_A(b,C)=X_0(b,C)-X_{\mathrm{meas}}.
}
$$

### Sliding Endpoint Position Residual

Using the measured duration of one sticking interval, define

$$
L=H-T_{s,\mathrm{meas}}.
$$

The release time $t_m$ is determined by the descending
negative-force threshold crossing:

$$
G\sin(\omega t_m+\delta)-PX_{\mathrm{meas}}=-C.
$$

At release,

$$
z(t_m)=
\begin{pmatrix}
X_{\mathrm{meas}}\\
0
\end{pmatrix},
\qquad
z=
\begin{pmatrix}
x\\
\dot x
\end{pmatrix}.
$$

Define

$$
z_p(t)=
\begin{pmatrix}
x_p(t)\\
\dot x_p(t)
\end{pmatrix},
\qquad
d=
\begin{pmatrix}
C/P\\
0
\end{pmatrix}.
$$

The negative-sliding solution evaluated at time $L$ is

$$
z_{\mathrm{end}}
=
z_p(t_m+L)+d
+
e^{ML}
\left[
\begin{pmatrix}
X_{\mathrm{meas}}\\
0
\end{pmatrix}
-z_p(t_m)-d
\right].
$$

Define the position residual

$$
\boxed{
R_x(b,C)
=
\begin{pmatrix}1&0\end{pmatrix}
z_{\mathrm{end}}
+X_{\mathrm{meas}}.
}
$$

### Parameter Estimation

The hybrid identification equations are

$$
\boxed{
\begin{cases}
R_A(b,C)=0,\\
R_x(b,C)=0.
\end{cases}
}
$$

With measurement and approximation errors, the numerical
estimates are defined by

$$
\boxed{
(\hat b,\hat C)
\in
\underset{(b,C)\in\mathcal D}{\operatorname{argmin}}
\left[
\left(\frac{R_A(b,C)}{X_{\mathrm{meas}}}\right)^2
+
\left(\frac{R_x(b,C)}{X_{\mathrm{meas}}}\right)^2
\right],
}
$$

where the admissible domain includes

$$
b\ge0,\qquad C\ge0,\qquad
|PX_{\mathrm{meas}}-C|<G,
$$

and requires the no-sticking amplitude expression to be real
and positive.

### Sensitivity to Sticking-Time Error

At a true stopping point,

$$
\frac{\partial x(t_m+L)}{\partial L}
=
\dot x(t_m+L)=0.
$$

Since $L=H-T_s$, the endpoint position has zero first-order
sensitivity to $T_s$ at an exact stop, with parameters and
release state held fixed.

By contrast, the endpoint velocity has sensitivity

$$
\frac{\partial \dot x(t_m+L)}{\partial T_s}
=
-\ddot x(t_m+L^-),
$$

which need not vanish.

This motivates using endpoint position rather than endpoint
velocity when the measured sticking duration is uncertain.
However, the sensitivity benefit is local and need not hold
at a hybrid solution whose endpoint velocity is nonzero.

Furthermore, reduced sensitivity to timing error does not
guarantee accurate parameter estimates. The two residual
equations may be poorly conditioned or nearly dependent,
particularly for short sticking intervals.

### Validation and Limitations

The unused endpoint velocity residual is

$$
\boxed{
R_v(\hat b,\hat C)
=
\begin{pmatrix}0&1\end{pmatrix}
z_{\mathrm{end}}.
}
$$

A physically consistent stopping point requires $R_v=0$.
A nonzero value measures inconsistency between the hybrid
estimate and the assumed stopping event.

The following should also be checked:

- Negative velocity throughout the intended sliding interval.
- Static-force feasibility during the intended sticking interval.
- Agreement of the full stick-slip simulation with measured motion.
- Sensitivity of the estimated parameters to changes in measured
  amplitude, sticking duration, and optimization initial guesses.

Because $X_0$ neglects sticking, this method is an approximate
identification procedure rather than an exact stick-slip solution.

Any applied output time shift is used only for plotting and
time-aligned comparison. It does not enter the parameter
identification equations.