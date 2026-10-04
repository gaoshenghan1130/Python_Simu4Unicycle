# Steady-state analysis in the paper's convention

## Paper variables and pseudo-velocities

The paper uses four pseudo-velocities:

\[
\omega_1=\dot\vartheta,\qquad
\omega_2=\dot\psi\sin\vartheta+\dot\varphi,\qquad
\omega_3=\dot\psi\cos\vartheta,
\]
\[
\sigma=\dot r-R\dot\vartheta.
\]

Here \(\vartheta\) is the wheel tilt angle, \(r\) is the position of the
movable mass along the axle, and \(\sigma\) is the axle-direction
pseudo-velocity of that mass. The paper's state vector is

\[
x=
\begin{bmatrix}
\omega_1&\omega_2&\omega_3&\vartheta&\sigma&r&
\psi&\varphi&x_G&y_G
\end{bmatrix}^{T}.
\]

The relevant kinematic equations are

\[
\dot\vartheta=\omega_1,\qquad
\dot\sigma=
\omega_1^2r+\omega_3^2r-\omega_2\omega_3R
-g\sin\vartheta+\frac{u}{m_0},
\]
\[
\dot r=\sigma+\omega_1R,
\qquad
\dot\psi=\frac{\omega_3}{\cos\vartheta},
\qquad
\dot\varphi=\omega_2-\omega_3\tan\vartheta.
\]

Therefore, \(\sigma\) is not the mass position; \(r\) is the mass position.
The paper has no quantity called \(\sigma_g\).

## Steady-state definition

The paper holds the essential dynamics constant:

\[
\omega_1=\omega_1^*,\quad
\omega_2=\omega_2^*,\quad
\omega_3=\omega_3^*,\quad
\vartheta=\vartheta^*,\quad
\sigma=\sigma^*,\quad
r=r^*.
\]

For the open-loop analysis \(u=0\), the kinematic equations give

\[
\boxed{\omega_1^*=0,\qquad \sigma^*=0.}
\]

The hidden coordinates \(\psi,\varphi,x_G,y_G\) can still move at constant
rates, so this is not the same as imposing \(\dot x=0\) on the complete
state.

## Steady-state compatibility equations

After substituting the steady-state conditions into the first six equations
of the paper's model, the remaining constraints are

\[
(\omega_3^*)^2r^*
-\omega_2^*\omega_3^*R
-g\sin\vartheta^*=0,
\]

\[
\begin{aligned}
0={}&\omega_2^*\omega_3^*
\left(6mR^2+4m_0Rr^*\tan\vartheta^*\right)
-4m_0gr^*\cos\vartheta^*\\
&-(\omega_3^*)^2
\left(mR^2+4m_0(r^*)^2\right)\tan\vartheta^*
+4mgR\sin\vartheta^*.
\end{aligned}
\]

Here \(m\) is the wheel mass and \(m_0\) is the movable point-mass. Thus
\(\vartheta^*,\dot\psi^*,\dot\varphi^*\), and \(r^*\) are not independent;
they must satisfy these equations.

## Conclusions from the steady-state analysis

### 1. Straight rolling

For

\[
\dot\psi^*=0,
\]

the only solution is

\[
\vartheta^*=0,\qquad r^*=0,
\]

while the pitch rate \(\dot\varphi^*\) may be arbitrary.

### 2. Generic turning-rolling

For \(\dot\psi^*\neq0\), the unicycle can have a tilted steady motion with
\(\vartheta^*\neq0\). Given the tilt angle and yaw rate, the compatibility
equations determine the corresponding \(r^*\) and \(\dot\varphi^*\), except
at

\[
\dot\psi^*=
\pm\sqrt{\frac{2m_0g}{3mR\cos^3\vartheta^*}}.
\]

### 3. Non-tilted turning

The movable mass creates a new steady motion with

\[
\vartheta^*=0,\qquad
\dot\psi^*_{1,2}
=\pm\sqrt{\frac{2m_0g}{3mR}}.
\]

The associated pitch rate and mass position are linked. If \(r^*=0\), this
special case becomes a spinning steady state.

### 4. Spinning

The regular non-tilted spinning state has

\[
\vartheta^*=0,\qquad
\dot\varphi^*=0,\qquad
r^*=0,
\]

with arbitrary yaw rate. The paper also finds tilted spinning states with
\(\vartheta^*\neq0\) and \(\dot\varphi^*=0\).

### 5. Physical feasibility

The mass must remain above the ground. The paper's physical feasibility
condition is

\[
\boxed{R\cos\vartheta^*+r^*\sin\vartheta^*>0.}
\]

### 6. Stability of straight rolling

The critical pitch rate is

\[
\boxed{\dot\varphi_{\mathrm{crit}}^*
=\sqrt{\frac{g}{2R}}.}
\]

The uncontrolled straight-rolling state is stable when
\[
|\dot\varphi^*|>\dot\varphi_{\mathrm{crit}}^*
\]
and unstable when it is below this value. This critical rate is independent
of \(m\) and \(m_0\). Turning-rolling stability is determined from the
linearized system by semi-analytical and numerical analysis.

## Input notation

The paper's original unicycle has one control input \(u\), applied as the
force

\[
\mathbf F=
\begin{bmatrix}0&u&0\end{bmatrix}^{T}
\]

on the movable mass. It does not define a separate input \(M\). Any \(M\)
used in the present extended model must therefore be derived from that
extended model and should not be attributed to the paper.
