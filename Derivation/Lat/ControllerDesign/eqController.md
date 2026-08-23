# Equilibrium-Based Controller Design

## Equilibrium curve

For the reduced lateral model, let \(m=m_{\rm rod}\) and define

\[
C=\frac{G+mgR}{mg}
=\frac{m_p(R+h)+m_wR+mR}{m}.
\]

The nonzero static equilibria satisfy

\[
r_{\rm eq}(\theta)=C\tan\theta,
\qquad
F_{\rm eq}(\theta)=mg\sin\theta.
\]

Thus the equilibrium curve is not a single target: every point
\((\theta,\,C\tan\theta)\) is an equilibrium when the corresponding
\(F_{\rm eq}\) is applied.

## Transverse controller: reaching the curve

Define the signed distance-like surface coordinate

\[
s=r-C\tan\theta,
\qquad
\dot{s}=\dot r-C\sec^2\theta\,\dot\theta.
\]

The previously tested controller is

\[
F=mg\sin\theta+k_s s+k_d\dot{s},
\qquad k_s<0,\quad k_d<0.
\]

It is useful for driving \(s\to0\), i.e., bringing the state close to the
equilibrium curve. However, it does not by itself select the origin: after
reaching \(s=0\), the system can remain close to a nonzero equilibrium.

## Force obtained from the twice-differentiated surface

Use the physical-coordinate equations of motion

\[
\begin{bmatrix}
J(r) & -mR\\
-mR & m
\end{bmatrix}
\begin{bmatrix}\ddot\theta\\\ddot r\end{bmatrix}
=
\begin{bmatrix}
G\sin\theta+mgR\sin\theta-mgr\cos\theta-2mr\dot r\dot\theta\\
-mg\sin\theta+F+mr\dot\theta^2
\end{bmatrix},
\]

where \(J(r)=J_0+mr^2\). Define

\[
D=mJ(r)-m^2R^2.
\]

Differentiating the surface once more gives

\[
\ddot{s}
=\ddot r-C\sec^2\theta\,\ddot\theta
-2C\sec^2\theta\tan\theta\,\dot\theta^2
=a_s(x)+b_s(x)F,
\]

with

\[
\begin{aligned}
A_\theta &=
G\sin\theta-mgr\cos\theta
-2mr\dot r\dot\theta+mRr\dot\theta^2,\\
A_r &=mR\!\left(G\sin\theta+mgR\sin\theta-mgr\cos\theta
-2mr\dot r\dot\theta\right)
+J(r)\!\left(-mg\sin\theta+mr\dot\theta^2\right),\\
a_s(x)&=\frac{A_r-Cm\sec^2\theta\,A_\theta}{D}
-2C\sec^2\theta\tan\theta\,\dot\theta^2,\\
b_s(x)&=\frac{J(r)-CmR\sec^2\theta}{D}.
\end{aligned}
\]

This permits the force to be chosen directly from a desired surface
acceleration \(v_s\):

\[
\boxed{\;F=\frac{v_s-a_s(x)}{b_s(x)}\;}.
\]

This expression is valid only where \(D\neq0\) and \(b_s(x)\neq0\); the
implemented force must also be saturated at the linear-motor limit.

## Adding a slow origin-seeking action

Choose

\[
v_s=-k_p s-k_d\dot{s}
     -k_\theta\theta-k_\omega\dot\theta,
\qquad
k_p,k_d,k_\theta,k_\omega>0.
\]

Then

\[
F=F_{\rm curve}+F_{\rm origin},
\]

where

\[
F_{\rm curve}=\frac{-a_s-k_p s-k_d\dot{s}}{b_s},
\qquad
F_{\rm origin}=\frac{-k_\theta\theta-k_\omega\dot\theta}{b_s}.
\]

\(F_{\rm curve}\) makes the trajectory return to the curve, while
\(F_{\rm origin}\) biases it when \(\theta\neq0\). In particular, the
origin is now the only point on \(s=0\) at which the added term vanishes.
The \(k_\theta\) and \(k_\omega\) gains should be deliberately small relative
to \(k_p,k_d\), so the motion toward the origin is slow and does not create a
large excursion away from the curve.

This is not an arbitrary extra force added after the fact. It is part of the
same feedback-linearized law for \(\ddot{s}\). With one actuator, adding an
independent force to the original controller would generally destroy curve
tracking.

## Simulation 

For the "reaching the curve" controller:

![alt text](c1.jpg) 


For the "origin-seeking" controller:

![alt text](c2.jpg)