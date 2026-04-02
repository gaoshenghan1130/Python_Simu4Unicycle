# Motor damping further analysis


## Overshoot of velocity at equilibrium point

While looking at the output of motordamping mode, we noticed that at the equilibrium point the velocity is larger than expected value, compared to the case without damping. Here we will analyze the reason for this phenomenon.

### Model

According to the model derived:

$$
\begin{align}\ddot{x} &= N_{11} \left(\frac{(M - T_{damp})}{R} + m h \dot{\gamma}^2 \sin \gamma\right) + N_{12} \left(-(M - T_{damp}) + m g h \sin \gamma\right), \\
\ddot{\gamma} &= N_{21} \left(\frac{(M - T_{damp})}{R} + m h \dot{\gamma}^2 \sin \gamma\right) + N_{22} \left(-(M - T_{damp}) + m g h \sin \gamma\right).
\end{align}
$$

Where:
$$
\begin{align}
N(q) = \begin{bmatrix}m + m_w & m h \cos \gamma \\
m h \cos \gamma & m h^2 + I\end{bmatrix}^{-1}. \\
T_{damp} = B \omega + B_0 sign(\omega) = B \left(\frac{\dot{x}_C}{R} -\dot{\gamma}\right) + B_0 sign\left(\frac{\dot{x}_C}{R} -\dot{\gamma}\right).
\end{align}
$$

At equilibrium, $\ddot{x} = 0$ and $\ddot{\gamma} = \dot{\gamma} = 0 $, we have:
$$
\begin{align}
0 &= N_{11} \left(\frac{M}{R} - \frac{T_{damp}}{R}\right) + N_{12} \left(-(M-T_{damp}) + m g h \sin \gamma\right), \\
0 &= N_{21} \left(\frac{M}{R} - \frac{T_{damp}}{R}\right) + N_{22} \left(-(M-T_{damp}) + m g h \sin \gamma\right). 
\end{align}
$$

In matrix form, we have:
$$
N \begin{bmatrix}\frac{M}{R} - \frac{T_{damp}}{R} \\ -M + m g h \sin \gamma\end{bmatrix} = 0.
$$

As $N$ is invertible, we have:
$$
\begin{align}
\frac{M}{R} - \frac{T_{damp}}{R} &= 0, \\
-(M - T_{damp}) + m g h \sin \gamma &= 0.
\end{align}
$$

So at equilibrium, we have:
$$
\begin{align}
\gamma &= 0, \ (\text{Align with the model result}) \\
T_{damp} &= M.
\end{align}
$$

Given that our PD control:
$$
M = K_{\gamma} (\gamma - 0) + K_{\dot{\gamma}} (\dot{\gamma} - 0) + K_v (\dot{x} - v_{exo}) = K_{\gamma} \gamma + K_{\dot{\gamma}} \dot{\gamma} + K_v (\dot{x} - v_{exp})
$$

For $M = T_{damp}$, when $B_0 = 0$ for simplification we have:
$$\begin{align}
K_v (\dot{x} - v_{exp}) &= B \left(\frac{\dot{x}}{R} -\dot{\gamma}\right) \\
\dot{x} &= v = \frac{K_v }{K_v - \frac{B}{R}}v_{exp} > v_{exp}
\end{align}
$$

And mathmatically, plugin $K_v = 3.3$ and $B = 0.306$, $R = 0.2527$, we have:
$$
\dot{x} = 1.57 v_{exp}.
$$