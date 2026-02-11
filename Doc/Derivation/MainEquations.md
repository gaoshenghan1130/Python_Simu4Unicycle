# Main Equation

$$
\begin{align}
\omega_1 &:= \dot{\vartheta}, \\
\omega_2 &:= \dot{\varphi} + \dot{\psi} \sin \vartheta, \\
\omega_3 &:= \dot{\psi} \cos \vartheta, \\
\sigma_T &:= \dot{\gamma} R, \\
\sigma_\gamma &:= \dot{\gamma} + \dot{\psi} R \cos \gamma + \dot{\psi} (h + R \cos \gamma) \sin \vartheta,
\end{align}
$$

$$
\begin{align}
\dot{x}_C &= \omega_1 R \sin \psi \cos \vartheta + \omega_2 R \cos \psi, \\
\dot{y}_C &= -\omega_1 R \cos \psi \cos \vartheta + \omega_2 R \sin \psi, \\
\dot{\psi} &= \frac{\omega_3}{\cos \vartheta}, \\
\dot{\varphi} &= \omega_1, \\
\dot{\vartheta} &= \omega_2 - \omega_3 \tan \vartheta, \\
\dot{r} &= \sigma_T + \omega_1 R, \\
\dot{\gamma} &= \sigma_\gamma - \frac{1}{h} \omega_2 R \cos \gamma - \omega_3 \tan \vartheta,
\end{align}
$$

Notations:

![Notation](../../Asset/static/notation.png)

- \( (x_C, y_C) \): Coordinates of the center of the wheel