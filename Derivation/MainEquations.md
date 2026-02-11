# Main Equation

$$
\begin{align}
\omega_1 &:= \dot{\vartheta}, \\
\omega_2 &:= \dot{\varphi} + \dot{\psi} \sin \vartheta, \\
\omega_3 &:= \dot{\psi} \cos \vartheta, \\
\sigma_r &:= \dot{r} - \dot{\vartheta} R, \\
\sigma_\gamma &:= \dot{\gamma}h + \dot{\varphi} R \cos \gamma + \dot{\psi} (h + R \cos \gamma) \sin \vartheta,
\end{align}
$$

$$
\begin{align}
\dot{x}_C &= \omega_1 R \sin \psi \cos \vartheta + \omega_2 R \cos \psi, \\
\dot{y}_C &= -\omega_1 R \cos \psi \cos \vartheta + \omega_2 R \sin \psi, \\
\dot{\psi} &= \frac{\omega_3}{\cos \vartheta}, \\
\dot{\vartheta} &= \omega_1, \\
\dot{\varphi} &= \omega_2 - \omega_3 \tan \vartheta, \\
\dot{r} &= \sigma_r + \omega_1 R, \\
\dot{\gamma} &= \sigma_\gamma  \frac{1}{h} - \omega_2 \frac{R}{h} \cos \gamma - \omega_3 \tan \vartheta,
\end{align}
$$

## Notations:

![Notation](../../Asset/static/notation.png)

| Symbol | Physical Meaning | Description |
|--------|-----------------|-------------|
| x_C, y_C | Wheel center coordinates | Position of the wheel center in the plane |
| ψ | Heading angle | Direction of the wheel / vehicle heading |
| ϑ | Lean angle | Tilt angle of the body along the forward/backward direction |
| φ | Wheel rotation angle | Rotation of the wheel around its axis |
| r | Linear balancer displacement | Distance of m_1 and the center |
| γ | Pendulum angle | Angle of the pendulum relative to vertical |
| R | Wheel radius | Geometric radius of the wheel |
| h | Pendulum length | Distance from wheel axis to the pendulum center of mass |
| σ_r | Velocity of the linear balancer | Velocity of m_1 absolute |
| σ_γ | Velocity of the pendulum | Velocity of m_2 absolute |