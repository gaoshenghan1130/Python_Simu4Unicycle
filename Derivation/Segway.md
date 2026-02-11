# Segway derivation from Main Equations

First list the main equations of the Segway system:

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

Given the Segway system, we have the following assumptions:
- The Segway is moving in a straight line, so $\dot{\vartheta} = \vartheta = 0$, $\dot{\psi} = \psi = 0$.

Plugin the assumptions into the main equations, we have:
$$
\begin{align}
\omega_1 &:= 0, \\
\omega_2 &:= \dot{\varphi}, \\
\omega_3 &:= 0, \\
\sigma_r &:= 0, \\
\sigma_\gamma &:= \dot{\gamma}h + \dot{\varphi} R \cos \gamma + \dot{\psi} (h + R \cos \gamma) \sin 0 = \dot{\gamma}h + \dot{\varphi} R \cos \gamma,
\end{align}
$$

and

$$
\begin{align}
\dot{x}_C &= \omega_1 R \sin 0 \cos 0 + \omega_2 R \cos 0 = \dot{\varphi} R, \\
\dot{y}_C &= -\omega_1 R \cos 0 \cos 0 + \omega_2 R \sin 0 = 0, \\
\dot{\psi} &= \frac{\omega_3}{\cos 0} = 0, \\
\dot{\vartheta} &= \omega_1 = 0, \\
\dot{\varphi} &= \omega_2 - \omega_3 \tan 0 = \dot{\varphi}, \\
\dot{r} &= \sigma_r + \omega_1 R = 0, \\
\dot{\gamma} &= \sigma_\gamma  \frac{1}{h} - \omega_2 \frac{R}{h} \cos \gamma - \omega_3 \tan 0 = \dot{\gamma},
\end{align}
$$

We can extract only the most relevant equations for the Segway system, which are:
$$
\begin{align}
\dot{x}_C &= \dot{\varphi} R, \\
\sigma_\gamma &:= \dot{\gamma}h + \dot{\varphi} R \cos \gamma, \\
\end{align}
$$