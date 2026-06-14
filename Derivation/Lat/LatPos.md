# Position of $m_L$ in Cartesian Coordinates

Center of the wheel:

$$
r_w = R \sin\theta \hat{i} + R \cos\theta \hat{j}
$$

Center of the rod:
$$
r_{rod} = (R \sin\theta + r\cos\theta) \hat{i} + (R \cos\theta - r\sin\theta) \hat{j}
$$

Center of the mass $m_L$:
$$
r_L = r_{rod} \pm \frac{l}{2} \left( \cos\theta \hat{i} - \sin\theta \hat{j} \right) = \\
(R \sin\theta + r\cos\theta \pm \frac{l}{2} \cos\theta) \hat{i} + (R \cos\theta - r\sin\theta \mp \frac{l}{2} \sin\theta) \hat{j}
$$

## Potential energy:

$$
V_{rod} = m_{rod} \langle r_{rod}, \hat{j} \rangle g = m_{rod} g (R \cos\theta - r\sin\theta)
$$

$$
V_{mass} = m_L \langle r_L, \hat{j} \rangle g = m_L g (R \cos\theta - r\sin\theta \mp \frac{l}{2} \sin\theta)
$$

Total potential energy on the rod and mass:
$$
V = 2V_{mass} + V_{rod} = 2 m_L g (R \cos\theta - r\sin\theta) + m_{rod} g (R \cos\theta - r\sin\theta) = (2 m_L + m_{rod}) g (R \cos\theta - r\sin\theta)
$$

## Kinetic energy:

Taking the time derivative of the position vectors:

$$
\dot{r}_{rod} = \left( (\dot{r} + R\dot{\theta})\cos\theta - r\dot{\theta}\sin\theta \right)\hat{i} - \left( (\dot{r} + R\dot{\theta})\sin\theta + r\dot{\theta}\cos\theta \right)\hat{j}
$$

$$
\dot{r}_{L} = \left( (\dot{r} + R\dot{\theta})\cos\theta - (r \pm \frac{l}{2})\dot{\theta}\sin\theta \right)\hat{i} - \left( (\dot{r} + R\dot{\theta})\sin\theta + (r \pm \frac{l}{2})\dot{\theta}\cos\theta \right)\hat{j}
$$

Evaluating the magnitude squared via Cartesian vector dot-products:

$$
\| \dot{r}_{rod} \|^2 = (\dot{r} + R \dot{\theta})^2 + (r\dot{\theta})^2
$$

$$
\| \dot{r}_L \|^2 = (\dot{r} + R \dot{\theta})^2 + (r \pm \frac{l}{2})^2 \dot{\theta}^2 = (\dot{r} + R \dot{\theta})^2 + (r\dot{\theta})^2 \pm l r \dot{\theta}^2 + \frac{1}{4}l^2\dot{\theta}^2
$$

Total kinetic energy on the rod and mass:

$$
T_{rod} = \frac{1}{2} m_{rod} \| \dot{r}_{rod} \|^2 + \frac{1}{2} I_{rod} \dot{\theta}^2 = \frac{1}{2} m_{rod} \left( (\dot{r} + R \dot{\theta})^2 + (r\dot{\theta})^2 \right) + \frac{1}{2} I_{rod} \dot{\theta}^2
$$

$$
T_{mass\_total} = 2 \times \left( \frac{1}{2} m_L \| \dot{r}_L \|^2 \right) = m_L \left( (\dot{r} + R \dot{\theta})^2 + (r\dot{\theta})^2 + l r \dot{\theta}^2 + \frac{1}{4}l^2\dot{\theta}^2 \right) + m_L \left( (\dot{r} + R \dot{\theta})^2 + (r\dot{\theta})^2 - l r \dot{\theta}^2 + \frac{1}{4}l^2\dot{\theta}^2 \right)
$$

Notice that the cross-terms $\pm m_L l r \dot{\theta}^2$ cancel out due to symmetry:

$$
T_{mass\_total} = 2m_L \left( (\dot{r} + R \dot{\theta})^2 + (r\dot{\theta})^2 \right) + \frac{1}{2} m_L l^2 \dot{\theta}^2
$$

$$
T = T_{rod} + T_{mass\_total} = \frac{1}{2} (2m_L + m_{rod}) \left( (\dot{r} + R \dot{\theta})^2 + (r\dot{\theta})^2 \right) + \frac{1}{2}\left(I_{rod} + m_L \frac{l^2}{2}\right)\dot{\theta}^2
$$

## Conclusion:

For the mass $m_L$ located at the ends of the rod, the kinetic and potential energy expressions are:

$$V = (2 m_L + m_{rod}) g (R \cos\theta - r\sin\theta)$$
$$T = \frac{1}{2} (2m_L + m_{rod}) \left( (\dot{r} + R \dot{\theta})^2 + (r\dot{\theta})^2 \right) + \frac{1}{2}\left(I_{rod} + m_L \frac{l^2}{2}\right)\dot{\theta}^2$$

Note that in the `MATLAB` code, we define parameters as:
```matlab
    par.mass_at_end = 0.5; % mass of the object attached at the end of the rod
    par.m_L = 0.15 + par.mass_at_end; % actually half of the rod
    %....
    par.I_rod = 1/12 * (0.3 ) * (0.314^2) + 2 * par.mass_at_end *2;
```

This way we can save some parameter in the code, and the kinetic and potential energy expressions can be simplified to:
$$V = 2 m_L g (R \cos\theta - r\sin\theta)$$
$$T = m_L \left( (\dot{r} + R \dot{\theta})^2 + (r\dot{\theta})^2 \right) + \frac{1}{2}I_{rod}\dot{\theta}^2$$

Which is the version we used also in [Lat.md](Lat.md).

