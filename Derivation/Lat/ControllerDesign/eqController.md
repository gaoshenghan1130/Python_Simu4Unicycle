# Equilibrium Based Controller Design

## Controller Design

$$
F = m_{rod} g \sin\theta - k_1 (r-r_{des})
$$

Where 
$$
r_{des} = \frac{G + m_{rod} g R}{m_{rod} g} \tan\theta
= \frac{m_{p}g(R+h) + m_{w}gR + m_{rod} g R}{m_{rod} g} \tan\theta \\
= (\frac{m_{p}}{m_{rod}}(R+h) + \frac{m_{w}}{m_{rod}}R + R) \tan\theta
$$

![alt text](eqcontrolbig.jpg)

Closer to $k > +50$ it seems to work better.

![alt text](epcontrolmid.jpg)

At around $k = 100$ it seems to work well.

![alt text](eqcontrollers.jpg)

At shorter timescales we can see the pattern:

![alt text](eq1s2.jpg) ![alt text](eq1s.jpg)

### When K is negative

When $k < 0$ the system will try tokeep on the equilibrium line:

![alt text](ngk.jpg)

![alt text](ngk2.jpg)