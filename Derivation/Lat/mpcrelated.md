# Try using simplified Mpc

Use the model derived with Appell:

$$
\begin{bmatrix} 
m_w R^2 + m_{rod} r^2 & 0 \\ 
0 & m_{rod} 
\end{bmatrix}
\begin{bmatrix} 
\dot{u}_1 \\ 
\dot{u}_2 
\end{bmatrix}
=
\begin{bmatrix} 
F R - m_{rod} g r \cos\theta + m_w g R \sin\theta - m_{rod} r (2 u_2 u_1 + R u_1^2) \\ 
F + m_{rod} g \sin\theta + m_{rod} r u_1^2 
\end{bmatrix}
$$

Where:

$$
u_1 = \dot{\theta} \\
u_2 = \dot{r} - R \dot{\theta} = \sigma
$$

To simply a bit:

$$
\begin{bmatrix}
m_w R^2 + m_{rod} r^2 & 0 \\
0 & m_{rod}
\end{bmatrix}^{-1}
=
\begin{bmatrix}
\dfrac{1}{m_w R^2 + m_{rod} r^2} & 0 \\
0 & \dfrac{1}{m_{rod}}
\end{bmatrix} \\
\implies \\
\begin{bmatrix} 
\dot{u}_1 \\ 
\dot{u}_2 
\end{bmatrix}
= \begin{bmatrix}
\dfrac{1}{m_w R^2 + m_{rod} r^2} & 0 \\
0 & \dfrac{1}{m_{rod}}
\end{bmatrix} \begin{bmatrix} 
- m_{rod} g r \cos\theta + m_w g R \sin\theta - m_{rod} r (2 u_2 u_1 + R u_1^2) \\ 
m_{rod} g \sin\theta + m_{rod} r u_1^2 
\end{bmatrix} + \begin{bmatrix} FR \\
F
\end{bmatrix}
$$

Further, let $x = [\theta, r - R\theta, \dot{\theta}, \dot{r} - R\dot{\theta}]$, and $I(x_2) = m_w R^2 + m_{rod} (x_2+Rx_1)^2$:

$$\begin{aligned}
\dot{x}_1 &= x_3 \\
\dot{x}_2 &= x_4 \\
\dot{x}_3 &= \frac{1}{I(x_1, x_2)} \Big[ - m_{rod} g (x_2 + R x_1) \cos x_1 + m_w g R \sin x_1 - m_{rod} (x_2 + R x_1) (2 x_4 x_3 + R x_3^2) + F R \Big] \\
\dot{x}_4 &= \frac{1}{m_{rod}} \Big[ m_{rod} g \sin x_1 + m_{rod} (x_2 + R x_1) x_3^2 + F \Big]
\end{aligned}
$$

Define $\mathcal{N} = - m_{rod} g (x_2 + Rx_1) \cos x_1 + m_w g R \sin x_1 - m_{rod} (x_2 + R x_1) (2 x_4 x_3 + R x_3^2)$

$$
\begin{aligned}
\dot{x}_1 &= x_3 \\
\dot{x}_2 &= x_4 \\
\dot{x}_3 &= \dfrac{\mathcal{N}}{I(x_2)} + \dfrac{R}{I(x_2)} F \\
\dot{x}_4 &= \frac{1}{m_{rod}} \Big[ m_{rod} g \sin x_1 + m_{rod} (x_2 + Rx_1) x_3^2 + F \Big]
\end{aligned}
$$

Formal MPC use $\mathbf{x}_{k+1} = \mathbf{A} \mathbf{x}_k + \mathbf{B} \mathbf{u}_k + \mathbf{d}$  

$$
\mathbf{B}_c = \frac{\partial \dot{\mathbf{x}}}{\partial F} = \begin{bmatrix} 0 \\ 0 \\ \dfrac{R}{I(x_2)} \\ \dfrac{1}{m_{rod}} \end{bmatrix}
$$

$$
\mathbf{A}_c = \frac{\partial \dot{\mathbf{x}}}{\partial \mathbf{x}} = \begin{bmatrix} 
0 & 0 & 1 & 0 \\ 
0 & 0 & 0 & 1 \\ 
\dfrac{\partial \dot{x}_3}{\partial x_1} & \dfrac{\partial \dot{x}_3}{\partial x_2} & \dfrac{\partial \dot{x}_3}{\partial x_3} & \dfrac{\partial \dot{x}_3}{\partial x_4} \\ 
\dfrac{\partial \dot{x}_4}{\partial x_1} & \dfrac{\partial \dot{x}_4}{\partial x_2} & \dfrac{\partial \dot{x}_4}{\partial x_3} & 0 
\end{bmatrix}
$$

We can solve $A_c$ with MATLAB

$$
\left(\begin{bmatrix}{cccc} 0 & 0 & 1 & 0\\ 0 & 0 & 0 & 1\\ -\frac{R\,m_{\mathrm{rod}}\,\left(R\,{x_{3}}^2+2\,x_{4}\,x_{3}\right)-g\,m_{\mathrm{rod}}\,\sin\left(x_{1}\right)\,\left(x_{2}+R\,x_{1}\right)+R\,g\,m_{\mathrm{rod}}\,\cos\left(x_{1}\right)-R\,g\,m_{w}\,\cos\left(x_{1}\right)}{m_{\mathrm{rod}}\,{\left(x_{2}+R\,x_{1}\right)}^2+R^2\,m_{w}}-\frac{2\,R\,m_{\mathrm{rod}}\,\left(x_{2}+R\,x_{1}\right)\,\left(F\,R-m_{\mathrm{rod}}\,\left(x_{2}+R\,x_{1}\right)\,\left(R\,{x_{3}}^2+2\,x_{4}\,x_{3}\right)-g\,m_{\mathrm{rod}}\,\cos\left(x_{1}\right)\,\left(x_{2}+R\,x_{1}\right)+R\,g\,m_{w}\,\sin\left(x_{1}\right)\right)}{{\left(m_{\mathrm{rod}}\,{\left(x_{2}+R\,x_{1}\right)}^2+R^2\,m_{w}\right)}^2} & -\frac{m_{\mathrm{rod}}\,\left(R\,{x_{3}}^2+2\,x_{4}\,x_{3}\right)+g\,m_{\mathrm{rod}}\,\cos\left(x_{1}\right)}{m_{\mathrm{rod}}\,{\left(x_{2}+R\,x_{1}\right)}^2+R^2\,m_{w}}-\frac{m_{\mathrm{rod}}\,\left(2\,x_{2}+2\,R\,x_{1}\right)\,\left(F\,R-m_{\mathrm{rod}}\,\left(x_{2}+R\,x_{1}\right)\,\left(R\,{x_{3}}^2+2\,x_{4}\,x_{3}\right)-g\,m_{\mathrm{rod}}\,\cos\left(x_{1}\right)\,\left(x_{2}+R\,x_{1}\right)+R\,g\,m_{w}\,\sin\left(x_{1}\right)\right)}{{\left(m_{\mathrm{rod}}\,{\left(x_{2}+R\,x_{1}\right)}^2+R^2\,m_{w}\right)}^2} & -\frac{m_{\mathrm{rod}}\,\left(x_{2}+R\,x_{1}\right)\,\left(2\,x_{4}+2\,R\,x_{3}\right)}{m_{\mathrm{rod}}\,{\left(x_{2}+R\,x_{1}\right)}^2+R^2\,m_{w}} & -\frac{2\,m_{\mathrm{rod}}\,x_{3}\,\left(x_{2}+R\,x_{1}\right)}{m_{\mathrm{rod}}\,{\left(x_{2}+R\,x_{1}\right)}^2+R^2\,m_{w}}\\ R\,{x_{3}}^2+g\,\cos\left(x_{1}\right) & {x_{3}}^2 & 2\,x_{3}\,\left(x_{2}+R\,x_{1}\right) & 0 \end{bmatrix}\right)
$$

To eliminate the 0 order error, calculate current state vector 

$$
\mathbf{d}_c = \dot{\mathbf{x}}\big|_{(\mathbf{x}_k, F_{k-1})} - \mathbf{A}_c \mathbf{x}_k - \mathbf{B}_c F_{k-1}
$$

Finally we get:

$$
\mathbf{x}_{k+1} = \mathbf{A} \mathbf{x}_k + \mathbf{B} u_k + \mathbf{d}
$$

Where $\mathbf{A} = \mathbf{I}_{4 \times 4} + \mathbf{A}_c \cdot \Delta t$, $\mathbf{B} = \mathbf{B}_c \cdot \Delta t$, $\mathbf{d} = \mathbf{d}_c \cdot \Delta t$

## Methods of entending along Time rod

Given

$$
\mathbf{X}_k = \begin{bmatrix} \mathbf{x}_{k+1} \\ \mathbf{x}_{k+2} \\ \vdots \\ \mathbf{x}_{k+N} \end{bmatrix}_{4N \times 1}, \quad \mathbf{U}_k = \begin{bmatrix} u_k \\ u_{k+1} \\ \vdots \\ u_{k+N-1} \end{bmatrix}_{N \times 1}
$$

We can just calculate the following equation to get the state of all time

$$
\mathbf{X}_k = \mathbf{M} \mathbf{x}_k + \mathbf{C} \mathbf{U}_k + \mathbf{D}
$$

Where:

$$
\mathbf{M} = \begin{bmatrix} \mathbf{A} \\ \mathbf{A}^2 \\ \vdots \\ \mathbf{A}^N \end{bmatrix}, \quad 
\mathbf{C} = \begin{bmatrix} 
\mathbf{B} & 0 & \dots & 0 \\ 
\mathbf{A}\mathbf{B} & \mathbf{B} & \dots & 0 \\ 
\vdots & \vdots & \ddots & \vdots \\ 
\mathbf{A}^{N-1}\mathbf{B} & \mathbf{A}^{N-2}\mathbf{B} & \dots & \mathbf{B} 
\end{bmatrix}, \quad 
\mathbf{D} = \begin{bmatrix} \mathbf{d} \\ \mathbf{A}\mathbf{d} + \mathbf{d} \\ \vdots \\ \sum_{i=0}^{N-1} \mathbf{A}^i \mathbf{d} \end{bmatrix}
$$

Define a cost function:

$$J = (\mathbf{X}_k - \mathbf{\mathcal{X}}_{ref})^T \mathbf{\bar{Q}} (\mathbf{X}_k - \mathbf{\mathcal{X}}_{ref}) + \mathbf{U}_k^T \mathbf{\bar{R}} \mathbf{U}_k
$$

Where:

$\mathbf{\bar{Q}} = \text{diag}(\mathbf{Q}, \dots, \mathbf{Q}_f)$，$\mathbf{\bar{R}} = \text{diag}(R_u, \dots, R_u)$，$\mathbf{\mathcal{X}}_{ref} = [\mathbf{x}_{ref}^T, \dots, \mathbf{x}_{ref}^T]^T$

Plugging $J$ into $
\mathbf{X}_k = \mathbf{M} \mathbf{x}_k + \mathbf{C} \mathbf{U}_k + \mathbf{D}
$, we can get the regressed form with constants eliminated

$$
\begin{aligned}
\min_{\mathbf{U}_k} \quad & \frac{1}{2} \mathbf{U}_k^T \mathbf{H} \mathbf{U}_k + \mathbf{G}^T \mathbf{U}_k \\
\text{s.t.} \quad & \mathbf{U}_{min} \le \mathbf{U}_k \le \mathbf{U}_{max}
\end{aligned}
$$

Where

$$
\mathbf{H} = 2 \left( \mathbf{C}^T \mathbf{\bar{Q}} \mathbf{C} + \mathbf{\bar{R}} \right) \\
\mathbf{G} = 2 \mathbf{C}^T \mathbf{\bar{Q}} \left( \mathbf{M}\mathbf{x}_k + \mathbf{D} - \mathbf{\mathcal{X}}_{ref} \right)
$$

Finally we can call `solve_qp` for this.



















