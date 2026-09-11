# 独轮车：符号与非线性模型

$\theta$ 对应 notebook 的 $\vartheta$；全部参数使用 SI 单位。MATLAB 模型输入为 $u=[F,M_2]^T$。

## 状态

$$
q=\begin{bmatrix}\psi&\theta&\phi&r&\gamma&x_G&y_G\end{bmatrix}^{T},
\qquad
\sigma=\begin{bmatrix}\sigma_1&\sigma_2&\sigma_3&\sigma_r&\sigma_g\end{bmatrix}^{T}.
$$

$$
\boxed{x=\begin{bmatrix}\sigma_1&\sigma_2&\sigma_3&\sigma_r&\sigma_g&\psi&\theta&\phi&r&\gamma&x_G&y_G\end{bmatrix}^{T}}
$$

| MATLAB 索引 | Notebook 名称 | 含义 | 单位 |
|---|---|---|---|
| 1 | `σ1` | $\dot\theta$，侧倾角速度 | rad/s |
| 2 | `σ2` | 车轮角速度在轮轴方向的分量 | rad/s |
| 3 | `σ3` | 车轮角速度在倾斜坐标系第三轴的分量 | rad/s |
| 4 | `σr` | $\dot r-R\dot\theta$ | m/s |
| 5 | `σg` | $\dot\gamma+\dot\psi\sin\theta$ | rad/s |
| 6 | `ψ` | 航向角 / yaw | rad |
| 7 | `ϑ` | 侧倾角 / lean | rad |
| 8 | `φ` | 车轮转角 | rad |
| 9 | `r` | 杆质心沿轮轴的有符号位移 | m |
| 10 | `γ` | 机身/倒立摆绕轮轴的倾角 | rad |
| 11 | `xG` | 轮心的全局 $x$ 坐标 | m |
| 12 | `yG` | 轮心的全局 $y$ 坐标 | m |

这里 `xG,yG` 根据其运动学对应论文的轮心坐标 $x_C,y_C$，不是整车总质心坐标，也不是地面接触轨迹坐标。

## 伪速度

$$
\boxed{
\begin{aligned}
\sigma_1&=\dot\theta,\\
\sigma_2&=\dot\phi+\dot\psi\sin\theta,\\
\sigma_3&=\dot\psi\cos\theta,\\
\sigma_r&=\dot r-R\dot\theta,\\
\sigma_g&=\dot\gamma+\dot\psi\sin\theta.
\end{aligned}}
$$

因此，不能把 $\sigma_r$ 当成 $\dot r$，也不能在一般运动下把 $\sigma_g$ 当成 $\dot\gamma$。

## 输入

原始 `Cfcn` 的输入为

$$u_{\rm nb}=\begin{bmatrix}F_1&M_2\end{bmatrix}^{T}.$$

仿真控制器先计算 $F$，再通过 `return np.array([-F, M2])` 传给模型。因此本文设计用输入为

$$\boxed{u=\begin{bmatrix}F&M_2\end{bmatrix}^{T},\qquad F=-F_1.}$$

$F_1$ 对应作用于车轮的横向内力约定，$F$ 对应相反方向、作用于移动杆的力约定；$M_2$ 为作用于车轮的驱动力矩，机身承受反力矩。单位分别为 N 和 N·m。

## 参数

参数数组顺序严格为

$$p=[g,R,m_w,J_{W1},J_{W2},m_r,J_R,B_R,h,m_p,J_{Px},J_{Py},J_{Pz},B_P].$$

| 参数 | 含义 | `ps_sim` 数值 | 单位 |
|---|---|---:|---|
| $g$ | 重力加速度 | 9.81 | m/s² |
| $R$ | 轮半径 | 0.253 | m |
| $m_w$ | 车轮质量 | 2.436 | kg |
| $J_{W1}$ | 车轮绕轮轴 $y$ 的惯量 | 0.09099921839 | kg·m² |
| $J_{W2}$ | 车轮绕 $x,z$ 的惯量 | 0.04591427768 | kg·m² |
| $m_r$ | 整个移动杆组件质量 | 2.3 | kg |
| $J_R$ | 杆绕其质心 $x,z$ 的惯量 | 0.0517629 | kg·m² |
| $B_R$ | 杆相对运动的粘性阻尼系数 | 0 | N·s/m |
| $h$ | 轮心至机身质心距离 | 0.025 | m |
| $m_p$ | 机身/倒立摆质量 | 2.799 | kg |
| $J_{Px}$ | 机身绕其质心 $x$ 的惯量 | 0.01290418213 | kg·m² |
| $J_{Py}$ | 机身绕其质心 $y$ 的惯量 | 0.02090219895 | kg·m² |
| $J_{Pz}$ | 机身绕其质心 $z$ 的惯量 | 0.01118711607 | kg·m² |
| $B_P$ | 车轮与机身之间的转动粘性阻尼 | 0 | N·m·s/rad |

惯量应理解为模型中的质心惯量；代码另外包含 $m_ph^2$、$m_rr^2$ 等平行轴项。若之后换用 CAD 数据，需要确认参考点，避免重复计入平行轴项。

## 完整非线性模型

### 1 总体形式

为了区分 notebook 的 `C` 与论文的惯性力向量，本文把重力、惯性项记为 $n(q,\sigma)$，把粘性项单独写出：

$$
\boxed{M(q)\dot\sigma+n(q,\sigma)+D\sigma=Q_u u,}
$$

$$
Q_u=\begin{bmatrix}
R&0\\0&1\\0&0\\1&0\\0&-1
\end{bmatrix},\qquad
D=\begin{bmatrix}
B_RR^2&0&0&B_RR&0\\
0&B_P&0&0&-B_P\\
0&0&0&0&0\\
B_RR&0&0&B_R&0\\
0&-B_P&0&0&B_P
\end{bmatrix}.
$$

注意第一项 $B_RR^2$ 表示 $B_R\,R^2$。阻尼对应的相对速度为

$$\dot r=R\sigma_1+\sigma_r,\qquad \dot\phi-\dot\gamma=\sigma_2-\sigma_g.$$

与原代码的关系为

$$C_{\rm nb}[0:5]=n+D\sigma-Q_u\begin{bmatrix}-F_1\\M_2\end{bmatrix}.$$

定义 $k(q,\sigma)=\dot q$，则

$$\boxed{\dot x=f(x,u)=\begin{bmatrix}M(q)^{-1}(Q_u u-n-D\sigma)\\k(q,\sigma)\end{bmatrix}.}$$

实现时使用 MATLAB `M \ (...)`，无需显式求逆。若要写成 12×12 的隐式形式，则质量矩阵为 $\mathcal M=\operatorname{blkdiag}(M,I_7)$。

### 2 质量矩阵

为压缩表达式，定义

$$a=J_{Px}-J_{Pz}+m_ph^2,\qquad b=Rm_ph,\qquad c=J_{Pz}+J_R+J_{W2},$$

$$s_\gamma=\sin\gamma,\quad c_\gamma=\cos\gamma,\quad t_\theta=\tan\theta.$$

这些是本整理文档的缩写，不是论文的 $c_1$。

$$
\boxed{M=\begin{bmatrix}
M_{11}&0&M_{13}&0&0\\
0&M_{22}&-Rm_rr&0&bc_\gamma\\
M_{13}&-Rm_rr&M_{33}&0&0\\
0&0&0&m_r&0\\
0&bc_\gamma&0&0&J_{Py}+m_ph^2
\end{bmatrix}}
$$

其中

$$
\begin{aligned}
M_{11}&=c+R^2(m_p+m_w)+2bc_\gamma+m_rr^2+ac_\gamma^2,\\
M_{13}&=-(b+ac_\gamma)s_\gamma,\\
M_{22}&=J_{W1}+R^2(m_p+m_r+m_w),\\
M_{33}&=c+m_rr^2+as_\gamma^2.
\end{aligned}
$$

### 3 完整重力与惯性项

以下五项逐项对应原始 `Cfcn` 的前五项，已移除输入项与粘性项。

$$
\begin{aligned}
n_1={}&-g\left[R(m_p+m_w)+hm_pc_\gamma\right]\sin\theta
       +gm_rr\cos\theta\\
&+Rm_rr\sigma_1^2+2m_rr\sigma_1\sigma_r\\
&+(b+ac_\gamma)s_\gamma t_\theta\sigma_1\sigma_3
 -2(b+ac_\gamma)s_\gamma\sigma_1\sigma_g\\
&-\left[J_{W1}+R^2(m_p+m_w)+bc_\gamma+Rm_rrt_\theta\right]\sigma_2\sigma_3\\
&+\left[bc_\gamma+m_rr^2+ac_\gamma^2+c\right]t_\theta\sigma_3^2\\
&+\left[J_{Px}-J_{Py}-J_{Pz}-2bc_\gamma-2ac_\gamma^2\right]\sigma_3\sigma_g.
\end{aligned}
$$

$$
\begin{aligned}
n_2={}&-bs_\gamma(\sigma_3^2+\sigma_g^2)-2Rm_r\sigma_3\sigma_r\\
&+\left[R^2(m_p-m_r+m_w)+bc_\gamma+Rm_rrt_\theta\right]\sigma_1\sigma_3.
\end{aligned}
$$

$$
\begin{aligned}
n_3={}&J_{W1}\sigma_1\sigma_2+bs_\gamma\sigma_2\sigma_3
       +ghm_ps_\gamma\sin\theta+2m_rr\sigma_3\sigma_r\\
&+\left[Rm_rr-(m_rr^2+as_\gamma^2+c)t_\theta\right]\sigma_1\sigma_3\\
&+\left[-J_{Px}+J_{Py}+J_{Pz}+2as_\gamma^2\right]\sigma_1\sigma_g\\
&-as_\gamma c_\gamma t_\theta\sigma_3^2
 +2as_\gamma c_\gamma\sigma_3\sigma_g.
\end{aligned}
$$

$$n_4=Rm_r\sigma_2\sigma_3+gm_r\sin\theta-m_rr(\sigma_1^2+\sigma_3^2).$$

$$
\begin{aligned}
n_5={}&bs_\gamma t_\theta\sigma_2\sigma_3-ghm_ps_\gamma\cos\theta\\
&+(b+ac_\gamma)s_\gamma\sigma_1^2
 +(a+bc_\gamma-2as_\gamma^2)\sigma_1\sigma_3
 -as_\gamma c_\gamma\sigma_3^2.
\end{aligned}
$$

这里 $m_rr$ 表示 $m_r\,r$，$m_rr^2$ 表示 $m_r\,r^2$。

### 4 七条运动学方程

$$
\boxed{
\begin{aligned}
\dot\psi&=\frac{\sigma_3}{\cos\theta},\\
\dot\theta&=\sigma_1,\\
\dot\phi&=\sigma_2-\sigma_3\tan\theta,\\
\dot r&=R\sigma_1+\sigma_r,\\
\dot\gamma&=\sigma_g-\sigma_3\tan\theta,\\
\dot x_G&=R\sigma_1\sin\psi\cos\theta+R\sigma_2\cos\psi,\\
\dot y_G&=-R\sigma_1\cos\psi\cos\theta+R\sigma_2\sin\psi.
\end{aligned}}
$$

坐标要求 $\cos\theta\ne0$。
