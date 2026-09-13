# Unicycle MATLAB simulation

解压后，在 MATLAB 打开并运行 `run_simulation.m`。无需额外 toolbox。

| 路径 | 内容 |
|---|---|
| `docs/model.md` | 符号定义、参数、完整非线性模型（Markdown + LaTeX） |
| `config/model_parameters.m` | 物理参数，默认杆质量 2.3 kg |
| `config/controller_parameters.m` | PD 增益、参考值、力/力矩限幅 |
| `config/simulation_settings.m` | 初始条件、时长、ODE 精度 |
| `model/` | 质量矩阵、动力学残差、运动学、初始状态及公开 RHS |
| `controllers/pd_controller.m` | 横向 theta/r PD 与纵向 gamma PD |
| `simulation/` | ode45 仿真及绘图 |

公开模型接口：`dx = model_rhs(t,x,u,p)`，其中 `u=[F;M2]`。
`F=-F1` 的符号变换已在模型内部处理；外部控制器不要再反号。

状态：`x=[sigma1;sigma2;sigma3;sigma_r;sigma_g;psi;theta;phi;r;gamma;xG;yG]`。
物理速度：`theta_dot=x(1)`、`r_dot=p.R*x(1)+x(4)`、`gamma_dot=x(5)-x(3)*tan(x(7))`。

默认参数、初始条件和 PD 增益沿用原 notebook 最后一个 cell；模型方程未改变。
纵向 PD 只控制 gamma，不控制前进速度。默认无限幅、无杆行程约束，80 度侧倾终止。
`output_dt` 是输出时间间隔；控制器连续求值，不是离散采样控制。
运行结果保存在 `result.t`、`result.X`、`result.U`；不自动保存图像或数据。

后续做极点配置时可直接对开环 `model_rhs` 线性化，控制器与模型互相独立。

验证：模型与原 notebook 的数值一致性和闭环仿真已用 Python 检查。
本环境没有 MATLAB/Octave，尚未在 MATLAB 中执行本项目。
