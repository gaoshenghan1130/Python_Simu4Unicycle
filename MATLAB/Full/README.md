# Full nonlinear unicycle：控制器模式与参数扫描

在 MATLAB 打开并运行 `run_simulation.m`。控制器模式以 `experiment_settings.m` 的当前设置为准；完整非线性方程未改变。默认和 `acker` 极点配置均无需额外 toolbox。

## 1. 选择控制器并运行

主要入口：`config/experiment_settings.m`。设置：

```matlab
e.controller.mode='pd';             % 原来的 PD 增益
% e.controller.mode='direct';       % 直接输入状态反馈矩阵 K
% e.controller.mode='pole_placement'; % 每次运行重新生成 K
```

- `pd`：在 `config/controller_parameters.m` 修改 `kp_theta`、`kd_theta`、`kp_r`、`kd_r`、`kp_gamma`、`kd_gamma`、参考值和限幅。保留原控制律符号：横向为参考值减状态，纵向 gamma 为状态减参考值。
- `direct`：在入口设置 `e.controller.K`（2×12）和 `e.controller.x_ref`（12×1）。控制律是 `u=-K*(x-x_ref)`，K 必须显式提供；不能把 PD 系数不经转换直接填进去。
- `pole_placement`：在 `config/pole_placement_settings.m` 修改极点；按本轮物理参数线性化、生成 K，再运行非线性仿真。参考状态固定为静止直立零状态。

直接给分块增益的示例（先把示例变量替换为你的数值）：

```matlab
e.controller.mode='direct';
e.controller.K=zeros(2,12);
e.controller.K(1,[7 9 1 4])=[Ktheta Kr Ksigma1 Ksigma_r];
e.controller.K(2,[8 10 2 5])=[Kphi Kgamma Ksigma2 Ksigma_g];
```

`F=-F1` 已在模型内部处理，外部控制器不要再反号。所有模式均使用 `force_limit`、`torque_limit` 限幅。

## 2. 一次扫描一个参数并叠加绘图

在 `experiment_settings.m` 中：

```matlab
e.controller.mode='pole_placement';
e.sweep.enabled=true;
e.sweep.parameter='parameters.mr';
e.sweep.values=[1.5 2.3 3.0];
e.sweep.index=[];
```

每一轮均从同一份基础配置开始，依次执行 **修改参数 → 生成控制器 → 仿真**。因此物理参数扫描在 pole placement 模式下同时改变模型和所生成的 K；若希望固定 K 比较不同模型，请使用 direct 模式。

| 目标字段 | 示例数列 | index | 模式 |
|---|---|---|---|
| `parameters.mr` | `[1.5 2.3 3]` | `[]` | 全部 |
| `parameters.BR` | `[0 0.02 0.05]` | `[]` | 全部 |
| `settings.theta0` | `[1 2.8 5]*pi/180` | `[]` | 全部 |
| `controller.kp_gamma` | `[2 3 4]` | `[]` | pd |
| `design.lateral` | `[-1 -2 -3]` | `1` | pole_placement |
| `controller.K` | `[-700 -800 -900]` | `13` | direct，K(1,7) |

向量/矩阵元素使用 MATLAB 线性索引；例如 2×12 的 K(1,7) 对应索引 13。数列为空、目标字段不存在、未指定向量索引或扫描未生效的控制器字段会报错。每轮参数值必须符合物理模型；一次只扫描一个实数参数。复极点必须成共轭对，完整复极点组合请在极点文件中编辑。

8 个子图分别叠加 theta、r、theta_dot、r_dot、gamma、gamma_dot、F、M2；图例显示参数和值。每轮使用自己的时间网格，侧倾提前终止会在命令行报告。

## 3. 清晰配置多个极点

在 `config/pole_placement_settings.m`：

```matlab
d.lateral      = [-2.25 -1.25 -2.00 -1.50];
d.longitudinal = [-3.00 -3.50 -4.00 -4.50];
d.method='acker';
```

横向分块为 `[theta,r,sigma1,sigma_r]`，纵向分块为 `[phi,gamma,sigma2,sigma_g]`，各指定 4 个极点，单位 s⁻¹。极点属于整个分块，**不是每个状态单独对应一个极点**。

- `acker`：内置 SISO Ackermann 公式，无需 toolbox，支持重复极点；如 `[-2 -2 -2 -2]`。
- `place`：需要 Control System Toolbox；本模型每个分块单输入，因此 4 个极点必须互异。
- 支持复共轭极点，例如 `[-2+1i -2-1i -3 -4]`；要求所有极点实部为负。
- 实际闭环极点、请求极点、A/B 和分块增益保存在 `results(k).controller.design_info`。重复根在数值计算中可能轻微分裂，因此同时用特征多项式验证配置结果。

也可以只计算：`[K,info]=pole_placement(model_parameters(),pole_placement_settings());`（先运行入口以添加路径）。无参数、无输出调用 `pole_placement` 会打印默认增益。

## 模型与论文/邮件的关系

本项目保留 12 维旧坐标模型：

`x=[sigma1;sigma2;sigma3;sigma_r;sigma_g;psi;theta;phi;r;gamma;xG;yG]`，输入 `u=[F;M2]`。

物理速度为 `theta_dot=x(1)`、`r_dot=R*x(1)+x(4)`、`gamma_dot=x(5)-x(3)*tan(x(7))`。

附件 PDF 使用 θ 表示路径角误差、ϑ 表示侧倾；邮件用 χ 表示路径角误差。本代码中的 theta 则始终表示侧倾，不能按符号名称直接对照。

按邮件，对于沿 x 轴的直线路径，χ=ψ、ε=y，因此不需要为此次整理扩展到 15 维路径坐标系统。附件论文第 IV 节围绕直线滚动状态设计控制器，使用路径误差和冗余状态消除后的控制输出；它和这里的静止直立分块不是同一个设计。

`lateral_states='balance'` 的 pole placement 沿用原脚本的 **零速度平衡点**，8 个可控状态，另外 4 个零极点不被配置（含偏航积分链）。它不是整个 12 维状态的渐近稳定或路径跟踪控制器。仿真初速度默认仍为原值 0.2 m/s，这只是非线性初值，并不改变线性化设计速度；该控制器会尝试停车。需要从静止测试时设置 `e.settings.forward_speed0=0`。

论文式 (36) 后的重复 −12 极点不能直接视作本模型的验证参数。默认极点为原脚本的示例值，未做实验调参。原 PD 的纵向只控制 gamma，不控制前进速度。默认无限幅、无杆行程约束，80 度侧倾终止；`output_dt` 是输出间隔，控制器连续求值。

## 文件和结果

| 文件 | 职责 |
|---|---|
| `config/experiment_settings.m` | 控制器模式、单参数扫描和本次运行覆盖值 |
| `config/controller_parameters.m` | 原 PD 增益、直接反馈设置、限幅 |
| `config/pole_placement_settings.m` | 全部期望极点与算法选择 |
| `config/model_parameters.m` / `simulation_settings.m` | 物理参数 / 仿真初值及精度 |
| `analysis/pole_placement.m` | 线性化、可控性检查、分块配置及验证 |
| `controllers/generate_controller.m` / `controller_output.m` | 生成控制器 / 统一求值 |
| `simulation/run_experiment.m` | 参数覆盖、生成、仿真的循环 |
| `simulation/plot_simulation.m` | 多次运行叠加绘图 |
| `model/` / `docs/model.md` | 原非线性方程 / 符号与推导 |

`results(k)` 保存 t、X、U、本轮参数、控制器、仿真设置、极点设置、标签和终止事件；`result=results(1)` 保留旧的单结果使用方式。不自动保存数据或图片。

回归验证：在 Full 目录中运行 `addpath('tests'); test_experiment`，检查原 PD 求值一致性、三种极点组合、参数变化后重新设计、直接增益复现、限幅、索引扫描、输入错误和多曲线绘图。

实测（MATLAB R2026a）：默认 PD 跑完 15 s。原脚本的示例极点在默认初值下，杆质量 `[1.5 2.3 3]` kg 扫描分别约在 `[1.906 1.594 1.436]` s 触发 80° 侧倾终止；示例极点不是已验证稳定的非线性控制参数。


## 新增：加入 χ 的直线滚动示例

直接运行 `run_chi_simulation.m`。它使用现有完整非线性模型，沿 +x 轴的参考航向为零，χ=psi；这里 theta 仍表示侧倾角。初始条件统一来自 `simulation_settings.m`（通过 `experiment_settings.m` 读取），脚本不再覆盖 theta0、gamma0、psi0 或 forward_speed0。脚本关闭扫描。设计速度 `d.forward_speed` 是参考速度，与初始速度独立。

`config/pole_placement_settings.m` 中：

```matlab
d.lateral_states='balance_chi'; % 常规入口启用 χ；示例脚本自动覆盖此项
d.forward_speed=0.2;            % 必须非零，直线滚动参考速度
d.chi_poles=[-2 -2.5 -3 -3.5 -4]; % 5 个横向极点
```

`balance_chi` 的反馈状态顺序是 `[theta,r,sigma1,sigma_r,chi]`；`d.lateral` 仅供静止 `balance` 模式使用。纵向仍使用 `d.longitudinal` 的 4 个极点，但参考 phi 和 xG 随时间前进，从而跟踪滚动状态而非停车。χ 是反馈状态，不存在一个仅属于 χ 的独立极点；5 个极点共同决定横向动态。

对当前 full model 在指定滚动速度下数值线性化，检查并在设计中消去不变量 `sigma3-a*theta`，其中 `a=A(3,1)`，不能直接照搬简化论文的系数。仿真保留全部 12 个状态。全系统仍有 3 个未配置零极点；不保证任意初值下全状态收敛，也不控制横向位置 epsilon。尤其初始不变量不为零时，不应把降阶极点稳定等同于完整状态回零。

先前专用初值测试（theta0=0、gamma0=0、psi0=5°、forward_speed0=0.2 m/s；并非当前配置初值）的实测结果（MATLAB R2026a，15 s）：χ 从 5° 降至约 0.000267°，最大侧倾约 4.526°，最大杆位移约 0.06769 m，末端速度 0.2 m/s，无侧倾终止。会额外显示 χ、omega3、前进速度曲线。

扫描 χ 设计极点可在常规入口设 `e.design.lateral_states='balance_chi'`、`e.sweep.parameter='design.chi_poles'`、`e.sweep.index=1`，再给出 `e.sweep.values`。扫描 `design.forward_speed` 改变设计和参考速度，不会自动修改 `settings.forward_speed0`。

验证：`addpath('tests'); test_chi_controller`。


## 杆阻尼与近原点极点速度扫描（当前配置）

`BR` 已设为 **6.5 N·s/m**，对应物理阻尼力 `-BR*r_dot`；模型原本已有该项。当前可配置极点已调整至 **−0.8 至 −1.2 s⁻¹**，具体以 `pole_placement_settings.m` 为准，前文的极点数值仅作为编辑示例。

运行 `run_damping_speed_study.m` 可复现 8 个速度、两组初值的 60 s 扫描，每个速度重新设计控制器并同步设置初始速度。当前配置初值（theta0=2.8°、gamma0=0.1 rad）全部未通过；相容的 0.1° 小航向扰动组全部通过末段收敛判据。线性闭环仍有 3 个零极点，不能声称完整状态渐近稳定。

完整推导、增益表、稳定性结果、测试条件及曲线见 [FullModelPolePlacement_withDamping.md](../../Derivation/FullModel/FullModelPolePlacement_withDamping.md)。数据输出至 `results/damping_speed_study/`。扫描额外设有 |r|=2 m 的数值发散终止条件，这不是硬件限位；普通仿真仍采用原有侧倾终止。


## run_chi_simulation：匹配初速并保持 gamma 为零

当前 `run_chi_simulation.m` 将设计/目标速度设置为 `simulation_settings.m` 的 `forward_speed0`（本次按用户选择设为 0.2 m/s），并显式将 gamma0、gamma_dot0 设为 0。它启用 `controller.hold_gamma_zero=true`，通过完整非线性动力学计算所需 M2，保持 gamma 为零到积分误差范围内；没有把状态轨迹或绘图数据强制清零。

这会替代原纵向极点反馈，`d.longitudinal` 的极点不再代表实际纵向闭环极点。实际速度不被强制锁定，可能随横向运动变化。横向仍按含 chi 的配置生成控制器；若以后把初速设为 0，脚本明确退回静止 balance 模式，因为静止时 chi 不可控。

脚本里的 `test_initial_perturbations=true` 会在你的配置初值之外，额外运行三组隔离扰动：chi0=0.1°、theta0=0.01°、theta0=0.1°，并在同一批图里叠加。设为 false 则只运行配置初值。所有试验的初速与目标速度相同。

控制器使用理想无限幅力矩，保证零 gamma 的模型约束；不能同时宣称原纵向速度控制器仍在工作。实现公式和本次结果补充在 `Derivation/FullModel/FullModelPolePlacement_withDamping.md`。参数以当前配置文件为准，本次测试时用户文件的 BR 已为 0，未擅自改回上轮实验的 6.5。
