# Full nonlinear unicycle

日常只使用两个 main，均可直接在 MATLAB 中打开并运行。

| 入口 | 用法 |
|---|---|
| `run_simulation.m` | 在 `parameter_groups` 中选择一个或多个参数组，依次运行并叠加曲线 |
| `run_parameter_sweep.m` | 选择一个 `parameter_group`，填写 `parameter` 和 `values`，依次尝试 |

## 1. 按参数组运行

```matlab
parameter_groups={'mate_open_loop'};
% parameter_groups={'mate_open_loop','mate_current_mass','chi_balance'};
```

所有参数组集中在 `config/simulation_preset.m`，两个入口共用。每组从 `config/experiment_settings.m` 及其调用的配置文件重新加载默认值，再应用组内覆盖值。

| 参数组 | 设置 |
|---|---|
| `configured` | 使用 config 中的控制器、物理参数和初值 |
| `rolling_balance` | 非零速度下仅配置前四个横向变量；当前默认 |
| `chi_epsilon_balance` | 在 chi_balance 的基础上加入 epsilon=yG，横向配置 6 个极点 |
| `chi_balance` | 含 chi 的极点配置；mr=2.3、BR=6.5、设计/初速=2.375、theta0=1°、psi0=0、30 s |
| `mate_open_loop` | F=0，gamma PD；mr=0.272、BR=BP=0、初速=2.375、theta0=1°、15 s |
| `mate_current_mass` | 与上一组相同，但保留模型配置中的 mr |
| `mate_configured` | F=0，gamma PD，BR=BP=0；初值、质量和 PD 增益来自配置 |

`mate_open_loop` 保留此前的参数值；质量变化不会自动缩放 JR。参数组中显式设置的值优先于基础配置。此入口始终关闭扫描。

## 2. 单参数数列扫描

在 `run_parameter_sweep.m` 顶部修改：

```matlab
parameter_group='mate_open_loop';
parameter='parameters.mr';
values=[0.272 1 2.3];
parameter_index=[];
```

只想为本次扫描修改基础组，可在 `experiment=simulation_preset(...)` 后添加覆盖值。每轮从同一个基础组开始，按数列顺序执行「修改参数 → 生成控制器 → 仿真」，不累积上一轮的参数。

| 参数 | 数列示例 | index |
|---|---|---|
| `parameters.BR` | `[0 3 6.5]` | `[]` |
| `settings.theta0` | `[0.1 1 5]*pi/180` | `[]` |
| `settings.forward_speed0` | `[1.5 2 2.375 3]` | `[]` |
| `controller.kp_gamma` | `[2 3 6]` | `[]` |
| `design.chi_poles` | `[-1.4 -1.6 -1.8]` | `1`，使用 chi_balance |

角度输入使用弧度。向量/矩阵元素使用 MATLAB 线性索引。扫描 design 需要 pole_placement 模式；未生效的控制器字段会报错。普通字段扫描只改变指定字段；特殊参数 `speed` 同时改变初始速度与目标／设计速度。极点配置模式会按每轮参数重新生成 K；固定 K 的比较应使用 direct 模式。

## 结果与文件

两个入口均生成 `results(k)`（各轮时间、状态、输入、实际参数、控制器、初值、设计参数及终止事件），并保留 `result=results(1)`。统一显示八项动力学曲线以及航向、omega3、前进速度；不自动保存文件。

- `config/`：基础配置与共享参数组。
- `simulation/`：扫描、积分、绘图。
- `controllers/`、`model/`、`analysis/`：控制器、完整非线性模型及分析函数。
- `tests/`：回归检查。在 Full 目录运行 `addpath('tests'); test_experiment`。
- `archive/`：旧专题实验和 [历史说明](archive/historical_notes.md)，用于复现历史报告，不作为日常 main。旧脚本的项目根路径已调整，结果仍写入原目录。

状态为 `x=[sigma1;sigma2;sigma3;sigma_r;sigma_g;psi;theta;phi;r;gamma;xG;yG]`。theta 表示侧倾；直线 +x 参考下 chi=psi。降阶极点配置不代表完整状态渐近稳定，具体推导见 [模型报告](../../Derivation/FullModel/FullModelPolePlacement_withDamping.md)。

## epsilon 路径反馈

`run_simulation.m` 选择 `chi_epsilon_balance` 时，反馈状态为
`[theta,r,sigma1,sigma_r,chi,epsilon]`，直线 +x 参考下 `chi=psi`、`epsilon=yG`。
在 `config/simulation_preset.m` 中修改该组的 `design.chi_epsilon_poles`（6 个极点）。
初始 epsilon 通过 `settings.yG0` 设置；`run_parameter_sweep.m` 当前扫描匹配的初始／目标速度。
航向图增加 epsilon 曲线。原来的 chi_balance 五极点模式仍可选。

加入 epsilon 后完整 12 状态仍有两个零极点，不能据此声称所有初值都收敛到零误差。
`addpath('analysis'); report=verify_epsilon_equilibria();` 可验证精确非线性相对平衡族，
并比较纯 epsilon 扰动、纯 theta 扰动及平衡族上的初值。
推导及数值结果见 [epsilon 平衡验证](../../Derivation/FullModel/EpsilonPolePlacementEquilibria.md)。

## 初始速度与目标速度同步扫描

同步速度扫描的设置示例：

```matlab
parameter_group='chi_epsilon_balance';
parameter='speed';
values=[0.5 1 1.5 2 2.375 3]; % m/s
parameter_index=[];
```

每轮同时设置 `settings.forward_speed0` 和 `design.forward_speed`，再重新计算 K，
因此初始速度和目标速度始终取同一个扫描值。实际运动速度仍由动力学决定，不会被强行锁定。
保留该参数组的 theta0=1° 等其余参数，每轮仿真 30 s。
此脚本显式使用项目内置 `acker`，不依赖 Control System Toolbox，也不改动全局算法设置。
`speed` 仅适用于滚动极点配置且纵向反馈启用的情况，数列不能包含 0。
若只想扫描初速或目标速度，分别使用 `settings.forward_speed0` 或 `design.forward_speed`。

## 仿真过程中在线重新线性化

运行 `run_mate_speed_relinearization.m` 可按 Máté 之前采用的流程进行在线 gain scheduling。ODE 每次计算控制输入时，都读取当前模拟状态的实际纵向速度 `v(t)=R*sigma2(t)`，在该速度对应的直线滚动状态重新线性化完整模型，重新进行 pole placement 得到 `K(t)`，然后计算本次控制输入。它不是只在每轮仿真开始前重新设计一次。

每轮初始速度与固定目标速度相等；仿真过程中目标速度保持不变，但线性化速度和 K 随实际速度变化。期望极点保持不变。默认比较 `theta0=0 deg` 和 `theta0=1 deg`，两者的初始偏航角速度均为零，并扫描多个临界速度以下的目标速度。

滚动路径状态在零速度处失去可控性，因此脚本在实际速度降至 `min_design_speed` 时停止该轮，而不会静默切换为静止控制器。这里仍使用本仓库的完整模型和当前物理参数，并不声称数值参数与 Máté 的模型完全相同。结果保存到 `results/mate_online_relinearization/`。

## 仅配置前四个横向变量

两个 main 当前选择 `rolling_balance`，使用 `design.lateral_states='balance_rolling'`。
横向反馈状态仅为 `[theta,r,sigma1,sigma_r]`；psi、epsilon 和 sigma3 的直接反馈增益为零。
在 `config/simulation_preset.m` 的 rolling_balance 分支修改：

```matlab
experiment.design.rolling_poles=[-1.6 -1.8 -2 -2.2];
```

纵向极点配置保留。此模式仍在当前非零设计速度处线性化，并沿用
`sigma3=a*theta` 的降阶设计关系，不是原来的静止 `balance` 模式。
速度扫描继续同步设置初始与目标速度，并保留入口中的速度数列。
使用内置 Ackermann 实现，无需 Control System Toolbox。五阶、六阶模式仍可通过参数组选择。
本次按要求仅修改代码，未运行验证。
