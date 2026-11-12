# Full nonlinear unicycle

根目录只保留三个入口，可在 MATLAB 编辑器直接运行。

| 入口 | 用途 |
|---|---|
| `run_simulation.m` | 按一个或多个参数组运行、叠加曲线 |
| `run_parameter_sweep.m` | 选择参数组，对一个参数的数列逐项扫描 |
| `run_study.m` | 统一运行专题研究，默认 `manifold_hardware` |

## 普通仿真与扫描

在 `run_simulation.m` 顶部选择 `parameter_groups`，例如：

```matlab
parameter_groups={'mate_open_loop','mate_current_mass','chi_balance'};
```

参数组集中在 `config/simulation_preset.m`：`configured`、`rolling_balance`、
`chi_balance`、`chi_epsilon_balance`、`mate_open_loop`、`mate_current_mass`、`mate_configured`。
每轮重新加载配置并应用覆盖值。普通仿真当前默认 `rolling_balance`。

在 `run_parameter_sweep.m` 顶部填写：

```matlab
parameter_group='chi_epsilon_balance';
parameter='speed';
values=[0.5 1 1.5 2 2.375 3];
parameter_index=[];
```

`speed` 同时修改初始与设计速度，并在每轮重新设计控制器，不能包含 0。
仅修改初速用 `settings.forward_speed0`；仅修改目标速度用 `design.forward_speed`。
也可扫描 `parameters.mr`、`parameters.BR`、`settings.theta0`、`controller.kp_gamma` 等。
角度用弧度；向量字段通过 `parameter_index` 指定 MATLAB 线性索引。
扫描 design 需要 pole_placement 模式，未生效的字段会报错。

两入口返回工作区 `results(k)` 和 `result=results(1)`，显示动力学与航向曲线，不自动保存文件。
`rolling_balance` 仅反馈 `[theta,r,sigma1,sigma_r]`；`chi_balance` 加入 chi；
`chi_epsilon_balance` 再加入 epsilon=yG。各组极点在 `simulation_preset.m` 修改。
降阶极点配置不代表完整 12 状态渐近稳定。

## 专题与目录

```matlab
run_study('list')
report=run_study('equilibrium_balance');
report=run_study('manifold_hardware',-1.5,false);
```

专题脚本移至 `studies/`，参数、名称及既有实验记录见 [专题说明](studies/README.md)。
函数型专题可继续传参，脚本型专题在对应文件顶部修改参数。

- `config/`：物理参数、控制器配置与参数组。
- `model/`、`controllers/`、`simulation/`：动力学、控制律、积分与绘图。
- `analysis/`：独立分析；`analysis/manifold/` 本次保持原样。
- `studies/`：现用专题实现；`archive/`：历史报告复现脚本。
- `tests/`：回归检查；`results/`：需要保存的数值输出。

除 `analysis/manifold/` 的原有行为外，图片默认仅显示，不写入磁盘。
脚本设 `save_figures=true`，分析函数传入末尾的 `save_figures` 参数，才导出图片。
平衡分析通过 `cfg.save_figures=true` 开启；MAT/CSV 数值保存沿用原行为。
历史仿真图片已清理，报告与推导文档的源插图保留。

在 Full 目录验证：`addpath('tests'); test_experiment`。
状态顺序为 `x=[sigma1;sigma2;sigma3;sigma_r;sigma_g;psi;theta;phi;r;gamma;xG;yG]`。
theta 为侧倾；直线 +x 参考下 chi=psi。模型说明见
[完整模型报告](../../Derivation/FullModel/FullModelPolePlacement_withDamping.md)。
