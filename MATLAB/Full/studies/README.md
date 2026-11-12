# 专题研究

在 Full 目录运行 `run_study('list')` 查看名称；`report=run_study('名称')` 执行。
脚本型专题的参数在本目录对应文件顶部修改；函数型专题可通过 `run_study` 继续传参。

| 名称 | 实现 | 类型 |
|---|---|---|
| `equilibrium_balance` | `run_equilibrium_balance_study.m` | 脚本 |
| `mate_speed_relinearization` | `run_mate_speed_relinearization.m` | 脚本 |
| `allowable_theta_vs_speed` | `run_allowable_theta_vs_speed.m` | 脚本 |
| `zero_yaw_manifold` | `run_zero_yaw_manifold_study.m` | 脚本 |
| `manifold_hardware` | `run_manifold_hardware.m` | 函数 |
| `manifold_hardware_tuning` | `run_manifold_hardware_tuning.m` | 函数 |
| `manifold_overshoot` | `run_manifold_overshoot_study.m` | 函数 |
| `direct_upright_turning` | `verify_direct_upright_turning.m` | 函数 |

本目录脚本默认 `save_figures=false`，显示图窗并保留原有 MAT/CSV 数值结果保存。
需要 PNG 时将对应脚本的 `save_figures` 改为 `true`。

`analysis/manifold/` 保持原样，因此调用其中分析函数的专题仍沿用其内部的图片和数据保存逻辑；
尤其 `zero_yaw_manifold` 调用的 `straight_test` 和 `manifold_overshoot` 会保存图片。
历史生成图已经清理，以下表格是既有实验记录，未重新计算。

## 仿真过程中在线重新线性化

运行 `run_study('mate_speed_relinearization')`，参数在 `studies/run_mate_speed_relinearization.m` 中修改 可按 Máté 之前采用的流程进行在线 gain scheduling。脚本先在速度网格上分别重新线性化完整模型并进行 pole placement，得到 `K(v)` 表。固定时间步循环每一步读取当前模拟速度 `v(t)=R*sigma2(t)`，在相邻设计点之间插值得到 `K(t)`，然后用 forward Euler 更新完整非线性状态。该入口不使用 `ode45`。

每轮初始速度与固定目标速度相等；仿真过程中目标速度保持不变，但线性化速度和 K 随实际速度变化。期望极点保持不变。默认比较 `theta0=0 deg` 和 `theta0=1 deg`，两者的初始偏航角速度均为零，并扫描多个临界速度以下的目标速度。

滚动路径状态在零速度处失去可控性，因此脚本在实际速度降至 `min_design_speed` 时停止该轮，而不会静默切换为静止控制器。这里仍使用本仓库的完整模型和当前物理参数，并不声称数值参数与 Máté 的模型完全相同。结果保存到 `results/mate_online_relinearization/`。

速度表间隔由 `gain_schedule_step` 设置，默认 `0.01 m/s`。减小它可更精细地逼近每次按当前速度重新设计的结果，但会增加启动时的预计算时间。若用于诊断并确实需要在每次控制器求值时重新计算 Jacobian，可将脚本中的 `gain_schedule.exact` 改为 `true`；该模式会非常慢。

固定积分步长由脚本中的 `fixed_step` 设置，默认 `0.001 s`，与 `MATLAB/Lat/simulation/timeConstantSimu.m` 一样采用显式 forward Euler。增大步长会加快运行，但也会降低数值精度和稳定性。

## 初始侧倾允许范围随速度变化

运行 `run_study('allowable_theta_vs_speed')`，参数在 `studies/run_allowable_theta_vs_speed.m` 中修改。脚本对每个速度重新线性化一次并计算固定 K，然后从 `theta0=0` 分别向正、负方向搜索能完成 30 s 仿真的最大初始侧倾，绘制允许区间。初始速度始终等于设计／参考速度，初始偏航角速度为零。大量边界试验使用固定步长 forward Euler；步长由 `fixed_step` 设置。

当前“allowable”要求未触发 80° 侧倾终止、完成整个仿真，并在最后5秒满足 theta、r、psi、gamma 的速度及前进速度误差阈值。阈值集中在脚本顶部的 `acceptance` 中。该定义仍不包含杆行程或执行器饱和。搜索上限和角度精度分别由 `maximum_test_angle` 和 `angle_tolerance` 设置；若边界达到搜索上限，结果表中的 `*_limit_capped` 会标记该值只是下界。结果保存在 `results/allowable_theta_vs_speed/`。

## Manifold LQR：1.2 m/s、7.5 cm 杆行程调参

运行 `report=run_study('manifold_hardware')`，默认初始侧倾 1.5°、速度 1.2 m/s，
杆质量 2.3 kg、BR=6.5，仿真 30 s。`run_study('manifold_hardware',-1.5,false)`
可验证负侧倾并关闭绘图。原 manifold 实验保留原 Q/R；力限幅为 27.5 N。

选定参数为 `Q=diag([2 1 1 2 1 0.5 20 200 5])`、`R=diag([0.01 0.1])`。
Q 对应 `[sigma1 sigma2 sigma3 sigma_r sigma_g psi theta r gamma]`，
先投影到八维可控子空间，再求 LQR；K 随目标平衡点重新计算。

| 初始侧倾 | 速度 m/s | 原杆峰值 cm | 调参后 cm | 原始峰值力 N | 峰值力矩 N m | 筛选通过 |
|---|---|---|---|---|---|---|
| ±1° | 1.2 | 4.6674 | 4.08 | 14.5266 | 0.0339 | 是 |
| ±1.5° | 1.2 | 6.9024 | 6.017 | 21.7456 | 0.0755 | 是 |
| 2° | 1.2 | 9.0474 | 7.866 | 28.9138 | 0.1323 | 否 |
| 3° | 1.2 | 提前终止 | 11.49 | 43.0491 | 0.2862 | 否 |
| 1.5° | 1.1 | 7.8625 | 6.73 | 24.9830 | 0.0736 | 是 |
| 1.5° | 1.3 | 6.0327 | 5.37 | 19.0173 | 0.0738 | 是 |

表中力为限幅前请求；实际施加的力始终限幅在 ±27.5 N。
1.2 m/s、±1.5° 的最后 5 s 力峰值约 0.2864 N，最终侧倾约 ±0.0064°。
因此通过行程/输入/稳态变化率筛选，但仍不满足原函数 0.001° 的精确直立判据。

`study=run_study('manifold_hardware_tuning')` 可复现 27 组权重搜索，以及 0.005 s
最大积分步长的正负侧倾和邻近速度复核；可用 `(speed,angles)` 自定义搜索工况。
结果位于 `results/manifold_hardware_tuning_v1.2/`。旧 2.375 m/s 数值结果保存在
`results/manifold_hardware_tuning/`，不再是当前默认配置。
搜索筛选要求完成 30 s、max|r|<0.075 m、原始力/力矩请求不超过
27.5 N/15 N m、最后 5 s 力不超过连续力 9.2 N，以及终态相对变化率 <1e-4。
`passed` 仅表示这些仿真筛选条件；`exact_target_reached` 单独记录精确目标条件。
2° 在本轮 27 组搜索中未找到通过项；这不证明所有 LQR 权重均不可行。

上述结果只覆盖表中离散工况，不构成连续吸引域证明。
模型使用连续状态反馈，尚未包括硬件 10 ms 采样保持、IMU 延迟/噪声和热模型；
短时力超过 9.2 N，不能据此证明热约束通过。15 N m 沿用既有仿真力矩限幅，
尚未独立核实为实际轮电机额定值。
