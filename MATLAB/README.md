# MATLAB 脚本导航

日常完整模型的三个入口位于 [Full/](Full/README.md)：
`run_simulation.m`（仿真）、`run_parameter_sweep.m`（扫描）、`run_study.m`（专题）。

- `Full/`：完整非线性模型。专题实现在 `studies/`，历史实验在 `archive/`。
- `Lat/`：横向／Segway 旧模型。`runner_lat.m` 为横向仿真，`runner.m` 为 Segway 对比，`tuner.m` 为参数拟合；空的 `ucrunner.m` 已移除。
- `FixedRod/`：固定杆模型，运行本目录 `run_simulation.m`，详见本目录 README。
- `friction/`：摩擦数据绘图，保留原始试验数据及源图。

不同模型的同名入口请从编辑器打开对应文件运行，避免 MATLAB 路径冲突。
图片默认仅显示；需要保存时显式开启 `save_figures`。
`Full/analysis/manifold/` 的代码和内部保存行为保持原样。
历史仿真图片已清理，数值数据及报告、推导的源插图保留。
