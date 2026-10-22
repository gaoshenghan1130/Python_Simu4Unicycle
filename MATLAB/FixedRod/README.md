# 机械固定居中杆模型

直接打开并运行本目录的 `run_simulation.m`。默认保留 gamma PD，横向无控制，严格锁定 r=0；杆质量和惯量取自 Full 配置。

顶部可修改速度、初始侧倾、初始 gamma、初始偏航率、PD 增益和仿真时长。`c.kp=c.kd=0` 表示完全开环。解析特征值要求 `p.BP=0`，代码对此显式检查。

状态顺序：`[sigma1;sigma2;sigma3;sigma_g;psi;theta;phi;gamma;xG;yG]`。
约束是 `sigma_r=-R*sigma1`；F 为锁定反力，不是控制输入。

运行检查：

```matlab
addpath('/Users/gaoshenghan/Desktop/Matlab_Simu4Unicycle/MATLAB/FixedRod');
test_fixed_rod
```

仿真结果和图片写入本目录 `results/`。当前目录的同名 main 可能与 Full 的 `run_simulation.m` 冲突，请用编辑器打开这里的文件运行，或使用绝对路径 `run(...)`。

推导见 [FixedRod_Model_Derivation.md](../../Derivation/FixedRod/FixedRod_Model_Derivation.md)。
