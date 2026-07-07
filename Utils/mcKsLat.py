import numpy as np
import matplotlib.pyplot as plt
from tqdm import tqdm

# 【新增】忽略仿真探索过程中预期的溢出警告，保持控制台整洁
np.seterr(over='ignore', invalid='ignore', under='ignore')

# ==========================================
# 1. 物理参数与基准 LQR
# ==========================================
mass_at_end = 0.24
m_L = 0.15 + mass_at_end
m_p = 4.1    
m_w = 3.5    
h = 0.115
R = 0.2527
g = 9.81

I_b = 0.012904182130007
I_w = 0.0459142776779452
I_rod = (1/12) * 0.3 * (0.314**2) + 2 * mass_at_end * (0.157**2)

K_linear = np.array([-781.4476, -146.7735, 229.6972, 41.4447])

theta0_deg = 0.20
x0 = np.array([theta0_deg * np.pi / 180, 0.0, 0.0, 0.0])

dt = 0.01 
T_total = 6.0
num_steps = int(T_total / dt)

# ==========================================
# 2. RK4 积分器
# ==========================================
def rk4_step(dynamics_func, t, x, dt_step):
    k1 = np.array(dynamics_func(t, x))
    k2 = np.array(dynamics_func(t + 0.5 * dt_step, x + 0.5 * dt_step * k1))
    k3 = np.array(dynamics_func(t + 0.5 * dt_step, x + 0.5 * dt_step * k2))
    k4 = np.array(dynamics_func(t + dt_step, x + dt_step * k3))
    return x + (dt_step / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)

# ==========================================
# 3. 蒙特卡洛遍历 (分级区间)
# ==========================================
N_samples = 1000
FORCE_LIMIT = 27.6 

print(f"Starting exploration of {N_samples} gain combinations...")

valid_n1, valid_n2, valid_n3, valid_n4 = [], [], [], []
results_max_theta, results_max_r, results_max_force = [], [], []

for k in tqdm(range(N_samples), desc="Simulating Gains", unit="iter"):
    # 【核心修正】根据状态变量的数量级差异，分配不同的搜索区间
    n1 = np.random.uniform(-3000, 3000) # theta^3 
    n2 = np.random.uniform(-200, 200)     # dtheta^3 
    n3 = np.random.uniform(-10, 30) # r^3 
    n4 = np.random.uniform(-0.25, 1.5)   # dr^3 
    n_params = [n1, n2, n3, n4]
    
    def dynamics(t, x):
        theta, dtheta, r, dr = x
        
        # 【新增安全锁】如果积分器内部已经跑到离谱的值，直接切断力矩防止 NaN 扩散
        if abs(theta) > 1.5 or abs(r) > 10.0 or abs(dtheta) > 50.0:
            return [0.0, 0.0, 0.0, 0.0]
            
        F_linear = -(K_linear[0]*theta + K_linear[1]*dtheta + K_linear[2]*r + K_linear[3]*dr)
        F_nonlinear = n_params[0]*(theta**3) + n_params[1]*(dtheta**3) + n_params[2]*(r**3) + n_params[3]*(dr**3)
        F_control = F_linear + F_nonlinear
        
        M11 = 2*m_L*(R**2 + r**2) + m_w*R**2 + m_p*(R + h)**2 + (I_w + I_b + I_rod)
        M12 = -2*m_L*R
        M21 = -2*m_L*R
        M22 = 2*m_L
        M = np.array([[M11, M12], [M21, M22]])
        
        RHS1 = -4*m_L*r*dr*dtheta + (2*m_L*R + m_w*R + m_p*(R + h))*g*np.sin(theta) - 2*m_L*g*r*np.cos(theta)
        RHS2 = F_control + 2*m_L*r*(dtheta**2) - 2*m_L*g*np.sin(theta)
        RHS = np.array([RHS1, RHS2])
        
        try:
            accel = np.linalg.solve(M, RHS)
        except np.linalg.LinAlgError:
            return [0.0, 0.0, 0.0, 0.0]
        
        return [dtheta, accel[0], dr, accel[1]]

    state_history = np.zeros((4, num_steps))
    state_history[:, 0] = x0
    x_current = x0.copy()
    t_current = 0.0
    
    success_flag = True
    
    for i in range(1, num_steps):
        x_current = rk4_step(dynamics, t_current, x_current, dt)
        t_current += dt
        
        if np.any(np.isnan(x_current)) or abs(x_current[0]) > 2.0 or abs(x_current[2]) > 5.0:
            success_flag = False
            break
            
        state_history[:, i] = x_current

    if success_flag:
        final_theta = abs(state_history[0, -1]) * 180 / np.pi
        
        if final_theta < 0.5:
            theta_arr = state_history[0, :]
            dtheta_arr = state_history[1, :]
            r_arr = state_history[2, :]
            dr_arr = state_history[3, :]
            
            F_lin_arr = -(K_linear[0]*theta_arr + K_linear[1]*dtheta_arr + K_linear[2]*r_arr + K_linear[3]*dr_arr)
            F_nonlin_arr = n_params[0]*(theta_arr**3) + n_params[1]*(dtheta_arr**3) + n_params[2]*(r_arr**3) + n_params[3]*(dr_arr**3)
            F_total_arr = F_lin_arr + F_nonlin_arr
            
            max_force = np.max(np.abs(F_total_arr))
            
            if max_force < FORCE_LIMIT:
                valid_n1.append(n_params[0])
                valid_n2.append(n_params[1])
                valid_n3.append(n_params[2])
                valid_n4.append(n_params[3])
                
                results_max_theta.append(np.max(np.abs(theta_arr)) * 180 / np.pi)
                results_max_r.append(np.max(np.abs(r_arr)))
                results_max_force.append(max_force)

print(f"\nExploration complete. Found {len(valid_n1)} valid nonlinear gain combinations under large perturbations satisfying the 27.6N constraint.")

# ==========================================
# 4. 可视化分析
# ==========================================
if len(valid_n1) > 0:
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(15, 6))
    
    sc1 = ax1.scatter(valid_n1, valid_n2, c=results_max_r, cmap='viridis', alpha=0.8, edgecolors='w', s=50)
    ax1.set_title(r'Nonlinear Gains: $n_1$ vs $n_2$ (Color = Max Cart Pos)')
    ax1.set_xlabel(r'$n_1$ (for $\theta^3$)')
    ax1.set_ylabel(r'$n_2$ (for $\dot{\theta}^3$)')
    ax1.axhline(0, color='k', linestyle='--', alpha=0.3)
    ax1.axvline(0, color='k', linestyle='--', alpha=0.3)
    ax1.grid(True, linestyle=':', alpha=0.6)
    fig.colorbar(sc1, ax=ax1, label='Max Cart Position $r$ (m)')
    
    sc2 = ax2.scatter(valid_n3, valid_n4, c=results_max_force, cmap='plasma', alpha=0.8, edgecolors='w', s=50)
    ax2.set_title(r'Nonlinear Gains: $n_3$ vs $n_4$ (Color = Max Force)')
    ax2.set_xlabel(r'$n_3$ (for $r^3$)')
    ax2.set_ylabel(r'$n_4$ (for $\dot{r}^3$)')
    ax2.axhline(0, color='k', linestyle='--', alpha=0.3)
    ax2.axvline(0, color='k', linestyle='--', alpha=0.3)
    ax2.grid(True, linestyle=':', alpha=0.6)
    fig.colorbar(sc2, ax=ax2, label='Max Control Force (N)')
    
    plt.tight_layout()
    plt.show()
else:
    print("未能找到有效增益。请尝试缩小阻尼增益 (n2, n4) 的范围。")