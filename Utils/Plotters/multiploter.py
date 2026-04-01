import matplotlib
# 使用 Agg 后端直接生成图片，不弹出窗口
matplotlib.use('Agg') 
import matplotlib.pyplot as plt
import os


def plot_comparison(data_dict, output_file='comparison_plot.png', isvelocity=False):
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 10), sharex=True)
    
    for name, data in data_dict.items():
        t0 = data[0][0]
        t = [row[0] - t0 for row in data]
        
        gamma_vals = [row[2] for row in data]
        x_c_vals = [row[3] for row in data]
        dx_c_vals = [row[4] for row in data]
         
        ax1.plot(t, gamma_vals, label=f"{name}", linestyle='-')
        if isvelocity:
            ax2.plot(t, dx_c_vals, label=f"{name}", linestyle='-')
        else:
            ax2.plot(t, x_c_vals, label=f"{name}", linestyle='-')

    ax1.set_title("Gamma_rad Comparison", fontsize=12)
    ax1.set_ylabel("Value (rad)")
    ax1.grid(True, alpha=0.3)
    ax1.legend()

    if not isvelocity:
        ax2.set_title("x_C Comparison", fontsize=12)
        ax2.set_ylabel("Value (m)")
        ax2.set_xlabel("Relative Time (s)")
        ax2.grid(True, alpha=0.3)
        ax2.legend()

    else:
        ax2.set_title("dx_C Comparison", fontsize=12)
        ax2.set_ylabel("Value (m/s)")
        ax2.set_xlabel("Relative Time (s)")
        ax2.grid(True, alpha=0.3)
        ax2.legend()

    plt.tight_layout()
    
    plt.savefig(output_file, dpi=300)
    plt.close()
    print(f"Generated: {os.path.abspath(output_file)}")
