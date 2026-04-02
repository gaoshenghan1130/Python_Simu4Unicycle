from scipy.signal import savgol_filter
import numpy as np
from scipy.signal import butter, filtfilt
from scipy.interpolate import UnivariateSpline

def splineFitFilter(data, s=0.05, columnIndex=3):
    """
    s (smoothing factor): 关键参数！
    s 越大，曲线越平滑（越不看重原始坑洼）；
    s 越小，曲线越贴合原始阶梯。
    """
    ndata = np.array(data, dtype=float)
    x = ndata[:, 0]  # 时间轴
    y = ndata[:, columnIndex] # 目标数据
    
    # 样条拟合：s 是平滑因子。针对你图中的量级（0.2-0.8），s 设为 0.05-0.2 比较合适
    spl = UnivariateSpline(x, y, s=s)
    
    y_smooth = spl(x)
    
    ndata[:, columnIndex] = y_smooth
    return ndata.tolist()

def polyFitFilter(data, degree=12, columnIndex=3):
    ndata = np.array(data, dtype=float)
    y = ndata[:, columnIndex]
    x = ndata[:, 0]  # the first column is time, which we can use as x-axis for fitting
    
    coeffs = np.polyfit(x, y, degree)
    
    y_smooth = np.polyval(coeffs, x)
    
    ndata[:, columnIndex] = y_smooth
    return ndata.tolist()

def butterLowpassFilter(data, cutoff=2.0, fs=100, order=2, columnIndex=3):
    ndata = np.array(data, dtype=float)
    x = ndata[:, columnIndex]
    
    nyquist = 0.5 * fs
    normal_cutoff = cutoff / nyquist
    b, a = butter(order, normal_cutoff, btype='low', analog=False) # type: ignore
    
    x_smooth = filtfilt(b, a, x)
    
    ndata[:, columnIndex] = x_smooth
    return ndata.tolist()

def sgFilter(data, window_size=20, columnIndex = 3):
    if data is None or len(data) < window_size:
        return data
    
    ndata = np.array(data, dtype=float)
    
    x_c = ndata[:, columnIndex] 
    x_c_smooth = savgol_filter(x_c, window_size, polyorder=1)
    

    ndata[:, columnIndex] = x_c_smooth
    
    return ndata.tolist()

def getvel(data):
    if data is None or len(data) < 2:
        return data
    
    res = [list(row) for row in data]
    
    for i in range(len(res)):
        while len(res[i]) < 5:
            res[i].append(0.0)
            
        if i == 0:
            res[i][4] = 0.0  
            continue
            t
        dt = res[i][0] - res[i-1][0]
        dx = res[i][3] - res[i-1][3]
        
        if dt != 0:
            res[i][4] = dx / dt
        else:
            res[i][4] = 0.0
            
    return res

def setInit2Zero(data):
    if data is None or len(data) == 0:
        return data
    
    ndata = np.array(data, dtype=float)
    init_row = ndata[0, :]
    
    res = ndata - init_row
    
    return res.tolist()