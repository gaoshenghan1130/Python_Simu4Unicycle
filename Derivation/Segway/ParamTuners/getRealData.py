import re
import csv
import io
from scipy.signal import savgol_filter
import numpy as np
from scipy.signal import butter, filtfilt
from scipy.interpolate import UnivariateSpline

def splineFitFilter(data, s=0.05, columnIndex=3):
    ndata = np.array(data, dtype=float)
    x = ndata[:, 0] 
    y = ndata[:, columnIndex]
    
    spl = UnivariateSpline(x, y, s=s)
    
    y_smooth = spl(x)
    
    ndata[:, columnIndex] = y_smooth
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


def parse_mixed_csv(file_path):
    # list of dicts {"mode": "...", "target_value": 0.0, "data": [...]}
    mode_segments = []
    
    # default balance mode
    current_mode = "balance"
    current_target = 0.0
    current_data = []

    # match with "--- SEND COMMAND: Mode=..., Value=... ---"
    command_pattern = re.compile(r"Mode=(\w+),\s*Value=([\d\.-]+)")

    with open(file_path, 'r', encoding='utf-8') as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            
            if line.startswith("\"--- SEND COMMAND:"):
                if current_data:
                    mode_segments.append({
                        "mode": current_mode,
                        "target_value": current_target,
                        "data": current_data
                    })
                
                match = command_pattern.search(line)
                if match:
                    current_mode = match.group(1)
                    current_target = float(match.group(2))
                

                current_data = []
                
            elif line[0].isdigit() or line[0] == '-':
                try:
                    row = [float(x) for x in line.split(',')]
                    row[3] = row[3] * 0.2527 # convert phi to x_c
                    current_data.append(row)
                except ValueError:
                    continue





        if current_data is not None:
            mode_segments.append({
                "mode": current_mode,
                "target_value": current_target,
                "data": current_data
            })


    #than perform velocity calculation and smoothing for each segment
    for segment in mode_segments:
                # datafiltered = sgFilter(current_data)
        # datawithvel = getvel(datafiltered)
        # zeroinit = current_data# setInit2Zero(datawithvel)

        # for i in range(10,200):
        #     segment['data'] = sgFilter(segment['data'], window_size=i, columnIndex=3)

        #segment['data'] = butterLowpassFilter(segment['data'], cutoff=5, fs=100, order=4, columnIndex=3)
         
        #segment['data'] = sgFilter(segment['data'], window_size=20, columnIndex=3)


        segment['data'] = splineFitFilter(segment['data'], s=0.3, columnIndex=3)


        segment['data'] = getvel(segment['data'])


       # segment['data'] = sgFilter(segment['data'], window_size=20, columnIndex=4)

        #set initial velocity to zero
        segment['data'] = setInit2Zero(segment['data'])

    return mode_segments

def getVelocityFromData(data):
    # data is on the 4th column, time is on the first column
    if data is None or len(data) < 2:
        return data

    res = []
    for item in data:
        row = [item[0], item[4]]  # time and velocity
        res.append(row) 
    return res

def getPositionFromData(data):
    # data is on the 4th column, time is on the first column
    if data is None or len(data) < 2:
        return data

    res = []
    for item in data:
        row = [item[0], item[3]]  # time and position
        res.append(row) 
    return res