import re
import csv
import io
from Phaser.advancedPhaser import getvel, sgFilter, setInit2Zero, butterLowpassFilter, polyFitFilter, splineFitFilter


def phaseMojucoData(file_path):
    phase_data = []
    with open(file_path, 'r', encoding='utf-8') as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            
            if line[0].isdigit() or line[0] == '-':
                try:
                    row = [float(x) for x in line.split(',')]
                    phase_data.append(row)
                except ValueError:
                    continue

    return phase_data

def phaseMathData(file_path):
    phase_data = []
    with open(file_path, 'r', encoding='utf-8') as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            
            if line[0].isdigit() or line[0] == '-':
                try:
                    row = [float(x) for x in line.split(',')]
                    phase_data.append(row)
                except ValueError:
                    continue

    res = []

    for line in phase_data:
        # gamma should be the 3rd column, x_c should be the 2th column
        t = line[0]
        x_c_dot = line[2]
        x_c = line[1]
        gamma = line[3]
        theta = 0

        newline = [t, theta, gamma, x_c, x_c_dot]
        res.append(newline)

    return res

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


        segment['data'] = splineFitFilter(segment['data'], s=0.1, columnIndex=3)


        segment['data'] = getvel(segment['data'])


       # segment['data'] = sgFilter(segment['data'], window_size=20, columnIndex=4)

        #set initial velocity to zero
        segment['data'] = setInit2Zero(segment['data'])

    return mode_segments
