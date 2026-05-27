clc;
clear;
close all;
addpath("controller/","model/","param/","paramTune/","plotter/","simulation/");

initParam = SegwayParamset();
%% Set Mode and Target Value inside paramset to drive the simulator
initParam.control_mode = 'position'; % must be velocity or position
initParam.desired_gamma = 0.0;
initParam.desired_velocity = 0.0;
initParam.desired_position = 1.0;

%% Define simulation function
function res  = sim(par)
    [t, Z]  = timeConstantSimu(par, @segway_rolling_resistance_model, @segway_controller_pd);
    res = [t(:),Z];
end

%% Load the data we want to tune to
current_path = fileparts(mfilename('fullpath'));
folder_path = fullfile(current_path, 'ndata');
files = dir(fullfile(folder_path, '*.csv'));
file_paths = fullfile(folder_path, {files.name});
targetVal4Read = initParam.desired_velocity;
if strcmp('position', initParam.control_mode)
    targetVal4Read = initParam.desired_position;
end
targetSegments = read_real_data(file_paths, initParam.control_mode, targetVal4Read);
targetDataSets = cell(1, numel(targetSegments));
for i = 1 : numel(targetSegments)
    % extract target mode 'position' or velocity
    thisSeg = targetSegments(i);
    if strcmp(thisSeg.mode, initParam.control_mode)
        t = thisSeg.time;
        Z = [thisSeg.position,thisSeg.velocity, thisSeg.gamma_deg.*(pi/180), thisSeg.gamma_deg.*(pi/180)];
        targetDataSets{i} = [t,Z];
    else
        continue
    end
end

fields = {
    'B',
    'B_0',
    'mu_rolling',
};

initParam.B = 0.05;
initParam.B_0 = 0.01402;
initParam.mu_rolling = 0.01;
initParam.m = 0.8;

stepSize = 0.1;

bestPar = tuneParam(initParam, @sim,  @dataSetPenalty_SumSquare_Improved, @randomTuneGenerator, targetDataSets, 0, fields, stepSize);

for i = 1:numel(fields)
    fprintf("%s value: %f\n", ...
        fields{i}, ...
        bestPar.(fields{i}));
end

res = sim(bestPar);
t = res(: , 1)+0.2;
Z = res(:, 2:end);
plot_sim_vs_real(t, Z, targetSegments, bestPar);