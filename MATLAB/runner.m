clc;
clear;
close all;
addpath("model/","controller/","plotter/","simulation/","param/")

%% Choose simulation setting
paramset = SegwayParamset();
controller = @segway_controller_pd;
model = @segway_rolling_resistance_model; 
simulator = @timeConstantSimu;
plotter = @plot_segway;
% dataset format [X, X_dot, gamma, gamma_dot] for segway model

%% Set target
paramset.control_mode = 'position'; % must be velocity or position
paramset.desired_gamma = 0.0;
paramset.desired_velocity = 0.0;
paramset.desired_position = 1.0;

%% Simulation runs here
[t,Z] = simulator(paramset, model, controller);

%% Plot
plotter(t,Z,paramset);

targetVal = paramset.desired_velocity;
if strcmp(paramset.control_mode, 'position')
    targetVal = paramset.desired_position;
end

current_path = fileparts(mfilename('fullpath'));


segments = plot_real_data(fullfile(current_path, 'data'), paramset.control_mode, targetVal, 15);


plot_sim_vs_real(t, Z, segments, paramset);
