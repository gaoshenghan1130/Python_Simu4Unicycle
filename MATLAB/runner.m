addpath("model/","controller/","plotter/","simulation/","param/")
clc;
clear;
close all;

%% Choose simulation setting
paramset = SegwayParamset();
controller = @segway_controller;
model = @segway_rolling_resistance_model; 
simulator = @ode45Simu;
plotter = @plot_segway;
% dataset format [X, X_dot, gamma, gamma_dot] for segway model

%% Set target
paramset.control_mode = 'velocity'; % must be velocity or position
paramset.desired_gamma = 0.0;
paramset.desired_velocity = 1.0;
paramset.desired_position = 0.5;

%% Simulation runs here
[t,Z] = simulator(paramset, model, controller);

%% Plot
plotter(t,Z,paramset);




