clc;
clear;
close all;
addpath("model/","controller/","plotter/","simulation/","param/")

%% Choose simulation setting
paramset = LatParam();

controller = @lat_controller;
model = @LatModel; 
simulator = @timeConstantSimu;
% dataset format [X, X_dot, gamma, gamma_dot] for segway model
%% Simulation runs here
[t,Z] = simulator(paramset, model, controller);

F_history = zeros(length(t), 1);
for i = 1:length(t)
    z_current = Z(i, :)'; 
    F_history(i) = controller(t(i), z_current, paramset); 
end

disp(max(F_history));

%% Plot
plot_lat(t, Z, F_history, paramset);
