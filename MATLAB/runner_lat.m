clc;
clear;
close all;
addpath("model/","controller/","plotter/","simulation/","param/")

%% Choose simulation setting
paramset = LatParam();

controller = @lat_controller;
model = @LatModel; 
simulator = @ode45Simu;
plotter = @plot_lat;
% dataset format [X, X_dot, gamma, gamma_dot] for segway model
%% Simulation runs here
[t,Z] = simulator(paramset, model, controller);

%% Plot
plotter(t,Z,paramset);
