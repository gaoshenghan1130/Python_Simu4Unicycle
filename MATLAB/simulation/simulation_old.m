clc
clear
close all

%% Parameters
% Values mirror Derivation/Segway/Parameters.py plus the tuning overrides in
% Segway_rollingResistance.ipynb.
scenario = 'velocity'; % 'velocity' or 'position'

par.m = 0.7;          % pendulum mass (kg)
par.m_w = 3.06;       % wheel mass (kg)
par.h = 0.115;        % pendulum COM height from wheel COM (m)
par.R = 0.2527;       % wheel radius (m)
par.I = 0.8;          % wheel inertia override used by the notebook (kg*m^2)
par.g = 9.81;         % gravity (m/s^2)

% Controller gains from Derivation/Segway/Parameters.py.
par.K_gamma = 3.0;
par.K_dgamma = 0.8;
par.K_velocity = 4;
par.K_position = 2.0;
par.K_vi = 0.0;
par.K_pi = 0.00;
par.posDeadZone = 0.005;
par.clip_integral = 1.0;

switch lower(scenario)
    case 'velocity'
        % Match Segway_rollingResistance.ipynb cell 6.
        par.B = 0.002213639008177662;
        par.B_0 = 0.001748261186143499;
        par.mu_rolling = 0.0015478352590933;
        par.smooth_factor = 100;
        par.control_mode = 'velocity';
        par.control_strategy = 'pd';
        par.desired_gamma = 0.0;
        par.desired_velocity = 0.5;
        par.desired_position = 0.0;
    case 'position'
        % Match Segway_rollingResistance.ipynb cell 8.
        par.B = 0.002213639008177662;
        par.B_0 = 0.001748261186143499;
        par.mu_rolling = 0.0015478352590933;
        par.smooth_factor = 100;
        par.control_mode = 'position';
        par.control_strategy = 'pd';
        par.desired_gamma = 0.0;
        par.desired_velocity = 0.0;
        par.desired_position = 1.0;
    otherwise
        error('simulation:BadScenario', ...
            'scenario must be velocity or position.');
end

%% Initial conditions
% z = [x; x_dot; gamma; gamma_dot]
z0 = [0.0; 0.0; 0.0; 0.0];

%% Simulation
Tend = 15;
t_eval = linspace(0, Tend, 1000);
options = odeset('RelTol', 1e-6, 'AbsTol', 1e-8, 'MaxStep', 0.01);

[t, Z] = ode45(@(t, z) segway_rolling_resistance_model( ...
    t, z, par, @segway_controller), t_eval, z0, options);

X = Z(:, 1);
X_dot = Z(:, 2);
gamma = Z(:, 3);
gamma_dot = Z(:, 4);

%% Plotting
figure

subplot(4, 1, 1)
box on
hold on
grid on
xlabel('$t \, (\mathrm{s})$', 'Interpreter', 'latex')
ylabel('$x \, (\mathrm{m})$', 'Interpreter', 'latex')
plot(t, X, 'LineWidth', 1.5)
if strcmpi(par.control_mode, 'position')
    plot(t, par.desired_position*ones(size(t)), ':', 'LineWidth', 1.2)
end
if strcmpi(par.control_mode, 'position')
    legend('$x$', '$x_d$', 'Interpreter', 'latex', 'Location', 'best')
else
    legend('$x$', 'Interpreter', 'latex', 'Location', 'best')
end
title('Longitudinal Segway Rolling Resistance Model')

subplot(4, 1, 2)
box on
hold on
grid on
xlabel('$t \, (\mathrm{s})$', 'Interpreter', 'latex')
ylabel('$\gamma \, (\mathrm{deg})$', 'Interpreter', 'latex')
plot(t, gamma*180/pi, 'LineWidth', 1.5)
plot(t, par.desired_gamma*180/pi*ones(size(t)), ':', 'LineWidth', 1.2)
legend('$\gamma$', '$\gamma_d$', 'Interpreter', 'latex', 'Location', 'best')

subplot(4, 1, 3)
box on
hold on
grid on
xlabel('$t \, (\mathrm{s})$', 'Interpreter', 'latex')
ylabel('$\dot{x} \, (\mathrm{m/s})$', 'Interpreter', 'latex')
plot(t, X_dot, 'LineWidth', 1.5)
plot(t, par.desired_velocity*ones(size(t)), ':', 'LineWidth', 1.2)
legend('$\dot{x}$', '$\dot{x}_d$', ...
    'Interpreter', 'latex', 'Location', 'best')

subplot(4, 1, 4)
box on
hold on
grid on
xlabel('$t \, (\mathrm{s})$', 'Interpreter', 'latex')
ylabel('$\dot{\gamma} \, (\mathrm{rad/s})$', 'Interpreter', 'latex')
plot(t, gamma_dot, 'LineWidth', 1.5)
legend('$\dot{\gamma}$', ...
    'Interpreter', 'latex', 'Location', 'best')
