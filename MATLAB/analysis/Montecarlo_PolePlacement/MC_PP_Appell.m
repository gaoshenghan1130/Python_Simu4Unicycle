clear; clc; close all;
addpath("param/", "model/");
par = LatParam();

syms theta theta_dot r r_dot F_sym real
syms m_w m_rod m_b g R h real
z_sym = [theta; theta_dot; r; r_dot];

u1 = theta_dot;
u2 = r_dot - R*theta_dot;

M_matrix = [
    m_w*R^2 + m_rod*r^2 + m_b * (R+h)^2, 0;
    0,                   m_rod
];
M_rightside = [
    F_sym*R - m_rod*g*r*cos(theta) + m_w*g*R*sin(theta) + m_b*g*(R+h)*sin(theta) - m_rod*r*(2*u2*u1 + R*u1^2);
    F_sym - m_rod*g*sin(theta) + m_rod*r*u1^2
];

accel_u = simplify(M_matrix \ M_rightside);
theta_ddot = accel_u(1);
r_ddot     = accel_u(2) + R*accel_u(1);

dz_sym = [theta_dot; theta_ddot; r_dot; r_ddot];

A_sym = jacobian(dz_sym, z_sym);
B_sym = jacobian(dz_sym, F_sym);

A_eq = subs(A_sym, [theta theta_dot r r_dot F_sym], [0 0 0 0 0]);
B_eq = subs(B_sym, [theta theta_dot r r_dot F_sym], [0 0 0 0 0]);

A_num = double(subs(A_eq, [m_w m_rod m_b g R h], [par.m_W 2*par.m_L par.m_B par.g par.R par.h]));
B_num = double(subs(B_eq, [m_w m_rod m_b g R h], [par.m_W 2*par.m_L par.m_B par.g par.R par.h]));

%% Pole Placement Monte Carlo Setup

% Aggressiveness multiplier. The goal is to push poles further into the left half-plane
multipliers = linspace(1, 15, 20); 

% 3 Base Pole Configurations
% Option 1: Balanced (All poles close together)
P_base_1 = [-1.0, -1.1, -1.2, -1.3]; 
% Option 2: Pendulum Priority (Theta poles are fast, Cart poles are slow)
P_base_2 = [-2.0, -2.1, -0.5, -0.6];
% Option 3: Cart Priority (Cart poles are fast, Theta poles are slow)
P_base_3 = [-0.5, -0.6, -2.0, -2.1];

Max_Force_Matrix = zeros(length(multipliers), 3);
z0 = [3*pi/180; 0; 0; 0]; % Initial State (3 degrees)
tspan = [0 5]; 

% Simulation Iteration
for i = 1:length(multipliers)
    % Scale the poles (Moving them left in the complex plane)
    p1 = P_base_1 * multipliers(i);
    p2 = P_base_2 * multipliers(i);
    p3 = P_base_3 * multipliers(i);
    
    % Calculate Gains using Pole Placement
    K1 = place(A_num, B_num, p1);
    K2 = place(A_num, B_num, p2);
    K3 = place(A_num, B_num, p3);
    
    % --- Simulate configuration 1 (Balanced) ---
    c1 = @(t,z,par) -K1*z;
    [~, z1] = ode45(@(t,z) LatModelAppell(t,z,par,c1), tspan, z0);
    F1 = max(abs(-K1 * z1'))'; % Vectorized force calculation (Much faster)
    
    % --- Simulate configuration 2 (Pendulum Priority) ---
    c2 = @(t,z,par) -K2*z;
    [~, z2] = ode45(@(t,z) LatModelAppell(t,z,par,c2), tspan, z0);
    F2 = max(abs(-K2 * z2'))';
    
    % --- Simulate configuration 3 (Cart Priority) ---
    c3 = @(t,z,par) -K3*z;
    [~, z3] = ode45(@(t,z) LatModelAppell(t,z,par,c3), tspan, z0);
    F3 = max(abs(-K3 * z3'))';
    
    % Store max forces for the current multiplier
    Max_Force_Matrix(i, 1) = max(F1);
    Max_Force_Matrix(i, 2) = max(F2);
    Max_Force_Matrix(i, 3) = max(F3);
end

disp('Simulation complete. Generating plots...');

%% Plot (Hardware Limits)
figure('Name', 'Pole Placement Sweep');
hold on;

% Plot the lines and save their handles for the legend
p1 = plot(multipliers, Max_Force_Matrix(:, 1), 'b-', 'LineWidth', 2);
p2 = plot(multipliers, Max_Force_Matrix(:, 2), 'g--', 'LineWidth', 2);
p3 = plot(multipliers, Max_Force_Matrix(:, 3), 'm:', 'LineWidth', 2);

% MOTOR PHYSICAL LIMITS
continuous_limit = 9.2;
peak_limit = 27.6;

% Draw the limit lines without inline text
y1 = yline(continuous_limit, 'Color', [0.8500 0.3250 0.0980], 'LineStyle', '--', 'LineWidth', 2);
y2 = yline(peak_limit, 'r-', 'LineWidth', 2);

% Academic formatting
set(gca, 'XScale', 'linear'); % Using linear scale for pole multipliers
xlabel('Pole Aggressiveness Multiplier');
ylabel('Maximum Motor Force (N)');
title('Pole Placement Force Demand vs Hardware Limits');

% Build the legend dynamically
legend([p1, p2, p3, y1, y2], ...
       'Balanced Poles', ...
       'Pendulum Priority', ...
       'Cart Priority', ...
       'Thermal Limit (9.2 N)', ...
       'Saturation Limit (27.6 N)', ...
       'Location', 'northwest');
grid on;