clear; clc; close all;
addpath("param/", "model/");
par = LatParam();

%% Appell Model Linearization

syms theta theta_dot r r_dot F_sym real
syms m_w m_rod m_b g R h real
z_sym = [theta; theta_dot; r; r_dot];

% Appell quasi velocities
u1 = theta_dot;
u2 = r_dot - R*theta_dot;

% Appell dynamics
M_matrix = [
    m_w*R^2 + m_rod*r^2 + m_b * (R+h)^2, 0;
    0,                   m_rod
];
M_rightside = [
    F_sym*R - m_rod*g*r*cos(theta) + m_w*g*R*sin(theta) + m_b*g*(R+h)*sin(theta) - m_rod*r*(2*u2*u1 + R*u1^2);
    F_sym - m_rod*g*sin(theta) + m_rod*r*u1^2
];

accel_u = simplify(M_matrix \ M_rightside);
u1_dot = accel_u(1);
u2_dot = accel_u(2);

theta_ddot = u1_dot;
r_ddot     = u2_dot + R*u1_dot;
dz_sym = [theta_dot; theta_ddot; r_dot; r_ddot];

A_sym = jacobian(dz_sym, z_sym);
B_sym = jacobian(dz_sym, F_sym);

A_eq = subs(A_sym, [theta theta_dot r r_dot F_sym], [0 0 0 0 0]);
B_eq = subs(B_sym, [theta theta_dot r r_dot F_sym], [0 0 0 0 0]);

A_num = double(subs(A_eq, [m_w m_rod m_b g R h], [par.m_W 2*par.m_L par.m_B par.g par.R par.h]));
B_num = double(subs(B_eq, [m_w m_rod m_b g R h], [par.m_W 2*par.m_L par.m_B par.g par.R par.h]));

%% LQR Controller Setup
% Defining some values for Q/R
Q = diag([10000, 100, 10, 1]); 
R_weight = 10;
K = lqr(A_num, B_num, Q, R_weight);

% Control law for the nonlinear
lqr_controller = @(t, z, par) -K * z;

% Linear state-space model for ode45: dz = (A - B*K)*z
% A_num and B_num linearized in previous step
linear_closed_loop = @(t, z) (A_num - B_num * K) * z;

%% Trajectory Divergence Analysis (Testing multiple angles)
% We will test 4 different initial conditions to visualize the boundary
test_angles_deg = [0.5, 1, 1.5, 2]; 
tspan = [0 4]; % 4 seconds simulation

figure('Name', 'Nonlinear Boundary Analysis', 'Position', [100, 100, 1200, 700]);

for i = 1:length(test_angles_deg)
    % Initial state: [theta, theta_dot, r, r_dot]
    z0 = [test_angles_deg(i) * pi/180; 0; 0; 0];
    
    % --- Simulate LINEAR Model ---
    [t_lin, z_lin] = ode45(linear_closed_loop, tspan, z0);
    
    % --- Simulate NONLINEAR Model ---
    [t_nonlin, z_nonlin] = ode45(@(t, z) LatModelAppell(t, z, par, lqr_controller), tspan, z0);
    
    % --- Plotting Theta (Angle) ---
    subplot(length(test_angles_deg), 2, 2*i - 1);
    plot(t_lin, z_lin(:,1) * 180/pi, 'b--', 'LineWidth', 2); hold on;
    plot(t_nonlin, z_nonlin(:,1) * 180/pi, 'r-', 'LineWidth', 1.5);
    
    ylabel('\theta (deg)', 'FontWeight', 'bold');
    title(sprintf('Angle Response (Initial Perturbation: %d\\circ)', test_angles_deg(i)));
    grid on;
    if i == 1
        legend('Linearized Math (Ideal)', 'Nonlinear Physics (Real)', 'Location', 'northeast');
    end
    
    % --- Plotting r (Position) ---
    subplot(length(test_angles_deg), 2, 2*i);
    plot(t_lin, z_lin(:,3), 'b--', 'LineWidth', 2); hold on;
    plot(t_nonlin, z_nonlin(:,3), 'r-', 'LineWidth', 1.5);
    
    ylabel('r (m)', 'FontWeight', 'bold');
    title(sprintf('Cart Position (Initial Perturbation: %d\\circ)', test_angles_deg(i)));
    grid on;
    
    % Add X-label only to the bottom plots
    if i == length(test_angles_deg)
        xlabel('Time (s)', 'FontWeight', 'bold');
        subplot(length(test_angles_deg), 2, 2*i - 1);
        xlabel('Time (s)', 'FontWeight', 'bold');
    end
end