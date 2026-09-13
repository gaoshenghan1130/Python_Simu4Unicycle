clear; clc;
addpath("param/", "model/");

%% 1. Load Parameters
par = LatParam();

%% 2. System Modeling & Linearization
syms theta theta_dot r r_dot F_sym real
syms m_L m_B m_W h g R real
syms Friction I_b I_w I_rod real

z_sym = [theta; theta_dot; r; r_dot];

% Mass matrix and right-hand side dynamics
M_matrix = [2*m_L + R^2 + m_B*(R+h)^2 + m_W*R^2 + I_b + I_w + I_rod,   2*m_L*R;
            2*m_L*R,                               2*m_L];
            
M_rightside = [
    2*m_L*R*sin(theta) + 2*m_L*g*r*cos(theta) + m_B*(R+h)*sin(theta) + m_W*g*R*sin(theta) - F_sym*R;
    2*m_L*g*sin(theta) + F_sym
];

% Solve for accelerations
accel = M_matrix \ M_rightside;
dz_sym = [
    theta_dot;
    accel(1);
    r_dot;
    accel(2)
];

% Calculate Jacobians for A and B matrices
A_sym = jacobian(dz_sym, z_sym);
B_sym = jacobian(dz_sym, F_sym);

% Evaluate at equilibrium point (all zeros)
A_eq = subs(A_sym, [z_sym; F_sym], [0; 0; 0; 0; 0]);
B_eq = subs(B_sym, [z_sym; F_sym], [0; 0; 0; 0; 0]);

% Substitute physical parameters
A_num = double(subs(A_eq, [m_L, m_B, m_W, h, g, R, I_b, I_w, I_rod], ...
    [par.m_L, par.m_B, par.m_W, par.h, par.g, par.R, par.I_b, par.I_w, par.I_rod]));
B_num = double(subs(B_eq, [m_L, m_B, m_W, h, g, R, I_b, I_w, I_rod], ...
    [par.m_L, par.m_B, par.m_W, par.h, par.g, par.R, par.I_b, par.I_w, par.I_rod]));

%% 3. Design LQR
Q = diag([1000, 100, 10, 10]); 
R_weight = 10; 
K = lqr(A_num, B_num, Q, R_weight);

disp('LQR Gain K:');
disp(K);

%% 4. Simulate with Delay (dde23)
z0 = [5 * pi/180; 0; 0; 0];
tspan = [0, 5];
delay_sec = 0.02;  % 50ms latency from IMU

% Z(:,1) represents the state vector exactly at (t - delay_sec)
delayed_controller = @(Z) -K * Z(:,1);

% Wrap the dynamics for dde23 to handle the delayed state Z
ddefun = @(t, z, Z) LatModel_Wrapper(t, z, Z, par, delayed_controller);

% History defines the state of the system before t = 0
history = z0; 

% Solve Delay Differential Equation
sol = dde23(ddefun, delay_sec, history, tspan);

% Extract outputs
t_out = sol.x';
z_out = sol.y';

% Recalculate Force F_out using delayed states for plotting
F_out = zeros(length(t_out), 1);
for i = 1:length(t_out)
    if t_out(i) <= delay_sec
        F_out(i) = -K * z0; % Uses history during the initial delay period
    else
        % Use deval to interpolate the state exactly at t - delay
        z_delayed = deval(sol, t_out(i) - delay_sec);
        F_out(i) = -K * z_delayed;
    end
end

%% 5. Plot Results
figure;
subplot(3,1,1);
plot(t_out, z_out(:,1), 'LineWidth', 1.5);
ylabel('Theta (rad)');
title(sprintf('LQR Controlled System with IMU Latency (Delay = %.3fs)', delay_sec));
grid on;

subplot(3,1,2);
plot(t_out, z_out(:,3), 'LineWidth', 1.5);
ylabel('Position r (m)');
grid on;

subplot(3,1,3);
plot(t_out, F_out, 'r', 'LineWidth', 1.5); 
xlabel('Time (s)');
ylabel('Force F (N)');
grid on;

%% --- Local Functions ---
% Note: Local functions must remain at the very bottom of the script file.

function dz = LatModel_Wrapper(t, z, Z, par, delayed_controller)
    % 1. Calculate the control effort based purely on the delayed state Z
    F_delayed = delayed_controller(Z);
    
    % 2. Create a dummy controller handle to feed back into your original model.
    % This overrides the normal calculation and forces it to use the delayed force.
    dummy_controller = @(t_dummy, z_dummy, par_dummy) F_delayed;
    
    % 3. Evaluate the original dynamics with the delayed control input
    dz = LatModel(t, z, par, dummy_controller);
end