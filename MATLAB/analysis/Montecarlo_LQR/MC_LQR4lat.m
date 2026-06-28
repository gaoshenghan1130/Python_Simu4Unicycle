clear; clc;

addpath("param/", "model/");
par = LatParam();
syms theta theta_dot r r_dot F_sym real
syms m_L m_B m_W h g R real
syms Friction I_b I_w I_rod real
z_sym = [theta; theta_dot; r; r_dot];

M_matrix = [2*m_L *  R^2 + m_B*(R+h)^2 + m_W*R^2 + 2*m_L*r^2 + par.I_b + par.I_w + par.I_rod,   -2*m_L*R;
            -2*m_L*R,                               2*m_L];
M_rightside = [
    2*m_L*g* R*sin(theta) - 2*m_L*g*r*cos(theta) + m_B* g*(R+h)*sin(theta) + m_W*g*R*sin(theta)  - 4  * m_L*r*r_dot* theta_dot;
    - 2*m_L*g*sin(theta) + F_sym + 2*m_L * r * theta_dot ^2 
];
accel = M_matrix \ M_rightside;
dz_sym = [
    theta_dot;
    accel(1);
    r_dot;
    accel(2)
];
A_sym = jacobian(dz_sym, z_sym);
B_sym = jacobian(dz_sym, F_sym);

A_eq = subs(A_sym, [z_sym; F_sym], [0; 0; 0; 0; 0]);
B_eq = subs(B_sym, [z_sym; F_sym], [0; 0; 0; 0; 0]);

A_num = double(subs(A_eq, [m_L, m_B, m_W, h, g, R, I_b, I_w, I_rod], [par.m_L, par.m_B, par.m_W, par.h, par.g, par.R, par.I_b, par.I_w, par.I_rod]));
B_num = double(subs(B_eq, [m_L, m_B, m_W, h, g, R, I_b, I_w, I_rod], [par.m_L, par.m_B, par.m_W, par.h, par.g, par.R, par.I_b, par.I_w, par.I_rod]));

%% 2. Monte Carlo Simulation Setup
% Define test range for Q/R
% q_theta_vals: Test from 1 to 1,000,000
q_theta_vals = logspace(0, 6, 15); 
% R_weights: Test from 1 to 100
R_weights = logspace(0, 2, 15);    

% Initialization
Max_Force_Matrix = zeros(length(q_theta_vals), length(R_weights));
z0 = [3*pi/180; 0; 0; 0];     % Initial State (3 degrees)
tspan = [0 10];               % Simulation Time

% 3. Monte Carlo iteration
for i = 1:length(q_theta_vals)
    for j = 1:length(R_weights)
        
        % Defining Q matrix (Keeping theta_dot, r, and r_dot tolerant)
        Q_iter = diag([q_theta_vals(i), 10, 10, 1]); 
        R_iter = R_weights(j);
        
        % LQR gains
        K_iter = lqr(A_num, B_num, Q_iter, R_iter);
        lqr_controller = @(t, z, par) -K_iter * z; 
        
        % Simulating model with specific gains
        [t_out, z_out] = ode45(@(t, z) LatModel_SignCorrection(t, z, par, lqr_controller), tspan, z0);
        
        % Force used in every iteration
        F_current_run = zeros(length(t_out), 1);
        for k = 1:length(t_out)
            F_current_run(k) = -K_iter * z_out(k,:)';  
        end
        
        % Save the max force applied during the run
        Max_Force_Matrix(i, j) = max(abs(F_current_run));
    end
end

%% 4. Plot (Hardware Limits)
figure('Name', 'Montecarlo LQR');
hold on;

% Select 3 values from the R matrix (start, middle, and end of the array)
idx_1 = 1;                              % Lowest R (No actuator limits)
idx_2 = round(length(R_weights)/2);     % Intermediate R
idx_3 = length(R_weights);              % Highest R (No actuator limited)

% Plot the lines for R values
plot(q_theta_vals, Max_Force_Matrix(:, idx_1), 'b-', 'LineWidth', 2);
plot(q_theta_vals, Max_Force_Matrix(:, idx_2), 'g--', 'LineWidth', 2);
plot(q_theta_vals, Max_Force_Matrix(:, idx_3), 'm:', 'LineWidth', 2);

% MOTOR PHYSICAL LIMITS
continuous_limit = 9.2;
peak_limit = 27.6;

% Draw the hardware limit lines
y1 = yline(continuous_limit, 'k--', 'LineWidth', 2);
y2 = yline(peak_limit, 'r-', 'LineWidth', 2);

set(gca, 'XScale', 'log'); % Logarithmic X-axis because Q_theta grows exponentially
xlabel('Angle Penalty (Q_{\theta})');
ylabel('Maximum Motor Force (N)');
title('LQR Force Demand vs Physical Hardware Limits');
legend(['R = ', num2str(R_weights(idx_1))], ...
       ['R = ', num2str(R_weights(idx_2))], ...
       ['R = ', num2str(R_weights(idx_3))], ...
       'Thermal Limit (9.2 N)', ...
       'Saturation Limit (27.6 N)', ...
       'Location', 'northwest');
grid on;