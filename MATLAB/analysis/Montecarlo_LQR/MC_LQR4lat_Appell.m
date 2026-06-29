%% 1. Symbolic Linearization of Appell Model
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
    F_sym*R ...
    - m_rod*g*r*cos(theta) ...
    + m_w*g*R*sin(theta) ...
    + m_b*g*(R+h)*sin(theta) ...
    - m_rod*r*(2*u2*u1 + R*u1^2);
    F_sym ...
    - m_rod*g*sin(theta) ...
    + m_rod*r*u1^2
];

accel_u = simplify(M_matrix \ M_rightside);
u1_dot = accel_u(1);
u2_dot = accel_u(2);

theta_ddot = u1_dot;
r_ddot     = u2_dot + R*u1_dot;
dz_sym = [
    theta_dot;
    theta_ddot;
    r_dot;
    r_ddot
];

% Linearization
A_sym = jacobian(dz_sym, z_sym);
B_sym = jacobian(dz_sym, F_sym);

A_eq = subs(A_sym,...
    [theta theta_dot r r_dot F_sym],...
    [0 0 0 0 0]);
B_eq = subs(B_sym,...
    [theta theta_dot r r_dot F_sym],...
    [0 0 0 0 0]);

% Parameter substitution
m_w_num   = par.m_W;
m_rod_num = 2*par.m_L;

A_num = double(subs(A_eq,...
    [m_w m_rod m_b g R h],...
    [m_w_num m_rod_num par.m_B par.g par.R par.h]));
B_num = double(subs(B_eq,...
    [m_w m_rod m_b g R h],...
    [m_w_num m_rod_num par.m_B par.g par.R par.h]));

%% Monte Carlo Simulation Setup
disp('Step 2: Starting Monte Carlo simulation...');

% Define test range for Q_theta
q_theta_vals = logspace(0, 6, 20); %From 1 to 1000000

% Define 3 specific test cases for the angular velocity penalty
% 1 = Relaxed, 100 = Moderate, 10000 = Aggressive
q_theta_dot_vals = [1, 100, 10000]; 

% Initialization
Max_Force_Matrix = zeros(length(q_theta_vals), length(q_theta_dot_vals));
z0 = [1*pi/180; 0; 0; 0];     % Initial State (3 degrees)
tspan = [0 10];               % Simulation Time
R_fixed = 1;                  % R is normalized to 1 to avoid ratio ambiguity

% Monte Carlo iteration (Sweeping Q_theta vs Q_theta_dot)
for i = 1:length(q_theta_vals)
    for j = 1:length(q_theta_dot_vals)
        
        % Q matrix: Iterating Theta and Theta_dot. Position/velocity of cart kept fixed.
        Q_iter = diag([q_theta_vals(i), q_theta_dot_vals(j), 10, 1]); 
        
        % LQR gains with fixed R
        K_iter = lqr(A_num, B_num, Q_iter, R_fixed);
        lqr_controller = @(t, z, par) -K_iter * z; 
        
        % Simulating model
        [t_out, z_out] = ode45(@(t, z) LatModelAppell(t, z, par, lqr_controller), tspan, z0);
        
        % Force reconstruction
        F_current_run = zeros(length(t_out), 1);
        for k = 1:length(t_out)
            F_current_run(k) = -K_iter * z_out(k,:)';  
        end
        
        % Save max force
        Max_Force_Matrix(i, j) = max(abs(F_current_run));
    end
end

disp('Simulation complete. Generating plots...');

%% Plot (Hardware Limits)
figure('Name', 'LQR Cross-Sections - Velocity Coupling');
hold on;

% Plot the lines and save their handles (p1, p2, p3) for the legend
p1 = plot(q_theta_vals, Max_Force_Matrix(:, 1), 'b-', 'LineWidth', 2);
p2 = plot(q_theta_vals, Max_Force_Matrix(:, 2), 'g--', 'LineWidth', 2);
p3 = plot(q_theta_vals, Max_Force_Matrix(:, 3), 'm:', 'LineWidth', 2);

% MOTOR PHYSICAL LIMITS
continuous_limit = 9.2;
peak_limit = 27.6;

% Draw the limit lines
y1 = yline(continuous_limit, 'Color', [0.8500 0.3250 0.0980], 'LineStyle', '--', 'LineWidth', 2);
y2 = yline(peak_limit, 'r-', 'LineWidth', 2);

% Academic formatting
set(gca, 'XScale', 'log'); 
xlabel('Angle Penalty (Q_{\theta})');
ylabel('Maximum Motor Force (N)');
title('Force Demand vs Hardware Limits (R = 1)');

% Build the legend dynamically
legend([p1, p2, p3, y1, y2], ...
       ['Q \theta_{dot} = ', num2str(q_theta_dot_vals(1)), ' (Relaxed)'], ...
       ['Q \theta_{dot} = ', num2str(q_theta_dot_vals(2)), ' (Moderate)'], ...
       ['Q \theta_{dot} = ', num2str(q_theta_dot_vals(3)), ' (Aggressive)'], ...
       'Thermal Limit (9.2 N)', ...
       'Saturation Limit (27.6 N)', ...
       'Location', 'northwest');
grid on;