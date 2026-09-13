clear; clc; close all;
addpath('param/', 'model/');
par = LatParam();

%% 1. Symbolic Linearization of Appell
disp('Step 1: Linearizing model...');
syms theta theta_dot r r_dot F_sym real
syms m_w m_rod m_b g R h real
z_sym = [theta; theta_dot; r; r_dot];
u1 = theta_dot;
u2 = r_dot - R*theta_dot;

M_matrix = [
    m_w * R^2 + m_rod * r^2 + m_b * (R+h)^2 + par.I_w + par.I_rod + par.I_b, 0;
    0,                       m_rod
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

%% 2. Monte Carlo Setup (LQR - 4D Random Qs)
N = 1500; % Kept at 2000 for reasonable computation time with ode45
disp(['Step 2: Starting LQR Monte Carlo (Q-Space) with N = ', num2str(N)]);

% Randomize Q in logarithmic scale (10^0 to 10^5)
q1_rand = 10.^(8 * rand(N, 1)); % theta
q2_rand = 10.^(8 * rand(N, 1)); % theta_dot
q3_rand = 10.^(8 * rand(N, 1)); % r
q4_rand = 10.^(8 * rand(N, 1)); % r_dot
R_fixed = 1;

% Variables to store results
nonlinear_stable_idx = false(N, 1);
hardware_valid_idx   = false(N, 1); 

z0 = [1 * pi/180; 0; 0; 0]; % Initial conditon theta, theta dot, r, r dot
tspan = [0, 15]; 

% Configuration to abort the simulation if the robot falls
options = odeset('Events', @fallDetector);

% 3. Nonlinear Physics and Hardware Verification
for i = 1:N
    Q = diag([q1_rand(i), q2_rand(i), q3_rand(i), q4_rand(i)]);
    K = lqr(A_num, B_num, Q, R_fixed);
    
    controller = @(t, z, par) -K * z;
    
    try
        % Add 'options' to the ode45 call
        [~, z_out] = ode45(@(t, z) LatModelAppell(t, z, par, controller), tspan, z0, options);
        
        max_theta = max(abs(z_out(:, 1)));  
        final_theta = abs(z_out(end, 1));
        
        % Extract max control force used
        F_out = -K * z_out'; 
        max_force = max(abs(F_out));
        
        % Criterion 1: Nonlinear Stability
        if (max_theta < 20 * pi/180) && (final_theta < 5 * pi/180) 
            nonlinear_stable_idx(i) = true;

            %disp(K);
            % Criterion 2: Hardware Feasible (Peak limit 27.6 N)
            if max_force <= 27.6
                hardware_valid_idx(i) = true;
            end
        end
    catch
        % System went unstable
    end
    
    if mod(i, 200) == 0
        fprintf('LQR Monte Carlo Progress: %d / %d\n', i, N);
    end
end

%% 4. Data Extraction and Plotting
disp(['Total mathematically stable Q matrices generated: ', num2str(N)]);
disp(['Passed nonlinear physics: ', num2str(sum(nonlinear_stable_idx))]);
disp(['Respected 27.6N hardware limit: ', num2str(sum(hardware_valid_idx))]);

figure('Name', 'LQR Design Space (Q Penalties)', 'Position', [100, 100, 1200, 500]);

% --- Subplot 1: Q1 (Theta) vs Q2 (Theta_dot) ---
subplot(1, 2, 1); hold on;
% Plot all generated points (Gray)
scatter(q1_rand, q2_rand, 6, [0.7 0.7 0.7], 'filled', 'MarkerFaceAlpha', 0.2, 'DisplayName', 'Linear Stable (All)');
% Plot non-linearly stable points (Blue)
scatter(q1_rand(nonlinear_stable_idx), q2_rand(nonlinear_stable_idx), 15, 'b', 'filled', 'MarkerFaceAlpha', 0.6, 'DisplayName', 'Nonlinear Stable');
% Plot hardware valid points (Green)
if any(hardware_valid_idx)
    scatter(q1_rand(hardware_valid_idx), q2_rand(hardware_valid_idx), 35, 'g', 'filled', 'DisplayName', 'Hardware Valid (< 27.6N)');
end
set(gca, 'XScale', 'log', 'YScale', 'log'); % Log scale is crucial for Q values
xlabel('Q_1 (\theta Penalty)'); ylabel('Q_2 (\dot{\theta} Penalty)');
title('LQR Design Space: Angle vs Angular Velocity');
legend('Location', 'best'); grid on; hold off;

% --- Subplot 2: Q3 (Position) vs Q4 (Linear Velocity) ---
subplot(1, 2, 2); hold on;
scatter(q3_rand, q4_rand, 6, [0.7 0.7 0.7], 'filled', 'MarkerFaceAlpha', 0.2, 'DisplayName', 'Linear Stable (All)');
scatter(q3_rand(nonlinear_stable_idx), q4_rand(nonlinear_stable_idx), 15, 'b', 'filled', 'MarkerFaceAlpha', 0.6, 'DisplayName', 'Nonlinear Stable');
if any(hardware_valid_idx)
    scatter(q3_rand(hardware_valid_idx), q4_rand(hardware_valid_idx), 35, 'g', 'filled', 'DisplayName', 'Hardware Valid (< 27.6N)');
end
set(gca, 'XScale', 'log', 'YScale', 'log');
xlabel('Q_3 (r Penalty)'); ylabel('Q_4 (\dot{r} Penalty)');
title('LQR Design Space: Position vs Linear Velocity');
legend('Location', 'best'); grid on; hold off;

%% Local Functions (Must be at the very bottom of the script)

function [value, isterminal, direction] = fallDetector(t, z)
    % Detect if the angle theta exceeds 20 degrees (approx 0.35 radians)
    % value = 0 when the angle hits the limit
    limit_rad = 27 * pi/180;
    value = limit_rad - abs(z(1)); 
    
    isterminal = 1; % 1 = Abort the integration immediately
    direction = 0;  % Detect crossing from any direction
end