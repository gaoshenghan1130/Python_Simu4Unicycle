clear; clc; close all;
addpath('param/', 'model/');
par = LatParam();

%% 1. Symbolic Linearization of Lagrangian Model
syms theta theta_dot r r_dot F_sym real
syms m_W m_L m_B g R h I_w I_rod I_b real

% Define the state vector order with positions and velocities
z_sym = [theta; theta_dot; r; r_dot]; 

% Matrix derived from the Lagrangian model
M_matrix = [
    2*m_L*(R^2 + r^2) + m_W*R^2 + m_B*(R+h)^2 + I_b + I_w + I_rod,  -2*m_L*R;
    -2*m_L*R,                                                       2*m_L
];

M_rightside = [
    -4*m_L*r*r_dot*theta_dot + (2*m_L*R + m_W*R + m_B*(R+h))*g*sin(theta) - 2*m_L*g*r*cos(theta);
    F_sym + 2*m_L*r*theta_dot^2 - 2*m_L*g*sin(theta)
];

% Solve for accelerations
accel_sym = simplify(M_matrix \ M_rightside);

% Form the derivative of the state vector (dz) according to z_sym order
dz_sym = [theta_dot; accel_sym(1); r_dot; accel_sym(2)];

% Jacobian
A_sym = jacobian(dz_sym, z_sym); 
B_sym = jacobian(dz_sym, F_sym);

% Evaluate at the equilibrium point
A_eq = subs(A_sym, [theta theta_dot r r_dot F_sym], [0 0 0 0 0]);
B_eq = subs(B_sym, [theta theta_dot r r_dot F_sym], [0 0 0 0 0]);

% 8. Substitute numerical parameters
A_num = double(subs(A_eq, [m_W m_L m_B g R h I_w I_rod I_b], ...
    [par.m_W par.m_L par.m_B par.g par.R par.h par.I_w par.I_rod par.I_b]));
B_num = double(subs(B_eq, [m_W m_L m_B g R h I_w I_rod I_b], ...
    [par.m_W par.m_L par.m_B par.g par.R par.h par.I_w par.I_rod par.I_b]));

%% 2. Monte Carlo Setup (Pole Placement)
N = 2000;
disp(['Step 2: Starting Pole Placement Monte Carlo with N = ', num2str(N)]);

% Randomize Poles (Always negative for linear stability)
p1_rand = -0.1 - 2 * rand(N, 1);
p2_rand = -0.1 - 2 * rand(N, 1);
p3_rand = -0.1 - 2 * rand(N, 1);
p4_rand = -0.1 - 2 * rand(N, 1);

nonlinear_stable_idx = false(N, 1);
hardware_valid_idx   = false(N, 1); 

z0 = [0.15 * pi/180; 0; 0; 0]; % Initial condition
tspan = [0, 15];

% Configuration to abort the simulation if the robot falls
options = odeset('Events', @fallDetector);

% Nonlinear Physics and Hardware Verification
for i = 1:N
    P = [p1_rand(i), p2_rand(i), p3_rand(i), p4_rand(i)];
        
    try
        K = place(A_num, B_num, P);
        controller = @(t, z, par) -K * z;
        
        % Add 'options' to the ode45 call
        [~, z_out] = ode45(@(t, z) LatModelAppell(t, z, par, controller), tspan, z0, options);
        
        max_theta = max(abs(z_out(:, 1)));  
        final_theta = abs(z_out(end, 1));
        
        F_out = -K * z_out'; 
        max_force = max(abs(F_out));
        

        if (max_theta < 20 * pi/180) && (final_theta < 5 * pi/180) 
            nonlinear_stable_idx(i) = true;
            if max_force <= 27.6
                disp(K);
                disp(P);
                hardware_valid_idx(i) = true;
            end
        end
        
    catch
        % Pole placement failure or unstable integration
    end
    
    if mod(i, 200) == 0
        fprintf('Pole Placement Monte Carlo Progress: %d / %d\n', i, N);
    end
end

%% Data Extraction and Plotting
disp(['Total stable pole sets generated: ', num2str(N)]);
disp(['Passed nonlinear physics: ', num2str(sum(nonlinear_stable_idx))]);
disp(['Respected 27.6N hardware limit: ', num2str(sum(hardware_valid_idx))]);

figure('Name', 'Pole Placement Design Space', 'Position', [100, 100, 1200, 500]);

% --- Subplot 1: p1 vs p2 (Pendulum Poles) ---
subplot(1, 2, 1); hold on;
scatter(p1_rand, p2_rand, 6, [0.7 0.7 0.7], 'filled', 'MarkerFaceAlpha', 0.2, 'DisplayName', 'Linear Stable (All)');
scatter(p1_rand(nonlinear_stable_idx), p2_rand(nonlinear_stable_idx), 15, 'b', 'filled', 'MarkerFaceAlpha', 0.6, 'DisplayName', 'Nonlinear Stable');
if any(hardware_valid_idx)
    scatter(p1_rand(hardware_valid_idx), p2_rand(hardware_valid_idx), 35, 'g', 'filled', 'DisplayName', 'Hardware Valid (< 27.6N)');
end
set(gca, 'XScale', 'linear', 'YScale', 'linear');
xlabel('Pole 1 (Real)'); ylabel('Pole 2 (Real)');
title('Design Space: Pendulum Poles (p_1 vs p_2)');
legend('Location', 'best'); grid on; hold off;

% --- Subplot 2: p3 vs p4 (Cart Poles) ---
subplot(1, 2, 2); hold on;
scatter(p3_rand, p4_rand, 6, [0.7 0.7 0.7], 'filled', 'MarkerFaceAlpha', 0.2, 'DisplayName', 'Linear Stable (All)');
scatter(p3_rand(nonlinear_stable_idx), p4_rand(nonlinear_stable_idx), 15, 'b', 'filled', 'MarkerFaceAlpha', 0.6, 'DisplayName', 'Nonlinear Stable');
if any(hardware_valid_idx)
    scatter(p3_rand(hardware_valid_idx), p4_rand(hardware_valid_idx), 35, 'g', 'filled', 'DisplayName', 'Hardware Valid (< 27.6N)');
end
set(gca, 'XScale', 'linear', 'YScale', 'linear');
xlabel('Pole 3 (Real)'); ylabel('Pole 4 (Real)');
title('Design Space: Cart Poles (p_3 vs p_4)');
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