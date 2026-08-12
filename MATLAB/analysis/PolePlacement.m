clear;
clc;
close all;

addpath("param/", "model/");

%% 1. System parameters

par = LatParam();

m_L = par.m_L;
m_B = par.m_B;
m_W = par.m_W;
h   = par.h;
R   = par.R;
g   = par.g;

J_theta = 2*m_L*R^2 ...
        + m_B*(R + h)^2 ...
        + m_W*R^2 ...
        + par.I_b ...
        + par.I_rod ...
        + par.I_w;

G_theta1 = 2*m_L*g*R ...
         + m_B*g*(R + h) ...
         + m_W*g*R;

G_theta2 = -2*m_L*g;
G_r1     = -2*m_L*g;

%% 2. Linearized state-space model
%
% Linear model state order:
% x = [theta; r; theta_dot; r_dot]

N = [1, 0,       0,          0;
     0, 1,       0,          0;
     0, 0, J_theta, -2*m_L*R;
     0, 0, -2*m_L*R,  2*m_L];

A_tilde = [0,        0,    1, 0;
           0,        0,    0, 1;
           G_theta1, G_r1, 0, 0;
           G_theta2, 0,    0, 0];

A = N \ A_tilde;
B = N \ [0; 0; 0; 1];

%% 3. Pole placement and closed-loop eigenvectors

P_desired = [-2.25, -1.25, -2.00, -1.50];
K_numeric = place(A, B, P_desired);

disp('Pole-placement gain K in [theta, r, theta_dot, r_dot] order:');
disp(K_numeric);

A_cl_numeric = A - B*K_numeric;
[eigenvectors_x, eigenvalue_matrix] = eig(A_cl_numeric);
eigenvalues = diag(eigenvalue_matrix);

disp('Closed-loop eigenvalues:');
disp(eigenvalues);

disp('Closed-loop eigenvectors in [theta, r, theta_dot, r_dot] order:');
disp(eigenvectors_x);

% LatModel_SignCorrection state order:
% z = [theta; theta_dot; r; r_dot]
T_x_to_z = [1, 0, 0, 0;
            0, 0, 1, 0;
            0, 1, 0, 0;
            0, 0, 0, 1];

eigenvectors_z = T_x_to_z*eigenvectors_x;

% Convert the feedback gain to the nonlinear model state order.
K_z = [K_numeric(1), K_numeric(3), K_numeric(2), K_numeric(4)];

%% 4. Nonzero equilibrium and nonlinear simulation

% The A and B matrices above are linearized at theta = 0. Therefore, the
% eigenvectors are exact at the upright origin and approximate when a
% nonzero equilibrium angle is used here.
theta_eq = 0.1;  % rad

G = par.m_W*par.g*par.R ...
  + par.m_B*par.g*(par.R + par.h);

r_eq = (G + 2*par.m_L*par.g*par.R) ...
     / (2*par.m_L*par.g) ...
     * tan(theta_eq);

F_eq = 2*m_L*g*sin(theta_eq);

% Nonlinear-model equilibrium state order:
% z_eq = [theta_eq; theta_dot_eq; r_eq; r_dot_eq]
z_eq = [0; 0; 0; 0];

% Equilibrium feedforward plus feedback about the equilibrium state.
controller = @(t, z, par) - K_z*(z - z_eq);

% Add a small perturbation so the return trajectory is visible.
z0 = z_eq + [0.1*pi/180; 0.00; 0.000; 0];

tspan = [0, 15];

[t_out, z_out] = ode45( ...
    @(t, z) LatModel_SignCorrection(t, z, par, controller), ...
    tspan, ...
    z0);

% Recalculate the input force along the trajectory.
F_out = zeros(length(t_out), 1);

for i = 1:length(t_out)
    F_out(i) = controller(t_out(i), z_out(i, :).', par);
end

% All eigenvectors describe state deviations about the equilibrium.
delta_z_out = z_out - z_eq.';

% Simulate the linear closed-loop model as well. This is the trajectory that
% corresponds exactly to A_cl_numeric and its eigenvectors.
delta_x0 = [z0(1) - theta_eq;
            z0(3) - r_eq;
            z0(2);
            z0(4)];

[t_linear, delta_x_linear] = ode45( ...
    @(t, delta_x) A_cl_numeric*delta_x, ...
    tspan, ...
    delta_x0);

delta_z_linear = (T_x_to_z*delta_x_linear.').';

if norm(delta_z_out(end, :)) > 10*norm(delta_z_out(1, :))
    warning(['The nonlinear trajectory diverged from the equilibrium. ', ...
        'Check whether the force sign and state order in ', ...
        'LatModel_SignCorrection agree with the A and B matrices.']);
end

%% 5. State and force time histories

figure('Name', 'Closed-loop time histories', 'Color', 'w');

subplot(3, 1, 1);
plot(t_out, z_out(:, 1), 'LineWidth', 1.5);
hold on;
yline(theta_eq, '--k', 'Equilibrium');
hold off;
ylabel('$\theta\; (\mathrm{rad})$', 'Interpreter', 'latex');
title('Pole-placement controlled nonlinear system');
grid on;

subplot(3, 1, 2);
plot(t_out, z_out(:, 3), 'LineWidth', 1.5);
hold on;
yline(r_eq, '--k', 'Equilibrium');
hold off;
ylabel('$r\; (\mathrm{m})$', 'Interpreter', 'latex');
grid on;

subplot(3, 1, 3);
plot(t_out, F_out, 'r', 'LineWidth', 1.5);
hold on;
yline(F_eq, '--k', 'Equilibrium');
hold off;
xlabel('$t\; (\mathrm{s})$', 'Interpreter', 'latex');
ylabel('$F\; (\mathrm{N})$', 'Interpreter', 'latex');
grid on;

%% 6. Eigenvectors and trajectory in the theta-r plane

figure('Name', 'Eigenvectors in theta-r plane', 'Color', 'w');
hold on;
grid on;
box on;

plot(delta_z_linear(:, 1), delta_z_linear(:, 3), ...
    'k', ...
    'LineWidth', 2.0, ...
    'DisplayName', 'Linear closed-loop trajectory');

plot(delta_z_out(:, 1), delta_z_out(:, 3), ...
    '--', ...
    'Color', [0.6, 0.6, 0.6], ...
    'LineWidth', 1.2, ...
    'DisplayName', 'Nonlinear trajectory');

plot(delta_z_out(1, 1), delta_z_out(1, 3), ...
    'ko', ...
    'MarkerFaceColor', 'y', ...
    'DisplayName', 'Initial state');

plot(0, 0, ...
    'kp', ...
    'MarkerSize', 11, ...
    'MarkerFaceColor', 'g', ...
    'DisplayName', 'Equilibrium');

mode_colors = lines(4);

% Fix the plot around the equilibrium so a diverging nonlinear trajectory
% cannot make the modal directions disappear.
theta_limit = max(2.5*abs(delta_z_linear(1, 1)), 0.03);
r_limit     = max(2.5*abs(delta_z_linear(1, 3)), 0.10);

for i = 1:4
    v_theta_r = real([eigenvectors_z(1, i); eigenvectors_z(3, i)]);
    projection_norm = norm(v_theta_r);

    if projection_norm < 1e-12
        continue;
    end

    % Scale the modal direction to fill 75 percent of the local plot window
    % while preserving its physical theta-r slope.
    modal_scale = 0.75/max(abs(v_theta_r(1))/theta_limit, ...
                           abs(v_theta_r(2))/r_limit);
    v_theta_r = modal_scale*v_theta_r;

    plot([-v_theta_r(1), v_theta_r(1)], ...
         [-v_theta_r(2), v_theta_r(2)], ...
        '--', ...
        'Color', mode_colors(i, :), ...
        'LineWidth', 1.2, ...
        'HandleVisibility', 'off');

    quiver(0, 0, ...
        v_theta_r(1), ...
        v_theta_r(2), ...
        0, ...
        'Color', mode_colors(i, :), ...
        'LineWidth', 2, ...
        'MaxHeadSize', 0.8, ...
        'DisplayName', sprintf('Mode %d: lambda = %.2f', ...
        i, real(eigenvalues(i))));

    quiver(0, 0, ...
        -v_theta_r(1), ...
        -v_theta_r(2), ...
        0, ...
        'Color', mode_colors(i, :), ...
        'LineWidth', 2, ...
        'MaxHeadSize', 0.8, ...
        'HandleVisibility', 'off');
end

xlabel('$\delta\theta\; (\mathrm{rad})$', ...
    'Interpreter', 'latex');
ylabel('$\delta r\; (\mathrm{m})$', ...
    'Interpreter', 'latex');
title('Closed-loop modes and local trajectories in the $\theta$-$r$ plane', ...
    'Interpreter', 'latex');
xlim([-theta_limit, theta_limit]);
ylim([-r_limit, r_limit]);
legend('Location', 'best');
hold off;

%% 7. Eigenvectors and trajectory in the theta_dot-r_dot plane

figure('Name', 'Eigenvectors in velocity plane', 'Color', 'w');
hold on;
grid on;
box on;

plot(delta_z_linear(:, 2), delta_z_linear(:, 4), ...
    'k', ...
    'LineWidth', 2.0, ...
    'DisplayName', 'Linear closed-loop trajectory');

plot(delta_z_out(:, 2), delta_z_out(:, 4), ...
    '--', ...
    'Color', [0.6, 0.6, 0.6], ...
    'LineWidth', 1.2, ...
    'DisplayName', 'Nonlinear trajectory');

plot(delta_z_out(1, 2), delta_z_out(1, 4), ...
    'ko', ...
    'MarkerFaceColor', 'y', ...
    'DisplayName', 'Initial state');

plot(0, 0, ...
    'kp', ...
    'MarkerSize', 11, ...
    'MarkerFaceColor', 'g', ...
    'DisplayName', 'Equilibrium');

theta_dot_limit = max(2.5*abs(delta_z_linear(1, 2)), 0.10);
r_dot_limit     = max(2.5*abs(delta_z_linear(1, 4)), 0.35);

for i = 1:4
    v_velocity = real([eigenvectors_z(2, i); eigenvectors_z(4, i)]);
    projection_norm = norm(v_velocity);

    if projection_norm < 1e-12
        continue;
    end

    modal_scale = 0.75/max(abs(v_velocity(1))/theta_dot_limit, ...
                           abs(v_velocity(2))/r_dot_limit);
    v_velocity = modal_scale*v_velocity;

    plot([-v_velocity(1), v_velocity(1)], ...
         [-v_velocity(2), v_velocity(2)], ...
        '--', ...
        'Color', mode_colors(i, :), ...
        'LineWidth', 1.2, ...
        'HandleVisibility', 'off');

    quiver(0, 0, ...
        v_velocity(1), ...
        v_velocity(2), ...
        0, ...
        'Color', mode_colors(i, :), ...
        'LineWidth', 2, ...
        'MaxHeadSize', 0.8, ...
        'DisplayName', sprintf('Mode %d: lambda = %.2f', ...
        i, real(eigenvalues(i))));

    quiver(0, 0, ...
        -v_velocity(1), ...
        -v_velocity(2), ...
        0, ...
        'Color', mode_colors(i, :), ...
        'LineWidth', 2, ...
        'MaxHeadSize', 0.8, ...
        'HandleVisibility', 'off');
end

xlabel('$\delta\dot{\theta}\; (\mathrm{rad/s})$', ...
    'Interpreter', 'latex');
ylabel('$\delta\dot{r}\; (\mathrm{m/s})$', ...
    'Interpreter', 'latex');
title(['Closed-loop modes and local trajectories in the ', ...
       '$\dot{\theta}$-$\dot{r}$ plane'], ...
    'Interpreter', 'latex');
xlim([-theta_dot_limit, theta_dot_limit]);
ylim([-r_dot_limit, r_dot_limit]);
legend('Location', 'best');
hold off;
