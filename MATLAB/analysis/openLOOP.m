clear;
clc;
close all;

addpath("param/", "model/");

%% 1. System parameters

par = LatParam();

m = par.m_rod;
R = par.R;
g = par.g;
J = par.J;
G = par.G;

%% 2. Select an open-loop equilibrium
%
% Use theta_eq = 0 to reproduce the upright-equilibrium results in the
% report. For example, change it to 0.1 to analyze the exact open-loop
% linearization at the corresponding nonzero equilibrium.

theta_eq = 0.0;  % rad

r_eq = (G + m*g*R)/(m*g)*tan(theta_eq);
F_eq = m*g*sin(theta_eq);
J_star = J + m*r_eq^2;

fprintf('Open-loop equilibrium:\n');
fprintf('  theta_eq = %.9f rad\n', theta_eq);
fprintf('  r_eq     = %.9f m\n', r_eq);
fprintf('  F_eq     = %.9f N\n', F_eq);
fprintf('  J_star   = %.9f kg m^2\n\n', J_star);

%% 3. Exact open-loop linearization at the selected equilibrium
%
% Generalized-speed state order:
%
% y = [theta; r; sigma_1; sigma_2]
%
% sigma_1 = theta_dot
% sigma_2 = r_dot - R*theta_dot

a_theta = ...
    (m*g*r_eq*sin(theta_eq) + G*cos(theta_eq))/J_star;

a_r = -m*g*cos(theta_eq)/J_star;
c   = -g*cos(theta_eq);

A_sigma = [
    0,       0,   1, 0;
    0,       0,   R, 1;
    a_theta, a_r, 0, 0;
    c,       0,   0, 0
];

B_sigma = [
    0;
    0;
    R/J_star;
    1/m
];

%% 4. Analytical open-loop eigenvalues and eigenvectors

mu = a_theta + R*a_r;

D = sqrt( ...
    mu^2 ...
    + 4*m*g^2*cos(theta_eq)^2/J_star);

lambda_real = sqrt((mu + D)/2);
omega       = sqrt((D - mu)/2);

% Fixed ordering:
%   1: unstable real mode
%   2: stable real mode
%   3: positive imaginary mode
%   4: negative imaginary mode
lambda_analytical = [
     lambda_real;
    -lambda_real;
     1i*omega;
    -1i*omega
];

% Normalize every eigenvector so that delta_theta = 1.
V_sigma = complex(zeros(4, 4));

for i = 1:4
    lambda_i = lambda_analytical(i);

    V_sigma(:, i) = [
        1;
        R - g*cos(theta_eq)/lambda_i^2;
        lambda_i;
        -g*cos(theta_eq)/lambda_i
    ];
end

disp('Analytical open-loop eigenvalues:');
disp(lambda_analytical);

disp(['Analytical eigenvectors in ', ...
      '[theta, r, sigma_1, sigma_2] order, with theta = 1:']);
disp(V_sigma);

%% 5. Convert to the physical-velocity state
%
% Physical state order:
%
% x = [theta; r; theta_dot; r_dot]
%
% The coordinate relation is x = S_sigma_to_x*y.

S_sigma_to_x = [
    1, 0, 0, 0;
    0, 1, 0, 0;
    0, 0, 1, 0;
    0, 0, R, 1
];

A_x = S_sigma_to_x*A_sigma/S_sigma_to_x;
B_x = S_sigma_to_x*B_sigma;
V_x = S_sigma_to_x*V_sigma;

disp(['The same eigenvectors in ', ...
      '[theta, r, theta_dot, r_dot] order, with theta = 1:']);
disp(V_x);

% Independent numerical check using MATLAB eig().
[V_x_numeric, D_x_numeric] = eig(A_x);
lambda_numeric = diag(D_x_numeric);

disp('Numerical eigenvalues returned by eig(A_x):');
disp(lambda_numeric);

% Verify A_x*v_i = lambda_i*v_i for the analytical vectors.
eigenvector_residuals = zeros(4, 1);

for i = 1:4
    eigenvector_residuals(i) = norm( ...
        A_x*V_x(:, i) ...
        - lambda_analytical(i)*V_x(:, i));
end

disp('Residual norms ||A_x*v_i - lambda_i*v_i||:');
disp(eigenvector_residuals);

if max(eigenvector_residuals) > 1e-9
    warning('The analytical eigenvectors failed the residual check.');
end

% eig() uses unit-norm eigenvectors, arbitrary column order, and arbitrary
% complex phase. Normalize its columns by theta only for easier inspection.
V_x_numeric_theta_normalized = V_x_numeric;

for i = 1:4
    if abs(V_x_numeric_theta_normalized(1, i)) > 1e-12
        V_x_numeric_theta_normalized(:, i) = ...
            V_x_numeric_theta_normalized(:, i) ...
            / V_x_numeric_theta_normalized(1, i);
    end
end

disp(['Numerical eig() eigenvectors in ', ...
      '[theta, r, theta_dot, r_dot] order, normalized by theta:']);
disp(V_x_numeric_theta_normalized);

%% 6. Nonlinear and linear open-loop simulations
%
% LatModel_SignCorrection state order:
%
% z = [theta; theta_dot; r; r_dot]

T_x_to_z = [
    1, 0, 0, 0;
    0, 0, 1, 0;
    0, 1, 0, 0;
    0, 0, 0, 1
];

V_z = T_x_to_z*V_x;

z_eq = [
    theta_eq;
    0;
    r_eq;
    0
];

% Open loop means that the applied force is held fixed at F_eq. There is
% no state feedback. At the upright equilibrium, F_eq = 0.
open_loop_input = @(t, z, par) F_eq; %#ok<INUSD>

% Small lean perturbation about the selected equilibrium.
z0 = z_eq + [
    0.1*pi/180;
    0;
    0;
    0
];

% The open-loop system contains an unstable real mode, so a short interval
% is used to keep the local comparison readable.
tspan = [0, 1.0];

ode_options = odeset( ...
    'RelTol', 1e-9, ...
    'AbsTol', 1e-11, ...
    'MaxStep', 0.002);

[t_out, z_out] = ode45( ...
    @(t, z) LatModel_SignCorrection( ...
        t, z, par, open_loop_input), ...
    tspan, ...
    z0, ...
    ode_options);

F_out = F_eq*ones(length(t_out), 1);

% State deviations for the nonlinear trajectory.
delta_z_out = z_out - z_eq.';

% Initial deviation in x = [theta; r; theta_dot; r_dot] order.
delta_x0 = [
    z0(1) - z_eq(1);
    z0(3) - z_eq(3);
    z0(2) - z_eq(2);
    z0(4) - z_eq(4)
];

[t_linear, delta_x_linear] = ode45( ...
    @(t, delta_x) A_x*delta_x, ...
    tspan, ...
    delta_x0, ...
    ode_options);

delta_z_linear = (T_x_to_z*delta_x_linear.').';

%% 7. State and force time histories

figure('Name', 'Open-loop time histories', 'Color', 'w');

subplot(3, 1, 1);
plot(t_out, z_out(:, 1), 'LineWidth', 1.5);
hold on;
yline(theta_eq, '--k', 'Equilibrium');
hold off;
ylabel('$\theta\; (\mathrm{rad})$', 'Interpreter', 'latex');
title('Nonlinear open-loop response');
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
yline(F_eq, '--k', 'Equilibrium input');
hold off;
xlabel('$t\; (\mathrm{s})$', 'Interpreter', 'latex');
ylabel('$F\; (\mathrm{N})$', 'Interpreter', 'latex');
grid on;

%% 8. Open-loop modal directions in the theta-r plane

figure('Name', 'Open-loop modes in theta-r plane', 'Color', 'w');
hold on;
grid on;
box on;

plot(delta_z_linear(:, 1), delta_z_linear(:, 3), ...
    'k', ...
    'LineWidth', 2.0, ...
    'DisplayName', 'Linear open-loop trajectory');

plot(delta_z_out(:, 1), delta_z_out(:, 3), ...
    '--', ...
    'Color', [0.6, 0.6, 0.6], ...
    'LineWidth', 1.3, ...
    'DisplayName', 'Nonlinear open-loop trajectory');

plot(delta_z_out(1, 1), delta_z_out(1, 3), ...
    'ko', ...
    'MarkerFaceColor', 'y', ...
    'DisplayName', 'Initial deviation');

plot(0, 0, ...
    'kp', ...
    'MarkerSize', 11, ...
    'MarkerFaceColor', 'g', ...
    'DisplayName', 'Equilibrium');

theta_limit = max([ ...
    1.15*max(abs(delta_z_linear(:, 1))), ...
    1.15*max(abs(delta_z_out(:, 1))), ...
    0.03]);

r_limit = max([ ...
    1.15*max(abs(delta_z_linear(:, 3))), ...
    1.15*max(abs(delta_z_out(:, 3))), ...
    0.10]);

% The +/- real modes have the same theta-r projection. The complex
% conjugate modes also have the same theta-r projection. Therefore, plot
% one direction for each pair instead of drawing duplicate arrows.
position_mode_indices = [1, 3];
position_mode_colors = lines(2);
position_mode_labels = {
    sprintf('Real pair: lambda = +/-%.3f', lambda_real), ...
    sprintf('Imaginary pair: lambda = +/-%.3fi', omega)
};

for k = 1:length(position_mode_indices)
    i = position_mode_indices(k);
    v_theta_r = real(V_x(1:2, i));

    modal_scale = 0.75/max( ...
        abs(v_theta_r(1))/theta_limit, ...
        abs(v_theta_r(2))/r_limit);

    v_theta_r = modal_scale*v_theta_r;

    plot( ...
        [-v_theta_r(1), v_theta_r(1)], ...
        [-v_theta_r(2), v_theta_r(2)], ...
        '--', ...
        'Color', position_mode_colors(k, :), ...
        'LineWidth', 1.5, ...
        'DisplayName', position_mode_labels{k});

    quiver( ...
        0, 0, ...
        v_theta_r(1), v_theta_r(2), ...
        0, ...
        'Color', position_mode_colors(k, :), ...
        'LineWidth', 2, ...
        'MaxHeadSize', 0.8, ...
        'HandleVisibility', 'off');

    quiver( ...
        0, 0, ...
        -v_theta_r(1), -v_theta_r(2), ...
        0, ...
        'Color', position_mode_colors(k, :), ...
        'LineWidth', 2, ...
        'MaxHeadSize', 0.8, ...
        'HandleVisibility', 'off');
end

xlabel('$\delta\theta\; (\mathrm{rad})$', ...
    'Interpreter', 'latex');
ylabel('$\delta r\; (\mathrm{m})$', ...
    'Interpreter', 'latex');
title('Open-loop modes in the $\theta$-$r$ plane', ...
    'Interpreter', 'latex');
xlim([-theta_limit, theta_limit]);
ylim([-r_limit, r_limit]);
legend('Location', 'best');
hold off;

%% 9. Open-loop modal directions in the theta_dot-r_dot plane

figure('Name', 'Open-loop modes in velocity plane', 'Color', 'w');
hold on;
grid on;
box on;

plot(delta_z_linear(:, 2), delta_z_linear(:, 4), ...
    'k', ...
    'LineWidth', 2.0, ...
    'DisplayName', 'Linear open-loop trajectory');

plot(delta_z_out(:, 2), delta_z_out(:, 4), ...
    '--', ...
    'Color', [0.6, 0.6, 0.6], ...
    'LineWidth', 1.3, ...
    'DisplayName', 'Nonlinear open-loop trajectory');

plot(delta_z_out(1, 2), delta_z_out(1, 4), ...
    'ko', ...
    'MarkerFaceColor', 'y', ...
    'DisplayName', 'Initial deviation');

plot(0, 0, ...
    'kp', ...
    'MarkerSize', 11, ...
    'MarkerFaceColor', 'g', ...
    'DisplayName', 'Equilibrium');

theta_dot_limit = max([ ...
    1.15*max(abs(delta_z_linear(:, 2))), ...
    1.15*max(abs(delta_z_out(:, 2))), ...
    0.10]);

r_dot_limit = max([ ...
    1.15*max(abs(delta_z_linear(:, 4))), ...
    1.15*max(abs(delta_z_out(:, 4))), ...
    0.35]);

velocity_mode_indices = [1, 3];
velocity_mode_colors = lines(2);
velocity_mode_labels = {
    sprintf('Real pair: lambda = +/-%.3f', lambda_real), ...
    sprintf('Imaginary pair: lambda = +/-%.3fi', omega)
};

for k = 1:length(velocity_mode_indices)
    i = velocity_mode_indices(k);

    if abs(imag(lambda_analytical(i))) < 1e-12
        % A real eigenmode has a real velocity projection.
        v_velocity = real(V_x(3:4, i));
    else
        % After theta normalization, an oscillatory eigenvector has real
        % position components and imaginary velocity components. Its
        % physical velocity direction is therefore obtained from imag(v).
        v_velocity = imag(V_x(3:4, i));
    end

    modal_scale = 0.75/max( ...
        abs(v_velocity(1))/theta_dot_limit, ...
        abs(v_velocity(2))/r_dot_limit);

    v_velocity = modal_scale*v_velocity;

    plot( ...
        [-v_velocity(1), v_velocity(1)], ...
        [-v_velocity(2), v_velocity(2)], ...
        '--', ...
        'Color', velocity_mode_colors(k, :), ...
        'LineWidth', 1.5, ...
        'DisplayName', velocity_mode_labels{k});

    quiver( ...
        0, 0, ...
        v_velocity(1), v_velocity(2), ...
        0, ...
        'Color', velocity_mode_colors(k, :), ...
        'LineWidth', 2, ...
        'MaxHeadSize', 0.8, ...
        'HandleVisibility', 'off');

    quiver( ...
        0, 0, ...
        -v_velocity(1), -v_velocity(2), ...
        0, ...
        'Color', velocity_mode_colors(k, :), ...
        'LineWidth', 2, ...
        'MaxHeadSize', 0.8, ...
        'HandleVisibility', 'off');
end

xlabel('$\delta\dot{\theta}\; (\mathrm{rad/s})$', ...
    'Interpreter', 'latex');
ylabel('$\delta\dot r\; (\mathrm{m/s})$', ...
    'Interpreter', 'latex');
title('Open-loop modes in the velocity plane', ...
    'Interpreter', 'latex');
xlim([-theta_dot_limit, theta_dot_limit]);
ylim([-r_dot_limit, r_dot_limit]);
legend('Location', 'best');
hold off;
