clear;
clc;
close all;

% Resolve folders relative to this script.
script_dir = fileparts(mfilename('fullpath'));

if isempty(script_dir)
    script_dir = pwd;
end

addpath(fullfile(script_dir, 'param'));

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
% Linear state:
% x = [theta; r; theta_dot; r_dot]
%
% Linearization at the stationary upright origin.
% No damping or friction, consistent with the supplied Appell model.

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

%% 3. Gains and closed-loop eigenvectors

P_desired = [-2.25, -1.25, -2.00, -1.50];

% true: keep your original manually specified gains.
% false: compute gains from P_desired.
use_manual_gain = false;

if use_manual_gain
    K_numeric = [-805.909522, 900, -46.963696, 50];
    disp('Using manually specified gains.');
else
    K_numeric = place(A, B, P_desired);
    disp('Using pole-placement gains.');
end

disp('K in [theta, r, theta_dot, r_dot] order:');
disp(K_numeric);

A_cl_numeric = A - B*K_numeric;

[eigenvectors_x, eigenvalue_matrix] = eig(A_cl_numeric);
eigenvalues = diag(eigenvalue_matrix);

disp('Actual closed-loop eigenvalues:');
disp(eigenvalues);

disp('Closed-loop eigenvectors in linear state order:');
disp(eigenvectors_x);

% Nonlinear state:
% z = [theta; theta_dot; r; r_dot]
%
% z = T_x_to_z*x
T_x_to_z = [1, 0, 0, 0;
            0, 0, 1, 0;
            0, 1, 0, 0;
            0, 0, 0, 1];

eigenvectors_z = T_x_to_z*eigenvectors_x;

% F = -K_numeric*x = -K_z*z
K_z = K_numeric*T_x_to_z.';

%% 4. Consistent equilibrium and initial conditions
% Both simulations use the origin because A and B were derived there.

theta_eq = 0;
r_eq     = 0;
F_eq     = 0;

z_eq = [theta_eq; 0; r_eq; 0];

controller = @(t, z) F_eq - K_z*(z(:) - z_eq);

% Identical physical perturbation for both simulations.
delta_z0 = [0.1*pi/180; 0; 0; 0];

z0       = z_eq + delta_z0;
delta_x0 = T_x_to_z.'*delta_z0;

tspan = [0, 15];
options = odeset('RelTol', 1e-8, 'AbsTol', 1e-10);

% The nonlinear model is defined at the end of this file.
[t_out, z_out] = ode45( ...
    @(t, z) appell_lateral_rhs(t, z, par, controller), ...
    tspan, z0, options);

[t_linear, delta_x_linear] = ode45( ...
    @(t, delta_x) A_cl_numeric*delta_x, ...
    tspan, delta_x0, options);

delta_z_out = z_out - z_eq.';
delta_z_linear = (T_x_to_z*delta_x_linear.').';

z_linear = delta_z_linear + z_eq.';

F_out = F_eq - delta_z_out*K_z.';
F_linear = F_eq - delta_x_linear*K_numeric.';

%% 5. State and force time histories

figure('Name', 'Closed-loop time histories', 'Color', 'w');

subplot(3, 1, 1);

plot(t_out, z_out(:, 1), 'b', 'LineWidth', 1.5);
hold on;
plot(t_linear, z_linear(:, 1), '--k', 'LineWidth', 1.2);
yline(theta_eq, ':', 'Equilibrium');
hold off;

ylabel('$\theta\;(\mathrm{rad})$', 'Interpreter', 'latex');
title('Linear and nonlinear closed-loop responses');
legend('Nonlinear', 'Linear', 'Location', 'best');
grid on;

subplot(3, 1, 2);

plot(t_out, z_out(:, 3), 'b', 'LineWidth', 1.5);
hold on;
plot(t_linear, z_linear(:, 3), '--k', 'LineWidth', 1.2);
yline(r_eq, ':', 'Equilibrium');
hold off;

ylabel('$r\;(\mathrm{m})$', 'Interpreter', 'latex');
grid on;

subplot(3, 1, 3);

plot(t_out, F_out, 'r', 'LineWidth', 1.5);
hold on;
plot(t_linear, F_linear, '--k', 'LineWidth', 1.2);
yline(F_eq, ':', 'Equilibrium');
hold off;

xlabel('$t\;(\mathrm{s})$', 'Interpreter', 'latex');
ylabel('$F\;(\mathrm{N})$', 'Interpreter', 'latex');
grid on;

%% 6. Eigenvectors and trajectories in the theta-r plane

plot_modal_projection( ...
    delta_z_linear, delta_z_out, ...
    eigenvectors_z, eigenvalues, ...
    [1, 3], [0.03, 0.10], ...
    'Position plane', ...
    '$\delta\theta\;(\mathrm{rad})$', ...
    '$\delta r\;(\mathrm{m})$');

%% 7. Eigenvectors and trajectories in the velocity plane

plot_modal_projection( ...
    delta_z_linear, delta_z_out, ...
    eigenvectors_z, eigenvalues, ...
    [2, 4], [0.10, 0.35], ...
    'Velocity plane', ...
    '$\delta\dot{\theta}\;(\mathrm{rad/s})$', ...
    '$\delta\dot r\;(\mathrm{m/s})$');

%% Local functions

function dz = appell_lateral_rhs(t, z, par, controller)
% Nonlinear equations from the supplied LatModel_Appel document.
%
% State: z = [theta; theta_dot; r; r_dot]
% Input: F acts positively on the moving rod.
% No damping, friction, or force saturation.

    z = z(:);

    theta     = z(1);
    theta_dot = z(2);
    r         = z(3);
    r_dot     = z(4);

    R = par.R;
    g = par.g;

    m_rod = 2*par.m_L;

    I0 = par.m_W*R^2 ...
       + par.m_B*(R + par.h)^2 ...
       + par.I_b ...
       + par.I_rod ...
       + par.I_w;

    G = g*(par.m_W*R + par.m_B*(R + par.h));

    % Pseudovelocities used in the Appell model.
    u1 = theta_dot;
    u2 = r_dot - R*theta_dot;

    F = controller(t, z);

    u1_dot = ( ...
          R*F ...
        - m_rod*g*r*cos(theta) ...
        + G*sin(theta) ...
        - m_rod*r*(2*u2*u1 + R*u1^2) ...
        ) / (I0 + m_rod*r^2);

    u2_dot = F/m_rod - g*sin(theta) + r*u1^2;

    % u2 = r_dot - R*theta_dot
    theta_ddot = u1_dot;
    r_ddot = u2_dot + R*u1_dot;

    dz = [theta_dot;
          theta_ddot;
          r_dot;
          r_ddot];
end

function plot_modal_projection( ...
    z_linear, z_nonlinear, V, lambda, ...
    indices, minimum_limits, figure_name, x_label, y_label)

    figure('Name', figure_name, 'Color', 'w');
    hold on;
    grid on;
    box on;

    ix = indices(1);
    iy = indices(2);

    plot(z_linear(:, ix), z_linear(:, iy), ...
        'k', 'LineWidth', 2, ...
        'DisplayName', 'Linear trajectory');

    plot(z_nonlinear(:, ix), z_nonlinear(:, iy), ...
        '--', 'Color', [0.6, 0.6, 0.6], ...
        'LineWidth', 1.2, ...
        'DisplayName', 'Nonlinear trajectory');

    plot(z_nonlinear(1, ix), z_nonlinear(1, iy), ...
        'ko', 'MarkerFaceColor', 'y', ...
        'DisplayName', 'Initial state');

    plot(0, 0, 'kp', ...
        'MarkerSize', 11, 'MarkerFaceColor', 'g', ...
        'DisplayName', 'Equilibrium');

    x_limit = max( ...
        1.2*max(abs(z_linear(:, ix))), minimum_limits(1));

    y_limit = max( ...
        1.2*max(abs(z_linear(:, iy))), minimum_limits(2));

    colors = lines(numel(lambda));

    for i = 1:numel(lambda)

        % Plot a conjugate pair only once.
        if imag(lambda(i)) < -1e-8
            continue;
        end

        v = V(:, i);

        % Fix arbitrary eigenvector phase for a reproducible projection.
        [~, pivot] = max(abs(v));
        v = v*exp(-1i*angle(v(pivot)));

        if abs(imag(lambda(i))) < 1e-8
            directions = real(v(indices));
            labels = {sprintf('\\lambda = %.3g', real(lambda(i)))};
        else
            % A complex mode has a real invariant plane, not one
            % real eigenvector direction. Show its projected basis.
            directions = [real(v(indices)), imag(v(indices))];

            labels = { ...
                sprintf('Re(v), \\lambda = %.3g + %.3gi', ...
                    real(lambda(i)), imag(lambda(i))), ...
                sprintf('Im(v), \\lambda = %.3g + %.3gi', ...
                    real(lambda(i)), imag(lambda(i)))};
        end

        for j = 1:size(directions, 2)

            direction = directions(:, j);

            scale_denominator = max( ...
                abs(direction(1))/x_limit, ...
                abs(direction(2))/y_limit);

            if scale_denominator < 1e-12
                continue;
            end

            direction = 0.75*direction/scale_denominator;

            if j == 1
                line_style = '--';
            else
                line_style = ':';
            end

            plot([-direction(1), direction(1)], ...
                 [-direction(2), direction(2)], ...
                 'LineStyle', line_style, ...
                 'Color', colors(i, :), ...
                 'LineWidth', 1.3, ...
                 'DisplayName', labels{j});
        end
    end

    xlabel(x_label, 'Interpreter', 'latex');
    ylabel(y_label, 'Interpreter', 'latex');
    title(figure_name);

    xlim([-x_limit, x_limit]);
    ylim([-y_limit, y_limit]);

    legend('Location', 'best');
    hold off;
end