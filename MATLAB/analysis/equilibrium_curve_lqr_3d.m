clear; clc; close all;

addpath("param/", "model/");
par = LatParam();

%% 1. Symbolic nonlinear model and linearization

syms theta theta_dot r r_dot F_sym real
syms m_L m_B m_W h g R real
syms I_b I_w I_rod real

z_sym = [theta; theta_dot; r; r_dot];

M_matrix = [2*m_L*R^2 + m_B*(R+h)^2 + m_W*R^2 ...
            + 2*m_L*r^2 + I_b + I_w + I_rod,  -2*m_L*R;
            -2*m_L*R,                            2*m_L];

M_rightside = [
    2*m_L*g*R*sin(theta) ...
    - 2*m_L*g*r*cos(theta) ...
    + m_B*g*(R+h)*sin(theta) ...
    + m_W*g*R*sin(theta) ...
    - 4*m_L*r*r_dot*theta_dot;
    -2*m_L*g*sin(theta) + F_sym + 2*m_L*r*theta_dot^2
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

symbolic_parameters = [m_L, m_B, m_W, h, g, R, I_b, I_w, I_rod];
numeric_parameters = [par.m_L, par.m_B, par.m_W, par.h, par.g, par.R, ...
                      par.I_b, par.I_w, par.I_rod];

A_num = double(subs(A_eq, symbolic_parameters, numeric_parameters));
B_num = double(subs(B_eq, symbolic_parameters, numeric_parameters));

%% 2. Design LQR and run simulation

Q = diag([1000, 100, 100, 10]);
R_weight = 30;

K = [-781.4476, -146.7735, 229.6972, 41.4447];% lqr(A_num, B_num, Q, R_weight);

disp('Gain K for z = [theta, theta_dot, r, r_dot]:');
disp(K);

lqr_controller = @(t, z, par) -K*z;

z0 = [0.18*pi/180; 0; 0; 0];
tspan = [0, 15];

[t_out, z_out] = ode45( ...
    @(t, z) LatModel_SignCorrection(t, z, par, lqr_controller), ...
    tspan, z0);

F_out = zeros(length(t_out), 1);
for i = 1:length(t_out)
    F_out(i) = lqr_controller(t_out(i), z_out(i,:)', par);
end

theta_out = z_out(:,1);
theta_dot_out = z_out(:,2);
r_out = z_out(:,3);
r_dot_out = z_out(:,4);

%% 3. Static equilibrium curve

% Simplified model parameters:
%
%   m = total moving rod mass = 2*m_L
%
%   G = m_B*g*(R+h) + m_W*g*R
%
% The rod contribution m*g*R appears after substituting the static
% equilibrium force F_eq = m*g*sin(theta).
m = 2*par.m_L;
G = par.m_B*par.g*(par.R + par.h) + par.m_W*par.g*par.R;

equilibrium_coefficient = ...
    (G + m*par.g*par.R)/(m*par.g);

% True static equilibrium:
%
%   r_eq(theta) = equilibrium_coefficient*tan(theta)
%   F_eq(theta) = m*g*sin(theta)
r_eq = @(theta_value) ...
    equilibrium_coefficient*tan(theta_value);

F_eq = @(theta_value) ...
    m*par.g*sin(theta_value);

fprintf('\nStatic equilibrium curve:\n');
fprintf('r_eq(theta) = %.8g*tan(theta) m\n', ...
    equilibrium_coefficient);

%% 4. Conventional time-history plots

figure('Color', 'w', 'Name', 'LQR time histories');

subplot(3,1,1);
plot(t_out, theta_out, 'LineWidth', 1.5);
ylabel('Theta (rad)');
title('Controlled System');
grid on;

subplot(3,1,2);
plot(t_out, r_out, 'LineWidth', 1.5);
ylabel('Position r (m)');
grid on;

subplot(3,1,3);
plot(t_out, F_out, 'r', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Force F (N)');
grid on;

%% 5. Plot ranges

[theta_min, theta_max] = expanded_limits(theta_out, 0.05*pi/180);
[theta_dot_min, theta_dot_max] = expanded_limits(theta_dot_out, 0.01);
[r_dot_min, r_dot_max] = expanded_limits(r_dot_out, 0.005);

% Avoid the singularities of tan(theta).
theta_min = max(theta_min, -80*pi/180);
theta_max = min(theta_max,  80*pi/180);

theta_grid = linspace(theta_min, theta_max, 151);
theta_dot_grid = linspace(theta_dot_min, theta_dot_max, 101);
r_dot_grid = linspace(r_dot_min, r_dot_max, 101);

r_eq_grid = r_eq(theta_grid);

[r_plot_min, r_plot_max] = expanded_limits( ...
    [r_out; r_eq_grid(:)], 0.01);

equilibrium_surface_color = [0.35, 0.72, 0.92];

%% 6. Separate 3-D plot with theta_dot as the vertical axis

[Theta_surface_1, ThetaDot_surface] = ...
    meshgrid(theta_grid, theta_dot_grid);

R_surface_1 = r_eq(Theta_surface_1);

figure('Color', 'w', ...
    'Name', 'Static equilibrium surface - theta dot');
hold on;

surf(Theta_surface_1, R_surface_1, ThetaDot_surface, ...
    'FaceColor', equilibrium_surface_color, ...
    'FaceAlpha', 0.28, ...
    'EdgeColor', 'none', ...
    'DisplayName', sprintf( ...
    '$r_{\\mathrm{eq}}=%.4g\\tan(\\theta)$', ...
    equilibrium_coefficient));

plot3(theta_out, r_out, theta_dot_out, ...
    'k-', ...
    'LineWidth', 2.3, ...
    'DisplayName', 'Simulation trajectory');

% The blue line is the actual static-equilibrium curve at theta_dot = 0.
plot3(theta_grid, r_eq_grid, zeros(size(theta_grid)), ...
    'b-', ...
    'LineWidth', 2.4, ...
    'DisplayName', 'Static-equilibrium curve');

scatter3(theta_out(1), r_out(1), theta_dot_out(1), ...
    55, [0.10, 0.65, 0.10], 'filled', ...
    'DisplayName', 'Initial state');

scatter3(theta_out(end), r_out(end), theta_dot_out(end), ...
    55, [0.75, 0.10, 0.75], 'filled', ...
    'DisplayName', 'Final state');

xlabel('$\theta$ (rad)', 'Interpreter', 'latex');
ylabel('$r$ (m)', 'Interpreter', 'latex');
zlabel('$\dot{\theta}$ (rad/s)', 'Interpreter', 'latex');
title({'Simulation relative to the static-equilibrium surface', ...
       'Vertical axis: $\dot{\theta}$'}, ...
      'Interpreter', 'latex');

xlim([theta_min, theta_max]);
ylim([r_plot_min, r_plot_max]);
zlim([theta_dot_min, theta_dot_max]);
grid on;
box on;
view(44, 27);
legend('Location', 'best', 'Interpreter', 'latex');

%% 7. Separate 3-D plot with r_dot as the vertical axis

[Theta_surface_2, RDot_surface] = ...
    meshgrid(theta_grid, r_dot_grid);

R_surface_2 = r_eq(Theta_surface_2);

figure('Color', 'w', ...
    'Name', 'Static equilibrium surface - r dot');
hold on;

surf(Theta_surface_2, R_surface_2, RDot_surface, ...
    'FaceColor', equilibrium_surface_color, ...
    'FaceAlpha', 0.28, ...
    'EdgeColor', 'none', ...
    'DisplayName', sprintf( ...
    '$r_{\\mathrm{eq}}=%.4g\\tan(\\theta)$', ...
    equilibrium_coefficient));

plot3(theta_out, r_out, r_dot_out, ...
    'k-', ...
    'LineWidth', 2.3, ...
    'DisplayName', 'Simulation trajectory');

% The blue line is the actual static-equilibrium curve at r_dot = 0.
plot3(theta_grid, r_eq_grid, zeros(size(theta_grid)), ...
    'b-', ...
    'LineWidth', 2.4, ...
    'DisplayName', 'Static-equilibrium curve');

scatter3(theta_out(1), r_out(1), r_dot_out(1), ...
    55, [0.10, 0.65, 0.10], 'filled', ...
    'DisplayName', 'Initial state');

scatter3(theta_out(end), r_out(end), r_dot_out(end), ...
    55, [0.75, 0.10, 0.75], 'filled', ...
    'DisplayName', 'Final state');

xlabel('$\theta$ (rad)', 'Interpreter', 'latex');
ylabel('$r$ (m)', 'Interpreter', 'latex');
zlabel('$\dot r$ (m/s)', 'Interpreter', 'latex');
title({'Simulation relative to the static-equilibrium surface', ...
       'Vertical axis: $\dot r$'}, ...
      'Interpreter', 'latex');

xlim([theta_min, theta_max]);
ylim([r_plot_min, r_plot_max]);
zlim([r_dot_min, r_dot_max]);
grid on;
box on;
view(44, 27);
legend('Location', 'best', 'Interpreter', 'latex');

%% 8. Final-state equilibrium comparison

r_eq_final = r_eq(theta_out(end));
F_eq_final = F_eq(theta_out(end));

fprintf('\nFinal state:\n');
fprintf('theta                = %+.6e rad\n', theta_out(end));
fprintf('r                    = %+.6e m\n', r_out(end));
fprintf('r_eq(theta)          = %+.6e m\n', r_eq_final);
fprintf('r-r_eq(theta)        = %+.6e m\n', r_out(end)-r_eq_final);
fprintf('theta_dot            = %+.6e rad/s\n', theta_dot_out(end));
fprintf('r_dot                = %+.6e m/s\n', r_dot_out(end));
fprintf('controller force     = %+.6e N\n', F_out(end));
fprintf('static-equilibrium F = %+.6e N\n', F_eq_final);

%% Local helper function

function [lower_limit, upper_limit] = expanded_limits(data, minimum_margin)
    finite_data = data(isfinite(data));

    if isempty(finite_data)
        lower_limit = -minimum_margin;
        upper_limit = minimum_margin;
        return;
    end

    lower_limit = min(finite_data);
    upper_limit = max(finite_data);
    data_range = upper_limit-lower_limit;
    margin = max(0.12*data_range, minimum_margin);

    lower_limit = lower_limit-margin;
    upper_limit = upper_limit+margin;
end
