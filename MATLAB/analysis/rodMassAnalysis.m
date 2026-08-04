% Sweep rod mass and redesign the LQR controller for every mass case.
clear; clc; close all;

addpath("param/", "model/");

%% 1. LQR settings and mass sweep settings

% State ordering:
% z = [theta; theta_dot; r; r_dot]
%
% For every m_L value:
%   1. Re-linearize the system
%   2. Redesign the LQR controller
%   3. Run the nonlinear simulation
Q = diag([100, 1000, 10, 100]);
R_weight = 10;

% Current definition in LatParam:
%
%   par.mass_at_end = 0.24;
%   par.m_L = 0.15 + par.mass_at_end;
%
% Nine cases are used. The first case is the current model.
base_rod_component_mass = 0.15;

mass_at_end_values = 3.0:0.30:4.0;
m_L_values = base_rod_component_mass + mass_at_end_values;

% Initial condition:
% z0 = [theta; theta_dot; r; r_dot]
z0 = [
    1.5*pi/180;
    0;
    0;
    0
];

tspan = [0, 15];

ode_options = odeset( ...
    'RelTol', 1e-8, ...
    'AbsTol', 1e-10, ...
    'Events', @stop_if_out_of_range);

number_of_cases = length(m_L_values);

%% 2. Generate colors from light blue to dark blue

% Smaller m_L: light blue
% Larger  m_L: dark blue
light_color = [0.68, 0.86, 0.98];
dark_color  = [0.02, 0.16, 0.48];

color_fraction = linspace(0, 1, number_of_cases)';
colors = zeros(number_of_cases, 3);

for i = 1:number_of_cases
    colors(i,:) = ...
        (1-color_fraction(i))*light_color ...
        + color_fraction(i)*dark_color;
end

%% 3. Storage for trajectories and summary data

t_all = cell(number_of_cases, 1);
z_all = cell(number_of_cases, 1);
F_all = cell(number_of_cases, 1);
par_all = cell(number_of_cases, 1);

K_all = zeros(number_of_cases, 4);
closed_loop_poles_all = zeros(number_of_cases, 4);
max_real_closed_loop_pole = zeros(number_of_cases, 1);

final_time = zeros(number_of_cases, 1);
completed_full_simulation = false(number_of_cases, 1);

peak_theta_deg = zeros(number_of_cases, 1);
peak_r_cm = zeros(number_of_cases, 1);
peak_theta_dot_degps = zeros(number_of_cases, 1);
peak_r_dot_mps = zeros(number_of_cases, 1);
peak_force_N = zeros(number_of_cases, 1);

final_theta_deg = zeros(number_of_cases, 1);
final_r_cm = zeros(number_of_cases, 1);

%% 4. Run all rod-mass cases

for i = 1:number_of_cases
    par_i = LatParam();

    % Keep both mass parameter fields consistent.
    par_i.mass_at_end = mass_at_end_values(i);
    par_i.m_L = m_L_values(i);

    % Re-linearize the system for the current rod mass.
    [A_i, B_i] = linearize_lat_model_at_origin(par_i);

    % Redesign the LQR controller for the current rod mass.
    K_i = lqr(A_i, B_i, Q, R_weight);

    % Control law:
    % F = -K*z
    controller_i = @(t, z, par) -K_i*z;

    K_all(i,:) = K_i;

    closed_loop_poles_i = eig(A_i-B_i*K_i);
    closed_loop_poles_all(i,:) = closed_loop_poles_i.';
    max_real_closed_loop_pole(i) = ...
        max(real(closed_loop_poles_i));

    % Run nonlinear simulation.
    [t_i, z_i] = ode45( ...
        @(t, z) LatModel_SignCorrection( ...
            t, z, par_i, controller_i), ...
        tspan, ...
        z0, ...
        ode_options);

    % Controller force at every saved simulation point.
    F_i = -(z_i*K_i');

    % Store trajectory.
    t_all{i} = t_i;
    z_all{i} = z_i;
    F_all{i} = F_i;
    par_all{i} = par_i;

    % Simulation completion information.
    final_time(i) = t_i(end);

    completed_full_simulation(i) = ...
        t_i(end) >= tspan(end)-1e-6;

    % Peak values.
    peak_theta_deg(i) = ...
        max(abs(z_i(:,1)))*180/pi;

    peak_theta_dot_degps(i) = ...
        max(abs(z_i(:,2)))*180/pi;

    peak_r_cm(i) = ...
        100*max(abs(z_i(:,3)));

    peak_r_dot_mps(i) = ...
        max(abs(z_i(:,4)));

    peak_force_N(i) = ...
        max(abs(F_i));

    % Final values.
    final_theta_deg(i) = ...
        z_i(end,1)*180/pi;

    final_r_cm(i) = ...
        100*z_i(end,3);
end

%% 5. Overlay all simulations in one figure

figure( ...
    'Color', 'w', ...
    'Name', 'Rod-mass sweep with redesigned LQR', ...
    'Position', [100, 80, 1250, 850]);

layout = tiledlayout(3, 2, ...
    'TileSpacing', 'compact', ...
    'Padding', 'compact');

%% Lean-angle plot

ax_theta = nexttile(layout, 1);

hold(ax_theta, 'on');
grid(ax_theta, 'on');
box(ax_theta, 'on');

yline(ax_theta, 0, ':', ...
    'HandleVisibility', 'off');

xlabel(ax_theta, 'Time (s)');

ylabel(ax_theta, '$\theta$ (deg)', ...
    'Interpreter', 'latex');

title(ax_theta, 'Lean angle');

%% Rod-position plot

ax_r = nexttile(layout, 2);

hold(ax_r, 'on');
grid(ax_r, 'on');
box(ax_r, 'on');

yline(ax_r, 0, ':', ...
    'HandleVisibility', 'off');

xlabel(ax_r, 'Time (s)');

ylabel(ax_r, '$r$ (cm)', ...
    'Interpreter', 'latex');

title(ax_r, 'Rod position');

%% Lean angular-velocity plot

ax_theta_dot = nexttile(layout, 3);

hold(ax_theta_dot, 'on');
grid(ax_theta_dot, 'on');
box(ax_theta_dot, 'on');

yline(ax_theta_dot, 0, ':', ...
    'HandleVisibility', 'off');

xlabel(ax_theta_dot, 'Time (s)');

ylabel(ax_theta_dot, '$\dot{\theta}$ (deg/s)', ...
    'Interpreter', 'latex');

title(ax_theta_dot, 'Lean angular velocity');

%% Rod-velocity plot

ax_r_dot = nexttile(layout, 4);

hold(ax_r_dot, 'on');
grid(ax_r_dot, 'on');
box(ax_r_dot, 'on');

yline(ax_r_dot, 0, ':', ...
    'HandleVisibility', 'off');

xlabel(ax_r_dot, 'Time (s)');

ylabel(ax_r_dot, '$\dot r$ (m/s)', ...
    'Interpreter', 'latex');

title(ax_r_dot, 'Rod velocity');

%% Controller-force plot

ax_force = nexttile(layout, 5);

hold(ax_force, 'on');
grid(ax_force, 'on');
box(ax_force, 'on');

yline(ax_force, 0, ':', ...
    'HandleVisibility', 'off');

xlabel(ax_force, 'Time (s)');

ylabel(ax_force, '$F$ (N)', ...
    'Interpreter', 'latex');

title(ax_force, 'Controller force');

%% Theta-r phase plot

ax_phase = nexttile(layout, 6);

hold(ax_phase, 'on');
grid(ax_phase, 'on');
box(ax_phase, 'on');

xline(ax_phase, 0, ':', ...
    'HandleVisibility', 'off');

yline(ax_phase, 0, ':', ...
    'HandleVisibility', 'off');

xlabel(ax_phase, '$\theta$ (deg)', ...
    'Interpreter', 'latex');

ylabel(ax_phase, '$r$ (cm)', ...
    'Interpreter', 'latex');

title(ax_phase, { ...
    '$\theta$--$r$ relationship', ...
    'solid: simulation, dashed: static equilibrium'}, ...
    'Interpreter', 'latex');

%% Plot every mass case

for i = 1:number_of_cases
    t_i = t_all{i};
    z_i = z_all{i};
    F_i = F_all{i};
    par_i = par_all{i};

    % Make the current physical model slightly thicker.
    if i == 1
        line_width = 3.0;
    else
        line_width = 1.8;
    end

    case_name = sprintf( ...
        '$m_L=%.2f$ kg, $m_{end}=%.2f$ kg', ...
        m_L_values(i), ...
        mass_at_end_values(i));

    % Lean angle
    plot(ax_theta, ...
        t_i, ...
        z_i(:,1)*180/pi, ...
        'Color', colors(i,:), ...
        'LineWidth', line_width, ...
        'DisplayName', case_name);

    % Rod position
    plot(ax_r, ...
        t_i, ...
        100*z_i(:,3), ...
        'Color', colors(i,:), ...
        'LineWidth', line_width, ...
        'DisplayName', case_name);

    % Lean angular velocity
    plot(ax_theta_dot, ...
        t_i, ...
        z_i(:,2)*180/pi, ...
        'Color', colors(i,:), ...
        'LineWidth', line_width, ...
        'DisplayName', case_name);

    % Rod velocity
    plot(ax_r_dot, ...
        t_i, ...
        z_i(:,4), ...
        'Color', colors(i,:), ...
        'LineWidth', line_width, ...
        'DisplayName', case_name);

    % Controller force
    plot(ax_force, ...
        t_i, ...
        F_i, ...
        'Color', colors(i,:), ...
        'LineWidth', line_width, ...
        'DisplayName', case_name);

    % Theta-r simulation trajectory
    plot(ax_phase, ...
        z_i(:,1)*180/pi, ...
        100*z_i(:,3), ...
        'Color', colors(i,:), ...
        'LineWidth', line_width, ...
        'DisplayName', case_name);

    %% Static equilibrium curve for the current m_L

    % Static equilibrium:
    %
    % r_eq = equilibrium_coefficient*tan(theta)
    %
    % where:
    %
    % equilibrium_coefficient
    %     = R + G/(m*g)
    %
    % moving_rod_mass = 2*m_L

    G_i = ...
        par_i.m_B*par_i.g*(par_i.R + par_i.h) ...
        + par_i.m_W*par_i.g*par_i.R;

    moving_rod_mass_i = 2*m_L_values(i);

    equilibrium_coefficient_i = ...
        (G_i ...
        + moving_rod_mass_i*par_i.g*par_i.R) ...
        /(moving_rod_mass_i*par_i.g);

    theta_eq_min = min(z_i(:,1));
    theta_eq_max = max(z_i(:,1));

    theta_eq_margin = max( ...
        0.1*(theta_eq_max-theta_eq_min), ...
        0.02*pi/180);

    theta_eq_grid = linspace( ...
        theta_eq_min-theta_eq_margin, ...
        theta_eq_max+theta_eq_margin, ...
        150);

    r_eq_grid = ...
        equilibrium_coefficient_i*tan(theta_eq_grid);

    plot(ax_phase, ...
        theta_eq_grid*180/pi, ...
        100*r_eq_grid, ...
        '--', ...
        'Color', colors(i,:), ...
        'LineWidth', 1.1, ...
        'HandleVisibility', 'off');
end

%% Figure legend and overall title

legend(ax_theta, ...
    'Location', 'best', ...
    'Interpreter', 'latex', ...
    'FontSize', 8);

title(layout, { ...
    'Rod-mass sweep with LQR redesigned for every mass', ...
    sprintf( ...
        '$Q=\\mathrm{diag}(1000,100,100,10),\\quad R=%g$', ...
        R_weight)}, ...
    'Interpreter', 'latex');

linkaxes( ...
    [ax_theta, ax_r, ax_theta_dot, ax_r_dot, ax_force], ...
    'x');

xlim(ax_theta, tspan);

%% 6. Print numerical trend table

mass_sweep_summary = table( ...
    mass_at_end_values(:), ...
    m_L_values(:), ...
    final_time, ...
    completed_full_simulation, ...
    peak_theta_deg, ...
    peak_r_cm, ...
    peak_theta_dot_degps, ...
    peak_r_dot_mps, ...
    peak_force_N, ...
    final_theta_deg, ...
    final_r_cm, ...
    K_all(:,1), ...
    K_all(:,2), ...
    K_all(:,3), ...
    K_all(:,4), ...
    max_real_closed_loop_pole, ...
    'VariableNames', { ...
        'mass_at_end_kg', ...
        'm_L_kg', ...
        'final_time_s', ...
        'completed', ...
        'peak_abs_theta_deg', ...
        'peak_abs_r_cm', ...
        'peak_abs_theta_dot_degps', ...
        'peak_abs_r_dot_mps', ...
        'peak_abs_force_N', ...
        'final_theta_deg', ...
        'final_r_cm', ...
        'K_theta', ...
        'K_theta_dot', ...
        'K_r', ...
        'K_r_dot', ...
        'max_real_closed_loop_pole'});

disp(' ');
disp('Rod-mass sweep summary:');
disp(mass_sweep_summary);

%% 7. Print closed-loop poles

disp(' ');
disp('Closed-loop poles for every rod-mass case:');

for i = 1:number_of_cases
    fprintf( ...
        '\nm_end = %.2f kg, m_L = %.2f kg\n', ...
        mass_at_end_values(i), ...
        m_L_values(i));

    disp(closed_loop_poles_all(i,:).');
end

%% Local model-linearization function

function [A, B] = linearize_lat_model_at_origin(par)
    % Analytical Jacobian of LatModel_SignCorrection at:
    %
    % z = [theta, theta_dot, r, r_dot] = 0
    % F = 0

    m_L = par.m_L;
    m_B = par.m_B;
    m_W = par.m_W;

    h = par.h;
    g = par.g;
    R = par.R;

    % Mass matrix at theta = 0 and r = 0.
    M0 = [
        2*m_L*R^2 ...
            + m_B*(R+h)^2 ...
            + m_W*R^2 ...
            + par.I_b ...
            + par.I_w ...
            + par.I_rod, ...
        -2*m_L*R;

        -2*m_L*R, ...
        2*m_L
    ];

    gravity_coefficient = ...
        2*m_L*g*R ...
        + m_B*g*(R+h) ...
        + m_W*g*R;

    % Columns correspond to [theta, r].
    rightside_position_jacobian = [
        gravity_coefficient, -2*m_L*g;
        -2*m_L*g,             0
    ];

    rightside_input_jacobian = [0; 1];

    acceleration_position_jacobian = ...
        M0\rightside_position_jacobian;

    acceleration_input_jacobian = ...
        M0\rightside_input_jacobian;

    % State order:
    % z = [theta; theta_dot; r; r_dot]
    A = zeros(4,4);

    A(1,2) = 1;
    A(3,4) = 1;

    A(2,[1,3]) = ...
        acceleration_position_jacobian(1,:);

    A(4,[1,3]) = ...
        acceleration_position_jacobian(2,:);

    B = zeros(4,1);

    B(2) = acceleration_input_jacobian(1);
    B(4) = acceleration_input_jacobian(2);
end

%% Local event function

function [value, isterminal, direction] = ...
    stop_if_out_of_range(~, z)

    theta_limit = 45*pi/180;
    theta_dot_limit = 30;
    r_limit = 1.0;
    r_dot_limit = 10.0;

    margins = [
        theta_limit-abs(z(1));
        theta_dot_limit-abs(z(2));
        r_limit-abs(z(3));
        r_dot_limit-abs(z(4))
    ];

    value = min(margins);
    isterminal = 1;
    direction = -1;
end