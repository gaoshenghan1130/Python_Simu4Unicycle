%% Nonzero closed-loop equilibria under linear state feedback
% f(theta) = K_theta*theta + K_r*a_eq*tan(theta) + m_rod*g*sin(theta)
% A positive root theta in (0, pi/2) gives r = a_eq*tan(theta).

clear; clc; close all;

p = LatParam();

%% Parameters and feedback gains: edit these values
m_rod = 2.3;                 % total translating rod mass [kg]
g     = 9.81;                % gravity [m/s^2]
R     = 0.2527;              % wheel radius [m]
h     = p.h;
G     = p.m_B*(R + h)*g + p.m_W*R*g;

K_theta  = -800;             % [N/rad]
K_r      = 850;              % [N/m]
K_sigma1 = 0;                % does not change static equilibria
K_sigma2 = 0;                % does not change static equilibria

theta_max_deg = 30;          % plotted equilibrium-angle range

%% Equilibrium functions
a_eq = (G + m_rod*g*R)/(m_rod*g);
f  = @(theta) K_theta.*theta + K_r*a_eq.*tan(theta) + m_rod*g.*sin(theta);
df = @(theta) K_theta + K_r*a_eq.*sec(theta).^2 + m_rod*g.*cos(theta);

theta = linspace(0, deg2rad(theta_max_deg), 20001);
f_values  = f(theta);
df_values = df(theta);

% Find the known root at theta=0 plus every nonzero root from sign changes.
theta_roots = findRootsOnGrid(f, theta);
theta_roots = theta_roots(theta_roots > 1e-8);

% Locations where f'(theta)=0, i.e., extrema of f(theta).
theta_turning = findRootsOnGrid(df, theta);
theta_turning = theta_turning(theta_turning > 1e-8);

%% Report equilibria
fprintf('a_eq = %.8f m\n', a_eq);
fprintf('f''(0) = %.8f\n', df(0));
fprintf('Derivative gains: K_sigma1 = %.6g, K_sigma2 = %.6g (no static effect)\n', ...
    K_sigma1, K_sigma2);

if isempty(theta_turning)
    fprintf('No stationary point of f(theta) in (0, %.1f deg).\n', theta_max_deg);
else
    fprintf('\nStationary points of f(theta):\n');
    for i = 1:numel(theta_turning)
        fprintf(' theta_p = %8.4f deg, f(theta_p) = %+12.6f\n', ...
            rad2deg(theta_turning(i)), f(theta_turning(i)));
    end
end

if isempty(theta_roots)
    fprintf('\nNo nonzero positive equilibrium found in the plotted interval.\n');
else
    fprintf('\nNonzero positive closed-loop equilibria:\n');
    fprintf(' theta* [deg]       r* [m]          F* [N]\n');
    for i = 1:numel(theta_roots)
        th = theta_roots(i);
        fprintf(' %11.6f   %+12.7f   %+12.7f\n', rad2deg(th), ...
            a_eq*tan(th), m_rod*g*sin(th));
    end
end

%% Plot f and f'
figure('Color', 'w', 'Position', [100 100 900 700]);
tiledlayout(2, 1, 'TileSpacing', 'compact');

nexttile; hold on; grid on; box on;
plot(rad2deg(theta), f_values, 'b-', 'LineWidth', 1.8, 'DisplayName', 'f(\theta)');
yline(0, 'k-', 'LineWidth', 1.0, 'DisplayName', 'f(\theta)=0');
if ~isempty(theta_turning)
    plot(rad2deg(theta_turning), f(theta_turning), 'mo', ...
        'MarkerFaceColor', 'm', 'DisplayName', 'f''(\theta_p)=0');
end
if ~isempty(theta_roots)
    plot(rad2deg(theta_roots), zeros(size(theta_roots)), 'ro', ...
        'MarkerFaceColor', 'r', 'DisplayName', 'nonzero equilibrium');
end
xlabel('\theta (deg)', 'Interpreter', 'tex');
ylabel('f(\theta)', 'Interpreter', 'tex');
title('Closed-loop static-equilibrium equation', 'Interpreter', 'tex');
legend('Location', 'best', 'Interpreter', 'tex');

nexttile; hold on; grid on; box on;
plot(rad2deg(theta), df_values, 'Color', [0.1 0.55 0.15], ...
    'LineWidth', 1.8, 'DisplayName', 'df/d\theta');
yline(0, 'k-', 'LineWidth', 1.0, 'DisplayName', 'df/d\theta=0');
if ~isempty(theta_turning)
    plot(rad2deg(theta_turning), zeros(size(theta_turning)), 'mo', ...
        'MarkerFaceColor', 'm', 'DisplayName', 'stationary point');
end
xlabel('\theta (deg)', 'Interpreter', 'tex');
ylabel('df/d\theta', 'Interpreter', 'tex');
title('Derivative used to identify extrema of f(\theta)', 'Interpreter', 'tex');
legend('Location', 'best', 'Interpreter', 'tex');

%% Critical gain boundary for different theta_p
% At the boundary where a nonzero equilibrium root is created or removed,
% f(theta_p)=0 and f'(theta_p)=0.  For each theta_p, solve
%
% [theta_p, a_eq*tan(theta_p)  ] [K_theta] = -m_rod*g*[sin(theta_p)]
% [1,       a_eq*sec(theta_p)^2] [K_r    ]             [cos(theta_p)]

theta_p_deg = linspace(0.1, theta_max_deg, 1500);
theta_p = deg2rad(theta_p_deg);

K_theta_critical = nan(size(theta_p));
K_r_critical     = nan(size(theta_p));

for i = 1:numel(theta_p)
    th = theta_p(i);

    gain_matrix = [th, a_eq*tan(th); ...
                   1,  a_eq*sec(th)^2];
    gain_rhs = -m_rod*g*[sin(th); cos(th)];

    critical_gain = gain_matrix\gain_rhs;
    K_theta_critical(i) = critical_gain(1);
    K_r_critical(i)     = critical_gain(2);
end

% Angles shown explicitly on the critical curve.
theta_mark_deg = [1, 2, 5, 10, 15, 20, 25, 30];
theta_mark_deg = theta_mark_deg(theta_mark_deg <= theta_max_deg);
theta_mark = deg2rad(theta_mark_deg);

den_mark = theta_mark.*sec(theta_mark).^2 - tan(theta_mark);
K_r_mark = m_rod*g/a_eq .* ...
    (sin(theta_mark) - theta_mark.*cos(theta_mark)) ./ den_mark;
K_theta_mark = -K_r_mark*a_eq.*sec(theta_mark).^2 ...
    - m_rod*g*cos(theta_mark);

% Check that the calculated gains satisfy both original equations.
critical_residual_1 = K_theta_critical.*theta_p ...
    + K_r_critical*a_eq.*tan(theta_p) ...
    + m_rod*g*sin(theta_p);
critical_residual_2 = K_theta_critical ...
    + K_r_critical*a_eq.*sec(theta_p).^2 ...
    + m_rod*g*cos(theta_p);

fprintf('\nCritical-gain curve residuals:\n');
fprintf(' max |f(theta_p)|  = %.3e\n', max(abs(critical_residual_1)));
fprintf(' max |f''(theta_p)| = %.3e\n', max(abs(critical_residual_2)));

fprintf('\nCritical gains for selected theta_p values:\n');
fprintf(' theta_p [deg]       K_theta             K_r\n');
for i = 1:numel(theta_mark_deg)
    fprintf(' %12.2f   %+14.7f   %+14.7f\n', ...
        theta_mark_deg(i), K_theta_mark(i), K_r_mark(i));
end

%% Plot critical gains versus theta_p
figure('Color', 'w', 'Position', [160 100 1100 720]);
tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

% Parametric critical boundary in the (K_r, K_theta) plane.
nexttile([2 1]); hold on; grid on; box on;
scatter(K_r_critical, K_theta_critical, 18, theta_p_deg, 'filled', ...
    'DisplayName', 'critical boundary');
plot(K_r_mark, K_theta_mark, 'ko', 'MarkerFaceColor', 'w', ...
    'LineWidth', 1.0, 'DisplayName', 'selected \theta_p');
plot(K_r, K_theta, 'rp', 'MarkerFaceColor', 'r', 'MarkerSize', 13, ...
    'DisplayName', 'current gains');

for i = 1:numel(theta_mark_deg)
    text(K_r_mark(i), K_theta_mark(i), ...
        sprintf('  %.0f^o', theta_mark_deg(i)), ...
        'FontSize', 9, 'VerticalAlignment', 'bottom');
end

xlabel('K_r', 'Interpreter', 'tex');
ylabel('K_\theta', 'Interpreter', 'tex');
title('f(\theta_p)=0 and f''(\theta_p)=0', 'Interpreter', 'tex');
legend('Location', 'best', 'Interpreter', 'tex');
cb = colorbar;
cb.Label.String = '\theta_p (deg)';
cb.Label.Interpreter = 'tex';

% K_theta on the critical boundary.
nexttile; hold on; grid on; box on;
plot(theta_p_deg, K_theta_critical, 'b-', 'LineWidth', 1.8);
plot(theta_mark_deg, K_theta_mark, 'ko', 'MarkerFaceColor', 'w');
xlabel('\theta_p (deg)', 'Interpreter', 'tex');
ylabel('K_\theta', 'Interpreter', 'tex');
title('Critical K_\theta', 'Interpreter', 'tex');

% K_r on the critical boundary.
nexttile; hold on; grid on; box on;
plot(theta_p_deg, K_r_critical, 'r-', 'LineWidth', 1.8);
plot(theta_mark_deg, K_r_mark, 'ko', 'MarkerFaceColor', 'w');
xlabel('\theta_p (deg)', 'Interpreter', 'tex');
ylabel('K_r', 'Interpreter', 'tex');
title('Critical K_r', 'Interpreter', 'tex');

%% Local function
function roots_out = findRootsOnGrid(fun, x_grid)
    y = fun(x_grid);
    bracket_idx = find(y(1:end-1).*y(2:end) < 0);
    roots_out = zeros(size(bracket_idx));
    for k = 1:numel(bracket_idx)
        interval = x_grid(bracket_idx(k):bracket_idx(k)+1);
        roots_out(k) = fzero(fun, interval);
    end
    roots_out = unique(round(roots_out, 12));
end
