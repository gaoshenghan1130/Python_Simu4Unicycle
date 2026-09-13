clear; clc;
addpath("param/","model/");

%% =========================================================
% Poster plotting configuration
%% =========================================================
POSTER_MODE = true;
POSTER_SCALE = 1.5;

if POSTER_MODE
    AXIS_FONT_SIZE   = 18 * POSTER_SCALE;
    LABEL_FONT_SIZE  = 22 * POSTER_SCALE;
    TITLE_FONT_SIZE  = 24 * POSTER_SCALE;
    COLORBAR_FONT_SIZE = 18 * POSTER_SCALE;
    AXIS_LINE_WIDTH  = 1.8 * POSTER_SCALE;
else
    AXIS_FONT_SIZE   = 11;
    LABEL_FONT_SIZE  = 13;
    TITLE_FONT_SIZE  = 14;
    COLORBAR_FONT_SIZE = 11;
    AXIS_LINE_WIDTH  = 1.0;
end

%% =========================================================
% Build linear model
%% =========================================================
par = LatParam();

m_L = par.m_L;
m_B = par.m_B;
m_W = par.m_W;
h   = par.h;
R   = par.R;
g   = par.g;

J_theta  = 2*m_L*R^2 + m_B*(R+h)^2 + m_W*R^2 ...
         + par.I_b + par.I_rod + par.I_w;

G_theta1 = 2*m_L*g*R + m_B*g*(R+h) + m_W*g*R;
G_theta2 = -2*m_L*g;
G_r1     = -2*m_L*g;

N = [1, 0, 0, 0;
     0, 1, 0, 0;
     0, 0, J_theta, -2*m_L*R;
     0, 0, -2*m_L*R, 2*m_L];

A_tilde = [0,        0,      1, 0;
           0,        0,      0, 1;
           G_theta1, G_r1,   0, 0;
           G_theta2, 0,      0, 0];

A = N \ A_tilde;
B = N \ [0; 0; 0; 1];

%% =========================================================
% Parameter sweep: poles defined by alpha and epsilon
%
% lambda1 = -1.75 - epsilon
% lambda2 = -1.75 + epsilon
% lambda3 = -1.75 - alpha*epsilon
% lambda4 = -1.75 + alpha*epsilon
%
% Feasibility condition:
% epsilon <= 0.5
% alpha*epsilon <= 0.5
%% =========================================================
alpha_vals   = linspace(0.05, 1.20, 120);
epsilon_vals = linspace(0.005, 0.50, 120);

CondV_map = nan(length(epsilon_vals), length(alpha_vals));
Valid_map = false(length(epsilon_vals), length(alpha_vals));

center_pole = -1.75;
pole_min = -2.25;
pole_max = -1.25;

for i = 1:length(epsilon_vals)
    eps_val = epsilon_vals(i);

    for j = 1:length(alpha_vals)
        alpha = alpha_vals(j);

        P_desired = [ ...
            center_pole - eps_val, ...
            center_pole + eps_val, ...
            center_pole - alpha*eps_val, ...
            center_pole + alpha*eps_val ];

        % Check whether all poles stay inside the feasible interval
        if any(P_desired < pole_min) || any(P_desired > pole_max)
            continue;
        end

        % Skip nearly repeated poles to avoid numerical issues
        if min(abs(diff(sort(P_desired)))) < 1e-6
            continue;
        end

        try
            K = place(A, B, P_desired);
            A_cl = A - B*K;

            [V, ~] = eig(A_cl);

            CondV_map(i, j) = cond(V);
            Valid_map(i, j) = true;

        catch
            CondV_map(i, j) = NaN;
            Valid_map(i, j) = false;
        end
    end
end

%% =========================================================
% Plot heatmap
%% =========================================================
figure('Position', [100, 100, 1000, 750]);

imagesc(alpha_vals, epsilon_vals, CondV_map);

ax1 = gca;
set(ax1, ...
    'YDir', 'normal', ...
    'ColorScale', 'log', ...
    'FontSize', AXIS_FONT_SIZE, ...
    'LineWidth', AXIS_LINE_WIDTH, ...
    'TickDir', 'out');

xlabel('\alpha', ...
    'FontSize', LABEL_FONT_SIZE, ...
    'FontWeight', 'bold');

ylabel('\epsilon', ...
    'FontSize', LABEL_FONT_SIZE, ...
    'FontWeight', 'bold');

title('Heatmap of cond(V) over (\alpha,\epsilon)', ...
    'FontSize', TITLE_FONT_SIZE, ...
    'FontWeight', 'bold');

cb1 = colorbar;
cb1.Label.String = 'cond(V)';
cb1.Label.FontSize = LABEL_FONT_SIZE;
cb1.Label.FontWeight = 'bold';
cb1.FontSize = COLORBAR_FONT_SIZE;
cb1.LineWidth = AXIS_LINE_WIDTH;

grid on;
box on;

%% Overlay invalid region
hold on;

[row_invalid, col_invalid] = find(~Valid_map);

if ~isempty(row_invalid)
    plot( ...
        alpha_vals(col_invalid), ...
        epsilon_vals(row_invalid), ...
        'k.', ...
        'MarkerSize', 5 * POSTER_SCALE);
end

hold off;

%% =========================================================
% Find best / worst valid points
%% =========================================================
CondV_valid = CondV_map;
CondV_valid(~Valid_map) = NaN;

[min_condV, idx_min] = min(CondV_valid(:), [], 'omitnan');
[max_condV, idx_max] = max(CondV_valid(:), [], 'omitnan');

[i_min, j_min] = ind2sub(size(CondV_valid), idx_min);
[i_max, j_max] = ind2sub(size(CondV_valid), idx_max);

fprintf( ...
    'Minimum cond(V) = %.6e at alpha = %.6f, epsilon = %.6f\n', ...
    min_condV, ...
    alpha_vals(j_min), ...
    epsilon_vals(i_min));

fprintf( ...
    'Maximum cond(V) = %.6e at alpha = %.6f, epsilon = %.6f\n', ...
    max_condV, ...
    alpha_vals(j_max), ...
    epsilon_vals(i_max));

%% =========================================================
% Contour plot
%% =========================================================
figure('Position', [150, 150, 1000, 750]);

contourf( ...
    alpha_vals, ...
    epsilon_vals, ...
    log10(CondV_map), ...
    30, ...
    'LineColor', 'none');

ax2 = gca;
set(ax2, ...
    'FontSize', AXIS_FONT_SIZE, ...
    'LineWidth', AXIS_LINE_WIDTH, ...
    'TickDir', 'out');

xlabel('\alpha', ...
    'FontSize', LABEL_FONT_SIZE, ...
    'FontWeight', 'bold');

ylabel('\epsilon', ...
    'FontSize', LABEL_FONT_SIZE, ...
    'FontWeight', 'bold');

title('Contour of log_{10}(cond(V)) over (\alpha,\epsilon)', ...
    'FontSize', TITLE_FONT_SIZE, ...
    'FontWeight', 'bold');

cb2 = colorbar;
cb2.Label.String = 'log_{10}(cond(V))';
cb2.Label.FontSize = LABEL_FONT_SIZE;
cb2.Label.FontWeight = 'bold';
cb2.FontSize = COLORBAR_FONT_SIZE;
cb2.LineWidth = AXIS_LINE_WIDTH;

grid on;
box on;

%% =========================================================
% Optional high-resolution export
%% =========================================================
% exportgraphics(figure(1), ...
%     'CondV_heatmap.pdf', ...
%     'ContentType', 'vector');
%
% exportgraphics(figure(2), ...
%     'CondV_contour.pdf', ...
%     'ContentType', 'vector');
%
% exportgraphics(figure(1), ...
%     'CondV_heatmap.png', ...
%     'Resolution', 600);
%
% exportgraphics(figure(2), ...
%     'CondV_contour.png', ...
%     'Resolution', 600);