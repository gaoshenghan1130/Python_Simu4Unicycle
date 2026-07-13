clear; clc;
addpath("param/","model/");

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

J_theta  = 2*m_L*R^2 + m_B*(R+h)^2 + m_W*R^2 + par.I_b + par.I_rod + par.I_w;
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
B = N \ [0;0;0;1];

%% =========================================================
% Parameter sweep: poles defined by alpha and epsilon
%
% lambda1 = -1.75 - epsilon
% lambda2 = -1.75 + epsilon
% lambda3 = -1.75 - alpha*epsilon
% lambda4 = -1.75 + alpha*epsilon
%
% Feasibility condition if you want poles inside [-2.25, -1.25]:
% epsilon <= 0.5
% alpha*epsilon <= 0.5
%% =========================================================
alpha_vals   = linspace(0.05, 1.20, 120);   % adjust as needed
epsilon_vals = linspace(0.005, 0.50, 120);  % adjust as needed

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

        % Optional: skip nearly repeated poles to avoid numerical issues
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
figure;
imagesc(alpha_vals, epsilon_vals, CondV_map);
set(gca, 'YDir', 'normal');
set(gca, 'ColorScale', 'log');   % recommended because cond(V) can vary a lot
xlabel('\alpha');
ylabel('\epsilon');
title('Heatmap of cond(V) over (\alpha,\epsilon)');
cb = colorbar;
cb.Label.String = 'cond(V)';
grid on;

%% Optional: overlay invalid region
hold on;
[row_invalid, col_invalid] = find(~Valid_map);
if ~isempty(row_invalid)
    plot(alpha_vals(col_invalid), epsilon_vals(row_invalid), 'k.', 'MarkerSize', 4);
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

fprintf('Minimum cond(V) = %.6e at alpha = %.6f, epsilon = %.6f\n', ...
    min_condV, alpha_vals(j_min), epsilon_vals(i_min));

fprintf('Maximum cond(V) = %.6e at alpha = %.6f, epsilon = %.6f\n', ...
    max_condV, alpha_vals(j_max), epsilon_vals(i_max));

%% =========================================================
% Optional: contour plot
%% =========================================================
figure;
contourf(alpha_vals, epsilon_vals, log10(CondV_map), 30, 'LineColor', 'none');
xlabel('\alpha');
ylabel('\epsilon');
title('Contour of log_{10}(cond(V)) over (\alpha,\epsilon)');
cb = colorbar;
cb.Label.String = 'log_{10}(cond(V))';
grid on;