clear; clc; close all;
addpath('simulation/', 'param/', 'model/');

N = 1000000; % Increased N since we are searching a much larger global volume

% =========================================================================
% 1. Global search region definition (adjust bounds as needed)
% =========================================================================
k1_min = -100; k1_max = 1000;
k2_min = -100; k2_max = 1000;
k3_min = -100; k3_max = 1000;
k4_min = -100; k4_max = 1000;

k1_rand = k1_min + (k1_max - k1_min) * rand(N, 1);       
k2_rand = k2_min + (k2_max - k2_min) * rand(N, 1);       
k3_rand = k3_min + (k3_max - k3_min) * rand(N, 1);         
k4_rand = k4_min + (k4_max - k4_min) * rand(N, 1);         

% =========================================================================
% 2. Mark the original "old linear constrained space" (for plotting comparison)
% =========================================================================
old_linear_mask = (k2_rand < (k1_rand - 16.68) / 1.66) & ...
                  (k2_rand > 0.1105 .* k1_rand + 44.23) & ...
                  (k3_rand > 1.66 .* k4_rand) & ...
                  (k4_rand > 0.1105 * k3_rand) & ...
                  (k1_rand <= 316.43); % Original k1 upper bound

% =========================================================================
% 3. Global Routh-Hurwitz stability criterion calculation
% =========================================================================
a3 = 0.5924 .* k4_rand - 0.06548 .* k3_rand;
a2 = 0.5924 .* k2_rand - 0.06548 .* k1_rand - 26.2 ;
a1 = 10.06 .* k3_rand - 16.7 .* k4_rand;
a0 = 10.06 .* k1_rand - 16.7 .* k2_rand - 167.8;

% Linear stability requires: all coefficients > 0, and cross-multiplication conditions met
cond_pos = (a3 > 0) & (a2 > 0) & (a1 > 0) & (a0 > 0);
cond1 = (a3 .* a2) > a1;
cond2 = (a3 .* a2 .* a1) > (a1.^2 + (a3.^2) .* a0);

% Final points satisfying linear stability in the entire space
final_valid = cond_pos & cond1 & cond2;

k1_final = k1_rand(final_valid);
k2_final = k2_rand(final_valid);
k3_final = k3_rand(final_valid);
k4_final = k4_rand(final_valid);
is_old_region_final = old_linear_mask(final_valid); % Mark which points are within the old region

disp(['Num linearly stable points found globally: ', num2str(length(k1_final))]);
disp(['Num of those within the old constraint region: ', num2str(sum(is_old_region_final))]);

% =========================================================================
% 4. Nonlinear model verification (sample from globally stable points)
% =========================================================================
disp('Start nonlinear model verification...');
num_test = min(10000, length(k1_final)); 

if num_test == 0
    disp('No points passed the linear stability test. Please expand k_min/k_max bounds.');
    return;
end

rand_indices = randperm(length(k1_final), num_test);
k1_test = k1_final(rand_indices);
k2_test = k2_final(rand_indices);
k3_test = k3_final(rand_indices);
k4_test = k4_final(rand_indices);
is_old_region_test = is_old_region_final(rand_indices); % Record whether the test point belongs to the old region

nonlinear_stable_idx = false(num_test, 1);
z0 = [0.02 * pi/180; 0; 0; 0]; % Initial state
tspan = [0, 15];
par = LatParam();

for i = 1:num_test
    K = [k1_test(i), k3_test(i), k2_test(i), k4_test(i)];
    controller = @(t, z, par) -K * z;
    
    try
        [t_out, z_out] = ode45(@(t, z) LatModel(t, z, par, controller), tspan, z0);
        
        theta_out = z_out(:, 1);
        r_out = z_out(:, 3);
        max_theta = max(abs(theta_out));  
        final_theta = abs(theta_out(end));
        max_r = max(abs(r_out));
        
        if (max_theta < 20 * pi/180) && (final_theta < 5 * pi/180) 
            nonlinear_stable_idx(i) = true;
        end
    catch E
        nonlinear_stable_idx(i) = false;
    end
    
    if mod(i, 500) == 0 % Increase log interval to avoid terminal spam
        fprintf('Completed nonlinear verification: %d / %d\n', i, num_test);
    end
end

k1_nl_stable = k1_test(nonlinear_stable_idx);
k2_nl_stable = k2_test(nonlinear_stable_idx);
k3_nl_stable = k3_test(nonlinear_stable_idx);
k4_nl_stable = k4_test(nonlinear_stable_idx);

disp(['Total nonlinear stable points: ', num2str(length(k1_nl_stable))]);

% =========================================================================
% 5. Plotting: Merge global search results with original Routh-Hurwitz boundaries
% =========================================================================
figure('Position', [100, 100, 1200, 500]);

% --- Subplot 1: k1 vs k2 ---
subplot(1, 2, 1); hold on;
k1_line = linspace(k1_min, k1_max, 200);

% Restore and plot the original linear boundary dashed lines
plot(k1_line, (k1_line - 16.68)/1.66, 'k--', 'LineWidth', 1.2, 'DisplayName', 'Old Boundary 1');
plot(k1_line, 0.1105*k1_line + 44.23, 'k--', 'LineWidth', 1.2, 'DisplayName', 'Old Boundary 2');
xline(316.43, 'k--', 'LineWidth', 1.2, 'DisplayName', 'Old k1 Max (316.43)');

% Plot global linearly stable points (transparent gray)
scatter(k1_test(~is_old_region_test), k2_test(~is_old_region_test), 6, [0.6 0.6 0.6], 'filled', 'MarkerFaceAlpha', 0.2, 'DisplayName', 'Linear Stable (Global)');
% Plot stable points that fall within the old linear constraint space (transparent red)
scatter(k1_test(is_old_region_test), k2_test(is_old_region_test), 8, 'r', 'filled', 'MarkerFaceAlpha', 0.3, 'DisplayName', 'Old Linear Region Feasible');

% Plot the final nonlinearly stable points (green)
if ~isempty(k1_nl_stable)
    scatter(k1_nl_stable, k2_nl_stable, 25, 'g', 'filled', 'DisplayName', 'Nonlinear Verified Stable');
end

xlim([k1_min, k1_max]); ylim([k2_min, k2_max]);
xlabel('k_1'); ylabel('k_2');
title('k_1 vs k_2 (Global Search & Old Bounds)');
legend('Location', 'best'); grid on; hold off;

% --- Subplot 2: k3 vs k4 ---
subplot(1, 2, 2); hold on;
k3_line = linspace(k3_min, k3_max, 200);

% Restore and plot the original linear boundary dashed lines
plot(k3_line, k3_line/1.66, 'k--', 'LineWidth', 1.2, 'DisplayName', 'Old Boundary 1');
plot(k3_line, 0.1105*k3_line, 'k--', 'LineWidth', 1.2, 'DisplayName', 'Old Boundary 2');

% Plot global linearly stable points (transparent gray)
scatter(k3_test(~is_old_region_test), k4_test(~is_old_region_test), 6, [0.6 0.6 0.6], 'filled', 'MarkerFaceAlpha', 0.2, 'DisplayName', 'Linear Stable (Global)');
% Plot stable points that fall within the old linear constraint space (transparent blue)
scatter(k3_test(is_old_region_test), k4_test(is_old_region_test), 8, 'b', 'filled', 'MarkerFaceAlpha', 0.3, 'DisplayName', 'Old Linear Region Feasible');

% Plot the final nonlinearly stable points (magenta)
if ~isempty(k3_nl_stable)
    scatter(k3_nl_stable, k4_nl_stable, 25, 'm', 'filled', 'DisplayName', 'Nonlinear Verified Stable');
end

xlim([k3_min, k3_max]); ylim([k4_min, k4_max]);
xlabel('k_3'); ylabel('k_4');
title('k_3 vs k_4 (Global Search & Old Bounds)');
legend('Location', 'best'); grid on; hold off;