clear; clc; close all;

addpath('simulation/', 'param/', 'model/');

% Target number of points strictly within the linear stable region
N_target = 1000; 

% =========================================================================
% 1. Global search region definition (adjust bounds as needed)
% =========================================================================
k1_min = -100; k1_max = 1000;
k2_min = -100; k2_max = 1000;
k3_min = -100; k3_max = 1000;
k4_min = -100; k4_max = 1000;

% =========================================================================
% 2. Monte Carlo Rejection Sampling for Linear Stability (Routh-Hurwitz)
% =========================================================================
disp(['Generating ', num2str(N_target), ' Monte Carlo samples strictly within the RH stable region...']);

k1_final = []; k2_final = []; k3_final = []; k4_final = [];
batch_size = 50000; % Generate in large batches to speed up valid point accumulation

while length(k1_final) < N_target
    % Generate random batch
    k1_batch = k1_min + (k1_max - k1_min) * rand(batch_size, 1);       
    k2_batch = k2_min + (k2_max - k2_min) * rand(batch_size, 1);       
    k3_batch = k3_min + (k3_max - k3_min) * rand(batch_size, 1);         
    k4_batch = k4_min + (k4_max - k4_min) * rand(batch_size, 1);         

    % Routh-Hurwitz coefficients
    a3 = 0.5924 .* k4_batch - 0.06548 .* k3_batch;
    a2 = 0.5924 .* k2_batch - 0.06548 .* k1_batch - 26.2 ;
    a1 = 10.06 .* k3_batch - 16.7 .* k4_batch;
    a0 = 10.06 .* k1_batch - 16.7 .* k2_batch - 167.8;

    % Linear stability requires: all coefficients > 0, and cross-multiplication conditions met
    cond_pos = (a3 > 0) & (a2 > 0) & (a1 > 0) & (a0 > 0);
    cond1 = (a3 .* a2) > a1;
    cond2 = (a3 .* a2 .* a1) > (a1.^2 + (a3.^2) .* a0);

    % Filter valid points in current batch
    valid_idx = cond_pos & cond1 & cond2;
    
    % Append to final arrays
    k1_final = [k1_final; k1_batch(valid_idx)];
    k2_final = [k2_final; k2_batch(valid_idx)];
    k3_final = [k3_final; k3_batch(valid_idx)];
    k4_final = [k4_final; k4_batch(valid_idx)];
end

% Truncate exactly to the target number of samples
k1_final = k1_final(1:N_target);
k2_final = k2_final(1:N_target);
k3_final = k3_final(1:N_target);
k4_final = k4_final(1:N_target);

disp('Successfully collected required linearly stable points.');

% =========================================================================
% 3. Mark the original "old linear constrained space" for comparison
% =========================================================================
is_old_region_final = (k2_final < (k1_final - 16.68) / 1.66) & ...
                      (k2_final > 0.1105 .* k1_final + 44.23) & ...
                      (k3_final > 1.66 .* k4_final) & ...
                      (k4_final > 0.1105 * k3_final) & ...
                      (k1_final <= 316.43);

disp(['Number of points within the old constraint region: ', num2str(sum(is_old_region_final))]);

% % =========================================================================
% % 4. Plotting Linear Stability Bounds
% % =========================================================================
% figure('Position',[100 100 1200 500]);
% 
% % --- k1-k2 Subplot ---
% subplot(1,2,1); hold on;
% scatter(k1_final, k2_final, 3, 'filled', 'MarkerFaceAlpha', 0.2);
% k1_line = linspace(k1_min, k1_max, 1000);
% plot(k1_line, (0.06548*k1_line + 26.2)/0.5924, 'r', 'LineWidth', 2);
% plot(k1_line, (10.06*k1_line - 167.8)/16.7, 'b', 'LineWidth', 2);
% xlabel('k_1'); ylabel('k_2');
% title('RH Stable Region');
% legend('RH samples', 'a_2=0', 'a_0=0');
% grid on; hold off;
% 
% % --- k3-k4 Subplot ---
% subplot(1,2,2); hold on;
% scatter(k3_final, k4_final, 3, 'filled', 'MarkerFaceAlpha', 0.2);
% k3_line = linspace(k3_min, k3_max, 1000);
% plot(k3_line, (0.06548/0.5924)*k3_line, 'r', 'LineWidth', 2);
% plot(k3_line, (10.06/16.7)*k3_line, 'b', 'LineWidth', 2);
% xlabel('k_3'); ylabel('k_4');
% title('RH Stable Region');
% legend('RH samples', 'a_3=0', 'a_1=0');
% grid on; hold off;

% =========================================================================
% 5. Nonlinear model verification 
% =========================================================================
disp('Starting nonlinear model verification...');

% We now test all of our generated linear stable points (up to a limit)
num_test = min(10000, length(k1_final)); 

% Randomize the testing order
rand_indices = randperm(length(k1_final), num_test);

k1_test = -k1_final(rand_indices); % Sign inversion per original control law
k2_test = -k2_final(rand_indices);
k3_test =  k3_final(rand_indices);
k4_test =  k4_final(rand_indices);

% Record whether the tested point falls within the old bounds
is_old_region_test = is_old_region_final(rand_indices); 

nonlinear_stable_idx = false(num_test, 1);
z0 = [0.005 * pi/180; 0; 0; 0]; % Initial state
tspan = [0, 10];
par = LatParam();

for i = 1:num_test
    K = [k1_test(i), k3_test(i), k2_test(i), k4_test(i)];
    controller = @(t, z, par) -K * z;
    
    try
        [t_out, z_out] = ode45(@(t, z) LatModel_SignCorrection(t, z, par, controller), tspan, z0);
        
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
    
    if mod(i, 500) == 0 % Print progress interval
        fprintf('Completed nonlinear verification: %d / %d\n', i, num_test);
    end
end

k1_nl_stable = k1_test(nonlinear_stable_idx);
k2_nl_stable = k2_test(nonlinear_stable_idx);
k3_nl_stable = k3_test(nonlinear_stable_idx);
k4_nl_stable = k4_test(nonlinear_stable_idx);

disp(['Total nonlinear stable points: ', num2str(length(k1_nl_stable))]);

% =========================================================================
% 6. Final Plotting: Compare global results with original boundaries
% =========================================================================
figure('Position', [100, 100, 1200, 500]);

% --- Subplot 1: k1 vs k2 ---
subplot(1, 2, 1); hold on;
k1_line = linspace(k1_min, k1_max, 200);

% Plot original linear boundary limits (dashed lines)
plot(k1_line, (k1_line - 16.68)/1.66, 'k--', 'LineWidth', 1.2, 'DisplayName', 'Old Boundary 1');
plot(k1_line, 0.1105*k1_line + 44.23, 'k--', 'LineWidth', 1.2, 'DisplayName', 'Old Boundary 2');
xline(316.43, 'k--', 'LineWidth', 1.2, 'DisplayName', 'Old k1 Max (316.43)');

% Scatter globally tested stable points (transparent gray)
scatter(k1_test(~is_old_region_test), k2_test(~is_old_region_test), 6, [0.6 0.6 0.6], 'filled', 'MarkerFaceAlpha', 0.2, 'DisplayName', 'Linear Stable (Global)');

% Scatter tested points mapping to the old linear constraint region (transparent red)
scatter(k1_test(is_old_region_test), k2_test(is_old_region_test), 8, 'r', 'filled', 'MarkerFaceAlpha', 0.3, 'DisplayName', 'Old Linear Region Feasible');

% Overlay final nonlinearly verified stable points (green)
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

% Plot original linear boundary limits (dashed lines)
plot(k3_line, k3_line/1.66, 'k--', 'LineWidth', 1.2, 'DisplayName', 'Old Boundary 1');
plot(k3_line, 0.1105*k3_line, 'k--', 'LineWidth', 1.2, 'DisplayName', 'Old Boundary 2');

% Scatter globally tested stable points (transparent gray)
scatter(k3_test(~is_old_region_test), k4_test(~is_old_region_test), 6, [0.6 0.6 0.6], 'filled', 'MarkerFaceAlpha', 0.2, 'DisplayName', 'Linear Stable (Global)');

% Scatter tested points mapping to the old linear constraint region (transparent blue)
scatter(k3_test(is_old_region_test), k4_test(is_old_region_test), 8, 'b', 'filled', 'MarkerFaceAlpha', 0.3, 'DisplayName', 'Old Linear Region Feasible');

% Overlay final nonlinearly verified stable points (magenta)
if ~isempty(k3_nl_stable)
    scatter(k3_nl_stable, k4_nl_stable, 25, 'm', 'filled', 'DisplayName', 'Nonlinear Verified Stable');
end

xlim([k3_min, k3_max]); ylim([k4_min, k4_max]);
xlabel('k_3'); ylabel('k_4');
title('k_3 vs k_4 (Global Search & Old Bounds)');
legend('Location', 'best'); grid on; hold off;