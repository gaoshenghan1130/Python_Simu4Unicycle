clear; clc; close all;

addpath('simulation/', 'param/', 'model/');

N = 500000; 
k1_rand = 316.43 * rand(N, 1);       
k2_rand = 190 * rand(N, 1);       
k3_rand = 100 * rand(N, 1);         
k4_rand = 50 * rand(N, 1);          

valid_linear = (k2_rand < (k1_rand - 16.68) / 1.66) & ...
               (k2_rand > 0.1105 .* k1_rand + 44.23) & ...
               (k3_rand > 1.66 .* k4_rand) & ...
               (k4_rand > 0.1105 * k3_rand);

k1_lin = k1_rand(valid_linear);
k2_lin = k2_rand(valid_linear);
k3_lin = k3_rand(valid_linear);
k4_lin = k4_rand(valid_linear);

a3 = 0.5924 .* k4_lin - 0.06548 .* k3_lin;
a2 = 0.5924 .* k2_lin - 0.06548 .* k1_lin - 26.2 ;
a1 = 10.06 .* k3_lin - 16.7 .* k4_lin;
a0 = 10.06 .* k1_lin - 16.7 .* k2_lin - 167.8;

cond1 = (a3 .* a2) > a1;
cond2 = (a3 .* a2 .* a1) > (a1.^2 + (a3.^2) .* a0);

final_valid = cond1 & cond2;

k1_final = k1_lin(final_valid);
k2_final = k2_lin(final_valid);
k3_final = k3_lin(final_valid);
k4_final = k4_lin(final_valid);
% 
% figure('Position', [100, 100, 1000, 400]);
% 
% subplot(1, 2, 1); hold on;

% k1_line = linspace(0, 350, 100);
% plot(k1_line, (k1_line - 16.68)/1.66, 'k--', 'LineWidth', 1);
% plot(k1_line, 0.1105*k1_line + 44.23, 'k--', 'LineWidth', 1);
% xline(316.43, 'k--');
% 
% scatter(k1_final, k2_final, 5, 'r', 'filled', 'MarkerFaceAlpha', 0.1);
% xlim([50, 350]); ylim([40, 180]);
% xlabel('k_1'); ylabel('k_2');
% title('Final Feasible Region for k_1, k_2 (Red Points)');
%grid on; hold off;

% subplot(1, 2, 2); hold on;

% k3_line = linspace(0, 100, 100);
% plot(k3_line, k3_line/1.66, 'k--', 'LineWidth', 1);
% plot(k3_line, 0.1105*k3_line, 'k--', 'LineWidth', 1);
% 
% scatter(k3_final, k4_final, 5, 'b', 'filled', 'MarkerFaceAlpha', 0.1);
% xlim([0, 100]); ylim([0, 50]);
% xlabel('k_3'); ylabel('k_4');
% title('Final Feasible Region for k_3, k_4 (Blue Points)');
%grid on; hold off;

disp(['num stabilized: ', num2str(length(k1_final))]);


%% Nonlinear model verification
disp('Start nonlinear model verification...');

num_test = min(10000, length(k1_final)); 
rand_indices = randperm(length(k1_final), num_test);

k1_test = k1_final(rand_indices);% theta
k2_test = k2_final(rand_indices);% r
k3_test = k3_final(rand_indices);% theta_dot
k4_test = k4_final(rand_indices);% r_dot

nonlinear_stable_idx = false(num_test, 1);

z0 = [0.02 * pi/180; 0; 0; 0]; % init
tspan = [0, 5];
par = LatParam();

for i = 1:num_test
    % theta, theta_dot, r, r_dot
    K = [k1_test(i), k3_test(i), k2_test(i), k4_test(i)];

    controller = @(t, z, par) -K * z;
    
    try
        [t_out, z_out] = ode45(@(t, z) LatModel(t, z, par, controller), tspan, z0);
        
        theta_out = z_out(:, 1);
        r_out = z_out(:,3);
        max_theta = max(abs(theta_out));  
        final_theta = abs(theta_out(end));
        max_r = max(abs(r_out));
        
        if (max_theta < 20 * pi/180) && (final_theta < 5 * pi/180) && max_r < 0.14
            nonlinear_stable_idx(i) = true;
        end
        
    iserror = 0;
    catch E
        iserror = 1;
        nonlinear_stable_idx(i) = false;
        disp(E);
        fprintf('Point %d, Error: %s\n', i, E.message);
    end
    
    if mod(i, 50) == 0
        fprintf('Completed %d / %d\n', i, num_test);
    end
end

k1_nl_stable = k1_test(nonlinear_stable_idx);
k2_nl_stable = k2_test(nonlinear_stable_idx);
k3_nl_stable = k3_test(nonlinear_stable_idx);
k4_nl_stable = k4_test(nonlinear_stable_idx);

disp(['Number for nonlinear: ', num2str(length(k1_nl_stable))]);

figure('Position', [150, 150, 1000, 400]);

subplot(1, 2, 1); hold on;
scatter(k1_test, k2_test, 10, 'k', 'filled', 'MarkerFaceAlpha', 0.2, 'DisplayName', 'Tested Points (Linear Stable)');
scatter(k1_nl_stable, k2_nl_stable, 20, 'g', 'filled', 'DisplayName', 'Nonlinear Stable');
xlim([50, 350]); ylim([40, 180]);
xlabel('k_1'); ylabel('k_2');
title('k_1 vs k_2 (Nonlinear Verification)');
legend('Location', 'best'); grid on; hold off;

subplot(1, 2, 2); hold on;
scatter(k3_test, k4_test, 10, 'k', 'filled', 'MarkerFaceAlpha', 0.2, 'DisplayName', 'Tested Points (Linear Stable)');
scatter(k3_nl_stable, k4_nl_stable, 20, 'b', 'filled', 'DisplayName', 'Nonlinear Stable');
xlim([0, 100]); ylim([0, 50]);
xlabel('k_3'); ylabel('k_4');
title('k_3 vs k_4 (Nonlinear Verification)');
legend('Location', 'best'); grid on; hold off;
