clear; close all; clc;

% Load the processed test data from the same folder as this script.
scriptFolder = fileparts(mfilename('fullpath'));
S = load(fullfile(scriptFolder, 'HO10025_friction_test_slow.mat'));

%% Complete time history
figure('Color', 'w', 'Name', 'HO10025 friction test - raw data');
tiledlayout(2, 1, 'TileSpacing', 'compact');

nexttile;
plot(S.time_s, S.velocity_radps, 'LineWidth', 1.1);
grid on;
xlabel('Time (s)');
ylabel('Speed (rad/s)');
title('HO10025 speed response');

nexttile;
yyaxis left;
stairs(S.time_s, S.torque_cmd_Nm, 'LineWidth', 1.2);
ylabel('Commanded torque (N m)');
yyaxis right;
plot(S.time_s, S.current_A, 'LineWidth', 0.8);
ylabel('Measured current (A)');
grid on;
xlabel('Time (s)');
title('Command and measured current');

%% Mean speed over the final 1 second of every 2-second torque level
figure('Color', 'w', 'Name', 'HO10025 friction test - final 1 s means');
errorbar(S.torque_levels_Nm, S.mean_velocity_last1s_radps, ...
    S.std_velocity_last1s_radps, 'o-', 'LineWidth', 1.2, ...
    'MarkerFaceColor', [0.20 0.45 0.85]);
grid on;
xlabel('Commanded torque (N m)');
ylabel('Mean speed over final 1 s (rad/s)');
title('Steady-speed estimate at each torque level');

summaryTable = table(S.torque_levels_Nm, ...
    S.mean_velocity_last1s_radps, S.std_velocity_last1s_radps, ...
    S.mean_current_last1s_A, S.samples_last1s, ...
    'VariableNames', {'TorqueCmd_Nm', 'MeanSpeed_radps', ...
    'StdSpeed_radps', 'MeanCurrent_A', 'Samples'});
disp(summaryTable);

%% Optional Coulomb + viscous fit: tau = C + b*omega
% Change this interval after inspecting the plots. Do not include stationary
% points or the high-speed saturated region.
fitTorqueRange_Nm = [0.5, 1.4];
fitMask = S.torque_levels_Nm >= fitTorqueRange_Nm(1) & ...
          S.torque_levels_Nm <= fitTorqueRange_Nm(2);

p = polyfit(S.mean_velocity_last1s_radps(fitMask), ...
            S.torque_levels_Nm(fitMask), 1);
b_Nms_per_rad = p(1);
C_Nm = p(2);

tauFit = polyval(p, S.mean_velocity_last1s_radps(fitMask));
tauUsed = S.torque_levels_Nm(fitMask);
R2 = 1 - sum((tauUsed - tauFit).^2) / ...
         sum((tauUsed - mean(tauUsed)).^2);

fprintf('\nFit using %.1f to %.1f N m:\n', fitTorqueRange_Nm);
fprintf('b = %.6f N m s/rad\n', b_Nms_per_rad);
fprintf('C = %.6f N m\n', C_Nm);
fprintf('R^2 = %.4f\n', R2);

if C_Nm < 0
    warning(['The fitted C is negative, so the selected data do not follow ', ...
             'a simple Coulomb-plus-viscous model. Adjust the fit range and ', ...
             'check whether 2 s was sufficient to reach steady speed.']);
end

% Separate torque-versus-speed view, matching tau = C + b*omega.
figure('Color', 'w', 'Name', 'HO10025 friction-model fit');
plot(S.mean_velocity_last1s_radps, S.torque_levels_Nm, 'o', ...
    'MarkerFaceColor', [0.75 0.75 0.75]);
hold on;
plot(S.mean_velocity_last1s_radps(fitMask), tauUsed, 'o', ...
    'MarkerFaceColor', [0.20 0.45 0.85]);
omegaLine = linspace(min(S.mean_velocity_last1s_radps(fitMask)), ...
                     max(S.mean_velocity_last1s_radps(fitMask)), 200);
plot(omegaLine, polyval(p, omegaLine), 'r-', 'LineWidth', 1.5);
grid on;
xlabel('Mean speed over final 1 s (rad/s)');
ylabel('Commanded torque (N m)');
title(sprintf('tau = C + b omega: b = %.4g, C = %.4g, R^2 = %.3f', ...
    b_Nms_per_rad, C_Nm, R2));
legend('All torque levels', 'Points used for fit', 'Linear fit', ...
    'Location', 'best');

