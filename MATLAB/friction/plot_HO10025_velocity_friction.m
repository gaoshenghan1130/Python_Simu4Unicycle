clear; close all; clc;

scriptFolder = fileparts(mfilename('fullpath'));
S = load(fullfile(scriptFolder, 'HO10025_velocity_friction_test.mat'));

%% Full time histories
figure('Color', 'w', 'Name', 'HO10025 velocity-controlled friction test');
tiledlayout(2, 1, 'TileSpacing', 'compact');

nexttile;
stairs(S.time_s, S.target_speed_radps, '--', 'LineWidth', 1.2);
hold on;
plot(S.time_s, S.velocity_radps, 'LineWidth', 1.1);
grid on;
xlabel('Time (s)');
ylabel('Speed (rad/s)');
title('Speed tracking');
legend('Target', 'Measured', 'Location', 'best');

nexttile;
yyaxis left;
plot(S.time_s, S.torque_cmd, 'LineWidth', 1.1);
ylabel('Controller command');
yyaxis right;
plot(S.time_s, S.current_A, 'LineWidth', 0.8);
ylabel('Measured current (A)');
grid on;
xlabel('Time (s)');
title('Input required to maintain speed');

%% Final-one-second statistics
figure('Color', 'w', 'Name', 'HO10025 final-one-second speed means');
complete = logical(S.level_complete);
hComplete = errorbar(S.target_levels_radps(complete), ...
    S.mean_velocity_last1s_radps(complete), ...
    S.std_velocity_last1s_radps(complete), ...
    'o-', 'LineWidth', 1.2, 'MarkerFaceColor', [0.20 0.45 0.85]);
hComplete.DisplayName = 'Complete levels';
hold on;
hIdeal = plot(S.target_levels_radps, S.target_levels_radps, ...
    'k--', 'LineWidth', 1.0);
hIdeal.DisplayName = 'Ideal tracking';
if any(~complete)
    hIncomplete = plot(S.target_levels_radps(~complete), ...
         S.mean_velocity_last1s_radps(~complete), 'rx', ...
         'MarkerSize', 9, 'LineWidth', 1.5);
    hIncomplete.DisplayName = 'Incomplete levels';
end
grid on;
xlabel('Target speed (rad/s)');
ylabel('Mean measured speed over final 1 s (rad/s)');
title('Steady-speed tracking check');
legend('show', 'Location', 'best');

summaryTable = table(S.target_levels_radps, S.level_duration_s, complete, ...
    S.mean_velocity_last1s_radps, S.std_velocity_last1s_radps, ...
    S.slope_velocity_last1s_radps2, S.mean_torque_cmd_last1s, ...
    S.std_torque_cmd_last1s, S.mean_current_last1s_A, ...
    'VariableNames', {'TargetSpeed_radps', 'Duration_s', 'Complete', ...
    'MeanSpeed_radps', 'StdSpeed_radps', 'SpeedSlope_radps2', ...
    'MeanCommand', 'StdCommand', 'MeanCurrent_A'});
disp(summaryTable);

%% Coulomb + viscous fit: input = C + b*omega
% Default: fit the averaged controller command. If the API command is
% current rather than torque, set useMeasuredCurrent=true and enter Kt.
useMeasuredCurrent = false;
motorTorqueConstant_NmPerA = 1.0;  % Replace with the actual Kt if needed.

if useMeasuredCurrent
    steadyInput = motorTorqueConstant_NmPerA * S.mean_current_last1s_A;
    steadyInputStd = motorTorqueConstant_NmPerA * S.std_current_last1s_A;
    inputLabel = 'Estimated motor torque K_t I (N m)';
else
    steadyInput = S.mean_torque_cmd_last1s;
    steadyInputStd = S.std_torque_cmd_last1s;
    inputLabel = 'Mean controller command over final 1 s';
end

% Ignore the low-speed stick-slip region. Select by commanded speed so the
% 2 rad/s level remains included even if its measured mean is slightly lower.
minimumFitTargetSpeed_radps = 2.0;
fitMask = complete & ...
          abs(S.target_levels_radps) >= minimumFitTargetSpeed_radps;
omegaUsed = S.mean_velocity_last1s_radps(fitMask);
inputUsed = steadyInput(fitMask);
p = polyfit(omegaUsed, inputUsed, 1);
b = p(1);
C = p(2);
inputFit = polyval(p, omegaUsed);
R2 = 1 - sum((inputUsed - inputFit).^2) / ...
         sum((inputUsed - mean(inputUsed)).^2);

fprintf('\nFit using complete levels with |target speed| >= %.2f rad/s:\n', ...
    minimumFitTargetSpeed_radps);
fprintf('b = %.6f input-unit s/rad\n', b);
fprintf('C = %.6f input-unit\n', C);
fprintf('R^2 = %.4f\n', R2);

figure('Color', 'w', 'Name', 'HO10025 friction-model fit');
hFitData = errorbar(S.mean_velocity_last1s_radps(fitMask), ...
    steadyInput(fitMask), steadyInputStd(fitMask), ...
    'o', 'LineWidth', 1.0, ...
    'MarkerFaceColor', [0.20 0.45 0.85]);
hFitData.DisplayName = 'Points used for fit';
hold on;
omegaLine = linspace(min(omegaUsed), max(omegaUsed), 200);
hLine = plot(omegaLine, polyval(p, omegaLine), 'r-', 'LineWidth', 1.5);
hLine.DisplayName = 'Linear fit';
grid on;
xlabel('Mean speed over final 1 s (rad/s)');
ylabel(inputLabel);
title(sprintf('Input = C + b omega: b = %.4g, C = %.4g, R^2 = %.3f', ...
    b, C, R2));
legend('show', 'Location', 'best');
