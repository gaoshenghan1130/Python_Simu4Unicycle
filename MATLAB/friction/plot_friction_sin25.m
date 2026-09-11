clear; clc;

S = load('friction_sine_A050.mat');

t = S.t_s;
idx = logical(S.analysis_mask);

figure('Color', 'w');
tiledlayout(3,1);

nexttile;
plot(t, 1000*S.xd_m, '--', 'LineWidth', 1.2);
hold on;
plot(t, 1000*S.r_m, 'LineWidth', 1);
plot(t(idx), 1000*S.fit_r_m(idx), 'LineWidth', 1.2);
ylabel('Position (mm)');
legend('Desired', 'Measured', 'Sine fit');
grid on;

nexttile;
plot(t, S.vd_mps, '--', 'LineWidth', 1.2);
hold on;
plot(t, S.v_mps);
ylabel('Velocity (m/s)');
legend('Desired', 'Measured');
grid on;

nexttile;
plot(t, S.Fpd_N, '--');
hold on;
plot(t, S.Fcmd_N);
ylabel('Force command (N)');
xlabel('Time (s)');
legend('PD', 'After CBF');
grid on;

figure('Color', 'w');
plot(t(idx), 1000*S.fit_residual_m(idx));
xlabel('Time (s)');
ylabel('Measured - sine fit (mm)');
title('Position fit residual');
grid on;

cycle_table = array2table( ...
    S.cycle_summary, ...
    'VariableNames', { ...
        'Cycle', 'Samples', 'Amplitude_m', ...
        'PhaseLag_deg', 'Offset_m', ...
        'FitRMSE_m', 'CBFSamples' ...
    });
disp(cycle_table);