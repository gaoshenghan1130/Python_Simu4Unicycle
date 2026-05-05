function segments = plot_real_data( ...
    project_log_dir, mode_name, target_value, time_limit)

clc; close all;

% =========================
% Select logs
% =========================
switch lower(mode_name)
    case 'position'
        file_paths = {
            fullfile(project_log_dir, 'P1.0x3.csv')
            fullfile(project_log_dir, 'P1.0x3(2).csv')
        };
        min_position_range = 0.05;
        min_velocity_peak = 0.02;

    case 'velocity'
        file_paths = {
            fullfile(project_log_dir, 'V0.5x3.csv')
            fullfile(project_log_dir, 'V0.5x3(2).csv')
        };
        min_position_range = 1.0;
        min_velocity_peak = 0.05;

    otherwise
        error('mode_name must be position or velocity.');
end

% =========================
% Load segments
% =========================
segments = read_real_data( ...
    file_paths, ...
    mode_name, ...
    target_value, ...
    'TimeLimit', time_limit, ...
    'ZeroSignals', true, ...
    'MinPositionRange', min_position_range, ...
    'MinVelocityPeak', min_velocity_peak);

fprintf('Loaded %d measured %s segment(s), target %.3g.\n', ...
    numel(segments), mode_name, target_value);

% =========================
% Plot style
% =========================
lineWidth = 1.5;
fontName = 'Helvetica';

applyStyle = @(ax) set(ax, ...
    'Box','on', ...
    'LineWidth',1, ...
    'XGrid','on','YGrid','on', ...
    'GridLineStyle',':', ...
    'GridAlpha',0.4, ...
    'FontName',fontName, ...
    'FontSize',14);

figure('Color','w', ...
    'Name',['Measured Data: ' mode_name], ...
    'Position',[200 50 1000 800]);

colors = 0.3 * lines(numel(segments)) + 0.7;

% =========================================================
% (1) position
% =========================================================
ax1 = subplot(4,1,1); hold on;

for i = 1:numel(segments)
    plot(segments(i).time, segments(i).position, ...
        'Color', colors(i,:), 'LineWidth', lineWidth);
end

if strcmpi(mode_name, 'position')
    plot([0 time_limit], [target_value target_value], 'k:', 'LineWidth', 1.5)
end

ylabel('$x \, (\mathrm{m})$', 'Interpreter', 'latex')
title(sprintf('Measured %s command intervals', mode_name))
xlim([0 time_limit])
applyStyle(ax1);

% =========================================================
% (2) velocity
% =========================================================
ax2 = subplot(4,1,2); hold on;

for i = 1:numel(segments)
    plot(segments(i).time, segments(i).velocity, ...
        'Color', colors(i,:), 'LineWidth', lineWidth);
end

if strcmpi(mode_name, 'velocity')
    plot([0 time_limit], [target_value target_value], 'k:', 'LineWidth', 1.5)
else
    plot([0 time_limit], [0 0], 'k:', 'LineWidth', 1.5)
end

ylabel('$\dot{x} \, (\mathrm{m/s})$', 'Interpreter', 'latex')
xlim([0 time_limit])
applyStyle(ax2);

% =========================================================
% (3) gamma
% =========================================================
ax3 = subplot(4,1,3); hold on;

for i = 1:numel(segments)
    plot(segments(i).time, segments(i).gamma_deg, ...
        'Color', colors(i,:), 'LineWidth', lineWidth);
end

plot([0 time_limit], [0 0], 'k:', 'LineWidth', 1.5)

ylabel('$\gamma \, (\mathrm{deg})$', 'Interpreter', 'latex')
xlim([0 time_limit])
applyStyle(ax3);

% =========================================================
% (4) gamma_dot
% =========================================================
ax4 = subplot(4,1,4); hold on;

for i = 1:numel(segments)
    plot(segments(i).time, segments(i).dgamma_degps, ...
        'Color', colors(i,:), 'LineWidth', lineWidth);
end

plot([0 time_limit], [0 0], 'k:', 'LineWidth', 1.5)

xlabel('$t \, (\mathrm{s})$', 'Interpreter', 'latex')
ylabel('$\dot{\gamma} \, (\mathrm{deg/s})$', 'Interpreter', 'latex')
xlim([0 time_limit])
applyStyle(ax4);

end