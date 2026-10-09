function validation = plot_critical_speed_control_study(input_speeds, test_duration)
%PLOT_CRITICAL_SPEED_CONTROL_STUDY Simulate and plot Case 1 at chosen speeds.
%
% Quick start (speed column in m/s):
%   validation = plot_critical_speed_control_study([1.50; 1.5019; 1.50195; 1.52], 1000);
%
% With no arguments, the editable defaults below are used. Each speed is the
% initial forward speed; all other Case 1 parameters and the initial tilt are
% held fixed. An 80 degree tilt event is used as the divergence indicator.

if nargin < 1 || isempty(input_speeds)
    % Edit this column to try your own initial speeds (m/s).
    input_speeds = [0.5; 0.8; 1.4; 1.6; 1.8; 2.2; 2.4; 2.8];
end
if nargin < 2 || isempty(test_duration)
    test_duration = 25; % seconds
end
validateattributes(input_speeds, {'numeric'}, {'vector','real','finite','positive'});
validateattributes(test_duration, {'numeric'}, {'scalar','real','finite','positive'});
speeds = input_speeds(:);

analysis_dir = fileparts(mfilename('fullpath'));
model_root = fileparts(analysis_dir);
addpath(genpath(model_root));

p = model_parameters();
controller = struct('mode','pd', ...
    'kp_theta',0, 'kd_theta',0, ...
    'kp_r',854.34418, 'kd_r',105.568949, ...
    'kp_gamma',3, 'kd_gamma',0.8, ...
    'theta_ref',0, 'theta_dot_ref',0, ...
    'r_ref',0, 'r_dot_ref',0, ...
    'gamma_ref',0, 'gamma_dot_ref',0, ...
    'force_limit',Inf, 'torque_limit',Inf);

settings = simulation_settings();
settings.t_end = test_duration;
settings.output_dt = 0.1;
settings.max_step = 0.005;
settings.theta0 = 0.1*pi/180;
settings.tilt_limit = 80*pi/180;

% Give each speed a distinct blue shade, ordered from light (slow) to dark.
[~, speed_order] = sort(speeds);
blue = [0.05 0.25 0.55];
shade = linspace(0.30, 1.00, numel(speeds));
colors = zeros(numel(speeds), 3);
for rank = 1:numel(speeds)
    colors(speed_order(rank), :) = (1-shade(rank))*[1 1 1] + shade(rank)*blue;
end

validation = repmat(struct('initial_speed',NaN, 'time',[], 'theta_deg',[], ...
    'rod_position',[], 'gamma_deg',[], 'forward_speed',[], ...
    'peak_abs_tilt_deg',NaN, 'tilt_event',false, 'event_time',NaN), numel(speeds), 1);

for k = 1:numel(speeds)
    run_settings = settings;
    run_settings.forward_speed0 = speeds(k);
    run = simulate_unicycle(p, controller, run_settings);

    validation(k).initial_speed = speeds(k);
    validation(k).time = run.t;
    validation(k).theta_deg = rad2deg(run.X(:,7));
    validation(k).rod_position = run.X(:,9);
    validation(k).gamma_deg = rad2deg(run.X(:,10));
    validation(k).forward_speed = p.R * run.X(:,2);
    validation(k).peak_abs_tilt_deg = max(abs(validation(k).theta_deg));
    if isfield(run, 'event_time') && ~isempty(run.event_time)
        validation(k).tilt_event = true;
        validation(k).event_time = run.event_time;
    end

    if validation(k).tilt_event
        fprintf('v0 = %.6f m/s: 80 deg event at %.3f s; peak |theta| = %.3f deg\n', ...
            speeds(k), validation(k).event_time, validation(k).peak_abs_tilt_deg);
    else
        fprintf('v0 = %.6f m/s: no 80 deg event by %.3f s; peak |theta| = %.3f deg\n', ...
            speeds(k), run.t(end), validation(k).peak_abs_tilt_deg);
    end
end

fig = figure('Color','w', 'Name','Case 1: user-selected initial speeds');
layout = tiledlayout(fig, 2, 2, 'TileSpacing','compact', 'Padding','compact');
title(layout, sprintf('Case 1 speed tests | m_{rod} = %.3g kg | horizon = %.0f s', ...
    p.mr, test_duration));

ax1 = nexttile(layout);
hold(ax1,'on'); grid(ax1,'on');
for k = 1:numel(speeds)
    plot(ax1, validation(k).time, validation(k).theta_deg, ...
        'Color',colors(k,:), 'LineWidth',1.2);
end
yline(ax1, 80, 'k--', '80 deg event');
yline(ax1,-80, 'k--', 'HandleVisibility','off');
xlabel(ax1,'Time (s)'); ylabel(ax1,'Tilt angle (deg)');
title(ax1,'Lean angle');
legend(ax1, compose('v_0 = %.6g m/s', speeds), 'Location','best');

ax2 = nexttile(layout);
hold(ax2,'on'); grid(ax2,'on');
for k = 1:numel(speeds)
    plot(ax2, validation(k).time, validation(k).rod_position, ...
        'Color',colors(k,:), 'LineWidth',1.2);
end
xlabel(ax2,'Time (s)'); ylabel(ax2,'Rod position (m)'); title(ax2,'Rod position');

ax3 = nexttile(layout);
hold(ax3,'on'); grid(ax3,'on');
for k = 1:numel(speeds)
    plot(ax3, validation(k).time, validation(k).gamma_deg, ...
        'Color',colors(k,:), 'LineWidth',1.2);
end
xlabel(ax3,'Time (s)'); ylabel(ax3,'Wheel angle (deg)'); title(ax3,'Wheel angle');

ax4 = nexttile(layout);
hold(ax4,'on'); grid(ax4,'on');
for k = 1:numel(speeds)
    plot(ax4, validation(k).time, validation(k).forward_speed, ...
        'Color',colors(k,:), 'LineWidth',1.2);
end
xlabel(ax4,'Time (s)'); ylabel(ax4,'Forward speed (m/s)'); title(ax4,'Forward speed');

output_png = fullfile(analysis_dir, 'critical_speed_control_study.png');
exportgraphics(fig, output_png, 'Resolution',200);
fprintf('Plot saved to: %s\n', output_png);
end
