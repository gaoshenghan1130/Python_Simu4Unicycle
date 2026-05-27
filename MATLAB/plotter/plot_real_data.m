function segments = plot_real_data( ...
    project_log_dir, mode_name, target_value, time_limit, colors)

% =========================
% Select logs
% =========================
switch lower(mode_name)
    case 'position'
        file_paths = {
            fullfile(project_log_dir, 'P1.0x3.csv')
            fullfile(project_log_dir, 'P1.0x3(2).csv')
        };

    case 'velocity'
        file_paths = {
            fullfile(project_log_dir, 'V0.5x3.csv')
            fullfile(project_log_dir, 'V0.5x3(2).csv')
        };

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
    'PhiToXcScale', 0.2527, ...
    'Color_set', colors);

end