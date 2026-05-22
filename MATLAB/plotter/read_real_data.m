function segments = read_real_data(file_paths, mode_name, target_value, varargin)
%READ_REAL_DATA Read measured command intervals from BLE CSV logs.
%
% This mirrors the Python logic in:
% Derivation/Segway/Tuner.ipynb::parse_command_interval_log
%
% Output segments is a struct array with fields:
% file_path, mode, target_value, time, position, velocity, gamma_deg,
% dgamma_degps, duration.
%
% Required CSV columns for the current measured logs:
% Timestamp, Gamma_rad, Phi_rad, Dphi_radps
%
% Optional column:
% Dgamma_radps

parser = inputParser;
parser.addParameter('TimeLimit', [], @(x) isempty(x) || isnumeric(x));
parser.addParameter('ZeroSignals', true, @(x) islogical(x) || isnumeric(x));
parser.addParameter('PhiToXcScale', 0.2527, @isnumeric);
parser.addParameter('Color_set', [0 0 1], @isnumeric);
parser.parse(varargin{:});
opts = parser.Results;

if ischar(file_paths) || isstring(file_paths)
    file_paths = cellstr(file_paths);
end

segments = struct( ...
    'file_path', {}, ...
    'mode', {}, ...
    'target_value', {}, ...
    'time', {}, ...
    'position', {}, ...
    'velocity', {}, ...
    'gamma_deg', {}, ...
    'dgamma_degps', {}, ...
    'duration', {}, ...
    'Color_set', {});

for file_index = 1:numel(file_paths)
    file_path = char(file_paths{file_index});
    file_segments = parse_one_log(file_path, mode_name, target_value, opts);
    segments = [segments, file_segments]; %#ok<AGROW>
end

end

function segments = parse_one_log(file_path, mode_name, target_value, opts)
if ~isfile(file_path)
    error('read_real_data:FileNotFound', 'CSV file not found: %s', file_path);
end

lines = readlines(file_path);
lines = strip(lines);
lines = lines(lines ~= "");
if isempty(lines)
    error('read_real_data:EmptyFile', 'CSV file is empty: %s', file_path);
end

header = split(string(lines(1)), ',').';
required_columns = ["Timestamp", "Gamma_rad", "Dgamma_radps", "Phi_rad", "Dphi_radps"];
for name = required_columns
    if ~any(header == name)
        error('read_real_data:MissingColumn', ...
            '%s is missing required column %s.', file_path, name);
    end
end

idx_time = find(header == "Timestamp", 1);
idx_gamma = find(header == "Gamma_rad", 1);
idx_phi = find(header == "Phi_rad", 1);
idx_dphi = find(header == "Dphi_radps", 1);
idx_dgamma = find(header == "Dgamma_radps", 1);

current_mode = 'balance';
current_target = 0.0;
current_rows = [];
raw_segments = {};

for line_index = 2:numel(lines)
    line = strtrim(char(char(lines(line_index))));
    if startsWith(line, '"--- SEND COMMAND:')
        if ~isempty(current_rows)
            raw_segments{end + 1} = make_raw_segment( ... %#ok<AGROW>
                file_path, current_mode, current_target, current_rows, opts.Color_set);
        end

        tokens = regexp(line, 'Mode=(\w+),\s*Value=([\d\.-]+)', ...
            'tokens', 'once');
        if ~isempty(tokens)
            current_mode = tokens{1};
            current_target = str2double(tokens{2});
        end
        current_rows = [];
        continue
    end

    if ~strcmpi(current_mode, mode_name) || ...
            ~is_close(current_target, target_value)
        continue
    end

    values = split_csv_line(line);
    if numel(values) < numel(header)
        fprintf('Skipping line %d due to insufficient columns: %s\n', line_index, line);
        continue
    end

    numeric_row = str2double(values);
    if isnan(numeric_row(idx_time))
        fprintf('Skipping line %d due to non-numeric timestamp: %s\n', line_index, line);
        continue
    end

    current_rows = [current_rows; numeric_row(:).']; %#ok<AGROW>
    
end
if ~isempty(current_rows)
    raw_segments{end + 1} = make_raw_segment( ...
        file_path, current_mode, current_target, current_rows, opts.Color_set);
end

segments = struct( ...
    'file_path', {}, ...
    'mode', {}, ...
    'target_value', {}, ...
    'time', {}, ...
    'position', {}, ...
    'velocity', {}, ...
    'gamma_deg', {}, ...
    'dgamma_degps', {}, ...
    'duration', {}, ...
    'Color_set', {});

for segment_index = 1:numel(raw_segments)
    raw = raw_segments{segment_index};
    rows = raw.rows;

    time = rows(:, idx_time);
    time = time - time(1);
    position = rows(:, idx_phi) * opts.PhiToXcScale;
    velocity = rows(:, idx_dphi) * opts.PhiToXcScale;
    gamma_deg = rows(:, idx_gamma) * 180/pi;

    if ~isempty(idx_dgamma)
        dgamma_degps = rows(:, idx_dgamma) * 180/pi;
    else
        dgamma_degps = numerical_gradient(rows(:, idx_gamma), time) * 180/pi;
    end

    if opts.ZeroSignals
        position = position - position(1);
        velocity = velocity - velocity(1);
        gamma_deg = gamma_deg - gamma_deg(1);
    end

    if ~isempty(opts.TimeLimit)
        keep = time <= opts.TimeLimit;
        time = time(keep);
        position = position(keep);
        velocity = velocity(keep);
        gamma_deg = gamma_deg(keep);
        dgamma_degps = dgamma_degps(keep);
    end

    if isempty(time)
        continue
    end

    segments(end + 1) = struct( ... %#ok<AGROW>
        'file_path', raw.file_path, ...
        'mode', raw.mode, ...
        'target_value', raw.target_value, ...
        'time', time, ...
        'position', position, ...
        'velocity', velocity, ...
        'gamma_deg', gamma_deg, ...
        'dgamma_degps', dgamma_degps, ...
        'duration', time(end), ...
        'Color_set', raw.Color_set);
end
end

function raw = make_raw_segment(file_path, mode_name, target_value, rows, color_set)
raw = struct( ...
    'file_path', file_path, ...
    'mode', mode_name, ...
    'target_value', target_value, ...
    'rows', rows, ...
    'Color_set', color_set);
end

function values = split_csv_line(line)
% Logs do not quote numeric data fields, so split is enough here.
values = split(string(line), ',').';
end

function tf = is_close(a, b)
tf = abs(a - b) <= 1e-9 * max(1.0, max(abs(a), abs(b)));
end

function dydt = numerical_gradient(y, t)
dydt = zeros(size(y));
if numel(y) < 2
    return
end

for i = 1:numel(y)
    if i == 1
        dt = t(2) - t(1);
        dydt(i) = safe_divide(y(2) - y(1), dt);
    elseif i == numel(y)
        dt = t(end) - t(end - 1);
        dydt(i) = safe_divide(y(end) - y(end - 1), dt);
    else
        dt = t(i + 1) - t(i - 1);
        dydt(i) = safe_divide(y(i + 1) - y(i - 1), dt);
    end
end
end

function out = safe_divide(num, den)
if den == 0
    out = 0.0;
else
    out = num / den;
end
end

function kept = filter_segments(segments, min_position_range, min_velocity_peak)
kept = segments;
if isempty(segments)
    return
end

keep_mask = true(size(segments));
for i = 1:numel(segments)
    position_range = max(segments(i).position) - min(segments(i).position);
    velocity_peak = max(abs(segments(i).velocity));
    if position_range < min_position_range || velocity_peak < min_velocity_peak
        keep_mask(i) = false;
    end
end

kept = segments(keep_mask);
end
