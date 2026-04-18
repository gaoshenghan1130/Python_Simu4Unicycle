%% 1. Configuration
% List of CSV files to process
file_list = { ...
    '/Users/gaoshenghan/Desktop/Matlab_Simu4Unicycle/Export/Matlab/Data/V0.5x3.csv', ...
    '/Users/gaoshenghan/Desktop/Matlab_Simu4Unicycle/Export/Matlab/Data/V0.5x3(2).csv' ...
    '/Users/gaoshenghan/Desktop/Matlab_Simu4Unicycle/Export/Matlab/Data/P1.0x3.csv'...
    '/Users/gaoshenghan/Desktop/Matlab_Simu4Unicycle/Export/Matlab/Data/P1.0x3(2).csv'...
};

% Initialize containers for grouped segments
all_velocity_segments = {};
all_position_segments = {};

%% 2. Batch Processing
for f = 1:length(file_list)
    fprintf('Processing file: %s\n', file_list{f});
    
    % Parse the custom mixed-format CSV
    segments = parse_mixed_csv(file_list{f});
    
    % Categorize segments by control mode
    for i = 1:length(segments)
        current_seg = segments{i};
        
        if strcmpi(current_seg.mode, 'velocity')
            all_velocity_segments{end+1} = current_seg; %#ok<SAGROW>
        elseif strcmpi(current_seg.mode, 'position')
            all_position_segments{end+1} = current_seg; %#ok<SAGROW>
        end
    end
end

%% 3. Save Grouped Data to .mat Files
% Define column metadata for clarity
% Columns: 1:Timestamp, 2:Theta_rad, 3:Gamma_rad, 4:Phi_rad, 5:Gamma_deg
col_info = '1:Timestamp, 2:Theta_rad, 3:Gamma_rad, 4:Phi_rad, 5:Gamma_deg';

if ~isempty(all_velocity_segments)
    save('experiment_velocity_segments.mat', 'all_velocity_segments', 'col_info');
    fprintf('Saved Velocity segments to experiment_velocity_segments.mat\n');
end

if ~isempty(all_position_segments)
    save('experiment_position_segments.mat', 'all_position_segments', 'col_info');
    fprintf('Saved Position segments to experiment_position_segments.mat\n');
end

%% --- Helper Function: Parser ---
function mode_segments = parse_mixed_csv(file_path)
    mode_segments = {};
    
    % Default initial states
    current_mode = 'balance';
    current_target = 0.0;
    current_data = [];
    command_pattern = 'Mode=(\w+),\s*Value=([\d\.-]+)';
    
    fid = fopen(file_path, 'r');
    if fid == -1, error('Cannot open file: %s', file_path); end
    
    while ~feof(fid)
        line = strtrim(fgetl(fid));
        if isempty(line), continue; end
        
        % ===== Command Line Handling =====
        if startsWith(line, '"--- SEND COMMAND:')
            % Store previous segment before starting a new one
            if ~isempty(current_data)
                segment.mode = current_mode;
                segment.target_value = current_target;
                segment.data = current_data;
                mode_segments{end+1} = segment;
            end
            
            % Parse new Mode and Target Value
            tokens = regexp(line, command_pattern, 'tokens');
            if ~isempty(tokens)
                current_mode = tokens{1}{1};
                current_target = str2double(tokens{1}{2});
            end
            current_data = [];
            
        % ===== Data Line Handling =====
        elseif isstrprop(line(1), 'digit') || line(1) == '-'
            row = str2double(strsplit(line, ','));
            
            % Skip rows with NaN values
            if any(isnan(row)), continue; end
            
            % Ensure the row has enough columns for conversion
            % Based on your requirement: Timestamp, Theta, Gamma, Phi
            % We add a 5th column: Gamma_deg (Column 3 * 180/pi)
            if numel(row) >= 4
                gamma_rad = row(3);
                row(5) = gamma_rad * (180/pi); % Convert Rad to Deg
            end
            
            current_data = [current_data; row];
        end
    end
    fclose(fid);
    
    % Save the final segment
    if ~isempty(current_data)
        segment.mode = current_mode;
        segment.target_value = current_target;
        segment.data = current_data;
        mode_segments{end+1} = segment;
    end
end