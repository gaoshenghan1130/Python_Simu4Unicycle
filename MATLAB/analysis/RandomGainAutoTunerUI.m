function RandomGainAutoTunerUI
% RANDOMGAINAUTOTUNERUI Bidirectional random-gain search with live plots.
%
% Starting gains:
%   k1 = 600, d1 = 60, k_theta = 10, d_theta = 5
%
% Default actuator limit:
%   |F| <= 30 N
%
% Each trial starts from the current best gain vector and randomly increases
% or decreases one or more gains. A candidate is accepted only when its
% nonlinear simulation score improves. Every accepted result is plotted.
% Select "Every candidate" in the UI to plot rejected trials as well.
%
% Run with:
%   RandomGainAutoTunerUI

    addProjectPaths();

    source_par = LatParam();
    defaults = makeDefaults(source_par);

    best_color = [0.0000, 0.4470, 0.7410];
    candidate_color = [0.8500, 0.3250, 0.0980];
    limit_color = [0.75, 0.15, 0.15];
    success_color = [0.10, 0.50, 0.20];
    neutral_color = [0.15, 0.15, 0.15];

    is_running = false;
    stop_requested = false;
    pause_requested = false;

    best_gains = [];
    best_score = Inf;
    best_result = [];
    best_metrics = [];
    best_cfg = [];
    accepted_history = struct([]);

    %% Main window

    fig = uifigure( ...
        'Name', 'Bidirectional Random Equilibrium-Curve Auto Tuner', ...
        'Color', 'w', ...
        'Position', [35, 35, 1700, 950]);
    fig.CloseRequestFcn = @closeWindow;

    main_grid = uigridlayout(fig, [1, 2]);
    main_grid.ColumnWidth = {500, '1x'};
    main_grid.Padding = [10, 10, 10, 10];
    main_grid.ColumnSpacing = 10;

    %% Left column

    left_panel = uipanel(main_grid, ...
        'Title', 'Automatic tuning parameters', ...
        'FontWeight', 'bold');
    left_panel.Layout.Row = 1;
    left_panel.Layout.Column = 1;

    left_grid = uigridlayout(left_panel, [7, 1]);
    left_grid.RowHeight = {32, 435, 58, 34, 100, 48, '1x'};
    left_grid.Padding = [8, 8, 8, 8];
    left_grid.RowSpacing = 7;

    intro_label = uilabel(left_grid, ...
        'Text', ['Candidates randomly increase or decrease from the current ', ...
        'best gains. A lower score is better.'], ...
        'FontAngle', 'italic');
    intro_label.Layout.Row = 1;

    tab_group = uitabgroup(left_grid);
    tab_group.Layout.Row = 2;

    tuning_tab = uitab(tab_group, 'Title', 'Random search');
    simulation_tab = uitab(tab_group, 'Title', 'Simulation');
    physical_tab = uitab(tab_group, 'Title', 'Physical model');
    score_tab = uitab(tab_group, 'Title', 'Score');

    %% Random-search tab

    tuning_grid = uigridlayout(tuning_tab, [12, 5]);
    tuning_grid.ColumnWidth = {90, 68, 68, 68, 100};
    tuning_grid.RowHeight = {25, 30, 30, 30, 30, 10, 30, 30, 30, 30, 10, '1x'};
    tuning_grid.Padding = [8, 8, 8, 8];
    tuning_grid.ColumnSpacing = 7;

    makeHeader(tuning_grid, 1, 1, 'Gain');
    makeHeader(tuning_grid, 1, 2, 'Start');
    makeHeader(tuning_grid, 1, 3, 'Minimum');
    makeHeader(tuning_grid, 1, 4, 'Maximum');
    makeHeader(tuning_grid, 1, 5, 'Max change/trial');

    makeGainLabel(tuning_grid, 2, 'k1  [N/m]');
    fields.start_k1 = addNumericCell(tuning_grid, 2, 2, ...
        defaults.start_k1, [0, 1e7], 'Starting k1.');
    fields.min_k1 = addNumericCell(tuning_grid, 2, 3, ...
        defaults.min_k1, [0, 1e7], 'Minimum allowed k1.');
    fields.max_k1 = addNumericCell(tuning_grid, 2, 4, ...
        defaults.max_k1, [0, 1e7], 'Maximum allowed k1.');
    fields.change_k1 = addNumericCell(tuning_grid, 2, 5, ...
        defaults.change_k1, [0, 1000], ...
        'Maximum random increase or decrease of k1 in percent.');

    makeGainLabel(tuning_grid, 3, 'd1  [N s/m]');
    fields.start_d1 = addNumericCell(tuning_grid, 3, 2, ...
        defaults.start_d1, [0, 1e7], 'Starting d1.');
    fields.min_d1 = addNumericCell(tuning_grid, 3, 3, ...
        defaults.min_d1, [0, 1e7], 'Minimum allowed d1.');
    fields.max_d1 = addNumericCell(tuning_grid, 3, 4, ...
        defaults.max_d1, [0, 1e7], 'Maximum allowed d1.');
    fields.change_d1 = addNumericCell(tuning_grid, 3, 5, ...
        defaults.change_d1, [0, 1000], ...
        'Maximum random increase or decrease of d1 in percent.');

    makeGainLabel(tuning_grid, 4, 'k_theta  [N/rad]');
    fields.start_ktheta = addNumericCell(tuning_grid, 4, 2, ...
        defaults.start_ktheta, [0, 1e7], 'Starting k_theta.');
    fields.min_ktheta = addNumericCell(tuning_grid, 4, 3, ...
        defaults.min_ktheta, [0, 1e7], 'Minimum allowed k_theta.');
    fields.max_ktheta = addNumericCell(tuning_grid, 4, 4, ...
        defaults.max_ktheta, [0, 1e7], 'Maximum allowed k_theta.');
    fields.change_ktheta = addNumericCell(tuning_grid, 4, 5, ...
        defaults.change_ktheta, [0, 1000], ...
        'Maximum random increase or decrease of k_theta in percent.');

    makeGainLabel(tuning_grid, 5, 'd_theta  [N s/rad]');
    fields.start_kdtheta = addNumericCell(tuning_grid, 5, 2, ...
        defaults.start_kdtheta, [0, 1e7], 'Starting d_theta.');
    fields.min_kdtheta = addNumericCell(tuning_grid, 5, 3, ...
        defaults.min_kdtheta, [0, 1e7], 'Minimum allowed d_theta.');
    fields.max_kdtheta = addNumericCell(tuning_grid, 5, 4, ...
        defaults.max_kdtheta, [0, 1e7], 'Maximum allowed d_theta.');
    fields.change_kdtheta = addNumericCell(tuning_grid, 5, 5, ...
        defaults.change_kdtheta, [0, 1000], ...
        'Maximum random increase or decrease of d_theta in percent.');

    fields.max_trials = addLabeledNumericField( ...
        tuning_grid, 7, 'Maximum trials', defaults.max_trials, ...
        [1, 1e7], 'Number of random candidate simulations.');
    fields.random_seed = addLabeledNumericField( ...
        tuning_grid, 8, 'Random seed', defaults.random_seed, ...
        [0, 2^32 - 1], 'Reproduce the same random search.');
    fields.mutation_probability = addLabeledNumericField( ...
        tuning_grid, 9, 'Gain-change probability', ...
        defaults.mutation_probability, [0.01, 1], ...
        'Independent probability that each searchable gain changes.');
    fields.min_improvement_percent = addLabeledNumericField( ...
        tuning_grid, 10, 'Minimum improvement [%]', ...
        defaults.min_improvement_percent, [0, 99.999], ...
        'Candidate must improve the best score by at least this percentage.');

    search_note = uitextarea(tuning_grid, ...
        'Editable', 'off', ...
        'Value', { ...
            'Proposal rule:', ...
            'candidate = best +/- random change', ...
            'candidate is clamped between Minimum and Maximum'}, ...
        'FontName', 'Courier New');
    search_note.Layout.Row = 12;
    search_note.Layout.Column = [1, 5];

    %% Simulation tab

    simulation_grid = uigridlayout(simulation_tab, [10, 2]);
    simulation_grid.ColumnWidth = {'1x', 135};
    simulation_grid.RowHeight = [repmat({30}, 1, 9), {'1x'}];
    simulation_grid.Padding = [10, 10, 10, 10];

    fields.theta0 = addTwoColumnField(simulation_grid, 1, ...
        'theta(0)  [rad]', defaults.theta0, [-1.45, 1.45], ...
        'Initial lean angle.');
    fields.theta_dot0 = addTwoColumnField(simulation_grid, 2, ...
        'theta_dot(0)  [rad/s]', defaults.theta_dot0, [-100, 100], ...
        'Initial lean rate.');
    fields.r0 = addTwoColumnField(simulation_grid, 3, ...
        'r(0)  [m]', defaults.r0, [-100, 100], ...
        'Initial rod position.');
    fields.r_dot0 = addTwoColumnField(simulation_grid, 4, ...
        'r_dot(0)  [m/s]', defaults.r_dot0, [-100, 100], ...
        'Initial rod velocity.');
    fields.duration = addTwoColumnField(simulation_grid, 5, ...
        'Duration  [s]', defaults.duration, [0.01, 1000], ...
        'Simulation end time.');
    fields.force_limit = addTwoColumnField(simulation_grid, 6, ...
        'Force limit  [N]', defaults.force_limit, [1e-6, 1e9], ...
        'Symmetric control-force saturation. Default is 30 N.');
    fields.rod_limit = addTwoColumnField(simulation_grid, 7, ...
        'Rod stop limit  [m]', defaults.rod_limit, [1e-6, 1e6], ...
        'Simulation stops if |r| exceeds this value.');
    fields.fall_limit_deg = addTwoColumnField(simulation_grid, 8, ...
        'Fall limit  [deg]', defaults.fall_limit_deg, [0.1, 89.9], ...
        'Simulation stops if |theta| exceeds this value.');
    fields.chunk_duration = addTwoColumnField(simulation_grid, 9, ...
        'ODE chunk  [s]', defaults.chunk_duration, [0.005, 5], ...
        'Shorter chunks make Pause and Stop more responsive.');

    high_accuracy = uicheckbox(simulation_grid, ...
        'Text', 'High accuracy: RelTol 1e-8, MaxStep 0.002 s (slower)', ...
        'Value', defaults.high_accuracy);
    high_accuracy.Layout.Row = 10;
    high_accuracy.Layout.Column = [1, 2];

    %% Physical-model tab

    physical_grid = uigridlayout(physical_tab, [9, 2]);
    physical_grid.ColumnWidth = {'1x', 135};
    physical_grid.RowHeight = repmat({30}, 1, 9);
    physical_grid.Padding = [10, 10, 10, 10];

    fields.R = addTwoColumnField(physical_grid, 1, ...
        'R  [m]', defaults.R, [1e-6, 100], 'Wheel radius.');
    fields.g = addTwoColumnField(physical_grid, 2, ...
        'g  [m/s^2]', defaults.g, [1e-6, 100], 'Gravity.');
    fields.m_W = addTwoColumnField(physical_grid, 3, ...
        'm_W  [kg]', defaults.m_W, [0, 1e5], 'Wheel mass.');
    fields.m_L = addTwoColumnField(physical_grid, 4, ...
        'm_L (each side)  [kg]', defaults.m_L, [1e-6, 1e5], ...
        'Moving mass on each side; m_rod = 2 m_L.');
    fields.m_B = addTwoColumnField(physical_grid, 5, ...
        'm_B  [kg]', defaults.m_B, [0, 1e5], 'Body mass.');
    fields.h = addTwoColumnField(physical_grid, 6, ...
        'h  [m]', defaults.h, [-100, 100], ...
        'Body center-of-mass offset.');
    fields.I_b = addTwoColumnField(physical_grid, 7, ...
        'I_b  [kg m^2]', defaults.I_b, [0, 1e6], 'Body inertia.');
    fields.I_w = addTwoColumnField(physical_grid, 8, ...
        'I_w  [kg m^2]', defaults.I_w, [0, 1e6], 'Wheel inertia.');
    fields.I_rod = addTwoColumnField(physical_grid, 9, ...
        'I_rod  [kg m^2]', defaults.I_rod, [0, 1e6], ...
        'Rod assembly inertia.');

    physical_names = { ...
        'R', 'g', 'm_W', 'm_L', 'm_B', 'h', 'I_b', 'I_w', 'I_rod'};
    for physical_idx = 1:numel(physical_names)
        fields.(physical_names{physical_idx}).ValueChangedFcn = ...
            @physicalParameterChanged;
    end

    %% Score tab

    score_grid = uigridlayout(score_tab, [9, 2]);
    score_grid.ColumnWidth = {'1x', 135};
    score_grid.RowHeight = [repmat({30}, 1, 8), {'1x'}];
    score_grid.Padding = [10, 10, 10, 10];

    fields.w_theta = addTwoColumnField(score_grid, 1, ...
        'Lean integral weight', defaults.w_theta, [0, 1e6], ...
        'Weight on the normalized theta-squared integral.');
    fields.w_r = addTwoColumnField(score_grid, 2, ...
        'Rod integral weight', defaults.w_r, [0, 1e6], ...
        'Weight on the normalized r-squared integral.');
    fields.w_rate = addTwoColumnField(score_grid, 3, ...
        'Velocity weight', defaults.w_rate, [0, 1e6], ...
        'Weight on theta_dot and r_dot.');
    fields.w_force = addTwoColumnField(score_grid, 4, ...
        'Force weight', defaults.w_force, [0, 1e6], ...
        'Weight on normalized force usage.');
    fields.w_terminal = addTwoColumnField(score_grid, 5, ...
        'Terminal-error weight', defaults.w_terminal, [0, 1e6], ...
        'Strong penalty for a nonzero final state.');
    fields.w_peak = addTwoColumnField(score_grid, 6, ...
        'Peak-excursion weight', defaults.w_peak, [0, 1e6], ...
        'Penalty for maximum |theta| and maximum |r|.');
    fields.w_settling = addTwoColumnField(score_grid, 7, ...
        'Settling-time weight', defaults.w_settling, [0, 1e6], ...
        'Penalty for late settling.');
    fields.w_saturation = addTwoColumnField(score_grid, 8, ...
        'Force-saturation weight', defaults.w_saturation, [0, 1e6], ...
        'Penalty for time spent at the force limit.');

    score_note = uitextarea(score_grid, ...
        'Editable', 'off', ...
        'Value', { ...
            'All state terms are normalized before scoring.', ...
            'A stopped/failed simulation receives a large penalty.', ...
            'Lower score is better.'});
    score_note.Layout.Row = 9;
    score_note.Layout.Column = [1, 2];

    %% Controls below tabs

    derived_panel = uipanel(left_grid, 'Title', 'Derived values');
    derived_panel.Layout.Row = 3;
    derived_grid = uigridlayout(derived_panel, [1, 1]);
    derived_grid.Padding = [6, 2, 6, 2];
    derived_label = uilabel(derived_grid, ...
        'Text', '', ...
        'FontName', 'Courier New', ...
        'FontSize', 11);

    plot_option_grid = uigridlayout(left_grid, [1, 2]);
    plot_option_grid.Layout.Row = 4;
    plot_option_grid.ColumnWidth = {125, '1x'};
    plot_option_grid.Padding = [2, 0, 2, 0];
    uilabel(plot_option_grid, 'Text', 'Live plotting mode:');
    plot_mode = uidropdown(plot_option_grid, ...
        'Items', {'Every new best', 'Every candidate'}, ...
        'Value', defaults.plot_mode);
    plot_mode.Layout.Column = 2;

    button_grid = uigridlayout(left_grid, [3, 2]);
    button_grid.Layout.Row = 5;
    button_grid.RowHeight = {28, 28, 28};
    button_grid.ColumnWidth = {'1x', '1x'};
    button_grid.Padding = [0, 0, 0, 0];

    start_button = uibutton(button_grid, ...
        'Text', 'Start random search', ...
        'FontWeight', 'bold', ...
        'ButtonPushedFcn', @startSearch);
    start_button.Layout.Row = 1;
    start_button.Layout.Column = [1, 2];

    pause_button = uibutton(button_grid, ...
        'Text', 'Pause', ...
        'Enable', 'off', ...
        'ButtonPushedFcn', @togglePause);
    pause_button.Layout.Row = 2;
    pause_button.Layout.Column = 1;

    stop_button = uibutton(button_grid, ...
        'Text', 'Stop', ...
        'Enable', 'off', ...
        'ButtonPushedFcn', @stopSearch);
    stop_button.Layout.Row = 2;
    stop_button.Layout.Column = 2;

    reset_button = uibutton(button_grid, ...
        'Text', 'Reset and plot start', ...
        'ButtonPushedFcn', @resetAndPlotStart);
    reset_button.Layout.Row = 3;
    reset_button.Layout.Column = 1;

    export_button = uibutton(button_grid, ...
        'Text', 'Export best result', ...
        'ButtonPushedFcn', @exportBestResult);
    export_button.Layout.Row = 3;
    export_button.Layout.Column = 2;

    status_label = uilabel(left_grid, ...
        'Text', 'Ready', ...
        'FontWeight', 'bold', ...
        'FontColor', neutral_color, ...
        'WordWrap', 'on');
    status_label.Layout.Row = 6;

    history_panel = uipanel(left_grid, ...
        'Title', 'Accepted improvements (newest first)');
    history_panel.Layout.Row = 7;
    history_grid = uigridlayout(history_panel, [1, 1]);
    history_grid.Padding = [2, 2, 2, 2];

    history_table = uitable(history_grid, ...
        'Data', [], ...
        'ColumnName', { ...
            'Trial', 'k1', 'd1', 'k_theta', 'd_theta', ...
            'Score', 'max theta', 'max r', 'sat %'}, ...
        'ColumnEditable', false(1, 9), ...
        'RowName', {});
    history_table.ColumnWidth = {45, 55, 50, 58, 58, 65, 68, 60, 50};

    %% Right plot column

    plot_grid = uigridlayout(main_grid, [3, 2]);
    plot_grid.Layout.Row = 1;
    plot_grid.Layout.Column = 2;
    plot_grid.RowHeight = {'1x', '1x', '1x'};
    plot_grid.ColumnWidth = {'1x', '1x'};
    plot_grid.Padding = [4, 4, 4, 4];
    plot_grid.RowSpacing = 8;
    plot_grid.ColumnSpacing = 8;

    ax.theta = uiaxes(plot_grid);
    ax.theta.Layout.Row = 1;
    ax.theta.Layout.Column = 1;
    configureAxis(ax.theta, 'Lean angle', 't  (s)', 'theta  (rad)');

    ax.r = uiaxes(plot_grid);
    ax.r.Layout.Row = 1;
    ax.r.Layout.Column = 2;
    configureAxis(ax.r, 'Rod position', 't  (s)', 'r  (m)');

    ax.s = uiaxes(plot_grid);
    ax.s.Layout.Row = 2;
    ax.s.Layout.Column = 1;
    configureAxis(ax.s, 'Surface error: s = r - C_eq tan(theta)', ...
        't  (s)', 's  (m)');

    ax.s_dot = uiaxes(plot_grid);
    ax.s_dot.Layout.Row = 2;
    ax.s_dot.Layout.Column = 2;
    configureAxis(ax.s_dot, 'Surface velocity', 't  (s)', 's dot  (m/s)');

    ax.F = uiaxes(plot_grid);
    ax.F.Layout.Row = 3;
    ax.F.Layout.Column = 1;
    configureAxis(ax.F, 'Control force', 't  (s)', 'F  (N)');

    ax.phase = uiaxes(plot_grid);
    ax.phase.Layout.Row = 3;
    ax.phase.Layout.Column = 2;
    configureAxis(ax.phase, 'Trajectory in the (theta, r) plane', ...
        'theta  (rad)', 'r  (m)');

    updateDerivedLabel();
    drawnow;
    initializeStartingPoint();

    %% UI helper functions

    function makeHeader(parent, row, column, text_value)
        label = uilabel(parent, ...
            'Text', text_value, ...
            'HorizontalAlignment', 'center', ...
            'FontWeight', 'bold');
        label.Layout.Row = row;
        label.Layout.Column = column;
    end

    function makeGainLabel(parent, row, text_value)
        label = uilabel(parent, 'Text', text_value);
        label.Layout.Row = row;
        label.Layout.Column = 1;
    end

    function field = addNumericCell(parent, row, column, value, limits, tip)
        field = uieditfield(parent, ...
            'numeric', ...
            'Value', value, ...
            'Limits', limits, ...
            'ValueDisplayFormat', '%.6g', ...
            'Tooltip', tip);
        field.Layout.Row = row;
        field.Layout.Column = column;
    end

    function field = addLabeledNumericField( ...
            parent, row, label_text, value, limits, tip)

        label = uilabel(parent, 'Text', label_text);
        label.Layout.Row = row;
        label.Layout.Column = [1, 3];

        field = addNumericCell(parent, row, 4, value, limits, tip);
        field.Layout.Column = [4, 5];
    end

    function field = addTwoColumnField( ...
            parent, row, label_text, value, limits, tip)

        label = uilabel(parent, 'Text', label_text);
        label.Layout.Row = row;
        label.Layout.Column = 1;

        field = uieditfield(parent, ...
            'numeric', ...
            'Value', value, ...
            'Limits', limits, ...
            'ValueDisplayFormat', '%.6g', ...
            'Tooltip', tip);
        field.Layout.Row = row;
        field.Layout.Column = 2;
    end

    %% Search callbacks

    function startSearch(~, ~)
        if is_running
            return;
        end

        updateDerivedLabel();

        try
            cfg = readConfiguration();
        catch ME
            showError(['Invalid parameter: ', ME.message]);
            return;
        end

        rng(round(cfg.random_seed), 'twister');
        stop_requested = false;
        pause_requested = false;
        is_running = true;
        setRunningControls(true);

        [ok, cancelled] = establishBaseline(cfg);
        if ~ok || cancelled || stop_requested || ~isvalid(fig)
            finishSearch('Search stopped while evaluating the starting gains.');
            return;
        end

        max_trials = round(cfg.max_trials);
        no_search_direction = false;

        for trial = 1:max_trials
            if stop_requested || ~isvalid(fig)
                break;
            end

            waitWhilePaused(trial, max_trials);
            if stop_requested || ~isvalid(fig)
                break;
            end

            [candidate_gains, can_change] = proposeCandidate( ...
                best_gains, cfg.min_gains, cfg.max_gains, ...
                cfg.change_percent, ...
                cfg.mutation_probability);

            if ~can_change
                no_search_direction = true;
                break;
            end

            status_label.Text = sprintf( ...
                ['Trial %d/%d: simulating candidate ', ...
                '[%.4g, %.4g, %.4g, %.4g]'], ...
                trial, max_trials, candidate_gains);
            status_label.FontColor = candidate_color;
            drawnow limitrate;

            candidate_cfg = applyGains(cfg, candidate_gains);
            progress_callback = @(fraction) showTrialProgress( ...
                trial, max_trials, fraction);

            [candidate_result, cancelled] = simulateController( ...
                candidate_cfg, progress_callback, @shouldStop);

            if cancelled || stop_requested || ~isvalid(fig)
                break;
            end

            candidate_metrics = evaluatePerformance( ...
                candidate_result, candidate_cfg);

            if strcmp(plot_mode.Value, 'Every candidate')
                plot_label = sprintf( ...
                    'Candidate trial %d, score %.6g', ...
                    trial, candidate_metrics.score);
                renderResult(candidate_result, candidate_cfg, ...
                    candidate_metrics, candidate_color, plot_label);
            end

            improvement_threshold = ...
                best_score*(1 - cfg.min_improvement_percent/100);

            if candidate_metrics.score < improvement_threshold
                old_score = best_score;
                best_gains = candidate_gains;
                best_score = candidate_metrics.score;
                best_result = candidate_result;
                best_metrics = candidate_metrics;
                best_cfg = candidate_cfg;

                addHistoryEntry(trial, best_gains, best_metrics);
                updateHistoryTable();

                plot_label = sprintf( ...
                    'NEW BEST at trial %d: %.6g -> %.6g', ...
                    trial, old_score, best_score);
                renderResult(best_result, best_cfg, best_metrics, ...
                    best_color, plot_label);

                status_label.Text = sprintf( ...
                    ['New best at trial %d/%d: score %.6g, ', ...
                    'gains [%.4g, %.4g, %.4g, %.4g]'], ...
                    trial, max_trials, best_score, best_gains);
                status_label.FontColor = success_color;
                drawnow;
            else
                status_label.Text = sprintf( ...
                    ['Trial %d/%d rejected: candidate %.6g, ', ...
                    'best %.6g'], ...
                    trial, max_trials, candidate_metrics.score, best_score);
                status_label.FontColor = neutral_color;
                drawnow limitrate;
            end
        end

        if ~isvalid(fig)
            return;
        end

        if ~isempty(best_result)
            renderResult(best_result, best_cfg, best_metrics, best_color, ...
                sprintf('Final best: score %.6g', best_score));
        end

        if stop_requested
            message = sprintf( ...
                'Stopped. Best score %.6g with gains [%.4g, %.4g, %.4g, %.4g].', ...
                best_score, best_gains);
        elseif no_search_direction
            message = sprintf( ...
                ['No gain has a nonzero search interval. Best score %.6g ', ...
                'with gains [%.4g, %.4g, %.4g, %.4g].'], ...
                best_score, best_gains);
        else
            message = sprintf( ...
                ['Search completed. Best score %.6g with gains ', ...
                '[%.4g, %.4g, %.4g, %.4g].'], ...
                best_score, best_gains);
        end

        finishSearch(message);
    end

    function initializeStartingPoint(varargin) %#ok<INUSD>
        if is_running || ~isvalid(fig)
            return;
        end

        updateDerivedLabel();

        try
            cfg = readConfiguration();
        catch ME
            showError(['Invalid parameter: ', ME.message]);
            return;
        end

        stop_requested = false;
        pause_requested = false;
        is_running = true;
        setRunningControls(true);

        [ok, cancelled] = establishBaseline(cfg);

        if ~isvalid(fig)
            return;
        end

        if ok && ~cancelled
            message = sprintf( ...
                ['Starting gains plotted: score %.6g, ', ...
                '[%.4g, %.4g, %.4g, %.4g].'], ...
                best_score, best_gains);
        else
            message = 'Starting-gain simulation was stopped.';
        end

        finishSearch(message);
    end

    function [ok, cancelled] = establishBaseline(cfg)
        best_gains = cfg.start_gains;
        baseline_cfg = applyGains(cfg, best_gains);

        status_label.Text = sprintf( ...
            'Evaluating starting gains [%.4g, %.4g, %.4g, %.4g]...', ...
            best_gains);
        status_label.FontColor = best_color;
        drawnow;

        [baseline_result, cancelled] = simulateController( ...
            baseline_cfg, @showBaselineProgress, @shouldStop);

        if cancelled || ~isvalid(fig)
            ok = false;
            return;
        end

        baseline_metrics = evaluatePerformance( ...
            baseline_result, baseline_cfg);

        best_result = baseline_result;
        best_metrics = baseline_metrics;
        best_score = baseline_metrics.score;
        best_cfg = baseline_cfg;

        accepted_history = struct([]);
        addHistoryEntry(0, best_gains, best_metrics);
        updateHistoryTable();

        renderResult(best_result, best_cfg, best_metrics, best_color, ...
            sprintf('Starting gains: score %.6g', best_score));
        ok = true;
    end

    function showBaselineProgress(fraction)
        if ~isvalid(fig)
            return;
        end

        status_label.Text = sprintf( ...
            'Evaluating starting gains: %.0f%%', 100*fraction);
        drawnow limitrate;
    end

    function showTrialProgress(trial, total_trials, fraction)
        if ~isvalid(fig)
            return;
        end

        status_label.Text = sprintf( ...
            'Trial %d/%d simulation: %.0f%% | current best %.6g', ...
            trial, total_trials, 100*fraction, best_score);
        drawnow limitrate;
    end

    function answer = shouldStop()
        answer = stop_requested || ~isvalid(fig);
    end

    function togglePause(~, ~)
        if ~is_running
            return;
        end

        pause_requested = ~pause_requested;

        if pause_requested
            pause_button.Text = 'Resume';
            status_label.Text = ...
                'Pause requested; waiting for the current simulation to finish.';
            status_label.FontColor = candidate_color;
        else
            pause_button.Text = 'Pause';
            status_label.Text = 'Resuming random search...';
            status_label.FontColor = success_color;
        end

        drawnow;
    end

    function waitWhilePaused(trial, max_trials)
        while pause_requested && ~stop_requested && isvalid(fig)
            status_label.Text = sprintf( ...
                'Paused before trial %d/%d. Press Resume to continue.', ...
                trial, max_trials);
            status_label.FontColor = candidate_color;
            drawnow;
            pause(0.05);
        end
    end

    function stopSearch(~, ~)
        stop_requested = true;
        pause_requested = false;
        pause_button.Text = 'Pause';

        if isvalid(fig)
            status_label.Text = ...
                'Stopping after the current ODE chunk...';
            status_label.FontColor = candidate_color;
            drawnow;
        end
    end

    function resetAndPlotStart(~, ~)
        if is_running
            return;
        end

        names = fieldnames(fields);
        for idx = 1:numel(names)
            name = names{idx};
            fields.(name).Value = defaults.(name);
        end

        high_accuracy.Value = defaults.high_accuracy;
        plot_mode.Value = defaults.plot_mode;
        updateDerivedLabel();
        initializeStartingPoint();
    end

    function physicalParameterChanged(~, ~)
        updateDerivedLabel();
        status_label.Text = ...
            'Physical parameter changed; restart the search to use it.';
        status_label.FontColor = candidate_color;
    end

    function exportBestResult(~, ~)
        if isempty(best_result)
            showError('No best result is available yet.');
            return;
        end

        assignin('base', 'random_tuning_best_result', best_result);
        assignin('base', 'random_tuning_best_config', best_cfg);
        assignin('base', 'random_tuning_best_metrics', best_metrics);
        assignin('base', 'random_tuning_history', accepted_history);

        status_label.Text = [ ...
            'Exported best result, config, metrics, and accepted history.'];
        status_label.FontColor = success_color;
    end

    function finishSearch(message)
        is_running = false;
        pause_requested = false;

        if ~isvalid(fig)
            return;
        end

        setRunningControls(false);
        pause_button.Text = 'Pause';
        status_label.Text = message;
        status_label.FontColor = success_color;
        drawnow;
    end

    function setRunningControls(running)
        if running
            start_button.Enable = 'off';
            pause_button.Enable = 'on';
            stop_button.Enable = 'on';
            reset_button.Enable = 'off';
            export_button.Enable = 'off';
            editable_state = 'off';
        else
            start_button.Enable = 'on';
            pause_button.Enable = 'off';
            stop_button.Enable = 'off';
            reset_button.Enable = 'on';
            export_button.Enable = 'on';
            editable_state = 'on';
        end

        names = fieldnames(fields);
        for idx = 1:numel(names)
            fields.(names{idx}).Enable = editable_state;
        end

        high_accuracy.Enable = editable_state;
        plot_mode.Enable = editable_state;
    end

    function closeWindow(~, ~)
        stop_requested = true;
        pause_requested = false;
        delete(fig);
    end

    %% Configuration and history

    function cfg = readConfiguration()
        names = fieldnames(fields);
        for idx = 1:numel(names)
            name = names{idx};
            value = fields.(name).Value;

            if ~isscalar(value) || ~isfinite(value)
                error('%s must be a finite scalar.', name);
            end

            cfg.(name) = value;
        end

        cfg.start_gains = [ ...
            cfg.start_k1, cfg.start_d1, ...
            cfg.start_ktheta, cfg.start_kdtheta];
        cfg.min_gains = [ ...
            cfg.min_k1, cfg.min_d1, ...
            cfg.min_ktheta, cfg.min_kdtheta];
        cfg.max_gains = [ ...
            cfg.max_k1, cfg.max_d1, ...
            cfg.max_ktheta, cfg.max_kdtheta];
        cfg.change_percent = [ ...
            cfg.change_k1, cfg.change_d1, ...
            cfg.change_ktheta, cfg.change_kdtheta];

        if any(cfg.min_gains > cfg.max_gains)
            error('Every minimum gain must be less than or equal to its maximum.');
        end

        if any(cfg.start_gains < cfg.min_gains) ...
                || any(cfg.start_gains > cfg.max_gains)
            error('Every starting gain must lie between its minimum and maximum.');
        end

        searchable = cfg.max_gains > cfg.min_gains ...
            & cfg.change_percent > 0;
        if ~any(searchable)
            error(['At least one gain must have a nonzero search interval ', ...
                'and a positive maximum change percentage.']);
        end

        cfg.max_trials = round(cfg.max_trials);
        cfg.random_seed = round(cfg.random_seed);
        cfg.high_accuracy = high_accuracy.Value;

        cfg.par.R = cfg.R;
        cfg.par.g = cfg.g;
        cfg.par.m_W = cfg.m_W;
        cfg.par.m_L = cfg.m_L;
        cfg.par.m_B = cfg.m_B;
        cfg.par.h = cfg.h;
        cfg.par.I_b = cfg.I_b;
        cfg.par.I_w = cfg.I_w;
        cfg.par.I_rod = cfg.I_rod;
        cfg.par.m_rod = 2*cfg.m_L;
        cfg.par.G = (cfg.m_W*cfg.R + cfg.m_B*(cfg.R + cfg.h))*cfg.g;

        cfg.C_eq = ...
            (cfg.par.G + cfg.par.m_rod*cfg.g*cfg.R) ...
            /(cfg.par.m_rod*cfg.g);
        cfg.fall_limit = cfg.fall_limit_deg*pi/180;
        cfg.z0 = [cfg.theta0; cfg.theta_dot0; cfg.r0; cfg.r_dot0];

        cfg.weights.theta = cfg.w_theta;
        cfg.weights.r = cfg.w_r;
        cfg.weights.rate = cfg.w_rate;
        cfg.weights.force = cfg.w_force;
        cfg.weights.terminal = cfg.w_terminal;
        cfg.weights.peak = cfg.w_peak;
        cfg.weights.settling = cfg.w_settling;
        cfg.weights.saturation = cfg.w_saturation;

        if cfg.high_accuracy
            cfg.rel_tol = 1e-8;
            cfg.abs_tol = 1e-10;
            cfg.max_step = 0.002;
        else
            cfg.rel_tol = 1e-6;
            cfg.abs_tol = 1e-8;
            cfg.max_step = min(0.01, cfg.chunk_duration/2);
        end
    end

    function updateDerivedLabel()
        m_rod = 2*fields.m_L.Value;
        G = ( ...
            fields.m_W.Value*fields.R.Value ...
            + fields.m_B.Value*(fields.R.Value + fields.h.Value)) ...
            *fields.g.Value;
        C_eq = (G + m_rod*fields.g.Value*fields.R.Value) ...
            /(m_rod*fields.g.Value);

        derived_label.Text = sprintf( ...
            'm_rod = %.4g kg    G = %.4g N m    C_eq = %.5g m', ...
            m_rod, G, C_eq);
    end

    function addHistoryEntry(trial, gains, metrics)
        entry.trial = trial;
        entry.k1 = gains(1);
        entry.d1 = gains(2);
        entry.ktheta = gains(3);
        entry.kdtheta = gains(4);
        entry.score = metrics.score;
        entry.max_theta = metrics.max_theta;
        entry.max_r = metrics.max_r;
        entry.max_force = metrics.max_force;
        entry.saturation_percent = 100*metrics.saturation_fraction;
        entry.settling_time = metrics.settling_time;
        entry.completed = metrics.completed;

        if isempty(accepted_history)
            accepted_history = entry;
        else
            accepted_history(end + 1) = entry; %#ok<AGROW>
        end
    end

    function updateHistoryTable()
        if isempty(accepted_history)
            history_table.Data = [];
            return;
        end

        count = numel(accepted_history);
        first_idx = max(1, count - 19);
        selected = accepted_history(count:-1:first_idx);
        rows = zeros(numel(selected), 9);

        for row_idx = 1:numel(selected)
            item = selected(row_idx);
            rows(row_idx, :) = [ ...
                item.trial, item.k1, item.d1, item.ktheta, ...
                item.kdtheta, item.score, item.max_theta, ...
                item.max_r, item.saturation_percent];
        end

        history_table.Data = rows;
    end

    %% Plotting

    function renderResult(result, cfg, metrics, color, plot_label)
        if ~isvalid(fig)
            return;
        end

        t = result.t;
        z = result.z;

        plotTimeHistory(ax.theta, t, z(:, 1), color);
        addSymmetricLimits(ax.theta, cfg.fall_limit, z(:, 1), limit_color, false);
        title(ax.theta, ['Lean angle - ', plot_label], 'Interpreter', 'none');

        plotTimeHistory(ax.r, t, z(:, 3), color);
        addSymmetricLimits(ax.r, cfg.rod_limit, z(:, 3), limit_color, false);
        title(ax.r, sprintf('Rod position - max |r| = %.4g m', ...
            metrics.max_r), 'Interpreter', 'none');

        plotTimeHistory(ax.s, t, result.s, color);
        title(ax.s, 'Surface error: s = r - C_eq tan(theta)', ...
            'Interpreter', 'none');

        plotTimeHistory(ax.s_dot, t, result.s_dot, color);
        title(ax.s_dot, 'Surface velocity', 'Interpreter', 'none');

        plotTimeHistory(ax.F, t, result.F, color);
        addSymmetricLimits(ax.F, cfg.force_limit, result.F, limit_color, true);
        title(ax.F, sprintf( ...
            'Control force - saturation %.2f%%', ...
            100*metrics.saturation_fraction), 'Interpreter', 'none');

        cla(ax.phase);
        hold(ax.phase, 'on');
        grid(ax.phase, 'on');
        box(ax.phase, 'on');

        theta_bound = max([ ...
            0.15, 1.15*max(abs(z(:, 1))), 1.15*abs(cfg.theta0)]);
        theta_bound = min(theta_bound, 0.95*pi/2);
        theta_curve = linspace(-theta_bound, theta_bound, 600);

        plot(ax.phase, theta_curve, cfg.C_eq*tan(theta_curve), ...
            '--k', 'LineWidth', 1.5, ...
            'DisplayName', 'r = C_eq tan(theta)');
        plot(ax.phase, z(:, 1), z(:, 3), ...
            'Color', color, 'LineWidth', 1.7, ...
            'DisplayName', 'Trajectory');
        plot(ax.phase, cfg.theta0, cfg.r0, 'ko', ...
            'MarkerFaceColor', 'y', 'MarkerSize', 7, ...
            'DisplayName', 'Initial state');
        plot(ax.phase, z(end, 1), z(end, 3), 'kx', ...
            'LineWidth', 1.7, 'MarkerSize', 8, ...
            'DisplayName', 'Final state');
        legend(ax.phase, 'Location', 'best', 'Interpreter', 'none');
        title(ax.phase, sprintf( ...
            'theta-r trajectory - settling %.4g s', ...
            metrics.settling_time), 'Interpreter', 'none');
        hold(ax.phase, 'off');

        drawnow;
    end

    function showError(message)
        if isvalid(fig)
            status_label.Text = message;
            status_label.FontColor = limit_color;
        end
    end

end


%% ========================================================================
% Defaults and parameter setup
% ========================================================================

function addProjectPaths()

    this_folder = fileparts(mfilename('fullpath'));
    candidate_roots = {pwd, this_folder, fileparts(this_folder)};

    for root_idx = 1:numel(candidate_roots)
        candidate_root = candidate_roots{root_idx};
        param_folder = fullfile(candidate_root, 'param');
        model_folder = fullfile(candidate_root, 'model');

        if isfolder(param_folder)
            addpath(param_folder);
        end

        if isfolder(model_folder)
            addpath(model_folder);
        end
    end

    if exist('LatParam', 'file') == 0
        error(['LatParam.m was not found. Put this UI in the project root ', ...
            'or in its analysis folder, or add param/ to the MATLAB path.']);
    end

end


function defaults = makeDefaults(par)

    required_fields = { ...
        'R', 'g', 'm_W', 'm_L', 'm_B', 'h', 'I_b', 'I_w', 'I_rod'};

    for idx = 1:numel(required_fields)
        name = required_fields{idx};

        if ~(isstruct(par) && isfield(par, name)) ...
                && ~(isobject(par) && isprop(par, name))
            error('LatParam() does not provide the required parameter "%s".', name);
        end
    end

    defaults.start_k1 = 931;
    defaults.start_d1 = 62;
    defaults.start_ktheta = 33;
    defaults.start_kdtheta = 7;

    defaults.min_k1 = 200;
    defaults.min_d1 = 5;
    defaults.min_ktheta = 0;
    defaults.min_kdtheta = 0;

    defaults.max_k1 = 1400;
    defaults.max_d1 = 100;
    defaults.max_ktheta = 80;
    defaults.max_kdtheta = 30;

    defaults.change_k1 = 12;
    defaults.change_d1 = 10;
    defaults.change_ktheta = 20;
    defaults.change_kdtheta = 20;

    defaults.max_trials = 500;
    defaults.random_seed = 1;
    defaults.mutation_probability = 0.65;
    defaults.min_improvement_percent = 0.05;

    defaults.theta0 = 0.10;
    defaults.theta_dot0 = 0;
    defaults.r0 = 0;
    defaults.r_dot0 = 0;
    defaults.duration = 30;
    defaults.force_limit = 30;
    defaults.rod_limit = 10;
    defaults.fall_limit_deg = 85;
    defaults.chunk_duration = 0.10;
    defaults.high_accuracy = false;

    defaults.R = par.R;
    defaults.g = par.g;
    defaults.m_W = par.m_W;
    defaults.m_L = par.m_L;
    defaults.m_B = par.m_B;
    defaults.h = par.h;
    defaults.I_b = par.I_b;
    defaults.I_w = par.I_w;
    defaults.I_rod = par.I_rod;

    defaults.w_theta = 1.0;
    defaults.w_r = 1.0;
    defaults.w_rate = 0.10;
    defaults.w_force = 0.05;
    defaults.w_terminal = 20.0;
    defaults.w_peak = 1.0;
    defaults.w_settling = 0.50;
    defaults.w_saturation = 0.50;

    defaults.plot_mode = 'Every new best';

end


function cfg = applyGains(cfg, gains)

    cfg.k1 = gains(1);
    cfg.d1 = gains(2);
    cfg.ktheta = gains(3);
    cfg.kdtheta = gains(4);

end


function [candidate, can_change] = proposeCandidate( ...
        best, minimum, maximum, change_percent, mutation_probability)

    search_scale = max([1, max(abs(minimum)), max(abs(maximum))]);
    tolerance = 100*eps(search_scale);
    available = find( ...
        maximum > minimum + tolerance & change_percent > 0);

    if isempty(available)
        candidate = best;
        can_change = false;
        return;
    end

    % A direction pointing outside a bound can be clipped back to the same
    % value. Retry a few times so a valid two-sided perturbation is almost
    % always generated, including when the best point lies on a boundary.
    maximum_attempts = 25;
    candidate = best;
    can_change = false;

    for attempt = 1:maximum_attempts %#ok<NASGU>
        selected = false(1, numel(best));
        selected(available) = ...
            rand(size(available)) < mutation_probability;

        if ~any(selected)
            chosen = available(randi(numel(available)));
            selected(chosen) = true;
        end

        random_fraction = rand(1, numel(best));
        random_sign = 2*(rand(1, numel(best)) >= 0.5) - 1;

        relative_step = ...
            abs(best).*(change_percent/100).*random_fraction;

        % The additive floor lets a zero-valued gain move away from zero.
        additive_floor = ...
            0.01*(maximum - minimum).*random_fraction;
        step_size = max(relative_step, additive_floor);

        proposal = best;
        proposal(selected) = ...
            best(selected) + random_sign(selected).*step_size(selected);
        proposal = min(maximum, max(minimum, proposal));

        if any(abs(proposal - best) > tolerance)
            candidate = proposal;
            can_change = true;
            return;
        end
    end

end


%% ========================================================================
% Nonlinear simulation
% ========================================================================

function [result, cancelled] = simulateController( ...
        cfg, progress_callback, stop_callback)

    controller = @(t, z, p) equilibriumCurveController( ...
        t, z, p, cfg.C_eq, cfg.k1, cfg.d1, ...
        cfg.ktheta, cfg.kdtheta, cfg.force_limit);
    model_rhs = @(t, z) latModelSignCorrection( ...
        t, z, cfg.par, controller);

    ode_options = odeset( ...
        'RelTol', cfg.rel_tol, ...
        'AbsTol', cfg.abs_tol, ...
        'MaxStep', cfg.max_step);

    t_all = 0;
    z_all = cfg.z0(:).';
    t_current = 0;
    z_current = cfg.z0(:);

    stopped_early = false;
    stop_message = '';
    cancelled = false;
    chunk_index = 0;

    if abs(z_current(3)) > cfg.rod_limit
        stopped_early = true;
        stop_message = 'The initial rod position exceeds the rod limit.';
    elseif abs(z_current(1)) > cfg.fall_limit
        stopped_early = true;
        stop_message = 'The initial lean angle exceeds the fall limit.';
    end

    while t_current < cfg.duration && ~stopped_early
        chunk_index = chunk_index + 1;
        t_segment_end = min( ...
            t_current + cfg.chunk_duration, cfg.duration);

        try
            [t_segment, z_segment] = ode45( ...
                model_rhs, [t_current, t_segment_end], ...
                z_current, ode_options);
        catch ME
            stopped_early = true;
            stop_message = ME.message;
            break;
        end

        if any(~isfinite(t_segment(:))) || any(~isfinite(z_segment(:)))
            stopped_early = true;
            stop_message = 'NaN or Inf appeared in the simulation.';
            break;
        end

        if numel(t_segment) > 1
            t_all = [t_all; t_segment(2:end)]; %#ok<AGROW>
            z_all = [z_all; z_segment(2:end, :)]; %#ok<AGROW>
        end

        previous_time = t_current;
        t_current = t_segment(end);
        z_current = z_segment(end, :).';

        if abs(z_current(3)) > cfg.rod_limit
            stopped_early = true;
            stop_message = 'The rod-position limit was exceeded.';
            break;
        end

        if abs(z_current(1)) > cfg.fall_limit
            stopped_early = true;
            stop_message = 'The lean-angle fall limit was exceeded.';
            break;
        end

        time_tolerance = 10*eps(max(1, abs(previous_time)));
        if t_current <= previous_time + time_tolerance
            stopped_early = true;
            stop_message = 'ode45 returned without advancing time.';
            break;
        end

        segment_tolerance = 1e-10*max(1, abs(t_segment_end));
        if t_current < t_segment_end - segment_tolerance
            stopped_early = true;
            stop_message = ...
                'ode45 returned before the requested segment endpoint.';
            break;
        end

        if mod(chunk_index, 5) == 0 || t_current >= cfg.duration
            progress_callback(t_current/cfg.duration);

            if stop_callback()
                cancelled = true;
                stopped_early = true;
                stop_message = 'Simulation cancelled by the user.';
                break;
            end
        end
    end

    theta = z_all(:, 1);
    theta_dot = z_all(:, 2);
    r = z_all(:, 3);
    r_dot = z_all(:, 4);

    r_eq = cfg.C_eq*tan(theta);
    r_eq_dot = cfg.C_eq./(cos(theta).^2).*theta_dot;
    s = r - r_eq;
    s_dot = r_dot - r_eq_dot;

    F = nan(size(t_all));
    for idx = 1:numel(t_all)
        try
            F(idx) = controller(t_all(idx), z_all(idx, :).', cfg.par);
        catch
            F(idx) = NaN;
        end
    end
    F = max(-cfg.force_limit, min(cfg.force_limit, F));

    result.t = t_all;
    result.z = z_all;
    result.r_eq = r_eq;
    result.r_eq_dot = r_eq_dot;
    result.s = s;
    result.s_dot = s_dot;
    result.F = F;
    result.stopped_early = stopped_early;
    result.stop_message = stop_message;

end


function F = equilibriumCurveController( ...
        ~, z, par, C_eq, k1, d1, ktheta, kdtheta, force_limit)

    theta = z(1);
    theta_dot = z(2);
    r = z(3);
    r_dot = z(4);

    sec2_theta = 1/(cos(theta)^2);
    s = r - C_eq*tan(theta);
    s_dot = r_dot - C_eq*sec2_theta*theta_dot;

    F_unsaturated = ...
        par.m_rod*par.g*sin(theta) ...
        - k1*s ...
        - d1*s_dot ...
        + ktheta*theta ...
        + kdtheta*theta_dot;
    F = max(-force_limit, min(force_limit, F_unsaturated));

    if ~isscalar(F) || ~isfinite(F)
        error('RandomGainAutoTunerUI:InvalidForce', ...
            'The controller returned NaN or Inf.');
    end

end


function dz = latModelSignCorrection(t, z, par, controller)

    z = z(:);
    if numel(z) ~= 4
        error(['State vector z must have four elements: ', ...
            'theta, theta_dot, r, r_dot.']);
    end

    F = controller(t, z, par);
    if ~isscalar(F) || ~isfinite(F)
        error('RandomGainAutoTunerUI:InvalidForce', ...
            'The controller force must be a finite scalar.');
    end

    theta = z(1);
    theta_dot = z(2);
    r = z(3);
    r_dot = z(4);

    R = par.R;
    g = par.g;
    m_w = par.m_W;
    m_L = par.m_L;
    m_p = par.m_B;
    h = par.h;

    M_matrix = [ ...
        2*m_L*(R^2 + r^2) + m_w*R^2 + m_p*(R + h)^2 ...
            + par.I_b + par.I_w + par.I_rod, -2*m_L*R; ...
        -2*m_L*R, 2*m_L];

    M_rightside = [ ...
        -4*m_L*r*r_dot*theta_dot ...
            + (2*m_L*R + m_w*R + m_p*(R + h))*g*sin(theta) ...
            - 2*m_L*g*r*cos(theta); ...
        F + 2*m_L*r*theta_dot^2 - 2*m_L*g*sin(theta)];

    if any(~isfinite(M_matrix(:))) || any(~isfinite(M_rightside(:)))
        error('RandomGainAutoTunerUI:InvalidModel', ...
            'The model contains NaN or Inf.');
    end

    if rcond(M_matrix) < 1e-12
        error('RandomGainAutoTunerUI:SingularMatrix', ...
            'M_matrix is singular or nearly singular.');
    end

    accel = M_matrix\M_rightside;
    if any(~isfinite(accel))
        error('RandomGainAutoTunerUI:InvalidAcceleration', ...
            'The computed acceleration contains NaN or Inf.');
    end

    dz = [theta_dot; accel(1); r_dot; accel(2)];

end


%% ========================================================================
% Performance score
% ========================================================================

function metrics = evaluatePerformance(result, cfg)

    t = result.t;
    z = result.z;
    F = result.F;

    theta = z(:, 1);
    theta_dot = z(:, 2);
    r = z(:, 3);
    r_dot = z(:, 4);

    theta_scale = max(abs(cfg.theta0), 0.05);
    r_equilibrium_initial = cfg.C_eq*tan(cfg.theta0);
    r_scale = max(abs(r_equilibrium_initial), 0.05);
    theta_rate_scale = max(theta_scale, 0.10);
    r_rate_scale = max(r_scale, 0.10);

    integration_time = max(t(end) - t(1), eps);

    theta_integral = trapz(t, (theta/theta_scale).^2)/integration_time;
    r_integral = trapz(t, (r/r_scale).^2)/integration_time;
    rate_integral = trapz(t, ...
        (theta_dot/theta_rate_scale).^2 ...
        + (r_dot/r_rate_scale).^2)/integration_time;
    force_integral = trapz(t, (F/cfg.force_limit).^2)/integration_time;

    final_normalized = [ ...
        theta(end)/theta_scale; ...
        r(end)/r_scale; ...
        theta_dot(end)/theta_rate_scale; ...
        r_dot(end)/r_rate_scale];
    terminal_error = sum(final_normalized.^2);

    max_theta = max(abs(theta));
    max_r = max(abs(r));
    peak_error = ...
        (max_theta/theta_scale)^2 + (max_r/r_scale)^2;

    saturation_indicator = abs(F) >= 0.995*cfg.force_limit;
    saturation_fraction = ...
        trapz(t, double(saturation_indicator))/integration_time;
    saturation_fraction = min(max(saturation_fraction, 0), 1);

    theta_tolerance = 0.005;
    r_tolerance = 0.005;
    theta_rate_tolerance = 0.01;
    r_rate_tolerance = 0.01;

    outside_settling_band = ...
        abs(theta) > theta_tolerance ...
        | abs(r) > r_tolerance ...
        | abs(theta_dot) > theta_rate_tolerance ...
        | abs(r_dot) > r_rate_tolerance;

    last_outside = find(outside_settling_band, 1, 'last');
    if isempty(last_outside)
        settling_time = 0;
    elseif last_outside >= numel(t)
        settling_time = cfg.duration;
    else
        settling_time = t(last_outside);
    end
    settling_fraction = min(settling_time/cfg.duration, 1);

    weights = cfg.weights;
    score = ...
        weights.theta*theta_integral ...
        + weights.r*r_integral ...
        + weights.rate*rate_integral ...
        + weights.force*force_integral ...
        + weights.terminal*terminal_error ...
        + weights.peak*peak_error ...
        + weights.settling*settling_fraction ...
        + weights.saturation*saturation_fraction;

    completed = ~result.stopped_early ...
        && t(end) >= cfg.duration - 1e-8*max(1, cfg.duration);

    if ~completed
        completion_fraction = min(max(t(end)/cfg.duration, 0), 1);
        score = score + 1e4 + 1e4*(1 - completion_fraction);
    end

    if ~isfinite(score)
        score = Inf;
    end

    finite_force = F(isfinite(F));
    if isempty(finite_force)
        max_force = NaN;
    else
        max_force = max(abs(finite_force));
    end

    metrics.score = score;
    metrics.theta_integral = theta_integral;
    metrics.r_integral = r_integral;
    metrics.rate_integral = rate_integral;
    metrics.force_integral = force_integral;
    metrics.terminal_error = terminal_error;
    metrics.peak_error = peak_error;
    metrics.max_theta = max_theta;
    metrics.max_r = max_r;
    metrics.max_force = max_force;
    metrics.saturation_fraction = saturation_fraction;
    metrics.settling_time = settling_time;
    metrics.completed = completed;

end


%% ========================================================================
% Plot helpers
% ========================================================================

function configureAxis(ax, title_text, x_text, y_text)

    title(ax, title_text, 'Interpreter', 'none');
    xlabel(ax, x_text, 'Interpreter', 'none');
    ylabel(ax, y_text, 'Interpreter', 'none');
    grid(ax, 'on');
    box(ax, 'on');
    ax.FontSize = 11;

end


function plotTimeHistory(ax, t, y, color)

    cla(ax);
    plot(ax, t, y, 'Color', color, 'LineWidth', 1.6);
    hold(ax, 'on');
    yline(ax, 0, '--k', 'LineWidth', 0.8);
    hold(ax, 'off');
    grid(ax, 'on');
    box(ax, 'on');

end


function addSymmetricLimits(ax, limit, data, color, always_show)

    finite_data = data(isfinite(data));

    if isempty(finite_data)
        data_scale = 0;
    else
        data_scale = max(abs(finite_data));
    end

    show_limit = always_show || limit <= max(5*data_scale, 1);

    if show_limit
        hold(ax, 'on');
        yline(ax, limit, '--', 'Color', color, 'LineWidth', 1.0);
        yline(ax, -limit, '--', 'Color', color, 'LineWidth', 1.0);
        hold(ax, 'off');
    end

end
