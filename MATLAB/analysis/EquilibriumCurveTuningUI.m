function EquilibriumCurveTuningUI
% EQUILIBRIUMCURVETUNINGUI Interactive tuner for the lateral controller.
%
% Put this file in the same project in which LatParam() is available, then
% run
%
%   EquilibriumCurveTuningUI
%
% Changing a numeric field automatically reruns the nonlinear simulation
% when "Auto update" is enabled. The nonlinear plant, controller, segmented
% ODE integration, singular-matrix protection, plots, and result export are
% all contained in this one file.

    % Support both common placements:
    %   project/EquilibriumCurveTuningUI.m
    %   project/analysis/EquilibriumCurveTuningUI.m
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

    source_par = LatParam();
    defaults = makeDefaults(source_par);

    line_color = [0.0000, 0.4470, 0.7410];
    warning_color = [0.8500, 0.3250, 0.0980];
    limit_color = [0.75, 0.15, 0.15];

    is_running = false;
    stop_requested = false;
    rerun_requested = false;
    last_result = [];
    last_cfg = [];

    %% Main window

    fig = uifigure( ...
        'Name', 'Equilibrium-Curve Controller Tuner', ...
        'Color', 'w', ...
        'Position', [50, 50, 1540, 900]);

    fig.CloseRequestFcn = @closeWindow;

    main_grid = uigridlayout(fig, [1, 2]);
    main_grid.ColumnWidth = {390, '1x'};
    main_grid.Padding = [10, 10, 10, 10];
    main_grid.ColumnSpacing = 10;

    %% Left control column

    left_panel = uipanel(main_grid, ...
        'Title', 'Parameters', ...
        'FontWeight', 'bold');
    left_panel.Layout.Row = 1;
    left_panel.Layout.Column = 1;

    left_grid = uigridlayout(left_panel, [7, 1]);
    left_grid.RowHeight = {34, 440, 58, 34, 70, 34, '1x'};
    left_grid.Padding = [8, 8, 8, 8];
    left_grid.RowSpacing = 7;

    intro_label = uilabel(left_grid, ...
        'Text', 'Edit a value and press Enter (or leave the field).', ...
        'FontAngle', 'italic');
    intro_label.Layout.Row = 1;

    tab_group = uitabgroup(left_grid);
    tab_group.Layout.Row = 2;

    controller_tab = uitab(tab_group, 'Title', 'Controller');
    simulation_tab = uitab(tab_group, 'Title', 'Simulation');
    physical_tab = uitab(tab_group, 'Title', 'Physical model');

    controller_grid = uigridlayout(controller_tab, [6, 2]);
    controller_grid.ColumnWidth = {'1x', 125};
    controller_grid.RowHeight = {30, 30, 30, 30, 16, '1x'};
    controller_grid.Padding = [10, 10, 10, 10];

    fields.k1 = addNumericField( ...
        controller_grid, 1, 'k1  [N/m]', defaults.k1, [-1e6, 1e6], ...
        'Equilibrium-curve position-error gain.');
    fields.d1 = addNumericField( ...
        controller_grid, 2, 'd1  [N s/m]', defaults.d1, [-1e6, 1e6], ...
        'Equilibrium-curve velocity-error gain.');
    fields.ktheta = addNumericField( ...
        controller_grid, 3, 'k_theta  [N/rad]', defaults.ktheta, [-1e6, 1e6], ...
        'Lean-angle feedback gain.');
    fields.kdtheta = addNumericField( ...
        controller_grid, 4, 'd_theta  [N s/rad]', defaults.kdtheta, [-1e6, 1e6], ...
        'Lean-rate feedback gain.');

    controller_note = uitextarea(controller_grid, ...
        'Editable', 'off', ...
        'Value', { ...
            'Controller:', ...
            'F = m_rod g sin(theta) - k1 s - d1 s_dot', ...
            '    + k_theta theta + d_theta theta_dot', ...
            's = r - C_eq tan(theta)'}, ...
        'FontName', 'Courier New');
    controller_note.Layout.Row = 6;
    controller_note.Layout.Column = [1, 2];

    simulation_grid = uigridlayout(simulation_tab, [9, 2]);
    simulation_grid.ColumnWidth = {'1x', 125};
    simulation_grid.RowHeight = repmat({30}, 1, 9);
    simulation_grid.Padding = [10, 10, 10, 10];

    fields.theta0 = addNumericField( ...
        simulation_grid, 1, 'theta(0)  [rad]', defaults.theta0, ...
        [-1.45, 1.45], 'Initial lean angle.');
    fields.theta_dot0 = addNumericField( ...
        simulation_grid, 2, 'theta_dot(0)  [rad/s]', ...
        defaults.theta_dot0, [-100, 100], 'Initial lean rate.');
    fields.r0 = addNumericField( ...
        simulation_grid, 3, 'r(0)  [m]', defaults.r0, ...
        [-100, 100], 'Initial rod position.');
    fields.r_dot0 = addNumericField( ...
        simulation_grid, 4, 'r_dot(0)  [m/s]', ...
        defaults.r_dot0, [-100, 100], 'Initial rod velocity.');
    fields.duration = addNumericField( ...
        simulation_grid, 5, 'Duration  [s]', defaults.duration, ...
        [0.01, 1000], 'Simulation end time.');
    fields.force_limit = addNumericField( ...
        simulation_grid, 6, 'Force limit  [N]', defaults.force_limit, ...
        [1e-6, 1e9], 'Symmetric force saturation limit.');
    fields.rod_limit = addNumericField( ...
        simulation_grid, 7, 'Rod limit  [m]', defaults.rod_limit, ...
        [1e-6, 1e6], 'Simulation stops when |r| exceeds this value.');
    fields.fall_limit_deg = addNumericField( ...
        simulation_grid, 8, 'Fall limit  [deg]', defaults.fall_limit_deg, ...
        [0.1, 89.9], 'Simulation stops when |theta| exceeds this value.');
    fields.chunk_duration = addNumericField( ...
        simulation_grid, 9, 'Update chunk  [s]', defaults.chunk_duration, ...
        [0.005, 5], 'Shorter chunks make the Stop button more responsive.');

    physical_grid = uigridlayout(physical_tab, [9, 2]);
    physical_grid.ColumnWidth = {'1x', 125};
    physical_grid.RowHeight = repmat({30}, 1, 9);
    physical_grid.Padding = [10, 10, 10, 10];

    fields.R = addNumericField( ...
        physical_grid, 1, 'R  [m]', defaults.R, [1e-6, 100], ...
        'Wheel radius.');
    fields.g = addNumericField( ...
        physical_grid, 2, 'g  [m/s^2]', defaults.g, [1e-6, 100], ...
        'Gravitational acceleration.');
    fields.m_W = addNumericField( ...
        physical_grid, 3, 'm_W  [kg]', defaults.m_W, [0, 1e5], ...
        'Wheel/body mass appearing in the reduced lateral model.');
    fields.m_L = addNumericField( ...
        physical_grid, 4, 'm_L (each side)  [kg]', defaults.m_L, ...
        [1e-6, 1e5], 'Moving mass on each side; m_rod = 2 m_L.');
    fields.m_B = addNumericField( ...
        physical_grid, 5, 'm_B  [kg]', defaults.m_B, [0, 1e5], ...
        'Body mass.');
    fields.h = addNumericField( ...
        physical_grid, 6, 'h  [m]', defaults.h, [-100, 100], ...
        'Body center-of-mass offset above the wheel center.');
    fields.I_b = addNumericField( ...
        physical_grid, 7, 'I_b  [kg m^2]', defaults.I_b, [0, 1e6], ...
        'Body inertia.');
    fields.I_w = addNumericField( ...
        physical_grid, 8, 'I_w  [kg m^2]', defaults.I_w, [0, 1e6], ...
        'Wheel inertia.');
    fields.I_rod = addNumericField( ...
        physical_grid, 9, 'I_rod  [kg m^2]', defaults.I_rod, [0, 1e6], ...
        'Rod assembly inertia.');

    derived_panel = uipanel(left_grid, 'Title', 'Derived values');
    derived_panel.Layout.Row = 3;
    derived_grid = uigridlayout(derived_panel, [1, 1]);
    derived_grid.Padding = [6, 2, 6, 2];
    derived_label = uilabel(derived_grid, ...
        'Text', '', ...
        'FontName', 'Courier New', ...
        'FontSize', 11);

    option_grid = uigridlayout(left_grid, [1, 2]);
    option_grid.Layout.Row = 4;
    option_grid.ColumnWidth = {'1x', '1x'};
    option_grid.Padding = [2, 0, 2, 0];

    auto_update = uicheckbox(option_grid, ...
        'Text', 'Auto update', ...
        'Value', true, ...
        'ValueChangedFcn', @autoUpdateChanged);
    auto_update.Layout.Column = 1;

    high_accuracy = uicheckbox(option_grid, ...
        'Text', 'High accuracy (slower)', ...
        'Value', false, ...
        'ValueChangedFcn', @parameterChanged);
    high_accuracy.Layout.Column = 2;

    button_grid = uigridlayout(left_grid, [2, 2]);
    button_grid.Layout.Row = 5;
    button_grid.RowHeight = {28, 28};
    button_grid.ColumnWidth = {'1x', '1x'};
    button_grid.Padding = [0, 0, 0, 0];

    run_button = uibutton(button_grid, ...
        'Text', 'Update now', ...
        'FontWeight', 'bold', ...
        'ButtonPushedFcn', @requestUpdate);
    run_button.Layout.Row = 1;
    run_button.Layout.Column = 1;

    stop_button = uibutton(button_grid, ...
        'Text', 'Stop', ...
        'Enable', 'off', ...
        'ButtonPushedFcn', @stopSimulation);
    stop_button.Layout.Row = 1;
    stop_button.Layout.Column = 2;

    reset_button = uibutton(button_grid, ...
        'Text', 'Restore defaults', ...
        'ButtonPushedFcn', @restoreDefaults);
    reset_button.Layout.Row = 2;
    reset_button.Layout.Column = 1;

    export_button = uibutton(button_grid, ...
        'Text', 'Export to workspace', ...
        'ButtonPushedFcn', @exportResult);
    export_button.Layout.Row = 2;
    export_button.Layout.Column = 2;

    status_label = uilabel(left_grid, ...
        'Text', 'Ready', ...
        'FontWeight', 'bold', ...
        'FontColor', [0.15, 0.15, 0.15]);
    status_label.Layout.Row = 6;

    summary_area = uitextarea(left_grid, ...
        'Editable', 'off', ...
        'FontName', 'Courier New', ...
        'Value', {'No result yet.'});
    summary_area.Layout.Row = 7;

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
    requestUpdate();

    %% Nested UI callbacks

    function field = addNumericField(parent, row, label_text, value, limits, tip)
        label = uilabel(parent, 'Text', label_text);
        label.Layout.Row = row;
        label.Layout.Column = 1;

        field = uieditfield(parent, ...
            'numeric', ...
            'Value', value, ...
            'Limits', limits, ...
            'ValueDisplayFormat', '%.6g', ...
            'Tooltip', tip, ...
            'ValueChangedFcn', @parameterChanged);
        field.Layout.Row = row;
        field.Layout.Column = 2;
    end

    function parameterChanged(~, ~)
        if ~isvalid(fig)
            return;
        end

        updateDerivedLabel();

        if auto_update.Value
            requestUpdate();
        else
            status_label.Text = 'Parameters changed - press Update now.';
            status_label.FontColor = warning_color;
        end
    end

    function autoUpdateChanged(source, ~)
        if source.Value
            requestUpdate();
        else
            status_label.Text = ...
                'Auto update is off - use Update now after editing.';
            status_label.FontColor = warning_color;
        end
    end

    function requestUpdate(varargin) %#ok<INUSD>
        if ~isvalid(fig)
            return;
        end

        if is_running
            rerun_requested = true;
            stop_requested = true;
            status_label.Text = 'Parameter change detected - restarting...';
            status_label.FontColor = warning_color;
            return;
        end

        rerun_requested = true;

        while rerun_requested && isvalid(fig)
            rerun_requested = false;
            stop_requested = false;
            runOneSimulation();
        end
    end

    function runOneSimulation()
        try
            cfg = readConfiguration();
        catch ME
            status_label.Text = ['Invalid parameter: ', ME.message];
            status_label.FontColor = limit_color;
            return;
        end

        is_running = true;
        run_button.Enable = 'off';
        reset_button.Enable = 'off';
        stop_button.Enable = 'on';
        status_label.Text = 'Simulating: 0%';
        status_label.FontColor = line_color;
        drawnow;

        progress_callback = @showProgress;
        stop_callback = @shouldStop;

        try
            [result, cancelled] = simulateControllerUI( ...
                cfg, progress_callback, stop_callback);
        catch ME
            result = [];
            cancelled = false;
            status_label.Text = ['Simulation error: ', ME.message];
            status_label.FontColor = limit_color;
        end

        is_running = false;

        if ~isvalid(fig)
            return;
        end

        run_button.Enable = 'on';
        reset_button.Enable = 'on';
        stop_button.Enable = 'off';

        if cancelled && rerun_requested
            return;
        end

        if isempty(result)
            return;
        end

        last_result = result;
        last_cfg = cfg;
        renderResult(result, cfg);
    end

    function showProgress(fraction)
        if ~isvalid(fig)
            return;
        end

        status_label.Text = sprintf('Simulating: %.0f%%', 100*fraction);
        drawnow limitrate;
    end

    function answer = shouldStop()
        answer = stop_requested || ~isvalid(fig);
    end

    function stopSimulation(~, ~)
        rerun_requested = false;
        stop_requested = true;

        if isvalid(fig)
            status_label.Text = 'Stopping after the current ODE chunk...';
            status_label.FontColor = warning_color;
        end
    end

    function restoreDefaults(~, ~)
        names = fieldnames(fields);

        for idx = 1:numel(names)
            name = names{idx};
            fields.(name).Value = defaults.(name);
        end

        high_accuracy.Value = false;
        updateDerivedLabel();

        if auto_update.Value
            requestUpdate();
        else
            status_label.Text = 'Defaults restored - press Update now.';
            status_label.FontColor = warning_color;
        end
    end

    function exportResult(~, ~)
        if isempty(last_result)
            status_label.Text = 'Nothing to export yet.';
            status_label.FontColor = warning_color;
            return;
        end

        assignin('base', 'equilibrium_curve_result', last_result);
        assignin('base', 'equilibrium_curve_config', last_cfg);
        status_label.Text = [ ...
            'Exported equilibrium_curve_result and ', ...
            'equilibrium_curve_config.'];
        status_label.FontColor = [0.10, 0.50, 0.20];
    end

    function closeWindow(~, ~)
        rerun_requested = false;
        stop_requested = true;
        delete(fig);
    end

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
            'm_rod = %.4g kg    G = %.4g N m    C_eq = %.4g m', ...
            m_rod, G, C_eq);
    end

    function renderResult(result, cfg)
        t = result.t;
        z = result.z;

        plotTimeHistory(ax.theta, t, z(:, 1), line_color);
        addSymmetricLimits(ax.theta, cfg.fall_limit, z(:, 1), limit_color);

        plotTimeHistory(ax.r, t, z(:, 3), line_color);
        addSymmetricLimits(ax.r, cfg.rod_limit, z(:, 3), limit_color);

        plotTimeHistory(ax.s, t, result.s, line_color);
        plotTimeHistory(ax.s_dot, t, result.s_dot, line_color);

        plotTimeHistory(ax.F, t, result.F, line_color);
        addSymmetricLimits(ax.F, cfg.force_limit, result.F, limit_color);

        cla(ax.phase);
        hold(ax.phase, 'on');
        grid(ax.phase, 'on');
        box(ax.phase, 'on');

        theta_bound = max([ ...
            0.15, ...
            1.15*max(abs(z(:, 1))), ...
            1.15*abs(cfg.theta0)]);
        theta_bound = min(theta_bound, 0.95*pi/2);
        theta_curve = linspace(-theta_bound, theta_bound, 600);

        plot(ax.phase, ...
            theta_curve, cfg.C_eq*tan(theta_curve), ...
            '--k', 'LineWidth', 1.5, ...
            'DisplayName', 'r = C_eq tan(theta)');
        plot(ax.phase, ...
            z(:, 1), z(:, 3), ...
            'Color', line_color, 'LineWidth', 1.7, ...
            'DisplayName', 'Trajectory');
        plot(ax.phase, ...
            cfg.theta0, cfg.r0, 'ko', ...
            'MarkerFaceColor', 'y', 'MarkerSize', 7, ...
            'DisplayName', 'Initial state');
        plot(ax.phase, ...
            z(end, 1), z(end, 3), 'kx', ...
            'LineWidth', 1.7, 'MarkerSize', 8, ...
            'DisplayName', 'Final state');
        legend(ax.phase, 'Location', 'best', 'Interpreter', 'none');
        hold(ax.phase, 'off');

        finite_force = result.F(isfinite(result.F));
        if isempty(finite_force)
            max_force = NaN;
        else
            max_force = max(abs(finite_force));
        end

        max_theta = max(abs(z(:, 1)));
        max_r = max(abs(z(:, 3)));
        final_state = z(end, :);

        summary_text = sprintf([ ...
            't final       = %.6f s\n', ...
            'theta final   = %+12.6f rad\n', ...
            'theta_dot     = %+12.6f rad/s\n', ...
            'r final       = %+12.6f m\n', ...
            'r_dot         = %+12.6f m/s\n', ...
            's final       = %+12.6f m\n', ...
            'max|theta|    = %.6f rad\n', ...
            'max|r|        = %.6f m\n', ...
            'max|F|        = %.6f N'], ...
            t(end), final_state(1), final_state(2), ...
            final_state(3), final_state(4), result.s(end), ...
            max_theta, max_r, max_force);

        summary_area.Value = regexp(summary_text, '\n', 'split');

        if result.stopped_early
            status_label.Text = ['Stopped early: ', result.stop_message];
            status_label.FontColor = limit_color;
        else
            status_label.Text = sprintf( ...
                'Completed: k1 = %.4g, d1 = %.4g, k_theta = %.4g, d_theta = %.4g', ...
                cfg.k1, cfg.d1, cfg.ktheta, cfg.kdtheta);
            status_label.FontColor = [0.10, 0.50, 0.20];
        end
    end

end


%% ========================================================================
% Simulation and model functions
% ========================================================================

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

    defaults.k1 = 700;
    defaults.d1 = 50;
    defaults.ktheta = 20;
    defaults.kdtheta = 3;

    defaults.theta0 = 0.10;
    defaults.theta_dot0 = 0;
    defaults.r0 = 0;
    defaults.r_dot0 = 0;
    defaults.duration = 30;
    defaults.force_limit = 100000;
    defaults.rod_limit = 10;
    defaults.fall_limit_deg = 85;
    defaults.chunk_duration = 0.10;

    defaults.R = par.R;
    defaults.g = par.g;
    defaults.m_W = par.m_W;
    defaults.m_L = par.m_L;
    defaults.m_B = par.m_B;
    defaults.h = par.h;
    defaults.I_b = par.I_b;
    defaults.I_w = par.I_w;
    defaults.I_rod = par.I_rod;

end


function [result, cancelled] = simulateControllerUI( ...
        cfg, progress_callback, stop_callback)

    controller = @(t, z, p) equilibriumCurveControllerUI( ...
        t, z, p, cfg.C_eq, cfg.k1, cfg.d1, ...
        cfg.ktheta, cfg.kdtheta, cfg.force_limit);

    model_rhs = @(t, z) latModelSignCorrectionUI( ...
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

    while t_current < cfg.duration
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


function F = equilibriumCurveControllerUI( ...
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
        error('EquilibriumCurveTuningUI:InvalidForce', ...
            'The controller returned NaN or Inf.');
    end

end


function dz = latModelSignCorrectionUI(t, z, par, controller)
% Same standard-form nonlinear model as LatModel_SignCorrection.

    z = z(:);

    if numel(z) ~= 4
        error(['State vector z must have four elements: ', ...
            'theta, theta_dot, r, r_dot.']);
    end

    F = controller(t, z, par);

    if ~isscalar(F) || ~isfinite(F)
        error('EquilibriumCurveTuningUI:InvalidForce', ...
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
        error('EquilibriumCurveTuningUI:InvalidModel', ...
            'The model contains NaN or Inf.');
    end

    if rcond(M_matrix) < 1e-12
        error('EquilibriumCurveTuningUI:SingularMatrix', ...
            'M_matrix is singular or nearly singular.');
    end

    accel = M_matrix\M_rightside;

    if any(~isfinite(accel))
        error('EquilibriumCurveTuningUI:InvalidAcceleration', ...
            'The computed acceleration contains NaN or Inf.');
    end

    dz = [theta_dot; accel(1); r_dot; accel(2)];

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


function addSymmetricLimits(ax, limit, data, color)

    finite_data = data(isfinite(data));

    if isempty(finite_data)
        data_scale = 0;
    else
        data_scale = max(abs(finite_data));
    end

    % Do not destroy the useful vertical scale when a deliberately huge
    % limit is being used to represent an effectively unlimited actuator.
    show_limit = limit <= max(5*data_scale, 1);

    if show_limit
        hold(ax, 'on');
        yline(ax, limit, '--', 'Color', color, 'LineWidth', 1.0);
        yline(ax, -limit, '--', 'Color', color, 'LineWidth', 1.0);
        hold(ax, 'off');
    end

end
