function RodCBF_UI
% RODCBF_UI Interactive PD + second-order HOCBF simulation for a flat rod.
%
% Simplified plant:
%       m_r * r_ddot = F
%
% Nominal PD controller:
%       F_nom = Kp*(r_ref - r) - Kd*r_dot
%
% Symmetric safety constraint:
%       -r_max <= r <= r_max
%
% The second-order HOCBF produces a lower and upper admissible force
% bound. The applied force is the projection of F_nom onto the CBF and
% actuator-force interval.
%
% Run this file from the MATLAB command window with:
%       RodCBF_UI

% This file uses only base MATLAB UI and numerical functionality.


    %% Default parameters

    defaults.mass       = 2.3;      % [kg]
    defaults.rMax       = 0.10;     % [m], software safety limit
    defaults.forceMax   = 30.0;     % [N]
    defaults.alpha0     = 6.0;      % [1/s]
    defaults.alpha1     = 6.0;      % [1/s]
    defaults.Kp         = 120.0;    % [N/m]
    defaults.Kd         = 15.0;     % [N*s/m]
    defaults.rRef       = 0.08;     % [m], inside the software safety limit
    defaults.r0         = -0.08;    % [m], gives a useful PD overshoot test
    defaults.v0         = 0.00;     % [m/s]
    defaults.tFinal     = 3.0;      % [s]
    defaults.dt         = 0.002;    % [s]
    defaults.enableCBF  = true;

    % Plot-layer visibility.
    defaults.showPositionLimits        = true;
    defaults.showBrakingRegion         = true;
    defaults.showFirstLevelSet         = true;
    defaults.showFirstLevelBoundaries  = true;
    defaults.showSecondOrderSet        = true;
    defaults.showPDRegion              = true;
    defaults.showPDEllipse             = true;
    defaults.showHOCBFForceBounds      = true;
    defaults.showActuatorForceLimits   = true;
    defaults.showMaxForceRecovery      = true;


    %% Build user interface

    fig = uifigure( ...
        'Name', 'Flat-Rod PD + HOCBF Tuner', ...
        'Position', [80, 40, 1550, 920]);

    mainGrid = uigridlayout(fig, [1, 2]);
    mainGrid.ColumnWidth = {380, '1x'};
    mainGrid.RowHeight = {'1x'};
    mainGrid.Padding = [8, 8, 8, 8];
    mainGrid.ColumnSpacing = 8;

    controlPanel = uipanel(mainGrid, 'Title', 'Simulation Parameters');
    controlPanel.Layout.Row = 1;
    controlPanel.Layout.Column = 1;

    controlGrid = uigridlayout(controlPanel, [16, 2]);
    controlGrid.ColumnWidth = {145, '1x'};
    controlGrid.RowHeight = [ ...
        repmat({27}, 1, 13), {32}, {175}, {'1x'}];
    controlGrid.Padding = [8, 8, 8, 8];
    controlGrid.RowSpacing = 5;

    massField = addNumericField(1,  'Rod mass m_r [kg]',       defaults.mass);
    rMaxField = addNumericField(2,  'Safe limit r_max [m]',    defaults.rMax);
    forceField = addNumericField(3, 'Force limit F_max [N]',   defaults.forceMax);
    alpha0Field = addNumericField(4,'alpha_0 [1/s]',           defaults.alpha0);
    alpha1Field = addNumericField(5,'alpha_1 [1/s]',           defaults.alpha1);
    kpField = addNumericField(6,    'PD gain K_p [N/m]',       defaults.Kp);
    kdField = addNumericField(7,    'PD gain K_d [N s/m]',     defaults.Kd);
    rRefField = addNumericField(8,  'Reference r_ref [m]',     defaults.rRef);
    r0Field = addNumericField(9,    'Initial r(0) [m]',        defaults.r0);
    v0Field = addNumericField(10,   'Initial r_dot(0) [m/s]',  defaults.v0);
    tFinalField = addNumericField(11,'Final time [s]',         defaults.tFinal);
    dtField = addNumericField(12,   'Integration step [s]',    defaults.dt);

    cbfLabel = uilabel(controlGrid, 'Text', 'CBF safety filter');
    cbfLabel.Layout.Row = 13;
    cbfLabel.Layout.Column = 1;

    cbfCheck = uicheckbox(controlGrid, ...
        'Text', 'Enabled', ...
        'Value', defaults.enableCBF);
    cbfCheck.Layout.Row = 13;
    cbfCheck.Layout.Column = 2;

    runButton = uibutton(controlGrid, ...
        'Text', 'Run / Update', ...
        'ButtonPushedFcn', @runSimulation);
    runButton.Layout.Row = 14;
    runButton.Layout.Column = 1;

    resetButton = uibutton(controlGrid, ...
        'Text', 'Reset Defaults', ...
        'ButtonPushedFcn', @resetDefaults);
    resetButton.Layout.Row = 14;
    resetButton.Layout.Column = 2;

    displayPanel = uipanel(controlGrid, ...
        'Title', 'Region / Boundary Display');
    displayPanel.Layout.Row = 15;
    displayPanel.Layout.Column = [1, 2];

    displayGrid = uigridlayout(displayPanel, [5, 2]);
    displayGrid.ColumnWidth = {'1x', '1x'};
    displayGrid.RowHeight = repmat({'1x'}, 1, 5);
    displayGrid.Padding = [6, 4, 6, 4];
    displayGrid.RowSpacing = 2;
    displayGrid.ColumnSpacing = 6;

    positionLimitsCheck = addDisplayCheck( ...
        1, 1, 'Position limits', defaults.showPositionLimits);
    brakingRegionCheck = addDisplayCheck( ...
        1, 2, 'Braking region', defaults.showBrakingRegion);
    firstLevelSetCheck = addDisplayCheck( ...
        2, 1, 'First-level set', defaults.showFirstLevelSet);
    firstLevelBoundsCheck = addDisplayCheck( ...
        2, 2, 'First-level bounds', defaults.showFirstLevelBoundaries);
    secondOrderSetCheck = addDisplayCheck( ...
        3, 1, 'Second-order set', defaults.showSecondOrderSet);
    pdRegionCheck = addDisplayCheck( ...
        3, 2, 'PD unchanged set', defaults.showPDRegion);
    pdEllipseCheck = addDisplayCheck( ...
        4, 1, 'PD invariant ellipse', defaults.showPDEllipse);
    hocbfForceBoundsCheck = addDisplayCheck( ...
        4, 2, 'HOCBF force bounds', defaults.showHOCBFForceBounds);
    actuatorForceLimitsCheck = addDisplayCheck( ...
        5, 1, 'Actuator force limits', defaults.showActuatorForceLimits);
    maxForceRecoveryCheck = addDisplayCheck( ...
        5, 2, '+Fmax recovery set', defaults.showMaxForceRecovery);

    statusArea = uitextarea(controlGrid, ...
        'Editable', 'off', ...
        'FontName', 'Monospaced');
    statusArea.Layout.Row = 16;
    statusArea.Layout.Column = [1, 2];

    plotGrid = uigridlayout(mainGrid, [2, 2]);
    plotGrid.Layout.Row = 1;
    plotGrid.Layout.Column = 2;
    plotGrid.RowHeight = {'1x', '1x'};
    plotGrid.ColumnWidth = {'1x', '1x'};
    plotGrid.Padding = [0, 0, 0, 0];
    plotGrid.RowSpacing = 8;
    plotGrid.ColumnSpacing = 8;

    phaseAxes = uiaxes(plotGrid);
    phaseAxes.Layout.Row = 1;
    phaseAxes.Layout.Column = 1;

    rTimeAxes = uiaxes(plotGrid);
    rTimeAxes.Layout.Row = 1;
    rTimeAxes.Layout.Column = 2;

    vTimeAxes = uiaxes(plotGrid);
    vTimeAxes.Layout.Row = 2;
    vTimeAxes.Layout.Column = 1;

    forceAxes = uiaxes(plotGrid);
    forceAxes.Layout.Row = 2;
    forceAxes.Layout.Column = 2;

    editableControls = { ...
        massField, rMaxField, forceField, ...
        alpha0Field, alpha1Field, kpField, kdField, ...
        rRefField, r0Field, v0Field, tFinalField, dtField};

    for idx = 1:numel(editableControls)
        editableControls{idx}.ValueChangedFcn = @parameterChanged;
    end

    cbfCheck.ValueChangedFcn = @parameterChanged;

    displayControls = { ...
        positionLimitsCheck, brakingRegionCheck, ...
        firstLevelSetCheck, firstLevelBoundsCheck, ...
        secondOrderSetCheck, pdRegionCheck, pdEllipseCheck, ...
        hocbfForceBoundsCheck, actuatorForceLimitsCheck, ...
        maxForceRecoveryCheck};

    for idx = 1:numel(displayControls)
        displayControls{idx}.ValueChangedFcn = @parameterChanged;
    end

    % The recovery-set calculation is more expensive than one trajectory.
    % Cache it because changing PD gains or plot visibility does not change
    % this controller-independent stress test.
    maximumForceRecoveryCache.key = [];
    maximumForceRecoveryCache.data = [];

    runSimulation();


    %% UI helper functions

    function field = addNumericField(row, labelText, defaultValue)
        label = uilabel(controlGrid, 'Text', labelText);
        label.Layout.Row = row;
        label.Layout.Column = 1;

        field = uieditfield(controlGrid, ...
            'numeric', ...
            'Value', defaultValue);
        field.Layout.Row = row;
        field.Layout.Column = 2;
    end


    function checkbox = addDisplayCheck(row, column, labelText, defaultValue)
        checkbox = uicheckbox(displayGrid, ...
            'Text', labelText, ...
            'Value', defaultValue);
        checkbox.Layout.Row = row;
        checkbox.Layout.Column = column;
    end


    function parameterChanged(~, ~)
        runSimulation();
    end


    function resetDefaults(varargin)
        massField.Value   = defaults.mass;
        rMaxField.Value   = defaults.rMax;
        forceField.Value  = defaults.forceMax;
        alpha0Field.Value = defaults.alpha0;
        alpha1Field.Value = defaults.alpha1;
        kpField.Value     = defaults.Kp;
        kdField.Value     = defaults.Kd;
        rRefField.Value   = defaults.rRef;
        r0Field.Value     = defaults.r0;
        v0Field.Value     = defaults.v0;
        tFinalField.Value = defaults.tFinal;
        dtField.Value     = defaults.dt;
        cbfCheck.Value    = defaults.enableCBF;

        positionLimitsCheck.Value = defaults.showPositionLimits;
        brakingRegionCheck.Value = defaults.showBrakingRegion;
        firstLevelSetCheck.Value = defaults.showFirstLevelSet;
        firstLevelBoundsCheck.Value = defaults.showFirstLevelBoundaries;
        secondOrderSetCheck.Value = defaults.showSecondOrderSet;
        pdRegionCheck.Value = defaults.showPDRegion;
        pdEllipseCheck.Value = defaults.showPDEllipse;
        hocbfForceBoundsCheck.Value = defaults.showHOCBFForceBounds;
        actuatorForceLimitsCheck.Value = ...
            defaults.showActuatorForceLimits;
        maxForceRecoveryCheck.Value = defaults.showMaxForceRecovery;

        runSimulation();
    end


    %% Simulation callback

    function runSimulation(varargin)
        try
            p = readParameters();

            statusArea.Value = {'Running simulation...'};
            drawnow limitrate nocallbacks;

            % Trajectory with the selected CBF setting.
            [t, x, logData] = simulateRod(p);

            % Separate nominal-PD trajectory with the CBF disabled. This is
            % always simulated so that it can be compared in state space.
            nominalParameters = p;
            nominalParameters.enableCBF = false;
            [tNominal, xNominal, nominalLogData] = ...
                simulateRod(nominalParameters);

            % Lyapunov ellipsoid for the unsaturated nominal PD dynamics.
            ellipseData = computePDEllipsoid(p);

            % Numerical basin from which a constant +F_max nominal command
            % can be safely overridden by the HOCBF filter.
            if p.showMaxForceRecovery
                recoveryData = computeMaximumForceRecoveryRegion(p);
            else
                recoveryData = emptyRecoveryData();
            end

            updatePlots( ...
                t, x, logData, ...
                tNominal, xNominal, nominalLogData, ...
                ellipseData, recoveryData, p);
            updateStatus( ...
                x, xNominal, logData, ellipseData, recoveryData, p);

        catch ME
            statusArea.Value = { ...
                'Simulation error:', ...
                ME.message};
            statusArea.BackgroundColor = [1.00, 0.88, 0.88];
        end
    end


    function p = readParameters()
        p.mass      = massField.Value;
        p.rMax      = rMaxField.Value;
        p.forceMax  = forceField.Value;
        p.alpha0    = alpha0Field.Value;
        p.alpha1    = alpha1Field.Value;
        p.Kp        = kpField.Value;
        p.Kd        = kdField.Value;
        p.rRef      = rRefField.Value;
        p.r0        = r0Field.Value;
        p.v0        = v0Field.Value;
        p.tFinal    = tFinalField.Value;
        p.dt        = dtField.Value;
        p.enableCBF = cbfCheck.Value;

        p.showPositionLimits = positionLimitsCheck.Value;
        p.showBrakingRegion = brakingRegionCheck.Value;
        p.showFirstLevelSet = firstLevelSetCheck.Value;
        p.showFirstLevelBoundaries = firstLevelBoundsCheck.Value;
        p.showSecondOrderSet = secondOrderSetCheck.Value;
        p.showPDRegion = pdRegionCheck.Value;
        p.showPDEllipse = pdEllipseCheck.Value;
        p.showHOCBFForceBounds = hocbfForceBoundsCheck.Value;
        p.showActuatorForceLimits = actuatorForceLimitsCheck.Value;
        p.showMaxForceRecovery = maxForceRecoveryCheck.Value;

        if ~isfinite(p.mass) || p.mass <= 0
            error('Rod mass must be positive.');
        end

        if ~isfinite(p.rMax) || p.rMax <= 0
            error('The safety limit r_max must be positive.');
        end

        if ~isfinite(p.forceMax) || p.forceMax <= 0
            error('The actuator-force limit must be positive.');
        end

        if ~isfinite(p.alpha0) || p.alpha0 <= 0 || ...
                ~isfinite(p.alpha1) || p.alpha1 <= 0
            error('alpha_0 and alpha_1 must both be positive.');
        end

        if ~isfinite(p.Kp) || ~isfinite(p.Kd)
            error('The PD gains must be finite.');
        end

        if ~isfinite(p.tFinal) || p.tFinal <= 0
            error('Final time must be positive.');
        end

        if ~isfinite(p.dt) || p.dt <= 0
            error('Integration step must be positive.');
        end

        if ~isfinite(p.r0) || ~isfinite(p.v0) || ~isfinite(p.rRef)
            error('Initial conditions and reference must be finite.');
        end
    end


    %% Fixed-step simulation

    function [t, x, logData] = simulateRod(p)
        numberOfSteps = max(1, ceil(p.tFinal/p.dt));

        if numberOfSteps > 2e5
            error(['The requested simulation has more than 200,000 ', ...
                   'steps. Increase the integration step.']);
        end

        t = linspace(0, p.tFinal, numberOfSteps + 1).';
        actualStep = t(2) - t(1);

        x = zeros(numberOfSteps + 1, 2);
        x(1, :) = [p.r0, p.v0];

        for k = 1:numberOfSteps
            currentState = x(k, :).';

            % Fourth-order Runge-Kutta integration.
            k1 = rodDynamics(currentState, p);
            k2 = rodDynamics(currentState + 0.5*actualStep*k1, p);
            k3 = rodDynamics(currentState + 0.5*actualStep*k2, p);
            k4 = rodDynamics(currentState + actualStep*k3, p);

            nextState = currentState ...
                + actualStep*(k1 + 2*k2 + 2*k3 + k4)/6;

            if any(~isfinite(nextState)) || any(abs(nextState) > 1e4)
                error('The simulation diverged. Check the gains and step size.');
            end

            x(k + 1, :) = nextState.';
        end

        sampleCount = size(x, 1);

        logData.force = zeros(sampleCount, 1);
        logData.nominalForce = zeros(sampleCount, 1);
        logData.cbfLower = zeros(sampleCount, 1);
        logData.cbfUpper = zeros(sampleCount, 1);
        logData.feasible = false(sampleCount, 1);
        logData.cbfActive = false(sampleCount, 1);
        logData.forceSaturated = false(sampleCount, 1);

        for k = 1:sampleCount
            [logData.force(k), ...
             logData.nominalForce(k), ...
             logData.cbfLower(k), ...
             logData.cbfUpper(k), ...
             logData.feasible(k), ...
             logData.cbfActive(k), ...
             logData.forceSaturated(k)] = ...
                controlAtState(x(k, 1), x(k, 2), p);
        end
    end


    function dx = rodDynamics(x, p)
        r = x(1);
        rDot = x(2);

        appliedForce = controlAtState(r, rDot, p);

        dx = [ ...
            rDot;
            appliedForce/p.mass];
    end


    %% PD controller and HOCBF safety filter

    function [appliedForce, nominalForce, cbfLower, cbfUpper, ...
              feasible, cbfActive, forceSaturated] = ...
              controlAtState(r, rDot, p)

        % Nominal PD control.
        nominalForce = p.Kp*(p.rRef - r) - p.Kd*rDot;

        alphaSum = p.alpha0 + p.alpha1;
        alphaProduct = p.alpha0*p.alpha1;

        % Negative-side HOCBF: F >= cbfLower.
        cbfLower = p.mass*( ...
            -alphaSum*rDot ...
            -alphaProduct*(p.rMax + r));

        % Positive-side HOCBF: F <= cbfUpper.
        cbfUpper = p.mass*( ...
            -alphaSum*rDot ...
            +alphaProduct*(p.rMax - r));

        combinedLower = max(cbfLower, -p.forceMax);
        combinedUpper = min(cbfUpper,  p.forceMax);

        feasible = (combinedLower <= combinedUpper);

        cbfActive = p.enableCBF && ...
            (nominalForce < cbfLower - 1e-9 || ...
             nominalForce > cbfUpper + 1e-9);

        if p.enableCBF
            if feasible
                % Closed-form solution of the one-dimensional CBF-QP.
                appliedForce = min( ...
                    max(nominalForce, combinedLower), ...
                    combinedUpper);
            else
                % The actuator cannot satisfy the CBF inequality. Apply the
                % closest achievable force in the required direction.
                if cbfLower > p.forceMax
                    appliedForce = p.forceMax;
                elseif cbfUpper < -p.forceMax
                    appliedForce = -p.forceMax;
                else
                    appliedForce = min( ...
                        max(nominalForce, -p.forceMax), ...
                        p.forceMax);
                end
            end
        else
            % PD control with only the physical actuator-force saturation.
            appliedForce = min( ...
                max(nominalForce, -p.forceMax), ...
                p.forceMax);
        end

        forceSaturated = abs(appliedForce) >= p.forceMax - 1e-8;
    end


    %% Nominal-PD Lyapunov ellipsoid

    function ellipseData = computePDEllipsoid(p)
        % The ellipse is a sublevel set
        %
        %       V(e) = e'*P*e <= rho,
        %
        % where e = [r-r_ref; r_dot].  P is obtained from
        %
        %       A_cl'*P + P*A_cl = -Q.
        %
        % rho is reduced until the entire ellipse lies inside the rod
        % position limit, actuator-force limit, first HOCBF set, and the
        % region in which the nominal PD force satisfies both HOCBF force
        % inequalities.  Therefore, when this ellipse exists, the nominal
        % PD controller can remain active without CBF modification inside it.

        ellipseData.available = false;
        ellipseData.r = [];
        ellipseData.v = [];
        ellipseData.rho = NaN;
        ellipseData.limitingConstraint = '';
        ellipseData.reason = '';

        if p.Kp <= 0 || p.Kd <= 0
            ellipseData.reason = ...
                'K_p and K_d must be positive for a stable PD ellipse.';
            return;
        end

        if abs(p.rRef) >= p.rMax
            ellipseData.reason = ...
                'r_ref must lie strictly inside the rod safety limits.';
            return;
        end

        closedLoopA = [ ...
            0,              1;
            -p.Kp/p.mass,  -p.Kd/p.mass];

        if any(real(eig(closedLoopA)) >= 0)
            ellipseData.reason = ...
                'The nominal PD closed-loop matrix is not Hurwitz.';
            return;
        end

        % Q only selects the shape of the Lyapunov function. The velocity
        % scaling prevents the two state units from being weighted blindly.
        maximumAcceleration = p.forceMax/p.mass;
        velocityScale = max( ...
            p.alpha0*p.rMax, ...
            sqrt(2*maximumAcceleration*p.rMax));

        Q = diag([ ...
            1/p.rMax^2, ...
            1/max(velocityScale, 1e-8)^2]);

        identity2 = eye(2);
        lyapunovOperator = ...
            kron(identity2, closedLoopA.') ...
            + kron(closedLoopA.', identity2);

        if rcond(lyapunovOperator) < 1e-12
            ellipseData.reason = ...
                'The Lyapunov equation is numerically singular.';
            return;
        end

        Pvector = -lyapunovOperator\Q(:);
        P = reshape(Pvector, 2, 2);
        P = 0.5*(P + P.');

        if min(eig(P)) <= 0
            ellipseData.reason = ...
                'The computed Lyapunov matrix is not positive definite.';
            return;
        end

        inverseP = P\eye(2);

        % 1) Keep the complete ellipse inside |r| <= r_max.
        positionMargin = min( ...
            p.rMax - p.rRef, ...
            p.rMax + p.rRef);
        rhoPosition = positionMargin^2/inverseP(1, 1);

        % 2) Keep the nominal PD force inside |F| <= F_max.
        forceDirection = [p.Kp; p.Kd];
        rhoForce = squaredLinearConstraintBound( ...
            p.forceMax, forceDirection, inverseP);

        alphaSum = p.alpha0 + p.alpha1;
        alphaProduct = p.alpha0*p.alpha1;

        % For e = [r-r_ref; r_dot], both HOCBF-force slacks have the same
        % direction with opposite signs.
        cbfForceDirection = [ ...
            p.mass*alphaProduct - p.Kp;
            p.mass*alphaSum - p.Kd];

        lowerForceSlackAtEquilibrium = ...
            p.mass*alphaProduct*(p.rMax + p.rRef);
        upperForceSlackAtEquilibrium = ...
            p.mass*alphaProduct*(p.rMax - p.rRef);

        rhoCBFLowerForce = squaredLinearConstraintBound( ...
            lowerForceSlackAtEquilibrium, ...
            cbfForceDirection, inverseP);
        rhoCBFUpperForce = squaredLinearConstraintBound( ...
            upperForceSlackAtEquilibrium, ...
            cbfForceDirection, inverseP);

        % 3) Keep the ellipse inside both first-level HOCBF conditions.
        upperPsiDirection = [-p.alpha0; -1];
        lowerPsiDirection = [ p.alpha0;  1];

        upperPsiAtEquilibrium = ...
            p.alpha0*(p.rMax - p.rRef);
        lowerPsiAtEquilibrium = ...
            p.alpha0*(p.rMax + p.rRef);

        rhoUpperPsi = squaredLinearConstraintBound( ...
            upperPsiAtEquilibrium, upperPsiDirection, inverseP);
        rhoLowerPsi = squaredLinearConstraintBound( ...
            lowerPsiAtEquilibrium, lowerPsiDirection, inverseP);

        rhoCandidates = [ ...
            rhoPosition;
            rhoForce;
            rhoCBFLowerForce;
            rhoCBFUpperForce;
            rhoUpperPsi;
            rhoLowerPsi];

        constraintNames = { ...
            'rod position';
            'actuator force';
            'lower HOCBF force';
            'upper HOCBF force';
            'upper first-level HOCBF';
            'lower first-level HOCBF'};

        [rho, limitingIndex] = min(rhoCandidates);

        if ~isfinite(rho) || rho <= 0
            ellipseData.reason = ...
                'No positive Lyapunov sublevel set satisfies all limits.';
            return;
        end

        [eigenvectors, eigenvalueMatrix] = eig(P);
        eigenvalues = diag(eigenvalueMatrix);
        angles = linspace(0, 2*pi, 400);
        unitCircle = [cos(angles); sin(angles)];
        errorBoundary = eigenvectors ...
            * diag(sqrt(rho./eigenvalues)) ...
            * unitCircle;

        ellipseData.available = true;
        ellipseData.r = p.rRef + errorBoundary(1, :);
        ellipseData.v = errorBoundary(2, :);
        ellipseData.rho = rho;
        ellipseData.limitingConstraint = ...
            constraintNames{limitingIndex};
        ellipseData.reason = '';
    end


    function rho = squaredLinearConstraintBound(offset, direction, inverseP)
        % For an ellipsoid e'*P*e <= rho, the largest value of a'*e is
        % sqrt(rho*a'*inv(P)*a). Requiring offset + a'*e >= 0 for the
        % entire ellipsoid gives rho <= offset^2/(a'*inv(P)*a).

        if offset < 0
            rho = -Inf;
            return;
        end

        denominator = direction.'*inverseP*direction;

        if denominator <= 1e-14
            rho = Inf;
        else
            rho = offset^2/denominator;
        end
    end


    %% Controller-independent second-order HOCBF-feasible set

    function polygon = computeHOCBFFeasiblePolygon(p)
        % Projection of the second-order HOCBF constraints onto the
        % (r, r_dot) plane when the force may be chosen anywhere in
        % [-F_max, F_max].  With
        %
        %   s = alpha_0 + alpha_1,  q = alpha_0*alpha_1,
        %
        % force feasibility is equivalent to
        %
        %   |s*r_dot + q*r| <= F_max/m_r + q*r_max.
        %
        % The plotted set also includes the position and first-level HOCBF
        % constraints.  Every inequality is affine, so the result is an
        % exact polygon obtained by half-plane clipping.

        alphaSum = p.alpha0 + p.alpha1;
        alphaProduct = p.alpha0*p.alpha1;
        maximumAcceleration = p.forceMax/p.mass;
        secondOrderBound = ...
            maximumAcceleration + alphaProduct*p.rMax;

        halfPlaneA = [ ...
             1,  0;
            -1,  0;
             p.alpha0,  1;
            -p.alpha0, -1;
             alphaProduct,  alphaSum;
            -alphaProduct, -alphaSum];

        halfPlaneB = [ ...
            p.rMax;
            p.rMax;
            p.alpha0*p.rMax;
            p.alpha0*p.rMax;
            secondOrderBound;
            secondOrderBound];

        velocityLimit = 2*p.alpha0*p.rMax + 1e-9;
        polygon = [ ...
            -p.rMax, -velocityLimit;
             p.rMax, -velocityLimit;
             p.rMax,  velocityLimit;
            -p.rMax,  velocityLimit];

        for constraintIndex = 1:size(halfPlaneA, 1)
            polygon = clipPolygonWithHalfPlane( ...
                polygon, ...
                halfPlaneA(constraintIndex, :).', ...
                halfPlaneB(constraintIndex));

            if isempty(polygon)
                return;
            end
        end
    end


    %% Numerical recovery set for a constant +F_max nominal command

    function recoveryData = computeMaximumForceRecoveryRegion(p)
        % The actual actuator force is not held at +F_max. Instead, the
        % nominal request is held at +F_max and the HOCBF filter is allowed
        % to reduce or reverse it:
        %
        %   F_nom(t) = +F_max,
        %   F_applied = clip(F_nom, F_lower, F_upper).
        %
        % A grid point is classified as recovered when:
        %   1) it starts in the physical braking region,
        %   2) |r| <= r_max and the HOCBF-QP remains feasible throughout
        %      the finite simulation horizon, and
        %   3) the final point lies in the first- and second-order HOCBF
        %      feasible set.
        %
        % This is a finite-horizon numerical approximation, not an analytic
        % controlled-invariant-set certificate.

        gridCount = 101;
        maximumRecoverySteps = 3000;
        recoveryStep = max([ ...
            p.dt, 0.005, p.tFinal/maximumRecoverySteps]);
        numberOfRecoverySteps = max(1, ceil(p.tFinal/recoveryStep));
        recoveryStep = p.tFinal/numberOfRecoverySteps;

        cacheKey = [ ...
            p.mass, p.rMax, p.forceMax, ...
            p.alpha0, p.alpha1, p.tFinal, ...
            recoveryStep, gridCount];

        if isequaln(maximumForceRecoveryCache.key, cacheKey)
            recoveryData = maximumForceRecoveryCache.data;
            return;
        end

        maximumAcceleration = p.forceMax/p.mass;
        velocityExtent = sqrt(4*maximumAcceleration*p.rMax);

        rVector = linspace(-p.rMax, p.rMax, gridCount);
        vVector = linspace(-velocityExtent, velocityExtent, gridCount);
        [rGrid, vGrid] = meshgrid(rVector, vVector);

        brakingUpper = sqrt(max( ...
            0, 2*maximumAcceleration*(p.rMax - rGrid)));
        brakingLower = -sqrt(max( ...
            0, 2*maximumAcceleration*(p.rMax + rGrid)));

        positionTolerance = max(1e-7, 1e-3*p.rMax);
        velocityTolerance = max(1e-7, p.alpha0*positionTolerance);
        feasibilityTolerance = 1e-9;

        initialCandidate = ...
            vGrid <= brakingUpper + velocityTolerance & ...
            vGrid >= brakingLower - velocityTolerance;

        successful = initialCandidate;
        rState = rGrid;
        vState = vGrid;

        % Explicit midpoint integration provides a useful compromise between
        % speed and boundary accuracy for the full vectorized state grid.
        for stepIndex = 1:numberOfRecoverySteps %#ok<NASGU>
            [acceleration1, feasible1] = ...
                maximumForceCBFAcceleration(rState, vState, p);

            midpointR = rState + 0.5*recoveryStep*vState;
            midpointV = vState + 0.5*recoveryStep*acceleration1;

            [acceleration2, feasible2] = ...
                maximumForceCBFAcceleration(midpointR, midpointV, p);

            rState = rState + recoveryStep*midpointV;
            vState = vState + recoveryStep*acceleration2;

            successful = successful & feasible1 & feasible2 & ...
                abs(rState) <= p.rMax + positionTolerance;
        end

        alphaSum = p.alpha0 + p.alpha1;
        alphaProduct = p.alpha0*p.alpha1;

        finalPsiPositive = ...
            -vState + p.alpha0*(p.rMax - rState);
        finalPsiNegative = ...
             vState + p.alpha0*(p.rMax + rState);
        finalSecondOrderExpression = ...
            alphaSum*vState + alphaProduct*rState;
        finalSecondOrderBound = ...
            maximumAcceleration + alphaProduct*p.rMax;

        finalHOCBFSet = ...
            abs(rState) <= p.rMax + positionTolerance & ...
            finalPsiPositive >= -velocityTolerance & ...
            finalPsiNegative >= -velocityTolerance & ...
            abs(finalSecondOrderExpression) <= ...
                finalSecondOrderBound + feasibilityTolerance;

        recoveredMask = successful & finalHOCBFSet;
        candidateCount = nnz(initialCandidate);
        recoveredCount = nnz(recoveredMask);

        recoveryData.available = recoveredCount > 0;
        recoveryData.rGrid = rGrid;
        recoveryData.vGrid = vGrid;
        recoveryData.mask = recoveredMask;
        recoveryData.initialCandidate = initialCandidate;
        recoveryData.candidateCount = candidateCount;
        recoveryData.recoveredCount = recoveredCount;
        recoveryData.recoveredFraction = ...
            recoveredCount/max(candidateCount, 1);
        recoveryData.horizon = p.tFinal;
        recoveryData.integrationStep = recoveryStep;
        recoveryData.gridCount = gridCount;

        maximumForceRecoveryCache.key = cacheKey;
        maximumForceRecoveryCache.data = recoveryData;
    end


    function [acceleration, feasible] = ...
            maximumForceCBFAcceleration(r, v, p)
        % Vectorized HOCBF filter for the adversarial nominal request
        % F_nom = +F_max.

        alphaSum = p.alpha0 + p.alpha1;
        alphaProduct = p.alpha0*p.alpha1;

        cbfLower = p.mass*( ...
            -alphaSum*v - alphaProduct*(p.rMax + r));
        cbfUpper = p.mass*( ...
            -alphaSum*v + alphaProduct*(p.rMax - r));

        combinedLower = max(cbfLower, -p.forceMax);
        combinedUpper = min(cbfUpper,  p.forceMax);
        feasible = combinedLower <= combinedUpper + 1e-9;

        % At feasible points, clipping +F_max returns the upper end of the
        % admissible interval whenever the CBF needs to intervene.
        appliedForce = min( ...
            max(p.forceMax, combinedLower), combinedUpper);
        appliedForce(~feasible) = 0;

        acceleration = appliedForce/p.mass;
    end


    function recoveryData = emptyRecoveryData()
        recoveryData.available = false;
        recoveryData.rGrid = [];
        recoveryData.vGrid = [];
        recoveryData.mask = [];
        recoveryData.initialCandidate = [];
        recoveryData.candidateCount = 0;
        recoveryData.recoveredCount = 0;
        recoveryData.recoveredFraction = NaN;
        recoveryData.horizon = NaN;
        recoveryData.integrationStep = NaN;
        recoveryData.gridCount = 0;
    end


    %% Exact region where the raw PD force is unchanged

    function polygon = computePDNoInterventionPolygon(p)
        % This polygon contains every (r, r_dot) for which all of the
        % following hold simultaneously:
        %
        %   1) |r| <= r_max,
        %   2) both first-level HOCBF conditions hold,
        %   3) |F_PD| <= F_max,
        %   4) F_CBF,- <= F_PD <= F_CBF,+.
        %
        % Therefore, inside this polygon, the applied force is exactly the
        % raw PD force: neither the HOCBF filter nor actuator saturation
        % changes it. All constraints are affine in [r; r_dot], so their
        % intersection is computed exactly by half-plane clipping.

        alphaSum = p.alpha0 + p.alpha1;
        alphaProduct = p.alpha0*p.alpha1;

        massAlphaProduct = p.mass*alphaProduct;
        massAlphaSum = p.mass*alphaSum;

        % Each row represents
        %
        %       halfPlaneA(k,:)*[r; r_dot] <= halfPlaneB(k).
        %
        % Rows 1-2: physical position limits.
        % Rows 3-4: first-level HOCBF conditions.
        % Rows 5-6: raw PD actuator-force limits.
        % Row 7:    F_PD <= F_CBF,+.
        % Row 8:    F_PD >= F_CBF,-.
        halfPlaneA = [ ...
             1,  0;
            -1,  0;
             p.alpha0,  1;
            -p.alpha0, -1;
            -p.Kp, -p.Kd;
             p.Kp,  p.Kd;
             massAlphaProduct - p.Kp, ...
             massAlphaSum - p.Kd;
             p.Kp - massAlphaProduct, ...
             p.Kd - massAlphaSum];

        halfPlaneB = [ ...
            p.rMax;
            p.rMax;
            p.alpha0*p.rMax;
            p.alpha0*p.rMax;
            p.forceMax - p.Kp*p.rRef;
            p.forceMax + p.Kp*p.rRef;
            massAlphaProduct*p.rMax - p.Kp*p.rRef;
            massAlphaProduct*p.rMax + p.Kp*p.rRef];

        % The first-level HOCBF set is contained in this bounding box.
        velocityLimit = 2*p.alpha0*p.rMax + 1e-9;
        polygon = [ ...
            -p.rMax, -velocityLimit;
             p.rMax, -velocityLimit;
             p.rMax,  velocityLimit;
            -p.rMax,  velocityLimit];

        for constraintIndex = 1:size(halfPlaneA, 1)
            polygon = clipPolygonWithHalfPlane( ...
                polygon, ...
                halfPlaneA(constraintIndex, :).', ...
                halfPlaneB(constraintIndex));

            if isempty(polygon)
                return;
            end
        end
    end


    function outputPolygon = clipPolygonWithHalfPlane( ...
            inputPolygon, normalVector, boundaryValue)
        % Sutherland-Hodgman clipping for the half-plane
        %
        %       normalVector'*point <= boundaryValue.

        outputPolygon = zeros(0, 2);

        if isempty(inputPolygon)
            return;
        end

        tolerance = 1e-10;
        vertexCount = size(inputPolygon, 1);

        for vertexIndex = 1:vertexCount
            startPoint = inputPolygon(vertexIndex, :);
            endPoint = inputPolygon( ...
                mod(vertexIndex, vertexCount) + 1, :);

            startValue = normalVector.'*startPoint.';
            endValue = normalVector.'*endPoint.';

            startInside = startValue <= boundaryValue + tolerance;
            endInside = endValue <= boundaryValue + tolerance;

            if startInside && endInside
                outputPolygon(end + 1, :) = endPoint; %#ok<AGROW>

            elseif startInside && ~endInside
                intersectionPoint = lineHalfPlaneIntersection( ...
                    startPoint, endPoint, normalVector, boundaryValue);
                outputPolygon(end + 1, :) = ...
                    intersectionPoint; %#ok<AGROW>

            elseif ~startInside && endInside
                intersectionPoint = lineHalfPlaneIntersection( ...
                    startPoint, endPoint, normalVector, boundaryValue);
                outputPolygon(end + 1, :) = ...
                    intersectionPoint; %#ok<AGROW>
                outputPolygon(end + 1, :) = endPoint; %#ok<AGROW>
            end
        end
    end


    function intersectionPoint = lineHalfPlaneIntersection( ...
            startPoint, endPoint, normalVector, boundaryValue)

        edgeDirection = endPoint - startPoint;
        denominator = normalVector.'*edgeDirection.';

        if abs(denominator) < 1e-14
            intersectionPoint = startPoint;
            return;
        end

        interpolationFraction = ...
            (boundaryValue - normalVector.'*startPoint.')/denominator;

        intersectionPoint = ...
            startPoint + interpolationFraction*edgeDirection;
    end


    %% Plotting

    function updatePlots( ...
            t, x, logData, ...
            tNominal, xNominal, nominalLogData, ...
            ellipseData, recoveryData, p)

        r = x(:, 1);
        rDot = x(:, 2);
        rNominal = xNominal(:, 1);
        rDotNominal = xNominal(:, 2);

        % Phase plane.
        % Use the reset option because ordinary cla does not delete objects
        % whose HandleVisibility is off (including the magenta envelopes and
        % the hidden xline objects created during earlier UI updates).
        legend(phaseAxes, 'off');
        cla(phaseAxes, 'reset');
        hold(phaseAxes, 'on');

        rEnvelope = linspace(-p.rMax, p.rMax, 300);
        velocityUpper = p.alpha0*(p.rMax - rEnvelope);
        velocityLower = -p.alpha0*(p.rMax + rEnvelope);

        % Force-limited viability envelope.  These parabolic boundaries
        % come from the minimum stopping distance v^2/(2*a_max).
        maximumAcceleration = p.forceMax/p.mass;
        brakingVelocityUpper = sqrt(max( ...
            0, 2*maximumAcceleration*(p.rMax - rEnvelope)));
        brakingVelocityLower = -sqrt(max( ...
            0, 2*maximumAcceleration*(p.rMax + rEnvelope)));

        if p.showBrakingRegion
            patch(phaseAxes, ...
                [rEnvelope, fliplr(rEnvelope)], ...
                [brakingVelocityUpper, fliplr(brakingVelocityLower)], ...
                [1.00, 0.90, 0.72], ...
                'FaceAlpha', 0.25, ...
                'EdgeColor', 'none', ...
                'DisplayName', 'force-limited braking region');
        end

        if p.showFirstLevelSet
            patch(phaseAxes, ...
                [-p.rMax, -p.rMax, p.rMax, p.rMax], ...
                [0, 2*p.alpha0*p.rMax, 0, -2*p.alpha0*p.rMax], ...
                [0.72, 0.93, 0.75], ...
                'FaceAlpha', 0.35, ...
                'EdgeColor', 'none', ...
                'DisplayName', 'first-level HOCBF set');
        end

        % Controller-independent projection of the second-order HOCBF
        % constraints under |F| <= F_max.
        if p.showSecondOrderSet
            secondOrderPolygon = computeHOCBFFeasiblePolygon(p);

            if size(secondOrderPolygon, 1) >= 3
                patch(phaseAxes, ...
                    secondOrderPolygon(:, 1), ...
                    secondOrderPolygon(:, 2), ...
                    [0.78, 0.68, 1.00], ...
                    'FaceAlpha', 0.10, ...
                    'EdgeColor', [0.45, 0.10, 0.70], ...
                    'LineStyle', ':', ...
                    'LineWidth', 2.0, ...
                    'DisplayName', ...
                    'second-order HOCBF feasible set');
            end
        end

        % Finite-horizon numerical basin for the constant nominal request
        % F_nom = +F_max. The dark-teal contour separates recovered and
        % unrecovered initial grid points.
        if p.showMaxForceRecovery && recoveryData.available
            [~, recoveryContour] = contour(phaseAxes, ...
                recoveryData.rGrid, recoveryData.vGrid, ...
                double(recoveryData.mask), [0.5, 0.5], ...
                'Color', [0.00, 0.48, 0.48], ...
                'LineStyle', '-', ...
                'LineWidth', 2.2);
            recoveryContour.DisplayName = ...
                '+F_{max} CBF recovery-set boundary';
        end

        % Exact state-space region in which the current raw PD force is
        % inside both HOCBF force bounds and the actuator-force limits.
        % Neither the CBF filter nor force saturation modifies F_PD here.
        if p.showPDRegion
            pdNoInterventionPolygon = ...
                computePDNoInterventionPolygon(p);

            if size(pdNoInterventionPolygon, 1) >= 3
                patch(phaseAxes, ...
                    pdNoInterventionPolygon(:, 1), ...
                    pdNoInterventionPolygon(:, 2), ...
                    [0.55, 0.78, 1.00], ...
                    'FaceAlpha', 0.38, ...
                    'EdgeColor', [0.05, 0.35, 0.75], ...
                    'LineWidth', 1.5, ...
                    'DisplayName', 'raw PD force unchanged region');
            end
        end

        if p.showFirstLevelBoundaries
            plot(phaseAxes, rEnvelope, velocityUpper, ...
                'm--', 'LineWidth', 1.4, ...
                'DisplayName', 'first-level HOCBF boundaries');
            plot(phaseAxes, rEnvelope, velocityLower, ...
                'm--', 'LineWidth', 1.4, ...
                'HandleVisibility', 'off');
        end

        if p.showPDEllipse && ellipseData.available
            plot(phaseAxes, ellipseData.r, ellipseData.v, ...
                'k-.', 'LineWidth', 1.8, ...
                'DisplayName', 'PD Lyapunov invariant ellipse');
        end

        if p.showPositionLimits
            xline(phaseAxes, p.rMax, '--r', ...
                'LineWidth', 1.2, ...
                'DisplayName', 'physical position limits');
            xline(phaseAxes, -p.rMax, '--r', ...
                'LineWidth', 1.2, ...
                'HandleVisibility', 'off');
        end

        plot(phaseAxes, r, rDot, ...
            'b-', 'LineWidth', 2.2, ...
            'DisplayName', 'CBF-filtered trajectory');
        plot(phaseAxes, rNominal, rDotNominal, ...
            '--', 'Color', [0.30, 0.30, 0.30], 'LineWidth', 1.8, ...
            'DisplayName', 'nominal PD-only trajectory');

        startPoint = plot(phaseAxes, r(1), rDot(1), ...
            'go', 'MarkerFaceColor', 'g', 'MarkerSize', 7, ...
            'DisplayName', 'start');
        endPoint = plot(phaseAxes, r(end), rDot(end), ...
            'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 7, ...
            'DisplayName', 'CBF end');

        grid(phaseAxes, 'on');
        xlabel(phaseAxes, 'r [m]');
        ylabel(phaseAxes, 'r dot [m/s]');
        title(phaseAxes, 'Rod Phase Plane');
        legend(phaseAxes, 'show', 'Location', 'best');
        hold(phaseAxes, 'off');

        % Position versus time.
        legend(rTimeAxes, 'off');
        cla(rTimeAxes, 'reset');
        hold(rTimeAxes, 'on');
        positionLine = plot(rTimeAxes, t, r, ...
            'b-', 'LineWidth', 2.0);
        nominalPositionLine = plot(rTimeAxes, tNominal, rNominal, ...
            '--', 'Color', [0.30, 0.30, 0.30], 'LineWidth', 1.5);
        referenceLine = yline(rTimeAxes, p.rRef, ':k', ...
            'LineWidth', 1.2);
        if p.showPositionLimits
            yline(rTimeAxes, p.rMax, '--r', 'HandleVisibility', 'off');
            yline(rTimeAxes, -p.rMax, '--r', 'HandleVisibility', 'off');
        end
        grid(rTimeAxes, 'on');
        xlabel(rTimeAxes, 'Time [s]');
        ylabel(rTimeAxes, 'r [m]');
        title(rTimeAxes, 'Rod Position');
        legend(rTimeAxes, ...
            [positionLine, nominalPositionLine, referenceLine], ...
            {'CBF-filtered r', 'nominal PD-only r', 'r reference'}, ...
            'Location', 'best');
        hold(rTimeAxes, 'off');

        % Velocity versus time.
        legend(vTimeAxes, 'off');
        cla(vTimeAxes, 'reset');
        hold(vTimeAxes, 'on');
        filteredVelocityLine = plot(vTimeAxes, t, rDot, ...
            'Color', [0.1, 0.55, 0.1], ...
            'LineWidth', 2.0);
        nominalVelocityLine = plot(vTimeAxes, tNominal, rDotNominal, ...
            '--', 'Color', [0.30, 0.30, 0.30], 'LineWidth', 1.5);
        yline(vTimeAxes, 0, ':k', 'HandleVisibility', 'off');
        grid(vTimeAxes, 'on');
        xlabel(vTimeAxes, 'Time [s]');
        ylabel(vTimeAxes, 'r dot [m/s]');
        title(vTimeAxes, 'Rod Velocity');
        legend(vTimeAxes, ...
            [filteredVelocityLine, nominalVelocityLine], ...
            {'CBF-filtered', 'nominal PD-only'}, ...
            'Location', 'best');
        hold(vTimeAxes, 'off');

        % Applied, nominal, and CBF force bounds versus time.
        legend(forceAxes, 'off');
        cla(forceAxes, 'reset');
        hold(forceAxes, 'on');
        appliedLine = plot(forceAxes, t, logData.force, ...
            'b-', 'LineWidth', 2.0);
        nominalLine = plot(forceAxes, t, logData.nominalForce, ...
            '--', 'Color', [0.25, 0.25, 0.25], 'LineWidth', 1.2);
        nominalOnlyLine = plot(forceAxes, tNominal, nominalLogData.force, ...
            '-.', 'Color', [0.85, 0.45, 0.05], 'LineWidth', 1.2);
        forceLegendHandles = [ ...
            appliedLine, nominalLine, nominalOnlyLine];
        forceLegendNames = { ...
            'CBF-filtered F', ...
            'F_{nom} on filtered state', ...
            'PD-only applied F'};

        if p.showHOCBFForceBounds
            lowerLine = plot(forceAxes, t, logData.cbfLower, ...
                'm:', 'LineWidth', 1.4);
            upperLine = plot(forceAxes, t, logData.cbfUpper, ...
                'c:', 'LineWidth', 1.4);
            forceLegendHandles = [ ...
                forceLegendHandles, lowerLine, upperLine]; %#ok<AGROW>
            forceLegendNames = [ ...
                forceLegendNames, {'CBF lower', 'CBF upper'}]; %#ok<AGROW>
        end

        if p.showActuatorForceLimits
            forceLimitLine = plot(forceAxes, ...
                [0, p.tFinal], [p.forceMax, p.forceMax], ...
                '--r', 'LineWidth', 1.2);
            plot(forceAxes, ...
                [0, p.tFinal], [-p.forceMax, -p.forceMax], ...
                '--r', 'LineWidth', 1.2, ...
                'HandleVisibility', 'off');
            forceLegendHandles = [ ...
                forceLegendHandles, forceLimitLine]; %#ok<AGROW>
            forceLegendNames = [ ...
                forceLegendNames, {'actuator force limits'}]; %#ok<AGROW>
        end
        grid(forceAxes, 'on');
        xlabel(forceAxes, 'Time [s]');
        ylabel(forceAxes, 'Force [N]');
        title(forceAxes, 'Control Force and HOCBF Bounds');
        legend(forceAxes, forceLegendHandles, forceLegendNames, ...
            'Location', 'best');
        hold(forceAxes, 'off');

        drawnow limitrate nocallbacks;
    end


    %% Result summary

    function updateStatus( ...
            x, xNominal, logData, ellipseData, recoveryData, p)
        r = x(:, 1);
        rDot = x(:, 2);
        rNominal = xNominal(:, 1);

        tolerance = 1e-5;
        limitViolated = any(abs(r) > p.rMax + tolerance);
        infeasibleCount = sum(~logData.feasible);

        psiPositiveInitial = ...
            -p.v0 + p.alpha0*(p.rMax - p.r0);
        psiNegativeInitial = ...
             p.v0 + p.alpha0*(p.rMax + p.r0);

        initialHOCBFSafe = ...
            abs(p.r0) <= p.rMax && ...
            psiPositiveInitial >= 0 && ...
            psiNegativeInitial >= 0;

        % For this symmetric double-integrator model, the complete
        % first-level HOCBF set is second-order force-feasible whenever
        % 2*alpha_0^2*r_max <= F_max/m_r.
        firstLevelEntirelyForceFeasible = ...
            2*p.alpha0^2*p.rMax <= p.forceMax/p.mass + tolerance;

        if p.enableCBF
            cbfText = 'enabled';
        else
            cbfText = 'disabled';
        end

        if limitViolated
            safetyText = 'LIMIT VIOLATED';
            statusArea.BackgroundColor = [1.00, 0.85, 0.85];
        elseif infeasibleCount > 0
            safetyText = 'CBF INFEASIBLE AT SOME SAMPLES';
            statusArea.BackgroundColor = [1.00, 0.94, 0.78];
        else
            safetyText = 'safe in this simulation';
            statusArea.BackgroundColor = [0.88, 1.00, 0.88];
        end

        statusLines = { ...
            ['CBF: ', cbfText]; ...
            ['Result: ', safetyText]; ...
            sprintf('Final state: r = %+.5f m', r(end)); ...
            sprintf('             v = %+.5f m/s', rDot(end)); ...
            sprintf('Maximum |r|: %.5f m', max(abs(r))); ...
            sprintf('Nominal-only max |r|: %.5f m', ...
                max(abs(rNominal))); ...
            sprintf('Minimum margin: %.5f m', ...
                min(p.rMax - abs(r))); ...
            sprintf('Maximum |v|: %.5f m/s', max(abs(rDot))); ...
            sprintf('Maximum |F|: %.3f N', ...
                max(abs(logData.force))); ...
            sprintf('CBF active: %.1f %% of samples', ...
                100*mean(logData.cbfActive)); ...
            sprintf('Force saturated: %.1f %% of samples', ...
                100*mean(logData.forceSaturated)); ...
            sprintf('Infeasible samples: %d', infeasibleCount); ...
            sprintf('Initial psi_1,+ = %+.5f', psiPositiveInitial); ...
            sprintf('Initial psi_1,- = %+.5f', psiNegativeInitial); ...
            sprintf('Initial HOCBF set satisfied: %s', ...
                yesNo(initialHOCBFSafe)); ...
            sprintf('Second-order set covers C_1: %s', ...
                yesNo(firstLevelEntirelyForceFeasible))};

        if ellipseData.available
            statusLines{end + 1} = sprintf( ...
                'PD ellipse rho: %.4e', ellipseData.rho);
            statusLines{end + 1} = [ ...
                'Ellipse limited by: ', ...
                ellipseData.limitingConstraint];
        else
            statusLines{end + 1} = [ ...
                'PD ellipse unavailable: ', ...
                ellipseData.reason];
        end

        if p.showMaxForceRecovery
            if recoveryData.available
                statusLines{end + 1} = sprintf( ...
                    '+Fmax recovery: %.1f %% of braking candidates', ...
                    100*recoveryData.recoveredFraction);
                statusLines{end + 1} = sprintf( ...
                    'Recovery grid: %d x %d, T = %.3f s', ...
                    recoveryData.gridCount, recoveryData.gridCount, ...
                    recoveryData.horizon);
            else
                statusLines{end + 1} = ...
                    '+Fmax recovery set: empty on current grid';
            end
        end

        statusArea.Value = statusLines;
    end


    function output = yesNo(condition)
        if condition
            output = 'yes';
        else
            output = 'no';
        end
    end
end
