function result = simulate_lateral_openloop_above_critical(relative_margin)
%SIMULATE_LATERAL_OPENLOOP_ABOVE_CRITICAL Run above the upper stability boundary.
% Finds the high-speed critical boundary with lateral force open loop and the
% longitudinal gamma-PD loop enabled, then simulates just above it. During
% the nonlinear run, an r-PD force keeps the rod centered (r=0).
% Plots are shown in MATLAB and are not exported or saved.
%
% Run from MATLAB/Full:
%   result = simulate_lateral_openloop_above_critical();
% Optional relative speed margin, e.g. 0.02 for 2% above the boundary.

if nargin < 1 || isempty(relative_margin)
    relative_margin = -0.02;
end
assert(isscalar(relative_margin) && isfinite(relative_margin) && ...
    relative_margin > -1 && relative_margin <= 0.5, ...
    'relative_margin must be in (-1, 0.5].');

projectRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(genpath(projectRoot));
experiment = simulation_preset('mate_current_mass');
p = experiment.parameters;
c = experiment.controller;
s = experiment.settings;
% Keep the critical-speed calculation lateral-force open loop; enable the
% rod-centering r-PD only for the nonlinear simulation.
c.center_rod = true;
c.r_ref = 0;
c.r_dot_ref = 0;

% Locate the last unstable speed, i.e. the upper boundary beyond which the
% current parameterized model has no exponentially growing dynamic mode.
scanSpeeds = 0.1:0.01:6.0;
growth = zeros(size(scanSpeeds));
for k = 1:numel(scanSpeeds)
    growth(k) = dominant_dynamic_growth(scanSpeeds(k),p,c);
end
unstable = growth > 1e-5;
lastUnstable = find(unstable,1,'last');
assert(~isempty(lastUnstable) && lastUnstable < numel(scanSpeeds), ...
    ['Could not bracket the high-speed critical boundary in 0.1-6 m/s. ' ...
     'Inspect the scan range or the model/controller setup.']);
lower = scanSpeeds(lastUnstable);
upper = scanSpeeds(lastUnstable+1);
while upper-lower > 1e-4
    midpoint = (lower+upper)/2;
    if dominant_dynamic_growth(midpoint,p,c) > 1e-5
        lower = midpoint;
    else
        upper = midpoint;
    end
end
criticalSpeed = (lower+upper)/2;
simulationSpeed = criticalSpeed*(1+relative_margin);

% Keep the initial perturbation and remaining simulation setup identical.
s.t_end = 30;
s.output_dt = 0.01;
s.max_step = 0.01;
s.theta0 = 0.1*pi/180;
s.theta_dot0 = 0;
s.r0 = 0;
s.r_dot0 = 0;
s.gamma0 = 0;
s.gamma_dot0 = 0;
s.psi0 = 0;
s.psi_dot0 = 0;
s.phi0 = 0;
s.xG0 = 0;
s.yG0 = 0;

result.critical_speed_mps = criticalSpeed;
result.critical_bracket_mps = [lower upper];
result.scan_speeds_mps = scanSpeeds;
result.scan_dominant_growth_per_s = growth;
result.relative_margin = relative_margin;
result.speed_mps = simulationSpeed;

fprintf('Current-model upper critical speed: %.8g m/s\n',criticalSpeed);
fprintf('Critical-speed bracket: [%.8g, %.8g] m/s\n',lower,upper);
fprintf('Simulating lateral open loop at %.8g m/s (%.3g%% above boundary).\n', ...
    simulationSpeed,100*relative_margin);
fprintf(['Simulation controller: rod r-PD (Kp=%.4g, Kd=%.4g); ' ...
    'gamma PD torque (Kp=%.4g, Kd=%.4g).\n'], ...
    c.kp_r,c.kd_r,c.kp_gamma,c.kd_gamma);

runSettings = s;
runSettings.forward_speed0 = simulationSpeed;
run = simulate_unicycle(p,c,runSettings);
run.label = sprintf('%.3g%% above upper critical speed',100*relative_margin);
result.run = run;
fprintf('Actual speed range [%.6g, %.6g] m/s; final theta %.6g deg\n', ...
    min(p.R*run.X(:,2)),max(p.R*run.X(:,2)),run.X(end,7)*180/pi);

fig = figure('Name','Lateral open loop above upper critical speed', ...
    'Color','w','Position',[80 80 1200 850]);
tiledlayout(2,2,'Padding','compact','TileSpacing','compact');
stateData = { ...
    @(r) r.X(:,7)*180/pi, '\theta [deg]'; ...
    @(r) r.X(:,9), 'Rod lateral coordinate r [m]'; ...
    @(r) r.X(:,10)*180/pi, '\gamma [deg]'; ...
    @(r) p.R*r.X(:,2), 'Forward speed [m/s]'};
for j = 1:4
    ax = nexttile;
    extractState = stateData{j,1};
    plot(ax,run.t,extractState(run),'LineWidth',1.25);
    grid(ax,'on'); xlabel(ax,'Time [s]'); ylabel(ax,stateData{j,2});
end
sgtitle(sprintf('Lateral open loop at %.4g m/s (upper v_{crit}=%.4g m/s)', ...
    simulationSpeed,criticalSpeed));
result.figure = fig;
end

function growth = dominant_dynamic_growth(speed,p,c)
% Linearize upright straight rolling and close only the gamma PD torque loop.
xEq = zeros(12,1);
xEq(2) = speed/p.R;
uEq = zeros(2,1);
A = complex_step_jacobian(@(x) model_rhs(0,x,uEq,p),xEq);
B = complex_step_jacobian(@(u) model_rhs(0,xEq,u,p),uEq);

% gamma_dot = sigma_g - sigma3*tan(theta); at upright rolling its
% first-order expression is sigma_g (state 5).
gammaFeedback = zeros(1,12);
gammaFeedback(5) = c.kd_gamma;
gammaFeedback(10) = c.kp_gamma;
Aclosed = A+B(:,2)*gammaFeedback;
lambda = eig(Aclosed);
dynamic = abs(lambda)>1e-5; % discard kinematic/phase zero modes
if any(dynamic)
    growth = max(real(lambda(dynamic)));
else
    growth = 0;
end
end

function J = complex_step_jacobian(fun,x)
J = zeros(numel(fun(x)),numel(x));
h = 1e-20;
for j = 1:numel(x)
    z = x;
    z(j) = z(j)+1i*h;
    J(:,j) = imag(fun(z))/h;
end
end
