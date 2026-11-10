%% Equilibrium-manifold balancing controller: editable parameters
% Run this script to simulate, show plots, and save data. SI unless specified.
% Analysis: Derivation/FullModel/analysis/equilibrium_balance_controller.md
speeds = [0, 0.5, 1.5];                   % Reference forward speed (m/s); can be [0 .5 1.5]
models = "nonlinear";            % Full nonlinear dynamics
initial_cases = ["custom"];       % custom / lean / compatible / equilibrium
initial_lean_deg = 3;
duration = 10;                    % Simulation horizon (s)
sample_time = 0.02;               % Saved/plot sampling interval (s)

% Lateral LQR state: [r-r_des; d/dt(r-r_des); theta-kappa*sigma3; sigma1]
Q_balance = [20, 2, 20, 2];
rho_F = 0.1;
force_limit = 30;                 % N

% Longitudinal LQR state: [phi_dot-V; gamma_dot; gamma]
Q_longitudinal = [1, 1, 5];
rho_M = 0.1;
torque_limit = 15;                % Nm

% Used when initial_cases includes "custom".
rod0 = 0;                        % m
rod_rate0 = 0;                   % m/s
lean_rate0_deg_s = 0;
yaw_rate0_deg_s = 0;
body0_deg = 0;                   % gamma
body_rate0_deg_s = 0;
heading0_deg = 0;

% Optional physical parameter changes. Empty means model_parameters defaults.
parameter_overrides = struct();  % Example: struct('BR',6.5,'BP',0)
stop_lean_deg = rad2deg(0.65);    % Numerical stop thresholds, not hardware ratings
stop_body_deg = rad2deg(0.65);
stop_rod_m = 0.4;
show_figures = true;              % Keep figure windows open

%% Run: no edits normally needed below
root=fileparts(mfilename('fullpath'));
addpath(fullfile(root,'analysis','equilibrium_balance'));
cfg=struct('speeds',speeds,'models',models,'initial_cases',initial_cases, ...
 'Q',Q_balance,'Q_long',Q_longitudinal,'rho_long',rho_M, ...
 'torque_limit',torque_limit,'sample_time',sample_time, ...
 'rod0',rod0,'rod_rate0',rod_rate0,'lean_rate0_deg_s',lean_rate0_deg_s, ...
 'yaw_rate0_deg_s',yaw_rate0_deg_s,'body0_deg',body0_deg, ...
 'body_rate0_deg_s',body_rate0_deg_s,'heading0_deg',heading0_deg, ...
 'parameter_overrides',parameter_overrides,'show_figures',show_figures, ...
 'stop_limits',[deg2rad(stop_lean_deg),deg2rad(stop_body_deg),stop_rod_m], ...
 'output_dir',fullfile(root,'results','equilibrium_balance_interactive'));
report=study_equilibrium_balance(rho_F,duration,initial_lean_deg,force_limit,cfg);
disp(report.metrics);
for j=1:numel(report.designs)
 d=report.designs{j};fprintf('v=%g m/s: K_balance = ',d.speed);disp(d.K);
end
fprintf('Plots, metrics.csv and study.mat saved to:\n%s\n',report.output_dir);
