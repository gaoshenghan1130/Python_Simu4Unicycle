% Run this script after editing config/experiment_settings.m.
project_root=fileparts(mfilename('fullpath'));
addpath(fullfile(project_root,'config'),fullfile(project_root,'model'), ...
    fullfile(project_root,'controllers'),fullfile(project_root,'simulation'), ...
    fullfile(project_root,'analysis'));
experiment=experiment_settings();
results=run_experiment(experiment);
plot_simulation(results);
if isfield(results(1).controller,'reference_rate'), plot_heading(results); end
result=results(1); % backward-compatible first run; all cases are in results
% results(k): t, X, U, parameters, controller (including generated K),
% settings, design, label, sweep_value, and tilt-event information.
