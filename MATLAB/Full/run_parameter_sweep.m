% MAIN 2: Choose ONE parameter group and try ONE parameter's values in order.
project_root=fileparts(mfilename('fullpath'));
addpath(fullfile(project_root,'config'),fullfile(project_root,'model'), ...
    fullfile(project_root,'controllers'),fullfile(project_root,'simulation'), ...
    fullfile(project_root,'analysis'));

parameter_group='chi_epsilon_balance';
parameter='speed'; % matched initial AND target/design speed [m/s]
values=[0.25 0.5 0.75 1 1.25 1.5 1.75 2 2.25 2.5];
parameter_index=[]; % [] for a scalar; MATLAB linear index for a vector/matrix.
% Examples:
% parameter='settings.theta0'; values=[0.1 1 5]*pi/180;
% parameter='settings.forward_speed0'; values=[1.5 2 2.375 3]; % initial only
% parameter='settings.yG0'; values=[-0.01 0 0.01];
% parameter='parameters.BR'; values=[0 3 6.5];
% parameter='controller.kp_gamma'; values=[2 3 6];
% Four-pole example: parameter='design.rolling_poles';
% values=[-1.4 -1.6 -1.8]; parameter_index=1;
% Six-pole example (parameter_group='chi_epsilon_balance'): parameter='design.chi_epsilon_poles';
% values=[-1.2 -1.4 -1.6]; parameter_index=1;
% With parameter_group='chi_balance':
% parameter='design.chi_poles'; values=[-1.4 -1.6 -1.8]; parameter_index=1;

experiment=simulation_preset(parameter_group);
experiment.design.method='acker'; % local implementation; no Control System Toolbox needed
% Optional base-group overrides apply equally to every value:
% experiment.settings.t_end=30;
experiment.sweep.enabled=true;
experiment.sweep.parameter=parameter;
experiment.sweep.values=values;
experiment.sweep.index=parameter_index;
results=run_experiment(experiment);
result=results(1);
plot_simulation(results);
plot_heading(results);
% Each value starts from the same group; controllers regenerate after the change.
% parameter='speed' sets settings.forward_speed0 and design.forward_speed together.
% Other parameter names change only that field. Zero speed is not a rolling design.
