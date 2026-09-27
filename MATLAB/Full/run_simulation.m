% MAIN 1: Run one or more parameter groups, in the listed order.
% Edit the groups in config/simulation_preset.m.
project_root=fileparts(mfilename('fullpath'));
addpath(fullfile(project_root,'config'),fullfile(project_root,'model'), ...
    fullfile(project_root,'controllers'),fullfile(project_root,'simulation'), ...
    fullfile(project_root,'analysis'));

% Available: 'rolling_balance', 'chi_epsilon_balance', 'configured', 'chi_balance', 'mate_open_loop',
%            'mate_current_mass', 'mate_configured'.
% Example comparison: {'mate_open_loop','mate_current_mass','chi_balance'}
parameter_groups={'rolling_balance'};
assert(iscellstr(parameter_groups) && ~isempty(parameter_groups), ...
    'parameter_groups must be a nonempty cell array of preset names.');

runs=cell(1,numel(parameter_groups));
for case_index=1:numel(parameter_groups)
    experiment=simulation_preset(parameter_groups{case_index});
    runs{case_index}=run_experiment(experiment);
    runs{case_index}.label=parameter_groups{case_index};
end
results=[runs{:}];
result=results(1);
plot_simulation(results);
plot_heading(results);
% results(k) contains the actual parameters, controller, settings and trajectory.
