% Run this script to simulate and plot. No additional toolbox required.
project_root=fileparts(mfilename('fullpath'));
addpath(fullfile(project_root,'config'),fullfile(project_root,'model'), ...
        fullfile(project_root,'controllers'),fullfile(project_root,'simulation'));
p=model_parameters();
c=controller_parameters();
s=simulation_settings();
result=simulate_unicycle(p,c,s);
plot_simulation(result);
% result.X: rows are time samples, columns follow the documented state order.
% result.U: columns are [F,M2]. Data remain in the workspace; no files saved.
