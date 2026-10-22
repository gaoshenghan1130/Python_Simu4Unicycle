function study = run_manifold_overshoot_study(theta0_deg,speeds,options)
%RUN_MANIFOLD_OVERSHOOT_STUDY Overshoot versus design speed for a given lean.
%   run_manifold_overshoot_study(5,1.5:0.25:6)
%   run_manifold_overshoot_study([0.25 1 5],[1.5:0.25:6 7 8 10])
%   run_manifold_overshoot_study(1,[2 2.5 3],struct('make_plots',false))
% Angles are degrees; speeds are m/s. Defaults test three initial angles.
% Outputs: MATLAB/Full/results/manifold_overshoot_vs_speed/.
root=fileparts(mfilename('fullpath'));
addpath(fullfile(root,'analysis','manifold'));
if nargin<1, theta0_deg=[]; end
if nargin<2, speeds=[]; end
if nargin<3, options=struct(); end
study=study_manifold_overshoot_vs_speed(theta0_deg,speeds,options);
end
