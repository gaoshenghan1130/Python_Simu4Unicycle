function report=run_manifold_hardware(theta0_deg,make_plots)
% Tuned profile: validated at +/-1 and +/-1.5 degrees, v=1.2 m/s.
% This is continuous-feedback simulation, not a delay/noise hardware test.
if nargin<1, theta0_deg=1.5; end
if nargin<2, make_plots=true; end
root=fileparts(mfilename('fullpath'));
addpath(fullfile(root,'analysis','manifold'));
options=struct('speed',1.2,'Q',diag([2 1 1 2 1 0.5 20 200 5]), ...
    'R',diag([0.01 0.1]));
report=verify_turning_manifold_approach(0,[],make_plots,theta0_deg,30,options);
report.peak_r_m=max(abs(report.X(:,9)));
report.rod_travel_passed=report.peak_r_m<0.075;
fprintf('Rod peak %.4f cm; <7.5 cm: %d\n',100*report.peak_r_m,report.rod_travel_passed);
end
