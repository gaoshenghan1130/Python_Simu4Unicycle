function report = verify_direct_upright_turning()
%VERIFY_DIRECT_UPRIGHT_TURNING Test a direct transfer to the upright turn.
% Starts from verify_turning_manifold_approach's default initial state and
% targets theta = 0 in one stage, without the intermediate 0.1-degree turn.

root = fileparts(mfilename('fullpath'));
addpath(root,fullfile(root,'analysis','manifold'));
report = verify_turning_manifold_approach(0);

fprintf('Direct upright target: Omega=%.8g rad/s, reached=%s\n', ...
    report.omega,string(report.reached_turning_manifold));
end
