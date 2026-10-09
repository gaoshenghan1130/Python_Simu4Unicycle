function results = compare_direct_and_two_stage_theta(theta0_values)
%COMPARE_DIRECT_AND_TWO_STAGE_THETA Compare direct and indirect upright control.
% Sweeps theta0 = 0.25:0.25:5 degrees. Direct: target theta=0 in one leg.
% Two-stage: target 0.1 degrees, then transfer its terminal state to target 0.
% The direct run lasts 50 s; the two-stage run uses two 25 s legs, so both
% methods have the same total simulation time, model parameters and criteria.

root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
addpath(root,fullfile(root,'analysis','manifold'));
custom_grid = nargin >= 1 && ~isempty(theta0_values);
if ~custom_grid
    theta0_values = (0.25:0.25:5.0)';
else
    assert(isvector(theta0_values) && all(isfinite(theta0_values(:))) && ...
        all(theta0_values(:)>0),'theta0_values must be a positive finite vector.');
    theta0_values = theta0_values(:);
end
n = numel(theta0_values);

direct_omega = nan(n,1);
direct_final_theta_deg = nan(n,1);
direct_final_rate_norm = nan(n,1);
direct_reached = false(n,1);
direct_status = strings(n,1);
direct_message = strings(n,1);

stage1_omega = nan(n,1);
stage1_final_theta_deg = nan(n,1);
stage1_reached = false(n,1);
stage2_omega = nan(n,1);
stage2_final_theta_deg = nan(n,1);
stage2_final_rate_norm = nan(n,1);
stage2_reached = false(n,1);
two_stage_status = strings(n,1);
two_stage_message = strings(n,1);

for k = 1:n
    theta0 = theta0_values(k);
    fprintf('\nInitial lean comparison: theta0 = %.2f deg (%d/%d)\n', ...
        theta0,k,n);

    % Direct method: start from the configured initial condition, target 0.
    try
        evalc('direct = verify_turning_manifold_approach(0,[],false,theta0,50);');
        direct_omega(k) = direct.omega;
        direct_final_theta_deg(k) = direct.X(end,7)*180/pi;
        direct_final_rate_norm(k) = direct.final_relative_rate_norm;
        direct_reached(k) = direct.reached_turning_manifold;
        if direct_reached(k)
            direct_status(k) = "success";
            direct_message(k) = "Direct upright target reached.";
        else
            direct_status(k) = "not_converged";
            direct_message(k) = "Simulation ended without meeting all convergence checks.";
        end
    catch err
        direct_status(k) = "error";
        direct_message(k) = string(err.message);
    end

    % Indirect method: general-turning target, then upright target.
    try
        evalc('leg1 = verify_turning_manifold_approach(0.1,[],false,theta0);');
        stage1_omega(k) = leg1.omega;
        stage1_final_theta_deg(k) = leg1.X(end,7)*180/pi;
        stage1_reached(k) = leg1.reached_turning_manifold;
        if ~stage1_reached(k)
            two_stage_status(k) = "stage1_not_converged";
            two_stage_message(k) = ...
                "General-turning stage ended without meeting all convergence checks.";
            fprintf('  Two-stage: stage 1 did not meet convergence checks.\n');
            continue
        end

        evalc('leg2 = verify_turning_manifold_approach(0,leg1.X(end,:)'',false);');
        stage2_omega(k) = leg2.omega;
        stage2_final_theta_deg(k) = leg2.X(end,7)*180/pi;
        stage2_final_rate_norm(k) = leg2.final_relative_rate_norm;
        stage2_reached(k) = leg2.reached_turning_manifold;
        if stage2_reached(k)
            two_stage_status(k) = "success";
            two_stage_message(k) = "Both stages reached their target manifolds.";
        else
            two_stage_status(k) = "stage2_not_converged";
            two_stage_message(k) = ...
                "Upright stage ended without meeting all convergence checks.";
        end
    catch err
        two_stage_status(k) = "error";
        two_stage_message(k) = string(err.message);
    end

    fprintf('  Direct: %s; two-stage: %s\n', ...
        direct_status(k),two_stage_status(k));
end

results = table(theta0_values,direct_omega,direct_final_theta_deg, ...
    direct_final_rate_norm,direct_reached,direct_status,direct_message, ...
    stage1_omega,stage1_final_theta_deg,stage1_reached,stage2_omega, ...
    stage2_final_theta_deg,stage2_final_rate_norm,stage2_reached, ...
    two_stage_status,two_stage_message, ...
    'VariableNames',{'theta0_deg','direct_omega_rad_s', ...
    'direct_final_theta_deg','direct_final_rate_norm','direct_reached', ...
    'direct_status','direct_message','stage1_omega_rad_s', ...
    'stage1_final_theta_deg','stage1_reached','stage2_omega_rad_s', ...
    'stage2_final_theta_deg','stage2_final_rate_norm','stage2_reached', ...
    'two_stage_status','two_stage_message'});

output_dir = fullfile(root,'results','direct_vs_two_stage_theta');
if ~isfolder(output_dir), mkdir(output_dir); end
if custom_grid
    report_name = 'direct_vs_two_stage_theta_custom.csv';
else
    report_name = 'direct_vs_two_stage_theta.csv';
end
report_path = fullfile(output_dir,report_name);
writetable(results,report_path);
fprintf('\nComparison saved to:\n%s\n',report_path);
disp(results(:,{'theta0_deg','direct_status','direct_final_theta_deg', ...
    'two_stage_status','stage2_final_theta_deg'}));
end
