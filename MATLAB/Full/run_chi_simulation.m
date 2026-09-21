% Straight rolling demo with chi (heading error) in lateral feedback.
% Chi=psi for the straight reference along +x; theta is the lean angle.
project_root=fileparts(mfilename('fullpath'));
addpath(fullfile(project_root,'config'),fullfile(project_root,'model'), ...
    fullfile(project_root,'controllers'),fullfile(project_root,'simulation'), ...
    fullfile(project_root,'analysis'));
experiment=experiment_settings();
experiment.controller.mode='pole_placement';
experiment.controller.hold_gamma_zero=true; % nonlinear torque enforces gamma=0
experiment.sweep.enabled=false;
% Match target/design speed to the configured initial speed; no acceleration command.
experiment.design.forward_speed=experiment.settings.forward_speed0;
experiment.settings.gamma0=0;
experiment.settings.gamma_dot0=0;
if experiment.design.forward_speed==0
    experiment.design.lateral_states='balance';
    fprintf('Zero speed: stationary balance poles only; chi is uncontrollable and is NOT pole-placed.\n');
else
    experiment.design.lateral_states='balance_chi';
end
fprintf('Initial: theta=%.6g deg, chi=%.6g deg, speed=%.6g m/s\n', ...
    experiment.settings.theta0*180/pi,experiment.settings.psi0*180/pi, ...
    experiment.settings.forward_speed0);
fprintf('Target/design speed = initial speed = %.6g m/s. gamma0=gamma_dot0=0.\n', ...
    experiment.design.forward_speed);
fprintf('Gamma is held at zero by model-based torque; speed is free to respond to lateral motion.\n');
% Optional additional cases at the SAME speed. Angles below are degrees.
% false: use only your configured chi/theta; true: also test isolated perturbations.
test_initial_perturbations=true;
results=run_experiment(experiment);
results(1).label='Configured initial state';
if test_initial_perturbations
    perturbations_deg=[0.1 0;0 0.01;0 0.1]; % [chi0, theta0]
    for case_index=1:size(perturbations_deg,1)
        trial=experiment;
        trial.settings.psi0=perturbations_deg(case_index,1)*pi/180;
        trial.settings.theta0=perturbations_deg(case_index,2)*pi/180;
        trial.settings.r0=0; trial.settings.theta_dot0=0;
        trial.settings.r_dot0=0; trial.settings.psi_dot0=0;
        trial_result=run_experiment(trial);
        trial_result.label=sprintf('chi0=%.3g deg, theta0=%.3g deg', ...
            perturbations_deg(case_index,1),perturbations_deg(case_index,2));
        results(end+1)=trial_result; %#ok<SAGROW>
    end
end
for case_index=1:numel(results)
    current=results(case_index);
    fprintf('%s: peak |gamma|=%.3g deg, final speed=%.6g m/s\n', ...
        current.label,max(abs(current.X(:,10)))*180/pi, ...
        current.parameters.R*current.X(end,2));
end
result=results(1);
plot_simulation(results);
plot_heading(results);

% Main mechanical coordinates together: rod displacement, lean, body pitch.
figure('Name','Chi control - r, theta, gamma');
coordinate_labels={'r [m]','theta [deg]','gamma [deg]'};
for j=1:numel(results)
    coordinates=[results(j).X(:,9), ...
        results(j).X(:,7)*180/pi,results(j).X(:,10)*180/pi];
    for k=1:3
        subplot(3,1,k); hold on;
        plot(results(j).t,coordinates(:,k),'LineWidth',1.2, ...
            'DisplayName',results(j).label);
        ylabel(coordinate_labels{k}); grid on;
        legend('show','Interpreter','none','Location','best');
    end
end
% Show gamma on a physical angle scale, rather than amplifying roundoff.
subplot(3,1,3); ylim([-0.01 0.01]);
xlabel('Time [s]');
