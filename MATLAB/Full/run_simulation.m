% Run directly for chi-feedback balance; edit the preset below.
% Use run_mode='configured' to read config/experiment_settings.m unchanged.
project_root=fileparts(mfilename('fullpath'));
addpath(fullfile(project_root,'config'),fullfile(project_root,'model'), ...
    fullfile(project_root,'controllers'),fullfile(project_root,'simulation'), ...
    fullfile(project_root,'analysis'));
experiment=experiment_settings();

% Select 'chi_balance', 'configured', or 'mate_open_loop'.
run_mode='chi_balance';
assert(ismember(run_mode,{'chi_balance','configured','mate_open_loop'}), ...
    'Unknown run_mode.');
if strcmp(run_mode,'chi_balance')
    experiment.controller.mode='pole_placement';
    experiment.controller.hold_gamma_zero=false;
    experiment.parameters.mr=2.3;
    experiment.parameters.BR=6.5;
    experiment.parameters.BP=0;
    experiment.design.lateral_states='balance_chi';
    experiment.design.forward_speed=2.375;
    experiment.design.chi_poles=[-1.6 -1.8 -2 -2.2 -2.4];
    experiment.design.longitudinal=[-0.8 -0.95 -1.05 -1.2];
    experiment.settings.forward_speed0=experiment.design.forward_speed;
    experiment.settings.t_end=30;
    % Initial perturbations: angles in degrees here, r in metres.
    % Independent nonzero theta can leave an offset in this reduced design.
    experiment.settings.theta0=0*pi/180;
    experiment.settings.r0=0;
    experiment.settings.psi0=1*pi/180; % chi=psi for the straight +x path
    experiment.settings.gamma0=0;
    experiment.settings.theta_dot0=0; experiment.settings.r_dot0=0;
    experiment.settings.psi_dot0=0; experiment.settings.gamma_dot0=0;
    experiment.settings.phi0=0; experiment.settings.xG0=0; experiment.settings.yG0=0;
    experiment.sweep.enabled=false;
    fprintf('Chi pole placement: mr=%g kg, b=%g, design/initial speed=%g m/s\n', ...
        experiment.parameters.mr,experiment.parameters.BR,experiment.design.forward_speed);
    fprintf('Initial theta=%g deg, r=%g m, chi=%g deg; longitudinal pole placement active.\n', ...
        experiment.settings.theta0*180/pi,experiment.settings.r0,experiment.settings.psi0*180/pi);
end
% Mate-style test: lateral open loop + longitudinal gamma-only PD balance.
mate_test=strcmp(run_mode,'mate_open_loop');
% 'oscillation_candidate': verified near 0.2/1 Hz (mr=0.272 kg).
% 'current_mass': same small-lean test but keep model_parameters.m rod mass.
% 'configured': keep all initial conditions and gains from config files.
mate_case='oscillation_candidate';
if mate_test
    experiment.controller.mode='lateral_open_loop';
    experiment.controller.hold_gamma_zero=false;
    experiment.controller.gamma_ref=0;
    experiment.controller.gamma_dot_ref=0;
    experiment.parameters.BR=0;
    experiment.parameters.BP=0;
    if ismember(mate_case,{'oscillation_candidate','current_mass'})
        if strcmp(mate_case,'oscillation_candidate')
            experiment.parameters.mr=0.272;
        end
        % Only mr changes; JR is retained (Mate's exact inertia is unknown).
        experiment.controller.kp_gamma=3;
        experiment.controller.kd_gamma=0.8;
        experiment.settings.forward_speed0=2.375;
        experiment.settings.theta0=1*pi/180;
        experiment.settings.gamma0=0;
        experiment.settings.r0=0;
        experiment.settings.theta_dot0=0; experiment.settings.r_dot0=0;
        experiment.settings.gamma_dot0=0;
        experiment.settings.psi0=0; experiment.settings.psi_dot0=0;
        experiment.settings.phi0=0; experiment.settings.xG0=0; experiment.settings.yG0=0;
        experiment.settings.t_end=15;
        experiment.sweep.enabled=false;
    elseif ~strcmp(mate_case,'configured')
        error('unicycle:MateCase','Unknown mate_case: %s',mate_case);
    end
    fprintf('Mate test case: %s; theta0=%g deg; gamma0=%g deg\n',mate_case, ...
        experiment.settings.theta0*180/pi,experiment.settings.gamma0*180/pi);
    fprintf('Lateral open loop: F=0. Longitudinal: gamma/gamma_dot PD only.\n');
    fprintf('BR=%g, BP=%g, mr=%g kg; kp_gamma=%g, kd_gamma=%g; initial speed=%g m/s\n', ...
        experiment.parameters.BR,experiment.parameters.BP,experiment.parameters.mr, ...
        experiment.controller.kp_gamma,experiment.controller.kd_gamma, ...
        experiment.settings.forward_speed0);
end
results=run_experiment(experiment);
plot_simulation(results);

% add psi
figure('Name','Full nonlinear unicycle - psi');
hold on;
for j=1:numel(results)
    plot(results(j).t,results(j).X(:,6)*180/pi,'LineWidth',1.2, ...
        'DisplayName',results(j).label);
end
xlabel('Time [s]'); ylabel('psi [deg]'); grid on;
legend('show','Interpreter','none','Location','best');
if isfield(results(1).controller,'reference_rate'), plot_heading(results); end
result=results(1); % backward-compatible first run; all cases are in results
% results(k): t, X, U, parameters, controller (including generated K),
% settings, design, label, sweep_value, and tilt-event information.
