% Online reproduction of Mate's repeated-linearization procedure.
%
% During each nonlinear simulation, EVERY controller evaluation:
%   1. reads the current signed longitudinal speed v(t)=R*sigma2(t);
%   2. interpolates K(t) from pole-placement designs precomputed at nearby
%      straight-rolling speeds;
%   3. applies u(t)=-K(t)*(x(t)-x_ref(t));
%   4. advances the nonlinear state with one fixed forward-Euler step.
%
% Initial speed equals target speed in each run. The target remains fixed;
% the linearization speed and K follow the simulated speed.

project_root=fileparts(mfilename('fullpath'));
addpath(fullfile(project_root,'config'),fullfile(project_root,'model'), ...
    fullfile(project_root,'controllers'),fullfile(project_root,'simulation'), ...
    fullfile(project_root,'analysis'));

%% Study settings
% Edit this list to focus the sweep around the calculated critical speed.
target_speeds=[2.375 2.5]; % m/s

% Compare Mate-like zero lean against the nonzero lean test.
case_names={'theta0=0 deg','theta0=1 deg'};
theta0_deg=[0 1];

% Rolling path states lose controllability at zero speed. Stop a run before
% reaching zero instead of silently switching to a stationary controller.
min_design_speed=0.02; % m/s

% Fixed-step integration, equivalent to the project's timeConstantSimu.
fixed_step=0.001; % s

% Pole placement is precomputed on this speed grid. A smaller step follows
% the exact K(v) curve more closely but takes longer to initialize.
gain_schedule_step=0.01; % m/s
max_schedule_speed=max(target_speeds)+1; % allow transient overspeed

%% Common model, poles, and initial-state settings
base=simulation_preset('chi_epsilon_balance');
base.design.method='acker'; % local implementation; no toolbox required
base.design.chi_epsilon_poles=[-1.4 -1.6 -1.8 -2 -2.2 -2.4];
base.design.longitudinal=[-0.8 -0.95 -1.05 -1.2];
base.settings.t_end=30;
base.settings.fixed_step=fixed_step;
base.settings.psi0=0;
base.settings.psi_dot0=0;
base.settings.r0=0;
base.settings.theta_dot0=0;
base.settings.r_dot0=0;
base.settings.gamma0=0;
base.settings.gamma_dot0=0;
base.settings.yG0=0;
base.sweep.enabled=false;

%% Precompute K(v) once, outside the fixed-step simulation
fprintf('Precomputing pole-placement gain schedule...\n');
gain_schedule=build_pole_gain_schedule(base.parameters,base.design, ...
    min_design_speed,max_schedule_speed,gain_schedule_step);
gain_schedule.exact=false;
fprintf('  %d designs from %.3g to %.3g m/s, step %.3g m/s.\n', ...
    numel(gain_schedule.speeds),gain_schedule.min_speed, ...
    gain_schedule.max_speed,gain_schedule.speed_step);

%% Run all target speeds and both initial-lean cases
num_cases=numel(case_names);
num_speeds=numel(target_speeds);
runs=cell(num_cases,num_speeds);
summary_rows=struct([]);
row_index=0;

for speed_index=1:num_speeds
    target_speed=target_speeds(speed_index);
    fprintf('\nTarget/initial speed %.6g m/s\n',target_speed);

    for case_index=1:num_cases
        settings=base.settings;
        settings.forward_speed0=target_speed;
        settings.theta0=theta0_deg(case_index)*pi/180;

        result=simulate_online_pole_placement( ...
            base.parameters,base.controller,base.design,settings, ...
            target_speed,min_design_speed,gain_schedule);
        result.label=sprintf('%s, target=%.6g m/s', ...
            case_names{case_index},target_speed);
        runs{case_index,speed_index}=result;

        row_index=row_index+1;
        summary_rows(row_index).case_name=case_names{case_index}; %#ok<SAGROW>
        summary_rows(row_index).target_speed_mps=target_speed;
        summary_rows(row_index).completed=isempty(result.event_time) && ...
            abs(result.t(end)-settings.t_end)<1e-8;
        summary_rows(row_index).stop_event=event_name(result.event_index);
        summary_rows(row_index).end_time_s=result.t(end);
        summary_rows(row_index).min_actual_speed_mps=min(result.current_speed);
        summary_rows(row_index).max_actual_speed_mps=max(result.current_speed);
        summary_rows(row_index).peak_theta_deg=max(abs(result.X(:,7)))*180/pi;
        summary_rows(row_index).final_theta_deg=result.X(end,7)*180/pi;
        summary_rows(row_index).peak_r_m=max(abs(result.X(:,9)));
        summary_rows(row_index).final_r_m=result.X(end,9);
        summary_rows(row_index).final_psi_deg=result.X(end,6)*180/pi;
        summary_rows(row_index).final_epsilon_m=result.X(end,12);
        summary_rows(row_index).final_speed_mps=result.current_speed(end);
        summary_rows(row_index).min_gain_inf_norm=min(result.gain_inf_norm);
        summary_rows(row_index).max_gain_inf_norm=max(result.gain_inf_norm);

        fprintf(['  %-12s: end=%7.3f s, speed=[%7.3f,%7.3f] m/s, ' ...
            'peak theta=%8.4f deg, stop=%s\n'],case_names{case_index}, ...
            result.t(end),min(result.current_speed),max(result.current_speed), ...
            summary_rows(row_index).peak_theta_deg, ...
            summary_rows(row_index).stop_event);
    end
end

summary=struct2table(summary_rows);

%% Time histories: one figure for each initial-lean case
for case_index=1:num_cases
    plot_online_case(runs(case_index,:),target_speeds,case_names{case_index});
end

%% Across-speed summary
figure('Name','Online relinearization summary','Position',[100 100 1050 760]);
for case_index=1:num_cases
    rows=strcmp(summary.case_name,case_names{case_index});
    subplot(2,2,1); hold on;
    plot(summary.target_speed_mps(rows),summary.peak_theta_deg(rows),'-o', ...
        'LineWidth',1.2,'DisplayName',case_names{case_index});
    subplot(2,2,2); hold on;
    plot(summary.target_speed_mps(rows),summary.final_theta_deg(rows),'-o', ...
        'LineWidth',1.2,'DisplayName',case_names{case_index});
    subplot(2,2,3); hold on;
    plot(summary.target_speed_mps(rows),summary.final_epsilon_m(rows),'-o', ...
        'LineWidth',1.2,'DisplayName',case_names{case_index});
    subplot(2,2,4); hold on;
    plot(summary.target_speed_mps(rows),summary.completed(rows),'-o', ...
        'LineWidth',1.2,'DisplayName',case_names{case_index});
end
summary_labels={'peak |theta| [deg]','final theta [deg]', ...
    'final epsilon [m]','completed 30 s'};
for panel=1:4
    subplot(2,2,panel); grid on; xlabel('Initial/target speed [m/s]');
    ylabel(summary_labels{panel}); legend('show','Location','best');
end

%% Save numerical results
output_dir=fullfile(project_root,'results','mate_online_relinearization');
if ~exist(output_dir,'dir'), mkdir(output_dir); end
writetable(summary,fullfile(output_dir,'summary.csv'));
save(fullfile(output_dir,'study.mat'), ...
    'runs','summary','target_speeds','case_names','theta0_deg', ...
    'min_design_speed','fixed_step','gain_schedule','base');
fprintf('\nSaved study data to %s\n',output_dir);

function name=event_name(indices)
if isempty(indices)
    name='none';
elseif indices(end)==1
    name='tilt_limit';
elseif indices(end)==2
    name='near_zero_speed';
else
    name=sprintf('event_%d',indices(end));
end
end

function plot_online_case(case_runs,target_speeds,case_name)
figure('Name',['Online relinearization: ' case_name], ...
    'Position',[80 80 1100 950]);
colors=speed_colors(target_speeds);
labels={'theta [deg]','r [m]','epsilon=yG [m]', ...
    'actual speed [m/s]','||K(t)||_inf'};
for speed_index=1:numel(target_speeds)
    result=case_runs{speed_index};
    data=[result.X(:,7)*180/pi,result.X(:,9),result.X(:,12), ...
        result.current_speed,result.gain_inf_norm];
    for panel=1:5
        subplot(5,1,panel); hold on;
        plot(result.t,data(:,panel),'Color',colors(speed_index,:), ...
            'LineWidth',1.05, ...
            'DisplayName',sprintf('target=%.6g',target_speeds(speed_index)));
        ylabel(labels{panel}); grid on;
    end
end
subplot(5,1,1); title(case_name); legend('show','Location','eastoutside');
subplot(5,1,5); xlabel('Time [s]');
end
