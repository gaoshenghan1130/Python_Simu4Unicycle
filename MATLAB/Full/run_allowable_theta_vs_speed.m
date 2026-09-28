% Maximum allowable initial lean angle versus constant forward speed.
%
% Definition used here:
%   theta0 is allowable when the full nonlinear simulation completes the
%   requested duration without reaching the lean limit AND becomes
%   quasi-steady within the rate/error thresholds configured below.
%
% At each speed, the model is linearized once at the corresponding straight
% constant-speed motion and a new K is calculated. K is fixed during that
% simulation. Initial speed equals design/reference speed.

project_root=fileparts(mfilename('fullpath'));
addpath(fullfile(project_root,'config'),fullfile(project_root,'model'), ...
    fullfile(project_root,'controllers'),fullfile(project_root,'simulation'), ...
    fullfile(project_root,'analysis'));

%% Study settings
speeds=[0.1:0.1:1,1.25:0.25:3]; % m/s; edit as needed
simulation_time=60;              % s
maximum_test_angle=30;           % deg; reported as capped if still allowable
initial_search_angle=0.05;       % deg
angle_tolerance=0.01;            % deg; binary-search resolution
fixed_step=0.001;                % s; forward-Euler integration step

% A run must also become quasi-steady over its final tail_duration seconds.
acceptance.tail_duration=5;              % s
acceptance.theta_dot_limit=0.01;         % deg/s
acceptance.r_dot_limit=1e-4;             % m/s
acceptance.psi_dot_limit=0.05;           % deg/s
acceptance.gamma_dot_limit=0.01;         % deg/s
acceptance.speed_error_limit=1e-3;       % m/s

% No rod-stroke or actuator constraint is included in this definition.

%% Common configuration
base=simulation_preset('chi_epsilon_balance');
base.design.method='acker'; % no Control System Toolbox required
base.design.chi_epsilon_poles=[-1.4 -1.6 -1.8 -2 -2.2 -2.4];
base.design.longitudinal=[-0.8 -0.95 -1.05 -1.2];
base.settings.t_end=simulation_time;
base.settings.fixed_step=fixed_step;
base.settings.psi0=0;
base.settings.psi_dot0=0;
base.settings.r0=0;
base.settings.theta_dot0=0;
base.settings.r_dot0=0;
base.settings.gamma0=0;
base.settings.gamma_dot0=0;
base.settings.phi0=0;
base.settings.xG0=0;
base.settings.yG0=0;
base.sweep.enabled=false;

positive_limit=zeros(size(speeds));
negative_limit=zeros(size(speeds));
positive_capped=false(size(speeds));
negative_capped=false(size(speeds));
gain_inf_norm=zeros(size(speeds));
boundary_runs=cell(numel(speeds),2); % column 1: negative, column 2: positive

%% Re-linearize once per speed, then search the connected allowable interval
for speed_index=1:numel(speeds)
    speed=speeds(speed_index);
    design=base.design;
    design.forward_speed=speed;
    controller=generate_controller(base.parameters,base.controller,design);
    gain_inf_norm(speed_index)=norm(controller.K,inf);

    settings=base.settings;
    settings.forward_speed0=speed;

    fprintf('\nSpeed %.6g m/s: searching theta0 limits...\n',speed);
    [positive_limit(speed_index),positive_capped(speed_index), ...
        boundary_runs{speed_index,2}]=find_theta_limit( ...
        +1,base.parameters,controller,settings, ...
        initial_search_angle,maximum_test_angle,angle_tolerance,acceptance);
    [negative_limit(speed_index),negative_capped(speed_index), ...
        boundary_runs{speed_index,1}]=find_theta_limit( ...
        -1,base.parameters,controller,settings, ...
        initial_search_angle,maximum_test_angle,angle_tolerance,acceptance);

    fprintf('  allowable theta0 interval: [%+.4f, %+.4f] deg%s\n', ...
        -negative_limit(speed_index),positive_limit(speed_index), ...
        capped_note(negative_capped(speed_index)||positive_capped(speed_index)));
end

%% Results table and plot
summary=table(speeds(:),-negative_limit(:),positive_limit(:), ...
    negative_capped(:),positive_capped(:),gain_inf_norm(:), ...
    'VariableNames',{'speed_mps','minimum_theta0_deg','maximum_theta0_deg', ...
    'negative_limit_capped','positive_limit_capped','gain_inf_norm'});
disp(summary);

figure('Name','Allowable initial lean versus speed','Position',[100 100 950 620]);
fill([speeds,fliplr(speeds)], ...
    [positive_limit,-fliplr(negative_limit)], ...
    [0.76 0.86 0.98],'EdgeColor','none','FaceAlpha',0.65, ...
    'DisplayName','allowable theta(0) interval');
hold on;
plot(speeds,positive_limit,'Color',[0.02 0.12 0.36], ...
    'LineWidth',1.8,'Marker','o','DisplayName','positive boundary');
plot(speeds,-negative_limit,'Color',[0.02 0.12 0.36], ...
    'LineWidth',1.8,'Marker','o','DisplayName','negative boundary');
yline(0,'k-','HandleVisibility','off');
grid on;
xlabel('Initial = design/reference speed [m/s]');
ylabel('Allowable initial lean theta(0) [deg]');
title(sprintf(['Completed %.0f s and quasi-steady over the final %.0f s ' ...
    '(lean limit %.0f deg)'],simulation_time,acceptance.tail_duration, ...
    base.settings.tilt_limit*180/pi));
legend('Location','best');

%% Save data and figure
output_dir=fullfile(project_root,'results','allowable_theta_vs_speed');
if ~exist(output_dir,'dir'), mkdir(output_dir); end
writetable(summary,fullfile(output_dir,'summary.csv'));
save(fullfile(output_dir,'study.mat'), ...
    'summary','boundary_runs','base','speeds','simulation_time', ...
    'maximum_test_angle','initial_search_angle','angle_tolerance','acceptance');
exportgraphics(gcf,fullfile(output_dir,'allowable_theta_vs_speed.png'), ...
    'Resolution',180);
fprintf('\nSaved allowable-theta study to %s\n',output_dir);

function [limit,capped,boundary_run]=find_theta_limit( ...
    direction,p,controller,settings,initial_angle,max_angle,tolerance,acceptance)
% Find the edge of the allowable interval connected to theta0=0.
low=0;
high=min(initial_angle,max_angle);
boundary_run=run_theta_case(direction*low,p,controller,settings);
if ~is_allowable(boundary_run,p,settings,acceptance)
    limit=0; capped=false;
    return;
end

% Expand outward until the first failure or the requested search cap.
while true
    candidate=run_theta_case(direction*high,p,controller,settings);
    if ~is_allowable(candidate,p,settings,acceptance)
        break;
    end
    low=high;
    boundary_run=candidate;
    if high>=max_angle
        limit=max_angle; capped=true;
        return;
    end
    high=min(2*high,max_angle);
end

% Refine the first pass/fail boundary.
while high-low>tolerance
    middle=(low+high)/2;
    candidate=run_theta_case(direction*middle,p,controller,settings);
    if is_allowable(candidate,p,settings,acceptance)
        low=middle;
        boundary_run=candidate;
    else
        high=middle;
    end
end
limit=low;
capped=false;
end

function result=run_theta_case(theta_deg,p,controller,settings)
trial=settings;
trial.theta0=theta_deg*pi/180;
result=simulate_fixed_gain(p,controller,trial);
result.theta0_deg=theta_deg;
end

function result=simulate_fixed_gain(p,controller,settings)
% Fast fixed-step simulation for the many basin-boundary trials.
dt=settings.fixed_step;
x=initial_state(settings,p);
num_steps=ceil(settings.t_end/dt);
t=(0:num_steps)'*dt;
t(end)=settings.t_end;
X=zeros(num_steps+1,12);
U=zeros(num_steps+1,2);
X(1,:)=x';
event_time=[]; event_state=[]; event_index=[];
last_sample=num_steps+1;

for sample=1:num_steps
    step_dt=t(sample+1)-t(sample);
    u=controller_output(t(sample),x,p,controller);
    U(sample,:)=u';
    x=x+step_dt*model_rhs(t(sample),x,u,p);
    X(sample+1,:)=x';
    if ~all(isfinite(x)) || abs(x(7))>=settings.tilt_limit
        last_sample=sample+1;
        event_time=t(last_sample);
        event_state=x';
        event_index=1;
        break;
    end
end

t=t(1:last_sample);
X=X(1:last_sample,:);
U=U(1:last_sample,:);
if all(isfinite(X(end,:)))
    U(end,:)=controller_output(t(end),X(end,:)',p,controller)';
end
result.t=t;
result.X=X;
result.U=U;
result.event_time=event_time;
result.event_state=event_state;
result.event_index=event_index;
result.parameters=p;
result.controller=controller;
result.settings=settings;
result.integration_method='fixed-step forward Euler';
end

function allowed=is_allowable(result,p,settings,acceptance)
allowed=isempty(result.event_time) ...
    && abs(result.t(end)-settings.t_end)<1e-8 ...
    && all(isfinite(result.X(:))) ...
    && all(isfinite(result.U(:)));
if ~allowed, return; end

tail=result.t>=settings.t_end-acceptance.tail_duration;
X=result.X(tail,:);
theta_dot=abs(X(:,1))*180/pi;
r_dot=abs(p.R*X(:,1)+X(:,4));
psi_dot=abs(X(:,3)./cos(X(:,7)))*180/pi;
gamma_dot=abs(X(:,5)-X(:,3).*tan(X(:,7)))*180/pi;
speed_error=abs(p.R*X(:,2)-settings.forward_speed0);
allowed=max(theta_dot)<=acceptance.theta_dot_limit ...
    && max(r_dot)<=acceptance.r_dot_limit ...
    && max(psi_dot)<=acceptance.psi_dot_limit ...
    && max(gamma_dot)<=acceptance.gamma_dot_limit ...
    && max(speed_error)<=acceptance.speed_error_limit;
end

function note=capped_note(capped)
if capped
    note=' (at least one side reached the search cap)';
else
    note='';
end
end
