function study = study_full_model_manifold_control(speed_sequence,simulation_speeds,rod_damping)
%STUDY_FULL_MODEL_MANIFOLD_CONTROL
% Sweep local transverse stability along the full-model turning manifold,
% Simulate a speed sweep; show up to five representative time responses.
%
% Example:
%   study = study_full_model_manifold_control();
%   study = study_full_model_manifold_control([1.4 1.5 1.6],[1.4 1.5 1.6]);
%   study = study_full_model_manifold_control([],[],3); % B_R = 3 N*s/m
% Speeds are forward speeds in m/s (not phi-dot).
% rod_damping is physical rod damping, not a feedback gain.

root=fileparts(fileparts(fileparts(mfilename('fullpath'))));
addpath(fullfile(root,'config'),fullfile(root,'model'), ...
    fullfile(root,'analysis','manifold'));

if nargin<1 || isempty(speed_sequence)
    speed_sequence=1.3:0.1:4.7;
end
if nargin<2 || isempty(simulation_speeds)
    simulation_speeds=speed_sequence;
end
if nargin<3 || isempty(rod_damping)
    rod_damping=0;%6.5; % B_R [N*s/m]; editable default
end
validateattributes(rod_damping,{'numeric'}, ...
    {'scalar','real','finite','nonnegative'});
validateattributes(speed_sequence,{'numeric'}, ...
    {'vector','real','finite','positive','nonempty'});
validateattributes(simulation_speeds,{'numeric'}, ...
    {'vector','real','finite','positive','nonempty'});
speed_sequence=sort(unique(speed_sequence(:).'));
simulation_speeds=sort(unique(simulation_speeds(:).'));

% Analyze an upright turning branch. The helper picks the yaw rate
% on the branch that matches the initial state's uncontrollable mode.
theta0_deg=5;
theta_target_deg=0;
t_end=5;
n=numel(speed_sequence);
branch_reports=cell(1,n);
omega=NaN(1,n); r_eq=omega; force_eq=omega; torque_eq=omega;
pole_count=8;
lambda=complex(NaN(pole_count,n));
status=strings(1,n);

fprintf('Full-model manifold linear sweep: %d speeds\n',n);
for k=1:n
    opt=struct('speed',speed_sequence(k),'linearize_only',true,'rod_damping',rod_damping);
    try
        branch_reports{k}=verify_turning_manifold_approach( ...
            theta_target_deg,[],false,theta0_deg,t_end,opt);
        q=branch_reports{k};
        omega(k)=q.omega;
        r_eq(k)=q.x_equilibrium(9);
        force_eq(k)=q.u_equilibrium(1);
        torque_eq(k)=q.u_equilibrium(2);
        lambda(:,k)=q.closed_poles(:);
        status(k)="ok";
        fprintf('  %d/%d: v=%.3f m/s, Omega=%.4f rad/s, rank=%d/9\n', ...
            k,n,speed_sequence(k),q.omega,q.controllable_rank);
    catch err
        status(k)="failed";
        fprintf('  %d/%d: v=%.3f m/s failed: %s\n', ...
            k,n,speed_sequence(k),err.message);
    end
end

valid=status=="ok";
if ~any(valid)
    error('No speed points produced a valid manifold linearization.');
end
lambda=track_branches(lambda);

simulations=cell(size(simulation_speeds));
final_theta_deg=NaN(size(simulation_speeds));
first_peak_theta_deg=final_theta_deg;
first_peak_time_s=final_theta_deg;
final_r_m=final_theta_deg; final_psi_dot=final_theta_deg;
final_relative_rate_norm=final_theta_deg;
reached_target=false(size(simulation_speeds));
force_saturation_fraction=final_theta_deg;
torque_saturation_fraction=final_theta_deg;
completed=false(size(simulation_speeds));
duration_s=final_theta_deg;
theta_outward_overshoot_deg=final_theta_deg;
rod_target_overshoot_m=final_theta_deg;
max_abs_rod_m=final_theta_deg;
rod_terminal_error_m=final_theta_deg;
simulation_status=strings(size(simulation_speeds));
fprintf('Selected nonlinear simulations: %s m/s\n', ...
    sprintf('%.3f ',simulation_speeds));
for k=1:numel(simulation_speeds)
    opt=struct('speed',simulation_speeds(k),'rod_damping',rod_damping);
    try
        q=verify_turning_manifold_approach( ...
            theta_target_deg,[],false,theta0_deg,t_end,opt);
    catch err
        simulation_status(k)="failed: "+string(err.message);
        fprintf('  v=%.3f failed: %s\n',simulation_speeds(k),err.message);
        continue
    end
    q.psi_dot=q.X(:,3)./cos(q.X(:,7));
    q.force_saturated=abs(q.U(:,1))>=30*(1-1e-8);
    q.torque_saturated=abs(q.U(:,2))>=15*(1-1e-8);
    q.force_saturation_fraction=mean(q.force_saturated);
    q.torque_saturation_fraction=mean(q.torque_saturated);
    simulations{k}=q;
    final_theta_deg(k)=rad2deg(q.X(end,7));
    theta_trace_deg=rad2deg(q.X(:,7));
    duration_s(k)=q.t(end);
    completed(k)=abs(q.t(end)-t_end)<1e-6;
    simulation_status(k)="completed";
    if ~completed(k), simulation_status(k)="stopped early"; end
    outward=sign(theta0_deg-theta_target_deg);
    if outward==0, outward=1; end
    theta_outward_overshoot_deg(k)=max(0,max(outward*(theta_trace_deg-theta0_deg)));
    rod_direction=sign(q.x_equilibrium(9)-q.X(1,9));
    if rod_direction==0
        rod_target_overshoot_m(k)=max(abs(q.X(:,9)-q.x_equilibrium(9)));
    else
        rod_target_overshoot_m(k)=max(0,max(rod_direction*(q.X(:,9)-q.x_equilibrium(9))));
    end
    max_abs_rod_m(k)=max(abs(q.X(:,9)));
    rod_terminal_error_m(k)=q.X(end,9)-q.x_equilibrium(9);
    dtheta=diff(theta_trace_deg);
    peak_idx=find(dtheta(1:end-1)>0 & dtheta(2:end)<=0,1,'first')+1;
    if isempty(peak_idx)
        [first_peak_theta_deg(k),peak_idx]=max(theta_trace_deg);
    else
        first_peak_theta_deg(k)=theta_trace_deg(peak_idx);
    end
    first_peak_time_s(k)=q.t(peak_idx);
    final_r_m(k)=q.X(end,9);
    final_psi_dot(k)=q.psi_dot(end);
    final_relative_rate_norm(k)=q.final_relative_rate_norm;
    reached_target(k)=q.reached_turning_manifold;
    force_saturation_fraction(k)=q.force_saturation_fraction;
    torque_saturation_fraction(k)=q.torque_saturation_fraction;
    fprintf('  v=%.3f: reached=%d, Omega*=%.4f, max|F|=%.3f N, max|M2|=%.3f N*m\n', ...
        simulation_speeds(k),q.reached_turning_manifold,q.omega, ...
        max(abs(q.U(:,1))),max(abs(q.U(:,2))));
end
initial_theta_overshoot_deg=first_peak_theta_deg(:)-theta0_deg;
simulation_summary=table(simulation_speeds(:),first_peak_time_s(:), ...
    first_peak_theta_deg(:),initial_theta_overshoot_deg,final_theta_deg(:), ...
    (final_theta_deg(:)-theta_target_deg),final_r_m(:),final_psi_dot(:), ...
    final_relative_rate_norm(:),reached_target(:), ...
    force_saturation_fraction(:),torque_saturation_fraction(:), ...
    'VariableNames',{'speed_m_s','first_theta_peak_time_s', ...
    'first_theta_peak_deg','overshoot_above_initial_theta_deg', ...
    'final_theta_deg','theta_error_deg', ...
    'final_r_m','final_psi_dot_rad_s','final_relative_rate_norm', ...
    'reached_target','force_saturation_fraction','torque_saturation_fraction'});
% These maxima cover the simulated interval; incomplete runs are excluded
% from the speed-trend curves. O_theta is excess above the initial angle.
simulation_summary.completed=completed(:);
simulation_summary.duration_s=duration_s(:);
simulation_summary.status=simulation_status(:);
simulation_summary.theta_outward_overshoot_deg=theta_outward_overshoot_deg(:);
simulation_summary.theta_overshoot_ratio=theta_outward_overshoot_deg(:)/abs(theta0_deg);
if theta0_deg==0, simulation_summary.theta_overshoot_ratio(:)=NaN; end
simulation_summary.rod_target_overshoot_m=rod_target_overshoot_m(:);
simulation_summary.max_abs_rod_m=max_abs_rod_m(:);
simulation_summary.rod_terminal_error_m=rod_terminal_error_m(:);
simulation_summary.rod_damping_Ns_m=repmat(rod_damping,height(simulation_summary),1);
disp(simulation_summary);

study.speed_sequence=speed_sequence;
study.rod_damping=rod_damping;
study.theta0_deg=theta0_deg;
study.theta_target_deg=theta_target_deg;
study.omega=omega;
study.r_equilibrium=r_eq;
study.force_equilibrium=force_eq;
study.torque_equilibrium=torque_eq;
study.transverse_poles=lambda;
study.linearization_status=status;
study.linearization_reports=branch_reports;
study.simulation_speeds=simulation_speeds;
study.simulations=simulations;
study.simulation_summary=simulation_summary;
study.first_peak_theta_deg=first_peak_theta_deg;
study.initial_theta_overshoot_deg=initial_theta_overshoot_deg;
% Keep time-response plots readable while all simulated speeds enter metrics.
available=find(~cellfun(@isempty,simulations));
selected= available;%available(unique(round(linspace(1,numel(available),min(5,numel(available))))));
shown_speeds=simulation_speeds(selected);
shown_runs=simulations(selected);

fig=figure('Color','w','Name','Full-model manifold control study', ...
    'Position',[100 80 1200 850]);
tabs=uitabgroup(fig);
branch_tab=uitab(tabs,'Title','Manifold operating point');
sim_tab=uitab(tabs,'Title','Selected simulations');
input_tab=uitab(tabs,'Title','Other states and inputs');
eig_tab=uitab(tabs,'Title','Linearized eigenvalues');
peak_tab=uitab(tabs,'Title','Initial theta peak');
overshoot_tab=uitab(tabs,'Title','Overshoot vs speed');
plot_branch(branch_tab,speed_sequence,omega,r_eq,force_eq,torque_eq,valid);
plot_selected_simulations(sim_tab,shown_speeds,shown_runs,theta_target_deg);
plot_other_states(input_tab,shown_speeds,shown_runs);
plot_poles(eig_tab,speed_sequence,lambda,valid);
display_peak=first_peak_theta_deg; display_peak(~completed)=NaN;
plot_theta_peak(peak_tab,simulation_speeds,display_peak,theta0_deg);
plot_overshoot(overshoot_tab,simulation_summary,theta0_deg,t_end);
tabs.SelectedTab=overshoot_tab;

out_mat=fullfile(root,'results','full_model_manifold_control_study.mat');
save(out_mat,'study');
fprintf('Saved manifold-control study: %s\n',out_mat);
save_report_artifacts(root,speed_sequence,omega,r_eq,force_eq,torque_eq, ...
    lambda,valid,shown_speeds,shown_runs,simulation_summary, ...
    theta_target_deg,display_peak,theta0_deg,t_end);
end

function save_report_artifacts(root,speed,omega,r_eq,force_eq,torque_eq, ...
    lambda,valid,sim_speeds,simulations,simulation_summary,theta_target_deg, ...
    first_peak_theta_deg,theta0_deg,t_end)
out_dir=fullfile(root,'results','full_model_manifold_control_study');
if ~isfolder(out_dir), mkdir(out_dir); end
fig_dir=fullfile(out_dir,'figures');
if ~isfolder(fig_dir), mkdir(fig_dir); end

max_real_pole=NaN(size(speed));
for k=find(valid)
    max_real_pole(k)=max(real(lambda(:,k)));
end
operating_points=table(speed(:),omega(:),r_eq(:),force_eq(:),torque_eq(:), ...
    max_real_pole(:),valid(:), ...
    'VariableNames',{'speed_m_s','yaw_rate_rad_s','rod_position_m', ...
    'feedforward_force_N','feedforward_torque_Nm', ...
    'max_real_transverse_pole','linearization_valid'});
writetable(operating_points,fullfile(out_dir,'manifold_operating_points.csv'));
writetable(simulation_summary,fullfile(out_dir,'simulation_summary.csv'));

report_figures={ ...
    '01_manifold_operating_point.png', ...
    '02_selected_nonlinear_responses.png', ...
    '03_other_states_inputs_and_xy_path.png', ...
    '04_linearized_transverse_eigenvalues.png', ...
    '05_initial_theta_peak_vs_speed.png', ...
    '06_overshoot_vs_speed.png'};
plotters={ ...
    @(f)plot_branch(f,speed,omega,r_eq,force_eq,torque_eq,valid), ...
    @(f)plot_selected_simulations(f,sim_speeds,simulations,theta_target_deg), ...
    @(f)plot_other_states(f,sim_speeds,simulations), ...
    @(f)plot_poles(f,speed,lambda,valid), ...
    @(f)plot_theta_peak(f,simulation_summary.speed_m_s,first_peak_theta_deg,theta0_deg), ...
    @(f)plot_overshoot(f,simulation_summary,theta0_deg,t_end)};
for k=1:numel(report_figures)
    f=figure('Visible','off','Color','w','Position',[100 80 1200 850]);
    plotters{k}(f);
    drawnow;
    exportgraphics(f,fullfile(fig_dir,report_figures{k}),'Resolution',220);
    close(f);
end
fprintf('Report figures and tables saved under: %s\n',out_dir);
end

function plot_overshoot(parent,s,theta0_deg,horizon)
tl=tiledlayout(parent,2,2,'TileSpacing','compact','Padding','compact');
values={s.theta_outward_overshoot_deg,100*s.theta_overshoot_ratio, ...
    s.rod_target_overshoot_m,s.max_abs_rod_m};
labels={'Outward lean overshoot (deg)','Lean overshoot / |initial lean| (%)', ...
    'Rod excess beyond target (m)','Maximum |r| (m)'};
for j=1:4
    ax=nexttile(tl); hold(ax,'on'); grid(ax,'on');
    y=values{j}; y(~s.completed)=NaN;
    plot(ax,s.speed_m_s,y,'-o','LineWidth',2,'DisplayName','Completed');
    stopped=~s.completed & isfinite(values{j});
    if any(stopped)
        plot(ax,s.speed_m_s(stopped),values{j}(stopped),'rx','LineWidth',2, ...
            'MarkerSize',9,'DisplayName','Stopped: partial-trajectory maximum');
    end
    xlabel(ax,'Initial/design speed (m/s)'); ylabel(ax,labels{j});
    legend(ax,'Location','best');
end
title(tl,sprintf('Initial lean = %g deg; requested horizon = %g s; sampled maxima',theta0_deg,horizon));
end

function tracked=track_branches(lambda)
tracked=lambda;
for k=2:size(lambda,2)
    prev=tracked(:,k-1); curr=lambda(:,k);
    a=find(isfinite(prev)); b=find(isfinite(curr));
    if numel(a)~=numel(b) || isempty(a), continue; end
    order=zeros(size(b)); remaining=b(:).';
    for j=1:numel(a)
        [~,ii]=min(abs(prev(a(j))-curr(remaining)));
        order(j)=remaining(ii); remaining(ii)=[];
    end
    tracked(a,k)=curr(order);
end
end

function plot_poles(parent,speed,lambda,valid)
tl=tiledlayout(parent,3,1,'TileSpacing','compact','Padding','compact');
ax1=nexttile(tl); hold(ax1,'on'); grid(ax1,'on'); box(ax1,'on');
ax2=nexttile(tl); hold(ax2,'on'); grid(ax2,'on'); box(ax2,'on');
ax3=nexttile(tl); hold(ax3,'on'); grid(ax3,'on'); box(ax3,'on');
colors=lines(size(lambda,1));
fast=median(abs(real(lambda(:,valid))),2)>5;
for j=1:size(lambda,1)
    use=valid & isfinite(lambda(j,:));
    if fast(j)
        plot(ax2,speed(use),real(lambda(j,use)),'-','Color',colors(j,:), ...
            'LineWidth',1.2);
    else
        plot(ax1,speed(use),real(lambda(j,use)),'-','Color',colors(j,:), ...
            'LineWidth',1.2);
    end
    plot(ax3,speed(use),imag(lambda(j,use)),'-','Color',colors(j,:), ...
        'LineWidth',1.2);
end
yline(ax1,0,'k--','HandleVisibility','off');
xlabel(ax1,'Forward speed v (m/s)'); ylabel(ax1,'Re(lambda) (1/s)');
title(ax1,'Real parts of slower transverse modes');
xlabel(ax2,'Forward speed v (m/s)'); ylabel(ax2,'Re(lambda) (1/s)');
title(ax2,'Real parts of fast mode(s)');
xlabel(ax3,'Forward speed v (m/s)'); ylabel(ax3,'Im(lambda) (rad/s)');
title(ax3,'Imaginary parts of all transverse modes');
title(tl,'Local LQR poles on the turning manifold (8 controllable directions)');
end

function plot_branch(parent,speed,omega,r_eq,force_eq,torque_eq,valid)
tl=tiledlayout(parent,2,2,'TileSpacing','compact','Padding','compact');
series={omega,r_eq,force_eq,torque_eq};
labels={'Selected yaw rate Omega* (rad/s)','Equilibrium rod position r* (m)', ...
    'Feedforward force F* (N)','Feedforward torque M_2* (N m)'};
for j=1:4
    ax=nexttile(tl); plot(ax,speed(valid),series{j}(valid),'-o','LineWidth',1.2);
    grid(ax,'on'); xlabel(ax,'Forward speed v (m/s)'); ylabel(ax,labels{j});
end
title(tl,'Operating point selected on the full-model manifold');
end

function plot_selected_simulations(parent,speeds,runs,theta_target_deg)
tl=tiledlayout(parent,3,1,'TileSpacing','compact','Padding','compact');
ax1=nexttile(tl); hold(ax1,'on'); grid(ax1,'on'); box(ax1,'on');
ax2=nexttile(tl); hold(ax2,'on'); grid(ax2,'on'); box(ax2,'on');
ax3=nexttile(tl); hold(ax3,'on'); grid(ax3,'on'); box(ax3,'on');
colors=turbo(numel(speeds));
for k=1:numel(speeds)
    q=runs{k};
    plot(ax1,q.t,rad2deg(q.X(:,7)),'Color',colors(k,:),'LineWidth',2);
    plot(ax2,q.t,q.X(:,9),'Color',colors(k,:),'LineWidth',2);
    plot(ax3,q.t,q.psi_dot,'Color',colors(k,:),'LineWidth',2);
end
yline(ax1,theta_target_deg,'k--','HandleVisibility','off');
xlabel(ax1,'Time (s)'); ylabel(ax1,'theta (deg)'); title(ax1,'Lean angle');
xlabel(ax2,'Time (s)'); ylabel(ax2,'r (m)'); title(ax2,'Rod position');
xlabel(ax3,'Time (s)'); ylabel(ax3,'d psi/dt (rad/s)'); title(ax3,'Yaw rate');
legend(ax1,arrayfun(@(v)sprintf('v_0 = %.3f m/s',v),speeds, ...
    'UniformOutput',false),'Location','best');
title(tl,'Nonlinear full-model trajectories for selected speeds');
end

function plot_theta_peak(parent,speeds,peak_theta_deg,theta0_deg)
ax=axes(parent); hold(ax,'on'); grid(ax,'on'); box(ax,'on');
plot(ax,speeds,peak_theta_deg,'-o','LineWidth',1.4,'MarkerSize',4);
yline(ax,theta0_deg,'k--','Initial lean','HandleVisibility','off');
xlabel(ax,'Forward speed v (m/s)');
ylabel(ax,'First local maximum of theta (deg)');
title(ax,'Initial lean overshoot versus forward speed');
if all(peak_theta_deg>theta0_deg)
    text(ax,0.98,0.96,'No overshoot sign change in tested range', ...
        'Units','normalized','HorizontalAlignment','right', ...
        'VerticalAlignment','top','BackgroundColor','w');
end
end

function plot_other_states(parent,speeds,runs)
idx=[1 2 3 4 5 6 8 10];
names={'sigma_1 (rad/s)','sigma_2 (rad/s)','sigma_3 (rad/s)', ...
    'sigma_r (rad/s)','sigma_g (rad/s)','psi (deg)','phi (rad)', ...
    'gamma (deg)'};
colors=turbo(numel(speeds));
tl=tiledlayout(parent,6,2,'TileSpacing','compact','Padding','compact');
for j=1:10
    ax=nexttile(tl); hold(ax,'on'); grid(ax,'on'); box(ax,'on');
    for k=1:numel(speeds)
        q=runs{k};
        if j<=8
            y=q.X(:,idx(j));
            if idx(j)==6 || idx(j)==10, y=rad2deg(y); end
            plot(ax,q.t,y,'Color',colors(k,:),'LineWidth',2);
        elseif j==9
            plot(ax,q.X(:,11),q.X(:,12),'Color',colors(k,:),'LineWidth',2);
        else
            y=q.U(:,1);
            plot(ax,q.t,y,'Color',colors(k,:),'LineWidth',2);
        end
    end
    if j<=8
        title(ax,names{j});
        xlabel(ax,'Time (s)');
    elseif j==9
        title(ax,'Ground-plane trajectory');
        xlabel(ax,'x_G (m)'); ylabel(ax,'y_G (m)');
        axis(ax,'equal');
    else
        title(ax,'Force F (N)');
        xlabel(ax,'Time (s)');
    end
end
ax=nexttile(tl,11,[1 2]); hold(ax,'on'); grid(ax,'on'); box(ax,'on');
for k=1:numel(speeds)
    q=runs{k};
    plot(ax,q.t,q.U(:,2),'Color',colors(k,:),'LineWidth',2);
end
title(ax,'Torque M_2 (N m)'); xlabel(ax,'Time (s)');
title(tl,'Other states, ground-plane path, and actuator inputs');
end
