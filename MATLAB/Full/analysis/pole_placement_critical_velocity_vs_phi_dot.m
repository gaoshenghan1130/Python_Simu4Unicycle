function study = pole_placement_critical_velocity_vs_phi_dot(phi_dot_sequence)
%POLE_PLACEMENT_CRITICAL_VELOCITY_VS_PHI_DOT
% Re-linearize and re-design pole-placement gains at each rolling speed.
% Three designs are compared: rod-only, theta/r, and theta/r/psi feedback.
% Each design has its own figure window with eigenvalue and simulation tabs.
%
% Example:
%   study = pole_placement_critical_velocity_vs_phi_dot(2:0.05:16);
% Input is phi_dot in rad/s; the simulated/linearized speed is R*phi_dot.

if nargin < 1 || isempty(phi_dot_sequence)
    phi_dot_sequence = 2:0.5:16; % rad/s; edit this sequence as needed
end
validateattributes(phi_dot_sequence,{'numeric'}, ...
    {'vector','real','finite','positive'});
phi_dot_sequence=phi_dot_sequence(:).';

root=fileparts(fileparts(mfilename('fullpath')));
addpath(genpath(root));
p=model_parameters();
d0=pole_placement_settings();
d0.method='acker'; % works without Control System Toolbox
d0.lateral_states='balance_chi';

% Fixed pole sets, held constant over the whole speed sequence.
rod_poles=[-1.0 -1.2];                  % rod-only reduced design
theta_rod_poles=[-2.2 -2.25 -1.1 -1.0]; % theta/r, without psi feedback
theta_rod_psi_poles=[-2.2 -2.25 -1.1 -1.0 -1.3];
d0.chi_poles=theta_rod_psi_poles;
closed_states=1:12; % plot every eigenvalue, including neutral integrator modes
speeds=p.R*phi_dot_sequence;
if isempty(speeds)
    error('phi_dot_sequence must contain at least one speed.');
end
speed_min=speeds(1);
speed_max=speeds(end);
speed_count=numel(speeds);
simcfg.duration=18;
simcfg.output_dt=0.05;
simcfg.max_step=0.01;
simcfg.initial_tilt=0.01*pi/180;
simcfg.tilt_limit=45*pi/180;
simcfg.min_mass_matrix_rcond=1e-12;

lambda_rod=complex(NaN(numel(closed_states),numel(speeds)));
lambda_theta_rod_only=lambda_rod;
lambda_theta_rod=lambda_rod;
K_rod=NaN(numel(speeds),12);
K_theta_rod_only=NaN(numel(speeds),12);
K_theta_rod=NaN(numel(speeds),12);
actual_rod_poles=cell(size(speeds));
actual_theta_rod_only_poles=cell(size(speeds));
actual_theta_rod_poles=cell(size(speeds));
theta_rod_psi_pole_error=NaN(size(speeds));

fprintf('Linearizing and placing poles at %d speed points...\n',numel(speeds));
drawnow;
for k=1:numel(speeds)
    v=speeds(k);
    [A,B]=linearize_plant(v,p);

    % Rod-only pole placement on projected coordinates [r; r_dot].
    % r_dot = R*sigma_1 + sigma_r in the full model.
    C=zeros(2,12); C(1,9)=1; C(2,1)=p.R; C(2,4)=1;
    E=zeros(12,2); E(9,1)=1; E(4,2)=1;
    Ar=C*A*E; Br=C*B(:,1);
    Kr=acker_siso(Ar,Br,rod_poles);
    K=zeros(2,12);
    K(1,:)=Kr*C;

    % Keep the longitudinal loop configured, while re-designing it locally
    % at this same speed as part of the standard pole-placement controller.
    d=d0; d.forward_speed=v;
    c=controller_parameters(); c.mode='pole_placement';
    c=generate_controller(p,c,d);
    K(2,:)=c.K(2,:);
    K_rod(k,:)=K(1,:);
    lambda_rod(:,k)=closed_eigenvalues(A-B*K,closed_states);
    actual_rod_poles{k}=eig(Ar-Br*Kr);

    % Joint theta/r placement, leaving psi uncontrolled.
    dtr=d0; dtr.lateral_states='balance_rolling';
    dtr.rolling_poles=theta_rod_poles; dtr.forward_speed=v;
    ctr=controller_parameters(); ctr.mode='pole_placement';
    ctr=generate_controller(p,ctr,dtr);
    K_theta_rod_only(k,:)=ctr.K(1,:);
    lambda_theta_rod_only(:,k)=closed_eigenvalues(A-B*ctr.K,closed_states);
    actual_theta_rod_only_poles{k}=ctr.design_info.actual_lateral;

    % Joint theta/r/psi placement using the five-state rolling chi block.
    K_theta_rod(k,:)=c.K(1,:);
    lambda_theta_rod(:,k)=closed_eigenvalues(A-B*c.K,closed_states);
    actual_theta_rod_poles{k}=c.design_info.actual_lateral;
    theta_rod_psi_pole_error(k)=max(abs(sort_complex(c.design_info.actual_lateral(:)) ...
        -sort_complex(theta_rod_psi_poles(:))));
    if k==1 || k==numel(speeds) || mod(k,10)==0
        fprintf('  Pole placement: %d/%d (v=%.3f m/s)\n',k,numel(speeds),v);
        drawnow;
    end
end

% Show the linear eigenvalue results before starting potentially long
% nonlinear simulations.
rod_fig=figure('Color','w','Name','Rod-only pole placement', ...
    'Position',[100 100 1100 800]);
rod_tabs=uitabgroup(rod_fig);
rod_eigen_tab=uitab(rod_tabs,'Title','Closed-loop eigenvalues');
plot_eigenvalue_spectrum(rod_eigen_tab,speeds,lambda_rod, ...
    'Pole placement: rod feedback only');

theta_fig=figure('Color','w','Name','Theta/rod feedback designs', ...
    'Position',[140 140 1100 800]);
tr_tabs=uitabgroup(theta_fig);
tr_eigen_tab=uitab(tr_tabs,'Title','Closed-loop eigenvalues');
plot_eigenvalue_spectrum(tr_eigen_tab,speeds,lambda_theta_rod_only, ...
    'Pole placement: theta and rod feedback');

theta_rod_psi_fig=figure('Color','w','Name','Theta + rod + psi pole placement', ...
    'Position',[180 180 1100 800]);
trp_tabs=uitabgroup(theta_rod_psi_fig);
trp_eigen_tab=uitab(trp_tabs,'Title','Closed-loop eigenvalues');
plot_eigenvalue_spectrum(trp_eigen_tab,speeds,lambda_theta_rod, ...
    'Pole placement: theta, rod, and psi feedback');
drawnow;

% Probe the suspected forward-speed transition, with a few trajectories only.
simulation_speeds=0.5 : 0.2: 2.1; % m/s
simulation_phi_dot=simulation_speeds/p.R;
sim_rod=cell(size(simulation_speeds));
sim_theta_rod_only=cell(size(simulation_speeds));
sim_theta_rod=cell(size(simulation_speeds));
design_names={'rod-only','theta+rod','theta+rod+psi'};
for k=1:numel(simulation_speeds)
    v=simulation_speeds(k);
    fprintf('  Nonlinear simulations: speed %.2f m/s (%d/%d)\n', ...
        v,k,numel(simulation_speeds));
    drawnow;
    [A,B]=linearize_plant(v,p);
    C=zeros(2,12); C(1,9)=1; C(2,1)=p.R; C(2,4)=1;
    E=zeros(12,2); E(9,1)=1; E(4,2)=1;
    Kr=acker_siso(C*A*E,C*B(:,1),rod_poles);
    Krod=zeros(2,12); Krod(1,:)=Kr*C;
    d=d0; d.forward_speed=v;
    c=controller_parameters(); c.mode='pole_placement';
    c=generate_controller(p,c,d);
    Krod(2,:)=c.K(2,:);
    sim_rod{k}=simulate_at_speed(v,Krod,p,simcfg);
    dtr=d0; dtr.lateral_states='balance_rolling';
    dtr.rolling_poles=theta_rod_poles; dtr.forward_speed=v;
    ctr=controller_parameters(); ctr.mode='pole_placement';
    ctr=generate_controller(p,ctr,dtr);
    sim_theta_rod_only{k}=simulate_at_speed(v,ctr.K,p,simcfg);
    sim_theta_rod{k}=simulate_at_speed(v,c.K,p,simcfg);
    runs={sim_rod{k},sim_theta_rod_only{k},sim_theta_rod{k}};
    for j=1:numel(runs)
        if any(runs{j}.event_code==2)
            fprintf('    %s stopped: mass matrix became ill-conditioned (rcond < %.1e).\n', ...
                design_names{j},simcfg.min_mass_matrix_rcond);
        end
    end
end

% Add simulation tabs after trajectories are computed; eigenvalue figures
% are already visible if a nonlinear run takes a long time.
rod_sim_tab=uitab(rod_tabs,'Title','Nonlinear simulations');
rod_other_tab=uitab(rod_tabs,'Title','Other state responses');
plot_simulations(rod_sim_tab,simulation_speeds,sim_rod, ...
    'Nonlinear simulation: rod feedback only');
plot_other_state_responses(rod_other_tab,simulation_speeds,sim_rod, ...
    'Other states: rod feedback only');

tr_sim_tab=uitab(tr_tabs,'Title','Nonlinear simulations');
tr_other_tab=uitab(tr_tabs,'Title','Other state responses');
plot_simulations(tr_sim_tab,simulation_speeds,sim_theta_rod_only, ...
    'Nonlinear simulation: theta and rod feedback');
plot_other_state_responses(tr_other_tab,simulation_speeds,sim_theta_rod_only, ...
    'Other states: theta and rod feedback');

trp_sim_tab=uitab(trp_tabs,'Title','Nonlinear simulations');
trp_other_tab=uitab(trp_tabs,'Title','Other state responses');
plot_simulations(trp_sim_tab,simulation_speeds,sim_theta_rod, ...
    'Nonlinear simulation: theta, rod, and psi feedback');
plot_other_state_responses(trp_other_tab,simulation_speeds,sim_theta_rod, ...
    'Other states: theta, rod, and psi feedback');
rod_tabs.SelectedTab=rod_eigen_tab;
tr_tabs.SelectedTab=tr_eigen_tab;
trp_tabs.SelectedTab=trp_eigen_tab;

study.parameters=p;
study.phi_dot_sequence=phi_dot_sequence;
study.speed_sequence=speeds;
study.rod_only.requested_poles=rod_poles;
study.rod_only.gain=K_rod;
study.rod_only.placed_poles=actual_rod_poles;
study.rod_only.closed_eigenvalues=lambda_rod;
study.theta_rod_only.requested_poles=theta_rod_poles;
study.theta_rod_only.gain=K_theta_rod_only;
study.theta_rod_only.placed_poles=actual_theta_rod_only_poles;
study.theta_rod_only.closed_eigenvalues=lambda_theta_rod_only;
study.theta_rod_psi.requested_poles=theta_rod_psi_poles;
study.theta_rod_psi.gain=K_theta_rod;
study.theta_rod_psi.placed_poles=actual_theta_rod_poles;
study.theta_rod_psi.closed_eigenvalues=lambda_theta_rod;
study.theta_rod_psi.max_pole_placement_error=max(theta_rod_psi_pole_error);
study.closed_states=closed_states;
study.simulation_settings=simcfg;
study.rod_only.simulation=sim_rod;
study.theta_rod_only.simulation=sim_theta_rod_only;
study.theta_rod_psi.simulation=sim_theta_rod;
study.simulation_speeds=simulation_speeds;
study.simulation_phi_dot=simulation_phi_dot;

out_mat=fullfile(root,'analysis','pole_placement_eigenvalues_vs_speed.mat');
save(out_mat,'study');
fprintf('Speed range: %.4f to %.4f m/s (%d points)\n', ...
    speed_min,speed_max,speed_count);
fprintf('Max theta/rod/psi pole-placement error: %.3e 1/s\n', ...
    study.theta_rod_psi.max_pole_placement_error);
fprintf('Nonlinear simulation speeds: %s m/s (phi_dot=%s rad/s)\n', ...
    sprintf('%.2f ',simulation_speeds),sprintf('%.2f ',simulation_phi_dot));
fprintf('Saved eigenvalue data: %s\n',out_mat);
end

function [A,B]=linearize_plant(speed,p)
x=zeros(12,1); x(2)=speed/p.R;
h=1e-20; A=zeros(12); B=zeros(12,2);
for j=1:12
    z=x; z(j)=z(j)+1i*h;
    A(:,j)=imag(model_rhs(0,z,[0;0],p))/h;
end
for j=1:2
    u=zeros(2,1); u(j)=1i*h;
    B(:,j)=imag(model_rhs(0,x,u,p))/h;
end
end

function K=acker_siso(A,B,poles)
n=size(A,1); B=B(:);
validateattributes(poles,{'numeric'},{'vector','numel',n,'finite'});
C=zeros(n); C(:,1)=B;
for j=2:n, C(:,j)=A*C(:,j-1); end
if rank(C)<n || rcond(C)<1e-13
    error('Rod-only reduced system is not controllable/conditioned at this speed.');
end
q=real(poly(poles(:).'));
P=eye(n);
for j=2:n+1, P=A*P+q(j)*eye(n); end
last=zeros(1,n); last(end)=1;
K=(last/C)*P;
if norm(poly(A-B*K)-q,inf)>1e-6*max(1,norm(q,inf))
    error('Rod-only pole assignment failed its characteristic-polynomial check.');
end
end

function lambda=closed_eigenvalues(Acl,states)
lambda=eig(Acl(states,states));
lambda=sort(lambda,'ComparisonMethod','real');
end

function values=sort_complex(values)
[~,order]=sortrows([real(values(:)),imag(values(:))],[1 2]);
values=values(order);
end

function run=simulate_at_speed(speed,K,p,cfg)
c=controller_parameters();
c.mode='direct'; c.K=K;
c.x_ref=zeros(12,1); c.x_ref(2)=speed/p.R;
c.reference_rate=zeros(12,1);
c.reference_rate(8)=speed/p.R;
c.reference_rate(11)=speed;
s=simulation_settings();
s.forward_speed0=speed;
s.theta0=cfg.initial_tilt;
s.tilt_limit=cfg.tilt_limit;

s.psi_dot0=0;
x0=initial_state(s,p);
control=@(t,x) controller_output(t,x,p,c);
rhs=@(t,x) model_rhs(t,x,control(t,x),p);
times=(0:cfg.output_dt:cfg.duration)';
if times(end)<cfg.duration, times(end+1)=cfg.duration; end
opts=odeset('RelTol',s.rel_tol,'AbsTol',s.abs_tol,'MaxStep',cfg.max_step, ...
    'Events',@stop_events);
[run.t,run.X,run.event_time,~,run.event_code]=ode45(rhs,times,x0,opts);
    function [value,isterminal,direction]=stop_events(~,x)
        M=mass_matrix(x,p);
        value=[cfg.tilt_limit-abs(x(7));rcond(M)-cfg.min_mass_matrix_rcond];
        isterminal=[1;1]; direction=[-1;-1];
    end
end

function plot_simulations(parent,speeds,runs,figure_title)
fig=ancestor(parent,'figure');
parent.Parent.SelectedTab=parent;
tl=tiledlayout(parent,2,1,'TileSpacing','compact','Padding','compact');
ax1=nexttile(tl); hold(ax1,'on'); grid(ax1,'on'); box(ax1,'on');
ax2=nexttile(tl); hold(ax2,'on'); grid(ax2,'on'); box(ax2,'on');
colors=turbo(numel(speeds));
for k=1:numel(speeds)
    run=runs{k};
    plot(ax1,run.t,rad2deg(run.X(:,7)),'Color',colors(k,:),'LineWidth',0.9);
    plot(ax2,run.t,run.X(:,9),'Color',colors(k,:),'LineWidth',0.9);
end
xlabel(ax1,'Time (s)'); ylabel(ax1,'Lean angle theta (deg)');
title(ax1,'Lean response');
xlabel(ax2,'Time (s)'); ylabel(ax2,'Rod position r (m)');
title(ax2,'Rod response');
cb=colorbar(ax2); cb.Label.String='Initial speed v_0 (m/s)';
colormap(fig,colors); clim(ax1,[min(speeds) max(speeds)]);
clim(ax2,[min(speeds) max(speeds)]);
title(tl,figure_title);
drawnow;
end

function plot_eigenvalue_spectrum(parent,speed,lambda,figure_title)
% Track eigenvalue branches by nearest-neighbor continuation, then connect
% the computed eigenvalues with lines (no marker-only speed samples).
lambda=track_branches(lambda);
parent.Parent.SelectedTab=parent;
tl=tiledlayout(parent,2,1,'TileSpacing','compact','Padding','compact');
ax1=nexttile(tl); hold(ax1,'on'); grid(ax1,'on'); box(ax1,'on');
ax2=nexttile(tl); hold(ax2,'on'); grid(ax2,'on'); box(ax2,'on');
colors=lines(size(lambda,1));
for j=1:size(lambda,1)
    valid=isfinite(lambda(j,:));
    plot(ax1,speed(valid),real(lambda(j,valid)),'-','Color',colors(j,:), ...
        'LineWidth',1.2);
    plot(ax2,speed(valid),imag(lambda(j,valid)),'-','Color',colors(j,:), ...
        'LineWidth',1.2);
end
yline(ax1,0,'k--','HandleVisibility','off');
yline(ax2,0,'k--','HandleVisibility','off');
xlabel(ax1,'Forward speed v = R phi-dot (m/s)');
ylabel(ax1,'Re(lambda) (1/s)');
title(ax1,'Real parts of closed-loop eigenvalues');
xlabel(ax2,'Forward speed v = R phi-dot (m/s)');
ylabel(ax2,'Im(lambda) (rad/s)');
title(ax2,'Imaginary parts of closed-loop eigenvalues');
title(tl,figure_title);
drawnow;
end

function plot_other_state_responses(parent,speeds,runs,figure_title)
% The main simulation tab focuses on theta and r; this tab shows all other
% states in the model's documented ordering.
state_idx=[1 2 3 4 5 6 8 10 11 12];
state_names={'sigma_1 (rad/s)','sigma_2 (rad/s)','sigma_3 (rad/s)', ...
    'sigma_r (rad/s)','sigma_g (rad/s)','psi (rad)','phi (rad)', ...
    'gamma (rad/s)','x_G (m)','y_G (m)'};
colors=turbo(numel(speeds));
parent.Parent.SelectedTab=parent;
tl=tiledlayout(parent,5,2,'TileSpacing','compact','Padding','compact');
for j=1:numel(state_idx)
    ax=nexttile(tl); hold(ax,'on'); grid(ax,'on'); box(ax,'on');
    for k=1:numel(speeds)
        run=runs{k};
        if state_idx(j)==6
            response=rad2deg(run.X(:,state_idx(j)));
        elseif ismember(state_idx(j),[7 8])
            response=rad2deg(run.X(:,state_idx(j)));
        else
            response=run.X(:,state_idx(j));
        end
        plot(ax,run.t,response,'Color',colors(k,:),'LineWidth',0.9);
    end
    title(ax,state_names{j}); xlabel(ax,'Time (s)');
    if j==1
        legend(ax,arrayfun(@(v)sprintf('v_0 = %.1f m/s',v),speeds, ...
            'UniformOutput',false),'Location','best');
    end
end
title(tl,figure_title);
drawnow;
end

function tracked=track_branches(lambda)
tracked=lambda;
for k=2:size(lambda,2)
    prev=tracked(:,k-1); curr=lambda(:,k);
    valid_prev=find(isfinite(prev)); valid_curr=find(isfinite(curr));
    if numel(valid_prev)~=numel(valid_curr), continue; end
    order=zeros(size(valid_curr)); available=valid_curr(:).';
    for j=1:numel(valid_prev)
        [~,m]=min(abs(prev(valid_prev(j))-curr(available)));
        order(j)=available(m); available(m)=[];
    end
    tracked(valid_prev,k)=curr(order);
end
end
