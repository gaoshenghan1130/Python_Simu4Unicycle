function report = verify_turning_manifold_approach(theta_target_deg,initial_state,make_plots,theta0_deg,t_end)
%VERIFY_TURNING_MANIFOLD_APPROACH Reach an input-supported turning state.
% Run the public entry point: addpath('MATLAB/Full'); verify_direct_upright_turning();
% To call this helper directly, also addpath('MATLAB/Full/analysis/manifold').
% Pass 0.1 to reproduce the earlier nonzero-lean general-turning run.
% For a second leg from that equilibrium:
% leg1 = verify_turning_manifold_approach(0.1);
% leg2 = verify_turning_manifold_approach(0,leg1.X(end,:)');
% Full nonlinear 12-state plant, local LQR on the controllable relative
% states. Requires Control System Toolbox.

root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
if nargin<1, theta_target_deg = 0; end
if nargin<3, make_plots = true; end
if nargin<4 || isempty(theta0_deg), theta0_deg = 5; end
if nargin<5 || isempty(t_end), t_end = 25; end
assert(isscalar(make_plots),'make_plots must be a scalar logical flag.');
assert(isscalar(theta_target_deg) && isfinite(theta_target_deg), ...
    'theta_target_deg must be a finite scalar.');
assert(isscalar(theta0_deg) && isfinite(theta0_deg), ...
    'theta0_deg must be a finite scalar.');
assert(isscalar(t_end) && isfinite(t_end) && t_end>0, ...
    't_end must be a positive finite scalar.');
addpath(fullfile(root,'config'),fullfile(root,'model'));
p = model_parameters();
p.mr = 2.3;
p.BR = 6.5;
p.BP = 0;
s = simulation_settings();

cfg.speed = 2.375;                 % R*phi_dot [m/s]
cfg.theta0 = theta0_deg*pi/180;    % initial lean
cfg.theta_target = theta_target_deg*pi/180;
if theta_target_deg==0
    branch_name = 'upright turning';
else
    branch_name = 'general turning-rolling';
end
cfg.psi_dot0 = 0;
cfg.r0 = 0;
cfg.t_end = t_end;
cfg.dt = 0.01;
cfg.force_limit = 30;
cfg.torque_limit = 15;
cfg.tilt_limit = 45*pi/180;

% The target yaw rate is selected on the exact steady-turning branch so
% the initial state matches the single uncontrollable linear mode.
if nargin<2 || isempty(initial_state)
    x0 = zeros(12,1);
    x0(2) = cfg.speed/p.R;
    x0(3) = cfg.psi_dot0*cos(cfg.theta0);
    x0(5) = cfg.psi_dot0*sin(cfg.theta0);
    x0(7) = cfg.theta0;
    x0(9) = cfg.r0;
else
    x0 = initial_state(:);
    assert(numel(x0)==12 && all(isfinite(x0)), ...
        'initial_state must be a finite 12-state vector.');
end
idx = [1:7 9 10];               % five sigma, psi, theta, r, gamma
match = @(om) zero_mode_mismatch(om,x0,cfg,p,idx);
grid_omega = linspace(-0.8,0.8,33);
values = arrayfun(match,grid_omega);
bracket = find(values(1:end-1).*values(2:end)<=0,1,'first');
assert(~isempty(bracket),'No compatible turning equilibrium in search range.');
omega = fzero(match,grid_omega(bracket:bracket+1));
[xe,ue,rate] = general_turning_state(cfg.theta_target,omega,cfg.speed,p);
xe([6 8 11 12]) = x0([6 8 11 12]);
rate = [zeros(5,1);model_kinematics(xe,p)];
equilibrium_residual = norm(model_rhs(0,xe,ue,p)-rate,inf);
assert(equilibrium_residual<1e-8 && abs(omega)>1e-4, ...
    'Selected point is not a general turning relative equilibrium.');

[A,B] = relative_jacobian(xe,ue,p,idx);
[U,S,~] = svd(ctrb(A,B));
sv = diag(S);
rank_c = sum(sv>1e-9*sv(1));
assert(rank_c==8,'Expected eight controllable relative directions.');
T = U(:,1:rank_c);
w = U(:,end);
Q = diag([2 1 1 2 1 0.5 20 20 5]);
R = diag([0.1 0.1]);
Kc = lqr(T'*A*T,T'*B,T'*Q*T,R);
K = Kc*T';
closed_poles = eig(T'*(A-B*K)*T);
assert(all(real(closed_poles)<-1e-5),'Transverse closed loop is unstable.');

control = @(t,x) turning_control(t,x,xe,ue,rate,K,idx,cfg);
rhs = @(t,x) model_rhs(t,x,control(t,x),p);
tspan = (0:cfg.dt:cfg.t_end)';
opts = odeset('RelTol',s.rel_tol,'AbsTol',s.abs_tol, ...
    'MaxStep',cfg.dt,'Events',@(~,x) tilt_event(x,cfg));
[t,X,te,xe_event,ie] = ode45(rhs,tspan,x0,opts);
Uhist = zeros(numel(t),2);
for j=1:numel(t)
    Uhist(j,:) = control(t(j),X(j,:)')';
end

target = xe'+t*rate';
target(:,11) = xe(11)+cfg.speed/omega* ...
    (sin(xe(6)+omega*t)-sin(xe(6)));
target(:,12) = xe(12)-cfg.speed/omega* ...
    (cos(xe(6)+omega*t)-cos(xe(6)));
error = X(:,idx)-target(:,idx);
error(:,6) = atan2(sin(error(:,6)),cos(error(:,6)));
shape_idx = [1:5 7:9];          % omit free heading phase
shape_error_norm = vecnorm(error(:,shape_idx),2,2);
theta_deg = X(:,7)*180/pi;
r_m = X(:,9);
psi_deg = unwrap(X(:,6))*180/pi;
psi_dot = X(:,3)./cos(X(:,7));
phi_dot = X(:,2)-X(:,3).*tan(X(:,7));
theta_dot = X(:,1);
r_dot = p.R*X(:,1)+X(:,4);
final_u = control(t(end),X(end,:)');
final_dx = model_rhs(t(end),X(end,:)',final_u,p);
final_relative_rate_norm = norm(final_dx([1:5 7 9 10]),inf);
tail = t>=max(t(1),t(end)-5);
tail_theta_range = max(theta_deg(tail))-min(theta_deg(tail));
tail_r_range = max(r_m(tail))-min(r_m(tail));
tail_yaw_rate_range = max(psi_dot(tail))-min(psi_dot(tail));
reached_turning_manifold = isempty(te) && ...
    final_relative_rate_norm<1e-4 && tail_theta_range<0.01 && ...
    tail_r_range<1e-3 && tail_yaw_rate_range<1e-3 && ...
    abs(theta_deg(end)-theta_target_deg)<0.001;

if make_plots
fig_time = figure('Name',['Approach to ',branch_name],'Color','w');
tiledlayout(2,3,'Padding','compact','TileSpacing','compact');
nexttile;
plot(t,theta_deg,'b',t,xe(7)*180/pi*ones(size(t)),'k--','LineWidth',1.4);
grid on; xlabel('t [s]'); ylabel('\theta [deg]');
legend('full model','reference','Location','best');
nexttile;
plot(t,r_m,'b',t,xe(9)*ones(size(t)),'k--','LineWidth',1.4);
grid on; xlabel('t [s]'); ylabel('r [m]');
nexttile;
plot(t,psi_dot,'b',t,omega*ones(size(t)),'k--','LineWidth',1.4);
grid on; xlabel('t [s]'); ylabel('d\psi/dt [rad/s]');
nexttile;
plot(t,psi_deg,'b',t,omega*t*180/pi,'k--','LineWidth',1.4);
grid on; xlabel('t [s]'); ylabel('\psi [deg]');
nexttile;
semilogy(t,max(shape_error_norm,1e-12),'b','LineWidth',1.4);
grid on; xlabel('t [s]'); ylabel('reference shape error norm');
nexttile;
plot(t,error(:,6)*180/pi,'b','LineWidth',1.4);
grid on; xlabel('t [s]'); ylabel('heading phase error [deg]');
sgtitle(['Full nonlinear approach to ',branch_name]);

% At fixed yaw rate and speed, solving the exact steady-state equations
% for r as theta varies produces this slice of the equilibrium surface.
theta_branch = linspace(min([theta_deg;xe(7)*180/pi])-0.15, ...
    max([theta_deg;xe(7)*180/pi])+0.15,65);
r_branch = zeros(size(theta_branch));
for j=1:numel(theta_branch)
    x_branch = general_turning_state(theta_branch(j)*pi/180,omega,cfg.speed,p);
    r_branch(j) = x_branch(9);
end

fig_phase = figure('Name','theta-r-psi state space','Color','w');
psi_axis = linspace(min([psi_deg;omega*t*180/pi])-2, ...
    max([psi_deg;omega*t*180/pi])+2,25);
[TH,PS] = meshgrid(theta_branch,psi_axis);
RR = repmat(r_branch,numel(psi_axis),1);
surf(TH,RR,PS,'FaceColor',[0.25 0.65 0.85], ...
    'FaceAlpha',0.18,'EdgeColor','none'); hold on;
plot3(theta_deg,r_m,psi_deg,'b','LineWidth',1.8);
plot3(xe(7)*180/pi*ones(size(t)),xe(9)*ones(size(t)), ...
    omega*t*180/pi,'k--','LineWidth',1.2);
plot3(theta_deg(1),r_m(1),psi_deg(1),'go','MarkerFaceColor','g');
plot3(theta_deg(end),r_m(end),psi_deg(end),'ro','MarkerFaceColor','r');
grid on; view(40,25);
xlabel('\theta [deg]'); ylabel('r [m]'); zlabel('\psi [deg]');
legend('equilibrium slice','trajectory','reference','start','end', ...
    'Location','best');

fig_shape = figure('Name','theta-psi_dot-r state space','Color','w');
tiledlayout(2,2,'Padding','compact','TileSpacing','compact');
nexttile;
plot3(theta_branch,omega*ones(size(theta_branch)),r_branch, ...
    'k--','LineWidth',1.5); hold on;
plot3(theta_deg,psi_dot,r_m,'b','LineWidth',1.8);
plot3(xe(7)*180/pi,omega,xe(9),'yp','MarkerFaceColor','y','MarkerSize',12);
plot3(theta_deg(1),psi_dot(1),r_m(1),'go','MarkerFaceColor','g');
plot3(theta_deg(end),psi_dot(end),r_m(end),'ro','MarkerFaceColor','r');
grid on; view(38,27);
xlabel('\theta [deg]'); ylabel('d\psi/dt [rad/s]'); zlabel('r [m]');
legend('equilibrium slice','trajectory','reference','start','end', ...
    'Location','best');
nexttile; plot(theta_deg,psi_dot,'b','LineWidth',1.5); hold on;
plot(xe(7)*180/pi,omega,'yp','MarkerFaceColor','y');
grid on; xlabel('\theta [deg]'); ylabel('d\psi/dt [rad/s]');
nexttile; plot(theta_deg,r_m,'b','LineWidth',1.5); hold on;
plot(theta_branch,r_branch,'k--','LineWidth',1.1);
grid on; xlabel('\theta [deg]'); ylabel('r [m]');
nexttile; plot(psi_dot,r_m,'b','LineWidth',1.5); hold on;
plot(omega,xe(9),'yp','MarkerFaceColor','y');
grid on; xlabel('d\psi/dt [rad/s]'); ylabel('r [m]');
sgtitle([branch_name,' in relative state space']);

fig_other = figure('Name','Other states and inputs','Color','w');
tiledlayout(3,3,'Padding','compact','TileSpacing','compact');
nexttile; plot(t,X(:,10)*180/pi,'b',t,zeros(size(t)),'k--');
grid on; xlabel('t [s]'); ylabel('\gamma [deg]');
nexttile; plot(t,unwrap(X(:,8)),'b',t,target(:,8),'k--');
grid on; xlabel('t [s]'); ylabel('\phi [rad]');
nexttile; plot(t,phi_dot,'b',t,(cfg.speed/p.R)*ones(size(t)),'k--');
grid on; xlabel('t [s]'); ylabel('d\phi/dt [rad/s]');
nexttile; plot(t,theta_dot*180/pi,'b');
grid on; xlabel('t [s]'); ylabel('d\theta/dt [deg/s]');
nexttile; plot(t,r_dot,'b');
grid on; xlabel('t [s]'); ylabel('dr/dt [m/s]');
nexttile; plot(t,p.R*X(:,2),'b',t,p.R*xe(2)*ones(size(t)),'k--');
grid on; xlabel('t [s]'); ylabel('R\sigma_2 [m/s]');
nexttile; plot(t,Uhist(:,1),'b',t,ue(1)*ones(size(t)),'k--');
grid on; xlabel('t [s]'); ylabel('F [N]');
nexttile; plot(t,Uhist(:,2),'b',t,ue(2)*ones(size(t)),'k--');
grid on; xlabel('t [s]'); ylabel('M_2 [N m]');
nexttile; plot(t,shape_error_norm,'b');
grid on; xlabel('t [s]'); ylabel('reference shape error norm');
sgtitle('Other full-model states and inputs (dashed: reference)');

end

report.t = t;
report.X = X;
report.U = Uhist;
report.initial_state = x0;
report.x_equilibrium = xe;
report.u_equilibrium = ue;
report.relative_rate = rate;
report.omega = omega;
report.equilibrium_residual = equilibrium_residual;
report.controllable_rank = rank_c;
report.initial_zero_mode_mismatch = w'*(x0(idx)-xe(idx));
report.closed_poles = closed_poles;
report.final_relative_rate_norm = final_relative_rate_norm;
report.final_reference_shape_error = shape_error_norm(end);
report.tail_theta_range_deg = tail_theta_range;
report.tail_r_range_m = tail_r_range;
report.tail_psi_dot_range = tail_yaw_rate_range;
report.reached_turning_manifold = reached_turning_manifold;
report.event_time = te;
report.event_state = xe_event;
report.event_index = ie;
report.time_plot_path = '';
report.phase_plot_path = '';
report.shape_plot_path = '';
report.other_plot_path = '';

fprintf('Initial: theta=%.6g deg, r=%.6g m, psi_dot=%.6g rad/s\n', ...
    x0(7)*180/pi,x0(9),x0(3)/cos(x0(7)));
fprintf('%s reference: theta=%.6g deg, r=%.6g m, psi_dot=%.6g rad/s\n', ...
    branch_name, ...
    xe(7)*180/pi,xe(9),omega);
fprintf('Feedforward F=%.6g N, M2=%.6g N*m; equilibrium residual %.3e\n', ...
    ue(1),ue(2),equilibrium_residual);
fprintf('Controllable rank %d/9; initial local zero-mode mismatch %.3e\n', ...
    rank_c,report.initial_zero_mode_mismatch);
fprintf('Final: theta=%.6g deg, r=%.6g m, psi_dot=%.6g rad/s\n', ...
    theta_deg(end),r_m(end),psi_dot(end));
fprintf('Final relative-rate norm %.3e; reference shape error %.3e\n', ...
    final_relative_rate_norm,shape_error_norm(end));
fprintf('Reached %s manifold: %s\n', ...
    branch_name,string(reached_turning_manifold));
if ~isempty(te)
    warning('Simulation stopped at tilt limit at t=%.4g s.',te(1));
end
end

function mismatch = zero_mode_mismatch(omega,x0,cfg,p,idx)
[xe,ue] = general_turning_state(cfg.theta_target,omega,cfg.speed,p);
xe(6) = x0(6);
[A,B] = relative_jacobian(xe,ue,p,idx);
[U,S,~] = svd(ctrb(A,B));
sv = diag(S);
assert(sum(sv>1e-9*sv(1))==8,'Controllability rank changed on branch.');
w = U(:,end);
if w(7)<0, w=-w; end
mismatch = w'*(x0(idx)-xe(idx));
end

function [x,u,rate] = general_turning_state(theta,omega,speed,p)
V = speed/p.R;
x_of_r = @(r) [0;V+omega*sin(theta);omega*cos(theta);0; ...
    omega*sin(theta);0;theta;0;r;0;0;0];
force = @(r) p.mr*(p.R*(V+omega*sin(theta))*omega*cos(theta) ...
    +p.g*sin(theta)-r*omega^2*cos(theta)^2);
u_of_r = @(r) [force(r);p.BP*V];
residual = @(r) first_residual(x_of_r(r),u_of_r(r),p);
r = fzero(residual,[-0.5 0.5]);
x = x_of_r(r);
u = u_of_r(r);
rate = [zeros(5,1);model_kinematics(x,p)];
assert(norm(model_residual(x,u,p),inf)<1e-8, ...
    'Turning compatibility equations not satisfied.');
end

function value = first_residual(x,u,p)
c = model_residual(x,u,p);
value = c(1);
end

function [A,B] = relative_jacobian(x,u,p,idx)
n = numel(idx);
A = zeros(n);
B = zeros(n,2);
h = 1e-20;
for j=1:n
    z = x; z(idx(j)) = z(idx(j))+1i*h;
    f = model_rhs(0,z,u,p);
    A(:,j) = imag(f(idx))/h;
end
for j=1:2
    v = u; v(j) = v(j)+1i*h;
    f = model_rhs(0,x,v,p);
    B(:,j) = imag(f(idx))/h;
end
end

function u = turning_control(t,x,xe,ue,rate,K,idx,cfg)
reference = xe+t*rate;
error = x(idx)-reference(idx);
error(6) = atan2(sin(error(6)),cos(error(6)));
u = ue-K*error;
limit = [cfg.force_limit;cfg.torque_limit];
u = max(-limit,min(limit,u));
end

function [value,isterminal,direction] = tilt_event(x,cfg)
value = cfg.tilt_limit-abs(x(7));
isterminal = 1;
direction = -1;
end
