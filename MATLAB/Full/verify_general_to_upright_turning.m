function report = verify_general_to_upright_turning()
%VERIFY_GENERAL_TO_UPRIGHT_TURNING Two successive full-model turn transfers.
% First reach a 0.1-degree general turn, then an input-supported turn at
% theta=0. Each segment uses verify_turning_manifold_approach.

root = fileparts(mfilename('fullpath'));
leg1 = verify_turning_manifold_approach(0.1);
assert(leg1.reached_turning_manifold,'First leg did not settle.');
leg2 = verify_turning_manifold_approach(0,leg1.X(end,:)');
assert(leg2.reached_turning_manifold,'Second leg did not settle.');

t_switch = leg1.t(end);
t = [leg1.t; t_switch+leg2.t(2:end)];
X = [leg1.X; leg2.X(2:end,:)];
theta_deg = X(:,7)*180/pi;
psi_dot = X(:,3)./cos(X(:,7));
r_m = X(:,9);
psi_deg = unwrap(X(:,6))*180/pi;

output_dir = fullfile(root,'results','general_to_upright_turning');
if ~isfolder(output_dir), mkdir(output_dir); end

fig_time = figure('Name','General turn to upright turn','Color','w');
tiledlayout(2,2,'Padding','compact','TileSpacing','compact');
nexttile; plot(t,theta_deg,'b','LineWidth',1.5); hold on;
yline(0,'k--'); xline(t_switch,'Color',[0.5 0.5 0.5]);
grid on; xlabel('t [s]'); ylabel('\theta [deg]');
nexttile; plot(t,r_m,'b','LineWidth',1.5); hold on;
xline(t_switch,'Color',[0.5 0.5 0.5]);
grid on; xlabel('t [s]'); ylabel('r [m]');
nexttile; plot(t,psi_dot,'b','LineWidth',1.5); hold on;
xline(t_switch,'Color',[0.5 0.5 0.5]);
grid on; xlabel('t [s]'); ylabel('d\psi/dt [rad/s]');
nexttile; plot(t,psi_deg,'b','LineWidth',1.5); hold on;
xline(t_switch,'Color',[0.5 0.5 0.5]);
grid on; xlabel('t [s]'); ylabel('\psi [deg]');
sgtitle('Stage 1: general turn; stage 2: upright turn');

fig_shape = figure('Name','Two-stage turning state space','Color','w');
tiledlayout(1,2,'Padding','compact','TileSpacing','compact');
nexttile;
plot3(theta_deg,psi_dot,r_m,'b','LineWidth',1.8); hold on;
plot3(theta_deg(1),psi_dot(1),r_m(1),'go','MarkerFaceColor','g');
plot3(theta_deg(numel(leg1.t)),psi_dot(numel(leg1.t)), ...
    r_m(numel(leg1.t)),'yo','MarkerFaceColor','y');
plot3(theta_deg(end),psi_dot(end),r_m(end),'ro','MarkerFaceColor','r');
grid on; view(38,26);
xlabel('\theta [deg]'); ylabel('d\psi/dt [rad/s]');
zlabel('r [m]');
legend('full trajectory','start','general turn','upright turn', ...
    'Location','best');
nexttile;
plot(theta_deg,psi_dot,'b','LineWidth',1.5); hold on;
plot(theta_deg(1),psi_dot(1),'go','MarkerFaceColor','g');
plot(theta_deg(numel(leg1.t)),psi_dot(numel(leg1.t)), ...
    'yo','MarkerFaceColor','y');
plot(theta_deg(end),psi_dot(end),'ro','MarkerFaceColor','r');
grid on; xlabel('\theta [deg]'); ylabel('d\psi/dt [rad/s]');
sgtitle('Full nonlinear trajectory in relative state space');

report.stage1 = leg1;
report.stage2 = leg2;
report.t = t;
report.X = X;
report.time_plot_path = fullfile(output_dir,'two_stage_time.png');
report.shape_plot_path = fullfile(output_dir,'two_stage_theta_psidot_r.png');
exportgraphics(fig_time,report.time_plot_path,'Resolution',180);
exportgraphics(fig_shape,report.shape_plot_path,'Resolution',180);

fprintf(['Two-stage final: theta=%.8g deg, psi_dot=%.8g rad/s, ' ...
    'r=%.8g m, stage-2 relative-rate norm=%.3e\n'], ...
    theta_deg(end),psi_dot(end),r_m(end),leg2.final_relative_rate_norm);
end
