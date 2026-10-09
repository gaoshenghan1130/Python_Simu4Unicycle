function report = verify_general_to_upright_turning()
%VERIFY_GENERAL_TO_UPRIGHT_TURNING Transfer from a general to upright turn.
% The first segment is used to initialize the compatible general-turn orbit;
% only the upright-turn simulation is plotted by this entry point.

root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
addpath(fullfile(root,'analysis','manifold'));
leg1 = verify_turning_manifold_approach(0.1,[],false);
assert(leg1.reached_turning_manifold,'First leg did not settle.');
leg2 = verify_turning_manifold_approach(0,leg1.X(end,:)',false);
assert(leg2.reached_turning_manifold,'Second leg did not settle.');

t = leg2.t;
X = leg2.X;
theta_deg = X(:,7)*180/pi;
psi_dot = X(:,3)./cos(X(:,7));
r_m = X(:,9);
psi_deg = unwrap(X(:,6))*180/pi;

fig_time = figure('Name','Upright turn from general turn','Color','w');
tiledlayout(2,2,'Padding','compact','TileSpacing','compact');
nexttile; plot(t,theta_deg,'b','LineWidth',1.5); hold on;
yline(0,'k--');
grid on; xlabel('t [s]'); ylabel('\theta [deg]');
nexttile; plot(t,r_m,'b','LineWidth',1.5); hold on;
grid on; xlabel('t [s]'); ylabel('r [m]');
nexttile; plot(t,psi_dot,'b','LineWidth',1.5); hold on;
grid on; xlabel('t [s]'); ylabel('d\psi/dt [rad/s]');
nexttile; plot(t,psi_deg,'b','LineWidth',1.5); hold on;
grid on; xlabel('t [s]'); ylabel('\psi [deg]');
sgtitle('Upright turning after transfer from general turning');

fig_shape = figure('Name','Upright turning state space','Color','w');
tiledlayout(1,2,'Padding','compact','TileSpacing','compact');
nexttile;
plot3(theta_deg,psi_dot,r_m,'b','LineWidth',1.8); hold on;
plot3(theta_deg(1),psi_dot(1),r_m(1),'go','MarkerFaceColor','g');
plot3(theta_deg(end),psi_dot(end),r_m(end),'ro','MarkerFaceColor','r');
grid on; view(38,26);
xlabel('\theta [deg]'); ylabel('d\psi/dt [rad/s]');
zlabel('r [m]');
legend('upright trajectory','start','upright turn', ...
    'Location','best');
nexttile;
plot(theta_deg,psi_dot,'b','LineWidth',1.5); hold on;
plot(theta_deg(1),psi_dot(1),'go','MarkerFaceColor','g');
plot(theta_deg(end),psi_dot(end),'ro','MarkerFaceColor','r');
grid on; xlabel('\theta [deg]'); ylabel('d\psi/dt [rad/s]');
sgtitle('Upright turning trajectory in relative state space');

report.stage1 = leg1;
report.stage2 = leg2;
report.t = t;
report.X = X;
report.time_plot_path = '';
report.shape_plot_path = '';

fprintf(['Two-stage final: theta=%.8g deg, psi_dot=%.8g rad/s, ' ...
    'r=%.8g m, stage-2 relative-rate norm=%.3e\n'], ...
    theta_deg(end),psi_dot(end),r_m(end),leg2.final_relative_rate_norm);
end
