function plot_lat(t, Z, F, par)
% Inputs:
% t: time vector
% Z: state matrix [theta, theta_dot, r, r_dot]
% F: control force vector
% par: parameter struct

lineWidth = 2;
fontName = 'Helvetica';
applyStyle = @(ax) set(ax, ...
    'Box','on', ...
    'LineWidth',1, ...
    'XGrid','on','YGrid','on', ...
    'GridLineStyle',':', ...
    'GridAlpha',0.4, ...
    'FontName',fontName, ...
    'FontSize',14);

theta = Z(:,1);
theta_dot = Z(:,2);
r = Z(:,3);
r_dot = Z(:,4);

figure('Color','w', ...
    'Name','Latitude Model with Force', ...
    'Position',[200 50 1200 800]);

% =========================================================
% (1) Theta vs time (占据左边整整一列)
% =========================================================
ax1 = subplot(4, 2, [1, 3, 5, 7]);
hold on;
plot(t, theta, 'k-', 'LineWidth', lineWidth);
xlabel('$t \, (s)$','Interpreter','latex','FontSize',16);
ylabel('$\theta \, (rad)$','Interpreter','latex','FontSize',16);
applyStyle(ax1);

% =========================================================
% (2) Theta_dot vs time (右边第 1 行)
% =========================================================
ax2 = subplot(4, 2, 2);
hold on;
plot(t, theta_dot, 'k-', 'LineWidth', lineWidth);
xlabel('$t \, (s)$','Interpreter','latex','FontSize',15);
ylabel('$\dot{\theta} \, (rad/s)$','Interpreter','latex','FontSize',15);
applyStyle(ax2);

% =========================================================
% (3) r vs time (右边第 2 行)
% =========================================================
ax3 = subplot(4, 2, 4);
hold on;
plot(t, r, 'k-', 'LineWidth', lineWidth);
xlabel('$t \, (s)$','Interpreter','latex','FontSize',15);
ylabel('$r \, (m)$','Interpreter','latex','FontSize',15);
applyStyle(ax3);

% =========================================================
% (4) r_dot vs time (右边第 3 行)
% =========================================================
ax4 = subplot(4, 2, 6);
hold on;
plot(t, r_dot, 'k-', 'LineWidth', lineWidth);
xlabel('$t \, (s)$','Interpreter','latex','FontSize',15);
ylabel('$\dot{r} \, (m/s)$','Interpreter','latex','FontSize',15);
applyStyle(ax4);

% =========================================================
% (5) Control Force F vs time (右边第 4 行)
% =========================================================
ax5 = subplot(4, 2, 8);
hold on;
plot(t, F, 'r-', 'LineWidth', lineWidth); % 用红色高亮显示力
xlabel('$t \, (s)$','Interpreter','latex','FontSize',15);
ylabel('$F \, (N)$','Interpreter','latex','FontSize',15);
applyStyle(ax5);

end