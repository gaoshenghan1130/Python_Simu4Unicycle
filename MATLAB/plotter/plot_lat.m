function plot_segway(t, Z, par)

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
    'Name',['Latitude Model: '], ...
    'Position',[200 50 1200 800]);

% =========================================================
% (1) Theta vs time
% =========================================================
ax1 = subplot(4,2,[1,3,5,7]);
hold on;

plot(t, theta, 'k-', 'LineWidth', lineWidth);
xlabel('$t \, (s)$','Interpreter','latex','FontSize',16);
ylabel('$theta \, (m)$','Interpreter','latex','FontSize',16);

applyStyle(ax1);

% =========================================================
% (2) Theta_dot vs time
% =========================================================
ax2 = subplot(3,2,2);
hold on;
plot(t, theta_dot, 'k-', 'LineWidth', lineWidth);

xlabel('$t \, (s)$','Interpreter','latex','FontSize',15);
ylabel('$\dot{theta} \, (m/s)$','Interpreter','latex','FontSize',15);

applyStyle(ax2);

ax3 = subplot(3,2,4);
hold on;
plot(t, r, 'k-', 'LineWidth', lineWidth);

xlabel('$t \, (s)$','Interpreter','latex','FontSize',15);
ylabel('$r \, (rad)$','Interpreter','latex','FontSize',15);

applyStyle(ax3);

% =========================================================
% (4) r_dot vs time
% =========================================================
ax4 = subplot(3,2,6);
hold on;
plot(t, r_dot, 'k-', 'LineWidth', lineWidth);

xlabel('$t \, (s)$','Interpreter','latex','FontSize',15);
ylabel('$\dot{r} \, (rad/s)$','Interpreter','latex','FontSize',15);

applyStyle(ax4);


end