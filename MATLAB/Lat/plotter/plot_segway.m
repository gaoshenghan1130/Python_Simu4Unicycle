function plot_segway(t, Z, par)

if ~strcmp(par.scenario, "Segway")
    error('Parameter set does not match the plotter')
end

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

X = Z(:,1);
X_dot = Z(:,2);
gamma = Z(:,3);
gamma_dot = Z(:,4);

figure('Color','w', ...
    'Name',['Segway Result: ' par.control_mode], ...
    'Position',[200 50 1200 800]);

% =========================================================
% (1) Main: state evolution (x vs time)
% =========================================================
ax1 = subplot(4,2,[1,3,5,7]);
hold on;

plot(t, X, 'k-', 'LineWidth', lineWidth);
xlabel('$t \, (s)$','Interpreter','latex','FontSize',16);
ylabel('$x_G \, (m)$','Interpreter','latex','FontSize',16);

applyStyle(ax1);

% =========================================================
% (2) gamma
% =========================================================
ax2 = subplot(3,2,2);
hold on;
plot(t, X_dot, 'k-', 'LineWidth', lineWidth);

xlabel('$t \, (s)$','Interpreter','latex','FontSize',15);
ylabel('$\dot{x} \, (m/s)$','Interpreter','latex','FontSize',15);

applyStyle(ax2);

ax3 = subplot(3,2,4);
hold on;
plot(t, gamma, 'k-', 'LineWidth', lineWidth);

xlabel('$t \, (s)$','Interpreter','latex','FontSize',15);
ylabel('$gamma \, (rad)$','Interpreter','latex','FontSize',15);

applyStyle(ax3);

% =========================================================
% (4) gamma dot
% =========================================================
ax4 = subplot(3,2,6);
hold on;
plot(t, gamma_dot, 'k-', 'LineWidth', lineWidth);

xlabel('$t \, (s)$','Interpreter','latex','FontSize',15);
ylabel('$\dot{gamma} \, (rad/s)$','Interpreter','latex','FontSize',15);

applyStyle(ax4);


end