function plot_segway_with_raw(t, Z, par, t_raw_list, Z_raw_list)

if ~strcmp(par.scenario, "Segway")
    error('Parameter set does not match the plotter')
end

lineWidth = 2;
bgLineWidth = 1.0;

fontName = 'Helvetica';

applyStyle = @(ax) set(ax, ...
    'Box','on', ...
    'LineWidth',1, ...
    'XGrid','on','YGrid','on', ...
    'GridLineStyle',':', ...
    'GridAlpha',0.4, ...
    'FontName',fontName, ...
    'FontSize',14);

% =========================
% main data
% =========================
X = Z(:,1);
X_dot = Z(:,2);
gamma = Z(:,3);
gamma_dot = Z(:,4);

figure('Color','w', ...
    'Name',['Segway Result: ' par.control_mode], ...
    'Position',[200 50 1200 800]);

% 自动生成不同灰度（避免全一样）
n_raw = length(Z_raw_list);
colors = linspace(0.5, 0.8, n_raw);  % 灰度梯度

% =========================================================
% (1) X
% =========================================================
ax1 = subplot(4,2,[1,3,5,7]); hold on;

for i = 1:n_raw
    Zi = Z_raw_list{i};
    ti = t_raw_list{i};

    plot(ti, Zi(:,1), '--', ...
        'Color', [colors(i) colors(i) colors(i)], ...
        'LineWidth', bgLineWidth);
end

plot(t, X, 'k-', 'LineWidth', lineWidth);

xlabel('$t \, (s)$','Interpreter','latex','FontSize',16);
ylabel('$x_G \, (m)$','Interpreter','latex','FontSize',16);
applyStyle(ax1);

% =========================================================
% (2) X_dot
% =========================================================
ax2 = subplot(4,2,2); hold on;

for i = 1:n_raw
    plot(t_raw_list{i}, Z_raw_list{i}(:,2), '--', ...
        'Color', [colors(i) colors(i) colors(i)], ...
        'LineWidth', bgLineWidth);
end

plot(t, X_dot, 'k-', 'LineWidth', lineWidth);

xlabel('$t \, (s)$','Interpreter','latex','FontSize',15);
ylabel('$\dot{x} \, (m/s)$','Interpreter','latex','FontSize',15);
applyStyle(ax2);

% =========================================================
% (3) gamma
% =========================================================
ax3 = subplot(4,2,4); hold on;

for i = 1:n_raw
    plot(t_raw_list{i}, Z_raw_list{i}(:,3), '--', ...
        'Color', [colors(i) colors(i) colors(i)], ...
        'LineWidth', bgLineWidth);
end

plot(t, gamma, 'k-', 'LineWidth', lineWidth);

xlabel('$t \, (s)$','Interpreter','latex','FontSize',15);
ylabel('$\gamma \, (rad)$','Interpreter','latex','FontSize',15);
applyStyle(ax3);

% =========================================================
% (4) gamma_dot
% =========================================================
ax4 = subplot(4,2,6); hold on;

for i = 1:n_raw
    plot(t_raw_list{i}, Z_raw_list{i}(:,4), '--', ...
        'Color', [colors(i) colors(i) colors(i)], ...
        'LineWidth', bgLineWidth);
end

plot(t, gamma_dot, 'k-', 'LineWidth', lineWidth);

xlabel('$t \, (s)$','Interpreter','latex','FontSize',15);
ylabel('$\dot{\gamma} \, (rad/s)$','Interpreter','latex','FontSize',15);
applyStyle(ax4);

end