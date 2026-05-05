function plot_sim_vs_real(t, Z, segments, par)
%PLOT_SIM_VS_REAL Compare simulation vs real BLE logs in one figure
%
% t        : simulation time (Nx1)
% Z        : simulation state [x, x_dot, gamma, gamma_dot]
% segments : struct array from read_real_data()
% par      : parameter struct (optional, only for naming)

clc; close all;

% =========================
% Style
% =========================
lineWidth_sim = 2.0;
lineWidth_real = 1.0;

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
% Colors
% =========================
realColor = [0.8 0.8 0.8];   % light gray for all experiments
simColor  = [0 0 0];         % black for simulation

% =========================
% Extract simulation states
% =========================
X = Z(:,1);
X_dot = Z(:,2);
gamma = Z(:,3);
gamma_dot = Z(:,4);

% =========================
% Figure
% =========================
figure('Color','w', ...
    'Name','Simulation vs Real Data', ...
    'Position',[200 50 1100 850]);

% =========================================================
% (1) Position
% =========================================================
ax1 = subplot(2,2,1); hold on;

for i = 1:numel(segments)
    plot(segments(i).time, segments(i).position, ...
        'Color', realColor, 'LineWidth', lineWidth_real);
end

plot(t, X, 'Color', simColor, 'LineWidth', lineWidth_sim);

ylabel('$x \, (\mathrm{m})$', 'Interpreter','latex')
title('Simulation vs Real Data Comparison')
applyStyle(ax1);

% =========================================================
% (2) Velocity
% =========================================================
ax2 = subplot(2,2,3); hold on;

for i = 1:numel(segments)
    plot(segments(i).time, segments(i).velocity, ...
        'Color', realColor, 'LineWidth', lineWidth_real);
end

plot(t, X_dot, 'Color', simColor, 'LineWidth', lineWidth_sim);

ylabel('$\dot{x} \, (\mathrm{m/s})$', 'Interpreter','latex')
applyStyle(ax2);

% =========================================================
% (3) Gamma
% =========================================================
ax3 = subplot(2,2,2); hold on;

for i = 1:numel(segments)
    plot(segments(i).time, segments(i).gamma_deg, ...
        'Color', realColor, 'LineWidth', lineWidth_real);
end

plot(t, gamma * 180/pi, 'Color', simColor, 'LineWidth', lineWidth_sim);

ylabel('$\gamma \, (\mathrm{deg})$', 'Interpreter','latex')
applyStyle(ax3);

% =========================================================
% (4) Gamma dot
% =========================================================
ax4 = subplot(2,2,4); hold on;

for i = 1:numel(segments)
    plot(segments(i).time, segments(i).dgamma_degps, ...
        'Color', realColor, 'LineWidth', lineWidth_real);
end

plot(t, gamma_dot * 180/pi, 'Color', simColor, 'LineWidth', lineWidth_sim);

xlabel('$t \, (\mathrm{s})$', 'Interpreter','latex')
ylabel('$\dot{\gamma} \, (\mathrm{deg/s})$', 'Interpreter','latex')
applyStyle(ax4);

end