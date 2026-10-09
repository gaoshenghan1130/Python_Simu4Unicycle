function results = critical_speed_control_study()
% Critical-speed comparison for progressively added lateral feedback.
% The linearization is evaluated at upright straight rolling.

root = fileparts(fileparts(mfilename('fullpath')));
addpath(genpath(root));
p = model_parameters();

cfg.v_grid = linspace(0.05, 20.0, 996);
cfg.zero_tol = 1e-8;
cfg.boundary_tol = 1e-7;
cfg.dynamic_states = [1:5 7 9:10]; % exclude psi, phi, xG, yG symmetries
cfg.simulation_duration = 30;
cfg.test_margins = [-0.02 0 0.02];

cases = controller_cases();
results = repmat(struct(), numel(cases), 1);

fprintf('Critical-speed study, m_r = %.4g kg\n', p.mr);
fprintf('Dynamic eigenvalue states: [%s]\n\n', sprintf('%d ', cfg.dynamic_states));

for k = 1:numel(cases)
    alpha = zeros(size(cfg.v_grid));
    spectra = cell(size(cfg.v_grid));
    for i = 1:numel(cfg.v_grid)
        [alpha(i), spectra{i}] = closed_loop_alpha(cfg.v_grid(i), p, cases(k), cfg);
    end

    crossings = find(diff(sign_with_tol(alpha, cfg.boundary_tol)) ~= 0);
    crossing_speeds = zeros(size(crossings));
    crossing_speeds = NaN(size(crossings));
    for i = crossings
        vb = fzero(@(v) closed_loop_alpha(v, p, cases(k), cfg), ...
            [cfg.v_grid(i), cfg.v_grid(i+1)]);
        crossing_speeds(crossings == i) = vb;
    end
    crossing_directions = sign(alpha(crossings+1)-alpha(crossings));
    low_idx = find(crossing_directions < 0, 1, 'first'); % unstable -> stable
    high_idx = find(crossing_directions > 0, 1, 'last'); % stable -> unstable
    low_critical = NaN; high_critical = NaN;
    if ~isempty(low_idx), low_critical = crossing_speeds(low_idx); end
    if ~isempty(high_idx), high_critical = crossing_speeds(high_idx); end

    results(k).name = cases(k).name;
    results(k).gains = cases(k);
    results(k).crossings = crossings;
    results(k).crossing_speeds = crossing_speeds;
    results(k).crossing_directions = crossing_directions;
    results(k).low_critical_speed = low_critical;
    results(k).high_critical_speed = high_critical;
    results(k).alpha_grid = alpha;
    results(k).speed_grid = cfg.v_grid;
    results(k).boundary_checks = cell(1,numel(crossing_speeds));
    for j=1:numel(crossing_speeds)
        results(k).boundary_checks{j}=verify_crossing(crossing_speeds(j),p,cases(k),cfg);
    end

    fprintf('Case %d: %s\n', k-1, cases(k).name);
    fprintf('  alpha range on scan: [%+.6e, %+.6e]\n', min(alpha), max(alpha));
    fprintf('  low-speed boundary (unstable -> stable): %.9f m/s\n', low_critical);
    fprintf('  high-speed boundary (stable -> unstable): %.9f m/s\n', high_critical);
    if ~isempty(crossing_speeds)
        fprintf('  all crossings: %s m/s\n', sprintf('%.9f ', crossing_speeds));
    end
    if ~isnan(low_critical)
        [~, ev] = closed_loop_alpha(low_critical, p, cases(k), cfg);
        ev = sort_eigenvalues(ev);
        fprintf('  low-boundary poles closest to imaginary axis:\n');
        fprintf('    % .9e %+.9ei  (%.9f Hz)\n', ...
            [real(ev(1:min(4,end))), imag(ev(1:min(4,end))) ...
             imag(ev(1:min(4,end)))/(2*pi)]');
        check_sides(low_critical, p, cases(k), cfg);
        results(k).validation = validate_case(low_critical,p,cases(k),cfg);
    else
        results(k).validation = [];
    end
    fprintf('\n');
end

for k=1:numel(results), results(k).neutral_boundary=NaN; end
results(1).neutral_boundary=find_real_mode_loss(p,cases(1),cfg);
fprintf('No-rod positive-real mode becomes neutral near %.9f m/s\n', ...
    results(1).neutral_boundary);

fprintf('Conclusion:\n');
v0 = results(1).low_critical_speed;
for k = 1:numel(results)
    if isfinite(v0) && isfinite(results(k).low_critical_speed)
        dv = results(k).low_critical_speed - v0;
        fprintf('  %-36s low boundary %.6f m/s (%+.6f vs Case 0)\n', ...
            results(k).name, results(k).low_critical_speed, dv);
    else
        fprintf('  %-36s low boundary %.6f m/s (no Case 0 reference crossing)\n', ...
            results(k).name, results(k).low_critical_speed);
    end
end
plot_control_pair(results(1), results(2), ...
    'Effect of rod-centering feedback', ...
    fullfile(root,'analysis','critical_speed_rod_unfixed_vs_centered.png'), ...
    [1.2 5.0],[-0.02 0.04]);
plot_control_pair(results(2), results(3), ...
    'Effect of adding lean-angle feedback', ...
    fullfile(root,'analysis','critical_speed_theta_feedback_comparison.png'), ...
    [1.2 8.0],[-0.35 0.35]);
save(fullfile(root,'analysis','critical_speed_control_results.mat'),'results','cfg','p');

end

function plot_control_pair(a,b,figure_title,output_file,x_limits,y_limits)
fig=figure('Color','w','Position',[120 120 950 620]);
ax=axes(fig); hold(ax,'on'); grid(ax,'on'); box(ax,'on');
plot(ax,a.speed_grid,a.alpha_grid,'LineWidth',1.8,'DisplayName',a.name);
plot(ax,b.speed_grid,b.alpha_grid,'LineWidth',1.8,'DisplayName',b.name);
yline(ax,0,'k--','Re(\lambda)=0','HandleVisibility','off');
mark_boundaries(ax,a);
mark_boundaries(ax,b);
xlim(ax,x_limits); ylim(ax,y_limits);
xlabel(ax,'Upright rolling speed (m/s)');
ylabel(ax,'Dominant dynamic Re(\lambda) (1/s)');
title(ax,figure_title);
subtitle(ax,'Zoomed near the boundary; larger positive growth rates are clipped.');
legend(ax,'Location','best');
exportgraphics(fig,output_file,'Resolution',200);
fprintf('Saved comparison plot: %s\n',output_file);
end

function mark_boundaries(ax,row)
if isfield(row,'neutral_boundary') && isfinite(row.neutral_boundary)
    xline(ax,row.neutral_boundary,'--','HandleVisibility','off');
end
if isfinite(row.low_critical_speed)
    xline(ax,row.low_critical_speed,':','HandleVisibility','off');
end
if isfinite(row.high_critical_speed)
    xline(ax,row.high_critical_speed,'-.','HandleVisibility','off');
end
end

function critical_speed=find_real_mode_loss(p,c,cfg)
% With no rod feedback, the positive real pole becomes a neutral imaginary
% pair, so an ordinary sign-change test on the spectral abscissa misses it.
scan=1.50:0.01:2.00;
flags=arrayfun(@(v) has_positive_real_pole(v,p,c,cfg),scan);
k=find(flags(1:end-1) & ~flags(2:end),1,'first');
if isempty(k)
    critical_speed=NaN;
    return;
end
lo=scan(k); hi=scan(k+1);
while hi-lo>1e-7
    mid=(lo+hi)/2;
    if has_positive_real_pole(mid,p,c,cfg), lo=mid; else, hi=mid; end
end
critical_speed=(lo+hi)/2;
end

function yes=has_positive_real_pole(v,p,c,cfg)
[~,ev]=closed_loop_alpha(v,p,c,cfg);
yes=any(real(ev)>1e-7 & abs(imag(ev))<1e-6);
end

function cases = controller_cases()
% Force signs follow pd_controller: F = -K*position - D*rate.
base = struct('kp_gamma', 3, 'kd_gamma', 0.8, ...
    'kp_r', 0, 'kd_r', 0, 'kp_theta', 0, 'kd_theta', 0, 'name', '');
cases(1) = base; cases(1).name = 'gamma PD only';
cases(2) = base; cases(2).name = 'gamma PD + r PD';
cases(2).kp_r = 854.34418; cases(2).kd_r = 105.568949;
cases(3) = cases(2); cases(3).name = 'gamma PD + r PD + lean PD';
cases(3).kp_theta = 20; cases(3).kd_theta = 4;
cases(4) = cases(2); cases(4).name = 'Case 3a: reduce rod stiffness';
cases(4).kp_r = 0.75*cases(2).kp_r;
cases(5) = cases(2); cases(5).name = 'Case 3b: increase rod damping';
cases(5).kd_r = 1.25*cases(2).kd_r;
cases(6) = cases(3); cases(6).name = 'Case 3c: increase lean stiffness';
cases(6).kp_theta = 40;
cases(7) = cases(3); cases(7).name = 'Case 3d: increase lean damping';
cases(7).kd_theta = 8;
% Case 4 uses the best low-boundary lateral law, selected after Cases 0-3.
cases(8) = cases(2); cases(8).name = 'Case 4: exact gamma-zero constraint';
cases(8).gamma_constraint = true;
cases(8).gamma_constraint_rate = 10;
end

function alpha = closed_loop_alpha_only(v, p, c, cfg)
[alpha, ~] = closed_loop_alpha(v, p, c, cfg);
end

function [alpha, ev] = closed_loop_alpha(v, p, c, cfg)
x = zeros(12,1); x(2) = v/p.R;
u = controller_linear(x, p, c);
A = complex_jacobian(@(z) model_rhs(0, z, controller_linear(z,p,c), p), x);
% controller_linear(x) is zero at the upright reference; u is retained
% above to make the operating point explicit and easy to audit.
assert(norm(u) < 1e-12, 'Controller must be zero at the reference state.');
ev = eig(A(cfg.dynamic_states, cfg.dynamic_states));
ev = ev(abs(ev) > cfg.zero_tol);
alpha = max(real(ev));
end

function u = controller_linear(x, p, c)
theta_dot = x(1); r_dot = p.R*x(1) + x(4);
gamma_dot = x(5) - x(3)*tan(x(7));
F = -c.kp_theta*x(7) - c.kd_theta*theta_dot ...
    -c.kp_r*x(9) - c.kd_r*r_dot;
M2 = c.kp_gamma*x(10) + c.kd_gamma*gamma_dot;
if isfield(c,'gamma_constraint') && ~isempty(c.gamma_constraint) && c.gamma_constraint
    M2 = gamma_zero_torque(0,x,F,p,c.gamma_constraint_rate);
end
u = [F; M2];
end

function J = complex_jacobian(fun, x)
h = 1e-20;
J = zeros(numel(fun(x)), numel(x));
for j = 1:numel(x)
    z = x; z(j) = z(j) + 1i*h;
    J(:,j) = imag(fun(z))/h;
end
end

function s = sign_with_tol(a, tol)
s = sign(a); s(abs(a) <= tol) = 0;
for i = 2:numel(s)
    if s(i) == 0, s(i) = s(i-1); end
end
end

function ev = sort_eigenvalues(ev)
[~, order] = sort(abs(real(ev)) + 1e-6*abs(imag(ev)));
ev = ev(order);
end

function check_sides(vb, p, c, cfg)
delta = max(1e-4, 1e-3*vb);
for v = [vb-delta, vb+delta]
    a = closed_loop_alpha_only(v, p, c, cfg);
    fprintf('  alpha at v = %.9f: %+.6e (%s)\n', v, a, stability_label(a));
end
end

function check = verify_crossing(vb,p,c,cfg)
delta=max(1e-4,1e-3*vb);
[~,check.eigenvalues_boundary]=closed_loop_alpha(vb,p,c,cfg);
check.speed_below=vb-delta;
[check.alpha_below,check.eigenvalues_below]=closed_loop_alpha(vb-delta,p,c,cfg);
check.speed_above=vb+delta;
[check.alpha_above,check.eigenvalues_above]=closed_loop_alpha(vb+delta,p,c,cfg);
end

function label = stability_label(alpha)
if alpha > 0, label = 'unstable'; else, label = 'stable/nonpositive'; end
end

function validation = validate_case(vcrit,p,c,cfg)
speeds = vcrit*(1+cfg.test_margins);
s = simulation_settings();
s.t_end = cfg.simulation_duration;
s.theta0 = 0.1*pi/180;
cn = c;
cn.mode = 'pd'; cn.theta_ref=0; cn.theta_dot_ref=0;
cn.r_ref=0; cn.r_dot_ref=0; cn.gamma_ref=0; cn.gamma_dot_ref=0;
cn.force_limit=Inf; cn.torque_limit=Inf;
if ~isfield(cn,'gamma_constraint') || isempty(cn.gamma_constraint), cn.gamma_constraint=false; end
if cn.gamma_constraint
    cn.hold_gamma_zero=true;
end
validation = repmat(struct(),1,numel(speeds));
for j=1:numel(speeds)
    sj=s; sj.forward_speed0=speeds(j);
    run=simulate_unicycle(p,cn,sj);
    validation(j).speed=speeds(j);
    validation(j).margin=cfg.test_margins(j);
    validation(j).time=run.t;
    validation(j).theta=run.X(:,7);
    validation(j).r=run.X(:,9);
    validation(j).gamma=run.X(:,10);
    validation(j).forward_speed=p.R*run.X(:,2);
    validation(j).peak_theta_deg=max(abs(run.X(:,7)))*180/pi;
    validation(j).peak_r=max(abs(run.X(:,9)));
    validation(j).peak_gamma_deg=max(abs(run.X(:,10)))*180/pi;
    validation(j).final_speed=p.R*run.X(end,2);
    fprintf('  nonlinear v=%.6f (%+.0f%%): peak theta %.4g deg, r %.4g m, gamma %.4g deg\n', ...
        validation(j).speed,100*validation(j).margin,validation(j).peak_theta_deg, ...
        validation(j).peak_r,validation(j).peak_gamma_deg);
end
end
