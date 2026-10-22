% RUN_ZERO_YAW_MANIFOLD_STUDY Verify a tilted straight-motion reference.
% Editable inputs. Each speed gets one window; blue=turn, orange=straight.
speeds = [1.4 1.5 1.6];        % Initial R*phi_dot [m/s]
theta0_deg = 5;               % Pure initial lean, r=0 and psi_dot=0

root = fileparts(mfilename('fullpath'));
addpath(fullfile(root,'analysis','manifold'));
% Zero-yaw design also recomputes equilibrium r, force and local LQR.
% Merely replacing omega in the old turning reference is inconsistent.
report = straight_test(speeds,theta0_deg);
out = fullfile(root,'results','zero_yaw_manifold_study');
if ~isfolder(out), mkdir(out); end

for v = speeds
    fig = figure('Color','w','Name',sprintf('Zero yaw reference: %g m/s',v), ...
        'Position',[80 60 1400 850]);
    tl = tiledlayout(fig,2,4,'TileSpacing','compact','Padding','compact');
    ax = gobjects(1,8);
    for j=1:8, ax(j)=nexttile(tl); hold(ax(j),'on'); grid(ax(j),'on'); end
    for k=1:numel(report.runs)
        q=report.runs{k}; m=q.description;
        if m.speed~=v, continue; end
        X=q.X; t=q.t;
        name=char(m.controller);
        if ~m.completed, name=[name ' (stopped early)']; end
        p=model_parameters();
        values={rad2deg(X(7,:)),X(9,:),X(3,:)./cos(X(7,:)), ...
            rad2deg(X(6,:)),p.R*X(2,:)};
        for j=1:5
            plot(ax(j),t,values{j},'LineWidth',2,'DisplayName',name);
        end
        plot(ax(6),X(11,:),X(12,:),'LineWidth',2,'DisplayName',name);
        % Applied actuator inputs after saturation, not all internal forces.
        plot(ax(7),t,q.U(1,:),'LineWidth',2,'DisplayName',name);
        plot(ax(8),t,q.U(2,:),'LineWidth',2,'DisplayName',name);
    end
    labels={'Lean angle (deg)','Rod position (m)','Yaw rate (rad/s)', ...
        'Heading (deg)','R sigma_2 (m/s)'};
    for j=1:5, xlabel(ax(j),'Time (s)'); ylabel(ax(j),labels{j}); end
    yline(ax(3),0,'k:','Target yaw rate');
    xlabel(ax(6),'x_G (m)'); ylabel(ax(6),'y_G (m)'); axis(ax(6),'equal');
    xlabel(ax(7),'Time (s)'); ylabel(ax(7),'Rod drive force F (N)');
    xlabel(ax(8),'Time (s)'); ylabel(ax(8),'Drive torque M_2 (N m)');
    legend(ax(1),'Location','best');
    title(tl,sprintf('Initial lean = %g deg, initial speed = %g m/s',theta0_deg,v));
    exportgraphics(tl,fullfile(out,sprintf('zero_yaw_v_%g.png',v)),'Resolution',220);
end
writetable(report.metrics,fullfile(out,'metrics.csv'));
save(fullfile(out,'study.mat'),'report','speeds','theta0_deg');
disp(report.metrics(report.metrics.controller=="matched straight", ...
    {'speed','completed','theta_final_deg','yaw_final','tail_max_abs_yaw', ...
    'heading_final_deg','r_final'}));
fprintf('Figures and data: %s\n',out);
