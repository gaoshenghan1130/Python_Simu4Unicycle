function study = scan_case1_nonlinear_divergence()
% Scan initial forward speeds for actual nonlinear tilt-stop divergence.
% A run is classified as divergent only if it reaches the 80 deg tilt event.
root = fileparts(fileparts(mfilename('fullpath')));
addpath(genpath(root));
p = model_parameters();

cfg.duration = 90;
cfg.long_duration = 1000;
cfg.long_speed_bracket = [1.5019 1.50195];
cfg.output_dt = 0.05;
cfg.max_step = 0.005;
cfg.tilt_limit = 80*pi/180;
cfg.theta0 = 0.1*pi/180;
cfg.coarse_speeds = unique([0.05:0.05:1.40, 1.40:0.02:1.72]);
cfg.refine_tolerance = 0.001;

c = struct('mode','pd','kp_theta',0,'kd_theta',0, ...
    'kp_r',854.34418,'kd_r',105.568949, ...
    'kp_gamma',3,'kd_gamma',0.8, ...
    'theta_ref',0,'theta_dot_ref',0,'r_ref',0,'r_dot_ref',0, ...
    'gamma_ref',0,'gamma_dot_ref',0, ...
    'force_limit',Inf,'torque_limit',Inf);

fprintf('Scanning %d initial speeds, simulation duration %.1f s, m_r=%.2f kg\n', ...
    numel(cfg.coarse_speeds),cfg.duration,p.mr);
summary = repmat(empty_summary(),1,numel(cfg.coarse_speeds));
for k=1:numel(cfg.coarse_speeds)
    summary(k)=run_one(cfg.coarse_speeds(k),p,c,cfg);
    fprintf('%3d/%3d  v0=%.4f  peak|theta|=%7.3f deg  v=[%.4f, %.4f]  %s\n', ...
        k,numel(cfg.coarse_speeds),summary(k).initial_speed, ...
        summary(k).peak_theta_deg,summary(k).min_speed,summary(k).max_speed, ...
        run_label(summary(k).diverged));
end

% Refine every observed transition in the tilt-event classification.
order = [summary.initial_speed];
refined = repmat(empty_summary(),1,0);
transition_brackets = zeros(0,2);
for k=1:numel(summary)-1
    if summary(k).diverged ~= summary(k+1).diverged
        lo=summary(k); hi=summary(k+1);
        while hi.initial_speed-lo.initial_speed > cfg.refine_tolerance
            vm=(lo.initial_speed+hi.initial_speed)/2;
            mid=run_one(vm,p,c,cfg);
            refined(end+1)=mid; %#ok<AGROW>
            if mid.diverged==lo.diverged, lo=mid; else, hi=mid; end
        end
        fprintf('Refined event transition: %.6f to %.6f m/s (%s -> %s)\n', ...
            lo.initial_speed,hi.initial_speed,run_label(lo.diverged),run_label(hi.diverged));
        transition_brackets(end+1,:)=[lo.initial_speed hi.initial_speed]; %#ok<AGROW>
    end
end

study.parameters=p;
study.controller=c;
study.settings=cfg;
study.coarse=summary;
study.refined=refined;
study.coarse_speed_grid=order;
study.speed_critical_eigenvalue=1.588584225;
study.ninety_second_event_brackets=transition_brackets;

% Recheck the narrow candidate interval long enough to distinguish a delayed
% tilt event from a trajectory that remains below the tilt stop for 1000 s.
longCfg=cfg; longCfg.duration=cfg.long_duration; longCfg.output_dt=0.1;
longChecks=repmat(empty_summary(),1,2);
for k=1:2
    longChecks(k)=run_one(cfg.long_speed_bracket(k),p,c,longCfg);
    fprintf('1000 s check v0=%.6f: peak|theta|=%.4f deg, event time=%.3f s, %s\n', ...
        longChecks(k).initial_speed,longChecks(k).peak_theta_deg, ...
        longChecks(k).event_time,long_label(longChecks(k).diverged,cfg.long_duration));
end
study.long_horizon_checks=longChecks;
study.long_horizon_event_bracket=cfg.long_speed_bracket;

fig=figure('Color','w','Position',[100 100 1100 680]);
tiledlayout(fig,2,1,'TileSpacing','compact','Padding','compact');
ax=nexttile; hold(ax,'on'); grid(ax,'on'); box(ax,'on');
hp=plot(ax,[summary.initial_speed],[summary.peak_theta_deg],'-o', ...
    'Color',[0.12 0.38 0.67],'MarkerFaceColor',[0.12 0.38 0.67]);
hTilt=yline(ax,80,'r--');
eventLabel='no event transition detected';
if ~isempty(transition_brackets)
    hEvent=xline(ax,mean(cfg.long_speed_bracket),'r-.');
    eventLabel=sprintf('1000 s event bracket: %.6f–%.6f m/s', ...
        cfg.long_speed_bracket(1),cfg.long_speed_bracket(2));
else
    hEvent=gobjects(1);
end
hLinear=xline(ax,study.speed_critical_eigenvalue,'k:');
legend(ax,[hp hTilt hEvent hLinear],{'peak angle by initial speed', ...
    '80° tilt-stop threshold',eventLabel, ...
    sprintf('linearized boundary: %.6f m/s',study.speed_critical_eigenvalue)}, ...
    'Location','best');
ylabel(ax,'Peak |\theta| over 90 s (deg)');
title(ax,'Case 1 nonlinear speed scan');
ax=nexttile; hold(ax,'on'); grid(ax,'on'); box(ax,'on');
plot(ax,[summary.initial_speed],[summary.min_speed],'-', ...
    'Color',[0.20 0.58 0.38],'LineWidth',1.2);
plot(ax,[summary.initial_speed],[summary.max_speed],'-', ...
    'Color',[0.76 0.30 0.18],'LineWidth',1.2);
xline(study.speed_critical_eigenvalue,'k:');
if ~isempty(transition_brackets)
    xline(mean(cfg.long_speed_bracket),'r-.');
end
xlabel(ax,'Initial forward speed (m/s)'); ylabel(ax,'Forward speed range (m/s)');
legend(ax,{'minimum speed','maximum speed'},'Location','best');

out=fullfile(root,'analysis','case1_nonlinear_divergence_scan.mat');
save(out,'study');
figFile=fullfile(root,'analysis','case1_nonlinear_divergence_scan.png');
exportgraphics(fig,figFile,'Resolution',200);
if any([summary.diverged])
    fprintf('At least one nonlinear run reached the 80 deg tilt event.\n');
else
    fprintf('No run reached the 80 deg tilt event in the scanned range and duration.\n');
end
fprintf('Saved data: %s\nSaved plot: %s\n',out,figFile);
end

function row=run_one(v0,p,c,cfg)
s=simulation_settings();
s.forward_speed0=v0;
s.theta0=cfg.theta0;
s.t_end=cfg.duration;
s.output_dt=cfg.output_dt;
s.max_step=cfg.max_step;
s.tilt_limit=cfg.tilt_limit;
r=simulate_unicycle(p,c,s);
row=empty_summary();
row.initial_speed=v0;
row.peak_theta_deg=max(abs(r.X(:,7)))*180/pi;
row.peak_r=max(abs(r.X(:,9)));
row.peak_gamma_deg=max(abs(r.X(:,10)))*180/pi;
speed=p.R*r.X(:,2);
row.min_speed=min(speed);
row.max_speed=max(speed);
row.final_speed=speed(end);
row.event_time=NaN;
if ~isempty(r.event_time), row.event_time=r.event_time(1); end
row.diverged=~isempty(r.event_time);
end

function row=empty_summary()
row=struct('initial_speed',NaN,'peak_theta_deg',NaN,'peak_r',NaN, ...
    'peak_gamma_deg',NaN,'min_speed',NaN,'max_speed',NaN, ...
    'final_speed',NaN,'event_time',NaN,'diverged',false);
end

function label=run_label(diverged)
if diverged, label='DIVERGED (tilt event)'; else, label='bounded for 90 s'; end
end

function label=long_label(diverged,duration)
if diverged
    label='DIVERGED (tilt event)';
else
    label=sprintf('no tilt event by %.0f s',duration);
end
end
