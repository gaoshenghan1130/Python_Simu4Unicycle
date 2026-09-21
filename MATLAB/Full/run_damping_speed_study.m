% Reproducible speed study: redesign at each speed, then integrate full model.
project_root=fileparts(mfilename('fullpath'));
addpath(fullfile(project_root,'config'),fullfile(project_root,'model'), ...
    fullfile(project_root,'controllers'),fullfile(project_root,'simulation'), ...
    fullfile(project_root,'analysis'));
output_dir=fullfile(project_root,'results','damping_speed_study');
if ~exist(output_dir,'dir'), mkdir(output_dir); end
speeds=[0.2 0.5 1 2 3 4 5 6];
p=model_parameters(); p.BR=6.5;
d=pole_placement_settings(); d.lateral_states='balance_chi';
d.chi_poles=[-0.8 -0.9 -1 -1.1 -1.2];
d.longitudinal=[-0.8 -0.95 -1.05 -1.2]; d.method='acker';
s=simulation_settings(); s.t_end=60; s.output_dt=0.02; s.max_step=0.01;
% Verify that BR is physical damping of r_dot, not just sigma_r.
x=initial_state(s,p); x(1)=0.13; x(4)=-0.04;
p0=p; p0.BR=0; delta=model_residual(x,[0;0],p)-model_residual(x,[0;0],p0);
rdot=p.R*x(1)+x(4);
assert(norm(delta-p.BR*rdot*[p.R;0;0;1;0],inf)<1e-12);
assert(abs(x(1:5)'*delta-p.BR*rdot^2)<1e-12);
records={}; runs=cell(2,numel(speeds)); designs=cell(1,numel(speeds));
for scenario=1:2
    for j=1:numel(speeds)
        v=speeds(j); dj=d; dj.forward_speed=v;
        c=controller_parameters(); c.mode='pole_placement';
        c=generate_controller(p,c,dj); designs{j}=c.design_info;
        sj=s; sj.forward_speed0=v;
        if scenario==1
            name='configured'; % retain theta0=2.8deg, gamma0=0.1rad, etc.
        else
            name='small_heading';
            % Compatible perturbation: zero initial rolling invariant.
            sj.theta0=0; sj.r0=0; sj.gamma0=0;
            sj.theta_dot0=0; sj.r_dot0=0; sj.gamma_dot0=0;
            sj.psi0=0.1*pi/180; sj.psi_dot0=0; sj.phi0=0;
            sj.xG0=0; sj.yG0=0;
        end
        fprintf('%s, design/initial speed %.3g m/s\n',name,v);
        r=study_simulate(p,c,sj); r.label=sprintf('v=%.3g m/s',v);
        runs{scenario,j}=r;
        row.scenario=name; row.speed=v; row.end_time=r.t(end);
        row.tilt_event=any(r.event_index==1);
        row.rod_event=any(r.event_index==2);
        row.complete=isempty(r.event_time) && abs(r.t(end)-sj.t_end)<1e-6;
        row.peak_theta_deg=max(abs(r.X(:,7)))*180/pi;
        row.peak_r_m=max(abs(r.X(:,9)));
        row.peak_gamma_deg=max(abs(r.X(:,10)))*180/pi;
        row.peak_chi_deg=max(abs(r.X(:,6)))*180/pi;
        row.peak_force_N=max(abs(r.U(:,1))); row.peak_torque_Nm=max(abs(r.U(:,2)));
        row.final_theta_deg=r.X(end,7)*180/pi; row.final_r_m=r.X(end,9);
        row.final_gamma_deg=r.X(end,10)*180/pi; row.final_chi_deg=r.X(end,6)*180/pi;
        row.final_speed=p.R*r.X(end,2);
        row.initial_invariant=c.design_info.invariant*initial_state(sj,p);
        actual=[c.design_info.actual_lateral;c.design_info.actual_longitudinal];
        row.max_assigned_real=max(real(actual));
        row.pole_error=max(abs(sort(actual)-sort([dj.chi_poles(:);dj.longitudinal(:)])));
        assert(row.pole_error<1e-5);
        row.zero_poles=sum(abs(c.design_info.full_poles)<1e-6);
        tail=r.t>=max(0,r.t(end)-10);
        row.tail_theta_deg=max(abs(r.X(tail,7)))*180/pi;
        row.tail_r_m=max(abs(r.X(tail,9)));
        row.tail_gamma_deg=max(abs(r.X(tail,10)))*180/pi;
        row.tail_chi_deg=max(abs(r.X(tail,6)))*180/pi;
        row.tail_speed_error=max(abs(p.R*r.X(tail,2)-v));
        % Empirical convergence only, not a nonlinear stability proof.
        row.converged=row.complete && row.tail_theta_deg<0.1 && ...
            row.tail_r_m<0.001 && row.tail_gamma_deg<0.1 && ...
            row.tail_chi_deg<0.1 && row.tail_speed_error<0.001;
        records{end+1}=row; %#ok<SAGROW>
        fprintf('  t=%.6g, peak(theta,r,gamma)=(%.5g,%.5g,%.5g), converged=%d\n', ...
            row.end_time,row.peak_theta_deg,row.peak_r_m,row.peak_gamma_deg,row.converged);
    end
end
records=[records{:}];
summary=struct2table(records);
writetable(summary,fullfile(output_dir,'summary.csv'));
save(fullfile(output_dir,'study.mat'),'runs','designs','summary','p','d','s','speeds');
fid=fopen(fullfile(output_dir,'summary.json'),'w');
assert(fid>=0); fprintf(fid,'%s',jsonencode(records,PrettyPrint=true)); fclose(fid);
fid=fopen(fullfile(output_dir,'designs.json'),'w');
assert(fid>=0); fprintf(fid,'%s',jsonencode(designs,PrettyPrint=true)); fclose(fid);
for scenario=1:2
    f=figure('Name',sprintf('Damping speed study %d',scenario),'Position',[80 80 1100 950]);
    colors=turbo(numel(speeds)); labs={'r [m]','theta [deg]','gamma [deg]','chi [deg]'};
    for j=1:numel(speeds)
        r=runs{scenario,j}; data=[r.X(:,9),r.X(:,[7 10 6])*180/pi];
        for k=1:4
            subplot(4,1,k); hold on;
            plot(r.t,data(:,k),'Color',colors(j,:),'LineWidth',1.2,'DisplayName',r.label);
            ylabel(labs{k}); grid on;
        end
    end
    subplot(4,1,1); legend('show','Location','eastoutside');
    subplot(4,1,4); xlabel('Time [s]');
    exportgraphics(f,fullfile(output_dir,sprintf('scenario_%d.png',scenario)),'Resolution',140);
end
fprintf('Saved study to %s\n',output_dir);


function r=study_simulate(p,c,s)
% Study-only numerical divergence guard at |r|=2 m; not a physical stroke.
x0=initial_state(s,p);
control=@(t,x) controller_output(t,x,p,c);
opts=odeset('RelTol',s.rel_tol,'AbsTol',s.abs_tol,'MaxStep',s.max_step, ...
    'Events',@(t,x) study_events(t,x,s));
[t,X,te,xe,ie]=ode45(@(t,x) model_rhs(t,x,control(t,x),p), ...
    (0:s.output_dt:s.t_end)',x0,opts);
U=zeros(numel(t),2);
for k=1:numel(t), U(k,:)=control(t(k),X(k,:)')'; end
r.t=t; r.X=X; r.U=U;
r.event_time=te; r.event_state=xe; r.event_index=ie;
r.parameters=p; r.controller=c; r.settings=s;
end

function [value,isterminal,direction]=study_events(~,x,s)
value=[s.tilt_limit-abs(x(7));2-abs(x(9))];
isterminal=[1;1]; direction=[-1;-1];
end
