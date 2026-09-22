function run_report_experiments()
% Comparable figures for FullModelPolePlacement_withDamping.md.
root=fileparts(mfilename('fullpath'));addpath(genpath(root));
out=fullfile(root,'..','..','Derivation','FullModel','figures','pole_placement_report');
if ~exist(out,'dir'),mkdir(out);end
p=model_parameters();p.BR=0;p.BP=0;
s=simulation_settings();s.t_end=30;s.output_dt=0.01;s.max_step=0.01;
s.theta0=0;s.r0=0;s.gamma0=0;s.theta_dot0=0;s.r_dot0=0;s.gamma_dot0=0;
s.psi0=0;s.psi_dot0=0;s.phi0=0;s.xG0=0;s.yG0=0;s.forward_speed0=2.375;
d=pole_placement_settings();d.lateral_states='balance_chi';d.forward_speed=2.375;
d.chi_poles=[-0.8 -0.9 -1 -1.1 -1.2];d.longitudinal=[-0.8 -0.95 -1.05 -1.2];d.method='acker';
c=controller_parameters();c.hold_gamma_zero=false;c.gamma_ref=0;c.gamma_dot_ref=0;
c.force_limit=Inf;c.torque_limit=Inf;all_runs={};records={};
for j=1:4
 masses=[0.272 1 2.3 0.272];damping=[0 0 0 6.5];
 pp=p;pp.mr=masses(j);pp.BR=damping(j);ss=s;ss.theta0=pi/180;
 cc=c;cc.mode='lateral_open_loop';cc.kp_gamma=3;cc.kd_gamma=0.8;
 r=integrate(pp,cc,ss);all_runs{end+1}=r;records{end+1}=measure(r,sprintf('open_%d',j),pp.mr,pp.BR);
 fig=figure('Visible','off','Position',[50 50 1150 900]);
 vals=[r.X(:,7)*180/pi,r.X(:,9),r.X(:,10)*180/pi,r.X(:,6)*180/pi,r.U];
 labs={'theta [deg]','r [m]','gamma [deg]','psi [deg]','F [N]','M2 [N m]'};
 for k=1:6
  subplot(3,2,k);plot(r.t,vals(:,k),'LineWidth',1.1);grid on;ylabel(labs{k});xlabel('Time [s]');xlim([0 s.t_end]);
 end
 sgtitle(sprintf('Lateral open loop + gamma PD | mr = %g kg | b = %g',pp.mr,pp.BR));
 exportgraphics(fig,fullfile(out,sprintf('open_%d.png',j)),'Resolution',140);close(fig);
 fprintf('Open mr=%g: thetaMax=%g gammaMax=%g\n',pp.mr,max(abs(vals(:,1))),max(abs(vals(:,3))));
end
groups={}; % Closed-loop experiments are handled by the focused study below.
records=[records{:}];summary=struct2table(records);
writetable(summary,fullfile(out,'summary.csv'));
fid=fopen(fullfile(out,'summary.json'),'w');fprintf(fid,'%s',jsonencode(records,PrettyPrint=true));fclose(fid);
save(fullfile(out,'experiments.mat'),'all_runs','summary','p','s','c','d','groups');
fprintf('Open-loop figures/data saved to %s\n',out);
run_pole_tolerance_study();
end
function r=integrate(p,c,s)
x=initial_state(s,p);control=@(t,x)controller_output(t,x,p,c);
opts=odeset('RelTol',s.rel_tol,'AbsTol',s.abs_tol,'MaxStep',s.max_step,'Events',@stop_events);
[t,X,te,xe,ie]=ode45(@(t,x)model_rhs(t,x,control(t,x),p),(0:s.output_dt:s.t_end)',x,opts);
U=zeros(numel(t),2);for j=1:numel(t),U(j,:)=control(t(j),X(j,:)')';end
r.t=t;r.X=X;r.U=U;r.event_time=te;r.event_state=xe;r.event_index=ie;
r.parameters=p;r.controller=c;r.settings=s;
end
function [value,terminal,direction]=stop_events(~,x)
% Numerical screening limits; no physical contact forces are added.
value=[80*pi/180-abs(x(7));0.5-abs(x(9));80*pi/180-abs(x(10))];terminal=ones(3,1);direction=-ones(3,1);
end
function row=measure(r,group,value,b)
row.group=group;row.value=value;row.b=b;row.end_time=r.t(end);
row.stop=0;if ~isempty(r.event_index),row.stop=r.event_index(end);end
row.theta_peak=max(abs(r.X(:,7)))*180/pi;row.r_peak=max(abs(r.X(:,9)));
row.gamma_peak=max(abs(r.X(:,10)))*180/pi;row.psi_peak=max(abs(r.X(:,6)))*180/pi;
row.force_peak=max(abs(r.U(:,1)));row.torque_peak=max(abs(r.U(:,2)));
row.final_theta=r.X(end,7)*180/pi;row.final_r=r.X(end,9);
row.final_gamma=r.X(end,10)*180/pi;row.final_psi=r.X(end,6)*180/pi;
last=r.t>=max(0,r.settings.t_end-5);
row.near_zero=row.stop==0 && abs(r.t(end)-r.settings.t_end)<1e-6 && any(last) && ...
 max(abs(r.X(last,7)))*180/pi<0.1 && max(abs(r.X(last,9)))<0.001 && ...
 max(abs(r.X(last,10)))*180/pi<0.1 && max(abs(r.X(last,6)))*180/pi<0.1;
end
