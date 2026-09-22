function run_pole_tolerance_study(reuse_saved)
if nargin<1,reuse_saved=false;end
% Recompute the rolling Jacobian at each speed; vary lateral poles only.
root=fileparts(mfilename('fullpath'));addpath(genpath(root));
out=fullfile(root,'..','..','Derivation','FullModel','figures','pole_tolerance');
if ~exist(out,'dir'),mkdir(out);end
% Use the saved report baseline so later global configuration edits cannot
% silently change the comparison with the open-loop figures.
base=load(fullfile(out,'..','pole_placement_report','experiments.mat'),'all_runs');
p=base.all_runs{1}.parameters;p.mr=2.3;p.BR=6.5;p.BP=0;
s=base.all_runs{1}.settings;s.theta0=0;s.t_end=30;
s.r0=0;s.gamma0=0;s.psi0=0;s.theta_dot0=0;s.r_dot0=0;s.gamma_dot0=0;s.psi_dot0=0;
c=base.all_runs{1}.controller;c.mode='pole_placement';c.hold_gamma_zero=false;
c.force_limit=Inf;c.torque_limit=Inf;
d=pole_placement_settings();d.lateral_states='balance_chi';d.method='acker';
d.longitudinal=[-0.8 -0.95 -1.05 -1.2];
all_runs={};records={};decisions={};theta_values=[0.00001 0.0001 0.001 0.01 0.1 1 5];
if reuse_saved
 cached=load(fullfile(out,'experiments.mat'),'all_runs','summary','p');
 assert(isequaln(cached.p,p),'Cached physical parameters differ; run without reuse_saved.');
 all_runs=cached.all_runs;records=num2cell(table2struct(cached.summary)');
end
for speed=[2.375 1]
 s.forward_speed0=speed;d.forward_speed=speed;scales=[0.5 1 2];scores=[];
 for scale=scales
  scores(end+1)=test_theta(scale);
 end
 % Extend each direction that improves the largest tested near-zero
 % theta over the baseline. Stop after four extensions (up to a center of -32) or no improvement.
 for direction=[-1 1]
  if direction<0,edge=0.5;score=scores(1);else,edge=2;score=scores(3);end
  if score>scores(2)
   for extra=1:4
    candidate=edge*2^direction;next_score=test_theta(candidate);scales(end+1)=candidate;
    decisions{end+1}=sprintf('v=%g: extended %g to %g; largest near-zero theta %g to %g deg',speed,edge,candidate,score,next_score);
    if next_score<=score,break;end
    edge=candidate;score=next_score;
   end
  end
 end
 scales=sort(scales);
 % Repeat r and chi initial-condition tests at every tested pole location.
 for scale=scales
  for value=[0.001 0.005 0.01],run_case('r',value,scale);end
  for value=[1 5 10],run_case('chi',value,scale);end
 end
 for group={'theta','r','chi'}
  name=group{1};fig=figure('Visible','off','Position',[30 30 max(1500,350*numel(scales)) 850]);
  colors=lines(7);
  for col=1:numel(scales)
   ids=find(cellfun(@(z)z.speed==speed && z.scale==scales(col) && strcmp(z.group,name),records));
   for row=1:3
    ax=subplot(3,numel(scales),(row-1)*numel(scales)+col);hold(ax,'on');
    for j=1:numel(ids)
     run=all_runs{ids(j)};rec=records{ids(j)};vals=[run.X(:,7)*180/pi run.X(:,9) run.X(:,6)*180/pi];
     label=sprintf('%g',rec.value);if rec.stop,label=sprintf('%s (stop %.1fs)',label,rec.end_time);end
     plot(run.t,vals(:,row),'Color',colors(j,:),'DisplayName',label,'LineWidth',1);
    end
    grid on;xlim([0 s.t_end]);labs={'theta [deg]','r [m]','chi [deg]'};ylabel(labs{row});
    if row==1,title(sprintf('Lateral poles near -%g',scales(col)));legend('show','Location','best','FontSize',7);end
    if row==3,xlabel('Time [s]');end
   end
  end
  unit='deg';if strcmp(name,'r'),unit='m';end
  sgtitle(sprintf('Initial %s [%s] | v = %g m/s | mr = %g kg | b = %g',name,unit,speed,p.mr,p.BR));
  exportgraphics(fig,fullfile(out,sprintf('v_%g_%s.png',speed,name)),'Resolution',150);close(fig);
 end
end
summary=struct2table([records{:}]);writetable(summary,fullfile(out,'summary.csv'));
fid=fopen(fullfile(out,'summary.json'),'w');fprintf(fid,'%s',jsonencode([records{:}],PrettyPrint=true));fclose(fid);
save(fullfile(out,'experiments.mat'),'all_runs','summary','p','s','c','d','decisions');
plot_pole_tolerance_summary(out);
fid=fopen(fullfile(out,'decisions.txt'),'w');fprintf(fid,'%s\n',decisions{:});fclose(fid);
 function score=test_theta(scale)
  score=0;
  for value=theta_values
   rec=run_case('theta',value,scale);
   if rec.near_zero,score=max(score,value);end
  end
 end
 function rec=run_case(group,value,scale)
  if reuse_saved && ~isempty(records)
   hit=find(cellfun(@(z)z.speed==speed && z.scale==scale && strcmp(z.group,group) && z.value==value,records),1);
   if ~isempty(hit),rec=records{hit};return;end
  end
  dd=d;dd.chi_poles=scale*[-0.8 -0.9 -1 -1.1 -1.2];cc=generate_controller(p,c,dd);ss=s;
  switch group
   case 'theta',ss.theta0=value*pi/180;
   case 'r',ss.r0=value;
   case 'chi',ss.psi0=value*pi/180;
  end
  x=initial_state(ss,p);control=@(t,x)controller_output(t,x,p,cc);
  opts=odeset('RelTol',ss.rel_tol,'AbsTol',ss.abs_tol,'MaxStep',ss.max_step,'Events',@stop_events);
  [t,X,te,~,ie]=ode45(@(t,x)model_rhs(t,x,control(t,x),p),(0:ss.output_dt:ss.t_end)',x,opts);
  rec.mass=p.mr;rec.damping=p.BR;rec.speed=speed;rec.scale=scale;rec.group=group;rec.value=value;rec.end_time=t(end);rec.stop=0;
  if ~isempty(ie),rec.stop=ie(end);end
  rec.theta_peak=max(abs(X(:,7)))*180/pi;rec.r_peak=max(abs(X(:,9)));rec.chi_peak=max(abs(X(:,6)))*180/pi;
  last=t>=25;rec.near_zero=rec.stop==0 && t(end)>=30-1e-6 && ...
   max(abs(X(last,7)))*180/pi<0.1 && max(abs(X(last,9)))<0.001 && ...
   max(abs(X(last,6)))*180/pi<0.1 && max(abs(X(last,10)))*180/pi<0.1;
  rec.final_theta=X(end,7)*180/pi;rec.final_r=X(end,9);rec.final_chi=X(end,6)*180/pi;
  U=zeros(numel(t),2);for k=1:numel(t),U(k,:)=control(t(k),X(k,:)')';end
  rec.force_peak=max(abs(U(:,1)));rec.torque_peak=max(abs(U(:,2)));
  run=struct('t',t,'X',X,'U',U,'events',te,'parameters',p,'settings',ss,'controller',cc,'design',dd);
  all_runs{end+1}=run;records{end+1}=rec;
  fprintf('v=%g scale=%g %s=%g end=%.3f stop=%d nearzero=%d\n',speed,scale,group,value,t(end),rec.stop,rec.near_zero);
  % Save progress in case a later design or run fails.
  save(fullfile(out,'progress.mat'),'all_runs','records');
 end
end
function [value,terminal,direction]=stop_events(~,x)
value=[80*pi/180-abs(x(7));0.5-abs(x(9));80*pi/180-abs(x(10))];terminal=ones(3,1);direction=-ones(3,1);
end
