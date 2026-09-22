% Reproduce the STRUCTURE described by Mate: F=0, gamma-only PD, BR=BP=0.
% His exact physical parameters and gains remain unknown.
root=fileparts(mfilename('fullpath')); addpath(genpath(root));
output_dir=fullfile(root,'results','mate_oscillation_study');
if ~exist(output_dir,'dir'), mkdir(output_dir); end
base_p=model_parameters(); base_p.BR=0; base_p.BP=0;
base_c=controller_parameters(); base_c.mode='lateral_open_loop'; base_c.hold_gamma_zero=false;
base_c.gamma_ref=0; base_c.gamma_dot_ref=0;
base_s=simulation_settings(); base_s.t_end=30; base_s.output_dt=0.01; base_s.max_step=0.02;
base_s.theta_dot0=0; base_s.r0=0; base_s.r_dot0=0;
base_s.gamma0=0; base_s.gamma_dot0=0; base_s.psi0=0; base_s.psi_dot0=0;
base_s.phi0=0; base_s.xG0=0; base_s.yG0=0;
% Mass-only sensitivity: JR is held fixed, NOT assumed to match Mate's rod.
cases=[];
for mass=[0.272 1 2.3]
    for speed=[1.5 1.75 2 2.375 3]
        for lean=[0.1 1 5]
            cases(end+1,:)=[mass speed lean 0 3 0.8]; %#ok<SAGROW>
        end
    end
end
% Isolate the effect of initial gamma and its PD tuning at a promising speed.
for gamma0=[0 0.1]
    for gains=[3 0.8;6 0.8;6 1.6;12 2.4]'
        cases(end+1,:)=[0.272 2.375 1 gamma0 gains']; %#ok<SAGROW>
    end
end
% Current configuration and a lateral perturbation added to that same state.
cases(end+1,:)=[base_p.mr 2 0 0.1 3 0.8];
cases(end+1,:)=[base_p.mr 2 1 0.1 3 0.8];
cases=unique(cases,'rows','stable');
rows={}; runs=cell(size(cases,1),1);
for k=1:size(cases,1)
    p=base_p; p.mr=cases(k,1);
    s=base_s; s.forward_speed0=cases(k,2);s.theta0=cases(k,3)*pi/180;s.gamma0=cases(k,4);
    c=base_c;c.kp_gamma=cases(k,5);c.kd_gamma=cases(k,6);
    fprintf('%d/%d: mr=%g v=%g theta0=%g deg gamma0=%g rad PD=(%g,%g)\n', ...
        k,size(cases,1),cases(k,:));
    r=screen_simulate(p,c,s); runs{k}=r;
    row.case_id=k;row.mass=p.mr;row.speed=s.forward_speed0;
    row.theta0_deg=s.theta0*180/pi;row.gamma0_rad=s.gamma0;
    row.kp=c.kp_gamma;row.kd=c.kd_gamma;
    row.end_time=r.t(end);row.completed=isempty(r.event_time)&&abs(r.t(end)-s.t_end)<1e-6;
    row.event_index=0;if ~isempty(r.event_index),row.event_index=r.event_index(end);end
    row.peak_theta_deg=max(abs(r.X(:,7)))*180/pi;
    row.peak_r_m=max(abs(r.X(:,9)));row.peak_gamma_deg=max(abs(r.X(:,10)))*180/pi;
    row.speed_min=min(p.R*r.X(:,2));row.speed_max=max(p.R*r.X(:,2));
    row.peak_torque=max(abs(r.U(:,2)));row.peak_force=max(abs(r.U(:,1)));
    mid=r.t>=10 & r.t<20;last=r.t>=20;
    row.mid_theta_range_deg=range_or_nan(r.X(mid,7))*180/pi;
    row.last_theta_range_deg=range_or_nan(r.X(last,7))*180/pi;
    row.amplitude_ratio=row.last_theta_range_deg/max(row.mid_theta_range_deg,1e-12);
    xd=r.X(:,1);row.turning_points=sum(xd(1:end-1).*xd(2:end)<0);
    row.bounded_test=row.completed && row.peak_theta_deg<10 && ...
        row.peak_gamma_deg<5 && row.peak_r_m<0.2 && ...
        max(abs([row.speed_min row.speed_max]-row.speed))<0.1*row.speed;
    row.oscillatory=row.bounded_test && row.last_theta_range_deg>0.01 && ...
        row.turning_points>=8 && row.amplitude_ratio>0.5 && row.amplitude_ratio<2;
    rows{end+1}=row; %#ok<SAGROW>
    fprintf('  end=%g, thetaMax=%g deg gammaMax=%g deg rMax=%g m osc=%d\n', ...
        row.end_time,row.peak_theta_deg,row.peak_gamma_deg,row.peak_r_m,row.oscillatory);
end
records=[rows{:}];summary=struct2table(records);
writetable(summary,fullfile(output_dir,'screening.csv'));
fid=fopen(fullfile(output_dir,'screening.json'),'w');fprintf(fid,'%s',jsonencode(records,PrettyPrint=true));fclose(fid);
save(fullfile(output_dir,'screening.mat'),'runs','summary','cases','base_p','base_s','base_c');
disp(summary(summary.oscillatory,{'case_id','mass','speed','theta0_deg','gamma0_rad','kp','kd','peak_theta_deg','peak_gamma_deg','amplitude_ratio'}));
fprintf('Saved screening results to %s\n',output_dir);

function r=screen_simulate(p,c,s)
x0=initial_state(s,p);control=@(t,x) controller_output(t,x,p,c);
opts=odeset('RelTol',s.rel_tol,'AbsTol',s.abs_tol,'MaxStep',s.max_step, ...
    'Events',@limits);
[t,X,te,xe,ie]=ode45(@(t,x) model_rhs(t,x,control(t,x),p), ...
    (0:s.output_dt:s.t_end)',x0,opts);
U=zeros(numel(t),2);for j=1:numel(t),U(j,:)=control(t(j),X(j,:)')';end
r.t=t;r.X=X;r.U=U;r.event_time=te;r.event_state=xe;r.event_index=ie;
r.parameters=p;r.controller=c;r.settings=s;
end
function [value,terminal,direction]=limits(~,x)
% Screening stops, NOT physical limits or reaction forces.
value=[45*pi/180-abs(x(7));0.5-abs(x(9));30*pi/180-abs(x(10))];
terminal=ones(3,1);direction=-ones(3,1);
end
function v=range_or_nan(x)
if isempty(x),v=NaN;else,v=max(x)-min(x);end
end
