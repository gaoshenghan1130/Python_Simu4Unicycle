function compare_mate_longitudinal_modes()
% Requires the saved output of run_mate_oscillation_study.
root=fileparts(fileparts(mfilename('fullpath')));addpath(genpath(root));
a=load(fullfile(root,'results','mate_oscillation_study','screening.mat'));base=a.runs{23};
rows={};
for j=1:2
 p=base.parameters;s=base.settings;s.t_end=15;s.max_step=0.005;c=base.controller;
 if j==1,c.mode='open_loop';else,c.mode='lateral_open_loop';end
 r=simulate_unicycle(p,c,s);
 row.mode=c.mode;row.mass=p.mr;row.speed=s.forward_speed0;row.theta0_deg=s.theta0*180/pi;
 row.end_time=r.t(end);row.peak_theta_deg=max(abs(r.X(:,7)))*180/pi;
 row.peak_gamma_deg=max(abs(r.X(:,10)))*180/pi;row.peak_r=max(abs(r.X(:,9)));
 row.peak_torque=max(abs(r.U(:,2)));rows{j}=row;disp(row);
end
fid=fopen(fullfile(root,'results','mate_oscillation_study','control_comparison.json'),'w');
fprintf(fid,'%s',jsonencode([rows{:}],PrettyPrint=true));fclose(fid);

end
