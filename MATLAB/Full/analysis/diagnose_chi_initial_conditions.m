function values=diagnose_chi_initial_conditions()
% Independent initial-condition cases; never changes config files.
% Columns: case,end_time,peak_theta,peak_r,peak_gamma,final_theta,final_r,final_gamma,final_chi,final_speed.
% Angles in returned values are degrees.
root=fileparts(fileparts(mfilename('fullpath'))); addpath(genpath(root));
p=model_parameters(); d=pole_placement_settings(); d.lateral_states='balance_chi';
c=controller_parameters(); c.mode='pole_placement'; c=generate_controller(p,c,d);
cases=[0 0.1 0;0.2 0.1 0;0.2 0 0.01;0.2 0 0.1;0 0 0];
rows={};
for j=1:size(cases,1)+1
 s=simulation_settings();s.t_end=60;s.output_dt=0.02;s.max_step=0.01;
 s.gamma0=0;s.theta0=0;s.r0=0;s.psi0=0;s.psi_dot0=0;
 if j<=size(cases,1)
  s.forward_speed0=cases(j,1);s.psi0=cases(j,2)*pi/180;s.theta0=cases(j,3)*pi/180;
 else
  s.forward_speed0=0.2;s.theta0=0.1*pi/180;
  a=c.design_info.A(3,1);s.psi_dot0=a*s.theta0/cos(s.theta0);
 end
 fprintf('Case %d: v0=%g, chi0=%g deg, theta0=%g deg, yawdot0=%g\n',j,s.forward_speed0,s.psi0*180/pi,s.theta0*180/pi,s.psi_dot0);
 r=simulate_unicycle(p,c,s);
 row=[j,r.t(end),max(abs(r.X(:,7)))*180/pi,max(abs(r.X(:,9))),max(abs(r.X(:,10)))*180/pi,r.X(end,7)*180/pi,r.X(end,9),r.X(end,10)*180/pi,r.X(end,6)*180/pi,p.R*r.X(end,2)];
 disp(row);rows{j}=row;
end
values=vertcat(rows{:});
end
