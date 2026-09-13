function u = pd_controller(t,x,p,c)
% Output is physical force F and wheel torque M2; no sign flip here.
theta=x(7); r=x(9); gamma=x(10);
theta_dot=x(1);
r_dot=p.R*x(1)+x(4);
gamma_dot=x(5)-x(3)*tan(theta);
F=c.kp_theta*(c.theta_ref-theta) ...
  +c.kd_theta*(c.theta_dot_ref-theta_dot) ...
  +c.kp_r*(c.r_ref-r)+c.kd_r*(c.r_dot_ref-r_dot);
M2=c.kp_gamma*(gamma-c.gamma_ref) ...
   +c.kd_gamma*(gamma_dot-c.gamma_dot_ref);
F=max(-c.force_limit,min(c.force_limit,F));
M2=max(-c.torque_limit,min(c.torque_limit,M2));
u=[F;M2];
end
