function c = controller_parameters()
% Controller mode: 'open_loop', 'pd', 'direct', or 'pole_placement'.
% pd preserves the original physical-derivative PD law and sign convention.
c.mode='pole_placement';
% direct: u=-K*(x-x_ref), with K sized 2-by-12 in the model state order.
% Fill K before selecting direct; an empty K is intentionally rejected.
c.K=[];
c.x_ref=zeros(12,1);
% pole_placement generates K anew for each run and uses x_ref=0 (rest).
% PD gains below are used only in pd mode.
c.kp_theta=-797.095497; c.kd_theta=-103.668186;
c.kp_r=854.34418; c.kd_r=105.568949;
c.kp_gamma=3.0; c.kd_gamma=0.8;
c.theta_ref=0; c.theta_dot_ref=0;
c.r_ref=0; c.r_dot_ref=0;
c.gamma_ref=0; c.gamma_dot_ref=0;
c.force_limit=Inf;    % N; set a finite value to simulate saturation
c.torque_limit=Inf;   % N*m
end
