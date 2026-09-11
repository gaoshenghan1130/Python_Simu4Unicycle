function c = controller_parameters()
% Same PD law and gains as the original notebook simulation.
c.kp_theta=-797.095497; c.kd_theta=-103.668186;
c.kp_r=854.34418; c.kd_r=105.568949;
c.kp_gamma=3.0; c.kd_gamma=0.8;
c.theta_ref=0; c.theta_dot_ref=0;
c.r_ref=0; c.r_dot_ref=0;
c.gamma_ref=0; c.gamma_dot_ref=0;
c.force_limit=Inf;    % N; set a finite value to simulate saturation
c.torque_limit=Inf;   % N*m
end
