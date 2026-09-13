function s = simulation_settings()
s.t_end=15; s.output_dt=0.005; s.max_step=0.005;
s.rel_tol=1e-8; s.abs_tol=1e-10;
s.tilt_limit=80*pi/180;
s.theta0=2.8*pi/180; s.r0=0; s.gamma0=0.1;
s.theta_dot0=0; s.r_dot0=0; s.gamma_dot0=0;
s.psi0=0; s.psi_dot0=0; s.phi0=0;
s.forward_speed0=0.2; % R*sigma2, wheel-center forward component
s.xG0=0; s.yG0=0;
end
