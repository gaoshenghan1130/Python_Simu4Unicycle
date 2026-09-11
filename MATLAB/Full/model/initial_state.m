function x0 = initial_state(s,p)
sigma1=s.theta_dot0;
sigma2=s.forward_speed0/p.R;
sigma3=s.psi_dot0*cos(s.theta0);
sigma_r=s.r_dot0-p.R*s.theta_dot0;
sigma_g=s.gamma_dot0+s.psi_dot0*sin(s.theta0);
x0=[sigma1;sigma2;sigma3;sigma_r;sigma_g; ...
    s.psi0;s.theta0;s.phi0;s.r0;s.gamma0;s.xG0;s.yG0];
end
