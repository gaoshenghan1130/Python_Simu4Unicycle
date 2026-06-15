function par = LatParam()
    par.mass_at_end = 0.3; % mass of the object attached at the end of the rod
    par.m_L = 0.15 + par.mass_at_end; % actually half of the rod
    par.m_B = 4.1;
    par.m_W = 3.5;
    par.h = 0.115;
    par.R = 0.2527;
    par.g = 9.81;
    par.I_b = 0.012904182130007; %x axis battery inertia
    par.I_w = 0.0459142776779452; %x axis  wheel inertia
    par.I_rod = 1/12 * (0.3 ) * (0.314^2) + 2 * par.mass_at_end * 0.157^2;
    par.K_theta_dot = 0.5;
    par.K_r = 2;
    par.K_r_dot = 0.5;


