function par = LatParam()
    par.m_L = 0.27;
    par.m_B = 3.1;
    par.m_W = 3.5;
    par.h = 0.115;
    par.R = 0.2527;
    par.g = 9.81;
    par.I_b = 0.012904182130007; %x axis
    par.I_w = 0.0459142776779452; %x axis 
    par.I_rod = 1/12 * (0.300) * (0.314^2); %mass [kg] length [m]

    par.K_theta = 2;
    par.K_theta_dot = 0.5;
    par.K_r = 2;
    par.K_r_dot = 0.5;


