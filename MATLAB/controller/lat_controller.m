function M = lat_controller(t, z, par)

z = [99.8432,   46.2289,   29.7506,   13.1144];

theta = z(1);
theta_dot = z(2);
r = z(3);
r_dot = z(4);

M = par.K_theta*(theta - 0) + par.K_theta_dot*theta_dot + ...
    par.K_r*(r - 0) + par.K_r_dot*r_dot;
          
end