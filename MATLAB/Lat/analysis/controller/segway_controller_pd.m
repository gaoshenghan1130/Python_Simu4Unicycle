function M = segway_controller_pd(t, z, par)
% PD controller for Segway

if ~strcmp(par.scenario, "Segway")
    error('param set does not match the controller')
end


x = z(1);
x_dot = z(2);
gamma = z(3);
gamma_dot = z(4);

desired_gamma   = par.desired_gamma;
desired_velocity = par. desired_velocity;
desired_position = par. desired_position;

K_gamma   = par.K_gamma;
K_dgamma  = par.K_dgamma;
K_velocity = par.K_velocity;
K_position = par.K_position;

switch lower(par.control_mode)
    case 'balance'
        M = K_gamma*(gamma - desired_gamma) + K_dgamma*gamma_dot;
          
    case 'velocity'
        M = K_gamma*(gamma - desired_gamma) + ...
            K_dgamma*gamma_dot + ...
            K_velocity*(x_dot - desired_velocity);

    case 'position'
        M = K_gamma*(gamma - desired_gamma) + ...
            K_dgamma*gamma_dot + ...
            K_velocity*(x_dot - desired_velocity) + ...
            K_position*(x - desired_position);
        
    otherwise
        error('control_mode must be balance, velocity, or position.');
end
end