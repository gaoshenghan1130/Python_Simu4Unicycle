function [t, Z] = timeConstantSimu(par, model, controller)
    % init
    z0 = [3 * pi/180; 0; 0; 0.0];

    % Simulation settings
    dt = 0.001; 
    T  = 15;           
    t  = 0:dt:T;

    Z = zeros(length(t), length(z0));
    Z(1, :) = z0';

    for k = 1:length(t)-1
        zk = Z(k, :)';
        dz = model(t(k), zk, par, controller);
        zk_next = zk + dt * dz;
        Z(k+1, :) = zk_next';
    end
end