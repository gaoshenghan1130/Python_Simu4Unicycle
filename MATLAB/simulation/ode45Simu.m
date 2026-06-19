function [t, Z] =  ode45Simu(par, model, controller)
    z0 = [0.001 * pi/180; 0.0; 0.0; 0.0];
    t_eval = linspace(0, 3, 1000);

    function dz = looper(t_ , Z_)
        dz = model(t_, Z_, par, controller);
    end

    [t, Z] = ode45(@looper, t_eval, z0);
end