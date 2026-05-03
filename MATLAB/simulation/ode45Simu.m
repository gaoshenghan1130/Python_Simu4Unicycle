function [t, Z] =  ode45Simu(par, model, controller)
    z0 = [0.0; 0.0; 0.0; 0.0];
    t_eval = linspace(0, 15, 1000);
    options = odeset('RelTol', 1e-6, 'AbsTol', 1e-8, 'MaxStep', 0.01);
    model =@segway_rolling_resistance_model;

    function dz = looper(t_ , Z_)
        dz = model(t_, Z_, par, controller);
    end

    [t, Z] = ode45(@looper, t_eval, z0, options);
end