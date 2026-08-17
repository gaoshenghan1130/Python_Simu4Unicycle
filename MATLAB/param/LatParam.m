function par = LatParam()

    %% Lateral moving assembly

    % Complete moving assembly in the full model:
    % 0.3 kg rod + two 1.0 kg end assemblies
    par.mass_at_end = 1.0;

    % Half of the rod plus one end mass
    par.m_L = 0.15 + par.mass_at_end;

    % Total moving mass:
    % 2 * (0.15 + 1.0) = 2.3 kg
    par.m_rod = 2 * par.m_L;


    %% Main body and wheel
    % Synchronized with the full-model parameters

    % mp in the full model
    par.m_B = 2.799;

    % mw in the full model
    par.m_W = 2.436;

    par.h = 0.025;
    par.R = 0.253;
    par.g = 9.81;


    %% Moments of inertia

    % JPx in the full model
    par.I_b = 0.01290418213;

    % JW2 in the full model
    par.I_w = 0.04591427768;

    % JR in the full model:
    % uniform 0.3 kg rod
    % + two 1.0 kg end masses at +/-0.157 m
    par.I_rod = ...
        (1/12) * 0.3 * 0.314^2 ...
        + 2 * par.mass_at_end * 0.157^2;


    %% Reduced-model constants

    % Constant part of the lateral angular inertia
    par.J = ...
        par.m_W * par.R^2 ...
        + par.m_B * (par.R + par.h)^2 ...
        + par.I_w ...
        + par.I_b ...
        + par.I_rod;

    % Lateral gravity coefficient
    par.G = ...
        par.m_W * par.g * par.R ...
        + par.m_B * par.g * (par.R + par.h);


    %% LQR gains
    %
    % Designed using the reduced state:
    %
    % x_reduced = [theta; q2; theta_dot; u2]
    %
    % where:
    %
    % q2 = r - R*theta
    % u2 = r_dot - R*theta_dot
    %
    % LQR weights:
    %
    % Q = diag([1000, 100, 100, 10])
    % R_weight = 30
    %
    % Gains below are transformed to the physical state:
    %
    % x_physical = [theta; r; theta_dot; r_dot]
    %
    % F = -K_physical*x_physical

    par.K_theta = -804.12848241;
    par.K_r = 882.43017806;
    par.K_theta_dot = -144.12564980;
    par.K_r_dot = 155.37589642;


    %% Gain vectors for different state orders

    % Physical state order:
    %
    % x_physical = [theta; r; theta_dot; r_dot]
    %
    % F = -par.K_physical*x_physical

    par.K_physical = [
        par.K_theta, ...
        par.K_r, ...
        par.K_theta_dot, ...
        par.K_r_dot ...
    ];


    % LatModelAppell state order:
    %
    % z = [theta; theta_dot; r; r_dot]
    %
    % F = -par.K_appell_state*z

    par.K_appell_state = [
        par.K_theta, ...
        par.K_theta_dot, ...
        par.K_r, ...
        par.K_r_dot ...
    ];

end