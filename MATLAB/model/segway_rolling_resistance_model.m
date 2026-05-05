function dz = segway_rolling_resistance_model(t, z, par, controller)
%SEGWAY_ROLLING_RESISTANCE_MODEL Longitudinal Segway model.
%
% Converted from Derivation/Segway/Models/Segway_model_rollingResistance.py.
%
% State order:
% z = [x; x_dot; gamma; gamma_dot]
%
% Control:
% M = wheel motor torque (N*m)
if ~strcmp(par.scenario, "Segway")
    error('param set does not math the model')
end

z = z(:);

x_dot = z(2);
gamma = z(3);
gamma_dot = z(4);

m = par.m;
m_w = par.m_w + par.I/par.R^2;
h = par.h;
R = par.R;
g = par.g;

M = controller(t, z, par);

mass_matrix = [m + m_w, m*h*cos(gamma); ...
               m*h*cos(gamma), m*h^2];

omega = x_dot/R - gamma_dot;
motor_damping = par.B*omega + par.B_0*sign(omega);

rolling_resistance = par.mu_rolling*(m + m_w)*g* ...
    tanh(par.smooth_factor*x_dot)/R;

rhs = [(M - motor_damping)/R - rolling_resistance + ...
       m*h*gamma_dot^2*sin(gamma); ...
       -(M - motor_damping) + m*g*h*sin(gamma)];

if any(~isfinite(mass_matrix(:))) || any(~isfinite(rhs))
    error('segway_rolling_resistance_model:NonFiniteDynamics', ...
        'Dynamics became non-finite at t = %.6g s.', t);
end

accel = mass_matrix\rhs;

dz = [x_dot; accel(1); gamma_dot; accel(2)];
end
