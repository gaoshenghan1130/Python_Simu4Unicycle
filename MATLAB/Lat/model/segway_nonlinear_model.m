function dz = segway_nonlinear_model(t, z, par, controller)
%SEGWAY_NONLINEAR_MODEL Nonlinear Segway model (no damping, no rolling resistance)
%
% State:
% z = [x; x_dot; gamma; gamma_dot]
%
% Input:
% M = motor torque (scalar)

if ~strcmp(par.scenario, "Segway")
    error('param set does not match the model')
end

z = z(:);
if numel(z) ~= 4
    error('State vector z must have four elements: [x; x_dot; gamma; gamma_dot]');
end

M = controller(t, z, par);
x   = z(1);
x_dot   = z(2);
gamma   = z(3);
gamma_dot = z(4);

m   = par.m;
m_w = par.m_w * 1.5;  
h   = par.h;
g   = par.g;
R   = par.R;

% Mass matrix
M_matrix = [m + m_w,      m*h*cos(gamma); ...
            m*h*cos(gamma), m*h^2];

% Inverse (same as your Python version)
N = inv(M_matrix);

% Accelerations
x_ddot = N(1,1) * (M / R + m*h*gamma_dot^2*sin(gamma)) + ...
         N(1,2) * (-M + m*g*h*sin(gamma));

gamma_ddot = N(2,1) * (M / R + m*h*gamma_dot^2*sin(gamma)) + ...
             N(2,2) * (-M + m*g*h*sin(gamma));

dz = [x_dot; x_ddot; gamma_dot; gamma_ddot];

end