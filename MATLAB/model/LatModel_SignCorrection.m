function dz = LatModel_SignCorrection(t, z, par, controller)
% Latitude Nonlinear Model (Standard Formulation)
%
% State:
% z = [theta; theta_dot; r; r_dot]
%
% Input:
% F = General Force

z = z(:);
if numel(z) ~= 4
    error('State vector z must have four elements: [theta; theta_dot; r; r_dot]');
end

% Controller force
F = controller(t, z, par);
assert(isscalar(F))
assert(isfinite(F))

% States
theta     = z(1);
theta_dot = z(2);
r         = z(3);
r_dot     = z(4);

% Parameters 
% (Note: Ensure your 'par' struct contains m_p. If your previous code used m_B, 
% you may need to map m_p = par.m_B instead)
R   = par.R;
g   = par.g;
m_w = par.m_W;
m_L = par.m_L; 
m_p = par.m_B; 
h   = par.h;

% Mass Matrix (M)
M_matrix = [
    2 * m_L * (R^2 + r^2) + m_w * R^2 + m_p * (R + h)^2,  -2 * m_L * R;
    -2 * m_L * R,                                          2 * m_L
];

% Right Side of the Equation (Forces, Coriolis, Gravity)
M_rightside = [
    -4 * m_L * r * r_dot * theta_dot + (2 * m_L * R + m_w * R + m_p * (R + h)) * g * sin(theta) - 2 * m_L * g * r * cos(theta);
    F + 2 * m_L * r * theta_dot^2 - 2 * m_L * g * sin(theta)
];

% Check for singularities or invalid numbers
if any(~isfinite(M_matrix(:)))
    error('M_matrix contains NaN or Inf');
end
if rcond(M_matrix) < 1e-12
    error('LatModel_SignCorrection:SingularMatrix', 'M_matrix singular');
end

% Solve for standard accelerations [theta_ddot; r_ddot]
accel = M_matrix \ M_rightside;
theta_ddot = accel(1);
r_ddot     = accel(2);

% State derivative
dz = [
    theta_dot;
    theta_ddot;
    r_dot;
    r_ddot
];
end