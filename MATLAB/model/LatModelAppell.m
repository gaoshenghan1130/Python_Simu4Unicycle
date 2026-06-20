function dz = LatModelAppell(t, z, par, controller)
% Latitude Nonlinear Model (Appell Formulation)
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

% Parameters (Mapped identically to the MPC implementation)
R     = par.R;
g     = par.g;
m_w   = (par.m_W * par.R + par.m_B * par.h) / par.R;
m_rod = 2 * par.m_L; 

% Appell quasi-velocities
u1 = theta_dot;
u2 = r_dot - R * theta_dot;

% Appell Mass Matrix
M_matrix = [
    m_w * R^2 + m_rod * r^2, 0;
    0,                       m_rod
];

% Appell Right Side (Forces/Coriolis/Gravity)
M_rightside = [
    F * R + m_rod * g * r * cos(theta) - m_w * g * R * sin(theta) - m_rod * r * (2 * u2 * u1 + R * u1^2);
    F + m_rod * g * sin(theta) + m_rod * r * u1^2
];

if any(~isfinite(M_matrix(:)))
    error('M_matrix contains NaN or Inf');
end
if rcond(M_matrix) < 1e-12
    error('LatModelAppell:SingularMatrix', 'M_matrix singular');
end

% Solve for quasi-accelerations [u1_dot; u2_dot]
accel_u = M_matrix \ M_rightside;
u1_dot = accel_u(1);
u2_dot = accel_u(2);

% Convert quasi-accelerations back to standard state accelerations
% Since u1 = theta_dot           => u1_dot = theta_ddot
% Since u2 = r_dot - R*theta_dot => u2_dot = r_ddot - R*theta_ddot
theta_ddot = u1_dot;
r_ddot     = u2_dot + R * theta_ddot;

% State derivative
dz = [
    theta_dot;
    theta_ddot;
    r_dot;
    r_ddot
];
end