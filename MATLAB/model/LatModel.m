function dz = LatModel(t, z, par, controller)
% Latitude Nonlinear Model
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

% States
theta     = z(1);
theta_dot = z(2);
r         = z(3);
r_dot     = z(4);

% Parameters
m_L = par.m_L;
m_B = par.m_B;
m_W = par.m_W;
h   = par.h;
g   = par.g;
R   = par.R;
I_b = par.I_b;
I_w = par.I_w;
I_rod = par.I_rod;

M_matrix = [2*m_L + R^2 + m_B*(R+h)^2 + m_W*R^2 + I_b + I_w + I_rod + 2*m_L*r^2,   2*m_L*R;
            2*m_L*R,                               2*m_L];
M_rightside = [
    2*m_L*R*sin(theta) + 2*m_L*g*r*cos(theta) + m_B*(R+h)*sin(theta) + m_W*g*R*sin(theta) - F*R - 4 * m_L*r*r_dot;
    2*m_L*g*sin(theta) + F + 2*m_L * r * theta_dot ^2 
];

% Accelerations
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