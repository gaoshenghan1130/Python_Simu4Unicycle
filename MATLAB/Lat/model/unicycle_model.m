function dX = unicycle_model(t, X, par, controller)
%UNICYCLE_MODEL Dynamics converted from the Python Kane model.
%
% State order:
% X = [psi theta phi r gamma dpsi dtheta dphi dr dgamma]'
%
% Control order returned by controller:
% u = [T_W; F_L]

if nargin < 4 || isempty(controller)
    controller = @unicycle_controller;
end

X = X(:);
if numel(X) ~= 10
    error('unicycle_model:BadStateSize', ...
        'State X must have 10 elements: [psi theta phi r gamma dpsi dtheta dphi dr dgamma].');
end
if any(~isfinite(X))
    error('unicycle_model:NonFiniteState', ...
        'State became non-finite at t = %.6g s.', t);
end

u = controller(t, X, par);
u = u(:);
if numel(u) ~= 2
    error('unicycle_model:BadControlSize', ...
        'Controller must return 2 elements: [T_W; F_L].');
end
if any(~isfinite(u))
    error('unicycle_model:NonFiniteControl', ...
        'Control became non-finite at t = %.6g s.', t);
end

%% States
psi = X(1); %#ok<NASGU>
theta = X(2);
phi = X(3); %#ok<NASGU>
r = X(4);
gamma = X(5);
dpsi = X(6);
dtheta = X(7);
dphi = X(8);
dr = X(9);
dgamma = X(10);

%% Parameters
R = par.R;
h = par.h;
m = par.m;
m_w = par.m_w;
m_l = par.m_l;
g = par.g;

%% Controls
T_W = u(1);
F_L = u(2);

%% Mass matrix and forcing vector
M = [1, 0, 0, 0, 0, 0, 0, 0, 0, 0; ...
     0, 1, 0, 0, 0, 0, 0, 0, 0, 0; ...
     0, 0, 1, 0, 0, 0, 0, 0, 0, 0; ...
     0, 0, 0, 1, 0, 0, 0, 0, 0, 0; ...
     0, 0, 0, 0, 1, 0, 0, 0, 0, 0; ...
     0, 0, 0, 0, 0, R^2*m_w*sin(theta)^2 + m*(R^2*sin(theta)^2 + 2*R*h*sin(theta)^2*cos(gamma) + h^2*sin(gamma)^2*cos(theta)^2 + h^2*sin(theta)^2) + m_l*(R*sin(theta) - r*cos(theta))^2, m*(-R*h*sin(gamma)*cos(theta) - h^2*sin(gamma)*cos(gamma)*cos(theta)), R^2*m_w*sin(theta) + R*m_l*(R*sin(theta) - r*cos(theta)) + m*(R^2*sin(theta) + R*h*sin(theta)*cos(gamma)), 0, m*(R*h*sin(theta)*cos(gamma) + h^2*sin(theta)); ...
     0, 0, 0, 0, 0, m*(-R*h*sin(gamma)*cos(theta) - h^2*sin(gamma)*cos(gamma)*cos(theta)), R^2*m_w + m*(R^2 + 2*R*h*cos(gamma) + h^2*cos(gamma)^2) + m_l*(R^2 + r^2), 0, -R*m_l, 0; ...
     0, 0, 0, 0, 0, R^2*m_w*sin(theta) + R*m_l*(R*sin(theta) - r*cos(theta)) + m*(R^2*sin(theta) + R*h*sin(theta)*cos(gamma)), 0, R^2*m + R^2*m_l + R^2*m_w, 0, R*h*m*cos(gamma); ...
     0, 0, 0, 0, 0, 0, -R*m_l, 0, m_l, 0; ...
     0, 0, 0, 0, 0, m*(R*h*sin(theta)*cos(gamma) + h^2*sin(theta)), 0, R*h*m*cos(gamma), 0, h^2*m];

f = [dpsi; ...
     dtheta; ...
     dphi; ...
     dr; ...
     dgamma; ...
     -2*R^2*dpsi*dtheta*m*sin(theta)*cos(theta) - 2*R^2*dpsi*dtheta*m_w*sin(theta)*cos(theta) - 2*R*dpsi*dtheta*h*m*sin(theta)*cos(gamma)*cos(theta) - R*dpsi*h*m*(dphi + dpsi*sin(theta))*sin(gamma)*cos(theta)^2 - R*m*(-h*(dgamma + dpsi*sin(theta))^2 - h*(-dpsi*sin(gamma)*cos(theta) + dtheta*cos(gamma))^2)*sin(gamma)*sin(theta) - R*m*(dpsi*dtheta*h*cos(theta) + h*(-dpsi*sin(gamma)*cos(theta) + dtheta*cos(gamma))*(dpsi*cos(gamma)*cos(theta) + dtheta*sin(gamma)))*sin(theta)*cos(gamma) - h*m*(h*(dgamma + dpsi*sin(theta))*(dpsi*cos(gamma)*cos(theta) + dtheta*sin(gamma)) - h*(-dgamma*dpsi*cos(gamma)*cos(theta) - dgamma*dtheta*sin(gamma) + dpsi*dtheta*sin(gamma)*sin(theta)))*sin(gamma)*cos(theta) + h*m*(-R*dpsi*(dphi + dpsi*sin(theta))*sin(theta) - R*dtheta^2)*sin(gamma)*sin(theta) - h*m*(dpsi*dtheta*h*cos(theta) + h*(-dpsi*sin(gamma)*cos(theta) + dtheta*cos(gamma))*(dpsi*cos(gamma)*cos(theta) + dtheta*sin(gamma)))*sin(theta) - m_l*(R*sin(theta) - r*cos(theta))*(R*dpsi*dtheta*cos(theta) - dpsi*dr*cos(theta) + 2*dpsi*dtheta*r*sin(theta) - dpsi*(-R*dtheta + dr)*cos(theta)); ...
     R^2*dpsi*m*(dphi + dpsi*sin(theta))*cos(theta) + R^2*dpsi*m_w*(dphi + dpsi*sin(theta))*cos(theta) + R*dpsi*h*m*(dphi + dpsi*sin(theta))*cos(gamma)*cos(theta) + R*g*m*sin(theta) + R*g*m_l*sin(theta) + R*g*m_w*sin(theta) + R*m*(h*(dgamma + dpsi*sin(theta))*(dpsi*cos(gamma)*cos(theta) + dtheta*sin(gamma)) - h*(-dgamma*dpsi*cos(gamma)*cos(theta) - dgamma*dtheta*sin(gamma) + dpsi*dtheta*sin(gamma)*sin(theta))) + R*m_l*(dpsi*(R*(dphi + dpsi*sin(theta)) - dpsi*r*cos(theta))*cos(theta) - dtheta^2*r) + g*h*m*sin(theta)*cos(gamma) - g*m_l*r*cos(theta) + h*m*(h*(dgamma + dpsi*sin(theta))*(dpsi*cos(gamma)*cos(theta) + dtheta*sin(gamma)) - h*(-dgamma*dpsi*cos(gamma)*cos(theta) - dgamma*dtheta*sin(gamma) + dpsi*dtheta*sin(gamma)*sin(theta)))*cos(gamma) - m_l*r*(-dpsi*(R*(dphi + dpsi*sin(theta)) - dpsi*r*cos(theta))*sin(theta) + dr*dtheta + dtheta*(-R*dtheta + dr)); ...
     -2*R^2*dpsi*dtheta*m*cos(theta) - 2*R^2*dpsi*dtheta*m_w*cos(theta) - R*m*(-h*(dgamma + dpsi*sin(theta))^2 - h*(-dpsi*sin(gamma)*cos(theta) + dtheta*cos(gamma))^2)*sin(gamma) - R*m*(dpsi*dtheta*h*cos(theta) + h*(-dpsi*sin(gamma)*cos(theta) + dtheta*cos(gamma))*(dpsi*cos(gamma)*cos(theta) + dtheta*sin(gamma)))*cos(gamma) - R*m_l*(R*dpsi*dtheta*cos(theta) - dpsi*dr*cos(theta) + 2*dpsi*dtheta*r*sin(theta) - dpsi*(-R*dtheta + dr)*cos(theta)) + T_W; ...
     -F_L - g*m_l*sin(theta) - m_l*(dpsi*(R*(dphi + dpsi*sin(theta)) - dpsi*r*cos(theta))*cos(theta) - dtheta^2*r); ...
     -2*R*dpsi*dtheta*h*m*cos(gamma)*cos(theta) + g*h*m*sin(gamma)*cos(theta) + h*m*(-R*dpsi*(dphi + dpsi*sin(theta))*sin(theta) - R*dtheta^2)*sin(gamma) - h*m*(dpsi*dtheta*h*cos(theta) + h*(-dpsi*sin(gamma)*cos(theta) + dtheta*cos(gamma))*(dpsi*cos(gamma)*cos(theta) + dtheta*sin(gamma)))];

if any(~isfinite(M(:))) || any(~isfinite(f))
    error('unicycle_model:NonFiniteDynamics', ...
        'Dynamics became non-finite at t = %.6g s.', t);
end

matrix_rcond = rcond(M);
if ~isfinite(matrix_rcond) || matrix_rcond < 1e-12
    error('unicycle_model:SingularMassMatrix', ...
        'Mass matrix is singular or ill-conditioned at t = %.6g s. rcond(M) = %.3e.', ...
        t, matrix_rcond);
end

dX = M\f;
end
