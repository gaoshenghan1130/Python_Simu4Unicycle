%% Pole placement at the stationary upright equilibrium
% Run with the supplied model functions in this folder.
% Requires Control System Toolbox (place).
% Feedback convention: u = -K*(x-x_eq), where u = [F; M2].
% This equilibrium has eight controllable states and four uncontrollable
% zero poles. Only the two independent 4-state blocks are assigned poles.
% No simulation is performed. The script prints gains only.
clear; clc;
addpath(fileparts(mfilename('fullpath')));

%% Parameters (SI units; same values as unicycle_model.md)
p.g   = 9.81;
p.R   = 0.253;
p.mw  = 2.436;
p.JW1 = 0.09099921839;
p.JW2 = 0.04591427768;
p.mr  = 2.3;
p.JR  = 0.0517629;
p.BR  = 0;
p.h   = 0.025;
p.mp  = 2.799;
p.JPx = 0.01290418213;
p.JPy = 0.02090219895;
p.JPz = 0.01118711607;
p.BP  = 0;

%% Desired poles: edit these two lines
% Illustrative choices, not experimentally tuned gains.
poles_lateral      = [-2.25, -1.25, -2.00, -1.50];
poles_longitudinal = [-3, -3.5, -4, -4.5];

%% Linearize the supplied full nonlinear model
% x = [sigma1 sigma2 sigma3 sigma_r sigma_g psi theta phi r gamma xG yG]'
% This decomposition is specific to rest, upright lean/body and centered rod.
x_eq = zeros(12,1);
u_eq = zeros(2,1);
assert(norm(model_rhs(0,x_eq,u_eq,p),inf) < 1e-12, ...
    'The selected point is not an equilibrium.');

% Complex-step differentiation is valid for the supplied smooth model.
% If nonanalytic operations such as abs/sign/clipping are added, replace it.
step = 1e-20;
A = zeros(12,12);
B = zeros(12,2);
for j = 1:12
    x_test = x_eq;
    x_test(j) = x_test(j) + 1i*step;
    A(:,j) = imag(model_rhs(0,x_test,u_eq,p))/step;
end
for j = 1:2
    u_test = u_eq;
    u_test(j) = u_test(j) + 1i*step;
    B(:,j) = imag(model_rhs(0,x_eq,u_test,p))/step;
end

%% Extract the independently controlled blocks
idx_lateral      = [7, 9, 1, 4]; % [theta, r, sigma1, sigma_r]
idx_longitudinal = [8, 10, 2, 5]; % [phi, gamma, sigma2, sigma_g]
A_lateral = A(idx_lateral,idx_lateral);
B_lateral = B(idx_lateral,1);
A_longitudinal = A(idx_longitudinal,idx_longitudinal);
B_longitudinal = B(idx_longitudinal,2);

% Verify the decomposition before using separate pole placement.
% z = [x_lateral; x_longitudinal; sigma3; psi; xG-R*phi; yG+R*theta]
T = eye(12);
T = T([idx_lateral,idx_longitudinal,3,6,11,12],:);
T(11,8) = -p.R;
T(12,7) = p.R;
A_z = T*A/T;
B_z = T*B;
A_expected = blkdiag(A_lateral,A_longitudinal,zeros(4));
A_expected(10,9) = 1; % yaw-rate integrator: psi_dot = sigma3
B_expected = zeros(12,2);
B_expected(1:4,1) = B_lateral;
B_expected(5:8,2) = B_longitudinal;
assert(norm(A_z-A_expected,inf) < 1e-9 && ...
       norm(B_z-B_expected,inf) < 1e-9, ...
       'The assumed stationary-equilibrium block decomposition does not hold.');
assert(rank(ctrb(A_lateral,B_lateral)) == 4, ...
    'Lateral block is not controllable with these parameters.');
assert(rank(ctrb(A_longitudinal,B_longitudinal)) == 4, ...
    'Longitudinal block is not controllable with these parameters.');

%% Compute gains and embed them in the original 12-state ordering
K_lateral = place(A_lateral,B_lateral,poles_lateral);
K_longitudinal = place(A_longitudinal,B_longitudinal,poles_longitudinal);
K = zeros(2,12);
K(1,idx_lateral) = K_lateral;
K(2,idx_longitudinal) = K_longitudinal;

%% Print gains only (the minus sign belongs to the feedback law)
fprintf('F = -K_lateral * [theta; r; sigma1; sigma_r]\n');
fprintf('K_lateral = [%.10g  %.10g  %.10g  %.10g]\n\n',K_lateral);
fprintf('M2 = -K_longitudinal * [phi; gamma; sigma2; sigma_g]\n');
fprintf('K_longitudinal = [%.10g  %.10g  %.10g  %.10g]\n\n',K_longitudinal);
fprintf('[F; M2] = -K*(x-x_eq)\n');
fprintf('K columns: sigma1 sigma2 sigma3 sigma_r sigma_g psi theta phi r gamma xG yG\n');
fprintf('K = [\n');
for i = 1:2
    fprintf(' %.10g',K(i,:));
    fprintf(';\n');
end
fprintf('];\n');
