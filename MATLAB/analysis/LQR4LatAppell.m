clear; clc;

addpath("param/", "model/");
par = LatParam();

%% Symbolic Linearization of Appell Model

syms theta theta_dot r r_dot F_sym real
syms m_w m_rod g R real

z_sym = [theta; theta_dot; r; r_dot];

%% Appell quasi velocities
u1 = theta_dot;
u2 = r_dot - R*theta_dot;

%% Appell dynamics

M_matrix = [
    m_w*R^2 + m_rod*r^2, 0;
    0,                   m_rod
];

M_rightside = [
    F_sym*R ...
    - m_rod*g*r*cos(theta) ...
    + m_w*g*R*sin(theta) ...
    - m_rod*r*(2*u2*u1 + R*u1^2);

    F_sym ...
    - m_rod*g*sin(theta) ...
    + m_rod*r*u1^2
];

accel_u = simplify(M_matrix \ M_rightside);

u1_dot = accel_u(1);
u2_dot = accel_u(2);

theta_ddot = u1_dot;
r_ddot     = u2_dot + R*u1_dot;

dz_sym = [
    theta_dot;
    theta_ddot;
    r_dot;
    r_ddot
];

%% Linearization

A_sym = jacobian(dz_sym, z_sym);
B_sym = jacobian(dz_sym, F_sym);

A_eq = subs(A_sym,...
    [theta theta_dot r r_dot F_sym],...
    [0 0 0 0 0]);

B_eq = subs(B_sym,...
    [theta theta_dot r r_dot F_sym],...
    [0 0 0 0 0]);

%% Parameter substitution

m_w_num   = (par.m_W*par.R^2 + par.m_B*(par.R + par.h)^2)/par.R^2;
m_rod_num = 2*par.m_L;

A_num = double(subs(A_eq,...
    [m_w m_rod g R],...
    [m_w_num m_rod_num par.g par.R]));

B_num = double(subs(B_eq,...
    [m_w m_rod g R],...
    [m_w_num m_rod_num par.g par.R]));


%% Controllability

Co = ctrb(A_num,B_num);

fprintf('rank(C) = %d\n',rank(Co));

%% LQR

Q = diag([
    1000 ... theta
    500  ... theta_dot
    100  ... r
    50  ... r_dot
]);

R_weight = 1;

K = lqr(A_num,B_num,Q,R_weight);

disp('LQR Gain K:');
disp(K);

%% Nonlinear Simulation

lqr_controller = @(t,z,par) -K*z;

z0 = [
    3*pi/180;
    0;
    0;
    0
];

tspan = [0 5];

[t_out,z_out] = ode45( ...
    @(t,z) LatModelAppell(t,z,par,lqr_controller), ...
    tspan,...
    z0);

%% Force history

F_out = zeros(length(t_out),1);

for k = 1:length(t_out)
    F_out(k) = lqr_controller( ...
        t_out(k), ...
        z_out(k,:)', ...
        par);
end

%% Plot

figure;

subplot(3,1,1)
plot(t_out,z_out(:,1),'LineWidth',1.5)
ylabel('\theta (rad)')
title('Appell LQR')
grid on

subplot(3,1,2)
plot(t_out,z_out(:,3),'LineWidth',1.5)
ylabel('r (m)')
grid on

subplot(3,1,3)
plot(t_out,F_out,'LineWidth',1.5)
ylabel('F (N)')
xlabel('Time (s)')
grid on