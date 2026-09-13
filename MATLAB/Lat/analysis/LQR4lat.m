clear; clc;

addpath("param/", "model/");
par = LatParam();
syms theta theta_dot r r_dot F_sym real
syms m_L m_B m_W h g R real
syms Friction I_b I_w I_rod real
z_sym = [theta; theta_dot; r; r_dot];

M_matrix = [2*m_L *  R^2 + m_B*(R+h)^2 + m_W*R^2 + 2*m_L*r^2 + par.I_b + par.I_w + par.I_rod,   -2*m_L*R;
            -2*m_L*R,                               2*m_L];
M_rightside = [
    2*m_L*g* R*sin(theta) - 2*m_L*g*r*cos(theta) + m_B* g*(R+h)*sin(theta) + m_W*g*R*sin(theta)  - 4  * m_L*r*r_dot* theta_dot;
    - 2*m_L*g*sin(theta) + F_sym + 2*m_L * r * theta_dot ^2 
];
accel = M_matrix \ M_rightside;
dz_sym = [
    theta_dot;
    accel(1);
    r_dot;
    accel(2)
];
A_sym = jacobian(dz_sym, z_sym);
B_sym = jacobian(dz_sym, F_sym);

A_eq = subs(A_sym, [z_sym; F_sym], [0; 0; 0; 0; 0]);
B_eq = subs(B_sym, [z_sym; F_sym], [0; 0; 0; 0; 0]);

% par.m_L = 5;
% par.m_B = 10;
% par.m_W = 4;
% par.h= 0.3;
% par.R = 0.3;

A_num = double(subs(A_eq, [m_L, m_B, m_W, h, g, R, I_b, I_w, I_rod], [par.m_L, par.m_B, par.m_W, par.h, par.g, par.R, par.I_b, par.I_w, par.I_rod]));
B_num = double(subs(B_eq, [m_L, m_B, m_W, h, g, R, I_b, I_w, I_rod], [par.m_L, par.m_B, par.m_W, par.h, par.g, par.R, par.I_b, par.I_w, par.I_rod]));

%% 3. Design LQR

Q = diag([1000, 100, 100, 10]); 
R_weight = 30; 
K = lqr(A_num, B_num, Q, R_weight);
disp('LQR Gain K:');
disp(K);

lqr_controller = @(t, z, par) - K * z; 
z0 = [0.95* pi/180; 0; 0; 0];
tspan = [0, 15];
[t_out, z_out] = ode45(@(t, z) LatModelAppell(t, z, par, lqr_controller), tspan, z0);

% Recalculate for F data
F_out = zeros(length(t_out), 1);
for i = 1:length(t_out)
    F_out(i) = lqr_controller(t_out(i), z_out(i,:)', par);
end

%% 5. Plot Results
figure;
subplot(3,1,1);
plot(t_out, z_out(:,1), 'LineWidth', 1.5);
ylabel('Theta (rad)');
title('LQR Controlled System');
grid on;

subplot(3,1,2);
plot(t_out, z_out(:,3), 'LineWidth', 1.5);
ylabel('Position r (m)');
grid on;

subplot(3,1,3);
plot(t_out, F_out, 'r', 'LineWidth', 1.5); 
xlabel('Time (s)');
ylabel('Force F (N)');
grid on;


