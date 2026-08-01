clear; clc;
addpath("param/","model/");
par = LatParam();

m_L = par.m_L;
m_B = par.m_B;
m_W = par.m_W;
h = par.h;
R = par.R;
g = par.g;

J_theta = 2 * m_L * R^2 + m_B * (R+h)^2 + m_W * R^2 + par.I_b + par.I_rod + par.I_w;
G_theta1 = 2*m_L * g * R + m_B * g * (R+h) + m_W * g * R; 
G_theta2 = -2 * m_L * g;
G_r1  = -2*m_L*g;


N = [1, 0, 0, 0;
     0, 1, 0,       0;
     0, 0, J_theta, -2*m_L*R; 
     0, 0, -2*m_L*R, 2*m_L];

A_tilde = [0,        0,      1, 0;
           0,        0,      0, 1;
           G_theta1, G_r1,   0, 0;
           G_theta2, 0,      0, 0];
A = N \ A_tilde; % N^-1 * A_tilde
B = N \ [0;0;0;1];

% Get eigenvalue
syms k1 k2 k3 k4 real
K_sym = [k1, k2, k3, k4];


A_cl = A - (B*K_sym); % N\K = N^-1 * K = B * K
lambda = sym('lambda');
char_poly = det(lambda * eye(4) - A_cl);

char_poly_clean = vpa(expand(char_poly), 4);

P_desired = 0.2 * [-2.25, -1.25, -2.00, -1.50]; %Poles obtained from MC_PP_lat_4P.m


K_numeric = place(A, B, P_desired);

disp('PolePlacement K = ');
disp(K_numeric);

A_cl_numeric = A - B * K_numeric;
[eigenvectors, eigenvalue_matrix] = eig(A_cl_numeric);
eigenvalues = diag(eigenvalue_matrix);
disp(eigenvalues)
disp(eigenvectors)

K_numeric_new_order = [K_numeric(1), K_numeric(3), K_numeric(2), K_numeric(4)];

controller = @(t, z, par) -K_numeric_new_order * z; 
z0 = [0.118 * pi/180; 0; 0; 0];
tspan = [0, 15];
[t_out, z_out] = ode45(@(t, z) LatModel_SignCorrection(t, z, par, controller), tspan, z0);

% Recalculate for F data
F_out = zeros(length(t_out), 1);
for i = 1:length(t_out)
    F_out(i) = controller(t_out(i), z_out(i,:)', par);
end

%% 5. Plot Results
figure;
subplot(3,1,1);
plot(t_out, z_out(:,1), 'LineWidth', 1.5);
ylabel('Theta (rad)');
title('Pole placement Controlled System');
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
