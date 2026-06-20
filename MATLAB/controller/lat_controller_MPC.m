function M = lat_controller_MPC(t, z, par)
%% Initialize params
% z = [theta, theta_dot, r, r_dot]
x1 = z(1); % theta
x2 = z(3) - par.R * z(1); % r - R * theta
x3 = z(2); % theta_dot
x4 = z(4) - par.R * z(2); % r_dot - R * theta_dot
x_k = [x1; x2; x3; x4]; 

theta_ref = 0;
r_ref = 0;
theta_dot_ref = 0;
r_dot_ref = 0;
x_ref = [theta_ref; r_ref - par.R * theta_ref; theta_dot_ref; r_dot_ref - par.R * theta_dot_ref];

m_w   = (par.m_W * par.R + par.m_B * (par.R+par.h))/par.R;
m_rod = 2 * par.m_L;
R     = par.R;
g     = par.g;
dt    = 0.02; % 100Hz
N     = 20;   % 

persistent F_prev 
if isempty(F_prev)
    F_prev = 0; 
end
I_x2 = m_w*R^2 + m_rod*(x2 + R * x1)^2;


F = F_prev;
%% Calculate A_c, B_C
A_c = [ ...
    0, 0, 1, 0; ...
    0, 0, 0, 1; ...
    - (R*m_rod*(2*x3*x4 + R*x3^2) - g*m_rod*sin(x1)*(x2 + R*x1) + R*g*m_rod*cos(x1) - R*g*m_w*cos(x1))/(m_rod*(x2 + R*x1)^2 + R^2*m_w) - (2*R*m_rod*(x2 + R*x1)*(F*R - m_rod*(x2 + R*x1)*(2*x3*x4 + R*x3^2) - g*m_rod*cos(x1)*(x2 + R*x1) + R*g*m_w*sin(x1)))/(m_rod*(x2 + R*x1)^2 + R^2*m_w)^2, - (m_rod*(2*x3*x4 + R*x3^2) + g*m_rod*cos(x1))/(m_rod*(x2 + R*x1)^2 + R^2*m_w) - (m_rod*(2*x2 + 2*R*x1)*(F*R - m_rod*(x2 + R*x1)*(2*x3*x4 + R*x3^2) - g*m_rod*cos(x1)*(x2 + R*x1) + R*g*m_w*sin(x1)))/(m_rod*(x2 + R*x1)^2 + R^2*m_w)^2, -(m_rod*(x2 + R*x1)*(2*x4 + 2*R*x3))/(m_rod*(x2 + R*x1)^2 + R^2*m_w), -(2*m_rod*x3*(x2 + R*x1))/(m_rod*(x2 + R*x1)^2 + R^2*m_w); ...
    R*x3^2 + g*cos(x1), x3^2, 2*x3*(x2 + R*x1), 0 ...
];
B_c = [0; 0; R/I_x2; 1/m_rod];

%% Get 0 order compensation term d_c
N_val = - m_rod*g*(x2 + R*x1)*cos(x1) + m_w*g*R*sin(x1) - m_rod*(x2 + R * x1)*(2*x4*x3 + R*x3^2);
x_dot_real = [ ...
    x3; ...
    x4; ...
    N_val/I_x2 + (R/I_x2)*F_prev; ...
    g*sin(x1) + (x2+ R * x1)*x3^2 + (1/m_rod)*F_prev ...
];
d_c = x_dot_real - A_c*x_k - B_c*F_prev;

%% Assemble
A = eye(4) + A_c * dt;
B = B_c * dt;
d = d_c * dt;
M_mpc = zeros(4*N, 4);
C_mpc = zeros(4*N, N);
D_mpc = zeros(4*N, 1);
A_power = eye(4);
d_sum = zeros(4,1);
for i = 1:N
    d_sum = A*d_sum + d;
    A_power = A_power * A;
    
    M_mpc((i-1)*4+1 : i*4, :) = A_power;
    D_mpc((i-1)*4+1 : i*4, :) = d_sum;
    
    for j = 1:i
        C_mpc((i-1)*4+1 : i*4, j) = (A^(i-j)) * B;
    end
end

%% Solve QP
R_u = 100; 

% [theta, r, theta_dot, r_dot]
Q_diagonal   = diag([800, 900, 800, 500]); 
Q_f_diagonal = diag([1500, 1600, 1000, 1000]);

Q_bar = zeros(4*N, 4*N);
R_bar = eye(N) * R_u;
X_ref_vector = zeros(4*N, 1);
for i = 1:N-1
    Q_bar((i-1)*4+1 : i*4, (i-1)*4+1 : i*4) = Q_diagonal; 
    X_ref_vector((i-1)*4+1 : i*4, 1) = x_ref;
end
Q_bar((N-1)*4+1 : N*4, (N-1)*4+1 : N*4) = Q_f_diagonal;
X_ref_vector((N-1)*4+1 : N*4, 1) = x_ref;

H = 2 * (C_mpc' * Q_bar * C_mpc + R_bar);
G = 2 * C_mpc' * Q_bar * (M_mpc * x_k + D_mpc - X_ref_vector);
H = (H + H') / 2; 

lb = ones(N, 1) * -27.6; 
ub = ones(N, 1) * 27.6; 

options = optimoptions('quadprog', 'Display', 'off');
[U_optimal, ~, exitflag] = quadprog(H, G, [], [], [], [], lb, ub, [], options);
if exitflag >= 0 && ~isempty(U_optimal)
    M = U_optimal(1);
    disp(M);
else
    M = F_prev; 
    
end
F_prev = M;
end