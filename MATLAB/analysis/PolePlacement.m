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
G_theta2 = 2 * m_L * g;
G_r1  = 2*m_L*g;


N = [1, 0, 0, 0;
     0, 1, 0,       0;
     0, 0, J_theta, 2*m_L*R^2; 
     0, 0, 2*m_L*R^2, 2*m_L];

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

char_poly_clean = vpa(expand(char_poly), 4)
