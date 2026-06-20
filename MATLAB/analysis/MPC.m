clear; clc;

syms x1 x2 x3 x4 F dt real
syms m_w m_rod R g real

x = [x1; x2; x3; x4];

I_x2 = m_w * R^2 + m_rod * x2^2;

f1 = x3;
f2 = x4 + R * x3;
f3 = (m_rod*g*x2*cos(x1) - m_w*g*R*sin(x1) - m_rod*x2*(2*x4*x3 + R*x3^2)) / I_x2;
f4 = g*sin(x1) + x2*x3^2;

f_x = [f1; f2; f3; f4];
g_x = [0; 0; R/I_x2; 1/m_rod];

x_dot = f_x + g_x * F;

A_c = jacobian(x_dot, x);
B_c = jacobian(x_dot, F);

A_c_simplified = simplify(A_c);
B_c_simplified = simplify(B_c);

clc;
fprintf('==================== [LaTeX Code] A_c Matrix ====================\n\n');
latex_Ac = latex(A_c_simplified);
latex_Ac = strrep(latex_Ac, '\begin{array}', '\begin{bmatrix}');
latex_Ac = strrep(latex_Ac, '\end{array}', '\end{bmatrix}');
disp(latex_Ac);

fprintf('\n==================== [LaTeX Code] B_c Matrix ====================\n\n');
latex_Bc = latex(B_c_simplified);
latex_Bc = strrep(latex_Bc, '\begin{array}', '\begin{bmatrix}');
latex_Bc = strrep(latex_Bc, '\end{array}', '\end{bmatrix}');
disp(latex_Bc);