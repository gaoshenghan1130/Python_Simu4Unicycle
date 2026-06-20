clear; clc;
syms x1 x2 x3 x4 F real
syms m_w m_rod R g real

% x1 = theta, x2 = r - R*theta
r_phys = x2 + R * x1; 

I_term = m_w * R^2 + m_rod * r_phys^2;

x1_dot = x3;
x2_dot = x4;

N_term = - m_rod*g*r_phys*cos(x1) + m_w*g*R*sin(x1) - m_rod*r_phys*(2*x4*x3 + R*x3^2);

x3_dot = (N_term + F*R) / I_term;

x4_dot = (m_rod*g*sin(x1) + m_rod*r_phys*x3^2 + F) / m_rod;

x_dot = [x1_dot; x2_dot; x3_dot; x4_dot];
x = [x1; x2; x3; x4];

A_c = jacobian(x_dot, x);
B_c = jacobian(x_dot, F);

A_c_simplified = simplify(A_c);
B_c_simplified = simplify(B_c);

clc;
fprintf('==================== [LaTeX Code] A_c Matrix ====================\n\n');
latex_Ac = latex(A_c_simplified);
latex_Ac = strrep(latex_Ac, '\begin{array}', '\begin{bmatrix}');
latex_Ac = strrep(latex_Ac, '\end{array}', '\end{bmatrix}');
latex_Ac = strrep(latex_Ac, '\begin{pmatrix}', '\begin{bmatrix}');
latex_Ac = strrep(latex_Ac, '\end{pmatrix}', '\end{bmatrix}');
disp(latex_Ac);

fprintf('\n==================== [LaTeX Code] B_c Matrix ====================\n\n');
latex_Bc = latex(B_c_simplified);
latex_Bc = strrep(latex_Bc, '\begin{array}', '\begin{bmatrix}');
latex_Bc = strrep(latex_Bc, '\end{array}', '\end{bmatrix}');
latex_Bc = strrep(latex_Bc, '\begin{pmatrix}', '\begin{bmatrix}');
latex_Bc = strrep(latex_Bc, '\end{pmatrix}', '\end{bmatrix}');
disp(latex_Bc);

fprintf('==================== [MATLAB Code] A_c Matrix ====================\n\n');
disp('A_c = [ ...');
for i = 1:size(A_c_simplified, 1)
    row_str = '';
    for j = 1:size(A_c_simplified, 2)
        row_str = [row_str, char(A_c_simplified(i,j))];
        if j < size(A_c_simplified, 2)
            row_str = [row_str, ', '];
        end
    end
    if i < size(A_c_simplified, 1)
        fprintf('    %s; ...\n', row_str);
    else
        fprintf('    %s ...\n];\n', row_str);
    end
end