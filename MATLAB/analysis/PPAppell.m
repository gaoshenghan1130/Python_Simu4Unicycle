clear;
clc;

addpath("param/", "model/");
par = LatParam();

%% =========================================================
% Parameters
%% =========================================================

m_L = par.m_L;
m_B = par.m_B;
m_W = par.m_W;

h = par.h;
R = par.R;
g = par.g;

% In the existing physical model, there are two moving masses.
% Therefore, m_rod = 2*m_L.
% If par.m_L already represents the total rod mass, change this to:
% m = par.m_L;
m = 2 * m_L;

% Simplified model:
% J does not include m*r^2.
J = m_W * R^2 ...
    + m_B * (R + h)^2 ...
    + par.I_w ...
    + par.I_b ...
    + par.I_rod;

G = m_W * g * R ...
    + m_B * g * (R + h);

%% =========================================================
% State-space matrix
%
% x = [theta; r; theta_dot; r_dot]
%% =========================================================

A = [0,       0,         1, 0;
     0,       0,         0, 1;
     G/J,    -m*g/J,      0, 0;
    -g,       0,          0, 0];

B = [0;
     0;
     R/J;
     1/m];


%% =========================================================
% Symbolic characteristic polynomial
%% =========================================================

syms lambda J_s G_s m_s g_s real

A_sym = [0,           0,              1, 0;
         0,           0,              0, 1;
         G_s/J_s,    -m_s*g_s/J_s,    0, 0;
        -g_s,         0,               0, 0];

char_poly_sym = collect(...
    expand(det(lambda * eye(4) - A_sym)), lambda);

fprintf('======== Symbolic Characteristic Polynomial ========\n');
pretty(char_poly_sym);

fprintf('Expected characteristic polynomial:\n');
fprintf('lambda^4 - (G/J) lambda^2 - m g^2/J\n\n');

%% =========================================================
% Numerical characteristic polynomial
%% =========================================================

char_coefficients = poly(A);

expected_coefficients = [ ...
    1, ...
    0, ...
    -G/J, ...
    0, ...
    -m*g^2/J];

fprintf('======== Numerical Polynomial Coefficients ========\n');
fprintf('MATLAB poly(A):\n');
disp(char_coefficients);

fprintf('Expected coefficients:\n');
disp(expected_coefficients);

poly_error = norm(char_coefficients - expected_coefficients);

fprintf('Coefficient error = %.6e\n\n', poly_error);

%% =========================================================
% Numerical eigenvalues and eigenvectors
%% =========================================================

[V, D] = eig(A);
eigenvalues = diag(D);

% Sort by real part first, then imaginary part
[~, order] = sortrows([real(eigenvalues), imag(eigenvalues)], [1, 2]);

eigenvalues = eigenvalues(order);
V = V(:, order);

fprintf('================ Eigenvalues ================\n');
disp(eigenvalues);

%% =========================================================
% Normalize each eigenvector so that v_theta = 1
%% =========================================================

V_normalized = zeros(size(V));

for i = 1:length(eigenvalues)

    if abs(V(1, i)) < 1e-12
        error('The theta component of eigenvector %d is too small.', i);
    end

    V_normalized(:, i) = V(:, i) / V(1, i);
end

fprintf('===== Numerical Eigenvectors: v_theta = 1 =====\n');
fprintf('Each column corresponds to one eigenvalue.\n');
fprintf('State order: [theta; r; theta_dot; r_dot]\n\n');

disp(V_normalized);

%% =========================================================
% Compare with analytical eigenvector
%
% v(lambda) = [1; -g/lambda^2; lambda; -g/lambda]
%% =========================================================

fprintf('=============== Mode-by-Mode Check ===============\n');

for i = 1:length(eigenvalues)

    lambda_i = eigenvalues(i);
    v_numeric = V_normalized(:, i);

    v_analytic = [1;
                 -g / lambda_i^2;
                  lambda_i;
                 -g / lambda_i];

    % Eigenvalue equation residual
    eigen_residual = norm(A * v_numeric - lambda_i * v_numeric);

    % Difference between MATLAB and analytical eigenvectors
    vector_error = norm(v_numeric - v_analytic);

    % Characteristic polynomial evaluated at lambda_i
    polynomial_residual = abs( ...
        lambda_i^4 ...
        - (G/J) * lambda_i^2 ...
        - m*g^2/J);

    fprintf('\n---------------------------------------------\n');
    fprintf('Mode %d\n', i);
    fprintf('lambda = %.10e %+.10ei\n', ...
        real(lambda_i), imag(lambda_i));

    fprintf('\nNumerical eigenvector:\n');
    printComplexVector(v_numeric);

    fprintf('Analytical eigenvector:\n');
    printComplexVector(v_analytic);

    fprintf('||A*v - lambda*v||       = %.6e\n', ...
        eigen_residual);

    fprintf('||v_numeric-v_analytic|| = %.6e\n', ...
        vector_error);

    fprintf('|p(lambda)|              = %.6e\n', ...
        polynomial_residual);

    % Sign check only applies to real eigenvalues
    if abs(imag(lambda_i)) < 1e-10

        theta_dot = real(v_numeric(3));
        r_dot = real(v_numeric(4));

        fprintf('\nReal-mode velocity signs:\n');
        fprintf('theta_dot = %+.8f\n', theta_dot);
        fprintf('r_dot     = %+.8f\n', r_dot);

        if theta_dot > 0 && r_dot < 0
            fprintf(['Result: theta_dot > 0 and r_dot < 0. ', ...
                     'Desired direction found.\n']);

        elseif theta_dot < 0 && r_dot > 0
            fprintf(['Result: theta_dot < 0 and r_dot > 0. ', ...
                     'Opposite orientation of the same mode.\n']);

        elseif theta_dot * r_dot > 0
            fprintf('Result: theta_dot and r_dot have the same sign.\n');

        else
            fprintf('Result: at least one velocity component is zero.\n');
        end
    else
        fprintf('\nComplex oscillatory mode: ordinary sign comparison ');
        fprintf('is not applicable.\n');
    end
end

%% =========================================================
% Analytical eigenvalues
%% =========================================================

mu_1 = (G + sqrt(G^2 + 4*J*m*g^2)) / (2*J);
mu_2 = (G - sqrt(G^2 + 4*J*m*g^2)) / (2*J);

lambda_analytical = [ ...
     sqrt(complex(mu_1));
    -sqrt(complex(mu_1));
     sqrt(complex(mu_2));
    -sqrt(complex(mu_2))];

[~, analytical_order] = sortrows( ...
    [real(lambda_analytical), imag(lambda_analytical)], [1, 2]);

lambda_analytical = lambda_analytical(analytical_order);

fprintf('\n=========== Analytical Eigenvalues ===========\n');
disp(lambda_analytical);

fprintf('=========== Numerical Eigenvalues ============\n');
disp(eigenvalues);

fprintf('Eigenvalue comparison error = %.6e\n', ...
    norm(eigenvalues - lambda_analytical));

%% =========================================================
% Local function
%% =========================================================

function printComplexVector(v)

    labels = {'theta', 'r', 'theta_dot', 'r_dot'};

    for k = 1:length(v)
        fprintf('%-10s = %.10e %+.10ei\n', ...
            labels{k}, real(v(k)), imag(v(k)));
    end
end