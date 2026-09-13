% --- Parámetros proporcionados ---
par.mass_at_end = 0.24;
par.m_L = 0.15 + par.mass_at_end;
par.m_B = 4.1;
par.m_W = 3.5;
par.h = 0.115;
par.R = 0.2527;
par.g = 9.81;
par.I_b = 0.012904182130007;
par.I_w = 0.0459142776779452;
par.I_rod = 1/12 * (0.3) * (0.314^2) + 2 * par.mass_at_end * 0.157^2;

% --- Parámetros ADRC ---
wo = 1;           % Ajustado para mayor velocidad de respuesta
wc = 15;           % Ajustado según estabilidad esperada
b0 = 1.0;          % Estimación inicial de ganancia (a escalar según tu actuador)
T = 0.001;         % Paso de integración

% Ganancias
beta1 = 3*wo; beta2 = 3*wo^2; beta3 = wo^3;
Kp = wc^2; Kd = 2*wc;

% Inicialización
z1 = 0.17; z2 = 0; z3 = 0; 
theta = 0.1; theta_dot = 0; r = 0; r_dot = 0; 
t = 0:T:2;
th_hist = zeros(1, length(t));

for i = 1:length(t)
    % 1. Control ADRC
    u0 = -Kp*z1 - Kd*z2; 
    u = (u0 - z3) / b0; 
    
    % 2. Dinámica (Tu ecuación matricial reducida a ddot_theta)
    % Inercia efectiva (denominador de la fila 1 de la matriz inversa)
    J_eff = (2*par.m_L*par.R^2 + par.m_B*(par.R+par.h)^2 + par.m_W*par.R^2 + ...
             par.I_b + par.I_w + par.I_rod + 2*par.m_L*r^2);
    
    % Fuerza de perturbación (numerador de la fila 1)
    F_pert = (2*par.m_L*par.g*par.R*sin(theta) + 2*par.m_L*par.g*r*cos(theta) + ...
              par.m_B*par.g*(par.R+par.h)*sin(theta) + par.m_W*par.g*par.R*sin(theta) - ...
              4*par.m_L*r*r_dot*theta_dot + u);
    
    ddot_theta = F_pert / J_eff;
    
    % Actualización de estados físicos
    theta = theta + theta_dot*T;
    theta_dot = theta_dot + ddot_theta*T;
    
    % 3. ESO (Observador)
    e = z1 - theta;
    z1 = z1 + (z2 - beta1*e)*T;
    z2 = z2 + (z3 - beta2*e + b0*u)*T;
    z3 = z3 + (-beta3*e)*T;
    
    th_hist(i) = theta;
end

plot(t, th_hist, 'LineWidth', 1.5); grid on;
title('Respuesta de \theta con ADRC (Parámetros Reales)');
xlabel('Tiempo (s)'); ylabel('\theta (rad)');