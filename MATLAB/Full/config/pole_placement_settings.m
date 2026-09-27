function d = pole_placement_settings()
% Select balance (stationary), balance_chi, or balance_chi_epsilon (rolling).
% Lateral poles: balance=4, balance_chi=5, balance_chi_epsilon=6. Units: 1/s.
% These poles are eigenvalues of the whole block, NOT one pole per state.
% balance feedback: [theta, r, sigma1, sigma_r]
% balance_chi feedback: [theta, r, sigma1, sigma_r, chi=psi]
% Longitudinal feedback order: [phi, gamma, sigma2, sigma_g]
d.lateral      = [-0.8, -0.95, -1.05, -1.2, 0];
d.longitudinal = [-0.8, -0.95, -1.05, -1.2];
% Examples (replace either complete vector):
% d.lateral = [-2+1i, -2-1i, -3, -4]; % conjugate pairs required
% d.lateral = [-2, -2, -2, -2];       % repeated poles: use acker
% The paper's repeated -12 poles refer to a different rolling-state design;
% copying them here does not reproduce that controller.
d.lateral_states='balance_chi'; % 'balance' (rest, 4 poles) or 'balance_chi' (rolling, 5 poles)
d.forward_speed=2; % m/s; nonzero design/reference speed for balance_chi
d.chi_poles=[-0.8 -0.9 -1.0 -1.1 -1.2]; % replaces d.lateral in balance_chi
% Four-state rolling feedback: [theta r sigma1 sigma_r], nonzero design speed.
d.rolling_poles=[-2.2 -2.25 -1.1 -1];
% Six-state lateral feedback: [theta r sigma1 sigma_r chi epsilon], epsilon=yG.
d.chi_epsilon_poles=[-1.4 -1.6 -1.8 -2 -2.2 -2.4];
d.method = 'place'; % 'acker': no toolbox, repeated poles allowed
                    % 'place': Control System Toolbox, distinct SISO poles
end
