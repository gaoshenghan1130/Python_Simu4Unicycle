function d = pole_placement_settings()
% Select balance (stationary) or balance_chi (straight rolling) below.
% balance: 4 lateral poles; balance_chi: 5 lateral poles. Units: 1/s.
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
d.method = 'acker'; % 'acker': no toolbox, repeated poles allowed
                    % 'place': Control System Toolbox, distinct SISO poles
end
