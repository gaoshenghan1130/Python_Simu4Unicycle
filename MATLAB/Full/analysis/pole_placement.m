function [K,info] = pole_placement(p,d)
% Stationary upright design using the CURRENT physical parameters.
% Edit config/pole_placement_settings.m to select all eight desired poles.
% With no arguments, print the default design; never clears the workspace.
root=fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(root,'model'),fullfile(root,'config'));
if nargin<1, p=model_parameters(); end
if nargin<2, d=pole_placement_settings(); end
if isfield(d,'lateral_states') && ~ismember(lower(char(d.lateral_states)),{'balance','balance_chi'})
    error('unicycle:LateralStates','Use balance or balance_chi for lateral_states.');
end
if isfield(d,'lateral_states') && strcmpi(d.lateral_states,'balance_chi')
    [K,info]=rolling_chi_placement(p,d);
    return;
end
%% Linearize the supplied full nonlinear model
% x = [sigma1 sigma2 sigma3 sigma_r sigma_g psi theta phi r gamma xG yG]'
% This decomposition is specific to rest, upright lean/body and centered rod.
x_eq = zeros(12,1);
u_eq = zeros(2,1);
assert(norm(model_rhs(0,x_eq,u_eq,p),inf) < 1e-12, ...
    'The selected point is not an equilibrium.');

% Complex-step differentiation is valid for the supplied smooth model.
% If nonanalytic operations such as abs/sign/clipping are added, replace it.
step = 1e-20;
A = zeros(12,12);
B = zeros(12,2);
for j = 1:12
    x_test = x_eq;
    x_test(j) = x_test(j) + 1i*step;
    A(:,j) = imag(model_rhs(0,x_test,u_eq,p))/step;
end
for j = 1:2
    u_test = u_eq;
    u_test(j) = u_test(j) + 1i*step;
    B(:,j) = imag(model_rhs(0,x_eq,u_test,p))/step;
end

%% Extract the independently controlled blocks
idx_lateral      = [7, 9, 1, 4]; % [theta, r, sigma1, sigma_r]
idx_longitudinal = [8, 10, 2, 5]; % [phi, gamma, sigma2, sigma_g]
A_lateral = A(idx_lateral,idx_lateral);
B_lateral = B(idx_lateral,1);
A_longitudinal = A(idx_longitudinal,idx_longitudinal);
B_longitudinal = B(idx_longitudinal,2);

% Verify the decomposition before using separate pole placement.
% z = [x_lateral; x_longitudinal; sigma3; psi; xG-R*phi; yG+R*theta]
T = eye(12);
T = T([idx_lateral,idx_longitudinal,3,6,11,12],:);
T(11,8) = -p.R;
T(12,7) = p.R;
A_z = T*A/T;
B_z = T*B;
A_expected = blkdiag(A_lateral,A_longitudinal,zeros(4));
A_expected(10,9) = 1; % yaw-rate integrator: psi_dot = sigma3
B_expected = zeros(12,2);
B_expected(1:4,1) = B_lateral;
B_expected(5:8,2) = B_longitudinal;
assert(norm(A_z-A_expected,inf) < 1e-9 && ...
       norm(B_z-B_expected,inf) < 1e-9, ...
       'The assumed stationary-equilibrium block decomposition does not hold.');
assert(rank(controllability_matrix(A_lateral,B_lateral)) == 4, ...
    'Lateral block is not controllable with these parameters.');
assert(rank(controllability_matrix(A_longitudinal,B_longitudinal)) == 4, ...
    'Longitudinal block is not controllable with these parameters.');

%% Compute gains and embed them in the original 12-state ordering
K_lateral = assign_poles(A_lateral,B_lateral,d.lateral,d.method);
K_longitudinal = assign_poles(A_longitudinal,B_longitudinal,d.longitudinal,d.method);
K = zeros(2,12);
K(1,idx_lateral) = K_lateral;
K(2,idx_longitudinal) = K_longitudinal;


info.A=A; info.B=B; info.x_eq=x_eq; info.u_eq=u_eq;
info.idx_lateral=idx_lateral; info.idx_longitudinal=idx_longitudinal;
info.K_lateral=K_lateral; info.K_longitudinal=K_longitudinal;
info.requested_lateral=d.lateral; info.requested_longitudinal=d.longitudinal;
info.actual_lateral=eig(A_lateral-B_lateral*K_lateral);
info.actual_longitudinal=eig(A_longitudinal-B_longitudinal*K_longitudinal);
info.full_poles=eig(A-B*K);
info.method=d.method;
if nargout==0
    fprintf('u = -K*x; columns: sigma1 sigma2 sigma3 sigma_r sigma_g psi theta phi r gamma xG yG\n');
    disp(K);
    fprintf('Lateral poles / longitudinal poles:\n');
    disp([info.actual_lateral,info.actual_longitudinal]);
    fprintf('Four unassigned zero poles remain in the full stationary model.\n');
end
end

function C=controllability_matrix(A,B)
C=[B,A*B,A^2*B,A^3*B];
end

function K=assign_poles(A,B,poles,method)
validateattributes(poles,{'numeric'},{'vector','numel',4,'finite'});
poles=poles(:).';
coeff=poly(poles);
if norm(imag(coeff),inf)>1e-10*max(1,norm(coeff,inf))
    error('unicycle:PolePairs','Complex poles must occur in conjugate pairs.');
end
if any(real(poles)>=0)
    error('unicycle:UnstablePoles','All requested poles must have negative real parts.');
end
switch lower(char(method))
    case 'acker'
        % SISO Ackermann formula supports repeated poles without a toolbox.
        C=controllability_matrix(A,B);
        if rcond(C)<1e-12
            error('unicycle:Conditioning','Controllability matrix is ill-conditioned; revise parameters.');
        end
        q=real(coeff);
        PA=A^4+q(2)*A^3+q(3)*A^2+q(4)*A+q(5)*eye(4);
        K=([0 0 0 1]/C)*PA;
    case 'place'
        if isempty(which('place'))
            error('unicycle:Toolbox','place requires Control System Toolbox; select acker instead.');
        end
        if numel(unique(poles))~=4
            error('unicycle:RepeatedPoles','SISO place requires distinct poles; select acker for repeated poles.');
        end
        K=place(A,B,poles);
    otherwise
        error('unicycle:PoleMethod','Unknown pole method: %s',method);
end
% Polynomial check is appropriate even for numerically split repeated roots.
actual=poly(A-B*K);
if norm(actual-real(coeff),inf)>1e-6*max(1,norm(coeff,inf))
    error('unicycle:PoleAccuracy','Pole assignment failed the characteristic-polynomial check.');
end
end
