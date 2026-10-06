function result = compare_yaw_lean_linearization(speed)
%COMPARE_YAW_LEAN_LINEARIZATION Compare the current model with paper Eq. (64).
% Linearizes the full model about upright straight rolling and compares
% d(sigma3_dot)/d(sigma1) with -2*phi_dot = -2*speed/R.
% Run from MATLAB with:
%   addpath('MATLAB/Full');
%   result = compare_yaw_lean_linearization();

if nargin < 1 || isempty(speed)
    speed = 2.375; % m/s, same forward speed used by the turning test
end
assert(isscalar(speed) && isfinite(speed) && speed > 0, ...
    'speed must be a positive finite scalar in m/s.');

fullRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(fullRoot, 'config'), fullfile(fullRoot, 'model'));
pBase = model_parameters();

% Test both the parameter set used by the current turning script and the
% same model with rod damping disabled, to identify its effect explicitly.
pCurrent = pBase;
pCurrent.mr = 2.3;
pCurrent.BR = 6.5;
pCurrent.BP = 0;
pNoRodDamping = pCurrent;
pNoRodDamping.BR = 0;

cases = {pCurrent, pNoRodDamping};
caseNames = {"turning-script parameters", "same parameters, BR = 0"};
paperCoefficient = -2 * speed / pBase.R;
fprintf('Straight-rolling linearization at v = %.9g m/s, R = %.9g m\n', ...
    speed, pBase.R);
fprintf('Paper Eq. (64) prediction -2*v/R: %.12g 1/s\n\n', paperCoefficient);

result.speed = speed;
result.radius = pBase.R;
result.paper_coefficient = paperCoefficient;
caseTemplate = struct('name', "", 'coefficient', NaN, ...
    'difference_from_paper', NaN, 'relative_error', NaN, ...
    'sigma3_dot_row', []);
result.cases = repmat(caseTemplate, 1, numel(cases));
for k = 1:numel(cases)
    p = cases{k};
    xEq = zeros(12, 1);
    xEq(2) = speed / p.R; % sigma2 = wheel spin rate at upright straight roll
    uEq = zeros(2, 1);   % no actuator input at the straight-rolling point

    A = complex_step_jacobian(@(x) model_rhs(0, x, uEq, p), xEq);
    % State order begins [sigma1 sigma2 sigma3 sigma_r sigma_g ...].
    coefficient = A(3, 1);
    result.cases(k).name = caseNames{k};
    result.cases(k).coefficient = coefficient;
    result.cases(k).difference_from_paper = coefficient - paperCoefficient;
    result.cases(k).relative_error = ...
        abs(coefficient - paperCoefficient) / max(abs(paperCoefficient), eps);
    result.cases(k).sigma3_dot_row = A(3, :);

    fprintf('%s:\n', caseNames{k});
    fprintf('  d(sigma3_dot)/d(sigma1): %.12g 1/s\n', coefficient);
    fprintf('  difference from paper:     %.12g 1/s\n', ...
        coefficient - paperCoefficient);
    fprintf('  relative difference:       %.6g %%\n', ...
        100 * result.cases(k).relative_error);
    fprintf('  sigma3_dot linearization row [sigma1 sigma2 sigma3 sigma_r sigma_g]:\n');
    fprintf('  [% .6g % .6g % .6g % .6g % .6g]\n\n', A(3, 1:5));
end
end

function J = complex_step_jacobian(fun, x)
f0 = fun(x);
J = zeros(numel(f0), numel(x));
h = 1e-20;
for j = 1:numel(x)
    z = x;
    z(j) = z(j) + 1i*h;
    J(:, j) = imag(fun(z)) / h;
end
end
