function result = eigenvalues_lateral_centered_rod(speeds, center_rod)
%EIGENVALUES_LATERAL_CENTERED_ROD Linear spectrum for lateral-control cases.
% Lateral mode retains gamma-PD; optional center_rod adds rod r/r_dot PD.
% Report the full 12-state spectrum. The stability-margin scan ignores only
% eigenvalues numerically equal to zero (phase/position modes).

if nargin < 1 || isempty(speeds)
    speeds = [0.5 1.0 1.5 1.588584 1.723 2.375 3.145 4.0];
end
if nargin < 2 || isempty(center_rod)
    center_rod = true;
end
validateattributes(speeds, {'numeric'}, {'vector','real','finite','positive'});
speeds = speeds(:).';

root = fileparts(fileparts(mfilename('fullpath')));
addpath(genpath(root));
experiment = simulation_preset('mate_current_mass');
p = experiment.parameters;
c = experiment.controller;
c.mode = 'lateral_open_loop';
c.center_rod = logical(center_rod);
c.r_ref = 0;
c.r_dot_ref = 0;
dynamic_states = 1:12;

result.parameters = p;
result.controller = c;
result.speeds = speeds;
result.center_rod = c.center_rod;
result.dynamic_states = dynamic_states;
result.spectra = cell(size(speeds));
result.max_real_part = zeros(size(speeds));

for k = 1:numel(speeds)
    A = linearized_closed_loop(speeds(k), p, c);
    lambda = eig(A(dynamic_states,dynamic_states));
    [~,order] = sort(real(lambda),'descend');
    result.spectra{k} = lambda(order);
    nonzero = lambda(abs(lambda) > 1e-8);
    result.max_real_part(k) = max(real(nonzero));
    fprintf('\nv = %.6f m/s, max Re(lambda) = %+.9e 1/s\n', ...
        speeds(k), result.max_real_part(k));
    fprintf('  Re(lambda) [1/s]       Im(lambda) [rad/s]\n');
    fprintf('  %+.9e              %+.9e\n', ...
        [real(lambda(order)) imag(lambda(order))].');
end

grid = 0.05:0.02:20;
if c.center_rod
    growth = arrayfun(@(v) dominant_growth(v,p,c,dynamic_states), grid);
    unstable = growth > 1e-6;
    crossings = find(diff(unstable) ~= 0);
    result.critical_speeds = zeros(size(crossings));
    for j = 1:numel(crossings)
        bracket = grid(crossings(j):crossings(j)+1);
        result.critical_speeds(j) = fzero( ...
            @(v) dominant_growth(v,p,c,dynamic_states), bracket);
    end
    fprintf('\nDominant-growth sign-change speeds (m/s): %s\n', ...
        mat2str(result.critical_speeds,10));
else
    has_real_growth = arrayfun( ...
        @(v) has_positive_real_pole(v,p,c,dynamic_states), grid);
    crossing = find(has_real_growth(1:end-1) & ~has_real_growth(2:end),1);
    result.critical_speeds = NaN;
    if ~isempty(crossing)
        lo = grid(crossing); hi = grid(crossing+1);
        while hi-lo > 1e-7
            mid = (lo+hi)/2;
            if has_positive_real_pole(mid,p,c,dynamic_states), lo=mid; else, hi=mid; end
        end
        result.critical_speeds = (lo+hi)/2;
    end
    fprintf('\nPositive-real-to-neutral transition (m/s): %s\n', ...
        mat2str(result.critical_speeds,10));
end
end

function growth = dominant_growth(speed,p,c,dynamic_states)
A = linearized_closed_loop(speed,p,c);
    lambda = eig(A(dynamic_states,dynamic_states));
lambda = lambda(abs(lambda) > 1e-8);
growth = max(real(lambda));
end

function yes = has_positive_real_pole(speed,p,c,dynamic_states)
A = linearized_closed_loop(speed,p,c);
lambda = eig(A(dynamic_states,dynamic_states));
yes = any(real(lambda)>1e-7 & abs(imag(lambda))<1e-6);
end

function A = linearized_closed_loop(speed,p,c)
x_eq = zeros(12,1);
x_eq(2) = speed/p.R;
% Apply the unsaturated PD law directly: min/max saturation is not
% complex-step differentiable, and both actuator limits are infinite here.
rhs = @(x) model_rhs(0,x,linear_pd_input(x,p,c),p);
h = 1e-20;
A = zeros(12);
for j = 1:12
    x = x_eq;
    x(j) = x(j) + 1i*h;
    A(:,j) = imag(rhs(x))/h;
end
end

function u = linear_pd_input(x,p,c)
gamma_dot = x(5)-x(3)*tan(x(7));
F = 0;
if c.center_rod
    r_dot = p.R*x(1)+x(4);
    F = c.kp_r*(c.r_ref-x(9))+c.kd_r*(c.r_dot_ref-r_dot);
end
M2 = c.kp_gamma*(x(10)-c.gamma_ref) ...
    +c.kd_gamma*(gamma_dot-c.gamma_dot_ref);
u = [F;M2];
end
