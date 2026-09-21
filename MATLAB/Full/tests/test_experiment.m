function test_experiment()
% MATLAB regression checks; no Control System Toolbox required.
root=fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(root,'config'),fullfile(root,'model'),fullfile(root,'controllers'), ...
    fullfile(root,'simulation'),fullfile(root,'analysis'));
e=experiment_settings(); e.controller.mode='pd'; e.settings.t_end=0.15;
p=e.parameters; c=e.controller; x=initial_state(e.settings,p);
assert(isequal(pd_controller(0,x,p,c),controller_output(0,x,p,c)));
r=run_experiment(e);
assert(numel(r)==1 && all(isfinite(r.X(:))));

% Distinct, repeated, and complex-conjugate poles.
d=e.design;
for kind=1:3
    if kind==2, d.lateral=[-2 -2 -2 -2]; d.longitudinal=[-3 -3 -3 -3]; end
    if kind==3, d.lateral=[-2+1i -2-1i -3 -4]; end
    [K,info]=pole_placement(p,d);
    assert(isreal(K) && isequal(size(K),[2 12]));
    assert(norm(poly(info.actual_lateral)-poly(d.lateral),inf)<1e-5);
    assert(norm(poly(info.actual_longitudinal)-poly(d.longitudinal),inf)<1e-5);
    assert(sum(abs(info.full_poles)<1e-5)==4);
end

% Damping is incorporated by linearizing current parameters.
pDamped=p; pDamped.BR=0.05; pDamped.BP=0.03;
pole_placement(pDamped,e.design);
if ~isempty(which('place'))
    d=e.design; d.method='place'; Kp=pole_placement(p,d);
    d.method='acker'; Ka=pole_placement(p,d);
    assert(norm(Kp-Ka,inf)<1e-7);
    d.method='place'; d.lateral=[-2 -2 -2 -2];
    expect_error(@() pole_placement(p,d),'unicycle:RepeatedPoles');
end

% Sweep must precede controller generation and be independent for each run.
e.controller.mode='pole_placement';
e.sweep.enabled=true; e.sweep.values=[1.5 2.3 3];
r=run_experiment(e);
for j=1:3
    assert(r(j).parameters.mr==e.sweep.values(j));
    [expected,~]=pole_placement(r(j).parameters,e.design);
    assert(norm(r(j).controller.K-expected,inf)<1e-10);
end
assert(norm(r(1).controller.K-r(3).controller.K,inf)>1e-3);

% Direct state feedback reproduces the generated controller exactly.
e.sweep.enabled=false;
a=run_experiment(e);
e.controller.mode='direct'; e.controller.K=a.controller.K;
b=run_experiment(e);
assert(norm(a.X-b.X,inf)<1e-10 && norm(a.U-b.U,inf)<1e-10);
e.controller.force_limit=0.1; e.controller.torque_limit=0.2;
c=generate_controller(p,e.controller,e.design);
u=controller_output(0,ones(12,1),p,c);
assert(abs(u(1))<=0.1 && abs(u(2))<=0.2);

% Indexed pole and PD-gain sweeps.
e=experiment_settings(); e.settings.t_end=0.1;
e.controller.mode='pole_placement'; e.sweep.enabled=true;
e.sweep.parameter='design.lateral'; e.sweep.index=1; e.sweep.values=[-1 -3];
q=run_experiment(e);
assert(q(1).design.lateral(1)==-1 && q(2).design.lateral(1)==-3);
e.controller.mode='pd'; e.sweep.parameter='controller.kp_gamma';
e.sweep.index=[]; e.sweep.values=[2 4];
q=run_experiment(e);
assert(q(1).controller.kp_gamma==2 && q(2).controller.kp_gamma==4);

% Reject silently ineffective sweeps and malformed poles/matrices.
e.sweep.parameter='design.lateral'; e.sweep.index=1;
expect_error(@() run_experiment(e),'unicycle:InactiveSweep');
d=e.design; d.lateral=[-1+1i -2 -3 -4];
expect_error(@() pole_placement(p,d),'unicycle:PolePairs');
d=e.design; d.lateral=[1 -2 -3 -4];
expect_error(@() pole_placement(p,d),'unicycle:UnstablePoles');

% Plot all sweep cases with one line per run per panel.
previous=get(groot,'DefaultFigureVisible');
cleanup=onCleanup(@() set(groot,'DefaultFigureVisible',previous));
set(groot,'DefaultFigureVisible','off');
f=plot_simulation(r);
axesHandles=findall(f,'Type','axes');
assert(numel(axesHandles)==8);
for j=1:8, assert(numel(findall(axesHandles(j),'Type','line'))==3); end
close(f);
fprintf('All Full-model experiment regression checks passed.\n');
end

function expect_error(action,id)
try
    action();
catch err
    assert(strcmp(err.identifier,id),'Unexpected error: %s',err.message);
    return;
end
error('Expected error %s was not raised.',id);
end
