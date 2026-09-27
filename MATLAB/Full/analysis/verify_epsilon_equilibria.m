function report=verify_epsilon_equilibria()
% Verify exact relative equilibria and perturbed nonlinear trajectories.
% This is an analysis function, not an additional main script.
root=fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(root,'config'),fullfile(root,'model'), ...
    fullfile(root,'controllers'),fullfile(root,'simulation'),fullfile(root,'analysis'));
e=simulation_preset('chi_epsilon_balance');
p=e.parameters; c=generate_controller(p,e.controller,e.design); info=c.design_info;
assert(p.BP==0 && ~c.hold_gamma_zero && isinf(c.force_limit) && isinf(c.torque_limit));
assert(abs(c.K(1,12))>1e-8);
assert(norm(poly(info.actual_lateral)-poly(e.design.chi_epsilon_poles),inf)<1e-6);
report.zero_poles=sum(abs(info.full_poles)<1e-6);
assert(report.zero_poles==2);
report.full_poles=info.full_poles; report.K=c.K; report.invariant=info.invariant;
% gamma=chi=0, sigma2=v/R, all other velocities zero.
% F=mr*g*sin(theta); r=[R*(mr+mp+mw)+h*mp]/mr*tan(theta).
% The epsilon feedback supplies exactly the force required by each lean.
angles=[-1 -0.1 0 0.1 1]; rows=cell(1,numel(angles));
for j=1:numel(angles)
    theta=angles(j)*pi/180;
    x=info.x_eq; x(7)=theta;
    x(9)=(p.R*(p.mr+p.mp+p.mw)+p.h*p.mp)/p.mr*tan(theta);
    force=p.mr*p.g*sin(theta);
    x(12)=(-force-c.K(1,7)*theta-c.K(1,9)*x(9))/c.K(1,12);
    residual=0;
    for t=[0 2 10]
        xt=x+t*info.reference_rate;
        u=controller_output(t,xt,p,c);
        residual=max(residual,norm(model_rhs(t,xt,u,p)-info.reference_rate,inf));
    end
    assert(residual<1e-9);
    % Arbitrary along-track translation also remains an exact solution.
    shifted=x; shifted(11)=0.37;
    assert(norm(model_rhs(0,shifted,controller_output(0,shifted,p,c),p)-info.reference_rate,inf)<1e-9);
    row.theta_deg=angles(j); row.r_m=x(9); row.epsilon_m=x(12);
    row.force_N=force; row.residual=residual; rows{j}=row;
end
report.equilibria=struct2table([rows{:}]);
% Isolated epsilon, isolated lean, and a point on the exact equilibrium curve.
cases={'epsilon_0.01m','theta_0.1deg','exact_theta_0.1deg'};
runs=cell(1,3); measurements=cell(1,3);
for j=1:3
    trial=e; trial.settings.t_end=60; trial.settings.output_dt=0.02;
    trial.settings.theta0=0; trial.settings.yG0=0;
    if j==1, trial.settings.yG0=0.01; end
    if j==2, trial.settings.theta0=0.1*pi/180; end
    if j==3
        trial.settings.theta0=0.1*pi/180;
        trial.settings.r0=report.equilibria.r_m(4);
        trial.settings.yG0=report.equilibria.epsilon_m(4);
    end
    r=run_experiment(trial); r.label=cases{j}; runs{j}=r;
    assert(isempty(r.event_time) && abs(r.t(end)-60)<1e-8);
    tail=r.t>=50;
    row=struct('name',cases{j},'theta_deg',r.X(end,7)*180/pi, ...
        'r_m',r.X(end,9),'chi_deg',r.X(end,6)*180/pi,'epsilon_m',r.X(end,12), ...
        'tail_epsilon_range_m',max(r.X(tail,12))-min(r.X(tail,12)), ...
        'final_relative_residual',norm(model_rhs(r.t(end),r.X(end,:)',r.U(end,:)',p)-info.reference_rate,inf));
    measurements{j}=row;
end
report.simulations=struct2table([measurements{:}]); report.runs=[runs{:}];
assert(max(abs(runs{3}.X(:,7)-0.1*pi/180))<1e-7);
disp(report.equilibria); disp(report.simulations);
fprintf('Full closed-loop zero poles: %d. Exact relative equilibrium family remains.\n',report.zero_poles);
end
