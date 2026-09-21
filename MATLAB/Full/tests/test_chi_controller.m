function test_chi_controller()
root=fileparts(fileparts(mfilename('fullpath'))); addpath(genpath(root));
e=experiment_settings(); e.controller.mode='pole_placement';
e.design.lateral_states='balance_chi'; e.sweep.enabled=false;
% Fixed regression fixture; slow-pole/current-initial-condition study is separate.
e.parameters.BR=0;
e.design.chi_poles=[-2 -2.5 -3 -3.5 -4];
e.design.longitudinal=[-3 -3.5 -4 -4.5];
e.settings.theta0=0; e.settings.gamma0=0; e.settings.psi0=5*pi/180;
e.settings.forward_speed0=e.design.forward_speed;
e.settings.t_end=15;
c=generate_controller(e.parameters,e.controller,e.design);
assert(abs(c.K(1,6))>1e-6);
assert(norm(sort(c.design_info.actual_lateral)-sort(e.design.chi_poles(:)),inf)<1e-6);
assert(sum(abs(c.design_info.full_poles)<1e-5)==3);
for t=[0 1 8]
    reference=c.x_ref+t*c.reference_rate;
    assert(norm(controller_output(t,reference,e.parameters,c),inf)<1e-12);
    assert(norm(model_rhs(t,reference,[0;0],e.parameters)-c.reference_rate,inf)<1e-10);
end
r=run_experiment(e);
assert(isempty(r.event_time) && abs(r.t(end)-15)<1e-9);
assert(abs(r.X(end,6))<0.01*pi/180);
assert(max(abs(r.X(:,7)))<6*pi/180);
% Recompute rolling K for every swept design speed.
e.settings.t_end=0.05; e.sweep.enabled=true;
e.sweep.parameter='design.forward_speed'; e.sweep.values=[0.15 0.3];
r=run_experiment(e);
assert(norm(r(1).controller.K-r(2).controller.K,inf)>1e-3);
for j=1:2
    assert(abs(r(j).controller.reference_rate(11)-e.sweep.values(j))<1e-12);
end
d=e.design; d.chi_poles=-2*ones(1,5);
[~,info]=pole_placement(e.parameters,d);
assert(norm(poly(info.actual_lateral)-poly(d.chi_poles),inf)<1e-5);
fprintf('Chi controller, trajectory reference, nonlinear response and sweep checks passed.\n');
end
