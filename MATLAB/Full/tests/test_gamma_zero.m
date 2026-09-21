function test_gamma_zero()
root=fileparts(fileparts(mfilename('fullpath'))); addpath(genpath(root));
p=model_parameters();
for v=[0 0.2]
    d=pole_placement_settings(); d.forward_speed=v;
    if v==0, d.lateral_states='balance'; else, d.lateral_states='balance_chi'; end
    c=controller_parameters();c.mode='pole_placement';c.hold_gamma_zero=true;
    c=generate_controller(p,c,d);
    % Check the acceleration identity away from the constrained manifold.
    x=zeros(12,1);x(2)=v/p.R;x(1)=0.02;x(3)=-0.03;x(5)=0.01;
    x(7)=0.04;x(9)=0.002;x(10)=0.003;
    u=controller_output(0,x,p,c); f=model_rhs(0,x,u,p);
    gd=x(5)-x(3)*tan(x(7));
    gdd=f(5)-f(3)*tan(x(7))-x(3)*x(1)/cos(x(7))^2;
    assert(abs(gdd+2*c.gamma_constraint_rate*gd+c.gamma_constraint_rate^2*x(10))<1e-9);
    s=simulation_settings();s.t_end=1;s.forward_speed0=v;
    s.gamma0=0;s.gamma_dot0=0;s.theta0=0.01*pi/180;s.psi0=0.1*pi/180;
    r=simulate_unicycle(p,c,s);
    assert(max(abs(r.X(:,10)))<1e-9);
    gd=r.X(:,5)-r.X(:,3).*tan(r.X(:,7));assert(max(abs(gd))<1e-8);
end
fprintf('Gamma-zero torque identity and integrated constraint checks passed.\n');
end
