function report = test_fixed_rod()
% Verify constraint force against all five original equations, analytic
% Jacobian/spectrum, and exact trajectories (including critical speed).
p=fixed_rod_setup(); c=struct('kp',3,'kd',0.8,'speed',0);
s=fixed_rod_linear(p,c); vc=s.critical_speed;
max_res=0; max_jac=0; max_poly=0; max_sol=0;
for gains=[0 1]
    c.kp=3*gains; c.kd=0.8*gains;
    for v=[0 0.8*vc vc 1.2*vc -1.2*vc]
        c.speed=v; s=fixed_rod_linear(p,c);
        z=zeros(10,1); z(2)=v/p.R;
        A=zeros(10);
        for j=1:10
            zz=z; zz(j)=zz(j)+1i*1e-20;
            A(:,j)=imag(fixed_rod_rhs(0,zz,p,c))/1e-20;
        end
        max_jac=max(max_jac,max(abs(A-s.A),[],'all'));
        max_poly=max(max_poly,max(abs(poly(A)-poly(s.eigenvalues))));
        for t=[0 0.02 0.15]
            y0=[0.01;-0.02;0.03];
            B=[0 1 0;s.k 0 s.d;0 -s.a 0];
            max_sol=max(max_sol,norm(fixed_rod_lateral_solution(t,y0,s)-expm(B*t)*y0,inf));
        end
        z=z+[.1;0;.15;.08;.2;.12;.3;.09;.1;-.1];
        [dz,F,x]=fixed_rod_rhs(0,z,p,c);
        torque=p.BP*v/p.R+c.kp*z(8)+c.kd*(z(4)-z(3)*tan(z(6)));
        full=model_rhs(0,x,[F;torque],p);
        expected=[dz(1:3);-p.R*dz(1);dz(4);dz(5:7);0;dz(8:10)];
        max_res=max(max_res,norm(full-expected,inf));
    end
end
assert(max_res<1e-10); assert(max_jac<1e-10);
assert(max_poly<1e-7); assert(max_sol<1e-10);
% Nonlinear small-perturbation consistency over a finite interval.
c.speed=1.2*vc; s=fixed_rod_linear(p,c);
z=zeros(10,1); z(2)=c.speed/p.R; z(6)=1e-6;
[t,Z]=ode45(@(t,z) fixed_rod_rhs(t,z,p,c),[0 0.5],z, ...
    odeset('RelTol',1e-11,'AbsTol',1e-13));
y=fixed_rod_lateral_solution(t,[z(6);0;0],s);
trajectory_error=max(abs(Z(:,[6 1 3])-y.'),[],'all');
assert(trajectory_error<1e-9);
report=struct('constraint_error',max_res,'jacobian_error',max_jac, ...
    'characteristic_polynomial_error',max_poly,'analytic_solution_error',max_sol, ...
    'small_perturbation_error',trajectory_error,'critical_speed',vc);
disp(report);
end
