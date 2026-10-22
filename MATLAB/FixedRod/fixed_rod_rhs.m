function [dz,F,x] = fixed_rod_rhs(~,z,p,c)
% z=[s1;s2;s3;sg;psi;theta;phi;gamma;xG;yG].
% Ideal mechanical lock: r=0, sr=-R*s1. F is the lock reaction,
% with the same sign as the public F input of Full/model_rhs.
z=z(:);
assert(numel(z)==10,'Expected 10 states.');
T=[1 0 0 0;0 1 0 0;0 0 1 0;-p.R 0 0 0;0 0 0 1];
x=[T*z(1:4);z(5:7);0;z(8:10)];
gamma_dot=z(4)-z(3)*tan(z(6));
% Constant feedforward balances wheel/body damping at nominal rolling.
M2=p.BP*c.speed/p.R+c.kp*z(8)+c.kd*gamma_dot;
M=mass_matrix(x,p);
f=model_residual(x,[0;M2],p);
dv=-(T.'*M*T)\(T.'*f);
dq=model_kinematics(x,p);
dz=[dv;dq([1 2 3 5 6 7])];
% Row 4 of M*T*dv+f=[R;0;0;1;0]*F.
reaction=M*T*dv+f;
F=reaction(4);
end
