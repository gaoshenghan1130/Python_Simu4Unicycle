function torque=gamma_zero_torque(t,x,F,p,rate)
% Exact-model torque that makes gamma_ddot+2*rate*gamma_dot+rate^2*gamma=0.
% Starting at gamma=gamma_dot=0 preserves zero up to integration roundoff.
% This uses the existing actuator; it replaces longitudinal speed feedback.
f0=model_rhs(t,x,[F;0],p);
f1=model_rhs(t,x,[F;1],p);
gamma_dot=x(5)-x(3)*tan(x(7));
acc0=f0(5)-f0(3)*tan(x(7))-x(3)*x(1)/cos(x(7))^2;
authority=(f1(5)-f0(5))-(f1(3)-f0(3))*tan(x(7));
if ~isfinite(authority) || abs(authority)<1e-10
    error('unicycle:GammaConstraint','Insufficient torque authority to hold gamma at zero.');
end
target_acc=-2*rate*gamma_dot-rate^2*x(10);
torque=(target_acc-acc0)/authority;
end
