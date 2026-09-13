function dx = model_rhs(t,x,u,p)
% Open-loop model. x has 12 states; u=[F;M2] in N and N*m.
% t is retained for compatibility with ODE solvers.
x=x(:); u=u(:);
assert(numel(x)==12 && numel(u)==2,'Expected x(12), u(2).');
M=mass_matrix(x,p);
c=model_residual(x,u,p);
dx=[M\(-c); model_kinematics(x,p)];
end
