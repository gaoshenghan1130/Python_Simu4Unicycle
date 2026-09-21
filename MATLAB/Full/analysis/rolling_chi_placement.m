function [K,info]=rolling_chi_placement(p,d)
% Straight rolling, chi=psi (reference heading zero), unchanged 12-state model.
% Eliminate the uncontrollable invariant sigma3-a*theta in the design only.
% The full nonlinear simulation retains sigma3 and all other states.
validateattributes(d.forward_speed,{'numeric'},{'real','finite','scalar','nonzero'});
x=zeros(12,1); x(2)=d.forward_speed/p.R;
rate=zeros(12,1); rate(8)=x(2); rate(11)=d.forward_speed;
assert(norm(model_rhs(0,x,[0;0],p)-rate,inf)<1e-10, ...
    'The selected rolling trajectory is not a solution.');
A=zeros(12); B=zeros(12,2); h=1e-20;
for j=1:12
    z=x; z(j)=z(j)+1i*h; A(:,j)=imag(model_rhs(0,z,[0;0],p))/h;
end
for j=1:2
    u=zeros(2,1); u(j)=1i*h; B(:,j)=imag(model_rhs(0,x,u,p))/h;
end
idx=[7 9 1 4 6]; % [theta r sigma1 sigma_r chi]
a=A(3,1);
constraint=zeros(1,12); constraint(3)=1; constraint(7)=-a;
assert(norm(constraint*A,inf)<1e-9 && norm(constraint*B,inf)<1e-9, ...
    'The assumed rolling invariant does not hold for these parameters.');
E=zeros(12,5); E(idx,:)=eye(5); E(3,1)=a;
Al=A(idx,:)*E; Bl=B(idx,1);
lon=[8 10 2 5]; An=A(lon,lon); Bn=B(lon,2);
% Check selected dynamics are closed apart from the retained invariant.
omitted=setdiff(1:12,[idx 3]);
assert(norm(A(idx,omitted),inf)<1e-9 && norm(B(idx,2),inf)<1e-9);
assert(norm(A(lon,setdiff(1:12,lon)),inf)<1e-9 && norm(B(lon,1),inf)<1e-9);
Kl=assign(Al,Bl,d.chi_poles,d.method);
Kn=assign(An,Bn,d.longitudinal,d.method);
K=zeros(2,12); K(1,idx)=Kl; K(2,lon)=Kn;
info.A=A; info.B=B; info.x_eq=x; info.reference_rate=rate;
info.u_eq=zeros(2,1); info.idx_lateral=idx; info.idx_longitudinal=lon;
info.K_lateral=Kl; info.K_longitudinal=Kn;
info.requested_lateral=d.chi_poles; info.requested_longitudinal=d.longitudinal;
info.actual_lateral=eig(Al-Bl*Kl); info.actual_longitudinal=eig(An-Bn*Kn);
info.full_poles=eig(A-B*K); info.invariant=constraint;
info.method=d.method; info.forward_speed=d.forward_speed;
end

function K=assign(A,B,poles,method)
n=size(A,1);
validateattributes(poles,{'numeric'},{'vector','numel',n,'finite'});
q=poly(poles(:).');
if any(real(poles)>=0) || norm(imag(q),inf)>1e-10*max(1,norm(q,inf))
    error('unicycle:RollingPoles','Use stable real poles or complex conjugate pairs.');
end
C=zeros(n); C(:,1)=B;
for j=2:n, C(:,j)=A*C(:,j-1); end
if rank(C)<n || rcond(C)<1e-14
    error('unicycle:RollingControllability','Rolling design is uncontrollable or ill-conditioned at this speed.');
end
switch lower(char(method))
    case 'acker'
        PA=eye(n);
        for j=2:n+1, PA=A*PA+real(q(j))*eye(n); end
        last=zeros(1,n); last(end)=1; K=(last/C)*PA;
    case 'place'
        if isempty(which('place')), error('unicycle:Toolbox','place unavailable; use acker.'); end
        if numel(unique(poles))~=n, error('unicycle:RepeatedPoles','Use acker for repeated poles.'); end
        K=place(A,B,poles);
    otherwise
        error('unicycle:PoleMethod','Unknown pole method.');
end
if norm(poly(A-B*K)-real(q),inf)>1e-6*max(1,norm(q,inf))
    error('unicycle:PoleAccuracy','Rolling pole assignment failed verification.');
end
end
