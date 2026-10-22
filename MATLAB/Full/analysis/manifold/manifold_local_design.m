function d = manifold_local_design(theta0_deg,speed,theta_target_deg)
%MANIFOLD_LOCAL_DESIGN Existing turning-manifold LQR, with a neutral-mode test.
% Avoid powers of A when finding the single uncontrollable zero mode.
% d.field(e) is the unsaturated autonomous nine-state relative vector field.
if nargin<3, theta_target_deg=0; end
validateattributes(theta0_deg,{'numeric'},{'scalar','real','finite'});
validateattributes(speed,{'numeric'},{'scalar','real','finite','positive'});
root=fileparts(fileparts(fileparts(mfilename('fullpath'))));
addpath(fullfile(root,'config'),fullfile(root,'model'));
p=model_parameters(); p.mr=2.3; p.BR=6.5; p.BP=0;
idx=[1:7 9 10];
x0=zeros(12,1); x0(2)=speed/p.R; x0(7)=deg2rad(theta0_deg);
theta=deg2rad(theta_target_deg);
om_grid=linspace(-0.8,0.8,81);
mis=arrayfun(@mismatch,om_grid);
brackets=find(mis(1:end-1).*mis(2:end)<=0);
assert(~isempty(brackets),'No matched equilibrium in the yaw-rate search range.');
roots_om=arrayfun(@(k)fzero(@mismatch,om_grid(k:k+1)),brackets);
[~,k]=min(abs(roots_om)); omega=roots_om(k);
[xe,ue,rate]=equilibrium(omega);
[A,B]=jacobian(xe,ue);
[T,w,neutral_residual]=neutral_subspace(A,B);
Q=diag([2 1 1 2 1 0.5 20 20 5]); Ru=diag([0.1 0.1]);
Ac=T'*A*T; Bc=T'*B;
[~,~,~,~,staircase]=ctrbf(Ac,Bc,eye(8));
assert(sum(staircase)==8,'The transverse pair is not controllable.');
K=lqr(Ac,Bc,T'*Q*T,Ru)*T';
J=A-B*K;
assert(max(real(eig(T'*J*T)))<-1e-5,'Unstable transverse LQR design.');
d=struct('theta0_deg',theta0_deg,'speed',speed, ...
    'theta_target_deg',theta_target_deg,'p',p,'idx',idx, ...
    'x0',x0,'xe',xe,'ue',ue,'rate',rate,'omega',omega, ...
    'A',A,'B',B,'K',K,'J',J,'T',T,'w',w, ...
    'e0',x0(idx)-xe(idx),'neutral_residual',neutral_residual, ...
    'match_residual',w'*(x0(idx)-xe(idx)), ...
    'raw_ctrb_rank',sum(svd(ctrb(A,B))>1e-9*norm(ctrb(A,B),2)), ...
    'transverse_poles',eig(T'*J*T),'force_limit',30,'torque_limit',15);
d.field=@field;

    function value=mismatch(om)
        [x,u]=equilibrium(om);
        [a,b]=jacobian(x,u);
        [~,ww]=neutral_subspace(a,b);
        value=ww'*(x0(idx)-x(idx));
    end

    function [x,u,rate0]=equilibrium(om)
        V=speed/p.R;
        x_of_r=@(r)[0;V+om*sin(theta);om*cos(theta);0; ...
            om*sin(theta);0;theta;0;r;0;0;0];
        force=@(r)p.mr*(p.R*(V+om*sin(theta))*om*cos(theta) ...
            +p.g*sin(theta)-r*om^2*cos(theta)^2);
        u_of_r=@(r)[force(r);0];
        % The steady-turning residual is quadratic in r. Select its small
        % root continuously connected to the ordinary turning branch.
        c0=residual(0); cp=residual(1); cm=residual(-1);
        aa=(cp+cm)/2-c0; bb=(cp-cm)/2;
        if abs(aa)<1e-12*max(1,abs(bb))
            r=-c0/bb;
        else
            discriminant=bb^2-4*aa*c0;
            assert(discriminant>=0,'Turning equilibrium has no real rod position.');
            qq=-0.5*(bb+sign(bb)*sqrt(discriminant));
            candidates=[qq/aa c0/qq];
            [~,small]=min(abs(candidates)); r=candidates(small);
        end
        x=x_of_r(r); u=u_of_r(r);
        rate0=[zeros(5,1);model_kinematics(x,p)];
        assert(norm(model_rhs(0,x,u,p)-rate0,inf)<1e-8, ...
            'Invalid steady-turning equilibrium.');
        function c1=residual(rr)
            c=model_residual(x_of_r(rr),u_of_r(rr),p); c1=c(1);
        end
    end

    function [a,b]=jacobian(x,u)
        a=zeros(9); b=zeros(9,2); h=1e-20;
        for j=1:9
            z=x; z(idx(j))=z(idx(j))+1i*h;
            f=model_rhs(0,z,u,p); a(:,j)=imag(f(idx))/h;
        end
        for j=1:2
            v=u; v(j)=v(j)+1i*h;
            f=model_rhs(0,x,v,p); b(:,j)=imag(f(idx))/h;
        end
    end

    function f=field(e)
        x=xe; x(idx)=xe(idx)+e;
        dx=model_rhs(0,x,ue-K*e,p);
        f=dx(idx)-rate(idx);
    end
end

function [T,w,residual]=neutral_subspace(A,B)
% For this model the uncontrollable direction has eigenvalue exactly zero:
% A'*w=0, B'*w=0. Verify that assumption instead of assuming a ctrb rank.
stack=[A';B'];
stack=stack./max(vecnorm(stack,2,2),1);
[~,S,V]=svd(stack,0); sv=diag(S);
assert(sv(end)<1e-10*sv(1) && sv(end-1)>1e-8*sv(1), ...
    'Expected one and only one uncontrollable neutral direction.');
w=V(:,end); if w(7)<0, w=-w; end
residual=max(norm(w'*A,inf),norm(w'*B,inf));
assert(residual<1e-8,'Neutral-direction residual is too large.');
T=null(w');
end
