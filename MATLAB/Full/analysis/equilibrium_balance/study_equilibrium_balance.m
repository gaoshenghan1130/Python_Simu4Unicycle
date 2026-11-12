function report=study_equilibrium_balance(rho,tf,leanDeg,Fmax,cfg)
if nargin<1,rho=.1;end
if nargin<2,tf=40;end
if nargin<3,leanDeg=1;end
if nargin<4,Fmax=30;end
if nargin<5,cfg=struct();end
cfg=defaults(cfg);
validateattributes(rho,{'numeric'},{'scalar','positive','finite'});
validateattributes(tf,{'numeric'},{'scalar','positive','finite'});
validateattributes(Fmax,{'numeric'},{'scalar','positive','finite'});
assert(all(ismember(string(cfg.models),["linear","nonlinear"])),'Unknown model.');
assert(all(ismember(string(cfg.initial_cases),["lean","compatible","equilibrium","custom"])),'Unknown initial case.');
assert(numel(cfg.Q)==4 && all(cfg.Q>0),'Q requires four positive weights.');
assert(numel(cfg.Q_long)==3 && all(cfg.Q_long>0),'Q_long requires three positive weights.');
validateattributes(cfg.speeds,{'numeric'},{'vector','real','finite','nonempty'});
validateattributes(cfg.sample_time,{'numeric'},{'scalar','positive','finite'});
validateattributes(cfg.torque_limit,{'numeric'},{'scalar','positive','finite'});
validateattributes(cfg.rho_long,{'numeric'},{'scalar','positive','finite'});
assert(numel(cfg.stop_limits)==3 && all(cfg.stop_limits>0));
% Equilibrium-manifold feedback; design at upright straight rolling.
% Uses baseline physical parameters, exact derivative of r_des, saturation,
% and a separate longitudinal LQR. Does not overwrite earlier studies.
root=fileparts(fileparts(fileparts(mfilename('fullpath'))));
addpath(fullfile(root,'config'),fullfile(root,'model'));
out=fullfile(root,'results',sprintf('equilibrium_balance_rho_%g_lean_%g_F_%g',rho,leanDeg,Fmax)); if strlength(string(cfg.output_dir))>0,out=char(cfg.output_dir);end
if ~isfolder(out),mkdir(out);end
p=model_parameters();
fields=fieldnames(cfg.parameter_overrides);
for j=1:numel(fields),assert(isfield(p,fields{j}),'Unknown physical parameter.');p.(fields{j})=cfg.parameter_overrides.(fields{j});end
runs={}; rows=[]; designs={};
for speed=cfg.speeds(:)'
 d=design(speed,p,rho,cfg); d.Fmax=Fmax; d.Mmax=cfg.torque_limit; verify_local_design(d,p); designs{end+1}=d;
 for mode=reshape(string(cfg.models),1,[])
  for init=reshape(string(cfg.initial_cases),1,[])
   x0=d.xe; x0(7)=deg2rad(leanDeg);
   if init=="custom"
    th=x0(7);om=deg2rad(cfg.yaw_rate0_deg_s);
    x0(1)=deg2rad(cfg.lean_rate0_deg_s);
    x0(3)=om*cos(th);x0(4)=cfg.rod_rate0-p.R*x0(1);
    x0(5)=deg2rad(cfg.body_rate0_deg_s)+om*sin(th);
    x0(2)=d.V+om*sin(th);
    x0(6)=deg2rad(cfg.heading0_deg);x0(9)=cfg.rod0;x0(10)=deg2rad(cfg.body0_deg);
    if mode=="linear",x0(3)=om;x0(5)=deg2rad(cfg.body_rate0_deg_s);x0(2)=d.V;end
   end
   if init=="compatible",x0(3)=-d.kappa*x0(7);end
   if init=="equilibrium"
    if speed==0,om=.15;th=0;
    else,om=.02;th=fzero(@(th)th-d.kappa*om*cos(th),[-.5 .5]);end
    rr=target(th,om,d);x0(1:5)=[0;d.V+om*sin(th);om*cos(th);0;om*sin(th)];
    x0(7)=th;x0(9)=rr;
    if mode=="linear"
      x0=d.xe;x0(3)=om;x0(7)=d.kappa*om;
      x0(9)=(d.GT*x0(7)+d.H*d.V*om)/(p.g*p.mr);
    end
   end
   opts=odeset('RelTol',2e-7,'AbsTol',1e-9,'MaxStep',min(.05,cfg.sample_time),'Events',@stop);
   [t,X]=ode45(@rhs,unique([0:cfg.sample_time:tf,tf]),x0,opts);
   U=zeros(numel(t),2); Z=zeros(numel(t),4); den=zeros(numel(t),1);
   for j=1:numel(t),[U(j,:),Z(j,:),den(j)]=control(X(j,:)',d,p,mode);end
   yaw=X(:,3)./cos(X(:,7)); if mode=="linear",yaw=X(:,3);end
   inv=X(:,3)+d.kappa*X(:,7);
   tail=t>t(end)-5;
   row=table(speed,mode,init,t(end),rad2deg(X(end,7)),X(end,9),yaw(end), ...
    max(vecnorm(Z(tail,:),2,2)),max(abs(U(:,1))),max(abs(X(:,9))), ...
    min(abs(den)),inv(end)-inv(1), ...
    'VariableNames',{'speed','model','initial','duration','theta_deg','r_m','yaw_rad_s', ...
    'tail_error','max_F','max_r','min_denominator','invariant_change'});
   row.max_theta_deg=max(abs(rad2deg(X(:,7))));
   row.max_gamma_deg=max(abs(rad2deg(X(:,10))));
   row.max_M2=max(abs(U(:,2)));
   row.completed=t(end)>=tf-1e-6;
   rows=[rows;row]; runs{end+1}=struct('t',t,'X',X,'U',U,'Z',Z,'description',row);
   fprintf('%s %s v=%.1f t=%.1f theta=%.4f yaw=%.5f err=%.3g\n',mode,init,speed,t(end),row.theta_deg,row.yaw_rad_s,row.tail_error);
  end
 end
end
report=struct('metrics',rows,'runs',{runs},'designs',{designs},'parameters',p,'rho',rho,'leanDeg',leanDeg,'Fmax',Fmax,'config',cfg,'output_dir',out);
writetable(rows,fullfile(out,'metrics.csv'));save(fullfile(out,'study.mat'),'report');
plot_equilibrium_balance(report,cfg.show_figures,cfg.save_figures);
    function dx=rhs(~,x)
      u=control(x,d,p,mode);
      if mode=="linear",dx=d.A*(x-d.xe)+d.B*(u-d.ue);dx(8)=dx(8)+d.V;dx(11)=dx(11)+speed;
      else,dx=model_rhs(0,x,u,p);end
    end
    function [val,term,dir]=stop(~,x)
      val=cfg.stop_limits(:)-abs(x([7 10 9]));term=ones(3,1);dir=-ones(3,1);
    end
end
function d=design(speed,p,rho,cfg)
x=zeros(12,1);x(2)=speed/p.R;u=[0;p.BP*x(2)];h=1e-20;A=zeros(12);B=zeros(12,2);
for j=1:12,z=x;z(j)=z(j)+1i*h;A(:,j)=imag(model_rhs(0,z,u,p))/h;end
for j=1:2,z=u;z(j)=z(j)+1i*h;B(:,j)=imag(model_rhs(0,x,z,p))/h;end
V=x(2);k=p.JW1*V/(p.JPz+p.JR+p.JW2);b=p.R*p.mp*p.h;
GT=p.g*(p.R*(p.mp+p.mw+p.mr)+p.h*p.mp);
H=p.JW1+p.R^2*(p.mp+p.mw+p.mr)+b;
c=p.JPz+p.JR+p.JW2;KT=c-p.JPy-p.mp*p.h^2-b-H;
idx=[1 4 9 7 3];Al=A(idx,idx);Bl=B(idx,1);
f=[0 0 0 p.mr*p.g p.mr*p.R*V];Af=Al+Bl*f;
e1=[0 0 1 -GT/(p.g*p.mr) -H*V/(p.g*p.mr)];
E=[e1;e1*Af;0 0 0 1 -k;1 0 0 0 0];
Ac=E*Af*pinv(E);Bc=E*Bl;
assert(norm(E*Af-Ac*E,'fro')<1e-8,'Error coordinates do not close.');
assert(rank(ctrb(Ac,Bc))==4);
Q=diag(cfg.Q);K=lqr(Ac,Bc,Q,rho);
lon=[2 5 10];Kl=lqr(A(lon,lon),B(lon,2),diag(cfg.Q_long),cfg.rho_long);
d=struct('A',A,'B',B,'xe',x,'ue',u,'V',V,'speed',speed,'kappa',k, ...
 'GT',GT,'H',H,'KT',KT,'p',p,'E',E,'K',K,'Kl',Kl,'ff',f,'idx',idx, ...
 'Ac',Ac,'Bc',Bc,'Q',Q,'poles',eig(Ac-Bc*K));
% Exact family tangent of the proposed feedback: E*v=0 and Af*v=0.
v=[0;0;(GT*k+H*V)/(p.g*p.mr);k;1];
assert(norm(E*v)<1e-9&&norm(Af*v)<1e-9);
end
function [u,z,den]=control(x,d,p,mode)
if mode=="linear"
 y=x(d.idx);z=d.E*y;F=d.ff*y-d.K*z;
 M=d.ue(2)-d.Kl*[x(2)-d.V;x(5);x(10)];den=1;
else
 th=x(7);om=x(3)/cos(th);rdes=target(th,om,d);h=1e-20;
 rt=imag(target(th+1i*h,om,d))/h;ro=imag(target(th,om+1i*h,d))/h;
 ff=p.mr*(p.R*(d.V+om*sin(th))*om*cos(th)+p.g*sin(th)-rdes*om^2*cos(th)^2);
 M=d.ue(2)-d.Kl*[x(2)-x(3)*tan(th)-d.V;x(5)-x(3)*tan(th);x(10)];
 M=max(-d.Mmax,min(d.Mmax,M));
 base=model_rhs(0,x,[0;M],p);gain=model_rhs(0,x,[1;M],p)-base;
 om0=base(3)/cos(th)+om*tan(th)*x(1);omF=gain(3)/cos(th);
 er=x(9)-rdes;ed0=p.R*x(1)+x(4)-rt*x(1)-ro*om0;
 z=[er;ed0;th-d.kappa*x(3);x(1)];den=1-d.K(2)*ro*omF;
 if abs(den)<1e-6,error('Implicit derivative feedback became singular.');end
 F=(ff-d.K*z)/den;
 F=max(-d.Fmax,min(d.Fmax,F));z(2)=ed0-ro*omF*F;
end
u=[max(-d.Fmax,min(d.Fmax,F));max(-d.Mmax,min(d.Mmax,M))];
end
function r=target(th,om,d)
p=d.p;
aa=p.mr*om^2*sin(th)*cos(th);
bb=p.g*p.mr*cos(th)-d.V*om*p.R*p.mr*sin(th)+om^2*p.R*p.mr*cos(2*th);
cc=-d.GT*sin(th)-d.V*om*d.H*cos(th)+om^2*d.KT*sin(th)*cos(th);
disc=bb^2-4*aa*cc;
if isreal(th)&&isreal(om)&&disc<0,error('No real local equilibrium root.');end
r=-2*cc/(bb+sqrt(disc));
end

function verify_local_design(d,p)
% Independent central-difference check of the implemented controller.
h=1e-6;JL=zeros(12);JN=zeros(12);
for j=1:12
 xp=d.xe;xm=d.xe;xp(j)=xp(j)+h;xm(j)=xm(j)-h;
 up=control(xp,d,p,"linear");um=control(xm,d,p,"linear");
 JL(:,j)=d.A(:,j)+d.B*(up-um)/(2*h);
 up=control(xp,d,p,"nonlinear");um=control(xm,d,p,"nonlinear");
 JN(:,j)=(model_rhs(0,xp,up,p)-model_rhs(0,xm,um,p))/(2*h);
end
assert(norm(JL-JN,'fro')/max(1,norm(JL,'fro'))<1e-6, ...
 'Implemented nonlinear feedback does not match the design Jacobian.');
assert(max(real(d.poles))<0,'LQR error dynamics are unstable.');
fprintf('Jacobian check passed at v=%.1f m/s.\n',d.speed);
end

function cfg=defaults(cfg)
d=struct('speeds',[0 .5 1.5],'models',["linear","nonlinear"], ...
 'initial_cases',["lean","compatible","equilibrium"], 'Q',[20 2 20 2], ...
 'Q_long',[1 1 5],'rho_long',.1,'torque_limit',15,'sample_time',.05, ...
 'stop_limits',[.65 .65 .4],'show_figures',true,'save_figures',false,'output_dir',"", ...
 'parameter_overrides',struct(),'rod0',0,'rod_rate0',0, ...
 'lean_rate0_deg_s',0,'yaw_rate0_deg_s',0,'body0_deg',0, ...
 'body_rate0_deg_s',0,'heading0_deg',0);
f=fieldnames(d);for i=1:numel(f),if ~isfield(cfg,f{i}),cfg.(f{i})=d.(f{i});end,end
end
