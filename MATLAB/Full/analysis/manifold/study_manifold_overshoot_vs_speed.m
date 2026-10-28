function study = study_manifold_overshoot_vs_speed(theta0_deg,speeds,opt)
%STUDY_MANIFOLD_OVERSHOOT_VS_SPEED Peak response: expm, quadratic, full plant.
% Use run_manifold_overshoot_study from MATLAB/Full as the public entry point.
% The quadratic model is a degree-two perturbation expansion, evaluated by
% a 54-by-54 matrix exponential. It is not a polynomial fit to simulations.
if nargin<1 || isempty(theta0_deg), theta0_deg=[0.25 1 5]; end
if nargin<2 || isempty(speeds), speeds=[1.5:0.25:6 7 8 10]; end
if nargin<3, opt=struct(); end
opt=defaults(opt,struct('target_deg',0,'horizon',25,'dt',0.005, ...
    'pivot_speed',2.5,'speed_step',0.02,'make_plots',true,'validate',true));
validateattributes(theta0_deg,{'numeric'},{'vector','real','finite','nonzero'});
validateattributes(speeds,{'numeric'},{'vector','real','finite','positive'});
theta0_deg=theta0_deg(:)'; speeds=sort(unique(speeds(:)'));
root=fileparts(fileparts(fileparts(mfilename('fullpath'))));
out=fullfile(root,'results','manifold_overshoot_vs_speed');
if ~isfolder(out), mkdir(out); end
runs=cell(numel(theta0_deg),numel(speeds));
rows={}; acceleration_rows={};
for a=1:numel(theta0_deg)
    for k=1:numel(speeds)
        d=manifold_local_design(theta0_deg(a),speeds(k),opt.target_deg);
        [sol,U,completed]=simulate_plant(d,opt,d.x0);
        t=linspace(0,sol.x(end),ceil(sol.x(end)/opt.dt)+1);
        nonlinear=@(s)plant_output(sol,d,s);
        [yn,dyn]=nonlinear(t);
        H=field_hessian(d,2e-4);
        [L,z0]=quadratic_lift(d.J,H,d.e0);
        C=zeros(2,9); C(1,7)=1; C(2,8)=1;
        offset=d.xe([7 9]);
        linear=@(s)matrix_output(d.J,d.e0,C,offset,s);
        quadratic=@(s)matrix_output(L,z0,[C zeros(2,45)],offset,s);
        [yl,dyl]=sample_matrix(d.J,d.e0,C,offset,t);
        [yq,dyq]=sample_matrix(L,z0,[C zeros(2,45)],offset,t);
        labels={'nonlinear','linear','quadratic'};
        ys={yn,yl,yq}; dys={dyn,dyl,dyq}; funs={nonlinear,linear,quadratic};
        for j=1:3
            m=peak_metrics(t,ys{j},dys{j},funs{j},d.x0([7 9]),offset);
            row=struct('theta0_deg',theta0_deg(a),'speed_m_s',speeds(k), ...
                'model',string(labels{j}),'completed',completed, ...
                'duration_s',t(end),'force_saturated',any(abs(U(1,:))>=d.force_limit-1e-8), ...
                'torque_saturated',any(abs(U(2,:))>=15-1e-8), ...
                'max_theta_trace_error_deg',max(abs(rad2deg(ys{j}(1,:)-yn(1,:)))), ...
                'max_r_trace_error_m',max(abs(ys{j}(2,:)-yn(2,:))));
            names=fieldnames(m);
            for jn=1:numel(names), row.(names{jn})=m.(names{jn}); end
            rows{end+1}=row; %#ok<AGROW>
        end
        u0=clamp(d.ue-d.K*d.e0,d);
        f0=model_rhs(0,d.x0,u0,d.p);
        p=d.p;
        I=p.JPx+p.JR+p.JW2+p.R^2*(p.mp+p.mw) ...
            +2*p.R*p.h*p.mp+p.h^2*p.mp;
        G=p.g*(p.R*(p.mp+p.mw)+p.h*p.mp);
        accel=(p.R*u0(1)+G*sin(deg2rad(theta0_deg(a))))/I;
        rod_accel=p.R*accel+u0(1)/p.mr-p.g*sin(deg2rad(theta0_deg(a)));
        jerk=complex_jacobian(d.field,d.e0)*d.field(d.e0);
        st=sign(theta0_deg(a)-opt.target_deg); if st==0, st=1; end
        aa=st*accel; bb=st*jerk(1);
        cubic_time=NaN; cubic_peak=NaN;
        if aa>0 && bb<0
            cubic_time=-2*aa/bb; cubic_peak=rad2deg(2*aa^3/(3*bb^2));
        end
        X=deval(sol,t);
        acceleration_rows{end+1}=struct('theta0_deg',theta0_deg(a), ...
            'speed_m_s',speeds(k),'force0_N',u0(1), ...
            'theta_accel_deg_s2',rad2deg(accel),'r_accel_m_s2',rod_accel, ...
            'formula_error',max(abs([accel-f0(1),rod_accel-p.R*f0(1)-f0(4)])), ...
            'theta_jerk_deg_s3',rad2deg(jerk(1)), ...
            'r_jerk_m_s3',p.R*jerk(1)+jerk(4), ...
            'cubic_peak_time_s',cubic_time,'cubic_outward_overshoot_deg',cubic_peak, ...
            'raw_ctrb_rank',d.raw_ctrb_rank,'neutral_residual',d.neutral_residual, ...
            'max_real_transverse_pole',max(real(d.transverse_poles)), ...
            'max_heading_error_rad',max(abs(X(6,:)-d.omega*t)), ...
            'omega_rad_s',d.omega,'r_reference_m',offset(2)); %#ok<AGROW>
        runs{a,k}=struct('t',t,'nonlinear',yn,'linear',yl,'quadratic',yq, ...
            'reference',offset,'completed',completed, ...
            'speed_range_m_s',d.p.R*[min(X(2,:)),max(X(2,:))], ...
            'max_heading_error_rad',max(abs(X(6,:)-d.omega*t)), ...
            'J',d.J,'K',d.K,'e0',d.e0);
        fprintf('theta0=%g deg, v=%g: completed=%d, peak theta=%.5g deg, max |r|=%.5g m\n', ...
            theta0_deg(a),speeds(k),completed,max(rad2deg(yn(1,:))),max(abs(yn(2,:))));
    end
end
study=struct('theta0_deg',theta0_deg,'speeds',speeds,'options',opt, ...
    'metrics',struct2table([rows{:}]),'initial_acceleration',struct2table([acceleration_rows{:}]), ...
    'runs',{runs});
[study.speed_series,study.local_peak_polynomials]=speed_series_check(theta0_deg,opt);
if opt.validate
    study.amplitude_validation=amplitude_check(opt);
else
    study.amplitude_validation=table();
end
writetable(study.metrics,fullfile(out,'overshoot_metrics.csv'));
writetable(study.initial_acceleration,fullfile(out,'initial_acceleration.csv'));
writetable(study.speed_series,fullfile(out,'speed_series_validation.csv'));
writetable(study.local_peak_polynomials,fullfile(out,'local_peak_polynomials.csv'));
if opt.validate
    writetable(study.amplitude_validation,fullfile(out,'amplitude_series_validation.csv'));
end
save(fullfile(out,'study.mat'),'study');
if opt.make_plots, make_figures(study,out); end
fprintf('Saved overshoot study: %s\n',out);
end

function [sol,U,completed]=simulate_plant(d,opt,x0)
rhs=@(t,x)model_rhs(t,x,control(d,t,x),d.p);
settings=odeset('RelTol',1e-9,'AbsTol',1e-11,'MaxStep',0.01, ...
    'Events',@(~,x)guard(x,d.p));
sol=ode45(rhs,[0 opt.horizon],x0,settings);
tt=linspace(0,sol.x(end),ceil(sol.x(end)/opt.dt)+1);
X=deval(sol,tt); U=zeros(2,numel(tt));
for k=1:numel(tt), U(:,k)=control(d,tt(k),X(:,k)); end
completed=abs(sol.x(end)-opt.horizon)<1e-7;
end

function u=control(d,t,x)
e=x(d.idx)-d.xe(d.idx)-t*d.rate(d.idx);
e(6)=atan2(sin(e(6)),cos(e(6)));
u=clamp(d.ue-d.K*e,d);
end

function u=clamp(u,d)
limit=[d.force_limit;d.torque_limit]; u=max(-limit,min(limit,u));
end

function [value,terminal,direction]=guard(x,p)
value=[pi/4-abs(x(7));rcond(mass_matrix(x,p))-1e-12];
terminal=[1;1]; direction=[-1;-1];
end

function [y,dy]=plant_output(sol,d,t)
X=deval(sol,t); y=X([7 9],:);
dy=[X(1,:);d.p.R*X(1,:)+X(4,:)];
end

function H=field_hessian(d,step)
n=9; H=zeros(n,n,n);
for k=1:n
    h=step*max(1,abs(d.xe(d.idx(k))));
    e=zeros(n,1); e(k)=h;
    H(:,:,k)=(complex_jacobian(d.field,e)-complex_jacobian(d.field,-e))/(2*h);
end
H=(H+permute(H,[1 3 2]))/2;
end

function J=complex_jacobian(fun,e)
n=numel(e); J=zeros(n); h=1e-20;
for k=1:n
    z=e; z(k)=z(k)+1i*h; J(:,k)=imag(fun(z))/h;
end
end

function [L,z0]=quadratic_lift(J,H,e0)
n=numel(e0); [ii,jj]=find(triu(true(n))); m=numel(ii);
lookup=zeros(n);
for p=1:m, lookup(ii(p),jj(p))=p; lookup(jj(p),ii(p))=p; end
D=zeros(n,m); Z=zeros(m); monomial=zeros(m,1);
for p=1:m
    i=ii(p); j=jj(p); monomial(p)=e0(i)*e0(j);
    D(:,p)=H(:,i,j); if i==j, D(:,p)=D(:,p)/2; end
    for k=1:n
        Z(p,lookup(k,j))=Z(p,lookup(k,j))+J(i,k);
        Z(p,lookup(i,k))=Z(p,lookup(i,k))+J(j,k);
    end
end
L=[J D;zeros(m,n) Z]; z0=[e0;monomial];
end

function [y,dy]=sample_matrix(M,z0,C,offset,t)
% expm at each uniform step is the exact linear transition, up to roundoff.
P=expm(M*(t(2)-t(1))); z=z0;
y=zeros(2,numel(t)); dy=y; CM=C*M;
for k=1:numel(t)
    y(:,k)=offset+C*z; dy(:,k)=CM*z; z=P*z;
end
end

function [y,dy]=matrix_output(M,z0,C,offset,t)
z=expm(M*t)*z0; y=offset+C*z; dy=C*M*z;
end

function m=peak_metrics(t,y,dy,fun,y0,ref)
% Refine stationary points using the continuous derivative, not sample maxima.
all_times=cell(1,2); all_values=cell(1,2);
first_peak=NaN(1,2); first_time=first_peak;
st=sign(y0(1)-ref(1)); if st==0, st=1; end
sr=sign(ref(2)-y0(2)); if sr==0, sr=1; end
orientation=[st sr];
for j=1:2
    change=find(dy(j,1:end-1).*dy(j,2:end)<0 & ...
        max(abs(dy(j,1:end-1)),abs(dy(j,2:end)))>1e-9);
    roots_t=zeros(1,numel(change));
    for k=1:numel(change)
        bracket=t(change(k):change(k)+1);
        roots_t(k)=fzero(@(s)derivative(fun,s,j),bracket);
        if isnan(first_time(j)) && orientation(j)*dy(j,change(k))>0
            first_time(j)=roots_t(k); val=fun(roots_t(k)); first_peak(j)=val(j);
        end
    end
    times=[t(1),roots_t,t(end)]; values=zeros(size(times));
    for k=1:numel(times)
        val=fun(times(k)); values(k)=val(j);
    end
    all_times{j}=times; all_values{j}=values;
end
[outward,it]=max(st*(all_values{1}-y0(1)));
[~,ir]=max(sr*(all_values{2}-ref(2)));
first_overshoot=NaN;
if isfinite(first_peak(1)), first_overshoot=rad2deg(max(0,st*(first_peak(1)-y0(1)))); end
m=struct('theta_peak_deg',rad2deg(all_values{1}(it)), ...
    'theta_peak_time_s',all_times{1}(it), ...
    'theta_outward_overshoot_deg',rad2deg(max(0,outward)), ...
    'theta_first_peak_deg',rad2deg(first_peak(1)), ...
    'theta_first_peak_time_s',first_time(1), ...
    'theta_first_outward_overshoot_deg',first_overshoot, ...
    'theta_target_crossing_deg',rad2deg(max(0,max(-st*(all_values{1}-ref(1))))), ...
    'theta_terminal_deg',rad2deg(y(1,end)), ...
    'r_peak_m',all_values{2}(ir),'r_peak_time_s',all_times{2}(ir), ...
    'r_first_peak_m',first_peak(2),'r_first_peak_time_s',first_time(2), ...
    'r_command_overshoot_m',max(0,max(sr*(all_values{2}-ref(2)))), ...
    'r_absmax_m',max(abs(all_values{2})), ...
    'r_terminal_error_m',y(2,end)-ref(2), ...
    'r_peak_above_terminal_m',max(0,max(sr*(all_values{2}-y(2,end)))));
end

function v=derivative(fun,t,j)
[~,dy]=fun(t); v=dy(j);
end

function [tab,coefficients]=speed_series_check(angles,opt)
rows={}; coefficient_rows={}; v0=opt.pivot_speed; h=opt.speed_step;
for a=1:numel(angles)
    d0=manifold_local_design(angles(a),v0,opt.target_deg);
    dm=manifold_local_design(angles(a),v0-h,opt.target_deg);
    dp=manifold_local_design(angles(a),v0+h,opt.target_deg);
    J0=d0.J; J1=(dp.J-dm.J)/(2*h); J2=(dp.J-2*J0+dm.J)/(2*h^2);
    e1=(dp.e0-dm.e0)/(2*h); e2=(dp.e0-2*d0.e0+dm.e0)/(2*h^2);
    r0=d0.xe([7 9]); r1=(dp.xe([7 9])-dm.xe([7 9]))/(2*h);
    r2=(dp.xe([7 9])-2*r0+dm.xe([7 9]))/(2*h^2);
    M=[J0 zeros(9,18);J1 J0 zeros(9);J2 J1 J0];
    z0=[d0.e0;e1;e2]; t=0:opt.dt:opt.horizon;
    C=zeros(2,9); C(1,7)=1; C(2,8)=1;
    for dv=[-0.25 -0.125 0 0.125 0.25]
        d=manifold_local_design(angles(a),v0+dv,opt.target_deg);
        offset=r0+dv*r1+dv^2*r2; CC=[C dv*C dv^2*C];
        [ys,dys]=sample_matrix(M,z0,CC,offset,t);
        [ye,dye]=sample_matrix(d.J,d.e0,C,d.xe([7 9]),t);
        fs=@(s)matrix_output(M,z0,CC,offset,s);
        fe=@(s)matrix_output(d.J,d.e0,C,d.xe([7 9]),s);
        ms=peak_metrics(t,ys,dys,fs,d.x0([7 9]),offset);
        me=peak_metrics(t,ye,dye,fe,d.x0([7 9]),d.xe([7 9]));
        if dv==0, center=me; end
        rows{end+1}=struct('theta0_deg',angles(a),'speed_m_s',v0+dv, ...
            'exact_theta_peak_deg',me.theta_peak_deg, ...
            'speed_series_theta_peak_deg',ms.theta_peak_deg, ...
            'exact_r_absmax_m',me.r_absmax_m,'speed_series_r_absmax_m',ms.r_absmax_m, ...
            'max_theta_trace_error_deg',max(abs(rad2deg(ys(1,:)-ye(1,:)))), ...
            'max_r_trace_error_m',max(abs(ys(2,:)-ye(2,:)))); %#ok<AGROW>
    end
    [p0,p1,p2,tp1]=peak_speed_coefficients(M,z0,C,r0,r1,r2, ...
        J0,J1,center.theta_first_peak_time_s,1);
    [q0,q1,q2,tr1]=peak_speed_coefficients(M,z0,C,r0,r1,r2, ...
        J0,J1,center.r_peak_time_s,2);
    st=sign(angles(a)-opt.target_deg); if st==0, st=1; end
    sr=sign(q0); if sr==0, sr=1; end
    coefficient_rows{end+1}=struct('theta0_deg',angles(a),'pivot_speed_m_s',v0, ...
        'theta_overshoot_c0_deg',st*(rad2deg(p0)-angles(a)), ...
        'theta_overshoot_c1_deg_per_m_s',st*rad2deg(p1), ...
        'theta_overshoot_c2_deg_per_m_s_squared',st*rad2deg(p2), ...
        'theta_peak_time_derivative',tp1, ...
        'r_abs_peak_c0_m',sr*q0,'r_abs_peak_c1_m_per_m_s',sr*q1, ...
        'r_abs_peak_c2_m_per_m_s_squared',sr*q2, ...
        'r_peak_time_derivative',tr1); %#ok<AGROW>
end
tab=struct2table([rows{:}]);
coefficients=struct2table([coefficient_rows{:}]);
end

function [p0,p1,p2,time_slope]=peak_speed_coefficients(M,z0,C,r0,r1,r2,J,J1,tp,j)
if ~isfinite(tp)
    p0=NaN; p1=NaN; p2=NaN; time_slope=NaN; return
end
eta=expm(M*tp)*z0; e=eta(1:9); ev=eta(10:18); ev2=eta(19:27);
c=C(j,:); mixed=c*(J1*e+J*ev); curvature=c*J^2*e;
assert(abs(curvature)>1e-12,'Degenerate peak: speed polynomial is not valid.');
p0=r0(j)+c*e; p1=r1(j)+c*ev;
p2=r2(j)+c*ev2-mixed^2/(2*curvature);
time_slope=-mixed/curvature;
end

function tab=amplitude_check(opt)
% Fixed controller and equilibrium: a real test of O(epsilon^2/epsilon^3).
d=manifold_local_design(5,opt.pivot_speed,opt.target_deg);
H=field_hessian(d,2e-4); Hhalf=field_hessian(d,1e-4);
hessian_relative_change=norm(H(:)-Hhalf(:))/max(norm(H(:)),eps);
Jcheck=complex_jacobian(d.field,zeros(9,1));
assert(norm(Jcheck-d.J,'fro')<1e-8,'Closed-loop Jacobian check failed.');
rows={}; C=zeros(2,9); C(1,7)=1; C(2,8)=1;
for epsilon=[1 0.5 0.25 0.125]
    x0=d.xe; x0(d.idx)=d.xe(d.idx)+epsilon*d.e0;
    [sol,U,completed]=simulate_plant(d,opt,x0);
    assert(completed && all(abs(U(1,:))<30) && all(abs(U(2,:))<15), ...
        'Amplitude validation left the smooth unsaturated regime.');
    t=linspace(0,sol.x(end),ceil(sol.x(end)/opt.dt)+1);
    yn=plant_output(sol,d,t);
    yl=sample_matrix(d.J,epsilon*d.e0,C,d.xe([7 9]),t);
    [L,z0]=quadratic_lift(d.J,H,epsilon*d.e0);
    yq=sample_matrix(L,z0,[C zeros(2,45)],d.xe([7 9]),t);
    rows{end+1}=struct('epsilon',epsilon, ...
        'linear_theta_error_deg',max(abs(rad2deg(yl(1,:)-yn(1,:)))), ...
        'quadratic_theta_error_deg',max(abs(rad2deg(yq(1,:)-yn(1,:)))), ...
        'linear_r_error_m',max(abs(yl(2,:)-yn(2,:))), ...
        'quadratic_r_error_m',max(abs(yq(2,:)-yn(2,:))), ...
        'hessian_relative_change',hessian_relative_change); %#ok<AGROW>
end
tab=struct2table([rows{:}]);
end

function make_figures(s,out)
angles=s.theta0_deg; models=["nonlinear","linear","quadratic"];
styles={'-','--',':'}; colors=lines(3); n=numel(angles);
f=figure('Color','w','Position',[60 60 1200 300*n]);
tl=tiledlayout(f,n,2,'TileSpacing','compact','Padding','compact');
for a=1:n
    for output=1:2
        ax=nexttile(tl); hold(ax,'on'); grid(ax,'on');
        for j=1:3
            use=s.metrics.theta0_deg==angles(a) & s.metrics.model==models(j) & s.metrics.completed;
            q=s.metrics(use,:);
            if output==1, y=q.theta_first_outward_overshoot_deg; else, y=q.r_command_overshoot_m; end
            plot(ax,q.speed_m_s,y,styles{j},'Color',colors(j,:),'LineWidth',1.4);
        end
        xlabel(ax,'Design speed v (m/s)');
        if output==1, ylabel(ax,'First outward theta overshoot (deg)'); else, ylabel(ax,'r beyond reference (m)'); end
        title(ax,sprintf('Initial theta = %g deg',angles(a)));
        if a==1 && output==1, legend(ax,models,'Location','best'); end
    end
end
title(tl,'Full model versus matrix-exponential predictions (completed trajectories)');
exportgraphics(f,fullfile(out,'01_overshoot_vs_speed.png'),'Resolution',180);

f=figure('Color','w','Position',[80 60 1200 900]);
tl=tiledlayout(f,3,2,'TileSpacing','compact','Padding','compact');
selected=unique(arrayfun(@(v)find(abs(s.speeds-v)==min(abs(s.speeds-v)),1),[1.5 2.5 5]));
for row=1:numel(selected)
    k=selected(row); q=s.runs{end,k};
    for output=1:2
        ax=nexttile(tl); hold(ax,'on'); grid(ax,'on');
        for j=1:3
            y=q.(models(j)); if output==1, y=rad2deg(y); end
            plot(ax,q.t,y(output,:),styles{j},'Color',colors(j,:),'LineWidth',1.3);
        end
        ref=q.reference(output); if output==1, ref=rad2deg(ref); end
        yline(ax,ref,'k:','HandleVisibility','off'); xlim(ax,[0 min(4,q.t(end))]);
        xlabel(ax,'Time (s)');
        if output==1, ylabel(ax,'theta (deg)'); else, ylabel(ax,'r (m)'); end
        title(ax,sprintf('theta0=%g deg, v=%g m/s',angles(end),s.speeds(k)));
        if row==1 && output==1, legend(ax,models,'Location','best'); end
    end
end
exportgraphics(f,fullfile(out,'02_transient_comparison.png'),'Resolution',180);

f=figure('Color','w','Position',[100 80 1200 500]); tl=tiledlayout(f,1,2);
for output=1:2
    ax=nexttile(tl); hold(ax,'on'); grid(ax,'on');
    for a=1:n
        q=s.initial_acceleration(s.initial_acceleration.theta0_deg==angles(a),:);
        if output==1, y=q.theta_accel_deg_s2; else, y=q.r_accel_m_s2; end
        plot(ax,q.speed_m_s,y,'-o','DisplayName',sprintf('theta0=%g deg',angles(a)));
    end
    yline(ax,0,'k--','HandleVisibility','off'); xlabel(ax,'Design speed v (m/s)');
    if output==1, ylabel(ax,'Initial theta acceleration (deg/s^2)'); else, ylabel(ax,'Initial r acceleration (m/s^2)'); end
    legend(ax,'Location','best');
end
exportgraphics(f,fullfile(out,'03_exact_initial_acceleration.png'),'Resolution',180);

f=figure('Color','w','Position',[100 60 1200 300*n]); tl=tiledlayout(f,n,2);
for a=1:n
    q=s.speed_series(s.speed_series.theta0_deg==angles(a),:);
    for output=1:2
        ax=nexttile(tl); grid(ax,'on'); hold(ax,'on');
        if output==1
            plot(ax,q.speed_m_s,q.exact_theta_peak_deg,'-o',q.speed_m_s,q.speed_series_theta_peak_deg,'--s');
            ylabel(ax,'Theta peak (deg)');
        else
            plot(ax,q.speed_m_s,q.exact_r_absmax_m,'-o',q.speed_m_s,q.speed_series_r_absmax_m,'--s');
            ylabel(ax,'Maximum |r| (m)');
        end
        xlabel(ax,'Design speed v (m/s)'); title(ax,sprintf('Initial theta=%g deg',angles(a)));
        if a==1 && output==1, legend(ax,'Exact linear expm','Speed Taylor through order 2'); end
    end
end
exportgraphics(f,fullfile(out,'04_speed_jacobian_series.png'),'Resolution',180);
end

function opt=defaults(opt,values)
names=fieldnames(values);
for k=1:numel(names)
    if ~isfield(opt,names{k}), opt.(names{k})=values.(names{k}); end
end
end
