function report=straight_test(speeds,angles)
% Compare the original matched turn with a matched, tilted straight orbit.
if nargin<1, speeds=[1.5 2.5]; end
if nargin<2, angles=[0.25 1 5]; end
validateattributes(speeds,{'numeric'},{'vector','positive','finite','real','nonempty'});
validateattributes(angles,{'numeric'},{'vector','finite','real','nonempty'});
root=fileparts(fileparts(fileparts(mfilename('fullpath'))));
addpath(fullfile(root,'model'),fullfile(root,'config'));
out=fullfile(root,'results','gabor_straight_path');
if ~isfolder(out), mkdir(out); end
rows={}; runs={};
for v=speeds(:)'
    fig=figure('Visible','off','Color','w','Position',[50 50 1300 850]);
    tiledlayout(numel(angles),3,'TileSpacing','compact');
    for a=1:numel(angles)
        base=manifold_local_design(angles(a),v);
        straight=straight_design(base);
        for mode=1:2
            if mode==1, d=base; name='matched turn'; else, d=straight; name='matched straight'; end
            opts=odeset('RelTol',1e-9,'AbsTol',1e-11,'MaxStep',0.02,'Events',@guard);
            sol=ode45(@(t,x)model_rhs(t,x,control(t,x,d),d.p),[0 15],d.x0,opts);
            t=linspace(0,sol.x(end),2001); X=deval(sol,t); U=zeros(2,numel(t));
            for k=1:numel(t), U(:,k)=control(t(k),X(:,k),d); end
            yaw=X(3,:)./cos(X(7,:));
            rows{end+1}=struct('theta0_deg',angles(a),'speed',v,'controller',string(name), ...
                'duration',t(end),'completed',abs(t(end)-40)<1e-6, ...
                'tail_max_abs_yaw',max(abs(yaw(t>=max(0,t(end)-5)))), ...
                'theta_ref_deg',rad2deg(d.xe(7)), ...
                'theta_final_deg',rad2deg(X(7,end)),'theta_peak_deg',max(rad2deg(X(7,:))), ...
                'yaw_ref',d.omega,'yaw_final',yaw(end),'heading_final_deg',rad2deg(X(6,end)), ...
                'y_final_m',X(12,end),'r_ref',d.xe(9), ...
                'r_final',X(9,end),'max_abs_r',max(abs(X(9,:))), ...
                'max_force',max(abs(U(1,:))),'max_torque',max(abs(U(2,:))), ...
                'neutral_match',d.w'*(d.x0(d.idx)-d.xe(d.idx))); %#ok<AGROW>
            runs{end+1}=struct('t',t,'X',X,'U',U,'description',rows{end}); %#ok<AGROW>
            nexttile((a-1)*3+1); hold on; plot(t,rad2deg(X(7,:)),'DisplayName',name); grid on;
            xlabel('Time (s)'); ylabel('\theta (deg)'); title(sprintf('Initial lean = %g deg',angles(a))); legend('Location','best');
            nexttile((a-1)*3+2); hold on; plot(t,yaw,'DisplayName',name); grid on;
            xlabel('Time (s)'); ylabel('Yaw rate (rad/s)');
            nexttile((a-1)*3+3); hold on; plot(X(11,:),X(12,:),'DisplayName',name); grid on; axis equal;
            xlabel('x_G (m)'); ylabel('y_G (m)');
        end
    end
    sgtitle(sprintf('Full nonlinear model: v_0 = %g m/s',v));
    exportgraphics(fig,fullfile(out,sprintf('comparison_v_%g.png',v)),'Resolution',170); close(fig);
end
report=struct('metrics',struct2table([rows{:}]),'runs',{runs});
writetable(report.metrics,fullfile(out,'metrics.csv'));
save(fullfile(out,'study.mat'),'report'); disp(report.metrics);

    function [value,terminal,direction]=guard(~,x)
        value=[pi/4-abs(x(7));rcond(mass_matrix(x,base.p))-1e-12];
        terminal=[1;1]; direction=[-1;-1];
    end
end

function d=straight_design(d)
% Select theta rather than yaw rate to satisfy the same local neutral match.
p=d.p; idx=d.idx;
theta=fzero(@match,[-0.3 0.3]);
[x,u,A,B,w]=candidate(theta);
T=null(w'); Q=diag([2 1 1 2 1 0.5 20 20 5]);
K=lqr(T'*A*T,T'*B,T'*Q*T,diag([0.1 0.1]))*T';
assert(max(real(eig(T'*(A-B*K)*T)))<0);
d.xe=x; d.ue=u; d.A=A; d.B=B; d.K=K; d.w=w; d.omega=0;
d.rate=zeros(12,1); d.rate(8)=d.speed/p.R; d.rate(11)=d.speed;
    function z=match(theta)
        [x,~,~,~,w]=candidate(theta); z=w'*(d.x0(idx)-x(idx));
    end
    function [x,u,A,B,w]=candidate(theta)
        x=zeros(12,1); x(2)=d.speed/p.R; x(7)=theta;
        x(9)=(p.R*(p.mr+p.mp+p.mw)+p.h*p.mp)/p.mr*tan(theta);
        u=[p.mr*p.g*sin(theta);0];
        f=model_rhs(0,x,u,p); assert(norm(f(1:5),inf)<1e-9);
        A=zeros(9); B=zeros(9,2); h=1e-20;
        for j=1:9
            z=x; z(idx(j))=z(idx(j))+1i*h; f=model_rhs(0,z,u,p); A(:,j)=imag(f(idx))/h;
        end
        for j=1:2
            z=u; z(j)=z(j)+1i*h; f=model_rhs(0,x,z,p); B(:,j)=imag(f(idx))/h;
        end
        stack=[A';B']; stack=stack./max(vecnorm(stack,2,2),1);
        [~,S,V]=svd(stack,0); w=V(:,end); if w(7)<0, w=-w; end
        assert(S(end,end)<1e-10 && max(norm(w'*A,inf),norm(w'*B,inf))<1e-8);
    end
end

function u=control(t,x,d)
e=x(d.idx)-d.xe(d.idx)-t*d.rate(d.idx);
e(6)=atan2(sin(e(6)),cos(e(6)));
u=d.ue-d.K*e; u=max(-[30;15],min([30;15],u));
end
