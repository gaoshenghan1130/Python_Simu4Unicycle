function study = run_manifold_hardware_tuning(speed,angles)
% Reproducible LQR search with saturated full nonlinear validation.
% Feasibility is specific to the tested initial lean and speed, not global.
if nargin<1, speed=1.2; end
if nargin<2, angles=[1 1.5]; end
validateattributes(speed,{'numeric'},{'scalar','real','finite','positive'});
validateattributes(angles,{'numeric'},{'vector','real','finite','positive'});
root=fileparts(mfilename('fullpath'));
addpath(fullfile(root,'analysis','manifold'));
cases=[angles(:) repmat(speed,numel(angles),1)];
designs=cell(size(cases,1),1);
for j=1:numel(designs)
    designs{j}=manifold_local_design(cases(j,1),cases(j,2));
end
base=[2 1 1 2 1 0.5 20 20 5];
rows={}; best=Inf; bestQ=[]; bestR=[];
for qt=[1 10 100]
for qr=[1 10 100]
    for rf=[0.01 0.1 1]
        q=base; q(8)=base(8)*qr; q(7)=base(7)*qt;
        Q=diag(q); R=diag([rf 0.1]);
        score=0; feasible=true;
        for j=1:numel(designs)
            d=designs{j};
            K=lqr(d.T'*d.A*d.T,d.T'*d.B,d.T'*Q*d.T,R)*d.T';
            m=simulate(d,K,0.01);
            row=struct('theta_weight',q(7),'rod_weight',q(8),'force_weight',rf, ...
                'theta0_deg',cases(j,1),'speed_m_s',cases(j,2));
            fields=fieldnames(m);
            for k=1:numel(fields), row.(fields{k})=m.(fields{k}); end
            rows{end+1}=row; %#ok<AGROW>
            feasible=feasible && m.passed;
            score=max(score,m.peak_r_m);
        end
        fprintf('Qt=%g Qr=%g Rf=%g worst r=%.5f m passed=%d\n',q(7),q(8),rf,score,feasible);
        if feasible && score<best, best=score; bestQ=Q; bestR=R; end
    end
end
end
study.cases=cases; study.search=struct2table([rows{:}]);
study.Q=bestQ; study.R=bestR;
study.validation=struct([]);
if ~isempty(bestQ)
    % Independent finer-step verification includes both lean signs and the
    % larger lean disturbances and nearby operating speeds.
    validation_cases=[cases;-cases(:,1) cases(:,2);2 speed;3 speed;5 speed;max(angles) max(speed-0.1,speed/2);max(angles) speed+0.1];
    for j=1:size(validation_cases,1)
        d=manifold_local_design(validation_cases(j,1),validation_cases(j,2));
        K=lqr(d.T'*d.A*d.T,d.T'*d.B,d.T'*bestQ*d.T,bestR)*d.T';
        tuned=simulate(d,K,0.005);
        baseline=simulate(d,d.K,0.005);
        entry=struct('theta0_deg',validation_cases(j,1), ...
            'speed_m_s',validation_cases(j,2),'baseline',baseline,'tuned',tuned,'K',K);
        if j==1, study.validation=entry; else, study.validation(j)=entry; end
    end
end
out=fullfile(root,'results',sprintf('manifold_hardware_tuning_v%g',speed));
if ~isfolder(out), mkdir(out); end
writetable(study.search,fullfile(out,'search.csv'));
save(fullfile(out,'study.mat'),'study');
disp(bestQ); disp(bestR);
end

function m=simulate(d,K,step)
limit=[27.5;15];
rhs=@(t,x) model_rhs(t,x,control(t,x),d.p);
opts=odeset('RelTol',1e-8,'AbsTol',1e-10,'MaxStep',step,'Events',@guard);
[t,X]=ode45(rhs,(0:step:30)',d.x0,opts);
raw=zeros(numel(t),2);
for k=1:numel(t), [~,raw(k,:)]=control(t(k),X(k,:)'); end
tail=t>=t(end)-5;
dx=rhs(t(end),X(end,:)');
m.peak_r_m=max(abs(X(:,9)));
m.peak_raw_force_N=max(abs(raw(:,1)));
m.peak_raw_torque_Nm=max(abs(raw(:,2)));
m.tail_force_N=max(abs(raw(tail,1)));
m.final_theta_deg=rad2deg(X(end,7));
m.final_rate=norm(dx([1:5 7 9 10]),inf);
m.completed=abs(t(end)-30)<1e-8;
m.passed=m.completed && m.peak_r_m<0.075 && ...
    m.peak_raw_force_N<=27.5 && m.peak_raw_torque_Nm<=15 && ...
    m.tail_force_N<=9.2 && m.final_rate<1e-4;
m.exact_target_reached=m.completed && m.final_rate<1e-4 && abs(m.final_theta_deg)<0.001;
    function [u,rawrow]=control(t,x)
        e=x(d.idx)-d.xe(d.idx)-t*d.rate(d.idx);
        e(6)=atan2(sin(e(6)),cos(e(6)));
        v=d.ue-K*e; rawrow=v'; u=max(-limit,min(limit,v));
    end
end

function [value,terminal,direction]=guard(~,x)
value=[pi/4-abs(x(7));0.15-abs(x(9))];
terminal=[1;1]; direction=[-1;-1];
end
