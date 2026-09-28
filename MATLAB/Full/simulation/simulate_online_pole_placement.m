function result = simulate_online_pole_placement( ...
    p,c,d,s,target_speed,min_design_speed,schedule)
% Fixed-step forward-Euler simulation, matching timeConstantSimu.
x0=initial_state(s,p);
assert(abs(s.theta0)<s.tilt_limit,'Initial tilt exceeds stop limit.');
assert(abs(target_speed)>min_design_speed, ...
    'Target speed must exceed the minimum rolling-design speed.');
if isfield(s,'fixed_step')
    dt=s.fixed_step;
else
    dt=s.max_step;
end
validateattributes(dt,{'numeric'},{'real','finite','positive','scalar'});

control=@(t,x) online_pole_controller_output( ...
    t,x,p,c,d,target_speed,min_design_speed,schedule);
num_steps=ceil(s.t_end/dt);
t=(0:num_steps)'*dt;
t(end)=s.t_end;
X=zeros(num_steps+1,12);
U=zeros(num_steps+1,2);
scheduled_speed=zeros(num_steps+1,1);
gain_inf_norm=zeros(num_steps+1,1);
X(1,:)=x0';
te=[]; xe=[]; ie=[];

last_sample=num_steps+1;
for sample=1:num_steps
    step_dt=t(sample+1)-t(sample);
    x=X(sample,:)';
    [u,scheduled_speed(sample),K]=control(t(sample),x);
    U(sample,:)=u';
    gain_inf_norm(sample)=norm(K,inf);
    X(sample+1,:)=(x+step_dt*model_rhs(t(sample),x,u,p))';

    if abs(X(sample+1,7))>=s.tilt_limit
        last_sample=sample+1; te=t(last_sample); xe=X(last_sample,:); ie=1;
        break;
    end
    if abs(p.R*X(sample+1,2))<=min_design_speed
        last_sample=sample+1; te=t(last_sample); xe=X(last_sample,:); ie=2;
        break;
    end
end

t=t(1:last_sample);
X=X(1:last_sample,:);
U=U(1:last_sample,:);
scheduled_speed=scheduled_speed(1:last_sample);
gain_inf_norm=gain_inf_norm(1:last_sample);
[u,scheduled_speed(end),K]=control(t(end),X(end,:)');
U(end,:)=u';
gain_inf_norm(end)=norm(K,inf);

result.t=t;
result.X=X;
result.U=U;
result.event_time=te;
result.event_state=xe;
result.event_index=ie;
result.parameters=p;
result.controller=c;
result.settings=s;
result.design=d;
result.target_speed=target_speed;
result.current_speed=p.R*X(:,2);
result.scheduled_speed=scheduled_speed;
result.gain_inf_norm=gain_inf_norm;
result.gain_schedule=schedule;
result.integration_method='fixed-step forward Euler';
result.fixed_step=dt;
end
