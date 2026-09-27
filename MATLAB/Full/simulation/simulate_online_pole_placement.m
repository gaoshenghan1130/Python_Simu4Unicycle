function result = simulate_online_pole_placement(p,c,d,s,target_speed,min_design_speed)
% Nonlinear simulation with pole placement repeated at every RHS call.
x0=initial_state(s,p);
assert(abs(s.theta0)<s.tilt_limit,'Initial tilt exceeds stop limit.');
assert(abs(target_speed)>min_design_speed, ...
    'Target speed must exceed the minimum rolling-design speed.');

control=@(t,x) online_pole_controller_output( ...
    t,x,p,c,d,target_speed,min_design_speed);
rhs=@(t,x) model_rhs(t,x,control(t,x),p);
times=(0:s.output_dt:s.t_end)';
if times(end)<s.t_end, times(end+1)=s.t_end; end
opts=odeset('RelTol',s.rel_tol,'AbsTol',s.abs_tol, ...
    'MaxStep',s.max_step,'Events',@(t,x) stop_events(t,x,s,p,min_design_speed));
[t,X,te,xe,ie]=ode45(rhs,times,x0,opts);

U=zeros(numel(t),2);
scheduled_speed=zeros(numel(t),1);
gain_inf_norm=zeros(numel(t),1);
for sample=1:numel(t)
    [u,scheduled_speed(sample),K]=control(t(sample),X(sample,:)');
    U(sample,:)=u';
    gain_inf_norm(sample)=norm(K,inf);
end

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
end

function [value,isterminal,direction]=stop_events(~,x,s,p,min_design_speed)
% Event 1: lean limit. Event 2: rolling design approaches zero speed.
value=[s.tilt_limit-abs(x(7));abs(p.R*x(2))-min_design_speed];
isterminal=[1;1];
direction=[-1;-1];
end
