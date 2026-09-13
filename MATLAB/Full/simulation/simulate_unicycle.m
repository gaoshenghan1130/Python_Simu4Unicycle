function result = simulate_unicycle(p,c,s)
% Continuous-time PD simulation; output_dt is not a controller sample time.
x0=initial_state(s,p);
assert(abs(s.theta0)<s.tilt_limit,'Initial tilt exceeds stop limit.');
control=@(t,x) pd_controller(t,x,p,c);
rhs=@(t,x) model_rhs(t,x,control(t,x),p);
times=(0:s.output_dt:s.t_end)';
if times(end)<s.t_end, times(end+1)=s.t_end; end
opts=odeset('RelTol',s.rel_tol,'AbsTol',s.abs_tol, ...
    'MaxStep',s.max_step,'Events',@(t,x) tilt_event(t,x,s));
[t,X,te,xe,ie]=ode45(rhs,times,x0,opts);
U=zeros(numel(t),2);
for i=1:numel(t), U(i,:)=control(t(i),X(i,:)')'; end
result.t=t; result.X=X; result.U=U;
result.event_time=te; result.event_state=xe; result.event_index=ie;
result.parameters=p; result.controller=c; result.settings=s;
end

function [value,isterminal,direction]=tilt_event(t,x,s)
value=s.tilt_limit-abs(x(7)); isterminal=1; direction=-1;
end
