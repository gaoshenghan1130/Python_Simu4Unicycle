function [u,scheduled_speed,K,info] = online_pole_controller_output( ...
    t,x,p,c,d,target_speed,min_design_speed,schedule)
% Select K at the CURRENT longitudinal speed.
% By default K is interpolated from precomputed pole-placement designs.
% Set schedule.exact=true to repeat linearization and placement every call.
current_speed=p.R*x(2);
if current_speed==0
    speed_sign=sign(target_speed);
    if speed_sign==0, speed_sign=1; end
else
    speed_sign=sign(current_speed);
end
% The simulation stops at this boundary. The clamp protects intermediate
% ODE evaluations while the event location is being determined.
scheduled_speed=speed_sign*max(abs(current_speed),min_design_speed);

local_design=d;
local_design.forward_speed=scheduled_speed;
if isfield(schedule,'exact') && schedule.exact
    [K,info]=pole_placement(p,local_design);
else
    scheduled_speed=min(schedule.max_speed,max(schedule.min_speed,scheduled_speed));
    K=interpolate_gain(schedule,scheduled_speed);
    info=[];
end

% Schedule K at the current speed, but retain this run's fixed target.
reference=zeros(12,1);
reference(2)=target_speed/p.R;
reference(8)=target_speed*t/p.R;
reference(11)=target_speed*t;
u=-K*(x-reference);

limits=[c.force_limit;c.torque_limit];
u=max(-limits,min(limits,u));
end

function K=interpolate_gain(schedule,speed)
% Fast interpolation on the uniform gain grid; no Jacobian inside ode45.
position=(speed-schedule.min_speed)/schedule.speed_step+1;
lower_index=max(1,min(floor(position),numel(schedule.speeds)-1));
upper_index=lower_index+1;
span=schedule.speeds(upper_index)-schedule.speeds(lower_index);
weight=(speed-schedule.speeds(lower_index))/span;
weight=max(0,min(1,weight));
K=(1-weight)*schedule.gains(:,:,lower_index) ...
    +weight*schedule.gains(:,:,upper_index);
end
