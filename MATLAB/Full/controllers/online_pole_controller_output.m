function [u,scheduled_speed,K,info] = online_pole_controller_output( ...
    t,x,p,c,d,target_speed,min_design_speed)
% Re-linearize and redesign K at the CURRENT longitudinal speed.
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
[K,info]=pole_placement(p,local_design);

% Schedule K at the current speed, but retain this run's fixed target.
reference=zeros(12,1);
reference(2)=target_speed/p.R;
reference(8)=target_speed*t/p.R;
reference(11)=target_speed*t;
u=-K*(x-reference);

limits=[c.force_limit;c.torque_limit];
u=max(-limits,min(limits,u));
end
