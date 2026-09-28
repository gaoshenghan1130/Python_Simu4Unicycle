function schedule = build_pole_gain_schedule(p,d,min_speed,max_speed,speed_step)
% Precompute pole-placement gains on a uniform positive-speed grid.
validateattributes([min_speed,max_speed,speed_step],{'numeric'}, ...
    {'real','finite','positive','vector','numel',3});
assert(max_speed>min_speed,'max_speed must exceed min_speed.');

speeds=min_speed:speed_step:max_speed;
if speeds(end)<max_speed, speeds(end+1)=max_speed; end
gains=zeros(2,12,numel(speeds));
for index=1:numel(speeds)
    local_design=d;
    local_design.forward_speed=speeds(index);
    gains(:,:,index)=pole_placement(p,local_design);
end

schedule.speeds=speeds;
schedule.gains=gains;
schedule.min_speed=speeds(1);
schedule.max_speed=speeds(end);
schedule.speed_step=speed_step;
end
