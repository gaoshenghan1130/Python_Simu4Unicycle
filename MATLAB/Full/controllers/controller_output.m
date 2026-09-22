function u = controller_output(t,x,p,c)
% All controller modes return the same physical input [F; M2].
if isfield(c,'mode') && strcmpi(c.mode,'open_loop')
    u=zeros(2,1);
    return; % Bypass ALL feedback, including hold_gamma_zero.
end
if isfield(c,'mode') && strcmpi(c.mode,'lateral_open_loop')
    gamma_dot=x(5)-x(3)*tan(x(7));
    M2=c.kp_gamma*(x(10)-c.gamma_ref) ...
        +c.kd_gamma*(gamma_dot-c.gamma_dot_ref);
    u=[0;max(-c.torque_limit,min(c.torque_limit,M2))];
    return; % Only gamma PD: no lateral feedback, speed loop, or gamma lock.
end
if ~isfield(c,'mode') || strcmpi(c.mode,'pd')
    u=pd_controller(t,x,p,c);
else
    reference=c.x_ref;
    if isfield(c,'reference_rate'), reference=reference+t*c.reference_rate; end
    u=-c.K*(x-reference);
    limits=[c.force_limit;c.torque_limit];
    u=max(-limits,min(limits,u));
end
if isfield(c,'hold_gamma_zero') && c.hold_gamma_zero
    u(2)=gamma_zero_torque(t,x,u(1),p,c.gamma_constraint_rate);
end

end
