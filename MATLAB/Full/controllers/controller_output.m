function u = controller_output(t,x,p,c)
% All controller modes return the same physical input [F; M2].
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
