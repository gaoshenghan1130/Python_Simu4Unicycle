function c = generate_controller(p,c,d)
% Called only AFTER applying each experiment's parameter override.
switch lower(char(c.mode))
    case 'open_loop'
        % No actuator input and no gamma-hold feedback in open loop.
        c.hold_gamma_zero=false;
        for field={'K','x_ref','reference_rate','design_info'}
            if isfield(c,field{1}), c=rmfield(c,field{1}); end
        end
        return;
    case 'lateral_open_loop'
        c.hold_gamma_zero=false;
        for field={'K','x_ref','reference_rate','design_info'}
            if isfield(c,field{1}), c=rmfield(c,field{1}); end
        end
        validateattributes([c.kp_gamma,c.kd_gamma,c.gamma_ref,c.gamma_dot_ref], ...
            {'numeric'},{'real','finite','vector','numel',4});
    case 'pd'
        % Preserve original gains and nonlinear physical derivatives.
    case 'direct'
        validateattributes(c.K,{'numeric'},{'real','finite','size',[2 12]});
        validateattributes(c.x_ref,{'numeric'},{'real','finite','size',[12 1]});
    case 'pole_placement'
        [c.K,c.design_info]=pole_placement(p,d);
        c.x_ref=c.design_info.x_eq;
        if isfield(c.design_info,'reference_rate')
            c.reference_rate=c.design_info.reference_rate;
        end
    otherwise
        error('unicycle:ControllerMode','Unknown controller mode: %s',c.mode);
end
validateattributes(c.force_limit,{'numeric'},{'real','scalar','positive'});
validateattributes(c.torque_limit,{'numeric'},{'real','scalar','positive'});
if isfield(c,'hold_gamma_zero') && c.hold_gamma_zero
    if ~isinf(c.torque_limit)
        error('unicycle:GammaLimit','Exact gamma hold requires unlimited torque in this diagnostic experiment.');
    end
    c.gamma_constraint_rate=10; % numerical drift correction [1/s]
    if isfield(c,'design_info')
        % The original longitudinal pole-placement law is replaced, not used.
        c.design_info.unconstrained_longitudinal_poles=c.design_info.actual_longitudinal;
        c.design_info.requested_longitudinal=[];
        c.design_info.K_longitudinal=[];
        c.K(2,:)=0;
        x=c.design_info.x_eq;
        J=zeros(12); h=1e-6;
        for j=1:12
            xp=x; xm=x; xp(j)=xp(j)+h; xm(j)=xm(j)-h;
            J(:,j)=(model_rhs(0,xp,controller_output(0,xp,p,c),p)- ...
                model_rhs(0,xm,controller_output(0,xm,p,c),p))/(2*h);
        end
        c.design_info.full_poles=eig(J);
        idx=c.design_info.idx_longitudinal;
        c.design_info.actual_longitudinal=eig(J(idx,idx));
        c.design_info.longitudinal_mode='gamma_zero (speed is not regulated)';
    end
end

end
