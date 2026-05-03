function M = segway_controller(t, z, par)
%SEGWAY_CONTROLLER PD/PID controller matching the Python Segway controller.
%
% z = [x; x_dot; gamma; gamma_dot]
% par.control_mode can be 'balance', 'velocity', or 'position'.
% par.control_strategy can be 'pd' or 'pid'.

persistent vel_error_integral pos_error_integral last_time

if isempty(vel_error_integral) || t == 0
    vel_error_integral = 0.0;
    pos_error_integral = 0.0;
    last_time = [];
end

if ~strcmp(par.scenario, "Segway")
    error('param set does not math the controller')
end


x = z(1);
x_dot = z(2);
gamma = z(3);
gamma_dot = z(4);

desired_gamma = get_field_or_default(par, 'desired_gamma', 0.0);
desired_velocity = get_field_or_default(par, 'desired_velocity', 0.0);
desired_position = get_field_or_default(par, 'desired_position', 0.0);

K_gamma = par.K_gamma;
K_dgamma = par.K_dgamma;
K_velocity = par.K_velocity;
K_position = par.K_position;

if strcmpi(par.control_strategy, 'pid')
    if ~isempty(last_time)
        dt = t - last_time;
        vel_error_integral = vel_error_integral + ...
            (x_dot - desired_velocity)*dt;
        if abs(x - desired_position) > par.posDeadZone
            pos_error_integral = pos_error_integral + ...
                (x - desired_position)*dt;
        end
        vel_error_integral = min(max(vel_error_integral, ...
            -par.clip_integral), par.clip_integral);
        pos_error_integral = min(max(pos_error_integral, ...
            -par.clip_integral), par.clip_integral);
    end
    last_time = t;
else
    vel_error_integral = 0.0;
    pos_error_integral = 0.0;
    last_time = t;
end

switch lower(par.control_mode)
    case 'balance'
        M = K_gamma*(gamma - desired_gamma) + K_dgamma*gamma_dot;
    case 'velocity'
        M = K_gamma*(gamma - desired_gamma) + ...
            K_dgamma*gamma_dot + ...
            K_velocity*(x_dot - desired_velocity);
        if strcmpi(par.control_strategy, 'pid')
            M = M + par.K_vi*vel_error_integral;
        end
    case 'position'
        M = K_gamma*(gamma - desired_gamma) + ...
            K_dgamma*gamma_dot + ...
            K_velocity*(x_dot - desired_velocity) + ...
            K_position*(x - desired_position);
        if strcmpi(par.control_strategy, 'pid')
            M = M + par.K_vi*vel_error_integral + ...
                par.K_pi*pos_error_integral;
        end
    otherwise
        error('segway_controller:BadControlMode', ...
            'control_mode must be balance, velocity, or position.');
end
end

function value = get_field_or_default(s, name, default_value)
if isfield(s, name)
    value = s.(name);
else
    value = default_value;
end
end
