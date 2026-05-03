function par = SegwayParamset()
    par.scenario = 'Segway'; % use Segway models, the rest of the code will check themselves according to this
    par.m = 0.7;          
    par.m_w = 3.06;       
    par.h = 0.115;        
    par.R = 0.2527;       
    par.I = 0.8;          
    par.g = 9.81;         

    par.K_gamma = 3.0;
    par.K_dgamma = 0.8;
    par.K_velocity = 4;
    par.K_position = 2.0;
    par.K_vi = 0.0;
    par.K_pi = 0.00;
    par.posDeadZone = 0.005;
    par.clip_integral = 1.0;

    par.B = 0.002213639008177662;
    par.B_0 = 0.001748261186143499;
    par.mu_rolling = 0.0015478352590933;
    par.smooth_factor = 100;

    par.control_mode = 'null'; % must be velocity or position, otherwise throw an error 
    par.control_strategy = 'pd';
    par.desired_gamma = 0.0;
    par.desired_velocity = 0.0;
    par.desired_position = 0.0;

    % switch lower(scenario)
    %     case 'velocity'
    %         par.control_mode = 'velocity';
    %         par.control_strategy = 'pd';
    %         par.desired_gamma = 0.0;
    %         par.desired_velocity = 0.5;
    %         par.desired_position = 0.0;
    %     case 'position'
    %         par.control_mode = 'position';
    %         par.control_strategy = 'pd';
    %         par.desired_gamma = 0.0;
    %         par.desired_velocity = 0.0;
    %         par.desired_position = 1.0;
    %     otherwise
    %         error('Unknown scenario: %s. Use ''velocity'' or ''position''.', scenario);
    % end
end