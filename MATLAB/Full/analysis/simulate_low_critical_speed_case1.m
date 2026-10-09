function validation = simulate_low_critical_speed_case1()
% Nonlinear validation around the lower Case-1 eigenvalue boundary.
root = fileparts(fileparts(mfilename('fullpath')));
addpath(genpath(root));
p = model_parameters();
vcrit = 1.588584225;
speeds = [0.98 1.00 1.02]*vcrit;

c = struct('mode','pd','kp_theta',0,'kd_theta',0, ...
    'kp_r',854.34418,'kd_r',105.568949, ...
    'kp_gamma',3,'kd_gamma',0.8, ...
    'theta_ref',0,'theta_dot_ref',0,'r_ref',0,'r_dot_ref',0, ...
    'gamma_ref',0,'gamma_dot_ref',0, ...
    'force_limit',Inf,'torque_limit',Inf);
s = simulation_settings();
s.t_end = 30;
s.theta0 = 0.1*pi/180;

validation = repmat(struct(),1,numel(speeds));
for k = 1:numel(speeds)
    sk = s; sk.forward_speed0 = speeds(k);
    run = simulate_unicycle(p,c,sk);
    validation(k).speed = speeds(k);
    validation(k).t = run.t;
    validation(k).theta = run.X(:,7);
    validation(k).r = run.X(:,9);
    validation(k).gamma = run.X(:,10);
    validation(k).forward_speed = p.R*run.X(:,2);
    validation(k).peak_theta_deg = max(abs(run.X(:,7)))*180/pi;
    validation(k).peak_r = max(abs(run.X(:,9)));
    validation(k).peak_gamma_deg = max(abs(run.X(:,10)))*180/pi;
    validation(k).final_speed = p.R*run.X(end,2);
    fprintf('v=%.9f: peak |theta|=%.6g deg, peak |r|=%.6g m, final v=%.9f m/s\n', ...
        validation(k).speed, validation(k).peak_theta_deg, validation(k).peak_r, ...
        validation(k).final_speed);
end

out = fullfile(root,'analysis','critical_speed_case1_validation.mat');
save(out,'validation','vcrit','c','p');
fprintf('Saved validation to %s\n',out);
end
