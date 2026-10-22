% Ideal centered locked rod: nonlinear vs exact linear solution.
% Run this file directly. No Symbolic/Control System Toolbox is needed.
clear; clc;
here=fileparts(mfilename('fullpath')); addpath(here);
p=fixed_rod_setup();
p.BP=0; % Closed-form spectrum assumes no wheel/body viscous damping.
c.kp=3; c.kd=0.8; % Set BOTH to zero for fully open-loop longitudinal motion.
c.speed=0;
s0=fixed_rod_linear(p,c);
speeds=[0,0.8,1.2]*s0.critical_speed;
theta0=0.1*pi/180; gamma0=0; yaw_rate0=0;
t_end=10; lean_limit=30*pi/180;
results=struct([]); colors=lines(numel(speeds));
figure('Name','Mechanically fixed centered rod');
for j=1:numel(speeds)
    c.speed=speeds(j); s=fixed_rod_linear(p,c);
    zref=zeros(10,1); zref(2)=c.speed/p.R;
    z0=zref; z0(6)=theta0; z0(8)=gamma0; z0(3)=yaw_rate0;
    opts=odeset('RelTol',1e-9,'AbsTol',1e-11, ...
        'MaxStep',0.02,'Events',@(t,z) stop_at_lean(t,z,lean_limit));
    [t,z,te,ze,ie]=ode45(@(t,z) fixed_rod_rhs(t,z,p,c), ...
        linspace(0,t_end,1001),z0,opts);
    zl=zeros(size(z)); force=zeros(size(t));
    for n=1:numel(t)
        nominal=zref; nominal(7)=c.speed/p.R*t(n); nominal(9)=c.speed*t(n);
        zl(n,:)=(nominal+expm(s.A*t(n))*(z0-zref)).';
        [~,force(n)]=fixed_rod_rhs(t(n),z(n,:).',p,c);
    end
    results(j).speed=c.speed; results(j).t=t; results(j).z=z;
    results(j).linear=zl; results(j).reaction_force=force;
    results(j).spectrum=s; results(j).parameters=p; results(j).controller=c;
    results(j).initial_state=z0; results(j).event_t=te;
    results(j).event_z=ze; results(j).event_id=ie;
    label=sprintf('v=%.3f m/s',c.speed);
    subplot(2,2,1); hold on; h=plot(t,z(:,6)*180/pi,'Color',colors(j,:),'DisplayName',label);
    plot(t,zl(:,6)*180/pi,'--','Color',h.Color,'HandleVisibility','off');
    subplot(2,2,2); hold on; plot(t,z(:,5)*180/pi,'Color',colors(j,:),'DisplayName',label);
    subplot(2,2,3); hold on; plot(t,p.R*z(:,2),'Color',colors(j,:),'DisplayName',label);
    subplot(2,2,4); hold on; plot(t,force,'Color',colors(j,:),'DisplayName',label);
    fprintf('\nv=%.9f m/s; analytic eigenvalues [1/s]:\n',c.speed); disp(s.eigenvalues.');
    if ~isempty(te), fprintf('Stopped at lean limit, t=%.6f s.\n',te(end)); end
end
subplot(2,2,1); ylabel('theta [deg]'); title('Solid: nonlinear; dashed: linear');
subplot(2,2,2); ylabel('psi [deg]');
subplot(2,2,3); ylabel('R sigma2 [m/s]');
subplot(2,2,4); ylabel('Lock reaction F [N]');
for k=1:4, subplot(2,2,k); xlabel('Time [s]'); grid on; legend('show','Location','best'); end
fprintf('\nCritical speed = %.9f m/s (lateral oscillatory threshold only).\n',s0.critical_speed);
out=fullfile(here,'results'); if ~exist(out,'dir'), mkdir(out); end
save(fullfile(out,'fixed_rod_results.mat'),'results');
exportgraphics(gcf,fullfile(out,'fixed_rod_simulation.png'),'Resolution',160);
function [value,isterminal,direction]=stop_at_lean(~,z,limit)
value=limit-abs(z(6)); isterminal=1; direction=-1;
end
