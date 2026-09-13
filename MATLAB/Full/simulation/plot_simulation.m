function plot_simulation(result)
t=result.t; X=result.X; U=result.U; R=result.parameters.R;
theta_dot=X(:,1); r_dot=R*X(:,1)+X(:,4);
gamma_dot=X(:,5)-X(:,3).*tan(X(:,7));
data=[X(:,7)*180/pi,X(:,9),theta_dot*180/pi,r_dot, ...
      X(:,10)*180/pi,gamma_dot*180/pi,U];
labels={'theta [deg]','r [m]','theta dot [deg/s]','r dot [m/s]', ...
        'gamma [deg]','gamma dot [deg/s]','F [N]','M2 [N m]'};
figure('Name','Full nonlinear unicycle - PD');
for k=1:8
    subplot(4,2,k); plot(t,data(:,k),'LineWidth',1.2);
    ylabel(labels{k}); grid on;
    if k>=7, xlabel('Time [s]'); end
end
fprintf('Final sampled time: %.6f s\n',t(end));
fprintf('Peak |theta|: %.6f deg\n',max(abs(data(:,1))));
fprintf('Peak |r|: %.6f m\n',max(abs(data(:,2))));
fprintf('Peak |F|: %.6f N\n',max(abs(U(:,1))));
fprintf('Peak |M2|: %.6f N m\n',max(abs(U(:,2))));
if ~isempty(result.event_time)
    fprintf('Stopped at tilt limit: %.6f s\n',result.event_time(end));
end
end
