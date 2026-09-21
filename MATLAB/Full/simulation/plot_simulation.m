function fig = plot_simulation(results)
% Overlay all runs; use each run's own time grid and physical parameters.
labels={'theta [deg]','r [m]','theta dot [deg/s]','r dot [m/s]', ...
        'gamma [deg]','gamma dot [deg/s]','F [N]','M2 [N m]'};
fig=figure('Name','Full nonlinear unicycle - controller comparison');
colors=lines(numel(results));
for j=1:numel(results)
    r=results(j); X=r.X; U=r.U;
    data=[X(:,7)*180/pi,X(:,9),X(:,1)*180/pi, ...
        r.parameters.R*X(:,1)+X(:,4),X(:,10)*180/pi, ...
        (X(:,5)-X(:,3).*tan(X(:,7)))*180/pi,U];
    if isfield(r,'label'), label=r.label; else, label=sprintf('Run %d',j); end
    for k=1:8
        subplot(4,2,k); hold on;
        plot(r.t,data(:,k),'LineWidth',1.2,'Color',colors(j,:),'DisplayName',label);
        ylabel(labels{k}); grid on;
        if k>=7, xlabel('Time [s]'); end
        if j==numel(results), legend('show','Interpreter','none','Location','best'); end
    end
    fprintf('%s: last sample %.6f s; peak |theta| %.6f deg; |r| %.6f m; |F| %.6f N; |M2| %.6f N m\n', ...
        label,r.t(end),max(abs(data(:,1))),max(abs(data(:,2))),max(abs(U(:,1))),max(abs(U(:,2))));
    if ~isempty(r.event_time)
        fprintf('  Stopped at tilt limit: %.6f s\n',r.event_time(end));
    end
end
end
