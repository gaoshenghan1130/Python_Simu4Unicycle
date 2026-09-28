function fig=plot_heading(results)
% Straight +x reference only; chi=psi. No curved path transformation.
fig=figure('Name','Straight rolling - chi feedback');
labels={'chi = psi [deg]','omega3 [rad/s]','Forward speed R*sigma2 [m/s]','epsilon = yG [m]'};
colors=lines(numel(results));
if isfield(results,'sweep_parameter') && ...
        all(strcmp({results.sweep_parameter},'speed'))
    colors=speed_colors([results.sweep_value]);
end
for j=1:numel(results)
    r=results(j);
    data=[r.X(:,6)*180/pi,r.X(:,3),r.parameters.R*r.X(:,2),r.X(:,12)];
    for k=1:4
        subplot(4,1,k); hold on;
        plot(r.t,data(:,k),'LineWidth',1.2,'Color',colors(j,:), ...
            'DisplayName',r.label);
        ylabel(labels{k}); grid on; legend('show','Interpreter','none');
    end
end
xlabel('Time [s]');
end
