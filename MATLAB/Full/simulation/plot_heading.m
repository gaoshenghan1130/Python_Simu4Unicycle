function fig=plot_heading(results)
% Straight +x reference only; chi=psi. No curved path transformation.
fig=figure('Name','Straight rolling - chi feedback');
labels={'chi = psi [deg]','omega3 [rad/s]','Forward speed R*sigma2 [m/s]'};
for j=1:numel(results)
    r=results(j);
    data=[r.X(:,6)*180/pi,r.X(:,3),r.parameters.R*r.X(:,2)];
    for k=1:3
        subplot(3,1,k); hold on;
        plot(r.t,data(:,k),'LineWidth',1.2,'DisplayName',r.label);
        ylabel(labels{k}); grid on; legend('show','Interpreter','none');
    end
end
xlabel('Time [s]');
end
