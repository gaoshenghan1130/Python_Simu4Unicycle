function plot_equilibrium_balance(report,showFigures)
% One figure per speed; includes all requested initial cases and models.
if nargin<2,showFigures=true;end
visibility='off';if showFigures,visibility='on';end
for speed=unique(report.metrics.speed)'
 f=figure('Visible',visibility,'Color','w','Position',[50 50 1400 850], ...
    'Name',sprintf('Equilibrium balance: v=%g m/s',speed));
 tl=tiledlayout(f,2,4,'TileSpacing','compact','Padding','compact');
 ax=gobjects(1,8);for a=1:8,ax(a)=nexttile(tl);hold(ax(a),'on');grid(ax(a),'on');end
 for j=1:numel(report.runs)
  q=report.runs{j};if q.description.speed~=speed,continue;end
  yaw=q.X(:,3)./cos(q.X(:,7));
  if q.description.model=="linear",yaw=q.X(:,3);end
  vals={rad2deg(q.X(:,7)),q.X(:,9),yaw,vecnorm(q.Z,2,2), ...
        q.U(:,1),q.U(:,2),rad2deg(q.X(:,10)),q.X(:,9)-q.Z(:,1)};
  label=q.description.model+" / "+q.description.initial;
  if ~q.description.completed,label=label+" (stopped)";end
  for a=1:8,plot(ax(a),q.t,vals{a},'DisplayName',label,'LineWidth',1.4);end
 end
 labs={'Lean (deg)','Rod position r (m)','Yaw rate (rad/s)', ...
       'Feedback error norm','Rod force F (N)','Torque M_2 (Nm)', ...
       'Body tilt (deg)','Desired rod position (m)'};
 for a=1:8,ylabel(ax(a),labs{a});xlabel(ax(a),'Time (s)');end
 yline(ax(5),report.Fmax,':','HandleVisibility','off');yline(ax(5),-report.Fmax,':','HandleVisibility','off');
 legend(ax(1),'Location','best','Interpreter','none');
 title(tl,sprintf('Equilibrium feedback: v=%g m/s, initial lean=%g deg, rho_F=%g', ...
       speed,report.leanDeg,report.rho));
 exportgraphics(f,fullfile(report.output_dir,sprintf('comparison_v_%g.png',speed)),'Resolution',160);
 if ~showFigures,close(f);end
end
end
