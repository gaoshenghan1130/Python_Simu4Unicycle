function plot_pole_tolerance_summary(out)
% Finite-horizon screening map; no assertion of asymptotic stability.
z=load(fullfile(out,'experiments.mat'),'summary','p');T=z.summary;
for speed=unique(T.speed)'
 rows=T(T.speed==speed & strcmp(T.group,'theta'),:);
 scales=unique(rows.scale);values=unique(rows.value);C=zeros(numel(values),numel(scales));
 labels=cell(size(C));
 for i=1:numel(values)
  for j=1:numel(scales)
   r=rows(rows.value==values(i) & rows.scale==scales(j),:);
   if r.stop~=0,C(i,j)=0;labels{i,j}=sprintf('Stop %.2f s',r.end_time);
   elseif r.near_zero,C(i,j)=2;labels{i,j}='Near zero';
   else,C(i,j)=1;labels{i,j}='Residual at 30 s';end
  end
 end
 fig=figure('Visible','off','Position',[50 50 1150 650]);imagesc(C,[0 2]);
 colormap([0.97 0.70 0.68;1 0.91 0.63;0.69 0.88 0.76]);
 for i=1:numel(values),for j=1:numel(scales),text(j,i,labels{i,j},'HorizontalAlignment','center','FontSize',10);end,end
 set(gca,'XTick',1:numel(scales),'XTickLabel',compose('-%g',scales),'YTick',1:numel(values),'YTickLabel',compose('%g',values),'FontSize',11);
 xlabel('Lateral pole center [1/s]');ylabel('Initial theta [deg]');
 title(sprintf('Initial lean tolerance | v = %g m/s | mr = %g kg | b = %g',speed,z.p.mr,z.p.BR));
 exportgraphics(fig,fullfile(out,sprintf('v_%g_tolerance.png',speed)),'Resolution',150);close(fig);
end
end
