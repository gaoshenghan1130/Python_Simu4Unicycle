function rows=scan_mate_modes()
% Columns: mass, speed, low/high imaginary-mode frequencies (Hz), frequency-match score.
root=fileparts(fileparts(mfilename('fullpath')));addpath(genpath(root));
pbase=model_parameters(); rows=[];
for mass=[0.272 1 2.3]
 p=pbase;p.mr=mass;p.BR=0;p.BP=0;
 for v=0.1:0.025:5
  x=zeros(12,1);x(2)=v/p.R;A=zeros(12);h=1e-20;
  for j=1:12,z=x;z(j)=z(j)+1i*h;A(:,j)=imag(model_rhs(0,z,[0;0],p))/h;end
  ev=eig(A([1 4 9 7 3],[1 4 9 7 3])); freq=sort(imag(ev(imag(ev)>1e-6))/(2*pi));
  if max(real(ev))<1e-6 && numel(freq)==2
   rows(end+1,:)=[mass v freq' abs(freq(1)-0.2)+abs(freq(2)-1)];
  end
 end
end
out=fullfile(root,'results','mate_oscillation_study');
if ~exist(out,'dir'),mkdir(out);end
writematrix(rows,fullfile(out,'linear_modes.csv'));
for mass=[0.272 1 2.3]
 a=rows(rows(:,1)==mass,:); [~,ii]=min(a(:,5));disp(a(ii,:)); fprintf('Pure imaginary range sampled: %g to %g\n',min(a(:,2)),max(a(:,2)));
end

end
