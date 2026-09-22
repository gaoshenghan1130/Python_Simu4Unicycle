function validate_mate_oscillations()
% Long-horizon validation of candidates from run_mate_oscillation_study.
root=fileparts(fileparts(mfilename('fullpath')));addpath(genpath(root));
out=fullfile(root,'results','mate_oscillation_study');
a=load(fullfile(out,'screening.mat'));
selected=[10 11 26 30 23 38]; % includes low-amplitude and large-amplitude cases
runs=cell(numel(selected),1);rows={};
for j=1:numel(selected)
    base=a.runs{selected(j)};s=base.settings;s.t_end=120;s.max_step=0.01;s.output_dt=0.01;
    r=integrate_candidate(base.parameters,base.controller,s);
    runs{j}=r;
    row.case_id=selected(j);row.mass=r.parameters.mr;row.speed=s.forward_speed0;
    row.theta0_deg=s.theta0*180/pi;row.end_time=r.t(end);
    row.completed=isempty(r.event_time)&&abs(r.t(end)-s.t_end)<1e-6;
    row.peak_theta_deg=max(abs(r.X(:,7)))*180/pi;row.peak_r=max(abs(r.X(:,9)));
    row.peak_gamma_deg=max(abs(r.X(:,10)))*180/pi;
    row.speed_min=min(r.parameters.R*r.X(:,2));row.speed_max=max(r.parameters.R*r.X(:,2));
    first=r.t>=10&r.t<40;last=r.t>=90;
    row.first_amplitude=span(r.X(first,7))*180/pi;row.last_amplitude=span(r.X(last,7))*180/pi;
    row.amplitude_ratio=row.last_amplitude/row.first_amplitude;
    [freq,power]=spectrum(r);
    [~,i1]=max(power(freq>0.05&freq<0.6));fl=freq(freq>0.05&freq<0.6);
    [~,i2]=max(power(freq>0.65&freq<1.5));fh=freq(freq>0.65&freq<1.5);
    row.measured_low_Hz=fl(i1);row.measured_high_Hz=fh(i2);
    row.peak_F=max(abs(r.U(:,1)));row.peak_M2=max(abs(r.U(:,2)));
    rows{j}=row;
    fprintf('Long case %d: end=%g thetaMax=%g gammaMax=%g ratio=%g frequencies=%g,%g\n', ...
        selected(j),row.end_time,row.peak_theta_deg,row.peak_gamma_deg,row.amplitude_ratio,row.measured_low_Hz,row.measured_high_Hz);
end
% Step/tolerance refinement of the two candidate masses, not every case.
refinements={};
for j=[1 2 3]
    b=runs{j};s=b.settings;s.max_step=0.005;s.rel_tol=s.rel_tol/10;s.abs_tol=s.abs_tol/10;
    r=integrate_candidate(b.parameters,b.controller,s);
    assert(size(r.X,1)==size(b.X,1));
    check.case_id=selected(j);check.max_state_difference=max(abs(r.X(:)-b.X(:)));
    check.max_theta_difference_deg=max(abs(r.X(:,7)-b.X(:,7)))*180/pi;
    assert(check.max_theta_difference_deg<0.001);
    refinements{end+1}=check; %#ok<AGROW>
end
records=[rows{:}];writetable(struct2table(records),fullfile(out,'validation.csv'));
fid=fopen(fullfile(out,'validation.json'),'w');fprintf(fid,'%s',jsonencode(records,PrettyPrint=true));fclose(fid);
fid=fopen(fullfile(out,'refinement.json'),'w');fprintf(fid,'%s',jsonencode([refinements{:}],PrettyPrint=true));fclose(fid);
save(fullfile(out,'validation.mat'),'runs','records','refinements','selected');
% Best meeting-frequency candidate: 1 degree, 0.272 kg, 2.375 m/s.
r=runs{2};
f=figure('Position',[50 50 1250 1050],'Name','Undamped lateral open loop, gamma PD');
vals=[r.X(:,7)*180/pi,r.X(:,9),r.X(:,10)*180/pi,r.parameters.R*r.X(:,2),r.U];
labs={'theta [deg]','r [m]','gamma [deg]','Forward speed [m/s]','F [N]','M2 [N m]'};
for k=1:6
    subplot(3,2,k);plot(r.t,vals(:,k),'LineWidth',1);grid on;ylabel(labs{k});xlabel('Time [s]');
end
exportgraphics(f,fullfile(out,'candidate_120s.png'),'Resolution',140);
f=figure('Position',[50 50 1100 750],'Name','Candidate oscillation frequencies');
subplot(2,1,1);plot(r.t,r.X(:,7)*180/pi,'LineWidth',1.2);xlim([0 30]);ylabel('theta [deg]');xlabel('Time [s]');grid on;
[freq,power]=spectrum(r);subplot(2,1,2);semilogy(freq,power/max(power),'LineWidth',1.2);xlim([0 1.5]);ylim([1e-6 1.2]);grid on;
xlabel('Frequency [Hz]');ylabel('Normalized roll-rate power');
exportgraphics(f,fullfile(out,'candidate_spectrum.png'),'Resolution',140);
fprintf('Long-horizon and refinement validation completed.\n');
end

function r=integrate_candidate(p,c,s)
x0=initial_state(s,p);control=@(t,x) controller_output(t,x,p,c);
opts=odeset('RelTol',s.rel_tol,'AbsTol',s.abs_tol,'MaxStep',s.max_step,'Events',@limits);
[t,X,te,xe,ie]=ode45(@(t,x) model_rhs(t,x,control(t,x),p),(0:s.output_dt:s.t_end)',x0,opts);
U=zeros(numel(t),2);for j=1:numel(t),U(j,:)=control(t(j),X(j,:)')';end
r.t=t;r.X=X;r.U=U;r.event_time=te;r.event_state=xe;r.event_index=ie;
r.parameters=p;r.controller=c;r.settings=s;
end
function [value,terminal,direction]=limits(~,x)
value=[45*pi/180-abs(x(7));0.5-abs(x(9));30*pi/180-abs(x(10))];terminal=ones(3,1);direction=-ones(3,1);
end
function v=span(x)
if isempty(x),v=NaN;else,v=max(x)-min(x);end
end
function [f,p]=spectrum(r)
% FFT of roll rate avoids the mean lean offset; analyze after the first 10 s.
keep=r.t>=min(10,r.t(end)/4);y=r.X(keep,1);y=y-mean(y);n=numel(y);
w=0.5-0.5*cos(2*pi*(0:n-1)'/(n-1));Y=fft(y.*w);m=floor(n/2)+1;
f=(0:m-1)'/(n*median(diff(r.t)));p=abs(Y(1:m)).^2;
end
