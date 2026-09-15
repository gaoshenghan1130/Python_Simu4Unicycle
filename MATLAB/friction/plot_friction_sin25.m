function fig = friction_dual_ui
% FRICTION_DUAL_UI  Compare 25 mm and 50 mm friction experiments.
% Save as friction_dual_ui.m and run: friction_dual_ui
% Requires MATLAB with uifigure/uigridlayout (R2018b or later).
% No Optimization Toolbox required. All model helpers are in this file.
% Shared manual m,P,D,b,C; independent A,f,X,Ts and time shift per dataset.
% Identification: no-sticking amplitude + stick-slip endpoint position.
% Time shifts are display alignment only, not identified physical delays.

fig = uifigure('Name','Rod friction | 25 mm and 50 mm', ...
    'Position',[60 50 1480 920]);
root = uigridlayout(fig,[1 2]);
root.ColumnWidth = {355,'1x'};
root.Padding = [10 10 10 10];
left = uigridlayout(root,[4 1]);
left.RowHeight = {240,'1x',78,44};
left.Padding = [0 0 0 0];

sharedPanel = uipanel(left,'Title','Shared model / manual parameters');
sg = uigridlayout(sharedPanel,[6 2]);
sg.ColumnWidth = {'1x',115};
sg.RowHeight = {28,28,28,28,28,28};
sg.RowSpacing=4; sg.Padding=[8 8 8 8];
mUI = numericRow(sg,1,'Mass m [kg]',1,[1e-6 Inf]);
PUI = numericRow(sg,2,'P [N/m]',100,[1e-6 Inf]);
DUI = numericRow(sg,3,'D [N s/m]',8,[0 Inf]);
bUI = numericRow(sg,4,'Manual b [N s/m]',6,[0 Inf]);
CUI = numericRow(sg,5,'Manual C [N]',0.42,[0 Inf]);
uilabel(sg,'Text','Static = kinetic friction');
uilabel(sg,'Text','Cs = Ck = C');

tabs = uitabgroup(left);
ctrl = cell(1,2);
cache = cell(1,2);
for k = 1:2
    tab = uitab(tabs,'Title',sprintf('%d mm',25*k));
    g = uigridlayout(tab,[12 2]);
    g.ColumnWidth = {'1x',115};
    g.RowSpacing=4; g.Padding=[8 8 8 8];
    g.RowHeight = {22,30,28,28,28,28,30,28,28,30,42,'1x'};
    lab = uilabel(g,'Text','MAT filename (editable, or Browse)');
    lab.Layout.Row=1; lab.Layout.Column=[1 2];
    c.file = uieditfield(g,'text','Value',sprintf('friction_sine_A%03d.mat',25*k));
    c.file.Layout.Row=2; c.file.Layout.Column=1;
    bt = uibutton(g,'Text','Browse...','ButtonPushedFcn',@(~,~)browseFile(k));
    bt.Layout.Row=2; bt.Layout.Column=2;
    c.A = numericRow(g,3,'Desired A [mm]',25*k,[1e-6 Inf]);
    c.f = numericRow(g,4,'Frequency [Hz]',0.5,[1e-6 Inf]);
    if k==1
        initialX=0; initialTs=0;
    else
        initialX=47.25; initialTs=35;
    end
    c.X = numericRow(g,5,'Measured X [mm]',initialX,[0 Inf]);
    c.Ts = numericRow(g,6,'One stop Ts [ms]',initialTs,[0 Inf]);
    bt = uibutton(g,'Text','Use measured peak amplitude', ...
        'ButtonPushedFcn',@(~,~)useMeasuredX(k));
    bt.Layout.Row=7; bt.Layout.Column=[1 2];
    c.shift = numericRow(g,8,'Output time shift [s]',-0.07,[-Inf Inf]);
    c.shift.ValueChangedFcn = @(~,~)redrawAll();
    c.extra = numericRow(g,9,'Extra measured offset [mm]',0,[-Inf Inf]);
    c.center = uicheckbox(g,'Text','Auto-center measured position','Value',true);
    c.center.Layout.Row=10; c.center.Layout.Column=[1 2];
    lab=uilabel(g,'Text',{'Negative time shift = earlier. Positive = later.'; ...
        'X=0: initialize from data. Ts=0: skip identification.'});
    lab.Layout.Row=11; lab.Layout.Column=[1 2];
    c.info = uitextarea(g,'Editable','off','Value',{'Ready. Choose the MAT file.'});
    c.info.Layout.Row=12; c.info.Layout.Column=[1 2];
    ctrl{k}=c;
end

buttons=uigridlayout(left,[2 2]);
buttons.Padding=[0 0 0 0]; buttons.RowHeight={30,30};
runManual=uibutton(buttons,'Text','Run manual: both files', ...
    'ButtonPushedFcn',@(~,~)runBoth(false));
runFull=uibutton(buttons,'Text','Identify + run both', ...
    'ButtonPushedFcn',@(~,~)runBoth(true));
zoomUI=uicheckbox(buttons,'Text','Analysis window only','Value',true, ...
    'ValueChangedFcn',@(~,~)redrawAll());
uilabel(buttons,'Text','Shift edits redraw instantly.');
status=uilabel(left,'Text','Edit parameters, then click a Run button.', ...
    'WordWrap','on');

right=uigridlayout(root,[3 1]);
right.Padding=[0 0 0 0];
right.RowHeight={'1x',140,105};
plots=uigridlayout(right,[3 2]);
plots.RowHeight={'1x','1x','1x'};
axesUI=gobjects(3,2);
for k=1:2
    for row=1:3
        ax=uiaxes(plots);
        ax.Layout.Row=row; ax.Layout.Column=k;
        axesUI(row,k)=ax;
        grid(ax,'on');
        xlabel(ax,'Time [s]');
    end
    ylabel(axesUI(1,k),'Position [mm]');
    ylabel(axesUI(2,k),'Velocity [m/s]');
    ylabel(axesUI(3,k),'PD force [N]');
    title(axesUI(1,k),sprintf('%d mm dataset',25*k));
    linkaxes(axesUI(:,k),'x');
end
results=uitable(right,'ColumnName', ...
    {'Dataset','Response','b [N s/m]','C [N]','X [mm]', ...
     'RMSE raw [mm]','RMSE aligned [mm]','Shift [s]'}, ...
    'ColumnEditable',false(1,8));
logUI=uitextarea(right,'Editable','off','Value',{ ...
    'Defaults preserve the supplied 50 mm settings (b=6, C=0.42).'; ...
    '25 mm: X initializes from data; enter its measured Ts to identify.'; ...
    'Changing m/P/D/b/C, offsets, X/Ts or file requires Run again.'; ...
    'Two independent identifications; the manual b,C are shared.'});

    function browseFile(k)
        [name,folder]=uigetfile('*.mat','Select experiment MAT file');
        if isequal(name,0), return; end
        ctrl{k}.file.Value=fullfile(folder,name);
        status.Text='File selected. Click Run to reload and simulate.';
    end

    function useMeasuredX(k)
        try
            data=readExperiment(ctrl{k});
            ctrl{k}.X.Value=1000*data.amplitude;
            ctrl{k}.info.Value={sprintf('Measured X = %.5f mm',1000*data.amplitude); ...
                sprintf('Removed offset = %+.5f mm',1000*data.offset)};
            status.Text='X updated from analysis_mask. Click Identify + run both.';
        catch err
            uialert(fig,err.message,'Could not read amplitude');
        end
    end

    function runBoth(doIdentify)
        runManual.Enable='off'; runFull.Enable='off';
        cleanup=onCleanup(@restoreButtons); %#ok<NASGU>
        messages={};
        for k=1:2
            status.Text=sprintf('Processing %d mm dataset...',25*k);
            drawnow;
            cache{k}=[];
            try
                c=ctrl{k};
                data=readExperiment(c);
                p=struct('m',mUI.Value,'P',PUI.Value,'D',DUI.Value, ...
                    'A',c.A.Value/1000,'w',2*pi*c.f.Value,'T',1/c.f.Value);
                if all(isnan(data.F))
                    data.F=p.P*(data.xd-data.x)+p.D*(data.vd-data.v);
                end
                if c.X.Value==0
                    c.X.Value=1000*data.amplitude;
                end
                X=c.X.Value/1000; Ts=c.Ts.Value/1000;
                z0=[data.x(1);data.v(1)];
                if abs(z0(2))<1e-3, z0(2)=0; end
                b=bUI.Value; C=CUI.Value;
                [xm,vm]=simulateRod(data.t,data.xd,data.vd,z0,p,b,C);
                fm=p.P*(data.xd-xm)+p.D*(data.vd-vm);
                q=struct('data',data,'manual',[xm vm fm], ...
                    'b',b,'C',C,'identified',[],'bi',NaN,'Ci',NaN,'p',p);
                notes={sprintf('Offset removed: %+.5f mm',1000*data.offset); ...
                    sprintf('Measured peak amplitude: %.5f mm',1000*data.amplitude)};
                if doIdentify && Ts>0
                    try
                        status.Text=sprintf('Identifying %d mm dataset...',25*k);
                        drawnow;
                        fit=identifyHybrid(p,X,Ts,max(b,1e-3),max(C,1e-3));
                        [xi,vi]=simulateRod(data.t,data.xd,data.vd,z0,p,fit.b,fit.C);
                        fi=p.P*(data.xd-xi)+p.D*(data.vd-vi);
                        q.identified=[xi vi fi]; q.bi=fit.b; q.Ci=fit.C;
                        notes=[notes; {sprintf('Identified b=%.6g, C=%.6g',fit.b,fit.C); ...
                            sprintf('Scaled residual: %.3g',fit.norm); ...
                            sprintf('Endpoint velocity: %.3g m/s',fit.vEnd); ...
                            sprintf('Checks: branch %d, slide %d, stick %d, endpoint %d', ...
                            fit.branch,fit.check.slidingOK,fit.check.stickingOK,fit.check.endpointOK)}];
                        if fit.norm>1e-3 || fit.exit<=0 || ~fit.branch || ...
                                ~fit.check.slidingOK || ~fit.check.stickingOK || ~fit.check.endpointOK
                            notes{end+1}='Approximation checks failed: inspect the simulated response.';
                        end
                    catch fitError
                        notes{end+1}=['Identification failed: ' fitError.message];
                    end
                elseif doIdentify
                    notes{end+1}='Identification skipped: enter measured Ts > 0 ms.';
                else
                    notes{end+1}='Manual simulation only. Use Identify + run both for the red line.';
                end
                if data.cbf
                    notes{end+1}='CBF/command changes detected; model uses unconstrained PD.';
                end
                if abs(data.amplitude-X)>0.1*X
                    notes{end+1}='Entered X differs >10% from raw half peak-to-peak.';
                end
                cache{k}=q;
                c.info.Value=notes;
                messages=[messages;{sprintf('--- %d mm ---',25*k)};notes]; %#ok<AGROW>
            catch err
                ctrl{k}.info.Value={['Error: ' err.message]};
                messages=[messages;{sprintf('%d mm: %s',25*k,err.message)}]; %#ok<AGROW>
            end
        end
        logUI.Value=messages;
        redrawAll();
        status.Text='Finished. Change parameters and rerun; time shifts update immediately.';
    end

    function restoreButtons()
        if isvalid(fig)
            runManual.Enable='on'; runFull.Enable='on';
        end
    end

    function redrawAll()
        rows=cell(0,8);
        for k=1:2
            for r=1:3, cla(axesUI(r,k)); end
            if isempty(cache{k})
                title(axesUI(1,k),sprintf('%d mm: load/run to display',25*k));
                continue;
            end
            q=cache{k}; d=q.data; shift=ctrl{k}.shift.Value;
            measured=[d.x d.v d.F];
            manual=interp1(d.t+shift,q.manual,d.t,'linear',NaN);
            identified=[];
            if ~isempty(q.identified)
                identified=interp1(d.t+shift,q.identified,d.t,'linear',NaN);
            end
            for r=1:3
                ax=axesUI(r,k); hold(ax,'on'); scale=1;
                if r==1, scale=1000; end
                plot(ax,d.t,scale*measured(:,r),'Color',[.12 .12 .12], ...
                    'DisplayName','Measured','LineWidth',1);
                plot(ax,d.t,scale*manual(:,r),'Color',[0 .45 .74], ...
                    'DisplayName','Manual b,C','LineWidth',1.2);
                if ~isempty(identified)
                    plot(ax,d.t,scale*identified(:,r),'Color',[.85 .2 .1], ...
                        'DisplayName','Identified b,C','LineWidth',1.2);
                end
                grid(ax,'on');
                if zoomUI.Value
                    xlim(ax,[min(d.t(d.mask)) max(d.t(d.mask))]);
                else
                    xlim(ax,[d.t(1) d.t(end)]);
                end
                if r==1, legend(ax,'show','Location','best'); end
                hold(ax,'off');
            end
            title(axesUI(1,k),sprintf('%d mm | offset %+.3f mm | shift %+.3f s', ...
                25*k,1000*d.offset,shift));
            ix=d.mask & all(isfinite(manual),2);
            if ~isempty(identified), ix=ix & all(isfinite(identified),2); end
            if nnz(ix)<2
                status.Text='Time shift leaves no comparison overlap in an analysis window.';
                continue;
            end
            amp=@(x)1000*(max(x(ix))-min(x(ix)))/2;
            rmse=@(x)1000*sqrt(mean((x(ix)-d.x(ix)).^2));
            tag=sprintf('%d mm',25*k);
            rows(end+1,:)={tag,'Measured',NaN,NaN,amp(d.x),0,0,0}; %#ok<AGROW>
            rows(end+1,:)={tag,'Manual',q.b,q.C,amp(manual(:,1)), ...
                rmse(q.manual(:,1)),rmse(manual(:,1)),shift}; %#ok<AGROW>
            if ~isempty(identified)
                rows(end+1,:)={tag,'Identified',q.bi,q.Ci,amp(identified(:,1)), ...
                    rmse(q.identified(:,1)),rmse(identified(:,1)),shift}; %#ok<AGROW>
            end
        end
        results.Data=rows;
    end
end

function field=numericRow(parent,row,label,value,limits)
lab=uilabel(parent,'Text',label);
lab.Layout.Row=row; lab.Layout.Column=1;
field=uieditfield(parent,'numeric','Value',value,'Limits',limits);
field.Layout.Row=row; field.Layout.Column=2;
end

function d=readExperiment(c)
filename=strtrim(c.file.Value);
if ~isfile(filename)
    error('File not found: %s. Edit the path or use Browse.',filename);
end
S=load(filename);
required={'t_s','xd_m','vd_mps','r_m','v_mps','analysis_mask'};
for j=1:numel(required)
    if ~isfield(S,required{j}), error('Missing field: %s',required{j}); end
end
d.t=S.t_s(:); d.xd=S.xd_m(:); d.vd=S.vd_mps(:);
d.x=S.r_m(:); d.v=S.v_mps(:); d.mask=logical(S.analysis_mask(:));
n=numel(d.t);
assert(n>=3 && all(diff(d.t)>0),'Time must increase and have at least 3 samples.');
assert(all([numel(d.xd),numel(d.vd),numel(d.x),numel(d.v),numel(d.mask)]==n), ...
    'Data arrays must have the same length.');
assert(all(isfinite([d.t;d.xd;d.vd;d.x;d.v])),'Data contains NaN or Inf.');
assert(nnz(d.mask)>=2,'analysis_mask needs at least 2 valid samples.');
d.offset=0;
if c.center.Value
    d.offset=(max(d.x(d.mask))+min(d.x(d.mask)))/2;
end
d.offset=d.offset+c.extra.Value/1000;
d.x=d.x-d.offset;
d.amplitude=(max(d.x(d.mask))-min(d.x(d.mask)))/2;
d.F=NaN(n,1);
if isfield(S,'Fpd_N')
    assert(numel(S.Fpd_N)==n,'Fpd_N length mismatch.');
    d.F=S.Fpd_N(:);
end
d.cbf=false;
if isfield(S,'Fcmd_N') && isfield(S,'Fpd_N')
    assert(numel(S.Fcmd_N)==n,'Fcmd_N length mismatch.');
    difference=abs(S.Fpd_N(:)-S.Fcmd_N(:));
    d.cbf=any(difference(d.mask)>1e-3);
end
end

function fit=identifyHybrid(p,X,Ts,bGuess,CGuess)
assert(X>0 && Ts>0 && Ts<pi/p.w,'Need X>0 and 0<Ts<T/2.');
G=p.A*hypot(p.P,p.D*p.w);
margin=1e-8*max(G,1);
lo=max(0,p.P*X-G)+margin; hi=p.P*X+G-margin;
assert(hi>lo,'No valid release-force interval.');
objective=@(q)identificationObjective(q,lo,hi,p,X,Ts);
opt=optimset('Display','off','MaxIter',1500,'MaxFunEvals',4000, ...
    'TolX',1e-9,'TolFun',1e-12);
bStarts=[bGuess,max(bGuess/5,1e-3),max(5*bGuess,1e-3)];
cStarts=[CGuess,lo+.25*(hi-lo),lo+.75*(hi-lo)];
best=Inf; bestQ=[]; fit.exit=NaN;
for j=1:3
    cc=min(max(cStarts(j),lo+.001*(hi-lo)),hi-.001*(hi-lo));
    fraction=(cc-lo)/(hi-lo);
    q0=[log(bStarts(j));log(fraction/(1-fraction))];
    [q,cost,flag]=fminsearch(objective,q0,opt);
    if cost<best, best=cost; bestQ=q; fit.exit=flag; end
end
assert(~isempty(bestQ) && best<1e19,'No valid identification candidate.');
[fit.b,fit.C]=decodeParameters(bestQ,lo,hi);
[~,detail]=hybridResidual(fit.b,fit.C,p,X,Ts);
fit.norm=sqrt(best); fit.vEnd=detail.zEnd(2);
fit.check=checkCandidate(fit.b,fit.C,p,X,Ts,detail);
fit.branch=checkNoStick(fit.b,fit.C,p);
end

function [b,C] = decodeParameters(q,Clo,Chi)
    b = exp(q(1));

    if q(2)>=0
        fraction = 1/(1+exp(-q(2)));
    else
        eq = exp(q(2));
        fraction = eq/(1+eq);
    end

    C = Clo+(Chi-Clo)*fraction;
end

function cost = identificationObjective(q,Clo,Chi,p,X,Ts)
    [b,C] = decodeParameters(q,Clo,Chi);

    if ~isfinite(b) || b>1e10
        cost = 1e20;
        return;
    end

    R = hybridResidual(b,C,p,X,Ts);

    % Both residuals have position units.
    scaled = R/X;

    if any(~isfinite(scaled))
        cost = 1e20;
    else
        cost = sum(scaled.^2);
    end
end

function [R,d] = hybridResidual(b,C,p,X,Ts)
    XnoStick = noStickAmplitude(b,C,p);

    [Rendpoint,d] = endpointResidual(b,C,p,X,Ts);

    R = [
        XnoStick-X;
        Rendpoint(1)
    ];

    d.XnoStick = XnoStick;
end

function [XnoStick,d] = noStickAmplitude(b,C,p)
    H = pi/p.w;
    c = C/p.P;

    B0 = p.A*hypot(p.P,p.D*p.w) / ...
        hypot(p.P-p.m*p.w^2,(p.D+b)*p.w);

    M = [
        0,1;
        -p.P/p.m,-(p.D+b)/p.m
    ];

    E = expm(M*H);

    % Equivalent to S=k1+k2 and Q=r1*k1+r2*k2.
    % Works at repeated roots as well.
    h0 = (eye(2)+E)\[-2*c;0];

    S = h0(1);
    Q = h0(2);

    radicand = B0^2-(Q/p.w)^2;

    XnoStick = NaN;
    U = NaN;

    if radicand>=0
        U = sqrt(radicand);
        candidate = c+S+U;

        if isfinite(candidate) && candidate>0
            XnoStick = candidate;
        end
    end

    d.M = M;
    d.h0 = h0;
    d.U = U;
    d.Q = Q;
    d.H = H;
end

function ok = checkNoStick(b,C,p)
    [Xns,d] = noStickAmplitude(b,C,p);

    if ~isfinite(Xns)
        ok = false;
        return;
    end

    tau = linspace(0,d.H,501);
    velocity = zeros(size(tau));

    for j = 1:numel(tau)
        yh = expm(d.M*tau(j))*d.h0;

        velocity(j) = ...
            -p.w*d.U*sin(p.w*tau(j)) ...
            -d.Q*cos(p.w*tau(j)) ...
            +yh(2);
    end

    ok = all(velocity(2:end-1)<0);
end

function [R,d] = endpointResidual(b,C,p,X,Ts)
    H = pi/p.w;
    L = H-Ts;

    G = p.A*hypot(p.P,p.D*p.w);
    delta = atan2(p.D*p.w,p.P);

    arg = (p.P*X-C)/G;
    arg = min(1,max(-1,arg));

    tm = (pi-asin(arg)-delta)/p.w;

    h = p.A*(p.P+1i*p.D*p.w) / ...
        (p.P-p.m*p.w^2+1i*(p.D+b)*p.w);

    M = [
        0,1;
        -p.P/p.m,-(p.D+b)/p.m
    ];

    zp0 = particularState(tm,h,p.w);
    zpL = particularState(tm+L,h,p.w);

    offset = [C/p.P;0];
    y0 = [X;0]-zp0-offset;

    zEnd = zpL+offset+expm(M*L)*y0;

    R = zEnd-[-X;0];

    d.tm = tm;
    d.L = L;
    d.h = h;
    d.M = M;
    d.y0 = y0;
    d.zEnd = zEnd;
end

function z = particularState(t,h,w)
    e = exp(1i*w*t);
    z = [imag(h*e);imag(1i*w*h*e)];
end

function result = checkCandidate(b,C,p,X,Ts,d)
    tau = linspace(0,d.L,501);
    velocity = zeros(size(tau));

    for j = 1:numel(tau)
        z = particularState(d.tm+tau(j),d.h,p.w) ...
            +[C/p.P;0]+expm(d.M*tau(j))*d.y0;

        velocity(j) = z(2);
    end

    tStick = linspace(d.tm+d.L,d.tm+pi/p.w,501);

    Fstick = p.P*p.A*sin(p.w*tStick) ...
        +p.D*p.A*p.w*cos(p.w*tStick)+p.P*X;

    velocityTolerance = 1e-7*max(p.w*X,1e-6);
    forceTolerance = 1e-7*max([C,p.P*X,1]);

    result.slidingOK = ...
        b>=0 && C>=0 && Ts>0 && ...
        max(velocity(2:end-1))<=velocityTolerance;

    result.stickingOK = ...
        max(abs(Fstick)-C)<=forceTolerance;

    % Practical diagnostic tolerance, not an exact mathematical test.
    result.endpointOK = ...
        abs(d.zEnd(1)+X)<=1e-3*X && ...
        abs(d.zEnd(2))<=1e-3*p.w*X;
end

function [xOut,vOut,stickIntervals] = simulateRod( ...
    tData,xd,vd,z0,p,b,C)

    gData = p.P*xd+p.D*vd;
    g = griddedInterpolant(tData,gData,'linear','nearest');

    tNow = tData(1);
    tEnd = tData(end);
    z = z0;

    tAll = tNow;
    zAll = z.';
    stickIntervals = zeros(0,2);

    maxStep = min(p.T/300,median(diff(tData)));
    launchStep = min(p.T*1e-8,maxStep*1e-3);

    options = odeset( ...
        'RelTol',1e-8, ...
        'AbsTol',1e-10, ...
        'MaxStep',maxStep);

    for segment = 1:20000

        if tNow>=tEnd
            break;
        end

        if abs(z(2))<1e-12
            z(2)=0;
            drive = g(tNow)-p.P*z(1);

            if abs(drive)<=C
                [tr,s] = loggedRelease( ...
                    tNow,z(1),tData,gData,g,p.P,C);

                stopTime = min(tr,tEnd);

                if stopTime>tNow
                    stickIntervals(end+1,:) = [tNow,stopTime];

                    tAll(end+1,1) = stopTime;
                    zAll(end+1,:) = [z(1),0];

                    tNow = stopTime;
                end

                if tNow>=tEnd
                    break;
                end
            else
                s = sign(drive);
            end
        else
            s = sign(z(2));
        end

        rhs = @(tt,zz) [
            zz(2);
            (g(tt)-p.P*zz(1)-(p.D+b)*zz(2)-C*s)/p.m
        ];

        tLaunch = min(tNow+launchStep,tEnd);

        [tt,zz] = ode45(rhs,[tNow,tLaunch],z,options);

        tAll = [tAll;tt(2:end)];
        zAll = [zAll;zz(2:end,:)];

        tNow = tt(end);
        z = zz(end,:).';

        if tNow>=tEnd
            break;
        end

        eventOptions = odeset(options, ...
            'Events',@(tt,zz) zeroVelocity(tt,zz,s));

        [tt,zz,te,ze] = ode45(rhs,[tNow,tEnd],z,eventOptions);

        tAll = [tAll;tt(2:end)];
        zAll = [zAll;zz(2:end,:)];

        tNow = tt(end);
        z = zz(end,:).';

        if isempty(te)
            break;
        end

        tNow = te(end);
        z = [ze(end,1);0];
        zAll(end,2)=0;
    end

    if tNow<tEnd
        error('Simulation segment limit reached.');
    end

    [tAll,ia] = unique(tAll,'stable');
    zAll = zAll(ia,:);

    xOut = interp1(tAll,zAll(:,1),tData,'linear');
    vOut = interp1(tAll,zAll(:,2),tData,'linear');
end

function [tr,s] = loggedRelease(t0,xHold,tData,gData,g,P,C)
    future = tData>t0;

    tk = [t0;tData(future)];
    force = [g(t0);gData(future)]-P*xHold;

    j = find(abs(force)>C,1,'first');

    if isempty(j)
        tr = Inf;
        s = 0;
        return;
    end

    s = sign(force(j));

    if j==1
        tr = t0;
        return;
    end

    threshold = s*C;

    fraction = (threshold-force(j-1)) / ...
               (force(j)-force(j-1));

    fraction = min(1,max(0,fraction));

    tr = tk(j-1)+fraction*(tk(j)-tk(j-1));
end

function [value,isterminal,direction] = zeroVelocity(~,z,s)
    value = z(2);
    isterminal = 1;
    direction = -s;
end