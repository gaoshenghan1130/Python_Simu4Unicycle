clear; clc; close all;

%% ============================================================
%  1. USER SETTINGS
% =============================================================

fileName = 'friction_sine_A050.mat';

m = 1.0;          % Moving mass [kg]
P = 100;          % Position gain [N/m]
D = 8;            % Velocity gain [N*s/m]

A = 0.050;        % Desired amplitude [m]
f = 0.5;          % Desired frequency [Hz]

% Both simulations assume Cs = Ck = C.
b_manual = 6.0;   % Viscous coefficient [N*s/m]
C_manual = 0.42;   % Coulomb friction [N]

% Measured steady-state quantities
X_meas  = 0.04725; % Half peak-to-peak amplitude [m]
Ts_meas = 0.035;   % ONE sticking interval [s]

% Identification starting guesses
b_guess = 1.0;
C_guess = 0.2;

% Output time alignment, applied to both simulations:
% Negative = earlier / left
% Positive = later / right
% Does not participate in parameter identification.
simTimeShift = -0.07;   % [s]

initialVelocityTolerance = 1e-3;   % [m/s]

%% ============================================================
%  2. LOAD DATA
% =============================================================

S = load(fileName);

t = S.t_s(:);
xd = S.xd_m(:);
vd = S.vd_mps(:);

xMeasuredRaw = S.r_m(:);
vMeasured = S.v_mps(:);
idxOriginal = logical(S.analysis_mask(:));

%% Automatically center measured position
% Use only the selected analysis window.
offsetMask = idxOriginal & isfinite(xMeasuredRaw);

assert(any(offsetMask), ...
    'No valid position samples in the analysis window.');

xMaxMeasured = max(xMeasuredRaw(offsetMask));
xMinMeasured = min(xMeasuredRaw(offsetMask));

positionOffset = (xMaxMeasured+xMinMeasured)/2;

% Apply the same correction to the entire measured position signal.
xMeasured = xMeasuredRaw-positionOffset;

fprintf('\n--- Automatic position centering ---\n');
fprintf('Original maximum = %.6f mm\n',1000*xMaxMeasured);
fprintf('Original minimum = %.6f mm\n',1000*xMinMeasured);
fprintf('Removed offset   = %+.6f mm\n',1000*positionOffset);
fprintf('Centered maximum = %.6f mm\n', ...
    1000*max(xMeasured(offsetMask)));
fprintf('Centered minimum = %.6f mm\n', ...
    1000*min(xMeasured(offsetMask)));

assert(all(diff(t)>0), 'Time must be strictly increasing.');
assert(numel(xd)==numel(t) && numel(vd)==numel(t));
assert(numel(xMeasured)==numel(t) && numel(vMeasured)==numel(t));
assert(numel(idxOriginal)==numel(t) && any(idxOriginal));
assert(all(isfinite([t;xd;vd;xMeasured;vMeasured])));
assert(m>0 && P>0 && D>=0 && A>0 && f>0);
assert(b_manual>=0 && C_manual>=0);
assert(isscalar(simTimeShift) && isfinite(simTimeShift));

omega = 2*pi*f;
T = 1/f;
H = T/2;

assert(isfinite(X_meas) && X_meas>0);
assert(isfinite(Ts_meas) && Ts_meas>0 && Ts_meas<H, ...
    'Use 0 < Ts_meas < T/2.');

p.m = m;
p.P = P;
p.D = D;
p.A = A;
p.w = omega;
p.T = T;

z0 = [xMeasured(1);vMeasured(1)];

if abs(z0(2))<initialVelocityTolerance
    z0(2)=0;
end

%% ============================================================
%  3. HYBRID PARAMETER IDENTIFICATION
%
%  Equation 1:
%    X_no_stick(b,C) - X_meas = 0
%
%  Equation 2:
%    x_slide(tm + L;b,C) + X_meas = 0
%
%  L = T/2 - Ts_meas
%
%  The endpoint velocity is NOT included in the objective.
%  It is retained as a validation quantity.
%
%  Assumptions:
%  - Same-period, half-wave symmetric response
%  - Cs = Ck = C
%  - Known sinusoidal reference A,f
%  - No CBF or saturation intervention
% =============================================================

G = A*hypot(P,D*omega);

% Release condition: |P*X-C| < G
margin = 1e-8*max(G,1);
Clo = max(0,P*X_meas-G)+margin;
Chi = P*X_meas+G-margin;

assert(Chi>Clo, 'No admissible release-force interval.');

objective = @(q) identificationObjective( ...
    q,Clo,Chi,p,X_meas,Ts_meas);

options = optimset( ...
    'Display','off', ...
    'MaxIter',1500, ...
    'MaxFunEvals',4000, ...
    'TolX',1e-9, ...
    'TolFun',1e-12);

bStarts = [
    max(b_guess,1e-3), ...
    max(b_guess/5,1e-3), ...
    max(5*b_guess,1e-3)
];

CStarts = [
    C_guess, ...
    Clo+0.25*(Chi-Clo), ...
    Clo+0.75*(Chi-Clo)
];

bestCost = Inf;
bestQ = [];
bestExit = NaN;

for j = 1:numel(bStarts)

    Cstart = min(max(CStarts(j),Clo+0.001*(Chi-Clo)), ...
                                  Chi-0.001*(Chi-Clo));

    fraction = (Cstart-Clo)/(Chi-Clo);
    q0 = [log(bStarts(j));log(fraction/(1-fraction))];

    [q,cost,exitflag] = fminsearch(objective,q0,options);

    if cost<bestCost
        bestCost = cost;
        bestQ = q;
        bestExit = exitflag;
    end
end

if isempty(bestQ) || bestCost>=1e19
    error('No valid identification candidate found.');
end

[b_identified,C_identified] = ...
    decodeParameters(bestQ,Clo,Chi);

[Rhybrid,detail] = hybridResidual( ...
    b_identified,C_identified,p,X_meas,Ts_meas);

candidateCheck = checkCandidate( ...
    b_identified,C_identified,p,X_meas,Ts_meas,detail);

% Validate the hypothetical no-sticking waveform separately.
noStickValid = checkNoStick( ...
    b_identified,C_identified,p);

fprintf('\n--- Hybrid parameter identification ---\n');
fprintf('Measured X               = %.8f m\n',X_meas);
fprintf('Measured Ts              = %.8f s\n',Ts_meas);
fprintf('Identified b             = %.8f N*s/m\n',b_identified);
fprintf('Identified C             = %.8f N\n',C_identified);
fprintf('No-sticking amplitude    = %.8f m\n',detail.XnoStick);
fprintf('Amplitude residual       = %.3g m\n',Rhybrid(1));
fprintf('Endpoint position error  = %.3g m\n',Rhybrid(2));
fprintf('Endpoint velocity        = %.3g m/s (validation only)\n', ...
    detail.zEnd(2));
fprintf('Scaled objective norm    = %.3g\n',sqrt(bestCost));
fprintf('Optimizer exit flag      = %d\n',bestExit);
fprintf('No-sticking branch check = %d\n',noStickValid);
fprintf('Sliding direction check  = %d\n',candidateCheck.slidingOK);
fprintf('Sticking force check     = %d\n',candidateCheck.stickingOK);
fprintf('Approx. endpoint check   = %d\n',candidateCheck.endpointOK);

if bestExit<=0 || sqrt(bestCost)>1e-3
    warning('Hybrid equations were not solved to a small residual.');
end

if ~noStickValid
    warning(['The no-sticking amplitude branch fails its velocity check. ', ...
        'Treat the amplitude expression as an unvalidated approximation.']);
end

if ~candidateCheck.slidingOK || ~candidateCheck.stickingOK ...
        || ~candidateCheck.endpointOK
    warning(['The hybrid estimate does not closely satisfy all ', ...
        'stick-slip boundary conditions. Inspect the simulations.']);
end

%% ============================================================
%  4. RUN BOTH MODELS
%
%  Fpd = P*(xd-x) + D*(vd-v)
%
%  Sliding:
%    m*a = Fpd - b*v - C*sign(v)
%
%  Sticking:
%    v = 0 and |Fpd| <= C
%
%  Simulation uses logged xd and vd.
% =============================================================

[xManualRaw,vManualRaw,stickingManualRaw] = simulateRod( ...
    t,xd,vd,z0,p,b_manual,C_manual);

[xIdentifiedRaw,vIdentifiedRaw,stickingIdentifiedRaw] = simulateRod( ...
    t,xd,vd,z0,p,b_identified,C_identified);

FmanualRaw = P*(xd-xManualRaw)+D*(vd-vManualRaw);
FidentifiedRaw = P*(xd-xIdentifiedRaw)+D*(vd-vIdentifiedRaw);

%% ============================================================
%  5. TIME ALIGNMENT
% =============================================================

tSim = t+simTimeShift;

xManual = interp1(tSim,xManualRaw,t,'linear',NaN);
vManual = interp1(tSim,vManualRaw,t,'linear',NaN);
Fmanual = interp1(tSim,FmanualRaw,t,'linear',NaN);

xIdentified = interp1(tSim,xIdentifiedRaw,t,'linear',NaN);
vIdentified = interp1(tSim,vIdentifiedRaw,t,'linear',NaN);
Fidentified = interp1(tSim,FidentifiedRaw,t,'linear',NaN);

stickingManual = stickingManualRaw+simTimeShift;
stickingIdentified = stickingIdentifiedRaw+simTimeShift;

valid = isfinite(xManual) & isfinite(vManual) ...
      & isfinite(Fmanual) ...
      & isfinite(xIdentified) & isfinite(vIdentified) ...
      & isfinite(Fidentified);

idx = idxOriginal & valid;

assert(nnz(idx)>=2, ...
    'Time shift leaves too few comparison samples.');

fprintf('\nSimulation time shift = %+.6f s\n',simTimeShift);

%% ============================================================
%  6. COMPARE AMPLITUDE AND RMSE
% =============================================================

Xdata = 0.5*(max(xMeasured(idx))-min(xMeasured(idx)));
Xmanual = 0.5*(max(xManual(idx))-min(xManual(idx)));
Xidentified = 0.5*(max(xIdentified(idx))-min(xIdentified(idx)));

rmseManualRaw = sqrt(mean( ...
    (xManualRaw(idx)-xMeasured(idx)).^2));

rmseIdentifiedRaw = sqrt(mean( ...
    (xIdentifiedRaw(idx)-xMeasured(idx)).^2));

rmseManual = sqrt(mean( ...
    (xManual(idx)-xMeasured(idx)).^2));

rmseIdentified = sqrt(mean( ...
    (xIdentified(idx)-xMeasured(idx)).^2));

comparison = table( ...
    {'Measured';'Manual model';'Identified model'}, ...
    [NaN;b_manual;b_identified], ...
    [NaN;C_manual;C_identified], ...
    1000*[Xdata;Xmanual;Xidentified], ...
    1000*[0;rmseManualRaw;rmseIdentifiedRaw], ...
    1000*[0;rmseManual;rmseIdentified], ...
    'VariableNames', { ...
    'Response', ...
    'b_Ns_per_m', ...
    'C_N', ...
    'Amplitude_mm', ...
    'RMSE_before_shift_mm', ...
    'RMSE_after_shift_mm'});

disp(comparison);

fprintf(['Amplitude is half peak-to-peak over the comparison window; ', ...
    'measurement noise can affect it.\n']);

if isfield(S,'Fpd_N') && isfield(S,'Fcmd_N')
    difference = abs(S.Fpd_N(:)-S.Fcmd_N(:));

    if any(difference(idxOriginal)>1e-3)
        warning(['PD and applied commands differ. ', ...
            'These simulations omit CBF/saturation intervention.']);
    end
end

%% ============================================================
%  7. PLOTS
% =============================================================

colors = [
    0.10 0.10 0.10;
    0.00 0.45 0.74;
    0.85 0.20 0.10
];

figure('Color','w','Name','Measured vs rod simulations');
tiledlayout(3,1);

ax1 = nexttile;

plot(t,1000*xMeasured,'Color',colors(1,:),'LineWidth',1);
hold on;
plot(t,1000*xManual,'Color',colors(2,:),'LineWidth',1.2);
plot(t,1000*xIdentified,'Color',colors(3,:),'LineWidth',1.2);

ylabel('Position (mm)');
legend('Measured','Manual b,C','Identified b,C','Location','best');
title(sprintf('Simulation time shift = %+.4f s',simTimeShift));
grid on;

ax2 = nexttile;

plot(t,vMeasured,'Color',colors(1,:),'LineWidth',1);
hold on;
plot(t,vManual,'Color',colors(2,:),'LineWidth',1.2);
plot(t,vIdentified,'Color',colors(3,:),'LineWidth',1.2);

ylabel('Velocity (m/s)');
grid on;

ax3 = nexttile;

if isfield(S,'Fpd_N')
    plot(t,S.Fpd_N(:),'Color',colors(1,:),'LineWidth',1);
else
    plot(t,P*(xd-xMeasured)+D*(vd-vMeasured), ...
        'Color',colors(1,:),'LineWidth',1);
end

hold on;
plot(t,Fmanual,'Color',colors(2,:),'LineWidth',1.2);
plot(t,Fidentified,'Color',colors(3,:),'LineWidth',1.2);

ylabel('PD force (N)');
xlabel('Time (s)');
legend('Measured/logged PD','Manual model','Identified model', ...
    'Location','best');
grid on;

linkaxes([ax1,ax2,ax3],'x');

figure('Color','w','Name','Analysis window comparison');

plot(t,1000*xMeasured,'Color',colors(1,:),'LineWidth',1);
hold on;
plot(t,1000*xManual,'Color',colors(2,:),'LineWidth',1.2);
plot(t,1000*xIdentified,'Color',colors(3,:),'LineWidth',1.2);

xlim([min(t(idx)),max(t(idx))]);
xlabel('Time (s)');
ylabel('Position (mm)');
legend('Measured','Manual b,C','Identified b,C','Location','best');

title({
    sprintf('Manual: b=%.4g, C=%.4g | Identified: b=%.4g, C=%.4g', ...
        b_manual,C_manual,b_identified,C_identified)
    sprintf('Simulation time shift = %+.4f s',simTimeShift)
});

grid on;

%% ============================================================
%  LOCAL FUNCTIONS
% =============================================================

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