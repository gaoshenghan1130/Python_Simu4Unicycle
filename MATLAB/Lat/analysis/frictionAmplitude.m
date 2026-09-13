clear; clc; close all;

%% Parameters
p.m = 1.0;       % Rod + attached masses [kg]
p.b = 1.0;       % Viscous friction [N*s/m]

p.Ck = 0.20;     % Kinetic Coulomb friction [N]
p.Cs = 2.5;     % Maximum static friction [N]

p.P = 100;      % Position gain [N/m]
p.D = 8;        % Velocity gain [N*s/m]

p.A = 0.05;     % Desired amplitude [m]
p.f = 0.5;      % Desired frequency [Hz]
p.w = 2*pi*p.f;

T = 1/p.f;
nPeriods = 30;
tEnd = nPeriods*T;

x0 = 0;
v0 = 0;

assert(p.m > 0 && p.P > 0 && p.D+p.b > 0);
assert(p.Cs >= p.Ck && p.Ck >= 0);
assert(p.A > 0 && p.f > 0);

%% Numerical settings
options = odeset( ...
    'RelTol',1e-9, ...
    'AbsTol',1e-11, ...
    'MaxStep',T/500);

% Tiny integrated step to leave a zero-velocity endpoint.
launchStep = T*1e-7;

%% Simulate: alternate sticking and sliding
t = 0;
z = [x0;v0];

tAll = t;
zAll = z.';

stickIntervals = zeros(0,2);
tRev = [];
xRev = [];

for segment = 1:10000

    if t >= tEnd
        break;
    end

    %% Determine whether the rod sticks or slides
    if abs(z(2)) < 1e-12
        z(2) = 0;

        % At zero velocity, viscous friction and velocity feedback vanish.
        drive = pdForce(t,z,p);

        if abs(drive) <= p.Cs
            % Hold x constant until |PD force| exceeds Cs.
            [tRelease,s] = nextRelease(t,z(1),p,T);

            tStop = min(tRelease,tEnd);

            if tStop > t
                stickIntervals(end+1,:) = [t,tStop];

                ts = linspace(t,tStop, ...
                    max(2,ceil((tStop-t)/(T/500))+1)).';

                zs = [z(1)*ones(size(ts)), zeros(size(ts))];

                tAll = [tAll;ts(2:end)];
                zAll = [zAll;zs(2:end,:)];

                t = tStop;
            end

            if t >= tEnd
                break;
            end
        else
            s = sign(drive);
        end
    else
        s = sign(z(2));
    end

    %% Sliding dynamics with fixed friction direction
    rhs = @(tt,zz) slidingModel(tt,zz,p,s);

    % Move away from v=0 by integrating, not by imposing a velocity kick.
    tLaunch = min(t+launchStep,tEnd);

    [tt,zz] = ode45(rhs,[t,tLaunch],z,options);

    tAll = [tAll;tt(2:end)];
    zAll = [zAll;zz(2:end,:)];

    t = tt(end);
    z = zz(end,:).';

    if t >= tEnd
        break;
    end

    % Stop at the next velocity zero.
    eventOptions = odeset(options, ...
        'Events',@(tt,zz) stopEvent(tt,zz,s));

    [tt,zz,te,ze] = ode45(rhs,[t,tEnd],z,eventOptions);

    tAll = [tAll;tt(2:end)];
    zAll = [zAll;zz(2:end,:)];

    t = tt(end);
    z = zz(end,:).';

    if isempty(te)
        break;
    end

    % Reassess static friction at the stop.
    t = te(end);
    z = [ze(end,1);0];

    tRev(end+1,1) = t;
    xRev(end+1,1) = z(1);
end

if t < tEnd
    error('Simulation segment limit reached.');
end

%% Extract signals
t = tAll;
x = zAll(:,1);
v = zAll(:,2);

xDesired = p.A*sin(p.w*t);
vDesired = p.A*p.w*cos(p.w*t);

Fpd = p.P*(xDesired-x) + p.D*(vDesired-v);

% Friction term subtracted in m*a = Fpd - Ffriction.
Ffriction = p.b*v + p.Ck*sign(v);

isSticking = false(size(t));

for j = 1:size(stickIntervals,1)
    inside = t >= stickIntervals(j,1) ...
           & t < stickIntervals(j,2);
    isSticking = isSticking | inside;
end

% Static friction balances the applied force.
Ffriction(isSticking) = Fpd(isSticking);

%% Numerical steady-state amplitude
steadyStart = tEnd-5*T;
steady = t >= steadyStart;

% Include accurately located zero-velocity extrema.
steadyExtrema = xRev(tRev >= steadyStart);
positions = [x(steady);steadyExtrema];

xMax = max(positions);
xMin = min(positions);
Xsim = (xMax-xMin)/2;

% Total sticking duration over the final five periods.
stickDuration = 0;

for j = 1:size(stickIntervals,1)
    intervalStart = max(stickIntervals(j,1),steadyStart);
    intervalEnd = min(stickIntervals(j,2),tEnd);
    stickDuration = stickDuration + max(0,intervalEnd-intervalStart);
end

steadySticking = stickDuration > T*1e-8;

% Compare the final two input periods.
tCheck = linspace(tEnd-T,tEnd,2001).';
xLast = interp1(t,x,tCheck,'linear');
xPrevious = interp1(t,x,tCheck-T,'linear');
cycleDifference = max(abs(xLast-xPrevious));

%% Analytical amplitude: original NO-STICKING formula
H = T/2;
c = p.Ck/p.P;

B0 = p.A*hypot(p.P,p.D*p.w) / ...
    hypot(p.P-p.m*p.w^2,(p.D+p.b)*p.w);

r = roots([p.m,p.D+p.b,p.P]);
r1 = r(1);
r2 = r(2);

Xtheory = NaN;

if abs(r1-r2) < 1e-7*max(1,max(abs(r)))
    warning('Analytical calculation skipped: nearly repeated roots.');
else
    k1 = 2*c*r2 / ((r1-r2)*(1+exp(r1*H)));
    k2 = -2*c*r1 / ((r1-r2)*(1+exp(r2*H)));

    S = real(k1+k2);
    Q = real(r1*k1+r2*k2);

    radicand = B0^2-(Q/p.w)^2;

    if radicand >= 0
        U = sqrt(radicand);
        candidate = c+S+U;

        tau = linspace(0,H,4001);

        vTheory = -p.w*U*sin(p.w*tau)-Q*cos(p.w*tau) ...
            + real(r1*k1*exp(r1*tau)+r2*k2*exp(r2*tau));

        if candidate > 0 && all(vTheory(2:end-1) < 0)
            Xtheory = candidate;
        else
            warning('No-sticking analytical candidate fails its motion conditions.');
        end
    else
        warning('No-sticking analytical formula has a negative radicand.');
    end
end

%% Print comparison
fprintf('\n--- Rod model with static and kinetic friction ---\n');
fprintf('Maximum static friction Cs = %.6f N\n',p.Cs);
fprintf('Kinetic friction Ck        = %.6f N\n',p.Ck);
fprintf('Input amplitude            = %.8f m\n',p.A);
fprintf('Simulated amplitude        = %.8f m\n',Xsim);
fprintf('No-sticking formula        = %.8f m\n',Xtheory);
fprintf('Last-cycle difference      = %.3g m\n',cycleDifference);
fprintf('Steady sticking duration   = %.8f s over %.2f s\n', ...
    stickDuration,5*T);

if isfinite(Xtheory)
    fprintf('Amplitude difference       = %.6f %%\n', ...
        100*abs(Xsim-Xtheory)/Xtheory);
end

if steadySticking
    fprintf(['NOTE: steady sticking occurs. The original amplitude ', ...
        'formula is only a reference, not a valid prediction here.\n']);
end

if cycleDifference > 1e-4*max(Xsim,1e-12)
    warning('Check convergence or increase nPeriods.');
end

%% Plot final three periods
show = t >= tEnd-3*T;

figure('Color','w','Name','Rod PD with static friction');

subplot(4,1,1);
plot(t(show),xDesired(show),'k--','LineWidth',1.1);
hold on;
plot(t(show),x(show),'b','LineWidth',1.2);

if isfinite(Xtheory)
    yline(Xtheory,'r--','LineWidth',1.1);
    yline(-Xtheory,'r--','LineWidth',1.1, ...
        'HandleVisibility','off');
    legend('Desired','Rod','No-sticking formula','Location','best');
else
    legend('Desired','Rod','Location','best');
end

grid on;
ylabel('Position [m]');
title(sprintf('Simulation amplitude = %.6f m',Xsim));

subplot(4,1,2);
plot(t(show),vDesired(show),'k--','LineWidth',1.1);
hold on;
plot(t(show),v(show),'b','LineWidth',1.2);
grid on;
ylabel('Velocity [m/s]');
legend('Desired','Rod','Location','best');

subplot(4,1,3);
plot(t(show),Fpd(show),'r','LineWidth',1.1);
hold on;
plot(t(show),Ffriction(show),'k--','LineWidth',1.1);
grid on;
ylabel('Force [N]');
legend('PD force','Friction term','Location','best');

subplot(4,1,4);
stairs(t(show),double(isSticking(show)),'LineWidth',1.2);
ylim([-0.1,1.1]);
yticks([0,1]);
yticklabels({'Sliding','Sticking'});
grid on;
xlabel('Time [s]');

%% Local functions

function F = pdForce(t,z,p)
    xd = p.A*sin(p.w*t);
    vd = p.A*p.w*cos(p.w*t);

    F = p.P*(xd-z(1)) + p.D*(vd-z(2));
end

function dz = slidingModel(t,z,p,s)
    F = pdForce(t,z,p);

    dz = [
        z(2);
        (F-p.b*z(2)-p.Ck*s)/p.m
    ];
end

function [value,isterminal,direction] = stopEvent(~,z,s)
    value = z(2);
    isterminal = 1;
    direction = -s;
end

function [tRelease,sRelease] = nextRelease(t0,xHold,p,T)
    % During sticking:
    % drive(t) = R*sin(w*t+delta) - P*xHold.
    %
    % Release at an outward crossing of +Cs or -Cs.

    R = p.A*hypot(p.P,p.D*p.w);
    delta = atan2(p.D*p.w,p.P);

    tRelease = Inf;
    sRelease = 0;

    for s = [-1,1]
        q = (p.P*xHold+s*p.Cs)/R;

        % At |q|=1 the force only touches the threshold.
        if abs(q) >= 1
            continue;
        end

        if s > 0
            angle = asin(q);       % Positive-going +Cs crossing
        else
            angle = pi-asin(q);    % Negative-going -Cs crossing
        end

        baseTime = (angle-delta)/p.w;
        k = ceil((t0-baseTime)/T);
        candidate = baseTime+k*T;

        % Handle roundoff when already at the release threshold.
        previous = candidate-T;
        timeTolerance = 1e-12*T;

        if abs(previous-t0) <= timeTolerance
            candidate = t0;
        elseif abs(candidate-t0) <= timeTolerance
            candidate = t0;
        end

        if candidate >= t0 && candidate < tRelease
            tRelease = candidate;
            sRelease = s;
        end
    end
end