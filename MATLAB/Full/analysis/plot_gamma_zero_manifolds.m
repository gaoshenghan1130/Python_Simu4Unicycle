function out = plot_gamma_zero_manifolds()
%PLOT_GAMMA_ZERO_MANIFOLDS Plot C(r,theta,psidot;phidot)=0 for gamma=0.
%
% The plotted surface is the remaining force-eliminated constraint on the
% gamma=0 steady-turning branch. Each selected phidot gives up to two r
% branches in the (r,theta,psidot) state space.

close all;
set(groot,'defaultAxesFontName','Times New Roman');
set(groot,'defaultTextInterpreter','latex');
set(groot,'defaultAxesTickLabelInterpreter','latex');
set(groot,'defaultLegendInterpreter','latex');

p.g = 9.81;
p.R = 0.253;
p.mw = 2.436;
p.JW1 = 0.09099921839;
p.JW2 = 0.04591427768;
p.mr = 2.3;
p.JR = 0.0517629;
p.mp = 2.799;
p.JPx = 0.01290418213;
p.JPy = 0.02090219895;
p.JPz = 0.01118711607;
p.h = 0.025;

speed_values = [1.0 1.5 2.0 ]; % m/s
phidot_values = speed_values/p.R;      % rad/s, phidot = v/R
theta_deg = linspace(-25,25,101);
psidot = linspace(-2,2,101);        % rad/s
[THETA,PSIDOT] = meshgrid(theta_deg*pi/180,psidot);

figure('Color','w','Name','Gamma-zero manifolds', ...
    'Position',[100 100 1200 760]);
hold on;
colors = lines(numel(phidot_values));
handles = gobjects(1,numel(phidot_values));

out = struct();
for k = 1:numel(phidot_values)
    phidot = phidot_values(k);
    [r_plus,r_minus,A0,A1,A2] = gamma_zero_roots( ...
        THETA,PSIDOT,phidot,p);

    % Keep the displayed region physically readable.
    r_plus(abs(r_plus)>0.5) = NaN;
    r_minus(abs(r_minus)>0.5) = NaN;

    h = surf(THETA*180/pi,PSIDOT,r_plus, ...
        'FaceColor',colors(k,:),'FaceAlpha',0.35,'EdgeColor','none');
    surf(THETA*180/pi,PSIDOT,r_minus, ...
        'FaceColor',colors(k,:),'FaceAlpha',0.35,'EdgeColor','none', ...
        'HandleVisibility','off');
    handles(k) = h;

    out(k).phidot = phidot;
    out(k).speed = speed_values(k);
    out(k).theta = theta_deg;
    out(k).psidot = psidot;
    out(k).r_plus = r_plus;
    out(k).r_minus = r_minus;
    out(k).A0 = A0;
    out(k).A1 = A1;
    out(k).A2 = A2;
end

xlabel('$\theta$ [deg]');
ylabel('$\dot\psi$ [rad/s]');
zlabel('$r$ [m]');
title('$\gamma=0$: constraint manifold for fixed $\dot\phi$');
legend_labels = arrayfun(@(v) sprintf('$v=%.1f$ m/s',v), ...
    speed_values,'UniformOutput',false);
legend(handles,legend_labels, ...
    'Location','eastoutside','Interpreter','latex','Box','off');
grid on; box on; axis tight;
view([-42 26]);
drawnow;

end

function [r_plus,r_minus,A0,A1,A2] = gamma_zero_roots(theta,psidot,phidot,p)
% Coefficients of C=A2*r^2+A1*r+A0 on gamma=0.
a = p.JPx-p.JPz+p.mp*p.h^2;
b = p.R*p.mp*p.h;
c = p.JPz+p.JR+p.JW2;

st = sin(theta);
ct = cos(theta);
ts = psidot.*st;
tc = psidot.*ct;
sigma2 = phidot+ts;

A2 = p.mr*tan(theta).*tc.^2;
A1 = p.mr*p.g*ct+p.R*p.mr*tc.^2 ...
    -p.R*p.mr*tan(theta).*sigma2.*tc;
A0 = -p.g*(p.R*(p.mp+p.mw+p.mr)+p.h*p.mp).*st ...
    -(p.JW1+p.R^2*(p.mp+p.mw+p.mr)+b).*sigma2.*tc ...
    +(b+a+c)*tan(theta).*tc.^2 ...
    +(p.JPx-p.JPy-p.JPz-2*b-2*a).*ts.*tc;

disc = A1.^2-4*A2.*A0;
r_plus = NaN(size(A0));
r_minus = NaN(size(A0));
quadratic = disc >= 0 & abs(A2)>1e-10;
r_plus(quadratic) = (-A1(quadratic)+sqrt(disc(quadratic)))./(2*A2(quadratic));
r_minus(quadratic) = (-A1(quadratic)-sqrt(disc(quadratic)))./(2*A2(quadratic));

% At theta=0, A2=0 and the constraint is linear in r.
linear = ~quadratic & abs(A1)>1e-10;
r_plus(linear) = -A0(linear)./A1(linear);
r_minus(linear) = r_plus(linear);
end
