function out = plot_steady_turning_constraints()
%PLOT_STEADY_TURNING_CONSTRAINTS Plot the three steady-turning constraints.
%
% The constraints are those in
% Derivation/FullModel/analysis/manifold.md after eliminating F and M2:
%
%   C1(theta,gamma,phidot,psidot) = 0,
%   C2(theta,gamma,r,phidot,psidot) = 0,
%   C3(theta,gamma,phidot,psidot) = 0.
%
% C2 is quadratic in r, so the script plots the two branches obtained from
% C2=0.

close all;
set(groot,'defaultAxesFontName','Times New Roman');
set(groot,'defaultTextInterpreter','latex');
set(groot,'defaultAxesTickLabelInterpreter','latex');
set(groot,'defaultLegendInterpreter','latex');

% Physical parameters used by the current full-model derivation.
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
p.BR = 0;
p.BP = 0;

% Steady-turning rates. Change these values for another slice of the
% relative-equilibrium set.
phidot = -1.0;      % rad/s
psidot = 0.5;       % rad/s, nonzero for turning

theta = linspace(-25,25,401)*pi/180;
gamma = linspace(-25,25,401)*pi/180;
[TH,GA] = meshgrid(theta,gamma);

[C1,C2_0,C3] = constraint_values(TH,GA,0,phidot,psidot,p);
[~,C2_p,~] = constraint_values(TH,GA,1,phidot,psidot,p);
[~,C2_m,~] = constraint_values(TH,GA,-1,phidot,psidot,p);

% C2(theta,gamma,r)=c2*r^2+c1*r+c0.
c0 = C2_0;
c1 = (C2_p-C2_m)/2;
c2 = (C2_p+C2_m)/2-c0;
disc = c1.^2-4*c2.*c0;
r_plus = NaN(size(c0));
r_minus = NaN(size(c0));
valid = disc >= 0 & abs(c2)>1e-12;
r_plus(valid) = (-c1(valid)+sqrt(disc(valid)))./(2*c2(valid));
r_minus(valid) = (-c1(valid)-sqrt(disc(valid)))./(2*c2(valid));
% Plot the branch with smaller absolute rod displacement.
r_eq = r_plus;
choose_minus = abs(r_minus)<abs(r_plus) | isnan(r_plus);
r_eq(choose_minus) = r_minus(choose_minus);

out.parameters = p;
out.phidot = phidot;
out.psidot = psidot;
out.theta = theta;
out.gamma = gamma;
out.C1 = C1;
out.C3 = C3;
out.r_from_C2 = r_eq;
out.r_plus = r_plus;
out.r_minus = r_minus;
out.C2_coefficients = {c2,c1,c0};

figure('Color','w','Name','Steady-turning constraints', ...
    'Position',[100 180 1500 470]);
tiledlayout(1,3,'TileSpacing','compact','Padding','compact');

nexttile;
hold on;
contour(TH*180/pi,GA*180/pi,C1,[0 0],'k','LineWidth',2.2);
plot(theta*180/pi,zeros(size(theta)),'k:','LineWidth',0.7);
plot(zeros(size(gamma)),gamma*180/pi,'k:','LineWidth',0.7);
hold off;
xlabel('$\theta$ [deg]'); ylabel('$\gamma$ [deg]');
title('$C_1(\theta,\gamma)=0$'); axis equal tight; box on;

nexttile;
hold on;
contour(TH*180/pi,GA*180/pi,C3,[0 0],'k','LineWidth',2.2);
plot(theta*180/pi,zeros(size(theta)),'k:','LineWidth',0.7);
plot(zeros(size(gamma)),gamma*180/pi,'k:','LineWidth',0.7);
hold off;
xlabel('$\theta$ [deg]'); ylabel('$\gamma$ [deg]');
title('$C_3(\theta,\gamma)=0$'); axis equal tight; box on;

nexttile;
hold on;
contour(TH*180/pi,GA*180/pi,C1,[0 0],'k','LineWidth',2.0);
contour(TH*180/pi,GA*180/pi,C3,[0 0],'r','LineWidth',2.0);
r_levels = [-0.10 0 0.10];
r_colors = [0.00 0.45 0.74; 0.47 0.67 0.19; 0.85 0.33 0.10];
for k = 1:numel(r_levels)
    [~,C2_slice,~] = constraint_values(TH,GA,r_levels(k),phidot,psidot,p);
    contour(TH*180/pi,GA*180/pi,C2_slice,[0 0], ...
        'Color',r_colors(k,:),'LineWidth',1.5,'LineStyle','--');
end
hold off;
xlabel('$\theta$ [deg]'); ylabel('$\gamma$ [deg]');
title('Constraint intersections'); axis equal tight; box on;
legend({'$C_1=0$','$C_3=0$','$C_2=0, r=-0.10$ m', ...
    '$C_2=0, r=0$ m','$C_2=0, r=0.10$ m'}, ...
    'Location','best','Interpreter','latex','Box','off');

title_text = ['Steady turning: $\dot{\phi}=' num2str(phidot,'%.3g') ...
    '$, $\dot{\psi}=' num2str(psidot,'%.3g') '$ rad/s'];
sgtitle(title_text,'Interpreter','latex');

fprintf('Steady-turning constraint slice: phidot = %.6g, psidot = %.6g rad/s\n', ...
    phidot,psidot);
fprintf('C2 was solved as c2*r^2+c1*r+c0=0; both roots are returned.\n');
end

function [C1,C2,C3] = constraint_values(theta,gamma,r,phidot,psidot,p)
% Evaluate the three input-eliminated constraints.
a = p.JPx-p.JPz+p.mp*p.h^2;
b = p.R*p.mp*p.h;
c = p.JPz+p.JR+p.JW2;
sg = sin(gamma);
cg = cos(gamma);
tt = tan(theta);
sigma2 = phidot+psidot*sin(theta);
sigma3 = psidot*cos(theta);
sigmag = psidot*sin(theta);

% Constraint from the third dynamic equation.
C1 = b*sg.*sigma2.*sigma3 ...
    +p.g*p.h*p.mp*sg.*sin(theta) ...
    +2*a*sg.*cg.*sigma3.*sigmag;

% First equation minus R times the fourth equation; F is eliminated.
n1 = -p.g*(p.R*(p.mp+p.mw)+p.h*p.mp*cg).*sin(theta) ...
    +p.g*p.mr*r.*cos(theta) ...
    -(p.JW1+p.R^2*(p.mp+p.mw)+b*cg+p.R*p.mr*r.*tt).*sigma2.*sigma3 ...
    +(b*cg+p.mr*r.^2+a*cg.^2+c).*tt.*sigma3.^2 ...
    +(p.JPx-p.JPy-p.JPz-2*b*cg-2*a*cg.^2).*sigma3.*sigmag;
n4 = p.R*p.mr*sigma2.*sigma3+p.g*p.mr*sin(theta)-p.mr*r.*sigma3.^2;
C2 = n1-p.R*n4;

% Second plus fifth dynamic equation; M2 is eliminated.
n2 = -b*sg.*(sigma3.^2+sigmag.^2);
n5 = b*sg.*tt.*sigma2.*sigma3 ...
    -p.g*p.h*p.mp*sg.*cos(theta) ...
    -a*sg.*cg.*sigma3.^2;
C3 = n2+n5;
end
