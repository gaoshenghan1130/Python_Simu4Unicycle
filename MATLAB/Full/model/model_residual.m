function c = model_residual(x,u,p)
% M*dsigma + c = 0. Public input u=[F;M2], F=-F1.
x=x(:); u=u(:);
s1=x(1);
s2=x(2);
s3=x(3);
sr=x(4);
sg=x(5);
psi=x(6);
theta=x(7);
phi=x(8);
r=x(9);
gamma=x(10);
xG=x(11);
yG=x(12);
g=p.g;
R=p.R;
mw=p.mw;
JW1=p.JW1;
JW2=p.JW2;
mr=p.mr;
JR=p.JR;
BR=p.BR;
h=p.h;
mp=p.mp;
JPx=p.JPx;
JPy=p.JPy;
JPz=p.JPz;
BP=p.BP;
F1=-u(1); M2=u(2);
c=zeros(5,1);
c(1) = BR*R^2*s1 + BR*R*sr + F1*R - R*g*(mp + mw)*sin(theta) + R*mr*r*s1^2 - ...
    g*h*mp*sin(theta)*cos(gamma) + g*mr*r*cos(theta) + 2*mr*r*s1*sr + ...
    s1*s3*(R*h*mp*sin(gamma)*tan(theta) + (JPx - JPz + h^2*mp)*sin(gamma)*cos(gamma)*tan(theta)) + ...
    s1*sg*(-2*R*h*mp*sin(gamma) - 2*(JPx - JPz + h^2*mp)*sin(gamma)*cos(gamma)) + s2*s3*(-JW1 - R^2*mp - ...
    R^2*mw - R*h*mp*cos(gamma) - R*mr*r*tan(theta)) + s3^2*(R*h*mp*cos(gamma)*tan(theta) + ...
    mr*r^2*tan(theta) + (JPx - JPz + h^2*mp)*cos(gamma)^2*tan(theta) + (JPz + JR + JW2)*tan(theta)) + ...
    s3*sg*(JPx - JPy - JPz - 2*R*h*mp*cos(gamma) - 2*(JPx - JPz + h^2*mp)*cos(gamma)^2);
c(2) = BP*s2 - BP*sg - M2 - R*h*mp*s3^2*sin(gamma) - R*h*mp*sg^2*sin(gamma) - 2*R*mr*s3*sr + ...
    s1*s3*(R^2*(mp - mr + mw) + R*h*mp*cos(gamma) + R*mr*r*tan(theta));
c(3) = JW1*s1*s2 + R*h*mp*s2*s3*sin(gamma) + g*h*mp*sin(gamma)*sin(theta) + 2*mr*r*s3*sr + ...
    s1*s3*(R*mr*r - mr*r^2*tan(theta) + (-JPx + JPz - h^2*mp)*sin(gamma)^2*tan(theta) + (-JPz - JR - ...
    JW2)*tan(theta)) + s1*sg*(-JPx + JPy + JPz + 2*(JPx - JPz + h^2*mp)*sin(gamma)^2) + s3^2*(-JPx + JPz ...
    - h^2*mp)*sin(gamma)*cos(gamma)*tan(theta) + 2*s3*sg*(JPx - JPz + h^2*mp)*sin(gamma)*cos(gamma);
c(4) = BR*R*s1 + BR*sr + F1 + R*mr*s2*s3 + g*mr*sin(theta) - mr*r*s1^2 - mr*r*s3^2;
c(5) = -BP*s2 + BP*sg + M2 + R*h*mp*s2*s3*sin(gamma)*tan(theta) - g*h*mp*sin(gamma)*cos(theta) + ...
    s1^2*(R*h*mp*sin(gamma) + (JPx - JPz + h^2*mp)*sin(gamma)*cos(gamma)) + s1*s3*(JPx - JPz + ...
    R*h*mp*cos(gamma) + h^2*mp - 2*(JPx - JPz + h^2*mp)*sin(gamma)^2) + s3^2*(-JPx + JPz - ...
    h^2*mp)*sin(gamma)*cos(gamma);
end
