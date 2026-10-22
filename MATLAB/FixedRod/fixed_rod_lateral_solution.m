function Y = fixed_rod_lateral_solution(t,y0,s)
% Exact solution [theta;s1;s3], including the critical-speed case.
t=t(:).'; y0=y0(:);
eta=y0(3)+s.a*y0(1);
f=s.d*eta;
if abs(s.q)<1e-12
    theta=y0(1)+y0(2)*t+f*t.^2/2;
    rate=y0(2)+f*t;
elseif s.q>0
    w=sqrt(s.q); shift=f/s.q;
    theta=(y0(1)+shift)*cosh(w*t)+y0(2)*sinh(w*t)/w-shift;
    rate=(y0(1)+shift)*w*sinh(w*t)+y0(2)*cosh(w*t);
else
    w=sqrt(-s.q); center=f/w^2;
    theta=center+(y0(1)-center)*cos(w*t)+y0(2)*sin(w*t)/w;
    rate=-(y0(1)-center)*w*sin(w*t)+y0(2)*cos(w*t);
end
Y=[theta;rate;eta-s.a*theta];
end
